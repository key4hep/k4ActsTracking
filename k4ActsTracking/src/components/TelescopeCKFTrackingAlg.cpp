/*
 * Copyright (c) 2014-2024 Key4hep-Project.
 *
 * This file is part of Key4hep.
 * See https://key4hep.github.io/key4hep-doc/ for further info.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// k4ActsTracking
#include "k4ActsTracking/CKFRunner.hxx"
#include "k4ActsTracking/CellIDSelector.h"
#include "k4ActsTracking/Helpers.hxx"
#include "k4ActsTracking/IActsGeoSvc.h"
#include "k4ActsTracking/Measurement.hxx"
#include "k4ActsTracking/SourceLink.hxx"

// k4FWCore
#include <k4FWCore/GaudiChecks.h>
#include <k4FWCore/Transformer.h>

// edm4hep
#include <edm4hep/TrackCollection.h>
#include <edm4hep/TrackerHit.h>
#include <edm4hep/TrackerHitPlaneCollection.h>
#include <edm4hep/TrackerHitSimTrackerHitLinkCollection.h>

// Gaudi
#include <Gaudi/Property.h>
#include <GaudiKernel/SmartIF.h>

// ACTS
#include <Acts/Definitions/Units.hpp>
#include <Acts/EventData/BoundTrackParameters.hpp>
#include <Acts/EventData/ParticleHypothesis.hpp>
#include <Acts/Geometry/GeometryContext.hpp>
#include <Acts/MagneticField/MagneticFieldContext.hpp>
#include <Acts/MagneticField/MagneticFieldProvider.hpp>
#include <Acts/Surfaces/PlaneSurface.hpp>
#include <Acts/Utilities/VectorHelpers.hpp>

#include <fmt/format.h>

// Standard
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <numeric>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace {
/// Build straight-line bound track parameters for a telescope seed.
///
/// In a field-free tracker (e.g. LUXE, whose dipole sits upstream) the
/// trajectory through the planar layers is a straight line, so the
/// conformal/helix estimate used by the cylindrical seeder does not apply.
/// The seed direction is taken from the bottom->top space points; the momentum
/// magnitude is not measurable without curvature, so a nominal value is used
/// (it does not affect the straight-line propagation, only the covariance
/// scale and the reported q/p). The parameters are expressed at the start SP:
/// the bottom one, or the top one with @p propagateBackward (outside-in), whose
/// surface @p startSurface is and whose hit @p startHit gives the time.
std::optional<Acts::BoundTrackParameters> estimateStraightLineSeedParameters(
    const Acts::GeometryContext& geoCtx, const Acts::Surface& startSurface, const Acts::Vector3& bottomPos,
    const Acts::Vector3& topPos, const edm4hep::TrackerHit& startHit, double nominalP, double errPos, double errPhi,
    double errLambda, double errRelP, double errTime, double errQOverP, bool propagateBackward) {
  const Acts::Vector3 dir = (topPos - bottomPos).normalized();
  const Acts::Vector3& startPos = propagateBackward ? topPos : bottomPos;

  // The start position is derived from the surface, so it lies on it; use a
  // generous tolerance to guard against round-off in the global<->local map.
  Acts::Result<Acts::Vector2> local = startSurface.globalToLocal(geoCtx, startPos, dir, 1 * Acts::UnitConstants::mm);
  if (!local.ok()) {
    return std::nullopt;
  }

  Acts::BoundVector params = Acts::BoundVector::Zero();
  params[Acts::eBoundLoc0] = (*local)[0];
  params[Acts::eBoundLoc1] = (*local)[1];
  params[Acts::eBoundPhi] = Acts::VectorHelpers::phi(dir);
  params[Acts::eBoundTheta] = Acts::VectorHelpers::theta(dir);
  params[Acts::eBoundQOverP] = 1.0 / nominalP; // charge sign is irrelevant at B = 0
  params[Acts::eBoundTime] = ACTSTracking::hitTime(startHit);

  Acts::BoundMatrix cov =
      ACTSTracking::makeInitialCovariance(nominalP, errPos, errPhi, errLambda, errRelP, errTime, errQOverP);
  return Acts::BoundTrackParameters(startSurface.getSharedPtr(), params, cov, Acts::ParticleHypothesis::pion());
}
} // namespace

/**
 * @brief CKF tracking for telescope geometries: planar layers stacked along the
 * beam in a field-free region (e.g. LUXE).
 *
 * The cylindrical triplet helix seeder of CKFTrackingAlg finds nothing in such
 * a geometry. Instead, the seed space points are grouped into layers along z,
 * collinear triplets over consecutive layers are formed, and straight-line
 * seed parameters with a nominal momentum feed the same Combinatorial Kalman
 * Filter (shared via ACTSTracking::CKFRunner). Beam-parallel tracks never
 * approach the beamline, so the AtIP track state is defined on a plane
 * perpendicular to the beam at ReferenceZ rather than on the beamline perigee.
 *
 * Inputs and outputs are the same as CKFTrackingAlg's, so it is a drop-in
 * replacement for it in a telescope reconstruction chain.
 */
struct TelescopeCKFTrackingAlg final
    : k4FWCore::MultiTransformer<std::tuple<edm4hep::TrackCollection, edm4hep::TrackCollection>(
          const edm4hep::TrackerHitPlaneCollection&, const edm4hep::TrackerHitSimTrackerHitLinkCollection&)> {
  TelescopeCKFTrackingAlg(const std::string& name, ISvcLocator* svcLoc);

  StatusCode initialize() override;

  StatusCode finalize() override;

  std::tuple<edm4hep::TrackCollection, edm4hep::TrackCollection>
  operator()(const edm4hep::TrackerHitPlaneCollection& trackerHitCollection,
             const edm4hep::TrackerHitSimTrackerHitLinkCollection& trackerHitRelations) const override;

private:
  /// A seed-selected space point: its global position and source link.
  struct SeedInput {
    Acts::Vector3 pos;
    ACTSTracking::SourceLink sourceLink;
  };

  /// Group @p seedInputs into layers along z, form collinear triplets over
  /// consecutive layers and build their straight-line seed parameters, adding
  /// the corresponding edm4hep seed tracks to @p seedCollection.
  std::vector<Acts::BoundTrackParameters> findSeeds(const std::vector<SeedInput>& seedInputs,
                                                    const ACTSTracking::HitContainer& hits,
                                                    edm4hep::TrackCollection& seedCollection,
                                                    Acts::MagneticFieldProvider::Cache& magCache) const;

  /// Time resolution (ns) of the hit: its time error, else the matching HitTimeResolutions entry.
  double hitTimeResolutionFor(const edm4hep::TrackerHitPlane& hit) const;

  // ----- Gaudi properties --------------------------------------------------

  /// @name Run control
  ///@{
  Gaudi::Property<bool> m_runCKF{this, "RunCKF", true, "Run tracking using CKF. False means stop at seeding."};
  Gaudi::Property<bool> m_doTwoWayCKF{this, "DoTwoWayCKF", false,
                                      "Run two-way CKF: first pass + smooth + second pass in opposite direction."};
  Gaudi::Property<bool> m_inflateCovarianceTwoWay{this, "InflateCovarianceTwoWay", true,
                                                  "Inflate covariance before the second CKF pass."};
  Gaudi::Property<double> m_twoWayInflateCovarianceFactor{this, "TwoWayInflateCovarianceFactor", 100.0,
                                                          "Covariance inflation factor for the second CKF pass."};
  Gaudi::Property<bool> m_propagateBackward{
      this, "PropagateBackward", false,
      "Find tracks outside-in: start the CKF at the most downstream seed SP and propagate backward through the other "
      "seed SPs towards the target."};
  Gaudi::Property<bool> m_extrapolateToCalo{
      this, "ExtrapolateToCalo", true,
      "Extrapolate fitted tracks to the calorimeter face and add an AtCalorimeter track state."};
  ///@}

  /// @name Telescope seeding and reference surface
  ///@{
  Gaudi::Property<float> m_layerZTolerance{this, "LayerZTolerance", 20.0,
                                           "Maximum |dz| [mm] between seed space points assigned to the same layer."};
  Gaudi::Property<float> m_collinearityCut{
      this, "CollinearityCut", 2.0,
      "Maximum transverse distance [mm] of the middle space point from the bottom-top line for a triplet seed."};
  Gaudi::Property<float> m_nominalMomentum{
      this, "NominalMomentum", 5.0,
      "Nominal seed momentum [GeV]. The field-free tracker does not measure the momentum, so it is not refined by "
      "the fit."};
  Gaudi::Property<float> m_referenceZ{
      this, "ReferenceZ", 0.0,
      "z [mm] of the beam-perpendicular reference plane the fitted tracks are extrapolated to for the AtIP track "
      "state. D0/Z0 of that state then read as the track's (x, y) on this plane."};
  ///@}

  /// @name Track-fit initial error estimates
  ///@{
  Gaudi::Property<double> m_initialTrackError_pos{this, "InitialTrackError_Pos", 10 * Acts::UnitConstants::um,
                                                  "Initial track error for local position."};
  Gaudi::Property<double> m_initialTrackError_phi{this, "InitialTrackError_Phi", 1 * Acts::UnitConstants::degree,
                                                  "Initial track error for phi."};
  Gaudi::Property<double> m_initialTrackError_relP{this, "InitialTrackError_RelP", 0.25,
                                                   "Initial track error for momentum (relative)."};
  Gaudi::Property<double> m_initialTrackError_qOverP{
      this, "InitialTrackError_QOverP", 0 / Acts::UnitConstants::GeV,
      "Initial track error for q/p (absolute), added in quadrature to the relative one."};
  Gaudi::Property<double> m_initialTrackError_lambda{this, "InitialTrackError_Lambda", 1 * Acts::UnitConstants::degree,
                                                     "Initial track error for lambda."};
  Gaudi::Property<double> m_initialTrackError_time{this, "InitialTrackError_Time", 100 * Acts::UnitConstants::ns,
                                                   "Initial track error for time."};
  ///@}

  /// @name CKF configuration
  ///@{
  Gaudi::Property<double> m_CKF_chi2CutOff{this, "CKF_Chi2CutOff", 15, "Maximum local chi2 contribution."};
  Gaudi::Property<int32_t> m_CKF_numMeasurementsCutOff{this, "CKF_NumMeasurementsCutOff", 10,
                                                       "Maximum measurements on a single surface."};
  Gaudi::Property<double> m_CKF_chi2CutOffOutlier{
      this, "CKF_Chi2CutOffOutlier", std::numeric_limits<double>::max(),
      "Maximum local chi2 for a failing hit to be kept as an outlier; above this it becomes a hole."};
  Gaudi::Property<unsigned int> m_maxPropagationSteps{
      this, "MaxPropagationSteps", ACTSTracking::kDefaultMaxPropagationSteps,
      "Maximum number of propagation steps for CKF track finding, reference-surface extrapolation and "
      "calorimeter-face extrapolation."};
  Gaudi::Property<bool> m_useHitTimeInCKF{
      this, "UseHitTimeInCKF", false,
      "If true, include hit time as a 3rd CKF measurement dimension (eBoundTime). Hit times must not be "
      "corrected for the time of flight (DDPlanarDigi CorrectTimesForPropagation=False)."};
  Gaudi::Property<std::map<std::string, double>> m_hitTimeResolutions{
      this,
      "HitTimeResolutions",
      {},
      "Fallback per-sensor time resolutions in ns, keyed by CellIDSelector selection string, for hits without a "
      "time error (e.g. digitized before EDM4hep 1.2). Only used with UseHitTimeInCKF. The selections must not "
      "overlap."};
  ///@}

  /// @name CKF branch stopper and track selection
  ///@{
  Gaudi::Property<bool> m_useBranchStopper{this, "UseBranchStopper", false, "Enable the CKF branch stopper."};
  Gaudi::Property<int> m_bsMaxHoles{this, "BranchStopper_MaxHoles", 2, "Stop a branch above this many holes."};
  Gaudi::Property<int> m_bsMaxOutliers{this, "BranchStopper_MaxOutliers", 2, "Stop a branch above this many outliers."};
  Gaudi::Property<int> m_bsMinMeasurements{
      this, "BranchStopper_MinMeasurements", 6,
      "When stopping, keep the branch if it has at least this many measurements, otherwise drop it."};
  Gaudi::Property<int> m_tsMinMeasurements{this, "TrackSelector_MinMeasurements", -1,
                                           "Acts::TrackSelector: minimum measurements per track (< 0: no selection)."};
  ///@}

  /// @name Seeding layer selection
  ///@{
  Gaudi::Property<std::vector<std::string>> m_seedingSensorsCellIDs{
      this,
      "SeedingSensorsCellIDs",
      {},
      "CellIDSelector selection strings for seed space-point filtering. "
      "Each entry is a comma-separated list of field:value constraints (e.g. \"layer:0|1|2|3\"). "
      "Multiple entries are OR-ed together. Omitted fields act as wildcards."};
  ///@}

  /// @name Multi-threading
  ///@{
  Gaudi::Property<int> m_numThreads{this, "NumThreads", 1, "Number of threads for internal TBB parallelism."};
  ///@}

  SmartIF<IActsGeoSvc> m_actsGeoSvc;

  k4ActsTracking::CellIDSelector m_seedSelector{};

  std::vector<std::pair<k4ActsTracking::CellIDSelector, double>> m_hitTimeResolutionSelectors{};

  // Shared Combinatorial Kalman Filter, built once in initialize() and reused
  // read-only across events and threads.
  std::optional<ACTSTracking::CKFRunner> m_ckfRunner{};

  // Calorimeter-face extrapolation monitoring, updated by the shared CKFRunner
  // and summarised in finalize().
  ACTSTracking::CaloExtrapMonitor m_caloMonitor{};

  mutable std::mutex m_seedMutex{};
  mutable std::mutex m_trackMutex{};
};

// ---------------------------------------------------------------------------
// Constructor / initialize / finalize
// ---------------------------------------------------------------------------

TelescopeCKFTrackingAlg::TelescopeCKFTrackingAlg(const std::string& name, ISvcLocator* svcLoc)
    : MultiTransformer(name, svcLoc,
                       {KeyValue("InputTrackerHitCollection", "TrackerHits"),
                        KeyValue("InputTrackerHitRelationCollection", "TrackerHitRelations")},
                       {KeyValue("OutputSeedCollection", "SeedTracks"), KeyValue("OutputTrackCollection", "Tracks")}) {}

StatusCode TelescopeCKFTrackingAlg::initialize() {
  m_actsGeoSvc = svcLoc()->service<IActsGeoSvc>("ActsGeoSvc");
  K4_GAUDI_CHECK(m_actsGeoSvc);

  m_seedSelector =
      k4ActsTracking::CellIDSelector(m_actsGeoSvc->cellIDEncodingString(), m_seedingSensorsCellIDs.value());

  std::vector<std::string> selections;
  for (const auto& [selection, resolution] : m_hitTimeResolutions.value()) {
    if (!(resolution > 0.0) || !std::isfinite(resolution)) {
      error() << "HitTimeResolutions[" << selection << "] must be finite and > 0, got " << resolution << endmsg;
      return StatusCode::FAILURE;
    }
    m_hitTimeResolutionSelectors.emplace_back(
        k4ActsTracking::CellIDSelector(m_actsGeoSvc->cellIDEncodingString(), {selection}), resolution);
    selections.push_back(selection);
  }
  for (std::size_t i = 0; i < m_hitTimeResolutionSelectors.size(); ++i) {
    for (std::size_t j = i + 1; j < m_hitTimeResolutionSelectors.size(); ++j) {
      if (m_hitTimeResolutionSelectors[i].first.overlaps(m_hitTimeResolutionSelectors[j].first)) {
        error() << "HitTimeResolutions selections \"" << selections[i] << "\" and \"" << selections[j] << "\" overlap"
                << endmsg;
        return StatusCode::FAILURE;
      }
    }
  }

  if (m_extrapolateToCalo && m_actsGeoSvc->caloSurfaceGeoIds().empty()) {
    warning() << "ExtrapolateToCalo requested but ActsGeoSvc provides no calorimeter-face surfaces; "
                 "no AtCalorimeter track states will be produced."
              << endmsg;
  }

  // The tracks run almost parallel to the beamline, so the default perigee (a
  // line along z) is unreachable. Use an unbounded plane at z = ReferenceZ with
  // its normal along the beam instead (identity rotation => local z = global z):
  // every forward track crosses it.
  Acts::Transform3 refTransform = Acts::Transform3::Identity();
  refTransform.translation() = Acts::Vector3(0, 0, m_referenceZ * Acts::UnitConstants::mm);
  auto referenceSurface = Acts::Surface::makeShared<Acts::PlaneSurface>(refTransform);

  std::optional<Acts::TrackSelector::Config> trackSelector;
  if (m_tsMinMeasurements >= 0) {
    trackSelector.emplace();
    trackSelector->minMeasurements = m_tsMinMeasurements;
  }

  m_ckfRunner.emplace(*m_actsGeoSvc,
                      ACTSTracking::CKFRunner::Config{.chi2CutOff = m_CKF_chi2CutOff,
                                                      .numMeasurementsCutOff = m_CKF_numMeasurementsCutOff,
                                                      .chi2CutOffOutlier = m_CKF_chi2CutOffOutlier,
                                                      .propagateBackward = m_propagateBackward,
                                                      .extrapolateToCalo = m_extrapolateToCalo,
                                                      .maxSteps = m_maxPropagationSteps,
                                                      .useBranchStopper = m_useBranchStopper,
                                                      .bsMaxHoles = m_bsMaxHoles,
                                                      .bsMaxOutliers = m_bsMaxOutliers,
                                                      .bsMinMeasurements = m_bsMinMeasurements,
                                                      .doTwoWayCKF = m_doTwoWayCKF,
                                                      .inflateCovarianceTwoWay = m_inflateCovarianceTwoWay,
                                                      .twoWayInflateCovarianceFactor = m_twoWayInflateCovarianceFactor,
                                                      .referenceSurface = referenceSurface,
                                                      .trackSelector = trackSelector});

  return StatusCode::SUCCESS;
}

StatusCode TelescopeCKFTrackingAlg::finalize() {
  if (m_extrapolateToCalo) {
    info() << m_caloMonitor.summary() << endmsg;
  }
  if (m_doTwoWayCKF && m_ckfRunner) {
    info() << m_ckfRunner->twoWaySummary() << endmsg;
  }
  return StatusCode::SUCCESS;
}

double TelescopeCKFTrackingAlg::hitTimeResolutionFor(const edm4hep::TrackerHitPlane& hit) const {
  if (hit.getTimeError() > 0) {
    return hit.getTimeError();
  }
  for (const auto& [selector, resolution] : m_hitTimeResolutionSelectors) {
    if (selector.accept(hit.getCellID())) {
      return resolution;
    }
  }
  throw std::runtime_error(fmt::format("TelescopeCKFTrackingAlg: hit with cellID {} has no time error and no "
                                       "HitTimeResolutions selection matches it",
                                       hit.getCellID()));
}

// ---------------------------------------------------------------------------
// Event processing
// ---------------------------------------------------------------------------

std::tuple<edm4hep::TrackCollection, edm4hep::TrackCollection> TelescopeCKFTrackingAlg::operator()(
    const edm4hep::TrackerHitPlaneCollection& trackerHitCollection,
    const edm4hep::TrackerHitSimTrackerHitLinkCollection& /*trackerHitRelations*/) const {
  edm4hep::TrackCollection seedCollection;
  edm4hep::TrackCollection trackCollection;

  const Acts::GeometryContext geoCtx = Acts::GeometryContext::dangerouslyDefaultConstruct();
  const Acts::MagneticFieldContext magCtx{};

  ACTSTracking::SourceLinkContainer sourceLinks;
  ACTSTracking::MeasurementContainer measurements;
  ACTSTracking::HitContainer hits;
  std::vector<SeedInput> seedInputs;

  // Build measurements + source links for all hits (shared with the other CKF
  // algorithms); for seed-selected surfaces also keep the global position.
  ACTSTracking::prepareTrackerHits(
      *this, *m_actsGeoSvc, geoCtx, trackerHitCollection, measurements, sourceLinks, hits, m_numThreads.value(),
      [&](const edm4hep::TrackerHitPlane& hit, const ACTSTracking::SourceLink& sourceLink,
          const Acts::Vector3& globalPos, const Acts::Surface& /*surface*/, const Acts::SquareMatrix2& /*localCov*/) {
        if (m_seedSelector.accept(hit.getCellID())) {
          // Single precision, as the space points of the cylindrical seeder.
          seedInputs.push_back({globalPos.cast<float>().cast<double>(), sourceLink});
        }
      },
      m_useHitTimeInCKF.value(), [this](const edm4hep::TrackerHitPlane& hit) { return hitTimeResolutionFor(hit); });

  debug() << fmt::format("Created {} sourceLinks and {} space points for seeding", sourceLinks.size(),
                         seedInputs.size())
          << endmsg;

  Acts::MagneticFieldProvider::Cache magCache = m_actsGeoSvc->magneticField()->makeCache(magCtx);

  const std::vector<Acts::BoundTrackParameters> paramseeds = findSeeds(seedInputs, hits, seedCollection, magCache);

  if (m_runCKF) {
    m_ckfRunner->findTracks(*this, measurements, sourceLinks, hits, paramseeds, magCache, trackCollection, m_trackMutex,
                            &m_caloMonitor);
  }

  debug() << "Track Collection Size: " << trackCollection.size() << endmsg;
  return std::make_tuple(std::move(seedCollection), std::move(trackCollection));
}

std::vector<Acts::BoundTrackParameters>
TelescopeCKFTrackingAlg::findSeeds(const std::vector<SeedInput>& seedInputs, const ACTSTracking::HitContainer& hits,
                                   edm4hep::TrackCollection& seedCollection,
                                   Acts::MagneticFieldProvider::Cache& magCache) const {
  const Acts::GeometryContext geoCtx = Acts::GeometryContext::dangerouslyDefaultConstruct();
  std::vector<Acts::BoundTrackParameters> paramseeds;

  // -------------------------------------------------------------------------
  // 1. Group the seed space points into layers along the beam (z) axis. The
  //    points are sorted by z and split into a new layer whenever the gap to
  //    the previous point exceeds the layer tolerance (telescope layers are
  //    well separated in z, so this greedy clustering is unambiguous).
  // -------------------------------------------------------------------------
  std::vector<std::size_t> order(seedInputs.size());
  std::iota(order.begin(), order.end(), 0);
  std::ranges::sort(order, [&](std::size_t a, std::size_t b) { return seedInputs[a].pos.z() < seedInputs[b].pos.z(); });

  std::vector<std::vector<std::size_t>> layers;
  for (std::size_t k : order) {
    if (layers.empty() || std::abs(seedInputs[k].pos.z() - seedInputs[layers.back().back()].pos.z()) >
                              static_cast<double>(m_layerZTolerance)) {
      layers.emplace_back();
    }
    layers.back().push_back(k);
  }

  if (layers.size() < 3) {
    debug() << "Telescope seeding: only " << layers.size() << " layer(s) with seed hits (need >= 3); no seeds."
            << endmsg;
    return paramseeds;
  }

  const double collinearityCut = m_collinearityCut;
  const double nominalP = m_nominalMomentum * Acts::UnitConstants::GeV;

  // -------------------------------------------------------------------------
  // 2. Form straight-line triplets over consecutive layer triplets. For each
  //    bottom/top pair the middle hit closest to the bottom->top line is kept,
  //    accepted only if within the collinearity cut.
  // -------------------------------------------------------------------------
  for (std::size_t l = 0; l + 2 < layers.size(); ++l) {
    const std::vector<std::size_t>& botLayer = layers[l];
    const std::vector<std::size_t>& midLayer = layers[l + 1];
    const std::vector<std::size_t>& topLayer = layers[l + 2];

    for (std::size_t bi : botLayer) {
      const Acts::Vector3& bottom = seedInputs[bi].pos;
      for (std::size_t ti : topLayer) {
        const Acts::Vector3& top = seedInputs[ti].pos;
        const double dz = top.z() - bottom.z();
        if (std::abs(dz) < 1e-6) {
          continue;
        }

        double bestResidual = collinearityCut;
        std::size_t bestMid = std::numeric_limits<std::size_t>::max();
        for (std::size_t mi : midLayer) {
          const Acts::Vector3& mid = seedInputs[mi].pos;
          const double t = (mid.z() - bottom.z()) / dz;
          const Acts::Vector3 pred = bottom + t * (top - bottom);
          const double resid = std::hypot(mid.x() - pred.x(), mid.y() - pred.y());
          if (resid < bestResidual) {
            bestResidual = resid;
            bestMid = mi;
          }
        }
        if (bestMid == std::numeric_limits<std::size_t>::max()) {
          continue;
        }

        // PropagateBackward: start the (backward) CKF from the top SP.
        const ACTSTracking::SourceLink& bottomSL = seedInputs[bi].sourceLink;
        const ACTSTracking::SourceLink& startSL = m_propagateBackward ? seedInputs[ti].sourceLink : bottomSL;
        const Acts::Surface* surface = m_actsGeoSvc->trackingGeometry()->findSurface(startSL.geometryId());
        if (surface == nullptr) {
          warning() << "Surface with geoID " << startSL.geometryId() << " not found in tracking geometry" << endmsg;
          continue;
        }

        std::optional<Acts::BoundTrackParameters> paramseed = estimateStraightLineSeedParameters(
            geoCtx, *surface, bottom, top, hits[startSL.index()], nominalP, m_initialTrackError_pos,
            m_initialTrackError_phi, m_initialTrackError_lambda, m_initialTrackError_relP, m_initialTrackError_time,
            m_initialTrackError_qOverP, m_propagateBackward);
        if (!paramseed) {
          continue;
        }
        paramseeds.push_back(*paramseed);

        auto seedTrackState = ACTSTracking::makeSeedTrackState(*this, *m_actsGeoSvc, geoCtx, *paramseed, magCache,
                                                               m_propagateBackward ? edm4hep::TrackState::AtLastHit
                                                                                   : edm4hep::TrackState::AtFirstHit);
        ACTSTracking::appendSeedTrack(seedCollection, m_seedMutex, seedTrackState,
                                      std::array{hits[bottomSL.index()], hits[seedInputs[bestMid].sourceLink.index()],
                                                 hits[seedInputs[ti].sourceLink.index()]});
      }
    }
  }

  debug() << "Telescope seeding: found " << paramseeds.size() << " seeds across " << layers.size() << " layers."
          << endmsg;
  return paramseeds;
}

DECLARE_COMPONENT(TelescopeCKFTrackingAlg)
