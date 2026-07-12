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
#include <edm4hep/MutableTrack.h>
#include <edm4hep/TrackCollection.h>
#include <edm4hep/TrackState.h>
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
#include <atomic>
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
/// Determines the momentum of telescope tracks from the target constraint.
///
/// A field-free tracker downstream of a dipole measures a track's position and
/// direction but not its momentum (no curvature), so the seed's nominal q/p
/// survives the fit. The momentum is instead fixed by requiring the track to
/// originate at the target: q/p is solved so that the field-aware
/// back-extrapolation from the most upstream measured state lands on the
/// reference-surface centre in the bending plane (local x = eBoundLoc0). The
/// bending-plane miss is ~linear in q/p (deflection ∝ ∫B·dl / p), so a few
/// secant iterations converge.
///
/// Optionally a momentum-carrying track state is added on a plane inside the
/// magnet, since the AtIP omega is degenerate where the field vanishes: its
/// omega encodes p via p [GeV] = 0.3 * |B| [T] / (|omega| [1/mm] * 1e3).
///
/// Used as the CKFRunner track hook; const and thread-safe.
class TelescopeMomentumConstraint {
public:
  TelescopeMomentumConstraint(const IActsGeoSvc& geo, std::shared_ptr<const Acts::Surface> referenceSurface,
                              double momentumReportZ, std::size_t maxSteps)
      : m_geo(geo), m_referenceSurface(std::move(referenceSurface)), m_maxSteps(maxSteps),
        m_propagator(ACTSTracking::makePropagator(geo, false)) {
    if (momentumReportZ > 0.0) {
      Acts::Transform3 transform = Acts::Transform3::Identity();
      transform.translation() = Acts::Vector3(0, 0, momentumReportZ * Acts::UnitConstants::mm);
      m_reportSurface = Acts::Surface::makeShared<Acts::PlaneSurface>(transform);
    }
  }

  /// CKFRunner track hook: constrain the track's q/p and, if requested, return
  /// a finisher that appends the in-magnet momentum state to the edm4hep track.
  ACTSTracking::CKFRunner::TrackFinisher operator()(ACTSTracking::CKFTrackContainer::TrackProxy& track,
                                                    Acts::MagneticFieldProvider::Cache& magCache) const {
    std::optional<Acts::BoundTrackParameters> upstream = upstreamMeasuredParams(track);
    if (!upstream) {
      ++m_nNoStart;
      return {};
    }
    std::optional<double> qOverP = solve(*upstream);
    if (!qOverP) {
      ++m_nFailed;
      return {};
    }
    ++m_nSolved;
    setQOverP(track, *qOverP);

    if (!m_reportSurface) {
      return {};
    }
    std::optional<edm4hep::TrackState> state = momentumReportState(*upstream, *qOverP, magCache);
    if (!state) {
      ++m_nNoReport;
      return {};
    }
    return [state = *state](edm4hep::MutableTrack& out) { out.addToTrackStates(state); };
  }

  std::string summary() const {
    return fmt::format("Momentum constraint: {} tracks constrained to the target, {} failed to propagate, {} had no "
                       "measured start state; {} constrained tracks got no in-magnet momentum state.",
                       m_nSolved.load(), m_nFailed.load(), m_nNoStart.load(), m_nNoReport.load());
  }

private:
  /// Upstream end of the measured trajectory (closest to the target). As in
  /// CaloStateAppender::findStartParams the two ends are compared
  /// geometrically, so the result does not depend on the fit direction.
  template <class TrackProxy>
  std::optional<Acts::BoundTrackParameters> upstreamMeasuredParams(const TrackProxy& track) const {
    std::optional<Acts::BoundTrackParameters> firstEnd;
    std::optional<Acts::BoundTrackParameters> lastEnd;
    for (const auto& state : track.trackStatesReversed()) {
      const auto flags = state.typeFlags();
      if (!state.hasSmoothed() || !flags.test(Acts::TrackStateFlag::HasMeasurement) ||
          flags.test(Acts::TrackStateFlag::IsOutlier)) {
        continue;
      }
      Acts::BoundTrackParameters params(state.referenceSurface().getSharedPtr(), state.smoothed(),
                                        state.smoothedCovariance(), track.particleHypothesis());
      if (!firstEnd) {
        firstEnd = params;
      }
      lastEnd = std::move(params);
    }
    if (!firstEnd) {
      return std::nullopt;
    }
    const Acts::Vector3 span = lastEnd->position(m_geoCtx) - firstEnd->position(m_geoCtx);
    return (firstEnd->direction().dot(span) >= 0.0) ? firstEnd : lastEnd;
  }

  /// Propagate @p start, with q/p replaced by @p qOverP, back to @p target.
  std::optional<Acts::BoundTrackParameters> propagate(const Acts::BoundTrackParameters& start, double qOverP,
                                                      const Acts::Surface& target) const {
    Acts::BoundVector pars = start.parameters();
    pars[Acts::eBoundQOverP] = qOverP;
    const Acts::BoundTrackParameters trial(start.referenceSurface().getSharedPtr(), pars, start.covariance(),
                                           start.particleHypothesis());
    ACTSTracking::CKFPropagator::Options<> opts(m_geoCtx, m_magCtx);
    opts.maxSteps = m_maxSteps;
    opts.direction = Acts::Direction::Backward(); // the target is upstream of the tracker
    auto result = m_propagator.propagate(trial, target, opts);
    if (!result.ok() || !result.value().endParameters.has_value()) {
      return std::nullopt;
    }
    return result.value().endParameters.value();
  }

  /// Secant solve of loc0(q/p) = 0 on the reference surface, |p| clamped.
  std::optional<double> solve(const Acts::BoundTrackParameters& start) const {
    constexpr int kMaxIter = 12;
    constexpr double kTolMm = 0.02; // sub-resolution landing accuracy
    const double seedQ = start.parameters()[Acts::eBoundQOverP];
    const double qSign = (seedQ >= 0.0) ? 1.0 : -1.0;
    const double qAbsMin = 1.0 / 200.0; // |p| <= 200 GeV
    const double qAbsMax = 1.0 / 0.05;  // |p| >= 0.05 GeV
    auto clampQ = [&](double q) { return qSign * std::clamp(std::abs(q), qAbsMin, qAbsMax); };
    auto missX = [&](double q) -> std::optional<double> {
      auto p = propagate(start, q, *m_referenceSurface);
      if (!p) {
        return std::nullopt;
      }
      return p->parameters()[Acts::eBoundLoc0];
    };

    double q0 = clampQ(seedQ != 0.0 ? seedQ : qSign * qAbsMin);
    auto m0 = missX(q0);
    if (!m0) {
      return std::nullopt;
    }
    double f0 = *m0;
    if (std::abs(f0) < kTolMm) {
      return q0;
    }
    double q1 = clampQ(q0 * 1.5);
    if (q1 == q0) {
      q1 = clampQ(q0 + qSign * 0.02);
    }
    auto m1 = missX(q1);
    if (!m1) {
      return std::nullopt;
    }
    double f1 = *m1;
    for (int it = 0; it < kMaxIter && std::abs(f1) >= kTolMm; ++it) {
      const double denom = f1 - f0;
      if (std::abs(denom) < 1e-12) {
        break;
      }
      const double q2 = clampQ(q1 - f1 * (q1 - q0) / denom);
      auto m2 = missX(q2);
      if (!m2) {
        return std::nullopt;
      }
      q0 = q1;
      f0 = f1;
      q1 = q2;
      f1 = *m2;
    }
    return q1;
  }

  /// Write @p qOverP into every state: q/p is constant along a field-free
  /// track, so the reference extrapolation carries it whichever end it starts from.
  static void setQOverP(ACTSTracking::CKFTrackContainer::TrackProxy& track, double qOverP) {
    for (auto state : track.trackStatesReversed()) {
      if (state.hasSmoothed()) {
        state.smoothed()[Acts::eBoundQOverP] = qOverP;
      }
      if (state.hasFiltered()) {
        state.filtered()[Acts::eBoundQOverP] = qOverP;
      }
      if (state.hasPredicted()) {
        state.predicted()[Acts::eBoundQOverP] = qOverP;
      }
    }
  }

  /// Momentum-carrying state on the in-magnet plane, or nullopt if the plane is
  /// not reached or has no field there.
  std::optional<edm4hep::TrackState> momentumReportState(const Acts::BoundTrackParameters& upstream, double qOverP,
                                                         Acts::MagneticFieldProvider::Cache& magCache) const {
    auto atField = propagate(upstream, qOverP, *m_reportSurface);
    if (!atField) {
      return std::nullopt;
    }
    const Acts::Vector3 pos = atField->position(m_geoCtx);
    auto fieldRes = m_geo.magneticField()->getField(pos, magCache);
    if (!fieldRes.ok()) {
      return std::nullopt;
    }
    const double bTesla = (*fieldRes).norm() / Acts::UnitConstants::T;
    const double absP = (qOverP != 0.0) ? 1.0 / std::abs(qOverP) : 0.0; // native q/p is 1/GeV
    if (!(absP > 0.0) || !(bTesla > 0.0)) {
      return std::nullopt;
    }
    const Acts::Vector3 dir = atField->direction();
    edm4hep::TrackState state{};
    state.location = edm4hep::TrackState::AtOther;
    state.phi = static_cast<float>(std::atan2(dir.y(), dir.x()));
    state.omega = static_cast<float>(std::copysign(1.0, qOverP) * 0.3 * bTesla / absP * 1e-3);
    state.tanLambda = static_cast<float>(dir.z() / std::hypot(dir.x(), dir.y()));
    state.referencePoint =
        edm4hep::Vector3f(static_cast<float>(pos.x()), static_cast<float>(pos.y()), static_cast<float>(pos.z()));
    return state;
  }

  const IActsGeoSvc& m_geo;
  Acts::GeometryContext m_geoCtx = Acts::GeometryContext::dangerouslyDefaultConstruct();
  Acts::MagneticFieldContext m_magCtx{};
  std::shared_ptr<const Acts::Surface> m_referenceSurface;
  std::shared_ptr<const Acts::Surface> m_reportSurface;
  std::size_t m_maxSteps;
  ACTSTracking::CKFPropagator m_propagator;

  mutable std::atomic<std::size_t> m_nSolved{0};
  mutable std::atomic<std::size_t> m_nFailed{0};
  mutable std::atomic<std::size_t> m_nNoStart{0};
  mutable std::atomic<std::size_t> m_nNoReport{0};
};

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
 * The momentum, which the field-free tracker cannot measure, is determined
 * from the requirement that the track originates at the target
 * (ConstrainMomentum, see TelescopeMomentumConstraint).
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
  Gaudi::Property<bool> m_constrainMomentum{
      this, "ConstrainMomentum", true,
      "Determine each track's q/p, which the field-free tracker leaves at NominalMomentum, from the requirement that "
      "it originates at the target (the ReferenceZ plane centre): q/p is solved so the field-aware back-extrapolation "
      "lands on the target in the bending plane. Needs the real magnetic field (ActsGeoSvc.UseDD4hepBField)."};
  Gaudi::Property<float> m_momentumReportZ{
      this, "MomentumReportZ", 2053.0,
      "z [mm] of a plane inside the bending magnet where a momentum-carrying AtOther track state is added, since the "
      "AtIP omega is degenerate where the field vanishes: p [GeV] = 0.3 * |B| [T] / (|omega| [1/mm] * 1e3). Default "
      "is the LUXE dipole centre (z in [1434, 2672] mm). <= 0 disables it. Only used with ConstrainMomentum."};
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

  // Target momentum constraint, used by m_ckfRunner as its track hook when
  // ConstrainMomentum is set. Declared before m_ckfRunner, which holds a
  // pointer to it, so that it outlives the runner.
  std::optional<TelescopeMomentumConstraint> m_momentumConstraint{};

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

  // The tracker does not measure the momentum, so fix it from the target
  // constraint through the CKFRunner track hook.
  ACTSTracking::CKFRunner::TrackHook trackHook;
  if (m_constrainMomentum) {
    m_momentumConstraint.emplace(*m_actsGeoSvc, referenceSurface, m_momentumReportZ.value(), m_maxPropagationSteps);
    trackHook = [constraint = &*m_momentumConstraint](ACTSTracking::CKFTrackContainer::TrackProxy& track,
                                                      Acts::MagneticFieldProvider::Cache& magCache) {
      return (*constraint)(track, magCache);
    };
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
                                                      .trackSelector = trackSelector,
                                                      .trackHook = trackHook});

  return StatusCode::SUCCESS;
}

StatusCode TelescopeCKFTrackingAlg::finalize() {
  if (m_extrapolateToCalo) {
    info() << m_caloMonitor.summary() << endmsg;
  }
  if (m_doTwoWayCKF && m_ckfRunner) {
    info() << m_ckfRunner->twoWaySummary() << endmsg;
  }
  if (m_momentumConstraint) {
    info() << m_momentumConstraint->summary() << endmsg;
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
