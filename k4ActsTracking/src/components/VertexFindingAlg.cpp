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
#include "k4ActsTracking/ActsGaudiLogger.h"

#include <k4FWCore/GaudiChecks.h>
#include <k4FWCore/Transformer.h>
#include <k4Interface/IGeoSvc.h>

#include <edm4hep/ReconstructedParticleCollection.h>
#include <edm4hep/TrackCollection.h>
#include <edm4hep/VertexCollection.h>
#include <edm4hep/VertexRecoParticleLinkCollection.h>

#include <Acts/Definitions/TrackParametrization.hpp>
#include <Acts/Definitions/Units.hpp>
#include <Acts/EventData/BoundTrackParameters.hpp>
#include <Acts/EventData/TrackContainer.hpp>
#include <Acts/EventData/VectorMultiTrajectory.hpp>
#include <Acts/EventData/VectorTrackContainer.hpp>
#include <Acts/Geometry/GeometryContext.hpp>
#include <Acts/MagneticField/MagneticFieldContext.hpp>
#include <Acts/MagneticField/MagneticFieldProvider.hpp>
#include <Acts/Propagator/Propagator.hpp>
#include <Acts/Propagator/SympyStepper.hpp>
#include <Acts/Utilities/AnnealingUtility.hpp>
#include <Acts/Utilities/Logger.hpp>
#include <Acts/Vertexing/AdaptiveGridDensityVertexFinder.hpp>
#include <Acts/Vertexing/AdaptiveGridTrackDensity.hpp>
#include <Acts/Vertexing/AdaptiveMultiVertexFinder.hpp>
#include <Acts/Vertexing/AdaptiveMultiVertexFitter.hpp>
#include <Acts/Vertexing/HelicalTrackLinearizer.hpp>
#include <Acts/Vertexing/ImpactPointEstimator.hpp>
#include <Acts/Vertexing/TrackAtVertex.hpp>
#include <Acts/Vertexing/Vertex.hpp>
#include <Acts/Vertexing/VertexingOptions.hpp>

#include <ActsPlugins/DD4hep/DD4hepFieldAdapter.hpp>
#include <ActsPlugins/EDM4hep/EDM4hepUtil.hpp>

#include <DD4hep/Detector.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <exception>
#include <memory>
#include <optional>
#include <tuple>
#include <vector>

/// Primary vertex finding with the ACTS adaptive multi-vertex finder (AMVF).
///
/// The algorithm takes a finished edm4hep track collection, e.g. CLD's
/// `SiTracks_Refitted` from ConformalTracking + RefitFinal, and only does
/// vertexing on it; it does no track finding or fitting of its own. Tracks are
/// translated into the Acts perigee parametrization by the centralised
/// converter `ActsPlugins::EDM4hepUtil::readTrack`, the inverse of the
/// conversion used when writing tracks out in `ACTSTracking::ACTS2edm4hep_track`,
/// so both directions share one convention.
///
/// Only a magnetic field is needed, no Acts tracking geometry: the vertexing
/// propagator runs without a navigator, so the field is taken straight from
/// DD4hep via `GeoSvc`. That also keeps the algorithm usable for detectors
/// whose Acts geometry carries no material yet.
///
/// Outputs:
///  - the vertices,
///  - one ReconstructedParticle per track the fit kept, pointing at its input
///    edm4hep::Track and attached to the vertex's `particles`, so vertices can
///    be traced back to tracks (and from there to MC truth),
///  - a vertex -> particle link per such track whose weight is the adaptive
///    fit's track weight, which edm4hep::Vertex itself has no place for.
struct VertexFindingAlg final
    : k4FWCore::MultiTransformer<std::tuple<edm4hep::VertexCollection, edm4hep::ReconstructedParticleCollection,
                                            edm4hep::VertexRecoParticleLinkCollection>(
          const edm4hep::TrackCollection&)> {
  using Propagator = Acts::Propagator<Acts::SympyStepper>;
  using Linearizer = Acts::HelicalTrackLinearizer;
  using Fitter = Acts::AdaptiveMultiVertexFitter;
  using Finder = Acts::AdaptiveMultiVertexFinder;
  using Seeder = Acts::AdaptiveGridDensityVertexFinder;

  using Output = std::tuple<edm4hep::VertexCollection, edm4hep::ReconstructedParticleCollection,
                            edm4hep::VertexRecoParticleLinkCollection>;

  VertexFindingAlg(const std::string& name, ISvcLocator* svcLoc)
      : MultiTransformer(name, svcLoc, {KeyValues("InputTracks", {"SiTracks_Refitted"})},
                         {KeyValues("OutputVertices", {"ACTSPrimaryVertices"}),
                          KeyValues("OutputParticles", {"ACTSPrimaryVertices_Particles"}),
                          KeyValues("OutputVertexParticleLinks", {"ACTSPrimaryVertices_ParticleLinks"})}) {}

  StatusCode initialize() override;

  Output operator()(const edm4hep::TrackCollection& inputTracks) const override;

  // ----- track selection ---------------------------------------------------
  Gaudi::Property<double> m_minPt{this, "MinPt", 100.0, "Minimum track pT to use in the vertex fit [MeV]"};
  Gaudi::Property<double> m_maxAbsD0{this, "MaxAbsD0", 10.0,
                                     "Maximum |d0| of a track w.r.t. the origin to be used [mm]"};
  Gaudi::Property<double> m_maxAbsZ0{this, "MaxAbsZ0", 50.0,
                                     "Maximum |z0| of a track w.r.t. the origin to be used [mm]"};

  // ----- vertex seeding (density of track z0 along the beam line) ----------
  Gaudi::Property<double> m_spatialBinExtent{this, "SeedBinExtent", 0.015,
                                             "Bin size along z of the seeding track-density grid [mm]"};
  Gaudi::Property<double> m_spatialWindow{this, "SeedWindow", 50.0,
                                          "Half-width of the z window filled in the seeding density grid [mm]"};
  /// The seeder only fills its density with tracks whose d0 significance with
  /// respect to the beam line is below this value (Acts default 3.5). Where the
  /// track d0 resolution is much finer than the transverse beam-spot size, as
  /// for FCC-ee, this can leave no track to seed from when the collision point
  /// is away from the beam line.
  Gaudi::Property<double> m_seedMaxD0Significance{
      this, "SeedMaxD0Significance", 3.5,
      "Maximum d0 significance w.r.t. the beam line for a track to enter the seeding density"};

  // ----- beam spot ----------------------------------------------------------
  /// Setting BeamSpotSize enables the beam-spot constraint: the vertex fit
  /// uses the beam spot as a prior, and the track-compatibility test around a
  /// seed includes the seed's (beam-spot) uncertainty. Without it, Acts tests
  /// compatibility as if the seed sat exactly on the beam line, which rejects
  /// precise tracks from collisions a few beam-spot widths off it.
  Gaudi::Property<std::vector<double>> m_beamSpotSize{
      this, "BeamSpotSize", {}, "Beam-spot size (sigma x, y, z) [mm]; empty = no beam-spot constraint"};
  Gaudi::Property<std::vector<double>> m_beamSpotPosition{
      this, "BeamSpotPosition", {0., 0.}, "Transverse beam-spot centre (x, y) [mm]"};

  // ----- vertex finding ----------------------------------------------------
  Gaudi::Property<double> m_tracksMaxZinterval{
      this, "TracksMaxZInterval", 1.0, "Tracks within this z distance of a seed are considered for it [mm]"};
  Gaudi::Property<double> m_tracksMaxSignificance{
      this, "TracksMaxSignificance", 5.0, "Maximum compatibility significance for a track to join a vertex"};
  Gaudi::Property<int> m_maxIterations{this, "MaxIterations", 1000, "Maximum number of vertex finding iterations"};
  Gaudi::Property<bool> m_doSmoothing{this, "DoSmoothing", true,
                                      "Refit the track parameters with the vertex position as constraint"};

  // ----- output -------------------------------------------------------------
  /// Same threshold as the minTrkWeight of Acts' VertexTruthMatcher, so a
  /// track counts as part of a vertex here exactly when it would there.
  Gaudi::Property<double> m_minOutputTrackWeight{
      this, "MinOutputTrackWeight", 0.1, "Minimum adaptive-fit weight for a track to be stored with its vertex"};

  /// edm4hep tracks from a Kalman fit that does not fit time (CLD, ILD, ...)
  /// carry time = -1 with a zero time uncertainty, which makes the 6x6
  /// covariance singular. Vertexing runs without time here, but a singular
  /// covariance still trips determinant checks, so the time variance is
  /// replaced by this value when it is not positive.
  Gaudi::Property<double> m_defaultTimeVariance{this, "DefaultTimeVariance", 1.0,
                                                "Time variance substituted when the input has none [ns^2]"};

private:
  SmartIF<IGeoSvc> m_geoSvc;

  std::shared_ptr<const Acts::MagneticFieldProvider> m_magneticField{nullptr};
  std::shared_ptr<Propagator> m_propagator{nullptr};
  std::unique_ptr<const Acts::Logger> m_actsLogger{nullptr};

  /// The linearizer is referenced by the fitter inside m_finder, so it has to
  /// stay put for the lifetime of the algorithm.
  std::optional<Acts::ImpactPointEstimator> m_ipEstimator{};
  std::optional<Linearizer> m_linearizer{};
  std::optional<Finder> m_finder{};

  /// Beam-spot constraint, set when BeamSpotSize is given
  std::optional<Acts::Vertex> m_beamSpot{};

  /// The conversion and the vertexing need contexts, but neither the geometry
  /// nor the DD4hep field is conditions-dependent here.
  Acts::GeometryContext m_gctx = Acts::GeometryContext::dangerouslyDefaultConstruct();
  Acts::MagneticFieldContext m_mctx{};
};

DECLARE_COMPONENT(VertexFindingAlg)

StatusCode VertexFindingAlg::initialize() {
  m_geoSvc = svcLoc()->service<IGeoSvc>("GeoSvc");
  K4_GAUDI_CHECK(m_geoSvc);

  m_actsLogger = makeActsGaudiLogger(this);

  // Magnetic field directly from DD4hep. No Acts tracking geometry is built:
  // the vertexing propagator is navigator-free and only needs the field.
  m_magneticField = std::make_shared<ActsPlugins::DD4hepFieldAdapter>(m_geoSvc->getDetector()->field());

  m_propagator = std::make_shared<Propagator>(Acts::SympyStepper(m_magneticField));

  // Estimates impact parameters and their uncertainties w.r.t. a vertex
  // candidate; used both for seeding decisions and inside the fitter.
  Acts::ImpactPointEstimator::Config ipCfg(m_magneticField, m_propagator);
  m_ipEstimator.emplace(ipCfg, m_actsLogger->cloneWithSuffix("IPEstimator"));

  // Linear expansion of the track parameters around the vertex candidate,
  // which is what makes the Kalman-style vertex fit possible.
  Linearizer::Config linCfg;
  linCfg.bField = m_magneticField;
  linCfg.propagator = m_propagator;
  m_linearizer.emplace(linCfg, m_actsLogger->cloneWithSuffix("Linearizer"));

  // Seeding: build a density of the tracks' z0 along the beam line and take
  // its maxima as vertex candidates.
  Acts::AdaptiveGridTrackDensity::Config densityCfg;
  densityCfg.spatialBinExtent = m_spatialBinExtent * Acts::UnitConstants::mm;
  densityCfg.spatialWindow = {-m_spatialWindow * Acts::UnitConstants::mm, m_spatialWindow * Acts::UnitConstants::mm};
  densityCfg.useTime = false;
  Seeder::Config seederCfg{Acts::AdaptiveGridTrackDensity(densityCfg)};
  seederCfg.extractParameters.connect<&Acts::InputTrack::extractParameters>();
  // The squared cut is what the seeder applies; it is derived from
  // maxD0TrackSignificance only when the config is constructed.
  seederCfg.maxD0TrackSignificance = m_seedMaxD0Significance;
  seederCfg.d0SignificanceCut = m_seedMaxD0Significance * m_seedMaxD0Significance;
  auto seeder = std::make_shared<Seeder>(seederCfg);

  // The fit assigns every track a weight between 0 and 1 instead of a yes/no
  // decision, and lowers the annealing temperature over the iterations so the
  // assignment hardens gradually.
  Fitter::Config fitterCfg(*m_ipEstimator);
  fitterCfg.annealingTool = Acts::AnnealingUtility(Acts::AnnealingUtility::Config(9., {1.0}));
  fitterCfg.doSmoothing = m_doSmoothing;
  fitterCfg.useTime = false;
  fitterCfg.extractParameters.connect<&Acts::InputTrack::extractParameters>();
  fitterCfg.trackLinearizer.connect<&Linearizer::linearizeTrack>(&m_linearizer.value());
  Fitter fitter(std::move(fitterCfg), m_actsLogger->cloneWithSuffix("AMVFitter"));

  Finder::Config finderCfg(std::move(fitter), std::move(seeder), *m_ipEstimator, m_magneticField);
  // Initial uncertainty the fit starts from, before any track is added. The
  // Acts default is 1e8 mm^2 in every dimension; shrinking that to the ~1e-6
  // mm^2 of a fitted vertex costs 14 orders of magnitude in the Kalman update
  // and the cancellation leaves negative variances on the diagonal. The
  // spatial value used here is the one Acts' own example uses; time keeps the
  // large default since it is on a different numerical scale (and unused).
  finderCfg.initialVariances = Acts::Vector4{1e+2, 1e+2, 1e+2, 1e+8};
  finderCfg.tracksMaxZinterval = m_tracksMaxZinterval * Acts::UnitConstants::mm;
  finderCfg.tracksMaxSignificance = m_tracksMaxSignificance;
  finderCfg.maxIterations = m_maxIterations;
  finderCfg.useTime = false;
  finderCfg.extractParameters.connect<&Acts::InputTrack::extractParameters>();

  if (!m_beamSpotSize.empty()) {
    if (m_beamSpotSize.size() != 3 || m_beamSpotPosition.size() != 2) {
      error() << "BeamSpotSize needs 3 values (sigma x, y, z) and BeamSpotPosition 2 (x, y)" << endmsg;
      return StatusCode::FAILURE;
    }
    // The seeder places seeds at the constraint position plus the z it
    // finds, so the constraint's z has to be 0.
    Acts::Vertex beamSpot(Acts::Vector4(m_beamSpotPosition[0] * Acts::UnitConstants::mm,
                                        m_beamSpotPosition[1] * Acts::UnitConstants::mm, 0., 0.));
    Acts::Vector4 variances;
    for (std::size_t i = 0; i < 3; ++i) {
      const double sigma = m_beamSpotSize[i] * Acts::UnitConstants::mm;
      variances[i] = sigma * sigma;
    }
    variances[3] = finderCfg.initialVariances[3]; // time is not used
    beamSpot.setFullCovariance(variances.asDiagonal());
    m_beamSpot = beamSpot;

    // Test track compatibility against the seed's uncertainty (the beam spot)
    // instead of treating the seed position as exact.
    finderCfg.useVertexCovForIPEstimation = true;
    info() << "Beam-spot constraint: sigma = (" << m_beamSpotSize[0] << ", " << m_beamSpotSize[1] << ", "
           << m_beamSpotSize[2] << ") mm" << endmsg;
  }

  m_finder.emplace(std::move(finderCfg), m_actsLogger->cloneWithSuffix("AMVFinder"));

  return StatusCode::SUCCESS;
}

VertexFindingAlg::Output VertexFindingAlg::operator()(const edm4hep::TrackCollection& inputTracks) const {
  edm4hep::VertexCollection vertices;
  edm4hep::ReconstructedParticleCollection particles;
  edm4hep::VertexRecoParticleLinkCollection vertexParticleLinks;
  auto output = [&]() { return Output{std::move(vertices), std::move(particles), std::move(vertexParticleLinks)}; };

  // --- 1. edm4hep tracks -> Acts track parameters -------------------------
  Acts::TrackContainer actsTracks{Acts::VectorTrackContainer{}, Acts::VectorMultiTrajectory{}};

  // The Acts::InputTrack objects below hold raw pointers into this vector, so
  // it must not reallocate while it is being filled.
  std::vector<Acts::BoundTrackParameters> trackParameters;
  trackParameters.reserve(inputTracks.size());
  // Position of each entry of trackParameters in inputTracks, to find the
  // edm4hep track behind a track the vertex fit used.
  std::vector<std::size_t> inputTrackIndex;
  inputTrackIndex.reserve(inputTracks.size());

  for (std::size_t iTrack = 0; iTrack < inputTracks.size(); ++iTrack) {
    const auto edmTrack = inputTracks[iTrack];
    // readTrack requires the state at the interaction point, which carries the
    // perigee parameters the vertex fit works with.
    const auto states = edmTrack.getTrackStates();
    const bool hasIPState = std::ranges::any_of(
        states, [](const auto& state) { return state.location == edm4hep::TrackState::AtIP; });
    if (!hasIPState) {
      warning() << "Skipping track without an AtIP track state" << endmsg;
      continue;
    }

    auto trackProxy = actsTracks.makeTrack();
    try {
      ActsPlugins::EDM4hepUtil::readTrack(m_gctx, m_mctx, edmTrack, trackProxy, *m_magneticField, *m_actsLogger);
    } catch (const std::exception& e) {
      warning() << "Track conversion failed, skipping track: " << e.what() << endmsg;
      continue;
    }

    const Acts::BoundTrackParameters params = trackProxy.createParametersAtReference();

    if (params.transverseMomentum() < m_minPt * Acts::UnitConstants::MeV) {
      continue;
    }
    if (std::abs(params.parameters()[Acts::eBoundLoc0]) > m_maxAbsD0 * Acts::UnitConstants::mm) {
      continue;
    }
    if (std::abs(params.parameters()[Acts::eBoundLoc1]) > m_maxAbsZ0 * Acts::UnitConstants::mm) {
      continue;
    }
    if (!params.covariance().has_value()) {
      warning() << "Skipping track without covariance" << endmsg;
      continue;
    }

    // Patch the missing time uncertainty, see m_defaultTimeVariance.
    Acts::BoundMatrix cov = params.covariance().value();
    if (cov(Acts::eBoundTime, Acts::eBoundTime) <= 0.) {
      cov(Acts::eBoundTime, Acts::eBoundTime) =
          m_defaultTimeVariance * Acts::UnitConstants::ns * Acts::UnitConstants::ns;
    }

    // A converted covariance that is not positive definite would poison the
    // vertex fit, and points at a conversion or input problem rather than a
    // vertexing one, so report it instead of letting it propagate silently.
    if (cov.diagonal().minCoeff() <= 0. || cov.determinant() <= 0.) {
      warning() << "Skipping track whose converted covariance is not positive definite (min diagonal "
                << cov.diagonal().minCoeff() << ", determinant " << cov.determinant() << ")" << endmsg;
      continue;
    }

    trackParameters.emplace_back(params.referenceSurface().getSharedPtr(), params.parameters(), cov,
                                 params.particleHypothesis());
    inputTrackIndex.push_back(iTrack);
  }

  debug() << "Converted " << trackParameters.size() << " of " << inputTracks.size() << " tracks for vertexing"
          << endmsg;

  if (trackParameters.empty()) {
    return output();
  }

  std::vector<Acts::InputTrack> vertexInput;
  vertexInput.reserve(trackParameters.size());
  for (const auto& params : trackParameters) {
    vertexInput.emplace_back(&params);
  }

  // --- 2. run the adaptive multi-vertex finder ----------------------------
  // With a beam-spot constraint, the fit uses it as a prior; without, the
  // vertex position comes from the tracks alone.
  const Acts::VertexingOptions finderOptions = m_beamSpot.has_value()
                                                   ? Acts::VertexingOptions(m_gctx, m_mctx, *m_beamSpot, true)
                                                   : Acts::VertexingOptions(m_gctx, m_mctx);
  auto finderState = m_finder->makeState(m_mctx);

  auto result = m_finder->find(vertexInput, finderOptions, finderState);
  if (!result.ok()) {
    error() << "Vertex finding failed: " << result.error().message() << endmsg;
    return output();
  }
  const std::vector<Acts::Vertex>& foundVertices = result.value();

  // --- 3. Acts vertices -> edm4hep ----------------------------------------
  // The hardest vertex, i.e. the one with the largest sum of pT^2 over its
  // tracks, is flagged as the primary vertex. Only tracks the adaptive fit
  // actually kept (weight > 0.5) contribute.
  std::size_t primaryIndex = 0;
  double maxSumPt2 = -1.;
  for (std::size_t i = 0; i < foundVertices.size(); ++i) {
    double sumPt2 = 0.;
    for (const auto& trackAtVertex : foundVertices[i].tracks()) {
      if (trackAtVertex.trackWeight > 0.5) {
        const double pt = trackAtVertex.fittedParams.transverseMomentum();
        sumPt2 += pt * pt;
      }
    }
    if (sumPt2 > maxSumPt2) {
      maxSumPt2 = sumPt2;
      primaryIndex = i;
    }
  }

  for (std::size_t i = 0; i < foundVertices.size(); ++i) {
    const Acts::Vertex& vertex = foundVertices[i];

    auto edmVertex = vertices.create();
    // Fills position, covariance, chi2 and ndf.
    ActsPlugins::EDM4hepUtil::writeVertex(vertex, edmVertex);
    if (i == primaryIndex) {
      edmVertex.setPrimary(true);
    }

    // One particle per track the fit kept, carrying the track's parameters at
    // the vertex and pointing back at the edm4hep track it came from.
    std::size_t nStored = 0;
    for (const auto& trackAtVertex : vertex.tracks()) {
      if (trackAtVertex.trackWeight < m_minOutputTrackWeight) {
        continue;
      }
      const auto* original = trackAtVertex.originalParams.as<Acts::BoundTrackParameters>();
      const auto paramsIndex = static_cast<std::size_t>(original - trackParameters.data());
      const auto edmTrack = inputTracks[inputTrackIndex[paramsIndex]];

      const Acts::BoundTrackParameters& atVertex = trackAtVertex.fittedParams;
      const Acts::Vector3 momentum = atVertex.momentum() / Acts::UnitConstants::GeV;
      const double mass = atVertex.particleHypothesis().mass() / Acts::UnitConstants::GeV;

      auto particle = particles.create();
      particle.setMomentum({static_cast<float>(momentum.x()), static_cast<float>(momentum.y()),
                            static_cast<float>(momentum.z())});
      particle.setMass(static_cast<float>(mass));
      particle.setEnergy(static_cast<float>(std::hypot(momentum.norm(), mass)));
      particle.setCharge(static_cast<float>(atVertex.charge()));
      particle.setReferencePoint(edmVertex.getPosition());
      particle.addToTracks(edmTrack);
      edmVertex.addToParticles(particle);

      auto link = vertexParticleLinks.create();
      link.setFrom(edmVertex);
      link.setTo(particle);
      link.setWeight(static_cast<float>(trackAtVertex.trackWeight));
      ++nStored;
    }

    debug() << "Vertex " << i << " at (" << vertex.position().x() << ", " << vertex.position().y() << ", "
            << vertex.position().z() << ") mm from " << vertex.tracks().size() << " tracks (" << nStored
            << " stored), chi2/ndf " << vertex.fitQuality().first << "/" << vertex.fitQuality().second << endmsg;
  }

  debug() << "Found " << vertices.size() << " vertices" << endmsg;

  return output();
}
