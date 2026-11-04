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

#include <Acts/Definitions/Direction.hpp>
#include <Acts/Definitions/TrackParametrization.hpp>
#include <Acts/Definitions/Units.hpp>
#include <Acts/EventData/BoundTrackParameters.hpp>
#include <Acts/Geometry/GeometryContext.hpp>
#include <Acts/MagneticField/MagneticFieldContext.hpp>
#include <Acts/MagneticField/MagneticFieldProvider.hpp>
#include <Acts/Propagator/Propagator.hpp>
#include <Acts/Propagator/PropagatorOptions.hpp>
#include <Acts/Propagator/SympyStepper.hpp>
#include <Acts/Surfaces/BoundaryTolerance.hpp>
#include <Acts/Surfaces/PerigeeSurface.hpp>
#include <Acts/Utilities/AnnealingUtility.hpp>
#include <Acts/Utilities/Logger.hpp>
#include <Acts/Vertexing/AdaptiveGridDensityVertexFinder.hpp>
#include <Acts/Vertexing/AdaptiveGridTrackDensity.hpp>
#include <Acts/Vertexing/AdaptiveMultiVertexFinder.hpp>
#include <Acts/Vertexing/AdaptiveMultiVertexFitter.hpp>
#include <Acts/Vertexing/GaussianTrackDensity.hpp>
#include <Acts/Vertexing/HelicalTrackLinearizer.hpp>
#include <Acts/Vertexing/IVertexFinder.hpp>
#include <Acts/Vertexing/ImpactPointEstimator.hpp>
#include <Acts/Vertexing/TrackAtVertex.hpp>
#include <Acts/Vertexing/TrackDensityVertexFinder.hpp>
#include <Acts/Vertexing/Vertex.hpp>
#include <Acts/Vertexing/VertexingOptions.hpp>

#include <ActsPlugins/DD4hep/DD4hepFieldAdapter.hpp>
#include <ActsPlugins/EDM4hep/EDM4hepUtil.hpp>

#include <DD4hep/Detector.h>

#include <Eigen/Cholesky>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <exception>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

/// Primary vertex finding with the ACTS adaptive multi-vertex finder (AMVF).
///
/// The algorithm takes a finished edm4hep track collection, e.g. CLD's
/// `SiTracks_Refitted` from ConformalTracking + RefitFinal, and only does
/// vertexing on it; it does no track finding or fitting of its own. Only the
/// perigee parameters of each track's AtIP track state are used.
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
///    be traced back to tracks (and from there to MC truth). An edm4hep::Track
///    carries no particle species, so mass and energy assume the Acts default
///    pion hypothesis and the PDG is left unset,
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

  /// What the finder's Acts::InputTrack points at: the converted parameters
  /// and the position of the edm4hep track they came from
  struct ConvertedTrack {
    Acts::BoundTrackParameters parameters;
    std::size_t inputIndex;
    static Acts::BoundTrackParameters extractParameters(const Acts::InputTrack& track) {
      return track.as<ConvertedTrack>()->parameters;
    }
  };

  using Output = std::tuple<edm4hep::VertexCollection, edm4hep::ReconstructedParticleCollection,
                            edm4hep::VertexRecoParticleLinkCollection>;

  VertexFindingAlg(const std::string& name, ISvcLocator* svcLoc)
      : MultiTransformer(name, svcLoc, {KeyValues("InputTracks", {"SiTracks_Refitted"})},
                         {KeyValues("OutputVertices", {"ACTSPrimaryVertices"}),
                          KeyValues("OutputParticles", {"ACTSPrimaryVertices_Particles"}),
                          KeyValues("OutputVertexParticleLinks", {"ACTSPrimaryVertices_ParticleLinks"})}) {}

  StatusCode initialize() override;

  Output operator()(const edm4hep::TrackCollection& inputTracks) const override;

  /// Converts the AtIP state of each input track and applies the track
  /// selection and uncertainty corrections; skipped tracks are left out
  std::vector<ConvertedTrack> convertTracks(const edm4hep::TrackCollection& inputTracks) const;
  /// Writes the found vertices, their particles and the weight links
  Output writeVertices(const std::vector<Acts::Vertex>& foundVertices,
                       const edm4hep::TrackCollection& inputTracks) const;

  // ----- track selection ---------------------------------------------------
  Gaudi::Property<double> m_minPt{this, "MinPt", 100.0, "Minimum track pT to use in the vertex fit [MeV]"};
  Gaudi::Property<double> m_maxAbsD0{this, "MaxAbsD0", 10.0,
                                     "Maximum |d0| of a track w.r.t. the beam-spot centre to be used [mm]"};
  Gaudi::Property<double> m_maxAbsZ0{this, "MaxAbsZ0", 50.0,
                                     "Maximum |z0| of a track w.r.t. the beam-spot centre to be used [mm]"};

  // ----- vertex seeding (density of track z0 along the beam line) ----------
  /// Both seeders take the maximum of the track density along the beam line
  /// as the next vertex candidate: "AdaptiveGrid" fills the density into z
  /// bins, "Gaussian" evaluates the sum of the tracks' Gaussians directly.
  Gaudi::Property<std::string> m_seeder{this, "Seeder", "AdaptiveGrid",
                                        "Vertex seeder: \"AdaptiveGrid\" (default) or \"Gaussian\""};
  Gaudi::Property<double> m_spatialBinExtent{this, "SeedBinExtent", 0.015,
                                             "Bin size along z of the AdaptiveGrid seeding density [mm]"};
  Gaudi::Property<double> m_spatialWindow{this, "SeedWindow", 50.0,
                                          "Half-width of the z window of the AdaptiveGrid seeding density [mm]"};
  /// Either seeder only fills its density with tracks whose d0 significance with
  /// respect to the beam line is below this value (Acts default 3.5). Where the
  /// track d0 resolution is much finer than the transverse beam-spot size, as
  /// for FCC-ee, this can leave no track to seed from when the collision point
  /// is away from the beam line.
  Gaudi::Property<double> m_seedMaxD0Significance{
      this, "SeedMaxD0Significance", 3.5,
      "Maximum d0 significance w.r.t. the beam line for a track to enter the seeding density"};

  // ----- beam spot ----------------------------------------------------------
  /// Centre of the beam spot, where seeds are placed. The seeder cuts on and
  /// fills its density with the tracks' d0 and z0 as given, so tracks whose
  /// perigee does not refer to this point are first propagated to a perigee
  /// surface there; MaxAbsD0 and MaxAbsZ0 then refer to it as well.
  Gaudi::Property<std::vector<double>> m_beamSpotPosition{
      this, "BeamSpotPosition", {0., 0., 0.}, "Beam-spot centre (x, y, z) [mm]"};
  /// Setting BeamSpotSize enables the beam-spot constraint: the vertex fit
  /// uses the beam spot as a prior, and the track-compatibility test around a
  /// seed includes the seed's (beam-spot) uncertainty. Without it, Acts tests
  /// compatibility as if the seed sat exactly on the beam line, which rejects
  /// precise tracks from collisions a few beam-spot widths off it.
  Gaudi::Property<std::vector<double>> m_beamSpotSize{
      this, "BeamSpotSize", {}, "Beam-spot size (sigma x, y, z) [mm]; empty = no beam-spot constraint"};

  // ----- vertex finding ----------------------------------------------------
  Gaudi::Property<double> m_tracksMaxZinterval{this, "TracksMaxZInterval", 1.0,
                                               "Tracks within this z distance of a seed are considered for it [mm]"};
  Gaudi::Property<double> m_tracksMaxSignificance{this, "TracksMaxSignificance", 5.0,
                                                  "Maximum compatibility significance for a track to join a vertex"};
  Gaudi::Property<int> m_maxIterations{this, "MaxIterations", 1000, "Maximum number of vertex finding iterations"};
  /// After a candidate is fitted, the tracks whose compatibility with it is
  /// below this chi2 leave the seed pool; the others can seed further vertices.
  Gaudi::Property<double> m_maxVertexChi2{this, "MaxVertexChi2", 18.42,
                                          "Track-vertex chi2 below which a track no longer seeds new vertices"};
  Gaudi::Property<double> m_maxMergeVertexSignificance{
      this, "MaxMergeVertexSignificance", 3.0,
      "A new vertex closer than this significance to an existing one is discarded as merged"};
  Gaudi::Property<bool> m_doFullSplitting{this, "DoFullSplitting", false,
                                          "Use the 3D (instead of the z) vertex distance in the merging test"};
  Gaudi::Property<bool> m_doNotBreakWhileSeeding{this, "DoNotBreakWhileSeeding", false,
                                                 "Keep searching for vertices after a candidate has been rejected"};

  // ----- vertex fit ----------------------------------------------------------
  /// The fit gives each track the weight
  ///   w = exp(-chi2 / 2T) / (exp(-chi2 / 2T) + exp(-cutOff / 2T)),
  /// so w = 0.5 at chi2 = AnnealingCutOff, and steps the temperature T through
  /// AnnealingTemperatures. A high first temperature lets tracks far from the
  /// seed still pull the vertex before the weights harden; a single T = 1
  /// means no annealing, which loses precise tracks from collisions off the
  /// beam line where the seed sits. The chi2 uses the track uncertainty only.
  /// Empty (the default) uses the Acts default ladder.
  Gaudi::Property<std::vector<double>> m_annealingTemperatures{
      this,
      "AnnealingTemperatures",
      {},
      "Temperatures of the deterministic annealing of the track weights, empty for the Acts default"};
  Gaudi::Property<double> m_annealingCutOff{this, "AnnealingCutOff", 9.0,
                                            "Track-vertex chi2 at which a track gets weight 0.5"};
  Gaudi::Property<unsigned int> m_fitterMaxIterations{this, "FitterMaxIterations", 30,
                                                      "Maximum number of iterations of the multi-vertex fit"};

  /// Multiplies the converted track covariance by the square of this factor,
  /// for input tracks whose uncertainties are known to be underestimated
  /// (pull widths above 1). 1 leaves the input untouched.
  Gaudi::Property<double> m_trackCovarianceScale{this, "TrackCovarianceScale", 1.0,
                                                 "Factor applied to the track parameter uncertainties"};
  /// Added in quadrature to sigma(d0) and sigma(z0) after the scaling above,
  /// for inputs that miss a momentum-independent term of the impact-parameter
  /// resolution (pulls growing with momentum). 0 leaves the input untouched.
  Gaudi::Property<double> m_impactParameterErrorTerm{this, "ImpactParameterErrorTerm", 0.0,
                                                     "Uncertainty added in quadrature to sigma(d0) and sigma(z0) [mm]"};

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
  /// The seeder selected by the Seeder property, or nullptr for an unknown name
  std::shared_ptr<const Acts::IVertexFinder> makeSeeder() const;

  SmartIF<IGeoSvc> m_geoSvc;

  std::shared_ptr<const Acts::MagneticFieldProvider> m_magneticField{nullptr};
  std::shared_ptr<Propagator> m_propagator{nullptr};
  std::unique_ptr<const Acts::Logger> m_actsLogger{nullptr};

  /// The linearizer is referenced by the fitter inside m_finder, so it has to
  /// stay put for the lifetime of the algorithm.
  std::optional<Acts::ImpactPointEstimator> m_ipEstimator{};
  std::optional<Linearizer> m_linearizer{};
  std::optional<Finder> m_finder{};

  /// Perigee surface at the beam-spot centre the input tracks are expressed on
  std::shared_ptr<const Acts::PerigeeSurface> m_beamLine{nullptr};
  /// Seed position, with the beam-spot covariance when BeamSpotSize is given
  Acts::Vertex m_beamSpot{};
  bool m_useBeamSpotConstraint{false};

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

  // Seeding: take the maximum of the tracks' z0 density along the beam line
  // as the next vertex candidate.
  auto seeder = makeSeeder();
  if (!seeder) {
    error() << "Unknown Seeder '" << m_seeder.value() << "'; expected \"AdaptiveGrid\" or \"Gaussian\"" << endmsg;
    return StatusCode::FAILURE;
  }

  // The fit assigns every track a weight between 0 and 1 instead of a yes/no
  // decision, and lowers the annealing temperature over the iterations so the
  // assignment hardens gradually.
  Fitter::Config fitterCfg(*m_ipEstimator);
  const auto isFinitePositive = [](double value) { return std::isfinite(value) && value > 0.; };
  const std::vector<double> temperatures = m_annealingTemperatures.empty()
                                               ? Acts::AnnealingUtility::Config{}.setOfTemperatures
                                               : m_annealingTemperatures.value();
  if (!std::ranges::all_of(temperatures, isFinitePositive)) {
    error() << "AnnealingTemperatures must all be finite and positive" << endmsg;
    return StatusCode::FAILURE;
  }
  fitterCfg.annealingTool = Acts::AnnealingUtility(Acts::AnnealingUtility::Config(m_annealingCutOff, temperatures));
  fitterCfg.maxIterations = m_fitterMaxIterations;
  // Always on: smoothing replaces each track's parameters by ones refitted at
  // its vertex, which the output particles report with the vertex as their
  // reference point. Without it they would still be the input parameters.
  fitterCfg.doSmoothing = true;
  fitterCfg.useTime = false;
  fitterCfg.extractParameters.connect<&ConvertedTrack::extractParameters>();
  fitterCfg.trackLinearizer.connect<&Linearizer::linearizeTrack>(&m_linearizer.value());
  Fitter fitter(std::move(fitterCfg), m_actsLogger->cloneWithSuffix("AMVFitter"));

  Finder::Config finderCfg(std::move(fitter), std::move(seeder), *m_ipEstimator, m_magneticField);
  // Initial uncertainty the fit starts from, before any track is added. The
  // Acts default is 1e8 mm^2 in every dimension; shrinking that to the ~1e-6
  // mm^2 of a fitted vertex costs 14 orders of magnitude in the Kalman update
  // and the cancellation leaves negative variances on the diagonal. The
  // spatial value used here is the one Acts' own example uses; time keeps the
  // large default since it is on a different numerical scale (and unused).
  constexpr double initialSpatialVariance = 1e+2 * Acts::UnitConstants::mm * Acts::UnitConstants::mm;
  finderCfg.initialVariances = Acts::Vector4{initialSpatialVariance, initialSpatialVariance, initialSpatialVariance,
                                             finderCfg.initialVariances[3]};
  finderCfg.tracksMaxZinterval = m_tracksMaxZinterval * Acts::UnitConstants::mm;
  finderCfg.tracksMaxSignificance = m_tracksMaxSignificance;
  finderCfg.maxIterations = m_maxIterations;
  finderCfg.maxVertexChi2 = m_maxVertexChi2;
  finderCfg.maxMergeVertexSignificance = m_maxMergeVertexSignificance;
  finderCfg.doFullSplitting = m_doFullSplitting;
  finderCfg.doNotBreakWhileSeeding = m_doNotBreakWhileSeeding;
  finderCfg.useTime = false;
  finderCfg.extractParameters.connect<&ConvertedTrack::extractParameters>();

  if (m_beamSpotPosition.size() != 3 ||
      !std::ranges::all_of(m_beamSpotPosition.value(), [](double value) { return std::isfinite(value); })) {
    error() << "BeamSpotPosition needs 3 finite values (x, y, z)" << endmsg;
    return StatusCode::FAILURE;
  }
  const Acts::Vector3 beamSpotCentre =
      Acts::Vector3{m_beamSpotPosition[0], m_beamSpotPosition[1], m_beamSpotPosition[2]} * Acts::UnitConstants::mm;
  m_beamLine = Acts::Surface::makeShared<Acts::PerigeeSurface>(beamSpotCentre);
  // The seeder places seeds at this position plus the z it finds from the
  // tracks' z0, which refer to the same point.
  m_beamSpot = Acts::Vertex{beamSpotCentre};
  info() << "Beam-spot centre: (" << m_beamSpotPosition[0] << ", " << m_beamSpotPosition[1] << ", "
         << m_beamSpotPosition[2] << ") mm" << endmsg;

  if (!m_beamSpotSize.empty()) {
    // A zero sigma would make the constraint covariance singular, which Acts
    // only rejects when the constraint is used, i.e. in every event.
    if (m_beamSpotSize.size() != 3 || !std::ranges::all_of(m_beamSpotSize.value(), isFinitePositive)) {
      error() << "BeamSpotSize needs 3 finite, positive values (sigma x, y, z)" << endmsg;
      return StatusCode::FAILURE;
    }
    Acts::Vector4 variances;
    for (std::size_t i = 0; i < 3; ++i) {
      const double sigma = m_beamSpotSize[i] * Acts::UnitConstants::mm;
      variances[i] = sigma * sigma;
    }
    variances[3] = finderCfg.initialVariances[3]; // time is not used
    m_beamSpot.setFullCovariance(variances.asDiagonal());
    m_useBeamSpotConstraint = true;

    // Test track compatibility against the seed's uncertainty (the beam spot)
    // instead of treating the seed position as exact.
    finderCfg.useVertexCovForIPEstimation = true;
    info() << "Beam-spot constraint: sigma = (" << m_beamSpotSize[0] << ", " << m_beamSpotSize[1] << ", "
           << m_beamSpotSize[2] << ") mm" << endmsg;
  }

  m_finder.emplace(std::move(finderCfg), m_actsLogger->cloneWithSuffix("AMVFinder"));

  return StatusCode::SUCCESS;
}

std::shared_ptr<const Acts::IVertexFinder> VertexFindingAlg::makeSeeder() const {
  // Both configs derive the squared d0 cut the seeder applies from the
  // significance only when they are constructed.
  const double maxD0Significance = m_seedMaxD0Significance;

  if (m_seeder.value() == "AdaptiveGrid") {
    Acts::AdaptiveGridTrackDensity::Config densityCfg;
    densityCfg.spatialBinExtent = m_spatialBinExtent * Acts::UnitConstants::mm;
    densityCfg.spatialWindow = {-m_spatialWindow * Acts::UnitConstants::mm, m_spatialWindow * Acts::UnitConstants::mm};
    densityCfg.useTime = false;

    Acts::AdaptiveGridDensityVertexFinder::Config cfg{Acts::AdaptiveGridTrackDensity(densityCfg)};
    cfg.extractParameters.connect<&ConvertedTrack::extractParameters>();
    cfg.maxD0TrackSignificance = maxD0Significance;
    cfg.d0SignificanceCut = maxD0Significance * maxD0Significance;
    return std::make_shared<Acts::AdaptiveGridDensityVertexFinder>(cfg);
  }

  if (m_seeder.value() == "Gaussian") {
    Acts::GaussianTrackDensity::Config densityCfg(maxD0Significance);
    densityCfg.extractParameters.connect<&ConvertedTrack::extractParameters>();
    return std::make_shared<Acts::TrackDensityVertexFinder>(
        Acts::TrackDensityVertexFinder::Config{Acts::GaussianTrackDensity(densityCfg)});
  }

  return nullptr;
}

std::vector<VertexFindingAlg::ConvertedTrack>
VertexFindingAlg::convertTracks(const edm4hep::TrackCollection& inputTracks) const {
  namespace EDM4hepDetail = ActsPlugins::EDM4hepUtil::detail;
  auto fieldCache = m_magneticField->makeCache(m_mctx);

  std::vector<ConvertedTrack> convertedTracks;
  convertedTracks.reserve(inputTracks.size());

  for (std::size_t iTrack = 0; iTrack < inputTracks.size(); ++iTrack) {
    // The vertex fit only needs the perigee parameters at the interaction
    // point, so only the AtIP state is converted.
    const auto states = inputTracks[iTrack].getTrackStates();
    const auto ipState =
        std::ranges::find_if(states, [](const auto& state) { return state.location == edm4hep::TrackState::AtIP; });
    if (ipState == states.end()) {
      warning() << "Skipping track without an AtIP track state" << endmsg;
      continue;
    }

    // As readTrack does for this state: q/p needs the local Bz, taken where
    // the particle is.
    const EDM4hepDetail::Parameters edmParams = EDM4hepDetail::unpackTrackState(*ipState);
    double bz = 0.;
    try {
      bz = EDM4hepDetail::localBz(*m_magneticField, fieldCache,
                                  EDM4hepDetail::pointOfClosestApproach(m_gctx, edmParams), *m_actsLogger);
    } catch (const std::exception& e) {
      warning() << "Track conversion failed, skipping track: " << e.what() << endmsg;
      continue;
    }
    Acts::BoundTrackParameters params = EDM4hepDetail::convertTrackParametersFromEdm4hep(bz, edmParams);

    // Express the track at the beam line, see m_beamSpotPosition. The
    // straight-line intersection picks the direction, as in Acts'
    // ImpactPointEstimator.
    if (params.referenceSurface().center(m_gctx) != m_beamLine->center(m_gctx)) {
      Acts::PropagatorPlainOptions propagationOptions(m_gctx, m_mctx);
      const auto intersection =
          m_beamLine
              ->intersect(m_gctx, params.position(m_gctx), params.direction(), Acts::BoundaryTolerance::Infinite())
              .closest();
      propagationOptions.direction = Acts::Direction::fromScalarZeroAsPositive(intersection.pathLength());
      auto atBeamLine = m_propagator->propagateToSurface(params, *m_beamLine, propagationOptions);
      if (!atBeamLine.ok()) {
        warning() << "Propagation to the beam line failed, skipping track: " << atBeamLine.error().message() << endmsg;
        continue;
      }
      params = std::move(*atBeamLine);
    }

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
    Acts::BoundMatrix cov = params.covariance().value() * (m_trackCovarianceScale * m_trackCovarianceScale);
    const double ipErrorTerm = m_impactParameterErrorTerm * Acts::UnitConstants::mm;
    cov(Acts::eBoundLoc0, Acts::eBoundLoc0) += ipErrorTerm * ipErrorTerm;
    cov(Acts::eBoundLoc1, Acts::eBoundLoc1) += ipErrorTerm * ipErrorTerm;
    if (cov(Acts::eBoundTime, Acts::eBoundTime) <= 0.) {
      cov(Acts::eBoundTime, Acts::eBoundTime) =
          m_defaultTimeVariance * Acts::UnitConstants::ns * Acts::UnitConstants::ns;
    }

    // A converted covariance that is not positive definite would poison the
    // vertex fit, and points at a conversion or input problem rather than a
    // vertexing one, so report it instead of letting it propagate silently.
    // A Cholesky decomposition exists exactly for positive-definite matrices.
    if (Eigen::LLT<Acts::BoundMatrix>(cov).info() != Eigen::Success) {
      warning() << "Skipping track whose converted covariance is not positive definite (min diagonal "
                << cov.diagonal().minCoeff() << ", determinant " << cov.determinant() << ")" << endmsg;
      continue;
    }

    params.covariance() = cov;
    convertedTracks.push_back({std::move(params), iTrack});
  }

  debug() << "Converted " << convertedTracks.size() << " of " << inputTracks.size() << " tracks for vertexing"
          << endmsg;

  return convertedTracks;
}

VertexFindingAlg::Output VertexFindingAlg::operator()(const edm4hep::TrackCollection& inputTracks) const {
  // The Acts::InputTrack objects below hold raw pointers into this vector, so
  // it must stay unchanged while the finder runs.
  const std::vector<ConvertedTrack> convertedTracks = convertTracks(inputTracks);
  if (convertedTracks.empty()) {
    return Output{};
  }

  std::vector<Acts::InputTrack> vertexInput;
  vertexInput.reserve(convertedTracks.size());
  for (const auto& track : convertedTracks) {
    vertexInput.emplace_back(&track);
  }

  // With a beam-spot constraint, the fit uses it as a prior; without, the
  // vertex position comes from the tracks alone and the beam spot only
  // places the seeds.
  const Acts::VertexingOptions finderOptions(m_gctx, m_mctx, m_beamSpot, m_useBeamSpotConstraint);
  auto finderState = m_finder->makeState(m_mctx);

  auto result = m_finder->find(vertexInput, finderOptions, finderState);
  if (!result.ok()) {
    error() << "Vertex finding failed: " << result.error().message() << endmsg;
    return Output{};
  }

  return writeVertices(result.value(), inputTracks);
}

VertexFindingAlg::Output VertexFindingAlg::writeVertices(const std::vector<Acts::Vertex>& foundVertices,
                                                         const edm4hep::TrackCollection& inputTracks) const {
  edm4hep::VertexCollection vertices;
  edm4hep::ReconstructedParticleCollection particles;
  edm4hep::VertexRecoParticleLinkCollection vertexParticleLinks;

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
      const auto edmTrack = inputTracks[trackAtVertex.originalParams.as<ConvertedTrack>()->inputIndex];

      const Acts::BoundTrackParameters& atVertex = trackAtVertex.fittedParams;
      const Acts::Vector3 momentum = atVertex.momentum() / Acts::UnitConstants::GeV;
      // An edm4hep track carries no species, so this is the converter's default
      // pion mass.
      const double mass = atVertex.particleHypothesis().mass() / Acts::UnitConstants::GeV;

      auto particle = particles.create();
      particle.setMomentum(
          {static_cast<float>(momentum.x()), static_cast<float>(momentum.y()), static_cast<float>(momentum.z())});
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

  return Output{std::move(vertices), std::move(particles), std::move(vertexParticleLinks)};
}
