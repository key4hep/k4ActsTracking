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

#include "k4ActsTracking/Helpers.hxx"

// edm4hep
#include <edm4hep/MutableTrack.h>
#include <edm4hep/TrackerHit.h>

// Standard
#include <cassert>
#include <filesystem>
#include <memory>
#include <optional>
#include <vector>

// ACTS
#include <Acts/EventData/ParticleHypothesis.hpp>
#include <Acts/MagneticField/InterpolatedBFieldMap.hpp>
#include <Acts/Propagator/ActorList.hpp>
#include <Acts/Propagator/PropagatorOptions.hpp>
#include <Acts/Propagator/detail/CovarianceEngine.hpp>
#include <Acts/Surfaces/BoundaryTolerance.hpp>
#include <Acts/Surfaces/PerigeeSurface.hpp>
#include <Acts/Surfaces/Surface.hpp>
#include <Acts/Utilities/Intersection.hpp>
#include <Acts/Utilities/Logger.hpp>

#include <algorithm>
#include <limits>

// ACTSTracking
#include "config.h.in"

namespace ACTSTracking {

  std::string findFile(const std::string& inpath) {
    if (inpath.empty())
      return inpath;

    // Already absolute path
    if (inpath[0] == '/')
      return inpath;

    // relative to cwd
    if (std::filesystem::exists(inpath)) {
      return inpath;
    }

    // relative to absolute paths
    if (std::filesystem::exists(ACTSTRACKING_SOURCEDIR + inpath)) {
      return ACTSTRACKING_SOURCEDIR + inpath;
    }

    if (std::filesystem::exists(ACTSTRACKING_DATADIR + inpath)) {
      return ACTSTRACKING_DATADIR + inpath;
    }

    // nothing was found :( <- :(((
    return inpath;
  }

  edm4hep::MutableTrack ACTS2edm4hep_track(const Acts::GeometryContext& gctx, const TrackResult& fitter_res,
                                           const HitContainer&                                hits,
                                           std::shared_ptr<const Acts::MagneticFieldProvider> magneticField,
                                           Acts::MagneticFieldProvider::Cache&                magCache) {
    // Create new object
    edm4hep::MutableTrack track{};

    // Basic properties
    track.setChi2(fitter_res.chi2());
    track.setNdf(fitter_res.nDoF());
    track.setNholes(fitter_res.nHoles());

    // Local z-field [Tesla] at a global position
    auto localBz = [&](const Acts::Vector3& pos) -> double {
      Acts::Result<Acts::Vector3> fieldRes = magneticField->getField(pos, magCache);
      if (!fieldRes.ok()) {
        throw std::runtime_error("Field lookup error: " + fieldRes.error().message());
      }
      return (*fieldRes)[2] / Acts::UnitConstants::T;
    };

    // IP track state. The fit reference surface is a perigee at the IP, so the
    // parameters are already in the perigee frame and the referencePoint ends up
    // at the origin.
    Acts::BoundTrackParameters ipParams{fitter_res.referenceSurface().getSharedPtr(), fitter_res.parameters(),
                                        fitter_res.covariance(), fitter_res.particleHypothesis()};
    track.addToTrackStates(ACTSTracking::ACTS2edm4hep_trackState(edm4hep::TrackState::AtIP, gctx, ipParams,
                                                                 localBz(Acts::Vector3(0, 0, 0))));

    std::vector<edm4hep::TrackerHit> hitsOnTrack;
    std::vector<edm4hep::TrackState> statesOnTrack;

    // Handle each measurement track state
    for (const auto& trk_state : fitter_res.trackStatesReversed()) {
      if (!trk_state.hasUncalibratedSourceLink())
        continue;

      auto sl = trk_state.getUncalibratedSourceLink().get<ACTSTracking::SourceLink>();

      assert((sl.index() < hits.size()) and "Source link index is outside the hit container bounds");
      const auto curr_hit = hits[sl.index()];

      hitsOnTrack.push_back(curr_hit);

      const Acts::Vector3 hitPos(curr_hit.getPosition().x, curr_hit.getPosition().y, curr_hit.getPosition().z);

      // Re-express the on-surface state at an ad-hoc perigee at the hit location
      // (handled inside ACTS2edm4hep_trackState) so D0/Z0 and the per-state
      // referencePoint are geometrically consistent. The curvature (omega)
      // conversion uses the local field at the hit.
      Acts::BoundTrackParameters stateParams{trk_state.referenceSurface().getSharedPtr(), trk_state.parameters(),
                                             trk_state.covariance(), fitter_res.particleHypothesis()};
      statesOnTrack.push_back(
          ACTSTracking::ACTS2edm4hep_trackState(edm4hep::TrackState::AtOther, gctx, stateParams, localBz(hitPos)));
    }

    std::reverse(hitsOnTrack.begin(), hitsOnTrack.end());
    std::reverse(statesOnTrack.begin(), statesOnTrack.end());

    // Add Track Hits
    for (const auto& hit : hitsOnTrack) {
      track.addToTrackerHits(hit);
    }

    if (!statesOnTrack.empty()) {
      statesOnTrack.back().location  = edm4hep::TrackState::AtLastHit;
      statesOnTrack.front().location = edm4hep::TrackState::AtFirstHit;
    }

    // Add Track States
    for (const auto& state : statesOnTrack) {
      track.addToTrackStates(state);
    }

    return track;
  }

  edm4hep::TrackState ACTS2edm4hep_trackState(int location, const Acts::GeometryContext& gctx,
                                              const Acts::BoundTrackParameters& params, double Bz) {
    const Acts::Surface& surface = params.referenceSurface();

    // Global position of the parameters on their reference surface.
    const Acts::Vector3 global = surface.localToGlobal(gctx, params.parameters().head<2>(), params.direction());

    // EDM4hep track states use a perigee parametrization (D0, Z0, phi,
    // omega, tanLambda) defined relative to a reference point. If the parameters
    // are not already on a perigee surface, re-express them at an ad-hoc perigee
    // created at their global position, transporting parameters and covariance.
    // Mirrors ActsPlugins EDM4hep::convertTrackParametersToEdm4hep.
    std::shared_ptr<const Acts::Surface> refSurface  = surface.getSharedPtr();
    Acts::BoundVector                    perigeePars = params.parameters();
    Acts::BoundMatrix                    perigeeCov  = params.covariance().value();

    if (dynamic_cast<const Acts::PerigeeSurface*>(refSurface.get()) == nullptr) {
      refSurface = Acts::Surface::makeShared<Acts::PerigeeSurface>(global);

      Acts::Result<Acts::BoundTrackParameters> converted =
          Acts::detail::boundToBoundConversion(gctx, params, *refSurface, Acts::Vector3{0, 0, Bz});
      if (!converted.ok()) {
        throw std::runtime_error("Bound-to-perigee conversion error: " + converted.error().message());
      }
      perigeePars = converted->parameters();
      perigeeCov  = converted->covariance().value();
    }

    edm4hep::TrackState trackState = ACTS2edm4hep_trackState(location, perigeePars, perigeeCov, Bz);

    // Reference point = center of the (perigee) reference surface.
    const Acts::Vector3 center = refSurface->center(gctx);
    trackState.referencePoint  = edm4hep::Vector3f(static_cast<float>(center.x()), static_cast<float>(center.y()),
                                                   static_cast<float>(center.z()));

    return trackState;
  }

  edm4hep::TrackState ACTS2edm4hep_trackState(int location, const Acts::BoundVector& value,
                                              const Acts::BoundMatrix& cov, double Bz) {
    // Create new object
    edm4hep::TrackState trackState{};

    // Basic properties
    trackState.location = location;

    // Trajectory parameters
    // Central values
    double d0     = value[Acts::eBoundLoc0];
    double z0     = value[Acts::eBoundLoc1];
    double phi    = value[Acts::eBoundPhi];
    double theta  = value[Acts::eBoundTheta];
    double qoverp = value[Acts::eBoundQOverP];

    double p         = 1e3 / qoverp;
    double omega     = (0.3 * Bz) / (p * std::sin(theta));
    double lambda    = M_PI / 2 - theta;
    double tanlambda = std::tan(lambda);

    trackState.phi       = phi;
    trackState.tanLambda = tanlambda;
    trackState.omega     = omega;
    trackState.D0        = d0;
    trackState.Z0        = z0;

    // Uncertainties (covariance matrix)
    Acts::Matrix<6, 6> jac = Acts::Matrix<6, 6>::Zero();

    jac(0, Acts::eBoundLoc0) = 1;

    jac(1, Acts::eBoundPhi) = 1;

    jac(2, Acts::eBoundTheta)  = omega / std::tan(theta);
    jac(2, Acts::eBoundQOverP) = omega / qoverp;

    jac(3, Acts::eBoundLoc1) = 1;

    jac(4, Acts::eBoundTheta) = std::pow(1 / std::cos(lambda), 2);

    auto trcov = (jac * cov * jac.transpose());

    int count = 0;
    for (int i = 0; i < 6; ++i) {
      for (int j = 0; j < i; ++j) {
        trackState.covMatrix[count] = trcov(j, i);
        count++;
      }
    }

    return trackState;
  }

  std::optional<edm4hep::TrackState> trackStateAt(const edm4hep::Track& track, int location) {
    for (const auto& state : track.getTrackStates()) {
      if (state.location == location) {
        return state;
      }
    }
    return std::nullopt;
  }

  Acts::ParticleHypothesis convertParticle(const edm4hep::MCParticle mcParticle) {
    switch (mcParticle.getPDG()) {
      case 11:
        return Acts::ParticleHypothesis{Acts::PdgParticle::eElectron};
      case -11:
        return Acts::ParticleHypothesis{Acts::PdgParticle::ePositron};
      case 13:
        return Acts::ParticleHypothesis{Acts::PdgParticle::eMuon};
      case -13:
        return Acts::ParticleHypothesis{Acts::PdgParticle::eAntiMuon};
      case 15:
        return Acts::ParticleHypothesis{Acts::PdgParticle::eTau};
      case -15:
        return Acts::ParticleHypothesis{Acts::PdgParticle::eAntiTau};
      case 22:
        return Acts::ParticleHypothesis{Acts::PdgParticle::eGamma};
      case 111:
        return Acts::ParticleHypothesis{Acts::PdgParticle::ePionZero};
      case 211:
        return Acts::ParticleHypothesis{Acts::PdgParticle::ePionPlus};
      case -211:
        return Acts::ParticleHypothesis{Acts::PdgParticle::ePionMinus};
      case 2112:
        return Acts::ParticleHypothesis{Acts::PdgParticle::eNeutron};
      case -2112:
        return Acts::ParticleHypothesis{Acts::PdgParticle::eAntiNeutron};
      case 2212:
        return Acts::ParticleHypothesis{Acts::PdgParticle::eProton};
      case -2212:
        return Acts::ParticleHypothesis{Acts::PdgParticle::eAntiProton};
    }

    Acts::PdgParticle      pdg  = Acts::PdgParticle::eInvalid;
    float                  mass = 0.0f;
    Acts::ChargeHypothesis charge_type{0.0f};
    return Acts::ParticleHypothesis{pdg, mass, charge_type};
  }

  namespace {
    /// Abort condition for the calorimeter-face extrapolation: terminate the
    /// propagation as soon as the navigator's current surface is one of the
    /// calorimeter-face surfaces. Works with the geometry navigator, which sets
    /// the current surface as it visits the calo volumes' passive surfaces.
    struct CaloSurfaceReached {
      const std::vector<Acts::GeometryIdentifier>* caloIds = nullptr;

      template <typename propagator_state_t, typename stepper_t, typename navigator_t>
      bool checkAbort(propagator_state_t& state, const stepper_t& /*stepper*/, const navigator_t& navigator,
                      const Acts::Logger& /*logger*/) const {
        if (caloIds == nullptr) {
          return false;
        }
        const Acts::Surface* current = navigator.currentSurface(state.navigation);
        if (current == nullptr) {
          return false;
        }
        return std::find(caloIds->begin(), caloIds->end(), current->geometryId()) != caloIds->end();
      }
    };
  }  // namespace

  CaloExtrapolationResult extrapolateToCaloFace(const CaloFacePropagator&                    propagator,
                                                const Acts::BoundTrackParameters&            start,
                                                const std::vector<Acts::GeometryIdentifier>& caloSurfaceGeoIds,
                                                const Acts::GeometryContext& gctx, const Acts::MagneticFieldContext& mctx,
                                                std::size_t maxSteps) {
    if (caloSurfaceGeoIds.empty()) {
      return {std::nullopt, CaloExtrapolationStatus::NoSurfaces};
    }

    using ActorList = Acts::ActorList<CaloSurfaceReached>;
    using Options   = CaloFacePropagator::Options<ActorList>;

    Options options{gctx, mctx};
    options.maxSteps                                    = maxSteps;
    options.actorList.get<CaloSurfaceReached>().caloIds = &caloSurfaceGeoIds;

    auto result = propagator.propagate(start, options);
    if (!result.ok()) {
      return {std::nullopt, CaloExtrapolationStatus::PropagationError};
    }

    const auto& output = result.value();
    if (!output.endParameters.has_value()) {
      return {std::nullopt, CaloExtrapolationStatus::NotReached};
    }

    // The propagation may also terminate at the world boundary; only treat it as
    // a success if it actually finished on a calo-face surface.
    const auto& endParams = output.endParameters.value();
    const auto  endId     = endParams.referenceSurface().geometryId();
    if (std::find(caloSurfaceGeoIds.begin(), caloSurfaceGeoIds.end(), endId) == caloSurfaceGeoIds.end()) {
      return {std::nullopt, CaloExtrapolationStatus::NotReached};
    }

    return {endParams, CaloExtrapolationStatus::Ok};
  }

}  // namespace ACTSTracking
