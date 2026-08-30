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
#include <Acts/EventData/AnyTrackStateProxy.hpp>
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

// ActsPlugins: centralised ACTS -> EDM4hep conversion
#include <ActsPlugins/EDM4hep/EDM4hepUtil.hpp>

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

  edm4hep::MutableTrack ACTS2edm4hep_track(const Acts::GeometryContext& gctx, const Acts::MagneticFieldContext& mctx,
                                           const TrackResult& fitter_res, const HitContainer& hits,
                                           std::shared_ptr<const Acts::MagneticFieldProvider> magneticField) {
    // Create new object
    edm4hep::MutableTrack track{};

    // Resolve the edm4hep hit behind a measurement track state. This is the one
    // piece of the conversion the centralised converter cannot do on its own,
    // since it depends on our SourceLink/HitContainer convention: the source
    // link index() is the position of the hit in the parallel HitContainer.
    ActsPlugins::EDM4hepUtil::TrackerHitLookup hitLookup =
        [&hits](const Acts::AnyConstTrackStateProxy& state) -> std::optional<edm4hep::TrackerHit> {
      if (!state.hasUncalibratedSourceLink()) {
        return std::nullopt;
      }
      const auto sl = state.getUncalibratedSourceLink().get<ACTSTracking::SourceLink>();
      assert((sl.index() < hits.size()) and "Source link index is outside the hit container bounds");
      if (sl.index() >= hits.size()) {
        return std::nullopt;
      }
      return hits[sl.index()];
    };

    // Everything else (chi2/ndf/nHoles, the AtIP state, one state per
    // measurement, the perigee re-expression and the local-field dependent
    // omega conversion) is handled by the centralised converter.
    ActsPlugins::EDM4hepUtil::writeTrack(gctx, mctx, fitter_res, track, *magneticField, hitLookup);

    return track;
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
                                                const Acts::GeometryContext&                 gctx,
                                                const Acts::MagneticFieldContext& mctx, std::size_t maxSteps) {
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
