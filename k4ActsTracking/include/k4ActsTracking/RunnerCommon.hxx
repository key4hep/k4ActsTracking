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
#pragma once

// Shared ACTS type aliases and small helpers used by both the combinatorial
// (CKFRunner) and standalone-fit (KFRunner) track runners.

// k4ActsTracking
#include "k4ActsTracking/IActsGeoSvc.h"

// edm4hep
#include <edm4hep/TrackerHit.h>

// podio
#include <podio/ObjectID.h>

// ACTS
#include <Acts/EventData/TrackContainer.hpp>
#include <Acts/EventData/VectorMultiTrajectory.hpp>
#include <Acts/EventData/VectorTrackContainer.hpp>
#include <Acts/Propagator/EigenStepper.hpp>
#include <Acts/Propagator/Navigator.hpp>
#include <Acts/Propagator/Propagator.hpp>

// Standard
#include <cstddef>
#include <cstdint>
#include <memory>

namespace ACTSTracking {

  /// Default maximum number of propagation steps shared by the track runners
  /// (CKFRunner, KFRunner). Single source of truth for the runners' Config
  /// defaults and the algorithms' MaxPropagationSteps property default.
  inline constexpr std::size_t kDefaultMaxPropagationSteps = 10000;

  /// Track container shared by the CKF and Kalman-fit runners.
  using CKFTrackContainer =
      Acts::TrackContainer<Acts::VectorTrackContainer, Acts::VectorMultiTrajectory, std::shared_ptr>;

  /// Stepper / navigator / propagator shared by the runners.
  using CKFStepper    = Acts::EigenStepper<>;
  using CKFNavigator  = Acts::Navigator;
  using CKFPropagator = Acts::Propagator<CKFStepper, CKFNavigator>;

  /// Build a propagator over the tracking geometry. @p resolvePassive selects
  /// whether passive surfaces (e.g. calorimeter faces) are resolved during
  /// navigation. The result depends only on the geometry and field, so callers
  /// can build the propagators they need once and reuse them across events.
  inline CKFPropagator makePropagator(const IActsGeoSvc& geo, bool resolvePassive) {
    CKFNavigator::Config navigatorCfg{geo.trackingGeometry()};
    navigatorCfg.resolvePassive   = resolvePassive;
    navigatorCfg.resolveMaterial  = true;
    navigatorCfg.resolveSensitive = true;
    return CKFPropagator(CKFStepper(geo.magneticField()), CKFNavigator(navigatorCfg));
  }

  /// Stable key for matching an edm4hep tracker hit across collections.
  inline std::uint64_t trackerHitKey(const edm4hep::TrackerHit& hit) {
    const podio::ObjectID id = hit.getObjectID();
    return (static_cast<std::uint64_t>(id.collectionID) << 32) | static_cast<std::uint32_t>(id.index);
  }

}  // namespace ACTSTracking
