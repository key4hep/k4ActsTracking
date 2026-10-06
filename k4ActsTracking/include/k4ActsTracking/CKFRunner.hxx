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

#include "k4ActsTracking/RunnerCommon.hxx"

#include <edm4hep/TrackCollection.h>

#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <vector>

namespace Gaudi {
class Algorithm;
}

namespace ACTSTracking {

/// Convert estimated seed parameters into an edm4hep seed TrackState.
template <class Alg>
edm4hep::TrackState makeSeedTrackState(const Alg& /*alg*/, const IActsGeoSvc& geo, const Acts::GeometryContext& geoCtx,
                                       const Acts::BoundTrackParameters& paramseed,
                                       Acts::MagneticFieldProvider::Cache& magCache) {
  return ActsPlugins::EDM4hepUtil::writeTrackState(geoCtx, edm4hep::TrackState::AtFirstHit, paramseed,
                                                   *geo.magneticField(), magCache);
}

/// Append a seed's hits and track state, serialising the collection mutation.
template <class HitRange>
void appendSeedTrack(edm4hep::TrackCollection& seedCollection, std::mutex& seedMutex,
                     const edm4hep::TrackState& seedTrackState, const HitRange& trackerHits) {
  std::lock_guard<std::mutex> lock(seedMutex);
  auto seedTrack = seedCollection.create();
  for (const auto& hit : trackerHits) {
    seedTrack.addToTrackerHits(hit);
  }
  seedTrack.addToTrackStates(seedTrackState);
}

/**
 * @brief Owns the ACTS Combinatorial Kalman Filter and runs it over seeds.
 *
 * Construct once and reuse across events. Concurrent findTracks() calls bind
 * their own event-local data and track containers; the shared propagators and
 * CKF are read-only. Output writes are serialised by the caller's mutex.
 * Non-copyable and non-movable.
 */
class CKFRunner {
public:
  struct Config {
    double chi2CutOff = 15;
    std::int32_t numMeasurementsCutOff = 10;
    double chi2CutOffOutlier = std::numeric_limits<double>::max();
    bool propagateBackward = false;
    bool extrapolateToCalo = false;
    std::size_t maxSteps = kDefaultMaxPropagationSteps;

    /// Add a second AtCalorimeter state when a barrel track reaches an endcap.
    bool addEndcapCaloState = false;

    /// Optionally stop branches on too many holes/outliers or low pT.
    bool useBranchStopper = false;
    int bsMaxHoles = 2;
    int bsMaxOutliers = 2;
    int bsMinMeasurements = 6;
    double bsPtMin = 0.0; ///< GeV; <= 0 disables the pT branch stop
    int bsPtMinMeasurements = 3;

    /// Reference for the AtIP state; null selects a perigee at the origin.
    /// Telescope clients may instead pass a plane perpendicular to the beam.
    std::shared_ptr<const Acts::Surface> referenceSurface{nullptr};
  };

  /// Build the event-independent propagators and CKF, including the reference
  /// extrapolator that gives the AtIP state well-defined beamline parameters.
  CKFRunner(const IActsGeoSvc& geo, const Config& cfg);
  ~CKFRunner();

  CKFRunner(const CKFRunner&) = delete;
  CKFRunner(CKFRunner&&) = delete;
  CKFRunner& operator=(const CKFRunner&) = delete;
  CKFRunner& operator=(CKFRunner&&) = delete;

  /// Find, smooth and convert tracks using event-local data. Optionally append
  /// calorimeter states and update the caller's calorimeter-face counters.
  void findTracks(const Gaudi::Algorithm& alg, const MeasurementContainer& measurements,
                  const SourceLinkContainer& sourceLinks, const HitContainer& hits,
                  const std::vector<Acts::BoundTrackParameters>& paramseeds,
                  Acts::MagneticFieldProvider::Cache& magCache, edm4hep::TrackCollection& trackCollection,
                  std::mutex& trackMutex, const CaloExtrapMonitor* caloMonitor = nullptr) const;

private:
  struct Impl;
  std::unique_ptr<Impl> m_impl;
};

} // namespace ACTSTracking
