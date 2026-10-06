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

#include "k4ActsTracking/ACTSSeededCKFTrackingAlg.hxx"

#include <Acts/Surfaces/PerigeeSurface.hpp>
#include <Acts/Utilities/TrackHelpers.hpp>

#include <chrono>

// Instantiate fitting separately from the template-heavy triplet seeding code.
StatusCode ACTSSeededCKFTrackingAlg::tracking(const std::vector<Acts::BoundTrackParameters>& paramseeds,
                                              const CKF& trackFinder, const TrackFinderOptions& ckfOptions,
                                              const Propagator& extrapPropagator,
                                              const Acts::PerigeeSurface& perigeeSurface,
                                              Propagator::Options<>& extrapOptions,
                                              const ACTSTracking::HitContainer& hits,
                                              edm4hep::TrackCollection& trackCollection) const {
  // Initialize track finder
  debug() << "Starting CKF track finding with " << paramseeds.size() << " seeds." << endmsg;

  auto trackContainer = std::make_shared<Acts::VectorTrackContainer>();
  auto trackStateContainer = std::make_shared<Acts::VectorMultiTrajectory>();
  TrackContainer tracks(trackContainer, trackStateContainer);

  auto trackStart = std::chrono::high_resolution_clock::now();

  for (std::size_t iseed = 0; iseed < paramseeds.size(); ++iseed) {
    tracks.clear();

    auto result = trackFinder.findTracks(paramseeds.at(iseed), ckfOptions, tracks);
    if (result.ok()) {
      const auto& fitOutput = result.value();
      for (const TrackContainer::TrackProxy& trackItem : fitOutput) {
        // Track smoothing
        auto trackTip = tracks.makeTrack();
        trackTip.copyFrom(trackItem);
        auto smoothResult = Acts::smoothTrack(geometryContext(), trackTip);
        if (!smoothResult.ok()) {
          warning() << "Track smoothing error: " << smoothResult.error() << endmsg;
          continue;
        }

        // Extrapolate the fitted track back to the perigee surface at the IP so
        // that the track parameters (in particular D0 and Z0) are expressed
        // there. This reproduces the TrackStateAtIP behaviour from older ACTS.
        auto extrapResult = Acts::extrapolateTrackToReferenceSurface(
            trackTip, perigeeSurface, extrapPropagator, extrapOptions, Acts::TrackExtrapolationStrategy::firstOrLast);
        if (!extrapResult.ok()) {
          warning() << "Track extrapolation to perigee failed: " << extrapResult.error() << endmsg;
          continue;
        }

        // Helpful debug output
        debug() << "Trajectory Summary" << endmsg;
        debug() << "\tchi2Sum       " << trackTip.chi2() << endmsg;
        debug() << "\tNDF           " << trackTip.nDoF() << endmsg;
        debug() << "\tnHoles        " << trackTip.nHoles() << endmsg;
        debug() << "\tnMeasurements " << trackTip.nMeasurements() << endmsg;
        debug() << "\tnOutliers     " << trackTip.nOutliers() << endmsg;
        debug() << "\tnStates       " << trackTip.nTrackStates() << endmsg;

        // Make track object
        auto track = ACTSTracking::ACTS2edm4hep_track(geometryContext(), magneticFieldContext(), trackTip, hits,
                                                      magneticField());

        // Save results
        {
          std::lock_guard lock{m_trackMutex};
          trackCollection.push_back(track);
        }
      }
    } else {
      warning() << "Track fit error: " << result.error() << endmsg;
    }
  }

  auto trackEnd = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> trackDuration = trackEnd - trackStart;
  // m_histTrackBuild->Fill(trackDuration.count());

  return StatusCode::SUCCESS;
}
