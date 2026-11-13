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

#include <edm4hep/TrackCollection.h>

#include <Gaudi/Property.h>
#include <k4FWCore/Transformer.h>

#include <Acts/AmbiguityResolution/GreedyAmbiguityResolution.hpp>

#include <podio/ObjectID.h>

#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

/// ACTS' GreedyAmbiguityResolution on an edm4hep track collection: removes tracks
/// with shared hits, worst first, until no track shares more than
/// MaximumSharedHits hits. The ACTS state is filled from the tracks' hits as in
/// GreedyAmbiguityResolution::computeInitialState. Output: subset collection.
struct GreedyAmbiguityResolutionAlg final
    : k4FWCore::Transformer<edm4hep::TrackCollection(const edm4hep::TrackCollection&)> {
  GreedyAmbiguityResolutionAlg(const std::string& name, ISvcLocator* svcLoc)
      : Transformer(name, svcLoc, {KeyValues("InputTrackCollectionName", {"Tracks"})},
                    {KeyValues("OutputTrackCollectionName", {"ResolvedTracks"})}) {}

  StatusCode initialize() override {
    Acts::GreedyAmbiguityResolution::Config cfg;
    cfg.maximumSharedHits = m_maximumSharedHits;
    cfg.maximumIterations = m_maximumIterations;
    cfg.nMeasurementsMin = m_nMeasurementsMin;
    m_resolution.emplace(cfg, makeActsGaudiLogger(this));
    return Transformer::initialize();
  }

  edm4hep::TrackCollection operator()(const edm4hep::TrackCollection& tracks) const override {
    Acts::GreedyAmbiguityResolution::State state;
    std::unordered_map<podio::ObjectID, std::size_t> measurementIndex;
    for (std::size_t i = 0; i < tracks.size(); ++i) {
      const auto& track = tracks[i];
      if (track.getTrackerHits().size() < m_nMeasurementsMin) {
        continue;
      }
      std::vector<std::size_t> measurements;
      for (const auto& hit : track.getTrackerHits()) {
        measurements.push_back(measurementIndex.try_emplace(hit.getObjectID(), measurementIndex.size()).first->second);
      }
      state.trackTips.push_back(static_cast<int>(i));
      // chi2/ndf as ACTS; a track without degrees of freedom ranks last
      state.trackChi2.push_back(track.getNdf() > 0 ? track.getChi2() / track.getNdf()
                                                   : std::numeric_limits<float>::max());
      state.measurementsPerTrack.push_back(std::move(measurements));
      state.splitMeasurementsPerTrack.emplace_back();
      state.selectedTracks.insert(state.numberOfTracks);
      ++state.numberOfTracks;
    }
    for (std::size_t iTrack = 0; iTrack < state.numberOfTracks; ++iTrack) {
      for (auto iMeasurement : state.measurementsPerTrack[iTrack]) {
        state.tracksPerMeasurement[iMeasurement].insert(iTrack);
      }
    }
    state.sharedMeasurementsPerTrack.assign(state.numberOfTracks, 0);
    for (std::size_t iTrack = 0; iTrack < state.numberOfTracks; ++iTrack) {
      for (auto iMeasurement : state.measurementsPerTrack[iTrack]) {
        if (state.tracksPerMeasurement[iMeasurement].size() > 1) {
          ++state.sharedMeasurementsPerTrack[iTrack];
        }
      }
    }

    m_resolution->resolve(state);

    edm4hep::TrackCollection out;
    out.setSubsetCollection();
    for (auto iTrack : state.selectedTracks) {
      out.push_back(tracks[state.trackTips[iTrack]]);
    }
    return out;
  }

private:
  Gaudi::Property<std::uint32_t> m_maximumSharedHits{this, "MaximumSharedHits", 0, "Maximum shared hits per track"};
  Gaudi::Property<std::uint32_t> m_maximumIterations{this, "MaximumIterations", 100000, "Maximum iterations"};
  Gaudi::Property<std::size_t> m_nMeasurementsMin{this, "NMeasurementsMin", 4, "Minimum hits per track"};
  std::optional<Acts::GreedyAmbiguityResolution> m_resolution;
};

DECLARE_COMPONENT(GreedyAmbiguityResolutionAlg)
