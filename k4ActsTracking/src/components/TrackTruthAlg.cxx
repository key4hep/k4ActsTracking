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

#include "k4ActsTracking/TrackTruthAlg.hxx"

// ACTSTracking
#include "k4ActsTracking/Helpers.hxx"

// TBB
#include <tbb/blocked_range.h>
#include <tbb/combinable.h>
#include <tbb/parallel_for.h>

// edm4hep
#include <edm4hep/MCParticle.h>
#include <edm4hep/SimTrackerHit.h>
#include <edm4hep/Track.h>
#include <edm4hep/TrackerHitPlane.h>
#include <edm4hep/TrackerHitSimTrackerHitLinkCollection.h>

//------------------------------------------------------------------------------------------------

DECLARE_COMPONENT(TrackTruthAlg)

TrackTruthAlg::TrackTruthAlg(const std::string& name, ISvcLocator* svcLoc)
    : MultiTransformer(name, svcLoc,
                       {KeyValues("InputTrackCollectionName", {"Tracks"}),
                        KeyValues("InputTrackerHit2SimTrackerHitRelationName", {"TrackMCRelation"})},
                       {KeyValues("OutputParticle2TrackRelationName", {"Particle2TrackRelationName"})}) {}

std::tuple<edm4hep::TrackMCParticleLinkCollection> TrackTruthAlg::operator()(
    const edm4hep::TrackCollection&                       tracks,
    const edm4hep::TrackerHitSimTrackerHitLinkCollection& trackerHitRelations) const {
  // Map TrackerHits to SimTrackerHits
  std::map<edm4hep::TrackerHit, edm4hep::SimTrackerHit> trackerHit2SimHit;
  for (const auto& hitRel : trackerHitRelations) {
    trackerHit2SimHit.emplace(hitRel.getFrom(), hitRel.getTo());
  }

  debug() << "Map size: " << trackerHit2SimHit.size() << endmsg;

  // Best match ifo for each MCParticle
  struct MatchInfo {
    edm4hep::Track track;
    float          frac = 0.f;
  };

  // Thread-local best maps
  tbb::combinable<std::map<edm4hep::MCParticle, MatchInfo>> tlsBestMaps;
  auto parallelTrackSearching = [&](const tbb::blocked_range<size_t>& r) {
    // Each TBB worker has its own best map
    auto& localBest = tlsBestMaps.local();
    // Count hits per MCParticle for this track
    std::map<edm4hep::MCParticle, uint32_t> trackHit2Mc;

    for (std::size_t iTrack = r.begin(); iTrack != r.end(); ++iTrack) {
      const auto& track      = tracks[iTrack];
      const auto& trackHits  = track.getTrackerHits();
      const auto& nTrackHits = trackHits.size();

      if (nTrackHits == 0) {
        continue;
      }

      // Clear the per-track hit counter
      trackHit2Mc.clear();

      for (const auto& hit : trackHits) {
        auto it = trackerHit2SimHit.find(hit);
        if (it == trackerHit2SimHit.end()) {
          continue;  // No sim hit found for this tracker hit
        }

        const auto& simHit   = it->second;
        auto        particle = simHit.getParticle();
        if (particle.isAvailable()) {
          ++trackHit2Mc[particle];  //Increment MC Particle counter
        }
      }

      // Update Best Matches
      for (const auto& [mcParticle, hitCount] : trackHit2Mc) {
        const float frac = static_cast<float>(hitCount) / static_cast<float>(nTrackHits);

        auto       matchIt = localBest.find(mcParticle);
        const bool better  = (matchIt == localBest.end()) ||  // no best matches exist
                            ((matchIt->second).frac < frac);  // this match is better (more hits on track)
        if (better) {
          auto& matchInfo = (matchIt == localBest.end()) ? localBest[mcParticle] : matchIt->second;
          matchInfo.track = track;
          matchInfo.frac  = frac;
        }
      }
    }  // for each track
  };  // parallelTrackSearching

  // Run in parallel if more than one thread is requested
  if (m_numThreads > 1) {
    tbb::task_arena arena(m_numThreads.value());
    arena.execute([&] { tbb::parallel_for(tbb::blocked_range<size_t>(0, tracks.size()), parallelTrackSearching); });
  } else {  // Serial execution
    parallelTrackSearching(tbb::blocked_range<size_t>(0, tracks.size()));
  }

  // Combine the thread-local best maps
  std::map<edm4hep::MCParticle, MatchInfo> mcBestMatch;
  tlsBestMaps.combine_each([&](const std::map<edm4hep::MCParticle, MatchInfo>& localBest) {
    for (const auto& [mcParticle, matchInfo] : localBest) {
      auto       matchIt = mcBestMatch.find(mcParticle);
      const bool better  = (matchIt == mcBestMatch.end()) ||          // no best matches exist
                          ((matchIt->second).frac < matchInfo.frac);  // this match is better (more hits on track)
      if (better) {
        mcBestMatch[mcParticle] = matchInfo;
      }
    }
  });

  // Save the best matches
  edm4hep::TrackMCParticleLinkCollection outColMC2T;
  for (const auto& [mcParticle, matchInfo] : mcBestMatch) {
    edm4hep::MutableTrackMCParticleLink link = outColMC2T.create();
    link.setFrom(matchInfo.track);
    link.setTo(mcParticle);
    link.setWeight(matchInfo.frac);
  }

  return std::make_tuple(std::move(outColMC2T));
}
