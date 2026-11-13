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

#include <edm4hep/TrackCollection.h>
#include <edm4hep/TrackerHitPlaneCollection.h>

#include <k4FWCore/Transformer.h>

#include <podio/ObjectID.h>

#include <string>
#include <unordered_set>
#include <vector>

/// The hits not used by any of the given tracks (a subset collection), as input
/// for a further tracking pass.
struct UnusedHitsAlg final
    : k4FWCore::Transformer<edm4hep::TrackerHitPlaneCollection(const edm4hep::TrackerHitPlaneCollection&,
                                                               const std::vector<const edm4hep::TrackCollection*>&)> {
  UnusedHitsAlg(const std::string& name, ISvcLocator* svcLoc)
      : Transformer(name, svcLoc,
                    {KeyValues("InputTrackerHitCollection", {"AllTrackerHits"}),
                     KeyValues("InputTrackCollections", {"CKFTracks"})},
                    {KeyValues("OutputTrackerHitCollection", {"UnusedTrackerHits"})}) {}

  edm4hep::TrackerHitPlaneCollection
  operator()(const edm4hep::TrackerHitPlaneCollection& hits,
             const std::vector<const edm4hep::TrackCollection*>& trackCollections) const override {
    std::unordered_set<podio::ObjectID> used;
    for (const auto* tracks : trackCollections) {
      for (const auto& track : *tracks) {
        for (const auto& hit : track.getTrackerHits()) {
          used.insert(hit.getObjectID());
        }
      }
    }
    edm4hep::TrackerHitPlaneCollection unused;
    unused.setSubsetCollection();
    for (const auto& hit : hits) {
      if (!used.contains(hit.getObjectID())) {
        unused.push_back(hit);
      }
    }
    return unused;
  }
};

DECLARE_COMPONENT(UnusedHitsAlg)
