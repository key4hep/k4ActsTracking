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
#include "catch2/catch_test_macros.hpp"

#include <k4ActsTracking/RunnerCommon.hxx>

#include <edm4hep/TrackCollection.h>
#include <edm4hep/TrackerHitPlaneCollection.h>

TEST_CASE("collectSeedHits distinguishes hits from different unwritten collections", "[seedhits]") {
  edm4hep::TrackerHitPlaneCollection first, second, merged;
  auto outer = first.create();
  outer.setPosition({20., 0., 0.});
  auto inner = second.create();
  inner.setPosition({10., 0., 0.});
  REQUIRE(outer.getObjectID() == inner.getObjectID());

  merged.setSubsetCollection();
  merged.push_back(outer);
  merged.push_back(inner);
  ACTSTracking::HitContainer hitContainer;
  ACTSTracking::SourceLinksByHit slByHit;
  for (const auto& hit : merged) {
    slByHit.emplace(hit, ACTSTracking::SourceLink(Acts::GeometryIdentifier{}, hitContainer.size()));
    hitContainer.push_back(hit);
  }

  // The GNN passes planar hits; CKF from seeds passes the track's hit relations.
  edm4hep::TrackCollection tracks;
  auto track = tracks.create();
  track.addToTrackerHits(outer);
  track.addToTrackerHits(inner);
  const auto checkSeedHits = [&](const auto& candidateHits) {
    const auto seedHits = ACTSTracking::collectSeedHits(candidateHits, slByHit, hitContainer);
    REQUIRE(seedHits.size() == 2);
    CHECK(seedHits[0].sl.index() == 1);
    CHECK(seedHits[0].r == 10.);
    CHECK(seedHits[1].sl.index() == 0);
    CHECK(seedHits[1].r == 20.);
  };
  checkSeedHits(merged);
  checkSeedHits(track.getTrackerHits());
}
