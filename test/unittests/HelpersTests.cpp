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
#include <edm4hep/TrackState.h>

// ACTS
#include <Acts/Definitions/TrackParametrization.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

using Catch::Approx;

namespace {
  // Build a track whose track states are stored in the order produced by
  // ACTS2edm4hep_track: AtIP first (index 0), then the hit states.
  edm4hep::MutableTrack makeTrackWithStates() {
    edm4hep::MutableTrack track;

    edm4hep::TrackState ip;
    ip.location  = edm4hep::TrackState::AtIP;
    ip.D0        = 1.5f;
    ip.Z0        = 2.5f;
    ip.tanLambda = 0.25f;
    ip.omega     = 0.001f;
    track.addToTrackStates(ip);

    edm4hep::TrackState firstHit;
    firstHit.location  = edm4hep::TrackState::AtFirstHit;
    firstHit.D0        = 10.0f;
    firstHit.Z0        = 20.0f;
    firstHit.tanLambda = 2.5f;
    firstHit.omega     = 0.002f;
    track.addToTrackStates(firstHit);

    edm4hep::TrackState lastHit;
    lastHit.location = edm4hep::TrackState::AtLastHit;
    lastHit.D0       = 30.0f;
    track.addToTrackStates(lastHit);

    return track;
  }
}  // namespace

TEST_CASE("trackStateAt returns the state matching the requested location") {
  const edm4hep::MutableTrack track = makeTrackWithStates();

  const auto ip = ACTSTracking::trackStateAt(track, edm4hep::TrackState::AtIP);
  REQUIRE(ip.has_value());
  CHECK(ip->location == edm4hep::TrackState::AtIP);
  CHECK(ip->D0 == Approx(1.5f));
  CHECK(ip->Z0 == Approx(2.5f));

  const auto firstHit = ACTSTracking::trackStateAt(track, edm4hep::TrackState::AtFirstHit);
  REQUIRE(firstHit.has_value());
  CHECK(firstHit->location == edm4hep::TrackState::AtFirstHit);
  CHECK(firstHit->D0 == Approx(10.0f));
}

TEST_CASE("trackStateAt returns nullopt when no state has the location") {
  const edm4hep::MutableTrack track = makeTrackWithStates();
  CHECK_FALSE(ACTSTracking::trackStateAt(track, edm4hep::TrackState::AtCalorimeter).has_value());

  const edm4hep::MutableTrack empty;
  CHECK_FALSE(ACTSTracking::trackStateAt(empty, edm4hep::TrackState::AtIP).has_value());
}

// Regression guard for the positional-vs-location getter bug: edm4hep's
// Track::getTrackStates(i) is indexed by position, and AtIP == 1, so
// getTrackStates(AtIP) returns the index-1 (first-hit) state, NOT the AtIP one.
// trackStateAt must resolve the *real* AtIP state regardless of its index.
TEST_CASE("trackStateAt is not fooled by the positional getTrackStates getter") {
  const edm4hep::MutableTrack track = makeTrackWithStates();

  const auto ip = ACTSTracking::trackStateAt(track, edm4hep::TrackState::AtIP);
  REQUIRE(ip.has_value());

  const edm4hep::TrackState positional = track.getTrackStates(edm4hep::TrackState::AtIP);
  CHECK(positional.location == edm4hep::TrackState::AtFirstHit);  // documents the footgun
  CHECK(ip->D0 == Approx(1.5f));                                  // trackStateAt gets it right
  CHECK(ip->D0 != Approx(positional.D0));
}

TEST_CASE("ACTS2edm4hep_trackState converts perigee parameters and derived quantities") {
  Acts::BoundVector params      = Acts::BoundVector::Zero();
  params[Acts::eBoundLoc0]      = 2.0;    // D0
  params[Acts::eBoundLoc1]      = 5.0;    // Z0
  params[Acts::eBoundPhi]       = 0.5;    // phi
  params[Acts::eBoundTheta]     = 1.0;    // theta [rad]
  params[Acts::eBoundQOverP]    = 0.5;    // q/p
  const Acts::BoundMatrix cov   = Acts::BoundMatrix::Zero();
  const double            Bz    = 2.0;    // Tesla

  const edm4hep::TrackState ts =
      ACTSTracking::ACTS2edm4hep_trackState(edm4hep::TrackState::AtIP, params, cov, Bz);

  const double p              = 1e3 / 0.5;
  const double expectedOmega  = (0.3 * Bz) / (p * std::sin(1.0));
  const double expectedTanLam = std::tan(M_PI / 2.0 - 1.0);

  CHECK(ts.location == edm4hep::TrackState::AtIP);
  CHECK(ts.D0 == Approx(2.0));
  CHECK(ts.Z0 == Approx(5.0));
  CHECK(ts.phi == Approx(0.5));
  CHECK(ts.omega == Approx(expectedOmega));
  CHECK(ts.tanLambda == Approx(expectedTanLam));
}
