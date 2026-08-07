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
#include "catch2/matchers/catch_matchers_floating_point.hpp"

#include <k4ActsTracking/HitFeatures.hxx>

#include <edm4hep/TrackerHitPlaneCollection.h>

#include <DDSegmentation/BitFieldCoder.h>

#include <cmath>
#include <string>
#include <vector>

namespace {
  const std::string kEncoding = "system:5,side:-2,layer:6,module:11,sensor:8";

  edm4hep::MutableTrackerHitPlane makeHit(float x, float y, float z, float time = 0.f, float eDep = 0.f,
                                          std::uint64_t cellID = 0) {
    edm4hep::MutableTrackerHitPlane hit{};
    hit.setPosition({x, y, z});
    hit.setTime(time);
    hit.setEDep(eDep);
    hit.setCellID(cellID);
    return hit;
  }

  float valueOf(const std::string& feature, const edm4hep::TrackerHitPlane& hit,
                const dd4hep::DDSegmentation::BitFieldCoder* decoder = nullptr) {
    return ACTSTracking::hitFeatureValue(hit, ACTSTracking::resolveHitFeature(feature, decoder), decoder);
  }
}  // namespace

TEST_CASE("resolveHitFeature") {
  dd4hep::DDSegmentation::BitFieldCoder decoder{kEncoding};

  SECTION("geometric features need no decoder") {
    for (const auto& name : {"x", "y", "z", "r", "phi", "theta", "eta", "t", "time", "E", "energy"}) {
      REQUIRE_NOTHROW(ACTSTracking::resolveHitFeature(name, nullptr));
    }
  }

  SECTION("names are case insensitive") {
    const auto hit = makeHit(3.f, 4.f, 5.f);
    REQUIRE(valueOf("R", hit) == valueOf("r", hit));
    REQUIRE(valueOf("PHI", hit) == valueOf("phi", hit));
    REQUIRE(valueOf("Energy", hit) == valueOf("energy", hit));
  }

  SECTION("unknown names throw") {
    REQUIRE_THROWS(ACTSTracking::resolveHitFeature("not_a_feature", &decoder));
    REQUIRE_THROWS(ACTSTracking::resolveHitFeature("", &decoder));
  }

  SECTION("CellID features need a decoder") {
    REQUIRE(ACTSTracking::hitFeatureNeedsCellID("layer_id"));
    REQUIRE(ACTSTracking::hitFeatureNeedsCellID("module_id"));
    REQUIRE(ACTSTracking::hitFeatureNeedsCellID("system_id"));
    REQUIRE(ACTSTracking::hitFeatureNeedsCellID("volume_id"));
    REQUIRE_FALSE(ACTSTracking::hitFeatureNeedsCellID("r"));
    REQUIRE_FALSE(ACTSTracking::hitFeatureNeedsCellID("not_a_feature"));

    REQUIRE_NOTHROW(ACTSTracking::resolveHitFeature("layer_id", &decoder));
    // Without a decoder these cannot be resolved, rather than silently yielding 0
    REQUIRE_THROWS(ACTSTracking::resolveHitFeature("layer_id", nullptr));
  }

  SECTION("a field missing from the encoding throws") {
    dd4hep::DDSegmentation::BitFieldCoder noLayer{"system:5,module:11"};
    REQUIRE_THROWS(ACTSTracking::resolveHitFeature("layer_id", &noLayer));
    REQUIRE_NOTHROW(ACTSTracking::resolveHitFeature("system_id", &noLayer));
  }
}

TEST_CASE("hitFeatureValue") {
  using Catch::Matchers::WithinAbs;

  SECTION("positions and derived quantities") {
    const auto hit = makeHit(3.f, 4.f, 12.f, 7.5f, 0.25f);

    REQUIRE_THAT(valueOf("x", hit), WithinAbs(3.0, 1e-5));
    REQUIRE_THAT(valueOf("y", hit), WithinAbs(4.0, 1e-5));
    REQUIRE_THAT(valueOf("z", hit), WithinAbs(12.0, 1e-5));
    // 3-4-5 triangle, so rho is 5 and the total radius 13
    REQUIRE_THAT(valueOf("r", hit), WithinAbs(5.0, 1e-5));
    REQUIRE_THAT(valueOf("phi", hit), WithinAbs(std::atan2(4.0, 3.0), 1e-5));
    REQUIRE_THAT(valueOf("theta", hit), WithinAbs(std::atan2(5.0, 12.0), 1e-5));
    REQUIRE_THAT(valueOf("t", hit), WithinAbs(7.5, 1e-5));
    REQUIRE_THAT(valueOf("energy", hit), WithinAbs(0.25, 1e-5));
  }

  SECTION("eta grows with z") {
    const auto central = makeHit(10.f, 0.f, 0.f);
    const auto forward = makeHit(10.f, 0.f, 50.f);
    REQUIRE_THAT(valueOf("eta", central), WithinAbs(0.0, 1e-5));
    REQUIRE(valueOf("eta", forward) > valueOf("eta", central));
  }

  SECTION("CellID fields are decoded") {
    dd4hep::DDSegmentation::BitFieldCoder decoder{kEncoding};
    // Build a CellID with known field values rather than hard-coding a number
    std::uint64_t cellID = 0;
    decoder.set(cellID, "system", 3);
    decoder.set(cellID, "layer", 7);
    decoder.set(cellID, "module", 42);
    const auto hit = makeHit(1.f, 1.f, 1.f, 0.f, 0.f, cellID);

    REQUIRE_THAT(valueOf("system_id", hit, &decoder), WithinAbs(3.0, 1e-5));
    REQUIRE_THAT(valueOf("volume_id", hit, &decoder), WithinAbs(3.0, 1e-5));
    REQUIRE_THAT(valueOf("layer_id", hit, &decoder), WithinAbs(7.0, 1e-5));
    REQUIRE_THAT(valueOf("module_id", hit, &decoder), WithinAbs(42.0, 1e-5));
  }
}

TEST_CASE("extractHitInformation") {
  edm4hep::TrackerHitPlaneCollection hits{};
  hits.push_back(makeHit(3.f, 4.f, 1.f, 10.f));
  hits.push_back(makeHit(6.f, 8.f, 2.f, 20.f));

  const auto features = ACTSTracking::resolveHitFeatures({"r", "z", "t"}, nullptr);
  const auto flat     = ACTSTracking::extractHitInformation(hits, features, nullptr);

  // Row-major (nHits x nFeatures)
  REQUIRE(flat.size() == 2 * 3);
  REQUIRE_THAT(flat[0], Catch::Matchers::WithinAbs(5.0, 1e-5));   // hit 0, r
  REQUIRE_THAT(flat[1], Catch::Matchers::WithinAbs(1.0, 1e-5));   // hit 0, z
  REQUIRE_THAT(flat[2], Catch::Matchers::WithinAbs(10.0, 1e-5));  // hit 0, t
  REQUIRE_THAT(flat[3], Catch::Matchers::WithinAbs(10.0, 1e-5));  // hit 1, r
  REQUIRE_THAT(flat[4], Catch::Matchers::WithinAbs(2.0, 1e-5));   // hit 1, z
  REQUIRE_THAT(flat[5], Catch::Matchers::WithinAbs(20.0, 1e-5));  // hit 1, t

  SECTION("no features gives an empty buffer") {
    REQUIRE(ACTSTracking::extractHitInformation(hits, {}, nullptr).empty());
  }
}
