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

#include "SeedRoad.h"

#include <cmath>
#include <stdexcept>
#include <vector>

using Catch::Matchers::WithinAbs;

namespace {
// Seed along the x axis: the helix is the line y = 0, z = 0
SeedRoad::SeedParams straightSeed() { return {}; }

// Curved seed with R = 100 mm, moving in +x at the perigee and climbing in z
SeedRoad::SeedParams curvedSeed() {
  SeedRoad::SeedParams p;
  p.kappa = 0.01;
  p.tanLambda = 0.5;
  return p;
}
} // namespace

TEST_CASE("Straight seed road keeps only hits inside the cuts", "[SeedRoad]") {
  // Without a cone the cuts stay fixed: |d_perp| < 0.5 mm, |d_n| < 1 mm, 0 <= s <= 2000 mm
  SeedRoad::RoadOptions opt;
  opt.coneAngle = 0.;

  SeedRoad::HitGeometry hits;
  hits.xyz = {
      {100., 0., 0.},  // 0: on the track
      {100., 0.2, 0.}, // 1: inside d_perp
      {100., 0., 0.5}, // 2: inside d_n
      {100., 0.8, 0.}, // 3: outside d_perp
      {-10., 0., 0.},  // 4: behind the perigee
      {2500., 0., 0.}, // 5: further along than maxS
      {100., 0., 1.2}, // 6: outside d_n, but a seed hit
  };

  const auto road = SeedRoad::buildRoadEdges({straightSeed()}, hits, opt, {{0, 6, 1}});

  REQUIRE(road.hit == std::vector<int64_t>{0, 1, 2, 6});
  REQUIRE(road.seed == std::vector<int64_t>{0, 0, 0, 0});
  REQUIRE_THAT(road.s[0], WithinAbs(100., 1e-9));
  REQUIRE_THAT(road.dPerp[0], WithinAbs(0., 1e-9));
  REQUIRE_THAT(road.dPerp[1], WithinAbs(0.2, 1e-9));
  REQUIRE_THAT(road.dN[2], WithinAbs(0.5, 1e-9));
  REQUIRE_THAT(road.dN[3], WithinAbs(1.2, 1e-9));
}

TEST_CASE("Hits on a curved helix have zero residuals and the arc length", "[SeedRoad]") {
  const SeedRoad::SeedParams seed = curvedSeed();
  const double s = 50.;
  const SeedRoad::Vec3 onHelix = SeedRoad::helixPosition(seed, s);

  const SeedRoad::Residuals onTrack = SeedRoad::helixResiduals(seed, onHelix, std::nullopt);
  REQUIRE_THAT(onTrack.s, WithinAbs(s, 1e-6));
  REQUIRE_THAT(onTrack.dPerp, WithinAbs(0., 1e-6));
  REQUIRE_THAT(onTrack.dN, WithinAbs(0., 1e-6));
  REQUIRE(std::isnan(onTrack.dT));

  // Moving the hit 0.3 mm along the transverse normal of the helix gives d_perp = 0.3
  const double ang = seed.phi0 + seed.kappa * s;
  const SeedRoad::Vec3 offset{onHelix[0] - 0.3 * std::sin(ang), onHelix[1] + 0.3 * std::cos(ang), onHelix[2]};
  const SeedRoad::Residuals shifted = SeedRoad::helixResiduals(seed, offset, std::nullopt);
  REQUIRE_THAT(shifted.s, WithinAbs(s, 1e-6));
  REQUIRE_THAT(shifted.dPerp, WithinAbs(0.3, 1e-6));
  REQUIRE_THAT(shifted.dN, WithinAbs(0., 1e-6));
}

TEST_CASE("Time cut removes hits with a large time residual", "[SeedRoad]") {
  const SeedRoad::SeedParams seed = curvedSeed();
  const double s = 50.;
  const double normT = std::sqrt(1. + seed.tanLambda * seed.tanLambda);
  const double expectedTime = s * normT / SeedRoad::kCMmPerNs;

  SeedRoad::HitGeometry hits;
  hits.xyz = {SeedRoad::helixPosition(seed, s), SeedRoad::helixPosition(seed, s)};
  hits.t = {expectedTime, expectedTime + 1.0}; // the second one is 1 ns late, cut at 0.4 ns

  SeedRoad::RoadOptions opt;
  opt.coneAngle = 0.;
  const auto road = SeedRoad::buildRoadEdges({seed}, hits, opt, {{0, 0, 0}});

  REQUIRE(road.hit == std::vector<int64_t>{0});
  REQUIRE_THAT(road.dT[0], WithinAbs(0., 1e-6));
}

TEST_CASE("Hit geometry is found by feature name in any order", "[SeedRoad]") {
  // Row: z = 5, phi = 0, r = 100 in the order z, phi, r
  const std::vector<float> nodes{5.f, 0.f, 100.f};
  const SeedRoad::HitGeometry hits = SeedRoad::hitGeometryFromFeatures(nodes, 1, {"z", "phi", "r"});

  REQUIRE(hits.xyz.size() == 1);
  REQUIRE_THAT(hits.xyz[0][0], WithinAbs(100., 1e-4));
  REQUIRE_THAT(hits.xyz[0][1], WithinAbs(0., 1e-4));
  REQUIRE_THAT(hits.xyz[0][2], WithinAbs(5., 1e-4));
  REQUIRE(hits.t.empty());

  REQUIRE_THROWS_AS(SeedRoad::hitGeometryFromFeatures(nodes, 1, {"z", "phi"}), std::invalid_argument);
}

TEST_CASE("Invalid seed hit inputs are rejected", "[SeedRoad]") {
  SeedRoad::HitGeometry hits;
  hits.xyz = {{100., 0., 0.}};
  const SeedRoad::RoadOptions opt{};

  REQUIRE_THROWS_AS(SeedRoad::buildRoadEdges({straightSeed()}, hits, opt, {}), std::invalid_argument);
  REQUIRE_THROWS_AS(SeedRoad::buildRoadEdges({straightSeed()}, hits, opt, {{0, 0, 7}}), std::out_of_range);
}
