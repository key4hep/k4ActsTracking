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
#include "catch2/matchers/catch_matchers_vector.hpp"

#include "EdgeDirection.h"

#include <stdexcept>
#include <vector>

// The (numNodes x 4) node feature buffer the tests below read r and z from,
// with r in column 1 and z in column 3 and two filler columns in between, so
// that a wrong stride or a swapped index cannot pass unnoticed.
namespace {
const std::vector<int> kIndices{1, 3};
const std::vector<float> kNodes{
    // filler   r      filler   z
    -1.f, 30.f, -1.f, 40.f, // node 0: distance 50
    -1.f, 0.f,  -1.f, 10.f, // node 1: distance 10
    -1.f, 30.f, -1.f, -40.f // node 2: distance 50, same as node 0
};
constexpr std::size_t kNumNodes = 3;
constexpr std::size_t kNumFeatures = 4;
} // namespace

TEST_CASE("nodeDistancesSq") {
  SECTION("r and z are taken from the configured columns") {
    REQUIRE_THAT(gnntracking::nodeDistancesSq(kNodes.data(), kNumNodes, kNumFeatures, kIndices),
                 Catch::Matchers::Equals(std::vector<float>{2500.f, 100.f, 2500.f}));
  }

  SECTION("padding rows past the real nodes are not looked at") {
    REQUIRE_THAT(gnntracking::nodeDistancesSq(kNodes.data(), kNumNodes - 1, kNumFeatures, kIndices),
                 Catch::Matchers::Equals(std::vector<float>{2500.f, 100.f}));
  }

  SECTION("no nodes is not an error") {
    REQUIRE(gnntracking::nodeDistancesSq(kNodes.data(), 0, kNumFeatures, kIndices).empty());
  }

  SECTION("the feature indices have to name two columns of the buffer") {
    // Built at run time, so that the compiler cannot see the short vector being
    // indexed on the (unreachable) path past the check and warn about it
    std::vector<int> indices{kIndices};
    indices.pop_back();
    REQUIRE_THROWS_AS(gnntracking::nodeDistancesSq(kNodes.data(), kNumNodes, kNumFeatures, indices),
                      std::invalid_argument);

    indices = kIndices;
    indices.push_back(0);
    REQUIRE_THROWS_AS(gnntracking::nodeDistancesSq(kNodes.data(), kNumNodes, kNumFeatures, indices),
                      std::invalid_argument);

    REQUIRE_THROWS_AS(
        gnntracking::nodeDistancesSq(kNodes.data(), kNumNodes, kNumFeatures, {1, static_cast<int>(kNumFeatures)}),
        std::invalid_argument);
    REQUIRE_THROWS_AS(gnntracking::nodeDistancesSq(kNodes.data(), kNumNodes, kNumFeatures, {-1, 3}),
                      std::invalid_argument);
  }
}

TEST_CASE("pointsOutward") {
  const auto distancesSq = gnntracking::nodeDistancesSq(kNodes.data(), kNumNodes, kNumFeatures, kIndices);

  SECTION("an edge points outward when it ends further from the interaction point") {
    REQUIRE(gnntracking::pointsOutward(distancesSq, 1, 0));
    REQUIRE_FALSE(gnntracking::pointsOutward(distancesSq, 0, 1));
  }

  SECTION("the node index breaks a tie in the distance") {
    // Node 0 and node 2 sit at the same distance, on opposite sides of z = 0
    REQUIRE(gnntracking::pointsOutward(distancesSq, 0, 2));
    REQUIRE_FALSE(gnntracking::pointsOutward(distancesSq, 2, 0));
  }

  SECTION("it is a strict order, so a node does not point outward of itself") {
    for (int node = 0; node < static_cast<int>(kNumNodes); ++node) {
      REQUIRE_FALSE(gnntracking::pointsOutward(distancesSq, node, node));
    }
  }

  SECTION("exactly one direction of every pair of distinct nodes points outward") {
    for (int a = 0; a < static_cast<int>(kNumNodes); ++a) {
      for (int b = a + 1; b < static_cast<int>(kNumNodes); ++b) {
        REQUIRE(gnntracking::pointsOutward(distancesSq, a, b) != gnntracking::pointsOutward(distancesSq, b, a));
      }
    }
  }
}
