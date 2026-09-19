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

#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

/// The one convention by which an edge of the GNN graph gets a direction: it
/// points from the hit closer to the interaction point to the one further out.
namespace gnntracking {

/// Number of node features the direction is computed from: r and z, in that
/// order, see nodeDistancesSq()
inline constexpr std::size_t kNumRadiusFeatures = 2;

/// Positions of r and z in a radius feature index vector
enum RadiusFeature : std::size_t { eR = 0, eZ };

/// The metric: the squared distance of a hit from the interaction point.
/// Squaring keeps the ordering and the value is only ever compared, so the
/// square root would be wasted.
[[nodiscard]] inline float distanceSq(float r, float z) { return r * r + z * z; }

/// Throws unless @p radiusFeatureIndices are the indices of the r and of the z
/// node feature, in that order, in a node feature row of @p numFeatures values
inline void checkRadiusFeatureIndices(const std::vector<int>& radiusFeatureIndices, std::size_t numFeatures) {
  if (radiusFeatureIndices.size() != kNumRadiusFeatures) {
    throw std::invalid_argument("Directing the edges needs exactly " + std::to_string(kNumRadiusFeatures) +
                                " node features (r, z), but " + std::to_string(radiusFeatureIndices.size()) +
                                " are configured");
  }
  for (const int index : radiusFeatureIndices) {
    if (index < 0 || static_cast<std::size_t>(index) >= numFeatures) {
      throw std::invalid_argument("Edge direction feature index " + std::to_string(index) + " is out of range for " +
                                  std::to_string(numFeatures) + " node features");
    }
  }
}

/// distanceSq() of the first @p numNodes rows of the flat, row-major
/// (numRows x @p numFeatures) node feature buffer @p nodeFeatures, taking r and
/// z from the columns @p radiusFeatureIndices names. Rows past @p numNodes (the
/// zero padding some models are exported with) are not looked at.
///
/// @throws std::invalid_argument if @p radiusFeatureIndices does not name two
///         columns of the buffer
[[nodiscard]] inline std::vector<float> nodeDistancesSq(const float* nodeFeatures, std::size_t numNodes,
                                                        std::size_t numFeatures,
                                                        const std::vector<int>& radiusFeatureIndices) {
  checkRadiusFeatureIndices(radiusFeatureIndices, numFeatures);
  const auto rIndex = static_cast<std::size_t>(radiusFeatureIndices[eR]);
  const auto zIndex = static_cast<std::size_t>(radiusFeatureIndices[eZ]);

  std::vector<float> distancesSq(numNodes);
  for (std::size_t n = 0; n < numNodes; ++n) {
    distancesSq[n] = distanceSq(nodeFeatures[n * numFeatures + rIndex], nodeFeatures[n * numFeatures + zIndex]);
  }
  return distancesSq;
}

/// Whether an edge between the two hits points away from the interaction point
/// when it is directed from @p nodeA to @p nodeB, i.e. whether that is the
/// direction it should be given.
///
/// The node index breaks ties, so that two hits at the same distance are still
/// ordered: (distance, index) is a strict total order, which makes the directed
/// graph acyclic by construction and its edge directions reproducible.
[[nodiscard]] inline bool pointsOutward(float distanceSqA, int nodeA, float distanceSqB, int nodeB) {
  return distanceSqA != distanceSqB ? distanceSqA < distanceSqB : nodeA < nodeB;
}

/// pointsOutward() for two nodes of the same nodeDistancesSq() buffer
[[nodiscard]] inline bool pointsOutward(const std::vector<float>& distancesSq, int nodeA, int nodeB) {
  return pointsOutward(distancesSq[static_cast<std::size_t>(nodeA)], nodeA,
                       distancesSq[static_cast<std::size_t>(nodeB)], nodeB);
}

} // namespace gnntracking
