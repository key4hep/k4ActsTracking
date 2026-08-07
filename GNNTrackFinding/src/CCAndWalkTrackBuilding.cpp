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
#include "CCAndWalkTrackBuilding.h"

#include <fmt/format.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

using ActsPlugins::PipelineTensors;
using ActsPlugins::Tensor;

namespace {
  /// Union-find over the node indices, to group the graph into its weakly
  /// connected components in one pass over the edges.
  class UnionFind {
  public:
    explicit UnionFind(std::size_t n) : m_parent(n), m_rank(n, 0) { std::iota(m_parent.begin(), m_parent.end(), 0); }

    int find(int x) {
      while (m_parent[x] != x) {
        m_parent[x] = m_parent[m_parent[x]];  // path halving
        x           = m_parent[x];
      }
      return x;
    }

    void unite(int a, int b) {
      a = find(a);
      b = find(b);
      if (a == b) {
        return;
      }
      if (m_rank[a] < m_rank[b]) {
        std::swap(a, b);
      }
      m_parent[b] = a;
      if (m_rank[a] == m_rank[b]) {
        ++m_rank[a];
      }
    }

  private:
    std::vector<int> m_parent;
    std::vector<int> m_rank;
  };
}  // namespace

CCAndWalkTrackBuilding::CCAndWalkTrackBuilding(const Config& cfg, std::unique_ptr<const Acts::Logger> logger)
    : m_cfg(cfg), m_logger(std::move(logger)) {
  if (m_cfg.minScore > m_cfg.addScore) {
    throw std::invalid_argument(
        fmt::format("The walk's minimum score ({}) is above its edge addition score ({}), so the branching threshold "
                    "would be looser than the threshold for following a single edge",
                    m_cfg.minScore, m_cfg.addScore));
  }
}

std::vector<int> CCAndWalkTrackBuilding::longestPathFrom(int start, const std::vector<std::vector<OutEdge>>& outEdges,
                                                         const std::vector<bool>& used) const {
  std::vector<int> best{};
  std::vector<int> path{};
  std::size_t      steps = 0;

  // Iterative depth first search over the branches. The graph is acyclic by
  // construction (see the header), so a path can never revisit a node and no
  // on-path bookkeeping is needed.
  const auto walk = [&](const auto& self, int node) -> void {
    path.push_back(node);
    if (path.size() > best.size()) {
      best = path;
    }

    if (++steps < m_cfg.maxWalkSteps) {
      // Neighbours that scored above addScore are all followed; if none did,
      // the single best one is, provided it clears minScore.
      std::vector<int> next{};
      const OutEdge*   bestEdge = nullptr;
      for (const auto& edge : outEdges[node]) {
        if (used[edge.target]) {
          continue;
        }
        if (edge.score > m_cfg.addScore) {
          next.push_back(edge.target);
        }
        if (bestEdge == nullptr || edge.score > bestEdge->score) {
          bestEdge = &edge;
        }
      }
      if (next.empty() && bestEdge != nullptr && bestEdge->score > m_cfg.minScore) {
        next.push_back(bestEdge->target);
      }

      for (const int target : next) {
        self(self, target);
      }
    }

    path.pop_back();
  };
  walk(walk, start);

  return best;
}

std::vector<std::vector<int>> CCAndWalkTrackBuilding::operator()(PipelineTensors                      tensors,
                                                                 std::vector<int>&                    spacePointIDs,
                                                                 const ActsPlugins::ExecutionContext& execContext) {
  const std::size_t numNodes = spacePointIDs.size();
  const std::size_t numEdges = tensors.edgeIndex.shape()[1];

  if (numEdges == 0) {
    ACTS_WARNING("No edges remained after edge classification");
    return {};
  }
  if (!tensors.edgeScores.has_value()) {
    throw std::runtime_error("CCAndWalkTrackBuilding needs the edge scores, but the pipeline did not produce any");
  }

  // Everything is walked on the host, so pull the tensors over if they are not
  // there already.
  const ActsPlugins::ExecutionContext cpuCtx{ActsPlugins::Device::Cpu(), execContext.stream};
  const auto toHost = [&cpuCtx](const auto& tensor) -> std::optional<std::decay_t<decltype(tensor)>> {
    if (tensor.device().isCpu()) {
      return std::nullopt;
    }
    return tensor.clone(cpuCtx);
  };
  const auto hostEdgeIndex    = toHost(tensors.edgeIndex);
  const auto hostScores       = toHost(*tensors.edgeScores);
  const auto hostNodeFeatures = toHost(tensors.nodeFeatures);

  const std::int64_t* edgeData  = hostEdgeIndex ? hostEdgeIndex->data() : tensors.edgeIndex.data();
  const float*        scoreData = hostScores ? hostScores->data() : tensors.edgeScores->data();
  const float*        nodeData  = hostNodeFeatures ? hostNodeFeatures->data() : tensors.nodeFeatures.data();

  const std::size_t numNodeFeatures = tensors.nodeFeatures.shape()[1];
  if (m_cfg.rFeatureIndex < 0 || static_cast<std::size_t>(m_cfg.rFeatureIndex) >= numNodeFeatures) {
    throw std::runtime_error(fmt::format("Radius feature index {} is out of range for {} node features",
                                         m_cfg.rFeatureIndex, numNodeFeatures));
  }
  const auto radiusOf = [&](int node) {
    return nodeData[static_cast<std::size_t>(node) * numNodeFeatures + static_cast<std::size_t>(m_cfg.rFeatureIndex)];
  };

  // Direct every edge from the hit at the smaller radius to the one at the
  // larger. (radius, index) is a strict total order, so this cannot produce a
  // cycle and sorting the nodes by it gives a topological order for free.
  const auto pointsOutward = [&](int a, int b) {
    const float ra = radiusOf(a);
    const float rb = radiusOf(b);
    return ra != rb ? ra < rb : a < b;
  };

  std::vector<std::pair<std::pair<int, int>, float>> directed{};
  directed.reserve(numEdges);
  std::size_t skipped = 0;
  for (std::size_t i = 0; i < numEdges; ++i) {
    const auto a = static_cast<int>(edgeData[i]);
    const auto b = static_cast<int>(edgeData[numEdges + i]);
    // A self loop carries no connectivity, and an index past the space points
    // is not a hit at all - neither should reach this stage, but neither can be
    // walked either.
    if (a == b || a < 0 || b < 0 || static_cast<std::size_t>(a) >= numNodes ||
        static_cast<std::size_t>(b) >= numNodes) {
      ++skipped;
      continue;
    }
    const auto [u, v] = pointsOutward(a, b) ? std::pair{a, b} : std::pair{b, a};
    directed.push_back({{u, v}, scoreData[i]});
  }
  if (skipped > 0) {
    ACTS_DEBUG(fmt::format("Skipped {} edges that were self loops or pointed outside the {} hits", skipped, numNodes));
  }
  if (directed.empty()) {
    ACTS_WARNING("No usable edges remained after edge classification");
    return {};
  }

  // The graph construction can deliver a pair of hits in both directions, which
  // after the orientation above become the same edge twice. Collapse those, so
  // that they do not count twice towards a node's degree.
  std::ranges::sort(directed, [](const auto& lhs, const auto& rhs) {
    return lhs.first != rhs.first ? lhs.first < rhs.first : lhs.second > rhs.second;
  });
  directed.erase(std::ranges::unique(directed, {}, [](const auto& e) { return e.first; }).begin(), directed.end());

  std::vector<std::vector<OutEdge>> outEdges(numNodes);
  std::vector<int>                  inDegree(numNodes, 0);
  UnionFind                         components(numNodes);
  for (const auto& [edge, score] : directed) {
    const auto [u, v] = edge;
    outEdges[u].push_back({v, score});
    ++inDegree[v];
    components.unite(u, v);
  }
  // Follow the better edge first. The node index breaks ties on the score, so
  // that two equally good branches are always explored in the same order and
  // the walk gives the same tracks from run to run - ranges::sort is not stable,
  // so ordering on the score alone would leave that to the implementation.
  for (auto& edges : outEdges) {
    std::ranges::sort(edges, [](const OutEdge& lhs, const OutEdge& rhs) {
      return lhs.score != rhs.score ? lhs.score > rhs.score : lhs.target < rhs.target;
    });
  }

  // Group the nodes by component, each already in topological order because the
  // node indices are visited in increasing radius.
  std::vector<int> byRadius(numNodes);
  std::iota(byRadius.begin(), byRadius.end(), 0);
  std::ranges::sort(byRadius, pointsOutward);

  std::vector<std::vector<int>> componentNodes(numNodes);
  for (const int node : byRadius) {
    componentNodes[components.find(node)].push_back(node);
  }

  std::vector<std::vector<int>> candidates{};
  std::vector<bool>             used(numNodes, false);
  std::size_t                   numSimple = 0;
  std::size_t                   numWalked = 0;

  for (auto& nodes : componentNodes) {
    if (nodes.size() < m_cfg.minCandidateSize) {
      continue;  // includes the isolated hits, which are their own component
    }

    // A component whose hits all have at most one edge in and one out is
    // already a path, and is taken as it is.
    const bool isSimplePath =
        std::ranges::all_of(nodes, [&](int node) { return inDegree[node] <= 1 && outEdges[node].size() <= 1; });
    if (isSimplePath) {
      candidates.emplace_back();
      candidates.back().reserve(nodes.size());
      for (const int node : nodes) {
        candidates.back().push_back(spacePointIDs[node]);
        used[node] = true;
      }
      ++numSimple;
      continue;
    }

    // Anything else is walked: take the longest path from the innermost unused
    // hit, retire its hits, and carry on from the next unused one.
    for (const int start : nodes) {
      if (used[start]) {
        continue;
      }
      const std::vector<int> path = longestPathFrom(start, outEdges, used);
      if (path.size() < m_cfg.minCandidateSize) {
        // Nothing worth keeping starts here. Only the starting hit is retired,
        // so the hits it reached stay available to a later, longer path.
        used[start] = true;
        continue;
      }
      candidates.emplace_back();
      candidates.back().reserve(path.size());
      for (const int node : path) {
        candidates.back().push_back(spacePointIDs[node]);
        used[node] = true;
      }
      ++numWalked;
    }
  }

  ACTS_DEBUG(
      fmt::format("Built {} track candidates from {} hits and {} edges: {} components were already paths, {} "
                  "came out of walking the rest",
                  candidates.size(), numNodes, directed.size(), numSimple, numWalked));

  return candidates;
}
