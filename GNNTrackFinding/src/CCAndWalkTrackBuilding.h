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

#include <Acts/Utilities/Logger.hpp>
#if __has_include("ActsPlugins/Gnn/Stages.hpp")
#include <ActsPlugins/Gnn/Stages.hpp>
#include <ActsPlugins/Gnn/Tensor.hpp>
#else
#include <Acts/Plugins/Gnn/Stages.hpp>
#include <Acts/Plugins/Gnn/Tensor.hpp>
namespace ActsPlugins {
  using Device           = Acts::Device;
  using ExecutionContext = Acts::ExecutionContext;
  using PipelineTensors  = Acts::PipelineTensors;
}  // namespace ActsPlugins
#endif

#include <cstddef>
#include <memory>
#include <vector>

/// Connected-components-and-walk track building.
///
/// Acts' BoostTrackBuilding emits every weakly connected component of the
/// classified graph as one track candidate, so two tracks that share a single
/// surviving edge come out as one candidate, and the edge scores are not used
/// at all. This is the algorithm the ExaTrkX / GNN4ITk pipeline uses instead:
///
///  1. the connected components of the graph are found (the score cut that
///     precedes this is the edge classifier's own, see EdgeClassifierCut);
///  2. a component in which every hit has at most one incoming and one outgoing
///     edge is already a path and is accepted as it is;
///  3. any other component is resolved by walking it: starting from each
///     unused node the graph is followed along its edges, and the longest path
///     that can be reached is taken as a candidate, its hits marked used, and
///     the search restarted from the next unused node.
///
/// The walk is steered by two thresholds. Every neighbour whose edge scores
/// above Config::addScore is followed (the walk branches and the longest of the
/// branches wins); if no neighbour reaches that, only the single best one is
/// followed, and only if it scores above Config::minScore.
///
/// The graph is directed by ordering the two hits of every edge by their
/// distance from the interaction point (r^2 + z^2, the same metric the graph
/// construction orders its edges by), which is what makes
/// "incoming"/"outgoing" and "walking outwards" meaningful. Ties are broken by
/// node index, so the ordering is strict and the directed graph is acyclic by
/// construction.
class CCAndWalkTrackBuilding final : public ActsPlugins::TrackBuildingBase {
public:
  struct Config {
    /// Columns of the radius and of z in the node feature tensor. The edges are
    /// directed by the distance from the interaction point (r^2 + z^2) these
    /// two form, see the class documentation.
    int rFeatureIndex{0};
    int zFeatureIndex{0};
    /// A neighbour scoring above this is always followed, and the walk branches
    /// if several do ("edge addition" in the paper).
    float addScore{0.6f};
    /// If no neighbour reaches addScore, the best one is followed if it scores
    /// above this, otherwise the walk stops.
    float minScore{0.1f};
    /// Candidates shorter than this are dropped instead of being emitted.
    std::size_t minCandidateSize{3};
    /// Upper bound on the nodes visited while searching for the longest path
    /// from one starting node. The search is exponential in the amount of
    /// branching, so a pathological component would otherwise stall the job;
    /// hitting the bound returns the longest path found so far.
    std::size_t maxWalkSteps{10000};
  };

  CCAndWalkTrackBuilding(const Config& cfg, std::unique_ptr<const Acts::Logger> logger);

  std::vector<std::vector<int>> operator()(ActsPlugins::PipelineTensors tensors, std::vector<int>& spacePointIDs,
                                           const ActsPlugins::ExecutionContext& execContext = {}) override;

  const Config& config() const { return m_cfg; }

private:
  /// One outgoing edge of a node, pointing at the hit further out
  struct OutEdge {
    int   target{};
    float score{};
  };

  /// The longest path reachable from @p start, following the rules described in
  /// the class documentation. Returns the node indices in walking order.
  std::vector<int> longestPathFrom(int start, const std::vector<std::vector<OutEdge>>& outEdges,
                                   const std::vector<bool>& used) const;

  Config                              m_cfg;
  const auto&                         logger() const { return *m_logger; }
  std::unique_ptr<const Acts::Logger> m_logger{nullptr};
};
