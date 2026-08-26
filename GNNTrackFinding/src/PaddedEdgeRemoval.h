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

#include <memory>

/// Drops the self loops that OnnxMetricLearning adds to pad the graph up to a
/// fixed number of edges (see its Config::fixedEdgeLength).
///
/// This runs as the last stage of the edge classifier chain, so that the
/// padding edges are gone before the track building. Letting them through is
/// not an option: they point at a padding node, i.e. at a row index beyond the
/// space point IDs, and Boost grows its graph to fit that index while the label
/// vector stays at the number of space points - which overruns it.
///
/// The self loop is what identifies them. The edge building never produces one,
/// so this neither needs to know how many edges were padding nor which of them
/// survived the classifier cuts.
class PaddedEdgeRemoval final : public ActsPlugins::EdgeClassificationBase {
public:
  explicit PaddedEdgeRemoval(std::unique_ptr<const Acts::Logger> logger);

  ActsPlugins::PipelineTensors operator()(ActsPlugins::PipelineTensors         tensors,
                                          const ActsPlugins::ExecutionContext& execContext = {}) override;

private:
  const auto&                         logger() const { return *m_logger; }
  std::unique_ptr<const Acts::Logger> m_logger{nullptr};
};
