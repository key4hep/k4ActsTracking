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

/// Logs the edges that survived an edge classifier, with their score and (if
/// the pipeline computes them) their six edge features.
///
/// This is a pass-through stage: it returns the tensors it was given unchanged
/// and only prints. It exists because the score is the one quantity that cannot
/// be logged where the edge features are computed - it is the classifier that
/// produces it, and the classifier is Acts'. Inserted after each classifier, so
/// that a chain of them shows what each one made of the graph.
class ClassifiedEdgePrinting final : public ActsPlugins::EdgeClassificationBase {
public:
  struct Config {
    /// How many edges to print. Ignored if printAll is set.
    std::size_t numEdgesShown{5};
    /// Print every edge instead of the first numEdgesShown ones. For a real
    /// event this is a lot of output.
    bool printAll{false};
  };

  ClassifiedEdgePrinting(const Config& cfg, std::unique_ptr<const Acts::Logger> logger);

  ActsPlugins::PipelineTensors operator()(ActsPlugins::PipelineTensors         tensors,
                                          const ActsPlugins::ExecutionContext& execContext = {}) override;

  const Config& config() const { return m_cfg; }

private:
  Config m_cfg{};

  const auto&                         logger() const { return *m_logger; }
  std::unique_ptr<const Acts::Logger> m_logger{nullptr};
};
