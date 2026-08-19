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
#include "ClassifiedEdgePrinting.h"

#include <fmt/format.h>
#include <fmt/ranges.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <type_traits>
#include <utility>

using ActsPlugins::PipelineTensors;
using ActsPlugins::Tensor;

ClassifiedEdgePrinting::ClassifiedEdgePrinting(const Config& cfg, std::unique_ptr<const Acts::Logger> logger)
    : m_cfg(cfg), m_logger(std::move(logger)) {}

PipelineTensors ClassifiedEdgePrinting::operator()(PipelineTensors                      tensors,
                                                   const ActsPlugins::ExecutionContext& execContext) {
  // Nothing is copied or gathered unless the output actually goes anywhere
  if (!logger().doPrint(Acts::Logging::DEBUG)) {
    return tensors;
  }

  const std::size_t numEdges = tensors.edgeIndex.shape()[1];
  const std::size_t numShown = config().printAll ? numEdges : std::min(config().numEdgesShown, numEdges);

  // Everything is printed from the host, so pull the tensors over if they are
  // not there already. This is the same idiom the track building uses.
  const ActsPlugins::ExecutionContext cpuCtx{ActsPlugins::Device::Cpu(), execContext.stream};
  const auto toHost = [&cpuCtx](const auto& tensor) -> std::optional<std::decay_t<decltype(tensor)>> {
    if (tensor.device().isCpu()) {
      return std::nullopt;
    }
    return tensor.clone(cpuCtx);
  };
  const std::optional<Tensor<std::int64_t>> hostEdges = toHost(tensors.edgeIndex);
  const std::int64_t*                       edgeData  = hostEdges ? hostEdges->data() : tensors.edgeIndex.data();

  // The scores are there for every classified graph, the edge features only if
  // the pipeline computes them at all (three-input classifiers).
  std::optional<Tensor<float>> hostScores{};
  const float*                 scoreData = nullptr;
  if (tensors.edgeScores.has_value()) {
    hostScores = toHost(*tensors.edgeScores);
    scoreData  = hostScores ? hostScores->data() : tensors.edgeScores->data();
  }

  std::optional<Tensor<float>> hostFeatures{};
  const float*                 featureData     = nullptr;
  std::size_t                  numEdgeFeatures = 0;
  if (tensors.edgeFeatures.has_value()) {
    hostFeatures    = toHost(*tensors.edgeFeatures);
    featureData     = hostFeatures ? hostFeatures->data() : tensors.edgeFeatures->data();
    numEdgeFeatures = tensors.edgeFeatures->shape()[1];
  }

  ACTS_DEBUG(fmt::format("{} of {} classified edges{}:", numShown, numEdges,
                         featureData != nullptr ? " (dr, dphi, dz, deta, phislope, rphislope)" : ""));
  for (std::size_t e = 0; e < numShown; ++e) {
    // The edge index is a (2 x numEdges) row-major tensor, so the source of
    // edge e is at e and its target at numEdges + e.
    std::string line = fmt::format("  edge {} ({} -> {})", e, edgeData[e], edgeData[numEdges + e]);
    if (scoreData != nullptr) {
      line += fmt::format(": score {}", scoreData[e]);
    }
    if (featureData != nullptr) {
      line +=
          fmt::format(", features {}", fmt::join(std::span(featureData + e * numEdgeFeatures, numEdgeFeatures), ", "));
    }
    ACTS_DEBUG(line);
  }

  return tensors;
}
