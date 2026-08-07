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
#include "PaddedEdgeRemoval.h"

#include <fmt/format.h>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>

using ActsPlugins::PipelineTensors;
using ActsPlugins::Tensor;

PaddedEdgeRemoval::PaddedEdgeRemoval(std::unique_ptr<const Acts::Logger> logger) : m_logger(std::move(logger)) {}

PipelineTensors PaddedEdgeRemoval::operator()(PipelineTensors                      tensors,
                                              const ActsPlugins::ExecutionContext& execContext) {
  const std::size_t numEdges = tensors.edgeIndex.shape()[1];

  // The edge index may live on the device, so bring it to the host to build the
  // mask. Keeping this on the host is what saves the package from needing a
  // CUDA kernel of its own; the selects below are device aware.
  const std::optional<Tensor<std::int64_t>> hostEdges =
      tensors.edgeIndex.device().isCpu()
          ? std::nullopt
          : std::optional{tensors.edgeIndex.clone({ActsPlugins::Device::Cpu(), execContext.stream})};
  const std::int64_t* edgeData = hostEdges.has_value() ? hostEdges->data() : tensors.edgeIndex.data();

  // The edge index is a (2 x numEdges) row-major tensor, so the source of edge i
  // is at i and its target at numEdges + i.
  auto        maskCpu  = Tensor<bool>::Create({numEdges, 1ul}, ActsPlugins::ExecutionContext{});
  bool*       maskData = maskCpu.data();
  std::size_t numKept  = 0;
  for (std::size_t i = 0; i < numEdges; ++i) {
    maskData[i] = edgeData[i] != edgeData[numEdges + i];
    numKept += static_cast<std::size_t>(maskData[i]);
  }

  if (numKept == numEdges) {
    ACTS_DEBUG("No padding edges left to remove");
    return tensors;
  }

  Tensor<bool> mask = tensors.edgeIndex.device().isCpu() ? std::move(maskCpu) : maskCpu.clone(execContext);

  auto                         newEdgeIndex = selectCols(tensors.edgeIndex, mask, execContext);
  std::optional<Tensor<float>> newEdgeFeatures{};
  if (tensors.edgeFeatures.has_value()) {
    newEdgeFeatures.emplace(selectRows(*tensors.edgeFeatures, mask, execContext));
  }
  std::optional<Tensor<float>> newEdgeScores{};
  if (tensors.edgeScores.has_value()) {
    newEdgeScores.emplace(selectRows(*tensors.edgeScores, mask, execContext));
  }

  ACTS_DEBUG(fmt::format("Removed {} padding edges that survived the classification, {} edges left", numEdges - numKept,
                         numKept));

  // Same contract as the Acts edge classifiers: an empty graph is reported this
  // way and the pipeline turns it into zero track candidates.
  if (newEdgeIndex.shape()[1] == 0) {
    throw ActsPlugins::NoEdgesError{};
  }

  return {std::move(tensors.nodeFeatures), std::move(newEdgeIndex), std::move(newEdgeFeatures),
          std::move(newEdgeScores)};
}
