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
#include "EdgePrintingHook.h"

#include "HostTensorView.h"

#include <fmt/format.h>
#include <fmt/ranges.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <utility>

EdgePrintingHook::EdgePrintingHook(Config cfg, const Acts::Logger& logger) : m_cfg(std::move(cfg)), m_logger(logger) {}

void EdgePrintingHook::operator()(const ActsPlugins::PipelineTensors& tensors,
                                  const ActsPlugins::ExecutionContext& execContext) const {
  const std::size_t stage = m_numCalls++;

  // Nothing is copied or gathered unless the output actually goes anywhere
  if (!logger().doPrint(Acts::Logging::DEBUG)) {
    return;
  }

  const std::string stageName =
      stage < m_cfg.stageNames.size() ? m_cfg.stageNames[stage] : fmt::format("stage {}", stage);
  const std::size_t numEdges = tensors.edgeIndex.shape()[1];
  const std::size_t numShown = m_cfg.printAll ? numEdges : std::min(m_cfg.numEdgesShown, numEdges);

  // Everything is printed from the host, so pull the tensors over if they are
  // not there already.
  const gnntracking::HostTensorView<std::int64_t> hostEdges{tensors.edgeIndex, execContext};
  const std::int64_t* edgeData = hostEdges.data();

  // The scores only exist once a classifier has run, the edge features only if
  // the pipeline computes them at all (three-input classifiers).
  std::optional<gnntracking::HostTensorView<float>> hostScores{};
  const float* scoreData = nullptr;
  if (tensors.edgeScores.has_value()) {
    scoreData = hostScores.emplace(*tensors.edgeScores, execContext).data();
  }

  std::optional<gnntracking::HostTensorView<float>> hostFeatures{};
  const float* featureData = nullptr;
  std::size_t numEdgeFeatures = 0;
  if (tensors.edgeFeatures.has_value()) {
    featureData = hostFeatures.emplace(*tensors.edgeFeatures, execContext).data();
    numEdgeFeatures = tensors.edgeFeatures->shape()[1];
  }

  ACTS_DEBUG(fmt::format("After {}: {} of {} edges{}:", stageName, numShown, numEdges,
                         featureData != nullptr ? " (edge features dr, dphi, dz, deta, phislope, rphislope)" : ""));
  for (std::size_t e = 0; e < numShown; ++e) {
    // The edge index is a (2 x numEdges) row-major tensor, so the source of
    // edge e is at e and its target at numEdges + e.
    std::string line = fmt::format("  edge {} ({} -> {})", e, edgeData[e], edgeData[numEdges + e]);
    if (scoreData != nullptr) {
      line += fmt::format(": score {}", scoreData[e]);
    }
    if (featureData != nullptr) {
      line += fmt::format("{} features {}", scoreData != nullptr ? "," : ":",
                          fmt::join(std::span(featureData + e * numEdgeFeatures, numEdgeFeatures), ", "));
    }
    ACTS_DEBUG(line);
  }
}
