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

#include <ActsPlugins/Gnn/Stages.hpp>
#include <ActsPlugins/Gnn/Tensor.hpp>
#include <ActsPlugins/Gnn/detail/TensorVectorConversion.hpp>

#include <fmt/format.h>
#include <torch/torch.h>

#include <algorithm>
#include <cstddef>
#include <numbers>
#include <stdexcept>
#include <vector>

#ifdef ACTS_GNN_WITH_CUDA
#include <cuda_runtime.h>
#endif

// Pieces shared by the graph constructions that output PipelineTensors, so
// that every one of them builds node and edge features the same way.
namespace gnntracking {

/// Number of node features the edge feature computation reads: r, phi, z, eta
inline constexpr std::size_t kNumEdgeFeatureInputs = 4;
/// Number of edge features it produces: dr, dphi, dz, deta, phislope, rphislope
inline constexpr std::size_t kNumEdgeFeatures = 6;
/// The scale the edge feature computation takes phi to be divided by: the
/// dphi wrap-around un-scales the difference by it, wraps it into [-pi, pi]
/// and scales it back. This is the convention of the ACORN training and of
/// Acts' makeEdgeFeatures(), so the edge feature scales have to scale phi by
/// exactly this for the features to be what the classifier was trained on.
inline constexpr float kEdgeFeaturePhiScale = std::numbers::pi_v<float>;

/// The six edge features of the edges, row-major numEdges x kNumEdgeFeatures.
/// The r, phi, z and eta are the node features at @p indices of the full
/// per-hit vectors in @p inputValues (@p numFeatures values each), divided by
/// @p scales if any. @p edgeList is the 2 x numEdges int64 edge index.
/// The features are the signed differences along each edge, so the edges have
/// to be oriented already.
inline torch::Tensor buildEdgeFeatures(const std::vector<float>& inputValues, std::size_t numNodes,
                                       std::size_t numFeatures, const torch::Tensor& edgeList,
                                       const std::vector<int>& indices, const std::vector<float>& scales) {
  // The six features are the ones Acts' makeEdgeFeatures() (ModuleMapUtils.cuh)
  // produces, which is the only place Acts fills them - but that one is CUDA
  // only, so the CPU pipeline has to compute them itself. Note that the edge
  // classifier scales its node input but passes the edge input through as it
  // is, so these are computed from the already scaled node values.
  enum EdgeFeatureInput { eR = 0, ePhi, eZ, eEta };
  constexpr float pi = std::numbers::pi_v<float>;

  // (numNodes x 4) buffer of the scaled r, phi, z and eta of every node
  std::vector<float> nodeValues(numNodes * kNumEdgeFeatureInputs);
  for (std::size_t n = 0; n < numNodes; ++n) {
    for (std::size_t f = 0; f < kNumEdgeFeatureInputs; ++f) {
      const float value = inputValues[n * numFeatures + static_cast<std::size_t>(indices[f])];
      nodeValues[n * kNumEdgeFeatureInputs + f] = scales.empty() ? value : value / scales[f];
    }
  }

  // Gathering the node values per edge with torch ops keeps the computation on
  // whichever device the edge building ran on.
  const auto nodeTensor =
      ActsPlugins::detail::vectorToTensor2D(nodeValues, kNumEdgeFeatureInputs).to(edgeList.device());
  const auto srcValues = nodeTensor.index_select(0, edgeList.select(0, 0).contiguous());
  const auto tgtValues = nodeTensor.index_select(0, edgeList.select(0, 1).contiguous());

  const auto dr = tgtValues.select(1, eR) - srcValues.select(1, eR);
  const auto dz = tgtValues.select(1, eZ) - srcValues.select(1, eZ);
  const auto deta = tgtValues.select(1, eEta) - srcValues.select(1, eEta);

  // phi is scaled by kEdgeFeaturePhiScale, so the difference is unscaled to
  // wrap it back into [-pi, pi] and then scaled again. A single wrap is enough
  // since the unscaled difference cannot leave [-2pi, 2pi].
  auto dphi = kEdgeFeaturePhiScale * (tgtValues.select(1, ePhi) - srcValues.select(1, ePhi));
  dphi = torch::where(dphi > pi, dphi - 2.f * pi, dphi);
  dphi = torch::where(dphi < -pi, dphi + 2.f * pi, dphi);
  dphi = dphi / kEdgeFeaturePhiScale;

  // Doublets on the same radius have no defined slope and get a flat zero. The
  // substitute denominator only keeps the discarded branch from producing infs.
  const auto hasDr = dr != 0.f;
  const auto phislope = torch::where(
      hasDr, torch::clamp(dphi / torch::where(hasDr, dr, torch::ones_like(dr)), -100.f, 100.f), torch::zeros_like(dr));
  const auto rphislope = 0.5f * (tgtValues.select(1, eR) + srcValues.select(1, eR)) * phislope;

  return torch::stack({dr, dphi, dz, deta, phislope, rphislope}, 1).contiguous();
}

/// Copies the node features (numFeatures values per node) to a tensor on the
/// device of @p execContext, see OnnxMetricLearning for why this is not built
/// from a torch tensor.
inline ActsPlugins::Tensor<float> makeNodeFeatureTensor(const std::vector<float>& nodeValues, std::size_t numFeatures,
                                                        const ActsPlugins::ExecutionContext& execContext) {
  const std::size_t numNodes = nodeValues.size() / numFeatures;
  auto nodeTensor = ActsPlugins::Tensor<float>::Create({numNodes, numFeatures}, execContext);
  if (!execContext.device.isCuda()) {
    std::copy(nodeValues.begin(), nodeValues.end(), nodeTensor.data());
  } else {
#ifdef ACTS_GNN_WITH_CUDA
    const auto nbytes = nodeValues.size() * sizeof(float);
    const auto status =
        execContext.stream.has_value()
            ? cudaMemcpyAsync(nodeTensor.data(), nodeValues.data(), nbytes, cudaMemcpyHostToDevice, *execContext.stream)
            : cudaMemcpy(nodeTensor.data(), nodeValues.data(), nbytes, cudaMemcpyHostToDevice);
    if (status != cudaSuccess) {
      throw std::runtime_error(fmt::format("Failed to copy node features to CUDA: {}", cudaGetErrorString(status)));
    }
#else
    throw std::runtime_error("cannot run on CUDA, k4ActsTracking/ACTS was built without CUDA support");
#endif
  }
  return nodeTensor;
}

} // namespace gnntracking
