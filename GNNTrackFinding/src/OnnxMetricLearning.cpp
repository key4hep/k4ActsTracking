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
#include "OnnxMetricLearning.h"
#include "ONNXInferenceModel.h"

#if __has_include("ActsPlugins/Gnn/detail/TensorVectorConversion.hpp")
#include <ActsPlugins/Gnn/detail/TensorVectorConversion.hpp>
#include <ActsPlugins/Gnn/detail/buildEdges.hpp>
#else
#include <Acts/Plugins/Gnn/detail/TensorVectorConversion.hpp>
#include <Acts/Plugins/Gnn/detail/buildEdges.hpp>
#endif

#include <onnxruntime_cxx_api.h>

#include <torch/torch.h>

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <fmt/ranges.h>

#include <cassert>
#include <memory>
#include <span>
#include <vector>

namespace {
  /// Conver the Acts log level into an ONNX one
  constexpr OrtLoggingLevel getOnnxLogLevel(Acts::Logging::Level lvl) {
    switch (lvl) {
      case Acts::Logging::VERBOSE:
        return ORT_LOGGING_LEVEL_VERBOSE;
      case Acts::Logging::DEBUG:
        return ORT_LOGGING_LEVEL_INFO;
      case Acts::Logging::INFO:
      case Acts::Logging::WARNING:
        return ORT_LOGGING_LEVEL_WARNING;
      case Acts::Logging::ERROR:
        return ORT_LOGGING_LEVEL_ERROR;
      case Acts::Logging::FATAL:
        return ORT_LOGGING_LEVEL_FATAL;
      case Acts::Logging::MAX:
        return ORT_LOGGING_LEVEL_WARNING;
    }
    return ORT_LOGGING_LEVEL_WARNING;
  }

  /// Convert ONNX element type to torch scalar type
  constexpr torch::ScalarType toTorchType(ONNXTensorElementDataType elementType) {
    switch (elementType) {
      case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT:
        return torch::kFloat32;
      case ONNX_TENSOR_ELEMENT_DATA_TYPE_DOUBLE:
        return torch::kFloat64;
      case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT32:
        return torch::kInt32;
      case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64:
        return torch::kInt64;
      default:
        throw std::runtime_error("Unsupported ONNX tensor element type");
    }
  }

  /// Convert the Onnx tensor into a torch Tensor.
  /// @note: This doesn't take ownership, it essentially just "re-skins" the data
  /// owned by the Onnx tensor
  torch::Tensor toTorchTensor(const Ort::Value& onnxTensor) {
    auto tensorInfo  = onnxTensor.GetTensorTypeAndShapeInfo();
    auto shape       = tensorInfo.GetShape();
    auto elementType = tensorInfo.GetElementType();

    const void* data      = onnxTensor.GetTensorData<void>();
    const auto  torchType = toTorchType(elementType);

    // Create torch tensor from existing data without copying
    auto torchTensor = torch::from_blob(const_cast<void*>(data), shape, torchType);
    return torchTensor;
  }

}  // namespace

OnnxMetricLearning::OnnxMetricLearning(const Config& cfg, std::unique_ptr<const Acts::Logger> lggr)
    : m_model("MetricLearning", getOnnxLogLevel(lggr->level()), cfg.device.isCuda(), cfg.device.index),
      m_config(cfg),
      m_logger(std::move(lggr)) {
  ACTS_INFO(fmt::format("Loading model from {}", config().modelPath));
  m_model.loadModel(config().modelPath);
}

ActsPlugins::PipelineTensors OnnxMetricLearning::operator()(std::vector<float>& inputValues, std::size_t numNodes,
                                                            const std::vector<uint64_t>&,
                                                            const ActsPlugins::ExecutionContext& execContext) {
  assert(inputValues.size() % numNodes == 0);
  std::vector inputShape = {static_cast<int64_t>(numNodes), static_cast<int64_t>(inputValues.size() / numNodes)};
  ACTS_DEBUG(fmt::format("Embedding input tensor shape: {}", inputShape));
  ACTS_DEBUG(fmt::format("First input space point: {}", std::span(inputValues.data(), inputShape[1])));

  const auto outputs = m_model.runInference(inputValues, inputShape);
  // The ONNX session returns its outputs in host memory. Move the embedding to
  // the pipeline's target device so that the edge building below (buildEdges
  // dispatches FRNN/CUDA vs KD-Tree/CPU based on the tensor's device) runs on
  // the same device as the rest of the pipeline.
  const auto torchDevice =
      execContext.device.isCuda() ? torch::Device(torch::kCUDA, execContext.device.index) : torch::Device(torch::kCPU);
  auto embeddedPoints = toTorchTensor(outputs[0]).to(torchDevice);
  assert(embeddedPoints.size(0) == inputShape[0]);  // Do not change the number of points
  assert(embeddedPoints.size(1) == config().embeddingDim);
  ACTS_DEBUG(fmt::format("Embedding output tensor shape: [{}, {}]", embeddedPoints.size(0), embeddedPoints.size(1)));
  ACTS_VERBOSE(fmt::format("Embedding space of first SP: [{}]", fmt::streamed(embeddedPoints.slice(0, 0, 1))));

  ACTS_DEBUG("Starting to build edges");
  auto edgeList =
      ActsPlugins::detail::buildEdges(embeddedPoints, m_config.rVal, m_config.knnVal, m_config.shuffleDirections);
  ACTS_DEBUG("Finished building edges");

  ACTS_VERBOSE(fmt::format("Shape of built edges: ({}, {})", edgeList.size(0), edgeList.size(1)));
  ACTS_VERBOSE(fmt::format("Slice of edgeList: {}", fmt::streamed(edgeList.slice(1, 0, 5))));

  return {ActsPlugins::detail::torchToActsTensor<float>(
              ActsPlugins::detail::vectorToTensor2D(inputValues, inputShape[1]), execContext),
          ActsPlugins::detail::torchToActsTensor<int64_t>(edgeList, execContext), std::nullopt, std::nullopt};
}
