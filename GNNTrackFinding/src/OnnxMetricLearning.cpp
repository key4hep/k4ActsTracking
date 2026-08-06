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
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <stdexcept>
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
  if (!m_model.loadModel(config().modelPath)) {
    throw std::runtime_error(fmt::format("Could not load the node embedding ONNX model from '{}'", config().modelPath));
  }

  // Take the embedding dimension from the model itself instead of having it
  // configured. The last axis of the (nNodes x embeddingDim) output carries it,
  // unless the model was exported with that axis dynamic, in which case ONNX
  // reports -1 and we can only report what the model actually returns.
  const auto& outputShape = m_model.outputShape(0);
  if (outputShape.size() >= 2) {
    m_embeddingDim = outputShape.back();
  }
  if (m_embeddingDim > 0) {
    ACTS_INFO(fmt::format("Model declares an embedding dimension of {}", m_embeddingDim));
  } else {
    ACTS_INFO(
        fmt::format("Model '{}' does not declare a fixed embedding dimension (output shape [{}]), taking it from "
                    "the inference output",
                    config().modelPath, fmt::join(outputShape, ", ")));
  }
}

ActsPlugins::PipelineTensors OnnxMetricLearning::operator()(std::vector<float>& inputValues, std::size_t numNodes,
                                                            const std::vector<uint64_t>&,
                                                            const ActsPlugins::ExecutionContext& execContext) {
  assert(inputValues.size() % numNodes == 0);

  const std::size_t         fullNumFeatures  = inputValues.size() / numNodes;
  const std::vector<int>&   selectedFeatures = config().selectedFeatures;
  const std::vector<float>& featureScales    = config().featureScales;

  // The model only sees the selected features (all of them if no selection is
  // configured)
  const std::size_t numFeatures = selectedFeatures.empty() ? fullNumFeatures : selectedFeatures.size();
  const std::vector inputShape  = {static_cast<int64_t>(numNodes), static_cast<int64_t>(numFeatures)};

  for (const int idx : selectedFeatures) {
    if (idx < 0 || static_cast<std::size_t>(idx) >= fullNumFeatures) {
      throw std::runtime_error("Selected feature index out of range");
    }
  }
  if (!featureScales.empty() && featureScales.size() != numFeatures) {
    throw std::runtime_error("featureScales size must match the number of input features");
  }

  // Select and scale the model inputs in one pass. This is done into a separate
  // buffer (and not in place) because the pipeline expects to get the full,
  // unscaled node features back from this stage. Only if neither a selection
  // nor a scaling is configured can the inputs be used as they are.
  std::vector<float> preparedValues{};
  if (!selectedFeatures.empty() || !featureScales.empty()) {
    preparedValues.resize(numNodes * numFeatures);
    for (std::size_t n = 0; n < numNodes; ++n) {
      for (std::size_t f = 0; f < numFeatures; ++f) {
        const std::size_t idx   = selectedFeatures.empty() ? f : static_cast<std::size_t>(selectedFeatures[f]);
        const float       value = inputValues[n * fullNumFeatures + idx];
        preparedValues[n * numFeatures + f] = featureScales.empty() ? value : value / featureScales[f];
      }
    }
  }
  const std::vector<float>& inferenceValues = preparedValues.empty() ? inputValues : preparedValues;

  ACTS_DEBUG(fmt::format("Embedding input tensor shape: {}", inputShape));
  ACTS_DEBUG(fmt::format("First input space point: {}", std::span(inferenceValues.data(), numFeatures)));

  const auto outputs = m_model.runInference(inferenceValues, inputShape);
  // The ONNX session returns its outputs in host memory. Move the embedding to
  // the pipeline's target device so that the edge building below (buildEdges
  // dispatches FRNN/CUDA vs KD-Tree/CPU based on the tensor's device) runs on
  // the same device as the rest of the pipeline.
  const auto torchDevice =
      execContext.device.isCuda() ? torch::Device(torch::kCUDA, execContext.device.index) : torch::Device(torch::kCPU);
  auto embeddedPoints = toTorchTensor(outputs[0]).to(torchDevice);
  assert(embeddedPoints.size(0) == inputShape[0]);  // Do not change the number of points
  // A model that declares its embedding dimension has to stick to it. This is
  // checked (rather than asserted) because a mismatch here means the loaded
  // model is not the one its own metadata describes.
  if (m_embeddingDim > 0 && embeddedPoints.size(1) != m_embeddingDim) {
    throw std::runtime_error(
        fmt::format("Node embedding model returned {} embedding dimensions, but its ONNX metadata declares {}",
                    embeddedPoints.size(1), m_embeddingDim));
  }
  ACTS_DEBUG(fmt::format("Embedding output tensor shape: [{}, {}]", embeddedPoints.size(0), embeddedPoints.size(1)));
  ACTS_VERBOSE(fmt::format("Embedding space of first SP: [{}]", fmt::streamed(embeddedPoints.slice(0, 0, 1))));

  ACTS_DEBUG("Starting to build edges");
  auto edgeList =
      ActsPlugins::detail::buildEdges(embeddedPoints, m_config.rVal, m_config.knnVal, m_config.shuffleDirections);
  ACTS_DEBUG("Finished building edges");

  ACTS_VERBOSE(fmt::format("Shape of built edges: ({}, {})", edgeList.size(0), edgeList.size(1)));
  ACTS_VERBOSE(fmt::format("Slice of edgeList: {}", fmt::streamed(edgeList.slice(1, 0, 5))));

  return {ActsPlugins::detail::torchToActsTensor<float>(
              // Return the original full-feature node tensor to the pipeline
              // (do not reduce the node features returned to the pipeline).
              ActsPlugins::detail::vectorToTensor2D(inputValues, fullNumFeatures), execContext),
          ActsPlugins::detail::torchToActsTensor<int64_t>(edgeList, execContext), std::nullopt, std::nullopt};
}
