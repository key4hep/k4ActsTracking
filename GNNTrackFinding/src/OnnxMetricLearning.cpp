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

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
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

  // A model exported with a fixed-size input pins its node axis instead of
  // leaving it dynamic. Read it so that a mismatch can be reported against the
  // configured padding rather than as a bare onnxruntime shape error.
  const auto& modelInputShape = m_model.inputShape(0);
  if (modelInputShape.size() >= 2) {
    m_inputLength = modelInputShape.front();
  }
  if (m_inputLength > 0) {
    ACTS_INFO(fmt::format("Model expects a fixed input length of {} nodes", m_inputLength));
  } else {
    ACTS_DEBUG(fmt::format("Model '{}' takes a variable number of input nodes (input shape [{}])", config().modelPath,
                           fmt::join(modelInputShape, ", ")));
  }

  if (config().fixedInputLength > 0) {
    ACTS_INFO(fmt::format("Zero-padding the model input up to {} nodes, the padding rows are {} the edge classifiers",
                          config().fixedInputLength, config().keepPadding ? "kept for" : "dropped before"));
  }

  // The edge feature computation is optional, but if it is configured it has to
  // be given exactly the four node features it is defined in terms of.
  if (!config().edgeFeatureIndices.empty()) {
    if (config().edgeFeatureIndices.size() != kNumEdgeFeatureInputs) {
      throw std::invalid_argument(
          fmt::format("The edge feature computation needs exactly {} node features (r, phi, z, eta), but {} are "
                      "configured",
                      kNumEdgeFeatureInputs, config().edgeFeatureIndices.size()));
    }
    if (!config().edgeFeatureScales.empty() && config().edgeFeatureScales.size() != kNumEdgeFeatureInputs) {
      throw std::invalid_argument(fmt::format(
          "edgeFeatureScales has {} entries, but has to have one per edge feature input ({}) or none at all",
          config().edgeFeatureScales.size(), kNumEdgeFeatureInputs));
    }
    ACTS_INFO(fmt::format("Computing {} edge features (dr, dphi, dz, deta, phislope, rphislope) for every built edge",
                          kNumEdgeFeatures));
  } else {
    ACTS_DEBUG("No edge features are computed, the edge classifiers have to be two-input models");
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

  // Optionally the input is extended with all-zero rows up to a fixed length,
  // for models that were exported with a fixed-size input. The padding rows sit
  // after the real nodes, so a node keeps the row index the rest of the pipeline
  // refers to it by, and the embedding of the padding rows is dropped again
  // below.
  const std::size_t fixedInputLength =
      config().fixedInputLength > 0 ? static_cast<std::size_t>(config().fixedInputLength) : 0;
  if (fixedInputLength != 0 && fixedInputLength < numNodes) {
    throw std::runtime_error(fmt::format(
        "Cannot zero-pad the node embedding model input to a fixed length of {} nodes, this segment already has {}. "
        "Increase EmbeddingFixedInputLength, or raise ThetaBins / PhiBins so that fewer hits land in one segment.",
        fixedInputLength, numNodes));
  }
  const std::size_t paddedNumNodes = std::max(fixedInputLength, numNodes);
  const std::vector inputShape     = {static_cast<int64_t>(paddedNumNodes), static_cast<int64_t>(numFeatures)};

  // Fail with a message that names the padding knob rather than letting
  // onnxruntime reject the tensor with a bare shape mismatch.
  if (m_inputLength > 0 && static_cast<int64_t>(paddedNumNodes) != m_inputLength) {
    throw std::runtime_error(fmt::format(
        "Node embedding model expects a fixed input length of {} nodes, but {} would be passed to it "
        "({} real nodes, zero-padding {}). Set EmbeddingFixedInputLength to {}.",
        m_inputLength, paddedNumNodes, numNodes, fixedInputLength == 0 ? "disabled" : "enabled", m_inputLength));
  }

  for (const int idx : selectedFeatures) {
    if (idx < 0 || static_cast<std::size_t>(idx) >= fullNumFeatures) {
      throw std::runtime_error("Selected feature index out of range");
    }
  }
  for (const int idx : config().edgeFeatureIndices) {
    if (idx < 0 || static_cast<std::size_t>(idx) >= fullNumFeatures) {
      throw std::runtime_error("Edge feature input index out of range");
    }
  }
  if (!featureScales.empty() && featureScales.size() != numFeatures) {
    throw std::runtime_error("featureScales size must match the number of input features");
  }

  // Select and scale the model inputs in one pass. This is done into a separate
  // buffer (and not in place) because the pipeline expects to get the full,
  // unscaled node features back from this stage. Only if none of a selection, a
  // scaling and a padding is configured can the inputs be used as they are.
  std::vector<float> preparedValues{};
  if (!selectedFeatures.empty() || !featureScales.empty() || paddedNumNodes != numNodes) {
    // The padding rows are the [numNodes, paddedNumNodes) tail of the buffer and
    // are never written to below, so zero-initialise the whole thing.
    preparedValues.assign(paddedNumNodes * numFeatures, 0.f);
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

  // Drop the embedding of the padding rows again. This is unconditional, and
  // independent of Config::keepPadding below: the padding rows are not hits, and
  // the network maps every all-zero row onto the same arbitrary (bias dependent)
  // point in embedding space, so leaving them in would have the edge building
  // connect all of them to each other and to whatever real node is nearby.
  // Slicing along the node axis of a contiguous tensor is a view, so this costs
  // nothing.
  if (paddedNumNodes != numNodes) {
    embeddedPoints = embeddedPoints.slice(0, 0, static_cast<int64_t>(numNodes));
    ACTS_DEBUG(fmt::format("Dropped {} padding nodes, {} real nodes go into the edge building",
                           paddedNumNodes - numNodes, numNodes));
  }
  ACTS_VERBOSE(fmt::format("Embedding space of first SP: [{}]", fmt::streamed(embeddedPoints.slice(0, 0, 1))));

  ACTS_DEBUG("Starting to build edges");
  auto edgeList =
      ActsPlugins::detail::buildEdges(embeddedPoints, m_config.rVal, m_config.knnVal, m_config.shuffleDirections);
  ACTS_DEBUG("Finished building edges");

  ACTS_VERBOSE(fmt::format("Shape of built edges: ({}, {})", edgeList.size(0), edgeList.size(1)));
  ACTS_VERBOSE(fmt::format("Slice of edgeList: {}", fmt::streamed(edgeList.slice(1, 0, 5))));

  auto edgeFeatures = buildEdgeFeatures(inputValues, numNodes, fullNumFeatures, edgeList);

  // Optionally pad the graph up to a fixed number of edges, for classifier
  // models exported with a fixed-size edge input. The padding edges are self
  // loops on the last (padding) node row: a self loop carries no connectivity,
  // it touches no real node so it cannot disturb their message passing, and it
  // is what PaddedEdgeRemoval recognises them by once the classifiers are done -
  // the edge building never produces one.
  const std::size_t numEdges = static_cast<std::size_t>(edgeList.size(1));
  const std::size_t fixedEdgeLength =
      config().fixedEdgeLength > 0 ? static_cast<std::size_t>(config().fixedEdgeLength) : 0;
  if (fixedEdgeLength != 0 && fixedEdgeLength < numEdges) {
    throw std::runtime_error(fmt::format(
        "Cannot zero-pad the edge classifier input to a fixed length of {} edges, this segment already has {}. "
        "Increase EdgeClassifierFixedInputLength, or lower EdgeBuildingRadius / EdgeBuildingKnn so that fewer edges "
        "are built.",
        fixedEdgeLength, numEdges));
  }
  if (fixedEdgeLength > numEdges) {
    const int64_t numPadEdges = static_cast<int64_t>(fixedEdgeLength - numEdges);
    const int64_t padNode     = static_cast<int64_t>(paddedNumNodes) - 1;
    edgeList                  = torch::cat({edgeList, torch::full({2, numPadEdges}, padNode, edgeList.options())}, 1);
    if (edgeFeatures.has_value()) {
      // All six features of a self loop on an all-zero node are zero anyway, so
      // this is the same thing buildEdgeFeatures() would have produced for them.
      edgeFeatures = torch::cat(
          {*edgeFeatures, torch::zeros({numPadEdges, static_cast<int64_t>(kNumEdgeFeatures)}, edgeFeatures->options())},
          0);
    }
    ACTS_DEBUG(fmt::format("Padded {} real edges with {} self loops on node {}, {} edges go to the edge classifiers",
                           numEdges, numPadEdges, padNode, fixedEdgeLength));
  }

  std::optional<ActsPlugins::Tensor<float>> actsEdgeFeatures{};
  if (edgeFeatures.has_value()) {
    actsEdgeFeatures.emplace(ActsPlugins::detail::torchToActsTensor<float>(edgeFeatures->contiguous(), execContext));
    ACTS_DEBUG(
        fmt::format("Edge feature tensor shape: [{}, {}]", actsEdgeFeatures->shape()[0], actsEdgeFeatures->shape()[1]));
  }

  // The node features handed on to the rest of the pipeline normally cover the
  // real hits only. An edge classifier that was itself exported at a fixed
  // number of nodes needs the padding rows back, so optionally re-pad the
  // full-feature buffer to the length the embedding model was fed. Every real
  // node keeps its row index, so the extra rows are just nodes without edges,
  // and the edge index built above stays valid. Only the edge classifiers look
  // at this node count - the track building sizes its graph from the space
  // point IDs, which stay at the real hits.
  std::vector<float> paddedInputValues{};
  if (config().keepPadding && paddedNumNodes != numNodes) {
    // The rows past the real hits are never written to, so zero-initialise.
    paddedInputValues.assign(paddedNumNodes * fullNumFeatures, 0.f);
    std::copy(inputValues.begin(), inputValues.end(), paddedInputValues.begin());
    ACTS_DEBUG(fmt::format("Kept {} padding nodes, {} rows go on to the edge classifiers", paddedNumNodes - numNodes,
                           paddedNumNodes));
  }
  std::vector<float>& downstreamValues = paddedInputValues.empty() ? inputValues : paddedInputValues;

  return {ActsPlugins::detail::torchToActsTensor<float>(
              // The full-feature node tensor, unscaled: each stage selects and
              // scales the features it needs from it.
              ActsPlugins::detail::vectorToTensor2D(downstreamValues, fullNumFeatures), execContext),
          ActsPlugins::detail::torchToActsTensor<int64_t>(edgeList, execContext), std::move(actsEdgeFeatures),
          std::nullopt};
}

std::optional<torch::Tensor> OnnxMetricLearning::buildEdgeFeatures(const std::vector<float>& inputValues,
                                                                   std::size_t numNodes, std::size_t fullNumFeatures,
                                                                   const torch::Tensor& edgeList) const {
  if (config().edgeFeatureIndices.empty()) {
    return std::nullopt;
  }

  // The six features are the ones Acts' makeEdgeFeatures() (ModuleMapUtils.cuh)
  // produces, which is the only place Acts fills them - but that one is CUDA
  // only, so the CPU pipeline has to compute them itself. Note that the edge
  // classifier scales its node input but passes the edge input through as it
  // is, so these are computed from the already scaled node values.
  enum EdgeFeatureInput { eR = 0, ePhi, eZ, eEta };
  constexpr float pi = static_cast<float>(M_PI);

  const auto& indices = config().edgeFeatureIndices;
  const auto& scales  = config().edgeFeatureScales;

  // (numNodes x 4) buffer of the scaled r, phi, z and eta of every node
  std::vector<float> nodeValues(numNodes * kNumEdgeFeatureInputs);
  for (std::size_t n = 0; n < numNodes; ++n) {
    for (std::size_t f = 0; f < kNumEdgeFeatureInputs; ++f) {
      const float value = inputValues[n * fullNumFeatures + static_cast<std::size_t>(indices[f])];
      nodeValues[n * kNumEdgeFeatureInputs + f] = scales.empty() ? value : value / scales[f];
    }
  }

  // Gathering the node values per edge with torch ops keeps the computation on
  // whichever device the edge building ran on.
  const auto nodeTensor =
      ActsPlugins::detail::vectorToTensor2D(nodeValues, kNumEdgeFeatureInputs).to(edgeList.device());
  const auto srcValues = nodeTensor.index_select(0, edgeList.select(0, 0).contiguous());
  const auto tgtValues = nodeTensor.index_select(0, edgeList.select(0, 1).contiguous());

  const auto dr   = tgtValues.select(1, eR) - srcValues.select(1, eR);
  const auto dz   = tgtValues.select(1, eZ) - srcValues.select(1, eZ);
  const auto deta = tgtValues.select(1, eEta) - srcValues.select(1, eEta);

  // phi is scaled by pi, so the difference is unscaled to wrap it back into
  // [-pi, pi] and then scaled again. A single wrap is enough since the unscaled
  // difference cannot leave [-2pi, 2pi].
  auto dphi = pi * (tgtValues.select(1, ePhi) - srcValues.select(1, ePhi));
  dphi      = torch::where(dphi > pi, dphi - 2.f * pi, dphi);
  dphi      = torch::where(dphi < -pi, dphi + 2.f * pi, dphi);
  dphi      = dphi / pi;

  // Doublets on the same radius have no defined slope and get a flat zero. The
  // substitute denominator only keeps the discarded branch from producing infs.
  const auto hasDr    = dr != 0.f;
  const auto phislope = torch::where(
      hasDr, torch::clamp(dphi / torch::where(hasDr, dr, torch::ones_like(dr)), -100.f, 100.f), torch::zeros_like(dr));
  const auto rphislope = 0.5f * (tgtValues.select(1, eR) + srcValues.select(1, eR)) * phislope;

  // Left as a torch tensor so that the caller can still pad it before it is
  // handed over to the pipeline.
  return torch::stack({dr, dphi, dz, deta, phislope, rphislope}, 1).contiguous();
}
