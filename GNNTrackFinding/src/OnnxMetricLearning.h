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

#include "ONNXInferenceModel.h"

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

#include <torch/torch.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

// Implementing this class close to what Acts does for Torch in a way that would
// make it somewhat straight forward to move it to Acts once it has matured

class OnnxMetricLearning final : public ActsPlugins::GraphConstructionBase {
public:
  /// Number of node features the (optional) edge feature computation reads,
  /// see Config::edgeFeatureIndices
  static constexpr std::size_t kNumEdgeFeatureInputs = 4;
  /// Number of edge features it produces, see Config::edgeFeatureIndices
  static constexpr std::size_t kNumEdgeFeatures = 6;

  struct Config {
    std::string        modelPath{};
    std::vector<int>   selectedFeatures{};  // If empty, use all features
    std::vector<float> featureScales{};     // Must be same size as selectedFeatures
    /// Indices of the r, phi, z and eta node features (in that order) in the
    /// full per-hit feature vector, from which the six edge features (dr, dphi,
    /// dz, deta, phislope, rphislope) are computed for every built edge. Edge
    /// classifier models with three inputs take these as their `edge_attr`
    /// input. Empty (the default) computes no edge features, which is what
    /// two-input models expect.
    std::vector<int> edgeFeatureIndices{};
    /// Scales for edgeFeatureIndices. The edge features are computed from the
    /// scaled node values, so these have to be the scales the edge classifier
    /// was trained with. Must be the same size as edgeFeatureIndices, or empty
    /// for no scaling.
    std::vector<float> edgeFeatureScales{};
    /// If > 0, the model input is padded with all-zero rows up to this many
    /// nodes, for models exported with a fixed-size input. The embedding of the
    /// padding rows is discarded before the edge building. 0 disables it.
    int   fixedInputLength{0};
    float rVal{1.6};                 // Same as TorchMetricLearning
    float knnVal{500.};              // Same as TorchMetricLearning
    bool  shuffleDirections{false};  // Same as TorchMetricLearning

    // Device the embedding model and edge building run on. Defaults to CPU;
    // CUDA requires a CUDA-enabled onnxruntime/torch build.
    ActsPlugins::Device device = ActsPlugins::Device::Cpu();
  };

  OnnxMetricLearning(const Config& cfg, std::unique_ptr<const Acts::Logger> logger);
  ~OnnxMetricLearning() = default;

  ActsPlugins::PipelineTensors operator()(std::vector<float>& inputValues, std::size_t numNodes,
                                          const std::vector<uint64_t>&         moduleIds,
                                          const ActsPlugins::ExecutionContext& execContext = {}) override;

  const Config& config() const { return m_config; }

  /// The embedding dimension the loaded model declares, or -1 if the model
  /// leaves that axis dynamic. Read from the ONNX metadata at construction, so
  /// it does not have to be configured.
  int64_t embeddingDim() const { return m_embeddingDim; }

  /// The fixed number of input nodes the loaded model declares, or -1 if it
  /// takes a variable number. Used to check that the (optionally padded) input
  /// is the length the model expects.
  int64_t inputLength() const { return m_inputLength; }

private:
  /// The six edge features (dr, dphi, dz, deta, phislope, rphislope) of every
  /// edge in @p edgeList, computed from the scaled node values selected by
  /// Config::edgeFeatureIndices. Returns std::nullopt if no edge feature inputs
  /// are configured, i.e. if the pipeline is not supposed to produce any.
  ///
  /// @param inputValues the flat (numNodes x fullNumFeatures) hit feature buffer
  /// @param edgeList the (2 x numEdges) edge index tensor, on the pipeline device
  std::optional<ActsPlugins::Tensor<float>> buildEdgeFeatures(const std::vector<float>& inputValues,
                                                              std::size_t numNodes, std::size_t fullNumFeatures,
                                                              const torch::Tensor&                 edgeList,
                                                              const ActsPlugins::ExecutionContext& execContext) const;

  mlutils::ONNXInferenceModel m_model;

  Config m_config;

  /// Output width of the embedding model as declared in the .onnx file
  /// (-1 if the model does not fix that axis), see embeddingDim()
  int64_t m_embeddingDim{-1};

  /// Fixed input length of the embedding model as declared in the .onnx file
  /// (-1 if the model does not fix that axis), see inputLength()
  int64_t m_inputLength{-1};

  // Common Acts infrastructure setup
  const auto&                         logger() const { return *m_logger; }
  std::unique_ptr<const Acts::Logger> m_logger{nullptr};
};
