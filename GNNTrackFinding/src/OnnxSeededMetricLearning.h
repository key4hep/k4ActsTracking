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
#include "SeedRoad.h"

#include <Acts/Utilities/Logger.hpp>
#include <ActsPlugins/Gnn/Stages.hpp>
#include <ActsPlugins/Gnn/Tensor.hpp>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

// Wrapper for the object condensation model exported with
// scripts/save_full_model.py (SeedNetworkExport). Alternative to
// OnnxMetricLearning in the inference pipeline.
class OnnxSeededMetricLearning final : public ActsPlugins::SeededGraphConstructionBase {
public:
  /// Number of inputs of the exported seed network, in the order of
  /// SEED_EXPORT_INPUTS: node_features, seed_inputs, seed_hits, edge_seed,
  /// edge_hit, edge_inputs
  static constexpr std::size_t kNumModelInputs = 6;
  /// Number of outputs of the exported seed network, in the order of
  /// SEED_EXPORT_OUTPUTS: hit_embedding, seed_embedding, seed_logit, edge_logit
  static constexpr std::size_t kNumModelOutputs = 4;

  struct Config {
    /// Indices of the node features the network takes, empty takes all of them
    std::vector<int> selectedFeatures{};
    /// Divides the selected node features before the network, one per selected feature
    std::vector<float> featureScales{};
    /// Declare the order of hit features in the inputs here! Needed for road building,
    /// and r, phi, z, (t) have to to be present (in any order)
    std::vector<std::string> hitFeatures{"r", "phi", "z", "t"};
    std::string modelPath{};
    /// Sigmoid of the seed logit above which a seed is kept
    float seedScoreThreshold{0.5};
    float rVal{1.6}; // Radius of the ball edges around the selected seeds
    /// Maximum number of hits in the ball of a seed, the nearest ones within rVal are kept
    std::size_t knn{200};
    /// Indices of the r, phi, z and eta node features (in that order) in the
    /// full per-hit feature vector, from which the six edge features (dr, dphi,
    /// dz, deta, phislope, rphislope) of the edges are computed. Same as
    /// OnnxMetricLearning::Config::edgeFeatureIndices. Empty computes none.
    std::vector<int> edgeFeatureIndices{};
    /// Scales for edgeFeatureIndices, empty for no scaling. Same as
    /// OnnxMetricLearning::Config::edgeFeatureScales.
    std::vector<float> edgeFeatureScales{};
    /// Indices of the (unscaled) r and z node features, in that order, by which
    /// the edges are oriented away from the interaction point. Same as
    /// OnnxMetricLearning::Config::distanceFeatureIndices. Empty leaves the
    /// edges as the seed balls built them.
    std::vector<std::size_t> distanceFeatureIndices{};
    /// Seed features and scales
    SeedRoad::SeedFeatureOptions seedFeatures{};
    /// Road cuts around the seed helices
    SeedRoad::RoadOptions road{};
    /// Road residuals of the hit-to-seed edges fed to the network
    SeedRoad::SeedHitEdgeOptions seedHitEdges{};
    // Device the network and edge building run on. Defaults to CPU;
    // CUDA requires a CUDA-enabled onnxruntime/torch build.
    ActsPlugins::Device device = ActsPlugins::Device::Cpu();
  };

  OnnxSeededMetricLearning(const Config& cfg, std::unique_ptr<const Acts::Logger> logger);
  ~OnnxSeededMetricLearning() = default;

  /// Seeds are the CKF seeds of the stage before the pipeline. Seed hit indices
  /// are positions in inputValues (node rows), not space point IDs.
  /// Returns the same PipelineTensors as OnnxMetricLearning: the node features,
  /// the hit-to-hit edge index and features, and no edge scores yet.
  ActsPlugins::PipelineTensors operator()(std::vector<float>& inputValues, std::size_t numNodes,
                                          const std::vector<ActsPlugins::Seed>& seeds,
                                          const std::vector<uint64_t>& moduleIds,
                                          const ActsPlugins::ExecutionContext& execContext = {}) override;

  const Config& config() const { return m_config; }

  /// The fixed number of input nodes the loaded model declares, or -1 if it
  /// takes a variable number. Used to check that the input is the length the model expects.
  int64_t inputLength() const { return m_inputLength; }

private:
  mlutils::ONNXInferenceModel m_model;

  Config m_config;
  int64_t m_inputLength{-1};

  // Common Acts infrastructure setup
  const auto& logger() const { return *m_logger; }
  std::unique_ptr<const Acts::Logger> m_logger{nullptr};
};
