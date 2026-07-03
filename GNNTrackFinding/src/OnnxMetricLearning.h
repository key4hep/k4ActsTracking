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

#include <cstdint>
#include <memory>
#include <string>

// Implementing this class close to what Acts does for Torch in a way that would
// make it somewhat straight forward to move it to Acts once it has matured

class OnnxMetricLearning final : public ActsPlugins::GraphConstructionBase {
public:
  struct Config {
    std::string modelPath{};
    int         embeddingDim{4};
    float       rVal{1.6};                 // Same as TorchMetricLearning
    float       knnVal{500.};              // Same as TorchMetricLearning
    bool        shuffleDirections{false};  // Same as TorchMetricLearning

    // For edge features
    float phiScale = 3.141592654;  // Same as TorchmetricLearning

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

private:
  mlutils::ONNXInferenceModel m_model;

  Config m_config;

  // Common Acts iunfrastructure setuup
  const auto&                         logger() const { return *m_logger; }
  std::unique_ptr<const Acts::Logger> m_logger{nullptr};
};
