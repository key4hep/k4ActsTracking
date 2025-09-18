#pragma once

#include "ONNXInferenceModel.h"

#include <Acts/Plugins/Gnn/Stages.hpp>
#include <Acts/Plugins/Gnn/Tensor.hpp>
#include <Acts/Utilities/Logger.hpp>

#include <cstdint>
#include <memory>
#include <string>

// Implementing this class close to what Acts does for Torch in a way that would
// make it somewhat straight forward to move it to Acts once it has matured

class OnnxMetricLearning final : public Acts::GraphConstructionBase {
public:
  struct Config {
    std::string modelPath{};
    int embeddingDim{4};
    float rVal{1.6};               // Same as TorchMetricLearning
    float knnVal{500.};            // Same as TorchMetricLearning
    bool shuffleDirections{false}; // Same as TorchMetricLearning

    // For edge features
    float phiScale = 3.141592654; // Same as TorchmetricLearning
  };

  OnnxMetricLearning(const Config& cfg, std::unique_ptr<const Acts::Logger> logger);
  ~OnnxMetricLearning() = default;

  Acts::PipelineTensors operator()(std::vector<float>& inputValues, std::size_t numNodes,
                                   const std::vector<uint64_t>& moduleIds,
                                   const Acts::ExecutionContext& execContext = {}) override;

  const Config& config() const { return m_config; }

private:
  mlutils::ONNXInferenceModel m_model;

  Config m_config;

  // Common Acts iunfrastructure setuup
  const auto& logger() const { return *m_logger; }
  std::unique_ptr<const Acts::Logger> m_logger{nullptr};
};
