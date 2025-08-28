#pragma once

#include <onnxruntime_cxx_api.h>

#include <memory>
#include <string>
#include <vector>

namespace mlutils {
class ONNXInferenceModel {
public:
  // Constructor
  explicit ONNXInferenceModel(const std::string& name);

  ONNXInferenceModel() = delete;
  ONNXInferenceModel(const ONNXInferenceModel&) = delete;
  ONNXInferenceModel& operator=(const ONNXInferenceModel&) = delete;
  ONNXInferenceModel(ONNXInferenceModel&&) = default;
  ONNXInferenceModel& operator=(ONNXInferenceModel&&) = default;

  // Destructor
  ~ONNXInferenceModel() = default;

  // Load model from file
  bool loadModel(const std::string& modelPath);

  // Run inference on input data
  std::vector<float> runInference(const std::vector<float>& inputData, const std::vector<int64_t>& inputShape);

  // Print model information to stream
  template <typename StreamT>
  void dumpModel(StreamT& stream) const;

private:
  // ONNX Runtime objects
  std::unique_ptr<Ort::Env> m_env{nullptr};
  std::unique_ptr<Ort::Session> m_session{nullptr};
  std::unique_ptr<Ort::SessionOptions> m_sessionOptions{nullptr};

  // Model metadata
  std::vector<std::string> m_inputNames{};
  std::vector<std::string> m_outputNames{};
  std::vector<std::vector<int64_t>> m_inputShapes{};
  std::vector<std::vector<int64_t>> m_outputShapes{};
  std::string m_envName{};

  bool m_modelLoaded{false};

  // Helper methods
  void extractModelInfo();
  void cleanup();
};

template <typename StreamT>
void ONNXInferenceModel::dumpModel(StreamT& stream) const {
  if (!m_modelLoaded) {
    stream << "Model not loaded" << std::endl;
    return;
  }

  stream << "=== ONNX Model Information ===" << std::endl;
  stream << "Environment Name: " << m_envName << std::endl;

  stream << "Inputs (" << m_inputNames.size() << "):" << std::endl;
  for (size_t i = 0; i < m_inputNames.size(); ++i) {
    stream << "  [" << i << "] " << m_inputNames[i] << " - Shape: [";
    for (size_t j = 0; j < m_inputShapes[i].size(); ++j) {
      if (j > 0)
        stream << ", ";
      stream << m_inputShapes[i][j];
    }
    stream << "]" << std::endl;
  }

  stream << "Outputs (" << m_outputNames.size() << "):" << std::endl;
  for (size_t i = 0; i < m_outputNames.size(); ++i) {
    stream << "  [" << i << "] " << m_outputNames[i] << " - Shape: [";
    for (size_t j = 0; j < m_outputShapes[i].size(); ++j) {
      if (j > 0)
        stream << ", ";
      stream << m_outputShapes[i][j];
    }
    stream << "]" << std::endl;
  }

  stream << "===============================" << std::endl;
}

} // namespace mlutils
