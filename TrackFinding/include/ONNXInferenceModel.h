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

  bool m_modelLoaded{false};

  // Helper methods
  void extractModelInfo();
  void cleanup();
};

} // namespace mlutils
