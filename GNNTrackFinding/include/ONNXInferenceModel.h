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

#include <onnxruntime_cxx_api.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <ostream>
#include <string>
#include <vector>

namespace mlutils {

class ONNXInferenceModel {
public:
  // Constructor. When useCuda is true the CUDA execution provider is appended
  // to the session options (requires a CUDA-enabled onnxruntime build).
  explicit ONNXInferenceModel(const std::string& name, OrtLoggingLevel logLevel = ORT_LOGGING_LEVEL_WARNING,
                              bool useCuda = false, std::size_t cudaDeviceIndex = 0);

  ONNXInferenceModel() = delete;
  ONNXInferenceModel(const ONNXInferenceModel&) = delete;
  ONNXInferenceModel& operator=(const ONNXInferenceModel&) = delete;
  ONNXInferenceModel(ONNXInferenceModel&&) = default;
  ONNXInferenceModel& operator=(ONNXInferenceModel&&) = default;

  // Destructor
  ~ONNXInferenceModel() = default;

  // Load model from file. Throws std::runtime_error naming the model and the
  // onnxruntime error if that fails, leaving no model loaded.
  void loadModel(const std::string& modelPath);

  // Run inference on input data
  [[nodiscard]] std::vector<Ort::Value> runInference(const std::vector<float>& inputData,
                                                     const std::vector<int64_t>& inputShape, int cudaDeviceIndex = -1);

  template <typename DeviceT>
  [[nodiscard]] std::vector<Ort::Value> runInference(const std::vector<float>& inputData,
                                                     const std::vector<int64_t>& inputShape, const DeviceT& device);

  // Print model information to stream
  template <typename StreamT>
  void dumpModel(StreamT& stream) const;

  // The shape the loaded model declares for its @p i-th input. Axes that the
  // model marks as dynamic are reported as -1. Returns an empty shape if no
  // model is loaded or if @p i is out of range.
  [[nodiscard]] const std::vector<int64_t>& inputShape(std::size_t i) const {
    static const std::vector<int64_t> empty{};
    return i < m_inputShapes.size() ? m_inputShapes[i] : empty;
  }

  // The shape the loaded model declares for its @p i-th output. Axes that the
  // model marks as dynamic are reported as -1. Returns an empty shape if no
  // model is loaded or if @p i is out of range.
  [[nodiscard]] const std::vector<int64_t>& outputShape(std::size_t i) const {
    static const std::vector<int64_t> empty{};
    return i < m_outputShapes.size() ? m_outputShapes[i] : empty;
  }

  // Number of inputs of the loaded model
  [[nodiscard]] std::size_t numInputs() const { return m_inputShapes.size(); }

  // Number of outputs of the loaded model
  [[nodiscard]] std::size_t numOutputs() const { return m_outputShapes.size(); }

private:
  // ONNX Runtime objects
  std::unique_ptr<Ort::Env> m_env{nullptr};
  std::unique_ptr<Ort::Session> m_session{nullptr};
  std::unique_ptr<Ort::SessionOptions> m_sessionOptions{nullptr};
  Ort::AllocatorWithDefaultOptions m_allocator{}; // default allocator

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

template <typename DeviceT>
std::vector<Ort::Value> ONNXInferenceModel::runInference(const std::vector<float>& inputData,
                                                         const std::vector<int64_t>& inputShape,
                                                         const DeviceT& device) {
  const int cudaDeviceIndex = device.isCuda() ? static_cast<int>(device.index) : -1;
  return runInference(inputData, inputShape, cudaDeviceIndex);
}

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
