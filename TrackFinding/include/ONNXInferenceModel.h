#pragma once

#include <onnxruntime_cxx_api.h>

#include <memory>
#include <string>
#include <vector>

namespace mlutils {
namespace detail {
  /**
   * @brief Type trait to extract the scalar type from nested containers.
   *
   * For scalar types, returns the type itself. For std::vector<T>, recursively
   * extracts the scalar type from T. This allows handling arbitrarily nested
   * vectors like std::vector<std::vector<float>>.
   */
  template <typename T>
  struct scalar_type {
    using type = T;
  };

  template <typename T>
  struct scalar_type<std::vector<T>> {
    using type = typename scalar_type<T>::type;
  };

  template <typename T>
  using scalar_type_t = typename scalar_type<T>::type;
} // namespace detail

/**
 * @brief Calculate the total number of scalar elements in a nested container.
 *
 * For scalar types, returns 1. For containers, recursively counts all scalar
 * elements. Useful for pre-allocating memory when flattening nested structures.
 *
 * @param value The value or container to count elements in
 * @return Total number of scalar elements
 */
template <typename T>
size_t totalSize(const T&) {
  return 1;
}

template <typename T>
size_t totalSize(const std::vector<T>& value) {
  size_t size = 0;
  for (const auto& v : value) {
    size += totalSize(v);
  }

  return size;
}

/**
 * @brief Flatten nested containers into a single vector of scalar values.
 *
 * Recursively traverses nested std::vector structures and extracts all scalar
 * values into a flat output vector. Supports arbitrary nesting levels.
 *
 * @param value The value or container to flatten
 * @param output The output vector to append flattened values to
 */
template <typename T>
void flatten(const T& value, std::vector<detail::scalar_type_t<T>>& output) {
  output.push_back(value);
}

template <typename T>
void flatten(const std::vector<T>& vec, std::vector<detail::scalar_type_t<T>>& output) {
  for (const auto& item : vec) {
    flatten(item, output);
  }
}

/**
 * @brief Flatten nested containers into a new vector of scalar values.
 *
 * Convenience function that creates a new vector and flattens the input into it
 *
 * @param value The value or container to flatten
 * @return A new vector containing all scalar values in flattened form
 */
template <typename T>
std::vector<detail::scalar_type_t<T>> flatten(const T& value) {
  std::vector<detail::scalar_type_t<T>> output{};
  output.reserve(totalSize(value));
  flatten(value, output);
  return output;
}

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
