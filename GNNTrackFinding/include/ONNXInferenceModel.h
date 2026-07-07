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

#include <algorithm>
#include <cassert>
#include <cctype>
#include <cstdint>
#include <memory>
#include <numeric>
#include <ranges>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
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
    template <typename T> struct scalar_type {
      using type = T;
    };

    template <typename T> struct scalar_type<std::vector<T>> {
      using type = typename scalar_type<T>::type;
    };

    template <typename T> using scalar_type_t = typename scalar_type<T>::type;
  }  // namespace detail

  /**
 * @brief Calculate the total number of scalar elements in a nested container.
 *
 * For scalar types, returns 1. For containers, recursively counts all scalar
 * elements. Useful for pre-allocating memory when flattening nested structures.
 *
 * @param value The value or container to count elements in
 * @return Total number of scalar elements
 */
  template <typename T> size_t totalSize(const T&) { return 1; }

  template <typename T> size_t totalSize(const std::vector<T>& value) {
    // No fold_left yet
    const auto sizes = value | std::views::transform([](const auto& v) { return totalSize(v); });
    return std::accumulate(sizes.begin(), sizes.end(), 0);
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
  template <typename T> void flatten(const T& value, std::vector<detail::scalar_type_t<T>>& output) {
    output.push_back(value);
  }

  template <typename T> void flatten(const std::vector<T>& vec, std::vector<detail::scalar_type_t<T>>& output) {
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
  template <typename T> std::vector<detail::scalar_type_t<T>> flatten(const T& value) {
    std::vector<detail::scalar_type_t<T>> output{};
    output.reserve(totalSize(value));
    flatten(value, output);
    return output;
  }

  /**
 * @brief Extract the dimensions of a nested container structure.
 *
 * For scalar types, returns an empty vector (0-dimensional).
 * For nested std::vector structures, returns the dimensions as a vector of sizes.
 *
 * @note Assumes that all vectors at the same nesting level have the same size
 * (i.e. rectangular structure) and only looks at the first element at each level to determine dimensions. Does not do
 * any validation of these assumptions!
 *
 * @param value The value or container to analyze
 * @return Vector of dimensions, where each element represents the size at that nesting level
 */
  template <typename T> std::vector<int64_t> getDimensions(const T&) { return std::vector<int64_t>{}; }

  template <typename T> std::vector<int64_t> getDimensions(const std::vector<T>& value) {
    if (value.empty()) {
      return std::vector<int64_t>{0};
    } else {
      // Get the dimensions of the first nested vector
      auto dims = getDimensions(value[0]);
      // Prepend the size of the current vector to the dimensions we already collected
      dims.insert(dims.begin(), value.size());
      return dims;
    }
  }

  /**
   * @brief Parse a comma-separated string into a vector of values.
   *
   * Splits @p str on commas, trims surrounding whitespace from each element and
   * converts it to type T. Empty elements are skipped.
   *
   * @param str The comma-separated string to parse
   * @return A vector of parsed values
   */
  template <typename T> std::vector<T> parseList(const std::string& str) {
    const auto trim = [](std::string s) {
      s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) { return !std::isspace(ch); }));
      s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) { return !std::isspace(ch); }).base(), s.end());
      return s;
    };

    std::vector<T>    result;
    std::stringstream ss(str);
    std::string       item;
    while (std::getline(ss, item, ',')) {
      const auto trimmed = trim(item);
      if (trimmed.empty()) {
        continue;
      }
      if constexpr (std::is_same_v<T, std::string>) {
        result.push_back(trimmed);
      } else if constexpr (std::is_same_v<T, int>) {
        result.push_back(std::stoi(trimmed));
      } else if constexpr (std::is_same_v<T, float>) {
        result.push_back(std::stof(trimmed));
      } else if constexpr (std::is_same_v<T, double>) {
        result.push_back(std::stod(trimmed));
      } else {
        throw std::runtime_error("parseList does not support this type");
      }
    }
    return result;
  }

  /**
   * @brief Parse a list of comma-separated strings into a vector of vectors.
   *
   * Each entry of @p strList is parsed with parseList<T>; empty entries yield an
   * empty sub-vector. Used e.g. to configure per-stage feature lists for
   * multiple edge classifiers.
   */
  template <typename T> std::vector<std::vector<T>> parseMultiList(const std::vector<std::string>& strList) {
    std::vector<std::vector<T>> result;
    result.reserve(strList.size());
    for (const auto& group : strList) {
      if (group.empty()) {
        result.emplace_back();
      } else {
        result.emplace_back(parseList<T>(group));
      }
    }
    return result;
  }

  class ONNXInferenceModel {
  public:
    // Constructor. When useCuda is true the CUDA execution provider is appended
    // to the session options (requires a CUDA-enabled onnxruntime build).
    explicit ONNXInferenceModel(const std::string& name, OrtLoggingLevel logLevel = ORT_LOGGING_LEVEL_WARNING,
                                bool useCuda = false, std::size_t cudaDeviceIndex = 0);

    ONNXInferenceModel()                                     = delete;
    ONNXInferenceModel(const ONNXInferenceModel&)            = delete;
    ONNXInferenceModel& operator=(const ONNXInferenceModel&) = delete;
    ONNXInferenceModel(ONNXInferenceModel&&)                 = default;
    ONNXInferenceModel& operator=(ONNXInferenceModel&&)      = default;

    // Destructor
    ~ONNXInferenceModel() = default;

    // Load model from file
    bool loadModel(const std::string& modelPath);

    template <typename T> [[nodiscard]] std::vector<Ort::Value> runInference(const T& inputData);

    // Run inference on input data
    [[nodiscard]] std::vector<Ort::Value> runInference(const std::vector<float>&   inputData,
                                                       const std::vector<int64_t>& inputShape);

    // Print model information to stream
    template <typename StreamT> void dumpModel(StreamT& stream) const;

  private:
    // ONNX Runtime objects
    std::unique_ptr<Ort::Env>            m_env{nullptr};
    std::unique_ptr<Ort::Session>        m_session{nullptr};
    std::unique_ptr<Ort::SessionOptions> m_sessionOptions{nullptr};
    Ort::AllocatorWithDefaultOptions     m_allocator{};  // default allocator

    // Model metadata
    std::vector<std::string>          m_inputNames{};
    std::vector<std::string>          m_outputNames{};
    std::vector<std::vector<int64_t>> m_inputShapes{};
    std::vector<std::vector<int64_t>> m_outputShapes{};
    std::string                       m_envName{};

    bool m_modelLoaded{false};

    // Helper methods
    void extractModelInfo();
    void cleanup();
  };

  template <typename T> std::vector<Ort::Value> ONNXInferenceModel::runInference(const T& inputData) {
    auto input = flatten(inputData);
    auto dims  = getDimensions(inputData);
    // Very basic check to at least avoid glaring mistakes in the inputData shape
    assert(input.size() == std::reduce(dims.begin(), dims.end(), 1u, std::multiplies<>()));

    return runInference(input, dims);
  }

  template <typename StreamT> void ONNXInferenceModel::dumpModel(StreamT& stream) const {
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

}  // namespace mlutils
