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

#include <algorithm>
#include <cctype>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

namespace gnntracking {

/**
 * @brief Parse a comma-separated string into a vector of values.
 *
 * Splits @p str on commas, trims surrounding whitespace from each element and
 * converts it to type T. Empty elements are skipped.
 *
 * @param str The comma-separated string to parse
 * @return A vector of parsed values
 */
template <typename T>
std::vector<T> parseList(const std::string& str) {
  const auto trim = [](std::string s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) { return !std::isspace(ch); }));
    s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) { return !std::isspace(ch); }).base(), s.end());
    return s;
  };

  std::vector<T> result;
  std::stringstream ss(str);
  std::string item;
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
template <typename T>
std::vector<std::vector<T>> parseMultiList(const std::vector<std::string>& strList) {
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

} // namespace gnntracking
