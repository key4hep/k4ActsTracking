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

#include "k4ActsTracking/CellIDSelector.h"

#include <algorithm>
#include <charconv>
#include <ranges>
#include <stdexcept>
#include <string_view>
#include <utility>

#include <iostream>

namespace {
  auto splitString(const std::string_view str, const char delim) {
    namespace rv = std::ranges::views;

    return str | rv::split(delim) |
           rv::transform([](auto&& subrange) { return std::string_view(subrange.begin(), subrange.end()); });
  }

  // Implement a very simple polyfill to maintain compatibility with c++20
  template <typename T, std::ranges::input_range R> auto to_vector(R&& range) {
    std::vector<T> container;
    if constexpr (std::ranges::sized_range<R>) {
      container.reserve(std::ranges::size(range));
    }
    std::ranges::copy(range, std::back_inserter(container));
    return container;
  }

  std::pair<std::string, std::vector<int>> getFieldAndValues(auto partialSelection) {
    const auto fieldConfig = to_vector<std::string_view>(splitString(partialSelection, ':'));
    if (fieldConfig.size() != 2) {
      throw std::invalid_argument(std::string(partialSelection) + "' is not a valid selection string");
    }

    const auto fieldName  = std::string(fieldConfig[0]);
    const auto fieldValue = fieldConfig[1];
    if (fieldValue.empty()) {
      throw std::invalid_argument("'" + std::string(partialSelection) +
                                  "' has an empty value — use '*' for a wildcard");
    }
    if (fieldValue == "*") {
      return {fieldName, {}};
    }

    namespace rv = std::ranges::views;
    auto layers  = splitString(fieldValue, '|') | rv::transform([&](auto&& elem) {
                    int val;
                    if (std::from_chars<int>(elem.data(), elem.data() + elem.size(), val).ec != std::errc{}) {
                      throw std::invalid_argument("'" + std::string(elem) + "' in '" + std::string(partialSelection) +
                                                   "' cannot be converted to an integer");
                    }
                    return val;
                  });
    return {fieldName, to_vector<int>(layers)};
  }

  struct FieldValue {
    std::string name;
    int         value;
  };

  std::vector<std::vector<FieldValue>> cartesianProductFields(auto&& fieldsAndValues) {
    // fieldsAndValues is a range with elements of shape pair<string,
    // vector<int>>. Each element in the range represents a field and values for
    // that field. What we want is each combination of {(field_i, value_i),
    // (field_j, value_j), (...)}
    std::vector<std::vector<FieldValue>> product{};

    for (const auto& [field, values] : fieldsAndValues) {
      std::vector<FieldValue> thisFieldValues{};
      thisFieldValues.reserve(values.size());
      for (const int value : values) {
        thisFieldValues.emplace_back(field, value);
      }

      // If we are in the first field product is still empty and we simply store
      // each individual value from this field there.
      if (product.empty()) {
        for (auto&& fv : thisFieldValues) {
          product.emplace_back(std::vector{std::move(fv)});
        }
      } else {
        // Otherwise we create new products by attaching the new field / values
        // to the existing ones and then simply replace the existing ones
        std::vector<std::vector<FieldValue>> newProduct{};
        for (auto existing : product) {
          for (const auto& fv : thisFieldValues) {
            existing.emplace_back(fv);
            newProduct.emplace_back(existing);
          }
        }
        product = std::move(newProduct);
      }
    }

    return product;
  }

}  // namespace

namespace k4ActsTracking {
  CellIDSelector::CellIDSelector(const std::string& encodingString, const std::vector<std::string>& selections)
      : m_decoder(encodingString) {
    for (const auto& selection : selections) {
      auto masks = getSelectionMasks(selection);
      if (masks.empty()) {
        // All fields are wildcards: this selection accepts every CellID,
        // represented by a zero mask (no bits to check).
        m_selectors.emplace_back(0ULL, 0ULL);
      } else {
        for (auto&& sel : masks) {
          m_selectors.emplace_back(std::move(sel));
        }
      }
    }
  }

  std::vector<CellIDSelector::Selector> CellIDSelector::getSelectionMasks(const std::string& selection) const {
    if (selection.empty()) {
      throw std::invalid_argument("selection string must not be empty");
    }
    // Get all the field names and values where we actually have to apply a
    // selection in the end. I.e. if a field is configured with '*', we filter
    // it out here already since it will be applied in the selection in any
    // case.
    namespace rv         = std::ranges::views;
    auto fieldsAndValues = splitString(selection, ',') |
                           rv::transform([](auto&& part) { return getFieldAndValues(part); }) |
                           rv::filter([](auto&& v) { return !v.second.empty(); });

    dd4hep::CellID mask = 0;
    for (const auto& [field, _] : fieldsAndValues) {
      mask |= m_decoder[field].mask();
    }

    std::vector<Selector> selectors{};
    for (const auto& fieldsValues : cartesianProductFields(fieldsAndValues)) {
      dd4hep::CellID value{};
      for (const auto& [name, val] : fieldsValues) {
        m_decoder.set(value, std::string(name), val);
      }
      selectors.emplace_back(mask, value);
    }

    return selectors;
  }

  bool CellIDSelector::accept(const dd4hep::CellID cellID) const {
    for (const auto& selector : m_selectors) {
      // The passed CellID has to be equal to any of the configured values after
      // masking irrelvant parts
      if ((selector.value & selector.mask) == (cellID & selector.mask)) {
        return true;
      }
    }
    return false;
  }
}  // namespace k4ActsTracking
