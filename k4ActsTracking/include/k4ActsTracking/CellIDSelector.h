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
#ifndef K4ACTSTRACKING_CELLIDSELECTOR_H
#define K4ACTSTRACKING_CELLIDSELECTOR_H

#include <DD4hep/BitFieldCoder.h>
#include <Parsers/Primitives.h>

#include <string>
#include <vector>

namespace k4ActsTracking {

  /// Selects detector hits by matching their DD4hep CellID against a set of
  /// user-defined selection strings.
  ///
  /// Each selection string describes a conjunction (AND) of field constraints.
  /// Multiple selection strings passed to the constructor are evaluated as a
  /// disjunction (OR): a CellID is accepted if it satisfies **at least one** of
  /// them.
  ///
  /// ## Selection string grammar
  ///
  /// A selection string is a comma-separated list of `<field>:<value>`
  /// constraints, where `<field>` must be a field name present in the DD4hep
  /// encoding string supplied to the constructor.
  ///
  /// ### Value syntax
  ///
  /// | Syntax          | Meaning                                      |
  /// |-----------------|----------------------------------------------|
  /// | `field:N`       | Field must equal the integer `N`             |
  /// | `field:*`       | Wildcard — field is not constrained          |
  /// | `field:N\|M\|…` | Field must equal one of `N`, `M`, … (OR)    |
  ///
  /// Fields that appear in the encoding string but are absent from the
  /// selection are treated as wildcards.  Specifying the same field more than
  /// once in a single selection string is not detected as an error: only the
  /// last occurrence takes effect.
  ///
  /// ### Examples
  ///
  /// ```
  /// // Accept hits in system 8, layer 3 (any module, sensor, side, …)
  /// CellIDSelector sel(enc, {"system:8,layer:3"});
  ///
  /// // Accept hits in system 5 on layers 1, 4, or 5 OR
  /// // in system 3 on layers 2, 6, or 8 OR
  /// // any hit whose sensor field equals 42
  /// CellIDSelector sel(enc, {"system:5,layer:1|4|5",
  ///                          "system:3,layer:2|6|8",
  ///                          "sensor:42"});
  ///
  /// // Wildcard: system is unconstrained, layer must be 3
  /// CellIDSelector sel(enc, {"system:*,layer:3"});
  /// // This is equivalent to
  /// CellIDSelector sel(enc, {"layer:3"});
  ///
  /// // Empty selection list — rejects every CellID
  /// CellIDSelector sel(enc, {});
  /// ```
  ///
  /// ### Expansion to (mask, value) pairs
  ///
  /// Internally each selection string is expanded into one or more
  /// `Selector{mask, value}` pairs via the Cartesian product of the
  /// per-field value lists.  A CellID passes the selector when
  /// `(cellID & mask) == (value & mask)`.  Wildcard fields contribute
  /// neither to `mask` nor to `value`.
  class CellIDSelector {
  public:
    /// A single bitmask/value pair produced from one element of the Cartesian
    /// product expansion of a selection string.
    struct Selector {
      dd4hep::CellID mask;   ///< Bits that are relevant for this selector
      dd4hep::CellID value;  ///< Expected bit pattern after masking
    };

    /// Construct a selector from a DD4hep encoding string and a list of
    /// selection strings.
    ///
    /// @param encodingString  DD4hep BitFieldCoder descriptor, e.g.
    ///                        `"system:8,side:-2,layer:5,module:7,sensor:10"`
    /// @param selections      List of selection strings (OR-ed together).
    ///                        An empty list rejects every CellID.
    /// @throws std::invalid_argument if a selection string is malformed or
    ///         references an unknown field.
    CellIDSelector(const std::string& encodingString, const std::vector<std::string>& selections);

    /// Return `true` if @p cellID satisfies at least one configured selector.
    bool accept(const dd4hep::CellID cellID) const;

    /// Expand a single selection string into its `Selector` pairs.
    ///
    /// Useful for inspection and testing.  The returned vector contains one
    /// entry per element of the Cartesian product of all non-wildcard field
    /// value lists.  An all-wildcard selection string returns an empty vector.
    ///
    /// @param selection  A single selection string (see class-level grammar).
    /// @throws std::invalid_argument if the string is malformed.
    std::vector<Selector> getSelectionMasks(const std::string& selection) const;

  private:
    dd4hep::BitFieldCoder m_decoder{};
    std::vector<Selector> m_selectors{};
  };
}  // namespace k4ActsTracking

#endif  //  K4ACTSTRACKING_CELLIDSELECTOR_H
