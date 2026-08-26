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

#include <edm4hep/TrackerHitPlaneCollection.h>

#include <DDSegmentation/BitFieldCoder.h>

#include <cstddef>
#include <string>
#include <vector>

namespace ACTSTracking {

  /// The per-hit quantity a configured feature name maps to. All CellID based
  /// features share one enumerator and are distinguished by the decoder field
  /// index stored alongside it.
  enum class HitFeature { X, Y, Z, R, Phi, Theta, Eta, Time, Energy, CellIdField };

  /// A configured input feature, resolved once to the quantity that has to be
  /// read from a hit. Keeping the resolution out of the event loop avoids
  /// per-hit string comparisons and CellID field name lookups.
  struct ResolvedFeature {
    HitFeature  kind{};
    std::size_t cellIdField{0};  ///< only used for HitFeature::CellIdField
  };

  /// The (case insensitive) feature names that resolveHitFeature() accepts, as a
  /// comma separated list. Meant for error messages and documentation.
  std::string supportedHitFeatureNames();

  /// Whether @p feature is one of the CellID based features, i.e. whether
  /// resolving it needs a decoder. Unknown names return false, they are reported
  /// by resolveHitFeature().
  bool hitFeatureNeedsCellID(const std::string& feature);

  /// Resolve a single feature name into the quantity that has to be read from a
  /// hit.
  ///
  /// @param decoder the CellID decoder to look field names up in. May be nullptr
  ///        if no CellID encoding is available, in which case requesting a
  ///        CellID based feature is an error.
  /// @throws std::runtime_error for an unknown feature name, or for a CellID
  ///         based feature whose field is not part of the encoding (or when no
  ///         decoder was given at all).
  ResolvedFeature resolveHitFeature(const std::string& feature, const dd4hep::DDSegmentation::BitFieldCoder* decoder);

  /// Resolve a list of feature names, see resolveHitFeature().
  std::vector<ResolvedFeature> resolveHitFeatures(const std::vector<std::string>&              features,
                                                  const dd4hep::DDSegmentation::BitFieldCoder* decoder);

  /// The value of one resolved feature for one hit.
  ///
  /// @param decoder only dereferenced for HitFeature::CellIdField, which
  ///        resolveHitFeature() only ever produces when a decoder was given.
  float hitFeatureValue(const edm4hep::TrackerHitPlane& hit, const ResolvedFeature& feature,
                        const dd4hep::DDSegmentation::BitFieldCoder* decoder);

  /// Extract the requested hit information into a flat, row-major
  /// (nHits x nFeatures) buffer, i.e. the layout the ONNX models expect.
  std::vector<float> extractHitInformation(const edm4hep::TrackerHitPlaneCollection&    hits,
                                           const std::vector<ResolvedFeature>&          features,
                                           const dd4hep::DDSegmentation::BitFieldCoder* decoder);

}  // namespace ACTSTracking
