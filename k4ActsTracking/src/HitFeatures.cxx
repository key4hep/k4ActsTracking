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
#include <k4ActsTracking/HitFeatures.hxx>

#include <Math/Point3D.h>

#include <fmt/format.h>

#include <algorithm>
#include <cassert>
#include <cctype>
#include <stdexcept>

namespace ACTSTracking {

  namespace {
    /// Lower-case an (ASCII) configuration string, so that feature names can be
    /// given in any case.
    std::string toLower(std::string str) {
      std::transform(str.begin(), str.end(), str.begin(), [](unsigned char c) { return std::tolower(c); });
      return str;
    }

    /// The CellID field a CellID based feature name reads, or nullptr if the
    /// name is not a CellID based feature.
    const char* cellIdFieldName(const std::string& lowerCaseFeature) {
      if (lowerCaseFeature == "module_id") {
        return "module";
      }
      if (lowerCaseFeature == "layer_id") {
        return "layer";
      }
      if (lowerCaseFeature == "system_id" || lowerCaseFeature == "volume_id") {
        return "system";
      }
      return nullptr;
    }
  }  // namespace

  std::string supportedHitFeatureNames() {
    return "x, y, z, r, phi, theta, eta, t (or time), E (or energy), module_id, layer_id, system_id (or volume_id)";
  }

  bool hitFeatureNeedsCellID(const std::string& feature) { return cellIdFieldName(toLower(feature)) != nullptr; }

  ResolvedFeature resolveHitFeature(const std::string& feature, const dd4hep::DDSegmentation::BitFieldCoder* decoder) {
    const auto key = toLower(feature);

    if (const char* field = cellIdFieldName(key); field != nullptr) {
      if (decoder == nullptr) {
        throw std::runtime_error(fmt::format(
            "Cannot use hit feature '{}': no CellID encoding is available to decode the '{}' field", feature, field));
      }
      // Look up the index of a CellID field once, so that decoding a hit is a
      // plain array access instead of a string based lookup.
      try {
        return ResolvedFeature{HitFeature::CellIdField, decoder->index(field)};
      } catch (const std::exception& ex) {
        throw std::runtime_error(fmt::format("Cannot use hit feature '{}': the CellID encoding has no '{}' field ({})",
                                             feature, field, ex.what()));
      }
    }

    if (key == "x") {
      return {HitFeature::X};
    }
    if (key == "y") {
      return {HitFeature::Y};
    }
    if (key == "z") {
      return {HitFeature::Z};
    }
    if (key == "r") {
      return {HitFeature::R};
    }
    if (key == "phi") {
      return {HitFeature::Phi};
    }
    if (key == "theta") {
      return {HitFeature::Theta};
    }
    if (key == "eta") {
      return {HitFeature::Eta};
    }
    if (key == "t" || key == "time") {
      return {HitFeature::Time};
    }
    if (key == "e" || key == "energy") {
      return {HitFeature::Energy};
    }

    throw std::runtime_error(
        fmt::format("Unknown hit feature '{}' (supported are: {})", feature, supportedHitFeatureNames()));
  }

  std::vector<ResolvedFeature> resolveHitFeatures(const std::vector<std::string>&              features,
                                                  const dd4hep::DDSegmentation::BitFieldCoder* decoder) {
    std::vector<ResolvedFeature> resolved{};
    resolved.reserve(features.size());
    for (const auto& f : features) {
      resolved.push_back(resolveHitFeature(f, decoder));
    }
    return resolved;
  }

  float hitFeatureValue(const edm4hep::TrackerHitPlane& hit, const ResolvedFeature& feature,
                        const dd4hep::DDSegmentation::BitFieldCoder* decoder) {
    const auto position = ROOT::Math::XYZPointF(hit.getPosition().x, hit.getPosition().y, hit.getPosition().z);

    switch (feature.kind) {
      case HitFeature::X:
        return position.x();
      case HitFeature::Y:
        return position.y();
      case HitFeature::Z:
        return position.z();
      case HitFeature::R:
        return position.rho();
      case HitFeature::Phi:
        return position.phi();
      case HitFeature::Theta:
        return position.theta();
      case HitFeature::Eta:
        return position.eta();
      case HitFeature::Time:
        return hit.getTime();
      case HitFeature::Energy:
        return hit.getEDep();
      case HitFeature::CellIdField:
        // resolveHitFeature() only produces this with a decoder in hand
        assert(decoder != nullptr);
        return static_cast<float>(decoder->get(hit.getCellID(), feature.cellIdField));
    }
    return 0.f;
  }

  std::vector<float> extractHitInformation(const edm4hep::TrackerHitPlaneCollection&    hits,
                                           const std::vector<ResolvedFeature>&          features,
                                           const dd4hep::DDSegmentation::BitFieldCoder* decoder) {
    std::vector<float> hitInfo{};
    hitInfo.reserve(hits.size() * features.size());

    for (const auto hit : hits) {
      for (const auto& feature : features) {
        hitInfo.push_back(hitFeatureValue(hit, feature, decoder));
      }
    }
    return hitInfo;
  }

}  // namespace ACTSTracking
