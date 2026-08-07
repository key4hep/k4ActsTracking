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
#include <k4ActsTracking/IActsGeoSvc.h>
#include <k4ActsTracking/HitFeatures.hxx>

#include <k4FWCore/GaudiChecks.h>
#include <k4FWCore/Transformer.h>

#include <GaudiKernel/SmartIF.h>

#include <Gaudi/Property.h>

#include <edm4hep/TrackerHitPlaneCollection.h>

#include <DDSegmentation/BitFieldCoder.h>

#include <fmt/format.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <numeric>
#include <optional>
#include <string>
#include <utility>
#include <vector>

/// Reorders a tracker hit collection by one of the hit features that
/// GNNTrackFinder understands.
///
/// The output is a subset collection, i.e. it refers to the very same hits as
/// the input and only changes the order they appear in. This is mostly useful to
/// make the hit order of a job deterministic and independent of how the
/// digitisation happened to emit them.
struct SortTrackerHitsAlg final
    : public k4FWCore::Transformer<edm4hep::TrackerHitPlaneCollection(const edm4hep::TrackerHitPlaneCollection&)> {
  SortTrackerHitsAlg(const std::string& name, ISvcLocator* svcLoc)
      : Transformer(name, svcLoc, {KeyValues("InputHitCollection", {"populate-me-properly"})},
                    {KeyValues("OutputHitCollection", {"SortedTrackerHits"})}) {}

  StatusCode initialize() override {
    // Only the CellID based features need the geometry (for the encoding
    // string), so sorting by e.g. "r" does not drag in the ACTS geometry.
    const dd4hep::DDSegmentation::BitFieldCoder* decoder = nullptr;
    if (ACTSTracking::hitFeatureNeedsCellID(m_sortBy.value())) {
      m_actsGeoSvc = svcLoc()->service<IActsGeoSvc>("ActsGeoSvc");
      K4_GAUDI_CHECK(m_actsGeoSvc);
      try {
        m_cellIDDecoder.emplace(m_actsGeoSvc->cellIDEncodingString());
      } catch (const std::exception& ex) {
        error() << fmt::format("Could not build a CellID decoder for sorting by '{}': {}", m_sortBy.value(), ex.what())
                << endmsg;
        return StatusCode::FAILURE;
      }
      decoder = &*m_cellIDDecoder;
    }

    // Resolve the feature once, so that the event loop neither compares strings
    // nor looks up CellID fields by name.
    try {
      m_feature = ACTSTracking::resolveHitFeature(m_sortBy.value(), decoder);
    } catch (const std::exception& ex) {
      error() << ex.what() << endmsg;
      return StatusCode::FAILURE;
    }

    info() << fmt::format("Sorting hits by '{}' in {} order", m_sortBy.value(),
                          m_descending.value() ? "descending" : "ascending")
           << endmsg;
    return StatusCode::SUCCESS;
  }

  edm4hep::TrackerHitPlaneCollection operator()(const edm4hep::TrackerHitPlaneCollection& hits) const override {
    const dd4hep::DDSegmentation::BitFieldCoder* decoder = m_cellIDDecoder ? &*m_cellIDDecoder : nullptr;

    // Compute the sort key of every hit once instead of re-deriving it for each
    // comparison (eta and phi are not free), then sort indices into the input.
    std::vector<float> keys{};
    keys.reserve(hits.size());
    for (const auto hit : hits) {
      float key = ACTSTracking::hitFeatureValue(hit, *m_feature, decoder);
      // A NaN key would break the strict weak ordering std::stable_sort needs,
      // so park those at the end instead of risking an out-of-bounds compare.
      if (std::isnan(key)) {
        key = std::numeric_limits<float>::infinity();
      }
      keys.push_back(key);
    }

    std::vector<std::size_t> order(hits.size());
    std::iota(order.begin(), order.end(), 0);
    // Stable, so that hits with an equal key keep their relative input order and
    // the result does not depend on the sort implementation.
    const bool descending = m_descending.value();
    std::stable_sort(order.begin(), order.end(), [&keys, descending](std::size_t lhs, std::size_t rhs) {
      return descending ? keys[rhs] < keys[lhs] : keys[lhs] < keys[rhs];
    });

    // A subset collection refers to the input hits rather than copying them, so
    // this only reorders references.
    edm4hep::TrackerHitPlaneCollection sorted{};
    sorted.setSubsetCollection(true);
    for (const std::size_t idx : order) {
      sorted.push_back(hits[idx]);
    }

    debug() << fmt::format("Sorted {} hits by '{}'", sorted.size(), m_sortBy.value()) << endmsg;
    return sorted;
  }

  Gaudi::Property<std::string> m_sortBy{
      this, "SortBy", "r",
      "The hit feature to sort by. Supported (case insensitive): x, y, z, r, phi, theta, eta, t (or time), E (or "
      "energy), module_id, layer_id, system_id (or volume_id). The CellID based ones need the ActsGeoSvc."};
  Gaudi::Property<bool> m_descending{this, "Descending", false,
                                     "Sort in descending instead of ascending order. Ties keep their input order "
                                     "either way."};

private:
  /// The feature resolved in initialize(), see m_sortBy
  std::optional<ACTSTracking::ResolvedFeature> m_feature{};

  /// CellID decoder, only built for the CellID based features (parsing the
  /// encoding string is too expensive to redo for every event).
  std::optional<dd4hep::DDSegmentation::BitFieldCoder> m_cellIDDecoder{};

  SmartIF<IActsGeoSvc> m_actsGeoSvc{nullptr};

public:
  void registerCallBack(Gaudi::StateMachine::Transition, std::function<void()>) {}
};

DECLARE_COMPONENT(SortTrackerHitsAlg)
