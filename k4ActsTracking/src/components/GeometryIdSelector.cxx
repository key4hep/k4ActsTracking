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

#include "k4ActsTracking/GeometryIdSelector.hxx"

#include <algorithm>

using namespace ACTSTracking;

GeometryIdSelector::GeometryIdSelector(const std::vector<Acts::GeometryIdentifier>& selection) {
  m_selection.resize(selection.size());
  std::transform(selection.begin(), selection.end(), m_selection.begin(),
                 [](const Acts::GeometryIdentifier& geoID) -> std::pair<Acts::GeometryIdentifier, Mask> {
                   return std::make_pair(geoID, makeMask(geoID));
                 });
}

GeometryIdSelector::Mask GeometryIdSelector::makeMask(const Acts::GeometryIdentifier& id) {
  // construct id from encoded value with all bits set
  Acts::GeometryIdentifier allSet = Acts::GeometryIdentifier(~Acts::GeometryIdentifier::Value(0u));
  // manually iterate over identifier levels starting from the lowest
  if (id.sensitive() != 0u) {
    // all levels are valid; keep all bits set.
    return allSet.value();
  }
  if (id.approach() != 0u) {
    return allSet.withSensitive(0u).value();
  }
  if (id.layer() != 0u) {
    return allSet.withSensitive(0u).withApproach(0u).value();
  }
  if (id.boundary() != 0u) {
    return allSet.withSensitive(0u).withApproach(0u).withLayer(0u).value();
  }
  if (id.volume() != 0u) {
    return allSet.withSensitive(0u).withApproach(0u).withLayer(0u).withBoundary(0u).value();
  }
  // no valid levels; all bits are zero.
  return GeometryIdSelector::Mask(0u);
}

bool GeometryIdSelector::check(const Acts::GeometryIdentifier& geoID) const {
  for (const std::pair<Acts::GeometryIdentifier, Mask>& reqGeoID : m_selection) {
    // equal within mask
    if ((geoID.value() & reqGeoID.second) == (reqGeoID.first.value() & reqGeoID.second))
      return true;
  }
  return false;
}
