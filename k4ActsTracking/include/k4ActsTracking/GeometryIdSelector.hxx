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

#include <Acts/Geometry/GeometryIdentifier.hpp>
#include <vector>

namespace ACTSTracking {
  /**
 * @brief Filter hits based on geometry ID's
 *
 * @author Karol Krizka
 * @version $Id$
 */
  class GeometryIdSelector {
  public:
    //! Type used to indicate mask
    using Mask = Acts::GeometryIdentifier::Value;

    GeometryIdSelector() = default;
    GeometryIdSelector(const std::vector<Acts::GeometryIdentifier>& selection);

    //! Determine if geometry belong to any requested geometries
    bool check(const Acts::GeometryIdentifier& geoID) const;

    //! Make mask for comparing wildcarded geometry identifier
    static Mask makeMask(const Acts::GeometryIdentifier& id);

  private:
    //! List of geometry's to match against along with the precomputed mask
    std::vector<std::pair<Acts::GeometryIdentifier, Mask>> m_selection;
  };

}  // namespace ACTSTracking
