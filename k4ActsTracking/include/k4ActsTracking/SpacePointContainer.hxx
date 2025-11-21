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

#include <Acts/EventData/SpacePointContainer.hpp>

namespace ACTSTracking {

  template <typename collection_t> class SpacePointContainer {
  public:
    using CollectionType = collection_t;
    using ValueType      = typename CollectionType::value_type;

    friend Acts::SpacePointContainer<ACTSTracking::SpacePointContainer<collection_t>, Acts::detail::RefHolder>;

    SpacePointContainer()                           = delete;
    SpacePointContainer(CollectionType&& container) = delete;
    explicit SpacePointContainer(CollectionType& container) : m_storage(container) {}
    explicit SpacePointContainer(CollectionType* container) : m_storage(container) {}

    SpacePointContainer(const SpacePointContainer<collection_t>&)                          = delete;
    SpacePointContainer<collection_t>& operator=(const SpacePointContainer<collection_t>&) = delete;

    SpacePointContainer(SpacePointContainer<collection_t>&& other) noexcept
        : m_storage(std::exchange(other.m_storage.ptr, nullptr)) {}

    SpacePointContainer<collection_t>& operator=(SpacePointContainer<collection_t>&& other) noexcept {
      m_storage = std::exchange(other.m_storage.ptr, nullptr);
      return *this;
    }

    ~SpacePointContainer() = default;

  private:
    std::size_t size_impl() const { return storage().size(); }

    float x_impl(std::size_t idx) const { return storage()[idx]->x(); }
    float y_impl(std::size_t idx) const { return storage()[idx]->y(); }
    float z_impl(std::size_t idx) const { return storage()[idx]->z(); }
    float varianceR_impl(std::size_t idx) const { return storage()[idx]->varianceR(); }
    float varianceZ_impl(std::size_t idx) const { return storage()[idx]->varianceZ(); }

    const ValueType& get_impl(std::size_t idx) const { return storage()[idx]; }

    std::any component_impl(Acts::HashedString key, std::size_t /*n*/) const {
      using namespace Acts::HashedStringLiteral;
      switch (key) {
        case "TopStripVector"_hash:
        case "BottomStripVector"_hash:
        case "StripCenterDistance"_hash:
        case "TopStripCenterPosition"_hash:
          return Acts::Vector3(0, 0, 0);
        default:
          throw std::runtime_error("no such component " + std::to_string(key));
      }
    }

    const CollectionType& storage() const { return *m_storage; }

    Acts::detail::RefHolder<CollectionType> m_storage;
  };

}  // namespace ACTSTracking
