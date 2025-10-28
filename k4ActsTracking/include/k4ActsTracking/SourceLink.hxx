#pragma once

#include <edm4hep/TrackerHitPlane.h>

#include "Acts/Surfaces/Surface.hpp"

#include "k4ActsTracking/GeometryContainers.hxx"

namespace ACTSTracking {
  //! \brief Link between an ACTS surface and hit index
  class SourceLink final {
  public:
    //! \brief Construct from geometry identifier and hit
    SourceLink(Acts::GeometryIdentifier gid, std::size_t index, edm4hep::TrackerHitPlane* edmhit)
        : m_geometryId(gid), m_index(index), m_edm4hephit(edmhit) {}

    // Construct an invalid source link. Must be default constructible to
    /// satisfy SourceLinkConcept.
    SourceLink()                  = default;
    SourceLink(const SourceLink&) = default;
    SourceLink(SourceLink&&)      = default;
    SourceLink& operator=(const SourceLink&) = default;
    SourceLink& operator=(SourceLink&&) = default;

    /// Access the geometry identifier.
    constexpr Acts::GeometryIdentifier geometryId() const { return m_geometryId; }
    /// Access the index.
    constexpr std::size_t index() const { return m_index; }
    /// Access the edm4hep TrackerHitPlane
    /// @TODO: We want this to be a TrackerHit to support multiple types of tracking detector. However, TrackerHitPlane is currently not derived from TrackerHit as expected.
    constexpr edm4hep::TrackerHitPlane* edm4hepTHitP() const { return m_edm4hephit; }

  private:
    Acts::GeometryIdentifier  m_geometryId;
    std::size_t               m_index      = -1;
    edm4hep::TrackerHitPlane* m_edm4hephit = nullptr;

    friend constexpr bool operator==(const SourceLink& lhs, const SourceLink& rhs) {
      return (lhs.m_geometryId == rhs.m_geometryId) and (lhs.m_index == rhs.m_index) and
             (lhs.m_edm4hephit == rhs.m_edm4hephit);
    }
    friend constexpr bool operator!=(const SourceLink& lhs, const SourceLink& rhs) { return not(lhs == rhs); }
  };

  /// Container of index source links
  using SourceLinkContainer = GeometryIdMultiset<SourceLink>;
  /// Accessor for the above source link container
  ///
  /// It wraps up a few lookup methods to be used in the Combinatorial Kalman
  /// Filter
  struct SourceLinkAccessor : GeometryIdMultisetAccessor<SourceLink> {
    using BaseIterator = GeometryIdMultisetAccessor<SourceLink>::Iterator;

    using Iterator = Acts::SourceLinkAdapterIterator<BaseIterator>;

    // get the range of elements with requested geoId
    std::pair<Iterator, Iterator> range(const Acts::Surface& surface) const {
      assert(container != nullptr);
      auto [begin, end] = container->equal_range(surface.geometryId());
      return {Iterator{begin}, Iterator{end}};
    }
  };

}  // namespace ACTSTracking
