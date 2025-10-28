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
