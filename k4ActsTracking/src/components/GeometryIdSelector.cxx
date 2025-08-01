#include "k4ActsTracking/GeometryIdSelector.hxx"

using namespace ACTSTracking;

GeometryIdSelector::GeometryIdSelector(
    const std::vector<Acts::GeometryIdentifier>& selection) {
  m_selection.resize(selection.size());
  std::transform(selection.begin(), selection.end(), m_selection.begin(),
                 [](const Acts::GeometryIdentifier& geoID)
                     -> std::pair<Acts::GeometryIdentifier, Mask> {
                   return std::make_pair(geoID, makeMask(geoID));
                 });
}

GeometryIdSelector::Mask GeometryIdSelector::makeMask(
    const Acts::GeometryIdentifier& id) {
  // construct id from encoded value with all bits set
  Acts::GeometryIdentifier allSet =
      Acts::GeometryIdentifier(~Acts::GeometryIdentifier::Value(0u));
  // manually iterate over identifier levels starting from the lowest
  if (id.sensitive() != 0u) {
    // all levels are valid; keep all bits set.
    return allSet.value();
  }
  if (id.approach() != 0u) {
    return allSet.setSensitive(0u).value();
  }
  if (id.layer() != 0u) {
    return allSet.setSensitive(0u).setApproach(0u).value();
  }
  if (id.boundary() != 0u) {
    return allSet.setSensitive(0u).setApproach(0u).setLayer(0u).value();
  }
  if (id.volume() != 0u) {
    return allSet.setSensitive(0u)
        .setApproach(0u)
        .setLayer(0u)
        .setBoundary(0u)
        .value();
  }
  // no valid levels; all bits are zero.
  return GeometryIdSelector::Mask(0u);
}

bool GeometryIdSelector::check(const Acts::GeometryIdentifier& geoID) const{
  for (const std::pair<Acts::GeometryIdentifier, Mask>& reqGeoID :
       m_selection) {
    // equal within mask
    if ((geoID.value() & reqGeoID.second) ==
        (reqGeoID.first.value() & reqGeoID.second))
      return true;
  }
  return false;
}
