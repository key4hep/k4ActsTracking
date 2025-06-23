#include "ExaTrkGNNTrackFinder.h"

ExaTrkGNNTrackFinder::ExaTrkGNNTrackFinder(const std::string& name, ISvcLocator* svcLoc)
    : Transformer(name, svcLoc, {KeyValues("InputHitCollections", {"populate-me-properly"})},
                  {KeyValues("OutputTrackCandidates", {"ExaTrkGNNTrackCands"})}) {}

edm4hep::TrackCollection
ExaTrkGNNTrackFinder::operator()(std::vector<const edm4hep::TrackerHitPlaneCollection*> const& inputTrackerHits) const {
  edm4hep::TrackCollection candidates;

  // TODO: actually meaningful implementation
  for (const auto& trackerHits : inputTrackerHits) {
    auto cand = candidates.create();
    for (const auto& hit : *trackerHits) {
      cand.addToTrackerHits(hit);
    }
  }

  return candidates;
}

DECLARE_COMPONENT(ExaTrkGNNTrackFinder)
