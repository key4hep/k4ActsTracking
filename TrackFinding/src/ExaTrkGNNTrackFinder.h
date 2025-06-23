#include <k4FWCore/Transformer.h>

#include <edm4hep/TrackCollection.h>
#include <edm4hep/TrackerHitPlaneCollection.h>

#include <vector>

struct ExaTrkGNNTrackFinder : public k4FWCore::Transformer<edm4hep::TrackCollection(
                                  std::vector<const edm4hep::TrackerHitPlaneCollection*> const&)> {

  ExaTrkGNNTrackFinder(const std::string& name, ISvcLocator* svcLoc);

  edm4hep::TrackCollection operator()(std::vector<const edm4hep::TrackerHitPlaneCollection*> const&) const override;

private:
};
