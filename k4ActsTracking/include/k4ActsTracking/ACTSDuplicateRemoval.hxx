#ifndef ACTSDuplicateRemoval_h
#define ACTSDuplicateRemoval_h 1

// edm4hep
#include <edm4hep/Track.h>
#include <edm4hep/TrackCollection.h>

// Gaudi
#include <k4FWCore/Transformer.h>

// k4FWCore
#include <k4FWCore/DataHandle.h>

//! \brief Remove track duplicates
/**
 * If tracks share more than 50% of hits, then
 * remove the best one.
 *
 * @author Karol Krizka
 * @author Samuel Ferraro
 * @version $Id$
 */
struct ACTSDuplicateRemoval final : k4FWCore::Transformer<edm4hep::TrackCollection(const edm4hep::TrackCollection&)> {
public:
  /**
         * @brief Constructor for ACTSDuplicateRemoval
         * @param name unique string identifier for this instance
         * @param svcLoc a Service Locator passed by the Gaudi AlgManager
         */
  ACTSDuplicateRemoval(const std::string& name, ISvcLocator* svcLoc);

  /**
         * @brief ACTSDuplicateRemoval operation. The workhorse of this Transformer.
         * @param trackCollection A collection of reconstructed tracks with possible duplicates
         * @return A Track Collection with duplicates removed
         */
  edm4hep::TrackCollection operator()(const edm4hep::TrackCollection& trackCollection) const override;
};

namespace ACTSTracking {
  //! Compare tracks by quality, best first
  /**
 * Decides which track is better using the following algorithm
 *  1. Track with higher number of hits is better
 *  2. Track with smaller chi2 is better
 *
 * 2. is only run with 1. is ambigious.
 */
  bool track_duplicate_compare(const edm4hep::Track& trk1, const edm4hep::Track& trk2);

  /**
 	 * @TODO Remove this when comparison in edm4hep gets better.
 	 */
  bool hitEqual(const edm4hep::TrackerHit& hit1, const edm4hep::TrackerHit& hit2);
}  // namespace ACTSTracking

#endif
