#ifndef TrackTruthAlg_h
#define TrackTruthAlg_h 1

// edm4hep
#include <edm4hep/MCParticleCollection.h>
#include <edm4hep/TrackCollection.h>
#include <edm4hep/TrackMCParticleLinkCollection.h>
#include <edm4hep/TrackerHitSimTrackerHitLinkCollection.h>

// k4FWCore
#include <k4FWCore/DataHandle.h>
#include <k4FWCore/Transformer.h>

// Standard
#include <string>
#include <tuple>
#include <vector>

/**
 * Helper processor that creates Relation collections for track to hit
 * associations to be used with TrackPerf.
 *
 * @param  TrackCollection                Track input collections
 * @param  Track2HitRelationName          output collection for track to
 * hit relations
 *
 * @author Samuel Ferraro, Unknown
 */

struct TrackTruthAlg final
    : k4FWCore::MultiTransformer<std::tuple<edm4hep::TrackMCParticleLinkCollection>(
          const edm4hep::TrackCollection&, const edm4hep::TrackerHitSimTrackerHitLinkCollection&)> {
public:
  /**
         * @brief Constructor for TrackTruthAlg
         * @param name unique string identifier for this instance
         * @param svcLoc a Service Locator passed by the Gaudi AlgManager
         */
  TrackTruthAlg(const std::string& name, ISvcLocator* svcLoc);

  /**
         * @brief TrackTruthAlg operation. The workhorse of this MultiTransformer.
         * @param track A collection of reconstructed, deduped, filtered tracks 
	 * @param trackerHitRelations A merged collection of associations between tracker hits and sim tracker hits
         * @return An association collection connecting Tracks to MCParticles
         */
  std::tuple<edm4hep::TrackMCParticleLinkCollection> operator()(
      const edm4hep::TrackCollection&                       tracks,
      const edm4hep::TrackerHitSimTrackerHitLinkCollection& trackerHitRelations) const;
};

#endif
