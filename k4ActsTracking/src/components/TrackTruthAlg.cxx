#include "k4ActsTracking/TrackTruthAlg.hxx"

// ACTSTracking
#include "k4ActsTracking/Helpers.hxx"

// edm4hep
#include <edm4hep/MCParticle.h>
#include <edm4hep/SimTrackerHit.h>
#include <edm4hep/Track.h>
#include <edm4hep/TrackerHitPlane.h>
#include <edm4hep/TrackerHitSimTrackerHitLink.h>

//------------------------------------------------------------------------------------------------

DECLARE_COMPONENT(TrackTruthAlg)

TrackTruthAlg::TrackTruthAlg(const std::string& name, ISvcLocator* svcLoc) : MultiTransformer(name, svcLoc, {
		KeyValues("InputTrackCollectionName", {"Tracks"}),
		KeyValues("InputTrackerHit2SimTrackerHitRelationName", {"TrackMCRelation"}) },
		{ KeyValues("OutputParticle2TrackRelationName", {"Particle2TrackRelationName"}) })	{}

std::tuple<edm4hep::TrackMCParticleLinkCollection> TrackTruthAlg::operator()(
			const edm4hep::TrackCollection& tracks,
                        const edm4hep::TrackerHitSimTrackerHitLinkCollection& trackerHitRelations) const{
	MsgStream log(msgSvc(), name());
	log << MSG::DEBUG << trackerHitRelations.size() << endmsg;

	// Map TrackerHits to SimTrackerHits
	std::map<edm4hep::TrackerHit, edm4hep::SimTrackerHit> trackerHit2SimHit;
	for (const auto& hitRel : trackerHitRelations) {
		edm4hep::TrackerHit trackerHit = hitRel.getFrom();
		edm4hep::SimTrackerHit simTrackerHit = hitRel.getTo();
		log << MSG::DEBUG <<"Hit:\n"<< trackerHit <<endmsg;
		log<< MSG::DEBUG << "Sim:\n"<<simTrackerHit<<endmsg;
		trackerHit2SimHit[trackerHit] = simTrackerHit;
	}

	log << MSG::DEBUG << "Map size: " << trackerHit2SimHit.size() << endmsg;
	// Map best matches MCP to Track
	std::map<edm4hep::MCParticle, edm4hep::Track> mcBestMatchTrack;
	std::map<edm4hep::MCParticle, float> mcBestMatchFrac;

	for (const auto& track: tracks) {
		//Get Track
		std::map<edm4hep::MCParticle, uint32_t> trackHit2Mc;
		for (auto& hit : track.getTrackerHits()) {
			//Search for SimHit
			const edm4hep::SimTrackerHit* simHit = nullptr;
			log << MSG::DEBUG << "hit:\n" << hit <<endmsg;
			/// @TODO: I am not happy with this. Again an edm4hep problem
			for (const auto& pair : trackerHit2SimHit) {
				log << MSG::DEBUG << "sim:\n" << pair.first<<endmsg;
				if (pair.first == hit) {
					simHit = (&pair.second);
					break;
				}
			}
			if (simHit && simHit->getParticle().isAvailable()) {
				trackHit2Mc[simHit->getParticle()]++; //Increment MC Particle counter
			}
		}

		// Update Best Matches
		for (const auto& [mcParticle, hitCount] : trackHit2Mc) {
			float frac = static_cast<float>(hitCount) / track.trackerHits_size();
			bool better = mcBestMatchTrack.count(mcParticle) == 0 || // no best matches exist
				      mcBestMatchFrac[mcParticle] < frac; // this match is better (more hits on track)
			if (better) {
				mcBestMatchTrack[mcParticle] = track;
				mcBestMatchFrac[mcParticle] = frac;
			}
		}
	}

	// Save the best matches
	edm4hep::TrackMCParticleLinkCollection outColMC2T;
	for (const auto& [mcParticle, track] : mcBestMatchTrack) {
		edm4hep::MutableTrackMCParticleLink link = outColMC2T.create();
		link.setFrom(track);
		link.setTo(mcParticle);
		link.setWeight(mcBestMatchFrac[mcParticle]);
	}

	return std::make_tuple(std::move(outColMC2T));
}
