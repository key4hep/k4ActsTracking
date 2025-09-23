#include "k4ActsTracking/ACTSDuplicateRemoval.hxx"

// edm4hep
#include <edm4hep/TrackerHit.h>
#include <edm4hep/MutableTrack.h>

// Standard
#include <algorithm>

// ACTSTracking
#include "k4ActsTracking/Helpers.hxx"

namespace ACTSTracking {
/**
 * @brief Return true if `trk1` and `trk2` share at least 50% of hits.
 * @param trk1 One of two tracks to compare
 * @param tkr2 One of two tracks to compare
 * @return True if they share too many of the same Tracker Hits
 */
inline bool tracks_equal(const edm4hep::Track& trk1, const edm4hep::Track& trk2) {
	// Get an iterator for the Track hits of the first Track
	uint32_t hitOlap = 0;
	// Loop through each track hit and see if it overlaps with the second track
	for (size_t itrackHit = 0; itrackHit < trk1.trackerHits_size(); ++itrackHit) {
		auto it = std::find_if(trk2.trackerHits_begin(), trk2.trackerHits_end(), [&](const auto& hit2){
			return hitEqual(trk1.getTrackerHits(itrackHit), hit2);
		});
		if (it != trk2.trackerHits_end()) {
			hitOlap++; // If it does overlap, increment hitOlap
		}
	}

	// Smaller track count
	uint32_t size = std::min(trk1.trackerHits_size(), trk2.trackerHits_size());
	return 2 * hitOlap > size;  // half of smaller track belong to larger track
}

/**
 * @brief Workaround function to get around comparision issue
 * @TODO: This is to get around an issue in edm4hep
 * @param hit1 A hit
 * @param hits A hit
 */
bool hitEqual(const edm4hep::TrackerHit& hit1, const edm4hep::TrackerHit& hit2) {
	return		hit1.getCellID() == hit2.getCellID() &&
		     	hit1.getType() == hit2.getType() &&
		     	hit1.getQuality() == hit2.getQuality() &&
		     	hit1.getTime() == hit2.getTime() &&
			hit1.getEDep() == hit2.getEDep() &&
			hit1.getEDepError() == hit2.getEDepError() &&
			hit1.getPosition() == hit2.getPosition();
}

/**
 * @brief Determine which of two tracks is of better quality
 * @details Quality is determined first by number of hits, then by chi2
 * @param trk1 One of two tracks to compare
 * @param trk2 One of two tracks to compare
 * @return True if the first track is of better quality
 */
bool track_quality_compare(const edm4hep::Track& trk1, const edm4hep::Track& trk2) {
	// If number of hits are different, then the one with more
	// hits should be chosen first
	if (trk1.trackerHits_size() != trk2.trackerHits_size())
		return trk1.trackerHits_size() > trk2.trackerHits_size();

	// Same number of hits means I want smaller chi2
	return trk1.getChi2() < trk2.getChi2();
}

/**
 * @brief Compares the tan(lambda) of two tracks. For ordering
 * @param trk1 One of two tracks to compare
 * @param trk2 One of two tracks to compare
 * @return True if the first track has lower tan(lambda) in the first track state
 */
inline bool track_duplicate_compare(const edm4hep::Track& trk1, const edm4hep::Track& trk2) {
	return trk1.getTrackStates(edm4hep::TrackState::AtIP).tanLambda < trk2.getTrackStates(edm4hep::TrackState::AtIP).tanLambda;
}
}  // namespace ACTSTracking

DECLARE_COMPONENT(ACTSDuplicateRemoval)

ACTSDuplicateRemoval::ACTSDuplicateRemoval(const std::string& name, ISvcLocator* svcLoc) : Transformer(name, svcLoc,
		KeyValues("InputTrackCollectionName", {"TruthTracks"}),
		KeyValues("OutputTrackCollectionName", {"DedupedTruthTracks"})) {}

edm4hep::TrackCollection ACTSDuplicateRemoval::operator()(const edm4hep::TrackCollection& trackCollection) const{
	MsgStream log(msgSvc(), name());

	// Make output collection
	edm4hep::TrackCollection outputTracks;
	outputTracks.setSubsetCollection();

	// Insertion sort input tracks
	std::vector<edm4hep::Track> sortedInput;
	for (const auto& track : trackCollection) {
		auto insertion_point = std::upper_bound(sortedInput.begin(), sortedInput.end(), track, ACTSTracking::track_duplicate_compare);
		sortedInput.insert(insertion_point, track);
	}

	int total = 0;
	int dupes = 0;
	int added = 0;
	// Loop through all inputs and search for nearby equals
	// Remove if they are too similar
	std::vector<edm4hep::Track> finalTracks;
	for (const edm4hep::Track& track : sortedInput) {
		total++;
		bool foundAnEqual = false;
		int startIdx = (finalTracks.size() >= 10) ? finalTracks.size() - 10 : 0;
		for (int i = startIdx; i < finalTracks.size(); ++i) {
			const edm4hep::Track& otherTrack = finalTracks[i];

			if (!ACTSTracking::tracks_equal(track, otherTrack)) continue;
			foundAnEqual = true;
			dupes++;
			if (ACTSTracking::track_quality_compare(track, otherTrack)) {
				finalTracks[i] = track;
				break;
			}
		}
		if (!foundAnEqual) {
			added++;
			finalTracks.push_back(track);
		}
	}

	for (const auto track : finalTracks) {
		outputTracks.push_back(track);
	}

	return outputTracks;
}
