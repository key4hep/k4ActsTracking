#include "ExaTrkGNNTrackFinder.h"

ExaTrkGNNTrackFinder::ExaTrkGNNTrackFinder(const std::string& name, ISvcLocator* svcLoc)
    : Transformer(name, svcLoc, {KeyValues("InputHitCollections", {"populate-me-properly"})},
                  {KeyValues("OutputTrackCandidates", {"ExaTrkGNNTrackCands"})}),
      m_edgeClassifier("EdgeClassifier"), m_nodeEmbedding("NodeEmbedding") {}

StatusCode ExaTrkGNNTrackFinder::initialize() {
  // Load the graph construction metric model
  if (!m_nodeEmbedding.loadModel(m_nodeEmbeddingModelPath.value())) {
    error() << "Failed to load node embedding / graph construction metric model from: "
            << m_nodeEmbeddingModelPath.value() << endmsg;
    return StatusCode::FAILURE;
  }

  // Load the edge classifier model
  if (!m_edgeClassifier.loadModel(m_edgeClassifierModelPath.value())) {
    error() << "Failed to load edge classifier model from: " << m_edgeClassifierModelPath.value() << endmsg;
    return StatusCode::FAILURE;
  }

  info() << "Successfully loaded both ONNX models" << endmsg;
  return StatusCode::SUCCESS;
}

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
