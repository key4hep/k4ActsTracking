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
  m_nodeEmbedding.dumpModel(debug());
  debug() << endmsg;

  // Load the edge classifier model
  if (!m_edgeClassifier.loadModel(m_edgeClassifierModelPath.value())) {
    error() << "Failed to load edge classifier model from: " << m_edgeClassifierModelPath.value() << endmsg;
    return StatusCode::FAILURE;
  }
  m_edgeClassifier.dumpModel(debug());
  debug() << endmsg;

  info() << "Successfully loaded both ONNX models" << endmsg;
  return StatusCode::SUCCESS;
}

edm4hep::TrackCollection
ExaTrkGNNTrackFinder::operator()(std::vector<const edm4hep::TrackerHitPlaneCollection*> const& inputTrackerHits) const {
  // Could use a std::array here, but that would make switching between 3D and
  // 4D a bit more cumbersome
  std::vector<std::vector<float>> embeddingInputs{};
  auto sizes = inputTrackerHits | std::views::transform([](const auto* c) { return c->size(); });
  auto totalHits = std::accumulate(sizes.begin(), sizes.end(), 0);
  embeddingInputs.reserve(totalHits);

  for (const auto* coll : inputTrackerHits) {
    for (const auto hit : *coll) {
      std::vector<float> hitInfo = {static_cast<float>(hit.getPosition().x), static_cast<float>(hit.getPosition().y),
                                    static_cast<float>(hit.getPosition().z), hit.getTime()};
      embeddingInputs.emplace_back(std::move(hitInfo));
    }
  }

  edm4hep::TrackCollection candidates;

  return candidates;
}

DECLARE_COMPONENT(ExaTrkGNNTrackFinder)
