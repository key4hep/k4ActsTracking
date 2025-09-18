#include "ExaTrkGNNTrackFinder.h"

#include "OnnxMetricLearning.h"

#include <Acts/Plugins/Gnn/BoostTrackBuilding.hpp>
#include <Acts/Plugins/Gnn/OnnxEdgeClassifier.hpp>
#include <Acts/Plugins/Gnn/Stages.hpp>

#include <k4ActsTracking/ActsGaudiLogger.h>

#include <fmt/format.h>

#include <algorithm>

namespace {
std::vector<float> extractHitInformation(const edm4hep::TrackerHitPlaneCollection& hits) {
  // Could use a std::array here, but that would make switching between 3D and
  // 4D a bit more cumbersome
  std::vector<std::vector<float>> embeddingInputs{};
  embeddingInputs.reserve(hits.size());

  for (const auto hit : hits) {
    std::vector<float> hitInfo = {static_cast<float>(hit.getPosition().x), static_cast<float>(hit.getPosition().y),
                                  static_cast<float>(hit.getPosition().z), hit.getTime()};
    embeddingInputs.emplace_back(std::move(hitInfo));
  }
  return mlutils::flatten(embeddingInputs);
}
} // namespace

ExaTrkGNNTrackFinder::ExaTrkGNNTrackFinder(const std::string& name, ISvcLocator* svcLoc)
    : Transformer(name, svcLoc, {KeyValues("InputHitCollections", {"populate-me-properly"})},
                  {KeyValues("OutputTrackCandidates", {"ExaTrkGNNTrackCands"})}) {}

StatusCode ExaTrkGNNTrackFinder::initialize() {
  m_logger = makeActsGaudiLogger(this);

  auto graphConstructor =
      std::make_shared<OnnxMetricLearning>(OnnxMetricLearning::Config{.modelPath = m_nodeEmbeddingModelPath.value(),
                                                                      .rVal = m_edgeBuildingRadius.value(),
                                                                      .knnVal = m_edgeBuildingKnn.value()},
                                           m_logger->clone(name() + ".MetricLearning"));
  auto edgeClassifier = std::make_shared<Acts::OnnxEdgeClassifier>(
      Acts::OnnxEdgeClassifier::Config{.modelPath = m_edgeClassifierModelPath.value()},
      m_logger->clone(name() + ".EdgeClassifier"));
  auto trackBuilder = std::make_shared<Acts::BoostTrackBuilding>(Acts::BoostTrackBuilding::Config{},
                                                                 m_logger->clone(name() + ".TrackBuilder"));

  try {
    m_pipeline = std::make_unique<Acts::GnnPipeline>(
        graphConstructor, std::vector<std::shared_ptr<Acts::EdgeClassificationBase>>{edgeClassifier}, trackBuilder,
        m_logger->clone(name() + ".Pipeline"));
  } catch (const std::invalid_argument& ex) {
    error() << "Failed to construct GNN Pipeline: " << ex.what() << endmsg;
    return StatusCode::FAILURE;
  }

  return StatusCode::SUCCESS;
}

edm4hep::TrackCollection
ExaTrkGNNTrackFinder::operator()(std::vector<const edm4hep::TrackerHitPlaneCollection*> const& inputTrackerHits) const {
  const auto allHits = [&inputTrackerHits]() {
    edm4hep::TrackerHitPlaneCollection hits{};
    hits.setSubsetCollection(true);

    for (const auto* coll : inputTrackerHits) {
      std::ranges::copy(*coll, std::back_inserter(hits));
    }
    return hits;
  }();
  debug() << fmt::format("Collected {} hits from {} collections", allHits.size(), inputTrackerHits.size()) << endmsg;
  auto embeddingInputs = extractHitInformation(allHits);
  debug() << fmt::format("Resulting in {} total inputs", embeddingInputs.size()) << endmsg;
  // Give hits their position in the global hits collection as index
  std::vector<int> hitIdcs(allHits.size());
  std::iota(hitIdcs.begin(), hitIdcs.end(), 0);

  const auto trackCandIdcs = m_pipeline->run(embeddingInputs, {}, hitIdcs, Acts::Device{Acts::Device::Type::eCPU, 0});

  edm4hep::TrackCollection trackCands{};
  for (const auto& candIdcs : trackCandIdcs) {
    auto track = trackCands.create();
    for (const auto idx : candIdcs) {
      track.addToTrackerHits(allHits[idx]);
    }
  }
  return trackCands;
}

DECLARE_COMPONENT(ExaTrkGNNTrackFinder)
