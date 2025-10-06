#include "ExaTrkGNNTrackFinder.h"

#include "OnnxMetricLearning.h"

#if __has_include("ActsPlugins/Gnn/Stages.hpp")
#include <ActsPlugins/Gnn/BoostTrackBuilding.hpp>
#include <ActsPlugins/Gnn/GnnPipeline.hpp>
#include <ActsPlugins/Gnn/OnnxEdgeClassifier.hpp>
#include <ActsPlugins/Gnn/Stages.hpp>
#else
#include <Acts/Plugins/Gnn/BoostTrackBuilding.hpp>
#include <Acts/Plugins/Gnn/GnnPipeline.hpp>
#include <Acts/Plugins/Gnn/OnnxEdgeClassifier.hpp>
#include <Acts/Plugins/Gnn/Stages.hpp>
namespace ActsPlugins {
using BoostTrackBuilding = Acts::BoostTrackBuilding;
using Device = Acts::Device;
using EdgeClassificationBase = Acts::EdgeClassificationBase;
using GnnPipeline = Acts::GnnPipeline;
using OnnxEdgeClassifier = Acts::OnnxEdgeClassifier;
} // namespace ActsPlugins
#endif

#include <k4ActsTracking/ActsGaudiLogger.h>

#include <Math/PositionVector3D.h>

#include <fmt/format.h>

#include <algorithm>

namespace {
std::vector<float> extractHitInformation(const edm4hep::TrackerHitPlaneCollection& hits) {
  // Could use a std::array here, but that would make switching between 3D and
  // 4D a bit more cumbersome
  std::vector<std::vector<float>> embeddingInputs{};
  embeddingInputs.reserve(hits.size());

  for (const auto hit : hits) {
    const auto position = ROOT::Math::XYZPointF(hit.getPosition().x, hit.getPosition().y, hit.getPosition().z);

    std::vector<float> hitInfo = {position.r(), position.phi(), position.z(), hit.getTime()};
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
  m_monitoringHist.createHistogram(*this);

  auto graphConstructor =
      std::make_shared<OnnxMetricLearning>(OnnxMetricLearning::Config{.modelPath = m_nodeEmbeddingModelPath.value(),
                                                                      .embeddingDim = m_embeddingDim.value(),
                                                                      .rVal = m_edgeBuildingRadius.value(),
                                                                      .knnVal = m_edgeBuildingKnn.value()},
                                           m_logger->clone(name() + ".MetricLearning"));

  std::vector<std::shared_ptr<ActsPlugins::EdgeClassificationBase>> edgeClassifiers{
      std::make_shared<ActsPlugins::OnnxEdgeClassifier>(
          ActsPlugins::OnnxEdgeClassifier::Config{.modelPath = m_edgeClassifierModelPath.value(),
                                                  .cut = m_edgeClassifierCut.value()},
          m_logger->clone(name() + ".EdgeClassifier"))};

  auto trackBuilder = std::make_shared<ActsPlugins::BoostTrackBuilding>(ActsPlugins::BoostTrackBuilding::Config{},
                                                                        m_logger->clone(name() + ".TrackBuilder"));

  try {
    m_pipeline = std::make_unique<ActsPlugins::GnnPipeline>(graphConstructor, edgeClassifiers, trackBuilder,
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
  assert(embeddingInputs.size() == allHits.size() * 4);
  // Give hits their position in the global hits collection as index
  std::vector<int> hitIdcs(allHits.size());
  std::iota(hitIdcs.begin(), hitIdcs.end(), 0);

  const auto trackCandIdcs =
      m_pipeline->run(embeddingInputs, {}, hitIdcs, ActsPlugins::Device{ActsPlugins::Device::Type::eCPU, 0});
  debug() << fmt::format("Received {} track candidates", trackCandIdcs.size()) << endmsg;

  edm4hep::TrackCollection trackCands{};
  auto histBuffer = m_monitoringHist.buffer();
  for (const auto& candIdcs : trackCandIdcs) {
    ++histBuffer[{allHits.size(), trackCands.size(), candIdcs.size()}];
    if (candIdcs.size() < m_minHitsPerTrk.value()) {
      continue;
    }
    auto track = trackCands.create();
    for (const auto idx : candIdcs) {
      track.addToTrackerHits(allHits[idx]);
    }
    // In order to be able to run the (Marlin based) refitting downstream we
    // need a single trackstate. For now we just fudge that by adding it here
    // with the minimal information that is necessary. Once we have a proper
    // pipeline that can start fitting from the hits alone, this can be removed
    // again.
    auto trackState = edm4hep::TrackState{};
    trackState.location = edm4hep::TrackState::AtFirstHit;
    track.addToTrackStates(trackState);
  }
  debug() << fmt::format("Produced {} output track candidates", trackCands.size()) << endmsg;
  return trackCands;
}

DECLARE_COMPONENT(ExaTrkGNNTrackFinder)
