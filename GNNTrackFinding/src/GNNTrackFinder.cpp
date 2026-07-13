/*
 * Copyright (c) 2014-2024 Key4hep-Project.
 *
 * This file is part of Key4hep.
 * See https://key4hep.github.io/key4hep-doc/ for further info.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "GNNTrackFinder.h"

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
  using BoostTrackBuilding     = Acts::BoostTrackBuilding;
  using Device                 = Acts::Device;
  using EdgeClassificationBase = Acts::EdgeClassificationBase;
  using GnnPipeline            = Acts::GnnPipeline;
  using OnnxEdgeClassifier     = Acts::OnnxEdgeClassifier;
}  // namespace ActsPlugins
#endif

#include <k4ActsTracking/ActsGaudiLogger.h>
#include <k4ActsTracking/KFRunner.hxx>
#include <k4ActsTracking/SourceLink.hxx>

#include <k4FWCore/GaudiChecks.h>

#include <Acts/EventData/SourceLink.hpp>
#include <Acts/Geometry/GeometryContext.hpp>
#include <Acts/MagneticField/MagneticFieldContext.hpp>
#include <Acts/MagneticField/MagneticFieldProvider.hpp>
#include <Acts/Utilities/CalibrationContext.hpp>

#include <Math/PositionVector3D.h>

#include <DDSegmentation/BitFieldCoder.h>

#include <fmt/format.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace {
  std::vector<float> extractHitInformation(const edm4hep::TrackerHitPlaneCollection& hits,
                                           const std::vector<std::string>&           features,
                                           const std::string&                        cellIDEncoding) {
    // Could use a std::array here, but that would make switching between 3D and
    // 4D a bit more cumbersome
    std::vector<std::vector<float>> embeddingInputs{};
    embeddingInputs.reserve(hits.size());

    // CellID decoder
    dd4hep::DDSegmentation::BitFieldCoder decoder{cellIDEncoding};

    for (const auto hit : hits) {
      const auto position = ROOT::Math::XYZPointF(hit.getPosition().x, hit.getPosition().y, hit.getPosition().z);

      std::vector<float> hitInfo{};
      hitInfo.reserve(features.size());
      const auto cellID = hit.getCellID();

      for (const auto& f : features) {
        std::string key = f;
        std::transform(key.begin(), key.end(), key.begin(), [](unsigned char c) { return std::tolower(c); });
        try {
          if (key == "x") {
            hitInfo.push_back(position.x());
          } else if (key == "y") {
            hitInfo.push_back(position.y());
          } else if (key == "z") {
            hitInfo.push_back(position.z());
          } else if (key == "r") {
            hitInfo.push_back(position.r());
          } else if (key == "phi") {
            hitInfo.push_back(position.phi());
          } else if (key == "t" || key == "time") {
            hitInfo.push_back(hit.getTime());
          } else if (key == "module_id") {
            hitInfo.push_back(static_cast<float>(decoder.get(cellID, decoder.index("module"))));
          } else if (key == "layer_id") {
            hitInfo.push_back(static_cast<float>(decoder.get(cellID, decoder.index("layer"))));
          } else if (key == "system_id" || key == "volume_id") {
            hitInfo.push_back(static_cast<float>(decoder.get(cellID, decoder.index("system"))));
          } else {
            throw std::runtime_error(fmt::format("Unknown hit feature '{}'", f));
          }
        } catch (const std::exception& ex) {
          throw std::runtime_error(
              fmt::format("Error extracting feature '{}' for hit with CellID {}: {}", f, cellID, ex.what()));
        }
      }
      embeddingInputs.emplace_back(std::move(hitInfo));
    }
    return mlutils::flatten(embeddingInputs);
  }

  /// Parse a device string ("cpu", "cuda", "cuda:<index>") into an Acts Device.
  /// Throws std::invalid_argument on an unrecognised value.
  ActsPlugins::Device parseDevice(std::string spec) {
    std::transform(spec.begin(), spec.end(), spec.begin(), [](unsigned char c) { return std::tolower(c); });

    if (spec == "cpu") {
      return ActsPlugins::Device::Cpu();
    }
    if (spec == "cuda") {
      return ActsPlugins::Device::Cuda();
    }
    if (spec.rfind("cuda:", 0) == 0) {
      return ActsPlugins::Device::Cuda(static_cast<std::size_t>(std::stoul(spec.substr(5))));
    }
    throw std::invalid_argument(fmt::format("Unknown device '{}', expected 'cpu', 'cuda' or 'cuda:<index>'", spec));
  }
}  // namespace

GNNTrackFinder::GNNTrackFinder(const std::string& name, ISvcLocator* svcLoc)
    : Transformer(name, svcLoc, {KeyValues("InputHitCollections", {"populate-me-properly"})},
                  {KeyValues("OutputTrackCandidates", {"GNNTrackCands"})}) {}

StatusCode GNNTrackFinder::initialize() {
  m_logger = makeActsGaudiLogger(this);
  m_monitoringHist.createHistogram(*this);

  m_actsGeoSvc = svcLoc()->service<IActsGeoSvc>("ActsGeoSvc");
  K4_GAUDI_CHECK(m_actsGeoSvc);

  try {
    m_runDevice = parseDevice(m_device.value());
  } catch (const std::invalid_argument& ex) {
    error() << ex.what() << endmsg;
    return StatusCode::FAILURE;
  }
  info() << fmt::format("Running GNN pipeline on device '{}'", m_device.value()) << endmsg;

  // Build the list of all hit features, and separately for the embedding model
  // and the edge classifiers.
  const auto& embeddingFeatures = mlutils::parseList<std::string>(m_inputFeaturesEmbedding.value());
  m_allHitFeatures.insert(m_allHitFeatures.end(), embeddingFeatures.begin(), embeddingFeatures.end());
  const auto& edgeClassifierFeaturesList = mlutils::parseMultiList<std::string>(m_inputFeaturesEdgeClassifier.value());
  for (const auto& edgeClassifierFeatures : edgeClassifierFeaturesList) {
    m_allHitFeatures.insert(m_allHitFeatures.end(), edgeClassifierFeatures.begin(), edgeClassifierFeatures.end());
  }
  // Remove duplicates from m_allHitFeatures preserving initial order
  std::unordered_set<std::string> seenFeatures;
  std::vector<std::string>         uniqueHitFeatures;
  uniqueHitFeatures.reserve(m_allHitFeatures.size());
  for (const auto& feature : m_allHitFeatures) {
    if (seenFeatures.insert(feature).second) {
      uniqueHitFeatures.push_back(feature);
    }
  }
  m_allHitFeatures = std::move(uniqueHitFeatures);
  
  debug() << fmt::format("All hit features: ");
  for (const auto& f : m_allHitFeatures) {
    debug() << fmt::format(" {},", f);
  }
  debug() << endmsg;

  // Translate the lists of input features into lists of indices in the full
  // per-hit feature vector for each model.
  const auto& allFeatures = m_allHitFeatures;
  m_embeddingFeatureIndices.clear();
  m_edgeClassifierFeatureIndices.clear();
  for (const auto& f : embeddingFeatures) {
    auto it = std::find(allFeatures.begin(), allFeatures.end(), f);
    if (it != allFeatures.end()) {
      m_embeddingFeatureIndices.push_back(static_cast<int>(std::distance(allFeatures.begin(), it)));
    }
  }
  for (const auto& edgeClassifierFeatures : edgeClassifierFeaturesList) {
    std::vector<int> featureIndices;
    featureIndices.reserve(edgeClassifierFeatures.size());
    for (const auto& f : edgeClassifierFeatures) {
      auto it = std::find(allFeatures.begin(), allFeatures.end(), f);
      if (it != allFeatures.end()) {
        featureIndices.push_back(static_cast<int>(std::distance(allFeatures.begin(), it)));
      }
    }
    m_edgeClassifierFeatureIndices.push_back(std::move(featureIndices));
  }

  // Check that the embedding dimension matches the number of features selected
  // for the graph construction model (if specified).
  if (m_embeddingDim.value() > 0 && !embeddingFeatures.empty() &&
      (static_cast<std::size_t>(m_embeddingDim.value()) != embeddingFeatures.size() ||
       static_cast<std::size_t>(m_embeddingDim.value()) != m_embeddingFeatureIndices.size())) {
    error() << fmt::format(
                   "Embedding dimension {} does not match the number of selected features {} for the graph "
                   "construction model",
                   m_embeddingDim.value(), embeddingFeatures.size())
            << endmsg;
    return StatusCode::FAILURE;
  }

  auto graphConstructor = std::make_shared<OnnxMetricLearning>(
      OnnxMetricLearning::Config{.modelPath        = m_nodeEmbeddingModelPath.value(),
                                 .selectedFeatures = m_embeddingFeatureIndices,
                                 .featureScales    = mlutils::parseList<float>(m_inputScalesEmbedding.value()),
                                 .embeddingDim     = m_embeddingDim.value(),
                                 .rVal             = m_edgeBuildingRadius.value(),
                                 .knnVal           = m_edgeBuildingKnn.value(),
                                 .device           = m_runDevice},
      m_logger->clone(name() + ".MetricLearning"));

  std::vector<std::shared_ptr<ActsPlugins::EdgeClassificationBase>> edgeClassifiers{};
  for (size_t i = 0; i < m_edgeClassifierModelPath.size(); ++i) {
    edgeClassifiers.push_back(std::make_shared<ActsPlugins::OnnxEdgeClassifier>(
        ActsPlugins::OnnxEdgeClassifier::Config{
            .modelPath        = m_edgeClassifierModelPath[i],
            .selectedFeatures = m_edgeClassifierFeatureIndices[i],
            .featureScales    = mlutils::parseList<float>(m_inputScalesEdgeClassifier[i]),
            .cut              = m_edgeClassifierCut[i],
            // The Acts Config defaults to Device::Cuda(); use the configured
            // device (default "cpu") since the onnxruntime build may not have a
            // CUDA execution provider.
            .device = m_runDevice},
        m_logger->clone(name() + fmt::format(".EdgeClassifier{}", i))));
  }

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

edm4hep::TrackCollection GNNTrackFinder::operator()(
    std::vector<const edm4hep::TrackerHitPlaneCollection*> const& inputTrackerHits) const {
  const auto allHits = [&inputTrackerHits]() {
    edm4hep::TrackerHitPlaneCollection hits{};
    hits.setSubsetCollection(true);

    for (const auto* coll : inputTrackerHits) {
      std::ranges::copy(*coll, std::back_inserter(hits));
    }
    return hits;
  }();
  debug() << fmt::format("Collected {} hits from {} collections", allHits.size(), inputTrackerHits.size()) << endmsg;
  auto embeddingInputs = extractHitInformation(allHits, m_allHitFeatures, m_actsGeoSvc->cellIDEncodingString());
  assert(embeddingInputs.size() == allHits.size() * m_allHitFeatures.size());
  // Give hits their position in the global hits collection as index
  std::vector<int> hitIdcs(allHits.size());
  std::iota(hitIdcs.begin(), hitIdcs.end(), 0);

  const auto trackCandIdcs = m_pipeline->run(embeddingInputs, {}, hitIdcs, m_runDevice);
  debug() << fmt::format("Received {} track candidates", trackCandIdcs.size()) << endmsg;

  // Default-construct ACTS contexts
  const Acts::GeometryContext      geoCtx = Acts::GeometryContext::dangerouslyDefaultConstruct();
  const Acts::MagneticFieldContext magCtx{};
  const Acts::CalibrationContext   calCtx{};

  // Build the ACTS measurements / source links for all GNN input hits, keeping the
  // source link of each hit so each candidate can be turned into a fit seed.
  ACTSTracking::SourceLinkContainer                           sourceLinks;
  ACTSTracking::MeasurementContainer                          measurements;
  ACTSTracking::HitContainer                                  hitContainer;
  std::unordered_map<std::uint64_t, ACTSTracking::SourceLink> slByHit;
  slByHit.reserve(allHits.size());

  ACTSTracking::prepareTrackerHits(
      *this, *m_actsGeoSvc, geoCtx, allHits, measurements, sourceLinks, hitContainer, /*numThreads=*/1,
      [&](const edm4hep::TrackerHitPlane& hit, const ACTSTracking::SourceLink& sl, const Acts::Vector3& /*globalPos*/,
          const Acts::Surface& /*surface*/,
          const Acts::SquareMatrix2& /*localCov*/) { slByHit.emplace(ACTSTracking::trackerHitKey(hit), sl); });

  Acts::MagneticFieldProvider::Cache magCache = m_actsGeoSvc->magneticField()->makeCache(magCtx);

  const ACTSTracking::KFRunner kfRunner(*m_actsGeoSvc, geoCtx, magCtx, calCtx, measurements, hitContainer,
                                        {.propagateBackward = m_propagateBackward});

  edm4hep::TrackCollection trackCands{};
  auto                     histBuffer = m_monitoringHist.buffer();
  for (const auto& candIdcs : trackCandIdcs) {
    ++histBuffer[{allHits.size(), trackCands.size(), candIdcs.size()}];
    if (candIdcs.size() < m_minHitsPerTrk.value()) {
      continue;
    }

    // Gather this candidate's hits and build the radius-ordered seed hits.
    std::vector<edm4hep::TrackerHitPlane> candHits;
    candHits.reserve(candIdcs.size());
    for (const auto idx : candIdcs) {
      candHits.push_back(allHits[idx]);
    }
    const std::vector<ACTSTracking::SeedHit> hits = ACTSTracking::collectSeedHits(candHits, slByHit, hitContainer);

    if (hits.size() < m_minHitsPerTrk.value()) {
      debug() << "Skipping candidate with " << hits.size() << " usable hits." << endmsg;
      continue;
    }

    // The radius-ordered source links (innermost / middle / outermost form the
    // seed) are handed to the Kalman fitter.
    std::vector<Acts::SourceLink> candSourceLinks;
    candSourceLinks.reserve(hits.size());
    for (const ACTSTracking::SeedHit& h : hits) {
      candSourceLinks.emplace_back(h.sl);
    }

    const ACTSTracking::SeedHit& bottom = hits.front();
    const ACTSTracking::SeedHit& middle = hits[hits.size() / 2];
    const ACTSTracking::SeedHit& top    = hits.back();

    const Acts::Surface* bottomSurface = m_actsGeoSvc->trackingGeometry()->findSurface(bottom.sl.geometryId());
    if (bottomSurface == nullptr) {
      warning() << "Surface with geoID " << bottom.sl.geometryId() << " not found in tracking geometry" << endmsg;
      continue;
    }

    std::optional<Acts::BoundTrackParameters> startParams = ACTSTracking::estimateSeedParameters(
        *this, *m_actsGeoSvc, geoCtx, *bottomSurface, bottom.pos, middle.pos, top.pos,
        hitContainer[bottom.sl.index()].getTime(), magCache, m_initialTrackError_pos, m_initialTrackError_phi,
        m_initialTrackError_lambda, m_initialTrackError_relP, m_initialTrackError_time);
    if (!startParams) {
      continue;
    }

    std::optional<edm4hep::MutableTrack> track = kfRunner.fit(*this, candSourceLinks, *startParams, magCache);
    if (!track) {
      continue;
    }
    trackCands.push_back(std::move(*track));
  }
  debug() << fmt::format("Produced {} fitted tracks from {} candidates", trackCands.size(), trackCandIdcs.size())
          << endmsg;
  return trackCands;
}

DECLARE_COMPONENT(GNNTrackFinder)
