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

#include <fmt/format.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

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

  auto graphConstructor =
      std::make_shared<OnnxMetricLearning>(OnnxMetricLearning::Config{.modelPath    = m_nodeEmbeddingModelPath.value(),
                                                                      .embeddingDim = m_embeddingDim.value(),
                                                                      .rVal         = m_edgeBuildingRadius.value(),
                                                                      .knnVal       = m_edgeBuildingKnn.value(),
                                                                      .device       = m_runDevice},
                                           m_logger->clone(name() + ".MetricLearning"));

  std::vector<std::shared_ptr<ActsPlugins::EdgeClassificationBase>> edgeClassifiers{
      std::make_shared<ActsPlugins::OnnxEdgeClassifier>(
          ActsPlugins::OnnxEdgeClassifier::Config{.modelPath = m_edgeClassifierModelPath.value(),
                                                  .cut       = m_edgeClassifierCut.value(),
                                                  // The Acts Config defaults to Device::Cuda(); use the configured
                                                  // device (default "cpu") since the onnxruntime build may not have a
                                                  // CUDA execution provider.
                                                  .device = m_runDevice},
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
  auto embeddingInputs = extractHitInformation(allHits);
  assert(embeddingInputs.size() == allHits.size() * 4);
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
  std::unordered_map<std::uint64_t, ACTSTracking::SourceLink> slByHit;
  slByHit.reserve(allHits.size());

  ACTSTracking::prepareTrackerHits(
      *this, *m_actsGeoSvc, geoCtx, allHits, measurements, sourceLinks, /*numThreads=*/1,
      [&](const edm4hep::TrackerHitPlane& hit, const ACTSTracking::SourceLink& sl, const Acts::Vector3& /*globalPos*/,
          const Acts::Surface& /*surface*/,
          const Acts::SquareMatrix2& /*localCov*/) { slByHit.emplace(ACTSTracking::trackerHitKey(hit), sl); });

  Acts::MagneticFieldProvider::Cache magCache = m_actsGeoSvc->magneticField()->makeCache(magCtx);

  const ACTSTracking::KFRunner kfRunner(*m_actsGeoSvc, geoCtx, magCtx, calCtx, measurements,
                                        {.propagateBackward = m_propagateBackward});

  // Per-hit seed info: global position + transverse radius, resolved once per
  // usable hit so the seed triplet can be ordered by radius.
  struct SeedHit {
    Acts::Vector3            pos;
    double                   r;
    ACTSTracking::SourceLink sl;
  };

  edm4hep::TrackCollection trackCands{};
  auto                     histBuffer = m_monitoringHist.buffer();
  for (const auto& candIdcs : trackCandIdcs) {
    ++histBuffer[{allHits.size(), trackCands.size(), candIdcs.size()}];
    if (candIdcs.size() < m_minHitsPerTrk.value()) {
      continue;
    }

    // Collect the source links + seed positions for this candidate's hits.
    std::vector<SeedHit>          hits;
    std::vector<Acts::SourceLink> candSourceLinks;
    hits.reserve(candIdcs.size());
    candSourceLinks.reserve(candIdcs.size());
    for (const auto idx : candIdcs) {
      auto it = slByHit.find(ACTSTracking::trackerHitKey(allHits[idx]));
      if (it == slByHit.end()) {
        continue;
      }
      const edm4hep::Vector3d p = it->second.edm4hepHit().getPosition();
      hits.push_back({Acts::Vector3(p.x, p.y, p.z), std::hypot(p.x, p.y), it->second});
    }

    if (hits.size() < m_minHitsPerTrk.value()) {
      debug() << "Skipping candidate with " << hits.size() << " usable hits." << endmsg;
      continue;
    }

    // Order hits by radius: innermost / middle / outermost form the seed, and
    // the (radius-ordered) source links are handed to the Kalman fitter.
    std::sort(hits.begin(), hits.end(), [](const SeedHit& a, const SeedHit& b) { return a.r < b.r; });
    for (const SeedHit& h : hits) {
      candSourceLinks.emplace_back(h.sl);
    }

    const SeedHit& bottom = hits.front();
    const SeedHit& middle = hits[hits.size() / 2];
    const SeedHit& top    = hits.back();

    const Acts::Surface* bottomSurface = m_actsGeoSvc->trackingGeometry()->findSurface(bottom.sl.geometryId());
    if (bottomSurface == nullptr) {
      warning() << "Surface with geoID " << bottom.sl.geometryId() << " not found in tracking geometry" << endmsg;
      continue;
    }

    std::optional<Acts::BoundTrackParameters> startParams = ACTSTracking::estimateSeedParameters(
        *this, *m_actsGeoSvc, geoCtx, *bottomSurface, bottom.pos, middle.pos, top.pos, bottom.sl.edm4hepHit().getTime(),
        magCache, m_initialTrackError_pos, m_initialTrackError_phi, m_initialTrackError_lambda,
        m_initialTrackError_relP, m_initialTrackError_time);
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
