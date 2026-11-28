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
#include "OnnxSeededMetricLearning.h"
#include "EdgeDirection.h"
#include "ONNXInferenceModel.h"
#include "OnnxMetricLearning.h"
#include "PipelineTensorUtils.h"

#include <fmt/format.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <memory>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace {
/// Hits per seed, the second dimension of the seed_hits input
constexpr std::size_t kHitsPerSeed = 3;

/// Copy the float values of an output tensor
std::vector<float> readFloatOutput(const Ort::Value& value) {
  const float* data = value.GetTensorData<float>();
  const std::size_t size = value.GetTensorTypeAndShapeInfo().GetElementCount();
  return {data, data + size};
}

/// Hit pairs (first < second) connected inside the ball of each selected seed.
/// The ball of a seed holds its knn nearest hits in embedding space that lie
/// within rVal of the seed, and every pair of hits in a ball is connected.
/// Returns the pairs sorted and without duplicates.
/// This is brute force over all hits per seed, which is probably fast enough
/// because it only loops over the selected condensation points, not all hit pairs.
std::vector<std::pair<int64_t, int64_t>>
buildSeedBallEdges(const std::vector<float>& hitEmbedding, const std::vector<float>& seedEmbedding,
                   std::size_t embeddingDim, const std::vector<int64_t>& selectedSeeds, float rVal, std::size_t knn) {
  const std::size_t numHits = hitEmbedding.size() / embeddingDim;
  const float rVal2 = rVal * rVal;
  std::vector<std::pair<int64_t, int64_t>> edges{};
  // Squared embedding distance to the current seed and the hit index
  std::vector<std::pair<float, int64_t>> distances(numHits);

  for (const int64_t seed : selectedSeeds) {
    const float* seedVector = &seedEmbedding[static_cast<std::size_t>(seed) * embeddingDim];
    for (std::size_t hit = 0; hit < numHits; ++hit) {
      const float* hitVector = &hitEmbedding[hit * embeddingDim];
      float distance2 = 0.f;
      for (std::size_t d = 0; d < embeddingDim; ++d) {
        const float diff = seedVector[d] - hitVector[d];
        distance2 += diff * diff;
      }
      distances[hit] = {distance2, static_cast<int64_t>(hit)};
    }

    // The knn nearest hits come first, sorted by distance
    const std::size_t numNearest = std::min(knn, numHits);
    std::partial_sort(distances.begin(), distances.begin() + static_cast<std::ptrdiff_t>(numNearest), distances.end());
    std::size_t ballSize = 0;
    while (ballSize < numNearest && distances[ballSize].first <= rVal2) {
      ++ballSize;
    }

    for (std::size_t i = 0; i < ballSize; ++i) {
      for (std::size_t j = i + 1; j < ballSize; ++j) {
        const int64_t a = distances[i].second;
        const int64_t b = distances[j].second;
        edges.emplace_back(std::min(a, b), std::max(a, b));
      }
    }
  }

  std::sort(edges.begin(), edges.end());
  edges.erase(std::unique(edges.begin(), edges.end()), edges.end());
  return edges;
}

/// Convert the Acts log level into an ONNX one
constexpr OrtLoggingLevel getOnnxLogLevel(Acts::Logging::Level lvl) {
  switch (lvl) {
  case Acts::Logging::VERBOSE:
    return ORT_LOGGING_LEVEL_VERBOSE;
  case Acts::Logging::DEBUG:
    return ORT_LOGGING_LEVEL_INFO;
  case Acts::Logging::INFO:
  case Acts::Logging::WARNING:
  case Acts::Logging::MAX:
    return ORT_LOGGING_LEVEL_WARNING;
  case Acts::Logging::ERROR:
    return ORT_LOGGING_LEVEL_ERROR;
  case Acts::Logging::FATAL:
    return ORT_LOGGING_LEVEL_FATAL;
  }
  return ORT_LOGGING_LEVEL_WARNING;
}
} // namespace

OnnxSeededMetricLearning::OnnxSeededMetricLearning(const Config& cfg, std::unique_ptr<const Acts::Logger> lggr)
    : m_model("SeededMetricLearning", getOnnxLogLevel(lggr->level()), cfg.device.isCuda(), cfg.device.index),
      m_config(cfg), m_logger(std::move(lggr)) {
  ACTS_INFO(fmt::format("Loading seeded model from {}", config().modelPath));
  m_model.loadModel(config().modelPath);

  if (m_model.numInputs() != kNumModelInputs || m_model.numOutputs() != kNumModelOutputs) {
    throw std::invalid_argument(fmt::format(
        "Model '{}' has {} inputs and {} outputs, expected {} and {} from the seeded object condensation export",
        config().modelPath, m_model.numInputs(), m_model.numOutputs(), kNumModelInputs, kNumModelOutputs));
  }

  if (!config().edgeFeatureIndices.empty()) {
    if (config().edgeFeatureIndices.size() != OnnxMetricLearning::kNumEdgeFeatureInputs) {
      throw std::invalid_argument(
          fmt::format("The edge features need exactly {} node features (r, phi, z, eta), but {} are configured",
                      OnnxMetricLearning::kNumEdgeFeatureInputs, config().edgeFeatureIndices.size()));
    }
    if (!config().edgeFeatureScales.empty() &&
        config().edgeFeatureScales.size() != OnnxMetricLearning::kNumEdgeFeatureInputs) {
      throw std::invalid_argument(fmt::format("edgeFeatureScales has {} entries, expected {}",
                                              config().edgeFeatureScales.size(),
                                              OnnxMetricLearning::kNumEdgeFeatureInputs));
    }
  }
  if (!config().distanceFeatureIndices.empty() &&
      config().distanceFeatureIndices.size() != gnntracking::kNumDistanceFeatures) {
    throw std::invalid_argument(
        fmt::format("The edge orientation needs exactly {} node features (r, z), but {} are configured",
                    gnntracking::kNumDistanceFeatures, config().distanceFeatureIndices.size()));
  }

  // The node dimension of the first input, -1 if the model takes any number of nodes
  if (!m_model.inputShape(0).empty()) {
    m_inputLength = m_model.inputShape(0).front();
  }
  ACTS_INFO(fmt::format("Seeded model expects {} input nodes", m_inputLength));
}

ActsPlugins::PipelineTensors OnnxSeededMetricLearning::operator()(std::vector<float>& inputValues, std::size_t numNodes,
                                                                  const std::vector<ActsPlugins::Seed>& seeds,
                                                                  const std::vector<uint64_t>& /*moduleIds*/,
                                                                  const ActsPlugins::ExecutionContext& execContext) {
  assert(numNodes > 0 && inputValues.size() % numNodes == 0);
  const std::size_t numFeatures = inputValues.size() / numNodes;

  // Seed hit indices refer to the rows of inputValues, i.e. to the position of
  // the node, not to the space point IDs.
  std::vector<SeedRoad::SeedParams> seedParams{};
  std::vector<SeedRoad::SeedHits> seedHits{};
  seedParams.reserve(seeds.size());
  seedHits.reserve(seeds.size());
  for (const ActsPlugins::Seed& seed : seeds) {
    if (seed.hitIndices.size() != kHitsPerSeed) {
      throw std::runtime_error(fmt::format("A seed needs {} hits, got {}", kHitsPerSeed, seed.hitIndices.size()));
    }
    SeedRoad::SeedParams params{};
    params.d0 = seed.d0;
    params.z0 = seed.z0;
    params.phi0 = seed.phi0;
    params.tanLambda = seed.tanLambda;
    params.kappa = seed.kappa;
    params.t0 = seed.t0;
    seedParams.push_back(params);
    seedHits.push_back({static_cast<int64_t>(seed.hitIndices[0]), static_cast<int64_t>(seed.hitIndices[1]),
                        static_cast<int64_t>(seed.hitIndices[2])});
  }

  if (seeds.empty()) {
    ACTS_DEBUG("No seeds for this graph, too few for a track candidate");
    throw ActsPlugins::NoEdgesError{};
  }

  const SeedRoad::HitGeometry hits = SeedRoad::hitGeometryFromFeatures(inputValues, numNodes, config().hitFeatures);
  const SeedRoad::RoadEdges road = SeedRoad::buildRoadEdges(seedParams, hits, config().road, seedHits);
  ACTS_DEBUG(fmt::format("Built {} seeds and {} road edges from {} nodes", seeds.size(), road.size(), numNodes));
  if (road.size() == 0) {
    ACTS_DEBUG("No road edges for these seeds");
    throw ActsPlugins::NoEdgesError{};
  }

  // Inputs of the seeded network, in the order of its inputs. Seed hit and
  // road edge indices are int64, the rest float.
  std::vector<float> seedInputs = SeedRoad::buildSeedInputs(seedParams, config().seedFeatures);
  std::vector<int64_t> seedHitValues{};
  seedHitValues.reserve(seedHits.size() * kHitsPerSeed);
  for (const SeedRoad::SeedHits& hitIndices : seedHits) {
    seedHitValues.insert(seedHitValues.end(), hitIndices.begin(), hitIndices.end());
  }
  std::vector<int64_t> edgeSeed = road.seed;
  std::vector<int64_t> edgeHit = road.hit;
  std::vector<float> edgeInputs = SeedRoad::buildEdgeInputs(road, config().seedHitEdges);

  const std::size_t numSeeds = seeds.size();
  const std::size_t numEdges = road.size();
  // buildEdgeInputs gives one constant input per edge without edge features
  const std::size_t numEdgeFeatures = edgeInputs.size() / numEdges;
  if (m_inputLength > 0 && static_cast<int64_t>(numNodes) != m_inputLength) {
    throw std::runtime_error(
        fmt::format("Seeded node model expects a fixed input length of {} nodes, but {} would be passed to it",
                    m_inputLength, numNodes));
  }

  // Features the network takes, empty selects all of them
  std::vector<int> selectedFeatures = config().selectedFeatures;
  if (selectedFeatures.empty()) {
    selectedFeatures.resize(numFeatures);
    std::iota(selectedFeatures.begin(), selectedFeatures.end(), 0);
  }
  for (const int feature : selectedFeatures) {
    if (feature < 0 || static_cast<std::size_t>(feature) >= numFeatures) {
      throw std::runtime_error(
          fmt::format("Selected feature {} is out of range for {} node features", feature, numFeatures));
    }
  }
  const std::size_t numSelected = selectedFeatures.size();
  const std::vector<float>& featureScales = config().featureScales;
  if (!featureScales.empty() && featureScales.size() != numSelected) {
    throw std::runtime_error(
        fmt::format("featureScales has {} entries, but {} features are selected", featureScales.size(), numSelected));
  }

  // The network sees the selected node features divided by featureScales, as in
  // training. This copy is separate because the road geometry above and the rest
  // of the pipeline need the unscaled values.
  std::vector<float> networkNodes(numNodes * numSelected, 0.f);
  for (std::size_t n = 0; n < numNodes; ++n) {
    for (std::size_t s = 0; s < numSelected; ++s) {
      float value = inputValues[n * numFeatures + static_cast<std::size_t>(selectedFeatures[s])];
      if (!featureScales.empty()) {
        value /= featureScales[s];
      }
      networkNodes[n * numSelected + s] = value;
    }
  }
  const std::vector<int64_t> nodeShape{static_cast<int64_t>(numNodes), static_cast<int64_t>(numSelected)};
  const std::vector<int64_t> seedInputShape{static_cast<int64_t>(numSeeds),
                                            static_cast<int64_t>(config().seedFeatures.features.size())};
  const std::vector<int64_t> seedHitShape{static_cast<int64_t>(numSeeds), static_cast<int64_t>(kHitsPerSeed)};
  const std::vector<int64_t> edgeShape{static_cast<int64_t>(numEdges)};
  const std::vector<int64_t> edgeInputShape{static_cast<int64_t>(numEdges), static_cast<int64_t>(numEdgeFeatures)};

  // The tensors wrap the buffers above, which stay alive until the inference is done.
  auto memoryInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  std::vector<Ort::Value> networkInputs{};
  networkInputs.push_back(Ort::Value::CreateTensor<float>(memoryInfo, networkNodes.data(), networkNodes.size(),
                                                          nodeShape.data(), nodeShape.size()));
  networkInputs.push_back(Ort::Value::CreateTensor<float>(memoryInfo, seedInputs.data(), seedInputs.size(),
                                                          seedInputShape.data(), seedInputShape.size()));
  networkInputs.push_back(Ort::Value::CreateTensor<int64_t>(memoryInfo, seedHitValues.data(), seedHitValues.size(),
                                                            seedHitShape.data(), seedHitShape.size()));
  networkInputs.push_back(Ort::Value::CreateTensor<int64_t>(memoryInfo, edgeSeed.data(), edgeSeed.size(),
                                                            edgeShape.data(), edgeShape.size()));
  networkInputs.push_back(Ort::Value::CreateTensor<int64_t>(memoryInfo, edgeHit.data(), edgeHit.size(),
                                                            edgeShape.data(), edgeShape.size()));
  networkInputs.push_back(Ort::Value::CreateTensor<float>(memoryInfo, edgeInputs.data(), edgeInputs.size(),
                                                          edgeInputShape.data(), edgeInputShape.size()));

  if (execContext.device.isCuda()) {
#ifdef ACTS_GNN_WITH_CUDA
    cudaSetDevice(execContext.device.index);
#else
    throw std::runtime_error("can not run on CUDA, k4ActsTracking/ACTS was built without CUDA support");
#endif
  }
  const int cudaDeviceIndex = execContext.device.isCuda() ? static_cast<int>(execContext.device.index) : -1;
  const auto outputs = m_model.runInference(networkInputs, cudaDeviceIndex, /*outputsOnHost=*/true);
  ACTS_DEBUG(fmt::format("Seeded network ran on {} seeds and {} road edges", numSeeds, numEdges));

  // Output order: hit_embedding, seed_embedding, seed_logit, edge_logit
  std::vector<float> hitEmbedding = readFloatOutput(outputs[0]);
  const std::vector<float> seedEmbedding = readFloatOutput(outputs[1]);
  const std::vector<float> seedLogits = readFloatOutput(outputs[2]);
  if (hitEmbedding.size() % numNodes != 0) {
    throw std::runtime_error("Seeded network hit embedding does not match the number of nodes");
  }
  const std::size_t embeddingDim = hitEmbedding.size() / numNodes;
  if (seedEmbedding.size() != numSeeds * embeddingDim || seedLogits.size() != numSeeds) {
    throw std::runtime_error("Seeded network outputs do not match the number of seeds");
  }

  std::vector<int64_t> selectedSeeds{};
  for (std::size_t seed = 0; seed < numSeeds; ++seed) {
    const float score = 1.f / (1.f + std::exp(-seedLogits[seed]));
    if (score >= config().seedScoreThreshold) {
      selectedSeeds.push_back(static_cast<int64_t>(seed));
    }
  }

  auto ballEdges =
      buildSeedBallEdges(hitEmbedding, seedEmbedding, embeddingDim, selectedSeeds, config().rVal, config().knn);
  ACTS_DEBUG(
      fmt::format("Selected {} of {} seeds and built {} ball edges", selectedSeeds.size(), numSeeds, ballEdges.size()));

  // Orient every edge outwards by r and z, as in OnnxMetricLearning
  if (!config().distanceFeatureIndices.empty()) {
    gnntracking::checkDistanceFeatureIndices(config().distanceFeatureIndices, numFeatures);
    const std::vector<float> distancesSq =
        gnntracking::nodeDistancesSq(inputValues, numNodes, numFeatures, config().distanceFeatureIndices);
    for (auto& [first, second] : ballEdges) {
      if (!gnntracking::pointsOutward(distancesSq, static_cast<int>(first), static_cast<int>(second))) {
        std::swap(first, second);
      }
    }
  }

  // Hit-to-hit edges as a 2 x numBallEdges int64 edge index, as in OnnxMetricLearning
  const int64_t numBallEdges = static_cast<int64_t>(ballEdges.size());
  std::vector<int64_t> edgeIndexValues(2 * ballEdges.size());
  for (std::size_t e = 0; e < ballEdges.size(); ++e) {
    edgeIndexValues[e] = ballEdges[e].first;
    edgeIndexValues[ballEdges.size() + e] = ballEdges[e].second;
  }
  // On the device of execContext, as the ACTS tensor conversion below copies from a blob with the same device
  const torch::Device torchDevice = execContext.device.isCuda()
                                        ? torch::Device(torch::kCUDA, static_cast<int64_t>(execContext.device.index))
                                        : torch::Device(torch::kCPU);
  const torch::Tensor edgeList = torch::tensor(edgeIndexValues, torch::kInt64).view({2, numBallEdges}).to(torchDevice);

  // Same guard as OnnxMetricLearning, too few edges to build any track candidate
  constexpr int64_t minUsableEdges = 2;
  if (numBallEdges < minUsableEdges) {
    ACTS_DEBUG(fmt::format("Built {} edges from {} nodes, too few for a track candidate", numBallEdges, numNodes));
    throw ActsPlugins::NoEdgesError{};
  }

  std::optional<torch::Tensor> edgeFeatures{};
  if (!config().edgeFeatureIndices.empty()) {
    if (config().edgeFeatureIndices.size() != OnnxMetricLearning::kNumEdgeFeatureInputs) {
      throw std::runtime_error(
          fmt::format("The edge features need exactly {} node features (r, phi, z, eta), but {} are configured",
                      OnnxMetricLearning::kNumEdgeFeatureInputs, config().edgeFeatureIndices.size()));
    }
    for (const int index : config().edgeFeatureIndices) {
      if (index < 0 || static_cast<std::size_t>(index) >= numFeatures) {
        throw std::runtime_error(
            fmt::format("Edge feature input index {} is out of range for {} node features", index, numFeatures));
      }
    }
    edgeFeatures = gnntracking::buildEdgeFeatures(inputValues, numNodes, numFeatures, edgeList,
                                                  config().edgeFeatureIndices, config().edgeFeatureScales);
    ACTS_DEBUG(fmt::format("Built {} edge features for {} edges", edgeFeatures->sizes()[0], numBallEdges));
  }

  std::optional<ActsPlugins::Tensor<float>> actsEdgeFeatures{};
  if (edgeFeatures.has_value()) {
    actsEdgeFeatures.emplace(ActsPlugins::detail::torchToActsTensor<float>(edgeFeatures->contiguous(), execContext));
  }

  auto nodeTensor = gnntracking::makeNodeFeatureTensor(inputValues, numFeatures, execContext);
  auto actsEdgeList = ActsPlugins::detail::torchToActsTensor<int64_t>(edgeList, execContext);

  return {std::move(nodeTensor), std::move(actsEdgeList), std::move(actsEdgeFeatures), std::nullopt};
}
