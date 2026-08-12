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

#include "CCAndWalkTrackBuilding.h"
#include "OnnxMetricLearning.h"
#include "PaddedEdgeRemoval.h"

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
  using TrackBuildingBase      = Acts::TrackBuildingBase;
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
#include <fmt/ranges.h>

#include <algorithm>
#include <array>
#include <cassert>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <optional>
#include <ranges>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace {
  /// The hit features the six edge features (dr, dphi, dz, deta, phislope,
  /// rphislope) are computed from. Their meaning is fixed by the formulas, so
  /// unlike the per-model input features these are not configurable - only
  /// their scales are, see GNNTrackFinder::m_edgeFeatureScales.
  const std::array<std::string, OnnxMetricLearning::kNumEdgeFeatureInputs> kEdgeFeatureInputs{"r", "phi", "z", "eta"};

  /// Track building by connected components alone, i.e. Acts' BoostTrackBuilding
  constexpr const char* kTrackBuildingCC = "connected-components";
  /// Track building that additionally walks the components that are not paths
  constexpr const char* kTrackBuildingCCAndWalk = "cc-and-walk";

  /// The hit feature the "cc-and-walk" track building orders the two hits of an
  /// edge by, to give the graph a direction. Not configurable: any other choice
  /// would not be a radius.
  const std::array<std::string, 1> kRadiusFeature{"r"};

  /// Lower-case an (ASCII) configuration string, so that the device
  /// specification can be given in any case.
  std::string toLower(std::string str) {
    std::transform(str.begin(), str.end(), str.begin(), [](unsigned char c) { return std::tolower(c); });
    return str;
  }

  /// Build bin edges for segmentation. Every bin's upper edge is extended by
  /// @p overlapFraction of the bin width into its successor.
  ///
  /// @param wrap whether the coordinate is periodic, i.e. whether the last bin
  ///        has a successor to reach into. If it is, the last bin's upper edge
  ///        is extended past @p max, and a value just above @p min falls into
  ///        it as well - see binsFor().
  std::vector<std::pair<double, double>> buildBinEdges(double min, double max, std::size_t numBins,
                                                       double overlapFraction, bool wrap) {
    if (numBins == 0) {
      throw std::invalid_argument("Number of bins must be greater than zero");
    }
    if (min >= max) {
      throw std::invalid_argument("Minimum value must be less than maximum value");
    }
    if (overlapFraction < 0.0 || overlapFraction >= 1.0) {
      throw std::invalid_argument("Overlap fraction must be in the range [0, 1)");
    }

    std::vector<std::pair<double, double>> edges(numBins);
    const double                           binWidth     = (max - min) / static_cast<double>(numBins);
    const double                           overlapWidth = binWidth * overlapFraction;

    for (std::size_t i = 0; i < numBins; ++i) {
      const double left  = min + i * binWidth;
      const double right = left + binWidth;
      edges[i]           = {left, right};
      if (i < numBins - 1 || wrap)
        edges[i].second += overlapWidth;
    }
    return edges;
  }

  /// Bins of @p edges that @p value falls into, written into @p bins. Returns
  /// how many there are: since a bin only ever overlaps with its successor, a
  /// value is in one or two of them.
  ///
  /// @param value must be inside the [min, max) range @p edges was built for
  /// @param invBinWidth numBins / (max - min), i.e. the inverse (unextended) bin width
  /// @param wrap whether the coordinate is periodic, see buildBinEdges(). The
  ///        two bins are adjacent, but with wrapping they can be the last and
  ///        the first one rather than i - 1 and i.
  std::size_t binsFor(double value, double min, double invBinWidth, const std::vector<std::pair<double, double>>& edges,
                      bool wrap, std::array<std::size_t, 2>& bins) {
    // Direct lookup instead of a scan over all bins. The clamping guards
    // against rounding differences w.r.t. the bin edges (and against a value
    // sitting exactly on max).
    std::size_t bin = std::min(static_cast<std::size_t>((value - min) * invBinWidth), edges.size() - 1);
    if (bin > 0 && value < edges[bin].first) {
      --bin;
    }
    bins[0] = bin;

    // The preceding bin's upper edge is extended by the overlap, so it may still
    // contain the value.
    if (bin > 0) {
      if (value < edges[bin - 1].second) {
        bins[1] = bin - 1;
        return 2;
      }
    } else if (wrap && edges.size() > 1) {
      // Bin 0's predecessor is the last bin, whose upper edge was extended past
      // max. Comparing against it means lifting the value by one period. With a
      // single bin there is no predecessor - it already covers the full period,
      // and returning it twice would put the hit into the same segment twice.
      const double period = static_cast<double>(edges.size()) / invBinWidth;
      if (value + period < edges.back().second) {
        bins[1] = edges.size() - 1;
        return 2;
      }
    }
    return 1;
  }

  /// Format bin edges for the debug output. An upper edge past @p max belongs to
  /// a bin that wraps around, so show where it actually reaches to.
  std::string formatBinEdges(const std::vector<std::pair<double, double>>& edges, double min, double max) {
    std::string formatted{};
    for (const auto& [low, high] : edges) {
      if (high > max) {
        formatted += fmt::format(" [{}, {} -> {}]", low, max, min + (high - max));
      } else {
        formatted += fmt::format(" [{}, {}]", low, high);
      }
    }
    return formatted;
  }

  /// Parse a device string ("cpu", "cuda", "cuda:<index>") into an Acts Device.
  /// Throws std::invalid_argument on an unrecognised value.
  ActsPlugins::Device parseDevice(const std::string& deviceSpec) {
    const auto spec = toLower(deviceSpec);

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

  if (m_extrapolateToCalo && m_actsGeoSvc->caloSurfaceGeoIds().empty()) {
    warning() << "ExtrapolateToCalo requested but ActsGeoSvc provides no calorimeter-face surfaces; "
                 "no AtCalorimeter track states will be produced."
              << endmsg;
  }
  if (m_addEndcapCaloState && !m_extrapolateToCalo) {
    error() << "AddEndcapCaloState requested but ExtrapolateToCalo is off; no AtCalorimeter track states "
               "are produced at all, so the setting has no effect."
            << endmsg;
    return StatusCode::FAILURE;
  } else if (m_addEndcapCaloState && m_actsGeoSvc->caloEndcapSurfaceGeoIds().empty()) {
    warning() << "AddEndcapCaloState requested but ActsGeoSvc provides no calorimeter endcap surfaces; "
                 "every track will keep a single AtCalorimeter state."
              << endmsg;
  }

  // All edge classifier properties are parallel lists with one entry per model
  const std::size_t nEdgeClassifiers = m_edgeClassifierModelPath.size();
  if (nEdgeClassifiers == 0) {
    error() << "No edge classifier model configured, please set EdgeClassifierModelPath" << endmsg;
    return StatusCode::FAILURE;
  }
  if (m_inputFeaturesEdgeClassifier.size() != nEdgeClassifiers ||
      m_inputScalesEdgeClassifier.size() != nEdgeClassifiers || m_edgeClassifierCut.size() != nEdgeClassifiers) {
    error() << fmt::format(
                   "Inconsistent edge classifier configuration: {} model paths, but {} input feature lists, {} input "
                   "scale lists and {} cut values (all have to have one entry per edge classifier model)",
                   nEdgeClassifiers, m_inputFeaturesEdgeClassifier.size(), m_inputScalesEdgeClassifier.size(),
                   m_edgeClassifierCut.size())
            << endmsg;
    return StatusCode::FAILURE;
  }

  const auto embeddingFeatures          = mlutils::parseList<std::string>(m_inputFeaturesEmbedding.value());
  const auto embeddingScales            = mlutils::parseList<float>(m_inputScalesEmbedding.value());
  const auto edgeFeatureScales          = mlutils::parseList<float>(m_edgeFeatureScales.value());
  const auto edgeClassifierFeaturesList = mlutils::parseMultiList<std::string>(m_inputFeaturesEdgeClassifier.value());
  const auto edgeClassifierScalesList   = mlutils::parseMultiList<float>(m_inputScalesEdgeClassifier.value());

  // The six edge features are defined in terms of r, phi, z and eta, so unlike
  // the model inputs there is nothing to select: all that is configurable is
  // whether they are computed at all, and the scales of those four features.
  const bool computeEdgeFeatures = m_computeEdgeFeatures.value();
  if (!computeEdgeFeatures && !edgeFeatureScales.empty()) {
    error() << "EdgeFeatureScales is set, but ComputeEdgeFeatures is false, so no edge features are computed" << endmsg;
    return StatusCode::FAILURE;
  }

  // There is no padding to keep without the padding itself
  if (m_keepEmbeddingPadding.value() && m_embeddingFixedInputLength.value() <= 0) {
    error() << "KeepEmbeddingPadding is set, but EmbeddingFixedInputLength is 0, so the embedding input is not padded"
            << endmsg;
    return StatusCode::FAILURE;
  }

  if (m_edgeClassifierFixedInputLength.value() > 0) {
    // The padding edges are self loops on a padding node, so there has to be
    // one. Anchoring them on a real hit instead would feed that hit as many
    // spurious messages as there are padding edges.
    if (!m_keepEmbeddingPadding.value()) {
      error() << "EdgeClassifierFixedInputLength needs KeepEmbeddingPadding, the padding edges are anchored on a "
                 "padding node so that they touch no real hit"
              << endmsg;
      return StatusCode::FAILURE;
    }
    // The padding happens once, in the graph construction. Every classifier
    // applies its own cut, so from the second one on the edge count is whatever
    // survived the previous cut and no longer the fixed length.
    if (nEdgeClassifiers > 1) {
      error() << fmt::format(
                     "EdgeClassifierFixedInputLength does not work with {} chained edge classifiers: each one "
                     "cuts on the score, so only the first would see the padded edge count",
                     nEdgeClassifiers)
              << endmsg;
      return StatusCode::FAILURE;
    }
  }

  const bool ccAndWalk = m_trackBuilding.value() == kTrackBuildingCCAndWalk;
  if (!ccAndWalk && m_trackBuilding.value() != kTrackBuildingCC) {
    error() << fmt::format(R"(Unknown TrackBuilding "{}", expected "{}" or "{}")", m_trackBuilding.value(),
                           kTrackBuildingCC, kTrackBuildingCCAndWalk)
            << endmsg;
    return StatusCode::FAILURE;
  }
  if (ccAndWalk && m_walkMinScore.value() > m_walkAddScore.value()) {
    error() << fmt::format(
                   "WalkMinScore ({}) is above WalkAddScore ({}), so the threshold for branching would be "
                   "looser than the one for following a single edge",
                   m_walkMinScore.value(), m_walkAddScore.value())
            << endmsg;
    return StatusCode::FAILURE;
  }

  // The models divide each feature by its scale, so there has to be exactly one
  // scale per feature (or none at all, in which case no scaling is applied).
  const auto checkScales = [this](const std::string& what, std::size_t nFeatures, std::size_t nScales) {
    if (nScales == 0 || nFeatures == nScales) {
      return true;
    }
    error() << fmt::format("Number of input scales ({}) does not match the number of input features ({}) for {}",
                           nScales, nFeatures, what)
            << endmsg;
    return false;
  };
  if (!checkScales("the node embedding model", embeddingFeatures.size(), embeddingScales.size())) {
    return StatusCode::FAILURE;
  }
  if (computeEdgeFeatures &&
      !checkScales(fmt::format("the edge feature computation ({}, in that order)", fmt::join(kEdgeFeatureInputs, ", ")),
                   kEdgeFeatureInputs.size(), edgeFeatureScales.size())) {
    return StatusCode::FAILURE;
  }
  for (std::size_t i = 0; i < nEdgeClassifiers; ++i) {
    if (!checkScales(fmt::format("edge classifier {}", i), edgeClassifierFeaturesList[i].size(),
                     edgeClassifierScalesList[i].size())) {
      return StatusCode::FAILURE;
    }
  }

  try {
    m_runDevice = parseDevice(m_device.value());

    // Build bin edges for theta/phi segmentation
    // theta runs from 0 to pi and stops there, phi is periodic and its last bin
    // reaches back around into the first one.
    m_thetaBinEdges = buildBinEdges(0.0, M_PI, m_thetaBins.value(), m_thetaOverlap.value(), /*wrap=*/false);
    m_phiBinEdges   = buildBinEdges(-M_PI, M_PI, m_phiBins.value(), m_phiOverlap.value(), /*wrap=*/true);
  } catch (const std::invalid_argument& ex) {
    error() << ex.what() << endmsg;
    return StatusCode::FAILURE;
  }
  info() << fmt::format("Running GNN pipeline on device '{}'", m_device.value()) << endmsg;
  debug() << fmt::format("Theta bin edges:{}", formatBinEdges(m_thetaBinEdges, 0.0, M_PI)) << endmsg;
  debug() << fmt::format("Phi bin edges:{}", formatBinEdges(m_phiBinEdges, -M_PI, M_PI)) << endmsg;

  // Build the deduplicated list of all hit features that have to be extracted,
  // preserving the order in which they are configured. The pipeline passes the
  // full per-hit feature vector to every stage, and each stage selects the
  // features it needs from it by index.
  m_allHitFeatures.clear();
  std::unordered_set<std::string> seenFeatures{};
  const auto                      addFeatures = [&seenFeatures, this](const auto& features) {
    for (const auto& feature : features) {
      if (seenFeatures.insert(feature).second) {
        m_allHitFeatures.push_back(feature);
      }
    }
  };
  addFeatures(embeddingFeatures);
  if (computeEdgeFeatures) {
    addFeatures(kEdgeFeatureInputs);
  }
  if (ccAndWalk) {
    addFeatures(kRadiusFeature);
  }
  for (const auto& edgeClassifierFeatures : edgeClassifierFeaturesList) {
    addFeatures(edgeClassifierFeatures);
  }
  debug() << fmt::format("All hit features: {}", fmt::join(m_allHitFeatures, ", ")) << endmsg;

  // Translate the lists of input features into lists of indices in the full
  // per-hit feature vector for each model.
  const auto featureIndices = [this](const auto& features) {
    std::vector<int> indices{};
    indices.reserve(features.size());
    for (const auto& f : features) {
      const auto it = std::find(m_allHitFeatures.begin(), m_allHitFeatures.end(), f);
      indices.push_back(static_cast<int>(std::distance(m_allHitFeatures.begin(), it)));
    }
    return indices;
  };
  m_embeddingFeatureIndices = featureIndices(embeddingFeatures);
  m_edgeFeatureIndices      = computeEdgeFeatures ? featureIndices(kEdgeFeatureInputs) : std::vector<int>{};
  m_radiusFeatureIndex      = ccAndWalk ? featureIndices(kRadiusFeature).front() : -1;
  m_edgeClassifierFeatureIndices.clear();
  m_edgeClassifierFeatureIndices.reserve(nEdgeClassifiers);
  for (const auto& edgeClassifierFeatures : edgeClassifierFeaturesList) {
    m_edgeClassifierFeatureIndices.push_back(featureIndices(edgeClassifierFeatures));
  }

  // Resolve the feature names into per-hit accessors once, so that the event
  // loop neither compares strings nor looks up CellID fields by name.
  try {
    m_cellIDDecoder.emplace(m_actsGeoSvc->cellIDEncodingString());
    m_resolvedHitFeatures = ACTSTracking::resolveHitFeatures(m_allHitFeatures, &*m_cellIDDecoder);
  } catch (const std::exception& ex) {
    error() << ex.what() << endmsg;
    return StatusCode::FAILURE;
  }

  // Building the stages loads the ONNX models, so anything from a missing model
  // file to an inconsistent pipeline shows up here.
  try {
    buildPipeline(embeddingScales, edgeFeatureScales, edgeClassifierScalesList);
  } catch (const std::exception& ex) {
    error() << "Failed to construct the GNN pipeline: " << ex.what() << endmsg;
    return StatusCode::FAILURE;
  }

  return StatusCode::SUCCESS;
}

void GNNTrackFinder::buildPipeline(const std::vector<float>&              embeddingScales,
                                   const std::vector<float>&              edgeFeatureScales,
                                   const std::vector<std::vector<float>>& edgeClassifierScales) {
  auto graphConstructor = std::make_shared<OnnxMetricLearning>(
      OnnxMetricLearning::Config{.modelPath          = m_nodeEmbeddingModelPath.value(),
                                 .selectedFeatures   = m_embeddingFeatureIndices,
                                 .featureScales      = embeddingScales,
                                 .edgeFeatureIndices = m_edgeFeatureIndices,
                                 .edgeFeatureScales  = edgeFeatureScales,
                                 .fixedInputLength   = m_embeddingFixedInputLength.value(),
                                 .keepPadding        = m_keepEmbeddingPadding.value(),
                                 .fixedEdgeLength    = m_edgeClassifierFixedInputLength.value(),
                                 .rVal               = m_edgeBuildingRadius.value(),
                                 .knnVal             = m_edgeBuildingKnn.value(),
                                 .device             = m_runDevice},
      m_logger->clone(name() + ".MetricLearning"));

  std::vector<std::shared_ptr<ActsPlugins::EdgeClassificationBase>> edgeClassifiers{};
  edgeClassifiers.reserve(m_edgeClassifierModelPath.size());
  for (std::size_t i = 0; i < m_edgeClassifierModelPath.size(); ++i) {
    edgeClassifiers.push_back(std::make_shared<ActsPlugins::OnnxEdgeClassifier>(
        ActsPlugins::OnnxEdgeClassifier::Config{.modelPath        = m_edgeClassifierModelPath[i],
                                                .selectedFeatures = m_edgeClassifierFeatureIndices[i],
                                                .featureScales    = edgeClassifierScales[i],
                                                .cut              = m_edgeClassifierCut[i],
                                                // The Acts Config defaults to Device::Cuda(); use the configured
                                                // device (default "cpu") since the onnxruntime build may not have a
                                                // CUDA execution provider.
                                                .device = m_runDevice},
        m_logger->clone(name() + fmt::format(".EdgeClassifier{}", i))));
  }

  // The padding edges have to be gone before the track building, so this runs
  // as the last link of the classifier chain.
  if (m_edgeClassifierFixedInputLength.value() > 0) {
    edgeClassifiers.push_back(std::make_shared<PaddedEdgeRemoval>(m_logger->clone(name() + ".PaddedEdgeRemoval")));
  }

  std::shared_ptr<ActsPlugins::TrackBuildingBase> trackBuilder{};
  if (m_trackBuilding.value() == kTrackBuildingCCAndWalk) {
    trackBuilder = std::make_shared<CCAndWalkTrackBuilding>(
        CCAndWalkTrackBuilding::Config{.rFeatureIndex    = m_radiusFeatureIndex,
                                       .addScore         = m_walkAddScore.value(),
                                       .minScore         = m_walkMinScore.value(),
                                       .minCandidateSize = m_minHitsPerTrk.value()},
        m_logger->clone(name() + ".TrackBuilder"));
  } else {
    trackBuilder = std::make_shared<ActsPlugins::BoostTrackBuilding>(ActsPlugins::BoostTrackBuilding::Config{},
                                                                     m_logger->clone(name() + ".TrackBuilder"));
  }
  info() << fmt::format("Building track candidates with the \"{}\" algorithm", m_trackBuilding.value()) << endmsg;

  m_pipeline = std::make_unique<ActsPlugins::GnnPipeline>(graphConstructor, edgeClassifiers, trackBuilder,
                                                          m_logger->clone(name() + ".Pipeline"));
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

  // Split all hits into theta/phi segments
  const std::size_t                               nThetaBins = m_thetaBins.value();
  const std::size_t                               nPhiBins   = m_phiBins.value();
  const std::size_t                               nSegments  = nThetaBins * nPhiBins;
  std::vector<edm4hep::TrackerHitPlaneCollection> thetaPhiHits(nSegments);
  for (auto& segHits : thetaPhiHits) {
    segHits.setSubsetCollection(true);
  }
  std::vector<std::vector<int>> hitIdcs(nSegments);

  // Make this configurable properties?
  const double thetaMin = 0.0;
  const double thetaMax = M_PI;
  const double phiMin   = -M_PI;
  const double phiMax   = M_PI;

  const double invThetaBinWidth = static_cast<double>(nThetaBins) / (thetaMax - thetaMin);
  const double invPhiBinWidth   = static_cast<double>(nPhiBins) / (phiMax - phiMin);

  // Largest representable values that are still inside the binned ranges
  const double thetaUpper = std::nextafter(thetaMax, thetaMin);
  const double phiUpper   = std::nextafter(phiMax, phiMin);

  int hitIndex = 0;
  for (const auto& hit : allHits) {
    const auto position = ROOT::Math::XYZPointF(hit.getPosition().x, hit.getPosition().y, hit.getPosition().z);

    // theta is in [0, pi] and phi in (-pi, pi], i.e. both are already in the
    // binned ranges, the clamping only removes the (closed) upper edge.
    const double theta = std::clamp<double>(position.theta(), thetaMin, thetaUpper);
    const double phi   = std::clamp<double>(position.phi(), phiMin, phiUpper);

    // With overlapping bins a hit can end up in two adjacent bins per
    // coordinate. In phi those two are not necessarily consecutive indices: the
    // last bin wraps around into the first one, so the bins are listed rather
    // than iterated over as a range.
    std::array<std::size_t, 2> thetaBins{};
    std::array<std::size_t, 2> phiBins{};
    const std::size_t          numThetaBinsFor =
        binsFor(theta, thetaMin, invThetaBinWidth, m_thetaBinEdges, /*wrap=*/false, thetaBins);
    const std::size_t numPhiBinsFor = binsFor(phi, phiMin, invPhiBinWidth, m_phiBinEdges, /*wrap=*/true, phiBins);

    for (std::size_t t = 0; t < numThetaBinsFor; ++t) {
      for (std::size_t p = 0; p < numPhiBinsFor; ++p) {
        const std::size_t segmentIdx = thetaBins[t] * nPhiBins + phiBins[p];
        thetaPhiHits[segmentIdx].push_back(hit);
        hitIdcs[segmentIdx].push_back(hitIndex);
      }
    }
    ++hitIndex;
  }

  debug() << fmt::format("Segmented hits into {} segments ({} theta bins x {} phi bins)", nSegments, nThetaBins,
                         nPhiBins)
          << endmsg;

  // Run GNN pipeline on all segments and collect the track candidates of all of
  // them in one flat list. The candidates refer to the indices in allHits.
  const std::size_t             nFeatures = m_resolvedHitFeatures.size();
  std::vector<std::vector<int>> trackCandIdcs;
  for (std::size_t segmentIdx = 0; segmentIdx < nSegments; ++segmentIdx) {
    const auto& segmentHits = thetaPhiHits[segmentIdx];
    // A candidate only ever contains hits of its own segment, so a segment with
    // fewer hits than a track needs cannot contribute one and is not worth an
    // inference. With a fine segmentation most segments are in this case.
    if (segmentHits.size() < m_minHitsPerTrk.value()) {
      const std::size_t thetaBin = segmentIdx / nPhiBins;
      const std::size_t phiBin   = segmentIdx % nPhiBins;
      debug() << fmt::format("Segment (thetaBin={}, phiBin={}) has {} hits, fewer than MinHitsPerTrack ({}), skipping",
                             thetaBin, phiBin, segmentHits.size(), m_minHitsPerTrk.value())
              << endmsg;
      continue;
    }
    auto& segmentHitIdcs  = hitIdcs[segmentIdx];
    auto  embeddingInputs = ACTSTracking::extractHitInformation(segmentHits, m_resolvedHitFeatures, &*m_cellIDDecoder);
    assert(embeddingInputs.size() == segmentHits.size() * nFeatures);

    // Full detailed output of inputs
    if (m_detailedDebugOut.value()) {
      debug() << "Embedding input tensor shape: (" << segmentHits.size() << ", " << nFeatures << ")" << endmsg;
      for (std::size_t i = 0; i < segmentHits.size(); ++i) {
        debug() << fmt::format("Input space point {}: ", i);
        for (std::size_t j = 0; j < nFeatures; ++j) {
          debug() << fmt::format("  {}: {}", m_allHitFeatures[j], embeddingInputs[i * nFeatures + j]);
        }
        debug() << endmsg;
      }
    }

    auto segmentCandIdcs = m_pipeline->run(embeddingInputs, {}, segmentHitIdcs, m_runDevice);
    debug() << fmt::format("Received {} track candidates", segmentCandIdcs.size()) << endmsg;

    // Full detailed output of track candidates
    if (m_detailedDebugOut.value()) {
      for (std::size_t i = 0; i < segmentCandIdcs.size(); ++i) {
        debug() << fmt::format("Track candidate {}: {}", i, fmt::join(segmentCandIdcs[i], " ")) << endmsg;
      }
    }

    trackCandIdcs.insert(trackCandIdcs.end(), std::make_move_iterator(segmentCandIdcs.begin()),
                         std::make_move_iterator(segmentCandIdcs.end()));
  }

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
                                        {.propagateBackward  = m_propagateBackward,
                                         .extrapolateToCalo  = m_extrapolateToCalo,
                                         .addEndcapCaloState = m_addEndcapCaloState});

  edm4hep::TrackCollection trackCands{};
  auto                     histBuffer = m_monitoringHist.buffer();
  for (const auto& candIdcs : trackCandIdcs) {
    ++histBuffer[{allHits.size(), trackCandIdcs.size(), candIdcs.size()}];
    if (candIdcs.size() < m_minHitsPerTrk.value()) {
      continue;
    }

    // Build the radius-ordered seed hits from this candidate's hits.
    const std::vector<ACTSTracking::SeedHit> hits = ACTSTracking::collectSeedHits(
        candIdcs | std::views::transform([&allHits](int idx) { return allHits[idx]; }), slByHit, hitContainer);

    if (hits.size() < m_minHitsPerTrk.value()) {
      debug() << "Skipping candidate with " << hits.size() << " usable hits." << endmsg;
      continue;
    }

    // The innermost / middle / outermost hit form the seed from which the
    // initial parameters are estimated.
    std::optional<Acts::BoundTrackParameters> startParams = ACTSTracking::estimateSeedParameters(
        *this, *m_actsGeoSvc, geoCtx, hits, hitContainer, magCache, m_initialTrackError_pos, m_initialTrackError_phi,
        m_initialTrackError_lambda, m_initialTrackError_relP, m_initialTrackError_time);
    if (!startParams) {
      continue;
    }

    // All (radius-ordered) source links of the candidate are handed to the
    // Kalman fitter.
    std::vector<Acts::SourceLink> candSourceLinks;
    candSourceLinks.reserve(hits.size());
    for (const ACTSTracking::SeedHit& h : hits) {
      candSourceLinks.emplace_back(h.sl);
    }

    std::optional<edm4hep::MutableTrack> track =
        kfRunner.fit(*this, candSourceLinks, *startParams, magCache, &m_caloMonitor);
    if (!track) {
      continue;
    }
    trackCands.push_back(std::move(*track));
  }
  debug() << fmt::format("Produced {} fitted tracks from {} candidates", trackCands.size(), trackCandIdcs.size())
          << endmsg;
  return trackCands;
}

StatusCode GNNTrackFinder::finalize() {
  if (m_extrapolateToCalo) {
    info() << m_caloMonitor.summary() << endmsg;
  }
  return StatusCode::SUCCESS;
}

DECLARE_COMPONENT(GNNTrackFinder)
