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
#pragma once

#include <k4FWCore/Transformer.h>

#include <k4ActsTracking/HitFeatures.hxx>
#include <k4ActsTracking/IActsGeoSvc.h>
#include <k4ActsTracking/RunnerCommon.hxx>

#include <Acts/Definitions/Units.hpp>
#include <Acts/Utilities/Logger.hpp>

#include <ActsPlugins/Gnn/GnnPipeline.hpp>

#include <GaudiKernel/SmartIF.h>

#include <Gaudi/Accumulators/RootHistogram.h>
#include <Gaudi/Property.h>

#include <edm4hep/TrackCollection.h>
#include <edm4hep/TrackerHitPlaneCollection.h>

#include <DDSegmentation/BitFieldCoder.h>

#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

struct GNNTrackFinder : public k4FWCore::Transformer<edm4hep::TrackCollection(
                            std::vector<const edm4hep::TrackerHitPlaneCollection*> const&)> {
  GNNTrackFinder(const std::string& name, ISvcLocator* svcLoc);

  StatusCode initialize() override;

  StatusCode finalize() override;

  edm4hep::TrackCollection operator()(std::vector<const edm4hep::TrackerHitPlaneCollection*> const&) const override;
  Gaudi::Property<std::size_t> m_thetaBins{this, "ThetaBins", 1, "Number of theta bins for segmentation."};
  Gaudi::Property<std::size_t> m_phiBins{this, "PhiBins", 1, "Number of phi bins for segmentation."};
  Gaudi::Property<double> m_thetaOverlap{this, "ThetaOverlap", 0.0,
                                         "Fractional theta overlap for segmentation (fraction of bin width)."};
  Gaudi::Property<double> m_phiOverlap{this, "PhiOverlap", 0.0,
                                       "Fractional phi overlap for segmentation (fraction of bin width)."};

  Gaudi::Property<std::string> m_nodeEmbeddingModelPath{
      this, "NodeEmbeddingModelPath", "",
      "Path to the ONNX model file for the node embedding / graph construction metric model"};
  Gaudi::Property<float> m_edgeBuildingRadius{
      this, "EdgeBuildingRadius", 1.6f, "Radius in embedding space within which two hits are connected by an edge"};
  Gaudi::Property<int> m_edgeBuildingKnn{
      this, "EdgeBuildingKnn", 500,
      "Maximum number of neighbours per hit in the edge building. Only the CUDA (FRNN) edge building applies it as a "
      "cap; the CPU (KD-tree) edge building keeps every neighbour within EdgeBuildingRadius and uses this only to "
      "reserve memory. Must be > 0."};
  Gaudi::Property<std::vector<std::string>> m_inputFeaturesEmbedding{
      this, "InputFeaturesEmbedding", {"r", "phi", "z", "t"}, "Hit features for the node embedding model."};
  Gaudi::Property<std::vector<float>> m_inputScalesEmbedding{
      this,
      "InputScalesEmbedding",
      {1.f, 1.f, 1.f, 1.f},
      "Scales for the hit features for the node embedding model, each feature is divided by its scale. Must be "
      "the same size as InputFeaturesEmbedding (or empty for no scaling), and none may be zero."};
  Gaudi::Property<int> m_embeddingFixedInputLength{
      this, "EmbeddingFixedInputLength", 0,
      "If > 0, pad the node embedding model input with all-zero rows up to this many nodes, for models exported "
      "with a fixed-size input. The padding rows are appended after the hits of the segment and their embedding is "
      "discarded before the edge building. A segment with more hits than this is an error. 0 (the default) "
      "disables the padding."};
  Gaudi::Property<bool> m_keepEmbeddingPadding{
      this, "KeepEmbeddingPadding", false,
      "If true, the zero rows added by EmbeddingFixedInputLength are kept in the node features handed to the edge "
      "classifiers, for classifier models that are themselves exported at that same fixed number of nodes. The edge "
      "building always runs on the real hits alone, whatever this is set to, so the padding rows arrive at the "
      "classifiers as nodes without edges. Needs EmbeddingFixedInputLength > 0."};
  Gaudi::Property<int> m_edgeClassifierFixedInputLength{
      this, "EdgeClassifierFixedInputLength", 0,
      "If > 0, pad the edge index and the edge features up to this many edges, for edge classifier models exported "
      "with a fixed-size edge input. The padding edges are self loops on the last padding node, so they touch no "
      "real hit, and they are removed again after the classification. A segment with more edges than this is an "
      "error. Needs KeepEmbeddingPadding, and only one edge classifier. 0 (the default) disables the padding."};

  Gaudi::Property<bool> m_sortEdges{
      this, "SortEdges", true,
      "If true, orient every built edge from the hit closer to the interaction point (by r^2 + z^2) to the one "
      "further out, before the edge features are computed. "};
  Gaudi::Property<bool> m_computeEdgeFeatures{
      this, "ComputeEdgeFeatures", false,
      "If true, compute the six edge features (dr, dphi, dz, deta, phislope, rphislope) for every built edge, which "
      "is what edge classifier models with three inputs take as their \"edge_attr\" input. False (the default) "
      "computes none, which is what two-input models expect."};
  Gaudi::Property<std::vector<float>> m_edgeFeatureScales{
      this,
      "EdgeFeatureScales",
      {},
      "The four scales the edge features are computed with. They are always computed from r, phi, z and eta, so "
      "these are the scales of those four - in that order, whichever order the models take their own inputs in. The "
      "edge features are handed to the classifiers unscaled, so these have to be the scales the classifier was "
      "trained with. The phi scale has to be pi, which the dphi wrap-around assumes (as the ACORN training does), so "
      "this is required when ComputeEdgeFeatures is true and not read otherwise."};

  Gaudi::Property<std::vector<std::string>> m_edgeClassifierModelPath{
      this, "EdgeClassifierModelPath", {}, "List of paths to ONNX model files for edge classifier(s)."};
  Gaudi::Property<std::vector<std::vector<std::string>>> m_inputFeaturesEdgeClassifier{
      this, "InputFeaturesEdgeClassifier", std::vector<std::vector<std::string>>{{"r", "phi", "z", "t"}},
      "Node features for the edge classifier models, one list per model. Each model gets its features in the order "
      "they are listed in, and a feature may be listed more than once."};
  // double rather than float: Gaudi parses nested vectors of double, but not of float
  Gaudi::Property<std::vector<std::vector<double>>> m_inputScalesEdgeClassifier{
      this, "InputScalesEdgeClassifier", std::vector<std::vector<double>>{{1., 1., 1., 1.}},
      "Scales for the node features of the edge classifier models, one list per model, each feature is divided by "
      "its scale. Each list must be the same size as the corresponding InputFeaturesEdgeClassifier list (or empty for "
      "no scaling), and none may be zero."};
  Gaudi::Property<std::vector<float>> m_edgeClassifierCut{
      this, "EdgeClassifierCut", {0.5f}, "List of cut values to use for the edge classifiers"};
  Gaudi::Property<bool> m_detailedDebugOut{this, "DetailedDebugOut", false,
                                           "If true, this will print all pipeline inputs and outputs in full detail!"};

  Gaudi::Property<uint32_t> m_minHitsPerTrk{this, "MinHitsPerTrack", 3,
                                            "Minimum number of hits per track for it to be considered for the output"};

  Gaudi::Property<std::string> m_trackBuilding{
      this, "TrackBuilding", "connected-components",
      "Track building algorithm: \"connected-components\" emits every connected component of the classified graph as "
      "one candidate (Acts' BoostTrackBuilding, which ignores the edge scores); \"cc-and-walk\" additionally resolves "
      "the components that are not already a path by walking them along the best-scoring edges, as in the ExaTrkX / "
      "GNN4ITk pipeline."};
  Gaudi::Property<float> m_walkAddScore{
      this, "WalkAddScore", 0.6f,
      "\"cc-and-walk\" only: a neighbour whose edge scores above this is always followed, and the walk branches if "
      "several do."};
  Gaudi::Property<float> m_walkMinScore{
      this, "WalkMinScore", 0.1f,
      "\"cc-and-walk\" only: if no neighbour reaches WalkAddScore, the best one is followed if it scores above this, "
      "otherwise the walk stops. Must not be above WalkAddScore."};

  /// @name Kalman-fit configuration
  ///@{
  Gaudi::Property<bool> m_propagateBackward{this, "PropagateBackward", false, "Extrapolates tracks towards beamline."};
  Gaudi::Property<bool> m_extrapolateToCalo{
      this, "ExtrapolateToCalo", true,
      "Extrapolate fitted tracks to the calorimeter face and add an AtCalorimeter track state."};
  Gaudi::Property<bool> m_addEndcapCaloState{
      this, "AddEndcapCaloState", false,
      "Give a track that crosses both calorimeter sections one AtCalorimeter track state per section instead of a "
      "single one. A track entering the barrel face close to the barrel/endcap corner goes on to enter the endcap "
      "too; with this enabled it gets a second AtCalorimeter state there, appended after the barrel one (the states "
      "are ordered as the track crosses them). Off by default, since a consumer looking up 'the' AtCalorimeter state "
      "by location would silently see only the barrel one. Ignored unless ExtrapolateToCalo is set."};
  Gaudi::Property<double> m_initialTrackError_pos{this, "InitialTrackError_Pos", 10 * Acts::UnitConstants::um,
                                                  "Initial track error for local position."};
  Gaudi::Property<double> m_initialTrackError_phi{this, "InitialTrackError_Phi", 1 * Acts::UnitConstants::degree,
                                                  "Initial track error for phi."};
  Gaudi::Property<double> m_initialTrackError_relP{this, "InitialTrackError_RelP", 0.25,
                                                   "Initial track error for momentum (relative)."};
  Gaudi::Property<double> m_initialTrackError_lambda{this, "InitialTrackError_Lambda", 1 * Acts::UnitConstants::degree,
                                                     "Initial track error for lambda."};
  Gaudi::Property<double> m_initialTrackError_time{this, "InitialTrackError_Time", 100 * Acts::UnitConstants::ns,
                                                   "Initial track error for time."};
  ///@}

  Gaudi::Property<std::string> m_device{
      this, "Device", "cpu",
      "Device to run the GNN pipeline on: \"cpu\" or \"cuda\" (optionally \"cuda:<index>\"). Requires a "
      "CUDA-enabled onnxruntime/torch build for \"cuda\"."};

private:
  /// Construct the pipeline stages from the (already validated) configuration.
  /// Loads the ONNX models and throws if that or the pipeline setup fails.
  void buildPipeline(const std::vector<float>& embeddingScales, const std::vector<float>& edgeFeatureScales,
                     const std::vector<std::vector<float>>& edgeClassifierScales);

  std::vector<std::string> m_allHitFeatures{};
  std::vector<ACTSTracking::ResolvedFeature> m_resolvedHitFeatures{};
  std::vector<std::pair<double, double>> m_thetaBinEdges{};
  std::vector<std::pair<double, double>> m_phiBinEdges{};
  std::vector<int> m_embeddingFeatureIndices{};
  std::vector<int> m_edgeFeatureIndices{};
  std::vector<std::size_t> m_distanceFeatureIndices{};
  std::vector<std::vector<int>> m_edgeClassifierFeatureIndices{};
  std::unique_ptr<ActsPlugins::GnnPipeline> m_pipeline{nullptr};
  std::unique_ptr<const Acts::Logger> m_logger{nullptr};
  ActsPlugins::Device m_runDevice{ActsPlugins::Device::Type::eCPU, 0};

  /// CellID decoder, built once from the geometry service's encoding string
  /// (parsing it is too expensive to redo for every event / segment).
  std::optional<dd4hep::DDSegmentation::BitFieldCoder> m_cellIDDecoder{};

  SmartIF<IActsGeoSvc> m_actsGeoSvc{nullptr};

  /// Calorimeter-face extrapolation monitoring, updated by the per-event
  /// KFRunner and summarised in finalize().
  ACTSTracking::CaloExtrapMonitor m_caloMonitor{};

public:
  void registerCallBack(Gaudi::StateMachine::Transition, std::function<void()>) {}

private:
  mutable Gaudi::Accumulators::RootHistogram<3> m_monitoringHist{this,
                                                                 "MonitoringHistogram",
                                                                 "Monitoring histogram for GNN track finding",
                                                                 {100, 0., 100.},
                                                                 {100, 0., 100.},
                                                                 {100, 0., 100}};
};
