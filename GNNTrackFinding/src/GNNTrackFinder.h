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

#include <k4ActsTracking/IActsGeoSvc.h>

#include <Acts/Definitions/Units.hpp>
#include <Acts/Utilities/Logger.hpp>

#include <GaudiKernel/SmartIF.h>
#if __has_include("ActsPlugins/Gnn/GnnPipeline.hpp")
#include <ActsPlugins/Gnn/GnnPipeline.hpp>
#else
#include <Acts/Plugins/Gnn/GnnPipeline.hpp>
namespace ActsPlugins {
  using GnnPipeline = Acts::GnnPipeline;
  using Device      = Acts::Device;
}  // namespace ActsPlugins
#endif

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
  /// The per-hit quantity a configured feature name maps to. All CellID based
  /// features share one enumerator and are distinguished by the decoder field
  /// index stored alongside it.
  enum class HitFeature { X, Y, Z, R, Phi, Theta, Eta, Time, Energy, CellIdField };

  /// A configured input feature, resolved once in initialize() to the quantity
  /// that has to be read from a hit. Keeping the resolution out of the event
  /// loop avoids per-hit string comparisons and CellID field name lookups.
  struct ResolvedFeature {
    HitFeature  kind{};
    std::size_t cellIdField{0};  ///< only used for HitFeature::CellIdField
  };

  GNNTrackFinder(const std::string& name, ISvcLocator* svcLoc);

  StatusCode initialize() override;

  edm4hep::TrackCollection     operator()(std::vector<const edm4hep::TrackerHitPlaneCollection*> const&) const override;
  Gaudi::Property<std::size_t> m_thetaBins{this, "ThetaBins", 1, "Number of theta bins for segmentation."};
  Gaudi::Property<std::size_t> m_phiBins{this, "PhiBins", 1, "Number of phi bins for segmentation."};
  Gaudi::Property<double>      m_thetaOverlap{this, "ThetaOverlap", 0.0,
                                         "Fractional theta overlap for segmentation (fraction of bin width)."};
  Gaudi::Property<double>      m_phiOverlap{this, "PhiOverlap", 0.0,
                                       "Fractional phi overlap for segmentation (fraction of bin width)."};

  Gaudi::Property<std::string> m_nodeEmbeddingModelPath{
      this, "NodeEmbeddingModelPath", "",
      "Path to the ONNX model file for the node embedding / graph construction metric model"};
  Gaudi::Property<float>       m_edgeBuildingRadius{this, "EdgeBuildingRadius", 0.1f,
                                              "The radius parameter for the KD-Tree that is used in edge building"};
  Gaudi::Property<float>       m_edgeBuildingKnn{this, "EdgeBuildingKnn", 100.f,
                                           "The KNN parameter for the KD-Tree that is used in edge building"};
  Gaudi::Property<std::string> m_inputFeaturesEmbedding{
      this, "InputFeaturesEmbedding", "r,phi,z,t",
      "Comma-separated list of hit features for the node embedding model."};
  Gaudi::Property<std::string> m_inputScalesEmbedding{
      this, "InputScalesEmbedding", "1,1,1,1",
      "Comma-separated list of scales for the hit features for the node embedding model. "
      "Must be same size as InputFeaturesEmbedding."};
  Gaudi::Property<int> m_embeddingDim{this, "EmbeddingDim", 4, "The embedding dimension for the node embedding model"};

  Gaudi::Property<std::vector<std::string>> m_edgeClassifierModelPath{
      this, "EdgeClassifierModelPath", {}, "List of paths to ONNX model files for edge classifier(s)."};
  Gaudi::Property<std::vector<std::string>> m_inputFeaturesEdgeClassifier{
      this,
      "InputFeaturesEdgeClassifier",
      {"r,phi,z,t"},
      "List of comma-separated lists of node features for the edge classifier models."};
  Gaudi::Property<std::vector<std::string>> m_inputScalesEdgeClassifier{
      this,
      "InputScalesEdgeClassifier",
      {"1,1,1,1"},
      "List of comma-separated lists of scales for the node features for the edge classifier models. "
      "Must be same size as InputFeaturesEdgeClassifier."};
  Gaudi::Property<std::vector<float>> m_edgeClassifierCut{
      this, "EdgeClassifierCut", {0.5f}, "List of cut values to use for the edge classifiers"};
  Gaudi::Property<bool> m_detailedDebugOut{this, "DetailedDebugOut", false,
                                           "If true, this will print all pipeline inputs and outputs in full detail!"};

  Gaudi::Property<uint32_t> m_minHitsPerTrk{this, "MinHitsPerTrack", 3,
                                            "Minimum number of hits per track for it to be considered for the output"};

  /// @name Kalman-fit configuration
  ///@{
  Gaudi::Property<bool> m_propagateBackward{this, "PropagateBackward", false, "Extrapolates tracks towards beamline."};
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
  void buildPipeline(const std::vector<float>&              embeddingScales,
                     const std::vector<std::vector<float>>& edgeClassifierScales);

  std::vector<std::string>                  m_allHitFeatures{};
  std::vector<ResolvedFeature>              m_resolvedHitFeatures{};
  std::vector<std::pair<double, double>>    m_thetaBinEdges{};
  std::vector<std::pair<double, double>>    m_phiBinEdges{};
  std::vector<int>                          m_embeddingFeatureIndices{};
  std::vector<std::vector<int>>             m_edgeClassifierFeatureIndices{};
  std::unique_ptr<ActsPlugins::GnnPipeline> m_pipeline{nullptr};
  std::unique_ptr<const Acts::Logger>       m_logger{nullptr};
  ActsPlugins::Device                       m_runDevice{ActsPlugins::Device::Type::eCPU, 0};

  /// CellID decoder, built once from the geometry service's encoding string
  /// (parsing it is too expensive to redo for every event / segment).
  std::optional<dd4hep::DDSegmentation::BitFieldCoder> m_cellIDDecoder{};

  SmartIF<IActsGeoSvc> m_actsGeoSvc{nullptr};

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
