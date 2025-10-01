#pragma once

#include <k4FWCore/Transformer.h>

#include <Acts/Plugins/Gnn/GnnPipeline.hpp>
#include <Acts/Utilities/Logger.hpp>

#include <Gaudi/Accumulators/RootHistogram.h>
#include <Gaudi/Property.h>

#include <edm4hep/TrackCollection.h>
#include <edm4hep/TrackerHitPlaneCollection.h>

#include <functional>
#include <memory>
#include <string>
#include <vector>

struct ExaTrkGNNTrackFinder : public k4FWCore::Transformer<edm4hep::TrackCollection(
                                  std::vector<const edm4hep::TrackerHitPlaneCollection*> const&)> {

  ExaTrkGNNTrackFinder(const std::string& name, ISvcLocator* svcLoc);

  StatusCode initialize() override;

  edm4hep::TrackCollection operator()(std::vector<const edm4hep::TrackerHitPlaneCollection*> const&) const override;

  Gaudi::Property<std::string> m_nodeEmbeddingModelPath{
      this, "NodeEmbeddingModelPath",
      "Path to the ONNX model file for the node embedding / graph construction metric model"};
  Gaudi::Property<float> m_edgeBuildingRadius{this, "EdgeBuildingRadius", 0.1f,
                                              "The radius parameter for the KD-Tree that is used in edge building"};
  Gaudi::Property<float> m_edgeBuildingKnn{this, "EdgeBuildingKnn", 100.f,
                                           "The KNN parameter for the KD-Tree that is used in edge building"};
  Gaudi::Property<int> m_embeddingDim{this, "EmbeddingDim", 4, "The embedding dimension for the node embedding model"};

  Gaudi::Property<std::string> m_edgeClassifierModelPath{this, "EdgeClassifierModelPath",
                                                         "Path to the ONNX model file for the edge classifier GNN"};
  Gaudi::Property<float> m_edgeClassifierCut{this, "EdgeClassifierCut", 0.5f,
                                             "Cut value to use for the edge classifier GNN"};

  Gaudi::Property<uint32_t> m_minHitsPerTrk{this, "MinHitsPerTrack", 3,
                                            "Minimum number of hits per track for it to be considered for the output"};

private:
  std::unique_ptr<Acts::GnnPipeline> m_pipeline{nullptr};
  std::unique_ptr<const Acts::Logger> m_logger{nullptr};

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
