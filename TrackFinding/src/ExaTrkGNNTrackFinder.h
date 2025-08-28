#pragma once

#include "ONNXInferenceModel.h"

#include <k4FWCore/Transformer.h>

#include <Gaudi/Property.h>

#include <edm4hep/TrackCollection.h>
#include <edm4hep/TrackerHitPlaneCollection.h>

#include <string>
#include <vector>

struct ExaTrkGNNTrackFinder : public k4FWCore::Transformer<edm4hep::TrackCollection(
                                  std::vector<const edm4hep::TrackerHitPlaneCollection*> const&)> {

  ExaTrkGNNTrackFinder(const std::string& name, ISvcLocator* svcLoc);

  StatusCode initialize() override;

  edm4hep::TrackCollection operator()(std::vector<const edm4hep::TrackerHitPlaneCollection*> const&) const override;

  Gaudi::Property<std::string> m_edgeClassifierModelPath{this, "EdgeClassifierModelPath",
                                                         "Path to the ONNX model file for the edge classifier GNN"};
  Gaudi::Property<std::string> m_nodeEmbeddingModelPath{
      this, "NodeEmbeddingModelPath",
      "Path to the ONNX model file for the node embedding / graph construction metric model"};

private:
  mlutils::ONNXInferenceModel m_edgeClassifier;
  mlutils::ONNXInferenceModel m_nodeEmbedding;
};
