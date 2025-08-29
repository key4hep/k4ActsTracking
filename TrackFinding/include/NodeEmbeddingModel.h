#pragma once

#include "ONNXInferenceModel.h"

#include <edm4hep/TrackerHitPlaneCollection.h>

#include <string>
#include <vector>

namespace mlutils {
class NodeEmbeddingModel {
public:
  NodeEmbeddingModel(const std::string& name) : m_nodeEmbedding(name) {}

  ~NodeEmbeddingModel() = default;
  NodeEmbeddingModel(const NodeEmbeddingModel&) = delete;
  NodeEmbeddingModel& operator=(const NodeEmbeddingModel&) = delete;
  NodeEmbeddingModel(NodeEmbeddingModel&&) = default;
  NodeEmbeddingModel& operator=(NodeEmbeddingModel&&) = default;

  inline bool loadModel(const std::string& modelPath) { return m_nodeEmbedding.loadModel(modelPath); }

  static std::vector<std::vector<float>>
  extractInformation(const std::vector<const edm4hep::TrackerHitPlaneCollection*>& collections);

  inline std::vector<Ort::Value> runInference(const std::vector<std::vector<float>>& inputData) {
    return m_nodeEmbedding.runInference(inputData);
  }

  template <typename StreamT>
  void dumpModel(StreamT& stream) const {
    m_nodeEmbedding.dumpModel(stream);
  }

private:
  mlutils::ONNXInferenceModel m_nodeEmbedding;
};

} // namespace mlutils
