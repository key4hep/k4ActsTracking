#include "OnnxMetricLearning.h"
#include "ONNXInferenceModel.h"

#include "Acts/Plugins/Gnn/detail/TensorVectorConversion.hpp"
#include "Acts/Plugins/Gnn/detail/buildEdges.hpp"

#include <onnxruntime_cxx_api.h>

#include <torch/torch.h>

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <fmt/ranges.h>

#include <cassert>
#include <memory>
#include <span>
#include <vector>

namespace {
/// Conver the Acts log level into an ONNX one
constexpr OrtLoggingLevel getOnnxLogLevel(Acts::Logging::Level lvl) {
  switch (lvl) {
  case Acts::Logging::VERBOSE:
    return ORT_LOGGING_LEVEL_VERBOSE;
  case Acts::Logging::DEBUG:
    return ORT_LOGGING_LEVEL_INFO;
  case Acts::Logging::INFO:
  case Acts::Logging::WARNING:
    return ORT_LOGGING_LEVEL_WARNING;
  case Acts::Logging::ERROR:
    return ORT_LOGGING_LEVEL_ERROR;
  case Acts::Logging::FATAL:
    return ORT_LOGGING_LEVEL_FATAL;
  case Acts::Logging::MAX:
    return ORT_LOGGING_LEVEL_WARNING;
  }
  return ORT_LOGGING_LEVEL_WARNING;
}

/// Convert ONNX element type to torch scalar type
constexpr torch::ScalarType toTorchType(ONNXTensorElementDataType elementType) {
  switch (elementType) {
  case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT:
    return torch::kFloat32;
  case ONNX_TENSOR_ELEMENT_DATA_TYPE_DOUBLE:
    return torch::kFloat64;
  case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT32:
    return torch::kInt32;
  case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64:
    return torch::kInt64;
  default:
    throw std::runtime_error("Unsupported ONNX tensor element type");
  }
}

/// Convert the Onnx tensor into a torch Tensor.
/// @note: This doesn't take ownership, it essentially just "re-skins" the data
/// owned by the Onnx tensor
torch::Tensor toTorchTensor(const Ort::Value& onnxTensor) {
  auto tensorInfo = onnxTensor.GetTensorTypeAndShapeInfo();
  auto shape = tensorInfo.GetShape();
  auto elementType = tensorInfo.GetElementType();

  std::vector<int64_t> torchShape(shape.begin(), shape.end());

  const void* data = onnxTensor.GetTensorData<void>();
  const auto torchType = toTorchType(elementType);

  // Create torch tensor from existing data without copying
  return torch::from_blob(const_cast<void*>(data), torchShape, torchType);
}

} // namespace

OnnxMetricLearning::OnnxMetricLearning(const Config& cfg, std::unique_ptr<const Acts::Logger> logger)
    : m_model("MetricLearning", getOnnxLogLevel(logger->level())), m_config(cfg), m_logger(std::move(logger)) {
  m_model.loadModel(config().modelPath);
}

Acts::PipelineTensors OnnxMetricLearning::operator()(std::vector<float>& inputValues, std::size_t numNodes,
                                                     const std::vector<uint64_t>&,
                                                     const Acts::ExecutionContext& execContext) {

  assert(inputValues.size() % numNodes == 0);
  std::vector inputShape = {static_cast<int64_t>(numNodes), static_cast<int64_t>(inputValues.size() / numNodes)};
  ACTS_DEBUG(fmt::format("Embedding input tensor shape: {}", inputShape));
  ACTS_VERBOSE(fmt::format("First input space point: {}", std::span(inputValues.data(), inputShape[1])));

  const auto outputs = m_model.runInference(inputValues, inputShape);
  auto embeddedPoints = toTorchTensor(outputs[0]);
  assert(embeddedPoints.size(0) == inputShape[0]); // Do not change the number of points
  assert(embeddedPoints.size(1) == config().embeddingDim);
  ACTS_VERBOSE(fmt::format("Embedding space of first SP: {}", fmt::streamed(embeddedPoints.slice(0, 0, 1))));

  auto edgeList = Acts::detail::buildEdges(embeddedPoints, m_config.rVal, m_config.knnVal, m_config.shuffleDirections);

  ACTS_VERBOSE(fmt::format("Shape of built edges: ({}, {})", edgeList.size(0), edgeList.size(1)));
  ACTS_VERBOSE(fmt::format("Slice of edgeList: {}", fmt::streamed(edgeList.slice(1, 0, 5))));

  return {
      Acts::detail::torchToActsTensor<float>(Acts::detail::vectorToTensor2D(inputValues, inputShape[1]), execContext),
      Acts::detail::torchToActsTensor<int64_t>(edgeList, execContext), std::nullopt, std::nullopt};
}
