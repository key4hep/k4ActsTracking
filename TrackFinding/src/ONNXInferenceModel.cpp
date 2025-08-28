#include "ONNXInferenceModel.h"

#include <iostream>
#include <stdexcept>

namespace mlutils {

ONNXInferenceModel::ONNXInferenceModel(const std::string& name)
    : m_env(std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, name.c_str())),
      m_sessionOptions(std::make_unique<Ort::SessionOptions>()), m_envName(name) {
  // Run model on a single CPU core
  m_sessionOptions->SetIntraOpNumThreads(1);
  // Let ONNX perform optimizations on the model graph to improve performance
  m_sessionOptions->SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
}

bool ONNXInferenceModel::loadModel(const std::string& modelPath) {
  try {
    cleanup();

    m_session = std::make_unique<Ort::Session>(*m_env, modelPath.c_str(), *m_sessionOptions);
    extractModelInfo();
    m_modelLoaded = true;

    return true;
  } catch (const std::exception& e) {
    std::cerr << "Error loading ONNX model: " << e.what() << std::endl;
    m_modelLoaded = false;
    return false;
  }
}

std::vector<float> ONNXInferenceModel::runInference(const std::vector<float>& inputData,
                                                    const std::vector<int64_t>& inputShape) {
  if (!m_modelLoaded) {
    throw std::runtime_error("Model not loaded");
  }

  try {
    // Create input tensor
    auto memoryInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    // We re-use the buffer provided by the inputData
    auto inputTensor = Ort::Value::CreateTensor(memoryInfo, const_cast<float*>(inputData.data()), inputData.size(),
                                                inputShape.data(), inputShape.size());

    // Prepare input/output names
    std::vector<const char*> inputNames;
    std::vector<const char*> outputNames;

    for (const auto& name : m_inputNames) {
      inputNames.push_back(name.c_str());
    }
    for (const auto& name : m_outputNames) {
      outputNames.push_back(name.c_str());
    }

    // Run inference
    auto outputTensors = m_session->Run(Ort::RunOptions{nullptr}, inputNames.data(), &inputTensor, 1,
                                        outputNames.data(), outputNames.size());

    // Extract output data
    float* outputData = outputTensors[0].GetTensorMutableData<float>();
    size_t outputSize = outputTensors[0].GetTensorTypeAndShapeInfo().GetElementCount();

    return std::vector<float>(outputData, outputData + outputSize);

  } catch (const std::exception& e) {
    throw std::runtime_error("Inference failed: " + std::string(e.what()));
  }
}

void ONNXInferenceModel::extractModelInfo() {
  if (!m_session) {
    return;
  }

  // Clear existing info
  m_inputNames.clear();
  m_outputNames.clear();
  m_inputShapes.clear();
  m_outputShapes.clear();

  Ort::AllocatorWithDefaultOptions allocator;

  // Extract input information
  size_t inputCount = m_session->GetInputCount();
  for (size_t i = 0; i < inputCount; ++i) {
    auto inputName = m_session->GetInputNameAllocated(i, allocator);
    m_inputNames.push_back(std::string(inputName.get()));

    auto inputTypeInfo = m_session->GetInputTypeInfo(i);
    auto tensorInfo = inputTypeInfo.GetTensorTypeAndShapeInfo();
    auto shape = tensorInfo.GetShape();
    m_inputShapes.push_back(shape);
  }

  // Extract output information
  size_t outputCount = m_session->GetOutputCount();
  for (size_t i = 0; i < outputCount; ++i) {
    auto outputName = m_session->GetOutputNameAllocated(i, allocator);
    m_outputNames.push_back(std::string(outputName.get()));

    auto outputTypeInfo = m_session->GetOutputTypeInfo(i);
    auto tensorInfo = outputTypeInfo.GetTensorTypeAndShapeInfo();
    auto shape = tensorInfo.GetShape();
    m_outputShapes.push_back(shape);
  }
}

void ONNXInferenceModel::cleanup() {
  m_session.reset();
  m_inputNames.clear();
  m_outputNames.clear();
  m_inputShapes.clear();
  m_outputShapes.clear();
  m_modelLoaded = false;
}

} // namespace mlutils
