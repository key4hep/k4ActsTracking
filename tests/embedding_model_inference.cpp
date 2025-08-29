#include "NodeEmbeddingModel.h"

#include "edm4hep/TrackerHitPlaneCollection.h"
#include "podio/Reader.h"

#include "TFile.h"
#include "TTree.h"

#include <iostream>

int main(int argc, char* argv[]) {
  if (argc != 4) {
    std::cerr << "Usage: " << argv[0] << " <onnx-model-file.onnx> <input_file.root> <output_file.root>" << std::endl;
    return 1;
  }

  auto embeddingModel = mlutils::NodeEmbeddingModel("embeddingModel");
  embeddingModel.loadModel(argv[1]);

  auto reader = podio::makeReader(argv[2]);
  auto event = reader.readEvent(0);

  const auto& hits = event.get<edm4hep::TrackerHitPlaneCollection>("IBTrackerHits");
  auto embeddingInputs = mlutils::NodeEmbeddingModel::extractInformation({&hits});

  const auto embeddingOutput = embeddingModel.runInference(embeddingInputs);

  auto outputFile = std::make_unique<TFile>(argv[3], "RECREATE");
  TTree* tree = new TTree("models", "Embedding model inputs and outputs");

  auto* embeddingInputsPtr = &embeddingInputs;
  tree->Branch("embeddingInputs", &embeddingInputsPtr);

  // Convert Ort::Value outputs back to vector<vector<float>> with same shape as inputs
  std::vector<std::vector<float>> embeddingOutputsConverted;
  embeddingOutputsConverted.reserve(embeddingInputs.size());

  if (!embeddingOutput.empty()) {
    const auto* outputData = embeddingOutput[0].GetTensorData<float>();
    const auto outputShape = embeddingOutput[0].GetTensorTypeAndShapeInfo().GetShape();

    const auto numRows = outputShape[0];
    const auto numCols = outputShape[1];

    for (int i = 0; i < numRows; ++i) {
      std::vector<float> row(outputData + i * numCols, outputData + (i + 1) * numCols);
      embeddingOutputsConverted.emplace_back(std::move(row));
    }
  }
  auto* embeddingOutputPtr = &embeddingOutputsConverted;
  tree->Branch("embeddingOutputs", &embeddingOutputPtr);

  tree->Fill();

  // Write and close the file
  tree->Write();
  outputFile->Close();

  return 0;
}
