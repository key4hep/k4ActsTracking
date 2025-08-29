#!/usr/bin/env sh

model_file=${MODEL_DIR}/graph_construction-MetricLearning.onnx
input_file=${TEST_BASE_DIR}/../inputs/multiMuonGun_reco_160.edm4hep.root

python3 ${TEST_BASE_DIR}/embedding_model_inference.py \
    ${model_file} \
    ${input_file} \
    --output-file python_inference_outputs.npz

./embedding_model_inference \
    ${model_file} \
    ${input_file} \
    cpp_inference_outputs.root

python3 ${TEST_BASE_DIR}/embedding_model_validation.py \
    python_inference_outputs.npz \
    cpp_inference_outputs.root
