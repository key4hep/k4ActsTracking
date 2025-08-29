#!/usr/bin/env python3

import onnxruntime as ort
import numpy as np
import os
import argparse
import podio


def load_onnx_model(model_path):
    """
    Load an ONNX model from file and return the inference session.

    Args:
        model_path (str): Path to the ONNX model file

    Returns:
        ort.InferenceSession: The loaded ONNX inference session

    Raises:
        FileNotFoundError: If the model file doesn't exist
        Exception: If the model fails to load
    """
    if not os.path.exists(model_path):
        raise FileNotFoundError(f"Model file not found: {model_path}")

    try:
        # Create inference session with CPU provider
        session = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])
        return session
    except Exception as e:
        raise Exception(f"Failed to load ONNX model from {model_path}: {str(e)}")


def extract_hit_features(tracker_hits):
    """
    Extract position (x,y,z) and time information from tracker hit collection.

    Args:
        tracker_hits (edm4hep.TrackerHitPlaneCollection): Collection of tracker hits

    Returns:
        np.ndarray: Array with shape (n_hits, 4) containing [x, y, z, time] for each hit
    """
    hit_data = []

    for hit in tracker_hits:
        position = hit.getPosition()
        time = hit.getTime()
        hit_data.append([position.x, position.y, position.z, time])

    return np.array(hit_data, dtype=np.float32)


def get_model_info(session):
    """
    Extract basic information about the loaded model.

    Args:
        session (ort.InferenceSession): The ONNX inference session

    Returns:
        dict: Dictionary containing model input/output information
    """
    inputs = session.get_inputs()
    outputs = session.get_outputs()

    input_info = []
    for inp in inputs:
        input_info.append({"name": inp.name, "type": inp.type, "shape": inp.shape})

    output_info = []
    for out in outputs:
        output_info.append({"name": out.name, "type": out.type, "shape": out.shape})

    return {"inputs": input_info, "outputs": output_info}


def print_model_info(model_info, model_path):
    """
    Print model information in a formatted way.

    Args:
        model_info (dict): Dictionary containing model input/output information
    """
    print(f"Model Information (loaded from {model_path}):")
    print("Inputs:")
    for inp in model_info["inputs"]:
        print(f"  - Name: {inp['name']}, Type: {inp['type']}, Shape: {inp['shape']}")

    print("Outputs:")
    for out in model_info["outputs"]:
        print(f"  - Name: {out['name']}, Type: {out['type']}, Shape: {out['shape']}")


def main():
    """
    Main function to demonstrate ONNX model loading and edm4hep file reading.
    """
    parser = argparse.ArgumentParser(
        description="Validate and inspect ONNX embedding models"
    )
    parser.add_argument("model_path", help="Path to the ONNX model file")
    parser.add_argument(
        "edm4hep_file", help="Path to edm4hep file to read IBTrackerHits from"
    )
    parser.add_argument(
        "-o",
        "--output-file",
        help="File to which the output tensors will be written",
        default="embedding_outputs.npz",
    )
    args = parser.parse_args()

    # Load ONNX model
    session = load_onnx_model(args.model_path)
    model_info = get_model_info(session)
    print_model_info(model_info, args.model_path)

    # Get one of the tracker hit collections from the input file and extract the
    # features in the same form as we do in c++
    reader = podio.root_io.Reader(args.edm4hep_file)
    events = reader.get("events")
    event = events[0]
    ib_tracker_hits = event.get("IBTrackerHits")

    hit_features = extract_hit_features(ib_tracker_hits)

    # Run inference with the loaded model
    print(f"\nRunning inference on {len(hit_features)} hits...")
    print(f"Input shape: {hit_features.shape}")

    input_name = model_info["inputs"][0]["name"]
    input_data = hit_features.astype(np.float32)

    outputs = session.run(None, {input_name: input_data})
    print(f"Number of outputs: {len(outputs)}")
    for i, output in enumerate(outputs):
        print(f"Output {i} shape: {output.shape}")
        print(f"Output {i} type: {output.dtype}")

    output_dict = {}

    output_dict["input_features"] = input_data
    output_dict["output_features"] = outputs[0]

    np.savez(args.output_file, **output_dict)


if __name__ == "__main__":
    main()
