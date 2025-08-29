#!/usr/bin/env python3

import numpy as np
import uproot
import argparse
import sys


def load_python_outputs(npz_file):
    """
    Load the outputs from the Python inference (NPZ format).

    Args:
        npz_file (str): Path to the NPZ file produced by embedding_model_inference.py

    Returns:
        tuple: (input_features, output_features) as numpy arrays
    """
    data = np.load(npz_file)
    input_features = data["input_features"]
    output_features = data["output_features"]
    return input_features, output_features


def load_cpp_outputs(root_file):
    """
    Load the outputs from the C++ inference (ROOT format) using uproot.

    Args:
        root_file (str): Path to the ROOT file produced by embedding_model_inference.cpp

    Returns:
        tuple: (input_features, output_features) as numpy arrays
    """
    try:
        # Try uproot first
        with uproot.open(root_file) as file:
            tree = file["models"]

            # Read the embedding inputs and outputs
            embedding_inputs = tree["embeddingInputs"].array(library="np")[0]
            embedding_outputs = tree["embeddingOutputs"].array(library="np")[0]

            # Convert to numpy arrays
            input_features = np.array(embedding_inputs, dtype=np.float32)
            output_features = np.array(embedding_outputs, dtype=np.float32)

        return input_features, output_features

    except Exception as uproot_error:
        print(f"Warning: uproot failed to read file ({uproot_error})")
        print("Trying alternative approach with PyROOT...")

        # Fallback to PyROOT if available
        import ROOT

        file = ROOT.TFile.Open(root_file, "READ")
        tree = file.Get("models")
        tree.GetEntry(0)  # Read first (and only) entry

        embedding_inputs_data = []
        embedding_outputs_data = []
        embedding_inputs_vec = getattr(tree, "embeddingInputs")
        embedding_outputs_vec = getattr(tree, "embeddingOutputs")

        # Convert ROOT vectors to Python lists, then to numpy arrays
        for i in range(embedding_inputs_vec.size()):
            row = []
            for j in range(embedding_inputs_vec[i].size()):
                row.append(embedding_inputs_vec[i][j])
            embedding_inputs_data.append(row)

        for i in range(embedding_outputs_vec.size()):
            row = []
            for j in range(embedding_outputs_vec[i].size()):
                row.append(embedding_outputs_vec[i][j])
            embedding_outputs_data.append(row)

        # Convert to numpy arrays
        input_features = np.array(embedding_inputs_data, dtype=np.float32)
        output_features = np.array(embedding_outputs_data, dtype=np.float32)

        file.Close()
        return input_features, output_features


def compare_arrays(arr1, arr2, name, rtol=1e-5, atol=1e-8):
    """
    Compare two arrays and report if they are numerically close.

    Args:
        arr1, arr2 (np.ndarray): Arrays to compare
        name (str): Name of the arrays being compared (for reporting)
        rtol (float): Relative tolerance for numpy.allclose
        atol (float): Absolute tolerance for numpy.allclose

    Returns:
        bool: True if arrays are close, False otherwise
    """
    print(f"\nComparing {name}:")
    print(f"  Shape 1: {arr1.shape}, Shape 2: {arr2.shape}")
    print(f"  Dtype 1: {arr1.dtype}, Dtype 2: {arr2.dtype}")

    if arr1.shape != arr2.shape:
        print(f"  ❌ FAIL: Shapes do not match!")
        return False

    # Check if arrays are numerically close
    are_close = np.allclose(arr1, arr2, rtol=rtol, atol=atol)

    if are_close:
        print(f"  ✅ PASS: Arrays are numerically close (rtol={rtol}, atol={atol})")
    else:
        print(f"  ❌ FAIL: Arrays are NOT numerically close")

        # Provide additional diagnostic information
        diff = np.abs(arr1 - arr2)
        max_diff = np.max(diff)
        mean_diff = np.mean(diff)

        print(f"    Max absolute difference: {max_diff}")
        print(f"    Mean absolute difference: {mean_diff}")

        # Show some examples of differences
        if diff.size > 0:
            flat_diff = diff.flatten()
            flat_arr1 = arr1.flatten()
            flat_arr2 = arr2.flatten()

            # Find indices of largest differences
            worst_indices = np.argsort(flat_diff)[-5:]  # Top 5 worst differences
            print(f"    Worst differences (showing up to 5):")
            for i, idx in enumerate(worst_indices):
                print(
                    f"      [{idx}]: {flat_arr1[idx]} vs {flat_arr2[idx]} (diff: {flat_diff[idx]})"
                )

    return are_close


def main():
    """
    Main function to validate that Python and C++ inference produce the same results.
    """
    parser = argparse.ArgumentParser(
        description="Validate that Python and C++ embedding model inference produce identical results"
    )
    parser.add_argument("python_file", help="Path to NPZ file from Python inference")
    parser.add_argument("cpp_file", help="Path to ROOT file from C++ inference")
    parser.add_argument(
        "--rtol",
        type=float,
        default=1e-5,
        help="Relative tolerance for comparison (default: 1e-5)",
    )
    parser.add_argument(
        "--atol",
        type=float,
        default=1e-8,
        help="Absolute tolerance for comparison (default: 1e-8)",
    )

    args = parser.parse_args()

    try:
        # Load outputs from both implementations
        print("Loading Python inference outputs...")
        py_inputs, py_outputs = load_python_outputs(args.python_file)

        print("Loading C++ inference outputs...")
        cpp_inputs, cpp_outputs = load_cpp_outputs(args.cpp_file)

    except Exception as e:
        print(f"Error loading files: {str(e)}")
        sys.exit(1)

    # Compare inputs and outputs
    print("=" * 60)
    print("VALIDATION RESULTS")
    print("=" * 60)

    inputs_match = compare_arrays(
        py_inputs, cpp_inputs, "Input Features", args.rtol, args.atol
    )
    outputs_match = compare_arrays(
        py_outputs, cpp_outputs, "Output Features", args.rtol, args.atol
    )

    # Final summary
    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)

    if inputs_match and outputs_match:
        print(
            "🎉 SUCCESS: All arrays match! Python and C++ implementations are consistent."
        )
        sys.exit(0)
    else:
        print(
            "💥 FAILURE: Arrays do not match. There may be differences between implementations."
        )
        sys.exit(1)


if __name__ == "__main__":
    main()
