#!/usr/bin/env python3

import ROOT
import argparse
import sys


def main():
    parser = argparse.ArgumentParser(description="Plot step_r vs step_z from ROOT file")
    parser.add_argument("input_file", help="Input ROOT file")
    parser.add_argument(
        "-o", "--output", default="step_plot.png", help="Output plot file"
    )
    args = parser.parse_args()

    # Enable multi-threading for RDataFrame
    ROOT.EnableImplicitMT()

    # Open the ROOT file and create RDataFrame
    try:
        df = ROOT.RDataFrame("events", args.input_file)
    except Exception as e:
        print(f"Error opening file {args.input_file}: {e}")
        sys.exit(1)

    # Define step_r as sqrt(step_x^2 + step_y^2)
    df = df.Define("step_r", "sqrt(step_x*step_x + step_y*step_y)")

    # Create the scatter plot
    hist = df.Graph("step_z", "step_r")

    # Create canvas and draw
    canvas = ROOT.TCanvas("canvas", "Step R vs Z", 800, 600)
    # hist.SetMarkerStyle(20)
    # hist.SetMarkerSize(0.5)
    hist.SetTitle("Step R vs Z;step_z;step_r")
    hist.Draw("AP")

    # Set axis limits
    hist.GetXaxis().SetRangeUser(-2000, 2000)
    hist.GetYaxis().SetRangeUser(0, 1000)

    # Save the plot
    canvas.SaveAs(args.output)
    print(f"Plot saved as {args.output}")


if __name__ == "__main__":
    main()
