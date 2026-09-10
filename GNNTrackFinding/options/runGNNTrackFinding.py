#!/usr/bin/env python3
#
# Copyright (c) 2014-2024 Key4hep-Project.
#
# This file is part of Key4hep.
# See https://key4hep.github.io/key4hep-doc/ for further info.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

from pathlib import Path

from k4FWCore import IOSvc, ApplicationMgr
from k4FWCore.parseArgs import parser
from Configurables import ActsGeoSvc, EventDataSvc, GeoSvc, GNNTrackFinder
from Configurables import Gaudi__Histograming__Sink__Root as RootHistoSink

from Gaudi.Configuration import INFO

# Example configuration for the GNN based track finding.
#
# NOTE: The values below are an example of how to configure the algorithm, they
# are not a tuned (or even meaningful) tracking configuration! In particular the
# input features, their scales and the segmentation have to match the models
# that are passed via --modelBase.

parser.add_argument(
    "--compactFile",
    help="The geometry compact file that is used to build the ACTS geometry",
    type=str,
    required=True,
)
parser.add_argument(
    "--modelBase", help="The base directory for the model", default=".", type=Path
)
parser.add_argument(
    "--device",
    help="Device to run the GNN pipeline on: 'cpu' or 'cuda' (optionally 'cuda:<index>')",
    default="cpu",
    type=str,
)
parser.add_argument(
    "--inputFile", help="The input file with the tracker hits", default="edm4hep.root"
)
parser.add_argument(
    "--outputFile", help="The output file", default="track_candidates.root"
)
parser.add_argument(
    "--monitoring",
    help="Fill (and write out) the monitoring histograms",
    action="store_true",
)
parser.add_argument(
    "--monitoringOutputFile",
    help="File into which the monitoring histograms should go (if enabled)",
    default="gnn_tracking_monitoring_hists.root",
    type=Path,
)

args = parser.parse_known_args()[0]

io_svc = IOSvc()
io_svc.Input = args.inputFile
io_svc.Output = args.outputFile

# The GNN track finder fits its candidates with the ACTS Kalman fitter, so it
# needs the ACTS geometry (and with it the DD4hep geometry)
geo_svc = GeoSvc("GeoSvc", detectors=[args.compactFile], EnableGeant4Geo=False)
acts_geo_svc = ActsGeoSvc("ActsGeoSvc")

TrackFinder = GNNTrackFinder(
    "GNNTrackFinder",
    # --- Graph construction (metric learning) --------------------------------
    NodeEmbeddingModelPath=str(
        args.modelBase / "graph_construction-MetricLearning.onnx"
    ),
    # The features (and their scales) the embedding model has been trained with.
    # Supported names: x, y, z, r, phi, theta, eta, t (time), E (energy),
    # module_id, layer_id, system_id
    InputFeaturesEmbedding="r,phi,z,t",
    InputScalesEmbedding="1,1,1,1",
    # If the embedding model was exported with a fixed-size input, pad the hits
    # of each segment with all-zero rows up to that length (0 = no padding).
    EmbeddingFixedInputLength=0,
    EdgeBuildingRadius=0.1,
    EdgeBuildingKnn=100.0,
    # --- Edge classification -------------------------------------------------
    # All four properties below are parallel lists with one entry per model, so
    # that several edge classifiers can be chained.
    EdgeClassifierModelPath=[
        str(args.modelBase / "edge_classifier-InteractionGNN.onnx")
    ],
    InputFeaturesEdgeClassifier=["r,phi,z,t"],
    InputScalesEdgeClassifier=["1,1,1,1"],
    EdgeClassifierCut=[0.5],
    # --- Hit segmentation ----------------------------------------------------
    # The hits can be split into (theta, phi) segments that are run through the
    # pipeline independently. The overlaps (as a fraction of the bin width) put
    # hits close to a bin boundary into both adjacent bins.
    ThetaBins=1,
    PhiBins=1,
    ThetaOverlap=0.0,
    PhiOverlap=0.0,
    # --- Track candidates ----------------------------------------------------
    MinHitsPerTrack=3,
    PropagateBackward=False,
    Device=args.device,
    # Set to True (together with OutputLevel=DEBUG) to dump all pipeline inputs
    # and outputs. This is *very* verbose.
    DetailedDebugOut=False,
    OutputLevel=INFO,
    InputHitCollections=[
        "IBTrackerHits",
        "IETrackerHits",
        "OBTrackerHits",
        "OETrackerHits",
        "VBTrackerHits",
        "VETrackerHits",
    ],
    OutputTrackCandidates=["GNNTrackCands"],
)

svcList = [geo_svc, acts_geo_svc, EventDataSvc()]

if args.monitoring:
    histSvc = RootHistoSink()
    histSvc.FileName = str(args.monitoringOutputFile)
    svcList.append(histSvc)

    TrackFinder.MonitoringHistogram_Axis0 = (100, 0.0, 20000.0, "nInputHits")
    TrackFinder.MonitoringHistogram_Axis1 = (100, 0.0, 20000.0, "nTrackCandidates")
    TrackFinder.MonitoringHistogram_Axis2 = (100, 0.0, 100.0, "trackLen")

ApplicationMgr(
    TopAlg=[TrackFinder],
    EvtSel="NONE",
    EvtMax=-1,
    ExtSvc=svcList,
    OutputLevel=INFO,
)
