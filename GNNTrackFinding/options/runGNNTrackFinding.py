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
from Configurables import EventDataSvc, ExaTrkGNNTrackFinder
from Configurables import Gaudi__Histograming__Sink__Root as RootHistoSink

from Gaudi.Configuration import DEBUG, VERBOSE

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
    "--monitoringOutputFile",
    help="File into which the monitoring histograms should go (if enabled)",
    default="gnn_tracking_monitoring_hists.root",
    type=Path,
)

args = parser.parse_known_args()[0]

io_svc = IOSvc()
io_svc.Input = "edm4hep.root"
io_svc.Output = "track_candidates.root"

TrackFinder = ExaTrkGNNTrackFinder(
    "GNNTrackFinder",
    EdgeClassifierModelPath=str(args.modelBase / "edge_classifier-InteractionGNN.onnx"),
    EdgeClassifierCut=0.5,
    NodeEmbeddingModelPath=str(
        args.modelBase / "graph_construction-MetricLearning.onnx"
    ),
    EdgeBuildingRadius=0.1,
    EdgeBuildingKnn=100.0,
    EmbeddingDim=4,
    MinHitsPerTrack=3,
    Device=args.device,
    OutputLevel=VERBOSE,
    InputHitCollections=[
        "IBTrackerHits",
        "IETrackerHits",
        "OBTrackerHits",
        "OETrackerHits",
        "VBTrackerHits",
        "VETrackerHits",
    ],
)

histSvc = RootHistoSink()
histSvc.FileName = str(args.monitoringOutputFile)

if args.monitoring:
    TrackFinder.MonitoringHistogram_Axis0 = (100, 0.0, 20000.0, "nInputHits")
    TrackFinder.MonitoringHistogram_Axis1 = (100, 0.0, 20000.0, "nTrackCandidates")
    TrackFinder.MonitoringHistogram_Axis2 = (100, 0.0, 100.0, "trackLen")

ApplicationMgr(
    TopAlg=[TrackFinder],
    EvtSel="NONE",
    EvtMax=1,
    ExtSvc=[EventDataSvc(), histSvc],
    OutputLevel=DEBUG,
)
