#!/usr/bin/env python3

from pathlib import Path

from k4FWCore import IOSvc, ApplicationMgr
from k4FWCore.parseArgs import parser
from Configurables import EventDataSvc, ExaTrkGNNTrackFinder
from Gaudi.Configuration import DEBUG, VERBOSE

parser.add_argument(
    "--modelBase", help="The base directory for the model", default=".", type=Path
)

args = parser.parse_known_args()[0]

io_svc = IOSvc()
io_svc.Input = "edm4hep.root"
io_svc.Output = "track_candidates.root"

TrackFinder = ExaTrkGNNTrackFinder(
    "GNNTrackFinder",
    EdgeClassifierModelPath=str(args.modelBase / "edge_classifier-InteractionGNN.onnx"),
    NodeEmbeddingModelPath=str(
        args.modelBase / "graph_construction-MetricLearning.onnx"
    ),
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

ApplicationMgr(
    TopAlg=[TrackFinder],
    EvtSel="NONE",
    EvtMax=1,
    ExtSvc=[EventDataSvc()],
    OutputLevel=DEBUG,
)
