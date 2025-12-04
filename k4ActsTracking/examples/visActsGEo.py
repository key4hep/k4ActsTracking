#!/usr/bin/env python3

from Gaudi.Configuration import VERBOSE, DEBUG

from Configurables import ActsGeoGen3Svc, GeoSvc, ActsTestPropagator, EventDataSvc
from k4FWCore import ApplicationMgr, IOSvc
from k4FWCore.parseArgs import parser

parser.add_argument("--compactFile", help="Compact file")

args = parser.parse_known_args()[0]

iosvc = IOSvc()
iosvc.Output = "steps.root"

geoSvc = GeoSvc()
geoSvc.detectors = [args.compactFile]

actsGeoSvc = ActsGeoGen3Svc("ActsGeoSvc")
actsGeoSvc.DetElementName = "InnerTrackerBarrel"
actsGeoSvc.LayerPatternExpr = r"layer\\d"
actsGeoSvc.OutputLevel = VERBOSE

propTest = ActsTestPropagator("TestPropagator")
propTest.OutputLevel = DEBUG
propTest.NumTracks = 20000


ApplicationMgr(
    TopAlg=[propTest],
    # TopAlg=[],
    ExtSvc=[geoSvc, actsGeoSvc, EventDataSvc()],
    EvtMax=1,
    EvtSel="NONE",
)
