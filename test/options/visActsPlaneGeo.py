#!/usr/bin/env python3

from Gaudi.Configuration import VERBOSE, DEBUG

from Configurables import GeoSvc, EventDataSvc, ActsGeoGen3PlaneSvc
from k4FWCore import ApplicationMgr, IOSvc
from k4FWCore.parseArgs import parser

parser.add_argument("--compactFile", help="Compact file")
args = parser.parse_known_args()[0]

#iosvc = IOSvc()
#iosvc.Output = "steps.root"

# DD4hep geometry service
geoSvc = GeoSvc()
geoSvc.detectors = [args.compactFile]

# Plane geometry service
actsGeoPlaneSvc = ActsGeoGen3PlaneSvc("ActsGeoPlaneSvc")
actsGeoPlaneSvc.DetElementName   = "Tracker"
actsGeoPlaneSvc.LayerPatternExpr = r"layer\d"
actsGeoPlaneSvc.OutputLevel      = VERBOSE

ApplicationMgr(
    TopAlg=[],
    ExtSvc=[geoSvc, actsGeoPlaneSvc, EventDataSvc()],
    EvtMax=1,
    EvtSel="NONE",
)

