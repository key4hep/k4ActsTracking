#!/usr/bin/env python3

from Gaudi.Configuration import VERBOSE

from Configurables import ActsGeoGen3Svc, GeoSvc
from k4FWCore import ApplicationMgr
from k4FWCore.parseArgs import parser

parser.add_argument("--compactFile", help="Compact file")

args = parser.parse_known_args()[0]

geoSvc = GeoSvc()
geoSvc.detectors = [args.compactFile]

actsGeoSvc = ActsGeoGen3Svc("ActsGeoSvc")
actsGeoSvc.DetElementName = "InnerTrackerBarrel"
actsGeoSvc.LayerPatternExpr = r"layer\\d"
actsGeoSvc.OutputLevel = VERBOSE


ApplicationMgr(TopAlg=[], ExtSvc=[geoSvc, actsGeoSvc])
