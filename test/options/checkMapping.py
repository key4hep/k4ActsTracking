#!/usr/bin/env python3

from Gaudi.Configuration import VERBOSE, DEBUG

from Configurables import GeoSvc, EventDataSvc, ActsGeoGen3PlaneSvc, TrackerMappingSvc
from Configurables import DumpSimTrackerHitCellIDAlg, PropagateToHitSurfaceAlg
from k4FWCore import ApplicationMgr, IOSvc
from k4FWCore.parseArgs import parser

parser.add_argument("--compactFile", help="Compact file")
args = parser.parse_known_args()[0]

# DD4hep geometry service
geoSvc = GeoSvc()
geoSvc.detectors = [args.compactFile]

# Plane geometry service
actsGeoPlaneSvc = ActsGeoGen3PlaneSvc("ActsGeoPlaneSvc")
actsGeoPlaneSvc.DetElementName   = "Tracker"
actsGeoPlaneSvc.LayerPatternExpr = r"layer\d"
actsGeoPlaneSvc.OutputLevel      = DEBUG

# Mapping service
mappingSvc = TrackerMappingSvc("TrackerMappingSvc")
mappingSvc.ActsGeoSvc = "ActsGeoPlaneSvc"
mappingSvc.OutputLevel = DEBUG

# I/O service
iosvc = IOSvc()
iosvc.Input = "positrons_1_edm4hep.root"
iosvc.OutputLevel = DEBUG

#--------------
# simple dump alg
dump = DumpSimTrackerHitCellIDAlg("DumpSimTrackerHitCellIDAlg")
dump.InputCollection = "SiHits"
dump.MappingSvc = "TrackerMappingSvc"
dump.MaxHits = 50
dump.OutputLevel = DEBUG

# simple prop check
prop = PropagateToHitSurfaceAlg("PropagateToHitSurfaceAlg")
prop.InputCollection = "SiHits"
prop.MappingSvc      = "TrackerMappingSvc"
prop.ActsGeoSvc      = "ActsGeoPlaneSvc"
prop.MaxHits         = 50
prop.BackstepMm      = 1.0
prop.AssumeCharge    = 0.0
prop.OutputLevel     = DEBUG
prop.CsvFile = "propagate_hits.csv"
prop.ObjFile = "propagate_segments.obj"

ApplicationMgr(
    TopAlg=[dump, prop],
    ExtSvc=[geoSvc, actsGeoPlaneSvc, mappingSvc, EventDataSvc(), iosvc],
    EvtMax=1,
    EvtSel="NONE",
)

