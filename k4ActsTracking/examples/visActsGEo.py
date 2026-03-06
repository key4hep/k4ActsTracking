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

from Gaudi.Configuration import VERBOSE, DEBUG

from Configurables import ActsGeoSvc, GeoSvc, ActsTestPropagator, EventDataSvc
from k4FWCore import ApplicationMgr, IOSvc
from k4FWCore.parseArgs import parser

parser.add_argument("--compactFile", help="Compact file")

args = parser.parse_known_args()[0]

iosvc = IOSvc()
iosvc.Output = "steps.root"

geoSvc = GeoSvc()
geoSvc.detectors = [args.compactFile]

actsGeoSvc = ActsGeoSvc("ActsGeoSvc")
actsGeoSvc.DumpVisualization = True
actsGeoSvc.ObjVisFileName = "full_mucoll_old_vertex_endcap.obj"
actsGeoSvc.OutputLevel = DEBUG

propTest = ActsTestPropagator("TestPropagator")
propTest.OutputLevel = DEBUG
propTest.NumTracks = 20000


ApplicationMgr(
    TopAlg=[],
    # TopAlg=[],
    ExtSvc=[geoSvc, actsGeoSvc, EventDataSvc()],
    EvtMax=1,
    EvtSel="NONE",
)
