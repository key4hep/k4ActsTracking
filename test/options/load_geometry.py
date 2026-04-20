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
import pathlib

from Gaudi.Configuration import INFO, VERBOSE
from Configurables import ActsGeoSvc, ApplicationMgr, GeoSvc
from k4FWCore.parseArgs import parser

parser.add_argument("--compactFile", help="The compact file of the geometry to load")

args = parser.parse_known_args()[0]

dd4hep_geo = GeoSvc("GeoSvc")
dd4hep_geo.detectors = [args.compactFile]
dd4hep_geo.EnableGeant4Geo = False

acts_geo = ActsGeoSvc("ActsGeoSvc")
acts_geo.OutputLevel = VERBOSE
acts_geo.DumpVisualization = True
acts_geo.ObjVisFileName = f"{pathlib.Path(args.compactFile).stem}-acts-geo.obj"

ApplicationMgr(
    TopAlg=[],
    EvtSel="NONE",
    EvtMax=1,
    ExtSvc=[dd4hep_geo, acts_geo],
    OutputLevel=INFO,
)
