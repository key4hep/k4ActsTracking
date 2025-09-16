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
import os
from Gaudi.Configuration import INFO

from Configurables import ActsGeoSvc, ApplicationMgr, GeoSvc

algList = []

try:
    odd_base = os.environ["OPENDATADETECTOR_DATA"]
except KeyError:
    # For older releases OPENDATADETCTOR_DATA has not been defined so we try to
    # retrieve it off the LD_LIBRARY_PATH
    ld_lib_paths = os.environ["LD_LIBRARY_PATH"].split(":")
    odd_paths = [p for p in ld_lib_paths if "opendatadetector" in p]
    if odd_paths:
        odd_base = f"{odd_paths[0].rsplit('/', 1)[0]}/share/OpenDataDetector"
    else:
        raise

dd4hep_geo = GeoSvc("GeoSvc")
dd4hep_geo.detectors = [f"{odd_base}/xml/OpenDataDetector.xml"]
dd4hep_geo.EnableGeant4Geo = False

acts_geo = ActsGeoSvc("ActsGeoSvc")
acts_geo.GeoSvcName = dd4hep_geo.name()
acts_geo.debugGeometry = True
acts_geo.outputFileName = "MyObjFile"

ApplicationMgr(
    TopAlg=algList,
    EvtSel="NONE",
    EvtMax=2,
    # order dependent...
    ExtSvc=[dd4hep_geo, acts_geo],
    OutputLevel=INFO,
)
