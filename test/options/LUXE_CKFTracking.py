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

import os
import sys

from Gaudi.Configuration import INFO
from Gaudi.Configurables import DDPlanarDigi
from k4FWCore import ApplicationMgr, IOSvc

sys.path.insert(0, os.path.dirname(__file__))

from _ckf_helpers import (
    make_ckf_tracking,
    make_services,
    make_hit_mergers,
)

svcList = make_services()
iosvc = IOSvc("IOSvc")

algList = []

# The LUXE_v0 geometry has a single planar tracker (DD4hep detector "Tracker")
# whose sensitive hits end up in the "SiHits" SimTrackerHit collection (named
# after the "SiHits" readout). Digitize those into space points for the CKF.
algList.append(
    DDPlanarDigi(
        "TrackerDigitizer",
        SubDetectorName="Tracker",
        IsStrip=False,
        ResolutionT=[0.03],
        ResolutionU=[0.007],
        ResolutionV=[0.007],
        SimTrackHitCollectionName=["SiHits"],
        SimTrkHitRelCollection=["SiHitsRelations"],
        TrackerHitCollectionName=["SiTrackerHits"],
    )
)

hit_merger, hit_rel_merger = make_hit_mergers(
    hit_collections=["SiTrackerHits"],
    relation_collections=["SiHitsRelations"],
)
algList.extend([hit_merger, hit_rel_merger])

ckf_tracking = make_ckf_tracking(
    hit_merger,
    hit_rel_merger,
    # LUXE is a four-layer (layer:0..3) telescope; use all of them as the pool
    # of seed space points. Omitted cellID fields act as wildcards.
    seeding_cellids=["layer:0|1|2|3"],
)
algList.append(ckf_tracking)

ApplicationMgr(
    TopAlg=algList, ExtSvc=svcList, OutputLevel=INFO, EvtSel="NONE", EvtMax=-1
)
