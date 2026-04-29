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

from Gaudi.Configuration import INFO
from k4FWCore import ApplicationMgr, IOSvc

from _ckf_helpers import (
    make_ckf_tracking,
    make_services,
    make_hit_mergers,
)

svcList = make_services()
iosvc = IOSvc("IOSvc")

hit_merger, hit_rel_merger = make_hit_mergers(
    hit_collections=[
        "VXDTrackerHits",
        "VXDEndcapTrackerHits",
        "ITrackerHits",
        "OTrackerHits",
        "ITrackerEndcapHits",
        "OTrackerEndcapHits",
    ],
    relation_collections=[
        "VXDTrackerHitRelations",
        "VXDEndcapTrackerHitRelations",
        "InnerTrackerBarrelHitsRelations",
        "InnerTrackerEndcapHitsRelations",
        "OuterTrackerBarrelHitsRelations",
        "OuterTrackerEndcapHitsRelations",
    ],
)

ckf_tracking = make_ckf_tracking(
    hit_merger,
    hit_rel_merger,
    seeding_cellids=["system:1|2", "system:3|4,layer:1"],
)

ApplicationMgr(
    TopAlg=[hit_merger, hit_rel_merger, ckf_tracking],
    ExtSvc=svcList,
    OutputLevel=INFO,
    EvtSel="NONE",
    EvtMax=-1,
)
