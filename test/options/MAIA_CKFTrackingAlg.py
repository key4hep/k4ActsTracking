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

for name in ("VertexBarrel", "VertexEndcap"):
    algList.append(
        DDPlanarDigi(
            f"{name}Digitizer",
            CorrectTimesForPropagation=True,
            IsStrip=False,
            ResolutionT=[0.03],
            ResolutionU=[0.005],
            ResolutionV=[0.005],
            SubDetectorName="Vertex",
            TimeWindowMax=[0.15],
            TimeWindowMin=[-0.09],
            UseTimeWindow=True,
            SimTrackHitCollectionName=[f"{name}Collection"],
            SimTrkHitRelCollection=[f"{name}HitsRelations"],
            TrackerHitCollectionName=[f"{name}Hits"],
        )
    )

for name in (
    "InnerTrackerBarrel",
    "InnerTrackerEndcap",
    "OuterTrackerBarrel",
    "OuterTrackerEndcap",
):
    algList.append(
        DDPlanarDigi(
            f"{name}Digitizer",
            CorrectTimesForPropagation=True,
            IsStrip="InnerBarrel" in name or "OuterEndcap" in name,
            ResolutionT=[0.06],
            ResolutionU=[0.007],
            ResolutionV=[0.09],
            SubDetectorName=name.replace("Barrel", "s").replace("Endcap", "s"),
            TimeWindowMax=[0.3],
            TimeWindowMin=[-0.18],
            UseTimeWindow=True,
            SimTrackHitCollectionName=[f"{name}Collection"],
            SimTrkHitRelCollection=[f"{name}HitsRelations"],
            TrackerHitCollectionName=[f"{name}Hits"],
        )
    )

hit_merger, hit_rel_merger = make_hit_mergers(
    hit_collections=[
        "VertexBarrelHits",
        "VertexEndcapHits",
        "InnerTrackerBarrelHits",
        "InnerTrackerEndcapHits",
        "OuterTrackerBarrelHits",
        "OuterTrackerEndcapHits",
    ],
    relation_collections=[
        "VertexBarrelHitsRelations",
        "VertexEndcapHitsRelations",
        "InnerTrackerBarrelHitsRelations",
        "InnerTrackerEndcapHitsRelations",
        "OuterTrackerBarrelHitsRelations",
        "OuterTrackerEndcapHitsRelations",
    ],
)
algList.extend([hit_merger, hit_rel_merger])

ckf_tracking = make_ckf_tracking(
    hit_merger,
    hit_rel_merger,
    seeding_cellids=["system:1", "system:2,layer:1|2|3"],
    # Keep hits with a chi2 up to this value as outliers instead of holes
    CKF_Chi2CutOffOutlier=25,
    # Terminate poor CKF branches early, aligned with a typical downstream
    # track selection (at least 8 hits, at most 2 holes)
    UseBranchStopper=True,
    BranchStopper_MaxHoles=2,
    BranchStopper_MaxOutliers=2,
    BranchStopper_MinMeasurements=8,
)
algList.append(ckf_tracking)

ApplicationMgr(
    TopAlg=algList, ExtSvc=svcList, OutputLevel=INFO, EvtSel="NONE", EvtMax=-1
)
