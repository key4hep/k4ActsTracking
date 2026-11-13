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

# Multi-pass CKF tracking for CLD, tuned on CLD_o2_v08 (muon guns, Z -> bb at 91 GeV); the
# chain itself is make_multipass_tracking in _ckf_helpers.py.

import os
import sys

from Gaudi.Configuration import INFO
from k4FWCore import ApplicationMgr, IOSvc

sys.path.insert(0, os.path.dirname(__file__))

from _ckf_helpers import (  # noqa: E402
    make_hit_mergers,
    make_multipass_tracking,
    make_services,
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

# Settings shared by all passes (lengths in mm, pT in MeV)
COMMON = dict(
    RunCKF=True,
    CKF_Chi2CutOff=30,
    CKF_NumMeasurementsCutOff=1,
    SeedFinding_RMax=150,
    SeedFinding_MinPt=50,
    SeedFinding_ImpactMax=10,
    SeedFinding_CollisionRegion=6,
    SeedFinding_SigmaScattering=50,
    SeedFinding_RadLengthPerSeed=0.1,
    TrackSelector_MinMeasurements=4,
    OutputLevel=INFO,
)

# Seeding layers (system 1|2: vertex barrel|endcap, 3|4: inner tracker, 5|6: outer tracker)
# and the settings that differ from COMMON, in the order the passes run
PASSES = [
    # prompt tracks
    ("Main", ["system:1|2", "system:3|4,layer:0"], {}),
    # soft prompt tracks; the seeding grid needs pT / (0.3 B) >= RMax / 2
    (
        "LowPt",
        ["system:1|2"],
        dict(
            SeedFinding_RMax=70,
            SeedFinding_DeltaRMax=50,
            SeedFinding_MinPt=25,
            SeedFinding_ImpactMax=30,
            SeedFinding_CollisionRegion=30,
        ),
    ),
    # tracks from decays inside the vertex detector
    (
        "Displaced",
        ["system:1|2", "system:3|4,layer:0"],
        dict(SeedFinding_MinPt=100, SeedFinding_ImpactMax=100, SeedFinding_CollisionRegion=200),
    ),
    # tracks starting outside the vertex detector
    (
        "Outer",
        ["system:3|4", "system:5,layer:0"],
        dict(
            SeedFinding_RMax=1050,
            SeedFinding_ZMax=2300,
            SeedFinding_DeltaRMax=400,
            SeedFinding_MinPt=400,
            SeedFinding_ImpactMax=300,
            SeedFinding_CollisionRegion=1000,
        ),
    ),
    # tracks starting in the outer tracker
    (
        "OT",
        ["system:5|6"],
        dict(
            SeedFinding_RMax=2200,
            SeedFinding_ZMax=2300,
            SeedFinding_DeltaRMax=700,
            SeedFinding_MinPt=700,
            SeedFinding_ImpactMax=1000,
            SeedFinding_CollisionRegion=2000,
        ),
    ),
]
AMBIGUITY = dict(MaximumSharedHits=0, NMeasurementsMin=4)
SPLIT_MAX_CHI2 = 50

algs = [hit_merger, hit_rel_merger] + make_multipass_tracking(
    hit_merger, hit_rel_merger, PASSES, COMMON, AMBIGUITY, SPLIT_MAX_CHI2
)

ApplicationMgr(
    TopAlg=algs,
    ExtSvc=svcList,
    OutputLevel=INFO,
    EvtSel="NONE",
    EvtMax=-1,
)
