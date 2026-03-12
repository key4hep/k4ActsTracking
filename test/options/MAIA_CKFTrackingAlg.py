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
# Options file for the Gen3-geometry CKF tracking algorithm (CKFTrackingAlg).
# Uses ActsGeoSvc for geometry, magnetic field, and hit-to-surface mapping,
# instead of the algorithm-level TGeo file loading used by ACTSSeededCKFTrackingAlg.

import os

from Gaudi.Configuration import INFO, VERBOSE
from Gaudi.Configurables import (
    ActsGeoSvc,
    CKFTrackingAlg,
    CollectionMerger,
    DDPlanarDigi,
    EventDataSvc,
    GeoSvc,
)

from k4FWCore import ApplicationMgr, IOSvc
from k4FWCore.parseArgs import parser

parser.add_argument(
    "--compactFile",
    help="The geometry compact file to use for reconstruction",
    type=str,
)

args = parser.parse_known_args()[0]


svcList = [
    GeoSvc("GeoSvc", detectors=[args.compactFile], EnableGeant4Geo=False),
    ActsGeoSvc("ActsGeoSvc"),
    EventDataSvc("EventDataSvc"),
]

iosvc = IOSvc(
    "IOSvc",
    Input=["particle_gun_MAIA_SIM.edm4hep.root"],
    Output="maia_ckftrackingalg_reco.edm4hep.root",
)


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


algList.append(
    CollectionMerger(
        "MergeHits",
        InputCollections=[
            "VertexBarrelHits",
            "VertexEndcapHits",
            "InnerTrackerBarrelHits",
            "InnerTrackerEndcapHits",
            "OuterTrackerBarrelHits",
            "OuterTrackerEndcapHits",
        ],
        OutputCollection=["MergedTrackerHits"],
    )
)

algList.append(
    CollectionMerger(
        "MergeHitsRelations",
        InputCollections=[
            "VertexBarrelHitsRelations",
            "VertexEndcapHitsRelations",
            "InnerTrackerBarrelHitsRelations",
            "InnerTrackerEndcapHitsRelations",
            "OuterTrackerBarrelHitsRelations",
            "OuterTrackerEndcapHitsRelations",
        ],
        OutputCollection=["MergedTrackerHitsRelations"],
    )
)

ckf_tracking = CKFTrackingAlg(
    "CKFTracking",
    RunCKF=True,
    CKF_Chi2CutOff=10,
    SeedFinding_RMax=150,
    SeedFinding_MinPt=500,
    SeedFinding_ImpactMax=3,
    CKF_NumMeasurementsCutOff=1,
    SeedFinding_SigmaScattering=50,
    SeedFinding_CollisionRegion=6,
    SeedFinding_RadLengthPerSeed=0.1,
    # Seeding layers use (volume, layer) pairs from the Gen3 / Blueprint
    # geometry. The volume/layer numbering differs from the TGeo-based path
    # used by ACTSSeededCKFTrackingAlg — update these once the geometry IDs
    # have been inspected at runtime (e.g. via ActsTestPropagator output).
    SeedingLayers=[],
    OutputTrackCollectionName=["AllTracks"],
    OutputSeedCollectionName=["SeedTracks"],
    InputTrackerHitCollectionName=["MergedTrackerHits"],
    InputTrackerHitRelationCollectionName=["MergedTrackerHitsRelations"],
    OutputLevel=VERBOSE,
)
algList.append(ckf_tracking)


ApplicationMgr(
    TopAlg=algList, ExtSvc=svcList, OutputLevel=INFO, EvtSel="NONE", EvtMax=-1
)
