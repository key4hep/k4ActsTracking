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

from Gaudi.Configuration import INFO, VERBOSE
from Gaudi.Configurables import (
    GeoSvc,
    ActsGeoSvc,
    CKFTrackingAlg,
    CollectionMerger,
    EventDataSvc,
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
    Input=["particle_gun_CLD_o2_v08_REC.edm4hep.root"],
    Output="particle_gun_CLD_o2_v08_ACTS_CKFTracking.edm4hep.root",
)

hit_merger = CollectionMerger(
    "MergeHits",
    InputCollections=[
        "VXDTrackerHits",
        "VXDEndcapTrackerHits",
        "ITrackerHits",
        "OTrackerHits",
        "ITrackerEndcapHits",
        "OTrackerEndcapHits",
    ],
    OutputCollection="AllTrackerHits",
)
hit_rel_merger = CollectionMerger(
    "MergeHitRelations",
    InputCollections=[
        "VXDTrackerHitRelations",
        "VXDEndcapTrackerHitRelations",
        "InnerTrackerBarrelHitsRelations",
        "InnerTrackerEndcapHitsRelations",
        "OuterTrackerBarrelHitsRelations",
        "OuterTrackerEndcapHitsRelations",
    ],
    OutputCollection="AllTrackerHitRelations",
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
    # SeedingLayersCellID uses CellIDSelector selection strings, where each
    # entry is a comma-separated list of field:value constraints read directly
    # from the MAIA_v0 compact XML encoding.  Multiple entries are OR-ed.
    # Omitted fields act as wildcards; "|" separates multiple values for a field.
    SeedingSensorsCellIDs=["system:1|2", "system:3|4,layer:1"],
    OutputTrackCollectionName=["CKFTracks"],
    OutputSeedCollectionName=["CKFSeedTracks"],
    InputTrackerHitCollectionName=hit_merger.OutputCollection,
    InputTrackerHitRelationCollectionName=hit_rel_merger.OutputCollection,
    OutputLevel=VERBOSE,
)


ApplicationMgr(
    TopAlg=[hit_merger, hit_rel_merger, ckf_tracking],
    ExtSvc=svcList,
    OutputLevel=INFO,
    EvtSel="NONE",
    EvtMax=-1,
)
