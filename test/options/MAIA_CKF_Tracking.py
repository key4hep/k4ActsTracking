#!/usr/bin/env python3

import os

from Gaudi.Configuration import INFO, VERBOSE
from Gaudi.Configurables import (
    ACTSSeededCKFTrackingAlg,
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
parser.add_argument(
    "--trackingGeoFile",
    help="The TGeo file containing the tracking geometry",
    type=str,
)
parser.add_argument(
    "--materialFile",
    help="The file containing the material mapping for the tracking geometry",
    type=str,
)
parser.add_argument(
    "--geoDescFile", help="The JSON file describing the subdetectors", type=str
)

args = parser.parse_known_args()[0]


svcList = [
    GeoSvc("GeoSvc", detectors=[args.compactFile], EnableGeant4Geo=False),
    EventDataSvc("EventDataSvc"),
]

iosvc = IOSvc(
    "IOSvc",
    Input=["particle_gun_MAIA_SIM.edm4hep.root"],
    Output="maia_ckf_tracking_reco.edm4hep.root",
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
            IsStrip="InnerBarrel" in name or "OuterEndcap" in name,  # Is this true?
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

# algList.append(
#     CollectionMerger(
#         "MergeHitsRelations",
#         InputCollections=[
#             "VertexBarrelHitsRelations",
#             "VertexEndcapHitsRelations",
#             "InnerTrackerBarrelHitsRelations",
#             "InnerTrackerEndcapHitsRelations",
#             "OuterTrackerBarrelHitsRelations",
#             "OuterTrackerEndcapHitsRelations",
#         ],
#         OutputCollection=["MergedTrackerHitsRelations"],
#     )
# )

ckf_tracking = ACTSSeededCKFTrackingAlg(
    "CKFTracking",
    MatFile=args.materialFile,
    TGeoFile=args.trackingGeoFile,
    TGeoDescFile=args.geoDescFile,
    DetectorSchema=args.compactFile,
    RunCKF="True",
    CKF_Chi2CutOff=10,
    SeedFinding_RMax=150,
    SeedFinding_MinPt=500,
    SeedFinding_ImpactMax=3,
    CKF_NumMeasurementsCutOff=1,
    SeedFinding_SigmaScattering=50,
    SeedFinding_CollisionRegion=6,
    SeedFinding_RadLengthPerSeed=0.1,
    SeedingLayers=[
        "13",
        "2",
        "13",
        "6",
        "13",
        "10",
        "13",
        "14",
        "14",
        "2",
        "14",
        "6",
        "14",
        "10",
        "14",
        "14",
        "15",
        "2",
        "15",
        "6",
        "15",
        "10",
        "15",
        "14",
        "8",
        "2",
        "17",
        "2",
        "18",
        "2",
    ],
    OutputTrackCollectionName=["AllTracks"],
    OutputSeedCollectionName=["SeedTracks"],
    InputTrackerHitCollectionName=["MergedTrackerHits"],
    OutputLevel=VERBOSE,
)
algList.append(ckf_tracking)


ApplicationMgr(
    TopAlg=algList, ExtSvc=svcList, OutputLevel=INFO, EvtSel="NONE", EvtMax=-1
)
