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

# ACTS primary vertex finding on an existing track collection.
#
# Detector agnostic: the geometry comes from --compactFile (only its magnetic
# field is used) and the input tracks from --inputTracks, so this runs on any
# reconstruction output that provides edm4hep tracks with an AtIP track state.
# No tracking is redone here; the tracks are only translated into the ACTS
# parametrization by ACTS' own EDM4hep converter.
#
# Example, on CLD tracks from ConformalTracking + RefitFinal:
#   k4run vertexing.py \
#       --compactFile $K4GEO/FCCee/CLD/compact/CLD_o2_v08/CLD_o2_v08.xml \
#       --IOSvc.Input particle_gun_CLD_o2_v08_REC.edm4hep.root \
#       --IOSvc.Output particle_gun_CLD_o2_v08_VTX.edm4hep.root

from Gaudi.Configuration import INFO
from Gaudi.Configurables import EventDataSvc, GeoSvc, VertexFindingAlg
from k4FWCore import ApplicationMgr, IOSvc
from k4FWCore.parseArgs import parser

parser.add_argument(
    "--compactFile",
    help="The geometry compact file to use (for the magnetic field)",
    type=str,
    required=True,
)
parser.add_argument(
    "--inputTracks",
    help="Name of the input track collection to run vertexing on",
    type=str,
    default="SiTracks_Refitted",
)
parser.add_argument(
    "--outputVertices",
    help="Name of the output vertex collection",
    type=str,
    default="ACTSPrimaryVertices",
)
parser.add_argument(
    "--beamSpotSize",
    help="Beam-spot size sigma x y z [mm]; enables the beam-spot constraint "
    "(CLD at 240 GeV: 0.0098 0.0000254 0.65)",
    type=float,
    nargs=3,
    default=None,
)
parser.add_argument(
    "--seedMaxD0Significance",
    help="Maximum d0 significance w.r.t. the beam line for a track to be used in seeding",
    type=float,
    default=3.5,
)
args = parser.parse_known_args()[0]

iosvc = IOSvc("IOSvc")
# Keep the input collections in the output file, so the vertices can be
# compared with the tracks and with LCFIPlus' PrimaryVertices side by side.
iosvc.outputCommands = ["keep *"]

svcList = [
    GeoSvc("GeoSvc", detectors=[args.compactFile], EnableGeant4Geo=False),
    EventDataSvc("EventDataSvc"),
]

vertexing = VertexFindingAlg(
    "VertexFindingAlg",
    InputTracks=[args.inputTracks],
    OutputVertices=[args.outputVertices],
    # One particle per track used by a vertex, and the vertex -> particle links
    # carrying the adaptive fit's track weights
    OutputParticles=[f"{args.outputVertices}_Particles"],
    OutputVertexParticleLinks=[f"{args.outputVertices}_ParticleLinks"],
    SeedMaxD0Significance=args.seedMaxD0Significance,
    BeamSpotSize=args.beamSpotSize or [],
    OutputLevel=INFO,
)

ApplicationMgr(
    TopAlg=[vertexing],
    ExtSvc=svcList,
    OutputLevel=INFO,
    EvtSel="NONE",
    EvtMax=-1,
)
