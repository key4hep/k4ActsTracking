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
"""Record the material along straight rays through the DD4hep geometry.

Walks geantino-like rays from the origin through the geometry the compact file
builds and writes what they cross in the format the mapping reads:

  k4run material_recording.py \\
      --compactFile $k4geo_DIR/MuColl/MAIA/compact/MAIA_v0/MAIA_v0.xml \\
      --numTracks 100000 --seed 1 \\
      --outputFile scan/material_tracks_1.root

Split a large scan across jobs by giving each its own --seed, then hand the
directory to material_mapping.py, which chains every *.root file inside.

Rays stop at the ECAL inner face unless --maxR/--maxZ say otherwise. To compare
against a Geant4 scan track by track, replay its directions:

  k4run material_recording.py --compactFile ... \\
      --replayFiles geant4_material_tracks.root --outputFile replayed.root
"""

from Gaudi.Configuration import INFO
from Configurables import ApplicationMgr, MaterialRecordingAlg
from k4FWCore.parseArgs import parser

from _ckf_helpers import expand_scan_inputs, make_services

parser.add_argument(
    "--outputFile",
    default="material-tracks.root",
    help="ROOT file to write the recorded material tracks to",
)
parser.add_argument(
    "--numTracks",
    type=int,
    default=None,
    help="Number of rays (default 1000); with --replayFiles every entry is replayed unless this is given",
)
parser.add_argument(
    "--seed",
    type=int,
    default=42,
    help="Seed of the direction sampling; give every job of a split scan its own",
)
parser.add_argument(
    "--etaRange",
    nargs=2,
    type=float,
    default=[-4.0, 4.0],
    metavar=("MIN", "MAX"),
    help="Pseudorapidity range, sampled flat",
)
parser.add_argument(
    "--maxR",
    type=float,
    default=-1.0,
    help="Radius [mm] at which rays stop; default: the ECAL barrel inner face",
)
parser.add_argument(
    "--maxZ",
    type=float,
    default=-1.0,
    help="|z| [mm] at which rays stop; default: the ECAL endcap inner face",
)
parser.add_argument(
    "--replayFiles",
    nargs="+",
    default=[],
    help="Scan file(s) and/or directories whose directions are replayed entry by entry, e.g. a Geant4 scan",
)

args = parser.parse_known_args()[0]

num_tracks = args.numTracks
if num_tracks is None:
    num_tracks = -1 if args.replayFiles else 1000

recording = MaterialRecordingAlg(
    "MaterialRecordingAlg",
    OutputFile=args.outputFile,
    NumTracks=num_tracks,
    Seed=args.seed,
    EtaRange=args.etaRange,
    MaxR=args.maxR,
    MaxZ=args.maxZ,
    ReplayFiles=expand_scan_inputs(args.replayFiles, "--replayFiles"),
    OutputLevel=INFO,
)

ApplicationMgr(
    TopAlg=[recording],
    # One-shot job, as for the mapping: EvtSel="NONE" is what makes the event
    # loop run at all when there is no event input.
    EvtMax=1,
    EvtSel="NONE",
    ExtSvc=make_services(),
    OutputLevel=INFO,
)
