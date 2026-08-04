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
"""Project a recorded geantino scan onto the blueprint's material receivers.

This is step 3 of doc/material_mapping.md. It has to run in the same job as
ActsGeoSvc, because the map is keyed by Acts::GeometryIdentifier and those are
assigned by the blueprint traversal order -- a map produced against any other
geometry will not match.

Usage:

  k4run material_mapping.py \\
      --compactFile $k4geo_DIR/MuColl/MAIA/compact/MAIA_v0/MAIA_v0.xml \\
      --inputFiles geant4_material_tracks.root \\
      --outputFile MAIA_v0_gen3_material_map.json

--inputFiles also takes directories, which is the convenient way to consume a
scan that was split across batch jobs by examples/material_recording_chunk.py:

  k4run material_mapping.py \\
      --compactFile $k4geo_DIR/MuColl/MAIA/compact/MAIA_v0/MAIA_v0.xml \\
      --inputFiles scan/ \\
      --outputFile MAIA_v0_gen3_material_map.json

Every *.root file directly inside such a directory is mapped, in sorted order.
Files and directories can be mixed freely. MaterialMappingAlg chains them, so
there is no hadd step.

Do NOT pass --materialMapFile here: the mapping needs the geometry to still
carry its proto-material placeholders, which is what a loaded map replaces.
"""

from Gaudi.Configuration import INFO
from Configurables import ApplicationMgr, MaterialMappingAlg
from k4FWCore.parseArgs import parser

from _ckf_helpers import expand_scan_inputs, make_services

parser.add_argument(
    "--inputFiles",
    nargs="+",
    default=[],
    help="ROOT file(s) with the recorded material tracks from the geantino scan, "
    "and/or directories, in which case every *.root file directly inside is used",
)
parser.add_argument(
    "--treeName",
    default="material_tracks",
    help="TTree name in the input files, must match the recording job's writer",
)
parser.add_argument(
    "--outputFile",
    default="material-map.json",
    help="Material map to write; feed this to ActsGeoSvc.MaterialMapFile afterwards",
)
parser.add_argument(
    "--maxTracks",
    type=int,
    default=-1,
    help="Stop after this many tracks; negative (default) processes everything",
)

args = parser.parse_known_args()[0]

mapping = MaterialMappingAlg(
    "MaterialMappingAlg",
    InputFiles=expand_scan_inputs(args.inputFiles),
    TreeName=args.treeName,
    OutputFile=args.outputFile,
    MaxTracks=args.maxTracks,
    OutputLevel=INFO,
)

ApplicationMgr(
    TopAlg=[mapping],
    # One-shot job: the algorithm reads the whole scan on its first execute.
    # EvtSel="NONE" is what makes the event loop run that one dummy event --
    # without it there is no event source and the loop runs zero events, so the
    # algorithm would initialize and finalize without ever executing.
    EvtMax=1,
    EvtSel="NONE",
    ExtSvc=make_services(),
    OutputLevel=INFO,
)
