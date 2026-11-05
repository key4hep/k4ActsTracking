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
"""Re-measure the mapped material by propagating the scan's geantinos through it.

Pass the map under test via --materialMapFile: the geometry then carries the
mapped material, so what gets recorded here is what reconstruction will see.

  k4run material_validation.py \\
      --compactFile $k4geo_DIR/MuColl/MAIA/compact/MAIA_v0/MAIA_v0.xml \\
      --materialMapFile MAIA_v0_gen3_material_map.json \\
      --inputFiles scan/ \\
      --outputFile propagated_material_tracks.root

--inputFiles takes files and/or directories, as for the mapping.

Pass the *same* scan the map was built from. Directions are taken from it entry
by entry, so the propagated output lines up track by track with the scan and
compare_material_tracks.py can difference them without any rebinning. For a
Geant4 scan of the whole detector, compare against a MaterialRecordingAlg replay
of its directions instead: the map holds only the material in front of the
calorimeter.

Then compare the two:

  compare_material_tracks.py scan/ propagated_material_tracks.root -o validation/
"""

from Gaudi.Configuration import INFO
from Configurables import ApplicationMgr, MaterialValidationAlg
from k4FWCore.parseArgs import parser

from k4ActsTracking.material_helpers import expand_scan_inputs, make_services

parser.add_argument(
    "--inputFiles",
    nargs="+",
    default=[],
    help="Geantino scan ROOT file(s), and/or directories, in which case every "
    "*.root file directly inside is used; only the track directions are used",
)
parser.add_argument(
    "--treeName",
    default="material_tracks",
    help="TTree name in the input files",
)
parser.add_argument(
    "--outputFile",
    default="propagated-material-tracks.root",
    help="Where to write the propagated material tracks",
)
parser.add_argument(
    "--maxTracks",
    type=int,
    default=-1,
    help="Stop after this many tracks; negative (default) processes everything",
)
parser.add_argument(
    "--assigner",
    choices=["intersection", "propagator"],
    default="intersection",
    help="How to find the material a geantino crosses: 'intersection' validates "
    "the map's content, 'propagator' additionally validates that navigation "
    "reaches it",
)

args = parser.parse_known_args()[0]

validation = MaterialValidationAlg(
    "MaterialValidationAlg",
    InputFiles=expand_scan_inputs(args.inputFiles),
    TreeName=args.treeName,
    OutputFile=args.outputFile,
    MaxTracks=args.maxTracks,
    Assigner=args.assigner,
    OutputLevel=INFO,
)

ApplicationMgr(
    TopAlg=[validation],
    # One-shot job, as for the mapping: EvtSel="NONE" is what makes the event
    # loop run at all when there is no event input.
    EvtMax=1,
    EvtSel="NONE",
    ExtSvc=make_services(material_map=True),
    OutputLevel=INFO,
)
