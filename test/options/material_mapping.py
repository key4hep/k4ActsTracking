#!/usr/bin/env python3
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

Do NOT pass --materialMapFile here: the mapping needs the geometry to still
carry its proto-material placeholders, which is what a loaded map replaces.
"""

from Gaudi.Configuration import INFO
from Configurables import ApplicationMgr, MaterialMappingAlg
from k4FWCore.parseArgs import parser

from _ckf_helpers import make_services

parser.add_argument(
    "--inputFiles",
    nargs="+",
    default=[],
    help="ROOT file(s) with the recorded material tracks from the geantino scan",
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
    InputFiles=args.inputFiles,
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
