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

from pathlib import Path

from Gaudi.Configuration import INFO
from Configurables import ApplicationMgr, MaterialMappingAlg
from k4FWCore.parseArgs import parser

from _ckf_helpers import make_services

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


def expand_input_files(entries):
    """Resolve --inputFiles to a flat list of scan files.

    Directories contribute every ``*.root`` file directly inside them, sorted so
    a given directory always maps in the same order. Anything else is taken as a
    file path. Duplicates are dropped, since chaining the same scan file twice
    would double count its geantinos rather than fail visibly.

    Bad paths are reported here rather than left to TChain::Add, which otherwise
    fails deep inside the algorithm with much less context.
    """
    files = []
    seen = set()
    expanded_any = False

    for entry in entries:
        path = Path(entry)
        if path.is_dir():
            found = sorted(p for p in path.glob("*.root") if p.is_file())
            if not found:
                raise SystemExit(
                    f"material_mapping.py: directory '{entry}' contains no .root "
                    "files. Note the search is not recursive."
                )
            expanded_any = True
            candidates = found
        elif path.exists():
            candidates = [path]
        else:
            raise SystemExit(f"material_mapping.py: '{entry}' does not exist.")

        for candidate in candidates:
            key = candidate.resolve()
            if key in seen:
                continue
            seen.add(key)
            files.append(str(candidate))

    if expanded_any:
        print(f"material_mapping.py: mapping {len(files)} scan file(s):")
        for f in files:
            print(f"    {f}")

    return files


mapping = MaterialMappingAlg(
    "MaterialMappingAlg",
    InputFiles=expand_input_files(args.inputFiles),
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
