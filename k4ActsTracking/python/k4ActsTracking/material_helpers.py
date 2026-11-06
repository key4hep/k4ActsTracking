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
"""Helpers shared by the material recording, mapping and validation options files."""

from pathlib import Path

from Configurables import ActsGeoSvc, EventDataSvc, GeoSvc
from k4FWCore.parseArgs import parser


def expand_scan_inputs(entries, option="--inputFiles"):
    """Resolve a geantino-scan --inputFiles list to a flat list of files.

    Shared by the recording (for --replayFiles), mapping and validation options
    files, which all chain a scan that batch jobs may have split across many
    files. ``option`` names the command line option in the messages.

    Directories contribute every ``*.root`` file directly inside them, sorted so
    a given directory always yields the same order. The search does not recurse,
    so a scan directory can hold logs and notes without confusing it. Anything
    that is not a directory is taken as a file path, and files and directories
    can be mixed.

    Duplicates are dropped, since chaining the same scan file twice would double
    count its geantinos rather than fail visibly -- easy to do by passing both a
    directory and one of its own members.

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
                    f"{option}: directory '{entry}' contains no .root files. "
                    "Note the search is not recursive."
                )
            expanded_any = True
            candidates = found
        elif path.exists():
            candidates = [path]
        else:
            raise SystemExit(f"{option}: '{entry}' does not exist.")

        for candidate in candidates:
            key = candidate.resolve()
            if key in seen:
                continue
            seen.add(key)
            files.append(str(candidate))

    if expanded_any:
        print(f"{option}: chaining {len(files)} scan file(s):")
        for f in files:
            print(f"    {f}")

    return files


def make_services(material_map=False):
    """GeoSvc, ActsGeoSvc and EventDataSvc for the geometry given with
    --compactFile.

    With material_map=True, ActsGeoSvc loads the map given with the (then
    required) --materialMapFile. The recording and mapping jobs must not load a
    map: the mapping needs the proto-material placeholders a map replaces.
    """
    parser.add_argument(
        "--compactFile",
        required=True,
        help="DD4hep compact file of the detector",
    )
    if material_map:
        parser.add_argument(
            "--materialMapFile",
            required=True,
            help="Material map (.json/.cbor/.root) to decorate the tracking geometry with",
        )
    args = parser.parse_known_args()[0]
    return [
        GeoSvc("GeoSvc", detectors=[args.compactFile], EnableGeant4Geo=False),
        ActsGeoSvc(
            "ActsGeoSvc",
            MaterialMapFile=args.materialMapFile if material_map else "",
        ),
        EventDataSvc("EventDataSvc"),
    ]
