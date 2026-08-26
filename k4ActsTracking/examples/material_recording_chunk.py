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
"""Record one chunk of a geantino scan, for splitting the scan across batch jobs.

ACTS' own ``material_recording.py`` exposes neither ``--skip`` nor ``--seed``, and
hardcodes ``RandomNumbers(seed=228)``. Submitting N batch jobs with it therefore
records the *same* geantinos N times: N times the files and the wall time, but a
single scan's worth of statistics, and nothing in the output looks wrong.

This wrapper adds the one missing knob. It reuses ACTS' ``runMaterialRecording``
unchanged -- so the generator settings and, importantly, the
``RootMaterialTrackWriter`` configuration stay exactly what the mapping step
expects -- and only supplies a Sequencer with ``skip`` set. The Sequencer derives
its per-event random seeds from the event number, so ``--skip`` alone makes the
chunks disjoint; the seed does not also need to vary.

Geant4 in ACTS is single-threaded by construction (the run manager is created as
``G4RunManagerType::SerialOnly``, behind a per-process singleton), so one process
per chunk is the only way to use more than one core.

Submit job ``i`` of ``N``, each recording ``E`` events::

    python3 material_recording_chunk.py --input MAIA_v0.gdml \
        --events E --skip $((i * E)) --tracks 1000 --output-dir scan/

Each job writes ``scan/geant4_material_tracks_<skip>.root``. Hand all of them to
the mapping step at once -- ``MaterialMappingAlg`` chains its ``InputFiles``, so
there is no need to ``hadd`` them first::

    k4run material_mapping.py --compactFile <compact.xml> \
        --inputFiles scan/geant4_material_tracks_*.root \
        --outputFile material-map.json

See doc/material_mapping.md for the workflow this belongs to.
"""

import argparse
import importlib.util
import sys
from pathlib import Path


def load_acts_material_recording(explicit_dir=None):
    """Import ACTS' installed ``material_recording.py`` as a module.

    The script ships under ``share/acts/Examples/Scripts/Python`` in the ACTS
    install, which is located relative to the imported ``acts`` package rather
    than by searching the filesystem: an environment may hold more than one ACTS
    install and only the one on ``PYTHONPATH`` is the one in use.
    """
    import acts

    candidates = []
    if explicit_dir is not None:
        candidates.append(Path(explicit_dir))
    candidates.append(
        Path(acts.__file__).parents[2] / "share" / "acts" / "Examples" / "Scripts" / "Python"
    )

    for directory in candidates:
        script = directory / "material_recording.py"
        if not script.is_file():
            continue
        spec = importlib.util.spec_from_file_location("acts_material_recording", script)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    raise SystemExit(
        "Could not find ACTS' material_recording.py in any of:\n  "
        + "\n  ".join(str(c) for c in candidates)
        + "\nPass --acts-scripts to point at the directory holding it. It is part "
        "of the ACTS examples, so the ACTS in use must be built with "
        "+examples +geant4 +python."
    )


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("-i", "--input", required=True, help="GDML export of the detector")
    p.add_argument("-n", "--events", type=int, default=100, help="events in this chunk")
    p.add_argument(
        "--skip",
        type=int,
        default=0,
        help="first event number of this chunk. Job i of N recording E events each "
        "uses --skip i*E. Chunks with different skips generate different geantinos.",
    )
    p.add_argument("-t", "--tracks", type=int, default=1000, help="geantinos per event")
    p.add_argument("--eta-range", nargs=2, type=float, metavar=("MIN", "MAX"), default=(-4.0, 4.0))
    p.add_argument(
        "--phi-range", nargs=2, type=float, metavar=("MIN_DEG", "MAX_DEG"), default=(0.0, 360.0)
    )
    p.add_argument("--material-track-collection", default="material_tracks")
    p.add_argument("--output-dir", default=".", help="directory for the chunk file")
    p.add_argument(
        "-o",
        "--output",
        default=None,
        help="output file stem, without extension. Defaults to "
        "geant4_material_tracks_<skip>, which keeps concurrent jobs from "
        "overwriting each other when they share an output directory.",
    )
    p.add_argument(
        "--acts-scripts",
        default=None,
        help="directory holding ACTS' material_recording.py, if it cannot be "
        "located relative to the acts module",
    )
    args = p.parse_args()

    gdml = Path(args.input).resolve()
    if not gdml.is_file():
        raise SystemExit(f"No such file: {gdml}")
    if gdml.suffix != ".gdml":
        raise SystemExit(
            f"Expected a .gdml file, got '{gdml.name}'. Export the detector first with\n"
            "  ddsim --compactFile <compact.xml> --outputFile /tmp/dump.root "
            "--numberOfEvents 1 --enableGun --geometry.dumpGDML <out.gdml>"
        )

    acts_material_recording = load_acts_material_recording(args.acts_scripts)

    import acts
    import acts.examples
    import acts.examples.geant4

    u = acts.UnitConstants

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = args.output if args.output is not None else f"geant4_material_tracks_{args.skip:08d}"
    output_base = out_dir / stem

    print(
        f"recording events [{args.skip}, {args.skip + args.events}) "
        f"x {args.tracks} geantinos -> {output_base}.root",
        flush=True,
    )

    sequencer = acts.examples.Sequencer(
        events=args.events, skip=args.skip, numThreads=1
    )

    acts_material_recording.runMaterialRecording(
        detector=acts.examples.geant4.GdmlDetector(path=str(gdml)),
        s=sequencer,
        tracksPerEvent=args.tracks,
        etaRange=tuple(args.eta_range),
        phiRange=(args.phi_range[0] * u.degree, args.phi_range[1] * u.degree),
        materialTrackCollectionName=args.material_track_collection,
        outputFileBase=str(output_base),
    ).run()

    return 0


if __name__ == "__main__":
    sys.exit(main())
