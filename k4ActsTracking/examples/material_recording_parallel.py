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
"""Run the ACTS geantino scan in parallel over several processes.

Geant4 in ACTS is single-threaded by construction (the run manager is created as
G4RunManagerType::SerialOnly, behind a per-process singleton), so the only way to
use more than one core is to run several processes and merge afterwards.

Modelled on ACTS' Examples/Scripts/Python/geant4_parallel.py, but for material
recording rather than full simulation. Each worker builds its own detector and
Geant4 run manager -- neither can be shared across processes -- and writes its own
ROOT file. k4ActsTracking's MaterialMappingAlg accepts a list of InputFiles and
chains them, so the chunks need no hadd.

  python3 material_recording_parallel.py --gdml MAIA_v0.gdml \
      --events 1000 --tracks 1000 --jobs 8 --output-dir scan/
"""

import argparse
from functools import partial
from multiprocessing import Pool
from pathlib import Path


def record_chunk(begin, end, gdml, tracks, eta_range, out_dir, seed):
    """Record events [begin, end). Runs in its own process."""
    import acts
    import acts.examples
    import acts.examples.geant4
    import acts.examples.hepmc3
    from acts.examples.root import RootMaterialTrackWriter

    u = acts.UnitConstants
    collection = "material_tracks"

    # skip= is what makes chunks differ: the Sequencer derives per-event seeds
    # from the event number, so with a fixed RNG seed and no skip every chunk
    # would generate identical geantinos.
    s = acts.examples.Sequencer(events=end - begin, skip=begin, numThreads=1)
    rnd = acts.examples.RandomNumbers(seed=seed)

    evGen = acts.examples.EventGenerator(
        level=acts.logging.WARNING,
        generators=[
            acts.examples.EventGenerator.Generator(
                multiplicity=acts.examples.FixedMultiplicityGenerator(n=1),
                vertex=acts.examples.GaussianVertexGenerator(
                    stddev=acts.Vector4(0, 0, 0, 0), mean=acts.Vector4(0, 0, 0, 0)
                ),
                particles=acts.examples.ParametricParticleGenerator(
                    pdg=acts.PdgParticle.eInvalid,
                    charge=0,
                    randomizeCharge=False,
                    mass=0,
                    p=(1 * u.GeV, 10 * u.GeV),
                    eta=eta_range,
                    phi=(0.0, 360.0 * u.degree),
                    numParticles=tracks,
                    etaUniform=True,
                ),
            )
        ],
        randomNumbers=rnd,
    )
    s.addReader(evGen)

    conv = acts.examples.hepmc3.HepMC3InputConverter(
        level=acts.logging.WARNING,
        inputEvent=evGen.config.outputEvent,
        outputParticles="particles_initial",
        outputVertices="vertices_initial",
        mergePrimaries=False,
    )
    s.addAlgorithm(conv)

    # Built here, not in the parent: the Geant4 run manager is a per-process
    # singleton and the detector is not picklable.
    detector = acts.examples.geant4.GdmlDetector(path=str(gdml))

    s.addAlgorithm(
        acts.examples.geant4.Geant4MaterialRecording(
            level=acts.logging.WARNING,
            detector=detector,
            randomNumbers=rnd,
            inputParticles=conv.config.outputParticles,
            outputMaterialTracks=collection,
            recordElementFractions=False,
        )
    )

    out = Path(out_dir) / f"geant4_material_tracks_{begin:07d}.root"
    s.addWriter(
        RootMaterialTrackWriter(
            level=acts.logging.WARNING,
            inputMaterialTracks=collection,
            treeName=collection,
            filePath=str(out),
            prePostStep=True,
            recalculateTotals=True,
        )
    )

    s.run()
    return str(out)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--gdml", required=True, help="GDML export of the detector")
    p.add_argument("--events", type=int, default=100)
    p.add_argument("--tracks", type=int, default=100, help="geantinos per event")
    p.add_argument("--jobs", type=int, default=8)
    p.add_argument("--eta-range", nargs=2, type=float, default=(-4.0, 4.0))
    p.add_argument("--output-dir", default=".")
    p.add_argument("--seed", type=int, default=228)
    args = p.parse_args()

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    chunk = max(1, -(-args.events // args.jobs))  # ceil
    begins = list(range(0, args.events, chunk))
    ends = [min(b + chunk, args.events) for b in begins]

    worker = partial(
        record_chunk,
        gdml=Path(args.gdml).resolve(),
        tracks=args.tracks,
        eta_range=tuple(args.eta_range),
        out_dir=out_dir.resolve(),
        seed=args.seed,
    )

    with Pool(len(begins)) as pool:
        files = pool.starmap(worker, zip(begins, ends))

    print(f"\nwrote {len(files)} chunk(s):")
    for f in files:
        print(f"  {f}")
    print("\nfeed them all to the mapping job:")
    print("  --inputFiles " + " ".join(files))


if __name__ == "__main__":
    main()
