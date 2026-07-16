# k4ActsTracking

[![Key4hep build](https://github.com/key4hep/k4ActsTracking/actions/workflows/key4hep-build.yaml/badge.svg)](https://github.com/key4hep/k4ActsTracking/actions/workflows/key4hep-build.yaml)
[![downstream-build](https://github.com/key4hep/k4ActsTracking/actions/workflows/downstream-build.yaml/badge.svg)](https://github.com/key4hep/k4ActsTracking/actions/workflows/downstream-build.yaml)
[![MuColl Image Build](https://github.com/key4hep/k4ActsTracking/actions/workflows/mucoll-ci.yml/badge.svg)](https://github.com/key4hep/k4ActsTracking/actions/workflows/mucoll-ci.yml)
[![pre-commit](https://github.com/key4hep/k4ActsTracking/actions/workflows/pre-commit.yml/badge.svg)](https://github.com/key4hep/k4ActsTracking/actions/workflows/pre-commit.yml)

`k4ActsTracking` provides [ACTS](https://acts.readthedocs.io/)-based track
reconstruction components for the [Key4hep](https://key4hep.github.io/key4hep-doc/)
software stack. It converts DD4hep tracking geometries into ACTS geometries and
exposes a set of Gaudi algorithms and services (geometry conversion, seeding,
combinatorial Kalman filter track finding, duplicate removal, track filtering
and truth matching) usable from `k4run` option files.

## Dependencies

* [ACTS](https://github.com/acts-project/acts) (`Core`, `PluginDD4hep`,
  `PluginJson`, `PluginRoot`)
* [DD4hep](https://github.com/AIDASoft/DD4hep) (`DDCore`, `DDRec`)
* [EDM4hep](https://github.com/key4hep/EDM4hep)
* [k4FWCore](https://github.com/key4hep/k4FWCore)
* [Gaudi](https://gitlab.cern.ch/gaudi/Gaudi)
* TBB

All of these are provided by a Key4hep release. The easiest way to get a working
environment is to source a nightly or stable Key4hep stack, e.g.:

```sh
source /cvmfs/sw-nightlies.hsf.org/key4hep/setup.sh
```

## Building

```sh
mkdir -p build install
cmake -B build -S . -GNinja \
  -DCMAKE_CXX_STANDARD=20 \
  -DCMAKE_INSTALL_PREFIX=$(pwd)/install \
  -DCMAKE_CXX_FLAGS=" -fdiagnostics-color=always -Werror -Wno-error=deprecated-declarations"
cmake --build build
cmake --build build --target install
```

After installing, make the package visible to Gaudi/`k4run`:

```sh
source ../install/setup.sh   # if generated, otherwise set the paths below
# or, manually:
export LD_LIBRARY_PATH=$PWD/install/lib:$LD_LIBRARY_PATH
export PYTHONPATH=$PWD/install/python:$PYTHONPATH
```

## Components

The Gaudi plugin module `k4ActsTrackingPlugins` provides, among others:

* **`ActsGeoSvc`** — builds an ACTS tracking geometry from a DD4hep compact
  file (consumed via `GeoSvc`). Can optionally dump the converted geometry to an
  `.obj` file for visualization.
* **`CKFTrackingAlg`** — seeding plus combinatorial Kalman filter (CKF) track
  finding on EDM4hep tracker hits. Seeds are built either with the cylindrical
  helix seeder (default, for collider/barrel geometries) or a straight-line
  telescope seeder for field-free planar detectors (`SeedingMode`). Fitted tracks
  can optionally be extrapolated to the calorimeter face to add an
  `AtCalorimeter` track state (`ExtrapolateToCalo`).
* **`CKFTrackingFromSeedsAlg`** — runs the same CKF, but seeded from an existing
  input track collection (e.g. candidates from an upstream pattern-recognition
  stage) instead of the internal seed finder.
* **`ACTSSeededCKFTrackingAlg`** — legacy seeded CKF tracking algorithm,
  superseded by `CKFTrackingAlg` and slated for removal.
* **`ACTSDuplicateRemoval`** — removes duplicate tracks produced by the CKF.
* **`FilterTracksAlg`** — applies quality cuts to a track collection.
* **`TrackTruthAlg`** — associates reconstructed tracks with truth particles.
* **`ActsTestPropagator`** — propagates ACTS particle-gun tracks through the
  converted geometry (useful for geometry validation).

## Usage

The algorithms are configured and run through `k4run` option files. See the
examples and the test option files for working configurations:

* [`k4ActsTracking/examples/test_visualize_acts_geo.py`](k4ActsTracking/examples/test_visualize_acts_geo.py)
  — load a compact file, convert it to ACTS geometry, optionally dump an `.obj`
  and run the test propagator:

  ```sh
  k4run k4ActsTracking/examples/test_visualize_acts_geo.py \
        --compactFile <detector>.xml --test-propagation
  ```

* [`test/options/MAIA_CKFTrackingAlg.py`](test/options/MAIA_CKFTrackingAlg.py),
  [`test/options/CLD_CKFTracking.py`](test/options/CLD_CKFTracking.py),
  [`test/options/ILD_CKFTracking.py`](test/options/ILD_CKFTracking.py) — full
  digitization + CKF tracking chains for the MAIA, CLD and ILD detectors (all
  using the default cylindrical seeding). The shared helpers live in
  [`test/options/_ckf_helpers.py`](test/options/_ckf_helpers.py).

* [`test/options/LUXE_CKFTracking.py`](test/options/LUXE_CKFTracking.py) — CKF
  tracking for the field-free LUXE telescope geometry, exercising the
  straight-line telescope seeding mode (`SeedingMode="Telescope"`).

  > **Note:** the parameters in these option files are tuned only for technical
  > tests and are *not* a meaningful physics tracking configuration.

## Tests

Tests are built when `BUILD_TESTING` is on (default) and run with `ctest` from
the build directory:

```sh
cd build
ctest --output-on-failure
```

The suite covers C++ unit tests plus end-to-end chains (`ddsim` simulation →
reconstruction → CKF tracking) for the MAIA, CLD and ILD geometries. The CLD and
ILD chains clone the corresponding upstream config repositories
([`CLDConfig`](https://github.com/key4hep/CLDConfig),
[`ILDConfig`](https://github.com/iLCSoft/ILDConfig)) on the fly, and geometries
are taken from `k4geo` (via `$k4geo_DIR`).

## License

Licensed under the Apache License, Version 2.0. See [LICENSE](LICENSE).
