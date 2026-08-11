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

The optional GNN track finding (see below) additionally needs ACTS with the
`PluginGnn`, [ONNX Runtime](https://onnxruntime.ai/) and Torch. These are not
part of a Key4hep release yet, see
[`GNNTrackFinding/README.md`](GNNTrackFinding/README.md).

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

The GNN track finding is **not** built by default. Add
`-DK4ACTSTRACKING_BUILD_GNN=ON` to the `cmake` call to enable it; only then are
the additional dependencies required.

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
  `.obj` file for visualization, and load a material map (`MaterialMapFile`, see
  [doc/material_mapping.md](doc/material_mapping.md)).
* **`CKFTrackingAlg`** — seeding plus combinatorial Kalman filter (CKF) track
  finding on EDM4hep tracker hits. Seeds are built either with the cylindrical
  helix seeder (default, for collider/barrel geometries) or a straight-line
  telescope seeder for field-free planar detectors (`SeedingMode`). Fitted tracks
  can optionally be extrapolated to the calorimeter face to add an
  `AtCalorimeter` track state (`ExtrapolateToCalo`), placed at the first calo
  face the track reaches. A track entering the barrel face near the
  barrel/endcap corner goes on to enter the endcap too; `AddEndcapCaloState`
  (off by default) gives such a track one `AtCalorimeter` state per section,
  ordered as the track crosses them — barrel first, then endcap. Leave it off
  unless the consumer is prepared to see more than one state with that
  location, since looking the state up by location alone yields only the
  barrel one.
* **`CKFTrackingFromSeedsAlg`** — runs the same CKF, but seeded from an existing
  input track collection (e.g. candidates from an upstream pattern-recognition
  stage) instead of the internal seed finder.
* **`ACTSSeededCKFTrackingAlg`** — legacy seeded CKF tracking algorithm,
  superseded by `CKFTrackingAlg` and slated for removal.
* **`MaterialMappingAlg`** — projects a recorded Geant4 geantino scan onto the
  material surfaces designated by the blueprint and writes the resulting
  material map. Runs alongside `ActsGeoSvc` so the map matches the geometry it
  was built from; see [doc/material_mapping.md](doc/material_mapping.md).
* **`ACTSDuplicateRemoval`** — removes duplicate tracks produced by the CKF.
* **`FilterTracksAlg`** — applies quality cuts to a track collection.
* **`TrackTruthAlg`** — associates reconstructed tracks with truth particles.
* **`SortTrackerHitsAlg`** — reorders a tracker hit collection by one hit
  feature (`SortBy`, optionally `Descending`), using the same feature vocabulary
  as `GNNTrackFinder`: `x`, `y`, `z`, `r`, `phi`, `theta`, `eta`, `t` (or
  `time`), `E` (or `energy`), `module_id`, `layer_id`, `system_id` (or
  `volume_id`), all case insensitive. The output is a *subset* collection, so it
  refers to the very same hits and only changes their order; hits with an equal
  key keep their input order. Only the CellID based features need the
  `ActsGeoSvc` (for the CellID encoding), so sorting by e.g. `r` does not pull in
  the ACTS geometry. Useful to give a downstream algorithm a deterministic hit
  order — for instance to feed `GNNTrackFinder` a reproducible input when its
  embedding model has a fixed input length.

  ```python
  from Configurables import SortTrackerHitsAlg

  sorter = SortTrackerHitsAlg(
      "SortHitsByR",
      InputHitCollection=["VertexBarrelHits"],
      OutputHitCollection=["VertexBarrelHitsSortedByR"],
      SortBy="r",
      Descending=False,
  )
  ```
* **`ActsTestPropagator`** — propagates ACTS particle-gun tracks through the
  converted geometry (useful for geometry validation).

Additionally, the optional `GNNTrackingTrackFinding` plugin module (enabled with
`-DK4ACTSTRACKING_BUILD_GNN=ON`) provides:

* **`GNNTrackFinder`** — ML based track finding: a metric-learning ONNX model
  embeds the hits, edges are built in embedding space, one or more ONNX edge
  classifiers score them, and the resulting track candidates are fitted with the
  ACTS Kalman fitter. Hits can be segmented in theta/phi to keep the graphs
  small. See [`GNNTrackFinding/README.md`](GNNTrackFinding/README.md) for the
  dependencies, the full list of properties and an example configuration.

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

* [`GNNTrackFinding/options/runGNNTrackFinding.py`](GNNTrackFinding/options/runGNNTrackFinding.py)
  — GNN based track finding (only available with
  `-DK4ACTSTRACKING_BUILD_GNN=ON`), needs the ONNX models to be passed via
  `--modelBase`.

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
