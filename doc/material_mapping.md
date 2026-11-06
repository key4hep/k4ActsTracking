<!--
Copyright (c) 2014-2024 Key4hep-Project.

This file is part of Key4hep.
See https://key4hep.github.io/key4hep-doc/ for further info.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
-->
# Material mapping for the Gen3 (blueprint) geometry

ACTS does not use the full simulation geometry for tracking. It keeps the
sensitive surfaces, drops the passive material, and puts that material back by
projecting it onto a few dedicated surfaces. The concepts are explained in the
[ACTS material mapping how-to](https://acts-project.github.io/material_mapping_howto.html);
this page covers how to do it for the geometry `ActsGeoSvc` builds:

1. **Designate** the surfaces that receive the material (in the blueprint).
2. **Record** the material along straight geantino rays.
3. **Map** the recorded material onto the designated surfaces.
4. **Use** the map in reconstruction.
5. **Validate** the map.

The options files for steps 2, 3 and 5 are installed in
`share/k4ActsTracking/options` (`k4ActsTracking/options` in the sources), and
the plotting script `compare_material_tracks.py` in
`share/k4ActsTracking/scripts`. For MAIA_v0, steps 2 to 5 are:

```bash
SHARE=$LCG_VIEW_DIR/share/k4ActsTracking  # or <install prefix>/share/k4ActsTracking
COMPACT=$k4geo_DIR/MuColl/MAIA/compact/MAIA_v0/MAIA_v0.xml
mkdir -p scan
for seed in 1 2 3 4 5 6 7 8 9 10; do
  k4run $SHARE/options/material_recording.py --compactFile $COMPACT \
    --numTracks 100000 --seed $seed --outputFile scan/material_tracks_$seed.root
done
k4run $SHARE/options/material_mapping.py --compactFile $COMPACT \
  --inputFiles scan/ --outputFile MAIA_v0_gen3_material_map.json
k4run $SHARE/options/material_validation.py --compactFile $COMPACT \
  --materialMapFile MAIA_v0_gen3_material_map.json \
  --inputFiles scan/ --outputFile propagated_material_tracks.root
$SHARE/scripts/compare_material_tracks.py scan/ propagated_material_tracks.root -o validation/
```

The Gen1 files in `data/` (`MAIA_v0_material.json`, `material-maps.json`) use
a different key scheme and do not work with a blueprint-built geometry.

**A map belongs to the geometry and blueprint it was made with.** Its
`Acts::GeometryIdentifier`s are assigned in traversal order of the blueprint, so
a change to a detector's tree in `DD4hepBlueprintConstruction.cpp` puts the
material on the wrong surfaces, without an error, and a change to the geometry
makes the recorded material out of date. Redo the map after either.

## 1. Designation

Receivers are designated in
[`DD4hepBlueprintConstruction.cpp`](../k4ActsTracking/src/components/DD4hepBlueprintConstruction.cpp),
in the `Material designation` block of the `Blueprints` namespace. This is done
for **MAIA_v0** (and MAIA_v1 and MuSIC_v2, which share its blueprint); the
other detectors carry no material yet.

The scheme is: **on every layer volume, designate the face pointing towards the
interaction point** (the inner cylinder of a barrel layer, the IP-facing disc of
an endcap layer). That face is shared with the inward neighbour, so every gap
between two layers gets one receiver. Two ACTS rules limit which faces can be
used:

- a portal **shared** by two volumes may carry material from one side only;
- a face that a container **merges** while stacking its children may not carry
  material at all.

| container stacks along | children's cylinder faces | children's disc faces |
| ---------------------- | ------------------------- | --------------------- |
| r                      | fused (one side allowed)  | merged (not allowed)  |
| z                      | merged (not allowed)      | fused (one side allowed) |

A layer whose IP-facing face is the extreme face of its container (rather than
shared with a neighbour inside it) is excluded with `LayerMaterial::skipInnermost`.
Nothing is lost: that face is the same surface as a receiver designated from the
other side (e.g. the beampipe's outer cylinder).

Passive structures without a volume of their own, such as MAIA's solenoid in
front of the calorimeter, get one so that their faces can be designated
(`addCylindricalSolenoid`).

For MAIA_v0 this gives 51 receivers. `ActsGeoSvc` prints the count on every
run:

```
ActsGeoSvc INFO 51 of 76706 surfaces in the tracking geometry carry material, 51 of them a proto-material placeholder.
```

A designation that breaks one of the rules aborts geometry construction and
names the portal.

### Adding material designators to a detector

1. Pass `LayerMaterial` to `makeGroupedBarrel` / `makeBarrel` /
   `attachEndcaps` / `makeRegularTracker` / `makeNestedInnerTracker`, and wrap
   containers with `withMaterial` where an envelope face should receive
   material. Without these arguments a helper designates nothing.
2. Give passive structures between the tracker and the calorimeter a volume, as
   above.
3. Check the scan region the recording and mapping jobs print (`Rays stop at
   ...`, `Mapping recorded material at ...`): it must be the ECAL inner face
   and enclose every receiver. It is taken from the first detector, by name,
   with the type flags `CALORIMETER | ELECTROMAGNETIC | BARREL` (not
   `FORWARD`) or `CALORIMETER | ELECTROMAGNETIC | ENDCAP` and its
   `LayeredCalorimeterData`. If that picks the wrong detector, set `--maxR` /
   `--maxZ` in both jobs.

## 2. Recording

A geantino neither interacts nor bends, so the recording is ray tracing: each
ray from the origin notes every volume it crosses, with its material and path
length. The recording knows nothing about the receivers; it only needs the same
compact file as reconstruction.

[`MaterialRecordingAlg`](../k4ActsTracking/src/components/MaterialRecordingAlg.cpp)
walks the rays with the TGeo navigator through the geometry `GeoSvc` builds:

```bash
k4run $SHARE/options/material_recording.py \
  --compactFile $k4geo_DIR/MuColl/MAIA/compact/MAIA_v0/MAIA_v0.xml \
  --numTracks 100000 --seed 1 \
  --outputFile scan/material_tracks_1.root
```

- Directions are flat in η (`--etaRange`, default −4 to 4) and φ.
- A job is single-threaded; 100 000 rays take about a minute. For a larger scan
  run several jobs with different `--seed`s into one directory.
- Rays stop at the ECAL inner face (printed as `Rays stop at ...`);
  `--maxR` / `--maxZ` (mm) set other values. A detector without a calorimeter
  face falls back to the extent of the tracking world.
- `Air` and `Vacuum` are not recorded, as in ACTS' Geant4 recording.
- `--replayFiles <files or directories>` takes the start points and directions
  from an existing scan instead of sampling new ones, so the two scans can be
  compared track by track.

### Cross-check with Geant4

ACTS provides the same recording as a ddsim plugin (`ActsMaterialRecorder`,
[acts#6304](https://github.com/acts-project/acts/pull/6304)), which runs Geant4
on the geometry DDG4 builds. It needs an ACTS build with the DD4hep and Geant4
plugins (library `ActsPluginDD4hepGeant4` on `LD_LIBRARY_PATH`), which the
Key4hep stack does not provide at the moment, and the steering file from ACTS'
`Examples/Scripts/Python` (`$ACTS_SCRIPTS` below):

```bash
ddsim --steeringFile $ACTS_SCRIPTS/material_recording_ddsim.py \
  --compactFile $k4geo_DIR/MuColl/MAIA/compact/MAIA_v0/MAIA_v0.xml \
  --numberOfEvents 200 --outputFile geant4_material_tracks.root \
  --outputConfig.recordingMaxR=1857 \
  --outputConfig.recordingMinZ=-2307.3 --outputConfig.recordingMaxZ=2307.3
```

The steering file shoots 1000 geantinos per event. The `recording*` limits
stop the rays at the ECAL face as above; the mapping clips its input to that
region anyway. To compare with `MaterialRecordingAlg` along the same
directions:

```bash
k4run $SHARE/options/material_recording.py \
  --compactFile $k4geo_DIR/MuColl/MAIA/compact/MAIA_v0/MAIA_v0.xml \
  --replayFiles geant4_material_tracks.root --outputFile tgeo_material_tracks.root
```

## 3. Mapping

[`MaterialMappingAlg`](../k4ActsTracking/src/components/MaterialMappingAlg.cpp)
uses the `ActsGeoSvc` to build the ACTS geometry and projects the scan onto
its receivers. It yields a material map that is guaranteed to have the same
`Acts::GeometryIdentifier`s as the geometry that will be used for tracking. The
version of `material_mapping.py` shipped by ACTS **does not yield usable
material maps.**

```bash
k4run $SHARE/options/material_mapping.py \
  --compactFile $k4geo_DIR/MuColl/MAIA/compact/MAIA_v0/MAIA_v0.xml \
  --inputFiles scan/ \
  --outputFile MAIA_v0_gen3_material_map.json
```

`--inputFiles` takes files and directories; every `*.root` file directly inside
a directory is chained in sorted order, so there is no `hadd` step.

Output for 1M rays from `MaterialRecordingAlg` (progress lines left out):

```
MaterialMappingAlg INFO Mapping material onto 51 designated surfaces.
MaterialMappingAlg INFO Mapping recorded material at r < 1857.0 mm, |z| < 2307.3 mm.
MaterialMappingAlg INFO Reading 1000000 recorded material tracks from tree 'material_tracks'.
MaterialMappingAlg INFO Mapped 1000000 recorded material tracks.
MaterialMappingAlg INFO Material in x/X0 summed over all tracks: 1.811e+08 mapped, 0.285 outside the region, 0 on tracks crossing no receiver.
MaterialMappingAlg INFO Wrote material for 51 surfaces to 'MAIA_v0_gen3_material_map.json'.
```

- The surface counts on the first and last line should agree with the
  `ActsGeoSvc` count; a smaller last number means some receivers were never
  crossed.
- Recorded material outside the scan region is not mapped, since the mapper
  gives every step to the nearest receiver and material behind the ECAL face
  would pile up on the outermost ones. For a scan that already stops there,
  "outside" is only rounding at the boundary. Keep `--maxR` / `--maxZ` equal to
  the recording's if you set them.
- "On tracks crossing no receiver" should be zero; that material is lost.
- Cylinders are binned 20 (φ) × 200 (z), discs 50 (r) × 20 (φ): 90 000 bins for
  MAIA_v0. A few million rays bring them to the percent level. Some bins stay
  empty however many rays are used, so judge the statistics by comparing maps
  from two independent scans.

| property | default | note |
| -------- | ------- | ---- |
| `InputFiles` | *(required)* | scan ROOT files, chained |
| `TreeName` | `material_tracks` | |
| `OutputFile` | `material-map.json` | written with `MaterialMapJsonConverter` |
| `MaxTracks` | `-1` | cap for quick tests; negative reads everything |
| `EmptyBinCorrection` | `true` | correct bins no geantino crossed |
| `PrePostStepInfo`, `SurfaceInfo`, `VolumeInfo` | `true`, `false`, `false` | must match how the scan was written; a mismatch shows up as ROOT branch errors |
| `MaxR`, `MaxZ` | `-1` | scan region (mm); non-positive: the ECAL inner face |

## 4. Using the map

Pass the map to `ActsGeoSvc` with the `MaterialMapFile` property, e.g. with
`--materialMapFile` of the CKF test jobs in `test/options`:

```bash
k4run test/options/MAIA_CKFTrackingAlg.py \
  --compactFile $k4geo_DIR/MuColl/MAIA/compact/MAIA_v0/MAIA_v0.xml \
  --materialMapFile MAIA_v0_gen3_material_map.json
```

The `ActsGeoSvc` line shows whether the map applied:

- `51 of them a proto-material placeholder`: none of its entries matched (map
  made for a different geometry or blueprint). The service warns.
- `0 of them a proto-material placeholder`: every receiver got material, as
  it should.
- a number in between: some receivers got no material (too few rays, or a
  receiver no ray crosses).

## 5. Validation

[`MaterialValidationAlg`](../k4ActsTracking/src/components/MaterialValidationAlg.cpp)
propagates the scan's directions through the mapped geometry and records the
material it finds, in the same format as the scan and entry by entry, so the
two can be compared track by track. Give it the scan the map was made from:

```bash
k4run $SHARE/options/material_validation.py \
  --compactFile $k4geo_DIR/MuColl/MAIA/compact/MAIA_v0/MAIA_v0.xml \
  --materialMapFile MAIA_v0_gen3_material_map.json \
  --inputFiles scan/ \
  --outputFile propagated_material_tracks.root
```

`--assigner intersection` (default) intersects the receivers, as the mapping
does, and checks the content of the map; `--assigner propagator` uses the
navigator and also checks that tracking reaches the material. A propagation
that fails is written with no material and counted in a warning at the end;
exclude those entries from the comparison.

Then plot the comparison:

```bash
$SHARE/scripts/compare_material_tracks.py scan/ propagated_material_tracks.root -o validation/
```

This writes the material against η and φ for both, with their ratio
(`x0_vs_eta.png`, `x0_vs_phi.png`), the per-track ratio (`x0_ratio.png`) and
the ratio in η-φ (`x0_map_eta_phi.png`), and prints the totals. Inside the
tracking acceptance the ratio should be close to 1; a deficit in some η range
means material that is on no receiver. At the edge of dense forward material
(MAIA's tungsten nozzle) single receiver bins average rays through and beside
it, so the ratio there is far from 1; limit `--etaRange` of the recording if
that region matters.

## 6. Adding a map to the repository

Upload the validated file to the data server under its md5 hash
(`md5sum <file>`), commit `data/<file>.md5` containing that hash, and add the
file name to `data_files` in `data/CMakeLists.txt`, which downloads it at build
time and installs it.
