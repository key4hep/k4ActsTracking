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
sensitive surfaces and throws the passive material away, then puts that material
back by *projecting* it onto a small number of dedicated surfaces. Getting that
material back is a four step process:

1. **Designate** which surfaces receive the projected material.
2. **Record** the material seen by geantinos shot through the full Geant4
   geometry.
3. **Map** the recorded material onto the designated surfaces and write it out.
4. **Read back** the resulting map when running reconstruction.

This document covers the Gen3 (blueprint) geometry built by `ActsGeoSvc`.

> **The Gen1 files in `data/` are not usable here.** `MAIA_v0_material.json` is a
> Gen1 *markup* file (every entry has null material), and `material-maps.json` is
> keyed on Gen1 TGeo volume/layer/approach identifiers, which a blueprint-built
> geometry does not have. Do not pass either to `MaterialMapFile`.

## What changed relative to the Gen1 workflow

The Gen1 workflow used a JSON file for two jobs at once: marking up which
surfaces should receive material, and serialising the mapped material. Step 1 no
longer needs a file. The receivers are designated *in code*, while the blueprint
tree is built, with `Acts::MaterialDesignatorBlueprintNode`. Only steps 3 and 4
still exchange a file.

## 1. Designation

Done in
[`DD4hepBlueprintConstruction.cpp`](../k4ActsTracking/src/components/DD4hepBlueprintConstruction.cpp),
see the `Material designation` block in the `Blueprints` namespace. Currently
wired up for **MAIA_v0** (and MuSIC_v2, which shares its blueprint).

Two ACTS rules decide which faces may be designated:

- a portal **shared** by two adjacent volumes (*fused* while stacking) may carry
  material from only one of the two sides — otherwise
  `Cannot fuse portals if both have material`;
- a face that a container has to **merge** while stacking its children may not
  carry material at all; construction aborts.

Which faces are which follows from the stacking direction:

| container stacks along | children's cylinder faces | children's disc faces |
| ---------------------- | ------------------------- | --------------------- |
| r                      | fused (safe)              | merged (unsafe)       |
| z                      | merged (unsafe)           | fused (safe)          |

A face that is *interior* to a stack is always safe. A face at the extreme of a
stack becomes the container's own face and propagates up the tree, where an
ancestor may merge it.

The scheme that follows from this: **designate, on every layer volume, only the
face pointing towards the interaction point** — the inner cylinder of a barrel
layer, and the IP-facing disc of an endcap layer. That is exactly the portal the
layer shares with its inward neighbour, so only one side is ever designated.

The exception is a layer whose IP-facing face happens to be the **extreme** face
of its container rather than an interior one. Such a face is not a shared portal
at all; it becomes the container's own face and is merged higher up. Those layers
are excluded with `LayerMaterial::skipInnermost`.

Note the rule is "*never designate a face that is the extreme of its container*",
**not** "always skip layer 0". Whether the innermost layer's inner cylinder is
the container extreme depends on what else sits inside the same radial stack, and
in MAIA the three barrels do not agree:

- `OuterTrackerBarrel` is a plain radial stack `[layer0 | layer1 | layer2]`, so
  `layer0`'s inner cylinder *is* the container's inner cylinder. The container is
  then a child of the `OuterTracker` z-stack, which merges its children's
  cylinder faces — so `layer0` **is skipped**. Removing the skip aborts
  construction with:

  ```
  [OuterTracker]: Material is designated on portal faces that are merged when stacking child volumes in AxisZ direction.
    - CylinderStackShell(dir=AxisR, 3 children) carries material on face InnerCylinder
  ```

- `InnerTrackerBarrel` is *not* a plain stack: `makeNestedInnerTracker` puts the
  whole `Vertex` node inside it as a radial child, giving
  `[Vertex | layer0 | layer1]`. So `layer0`'s inner cylinder is an interior
  portal, fused with `Vertex`'s outer cylinder — and since nothing designates the
  vertex's outer side, exactly one side carries material. `layer0` **is
  designated**.

- `VertexBarrel` is a plain stack inside the `Vertex` z-stack, so `layer_0`
  **is skipped**, for the same reason as the outer tracker.

Skipping loses nothing, because the excluded face always survives as part of a
receiver that *is* designated. Following the outer-tracker case up the tree:
`OuterTrackerBarrel.InnerCylinder` → merged into `OuterTracker.InnerCylinder` →
fused with `InnerTracker.OuterCylinder`, which is designated. The two are the
same physical portal, so designating both would have been a double designation of
one surface even if it were legal. Likewise the vertex barrel's skipped face is
the beampipe's outer cylinder, shared all the way down.

### The solenoid

The layer surfaces alone are not enough for a detector whose solenoid sits
between the tracker and the calorimeter, as MAIA_v0's does: the coil spans
**r = 1500 → 1857 mm** and the ECAL barrel face is at **1857 mm**, so every
extrapolation to the calorimeter face crosses several hundred mm of vacuum tank
and conductor.

The radial stack closes gaps by expanding the inner volume, so the outer tracker
— whose sensors end at 1498.5 mm — was stretched all the way out to 1856 mm,
leaving the coil straddling a single volume with no boundary anywhere inside it.
Its material could only be projected onto the outermost tracker layer (well inside
the coil) or onto that stretched boundary (hard against the calorimeter face).

`addCylindricalSolenoid` inserts a `Solenoid` volume covering the coil, which
splits that region:

```
OuterTracker   612.50 → 1499.80   ← outer cylinder designated (coil inner face)
Solenoid      1499.80 → 1856.00   ← outer cylinder designated (coil outer face)
CaloBarrel    1856.00 → 1923.51       ECAL face at 1857
```

The two designations now bracket the coil, so the mapping can spread its material
across the thickness instead of lumping it at one edge. A surface *inside* the
coil might be better still; that is the obvious next refinement.

The inner radius is read from the `Solenoid` DetElement's envelope shape rather
than hard-coded, and the insertion is **guarded** on the coil actually lying
inside the calorimeter face. MuSIC_v2 shares this blueprint but puts its coil at
r = 2055 mm, outside its calorimeter (1690 → 1960 mm), where it is irrelevant for
the extrapolation and a volume would not fit the radial stack at all — so it
correctly gets none.

### Receiver inventory

For MAIA this yields **51 receivers**:

| receiver                                        | count |
| ----------------------------------------------- | ----: |
| `Beampipe` outer cylinder                           |     1 |
| `VertexBarrel` layers (`layer_0` skipped: extreme)  |     4 |
| `VertexEndcap` layers                               |    16 |
| `InnerTrackerBarrel` layers (none skipped: interior) |     3 |
| `InnerTrackerEndcap` layers                         |    14 |
| `OuterTrackerBarrel` layers (`layer0` skipped: extreme) |  2 |
| `OuterTrackerEndcap` layers                         |     8 |
| `InnerTracker` / `OuterTracker` outer cylinders     |     2 |
| `Solenoid` outer cylinder                           |     1 |

`ActsGeoSvc` reports the total on every run:

```
ActsGeoSvc INFO 51 of 76706 surfaces in the tracking geometry carry material, 51 of them a proto-material placeholder.
```

Getting the designation wrong is not silent: ACTS throws during construction and
names the offending portal. The `load_geo_*` tests catch it.

### Adding a detector

The helpers in the `Blueprints` namespace all take optional material arguments
that default to "no material", so the other detectors (`ILD_FCCee_v01/v02`,
`CLD_o2_v07`, `LUXE_v0`) are unaffected. To add one, work out its tree's fused
and merged faces using the table above, then pass `LayerMaterial` into
`makeGroupedBarrel` / `makeBarrel` / `attachEndcaps` / `makeRegularTracker` /
`makeNestedInnerTracker`, and wrap whole containers with `withMaterial` where a
subdetector envelope should also receive material.

Also check for passive structures that sit in the path to the calorimeter face
but have no volume in the tree — a solenoid, a support tube, a cryostat. Anything
like that ends up inside whichever volume the radial stack stretches over it,
with no surface nearby to project onto. `addCylindricalSolenoid` is the pattern:
give the structure a volume of its own so its boundaries become designatable
surfaces.

## 2. Recording the geantino scan

The recording samples the **Geant4** geometry. It knows nothing about the
blueprint or the designated surfaces, so all it depends on is the DD4hep compact
file being the same one reconstruction uses.

### 2a. Export the detector to GDML

Export the geometry to GDML instead and use ACTS'
`GdmlDetector`, which goes straight to Geant4 and skips the ACTS conversion
entirely.

```bash
ddsim --compactFile $k4geo_DIR/MuColl/MAIA/compact/MAIA_v0/MAIA_v0.xml --outputFile /tmp/gdmldump.root --numberOfEvents 1 --enableGun --geometry.dumpGDML MAIA_v0.gdml
```

This produces a ~44 MB `MAIA_v0.gdml`. Check the log is free of shape conversion
errors — anything DD4hep cannot express in GDML silently drops out of the scan
and its material will be missing from the map.

### 2b. Run the scan

For a quick scan, ACTS' own script is installed with the stack and takes a GDML
file directly:

```bash
python3 $ACTS_SCRIPTS/material_recording.py --input MAIA_v0.gdml -n 1000 -t 1000 --eta-range -4 4 -o geant4_material_tracks
```

For a real scan, split it across batch jobs with
[`material_recording_chunk.py`](../k4ActsTracking/examples/material_recording_chunk.py).
Job `i` of `N`, each recording `E` events:

```bash
python3 k4ActsTracking/examples/material_recording_chunk.py --input MAIA_v0.gdml \
    --events E --skip $((i * E)) --tracks 1000 --output-dir scan/
```

Each job writes `scan/geant4_material_tracks_<skip>.root`. Pass them all to the
mapping step at once — `MaterialMappingAlg` chains its `InputFiles`, so there is
no `hadd` step.

> **Why the wrapper is needed.** ACTS' script exposes neither `--skip` nor
> `--seed`, and hardcodes `RandomNumbers(seed=228)`. Submitting N batch jobs with
> it as-is records the *same* geantinos N times: N times the files and the wall
> time, one scan's worth of statistics, and nothing in the output looks wrong.
> The wrapper adds only the missing knob — it calls ACTS' `runMaterialRecording`
> unchanged, so the generator and `RootMaterialTrackWriter` settings stay exactly
> what step 3 expects.

**The scan is single-threaded, and cannot be otherwise.**
`material_recording.py` hardcodes `numThreads=1`, and that is not an oversight:
ACTS creates its Geant4 run manager as
`G4RunManagerFactory::CreateRunManager(G4RunManagerType::SerialOnly)`
(`Examples/Algorithms/Geant4/src/Geant4Manager.cpp`), i.e. never Geant4's MT or
Tasking run manager, and `Geant4Manager` is a process-wide singleton whose
`createHandle` throws *"creating a second handle is prohibited"*.

> `--skip` is what makes chunks differ: the Sequencer derives its per-event
> random seeds from the event number. Verified — two runs differing only in
> `--skip` produce entirely different geantino directions, so the seed does not
> also need to vary.

**Statistics.** In the MAIA_v0 case, the 51 receivers carry roughly 15 000 bins
in total (cylinder
faces 20 × 20, disc faces 10 × 20). At ~100 entries per bin you need of order
1.5 M surface crossings; a geantino crossing the full barrel hits on the order of
10 receivers, so O(10⁶) geantinos is a sensible starting point. The scan is
cheap; do not skimp on it. Check the occupancy of the resulting map and scale up
if bins are empty.

**Compatibility checklist.** Both scripts above configure the writer correctly,
but if you write your own driver these must hold, because they are what the step
3 reader is configured against:

| `RootMaterialTrackWriter` | value | why |
| ------------------------- | ----- | --- |
| `treeName`                | `material_tracks` | what the reader opens |
| `prePostStep`             | `True`  | Geant4 steps need pre/post positions |
| `recalculateTotals`       | `True`  | cross-check of the per-step sums |
| `storeSurface`            | `False` | the scan knows nothing of our surfaces |
| `storeVolume`             | `False` | likewise |

## 3. Mapping

> **The mapping must run against *our* geometry.** This is the one thing that
> cannot be delegated to an external ACTS job. The map is keyed by
> `Acts::GeometryIdentifier`, and a stock ACTS DD4hep conversion produces
> completely different volume numbering from our blueprint. A map produced
> outside this repository will load without complaint and decorate nothing (or,
> worse, the wrong surfaces).
>
> This matters more now that `acts.examples` *is* importable in the stack: ACTS
> ships its own `Examples/Scripts/Python/material_mapping.py`, and it will run
> happily here. Do not use it. It maps onto whatever geometry it built itself,
> not onto ours.

That is why the mapping is a Gaudi algorithm here rather than an external
script: `MaterialMappingAlg` runs in the same job as `ActsGeoSvc`, so it projects
onto the very geometry reconstruction will use. It needs no dependencies beyond
what `k4ActsTrackingPlugins` already links (`Acts::Core` for the mapper,
`Acts::PluginRoot` to read the scan, `Acts::PluginJson` to write the map).

```bash
k4run test/options/material_mapping.py \
  --compactFile $k4geo_DIR/MuColl/MAIA/compact/MAIA_v0/MAIA_v0.xml \
  --inputFiles geant4_material_tracks.root \
  --outputFile MAIA_v0_gen3_material_map.json
```

Expected output:

```
MaterialMappingAlg INFO Mapping material onto 51 designated surfaces.
MaterialMappingAlg INFO Reading 20000 recorded material tracks from tree 'material_tracks'.
MaterialMappingAlg INFO Mapped 20000 recorded material tracks.
MaterialMappingAlg INFO Wrote material for 51 surfaces to 'MAIA_v0_gen3_material_map.json'.
```

The surface count on the first and last lines must agree, and must match what
`ActsGeoSvc` reports for the geometry. If the last number is smaller, some
receivers were never crossed by a geantino.

**Do not pass `--materialMapFile` to the mapping job.** The mapper identifies its
targets by their proto-material placeholders, and loading a map replaces exactly
those. The algorithm refuses to run if it finds no receivers, which is what that
mistake looks like.

### Properties

| property | default | note |
| -------- | ------- | ---- |
| `InputFiles` | *(required)* | scan ROOT files; several may be chained |
| `TreeName` | `material_tracks` | must match the recording job's `treeName` |
| `OutputFile` | `material-map.json` | `.json` via `MaterialMapJsonConverter` |
| `MaxTracks` | `-1` | cap for quick smoke tests; negative reads everything |
| `EmptyBinCorrection` | `true` | correct bins no geantino crossed |
| `PrePostStepInfo` | `true` | must match the writer's `prePostStep` |
| `SurfaceInfo` | `false` | must match the writer's `storeSurface` |
| `VolumeInfo` | `false` | must match the writer's `storeVolume` |

The last three are the ones to check first if the input will not read: they
select which branches the reader binds, so a mismatch with step 2's writer
settings shows up as ROOT branch errors rather than as wrong numbers.

### Two things about the job structure

It is a **one-shot job, not an event loop**: the algorithm reads the whole scan
on its first `execute()`. The options file therefore sets `EvtMax=1` *and*
`EvtSel="NONE"`. The second one matters — without an event selector there is no
event source, the loop runs zero events, and the algorithm initializes and
finalizes without ever executing. That failure is silent apart from the
algorithm refusing to write an empty map.

The receivers are found with `MaterialSurfaces::collectProtoMaterialSurfaces`
([`MaterialSurfaces.h`](../k4ActsTracking/src/components/MaterialSurfaces.h)),
which `ActsGeoSvc` also uses for its proto count, so the two cannot disagree
about what counts as a designated surface.

The writer is configured for a Gen3 geometry: `processBoundaries = true`,
everything else off. The receivers are volume portals, the sensors carry no
material, and there is no volume material.

## 4. Reading the map back

`Acts::Blueprint::construct` always passes a null material decorator to the
`Acts::TrackingGeometry` constructor and closes the geometry itself, so the Gen1
closure path that normally applies a decorator never runs. `ActsGeoSvc`
therefore applies the map itself, after construction, by walking the geometry and
decorating volumes, portal surfaces and sensitive surfaces
(`MaterialDecorationVisitor` in
[`ActsGeoSvc.cpp`](../k4ActsTracking/src/components/ActsGeoSvc.cpp)).

Point the `MaterialMapFile` property at the map:

```bash
k4run test/options/MAIA_CKFTrackingAlg.py \
  --compactFile $k4geo_DIR/MuColl/MAIA/compact/MAIA_v0/MAIA_v0.xml \
  --materialMapFile MAIA_v0_gen3_material_map.json
```

A `.root` file is read with `ActsPlugins::RootMaterialDecorator`, anything else
(`.json`, `.cbor`) with `Acts::JsonMaterialDecorator`. A missing or unparseable
file fails service initialisation with a message naming the file.

Check the summary line afterwards. The proto count is the diagnostic:

- `51 ... 51 of them a proto-material placeholder` — no map applied, or none of
  its entries matched. A proto placeholder contributes no actual material, so
  tracking still sees none; the service warns about this.
- `51 ... 0 of them a proto-material placeholder` — the map filled in every
  designated receiver. This is what a correct map looks like.

## 5. Testing the result

Work up this ladder; each rung isolates a different failure.

**5a. The designation is intact (no map needed).**

```bash
ctest -R load_geo --output-on-failure
```

Every detector must construct. A designation on a merged portal aborts here with
the offending portal named. Then confirm the receiver count:

```bash
k4run k4ActsTracking/examples/test_visualize_acts_geo.py --compactFile $k4geo_DIR/MuColl/MAIA/compact/MAIA_v0/MAIA_v0.xml
```

Expect `51 ... 51 of them a proto-material placeholder`. A different total means
the blueprint changed and **any existing map is now invalid** — regenerate it.

**5b. The map keys line up.** The cheapest real check, and the one that catches a
map produced against the wrong geometry:

```bash
k4run k4ActsTracking/examples/test_visualize_acts_geo.py --compactFile $k4geo_DIR/MuColl/MAIA/compact/MAIA_v0/MAIA_v0.xml --materialMapFile MAIA_v0_gen3_material_map.json
```

- proto count drops `51 → 0`: correct.
- proto count unchanged at 51: none of the map's identifiers matched. The map was
  built against a different geometry — almost always an external ACTS job, or a
  map predating a blueprint change.
- proto count drops only partly: some receivers got no material. Usually too few
  geantinos, or a receiver the scan never crosses.
- **total** rises above 51: the map carries entries for surfaces we never
  designated. Harmless but a sign the map writer config in step 3 was too
  permissive (`processSensitives` left on).

**5c. Round-trip check with a synthetic scan.** You do not need a real Geant4
scan to exercise steps 3 and 4. Any file in the recording format works, so a
throwaway generator that writes straight rays with material steps along them
(using `ActsPlugins::RootMaterialTrackIo` in write mode, so the format is
guaranteed to match) is enough to confirm the plumbing before committing to a
long scan. Running that through `material_mapping.py` and back through
`MaterialMapFile` should give `51 -> 0` proto, a map with 51 `(volume,
boundary)` entries of type `binned`. It will not give physically meaningful
material, only a correct pipeline.

**5d. Physics validation.** `Acts::MaterialValidator` and
`Acts::PropagatorMaterialAssigner` are in the stack: propagate geantinos through
the *mapped* geometry and compare the accumulated X₀/L₀ against the original
scan, binned in η and φ. Agreement to a few percent is the target; a systematic
deficit means material that fell outside every receiver, which for MAIA most
likely means the region between the outer tracker and the solenoid, or the
nozzles.

**5e. No tracking regression.**

```bash
ctest -R reco_MAIA_Gen3 --output-on-failure
```

Note that adding real material *should* change the fit results — this test checks
the chain still runs, not that the numbers are unchanged.

**5f. Add the map to the repository.** Once validated, add the file to
`data/file_list.txt` with its md5 and upload it alongside the other data files;
`data/CMakeLists.txt` downloads and installs it at configure time.

## Caveat: maps are tied to the blueprint that produced them

Maps are keyed by `Acts::GeometryIdentifier`. For a Gen3 geometry those are
assigned by traversal order in `Blueprint::construct` (volume / boundary /
sensitive — there is no layer or approach component). **Changing the blueprint
tree silently invalidates an existing map**, because the same identifier now
refers to a different surface. Regenerate the map whenever
`DD4hepBlueprintConstruction.cpp` changes the MAIA tree.

`Acts::PortalDesignatorBlueprintNode` (ACTS
[#5593](https://github.com/acts-project/acts/pull/5593)) gives portals stable
string tags and is the obvious way to make this robust later; it is not used
yet.
