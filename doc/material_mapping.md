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

This document covers the Gen3 (blueprint) geometry built by `ActsGeoSvc`. It
supersedes the [MuColl wiki
procedure](https://mcd-wiki.web.cern.ch/software/howto/acts_geo/), which applies
only to the deprecated Gen1/TGeo path (`ACTSAlgBase`, `MatFile`).

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

This also means the wiki's TGeo detour (`dd2tgeo`, `ActsExampleGeometryTGeo`,
`writeMapConfig.py` / `configureMap.py`) is gone entirely.

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

The blueprint originally had no volume there. The radial stack closes gaps by
expanding the inner volume, so the outer tracker — whose sensors end at
1498.5 mm — was stretched all the way out to 1856 mm, leaving the coil straddling
a single volume with no boundary anywhere inside it. Its material could only be
projected onto the outermost tracker layer (well inside the coil) or onto that
stretched boundary (hard against the calorimeter face).

`addCylindricalSolenoid` inserts a `Solenoid` volume covering the coil, which
splits that region:

```
OuterTracker   612.50 → 1499.80   ← outer cylinder designated (coil inner face)
Solenoid      1499.80 → 1856.00   ← outer cylinder designated (coil outer face)
CaloBarrel    1856.00 → 1923.51       ECAL face at 1857
```

The two designations now bracket the coil, so the mapping can spread its material
across the thickness instead of lumping it at one edge. A surface *inside* the
coil would be better still; that is the obvious next refinement.

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

This is the one step that needs tooling outside this repository: the key4hep and
MuColl stacks build ACTS with `Core`, `PluginDD4hep`, `PluginJson` and
`PluginRoot` only — no `ActsExamples`, no `acts` Python bindings. So the scan runs
in a separate ACTS build.

The recording only samples the **Geant4** geometry. It knows nothing about the
blueprint or the designated surfaces, so it depends solely on the DD4hep compact
file being the same one reconstruction uses.

### 2a. Export the detector to GDML

Do not hand MAIA's compact file to ACTS' `DD4hepDetector`: that class eagerly
builds a *Gen1* ACTS tracking geometry, which is exactly the conversion that does
not work for these detectors and is the reason this repository has its own
blueprint code. Export the geometry to GDML instead and use ACTS'
`GdmlDetector`, which goes straight to Geant4 and skips the ACTS conversion
entirely. It also means the ACTS container needs no k4geo installation.

Run this in the MuColl/key4hep stack:

```bash
ddsim --compactFile $k4geo_DIR/MuColl/MAIA/compact/MAIA_v0/MAIA_v0.xml --outputFile /tmp/gdmldump.root --numberOfEvents 1 --enableGun --geometry.dumpGDML MAIA_v0.gdml
```

This produces a ~44 MB `MAIA_v0.gdml`. Check the log is free of shape conversion
errors — anything DD4hep cannot express in GDML silently drops out of the scan
and its material will be missing from the map.

### 2b. Get an ACTS build with Examples, Geant4 and Python

Pin it to the **same commit the stack was built from**, otherwise the recorded
ROOT tree layout may not match what `ActsPlugins::RootMaterialTrackIo` expects
when reading it back in step 3. The stack reports version `999.999.999` (an
untagged `main` build); the commit is in the spack metadata:

```bash
grep -o '"commit":"[a-f0-9]*"' $(dirname $(dirname $(find /opt/spack -name libActsCore.so | head -1)))/.spack/spec.json | head -1
```

At the time of writing that is `f6eb3bf2d76b25a3c683cc7020dc799fc5cdb769`. Build
that commit with `-DACTS_BUILD_EXAMPLES=ON -DACTS_BUILD_EXAMPLES_GEANT4=ON
-DACTS_BUILD_EXAMPLES_PYTHON_BINDINGS=ON`, or use the official ACTS container at
the matching tag.

### 2c. Run the scan

`Examples/Scripts/Python/material_recording.py` in the ACTS source tree already
takes a GDML file:

```bash
python material_recording.py --input MAIA_v0.gdml -n 1000 -t 1000 --eta-range -4 4 -o geant4_material_tracks
```

That writes `geant4_material_tracks.root`.

**Statistics.** The 51 receivers carry roughly 15 000 bins in total (cylinder
faces 20 × 20, disc faces 10 × 20). At ~100 entries per bin you need of order
1.5 M surface crossings; a geantino crossing the full barrel hits on the order of
10 receivers, so ~10⁶ geantinos is a sensible starting point. Check the
occupancy in the resulting map and scale up if bins are empty — the accumulator
corrects for empty bins, but a map built from too few tracks is noisy rather than
obviously wrong.

**Compatibility checklist.** `material_recording.py` already configures the
writer correctly, but if you write your own driver these must hold, because they
are what the step 3 reader is configured against:

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

**This algorithm does not exist yet** — it is the remaining piece of work. All
the machinery is already in the stack and already linked by
`k4ActsTrackingPlugins`, so it needs no new dependencies. What it has to do:

1. **Get the geometry and the receivers.** Take `trackingGeometry()` from
   `IActsGeoSvc`, then collect the surfaces whose `surfaceMaterial()` is an
   `Acts::ProtoGridSurfaceMaterial` or `Acts::ProtoSurfaceMaterial`. That is
   exactly the test `MaterialDecorationVisitor` in
   [`ActsGeoSvc.cpp`](../k4ActsTracking/src/components/ActsGeoSvc.cpp) already
   does for its proto count — reuse it rather than reimplementing. Expect 51 for
   MAIA.

2. **Build the mapper** (all `Acts::`, from Core):

   ```cpp
   IntersectionMaterialAssigner::Config assignerCfg;
   assignerCfg.surfaces = materialSurfaces;

   BinnedSurfaceMaterialAccumulator::Config accCfg;
   accCfg.materialSurfaces   = materialSurfaces;
   accCfg.emptyBinCorrection = true;

   MaterialMapper::Config mapperCfg;
   mapperCfg.assignmentFinder = std::make_shared<IntersectionMaterialAssigner>(assignerCfg, ...);
   mapperCfg.surfaceMaterialAccumulator =
       std::make_shared<BinnedSurfaceMaterialAccumulator>(accCfg, ...);
   ```

3. **Read the tracks** with `ActsPlugins::RootMaterialTrackIo`, configured to
   match the writer settings from step 2c:

   ```cpp
   RootMaterialTrackIo::Config ioCfg;
   ioCfg.prePostStepInfo   = true;   // writer had prePostStep = True
   ioCfg.surfaceInfo       = false;  // writer had storeSurface = False
   ioCfg.volumeInfo        = false;  // writer had storeVolume = False
   ioCfg.recalculateTotals = false;
   ```

   `connectForRead(chain)` on a `TChain` of the `material_tracks` tree, then
   `GetEntry(i)` followed by `read()` per track.

4. **Map and finalize.** `createState(gctx)`, then `mapMaterial(state, gctx,
   mctx, track)` for every recorded track — it returns the mapped and unmapped
   halves, and the unmapped one is the diagnostic worth writing out — then
   `finalizeMaps(state, gctx)`, which returns an
   `Acts::TrackingGeometryMaterial`.

5. **Write the map** with `Acts::MaterialMapJsonConverter::materialMapsToJson`.
   Use `processSensitives = false`, `processBoundaries = true`,
   `processVolumes = false`: in a Gen3 geometry the receivers are portals
   (`boundary`), the sensors carry no material, and there is no volume material.
   `ActsPlugins::RootMaterialMapIo` writes the same content as ROOT if you prefer
   that format.

Run it in the same job as `ActsGeoSvc` so the geometry it maps onto is bit-for-bit
the one reconstruction will use.

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

**5c. Physics validation.** `Acts::MaterialValidator` and
`Acts::PropagatorMaterialAssigner` are in the stack: propagate geantinos through
the *mapped* geometry and compare the accumulated X₀/L₀ against the original
scan, binned in η and φ. Agreement to a few percent is the target; a systematic
deficit means material that fell outside every receiver, which for MAIA most
likely means the region between the outer tracker and the solenoid, or the
nozzles.

**5d. No tracking regression.**

```bash
ctest -R reco_MAIA_Gen3 --output-on-failure
```

Note that adding real material *should* change the fit results — this test checks
the chain still runs, not that the numbers are unchanged.

**5e. Add the map to the repository.** Once validated, add the file to
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
