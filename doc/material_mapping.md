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

## 2. Recording (external)

Unchanged from the wiki procedure, and the only step that still needs tooling
outside this repository: the key4hep/MuColl stacks build ACTS without
`ActsExamples` and without the `acts` Python bindings. Use a separate ACTS build
(as the ACTSMCC workspace did) to produce `geant4_material_tracks.root` from the
MAIA DD4hep geometry:

```bash
ActsExampleMaterialRecordingDD4hep --response-file <your-geantinoscan.response>
```

The output format is what `ActsPlugins::RootMaterialTrackIo` reads, so it feeds
straight into step 3.

## 3. Mapping

**Not yet implemented in this repository.** Everything needed is present in the
stack, so this is intended to become a Gaudi algorithm here rather than an
external script:

- `Acts::MaterialMapper`, `Acts::IntersectionMaterialAssigner`,
  `Acts::BinnedSurfaceMaterialAccumulator` — all in ACTS **Core**;
- `ActsPlugins::RootMaterialTrackIo` — reads `geant4_material_tracks.root`;
- `Acts::MaterialMapJsonConverter` / `ActsPlugins::RootMaterialMapIo` — writes
  the map.

The surfaces to hand the mapper are those designated in step 1, collected from
the constructed `Acts::TrackingGeometry`.

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

- `50 ... 50 of them a proto-material placeholder` — no map applied, or none of
  its entries matched. A proto placeholder contributes no actual material, so
  tracking still sees none; the service warns about this.
- `54 ... 41 of them a proto-material placeholder` — the map was applied to the
  surfaces it had entries for.

### Caveat: maps are tied to the blueprint that produced them

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
