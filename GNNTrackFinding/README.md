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
# GNN based track finding

This subpackage provides `GNNTrackFinder`, a Gaudi algorithm that runs an
ML based track finding pipeline (following the approaches of
[ExaTrkX](https://exatrkx.github.io/) / GNN4ITk) on EDM4hep tracker hits. The
machine learning stages are run via the ACTS
[`PluginGnn`](https://acts.readthedocs.io/) with
[ONNX Runtime](https://onnxruntime.ai/) as inference backend.

## The pipeline

`GNNTrackFinder` consumes one or more `edm4hep::TrackerHitPlaneCollection`s and
produces an `edm4hep::TrackCollection` of fitted track candidates. Per event it

1. **collects** all input hits into a single (subset) collection,
2. **segments** them into `ThetaBins` x `PhiBins` (theta, phi) bins that are run
   through the pipeline independently (see [Segmentation](#segmentation)),
3. **extracts the configured features** per hit (positions, time and CellID
   fields) into the flat `(nHits x nFeatures)` input tensor,
4. runs the **graph construction**: a metric learning ONNX model embeds every
   hit into a space in which a KD-tree (CPU) / FRNN (CUDA) radius + KNN search
   builds the candidate edges (`OnnxMetricLearning`),
5. runs one or more **edge classifiers** (ACTS `OnnxEdgeClassifier`) that score
   the edges and drop everything below the configured cut,
6. **builds track candidates** from the remaining edges via the ACTS
   `BoostTrackBuilding` (weakly connected components),
7. **fits** every candidate with at least `MinHitsPerTrack` hits: the innermost,
   middle and outermost hit (ordered by transverse radius) give the initial
   parameters, which are then handed to the ACTS Kalman fitter
   (`ACTSTracking::KFRunner`) together with all hits of the candidate,
8. **extrapolates** every fitted track to the calorimeter face to add its
   `AtCalorimeter` track state (`ExtrapolateToCalo`). This is the same
   extrapolation the CKF algorithms run, so a track carries the same calo state
   however it was reconstructed.

Candidates whose parameter estimation or fit fails are dropped, so the output
contains only successfully fitted tracks.

Because of the Kalman fit, the algorithm needs the ACTS geometry, i.e. an
`ActsGeoSvc` (and with it a `GeoSvc`) has to be configured in the options file.

## Building

This subpackage is **not** built by default. Because it pulls in Torch,
onnxruntime and the Acts Gnn plugin, it is guarded behind a CMake option and
has to be enabled explicitly when configuring `k4ActsTracking`:
```
-DK4ACTSTRACKING_BUILD_GNN=ON
```
When the option is `OFF` (the default) none of the dependencies below are
required and the rest of `k4ActsTracking` builds without them.

## Dependencies
In order to build this package you need a couple of dependencies that are not
yet found in the Key4hep stack. Specifically, you need
- Acts built with the `PluginGnn`.
  - This in turn requires the c++ library of [pytorch_scatter](https://github.com/rusty1s/pytorch_scatter) to be built.
  - Acts needs [acts#4631](https://github.com/acts-project/acts/pull/4631) to be able to build without CUDA support

In particular the GNN plugin and the c++ library of pytorch_scatter are not yet
available via spack, so they need some manual intervention.

The specific flags for building ACTS with the GNN plugin **but without CUDA
support are**
```
-DACTS_BUILD_PLUGIN_GNN=ON \
-DACTS_GNN_ENABLE_CUDA=OFF \
-DACTS_GNN_ENABLE_ONNX=ON \
-DACTS_GNN_ENABLE_TORCH=ON
```

## Running

[`options/runGNNTrackFinding.py`](options/runGNNTrackFinding.py) is a complete
example configuration:

```sh
k4run GNNTrackFinding/options/runGNNTrackFinding.py \
      --compactFile <detector>.xml \
      --modelBase <directory with the .onnx models> \
      --inputFile <digitized edm4hep file> \
      --device cpu
```

> **Note:** the models are not shipped with this package and the values in the
> example options file are *not* a tuned tracking configuration. The input
> features and their scales have to match the models that are used. The
> embedding dimension is not configured: it is read from the `.onnx` file of the
> node embedding model.

## Configuration

### Models and inference

| Property | Default | Description |
| --- | --- | --- |
| `NodeEmbeddingModelPath` | `""` | Path to the ONNX model of the metric learning / graph construction stage |
| `EdgeBuildingRadius` | `0.1` | Radius parameter of the edge building in embedding space |
| `EdgeBuildingKnn` | `100` | KNN parameter of the edge building in embedding space |
| `EdgeClassifierModelPath` | `[]` | Paths to the ONNX models of the edge classifiers |
| `EdgeClassifierCut` | `[0.5]` | Score cut of each edge classifier |
| `Device` | `"cpu"` | Device the pipeline runs on: `cpu`, `cuda` or `cuda:<index>`. `cuda` needs a CUDA enabled onnxruntime / torch build |

`EdgeClassifierModelPath`, `EdgeClassifierCut`, `InputFeaturesEdgeClassifier`
and `InputScalesEdgeClassifier` are parallel lists with **one entry per edge
classifier model**, so that several classifiers can be chained. Inconsistent
list lengths are rejected in `initialize`.

### Input features

| Property | Default | Description |
| --- | --- | --- |
| `InputFeaturesEmbedding` | `"r,phi,z,t"` | Comma separated features for the embedding model |
| `InputScalesEmbedding` | `"1,1,1,1"` | Comma separated scales, each feature is divided by its scale |
| `EmbeddingFixedInputLength` | `0` | If `> 0`, pad the embedding model input with all-zero rows up to this many nodes. `0` disables the padding |
| `KeepEmbeddingPadding` | `False` | Keep those padding rows in the node features handed to the edge classifiers, see below |
| `EdgeClassifierFixedInputLength` | `0` | If `> 0`, pad the edge index and edge features up to this many edges, see below |
| `InputFeaturesEdgeClassifier` | `["r,phi,z,t"]` | Per classifier list of comma separated features |
| `InputScalesEdgeClassifier` | `["1,1,1,1"]` | Per classifier list of comma separated scales |
| `ComputeEdgeFeatures` | `False` | Compute the six edge features a three-input classifier needs, see below |
| `EdgeFeatureScales` | `""` | Scales of `r`, `phi`, `z`, `eta` used for that computation |

The supported (case insensitive) feature names are

| Name | Meaning |
| --- | --- |
| `x`, `y`, `z` | Global hit position |
| `r` | Transverse radius of the hit position |
| `phi` | Azimuthal angle of the hit position |
| `theta` | Polar angle of the hit position |
| `eta` | Pseudorapidity of the hit position |
| `t` (or `time`) | Hit time |
| `E` (or `energy`) | Energy deposited by the hit (`eDep`) |
| `module_id` | `module` field of the CellID |
| `layer_id` | `layer` field of the CellID |
| `system_id` (or `volume_id`) | `system` field of the CellID |

Each stage selects the features it needs from the deduplicated union of all
configured features, so the different stages can use different (sub)sets. The
scale lists have to have either the same number of entries as the corresponding
feature list, or none at all (in which case no scaling is applied). All of this
is validated in `initialize`, as are the CellID based features, which have to be
part of the CellID encoding of the geometry.

#### Zero padding the embedding model input to a fixed length

The number of hits handed to the node embedding model varies from segment to
segment, so the model normally has to accept a variable number of input nodes. A
model that was instead exported with a **fixed-size input** can be used by
padding each segment up to that length with all-zero rows:

```python
# a model whose input is fixed at 4096 nodes
EmbeddingFixedInputLength=4096,
```

The padding rows are appended *after* the hits of the segment, so every real hit
keeps the row index the rest of the pipeline identifies it by. Their embedding is
discarded again as soon as the inference returns, before the edge building: an
all-zero row is not a hit, and the network maps every one of them onto the same
arbitrary point in embedding space, which would otherwise get connected to its
neighbours and to real hits. That part is unconditional — the edge building
always runs on the real hits alone.

By default nothing downstream of the embedding stage sees the padding either.
`KeepEmbeddingPadding` changes that for the node features only:

```python
EmbeddingFixedInputLength=100,
KeepEmbeddingPadding=True,
```

The edge classifiers then get a node tensor of `EmbeddingFixedInputLength` rows,
of which the trailing ones are all-zero nodes with no edges attached. This is for
classifier models that were themselves exported at a fixed number of nodes. It
does not affect the edge index, the edge features or the track building, which
sizes its graph from the space point IDs and so always stays on the real hits.

#### Padding the edge index to a fixed length

A classifier exported at a fixed node count is usually also exported at a fixed
*edge* count. `EdgeClassifierFixedInputLength` pads the edge index and, when they
are computed, the edge features up to that many edges:

```python
EmbeddingFixedInputLength=100,
KeepEmbeddingPadding=True,
EdgeClassifierFixedInputLength=2000,
```

The padding edges are **self loops on the last padding node**. That choice does
the work here:

- they touch no real hit, so they add nothing to any real node's message passing
  (anchoring them on a real hit instead would feed it as many spurious messages
  as there are padding edges);
- the edge building never produces a self loop, so they can be told apart from
  real edges afterwards without bookkeeping;
- a `PaddedEdgeRemoval` stage, appended to the classifier chain automatically,
  drops every self loop once the classifiers are done.

That last stage is not optional. A padding edge that passed the score cut would
otherwise reach the track building pointing at a row index beyond the space point
IDs, and Boost grows its graph to fit that index while the label vector stays at
the number of space points — which overruns it.

This needs `KeepEmbeddingPadding` (there has to be a padding node to anchor the
self loops on) and only works with a **single** edge classifier: the padding
happens once, in the graph construction, and each classifier cuts on the score,
so from the second one on the edge count is whatever survived rather than the
fixed length. Both are rejected in `initialize`, as is a segment with more real
edges than the configured length.

#### A warning about fixed-size exports

Check that a model exported this way was exported with `model.eval()`. If its
BatchNorm statistics are computed at runtime instead of frozen, the padding rows
enter the normalisation and change the scores of the real edges — the padding is
then not inert no matter how carefully it is chosen, and the same edge gets a
different score depending on what else is in its segment.

This is off by default (`0`) and only affects the node embedding stage. A segment
with **more** hits than `EmbeddingFixedInputLength` is an error: either raise the
length, or raise `ThetaBins` / `PhiBins` so that fewer hits land in one segment.
When the model itself declares a fixed input length, a mismatch between it and
the padded input is reported with a message naming this property rather than as a
bare onnxruntime shape error.

#### Edge features

An edge classifier that was exported with **three inputs** takes a per-edge
feature tensor (`edge_attr`) alongside the node features and the edge index. The
six features it expects are the ones Acts computes in `makeEdgeFeatures()` —
`dr`, `dphi`, `dz`, `deta`, `phislope` and `rphislope` — but Acts only fills them
in its CUDA module map stage, so this package computes them itself as part of the
graph construction.

Those six are always computed from `r`, `phi`, `z` and `eta`, so there is nothing
to select — only whether to compute them at all, and with which scales. The
scales go **in that order**, whatever order the models take their own inputs in.

```python
ComputeEdgeFeatures=True,
EdgeFeatureScales="1000,3.14,1000,1",
```

`phislope` is `dphi / dr` clamped to `[-100, 100]` and `rphislope` is that times
the mean radius of the two hits; edges between hits at the same radius get a flat
zero for both. The `dphi` wrap-around assumes that `phi` is scaled by pi, as in
the example above.

The edge classifier scales its *node* input (`InputScalesEdgeClassifier`) but
passes the edge input through as it is, so these have to be the scaling the
classifier was trained with — it is what the edge features are computed from.

Leaving `ComputeEdgeFeatures` off (the default) computes no edge features, which
is what a two-input classifier expects. Configuring a three-input model without
it fails with *"ONNX edge classifier model has three inputs, but no edge features
provided!"* from Acts. Setting `EdgeFeatureScales` while the flag is off is
rejected in `initialize` rather than silently ignored.

### Segmentation

| Property | Default | Description |
| --- | --- | --- |
| `ThetaBins` | `1` | Number of theta bins the hits are split into |
| `PhiBins` | `1` | Number of phi bins the hits are split into |
| `ThetaOverlap` | `0.0` | Overlap of adjacent theta bins as a fraction of the bin width |
| `PhiOverlap` | `0.0` | Overlap of adjacent phi bins as a fraction of the bin width |

The hits are split into `ThetaBins x PhiBins` segments of equal width in
theta (over `[0, pi]`) and phi (over `[-pi, pi]`), and each non-empty segment is
run through the pipeline separately. This keeps the graphs (and with them the
inference cost, which grows faster than linearly with the number of hits) small.
A non-zero overlap extends every bin into its successor, so that hits close to a
bin boundary end up in both bins and tracks crossing the boundary can still be
found.

> **Note:** the phi bins do *not* wrap around, i.e. the last phi bin does not
> overlap with the first one. Track candidates crossing `phi = +/- pi` can
> therefore be split between two segments.

### Track candidates and fit

| Property | Default | Description |
| --- | --- | --- |
| `TrackBuilding` | `"connected-components"` | Track building algorithm, see below |
| `WalkAddScore` | `0.6` | `"cc-and-walk"` only: score above which a neighbour is always followed |
| `WalkMinScore` | `0.1` | `"cc-and-walk"` only: score below which the walk stops |
| `MinHitsPerTrack` | `3` | Minimum number of hits for a candidate to be fitted |
| `PropagateBackward` | `false` | Extrapolate the fitted tracks towards the beamline |
| `ExtrapolateToCalo` | `true` | Extrapolate the fitted tracks to the calorimeter face and add an `AtCalorimeter` track state |
| `AddEndcapCaloState` | `false` | Give a track that crosses both calorimeter sections one `AtCalorimeter` state per section (barrel first, then endcap) instead of a single one |
| `InitialTrackError_Pos` | `10 um` | Initial uncertainty of the local position |
| `InitialTrackError_Phi` | `1 degree` | Initial uncertainty of phi |
| `InitialTrackError_Lambda` | `1 degree` | Initial uncertainty of lambda |
| `InitialTrackError_RelP` | `0.25` | Initial relative momentum uncertainty |
| `InitialTrackError_Time` | `100 ns` | Initial uncertainty of the time |

#### Track building algorithms

`"connected-components"` (the default) is Acts' `BoostTrackBuilding`: it emits
every weakly connected component of the classified graph as one candidate. It is
simple and fast, but it never looks at the edge scores, and it makes no attempt
to separate tracks — two that share a single surviving edge come out as one
candidate, and every hit that ended up with no edge becomes a candidate of its
own.

`"cc-and-walk"` is the algorithm the ExaTrkX / GNN4ITk pipeline uses:

1. the connected components are found, as above;
2. a component in which every hit has at most one incoming and one outgoing edge
   is already a path and is accepted as it is, whatever its scores — the score
   cut has already happened, in the edge classifier;
3. any other component is *walked*: starting from its innermost unused hit the
   graph is followed outwards, the longest path that can be reached is taken as
   a candidate, its hits are retired, and the search restarts from the next
   unused hit. Candidates shorter than `MinHitsPerTrack` are dropped rather than
   emitted, which is also what removes the isolated hits.

The walk is steered by two thresholds. Every neighbour scoring above
`WalkAddScore` is followed — the walk branches if several do, and the longest
branch wins. If none reaches it, only the single best neighbour is followed, and
only if it scores above `WalkMinScore`.

To make "incoming", "outgoing" and "outwards" mean something, the graph is
directed by ordering the two hits of each edge by radius, with the node index
breaking ties. That is a strict total order, so the directed graph is acyclic by
construction and the walk cannot loop. `r` is added to the extracted hit features
automatically when this algorithm is selected.

Ties between two equally long paths go to the lower node index, so the same event
always gives the same tracks.

On the 10-event sample used during development, switching from
`"connected-components"` to `"cc-and-walk"` took the candidate count from 114 to
4 while the number of fitted tracks stayed the same (5 vs 4) — i.e. it removes
almost only candidates that the fit was going to reject anyway.

### Monitoring and debugging

`MonitoringHistogram` is a 3d histogram filled once per track candidate with
(number of input hits, number of track candidates, candidate length). It is
written out by a `Gaudi__Histograming__Sink__Root` sink, see the example options
file for how to configure the axes and the output file.

`DetailedDebugOut` (together with `OutputLevel=DEBUG`) prints all pipeline
inputs and outputs in full detail. This is *very* verbose and only useful for
debugging on small inputs.

## Possible future improvements
Many parts of this are currently in a prototype stage to get some results. This
also means that there is plenty of opportunity to improve on the current
implementation. I keep this list here as a reminder for later
- Generalize `mlutils::{flatten,getDimensions,totalSize}` to also handle
  `std::vector<std::array>` which would probably offer better performance due to
  the better memory layout.
- Make the `ONNXInferenceModel::runInference` thread-safe such that it can be
  marked as `const` to avoid the `mutable` statements in `operator()` of
  Functional algorithms
- The `OnnxMetricLearning` class should almost certainly be upstreamed to the
  Acts GNN plugin.
- Run the (independent) theta/phi segments concurrently instead of sequentially.
- Make the phi segmentation wrap around, so that candidates crossing
  `phi = +/- pi` are not split.
