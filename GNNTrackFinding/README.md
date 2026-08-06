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
   (`ACTSTracking::KFRunner`) together with all hits of the candidate.

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
> features, their scales and the embedding dimension have to match the models
> that are used.

## Configuration

### Models and inference

| Property | Default | Description |
| --- | --- | --- |
| `NodeEmbeddingModelPath` | `""` | Path to the ONNX model of the metric learning / graph construction stage |
| `EmbeddingDim` | `4` | Output dimension of the embedding model. Has to match `InputFeaturesEmbedding` |
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
| `InputFeaturesEdgeClassifier` | `["r,phi,z,t"]` | Per classifier list of comma separated features |
| `InputScalesEdgeClassifier` | `["1,1,1,1"]` | Per classifier list of comma separated scales |

The supported (case insensitive) feature names are

| Name | Meaning |
| --- | --- |
| `x`, `y`, `z` | Global hit position |
| `r` | Transverse radius of the hit position |
| `phi` | Azimuthal angle of the hit position |
| `t` (or `time`) | Hit time |
| `module_id` | `module` field of the CellID |
| `layer_id` | `layer` field of the CellID |
| `system_id` (or `volume_id`) | `system` field of the CellID |

Each stage selects the features it needs from the deduplicated union of all
configured features, so the different stages can use different (sub)sets. The
scale lists have to have either the same number of entries as the corresponding
feature list, or none at all (in which case no scaling is applied). All of this
is validated in `initialize`, as are the CellID based features, which have to be
part of the CellID encoding of the geometry.

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
| `MinHitsPerTrack` | `3` | Minimum number of hits for a candidate to be fitted |
| `PropagateBackward` | `false` | Extrapolate the fitted tracks towards the beamline |
| `InitialTrackError_Pos` | `10 um` | Initial uncertainty of the local position |
| `InitialTrackError_Phi` | `1 degree` | Initial uncertainty of phi |
| `InitialTrackError_Lambda` | `1 degree` | Initial uncertainty of lambda |
| `InitialTrackError_RelP` | `0.25` | Initial relative momentum uncertainty |
| `InitialTrackError_Time` | `100 ns` | Initial uncertainty of the time |

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
