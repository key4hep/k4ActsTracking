# MLTracking

This is a (temporarily separate) repository for experimenting and implementing
an ML based track finding algorithm (based on approaches in ExaTrk). The goal of
this is to move this wholesale into
[k4RecTracker](https://github.com/key4hep/k4RecTracker) or
[k4ActsTracking](https://github.com/key4hep/k4ActsTracking) once it has matured
enough.

## Dependencies
In order to build this package you need a couple of dependencies that are not
yet found in the Key4hep stack. Specifically, you need
- Acts built with the `PluginGnn`.
  - This in turn requires the c++ library of [pytorch_scatter](https://github.com/rusty1s/pytorch_scatter) to be built.
  - Acts needs [acts#4631](https://github.com/acts-project/acts/pull/4631) to be able to build without CUDA support
- k4ActsTracking built with [k4ActsTracking#27](https://github.com/key4hep/k4ActsTracking/pull/27)

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


## Possible future improvements
Many parts of this are currently in a prototype stage to get some results. This
also means that there is plenty of opportunity to improve on the current
implementation. I keep this list here as a reminder for later
- Generalize `mlutils::{flatten,getDimensions,totalSize}` to also handle
  `std::vector<std::array>` which would probably offer better performance due to
  the better memory layout.
  - Switch to use that in the `ExaTrkGNNTrackFinder`
- Make the `ONNXInferenceModel::runInference` thread-safe such that it can be
  marked as `const` to avoid the `mutable` statements in `operator()` of
  Functional algorithms
- The `OnnxMetricLearning` class should almost certainly be upstreamed to the
  Acts GNN plugin.
