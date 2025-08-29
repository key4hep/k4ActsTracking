# MLTracking

This is a (temporarily separate) repository for experimenting and implementing
an ML based track finding algorithm (based on approaches in ExaTrk). The goal of
this is to move this wholesale into
[k4RecTracker](https://github.com/key4hep/k4RecTracker) once it has matured enough.


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
- Extract the embedding model and the edge classifier model into separate
  classes as thin wrappers around `ONNXInferenceModel` such that they can be
  easier tested in isolation
