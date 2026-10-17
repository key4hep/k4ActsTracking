# v00-06

* 2026-10-16 Jackson Carl Burzynski ([PR#125](https://github.com/Key4hep/k4ActsTracking/pull/125))
  - Add `InitialTrackError_QOverP` to `CKFTrackingAlg` and `GNNTrackFinder`: an absolute initial q/p error added in quadrature to the relative one from `InitialTrackError_RelP`. The default of 0 keeps the old behaviour

* 2026-10-15 Jackson Carl Burzynski ([PR#124](https://github.com/Key4hep/k4ActsTracking/pull/124))
  Fix the hit covariance in `prepareTrackerHits`: rotate `du`/`dv` from the hit's u/v axes into the surface's local frame instead of assigning them directly to loc0/loc1

* 2026-10-14 Jackson Carl Burzynski ([PR#131](https://github.com/Key4hep/k4ActsTracking/pull/131))
  - Make sure that the `SET` sensor axes for the ILD blueprints are properly oriented to make them have a proper orthonormal base in the ACTS geometry

* 2026-10-08 Juan Miguel Carceller ([PR#126](https://github.com/Key4hep/k4ActsTracking/pull/126))
  - Run ACTS master CI directly in the Key4hep job container, mount CVMFS explicitly, and pin its actions to the latest release commits.

* 2026-10-07 Mark Larson ([PR#123](https://github.com/Key4hep/k4ActsTracking/pull/123))
  - Adds an example of how to log how many Threading Building Blocks (TBB) threads are running within the parallel seeding + CKF, allowing seeing whether NumThreads > 1 (and thus that multi threading is working as expected)

* 2026-10-07 Juan Miguel Carceller ([PR#122](https://github.com/Key4hep/k4ActsTracking/pull/122))
  - Remove `ACTSSeededCKFTrackingAlg` and its unused geometry helpers. Use `CKFTrackingAlg` instead.

* 2026-10-07 Juan Miguel Carceller ([PR#120](https://github.com/Key4hep/k4ActsTracking/pull/120))
  - Reduce CKFTrackingAlg.cpp compile memory by removing unused ACTS headers.

* 2026-10-07 Federico Meloni ([PR#117](https://github.com/Key4hep/k4ActsTracking/pull/117))
  - GNNTrackFinding: print the graph after every pipeline stage through a `GnnHook` (`EdgePrintingHook`) passed to `GnnPipeline::run`, instead of the `ClassifiedEdgePrinting`. The pipeline now runs the same stages whatever the output level.
  - The output goes to the `GNNTrackFinder.Edges` logger and now also covers the graph as built.

* 2026-10-07 Federico Meloni ([PR#113](https://github.com/Key4hep/k4ActsTracking/pull/113))
  - GNNTrackFinding: remove the unused `mlutils::{flatten,getDimensions,totalSize}` helpers and templated `runInference` overloads from `ONNXInferenceModel.h`.
  - GNNTrackFinding: use native Gaudi list properties instead of comma-separated strings. Old string configurations are rejected at configuration time. `parseList` / `parseMultiList` are removed.
  - Add a `HostTensorView` helper that replaces three copies of the host-copy code in the pipeline stages.
  - Remove the edge deduplication from the edge ordering and from `CCAndWalkTrackBuilding`.

* 2026-10-07 Jackson Carl Burzynski ([PR#72](https://github.com/Key4hep/k4ActsTracking/pull/72))
  - Add optional time-aware (4D) tracking to `CKFTrackingAlg`. Hit time as a CKF measurement dimension with per-sensor time resolutions, and optional seed doublet and triplet time cuts using the ACTS seeding time support. Requires digitised times without the time-of-flight correction
  - Convert all hit times to ACTS native units before passing them to ACTS, fixing seed t0 that was previously in raw ns

* 2026-10-06 Federico Meloni ([PR#112](https://github.com/Key4hep/k4ActsTracking/pull/112))
  - `EdgeBuildingKnn` is now an `int` and must be > 0. Its documentation now says that only the CUDA (FRNN) edge building caps the neighbours at this value; the CPU KD-tree only uses it to reserve memory.
  - ONNX model loading errors now carry the model path and the onnxruntime message and reach the Gaudi log, instead of being printed to stderr.
  - Fix CUDA-targeted embedding outputs in host memory staying on the CPU (latent, not reachable with the current output binding).
  - README: document that overlapping segments can produce duplicate tracks, and remove outdated items from the future-improvements list.

* 2026-10-05 Federico Meloni ([PR#111](https://github.com/Key4hep/k4ActsTracking/pull/111))
  - GNNTrackFinding: require Acts >= 48.1.0 (or main) when `K4ACTSTRACKING_BUILD_GNN` is on, for [acts#5646](https://github.com/acts-project/acts/pull/5646) (node feature selection/scaling in `OnnxEdgeClassifier`) and [acts#6142](https://github.com/acts-project/acts/pull/6142) (features passed in the configured order). Checked after `find_package` rather than passed to it, since Acts declares `SameMajorVersion` compatibility, which would reject later major versions and main (999.999.999).
  - Drop the fallback to the pre-`ActsPlugins` Gnn headers and namespace aliases.
  - Edge classifier input features are now passed to the model in the order given and may be repeated.
  - Reject zero input scales (embedding, edge features, edge classifiers) in `initialize()`, instead of an exception or infinite inputs at the first event.

* 2026-10-02 Federico Meloni ([PR#110](https://github.com/Key4hep/k4ActsTracking/pull/110))
  - Remove wget-based data retrieval in favour of ExternalData

* 2026-10-02 Baucki ([PR#94](https://github.com/Key4hep/k4ActsTracking/pull/94))
  - Small change to ONNXInferenceModel adding a dedicated CUDA path, binding ONNX outputs directly to CUDA device
  - Small fix to OnnxMetricLearning to ensure output tensor is on the correct device for buildEdgesFRNN (ACTS) when running on CUDA
  - Copy node feature input tensor for downstream pipeline (edge classifiers) directly into ACTS tensor on CUDA device memory to force ACTS' OnnxEdgeClassifier to run on CUDA (device is picked up from [input tensor](https://github.com/acts-project/acts/blob/main/Plugins/Gnn/src/OnnxEdgeClassifier.cpp#L115)); skips intermediate conversion to torch tensor to move to CUDA device (std::vector -> torchTensor -> ActsTensor)

* 2026-09-24 Federico Meloni ([PR#95](https://github.com/Key4hep/k4ActsTracking/pull/95))
  - Use the centralised ACTS EDM4hep converter (`ActsPlugins::EDM4hepUtil`) instead of the local conversion code. `ACTS2edm4hep_track` is now a thin wrapper around `writeTrack`, and the seed (`AtFirstHit`) and calorimeter (`AtCalorimeter`) track states are built with `writeTrackState`.
  - Remove `ACTS2edm4hep_trackState`, whose LCIO parameter conversion and covariance packing are now provided by the ACTS plugin.
  - Fix the track state covariance layout: it now follows the EDM4hep parameter order (`edm4hep::TrackParams`), so downstream consumers read the correct elements.
  - Link against `Acts::PluginEDM4hep`.

* 2026-09-23 Juan Miguel Carceller ([PR#106](https://github.com/Key4hep/k4ActsTracking/pull/106))
  - Build the EDM4hep plugin in CI

* 2026-09-23 Federico Meloni ([PR#105](https://github.com/Key4hep/k4ActsTracking/pull/105))
  - Added pass-through stage `ClassifiedEdgePrinting` to ease the validation of the GNN pipeline with external frameworks

* 2026-09-22 Federico Meloni ([PR#103](https://github.com/Key4hep/k4ActsTracking/pull/103))
  - Added SortEdges flag to re-orient edges from the hit closer to the interaction point to the one further out, matching the convention used during training

* 2026-09-19 Paul Gessinger ([PR#104](https://github.com/Key4hep/k4ActsTracking/pull/104))
  - Adapt to https://github.com/acts-project/acts/pull/6123

* 2026-09-18 Juan Miguel Carceller ([PR#102](https://github.com/Key4hep/k4ActsTracking/pull/102))
  - Replace deprecated (https://github.com/acts-project/acts/pull/5738)  material-binning APIs with AxisSpec

* 2026-09-17 Thomas Madlener ([PR#101](https://github.com/Key4hep/k4ActsTracking/pull/101))
  - Enforce formatting of pyhton files via ruff in pre-commit and CI

* 2026-09-17 Thomas Madlener ([PR#99](https://github.com/Key4hep/k4ActsTracking/pull/99))
  - Update the `.clang-format` configuration to the one the rest of Key4hep uses
  - Format all c++ files to comply

* 2026-09-17 Juan Miguel Carceller ([PR#98](https://github.com/Key4hep/k4ActsTracking/pull/98))
  - Fix k4ActsTracking compatibility with the current ACTS propagator and grid-bin finder APIs after https://github.com/acts-project/acts/pull/6004

* 2026-09-17 Joshua Beirer ([PR#97](https://github.com/Key4hep/k4ActsTracking/pull/97))
  - Fixed a cryptic Gaudi `TypeError`/`ValueError` when `--compactFile` was omitted in `CLD_CKFTracking.py`, `runGNNTrackFinding.py`, and `test_visualize_acts_geo.py` by making the argument required, with a clear error message.

* 2026-09-17 Thomas Madlener ([PR#96](https://github.com/Key4hep/k4ActsTracking/pull/96))
  - Make it possible to trigger the pre-commit workflow from the github UI

# v00-05

* 2026-08-12 Federico Meloni ([PR#93](https://github.com/key4hep/k4ActsTracking/pull/93))
  - Added further extrapolation of tracks to calorimeter endcaps  to satisfy downstream k4GaudiPandora logic (gated behind the AddEndcapCaloState algorithm property)
  - Share the calo extrapolation code so that KFRunner (used to fit the GNN track candidates) can run it too

* 2026-08-11 Federico Meloni ([PR#88](https://github.com/key4hep/k4ActsTracking/pull/88))
  - Add material mapping and validation for the Gen3 (blueprint) tracking geometry, so
    that reconstruction sees the passive material the DD4hep-to-ACTS conversion
    discards. The full workflow is documented in
    [doc/material_mapping.md](doc/material_mapping.md).

* 2026-08-07 Federico Meloni ([PR#91](https://github.com/key4hep/k4ActsTracking/pull/91))
  - Added support for InteractionGNN2 , which requires computing edge features in the pipeline.
  - Added support for fixed-size-input edge classifiers
  - Added connected-component and walk track building algorithm

* 2026-08-07 Federico Meloni ([PR#90](https://github.com/key4hep/k4ActsTracking/pull/90))
  - Turn on GNN build in MuColl CI test.
  - Removed embedding dimension check to allow for general models with N_embedding dimesions != N_features. The embedding target dimension is now taken from the serialised model file directly.
  - Moved hit features to separate files and into k4actstracking sub-package
  - Added option to 0-pad up to a fixed length the model inputs.
  - Added a hit sorting algorithm to pre-process inputs for the models that need it.

* 2026-08-06 Federico Meloni ([PR#59](https://github.com/key4hep/k4ActsTracking/pull/59))
  - Add a GNN-based track finding pipeline (`GNNTrackFinder`). It runs metric-learning graph construction and edge classification via ONNX Runtime and the ACTS `PluginGnn` to build track candidates from EDM4hep tracker hits.
  - Guard the GNN code behind the new `K4ACTSTRACKING_BUILD_GNN` CMake option (default `OFF`). When enabled it pulls in the required `Torch`, `onnxruntime` and ACTS `PluginGnn` dependencies, so builds without ML dependencies are unaffected.
  - Support configurable per-stage input features and scales (`x`, `y`, `z`, `r`,`phi`, `t`, and module/layer/system IDs) and a list of edge classifiers, each with its own feature selection.
  - Add `ONNXInferenceModel` and `OnnxMetricLearning` helpers, plus the `mlutils::parseList`/`parseMultiList` utilities for parsing comma-separated configuration.
  - Add an example Gaudi options file (`GNNTrackFinding/options/runGNNTrackFinding.py`) and unit tests for the ONNX inference model.

* 2026-07-28 Juan Miguel Carceller ([PR#86](https://github.com/key4hep/k4ActsTracking/pull/86))
  - Fix the build with recent ACTS releases by using `Acts::Blueprint` instead of `Acts::Experimental::Blueprint`.

* 2026-07-17 Juan Miguel Carceller ([PR#84](https://github.com/key4hep/k4ActsTracking/pull/84))
  - Include `fmt/format.h` instead of `fmt/core.h`. It seems in fmt 12.2.0 `fmt::format` is not available in `core.h` anymore.

* 2026-07-16 Juan Miguel Carceller ([PR#83](https://github.com/key4hep/k4ActsTracking/pull/83))
  - Use Key4hep@LCG stacks in the acts-master-ci.yml workflow

* 2026-07-16 Federico Meloni ([PR#79](https://github.com/key4hep/k4ActsTracking/pull/79))
  - Fix track state lookup in `FilterTracksAlg` and `ACTSDuplicateRemoval`: these  used `Track::getTrackStates(edm4hep::TrackState::AtIP)`, but the getter is positional (not keyed by location), so it returned the first-hit state instead of the AtIP one. The `FilterTracksAlg` pT cut was therefore computed from the wrong track state.
      - Add an `ACTSTracking::trackStateAt(track, location)` helper that resolves a
        track state by its `location` field, returning `std::optional`.
      - `FilterTracksAlg` now skips (with a warning) tracks that have no AtIP state.
  - Remove the triplicated "create seed track under lock" block in the seeding algorithms by extracting an `ACTSTracking::appendSeedTrack` helper (shared by the cylindrical and telescope seeders in `CKFTrackingAlg` and by `CKFTrackingFromSeedsAlg`).
  - Make the maximum number of propagation steps configurable and consistent: add a `MaxPropagationSteps` property to `CKFTrackingAlg` and `CKFTrackingFromSeedsAlg`, thread it through the calorimeter-face extrapolation (previously hard-coded), and drive all defaults from a single shared `ACTSTracking::kDefaultMaxPropagationSteps` constant (also used by `KFRunner`).
  - Add unit tests for `trackStateAt`, including a regression guard against the  positional-getter bug. Move `Helpers` into the `k4ActsTracking` library so the shared utilities are linkable from the unit tests.
  - Add a runtime deprecation warning to `ACTSSeededCKFTrackingAlg`: it is a legacy algorithm and will be removed once material effects are added to `CKFTrackingAlg`.

* 2026-07-14 Juan Miguel Carceller ([PR#81](https://github.com/key4hep/k4ActsTracking/pull/81))
  - Follow rename from Seeding2 to Seeding in ACTS (https://github.com/acts-project/acts/pull/5643)

* 2026-07-14 Juan Miguel Carceller ([PR#80](https://github.com/key4hep/k4ActsTracking/pull/80))
  - Add a workflow to build k4ActsTracking on top of ACTS master

* 2026-07-11 Federico Meloni ([PR#78](https://github.com/key4hep/k4ActsTracking/pull/78))
  - Fix trackstates at measurement points creating perigees and adding reference points

* 2026-07-11 Federico Meloni ([PR#75](https://github.com/key4hep/k4ActsTracking/pull/75))
  - Add support for straight-line ("telescope") tracking of field-free planar geometries, and end-to-end test + CI coverage for the LUXE_v0 geometry.
  - `CKFTrackingAlg`: new `SeedingMode` property (default `"Cylindrical"`). `"Telescope"` selects a straight-line, layer-based seeder that groups the seed-selected space points into layers along z and forms collinear triplets, replacing the collider helix seeder (which produces no seeds in a field-free tracker). Configurable via `Telescope_LayerZTolerance`, `Telescope_CollinearityCut` and `Telescope_NominalMomentum`. The cylindrical/barrel code path is unchanged, so existing clients see no behaviour change.
  - `CKFRunner`: the fitted-track reference surface is now configurable (`Config::referenceSurface`). It defaults to the origin `PerigeeSurface` (unchanged collider d0/z0 parameterisation); `CKFTrackingAlg` supplies a beam-perpendicular `PlaneSurface` at `Telescope_ReferenceZ` in telescope mode, so beam-parallel tracks — which never reach the beamline perigee - get a reachable, well-defined AtIP reference (D0/Z0 read as the track position on that plane, omega = 0).
  - `ActsGeoSvc`: new `UseDD4hepBField` property (default `false`). When set, ACTS uses the real, position-dependent DD4hep field via ACTS' `DD4hepFieldAdapter` for all propagation/extrapolation, instead of a uniform `Acts::ConstantBField` sampled at the origin. This lets the IP back-extrapolation account for localized fields such as the LUXE dipole (the tracker itself is field-free). The default preserves the current constant-field behaviour.
  - Added a self-contained `LUXE_v0` chain (particle-gun sim → `DDPlanarDigi` → CKF) with `test/options/LUXE_CKFTracking.py`, a `make_telescope_ckf_tracking` helper in `_ckf_helpers.py`, a geometry-load test, and a dedicated `luxe-ci.yml` workflow running against `ghcr.io/luxesoftware/luxe-sw`. The LUXE tests are only registered when `luxegeo` is available.
  - Extend the track extrapolation to the ECAL face to telescope geometries (LUXE). The LUXE ECALp is modelled as one planar surface (derived from the DD4hep box shape and world placement) enclosed in a cuboid calo volume stacked behind the tracker layers along the beam axis. Requires the ECALp to be flagged as an electromagnetic calorimeter (`CALORIMETER | ELECTROMAGNETIC`, neither `BARREL` nor `ENDCAP`) in the luxegeo geometry.

* 2026-07-10 Jackson Burzynski ([PR#76](https://github.com/key4hep/k4ActsTracking/pull/76))
  - Add CKF branch stopper, outlier chi2 cut, and FilterTracksAlg fixes

* 2026-07-10 Federico Meloni ([PR#73](https://github.com/key4hep/k4ActsTracking/pull/73))
  - SourceLink shrunk to two 8-byte members (geometry identifier + index) so it fits inside the ACTS SourceLink small-buffer (ACTS_SOURCELINK_SBO_SIZE, default 16 bytes) and is stored in place rather than heap-allocated on every wrap.

* 2026-07-08 Paul Gessinger ([PR#67](https://github.com/key4hep/k4ActsTracking/pull/67))
  - Add cmake helper to build ACTS integrated with the k4ActsTracking build

* 2026-07-02 Federico Meloni ([PR#69](https://github.com/key4hep/k4ActsTracking/pull/69))
  - Introduced standalone Kalman Fitter
  - Added an algorithm to allow running CKF on a externally-provided collection of seeds
  - Refactored CKFTrackingAlg to use common CKFRunner 
  - Add track state at IP for final output tracks

* 2026-07-02 Federico Meloni ([PR#64](https://github.com/key4hep/k4ActsTracking/pull/64))
  - Implemented track extrapolation to calorimeter surface from dd4hep geometry
  - Saved track state AtCaloSurface
  - Save track state at IP (porting over from @samf25's https://github.com/tmadlener/k4ActsTracking/pull/1)

* 2026-06-23 Federico Meloni ([PR#71](https://github.com/key4hep/k4ActsTracking/pull/71))
  - Fixed gcc15 warnings (treated as failures) in ubuntu26 CI build tests

* 2026-06-21 Federico Meloni ([PR#70](https://github.com/key4hep/k4ActsTracking/pull/70))
  - Migrated ACTSSeededCKFTrackingAlg to spacepointcontainer2

* 2026-06-20 Federico Meloni ([PR#63](https://github.com/key4hep/k4ActsTracking/pull/63))
  - Migrated CKFTrackingAlg seeding to SpacePointContainer2/Seeding2 API
  - Exposed configuration properties for seeding and seed filtering/weighting

* 2026-06-12 Federico Meloni ([PR#68](https://github.com/key4hep/k4ActsTracking/pull/68))
  - Updating the headers not to use deprecated APIs (https://github.com/acts-project/acts/commit/0045fb428dee8bf1246c5152a5e49972d3e25232)

* 2026-06-08 Juan Miguel Carceller ([PR#66](https://github.com/key4hep/k4ActsTracking/pull/66))
  - Link to k4FWCore::k4Interface, used in a few files in the plugins

# Unreleased

* 2026-08-05 Thomas Madlener, Lukas Bauckhage, Federico Meloni ([PR#59](https://github.com/key4hep/k4ActsTracking/pull/59))
  - Add a GNN based track finding pipeline (`GNNTrackFinder`). It runs metric-learning graph construction and edge classification via ONNX Runtime and the ACTS `PluginGnn` to build track candidates from EDM4hep tracker hits, and fits the resulting candidates with the ACTS Kalman fitter to produce an `edm4hep::TrackCollection`.
  - Guard the GNN code behind the new `K4ACTSTRACKING_BUILD_GNN` CMake option (default `OFF`). When enabled it pulls in the required `Torch`, `onnxruntime` and ACTS `PluginGnn` dependencies, so builds without ML dependencies are unaffected.
  - Support configurable per-stage input features and scales (`x`, `y`, `z`, `r`, `phi`, `theta`, `eta`, `t`, `E`, and module/layer/system IDs) and a list of edge classifiers, each with its own feature selection, scales and cut value.
  - Add an optional segmentation of the input hits into overlapping theta/phi bins (`ThetaBins`, `PhiBins`, `ThetaOverlap`, `PhiOverlap`) that are run through the pipeline independently.
  - Allow to select the device the pipeline runs on via the `Device` property (`cpu`, `cuda` or `cuda:<index>`), and validate the (parallel) model configuration lists, the input scales and the CellID based input features in `initialize`.
  - Add a monitoring histogram (number of input hits, number of track candidates, candidate length) and a `DetailedDebugOut` property for dumping all pipeline inputs and outputs.
  - Add `ONNXInferenceModel` and `OnnxMetricLearning` helpers, plus the `mlutils::parseList`/`parseMultiList` utilities for parsing comma-separated configuration.
  - Add an example Gaudi options file (`GNNTrackFinding/options/runGNNTrackFinding.py`) and unit tests for the ONNX inference model.
  - Move the shared runner helpers (`prepareTrackerHits`, `estimateSeedParameters`, `SeedHit`, `collectSeedHits`, ...) from `CKFRunner.hxx` into a new `RunnerCommon.hxx`, and add an `estimateSeedParameters` overload that takes the radius-ordered seed hits, so that `CKFTrackingFromSeedsAlg` and `GNNTrackFinder` share the seed building.

# v00-04

* 2026-06-03 Thomas Madlener ([PR#49](https://github.com/key4hep/k4ActsTracking/pull/49))
  - Added an initial proof-of-concept CKF tracking workflow for Gen3/DD4hep geometries, including DD4hep-to-ACTS conversion, CellID-based seed selection, and reusable test helpers.
  - Added end-to-end test coverage for MAIA_v0, CLD, and ILD_FCCee_v01 tracking chains.

* 2026-04-28 Thomas Madlener ([PR#57](https://github.com/key4hep/k4ActsTracking/pull/57))
  - Make error messages from ACTS (propagated by us via exceptions) more useful by including the error message instead of just the numerical error code
  - Remove a few unused services an includes from the ACTSSeededCKFTrackingAlg
  - Make helper functions accept a const magnetic field because it can

* 2026-04-28 Thomas Madlener ([PR#58](https://github.com/key4hep/k4ActsTracking/pull/58))
  - Do not fail the mucoll based CI just because of deprecation warnings

* 2026-04-27 Thomas Madlener ([PR#36](https://github.com/key4hep/k4ActsTracking/pull/36))
  - Rework the ActsGeoSvc to do the conversion from DD4hep to ACTS Gen3 geometry in-memory via the ACTS blueprint API. This does not yet update any tracking functionality, it only adds geometry conversion
    - Populate a CellID to Acts::Surface mapping on the fly after geometry conversion.
    - Provide a uniform & constant magnetic field taking the value at (0, 0, 0) from the DD4hep geometry
  - Provide an extensible way of adding more detectors. For now a subset of detectors living in k4geo is handled
    - MuonCollider: MAIA_v0, MuSIC_v2
    - FCCee: ILD_FCCee_v01, ILD_FCCee_v02, CLD_o2_v08, CLD_o2_v07
    - LUXE: LUXE_v0 (lives in https://github.com/LUXESoftware/luxegeo)

# v00-03

* 2026-03-19 Juan Miguel Carceller ([PR#50](https://github.com/key4hep/k4ActsTracking/pull/50))
  - Use ChargeHypothesis instead of AnyCharge that has been removed in https://github.com/acts-project/acts/pull/5076

* 2026-03-12 Thomas Madlener ([PR#48](https://github.com/key4hep/k4ActsTracking/pull/48))
  - Cleanup the `GeometryIdMappingTool`
    - Replace several overloads of `getGeometryID` with one templated member function
    - Mark all `getGeometryID` methods as `const` as they are pure queries and should not alter internal state
    - Cache the field indices for the internal cell id decoder to not have to go through several map lookups for every hit

* 2026-03-11 Thomas Madlener ([PR#45](https://github.com/key4hep/k4ActsTracking/pull/45))
  - Fix several deprecation warnings from Acts

* 2026-03-10 Samuel Ferraro ([PR#46](https://github.com/key4hep/k4ActsTracking/pull/46))
  - Upstream the possibility to run CKF Tracking on multiple threads ([MuonCollidersoft/k4ActsTracking#17](https://github.com/MuonColliderSoft/k4ActsTracking/pull/17))
  - Create a dedicated `findSeeds` function to keep split between seeding and tracking introduced in #42 in place.

* 2026-03-09 Thomas Madlener ([PR#47](https://github.com/key4hep/k4ActsTracking/pull/47))
  - Make the conversion functions from ACTS to EDM4hep return handles instead of pointers to handles.

* 2026-02-25 Juan Miguel Carceller ([PR#43](https://github.com/key4hep/k4ActsTracking/pull/43))
  - Convert to TGeoAxes following changes in ACTS

* 2026-02-23 Baucki (DESY atlaslap122) ([PR#42](https://github.com/key4hep/k4ActsTracking/pull/42))
  - Refactoring of the existing ACTSSeededCKFTrackingAlg algorithm to a separate method `tracking` to allow for the possibility of externally built seeds. Existing seed finding is unchanged, followed by a call to the new `tracking`.
  - Preparation for future algorithms to inherit from the `ACTSSeededCKFTrackingAlg` class and use the same `tracking` method on e.g. externally built track seeds (`edm4hep::TrackCollection`).

* 2026-01-14 Juan Miguel Carceller ([PR#41](https://github.com/key4hep/k4ActsTracking/pull/41))
  - Require EDM4hep in CMakeLists.txt and link to it

* 2026-01-14 Thomas Madlener ([PR#40](https://github.com/key4hep/k4ActsTracking/pull/40))
  - Cleanup the CMake configuration slightly and explicitly state files that should be built
  - Remove configuration targetting Gaudi  v35
  - Remove the unused `EmptyAlg` and its example

# v00-02

* 2025-12-17 Thomas Madlener ([PR#38](https://github.com/key4hep/k4ActsTracking/pull/38))
  - Remove calls to removed config methods upstream. These were no-ops since v42 of Acts so functionality should be unchanged.

* 2025-12-17 samf25 ([PR#25](https://github.com/key4hep/k4ActsTracking/pull/25))
  - First implementation of full set of algorithms needed to do tracking, from porting of https://github.com/MuonColliderSoft/ACTSTracking
  - Based on ACTS 44.1.0
  - Still using the "old" TGeo based geometry

* 2025-11-21 Thomas Madlener ([PR#34](https://github.com/key4hep/k4ActsTracking/pull/34))
  - Switch to Key4hep stack to run pre-commit in CI

# v00-01

* 2025-10-23 Thomas Madlener ([PR#33](https://github.com/key4hep/k4ActsTracking/pull/33))
  - Make the mucoll image based CI workflow run again by cleaning the github runner to get enough space

* 2025-10-23 Thomas Madlener ([PR#32](https://github.com/key4hep/k4ActsTracking/pull/32))
  - Overhaul cmake configuration
    - Define a project version
    - Reduce dependencies to minimal set
    - Properly propagate dependency and version information to downstream consumers
    - Make sure to use common Key4hep cmake settings

* 2025-10-06 Thomas Madlener ([PR#30](https://github.com/key4hep/k4ActsTracking/pull/30))
  - Cleanup `ActsGeoSvc`
    - Remove unused includes
    - Use Gaudi integrated Acts logging introduced in #27 
    - Cleanup logging by using logging facilities directly

* 2025-10-06 Thomas Madlener ([PR#29](https://github.com/key4hep/k4ActsTracking/pull/29))
  - Fix the include paths to Acts plugins since they have changed upstream in: https://github.com/acts-project/acts/pull/4640

* 2025-09-23 Thomas Madlener ([PR#27](https://github.com/key4hep/k4ActsTracking/pull/27))
  - Create the necessary wrappers for passing the Gaudi logging services into ACTS components and algorithms (which usually take a logger as constructor argument).

* 2025-09-22 Thomas Madlener ([PR#28](https://github.com/key4hep/k4ActsTracking/pull/28))
  - Add a CI workflow based on a MuonCollider SW stack image to ease the integration of muon colider developments

* 2025-09-17 Thomas Madlener ([PR#26](https://github.com/key4hep/k4ActsTracking/pull/26))
  - Switch to new ACTS CMake target names if available
  - Fix test using OpenDataDetector by bootstrapping location from environment if not already available

* 2025-08-07 Wouter Deconinck ([PR#24](https://github.com/key4hep/k4ActsTracking/pull/24))
  - Add LANGUAGES CXX to CMakeLists.txt

* 2025-03-24 Wouter Deconinck ([PR#19](https://github.com/key4hep/k4ActsTracking/pull/19))
  - explicitly use `Gaudi::Property<std::string>::value()` in `ActsGeoSvc.cpp`
  - Use `OPENDATADETECTOR_DATA` to get to the ODD geometry file since `OPENDATADETECTOR` has been removed.

* 2025-03-24 Wouter Deconinck ([PR#18](https://github.com/key4hep/k4ActsTracking/pull/18))
  - Install interface header into suitably prefixed directory

* 2025-03-23 Wouter Deconinck ([PR#20](https://github.com/key4hep/k4ActsTracking/pull/20))
  - Remove dependence on GaudiAlg

* 2023-12-01 Leonhard Reichenbach ([PR#17](https://github.com/key4hep/k4ActsTracking/pull/17))
  - Use GeoSvc to get dd4hep geometry instead of loading it ourselves
  - Test if exported geometry contains anything

* 2023-10-17 Leonhard Reichenbach ([PR#13](https://github.com/key4hep/k4ActsTracking/pull/13))
  - Update CI to include tests builds and geometry loading
  - Update CMake files and code to work with newer ACTS versions

* 2023-09-20 Leonhard Reichenbach ([PR#12](https://github.com/key4hep/k4ActsTracking/pull/12))
  - Rename GeoSvc to ActsGeoSvc to avoid clash with `k4FWcore/GeoSvc`

* 2023-09-19 Leonhard Reichenbach ([PR#15](https://github.com/key4hep/k4ActsTracking/pull/15))
  - Replace clang-format workflow with pre-commit
  - Add license

* 2022-07-29 Valentin Volkl ([PR#9](https://github.com/key4hep/k4ActsTracking/pull/9))
  - Make sure to find dependencies properly

* 2022-07-27 Paul Gessinger ([PR#8](https://github.com/key4hep/k4ActsTracking/pull/8))
  - Add basic CI and ACTS geometry service

* 2021-10-27 Andre Sailer ([PR#2](https://github.com/key4hep/k4ActsTracking/pull/2))
  - Cleanup remnants of original project template

* 2021-10-27 Andre Sailer ([PR#1](https://github.com/key4hep/k4ActsTracking/pull/1))
  - Add clang-format config and workflow

