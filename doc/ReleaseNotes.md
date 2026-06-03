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

