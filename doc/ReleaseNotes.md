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

