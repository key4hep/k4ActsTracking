
# ACTSTracking

Package for running track reconstructions using the [ACTS](https://github.com/acts-project/acts/) library. This is not tied to a given geometry, but has been developed for the Muon Collider detector model v1.

ACTSTracking is distributed under the [GPLv3 License](http://www.gnu.org/licenses/gpl-3.0.en.html).

[![License](https://www.gnu.org/graphics/gplv3-127x51.png)](https://www.gnu.org/licenses/gpl-3.0.en.html)

## License and Copyright
Copyright (C), ACTSTracking Authors

ACTSTracking is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License long with this program.  If not, see <http://www.gnu.org/licenses/>.

## Resource Files
The ACTSTracking package contains several resource files (tracking geometry, material description). Their path is given to processors via configuration parameters (ie `TGeoFile`). If the path is relative, then the following locations are search for it:
-   current working directory
-   `ACTSTRACKING_SOURCEDIR = ${CMAKE_SOURCE_DIR}`, aka source code location
-   `ACTSTRACKING_DATADIR = ${CMAKE_INSTALL_FULL_DATADIR}/ACTSTracking}`, aka result of  `make install`

## Available Processors
See `examples/` for steering files demonstrating the usage of the provided processors.

### ACTSProcBase
Base processor implementing some of the common actions. This processor is not to be used directly. It is designed to be inherited by other processors that implement the specific track reconstruction algorithm

The responsibilities include:
 - Load tracking detector geometry and material.
 - Create magnetic field description.

| **Parameter** | **Description**                                           |
|---------------|-----------------------------------------------------------|
| MatFile       | Path to the material description json file. Can be empty. |
| TGeoFile      | Path to the tracker geometry file.                        |

#### Tracking Geometry
The tracking geometry is loaded from a `TGeoManager` stored inside a ROOT file at parameter `TGeoFile`. The building of the ACTS volumes is done using the [ACTS TGeo plugin](https://acts.readthedocs.io/en/latest/plugins/tgeo.html). The builder parameters are currently hardcoded for the MuColl_v1 detector model.

Material is optionally loaded using by specifying the `MatFile` parameter. It should corresponds to the JSON file generated using the [ACTS material mapping tools](https://acts.readthedocs.io/en/latest/examples/howto/run_material_mapping.html).

Files corresponding to the MuColl_v1 detector model are available inside `data/`.

#### Magnetic Field
A constant magnetic field is assumed. The strength is taken from the loaded DD4hep detector description.

### ACTSTruthTrackingProc
Processor that implements truth tracking. Pseudo-tracks are formed by finding hits associated to a charged MC particles. The pseudo-tracks are then fit using a Kalman Filter. This processor is equivalent to `TruthTrackFinder`.

| **Parameter**                   | **Description**                                          |
|---------------------------------|----------------------------------------------------------|
| TrackerHitCollectionNames       | Name of the TrackerHit input collections                 |
| SimTrackerHitRelCollectionNames | Name of TrackerHit to SimTrackerHit relation collections |
| MCParticleCollectionName        | Name of the MCParticle input collection                  |
| TrackCollectionName             | Name of track output collection                          |

### ACTSTruthCKFTrackingProc
Processor that implements truth-seeded Combinatorial Kalman Filter. Seeds for the CKF algorithm are formed by using the kinematics of charged MC particles, with vertex set at (0,0,0).

#### Input/Output Configuration
| **Parameter**             | **Description**                                               |
|---------------------------|---------------------------------------------------------------|
| TrackerHitCollectionNames | Name of the TrackerHit input collections.                     |
| MCParticleCollectionName  | Name of the MCParticle input collection (used for seeding).   |
| TrackCollectionName       | Name of track output collection.                              |

#### CKF Configuration
| **Parameter**             | **Description**                                               |
|---------------------------|---------------------------------------------------------------|
| CKF_Chi2CutOff            | Maximum local chi2 contribution.                              |
| CKF_NumMeasurementsCutOff | Maximum number of associated measurements on a single surface.|


### ACTSSeededCKFTrackingProc
Processor that implements tripled seeding Combinatorial Kalman Filter. This processor can be used for track reconstruction without any truth assumptions.

#### Input/Output Configuration
| **Parameter**                | **Description**                                                                                                           |
|------------------------------|---------------------------------------------------------------------------------------------------------------------------|
| TrackerHitCollectionNames    | Name of the TrackerHit input collections.                                                                                 |
| SeedCollectionName           | Name of seed output collection.                                                                                           |
| TrackCollectionName          | Name of track output collection.                                                                                          |

#### Triplet Seeding Configuration
| **Parameter**                | **Description**                                                                                                           |
|------------------------------|---------------------------------------------------------------------------------------------------------------------------|
| SeedingLayers                | Layers to use for seeding in format `VolID LayID`, one per line. ID's are ACTS GeometryID's. `*` can be used to wildcard. |
| SeedFinding_RMax             | Maximum radius of hits to consider.                                                                                       |
| SeedFinding_DeltaRMin        | Minimum dR between hits in a seed.                                                                                        |
| SeedFinding_DeltaRMax        | Maximum dR between hits in a seed.                                                                                        |
| SeedFinding_CollisionRegion  | Size of the collision region in one direction (assumed symmetric).                                                        |
| SeedFinding_ZMax             | Maximum z of hits hits to consider.                                                                                       |
| SeedFinding_RadLengthPerSeed | Average radiation length per seed.                                                                                        |
| SeedFinding_SigmaScattering  | Number of sigmas to allow in scattering angle.                                                                            |
| SeedFinding_MinPt            | Minimum pT of tracks to seed.                                                                                             |

#### CKF Configuration
| **Parameter**                | **Description**                                                                                                           |
|------------------------------|---------------------------------------------------------------------------------------------------------------------------|
| RunCKF                       | Run tracking using CKF. False means stop at the seeding stage.                                                            |
| InitialTrackError_RelP       | Track error estimate, momentum component (relative).                                                                      |
| InitialTrackError_Phi        | Track error estimate, phi (radians).                                                                                      |
| InitialTrackError_Lambda     | Track error estimate, lambda (radians).                                                                                   |
| InitialTrackError_Pos        | Track error estimate, local position (mm).                                                                                |
| CKF_Chi2CutOff               | Maximum local chi2 contribution.                                                                                          |
| CKF_NumMeasurementsCutOff    | Maximum number of associated measurements on a single surface.                                                            |

### ACTSDuplicateRemoval
Processors to remove overlapping (sharing multiple hits) tracks. The algorithms works as follows:
1. Sort input tracks by tan(lambda).
2. Create final tracks container
3. Compare each sorted input track against last 10 final tracks
	1. Skip tracks not sharing more than 50% of hits. Shared fractions are defined as `shared hits / min(track 1 hits, track 2 hits)`
	2. Replace final track if input track is of higher quality. Higher quality tracks has more hits. In case of same number of hits, the higher quality track has a smaller chi2.
