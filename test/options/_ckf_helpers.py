#!/usr/bin/env python3
#
# Copyright (c) 2014-2024 Key4hep-Project.
#
# This file is part of Key4hep.
# See https://key4hep.github.io/key4hep-doc/ for further info.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

import os
import sys

from Gaudi.Configuration import INFO
from Gaudi.Configurables import (
    ActsGeoSvc,
    CKFTrackingAlg,
    CollectionMerger,
    EventDataSvc,
    GeoSvc,
)
from k4FWCore.parseArgs import parser


def _get_compact_file():
    parser.add_argument(
        "--compactFile",
        help="The geometry compact file to use for reconstruction",
        type=str,
    )
    args = parser.parse_known_args()[0]
    return args.compactFile


def make_services():
    """Configure all the necessary services (including getting the geometry from
    the command line geometry)"""
    compact_file = _get_compact_file()
    return [
        GeoSvc("GeoSvc", detectors=[compact_file], EnableGeant4Geo=False),
        ActsGeoSvc("ActsGeoSvc"),
        EventDataSvc("EventDataSvc"),
    ]


def make_hit_mergers(
    hit_collections,
    relation_collections,
    hits_output="AllTrackerHits",
    relations_output="AllTrackerHitRelations",
):
    """Configure the hit (and relation) mergers"""
    hit_merger = CollectionMerger(
        "MergeHits",
        InputCollections=hit_collections,
        OutputCollection=hits_output,
    )
    hit_rel_merger = CollectionMerger(
        "MergeHitRelations",
        InputCollections=relation_collections,
        OutputCollection=relations_output,
    )
    return hit_merger, hit_rel_merger


def make_ckf_tracking(
    hit_merger,
    hit_rel_merger,
    seeding_cellids,
    **ckf_args,
):
    """Configure the CKFTrackingAlg such that it does **some** tracking

    Additional CKFTrackingAlg properties can be passed as keyword arguments.

    NOTE: This is really just an example on how to configure it for some
    technical tests! This will not work for any meaningful tracking!
    """

    return CKFTrackingAlg(
        "CKFTracking",
        RunCKF=True,
        CKF_Chi2CutOff=10,
        SeedFinding_RMax=150,
        SeedFinding_MinPt=500,
        SeedFinding_ImpactMax=3,
        CKF_NumMeasurementsCutOff=1,
        SeedFinding_SigmaScattering=50,
        SeedFinding_CollisionRegion=6,
        SeedFinding_RadLengthPerSeed=0.1,
        SeedingSensorsCellIDs=seeding_cellids,
        OutputTrackCollection="CKFTracks",
        OutputSeedCollection="CKFTrackSeeds",
        InputTrackerHitCollection=hit_merger.OutputCollection,
        InputTrackerHitRelationCollection=hit_rel_merger.OutputCollection,
        OutputLevel=INFO,
        **ckf_args,
    )


def make_telescope_ckf_tracking(
    hit_merger,
    hit_rel_merger,
    seeding_cellids,
    layer_z_tolerance=20.0,
    collinearity_cut=2.0,
    nominal_momentum=5.0,
    reference_z=0.0,
    **ckf_args,
):
    """Configure the CKFTrackingAlg with the straight-line *telescope* seeder.

    For field-free planar geometries (e.g. LUXE), where the default collider
    helix seeder finds nothing. This is a thin wrapper over make_ckf_tracking
    that flips SeedingMode to "Telescope" and exposes the telescope-specific
    knobs as named arguments (the cylindrical SeedFinding_* values set by
    make_ckf_tracking are ignored in this mode). It does not affect the
    collider/barrel configuration.

    Parameters
    ----------
    layer_z_tolerance : float
        Max |dz| [mm] between space points assigned to the same layer.
    collinearity_cut : float
        Max transverse distance [mm] of the middle hit from the bottom-top line.
    nominal_momentum : float
        Nominal seed momentum [GeV] for the field-free straight-line seeds.
    reference_z : float
        z [mm] of the beam-perpendicular plane the fitted tracks are
        extrapolated to for the AtIP state (replaces the beamline perigee,
        which is unreachable for beam-parallel tracks).
    """

    return make_ckf_tracking(
        hit_merger,
        hit_rel_merger,
        seeding_cellids,
        SeedingMode="Telescope",
        Telescope_LayerZTolerance=layer_z_tolerance,
        Telescope_CollinearityCut=collinearity_cut,
        Telescope_NominalMomentum=nominal_momentum,
        Telescope_ReferenceZ=reference_z,
        **ckf_args,
    )
