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
    TelescopeCKFTrackingAlg,
)
from k4FWCore.parseArgs import parser


def _get_compact_file():
    parser.add_argument(
        "--compactFile",
        help="The geometry compact file to use for reconstruction",
        type=str,
        required=True,
    )
    args = parser.parse_known_args()[0]
    return args.compactFile


def _get_material_map_file():
    parser.add_argument(
        "--materialMapFile",
        help="ACTS material map (.json/.cbor/.root) to decorate the tracking "
        "geometry with. Without it the geometry carries only the proto-material "
        "placeholders that the material mapping projects onto, i.e. tracking "
        "sees no passive material.",
        type=str,
        default="",
    )
    args = parser.parse_known_args()[0]
    return args.materialMapFile


def make_services(use_dd4hep_field=False):
    """Configure all the necessary services (including getting the geometry from
    the command line geometry).

    Set use_dd4hep_field=True to make ACTS use the real, position-dependent
    DD4hep field (needed for localized fields such as the LUXE dipole). The
    default keeps the constant-field behaviour used by the collider clients.
    """
    compact_file = _get_compact_file()
    return [
        GeoSvc("GeoSvc", detectors=[compact_file], EnableGeant4Geo=False),
        ActsGeoSvc(
            "ActsGeoSvc",
            UseDD4hepBField=use_dd4hep_field,
            MaterialMapFile=_get_material_map_file(),
        ),
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

    props = dict(
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
    )
    props.update(ckf_args)
    return CKFTrackingAlg("CKFTracking", **props)


def make_telescope_ckf_tracking(
    hit_merger,
    hit_rel_merger,
    seeding_cellids,
    layer_z_tolerance=20.0,
    collinearity_cut=2.0,
    nominal_momentum=5.0,
    reference_z=0.0,
    constrain_momentum=True,
    **ckf_args,
):
    """Configure the TelescopeCKFTrackingAlg (straight-line telescope seeding).

    For field-free planar geometries (e.g. LUXE), where the collider helix
    seeder of CKFTrackingAlg finds nothing. Uses the same CKF settings and
    collection names as make_ckf_tracking and exposes the telescope-specific
    knobs as named arguments. Additional TelescopeCKFTrackingAlg properties can
    be passed as keyword arguments.

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
    constrain_momentum : bool
        Determine each track's otherwise-unconstrained q/p from the requirement
        that it originates at the target (the reference_z plane centre), so the
        field-aware back-extrapolation lands on the target and the reconstructed
        momentum matches the dipole deflection. On by default; set False to keep
        the seed's nominal momentum. Requires the real DD4hep field (see
        make_services(use_dd4hep_field=True)). The companion option
        MomentumReportZ (the in-magnet plane where a momentum-carrying track
        state is added) can be passed through **ckf_args.
    """

    props = dict(
        RunCKF=True,
        CKF_Chi2CutOff=10,
        CKF_NumMeasurementsCutOff=1,
        LayerZTolerance=layer_z_tolerance,
        CollinearityCut=collinearity_cut,
        NominalMomentum=nominal_momentum,
        ReferenceZ=reference_z,
        ConstrainMomentum=constrain_momentum,
        SeedingSensorsCellIDs=seeding_cellids,
        OutputTrackCollection="CKFTracks",
        OutputSeedCollection="CKFTrackSeeds",
        InputTrackerHitCollection=hit_merger.OutputCollection,
        InputTrackerHitRelationCollection=hit_rel_merger.OutputCollection,
        OutputLevel=INFO,
    )
    props.update(ckf_args)
    return TelescopeCKFTrackingAlg("CKFTracking", **props)
