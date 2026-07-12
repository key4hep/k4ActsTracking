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
from Gaudi.Configurables import ACTSDuplicateRemoval, DDPlanarDigi, FilterTracksAlg
from k4FWCore import ApplicationMgr, IOSvc

sys.path.insert(0, os.path.dirname(__file__))

from _ckf_helpers import (
    make_telescope_ckf_tracking,
    make_services,
    make_hit_mergers,
)

# Use the real DD4hep field so ACTS accounts for the localized LUXE dipole when
# extrapolating back to the IP (the tracker itself is field-free).
svcList = make_services(use_dd4hep_field=True)
iosvc = IOSvc("IOSvc")

algList = []

# The LUXE_v0 geometry has a single planar tracker (DD4hep detector "Tracker")
# whose sensitive hits end up in the "SiHits" SimTrackerHit collection (named
# after the "SiHits" readout). Digitize those into space points for the CKF.
algList.append(
    DDPlanarDigi(
        "TrackerDigitizer",
        SubDetectorName="Tracker",
        IsStrip=False,
        ResolutionT=[2000.],
        ResolutionU=[0.005],
        ResolutionV=[0.005],
        # The LUXE "SiHits" readout encodes a fine CartesianGridXY (the x/y
        # segmentation sits in bits 32-63), while the DD4hep surface map is keyed
        # at sensor level (system:1,side:1,layer:2,module:1,sensor:5 -> bits 0-9).
        # DDPlanarDigi looks up the surface with the *full* cellID, so mask it to
        # the low 32 bits to drop the segmentation and match the surface volumeID.
        CellIDBits=32,
        SimTrackHitCollectionName=["SiHits"],
        SimTrkHitRelCollection=["SiHitsRelations"],
        TrackerHitCollectionName=["SiTrackerHits"],
    )
)

hit_merger, hit_rel_merger = make_hit_mergers(
    hit_collections=["SiTrackerHits"],
    relation_collections=["SiHitsRelations"],
)
algList.extend([hit_merger, hit_rel_merger])

# LUXE's tracker sits in a field-free region (the dipole is localized upstream),
# so the collider helix seeder finds nothing. Use the straight-line telescope
# seeder instead; this configuration does not affect the barrel/collider clients.
ckf_tracking = make_telescope_ckf_tracking(
    hit_merger,
    hit_rel_merger,
    # LUXE is a four-layer (layer:0..3) telescope; use all of them as the pool
    # of seed space points. Omitted cellID fields act as wildcards.
    seeding_cellids=["layer:0|1|2|3"],
)
algList.append(ckf_tracking)

# The straight-line seeder is combinatorial (~5x more tracks than true e+), so
# clean the output: remove duplicates that share >50% of hits, then keep only
# well-formed telescope tracks. These are LUXE-only; the collider clients run
# their own selection. FilterTracksAlg's collider-oriented cuts that don't apply
# to a planar telescope are disabled here:
#  - the vertex/inner/outer subdetector-hit cuts (LUXE fills no such subdetectors),
#  - the pT cut (omega, hence pT, is degenerate in the field-free AtIP reference).
# What remains is the requirement of a complete four-hit track with no holes.
algList.append(
    ACTSDuplicateRemoval(
        "TrackDuplicateRemoval",
        InputTrackCollectionName=["CKFTracks"],
        OutputTrackCollectionName=["CKFTracksDeduped"],
    )
)
algList.append(
    FilterTracksAlg(
        "FilterTracks",
        InputTrackCollectionName=["CKFTracksDeduped"],
        OutputTrackCollectionName=["CKFTracksFiltered"],
        NHitsTotal=3,  # keep tracks with > 3 hits, i.e. the full four-layer telescope
        NHitsVertex=0,  # disable: LUXE has no vertex subdetector
        NHitsInner=0,  # disable: LUXE has no inner-tracker subdetector
        NHitsOuter=0,  # disable: LUXE has no outer-tracker subdetector
        MinPt=0.0,  # disable: pT is degenerate where the field vanishes (omega ~ 0)
        MaxHoles=0,  # require no holes
        OutputLevel=INFO,
    )
)

ApplicationMgr(
    TopAlg=algList, ExtSvc=svcList, OutputLevel=INFO, EvtSel="NONE", EvtMax=-1
)
