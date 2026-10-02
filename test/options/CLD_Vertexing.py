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

# ACTS primary vertexing with the settings tuned for CLD, on CLD's own tracks
# (SiTracks_Refitted from ConformalTracking + RefitFinal), with the FCC-ee beam
# spot of the chosen centre-of-mass energy as constraint.
#
#   k4run CLD_Vertexing.py \
#       --compactFile $K4GEO/FCCee/CLD/compact/CLD_o2_v08/CLD_o2_v08.xml \
#       --cms 240 \
#       --IOSvc.Input particle_gun_CLD_o2_v08_REC.edm4hep.root \
#       --IOSvc.Output particle_gun_CLD_o2_v08_VTX.edm4hep.root
#
# Any setting below can still be overridden on the command line with
# --VertexFindingAlg.<Property>.

import os
import sys

from Gaudi.Configuration import INFO
from k4FWCore import ApplicationMgr
from k4FWCore.parseArgs import parser

sys.path.insert(0, os.path.dirname(__file__))

from _vertexing_helpers import make_vertexing

# Beam-spot sizes (sigma x, y, z) [mm] per centre-of-mass energy [GeV]. Copied
# from BEAM_SPOT_SIZES in CLDConfig's CLDReconstruction.py, which LCFIPlus uses
# there; keep the two in sync.
BEAM_SPOT_SIZES = {
    91: (5.96e-3, 23.8e-6, 0.397),
    160: (14.7e-3, 46.5e-6, 0.97),
    240: (9.8e-3, 25.4e-6, 0.65),
    365: (27.3e-3, 48.8e-6, 1.33),
}

parser.add_argument(
    "--cms",
    help="Centre-of-mass energy [GeV], selects the beam-spot size",
    type=int,
    choices=sorted(BEAM_SPOT_SIZES),
    default=240,
)
args = parser.parse_known_args()[0]

# Tuned on CLD_o2_v08 (Key4hep nightly 2026-09-30) on three event types at once,
# each split into a tuning and an independent test sample: a 10-muon gun
# (1.5-100 GeV, 240 GeV beam spot; 6000 + 5000 events), Z -> bb at 91 GeV
# (5000 + 5000) and Z -> uu/dd at 91 GeV (2500 + 2500). The settings are the
# ones closest to the best on all three, not the best on any one. On the test
# samples, compared with LCFIPlus on the same tracks:
#   muon gun:  PV efficiency 100 % vs 99.3 %, sigma_eff x / z 1.05 / 1.06 um vs
#              1.21 / 1.12 um, vertex pull widths 1.0 vs 1.65
#   Z -> bb:   PV efficiency 82.5 % vs 72.5 %, sigma_eff x / z 4.8 / 8.1 um vs
#              5.9 / 10.5 um, 1.95 vs 2.58 tracks from b/c decays in the PV
#   Z -> uu/dd: PV efficiency 99.6 % vs 99.2 %, sigma_eff x / z 2.96 / 3.18 um vs
#              3.06 / 3.18 um, pull widths 1.0 vs 1.2
svcList, vertexing = make_vertexing(
    BeamSpotSize=list(BEAM_SPOT_SIZES[args.cms]),
    # CLD's d0 resolution (~1 um for hard tracks) is far below the transverse
    # beam-spot size, so the Acts default of 3.5 sigma w.r.t. the beam line
    # leaves no track to seed from when the collision is off the beam line.
    # 10 and 30 give identical results.
    SeedMaxD0Significance=10.0,
    # The fit starts on the beam line. A hot start gives precise tracks from
    # collisions off it (chi2 ~ 100 at the seed) a say before the weights
    # harden; the Acts default ladder starts at 64 and loses a few such events.
    # Also improves Z -> bb (z resolution, 2-3x fewer split vertices).
    AnnealingTemperatures=[256.0, 64.0, 16.0, 4.0, 2.0, 1.5, 1.0],
    # chi2 at which a track gets weight 0.5, the Acts default. 12 and 16 let
    # more b/c-decay tracks into the PV (Z -> bb efficiency -1.3 / -3 points)
    # and gain only 0.01-0.02 um on the muon gun.
    AnnealingCutOff=9.0,
    # Calibration of CLD's track uncertainties, which miss a momentum-independent
    # term: d0 / z0 pulls w.r.t. the true vertex are ~1.0-1.1 below a few GeV and
    # grow to ~1.7 / 1.5 at 100 GeV. Fitting sigma_true^2 = (k sigma)^2 + c^2 to
    # the pull widths in momentum bins, with the prompt tracks of the muon gun
    # and Z -> bb tuning samples together (0.3-100 GeV), gives k = 1.09 / 1.08 and
    # c = 1.8 um for d0 / z0; pulls are then 0.94-1.08 in both samples. Costs
    # ~1 point of Z -> bb PV efficiency (slightly more compatible displaced
    # tracks), gives correct vertex uncertainties everywhere. Specific to CLD's
    # track fit: rederive it when the tracking or the geometry changes.
    TrackCovarianceScale=1.08,
    ImpactParameterErrorTerm=0.0018,
)

ApplicationMgr(
    TopAlg=[vertexing],
    ExtSvc=svcList,
    OutputLevel=INFO,
    EvtSel="NONE",
    EvtMax=-1,
)
