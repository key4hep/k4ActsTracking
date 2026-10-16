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

# Beam-spot sizes (sigma x, y, z) [mm] per centre-of-mass energy [GeV]: the
# collision region of the FCC Feasibility Study Report parameters
# (arXiv:2505.00274, Table 2), as given in FCCeePhysicsPerformance's
# General/README.md ("Summary table, FSR parameters (2025)"). 
BEAM_SPOT_SIZES = {
    91: (6.36e-3, 28.3e-6, 0.424),
    160: (15.56e-3, 31.8e-6, 0.999),
    240: (9.19e-3, 22.6e-6, 0.606),
    365: (26.17e-3, 31.1e-6, 1.198),
}

parser.add_argument(
    "--cms",
    help="Centre-of-mass energy [GeV], selects the beam-spot size",
    type=int,
    choices=sorted(BEAM_SPOT_SIZES),
    default=240,
)
args = parser.parse_known_args()[0]

# Tuned on CLD_o2_v08 (Key4hep nightly 2026-09-30)
svcList, vertexing = make_vertexing(
    BeamSpotSize=list(BEAM_SPOT_SIZES[args.cms]),
    SeedMaxD0Significance=10.0,
    AnnealingTemperatures=[256.0, 64.0, 16.0, 4.0, 2.0, 1.5, 1.0],
    AnnealingCutOff=9.0,
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
