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

# GreedyAmbiguityResolutionAlg on a track collection, e.g. the synthetic tracks of
# test/scripts/greedy_ambiguity_tracks.py:
#   k4run greedy_ambiguity_resolution.py --IOSvc.Input <in> --IOSvc.Output <out>

from Gaudi.Configuration import INFO
from Gaudi.Configurables import GreedyAmbiguityResolutionAlg, EventDataSvc
from k4FWCore import ApplicationMgr, IOSvc

iosvc = IOSvc("IOSvc")
iosvc.outputCommands = ["keep *"]

ApplicationMgr(
    TopAlg=[
        GreedyAmbiguityResolutionAlg(
            "GreedyAmbiguityResolutionAlg",
            InputTrackCollectionName=["Tracks"],
            OutputTrackCollectionName=["ResolvedTracks"],
        )
    ],
    ExtSvc=[EventDataSvc("EventDataSvc")],
    OutputLevel=INFO,
    EvtSel="NONE",
    EvtMax=-1,
)
