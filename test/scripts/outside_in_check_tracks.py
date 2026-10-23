#!/usr/bin/env python
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

# Lightweight check on the outside-in CKF output: every track is ordered
# inside-out, i.e. its AtFirstHit state is at a smaller radius than AtLastHit.
import argparse

import edm4hep
import ROOT
from podio.reading import get_reader

# Load edm4hep vectors utils for rxy function
ROOT.gInterpreter.LoadFile("edm4hep/utils/vector_utils.h")
rxy = edm4hep.utils.magnitudeTransverse

parser = argparse.ArgumentParser(description="Check the outside-in CKF output")
parser.add_argument("file", help="CKF output file")
parser.add_argument("--tracks", default="CKFTracks", help="Track collection")
args = parser.parse_args()

TS = edm4hep.TrackState

n_tracks = 0
for i, frame in enumerate(get_reader(args.file).get("events")):
    for j, track in enumerate(frame.get(args.tracks)):
        n_tracks += 1
        tag = f"event {i}, track {j}"
        first = track.getTrackState(TS.AtFirstHit)
        last = track.getTrackState(TS.AtLastHit)
        assert first.has_value() and last.has_value(), f"{tag}: no AtFirstHit or AtLastHit state"
        r_first, r_last = rxy(first.value().referencePoint), rxy(last.value().referencePoint)
        assert r_first < r_last, (
            f"{tag}: AtFirstHit at r = {r_first:.1f} mm, not inside AtLastHit at r = {r_last:.1f} mm"
        )

print(f"{n_tracks} tracks checked")
