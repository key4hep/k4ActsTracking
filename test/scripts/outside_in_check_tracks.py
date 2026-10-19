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
import math

import edm4hep
from podio.reading import get_reader

parser = argparse.ArgumentParser(description="Check the outside-in CKF output")
parser.add_argument("file", help="CKF output file")
parser.add_argument("--tracks", default="CKFTracks", help="Track collection")
args = parser.parse_args()

TS = edm4hep.TrackState


def rxy(p):
    return math.hypot(p.x, p.y)


def state_at(track, location):
    return next((ts for ts in track.getTrackStates() if ts.location == location), None)


n_tracks = 0
for i, frame in enumerate(get_reader(args.file).get("events")):
    for j, track in enumerate(frame.get(args.tracks)):
        n_tracks += 1
        tag = f"event {i}, track {j}"
        first = state_at(track, TS.AtFirstHit)
        last = state_at(track, TS.AtLastHit)
        assert first is not None and last is not None, f"{tag}: no AtFirstHit or AtLastHit state"
        assert rxy(first.referencePoint) < rxy(last.referencePoint), (
            f"{tag}: AtFirstHit at r = {rxy(first.referencePoint):.1f} mm, "
            f"not inside AtLastHit at r = {rxy(last.referencePoint):.1f} mm"
        )

print(f"{n_tracks} tracks checked")
