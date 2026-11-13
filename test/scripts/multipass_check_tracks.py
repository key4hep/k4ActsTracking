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
"""Check the output of make_multipass_tracking: every track has an AtIP state and enough hits,
and no hit is on two tracks (the ambiguity resolution allows no shared hits)."""

import argparse

import edm4hep
from podio.reading import get_reader

parser = argparse.ArgumentParser(description="Check the multi-pass CKF output")
parser.add_argument("file", help="CKF output file")
parser.add_argument("--tracks", default="CKFTracks", help="Track collection")
parser.add_argument("--minHits", type=int, default=4, help="Minimum hits per track")
# The CLD ctest sample has about 28 reconstructable muons; conformal tracking finds 26
parser.add_argument(
    "--minTracks", type=int, default=15, help="Minimum number of tracks in the file"
)
args = parser.parse_args()

n_tracks = 0
for i, frame in enumerate(get_reader(args.file).get("events")):
    seen = set()
    for j, track in enumerate(frame.get(args.tracks)):
        n_tracks += 1
        tag = f"event {i}, track {j}"
        hits = [
            (h.getObjectID().collectionID, h.getObjectID().index) for h in track.getTrackerHits()
        ]
        assert len(hits) >= args.minHits, f"{tag}: {len(hits)} hits"
        assert track.getTrackState(edm4hep.TrackState.AtIP).has_value(), f"{tag}: no AtIP state"
        shared = seen.intersection(hits)
        assert not shared, f"{tag}: {len(shared)} hits also on another track"
        seen.update(hits)

print(f"{n_tracks} tracks")
assert n_tracks >= args.minTracks, f"Sharp drop in tracks (less than {args.minTracks})"
