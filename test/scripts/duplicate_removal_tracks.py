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

# Synthetic tracks for ACTSDuplicateRemoval (test/options/duplicate_removal.py).
#
# "write" writes one event per case below, "check" compares the tracks kept in
# the output with the expected ones. Tracks are made of dummy hits; two tracks
# are equal if more than half of the hits of the shorter one are shared, and the
# one with more hits (then the lower chi2) is better. The tracks are listed in
# the order the algorithm visits them (increasing AtIP tanLambda), and the track
# type is the track id.

import argparse
import sys

import edm4hep
import podio
from podio import root_io
from podio.reading import get_reader

# (id, hit indices, chi2) per track, and the ids that must be kept
CASES = [
    # A better track replaces all equal kept tracks: the full track 3 is equal
    # to both of its halves 1 and 2, which are not equal to each other.
    ([(1, range(0, 4), 1.0), (2, range(4, 8), 1.0), (3, range(0, 8), 1.0)], {3}),
    # A track is dropped at the first equal kept track that is at least as good:
    # track 3 is worse than track 1 and better than track 2, equal to both.
    ([(1, range(0, 8), 1.0), (2, range(6, 10), 1.0), (3, range(2, 10), 2.0)], {1, 2}),
    # Two copies of the full track next to its two halves: only one survives.
    (
        [
            (1, range(0, 4), 1.0),
            (2, range(4, 8), 1.0),
            (3, range(0, 8), 1.0),
            (4, range(0, 8), 2.0),
        ],
        {3},
    ),
]

parser = argparse.ArgumentParser(description="Synthetic tracks for ACTSDuplicateRemoval")
parser.add_argument("mode", choices=["write", "check"])
parser.add_argument("file", help="Input file to write, or output file to check")
parser.add_argument("--tracks", default="Tracks", help="Input track collection")
parser.add_argument("--deduped", default="DedupedTracks", help="Output track collection")
args = parser.parse_args()

if args.mode == "write":
    writer = root_io.Writer(args.file)
    for tracks, _ in CASES:
        hits = edm4hep.TrackerHit3DCollection()
        hit_list = [hits.create() for _ in range(10)]
        track_coll = edm4hep.TrackCollection()
        for i, (track_id, hit_indices, chi2) in enumerate(tracks):
            track = track_coll.create()
            track.setType(track_id)
            track.setChi2(chi2)
            for index in hit_indices:
                track.addToTrackerHits(hit_list[index])
            state = edm4hep.TrackState()
            state.location = edm4hep.TrackState.AtIP
            state.tanLambda = 0.1 * i
            track.addToTrackStates(state)
        frame = podio.Frame()
        frame.put(hits, "TrackerHits")
        frame.put(track_coll, args.tracks)
        writer.write_frame(frame, "events")
    sys.exit(0)

errors = []
frames = list(get_reader(args.file).get("events"))
if len(frames) != len(CASES):
    errors.append(f"{len(frames)} events, expected {len(CASES)}")
for i_event, (frame, (_, expected)) in enumerate(zip(frames, CASES)):
    kept = sorted(track.getType() for track in frame.get(args.deduped))
    if kept != sorted(expected):
        errors.append(f"event {i_event}: kept tracks {kept}, expected {sorted(expected)}")

for error in errors:
    print(error)
sys.exit(1 if errors else 0)
