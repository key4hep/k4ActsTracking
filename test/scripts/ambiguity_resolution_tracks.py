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

# Synthetic tracks for ACTSDuplicateRemoval and GreedyAmbiguityResolutionAlg
# (test/options/ambiguity_resolution.py), both run on the same tracks.
#
# "write" writes one event per case below, "check" compares the tracks kept by
# each algorithm with the expected ones. Tracks are made of dummy hits, a hit is
# shared if two tracks use it. The track type is the track id.
# - ACTSDuplicateRemoval visits the tracks in order of increasing AtIP tanLambda,
#   which is the order of the list. Two tracks are equal if more than half of the
#   hits of the shorter one are shared; the one with more hits, then the lower
#   chi2, is better.
# - GreedyAmbiguityResolutionAlg (MaximumSharedHits 0, NMeasurementsMin 4) removes
#   the worst track while tracks share hits: the one with the largest fraction of
#   shared hits, then the larger chi2/ndf (same number of hits) or the fewer hits.

import argparse
import sys
from collections.abc import Sequence
from typing import NamedTuple

import edm4hep
import podio
from podio import root_io
from podio.reading import get_reader


class Track(NamedTuple):
    id: int
    hits: Sequence[int]  # indices of the dummy hits
    chi2: float
    ndf: int


class Case(NamedTuple):
    tracks: list[Track]
    duplicate_removal: set[int]  # ids kept by ACTSDuplicateRemoval
    greedy: set[int]  # ids kept by GreedyAmbiguityResolutionAlg


CASES = [
    # Track 3 is made of tracks 1 and 2. Duplicate removal: 3 is equal to both and
    # has more hits. Greedy: all three share all their hits, the shorter ones go.
    Case(
        tracks=[
            Track(1, range(0, 4), 1.0, 4),
            Track(2, range(4, 8), 1.0, 4),
            Track(3, range(0, 8), 1.0, 8),
        ],
        duplicate_removal={3},
        greedy={3},
    ),
    # The same with two copies of track 3: only the one with the lower chi2 stays.
    Case(
        tracks=[
            Track(1, range(0, 4), 1.0, 4),
            Track(2, range(4, 8), 1.0, 4),
            Track(3, range(0, 8), 1.0, 8),
            Track(4, range(0, 8), 2.0, 8),
        ],
        duplicate_removal={3},
        greedy={3},
    ),
    # Same number of hits, 3 of 5 shared: the larger chi2 (chi2/ndf) goes.
    Case(
        tracks=[Track(1, range(0, 5), 5.0, 5), Track(2, range(2, 7), 10.0, 5)],
        duplicate_removal={1},
        greedy={1},
    ),
    # 2 of the 4 hits of track 2 are shared, not more than half: not equal for the
    # duplicate removal. Greedy: track 2 has the larger fraction of shared hits and
    # goes, although its chi2/ndf is better.
    Case(
        tracks=[Track(1, range(0, 6), 18.0, 6), Track(2, range(4, 8), 4.0, 4)],
        duplicate_removal={1, 2},
        greedy={1},
    ),
    # Track 3 is equal to track 1 (better) and track 2 (worse). Duplicate removal:
    # it is dropped at track 1, track 2 stays. Greedy: track 2 (all hits shared,
    # fewer hits) goes, then track 3 (larger chi2/ndf).
    Case(
        tracks=[
            Track(1, range(0, 8), 1.0, 8),
            Track(2, range(6, 10), 1.0, 4),
            Track(3, range(2, 10), 2.0, 8),
        ],
        duplicate_removal={1, 2},
        greedy={1},
    ),
    # Track 2 shares half of its hits with track 1, the other half with track 3.
    # Duplicate removal: track 2 is equal to both and has more hits. Greedy:
    # track 2 has the largest fraction of shared hits, removing it resolves both.
    Case(
        tracks=[
            Track(1, range(0, 6), 6.0, 6),
            Track(2, range(2, 10), 8.0, 8),
            Track(3, range(6, 12), 6.0, 6),
        ],
        duplicate_removal={2},
        greedy={1, 3},
    ),
    # Track 2 has no degrees of freedom (chi2 0): the duplicate removal keeps it for
    # its lower chi2, the greedy resolution ranks it last.
    Case(
        tracks=[Track(1, range(0, 5), 50.0, 5), Track(2, range(2, 7), 0.0, 0)],
        duplicate_removal={2},
        greedy={1},
    ),
    # Track 1 has fewer hits than NMeasurementsMin (4): not in the greedy output.
    Case(
        tracks=[Track(1, range(0, 3), 3.0, 3), Track(2, range(10, 14), 4.0, 4)],
        duplicate_removal={1, 2},
        greedy={2},
    ),
]

# The number of tracker hits created for each case (regardless of whether all of
# them are used in the end)
N_HITS = 16

parser = argparse.ArgumentParser(
    description="Synthetic tracks for ACTSDuplicateRemoval and GreedyAmbiguityResolutionAlg"
)
parser.add_argument("mode", choices=["write", "check"])
parser.add_argument("file", help="Input file to write, or output file to check")
parser.add_argument("--tracks", default="Tracks", help="Input track collection")
parser.add_argument("--deduped", default="DedupedTracks", help="ACTSDuplicateRemoval output")
parser.add_argument(
    "--resolved", default="ResolvedTracks", help="GreedyAmbiguityResolutionAlg output"
)
args = parser.parse_args()

if args.mode == "write":
    writer = root_io.Writer(args.file)
    for case in CASES:
        hits = edm4hep.TrackerHit3DCollection()
        hit_list = [hits.create() for _ in range(N_HITS)]
        track_coll = edm4hep.TrackCollection()
        for i, track in enumerate(case.tracks):
            edm_track = track_coll.create()
            edm_track.setType(track.id)
            edm_track.setChi2(track.chi2)
            edm_track.setNdf(track.ndf)
            for index in track.hits:
                edm_track.addToTrackerHits(hit_list[index])
            state = edm4hep.TrackState()
            state.location = edm4hep.TrackState.AtIP
            state.tanLambda = 0.1 * i
            edm_track.addToTrackStates(state)
        frame = podio.Frame()
        frame.put(hits, "TrackerHits")
        frame.put(track_coll, args.tracks)
        writer.write_frame(frame, "events")
    sys.exit(0)

errors = []
frames = get_reader(args.file).get("events")
if len(frames) != len(CASES):
    errors.append(f"{len(frames)} events, expected {len(CASES)}")
for i_event, (frame, case) in enumerate(zip(frames, CASES)):
    for collection, expected in (
        (args.deduped, case.duplicate_removal),
        (args.resolved, case.greedy),
    ):
        kept = sorted(track.getType() for track in frame.get(collection))
        if kept != sorted(expected):
            errors.append(
                f"event {i_event}, {collection}: kept tracks {kept}, expected {sorted(expected)}"
            )

for error in errors:
    print(error)
sys.exit(1 if errors else 0)
