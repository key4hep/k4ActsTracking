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

# Synthetic tracks for GreedyAmbiguityResolutionAlg
# (test/options/greedy_ambiguity_resolution.py, MaximumSharedHits 0, NMeasurementsMin 4).
#
# "write" writes one event per case below, "check" compares the tracks kept in
# the output with the expected ones. Tracks are made of dummy hits, a hit is
# shared if two tracks use it. While tracks share hits, the worst one is
# removed: the one with the largest fraction of shared hits, then the larger
# chi2/ndf (same number of hits) or the fewer hits. The track type is the track id.

import argparse
import sys

import edm4hep
import podio
from podio import root_io
from podio.reading import get_reader

# (id, hit indices, chi2, ndf) per track, and the ids that must be kept
CASES = [
    # The larger fraction of shared hits goes, even with the better chi2/ndf:
    # tracks 1 and 2 share two hits, 2/6 of track 1 and 2/4 of track 2.
    ([(1, range(0, 6), 18.0, 6), (2, range(4, 8), 4.0, 4)], {1}),
    # Same fraction and number of hits: the larger chi2/ndf goes.
    ([(1, range(0, 5), 5.0, 5), (2, range(3, 8), 10.0, 5)], {1}),
    # A track without degrees of freedom ranks last.
    ([(1, range(0, 5), 50.0, 5), (2, range(3, 8), 0.0, 0)], {1}),
    # Track 2 shares one hit with each of 1 and 3: removing it resolves both.
    ([(1, range(0, 6), 6.0, 6), (2, [5, 6, 12, 13], 4.0, 4), (3, range(6, 12), 6.0, 6)], {1, 3}),
    # Tracks with fewer hits than NMeasurementsMin are not in the output.
    ([(1, range(0, 3), 3.0, 3), (2, range(10, 14), 4.0, 4)], {2}),
]

parser = argparse.ArgumentParser(description="Synthetic tracks for GreedyAmbiguityResolutionAlg")
parser.add_argument("mode", choices=["write", "check"])
parser.add_argument("file", help="Input file to write, or output file to check")
parser.add_argument("--tracks", default="Tracks", help="Input track collection")
parser.add_argument("--resolved", default="ResolvedTracks", help="Output track collection")
args = parser.parse_args()

if args.mode == "write":
    writer = root_io.Writer(args.file)
    for tracks, _ in CASES:
        hits = edm4hep.TrackerHit3DCollection()
        hit_list = [hits.create() for _ in range(16)]
        track_coll = edm4hep.TrackCollection()
        for track_id, hit_indices, chi2, ndf in tracks:
            track = track_coll.create()
            track.setType(track_id)
            track.setChi2(chi2)
            track.setNdf(ndf)
            for index in hit_indices:
                track.addToTrackerHits(hit_list[index])
        frame = podio.Frame()
        frame.put(hits, "TrackerHits")
        frame.put(track_coll, args.tracks)
        writer.write_frame(frame, "events")
    sys.exit(0)

errors = []
frames = get_reader(args.file).get("events")
if len(frames) != len(CASES):
    errors.append(f"{len(frames)} events, expected {len(CASES)}")
for i_event, (frame, (_, expected)) in enumerate(zip(frames, CASES)):
    kept = sorted(track.getType() for track in frame.get(args.resolved))
    if kept != sorted(expected):
        errors.append(f"event {i_event}: kept tracks {kept}, expected {sorted(expected)}")

for error in errors:
    print(error)
sys.exit(1 if errors else 0)
