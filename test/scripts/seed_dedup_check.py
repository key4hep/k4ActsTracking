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

# Compare a CKF run with SeedDeduplication against the same run without it.
# Both must use NumThreads = 1, so deduplication spans the whole event.
#
# Deduplication only skips the CKF for some seeds, and the CKF from one seed
# does not depend on any other, so per event:
#   - the seed collections are identical (seeds are written before the CKF),
#   - the deduplicated tracks are the baseline tracks with some removed, in the
#     same order (same hits, ndf and chi2),
# and over all events at least one track is removed (unless --allowNoRemoval).
import argparse
import math

from podio.reading import get_reader

parser = argparse.ArgumentParser(
    description="Check the SeedDeduplication CKF output against a baseline"
)
parser.add_argument("baseline", help="CKF output without SeedDeduplication")
parser.add_argument("dedup", help="CKF output with SeedDeduplication")
parser.add_argument("--tracks", default="CKFTracks", help="Track collection")
parser.add_argument("--seeds", default="CKFTrackSeeds", help="Seed collection")
parser.add_argument(
    "--allowNoRemoval", action="store_true", help="Pass even if deduplication removed no track"
)
args = parser.parse_args()


def hit_keys(track):
    return tuple(
        (h.getObjectID().collectionID, h.getObjectID().index) for h in track.getTrackerHits()
    )


def summary(track):
    return hit_keys(track), track.getNdf(), track.getChi2()


def same(a, b):
    return a[0] == b[0] and a[1] == b[1] and math.isclose(a[2], b[2], rel_tol=1e-9, abs_tol=1e-9)


base_frames = get_reader(args.baseline).get("events")
dedup_frames = get_reader(args.dedup).get("events")
assert len(base_frames) == len(dedup_frames), (
    f"{len(base_frames)} baseline events, {len(dedup_frames)} with deduplication"
)

n_seeds = n_base = n_dedup = 0
for i, (base, dedup) in enumerate(zip(base_frames, dedup_frames)):
    base_seeds = [hit_keys(s) for s in base.get(args.seeds)]
    dedup_seeds = [hit_keys(s) for s in dedup.get(args.seeds)]
    assert base_seeds == dedup_seeds, (
        f"event {i}: seeds differ ({len(base_seeds)} vs {len(dedup_seeds)})"
    )
    n_seeds += len(base_seeds)

    base_tracks = [summary(t) for t in base.get(args.tracks)]
    dedup_tracks = [summary(t) for t in dedup.get(args.tracks)]
    n_base += len(base_tracks)
    n_dedup += len(dedup_tracks)
    assert len(dedup_tracks) <= len(base_tracks), (
        f"event {i}: {len(dedup_tracks)} tracks with deduplication, {len(base_tracks)} without"
    )

    # Each deduplicated track must be the next matching baseline track.
    j = 0
    for k, track in enumerate(dedup_tracks):
        while j < len(base_tracks) and not same(base_tracks[j], track):
            j += 1
        assert j < len(base_tracks), (
            f"event {i}: deduplicated track {k} ({len(track[0])} hits) is not in the baseline in order"
        )
        j += 1

print(f"{n_seeds} seeds; {n_base} tracks without deduplication, {n_dedup} with")
assert n_base > 0, "No baseline tracks"
assert args.allowNoRemoval or n_dedup < n_base, "Seed deduplication removed no track"
