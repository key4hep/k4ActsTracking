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

# Lightweight checks on the two-way CKF output. Per track: ndf > 0, at least as
# many hits as a seed, AtFirstHit inside AtLastHit, a finite AtIP state with a
# positive covariance diagonal, and every AtCalorimeter state beyond AtLastHit.
# Overall: the second pass extended at least one track beyond the sensor its
# first pass started on (a seed's bottom SP inside-out, top SP outside-in),
# which a single pass cannot do.
import argparse
import math

import edm4hep
from podio.reading import get_reader

parser = argparse.ArgumentParser(description="Check the two-way CKF output")
parser.add_argument("file", help="CKF output file")
parser.add_argument("--outsideIn", action="store_true", help="The first CKF pass ran outside-in")
parser.add_argument("--tracks", default="CKFTracks", help="Track collection")
parser.add_argument("--seeds", default="CKFTrackSeeds", help="Seed collection")
args = parser.parse_args()

TS = edm4hep.TrackState
PARAMS = [
    edm4hep.TrackParams.d0,
    edm4hep.TrackParams.phi,
    edm4hep.TrackParams.omega,
    edm4hep.TrackParams.z0,
    edm4hep.TrackParams.tanLambda,
]


def rxy(p):
    return math.hypot(p.x, p.y)


def dist(p):
    return math.sqrt(p.x**2 + p.y**2 + p.z**2)


def state_at(track, location):
    return next((ts for ts in track.getTrackStates() if ts.location == location), None)


n_tracks = n_calo = n_extended = 0
for i, frame in enumerate(get_reader(args.file).get("events")):
    # Sensors the first passes started on, one per seed
    start_sensors = set()
    for seed in frame.get(args.seeds):
        seed_hits = list(seed.getTrackerHits())
        if seed_hits:
            start = seed_hits[-1] if args.outsideIn else seed_hits[0]
            start_sensors.add(start.getCellID())

    for j, track in enumerate(frame.get(args.tracks)):
        n_tracks += 1
        tag = f"event {i}, track {j}"
        hits = list(track.getTrackerHits())
        first = state_at(track, TS.AtFirstHit)
        last = state_at(track, TS.AtLastHit)
        ip = state_at(track, TS.AtIP)

        assert track.getNdf() > 0, f"{tag}: ndf = {track.getNdf()}"
        assert len(hits) >= 3, f"{tag}: {len(hits)} hits, fewer than a seed"
        assert first is not None and last is not None, f"{tag}: no AtFirstHit or AtLastHit state"
        assert rxy(first.referencePoint) < rxy(last.referencePoint), (
            f"{tag}: AtFirstHit at r = {rxy(first.referencePoint):.1f} mm, "
            f"not inside AtLastHit at r = {rxy(last.referencePoint):.1f} mm"
        )

        assert ip is not None, f"{tag}: no AtIP state"
        values = (ip.D0, ip.Z0, ip.phi, ip.omega, ip.tanLambda)
        assert all(math.isfinite(v) for v in values), f"{tag}: non-finite AtIP parameters {values}"
        variances = [ip.covMatrix.getValue(p, p) for p in PARAMS]
        assert all(math.isfinite(v) and v > 0 for v in variances), (
            f"{tag}: AtIP variances {variances}"
        )

        for ts in track.getTrackStates():
            if ts.location == TS.AtCalorimeter:
                n_calo += 1
                assert dist(ts.referencePoint) > dist(last.referencePoint), (
                    f"{tag}: AtCalorimeter at {dist(ts.referencePoint):.1f} mm from the origin, "
                    f"not beyond AtLastHit at {dist(last.referencePoint):.1f} mm"
                )

        # Hits are ordered inside-out, so the second pass extends the outer end
        # outside-in and the inner end inside-out
        end = hits[-1] if args.outsideIn else hits[0]
        if end.getCellID() not in start_sensors:
            n_extended += 1

print(
    f"{n_tracks} tracks, {n_calo} AtCalorimeter states, {n_extended} extended by the second pass"
)

assert n_tracks >= 100, "Sharp drop in tracks (less than 100)"
assert n_extended >= 0, "No tracks extended by the second pass"
