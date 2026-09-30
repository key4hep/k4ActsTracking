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

# Validate the output of VertexFindingAlg (test/options/vertexing.py).
#
# Meant for particle-gun samples where all tracks come from the origin: every
# event must have vertices, exactly one of them primary and close to the
# origin, and the vertex -> particle -> track relations and the link weights
# must be consistent with each other.

import argparse
import math
import sys

from podio.reading import get_reader

parser = argparse.ArgumentParser()
parser.add_argument("inputFile", help="edm4hep file written by vertexing.py")
parser.add_argument("--vertices", default="ACTSPrimaryVertices", help="Vertex collection name")
parser.add_argument(
    "--minTrackWeight",
    type=float,
    default=0.1,
    help="MinOutputTrackWeight the vertexing ran with",
)
parser.add_argument(
    "--maxTransverseDistance",
    type=float,
    default=0.2,
    help="Maximum |x| and |y| of the primary vertex w.r.t. the origin [mm]",
)
parser.add_argument(
    "--maxLongitudinalDistance",
    type=float,
    default=1.0,
    help="Maximum |z| of the primary vertex w.r.t. the origin [mm]",
)
args = parser.parse_args()


def object_key(obj):
    object_id = obj.getObjectID()
    return (object_id.collectionID, object_id.index)


errors = []
n_events = 0
for i_event, frame in enumerate(get_reader(args.inputFile).get("events")):
    n_events += 1

    def fail(message):
        errors.append(f"event {i_event}: {message}")

    vertices = frame.get(args.vertices)
    particles = frame.get(f"{args.vertices}_Particles")
    links = frame.get(f"{args.vertices}_ParticleLinks")

    if len(vertices) == 0:
        fail("no vertex found")
        continue

    primaries = [vertex for vertex in vertices if vertex.isPrimary()]
    if len(primaries) != 1:
        fail(f"{len(primaries)} primary vertices instead of 1")

    # Particles of each vertex, and the weight expected on the link to each.
    vertex_particles = {}
    for vertex in vertices:
        position = vertex.getPosition()
        coordinates = (position.x, position.y, position.z, vertex.getChi2())
        if not all(math.isfinite(value) for value in coordinates):
            fail(f"vertex with non-finite position or chi2 {coordinates}")
        keys = set()
        for particle in vertex.getParticles():
            tracks = particle.getTracks()
            if len(tracks) != 1 or not tracks[0].isAvailable():
                fail("vertex particle does not point to exactly one input track")
            keys.add(object_key(particle))
        vertex_particles[object_key(vertex)] = keys

    for vertex in primaries:
        position = vertex.getPosition()
        if (
            abs(position.x) > args.maxTransverseDistance
            or abs(position.y) > args.maxTransverseDistance
            or abs(position.z) > args.maxLongitudinalDistance
        ):
            fail(f"primary vertex at ({position.x}, {position.y}, {position.z}) mm, not at origin")
        if len(vertex.getParticles()) < 2:
            fail(f"primary vertex has {len(vertex.getParticles())} particles")

    all_particle_keys = set().union(*vertex_particles.values())
    if len(all_particle_keys) != len(particles):
        fail(f"{len(particles)} particles, but {len(all_particle_keys)} attached to vertices")
    if len(links) != len(particles):
        fail(f"{len(links)} vertex-particle links for {len(particles)} particles")

    for link in links:
        weight = link.getWeight()
        # The weights are stored as float, so allow for the rounding
        if not (math.isfinite(weight) and args.minTrackWeight - 1e-6 <= weight <= 1.0 + 1e-6):
            fail(f"link weight {weight} outside [{args.minTrackWeight}, 1]")
        vertex_key = object_key(link.getFrom())
        if object_key(link.getTo()) not in vertex_particles.get(vertex_key, set()):
            fail("link to a particle that is not attached to its vertex")

if n_events == 0:
    errors.append("no events in the input file")

for error in errors:
    print(f"ERROR: {error}")
print(f"Checked {n_events} events, {len(errors)} errors")
sys.exit(1 if errors else 0)
