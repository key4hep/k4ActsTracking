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

from Gaudi.Configuration import INFO
from Gaudi.Configurables import EventDataSvc, GeoSvc, VertexFindingAlg
from k4FWCore import IOSvc
from k4FWCore.parseArgs import parser


def make_vertexing(**properties):
    """Set up ACTS primary vertexing on an existing track collection.

    Shared by the generic vertexing.py and the per-detector option files: adds
    the --compactFile (only its magnetic field is used), --inputTracks and
    --outputVertices arguments, keeps every input collection in the output, and
    returns the services and the VertexFindingAlg. Keyword arguments are passed
    on as VertexFindingAlg properties; properties given on the k4run command
    line (--VertexFindingAlg.<Property>) still override them.
    """
    parser.add_argument(
        "--compactFile",
        help="The geometry compact file to use (for the magnetic field)",
        type=str,
        required=True,
    )
    parser.add_argument(
        "--inputTracks",
        help="Name of the input track collection to run vertexing on",
        type=str,
        default="SiTracks_Refitted",
    )
    parser.add_argument(
        "--outputVertices",
        help="Name of the output vertex collection",
        type=str,
        default="ACTSPrimaryVertices",
    )
    args = parser.parse_known_args()[0]

    iosvc = IOSvc("IOSvc")
    # Keep the input collections in the output file, so the vertices can be
    # compared with the tracks and with other vertex collections side by side.
    iosvc.outputCommands = ["keep *"]

    services = [
        GeoSvc("GeoSvc", detectors=[args.compactFile], EnableGeant4Geo=False),
        EventDataSvc("EventDataSvc"),
    ]

    vertexing = VertexFindingAlg(
        "VertexFindingAlg",
        InputTracks=[args.inputTracks],
        OutputVertices=[args.outputVertices],
        # One particle per track used by a vertex, and the vertex -> particle
        # links carrying the adaptive fit's track weights
        OutputParticles=[f"{args.outputVertices}_Particles"],
        OutputVertexParticleLinks=[f"{args.outputVertices}_ParticleLinks"],
        OutputLevel=INFO,
        **properties,
    )
    return services, vertexing
