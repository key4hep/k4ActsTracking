#[[
Copyright (c) 2014-2024 Key4hep-Project.

This file is part of Key4hep.
See https://key4hep.github.io/key4hep-doc/ for further info.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
]]
add_library(GNNTrackingONNXInferenceModels SHARED
  src/ONNXInferenceModel.cpp
)

target_include_directories(GNNTrackingONNXInferenceModels
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(GNNTrackingONNXInferenceModels
  PUBLIC
    onnxruntime::onnxruntime
    EDM4HEP::edm4hep
)

# NOTE: This is an internal helper library of the GNN track finding module and
# is not part of the exported k4ActsTracking package, it is only installed
# because the Gaudi module links against it.
install(TARGETS GNNTrackingONNXInferenceModels
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
)

set(sources
    src/CCAndWalkTrackBuilding.cpp
    src/ClassifiedEdgePrinting.cpp
    src/GNNTrackFinder.cpp
    src/OnnxMetricLearning.cpp
    src/PaddedEdgeRemoval.cpp
)

gaudi_add_module(GNNTrackingTrackFinding
  SOURCES ${sources}
  LINK
    Gaudi::GaudiKernel
    k4FWCore::k4FWCore
    EDM4HEP::edm4hep
    GNNTrackingONNXInferenceModels
    k4ActsTracking::k4ActsTracking
    torch
    Acts::Core
    Acts::PluginGnn
    TBB::tbb
    ROOT::Physics
)
# NOTE: the GNNTrackFinding/include directory comes in via the usage
# requirements of GNNTrackingONNXInferenceModels
