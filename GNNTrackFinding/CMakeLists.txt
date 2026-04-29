add_library(GNNTrackingONNXInferenceModels SHARED
  src/ONNXInferenceModel.cpp
)

target_include_directories(GNNTrackingONNXInferenceModels
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(GNNTrackingONNXInferenceModels
  PUBLIC
    onnxruntime::onnxruntime
    EDM4HEP::edm4hep
)

install(TARGETS GNNTrackingONNXInferenceModels
  EXPORT GNNTrackingTargets
  DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

set(sources
    src/ExaTrkGNNTrackFinder.cpp
    src/OnnxMetricLearning.cpp
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
    Acts::PluginGnn
    ROOT::Physics
)

target_include_directories(GNNTrackingTrackFinding
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
