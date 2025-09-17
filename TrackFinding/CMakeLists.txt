add_library(MLTrackingONNXInferenceModels SHARED
  src/ONNXInferenceModel.cpp
)

target_include_directories(MLTrackingONNXInferenceModels
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(MLTrackingONNXInferenceModels
  PUBLIC
    onnxruntime::onnxruntime
    EDM4HEP::edm4hep
)

install(TARGETS MLTrackingONNXInferenceModels
  EXPORT MLTrackingTargets
  DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

set(sources
    src/ExaTrkGNNTrackFinder.cpp
    src/OnnxMetricLearning.cpp
)

gaudi_add_module(k4RecTrackerTrackFinding
  SOURCES ${sources}
  LINK
    Gaudi::GaudiKernel
    k4FWCore::k4FWCore
    EDM4HEP::edm4hep
    MLTrackingONNXInferenceModels
    k4ActsTracking::k4ActsTracking
    torch
    Acts::PluginGnn
)

target_include_directories(k4RecTrackerTrackFinding
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
