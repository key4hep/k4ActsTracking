add_library(MLTrackingONNXInferenceModel SHARED
  src/ONNXInferenceModel.cpp
)

target_include_directories(MLTrackingONNXInferenceModel
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(MLTrackingONNXInferenceModel
  PUBLIC
    onnxruntime::onnxruntime
)

install(TARGETS MLTrackingONNXInferenceModel
  EXPORT MLTrackingTargets
  DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

set(sources
    src/ExaTrkGNNTrackFinder.cpp
)

gaudi_add_module(k4RecTrackerTrackFinding
  SOURCES ${sources}
  LINK
    Gaudi::GaudiKernel
    k4FWCore::k4FWCore
    EDM4HEP::edm4hep
    MLTrackingONNXInferenceModel
)

target_include_directories(k4RecTrackerTrackFinding
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
