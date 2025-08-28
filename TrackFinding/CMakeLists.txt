set(sources
    src/ExaTrkGNNTrackFinder.cpp
    src/ONNXInferenceModel.cpp
)

gaudi_add_module(k4RecTrackerTrackFinding
  SOURCES ${sources}
  LINK
    Gaudi::GaudiKernel
    k4FWCore::k4FWCore
    EDM4HEP::edm4hep
    onnxruntime::onnxruntime
)

target_include_directories(k4RecTrackerTrackFinding
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
