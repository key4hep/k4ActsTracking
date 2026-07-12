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

set(ACTS_LIBRARY_TARGETS "Acts::Core")
gaudi_add_library(k4ActsTracking
  SOURCES
    src/ActsGaudiLogger.cpp
    src/CellIDSelector.cxx
    src/components/Helpers.cxx
  LINK
    DD4hep::DDCore
    EDM4HEP::edm4hep
    k4FWCore::k4FWCore
    ${ACTS_LIBRARY_TARGETS}
)
install(TARGETS k4ActsTracking
  EXPORT k4ActsTrackingTargets
  INCLUDES DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
)

set(_plugin_sources
  src/components/ACTSAlgBase.cxx
  src/components/ACTSDuplicateRemoval.cxx
  src/components/ACTSSeededCKFTrackingAlg.cxx
  src/components/ActsGeoSvc.cpp
  src/components/FilterTracksAlg.cxx
  src/components/GeometryIdMappingTool.cxx
  src/components/GeometryIdSelector.cxx
  src/components/TrackTruthAlg.cxx
  src/components/ActsTestPropagator.cpp
  src/components/DD4hepBlueprintConstruction.cpp
  src/components/CKFTrackingAlg.cpp
  src/components/CKFTrackingFromSeedsAlg.cpp
)

gaudi_add_module(k4ActsTrackingPlugins
  SOURCES ${_plugin_sources}
  LINK
  Gaudi::GaudiKernel
  EDM4HEP::edm4hep
  k4FWCore::k4FWCore
  k4FWCore::k4Interface
  DD4hep::DDCore DD4hep::DDRec
  k4ActsTracking
  Acts::Core
  Acts::PluginDD4hep
  Acts::PluginJson
  Acts::PluginRoot
)
target_include_directories(k4ActsTrackingPlugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

install(TARGETS k4ActsTrackingPlugins
  EXPORT k4ActsTrackingTargets
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
  COMPONENT dev)

set(GAUDI_GENCONF_DIR "genConfDir")
set_property(TARGET k4ActsTrackingPlugins PROPERTY GAUDI_GENCONF TRUE)
