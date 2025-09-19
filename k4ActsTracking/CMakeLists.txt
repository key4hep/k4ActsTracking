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
if (Acts_VERSION VERSION_LESS 43.0.0)
  set(ACTS_LIBRARY_TARGETS "ActsCore")
endif()

gaudi_add_library(k4ActsTracking
  SOURCES
    src/ActsGaudiLogger.cpp
  LINK
    k4FWCore::k4FWCore
    ${ACTS_LIBRARY_TARGETS}
)
install(TARGETS k4ActsTracking
  EXPORT k4ActsTrackingTargets
  INCLUDES DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
)

file(GLOB _plugin_sources src/components/*.c*)
set(ACTS_PLUGIN_TARGETS "Acts::Core;Acts::PluginDD4hep;Acts::PluginJson;Acts::PluginRoot")
# Before namespaced targets we have to do a bit more work
if (Acts_VERSION VERSION_LESS 43.0.0)
  set(ACTS_PLUGIN_TARGETS "ActsCore;ActsPluginDD4hep;ActsPluginJson")
  if (Acts_VERSION VERSION_LESS 42.0.0)
    list(APPEND ACTS_PLUGIN_TARGETS "ActsPluginTGeo")
  else()
    list(APPEND ACTS_PLUGIN_TARGETS "ActsPluginRoot")
  endif()
endif()

gaudi_add_module(k4ActsTrackingPlugins
  SOURCES ${_plugin_sources}
  LINK
  Gaudi::GaudiKernel
  k4FWCore::k4FWCore
  EDM4HEP::edm4hep
  DD4hep::DDCore DD4hep::DDRec
  ${ACTS_PLUGIN_TARGETS}
)
target_include_directories(k4ActsTrackingPlugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
if (Acts_VERSION VERSION_LESS 42.0.0)
  target_compile_definitions(k4ActsTrackingPlugins PUBLIC K4ACTSTRACKING_ACTS_HAS_TGEO_PLUGIN)
endif()

if (Acts_VERSION VERSION_LESS 33.0.0)
  target_compile_definitions(k4ActsTrackingPlugins PRIVATE K4ACTSTRACKING_ACTS_V32)
endif()

install(TARGETS k4ActsTrackingPlugins
  EXPORT k4ActsTrackingTargets
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
  COMPONENT dev)

set(GAUDI_GENCONF_DIR "genConfDir")

function(set_test_env _testname)
  set_property(TEST ${_testname} APPEND PROPERTY ENVIRONMENT
    LD_LIBRARY_PATH=${CMAKE_BINARY_DIR}:$<TARGET_FILE_DIR:k4ActsTrackingPlugins>:$<TARGET_FILE_DIR:ROOT::Core>:$<TARGET_FILE_DIR:k4FWCore::k4FWCore>:$<TARGET_FILE_DIR:EDM4HEP::edm4hep>:$<TARGET_FILE_DIR:podio::podio>:$ENV{LD_LIBRARY_PATH}
    PYTHONPATH=${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}/${GAUDI_GENCONF_DIR}:$<TARGET_FILE_DIR:k4FWCore::k4FWCore>/../python:$ENV{PYTHONPATH}
    PATH=$<TARGET_FILE_DIR:k4FWCore::k4FWCore>/../bin:$ENV{PATH}
    K4ACTSTRACKING=${CMAKE_CURRENT_LIST_DIR}/
    )
endfunction()
