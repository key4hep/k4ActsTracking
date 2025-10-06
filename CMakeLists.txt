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
CMAKE_MINIMUM_REQUIRED(VERSION 3.18)

project(k4ActsTracking LANGUAGES CXX)

SET( ${PROJECT_NAME}_VERSION_MAJOR 0 )
SET( ${PROJECT_NAME}_VERSION_MINOR 1 )
SET( ${PROJECT_NAME}_VERSION_PATCH 0 )

SET( ${PROJECT_NAME}_VERSION  "${${PROJECT_NAME}_VERSION_MAJOR}.${${PROJECT_NAME}_VERSION_MINOR}.${${PROJECT_NAME}_VERSION_PATCH}" )

find_package(k4FWCore REQUIRED)
find_package(Acts REQUIRED COMPONENTS Core PluginDD4hep)
find_package(DD4hep REQUIRED COMPONENTS DDCore DDRec)


#---------------------------------------------------------------

include(GNUInstallDirs)

# Set up C++ Standard
# ``-DCMAKE_CXX_STANDARD=<standard>`` when invoking CMake
set(CMAKE_CXX_STANDARD 20 CACHE STRING "")

if(NOT CMAKE_CXX_STANDARD MATCHES "20|23")
  message(FATAL_ERROR "Unsupported C++ standard: ${CMAKE_CXX_STANDARD} (use 20 or 23)")
endif()

include(CTest)

add_subdirectory(k4ActsTracking)
if (BUILD_TESTING)
  add_subdirectory(test)
endif()

include(cmake/createk4ActsTrackingConfig.cmake)
