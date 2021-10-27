CMAKE_MINIMUM_REQUIRED(VERSION 3.18)

project(k4ActsTracking)

find_package(ROOT COMPONENTS RIO Tree)
find_package(EDM4HEP)
find_package(k4FWCore)
find_package(Acts)

#---------------------------------------------------------------
# Load macros and functions for Gaudi-based projects

find_package(Gaudi)
#---------------------------------------------------------------

include(GNUInstallDirs)

# Set up C++ Standard
# ``-DCMAKE_CXX_STANDARD=<standard>`` when invoking CMake
set(CMAKE_CXX_STANDARD 17 CACHE STRING "")

if(NOT CMAKE_CXX_STANDARD MATCHES "14|17|20")
  message(FATAL_ERROR "Unsupported C++ standard: ${CMAKE_CXX_STANDARD}")
endif()

include(CTest)

add_subdirectory(k4ActsTracking)
