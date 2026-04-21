set(CATCH2_MIN_VERSION 3.5.0)

set(USE_EXTERNAL_CATCH2 AUTO CACHE STRING "Link against an external Catch2 v3 static library, otherwise build it locally")
set_property(CACHE USE_EXTERNAL_CATCH2 PROPERTY STRINGS AUTO ON OFF)

if(USE_EXTERNAL_CATCH2)
  if (USE_EXTERNAL_CATCH2 STREQUAL AUTO)
    find_package(Catch2 ${CATCH2_MIN_VERSION})
  else()
    find_package(Catch2 ${CATCH2_MIN_VERSION} REQUIRED)
  endif()
endif()

if(NOT Catch2_FOUND)
  message(STATUS "Fetching local copy of Catch2 library for unit-tests...")
  # Build Catch2 with the default flags, to avoid generating warnings when we
  # build it
  set(CXX_FLAGS_CMAKE_USED ${CMAKE_CXX_FLAGS})
  set(CMAKE_CXX_FLAGS ${CXX_FLAGS_CMAKE_DEFAULTS})
  Include(FetchContent)
  FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v${CATCH2_MIN_VERSION}
    )
  FetchContent_MakeAvailable(Catch2)
  set(CMAKE_MODULE_PATH ${Catch2_SOURCE_DIR}/extras ${CMAKE_MODULE_PATH})

  # Disable clang-tidy on external contents
  set_target_properties(Catch2 PROPERTIES CXX_CLANG_TIDY "")

  # Hack around the fact, that the include directories are not declared as
  # SYSTEM for the targets defined this way. Otherwise warnings can still occur
  # in Catch2 code when templates are evaluated (which happens quite a bit)
  get_target_property(CATCH2_IF_INC_DIRS Catch2 INTERFACE_INCLUDE_DIRECTORIES)
  set_target_properties(Catch2 PROPERTIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${CATCH2_IF_INC_DIRS}")

  # Reset the flags
  set(CMAKE_CXX_FLAGS ${CXX_FLAGS_CMAKE_USED})
else()
  # Same hack as above for the system-installed Catch2: mark its include
  # directories as SYSTEM to suppress warnings from Catch2 headers
  get_target_property(CATCH2_IF_INC_DIRS Catch2::Catch2 INTERFACE_INCLUDE_DIRECTORIES)
  set_target_properties(Catch2::Catch2 PROPERTIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${CATCH2_IF_INC_DIRS}")
endif()

add_executable(unittest_k4ActsTracking CellIDSelectionTests.cpp)
target_link_libraries(unittest_k4ActsTracking PUBLIC k4ActsTracking PRIVATE Catch2::Catch2WithMain)
# Catch2 TEST_CASE/SECTION macros expand __COUNTER__ at the call site in user
# code, so marking Catch2 headers as system includes is not sufficient to
# suppress this warning, present with Clang 22
target_compile_options(unittest_k4ActsTracking PRIVATE -Wno-c2y-extensions)

include(Catch)
catch_discover_tests(unittest_k4ActsTracking
  WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
  TEST_PREFIX "UT_" # make it possible to filter easily with -R ^UT
)
