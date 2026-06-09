# The setup below allows optionally building ACTS directly with the k4ActsTracking build.
# This avoids having to run separate build and install steps, and also allows getting a 
# combined compilation database out.
if(IS_DIRECTORY ${ACTS_SOURCE_DIR})
  message(STATUS "Using ACTS from: ${ACTS_SOURCE_DIR}")

  include(FetchContent)

  # Build configuration needed for k4ActsTracking.
  set(ACTS_USE_SYSTEM_NLOHMANN_JSON ON CACHE BOOL "Use system json")
  set(ACTS_BUILD_PLUGIN_JSON ON CACHE BOOL "Build json plugin")
  set(ACTS_BUILD_PLUGIN_ROOT ON CACHE BOOL "Build root plugin")
  set(ACTS_BUILD_PLUGIN_DD4HEP ON CACHE BOOL "Build dd4hep plugin")


  set(FETCHCONTENT_SOURCE_DIR_ACTS "${ACTS_SOURCE_DIR}" CACHE PATH "")

  # This makes `find_package(Acts)` transparently defer to `FetchContent`
  # to being the ACTS build in.
  FetchContent_Declare(Acts
    SYSTEM
    OVERRIDE_FIND_PACKAGE
  )
endif()
