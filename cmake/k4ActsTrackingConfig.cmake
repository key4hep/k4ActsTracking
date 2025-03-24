include(CMakeFindDependencyMacro)
find_dependency(podio REQUIRED)
find_dependency(k4FWCore REQUIRED)
find_dependency(Acts REQUIRED COMPONENTS Core PluginDD4hep)
find_dependency(Gaudi REQUIRED)

# - Include the targets file to create the imported targets that a client can
# link to (libraries) or execute (programs)
include("${CMAKE_CURRENT_LIST_DIR}/k4ActsTrackingTargets.cmake")

get_property(TEST_K4ACTSTRACKING_LIBRARY TARGET k4ActsTracking::k4ActsTrackingPlugins PROPERTY LOCATION)
find_package_handle_standard_args(k4ActsTracking DEFAULT_MSG CMAKE_CURRENT_LIST_FILE TEST_K4ACTSTRACKING_LIBRARY)
