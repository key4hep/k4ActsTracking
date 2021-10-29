file(GLOB _plugin_sources src/components/*.cpp)
gaudi_add_module(k4ActsTrackingPlugins
  SOURCES ${_plugin_sources}
  LINK Gaudi::GaudiKernel Gaudi::GaudiAlgLib DD4hep::DDCore)



file(GLOB _dd4hep_plugin_sources src/dd4hepplugin/*.cpp)
add_dd4hep_plugin(k4ActsDD4hepPlugin SHARED ${_dd4hep_plugin_sources})
target_link_libraries(k4ActsDD4hepPlugin DD4hep::DDCore DD4hep::DDRec DD4hep::DDG4 DD4hep::DDParsers)


SET(compile_flags -Wformat-security -Wno-long-long -Wdeprecated -fdiagnostics-color=auto -Wall -Wextra -pedantic)

target_compile_options(k4ActsTrackingPlugins PUBLIC ${compile_flags})
target_compile_options(k4ActsDD4hepPlugin PUBLIC  ${compile_flags})


function(set_test_env _testname)
  set_property(TEST ${_testname} APPEND PROPERTY ENVIRONMENT
    LD_LIBRARY_PATH=${CMAKE_BINARY_DIR}:$<TARGET_FILE_DIR:k4ActsTrackingPlugins>:$<TARGET_FILE_DIR:k4ActsDD4hepPlugin>:$<TARGET_FILE_DIR:ROOT::Core>:$<TARGET_FILE_DIR:k4FWCore::k4FWCore>:$<TARGET_FILE_DIR:EDM4HEP::edm4hep>:$<TARGET_FILE_DIR:podio::podio>:$ENV{LD_LIBRARY_PATH}
    PYTHONPATH=${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}/genConf:$<TARGET_FILE_DIR:k4FWCore::k4FWCore>/../python:$ENV{PYTHONPATH}
    PATH=$<TARGET_FILE_DIR:k4FWCore::k4FWCore>/../bin:$ENV{PATH}
    K4ACTSTRACKING=${CMAKE_CURRENT_LIST_DIR}/
    )
endfunction()

add_test(NAME ActsAlgTest
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  COMMAND k4run k4ActsTracking/options/run_acts_plugin.py)
set_test_env(ActsAlgTest)
