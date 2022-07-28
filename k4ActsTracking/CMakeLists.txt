file(GLOB _plugin_sources src/components/*.cpp)
gaudi_add_module(k4ActsTrackingPlugins
  SOURCES ${_plugin_sources}
  LINK
  Gaudi::GaudiKernel Gaudi::GaudiAlgLib
  k4FWCore::k4FWCore
  ActsCore ActsPluginDD4hep
  DD4hep::DDCore 
  # DD4hep::DDRec
  # EDM4HEP::edm4hep
  )

set(GAUDI_GENCONF_DIR "genConfDir")

function(set_test_env _testname)
  set_property(TEST ${_testname} APPEND PROPERTY ENVIRONMENT
    LD_LIBRARY_PATH=${CMAKE_BINARY_DIR}:$<TARGET_FILE_DIR:k4ActsTrackingPlugins>:$<TARGET_FILE_DIR:ROOT::Core>:$<TARGET_FILE_DIR:k4FWCore::k4FWCore>:$<TARGET_FILE_DIR:EDM4HEP::edm4hep>:$<TARGET_FILE_DIR:podio::podio>:$ENV{LD_LIBRARY_PATH}
    PYTHONPATH=${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}/${GAUDI_GENCONF_DIR}:$<TARGET_FILE_DIR:k4FWCore::k4FWCore>/../python:$ENV{PYTHONPATH}
    PATH=$<TARGET_FILE_DIR:k4FWCore::k4FWCore>/../bin:$ENV{PATH}
    K4ACTSTRACKING=${CMAKE_CURRENT_LIST_DIR}/
    )
endfunction()

add_test(NAME GeoSvc
         WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
         COMMAND gaudirun.py ${CMAKE_CURRENT_SOURCE_DIR}/python/geosvc.py
                                               )
 set_test_env(GeoSvc)
