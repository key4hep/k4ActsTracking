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

#--- The genConf directory has been renamed to genConfDir in Gaudi 35r1
#--- See https://gitlab.cern.ch/gaudi/Gaudi/-/merge_requests/1158
set(GAUDI_GENCONF_DIR "genConfDir")
if (${Gaudi_VERSION} VERSION_LESS 35.1)
  set(GAUDI_GENCONF_DIR "genConf")
endif()

function(set_test_env _testname)
  #set_property(TEST ${_testname} APPEND PROPERTY ENVIRONMENT "ROOT_INCLUDE_PATH=$<$<TARGET_EXISTS:podio::podio>:$<TARGET_FILE_DIR:podio::podio>/../include>:$<$<TARGET_EXISTS:EDM4HEP::edm4hep>:$<TARGET_FILE_DIR:EDM4HEP::edm4hep>/../include>:$ENV{ROOT_INCLUDE_PATH}")
  set_property(TEST ${_testname} APPEND PROPERTY ENVIRONMENT "LD_LIBRARY_PATH=${PROJECT_BINARY_DIR}:${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}:$<$<TARGET_EXISTS:ROOT::Core>:$<TARGET_FILE_DIR:ROOT::Core>>:$<$<TARGET_EXISTS:EDM4HEP::edm4hep>:$<TARGET_FILE_DIR:EDM4HEP::edm4hep>>:$<$<TARGET_EXISTS:podio::podio>:$<TARGET_FILE_DIR:podio::podio>>:$ENV{LD_LIBRARY_PATH}")
  set_property(TEST ${_testname} APPEND PROPERTY ENVIRONMENT "PYTHONPATH=${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}/${GAUDI_GENCONF_DIR}:$ENV{PYTHONPATH}")
endfunction()

add_test(NAME LoadODD
         WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
         COMMAND k4run options/geosvc.py)
set_test_env(LoadODD)

add_test(NAME CheckODDObjFile
         WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
         COMMAND test -s MyObjFile.mtl)
set_test_env(CheckODDObjFile)
set_tests_properties(CheckODDObjFile PROPERTIES DEPENDS LoadODD)
