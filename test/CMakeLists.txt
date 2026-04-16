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


function(set_test_env _testname)
  set_property(TEST ${_testname} APPEND PROPERTY ENVIRONMENT "LD_LIBRARY_PATH=${PROJECT_BINARY_DIR}:${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}:$<$<TARGET_EXISTS:ROOT::Core>:$<TARGET_FILE_DIR:ROOT::Core>>:$<$<TARGET_EXISTS:EDM4HEP::edm4hep>:$<TARGET_FILE_DIR:EDM4HEP::edm4hep>>:$<$<TARGET_EXISTS:podio::podio>:$<TARGET_FILE_DIR:podio::podio>>:$ENV{LD_LIBRARY_PATH}")
  set_property(TEST ${_testname} APPEND PROPERTY ENVIRONMENT "PYTHONPATH=${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}/${GAUDI_GENCONF_DIR}:$ENV{PYTHONPATH}")
endfunction()

# add_geometry_load_test(<test-name> <path-to-compact-xml>)
#
# Add a test that loads the passed compact file and converts the geometry into
# an ACTS geometry before dumping it to an .obj file
function(add_geometry_load_test _compact_file)
  get_filename_component(_name "${_compact_file}" NAME_WE)
  add_test(NAME load_geo_${_name}
           WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
           COMMAND k4run ${CMAKE_CURRENT_SOURCE_DIR}/options/load_geometry.py
                   --compactFile "${_compact_file}")
  set_test_env(load_geo_${_name})
endfunction()

add_geometry_load_test($ENV{k4geo_DIR}/MuColl/MAIA/compact/MAIA_v0/MAIA_v0.xml)
add_geometry_load_test($ENV{k4geo_DIR}/MuColl/MuSIC/compact/MuSIC_v2/MuSIC_v2.xml)
add_geometry_load_test($ENV{k4geo_DIR}/FCCee/ILD_FCCee/compact/ILD_FCCee_v01/ILD_FCCee_v01.xml)
add_geometry_load_test($ENV{k4geo_DIR}/FCCee/ILD_FCCee/compact/ILD_FCCee_v02/ILD_FCCee_v02.xml)
add_geometry_load_test($ENV{k4geo_DIR}/FCCee/CLD/compact/CLD_o2_v07/CLD_o2_v07.xml)
add_geometry_load_test($ENV{k4geo_DIR}/FCCee/CLD/compact/CLD_o2_v08/CLD_o2_v08.xml)
