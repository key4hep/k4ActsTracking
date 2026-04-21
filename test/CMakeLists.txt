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

set(test_environment "\
LD_LIBRARY_PATH=\
${PROJECT_BINARY_DIR}:\
${PROJECT_BINARY_DIR}/${PROJECT_NAME}:\
$ENV{LD_LIBRARY_PATH};
PYTHONPATH=\
${PROJECT_BINARY_DIR}/${PROJECT_NAME}/genConfDir:\
$ENV{PYTHONPATH};\
GAUDI_PLUGIN_PATH=\
${PROJECT_BINARY_DIR}/${PROJECT_NAME}:\
$ENV{GAUDI_PLUGIN_PATH}"
)

function(set_test_env _testname)
  set_tests_properties(${_testname} PROPERTIES ENVIRONMENT "${test_environment}")
endfunction()

# add_geometry_load_test(<test-name> <path-to-compact-xml>)
#
# Add a test that loads the passed compact file and converts the geometry into
# an ACTS geometry before dumping it to an .obj file
function(add_geometry_load_test _compact_file)
  get_filename_component(_name "${_compact_file}" NAME_WE)
  add_test(NAME load_geo_${_name}
           WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
           COMMAND k4run ${PROJECT_SOURCE_DIR}/k4ActsTracking/examples/test_visualize_acts_geo.py
                   --compactFile "${_compact_file}"
                   --test-propagation
                 )
  set_test_env(load_geo_${_name})
endfunction()

add_geometry_load_test($ENV{k4geo_DIR}/MuColl/MAIA/compact/MAIA_v0/MAIA_v0.xml)
add_geometry_load_test($ENV{k4geo_DIR}/MuColl/MuSIC/compact/MuSIC_v2/MuSIC_v2.xml)
add_geometry_load_test($ENV{k4geo_DIR}/FCCee/ILD_FCCee/compact/ILD_FCCee_v01/ILD_FCCee_v01.xml)
add_geometry_load_test($ENV{k4geo_DIR}/FCCee/ILD_FCCee/compact/ILD_FCCee_v02/ILD_FCCee_v02.xml)
add_geometry_load_test($ENV{k4geo_DIR}/FCCee/CLD/compact/CLD_o2_v07/CLD_o2_v07.xml)
add_geometry_load_test($ENV{k4geo_DIR}/FCCee/CLD/compact/CLD_o2_v08/CLD_o2_v08.xml)


add_test(NAME sim_MAIA
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  COMMAND ddsim --compactFile=$ENV{k4geo_DIR}/MuColl/MAIA/compact/MAIA_v0/MAIA_v0.xml
                --runType=batch -G -N=10
                --outputFile=particle_gun_MAIA_SIM.edm4hep.root
                --gun.multiplicity 10
                --gun.particle "mu-"
                --gun.distribution uniform
                --gun.thetaMax 2.96  # rougly 170 degrees
                --gun.thetaMin -2.96  # rougly -170 degrees
                --gun.momentumMin 5000
                --gun.momentumMax 500000
)

# Setup test dependencies such that they are run in the right order always
set_tests_properties(
  sim_MAIA
  PROPERTIES
    FIXTURES_SETUP sim_maia_fixture
)

add_test(NAME reco_MAIA
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  COMMAND k4run ${CMAKE_CURRENT_LIST_DIR}/options/MAIA_CKF_Tracking.py
          --compactFile=$ENV{k4geo_DIR}/MuColl/MAIA/compact/MAIA_v0/MAIA_v0.xml
          --trackingGeoFile=${CMAKE_CURRENT_LIST_DIR}/../data/MAIA_v0.root
          --geoDescFile=${CMAKE_CURRENT_LIST_DIR}/../data/MAIA_v0.json
          --materialFile=${CMAKE_CURRENT_LIST_DIR}/../data/MAIA_v0_material.json
)
set_test_env(reco_MAIA)

# Gen3 geometry path: CKFTrackingAlg driven by ActsGeoSvc
add_test(NAME reco_MAIA_Gen3
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  COMMAND k4run ${CMAKE_CURRENT_LIST_DIR}/options/MAIA_CKFTrackingAlg.py
          --compactFile=$ENV{k4geo_DIR}/MuColl/MAIA/compact/MAIA_v0/MAIA_v0.xml
)
set_test_env(reco_MAIA_Gen3)

set_tests_properties(
  reco_MAIA
  reco_MAIA_Gen3

  PROPERTIES
    FIXTURES_REQUIRED sim_maia_fixture
)

add_subdirectory(unittests)
