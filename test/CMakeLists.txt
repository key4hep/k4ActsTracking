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

add_subdirectory(unittests)

# Force-load the freshly built Gaudi plugin module.
#
# When the sourced key4hep stack already ships this package (e.g. the nightly
# view contains an older libk4ActsTrackingPlugins.so), Gaudi's PluginService
# resolves plugin libraries relative to where libGaudiPluginService.so lives
# (the view) and ignores LD_LIBRARY_PATH/GAUDI_PLUGIN_PATH for the dlopen. The
# local build is then shadowed by the stale view copy, so components added on
# this branch (e.g. CKFTrackingAlg) are reported as "No factory available".
# LD_PRELOADing the locally built module makes its factories register first;
# PluginService keeps the first registration, so the local components win.
set(test_environment "\
LD_PRELOAD=$<TARGET_FILE:k4ActsTrackingPlugins>;\
LD_LIBRARY_PATH=\
${PROJECT_BINARY_DIR}:\
${PROJECT_BINARY_DIR}/${PROJECT_NAME}:\
$ENV{LD_LIBRARY_PATH};\
PYTHONPATH=\
${CMAKE_CURRENT_SOURCE_DIR}/options:\
${PROJECT_BINARY_DIR}/${PROJECT_NAME}/genConfDir:\
$ENV{PYTHONPATH};\
GAUDI_PLUGIN_PATH=\
${PROJECT_BINARY_DIR}:\
${PROJECT_BINARY_DIR}/${PROJECT_NAME}:\
$ENV{GAUDI_PLUGIN_PATH}"
)

function(set_test_env _testname)
  set_tests_properties(${_testname} PROPERTIES ENVIRONMENT "${test_environment}")
endfunction()

# add_clone_config(URL <git-url>)
#
# Derives the clone directory, test name (clone_<dir>), and fixture name (<dir>)
# from the URL basename. Skips the clone if the directory already exists.
function(add_clone_config)
  cmake_parse_arguments(ARG "" "URL" "" ${ARGN})
  get_filename_component(_dir "${ARG_URL}" NAME_WE)
  set(_cmd "if [ ! -d ${_dir} ]; then git clone ${ARG_URL} --depth 1; fi")
  add_test(NAME clone_${_dir} COMMAND bash -c "${_cmd}")
  set_tests_properties(clone_${_dir} PROPERTIES FIXTURES_SETUP ${_dir})
endfunction()

# add_sim_test(<test-name>
#              DDSIM_EXTRA_ARGS <arg>...
#              [WORKING_DIRECTORY <dir>]
#              [FIXTURES_REQUIRED <fix>...] [FIXTURES_SETUP <fix>])
#
# Runs ddsim with a fixed 10-event mu- particle gun (multiplicity 10, uniform
# distribution). Detector-specific args (--compactFile, --outputFile,
# --gun.momentumMin/Max, etc.) go in DDSIM_EXTRA_ARGS and are forwarded verbatim.
function(add_sim_test _name)
  cmake_parse_arguments(ARG ""
    "WORKING_DIRECTORY;FIXTURES_SETUP"
    "EXTRA_ARGS;FIXTURES_REQUIRED" ${ARGN})
  if(NOT ARG_WORKING_DIRECTORY)
    set(ARG_WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")
  endif()
  add_test(NAME ${_name}
    WORKING_DIRECTORY ${ARG_WORKING_DIRECTORY}
    COMMAND ddsim --runType=batch -G -N=10
                  --gun.multiplicity 10
                  --gun.particle mu-
                  --gun.distribution uniform
                  ${ARG_EXTRA_ARGS}
  )
  if(ARG_FIXTURES_REQUIRED)
    set_tests_properties(${_name} PROPERTIES FIXTURES_REQUIRED "${ARG_FIXTURES_REQUIRED}")
  endif()
  if(ARG_FIXTURES_SETUP)
    set_tests_properties(${_name} PROPERTIES FIXTURES_SETUP "${ARG_FIXTURES_SETUP}")
  endif()
endfunction()

# add_k4run_test(<test-name>
#                OPTIONS_FILE <py>
#                [WORKING_DIRECTORY <dir>]
#                [EXTRA_ARGS <arg>...]
#                [FIXTURES_REQUIRED <fix>...] [FIXTURES_SETUP <fix>]
#                [USE_TEST_ENV])
#
# USE_TEST_ENV applies the build-tree env (needed when the option file loads
# this project's Gaudi plugins).
function(add_k4run_test _name)
  cmake_parse_arguments(ARG "USE_TEST_ENV"
    "OPTIONS_FILE;WORKING_DIRECTORY;FIXTURES_SETUP"
    "EXTRA_ARGS;FIXTURES_REQUIRED" ${ARGN})
  if(NOT ARG_WORKING_DIRECTORY)
    set(ARG_WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")
  endif()
  add_test(NAME ${_name}
    WORKING_DIRECTORY ${ARG_WORKING_DIRECTORY}
    COMMAND k4run ${ARG_OPTIONS_FILE} ${ARG_EXTRA_ARGS}
  )
  if(ARG_USE_TEST_ENV)
    set_test_env(${_name})
  endif()
  if(ARG_FIXTURES_REQUIRED)
    set_tests_properties(${_name} PROPERTIES FIXTURES_REQUIRED "${ARG_FIXTURES_REQUIRED}")
  endif()
  if(ARG_FIXTURES_SETUP)
    set_tests_properties(${_name} PROPERTIES FIXTURES_SETUP "${ARG_FIXTURES_SETUP}")
  endif()
endfunction()

# add_geometry_load_test(<test-name> <path-to-compact-xml>)
#
# Add a test that loads the passed compact file and converts the geometry into
# an ACTS geometry before dumping it to an .obj file
function(add_geometry_load_test _compact_file)
  get_filename_component(_name "${_compact_file}" NAME_WE)
  add_k4run_test(load_geo_${_name}
    OPTIONS_FILE ${PROJECT_SOURCE_DIR}/k4ActsTracking/examples/test_visualize_acts_geo.py
    EXTRA_ARGS --compactFile "${_compact_file}"
               --test-propagation
    USE_TEST_ENV
  )
endfunction()

add_geometry_load_test($ENV{k4geo_DIR}/MuColl/MAIA/compact/MAIA_v0/MAIA_v0.xml)
add_geometry_load_test($ENV{k4geo_DIR}/MuColl/MuSIC/compact/MuSIC_v2/MuSIC_v2.xml)
add_geometry_load_test($ENV{k4geo_DIR}/FCCee/ILD_FCCee/compact/ILD_FCCee_v01/ILD_FCCee_v01.xml)
add_geometry_load_test($ENV{k4geo_DIR}/FCCee/ILD_FCCee/compact/ILD_FCCee_v02/ILD_FCCee_v02.xml)
add_geometry_load_test($ENV{k4geo_DIR}/FCCee/CLD/compact/CLD_o2_v07/CLD_o2_v07.xml)
add_geometry_load_test($ENV{k4geo_DIR}/FCCee/CLD/compact/CLD_o2_v08/CLD_o2_v08.xml)

# The LUXE geometry lives in luxegeo, which is only present in the LUXE software
# stack (ghcr.io/luxesoftware/luxe-sw, see luxe-ci.yml). Guard on luxegeo_DIR so
# these tests are not registered on the key4hep/MuColl stacks where luxegeo is
# absent (they would otherwise fail with an unresolved compact file path).
if(DEFINED ENV{luxegeo_DIR})
  add_geometry_load_test($ENV{luxegeo_DIR}/LUXE/LUXE_v0.xml)
endif()


# === MAIA chain (no upstream config, no standard reco baseline) ===
set(_maia_compact $ENV{k4geo_DIR}/MuColl/MAIA/compact/MAIA_v0/MAIA_v0.xml)

add_sim_test(sim_MAIA
  EXTRA_ARGS
    --compactFile=${_maia_compact}
    --outputFile=particle_gun_MAIA_SIM.edm4hep.root
    --gun.momentumMin 5000 --gun.momentumMax 500000
    --gun.thetaMin -2.96 --gun.thetaMax 2.96  # roughly +/-170 degrees
  FIXTURES_SETUP sim_maia_fixture
)

add_k4run_test(reco_MAIA_Gen3
  OPTIONS_FILE ${CMAKE_CURRENT_LIST_DIR}/options/MAIA_CKFTrackingAlg.py
  EXTRA_ARGS --compactFile=${_maia_compact}
             --IOSvc.Input particle_gun_MAIA_SIM.edm4hep.root
             --IOSvc.Output particle_gun_MAIA_CKFTracks.edm4hep.root
  USE_TEST_ENV
  FIXTURES_REQUIRED sim_maia_fixture
)


# === LUXE chain (self-contained: particle gun + digitization + CKF) ===
#
# LUXE_v0 is a forward "telescope" (planar tracker layers stacked along z plus a
# downstream ECAL slab), so it needs a forward positron gun rather than the 4pi
# mu- gun baked into add_sim_test(). Mirror the luxegeo README example, which
# relies on the dipole field to steer the positrons into the tracker acceptance.
#
# Only registered when luxegeo is available (see the guard note above); on the
# key4hep/MuColl CI stacks luxegeo_DIR is unset and this chain is skipped.
if(DEFINED ENV{luxegeo_DIR})
  set(_luxe_compact $ENV{luxegeo_DIR}/LUXE/LUXE_v0.xml)

  add_test(NAME sim_LUXE_v0
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMAND ddsim --compactFile ${_luxe_compact}
                  --runType batch
                  --numberOfEvents 10
                  --enableGun
                  --gun.particle e+
                  --gun.multiplicity 10
                  # Shoot every positron in the same fixed direction, along the
                  # beam axis (+z) towards the instrumented tracker/ECAL region.
                  # isotrop=False disables the random per-particle directions of
                  # the default gun distribution; the dipole field then steers
                  # the positrons into the tracker acceptance.
                  --gun.direction "0 0 1"
                  --gun.isotrop false
                  # Uniform momentum between 2 and 8 GeV (bare numbers are MeV,
                  # as in the MAIA chain). At these energies E ~= p for the
                  # positrons, so this is effectively a uniform 2-8 GeV energy.
                  --gun.momentumMin 2000
                  --gun.momentumMax 8000
                  --outputFile particle_gun_LUXE_v0_SIM.edm4hep.root
  )
  set_tests_properties(sim_LUXE_v0 PROPERTIES FIXTURES_SETUP sim_luxe_fixture)

  add_k4run_test(reco_LUXE_v0
    OPTIONS_FILE ${CMAKE_CURRENT_LIST_DIR}/options/LUXE_CKFTracking.py
    EXTRA_ARGS --compactFile=${_luxe_compact}
               --IOSvc.Input particle_gun_LUXE_v0_SIM.edm4hep.root
               --IOSvc.Output particle_gun_LUXE_v0_CKFTracks.edm4hep.root
    USE_TEST_ENV
    FIXTURES_REQUIRED sim_luxe_fixture
  )
endif()


# === CLD chain ===
set(_cld_compact $ENV{k4geo_DIR}/FCCee/CLD/compact/CLD_o2_v08/CLD_o2_v08.xml)
set(_cld_dir ${CMAKE_CURRENT_BINARY_DIR}/CLDConfig/CLDConfig)

add_clone_config(URL https://github.com/key4hep/CLDConfig.git)

add_sim_test(sim_CLD_o2_v08
  WORKING_DIRECTORY ${_cld_dir}
  EXTRA_ARGS
    --compactFile=${_cld_compact}
    --outputFile=particle_gun_CLD_o2_v08_SIM.edm4hep.root
    --gun.momentumMin 1500 --gun.momentumMax 100000
  FIXTURES_REQUIRED CLDConfig
  FIXTURES_SETUP sim_CLD_fixture
)

add_k4run_test(std_reco_CLD_o2_v08
  WORKING_DIRECTORY ${_cld_dir}
  OPTIONS_FILE CLDReconstruction.py
  EXTRA_ARGS --compactFile ${_cld_compact}
             --inputFiles=particle_gun_CLD_o2_v08_SIM.edm4hep.root
             --trackingOnly
             --outputBasename particle_gun_CLD_o2_v08
  FIXTURES_REQUIRED sim_CLD_fixture
  FIXTURES_SETUP reco_CLD_fixture
)

add_k4run_test(ckf_tracking_CLD_o2_v08
  WORKING_DIRECTORY ${_cld_dir}
  OPTIONS_FILE ${CMAKE_CURRENT_LIST_DIR}/options/CLD_CKFTracking.py
  EXTRA_ARGS --compactFile ${_cld_compact}
             --IOSvc.Input particle_gun_CLD_o2_v08_REC.edm4hep.root
             --IOSvc.Output particle_gun_CLD_o2_v08_CKFTracks.edm4hep.root
  USE_TEST_ENV
  FIXTURES_REQUIRED reco_CLD_fixture
)


# === ILD chain ===
set(_ild_compact $ENV{k4geo_DIR}/FCCee/ILD_FCCee/compact/ILD_FCCee_v01/ILD_FCCee_v01.xml)
set(_ild_dir ${CMAKE_CURRENT_BINARY_DIR}/ILDConfig/StandardConfig/production)

add_clone_config(URL https://github.com/iLCSoft/ILDConfig.git)

add_sim_test(sim_ILD_FCCee_v01
  WORKING_DIRECTORY ${_ild_dir}
  EXTRA_ARGS
    --compactFile=${_ild_compact}
    --outputFile=particle_gun_ILD_FCCee_v01_SIM.edm4hep.root
    --gun.momentumMin 1500 --gun.momentumMax 1000000
  FIXTURES_REQUIRED ILDConfig
  FIXTURES_SETUP sim_ILD_FCCee_v01_fixture
)

add_k4run_test(std_reco_ILD_FCCee_v01
  WORKING_DIRECTORY ${_ild_dir}
  OPTIONS_FILE ILDReconstruction.py
  EXTRA_ARGS --compactFile ${_ild_compact}
             --inputFiles particle_gun_ILD_FCCee_v01_SIM.edm4hep.root
             --trackingOnly
             --outputFileBase particle_gun_ILD_FCCee_v01
  FIXTURES_REQUIRED sim_ILD_FCCee_v01_fixture
  FIXTURES_SETUP reco_ILD_fixture
  USE_TEST_ENV
)

add_k4run_test(ckf_tracking_ILD_FCCee_v01
  WORKING_DIRECTORY ${_ild_dir}
  OPTIONS_FILE ${CMAKE_CURRENT_LIST_DIR}/options/ILD_CKFTracking.py
  EXTRA_ARGS --compactFile ${_ild_compact}
             --IOSvc.Input particle_gun_ILD_FCCee_v01_REC.edm4hep.root
             --IOSvc.Output particle_gun_ILD_FCCee_v01_CKFTracks.edm4hep.root
  USE_TEST_ENV
  FIXTURES_REQUIRED reco_ILD_fixture
)

# Explicitly add the ILD working directory to PYTHONPATH for ILD tests.
#
# This APPENDs a second PYTHONPATH= entry to the test ENVIRONMENT; CTest applies
# env entries in order, so a later duplicate of the same variable OVERRIDES the
# earlier one set by USE_TEST_ENV. We must therefore keep the local genConfDir
# (and options) ahead of the inherited $ENV{PYTHONPATH} here too — otherwise the
# CVMFS view's k4ActsTrackingPluginsConf (which lacks CKFTrackingAlg) shadows the
# local configurable and k4run fails with "Configurable CKFTrackingAlg not found".
set(_ild_pythonpath "\
${_ild_dir}:\
${CMAKE_CURRENT_SOURCE_DIR}/options:\
${PROJECT_BINARY_DIR}/${PROJECT_NAME}/genConfDir:\
$ENV{PYTHONPATH}")
set_property(TEST std_reco_ILD_FCCee_v01 APPEND PROPERTY
  ENVIRONMENT "PYTHONPATH=${_ild_pythonpath}")
set_property(TEST ckf_tracking_ILD_FCCee_v01 APPEND PROPERTY
  ENVIRONMENT "PYTHONPATH=${_ild_pythonpath}")
