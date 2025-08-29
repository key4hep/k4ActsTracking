set(CATCH2_MIN_VERSION 3.5.0)
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
endif()

add_executable(unittests_mltracking unittests.cpp)

target_link_libraries(unittests_mltracking PRIVATE Catch2::Catch2WithMain MLTrackingONNXInferenceModels)
include(Catch)
catch_discover_tests(unittests_mltracking)

add_executable(embedding_model_inference embedding_model_inference.cpp)
target_link_libraries(embedding_model_inference PRIVATE podio::podioIO EDM4HEP::edm4hep MLTrackingONNXInferenceModels)

add_test(NAME validate_embedding_model COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/validate_embedding_model.sh)
set_tests_properties(validate_embedding_model
  PROPERTIES
    ENVIRONMENT
    "TEST_BASE_DIR=${CMAKE_CURRENT_SOURCE_DIR};MODEL_DIR=${PROJECT_SOURCE_DIR}/models"
)
