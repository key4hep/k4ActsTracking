set(CMAKE_INSTALL_CMAKEDIR "${CMAKE_INSTALL_LIBDIR}/cmake")

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
  ${PROJECT_BINARY_DIR}/k4ActsTrackingVersion.cmake
  VERSION ${PROJECT_NAME}_VERSION
  COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
  ${PROJECT_SOURCE_DIR}/cmake/k4ActsTrackingConfig.cmake.in
  ${PROJECT_BINARY_DIR}/k4ActsTrackingConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_CMAKEDIR}/${PROJECT_NAME}
)

install(FILES
  ${PROJECT_BINARY_DIR}/k4ActsTrackingConfig.cmake
  ${PROJECT_BINARY_DIR}/k4ActsTrackingVersion.cmake
  DESTINATION ${CMAKE_INSTALL_CMAKEDIR}/${PROJECT_NAME}
)

install(EXPORT ${PROJECT_NAME}Targets
  NAMESPACE ${PROJECT_NAME}::
  FILE "${PROJECT_NAME}Targets.cmake"
  DESTINATION ${CMAKE_INSTALL_CMAKEDIR}/${PROJECT_NAME}
)
