

################################################################################
#
# Generate package config files so that the build tree can be referenced by
# other projects.
#
################################################################################

SET(PACKAGE_NAME Rcs)

# Write version file
INCLUDE(CMakePackageConfigHelpers)
WRITE_BASIC_PACKAGE_VERSION_FILE(
  "${CMAKE_CURRENT_BINARY_DIR}/${PACKAGE_NAME}ConfigVersion.cmake"
  VERSION 2.0
  COMPATIBILITY AnyNewerVersion
)

# Write targets file
EXPORT(TARGETS RcsCore RcsPhysics RcsGraphics RcsGui
  FILE "${CMAKE_CURRENT_BINARY_DIR}/${PACKAGE_NAME}Targets.cmake"
#  NAMESPACE Rcs:: # Not really needed, since the targets are prefixed with Rcs anyways
)
# TODO Maybe write to separate target files and use as components?

# Write config file
CONFIGURE_PACKAGE_CONFIG_FILE(
  "cmake/RcsConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/${PACKAGE_NAME}Config.cmake"
  INSTALL_DESTINATION "${CMAKE_INSTALL_DIR}/lib/cmake/${PACKAGE_NAME}"
)
# Also copy over Externals and FindQwt file to provide dependencies
CONFIGURE_FILE(cmake/Externals.cmake
  "${CMAKE_CURRENT_BINARY_DIR}/Externals.cmake"
  COPYONLY
)
CONFIGURE_FILE(cmake/FindQwt.cmake
  "${CMAKE_CURRENT_BINARY_DIR}/FindQwt.cmake"
  COPYONLY
)

# Add to user package registry if desired
OPTION(WRITE_PACKAGE_REGISTRY "Add build tree to user package registry" OFF)
IF(${WRITE_PACKAGE_REGISTRY})
  EXPORT(PACKAGE ${PACKAGE_NAME})
ENDIF()
