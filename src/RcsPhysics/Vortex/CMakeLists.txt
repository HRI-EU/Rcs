cmake_minimum_required(VERSION 2.8)

PROJECT(RcsVortex)

# Check if we are included as subproject (on old systems) or separate
SET(RCSVORTEX_MASTER_PROJECT OFF)
IF(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  SET(RCSVORTEX_MASTER_PROJECT ON)
ENDIF()

IF(RCSVORTEX_MASTER_PROJECT)
  # We need to use gcc 4.8 for this
  FIND_PROGRAM(GCC_48 "gcc-4.8")
  FIND_PROGRAM(GXX_48 "g++-4.8")
  SET(CMAKE_C_COMPILER ${GCC_48})
  SET(CMAKE_CXX_COMPILER ${GXX_48})

  # Use compile settings and dependency management from Rcs proper
  LIST(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../../cmake)
  INCLUDE(Externals)
  INCLUDE(CompileSettings)

  SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

ELSE()
  # As subproject, we need no special library installation
  SET(RCSVORTEX_INSTALL_LIBDIR "lib")
ENDIF()

SET(SRCS
    VortexHelpers.cpp
    VortexSimulation.cpp)

# Define the library
ADD_LIBRARY(RcsVortex SHARED ${SRCS})

# Vortex dependency
TARGET_INCLUDE_DIRECTORIES(RcsVortex SYSTEM PRIVATE ${VORTEX_INCLUDE_DIR})
TARGET_COMPILE_DEFINITIONS(RcsVortex PRIVATE ${VORTEX_DEFINITIONS})
# RcsCore dependencies
TARGET_INCLUDE_DIRECTORIES(RcsVortex PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../RcsCore
    ${LIBXML2_INCLUDE_DIR})

# We intentionally do not link against Vortex and RcsCore. This allows the resulting library to use the version available
# at runtime. The linker needs us to specify vortex libraries explicitly anyways. Thus, a single prebuilt binary can be
# used with any vortex installation.

# Always use C++11
TARGET_COMPILE_OPTIONS(RcsVortex PRIVATE "-std=c++11")
# Avoid symbol conflicts
SET_TARGET_PROPERTIES(RcsVortex PROPERTIES CXX_VISIBILITY_PRESET "hidden")

# The install command is used to put the library into Rcs' own library dir.
INSTALL(TARGETS RcsVortex DESTINATION ${RCSVORTEX_INSTALL_LIBDIR})
