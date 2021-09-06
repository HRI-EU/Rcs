cmake_minimum_required(VERSION 3.4)

PROJECT(RcsVortex)

# Check if we are included as subproject (on old systems) or separate
SET(RCSVORTEX_MASTER_PROJECT OFF)
IF(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  SET(RCSVORTEX_MASTER_PROJECT ON)
  
  IF (NOT CMAKE_BUILD_TYPE)
      SET(CMAKE_BUILD_TYPE "Release" CACHE STRING
	"Choose the type of build, options are: Debug Release." FORCE)
  ENDIF (NOT CMAKE_BUILD_TYPE)
  
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
  FIND_PACKAGE(Vortex REQUIRED)
  FIND_PACKAGE(LibXml2 REQUIRED)

  SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

  # As subproject, we need no special library installation
  SET(RCSVORTEX_INSTALL_LIBDIR "lib")
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
    ${LIBXML2_INCLUDE_DIR}
    ${PTHREAD_INCLUDE_DIR})

IF (WIN32)
  TARGET_LINK_LIBRARIES(RcsVortex RcsCore ${VORTEX_LIBRARIES})
ENDIF()

# We intentionally do not link against Vortex and RcsCore. This allows the
# resulting library to use the version available at runtime. The linker
# needs us to specify vortex libraries explicitly anyways. Thus, a single
# prebuilt binary can be used with any vortex installation.
IF(RCSVORTEX_MASTER_PROJECT)
  TARGET_LINK_LIBRARIES(RcsVortex ${VORTEX_LIBRARIES})
ENDIF()

# Always use C++11
IF (UNIX)
 TARGET_COMPILE_OPTIONS(RcsVortex PRIVATE "-std=c++11")
ENDIF()

# Avoid symbol conflicts
SET_TARGET_PROPERTIES(RcsVortex PROPERTIES CXX_VISIBILITY_PRESET "hidden")

# The install command is used to put the library into Rcs' own library dir.
IF(NOT RCSVORTEX_MASTER_PROJECT)
  #INSTALL(TARGETS RcsVortex DESTINATION ${RCSVORTEX_INSTALL_LIBDIR})
ENDIF()

INSTALL(FILES ${PROJECT_BINARY_DIR}/libRcsVortex.so DESTINATION ${RCSVORTEX_INSTALL_LIBDIR})
