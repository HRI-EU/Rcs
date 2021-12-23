CMAKE_MINIMUM_REQUIRED(VERSION 3.2)
SET(BST_INSTALL_CATEGORY Libraries) 

PROJECT(Rcs)

################################################################################
# Check if project is stand-alone or inside tree with dependencies
################################################################################
IF(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    SET(CMAKE_INSTALL_PREFIX "$ENV{SIT}/Libraries/Rcs/1.0" CACHE PATH "Default sand box directory" FORCE)
  ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  
  MESSAGE(STATUS "Compiling Rcs as stand-alone project")
  SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
  SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
  STRING(LENGTH "${CMAKE_SOURCE_DIR}/" RCS_BASE_PATH_LENGTH)
  ADD_DEFINITIONS("-DRCS_BASE_PATH_LENGTH=${RCS_BASE_PATH_LENGTH}")
ENDIF()

IF (NOT CMAKE_BUILD_TYPE)
  IF (WIN32)
    SET(CMAKE_BUILD_TYPE "Release" CACHE STRING
       "Choose the type of build, options are: Debug Release." FORCE)
  ELSE()
    SET(CMAKE_BUILD_TYPE "Release" CACHE STRING 
       "Choose the type of build, options are: Debug Release." FORCE)
  ENDIF()
ENDIF (NOT CMAKE_BUILD_TYPE)

IF(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
  SET(RCS_DEBUG_SUFFIX d)
ENDIF()

SET(RCS_THIRDPARTY_DIR ${PROJECT_SOURCE_DIR}/external)
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
SET(LIBRARY_LINKING_TYPE "SHARED")
SITE_NAME(HOSTNAME)

MESSAGE(STATUS "Using cmake ${CMAKE_VERSION}")
MESSAGE(STATUS "Compiler is: ${CMAKE_CXX_COMPILER_ID}")

################################################################################
#
# Copy config directories:
#   - Symlink for Linux
#   - Deep copy of the config directory for windows
#
# This needs to go before including the sub directories, since they 
# partially create links /copies inside the config directory
#
# When unpacking the linux source under windows, symbolic links are
# interpreted as files. In this case, we delete the file, otherwise there
# will be no copying dones.
#
################################################################################
SET(CFG_FROM ${CMAKE_CURRENT_SOURCE_DIR}/config)
SET(CFG_TO ${PROJECT_BINARY_DIR}/config)

IF (WIN32 AND (NOT IS_DIRECTORY "${CFG_TO}") AND (EXISTS "${CFG_TO}"))
  MESSAGE(STATUS "Removing non-directory ${CFG_TO}")
  FILE(REMOVE ${CFG_TO})
ENDIF()

IF(NOT EXISTS "${CFG_TO}")
  IF(UNIX)
    SET(CONFIGDIR_COPY_COMMAND create_symlink)
    MESSAGE(STATUS "Linking config directory from ${CFG_FROM} to ${CFG_TO}")
  ELSE()
    SET(CONFIGDIR_COPY_COMMAND copy_directory)
    MESSAGE(STATUS "Copying config directory from ${CFG_FROM} to ${CFG_TO}")
  ENDIF()

  EXECUTE_PROCESS (COMMAND
   ${CMAKE_COMMAND} -E ${CONFIGDIR_COPY_COMMAND} ${CFG_FROM} ${CFG_TO})
ELSE()
  MESSAGE(STATUS "Config directory ${CFG_TO} already exists - skipping")
ENDIF()

################################################################################
#
# Default options
#
################################################################################
OPTION(HEADLESS_BUILD "Build without graphics and Guis" OFF)
OPTION(USE_EIGEN3 "Use Eigen3 in RcsCore" OFF)
OPTION(ENABLE_DEBUG_MACROS "Enable debug macros also in release mode" ON)
OPTION(BUILD_RCS_LIBRARIES_ONLY "Build only libs, but not examples and tests" OFF)

IF(UNIX)
  OPTION(ENABLE_C++11 "Compile with C++11 support enabled" OFF)
ENDIF()

FIND_PACKAGE(WM5 QUIET)
IF (WM5_FOUND)
  OPTION(USE_WM5 "Use Wildmagic library" ON)
ELSE()
  OPTION(USE_WM5 "Use Wildmagic library" OFF)
ENDIF()

FIND_PACKAGE(Vortex QUIET)
IF (VORTEX_FOUND)
  OPTION(USE_VORTEX "Use the Vortex physics library" ON)
ELSE()
  OPTION(USE_VORTEX "Use the Vortex physics library" OFF)
ENDIF()

IF(UNIX)
  FIND_PACKAGE(Bullet QUIET)
ENDIF()
  
IF(BULLET_FOUND)
  SET(USE_BULLET 2.83_float CACHE STRING "Bullet version to use")
ELSE()
  SET(USE_BULLET OFF CACHE STRING "Bullet version to use")
ENDIF()
SET_PROPERTY(CACHE USE_BULLET PROPERTY STRINGS 2.89_double 2.83_float 2.83_double OFF)

################################################################################
#
# Dependency settings and compile settings. These must go after Externals.cmake
#
################################################################################
INCLUDE(Externals)
INCLUDE(CompileSettings)

IF (WIN32)
  ADD_DEFINITIONS(-D_USE_MATH_DEFINES)   # M_PI etc. for windows
ENDIF(WIN32)

SET(ARCH ${CMAKE_SYSTEM_NAME})

IF(${CMAKE_BUILD_TYPE} STREQUAL "Release" AND NOT ENABLE_DEBUG_MACROS)
  ADD_DEFINITIONS(-DRCS_NO_DEBUG)
  MESSAGE(STATUS "Compiling in RELEASE mode, debug macros DISABLED")
ENDIF()

################################################################################
#
# Add the automatically determined parts of the RPATH which point to directories
# outside the build tree to the install RPATH. Required for Vortex and other
# dependencies from the SIT.
# Note: This only works with CMake >= 3.0. With 2.8, the library directories of
# the dependencies must be added to the LD_LIBRARY_PATH manually in order to
# use the installed executables. This has no effect on using the package in
# another CMake project.
#
################################################################################
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

################################################################################
#
# The RPATH to be used when installing, but only if it's not a system directory
# See CMake wiki:
# (https://gitlab.kitware.com/cmake/community/wikis/doc/cmake/RPATH-handling)
#
################################################################################
LIST(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
IF("${isSystemDir}" STREQUAL "-1")
   SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
ENDIF("${isSystemDir}" STREQUAL "-1")

################################################################################
#
# Setup exported target collection
#
################################################################################
# Setup RCS_EXPORT_LIBRARIES so the component CMakeLists.txt can modify it
SET(RCS_EXPORT_LIBRARIES "" CACHE INTERNAL "")
# Create a macro for adding an export library
FUNCTION(RCS_ADD_EXPORT_LIBRARY library)
  SET(RCS_EXPORT_LIBRARIES ${RCS_EXPORT_LIBRARIES} ${library} CACHE INTERNAL "")
ENDFUNCTION()

################################################################################
#
# If a user-file is in the cmake directory, it will be included.
#
################################################################################
SET(USER_LIST "${PROJECT_SOURCE_DIR}/cmake/CMakeLists-$ENV{USER}.txt")

IF(EXISTS ${USER_LIST})
  MESSAGE(STATUS "Including " ${USER_LIST})
  INCLUDE(${USER_LIST})
ELSE()
  MESSAGE(STATUS "Not found: " ${USER_LIST})
ENDIF()

################################################################################
#
# Add source code directories
#
################################################################################
ADD_SUBDIRECTORY (src)

IF (NOT BUILD_RCS_LIBRARIES_ONLY)
  ADD_SUBDIRECTORY (bin)
  ADD_SUBDIRECTORY (examples)
ENDIF()

IF(EXISTS "${PROJECT_SOURCE_DIR}/projects")
  MESSAGE(STATUS "Including projects directory" )
  ADD_SUBDIRECTORY (projects)
ENDIF()

ADD_SUBDIRECTORY (doc)

################################################################################
#
# Configure unit test script
#
################################################################################
IF(UNIX)
  SET(TEST_EXE_DIR "${EXECUTABLE_OUTPUT_PATH}")
  SET(TESTSCRIPT_NAME "${TEST_EXE_DIR}/rcs_unittest.sh")
ELSEIF(WIN32)
  SET(TEST_EXE_DIR "${EXECUTABLE_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}")
  SET(TESTSCRIPT_NAME "${TEST_EXE_DIR}/rcs_unittest.bat")
ENDIF()

IF(UNIX)
  CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/unittest.in ${TESTSCRIPT_NAME} @ONLY)
  ADD_CUSTOM_TARGET(rcsunittest 
      COMMAND ${TESTSCRIPT_NAME}
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
      COMMENT "Performing Rcs unittests" VERBATIM)
ELSE()
  MESSAGE(STATUS "Rcs unit tests for Linux only")
ENDIF()

################################################################################
#
# Export script
#
################################################################################
INCLUDE(Export)
