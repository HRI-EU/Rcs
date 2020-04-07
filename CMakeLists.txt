CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
SET(BST_INSTALL_CATEGORY Libraries) 

PROJECT(Rcs)

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

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

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
  MESSAGE(STATUS "Removing non-directory ${CONFIG_DST_DIR}")
  FILE(REMOVE ${CONFIG_DST_DIR})
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
# Create shared libraries for Linux and Windows
# For CMake 3.4 and higher, there is an easy way to create windows dlls. For
# details, see http://www.kitware.com/blog/home/post/939
#
################################################################################
IF(UNIX)
  MESSAGE(STATUS "Creating dynamically linked libraries (.so) under Unix")
  SET(LIBRARY_LINKING_TYPE "SHARED")
ENDIF()

IF(WIN32)
  MESSAGE("We are on a windows machine")
  IF (CMAKE_VERSION VERSION_LESS 3.4)
    MESSAGE(FATAL_ERROR "CMake version too old, at least 3.4 is needed")
  ELSE()
    MESSAGE("Creating windows dlls")
    SET(LIBRARY_LINKING_TYPE "SHARED")
    SET(BUILD_SHARED_LIBS ON)
    SET(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
    SET(CREATE_WIN_DLL TRUE)
    ADD_DEFINITIONS(-DWIN_DLL)
  ENDIF()
ENDIF(WIN32)

SITE_NAME(HOSTNAME)

################################################################################
#
# Default options
#
################################################################################
OPTION(HEADLESS_BUILD "Build without graphics and Guis" OFF)
OPTION(USE_WM5 "Use Wildmagic library" OFF)
# OPTION(USE_OCTOMAP "Use Octomap library" OFF)
OPTION(USE_EIGEN3 "Use Eigen3 library" OFF)
OPTION(ENABLE_DEBUG_MACROS "Enable debug macros also in release mode" ON)

IF(UNIX)
  OPTION(ENABLE_C++11 "Compile with C++11 support enabled" OFF)
ENDIF()

SET(USE_VORTEX OFF CACHE STRING "Vortex version to use")
SET_PROPERTY(CACHE USE_VORTEX PROPERTY STRINGS OFF ESSENTIALS)

SET(USE_BULLET OFF CACHE STRING "Bullet version to use")
SET_PROPERTY(CACHE USE_BULLET PROPERTY STRINGS latest 2.83_float 2.83_double OFF)

################################################################################
#
# Dependency settings and compile settings. These must go after Externals.cmake
#
################################################################################
INCLUDE(Externals)
INCLUDE(CompileSettings)

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
# the dependencies must be added to the LD_LIBRARY_PATH manually in oredr to
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
# Flags to strip down the file names for logging at compile time. Adapted from
# http://stackoverflow.com/questions/8487986/file-macro-shows-full-path
#
################################################################################
IF(UNIX)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__FILENAME__='\"$(subst ${CMAKE_SOURCE_DIR}/,,$(abspath $<))\"'")
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D__FILENAME__='\"$(subst ${CMAKE_SOURCE_DIR}/,,$(abspath $<))\"'")
ENDIF()

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
ADD_SUBDIRECTORY (bin)
ADD_SUBDIRECTORY (examples)

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
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/unittest.in
  ${CMAKE_CURRENT_BINARY_DIR}/unittest.sh @ONLY)

ADD_CUSTOM_TARGET(unittest 
    COMMAND ${CMAKE_CURRENT_BINARY_DIR}/unittest.sh
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Performing unittests" VERBATIM)

################################################################################
#
# Export script
#
################################################################################
INCLUDE(Export)
