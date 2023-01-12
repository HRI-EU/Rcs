CMAKE_MINIMUM_REQUIRED(VERSION 3.4)
SET(BST_INSTALL_CATEGORY Libraries) 

STRING (REPLACE "\\" "/" HGR "$ENV{SIT}")
STRING (REPLACE "\\" "/" MKPLT "$ENV{MAKEFILE_PLATFORM}")

################################################################################
# Check if project is stand-alone or inside tree with dependencies
################################################################################
IF(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  MESSAGE(STATUS "Compiling Rcs as stand-alone project")
  PROJECT(Rcs)
  IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    SET(CMAKE_INSTALL_PREFIX "$ENV{SIT}/Libraries/Rcs/1.0" CACHE PATH "Default sand box directory" FORCE)
  ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  
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

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
LIST(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
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
OPTION(ENABLE_DEBUG_MACROS "Enable debug macros also in release mode" ON)
OPTION(BUILD_RCS_LIBRARIES_ONLY "Build only libs, but not examples and tests" OFF)

################################################################################
#
# Compile settings
#
################################################################################
SET(BUILD_SHARED_LIBS ON)

IF(UNIX)
  OPTION(ENABLE_C++11 "Compile with C++11 support enabled" OFF)

  IF(ENABLE_C++11)
    SET(CMAKE_CXX_STANDARD 11)
    SET(CMAKE_CXX_STANDARD_REQUIRED ON)
  ELSE()
    SET(CMAKE_CXX_FLAGS "-std=c++0x")
  ENDIF()

ENDIF()

# Create shared libraries for Linux and Windows: see http://www.kitware.com/blog/home/post/939
IF(WIN32)
  ADD_DEFINITIONS(-D_CRT_SECURE_NO_DEPRECATE -D_CRT_NONSTDC_NO_DEPRECATE)
  SET(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
  #SET(CREATE_WIN_DLL TRUE)
  #ADD_DEFINITIONS(-DWIN_DLL)
ELSEIF(UNIX)
  SET(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -Wall -pedantic -fPIC -Wno-long-long -Wno-variadic-macros -std=c99")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -fPIC -Wno-format -Wno-long-long -Wno-variadic-macros")
  ADD_DEFINITIONS(-D__linux__)

  # Flag -rdynamic needed for backtrace()
  IF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    SET(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -rdynamic")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -rdynamic")
  ENDIF()
  
  IF(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
    # setting march to core2 to enable valgrind debugging (also good for Xeon)
    # Low level optimization for debug mode, flag for checking stack corruption, flag for debug output
    SET(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -march=core2 -O0 -fstack-protector-all -ggdb")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=core2 -O0 -fstack-protector-all -ggdb")
  ELSE()
    # setting march to native for optimal performance on local machine
    # Strong optimization for release mode
    SET(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -march=native -ggdb")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -ggdb")
  ENDIF()
  
ENDIF()

################################################################################
#
# Dependency settings and compile settings. These must go after Externals.cmake
#
################################################################################
#INCLUDE(Externals)

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
ADD_SUBDIRECTORY (examples)

IF (NOT BUILD_RCS_LIBRARIES_ONLY)
  ADD_SUBDIRECTORY (bin)
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
