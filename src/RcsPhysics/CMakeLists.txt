###############################################################################
#
# CMakeLists file for the RcsPhysics library
#
###############################################################################
IF (UNIX)
  SET(OpenGL_GL_PREFERENCE GLVND) # Removes warning OpenGL_GL_PREFERENCE has not been set to "GLVND" or "LEGACY"
ENDIF(UNIX)

###############################################################################
# MuJoCo. The discovery lives in cmake/Findmujoco.cmake, which also covers the
# macOS release that ships no cmake package configuration file. It provides
# the imported target mujoco::mujoco.
###############################################################################
FIND_PACKAGE(mujoco QUIET)

IF (mujoco_FOUND)
  OPTION(USE_MUJOCO "Use the Mujoco physics library" ON)
ELSE()
  OPTION(USE_MUJOCO "Use the Mujoco physics library" OFF)
ENDIF()

# USE_MUJOCO can be ON in the cache while no MuJoCo installation is available,
# for instance after enabling it by hand, or after moving a build tree to
# another machine. We fail here with an actionable message, rather than later
# in the generate step with an unresolved mujoco::mujoco target.
IF (USE_MUJOCO AND NOT mujoco_FOUND)
  MESSAGE(FATAL_ERROR "USE_MUJOCO is enabled, but no MuJoCo installation was "
          "found. Either point cmake to one with -DMUJOCO_DIR=<prefix>, where "
          "<prefix> is the root directory of a MuJoCo installation, or disable "
          "the Mujoco simulation with -DUSE_MUJOCO=OFF.")
ENDIF()

###############################################################################
# The Mujoco debug window is an optional add-on to the Mujoco simulation. It
# is the only part that needs glfw, therefore the simulation itself can be
# built without it, for instance on a machine that has no OpenGL at all. All
# common glfw distributions ship a cmake package configuration file defining
# the imported target glfw.
#
# This option only says whether the debug window is compiled in, which solely
# depends on glfw being available. Whether it can actually be opened is a
# runtime question, see MujocoDebugWindow::isSupported() - under macOS it
# cannot, since glfw refuses to be initialized from the thread that the window
# runs in. Everything else about the Mujoco simulation is unaffected.
###############################################################################
IF (USE_MUJOCO)
  FIND_PACKAGE(glfw3 CONFIG QUIET)

  IF (glfw3_FOUND)
    OPTION(RCS_USE_MUJOCO_DEBUGWINDOW "Build the Mujoco debug window (needs glfw)" ON)
  ELSE()
    OPTION(RCS_USE_MUJOCO_DEBUGWINDOW "Build the Mujoco debug window (needs glfw)" OFF)
  ENDIF()

  IF (RCS_USE_MUJOCO_DEBUGWINDOW AND NOT glfw3_FOUND)
    MESSAGE(FATAL_ERROR "RCS_USE_MUJOCO_DEBUGWINDOW is enabled, but glfw was not "
            "found. Install it (brew install glfw, apt install libglfw3-dev, "
            "vcpkg install glfw3, ...) or disable the debug window with "
            "-DRCS_USE_MUJOCO_DEBUGWINDOW=OFF.")
  ENDIF()
ENDIF(USE_MUJOCO)

FIND_PACKAGE(VORTEX QUIET)
IF (VORTEX_FOUND)
  OPTION(USE_VORTEX "Use the Vortex physics library" ON)
ELSE()
  OPTION(USE_VORTEX "Use the Vortex physics library" OFF)
ENDIF()

FIND_PACKAGE(Bullet QUIET)
IF(BULLET_FOUND)
  SET(USE_BULLET 2.83_float CACHE STRING "Bullet version to use")
ELSE()
  SET(USE_BULLET OFF CACHE STRING "Bullet version to use")
ENDIF()
SET_PROPERTY(CACHE USE_BULLET PROPERTY STRINGS 3.21_float 3.21_double 2.83_double OFF)

IF(USE_VORTEX AND VORTEX_FOUND)
  SET(RCSVORTEX_LIBRARY ${LIBRARY_OUTPUT_PATH}/libRcsVortex.so)
  IF(WIN32 OR CMAKE_CXX_COMPILER_VERSION VERSION_LESS "5.0")
    # GCC 4.8, we can simply include the vortex library. The same goes for any
    # windows system.
    ADD_SUBDIRECTORY(Vortex)
    # Do not use RcsVortex as target dependency, to avoid export problems. The
    # dependency must be added manually for the external project anyways.
  ELSE()
    # We need to check if the right GCC is available
    FIND_PROGRAM(GCC_48 "gcc-4.8")
    FIND_PROGRAM(GXX_48 "g++-4.8")
    IF(GCC_48 AND GXX_48)
      # Build now using ExternalProject
      INCLUDE(ExternalProject)

      ExternalProject_Add(RcsVortex
          SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Vortex
          INSTALL_DIR ${LIBRARY_OUTPUT_PATH}
          BUILD_ALWAYS TRUE
        #INSTALL_COMMAND cmake -E echo "Skipping install step."
          CMAKE_CACHE_ARGS
            #-DCMAKE_INSTALL_PREFIX:PATH=${INSTALL_DIR}
            -DCMAKE_INSTALL_PREFIX:PATH="$ENV{SIT}/Libraries/Rcs/1.0"
            -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
            -DCMAKE_C_COMPILER:PATH=${GCC_48}
            -DCMAKE_CXX_COMPILER:PATH=${GXX_48}
            -DUSE_VORTEX:STRING=${USE_VORTEX}
            -DVORTEX_ESSENTIALS_DIR:PATH=${VORTEX_ESSENTIALS_DIR}
            -DRCSVORTEX_INSTALL_LIBDIR:PATH=${LIBRARY_OUTPUT_PATH})
    ELSE()
      # Can't build, use prebuilt version
      SET(RCSVORTEX_LIBRARY_PREBUILT "${VORTEX_ESSENTIALS_DIR}/lib/libRcsVortex.so" CACHE PATH "Prebuilt RcsVortex library. Required since we can't build it ourselves.")

      IF(NOT EXISTS ${RCSVORTEX_LIBRARY_PREBUILT})
        MESSAGE(FATAL_ERROR "Cannot build RcsVortex library, so we need a prebuilt version!")
      ENDIF()
      
      # Copy library into output dir
      ADD_CUSTOM_TARGET(RcsVortex)
      ADD_CUSTOM_COMMAND(TARGET RcsVortex PRE_BUILD
          COMMAND ${CMAKE_COMMAND} -E copy ${RCSVORTEX_LIBRARY_PREBUILT} ${LIBRARY_OUTPUT_PATH})
    ENDIF()
    # Install custom library
    INSTALL(FILES ${RCSVORTEX_LIBRARY} DESTINATION lib)
  ENDIF()
ENDIF(USE_VORTEX AND VORTEX_FOUND)

# Build a separate library for Bullet physics. We separate it since we
# have to link it to the RcsPhysics library with the no-as-needed linker
# option. Otherwise, the static self-registration meachnism (for
# instance in the function table initialization) will never be called.
IF(USE_BULLET)
  SET(RCS_BULLET_SRCS 
    BulletDistance.cpp
    BulletHelpers.cpp
    BulletHingeJoint.cpp
    BulletSliderJoint.cpp
    BulletRigidBody.cpp
    BulletSimulation.cpp
    BulletSoftSimulation.cpp)
  
  IF(NOT RCS_HEADLESS_BUILD)    
    LIST(APPEND RCS_BULLET_SRCS
      BulletDebugNode.cpp
      BulletDebugDrawer.cpp)
  ENDIF(NOT RCS_HEADLESS_BUILD)

  ADD_LIBRARY(RcsBullet ${RCS_BULLET_SRCS})

  IF(RCS_HEADLESS_BUILD)
    TARGET_LINK_LIBRARIES(RcsBullet ${BULLET_LIBRARIES} RcsCore)
    TARGET_COMPILE_DEFINITIONS(RcsBullet PRIVATE -DHEADLESS_BUILD)
  ELSE(RCS_HEADLESS_BUILD)
    TARGET_LINK_LIBRARIES(RcsBullet ${BULLET_LIBRARIES} RcsGraphics)
  ENDIF(RCS_HEADLESS_BUILD)

  TARGET_INCLUDE_DIRECTORIES(RcsBullet SYSTEM PRIVATE ${BULLET_INCLUDE_DIR})
  TARGET_COMPILE_DEFINITIONS(RcsBullet PRIVATE ${BULLET_DEFINITIONS} -DUSE_BULLET)

  # We add this option to force the linker to export this symbol. This
  # solves the issue that it will not be exposed, since all initialization 
  # happens inside statically. Under Unix, we can use -Wl,--no-as-needed
  IF(WIN32)
    TARGET_LINK_OPTIONS(RcsBullet PUBLIC /include:RcsShape_distanceBullet)
  ENDIF(WIN32)

ENDIF(USE_BULLET)


# Build the actual physics library. The PhysicsHeadless.cpp exists for the case
# that no other sources are added to the RcsPhysics target.
SET(RCS_PHYSICS_SRCS PhysicsHeadless.cpp)

# Add Mujoco source files. We directly link it into the RcsPhysics library later.
IF(USE_MUJOCO)
  MESSAGE(STATUS "Enabling Mujoco simulation: ${mujoco_ORIGIN}")
  LIST(APPEND RCS_PHYSICS_SRCS MujocoSimulation.cpp)

  IF(RCS_USE_MUJOCO_DEBUGWINDOW)
    MESSAGE(STATUS "Enabling Mujoco debug window")
    LIST(APPEND RCS_PHYSICS_SRCS MujocoDebugWindow.cpp)
  ENDIF(RCS_USE_MUJOCO_DEBUGWINDOW)
ENDIF(USE_MUJOCO)

IF(NOT RCS_HEADLESS_BUILD)
  LIST(APPEND RCS_PHYSICS_SRCS 
       ContactsNode.cpp 
       ForceDragger.cpp
       PhysicsNode.cpp)
ENDIF()

ADD_LIBRARY(RcsPhysics ${RCS_PHYSICS_SRCS})

TARGET_INCLUDE_DIRECTORIES(RcsPhysics PUBLIC 
    # Header source location
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    
    # Header install location
    $<INSTALL_INTERFACE:include/RcsPhysics>
)
TARGET_LINK_LIBRARIES(RcsPhysics RcsCore)

IF(NOT RCS_HEADLESS_BUILD)
  TARGET_LINK_LIBRARIES(RcsPhysics RcsGraphics)
ELSE()
  TARGET_COMPILE_DEFINITIONS(RcsPhysics PRIVATE -DHEADLESS_BUILD)
ENDIF()

# We mark the Bullet include directories as private, since the standard use
# case is to use the physics simulation classes through the factory class.
# In case an explicit use of Bullet is desired, the include path needs to
# be added explicitely, see for instance the target ExampleBullet.
IF(USE_BULLET)
  TARGET_INCLUDE_DIRECTORIES(RcsPhysics SYSTEM PRIVATE ${BULLET_INCLUDE_DIR})
  TARGET_COMPILE_DEFINITIONS(RcsPhysics PRIVATE ${BULLET_DEFINITIONS} -DUSE_BULLET)
  SET(BULLET_INSTALL_TARGET RcsBullet)

  IF (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    TARGET_LINK_LIBRARIES(RcsPhysics -Wl,--no-as-needed RcsBullet -Wl,--as-needed)
  ELSE()
    TARGET_LINK_LIBRARIES(RcsPhysics RcsBullet)
  ENDIF()
  
ENDIF(USE_BULLET)

# Link MujocoSimulation into the RcsPhysics library. Both mujoco::mujoco and
# the glfw target carry their own include directories, therefore no explicit
# include path is needed here. glfw is only linked if the debug window is
# built, it is of no use to the simulation itself.
IF(USE_MUJOCO)
  TARGET_LINK_LIBRARIES(RcsPhysics mujoco::mujoco)
  TARGET_COMPILE_DEFINITIONS(RcsPhysics PRIVATE -DUSE_MUJOCO)

  IF(RCS_USE_MUJOCO_DEBUGWINDOW)
    TARGET_LINK_LIBRARIES(RcsPhysics glfw)
    TARGET_COMPILE_DEFINITIONS(RcsPhysics PRIVATE -DRCS_USE_MUJOCO_DEBUGWINDOW)
  ENDIF(RCS_USE_MUJOCO_DEBUGWINDOW)
ENDIF(USE_MUJOCO)

IF(USE_VORTEX AND VORTEX_FOUND)
  # Link RcsVortex library here, so RcsPhysics is still the goto dependency for
  # it. Use no-as-needed to make sure the library is loaded, only then will the
  # physics engine be registered.
  IF (UNIX)
    TARGET_LINK_LIBRARIES(RcsPhysics -Wl,--no-as-needed ${RCSVORTEX_LIBRARY} -Wl,--as-needed ${VORTEX_LIBRARIES})
    ENDIF(UNIX)
  ADD_DEPENDENCIES(RcsPhysics RcsVortex)
ENDIF()

# copy the .dll file to the same folder as the executable
IF (WIN32)
#  ADD_CUSTOM_COMMAND(
#    TARGET RcsPhysics POST_BUILD
#    COMMAND ${CMAKE_COMMAND} -E copy
#    ${LIBRARY_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}/RcsPhysics.dll
#    ${EXECUTABLE_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}/RcsPhysics.dll)

#  IF (USE_BULLET)
#    ADD_CUSTOM_COMMAND(
#      TARGET RcsBullet POST_BUILD
#      COMMAND ${CMAKE_COMMAND} -E copy
#      ${LIBRARY_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}/RcsBullet.dll
#      ${EXECUTABLE_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}/RcsBullet.dll)
#  ENDIF(USE_BULLET)

  # Copy the MuJoCo dll next to the executables. The location is taken from
  # the imported target, so that it is correct for any MuJoCo installation.
  IF(USE_MUJOCO)
    ADD_CUSTOM_COMMAND(
      TARGET RcsPhysics POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
      $<TARGET_FILE:mujoco::mujoco>
      ${EXECUTABLE_OUTPUT_PATH}/${CMAKE_BUILD_TYPE})
  ENDIF(USE_MUJOCO)

ENDIF(WIN32)

# Mark for export
IF (USE_BULLET)
  RCS_ADD_EXPORT_LIBRARY(RcsBullet)
ENDIF(USE_BULLET)
RCS_ADD_EXPORT_LIBRARY(RcsPhysics)

# Installation
# Headers
INSTALL(
  DIRECTORY . 
  DESTINATION include/RcsPhysics
  FILES_MATCHING PATTERN "*.h"
)
# Install the binaries. Variable BULLET_INSTALL_TARGET is empty if Bullet is
# not enabled
#INSTALL(TARGETS ${BULLET_INSTALL_TARGET} RcsPhysics EXPORT RcsExport DESTINATION lib)
INSTALL(TARGETS  ${BULLET_INSTALL_TARGET} RcsPhysics EXPORT RcsExport
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib/Rcs)
