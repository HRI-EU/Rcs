
# Build the vortex sub-library
FIND_PACKAGE(Vortex)

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
# option. Otherwise, te static self-registration meachnism (for
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
  IF(NOT HEADLESS_BUILD)
    LIST(APPEND RCS_BULLET_SRCS
      BulletDebugNode.cpp
      BulletDebugDrawer.cpp)
  ENDIF(NOT HEADLESS_BUILD)

  ADD_LIBRARY(RcsBullet ${LIBRARY_LINKING_TYPE} ${RCS_BULLET_SRCS})
  TARGET_LINK_LIBRARIES(RcsBullet ${BULLET_LIBRARIES} RcsGraphics)
  TARGET_INCLUDE_DIRECTORIES(RcsBullet SYSTEM PRIVATE ${BULLET_INCLUDE_DIR})
  TARGET_COMPILE_DEFINITIONS(RcsBullet PRIVATE ${BULLET_DEFINITIONS})

  # We add this option to force the linker to export this symbol. This
  # solves the issue that it will not be exposed, since all initialization 
  # happens inside statically. Under Unix, we can use -Wl,--no-as-needed
  IF(WIN32)
    TARGET_LINK_OPTIONS(RcsBullet PUBLIC /include:RcsShape_distanceBullet)
  ENDIF()

ENDIF(USE_BULLET)


# Build the actual physics library. Currently the eadless mode is a bit broken.
SET(RCS_PHYSICS_SRCS)
IF(NOT HEADLESS_BUILD)
  LIST(APPEND RCS_PHYSICS_SRCS 
       ContactsNode.cpp 
       ForceDragger.cpp
       PhysicsNode.cpp)
ENDIF()

ADD_LIBRARY(RcsPhysics ${LIBRARY_LINKING_TYPE} ${RCS_PHYSICS_SRCS})

TARGET_INCLUDE_DIRECTORIES(RcsPhysics PUBLIC 
    # Header source location
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    
    # Header install location
    $<INSTALL_INTERFACE:include/RcsPhysics>
)
TARGET_LINK_LIBRARIES(RcsPhysics RcsCore)

IF(NOT HEADLESS_BUILD)
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
  TARGET_COMPILE_DEFINITIONS(RcsPhysics PRIVATE ${BULLET_DEFINITIONS})
  SET(BULLET_INSTALL_TARGET RcsBullet)

  IF (UNIX)
    TARGET_LINK_LIBRARIES(RcsPhysics -Wl,--no-as-needed RcsBullet -Wl,--as-needed)
  ELSE()
    TARGET_LINK_LIBRARIES(RcsPhysics RcsBullet)
  ENDIF()
  
ENDIF(USE_BULLET)

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
ADD_CUSTOM_COMMAND(
    TARGET RcsBullet POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
    ${LIBRARY_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}/RcsBullet.dll
    ${EXECUTABLE_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}/RcsBullet.dll)
ADD_CUSTOM_COMMAND(
    TARGET RcsPhysics POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
    ${LIBRARY_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}/RcsPhysics.dll
    ${EXECUTABLE_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}/RcsPhysics.dll)
ENDIF()

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
INSTALL(TARGETS ${BULLET_INSTALL_TARGET} RcsPhysics EXPORT RcsExport DESTINATION lib)
