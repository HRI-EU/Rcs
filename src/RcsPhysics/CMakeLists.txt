
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
        #INSTALL_COMMAND cmake -E echo "Skipping install step."
          CMAKE_CACHE_ARGS
            -DCMAKE_INSTALL_PREFIX:${INSTALL_DIR}
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

# Build the actual physics library
SET(RCS_PHYSICS_SRCS)

IF(NOT HEADLESS_BUILD)
  LIST(APPEND RCS_PHYSICS_SRCS 
       ContactsNode.cpp 
       ForceDragger.cpp
       PhysicsNode.cpp)
ENDIF()

IF(USE_BULLET)
  LIST(APPEND RCS_PHYSICS_SRCS 
    BulletDistance.cpp
    BulletHelpers.cpp
    BulletHingeJoint.cpp
    BulletSliderJoint.cpp
    BulletRigidBody.cpp
    BulletSimulation.cpp)
  IF(NOT HEADLESS_BUILD)
    LIST(APPEND RCS_PHYSICS_SRCS
      BulletDebugNode.cpp
      BulletDebugDrawer.cpp)
  ENDIF(NOT HEADLESS_BUILD)
ENDIF(USE_BULLET)


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

IF(USE_BULLET)
  TARGET_INCLUDE_DIRECTORIES(RcsPhysics SYSTEM PRIVATE ${BULLET_INCLUDE_DIRS})
  TARGET_LINK_LIBRARIES(RcsPhysics ${BULLET_LIBRARIES})
  TARGET_COMPILE_DEFINITIONS(RcsPhysics PRIVATE ${BULLET_DEFINITIONS})
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

IF(USE_BULLET)
  ADD_EXECUTABLE(ExampleBullet ExampleBullet.cpp)
  TARGET_LINK_LIBRARIES(ExampleBullet RcsPhysics)
  TARGET_INCLUDE_DIRECTORIES(ExampleBullet SYSTEM PRIVATE ${BULLET_INCLUDE_DIRS})
  TARGET_COMPILE_DEFINITIONS(RcsPhysics PRIVATE ${BULLET_DEFINITIONS})
ENDIF(USE_BULLET)

# Mark for export
RCS_ADD_EXPORT_LIBRARY(RcsPhysics)

# Installation
# Headers
INSTALL(
  DIRECTORY . 
  DESTINATION include/RcsPhysics
  FILES_MATCHING PATTERN "*.h"
)
# Install the binaries
INSTALL(TARGETS RcsPhysics EXPORT RcsExport DESTINATION lib)
