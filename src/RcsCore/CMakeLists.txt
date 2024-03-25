###############################################################################
#
# CMakeLists file for the RcsCore library
#
###############################################################################
FIND_PACKAGE(octomap QUIET)
IF (octomap_FOUND)
  OPTION(USE_OCTOMAP "Use OctoMap library for distance calculation" ON)
ELSE(octomap_FOUND)
  OPTION(USE_OCTOMAP "Use OctoMap library for distance calculation" OFF)
ENDIF(octomap_FOUND)

FIND_PACKAGE (Eigen3 3.2.0 QUIET)
IF (Eigen3_FOUND)
  OPTION(USE_EIGEN3 "Use Eigen3 in RcsCore" ON)
ELSE(Eigen3_FOUND)
  OPTION(USE_EIGEN3 "Use Eigen3 in RcsCore" OFF)
ENDIF(Eigen3_FOUND)

IF (WM5_FOUND)
  MESSAGE(STATUS "WM5: NOT trying to find package")
ELSE()
  MESSAGE(STATUS "WM5: Trying to find package")
  FIND_PACKAGE(WM5 QUIET)
  SET(WM5_WITH_FIND_PACKAGE ON)
ENDIF()

IF (WM5_FOUND)
  OPTION(USE_WM5 "Use Wildmagic library" ON)
ELSE(WM5_FOUND)
  OPTION(USE_WM5 "Use Wildmagic library" OFF)
ENDIF(WM5_FOUND)

FIND_PACKAGE(qpOASES QUIET)
IF (QPOASES_FOUND)
  OPTION(USE_QPOASES "Use qpOASES library" ON)
ELSE(QPOASES_FOUND)
  OPTION(USE_QPOASES "Use qpOASES library" OFF)
ENDIF(QPOASES_FOUND)

SET(RCS_CORE_SRCS
CompositeTask.cpp
ControllerBase.cpp
IkSolverConstraintRMR.cpp
IkSolverProjectedQP.cpp
IkSolverPrioRMR.cpp
IkSolverRMR.cpp
KeyCatcherBase.cpp
KineticSimulation.cpp
PeriodicCallback.cpp
PhysicsBase.cpp
PhysicsConfig.cpp
PhysicsFactory.cpp
Rcs_basicMath.c
Rcs_body.c
Rcs_broadphase.c
Rcs_BVHParser.c
Rcs_cmdLine.cpp
Rcs_collisionModel.c
Rcs_distanceWM5.cpp
Rcs_dynamics.c
Rcs_filters.cpp
Rcs_geometry.c
Rcs_gradientTests.c
Rcs_graph.c
Rcs_graphParser.c
Rcs_HTr.c
Rcs_intersectionWM5.cpp 
Rcs_joint.c
Rcs_kinematics.c
Rcs_linAlg.c
Rcs_macros.c
Rcs_Mat3d.c
Rcs_mathTests.cpp
Rcs_MatNd.c
Rcs_mesh.c
Rcs_graphOpenRAVEParser.c
Rcs_parser.c
Rcs_quaternion.c
Rcs_resourcePath.cpp
Rcs_sensor.c
Rcs_shape.c
Rcs_stackTrace.cpp
Rcs_stlParser.cpp
Rcs_timer.c
Rcs_mujocoParser.c
Rcs_URDFParser.c
Rcs_utils.c
Rcs_utilsCPP.cpp
Rcs_Vec3d.c
Rcs_VecNd.c
Rcs_material.cpp
SolverRAC.cpp
TaskAngularMomentum3D.cpp
#TaskCollision.cpp
TaskCOM1D.cpp
TaskCOM3D.cpp
Task.cpp
TaskCompositeXml.cpp
TaskCylindrical1D.cpp
TaskCylindrical3D.cpp
TaskCylindricalPos3D.cpp
TaskCylindricalForce1D.cpp
TaskDifferentialConstraint1D.cpp
TaskDistance.cpp
TaskDistance1D.cpp
TaskDistance3D.cpp
TaskEuler1D.cpp
TaskEuler3D.cpp
TaskFactory.cpp
TaskGenericEuler3D.cpp
TaskGenericIK.cpp
TaskInclination.cpp
TaskJoint.cpp
TaskJointLimit.cpp
TaskJointLimitPlateau.cpp
TaskJoints.cpp
TaskOmega1D.cpp
TaskOmega3D.cpp
TaskPolar2D.cpp
TaskPolarTarget2D.cpp
TaskPolarSurfaceNormal.cpp
TaskPose5D.cpp
TaskPose6D.cpp
TaskPosition1D.cpp
TaskPosition2D.cpp
TaskPosition3D.cpp
TaskPositionForce1D.cpp
TaskPositionForce3D.cpp
TaskPositionTarget3D.cpp
TaskRadial.cpp
TaskSpherical1D.cpp
TaskSpherical3D.cpp
TaskSphericalForce1D.cpp
TaskStaticEffort.cpp
TaskTangentCircle.cpp
TaskVelocity1D.cpp
TaskVelocity3D.cpp
TaskVelocityJoint.cpp
ViaPointSequence.cpp
Rcs_thirdPartyMath.c
EulerAngles.c
TaskRegion.cpp
TaskRegionBoundingBox.cpp
TaskRegionBoxInterval.cpp
TaskRegionFactory.cpp
TaskSpaceBlender.cpp
TaskSleeve.cpp
Rcs_octree.cpp
ExampleBase.cpp
ExampleFactory.cpp
IkSolverQPOA.cpp
URDFAttributes.cpp
URDFElement.cpp
URDFGenerator.cpp
)

IF(WIN32)
  LIST(APPEND RCS_CORE_SRCS Rcs_msvc.cpp)
  FIND_PACKAGE(LibXml2 REQUIRED)
  FIND_PACKAGE(PThreads4W REQUIRED)
  SET(PTHREAD_LIBRARIES PThreads4W::PThreads4W)
ELSE(WIN32)
  FIND_PACKAGE(LibXml2 REQUIRED)
  SET(PTHREAD_LIBRARIES pthread)
ENDIF(WIN32)

IF(USE_EIGEN3)
  IF (Eigen3_FOUND)
    SET(SRC_EIGEN3 Rcs_eigen.cpp)
    IF (UNIX)
      SET(FLAGS_E3 "-O3")
    ENDIF(UNIX)
    LIST(APPEND RCS_CORE_SRCS ${SRC_EIGEN3})
    SET_PROPERTY(SOURCE Rcs_mathTests.cpp PROPERTY COMPILE_DEFINITIONS USE_EIGEN3 EIGEN_MPL2_ONLY)
    SET_PROPERTY(SOURCE Rcs_linAlg.c PROPERTY COMPILE_DEFINITIONS USE_EIGEN3 EIGEN_MPL2_ONLY)
    SET_PROPERTY(SOURCE Rcs_linAlg.c PROPERTY COMPILE_FLAGS "${FLAGS_E3}")
    SET_PROPERTY(SOURCE ${SRC_EIGEN3} PROPERTY COMPILE_FLAGS "-I${EIGEN3_INCLUDE_DIR} ${FLAGS_E3}")
  ELSE(Eigen3_FOUND)
    MESSAGE(WARNING "Option USE_EIGEN3 is set, but Eigen3 was not found - skipping Eigen3 dependencies")
  ENDIF(Eigen3_FOUND)
ENDIF()

IF(USE_WM5)
  IF (WM5_FOUND)
    IF (WM5_WITH_FIND_PACKAGE)
      SET(SRC_WM5 Rcs_intersectionWM5.cpp Rcs_distanceWM5.cpp)
      SET_PROPERTY(SOURCE ${SRC_WM5} PROPERTY COMPILE_DEFINITIONS ${WM5_DEFINITIONS})
      SET_PROPERTY(SOURCE ${SRC_WM5} PROPERTY COMPILE_FLAGS -I${WM5_INCLUDE_DIR} )
      SET_PROPERTY(SOURCE ${SRC_WM5} APPEND_STRING PROPERTY COMPILE_FLAGS " ${WM5_FLAGS} ")
    ENDIF()
  ELSE(WM5_FOUND)
    MESSAGE(WARNING "Option USE_WM5 is set, but WM5 was not found - skipping WM5 dependencies")
  ENDIF(WM5_FOUND)
ENDIF(USE_WM5)

IF(USE_QPOASES)
  IF (QPOASES_FOUND)
    SET(SRC_QPOASES IkSolverQPOA.cpp)
    SET_PROPERTY(SOURCE ${SRC_QPOASES} PROPERTY COMPILE_DEFINITIONS ${QPOASES_DEFINITIONS})
    SET_PROPERTY(SOURCE ${SRC_QPOASES} PROPERTY COMPILE_FLAGS -I${QPOASES_INCLUDE_DIR} )
  ELSE(QPOASES_FOUND)
    MESSAGE(WARNING "Option USE_QPOASES is set, but qpOASES was not found - skipping qpOASES dependencies")
  ENDIF(QPOASES_FOUND)
ENDIF(USE_QPOASES)
ADD_LIBRARY(RcsCore ${RCS_CORE_SRCS})

IF(WIN32)
  SET_SOURCE_FILES_PROPERTIES(${RCS_CORE_SRCS} PROPERTIES LANGUAGE CXX)
  TARGET_COMPILE_DEFINITIONS(RcsCore PRIVATE -DRCSCORE_EXPORTS)
ELSE()
  TARGET_COMPILE_DEFINITIONS(RcsCore PRIVATE -D_GNU_SOURCE)
ENDIF()

TARGET_INCLUDE_DIRECTORIES(RcsCore PUBLIC
    # Header source location
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    
    # Header install location
    $<INSTALL_INTERFACE:include/RcsCore>
    
    # Regular dependencies
    ${LIBXML2_INCLUDE_DIR}
    ${PTHREAD_INCLUDE_DIR})

TARGET_LINK_LIBRARIES(RcsCore ${LIBXML2_LIBRARIES} ${PTHREAD_LIBRARIES})

IF(WM5_FOUND)
  TARGET_LINK_LIBRARIES(RcsCore ${WM5_LIBRARIES})
ENDIF(WM5_FOUND)

IF(USE_OCTOMAP)
  IF (octomap_FOUND)
    SET(OCTOMAP_DEFINITIONS -DUSE_OCTOMAP)
    TARGET_COMPILE_DEFINITIONS(RcsCore PRIVATE ${OCTOMAP_DEFINITIONS})
    TARGET_INCLUDE_DIRECTORIES(RcsCore SYSTEM PRIVATE ${OCTOMAP_INCLUDE_DIRS})
    TARGET_LINK_LIBRARIES(RcsCore ${OCTOMAP_LIBRARIES})
  ELSE(octomap_FOUND)
    MESSAGE(STATUS "Compiling RcsCore without OctoMap support")
  ENDIF(octomap_FOUND)
ENDIF(USE_OCTOMAP)
IF(USE_QPOASES)
  TARGET_LINK_LIBRARIES(RcsCore ${QPOASES_LIBRARIES})
ENDIF(USE_QPOASES)

# copy the .dll file to the same folder as the executable
IF (WIN32)
ADD_CUSTOM_COMMAND(
    TARGET RcsCore POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
    ${LIBRARY_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}/RcsCore.dll
    ${EXECUTABLE_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}/RcsCore.dll)
ENDIF()

# Mark for export
RCS_ADD_EXPORT_LIBRARY(RcsCore)

# Install the headers
INSTALL(
  DIRECTORY . 
  DESTINATION include/RcsCore
  FILES_MATCHING PATTERN "*.h"
)
# Install the binaries
INSTALL(TARGETS RcsCore EXPORT RcsExport DESTINATION lib)
