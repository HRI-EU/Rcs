###############################################################################
#
# CMakeLists file for the RcsCore library
#
###############################################################################
SET(RCS_CORE_SRCS
CompositeTask.cpp
ControllerBase.cpp
HardwareComponent.cpp
IkSolverConstraintRMR.cpp
IkSolverPrioRMR.cpp
IkSolverRMR.cpp
KeyCatcherBase.cpp
KineticSimulation.cpp
MotionControlLayer.cpp
PeriodicCallback.cpp
PhysicsBase.cpp
PhysicsConfig.cpp
PhysicsFactory.cpp
Rcs_basicMath.c
Rcs_body.c
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
Rcs_URDFParser.c
Rcs_utils.c
Rcs_utilsCPP.cpp
Rcs_Vec3d.c
Rcs_VecNd.c
SensorComponent.cpp
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
TaskSpherical1D.cpp
TaskSpherical3D.cpp
TaskSphericalForce1D.cpp
TaskStaticEffort.cpp
TaskVelocity1D.cpp
TaskVelocity3D.cpp
TaskVelocityJoint.cpp
ViaPointSequence.cpp
${RCS_THIRDPARTY_DIR}/Rcs_thirdPartyMath.c
${RCS_THIRDPARTY_DIR}/EulerAngles.c
TaskRegion.cpp
TaskRegionBoundingBox.cpp
TaskRegionBoxInterval.cpp
TaskRegionFactory.cpp
TaskSpaceBlender.cpp
TaskSleeve.cpp
)

IF(WIN32)
  LIST(APPEND RCS_CORE_SRCS Rcs_msvc.cpp)
  FIND_PACKAGE(LibXml2-win REQUIRED)
  SET(PTHREAD_LIBRARIES ${HGR}/External/pthreads-win/2.91/lib/${MKPLT}/RcsPthreadsVC2.lib)
  SET(PTHREAD_INCLUDE_DIR ${HGR}/External/pthreads-win/2.91/include)
ELSE(WIN32)
  FIND_PACKAGE(LibXml2 REQUIRED)
  SET(PTHREAD_LIBRARIES pthread)
ENDIF(WIN32)

IF(USE_EIGEN3)
  FIND_PACKAGE (Eigen3 3.2.0 REQUIRED)
  IF (Eigen3_FOUND)
    SET(SRC_EIGEN3 Rcs_eigen.cpp)
    LIST(APPEND RCS_CORE_SRCS ${SRC_EIGEN3})
    SET_PROPERTY(SOURCE Rcs_mathTests.cpp PROPERTY COMPILE_DEFINITIONS USE_EIGEN3 EIGEN_MPL2_ONLY)
    SET_PROPERTY(SOURCE Rcs_linAlg.c PROPERTY COMPILE_DEFINITIONS USE_EIGEN3 EIGEN_MPL2_ONLY)
    SET_PROPERTY(SOURCE ${SRC_EIGEN3} PROPERTY COMPILE_FLAGS -I${EIGEN3_INCLUDE_DIR})
  ELSE(Eigen3_FOUND)
    MESSAGE(FATAL_ERROR "Option USE_EIGEN3 is set, but Eigen3 was not found")
  ENDIF(Eigen3_FOUND)
ENDIF()

IF(USE_WM5)
  FIND_PACKAGE(WM5)
  IF (WM5_FOUND)
    SET(SRC_WM5 Rcs_intersectionWM5.cpp Rcs_distanceWM5.cpp)
    SET_PROPERTY(SOURCE ${SRC_WM5} PROPERTY COMPILE_DEFINITIONS ${WM5_DEFINITIONS})
    SET_PROPERTY(SOURCE ${SRC_WM5} PROPERTY COMPILE_FLAGS -I${WM5_INCLUDE_DIR} )
    SET_PROPERTY(SOURCE ${SRC_WM5} APPEND_STRING PROPERTY COMPILE_FLAGS " ${WM5_FLAGS} ")
  ELSE(WM5_FOUND)
    MESSAGE(FATAL_ERROR "Option USE_WM5 is set, but WM5 was not found")
  ENDIF(WM5_FOUND)
ENDIF(USE_WM5)

ADD_LIBRARY(RcsCore ${LIBRARY_LINKING_TYPE} ${RCS_CORE_SRCS})

IF(WIN32)
  SET_SOURCE_FILES_PROPERTIES(${RCS_CORE_SRCS} PROPERTIES LANGUAGE CXX)
  TARGET_COMPILE_DEFINITIONS(RcsCore PRIVATE -DRCSCORE_EXPORTS)
ELSE()
  TARGET_COMPILE_DEFINITIONS(RcsCore PRIVATE -D_GNU_SOURCE)
ENDIF()

TARGET_INCLUDE_DIRECTORIES(RcsCore PUBLIC
    # Header source location
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${RCS_THIRDPARTY_DIR}>
    
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
  TARGET_COMPILE_DEFINITIONS(RcsCore PRIVATE ${OCTOMAP_DEFINITIONS})
  TARGET_INCLUDE_DIRECTORIES(RcsCore SYSTEM PRIVATE ${OCTOMAP_INCLUDE_DIR})
  TARGET_LINK_LIBRARIES(RcsCore ${OCTOMAP_LIBRARIES})
ENDIF(USE_OCTOMAP)

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

# Installation
# Headers
INSTALL(
  DIRECTORY . 
  DESTINATION include/RcsCore
  FILES_MATCHING PATTERN "*.h"
)
# The external headers go to the same directory
INSTALL(
  FILES ${RCS_THIRDPARTY_DIR}/EulerAngles.h
        ${RCS_THIRDPARTY_DIR}/QuatTypes.h
  DESTINATION include/RcsCore
)
# Install the binaries
INSTALL(TARGETS RcsCore EXPORT RcsExport DESTINATION lib)
