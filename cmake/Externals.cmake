################################################################################
#
# Settings for finding files and directories
#
################################################################################

string (REPLACE "\\" "/" HGR "$ENV{SIT}")
string (REPLACE "\\" "/" MKPLT "$ENV{MAKEFILE_PLATFORM}")

################################################################################
#
# Settings for bullet physics
#
################################################################################
IF(UNIX)
  SET(BT_LIB_PREFIX "lib")
  SET(BT_LIB_SUFFIX "so")
ELSEIF(WIN32)
  SET(BT_LIB_PREFIX "")
  SET(BT_LIB_SUFFIX "lib")
ENDIF()

##############################################################################
# Bullet 2.83 double version
##############################################################################
IF (USE_BULLET STREQUAL 2.83_double)

  IF(DEFINED ENV{SIT})
    # SIT available, use that version by default
    SET(BT_LIB_DIR ${HGR}/External/Bullet/2.83/lib/${MKPLT} CACHE PATH "Bullet library directory")
    SET(BULLET_INCLUDE_DIRS ${HGR}/External/Bullet/2.83/include CACHE PATH "Bullet include directory")
  ELSE()
    # SIT not available, path must be set
    SET(BT_LIB_DIR "" CACHE PATH "Bullet library directory")
    SET(BULLET_INCLUDE_DIRS "" CACHE PATH "Bullet include directory")
  ENDIF()

  SET(BULLET_DEFINITIONS -DUSE_BULLET -DBT_USE_DOUBLE_PRECISION)
  SET(BULLET_CXX_FLAGS "-isystem ${BULLET_INCLUDE_DIRS}")

  IF(UNIX)
    SET(BULLET_LIBRARIES
      ${BT_LIB_DIR}/libLinearMath.so 
      ${BT_LIB_DIR}/libBulletCollision.so
      ${BT_LIB_DIR}/libBulletDynamics.so 
      ${BT_LIB_DIR}/libBulletSoftBody.so 
      )
  ELSEIF(WIN32)
    SET(BULLET_LIBRARIES
      ${BT_LIB_DIR}/BulletCollision.lib 
      ${BT_LIB_DIR}/BulletDynamics.lib 
      ${BT_LIB_DIR}/BulletFileLoader.lib 
      ${BT_LIB_DIR}/BulletSoftBody.lib 
      ${BT_LIB_DIR}/BulletInverseDynamics.lib
      ${BT_LIB_DIR}/BulletInverseDynamicsUtils.lib
      ${BT_LIB_DIR}/BulletWorldImporter.lib 
      ${BT_LIB_DIR}/BulletXmlWorldImporter.lib 
      ${BT_LIB_DIR}/ConvexDecomposition.lib 
      ${BT_LIB_DIR}/GIMPACTUtils.lib 
      ${BT_LIB_DIR}/LinearMath.lib 
      opengl32 glu32
      #${BT_LIB_DIR}/glut32.lib
      )
  ENDIF()
  
ELSEIF(USE_BULLET STREQUAL 2.83_float)

  FIND_PACKAGE(Bullet REQUIRED)
  SET(BULLET_DEFINITIONS -DUSE_BULLET)

ENDIF()

################################################################################
#
# Settings for OpenSceneGraph 
# 
################################################################################
IF(NOT HEADLESS_BUILD)

  IF(WIN32)

    SET(OPENSCENEGRAPH_INCLUDE_DIRS ${HGR}/External/OpenSceneGraph/3.6.2/${MKPLT}/include)
    SET(OSG_BINARY_DIR ${HGR}/External/OpenSceneGraph/3.6.2/${MKPLT}/bin)
    SET(OSG_LIBRARY_DIR ${HGR}/External/OpenSceneGraph/3.6.2/${MKPLT}/lib)

    SET(OPENSCENEGRAPH_LIBRARIES
      ${OSG_LIBRARY_DIR}/osgAnimation${RCS_DEBUG_SUFFIX}.lib
      ${OSG_LIBRARY_DIR}/osg${RCS_DEBUG_SUFFIX}.lib
      ${OSG_LIBRARY_DIR}/osgDB${RCS_DEBUG_SUFFIX}.lib
      ${OSG_LIBRARY_DIR}/osgFX${RCS_DEBUG_SUFFIX}.lib
      ${OSG_LIBRARY_DIR}/osgGA${RCS_DEBUG_SUFFIX}.lib
      ${OSG_LIBRARY_DIR}/osgManipulator${RCS_DEBUG_SUFFIX}.lib
      ${OSG_LIBRARY_DIR}/osgParticle${RCS_DEBUG_SUFFIX}.lib
      ${OSG_LIBRARY_DIR}/osgPresentation${RCS_DEBUG_SUFFIX}.lib
      ${OSG_LIBRARY_DIR}/osgShadow${RCS_DEBUG_SUFFIX}.lib
      ${OSG_LIBRARY_DIR}/osgSim${RCS_DEBUG_SUFFIX}.lib
      ${OSG_LIBRARY_DIR}/osgTerrain${RCS_DEBUG_SUFFIX}.lib
      ${OSG_LIBRARY_DIR}/osgText${RCS_DEBUG_SUFFIX}.lib
      ${OSG_LIBRARY_DIR}/osgUtil${RCS_DEBUG_SUFFIX}.lib
      ${OSG_LIBRARY_DIR}/osgViewer${RCS_DEBUG_SUFFIX}.lib
      ${OSG_LIBRARY_DIR}/osgVolume${RCS_DEBUG_SUFFIX}.lib
      ${OSG_LIBRARY_DIR}/osgWidget${RCS_DEBUG_SUFFIX}.lib
      ${OSG_LIBRARY_DIR}/OpenThreads${RCS_DEBUG_SUFFIX}.lib
      )

  ELSE(WIN32)

    SET(OpenSceneGraph_MARK_AS_ADVANCED TRUE)
    FIND_PACKAGE(OpenSceneGraph REQUIRED
      OpenThreads 
      osgDB 
      osg 
      osgManipulator 
      osgShadow 
      osgText 
      osgUtil 
      osgViewer 
      osgFX 
      osgGA)

  ENDIF(WIN32)

ENDIF(NOT HEADLESS_BUILD)

################################################################################
#
# Settings for pthreads. 
#
################################################################################
IF(WIN32)
  SET(PTHREAD_LIBRARIES ${HGR}/External/pthreads-win/2.91/lib/${MKPLT}/RcsPthreadsVC2.lib)
  SET(PTHREAD_INCLUDE_DIR ${HGR}/External/pthreads-win/2.91/include)
ELSE(WIN32)
  SET(PTHREAD_LIBRARIES pthread)
ENDIF()

################################################################################
#
# Settings for qwt
#
################################################################################
IF(NOT HEADLESS_BUILD)

  IF(WIN32)

    IF (NOT MSVC_VERSION VERSION_LESS 1900)
	
      SET(QWT_INCLUDE_DIRS ${HGR}/External/qwt/6.1.3/include)
      SET(QWT_LIBRARY_DIR ${HGR}/External/qwt/6.1.3/lib/${MKPLT})
      SET(QWT_MAJOR_VERSION 6)
  
      ADD_LIBRARY(libqwt STATIC IMPORTED)
      SET_PROPERTY(TARGET libqwt PROPERTY IMPORTED_LOCATION ${QWT_LIBRARY_DIR}/qwt.lib)
      SET_PROPERTY(TARGET libqwt PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${QWT_INCLUDE_DIRS})
      SET(QWT_LIBRARIES libqwt)

	ELSE()
	
    SET(QWT_INCLUDE_DIRS ${HGR}/External/qwt/5.2/include)
    SET(QWT_INCLUDE_DIR ${HGR}/External/qwt/5.2/include)
    SET(QWT_LIBRARY_DIR ${HGR}/External/qwt/5.2/lib/${MKPLT})
    SET(QWT_MAJOR_VERSION 5)
  
    ADD_LIBRARY(libqwt STATIC IMPORTED)
    SET_PROPERTY(TARGET libqwt PROPERTY IMPORTED_LOCATION ${QWT_LIBRARY_DIR}/qwt5${RCS_DEBUG_SUFFIX}.lib)
    SET_PROPERTY(TARGET libqwt PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${HGR}/External/qwt/5.2/include)
    SET(QWT_LIBRARIES libqwt)
	
	ENDIF()

  ELSE(WIN32)

    FIND_PACKAGE(Qwt REQUIRED)

  ENDIF(WIN32)

ENDIF(NOT HEADLESS_BUILD)

################################################################################
#
# Settings for Qt
#
################################################################################
IF(NOT HEADLESS_BUILD)

  IF (NOT MSVC_VERSION VERSION_LESS 1900)
    MESSAGE("Configuring for Qt5")
	#SET(CMAKE_AUTOMOC ON)
    FIND_PACKAGE(Qt5 COMPONENTS Core Gui Widgets)
  ELSE()

  IF (QWT_MAJOR_VERSION VERSION_LESS 6)
    FIND_PACKAGE(Qt4 REQUIRED)
    INCLUDE(${QT_USE_FILE})
  ELSE()
    FIND_PACKAGE(Qt5 COMPONENTS Core Gui Widgets REQUIRED)
  ENDIF()

  ENDIF()

ENDIF(NOT HEADLESS_BUILD)

################################################################################
#
# Eigen3 math library
# 
################################################################################
IF(USE_EIGEN3)
  FIND_PACKAGE (Eigen3 3.2.0 QUIET)

  IF(NOT Eigen3_FOUND)
    MESSAGE("-- Using Eigen3 from Rcs external directory")
    SET(EIGEN3_INCLUDE_DIR ${RCS_THIRDPARTY_DIR}/Eigen)
  ENDIF()

ENDIF()
