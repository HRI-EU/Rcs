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
    SET(BULLET_INCLUDE_DIR ${HGR}/External/Bullet/2.83/include CACHE PATH "Bullet include directory")
  ELSE()
    # SIT not available, path must be set
    SET(BT_LIB_DIR "" CACHE PATH "Bullet library directory")
    SET(BULLET_INCLUDE_DIR "" CACHE PATH "Bullet include directory")
  ENDIF()

  SET(BULLET_DEFINITIONS -DUSE_BULLET -DBT_USE_DOUBLE_PRECISION)
  SET(BULLET_CXX_FLAGS "-isystem ${BULLET_INCLUDE_DIR}")

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
  
ELSEIF(USE_BULLET STREQUAL latest)

  IF(DEFINED ENV{SIT})
    # SIT available, use that version by default
    SET(BT_LIB_DIR ${HGR}/External/bullet3/lib)
    SET(BULLET_INCLUDE_DIR ${HGR}/External/bullet3/include/bullet)
  ELSE()
    # SIT not available, path must be set
    SET(BT_LIB_DIR "" CACHE PATH "Bullet library directory")
    SET(BULLET_INCLUDE_DIR "" CACHE PATH "Bullet include directory")
  ENDIF()

  SET(BULLET_DEFINITIONS -DUSE_BULLET -DBT_USE_DOUBLE_PRECISION)

  SET(BULLET_LIBRARIES
    ${BT_LIB_DIR}/libLinearMath.so 
    ${BT_LIB_DIR}/libBulletCollision.so
    ${BT_LIB_DIR}/libBulletDynamics.so 
    ${BT_LIB_DIR}/libBulletSoftBody.so 
    )
  
ELSEIF(USE_BULLET STREQUAL 2.89_double)

  SET(BULLET_FOUND 1)
  SET(BULLET_ROOT_DIR     "${HGR}/External/Bullet/2.89" )
  SET(BULLET_USE_FILE     "lib/cmake/bullet/UseBullet.cmake" )
  SET(BULLET_INCLUDE_DIR  "${BULLET_ROOT_DIR}/include/bullet" )
  SET(BULLET_INCLUDE_DIRS "${BULLET_ROOT_DIR}/include/bullet" )
  SET(BULLET_LIBRARIES
    ${BULLET_ROOT_DIR}/lib/libLinearMath.so
    ${BULLET_ROOT_DIR}/lib/libBullet3Common.so
    ${BULLET_ROOT_DIR}/lib/libBulletInverseDynamics.so
    ${BULLET_ROOT_DIR}/lib/libBulletCollision.so
    ${BULLET_ROOT_DIR}/lib/libBulletDynamics.so
    ${BULLET_ROOT_DIR}/lib/libBulletSoftBody.so)
  SET(BULLET_VERSION_STRING "2.89" )
  SET(BULLET_DEFINITIONS -DUSE_BULLET -DBT_USE_DOUBLE_PRECISION)

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
