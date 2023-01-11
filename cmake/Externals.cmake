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
  
ELSEIF(USE_BULLET STREQUAL 3.21_double)

  SET(BULLET_FOUND 1)
  SET(BULLET_ROOT_DIR     "${HGR}/External/Bullet/3.21" )
  SET(BULLET_USE_FILE     "lib/cmake/bullet/UseBullet.cmake" )
  SET(BULLET_INCLUDE_DIR  "${BULLET_ROOT_DIR}/include/bullet" )
  SET(BULLET_INCLUDE_DIRS "${BULLET_ROOT_DIR}/include/bullet" )
  SET(BULLET_LIBRARIES
    ${BULLET_ROOT_DIR}/bin/LinearMath_vs2010_x64_release.lib
    ${BULLET_ROOT_DIR}/bin/Bullet3Common_vs2010_x64_release.lib
    #${BULLET_ROOT_DIR}/bin/libBulletInverseDynamics.so
    ${BULLET_ROOT_DIR}/bin/BulletCollision_vs2010_x64_release.lib
    ${BULLET_ROOT_DIR}/bin/BulletDynamics_vs2010_x64_release.lib
    ${BULLET_ROOT_DIR}/bin/BulletSoftBody_vs2010_x64_release.lib)
  SET(BULLET_VERSION_STRING "3.21" )
  SET(BULLET_DEFINITIONS -DUSE_BULLET -DBT_USE_DOUBLE_PRECISION)

ENDIF()
