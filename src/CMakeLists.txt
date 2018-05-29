ADD_SUBDIRECTORY (RcsCore)

IF(NOT HEADLESS_BUILD)
  ADD_SUBDIRECTORY (RcsGraphics)
  ADD_SUBDIRECTORY (RcsGui)
ENDIF()

ADD_SUBDIRECTORY (RcsPhysics)

IF(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/RcsTrajectory")
  MESSAGE("-- Including RcsTrajectory directory" )
  ADD_SUBDIRECTORY (RcsTrajectory)
ENDIF()

IF(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/RcsComponent")
  MESSAGE("-- Including RcsComponent directory" )
  ADD_SUBDIRECTORY (RcsComponent)
ENDIF()

IF(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/RcsSearch")
  MESSAGE("-- Including RcsSearch directory" )
  ADD_SUBDIRECTORY (RcsSearch)
ENDIF()

