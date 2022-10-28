ADD_EXECUTABLE(ExamplePeriodicCallback ExamplePeriodicCallback.cpp)
TARGET_LINK_LIBRARIES(ExamplePeriodicCallback RcsCore)

IF(NOT HEADLESS_BUILD)

  ADD_EXECUTABLE(ExampleHighGui ExampleHighGui.cpp)
  TARGET_LINK_LIBRARIES(ExampleHighGui RcsCore RcsGui)

  ADD_EXECUTABLE(ExampleIK ExampleIK.cpp)
  TARGET_LINK_LIBRARIES(ExampleIK RcsCore RcsGui RcsGraphics RcsPhysics)

  ADD_EXECUTABLE(ExamplePrioritizedIK ExamplePrioritizedIK.cpp)
  TARGET_LINK_LIBRARIES(ExamplePrioritizedIK RcsCore RcsGui RcsGraphics)

  ADD_EXECUTABLE(ExampleForwardKinematics ExampleForwardKinematics.cpp)
  TARGET_LINK_LIBRARIES(ExampleForwardKinematics RcsCore RcsGui RcsGraphics)

  ADD_EXECUTABLE(ExampleKinetics ExampleKinetics.cpp)
  TARGET_LINK_LIBRARIES(ExampleKinetics RcsCore RcsGui RcsGraphics RcsPhysics)

  ADD_LIBRARY(RcsExamples ExampleFK.cpp ExampleInvKin.cpp ExampleMath.cpp ExamplePhysics.cpp ExampleDistance.cpp)
  TARGET_LINK_LIBRARIES(RcsExamples RcsGraphics RcsPhysics RcsGui)

  TARGET_INCLUDE_DIRECTORIES(RcsExamples PUBLIC
    # Header source location
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    
    # Header install location
    $<INSTALL_INTERFACE:include/RcsExamples>
  )
 
  # We add this option to force the linker to export the symbols. This
  # solves the issue that it will not be exposed, since all initialization 
  # happens inside statically. Under Unix, we can use -Wl,--no-as-needed
  IF (WIN32)
    TARGET_LINK_OPTIONS(RcsExamples PUBLIC /INCLUDE:ExampleInfo)
  ENDIF()

  # Mark for export
  RCS_ADD_EXPORT_LIBRARY(RcsExamples)

  # copy the .dll file to the same folder as the executable
 IF (WIN32)
  ADD_CUSTOM_COMMAND(
    TARGET RcsExamples POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
    ${LIBRARY_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}/RcsExamples.dll
    ${EXECUTABLE_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}/RcsExamples.dll)

    ADD_EXECUTABLE(ExampleRunnerWin WIN32 ExampleRunner.cpp)
    TARGET_LINK_LIBRARIES(ExampleRunnerWin RcsExamples)
    TARGET_COMPILE_OPTIONS(ExampleRunnerWin PRIVATE -DNO_CONSOLE -DRCS_NO_DEBUG)
    TARGET_LINK_OPTIONS(ExampleRunnerWin PRIVATE /entry:mainCRTStartup /subsystem:windows)
  ENDIF(WIN32)
  
  ADD_EXECUTABLE(ExampleRunner ExampleRunner.cpp)

  IF (UNIX)
    TARGET_LINK_LIBRARIES(ExampleRunner -Wl,--no-as-needed RcsExamples -Wl,--as-needed)
  ELSE()
    TARGET_LINK_LIBRARIES(ExampleRunner RcsExamples)
  ENDIF()

  
  

  IF(USE_BULLET)
    ADD_EXECUTABLE(ExampleBullet ExampleBullet.cpp)
    TARGET_LINK_LIBRARIES(ExampleBullet RcsPhysics)
    TARGET_INCLUDE_DIRECTORIES(ExampleBullet SYSTEM PRIVATE ${BULLET_INCLUDE_DIR})
    TARGET_COMPILE_DEFINITIONS(RcsPhysics PRIVATE ${BULLET_DEFINITIONS})
  ENDIF(USE_BULLET)


  # Install the headers
  INSTALL(
    DIRECTORY . 
    DESTINATION include/RcsExamples
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
    )

  # Install the binaries
  INSTALL(TARGETS ExampleRunner ExampleHighGui ExamplePrioritizedIK ExampleForwardKinematics ExampleKinetics RUNTIME DESTINATION bin LIBRARY DESTINATION lib)
  INSTALL(TARGETS RcsExamples EXPORT RcsExport DESTINATION lib)

ENDIF(NOT HEADLESS_BUILD)
