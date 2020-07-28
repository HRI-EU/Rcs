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

  # Install the binaries
  INSTALL(TARGETS ExampleHighGui ExamplePrioritizedIK ExampleForwardKinematics ExampleKinetics RUNTIME DESTINATION bin LIBRARY DESTINATION lib)

ENDIF(NOT HEADLESS_BUILD)
