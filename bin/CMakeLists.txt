ADD_EXECUTABLE(TestCore TestCore.cpp)
TARGET_LINK_LIBRARIES(TestCore RcsCore)

ADD_EXECUTABLE(TestMath TestMath.cpp)
TARGET_LINK_LIBRARIES(TestMath RcsCore)

ADD_EXECUTABLE(TestGraph TestGraph.cpp)
TARGET_LINK_LIBRARIES(TestGraph RcsCore)

IF(NOT HEADLESS_BUILD)

  ADD_EXECUTABLE(Rcs Rcs.cpp)
  TARGET_LINK_LIBRARIES(Rcs RcsCore RcsGui RcsGraphics RcsPhysics)

  ADD_EXECUTABLE(TestGraphics TestGraphics.cpp)
  TARGET_LINK_LIBRARIES(TestGraphics RcsGraphics)

ENDIF(NOT HEADLESS_BUILD)