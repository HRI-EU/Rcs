ADD_EXECUTABLE(UtilTests RcsCore.cpp)
TARGET_LINK_LIBRARIES(UtilTests RcsCore)

ADD_EXECUTABLE(MathTests MathTests.cpp)
TARGET_LINK_LIBRARIES(MathTests RcsCore)

ADD_EXECUTABLE(GraphTests RcsGraph.cpp)
TARGET_LINK_LIBRARIES(GraphTests RcsCore)

IF(NOT HEADLESS_BUILD)

  ADD_EXECUTABLE(Rcs Rcs.cpp)
  TARGET_LINK_LIBRARIES(Rcs RcsCore RcsGui RcsGraphics RcsPhysics)

  ADD_EXECUTABLE(TestGraphics TestGraphics.cpp)
  TARGET_LINK_LIBRARIES(TestGraphics RcsGraphics)

ENDIF(NOT HEADLESS_BUILD)