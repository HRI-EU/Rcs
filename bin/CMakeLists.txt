ADD_EXECUTABLE(TestCore TestCore.cpp)
TARGET_LINK_LIBRARIES(TestCore RcsCore)

ADD_EXECUTABLE(TestMath TestMath.cpp)
TARGET_LINK_LIBRARIES(TestMath RcsCore)

ADD_EXECUTABLE(TestGraph TestGraph.cpp)
TARGET_LINK_LIBRARIES(TestGraph RcsCore)
IF(ENABLE_C++11)
  TARGET_COMPILE_DEFINITIONS(TestGraph PRIVATE -DUSE_CPP_11)
ENDIF(ENABLE_C++11)

IF(NOT HEADLESS_BUILD)

  ADD_EXECUTABLE(Rcs Rcs.cpp)
  TARGET_LINK_LIBRARIES(Rcs RcsCore RcsGui RcsGraphics RcsPhysics)

  ADD_EXECUTABLE(TestGraphics TestGraphics.cpp)
  TARGET_LINK_LIBRARIES(TestGraphics RcsGraphics)

  ADD_EXECUTABLE(TestGui TestGui.cpp)
  TARGET_LINK_LIBRARIES(TestGui RcsGui)

  ADD_EXECUTABLE(TestOrientations TestOrientations.cpp)
  TARGET_LINK_LIBRARIES(TestOrientations RcsCore RcsGui RcsGraphics)

ENDIF(NOT HEADLESS_BUILD)