ADD_EXECUTABLE(UtilTests RcsCore.cpp)
TARGET_LINK_LIBRARIES(UtilTests RcsCore)

ADD_EXECUTABLE(TestGraphics TestGraphics.cpp)
TARGET_LINK_LIBRARIES(TestGraphics RcsGraphics)

ADD_EXECUTABLE(MathTests MathTests.cpp)
TARGET_LINK_LIBRARIES(MathTests RcsCore)

ADD_EXECUTABLE(GraphTests RcsGraph.cpp)
TARGET_LINK_LIBRARIES(GraphTests RcsCore)

ADD_EXECUTABLE(Rcs Rcs.cpp)
TARGET_LINK_LIBRARIES(Rcs RcsCore RcsGui RcsGraphics RcsPhysics)

# ADD_EXECUTABLE(TestVia TestVia.cpp)
# TARGET_LINK_LIBRARIES(TestVia RcsPhysics RcsGui RcsTrajectory)

# ADD_EXECUTABLE(BallOnPlate BallOnPlate.cpp)
# TARGET_LINK_LIBRARIES(BallOnPlate RcsPhysics RcsGui)
