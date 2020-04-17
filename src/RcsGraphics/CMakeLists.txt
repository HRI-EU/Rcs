###############################################################################
#
# CMakeLists file for the RcsGraphics library
#
###############################################################################
SET(RCS_GRAPHICS_SRCS
ArrowNode.cpp
BodyNode.cpp
BodyPointDragger.cpp
BoxNode.cpp
CapsuleNode.cpp
COSNode.cpp
CylinderNode.cpp
DepthRenderer.cpp
FTSensorNode.cpp
GraphNode.cpp
HUD.cpp
KeyCatcher.cpp
MeshNode.cpp
MouseDragger.cpp
NodeBase.cpp
PPSSensorNode.cpp
Rcs_graphicsUtils.cpp
RcsViewer.cpp
RigidBodyTracker.cpp
SphereNode.cpp
SSRNode.cpp
TargetSetter.cpp
TextNode3D.cpp
TorusNode.cpp
TransformTrace.cpp
VectorFieldNode.cpp
VertexArrayNode.cpp
)

ADD_LIBRARY(RcsGraphics ${LIBRARY_LINKING_TYPE} ${RCS_GRAPHICS_SRCS})

TARGET_INCLUDE_DIRECTORIES(RcsGraphics PUBLIC
    # Header source location
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    
    # Header install location
    $<INSTALL_INTERFACE:include/RcsGraphics>
    
    # Regular dependencies
    ${OPENSCENEGRAPH_INCLUDE_DIRS}
)

IF(USE_OCTOMAP)
  TARGET_INCLUDE_DIRECTORIES(RcsGraphics SYSTEM PRIVATE ${OCTOMAP_INCLUDE_DIR})
  TARGET_COMPILE_DEFINITIONS(RcsGraphics PRIVATE ${OCTOMAP_DEFINITIONS})
ENDIF(USE_OCTOMAP)

TARGET_LINK_LIBRARIES(RcsGraphics RcsCore ${OPENSCENEGRAPH_LIBRARIES})

# Mark for export
RCS_ADD_EXPORT_LIBRARY(RcsGraphics)

# Installation
# Headers
INSTALL(
  DIRECTORY . 
  DESTINATION include/RcsGraphics
  FILES_MATCHING PATTERN "*.h"
)
# Install the binaries
INSTALL(TARGETS RcsGraphics EXPORT RcsExport DESTINATION lib)
