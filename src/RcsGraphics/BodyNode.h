/*******************************************************************************

  Copyright (c) 2017, Honda Research Institute Europe GmbH

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are
  met:

  1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
  IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#ifndef RCS_BODYNODE_H
#define RCS_BODYNODE_H

#include "ShapeNode.h"

#include <Rcs_graph.h>

#include <osg/Geode>
#include <osg/PositionAttitudeTransform>
#include <osg/ShapeDrawable>
#include <osg/Switch>
#include <osg/Material>
#include <osg/Texture2D>
#include <osgText/Text>



namespace Rcs
{

/*!
 * \ingroup RcsGraphics
 *
 * \brief Node to display an RcsBody data structure.
 *
 *        A visual representation of all shapes of the body is added to the
 *        node. Further, the BodyNode allows to display / hide / toggle
 *        different kinds of visualization modes: A graphics model, a physics
 *        model, a collision model and a depth model. These models can be
 *        defined in the bodie's xml description by graphics="false/true",
 *        physics="false/true", collision="false/true" and depth="false/true".
 *        There are also convenience keys available, see the documentation of
 *        the GraphNode class for details. The visualization of all coordinate
 *        frames can be toggled independently. The class also can show some
 *        debug information, such as the position of joints and bodies etc.
 *
 *        The class also supports resizeable shapes. Shapes can be made
 *        resizeable by setting the resizeable argument in the constructor, or
 *        by setting the shape's resizeable attribute in the xml description to
 *        true. In this case, the OpenGL display lists are enabled, and an
 *        update callback to update the geometries is enabled.
 *
 *        The name of the node corresponds to the name of the body.
 */
class BodyNode: public osg::PositionAttitudeTransform
{
  friend class BodyUpdateCallback;

public:

  BodyNode(const RcsBody* bdy, const RcsGraph* graph, float scale = 1.0f,
           bool resizeable = true);
  virtual const char* className() const;
  void setTransformation(const HTr* A_BI);
  void displayCollisionNode(bool visibility = true);
  void displayGraphicsNode(bool visibility = true);
  void displayPhysicsNode(bool visibility = true);
  void displayReferenceNode(bool visibility = true);
  void displayDepthNode(bool visibility = true);
  void displayDebugInformation(bool visible = true);
  void toggleCollisionNode();
  void toggleGraphicsNode();
  void togglePhysicsNode();
  void toggleReferenceNode();
  void toggleDepthNode();
  void toggleDebugInformation();
  bool collisionNodeVisible() const;
  bool graphicsNodeVisible() const;
  bool physicsNodeVisible() const;
  bool depthNodeVisible() const;
  bool referenceFramesVisible() const;
  bool debugInformationVisible() const;
  void setAlpha(float alpha);
  void setAlpha(float alpha, osg::Node* node);
  bool setTexture(std::string textureFile);
  const RcsBody* body() const;
  const RcsBody* parent() const;
  int bodyId() const;
  const HTr* getTransformPtr() const;
  void setTransformPtr(const HTr* A_BI);
  void hide();
  void show();
  void setVisibility(bool visible);
  void setGhostMode(bool enabled, const std::string& matname="");
  void setDynamicMeshUpdate(bool enabled);
  bool getDynamicMeshUpdate() const;
  void setMaterial(const std::string& material, double alpha=1.0);
  void setParent(const RcsBody* parent);

protected:

  void updateCallback(osg::Node* node, osg::NodeVisitor* nv);
  void updateDynamicMeshes();
  virtual ~BodyNode();
  osg::Switch* addDebugInformation(const RcsGraph* graph);

  const RcsGraph* graphPtr;
  const HTr* A_BI_;
  int bdyId;
  int parentId;
  bool ghostMode;
  bool dynamicMeshUpdate;
  bool refNode;
  bool initializeDebugInfo;
  std::vector<osg::ref_ptr<ShapeNode>> _shapeNodes;
  osg::ref_ptr<osg::Switch> _collisionNode;
  osg::ref_ptr<osg::Switch> _graphicsNode;
  osg::ref_ptr<osg::Switch> _physicsNode;
  osg::ref_ptr<osg::Switch> _depthNode;
  osg::ref_ptr<osg::Switch> _nodeSwitch;
  osg::ref_ptr<osg::Switch> _debugNode;
  osg::ref_ptr<osgText::Text> _debugText;
  osg::ref_ptr<osg::Vec3Array> _debugLine;
  osg::ref_ptr<osg::Geometry> _debugLineGeometry;
};



}   // namespace Rcs


#endif // RCS_BODYNODE_H
