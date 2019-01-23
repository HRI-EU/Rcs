/*******************************************************************************

  Copyright (c) 2017, Honda Research Institute Europe GmbH.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.

  3. All advertising materials mentioning features or use of this software
     must display the following acknowledgement: This product includes
     software developed by the Honda Research Institute Europe GmbH.

  4. Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER "AS IS" AND ANY EXPRESS OR
  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#ifndef RCS_BODYNODE_H
#define RCS_BODYNODE_H

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
 */
class BodyNode: public osg::PositionAttitudeTransform
{
  friend class BodyUpdateCallback;

public:

  BodyNode(const RcsBody* bdy, float scale = 1.0f, bool resizeable = true);
  virtual const char* className() const;
  void setTransformation(const HTr* A_BI);
  void displayCollisionNode(bool visibility = true);
  void displayGraphicsNode(bool visibility = true);
  void displayPhysicsNode(bool visibility = true);
  void displayReferenceNode(bool visibility = true);
  void displayDebugInformation(bool visible = true);
  void toggleCollisionNode();
  void toggleGraphicsNode();
  void togglePhysicsNode();
  void toggleReferenceNode();
  void toggleDebugInformation();
  bool collisionNodeVisible() const;
  bool graphicsNodeVisible() const;
  bool physicsNodeVisible() const;
  bool referenceFramesVisible() const;
  bool debugInformationVisible() const;
  void setAlpha(float alpha);
  void setAlpha(float alpha, osg::Node* node);
  bool setTexture(std::string textureFile);
  const RcsBody* body() const;
  const HTr* getTransformPtr() const;
  void setTransformPtr(const HTr* A_BI);
  void updateDynamicShapes();
  void hide();
  void setGhostMode(bool enabled, const std::string& matname="");

protected:

  virtual ~BodyNode();
  osg::Switch* addShapes(int mask);
  osg::Switch* addDebugInformation();

  const RcsBody* bdy;
  const HTr* A_BI;
  bool _resizeable;
  bool ghostMode;
  osg::ref_ptr<osg::Switch> _collisionNode;
  osg::ref_ptr<osg::Switch> _graphicsNode;
  osg::ref_ptr<osg::Switch> _physicsNode;
  osg::ref_ptr<osg::Switch> _refNode;
  std::vector<osg::ref_ptr<osg::Geode> > _dynamicShapes;
  osg::ref_ptr<osg::Switch> _debugNode;
  osg::ref_ptr<osgText::Text> _debugText;
  osg::ref_ptr<osg::Vec3Array> _debugLine;
  osg::ref_ptr<osg::Geometry> _debugLineGeometry;

  static char _fontFile[256];
  static bool _fontFileFound;
  static bool _fontFileSearched;
  static std::map<std::string, osg::ref_ptr<osg::Node> > _meshBuffer;
  static std::map<std::string, osg::ref_ptr<osg::Texture2D> > _textureBuffer;
};



}   // namespace Rcs


#endif // RCS_BODYNODE_H

