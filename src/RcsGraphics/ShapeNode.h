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

#ifndef RCS_SHAPENODE_H
#define RCS_SHAPENODE_H

#include <Rcs_graph.h>
#include <NodeBase.h>

#include <osg/PositionAttitudeTransform>

#include <vector>


namespace Rcs
{

class ShapeNode : public osg::PositionAttitudeTransform
{
public:
  ShapeNode(const RcsShape* shape, bool resizeable);
  void displayFrames(bool visibility = true);
  void toggleFrames();
  void updateDynamicShapes();

protected:
  virtual ~ShapeNode();
  void addShape(bool resizeable);
  void addTexture(const char* textureFile);
  void setMaterial(const char* color, osg::Node* node);
  void addGeometry(osg::Shape* s);
  void addDrawable(osg::Drawable* d);

  struct ShapeUpdater : public osg::Referenced
  {
    ShapeUpdater(ShapeNode* node);
    void addGeometry(osg::Shape* s);
    void addDrawable(osg::Drawable* d);
    const RcsShape* shape();
    void updateDynamicShapes();

    ShapeNode* shapeNode;
    std::vector<osg::Shape*> geometry;
    std::vector<osg::Drawable*> drawable;
    double extents[3];
    HTr A_CB;
  };

  const RcsShape* shape;
  std::vector<osg::ref_ptr<NodeBase>> frames;
  osg::ref_ptr<ShapeUpdater> shapeUpdater;
};




}   // namespace Rcs


#endif // RCS_SHAPENODE_H
