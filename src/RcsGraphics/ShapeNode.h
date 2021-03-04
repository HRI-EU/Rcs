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

#include <vector>


namespace Rcs
{

/*! \ingroup RcsGraphics
 *  \brief Node to display a RcsShape. The class has a flag to indicate if the
 *         node is resizeable or not. We do this since resizeable shapes often
 *         lead to a performance overhead when transferring data from CPU to
 *         GPU. If the node is updateable, the update function will be called
 *         during the viewer's update traversal, and it will check if the
 *         shape parameters have changed. Updating is done for the shape's
 *         relative transformation to the body (RcsShape::A_CB), the extents
 *         and the color.
 *
 *         Here is the node structure of this node:
 *
 *         ShapeNode (osg::PositionAttitudeTransform) relative to body
 *              |
 *              ---> geode (osg::Geode)
 *                        |
 *                        ---> sd (osg::Drawable)
 *                               |
 *                               ---> capsule, box, etc (osg::Capsule ...)
 *
 *        In a few cases (Meshes etc.), the Drawable is replaced by a
 *        osg::Geometry.
 */
class ShapeNode : public osg::PositionAttitudeTransform
{
public:
  ShapeNode(const RcsShape* shape, bool resizeable);
  void displayFrame(bool visibility = true);
  void toggleFrame();
  void updateDynamicShapes();

protected:
  virtual ~ShapeNode();
  void addShape(bool resizeable);
  bool addTexture(const char* textureFile);

  struct ShapeUpdater : public osg::Referenced
  {
    ShapeUpdater(ShapeNode* node);
    void addDrawable(osg::Drawable* d);
    void addSubNode(osg::Node* nd);
    const RcsShape* shape();
    void updateDynamicShapes();

    ShapeNode* shapeNode;
    osg::ref_ptr<osg::Drawable> drawable;
    osg::ref_ptr<osg::Node> subNode;
    double extents[3];
    HTr A_CB;
    std::string color;
  };

  const RcsShape* shape;
  osg::ref_ptr<NodeBase> frame;
  osg::ref_ptr<ShapeUpdater> shapeUpdater;
};




}   // namespace Rcs


#endif // RCS_SHAPENODE_H
