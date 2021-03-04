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

#include "VectorFieldNode.h"
#include "RcsViewer.h"
#include "Rcs_graphicsUtils.h"

#include <Rcs_macros.h>

#include <osg/Geode>
#include <osg/Point>
#include <osg/LineWidth>

namespace Rcs
{


/****************************************************************************

  \brief Update callback: Sets the bodies transformation to the node.

****************************************************************************/

class VectorFieldUpdateCallback : public osg::NodeCallback
{

public:

  VectorFieldUpdateCallback(VectorFieldNode* node)
  {
    RCHECK(node);
    this->node = node;
  }

  virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
  {
    const HTr* A = this->node->getTransform();

    this->node->setTransformation(A);

    // traverse subtree
    traverse(node, nv);
  }

protected:
  VectorFieldNode* node;
};


VectorFieldNode::VectorFieldNode(const MatNd* xdx, const HTr* refFrame, double scale, bool withPoints, const char* color)
{
  setName("VectorFieldNode");
  this->refFrame = refFrame;
  this->points = new osg::Vec3Array;
  this->lines = new osg::Vec3Array;

  for (unsigned int i = 0; i < xdx->m; i++)
  {
    double* row = MatNd_getRowPtr(xdx, i);

    if (withPoints)
    {
      this->points->push_back(osg::Vec3(row[0], row[1], row[2]));
    }

    this->lines->push_back(osg::Vec3(row[0], row[1], row[2]));
    this->lines->push_back(osg::Vec3(row[0] + row[3] * scale, row[1] + row[4] * scale, row[2] + row[5] * scale));
  }

  init(color);
}

VectorFieldNode::VectorFieldNode(const MatNd* x, const MatNd* dx, const HTr* refFrame, double scale, bool withPoints, const char* color)
{
  setName("VectorFieldNode");
  this->refFrame = refFrame;
  this->points = new osg::Vec3Array;
  this->lines = new osg::Vec3Array;

  for (unsigned int i = 0; i < x->m; i++)
  {
    double* rowx = MatNd_getRowPtr(x, i);
    double* rowdx = MatNd_getRowPtr(dx, i);

    if (withPoints)
    {
      this->points->push_back(osg::Vec3(rowx[0], rowx[1], rowx[2]));
    }

    this->lines->push_back(osg::Vec3(rowx[0], rowx[1], rowx[2]));
    this->lines->push_back(osg::Vec3(rowx[0] + rowdx[0]*scale, rowx[1] + rowdx[1]*scale, rowx[2] + rowdx[2]*scale));
  }

  init(color);

}

VectorFieldNode::~VectorFieldNode()
{

}


void VectorFieldNode::setTransformation(const HTr* A_KI)
{
  if (A_KI)
  {
    setPosition(osg::Vec3(A_KI->org[0], A_KI->org[1], A_KI->org[2]));

    osg::Quat qA;
    qA.set(osg::Matrix(A_KI->rot[0][0], A_KI->rot[0][1], A_KI->rot[0][2], 0.0,
                       A_KI->rot[1][0], A_KI->rot[1][1], A_KI->rot[1][2], 0.0,
                       A_KI->rot[2][0], A_KI->rot[2][1], A_KI->rot[2][2], 0.0,
                       0.0, 0.0, 0.0, 1.0));

    setAttitude(qA);
  }
  else
  {
    setPosition(osg::Vec3(0.0, 0.0, 0.0));

    osg::Quat qA;
    setAttitude(qA);
  }
}

const HTr* VectorFieldNode::getTransform() const
{
  return this->refFrame;
}

void VectorFieldNode::init(const char* color)
{
  // create the Geode (Geometry Node) to contain all our geometry objects.
  osg::ref_ptr<osg::Geode> geode = new osg::Geode();
  addChild(geode.get());
  geode->setNodeMask(geode->getNodeMask() & ~CastsShadowTraversalMask);
  geode->setNodeMask(geode->getNodeMask() & ~ReceivesShadowTraversalMask);
  setNodeMaterial(color, geode.get());


  // points geometry
  this->pointsGeom = new osg::Geometry;
  osg::DrawArrays* ps = new osg::DrawArrays(osg::PrimitiveSet::POINTS);
  ps->setCount(this->points->size());
  this->pointsGeom->setVertexArray(this->points.get());
  this->pointsGeom->addPrimitiveSet(ps);


  // Adjust the point and line size
  osg::StateSet* stateset = geode->getOrCreateStateSet();

  osg::Point* point = new osg::Point(4.0);
  stateset->setAttribute(point, osg::StateAttribute::ON);

  osg::LineWidth* line = new osg::LineWidth(1.0);
  stateset->setAttribute(line, osg::StateAttribute::ON);

  geode->setStateSet(stateset);

  // lines geometry
  this->linesGeom = new osg::Geometry;
  ps = new osg::DrawArrays(osg::PrimitiveSet::LINES);
  ps->setCount(this->lines->size());
  this->linesGeom->setVertexArray(this->lines.get());
  this->linesGeom->addPrimitiveSet(ps);

  // add the points geometry to the geode.
  geode->addDrawable(this->pointsGeom.get());

  // add the lines geometry to the geode.
  geode->addDrawable(this->linesGeom.get());

  //  // set the colors as before, plus using the above
  //  _colors = new osg::Vec4Array;
  //  _pointsGeom->setColorArray(_colors.get() );
  //  _pointsGeom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
  ////   _pointsGeom->setColorBinding(osg::Geometry::BIND_OVERALL);

  //  // set the normal in the same way color.
  //  osg::Vec3Array* normals = new osg::Vec3Array;
  //  normals->push_back(osg::Vec3(0.0f,-1.0f,0.0f));
  //  _pointsGeom->setNormalArray(normals);
  //  _pointsGeom->setNormalBinding(osg::Geometry::BIND_OVERALL);

  // Add event callback handler
  setEventCallback(new VectorFieldUpdateCallback(this));
}

}  // namespace Rcs
