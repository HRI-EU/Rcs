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

#include "ExtrudedPolygonNode.h"

#include <Rcs_macros.h>

#include <RcsViewer.h>

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/ShadeModel>
#include <osg/Geometry>
#include <osgUtil/SmoothingVisitor>
#include <osgUtil/Tessellator>
#include <osgGA/StateSetManipulator>
#include <osgViewer/Viewer>
#include <osgUtil/Simplifier>

#include <cmath>


namespace Rcs
{

/*******************************************************************************
 * This function is under Public Domain.
 * From: https://github.com/xarray/osgRecipes (Public Domain)
 ******************************************************************************/
osg::Geometry* createExtrusion(osg::Vec3Array* vertices,
                               const osg::Vec3& direction)
{
  osg::ref_ptr<osg::Vec3Array> newVertices = new osg::Vec3Array;
  newVertices->insert(newVertices->begin(), vertices->begin(), vertices->end());

  unsigned int numVertices = vertices->size();
  osg::Vec3 offset = direction;
  for (osg::Vec3Array::reverse_iterator ritr=vertices->rbegin();
       ritr!=vertices->rend(); ++ritr)
  {
    newVertices->push_back((*ritr) + offset);
  }

  osg::ref_ptr<osg::Geometry> extrusion = new osg::Geometry;
  extrusion->setVertexArray(newVertices.get());
  extrusion->addPrimitiveSet(new osg::DrawArrays(GL_POLYGON, 0, numVertices));
  extrusion->addPrimitiveSet(new osg::DrawArrays(GL_POLYGON, numVertices,
                                                 numVertices));

  osgUtil::Tessellator tessellator;
  tessellator.setTessellationType(osgUtil::Tessellator::TESS_TYPE_POLYGONS);
  tessellator.setWindingType(osgUtil::Tessellator::TESS_WINDING_ODD);
  tessellator.retessellatePolygons(*extrusion);

  osg::ref_ptr<osg::DrawElementsUInt> sideIndices;
  sideIndices = new osg::DrawElementsUInt(GL_QUAD_STRIP);
  for (unsigned int i=0; i<numVertices; ++i)
  {
    sideIndices->push_back(i);
    sideIndices->push_back((numVertices-1-i) + numVertices);
  }
  sideIndices->push_back(0);
  sideIndices->push_back(numVertices*2 - 1);
  extrusion->addPrimitiveSet(sideIndices.get());

  // float sampleRatio = 1.0;
  // float maximumError = 0.1;
  // float maximumLength = 0.5;

  // osgUtil::Simplifier simplifier(sampleRatio, maximumError, maximumLength);
  // simplifier.simplify(*extrusion);

  osgUtil::SmoothingVisitor::smooth(*extrusion);

  return extrusion.release();
}


// class ExtrudedPolygonNode : public NodeBase
// {
// public:

ExtrudedPolygonNode::ExtrudedPolygonNode(const MatNd* poly2D,
                                         const double extrudeDir[3])
{
  osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
  for (unsigned int i=0; i<poly2D->m; i++)
  {
    const double* row = MatNd_getRowPtr(poly2D, i);
    vertices->push_back(osg::Vec3(row[0], row[1], 0.0));
  }

  osg::Vec3 dir(extrudeDir[0], extrudeDir[1], extrudeDir[2]);
  osg::Geometry* extrusion = createExtrusion(vertices.get(), dir);
  osg::ref_ptr<osg::Geode> geode = new osg::Geode();
  geode->addDrawable(extrusion);

  // Geometry casts shadows, but doesn't receive them
  geode->setNodeMask(geode->getNodeMask() & ~ReceivesShadowTraversalMask);

  this->patPtr()->addChild(geode.get());

  setMaterial("DARKRED_TRANS", 0.5);
}

ExtrudedPolygonNode::~ExtrudedPolygonNode()
{
}


// };


} // namespace Rcs
