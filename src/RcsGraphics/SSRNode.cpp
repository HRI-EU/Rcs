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

#include "SSRNode.h"
#include "Rcs_graphicsUtils.h"

#include <Rcs_Vec3d.h>

#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Geometry>



namespace Rcs
{

SSRGeometry::SSRGeometry(const double xyz[3], unsigned int nSegments) :
  mesh(NULL), nSeg(nSegments)
{
  Vec3d_copy(this->extents, xyz);
  this->mesh = RcsMesh_createSSR(xyz, nSegments);
  createGeometryFromMesh2(this, mesh);
}

SSRGeometry::~SSRGeometry()
{
  RcsMesh_destroy(this->mesh);
}

void SSRGeometry::update(const double xyz[3])
{
  if (!Vec3d_isEqual(extents, xyz, 0.0))
  {
    Vec3d_copy(this->extents, xyz);
    RcsMesh_copySSR(mesh, extents, nSeg);
    updateGeometryFromMesh2(this, mesh);
    dirtyBound();
  }
}





SSRNode::SSRNode(const double center[3], double A_KI[3][3],
                 const double extent[2], const double r,
                 bool resizeable) : NodeBase()
{
  setName("SSRNode");

  double xyz[3];
  Vec3d_set(xyz, extent[0], extent[1], 2.0*r);
  osg::ref_ptr<osg::Geometry> g = new SSRGeometry(xyz, 16);
  g->setUseDisplayList(!resizeable);
  osg::ref_ptr<osg::Geode> geode = new osg::Geode();
  geode->addDrawable(g.get());
  this->patPtr()->addChild(geode.get());
  setPosition(center);   // Can cope with NULL
  setRotation(A_KI);
}

}   // namespace Rcs
