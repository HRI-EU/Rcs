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

#include "CapsuleNode.h"
#include "Rcs_graphicsUtils.h"
#include "Rcs_macros.h"

#include <osg/Geode>
#include <osg/ShapeDrawable>

/*

When at some point moving up to osg version 3.6.5, there is a convenience
method to create geometries from shapes:

#include <osg/Version>

#if OSG_VERSION_GREATER_OR_EQUAL(3,6,5)
  osg::ref_ptr<osg::TessellationHints> hints = new osg::TessellationHints;
  hints->setDetailRatio(2.0);
  osg::convertShapeToGeometry(*capsule.get(), hints.get(), osg::Vec4(0.3,0.3,0.3,1.0), osg::Array::BIND_PER_VERTEX);
#endif
*/

namespace Rcs
{

CapsuleGeometry::CapsuleGeometry(double r, double l, bool resizeable, unsigned int seg) :
  radius(r), length(l)
{
  setName("CapsuleGeometry");

  if (resizeable)
  {
    initMesh(seg);
  }
  else
  {
    initPrimitive();
  }

}

void CapsuleGeometry::initMesh(unsigned int seg)
{
  RcsMeshData* mesh;

  // Capsule hull
  {
    mesh = RcsMesh_createCylinderHull(1.0, 1.0, 1.0, seg, 1, 2.0*M_PI);
    RcsMesh_shift(mesh, 0.0, 0.0, 0.5);
    osg::ref_ptr<osg::Geometry> hullMesh = createGeometryFromMesh2(mesh);
    hullMesh->setUseDisplayList(false);
    hullMesh->setUseVertexBufferObjects(true);
    hullMesh->setDataVariance(osg::Object::DYNAMIC);
    osg::ref_ptr<osg::Geode> hullGeo = new osg::Geode;
    osg::StateSet* hullSS = hullGeo->getOrCreateStateSet();
    hullSS->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
    hullSS->setMode(GL_RESCALE_NORMAL, osg::StateAttribute::ON);
    hullGeo->addDrawable(hullMesh.get());
    this->hull = new osg::PositionAttitudeTransform;
    hull->addChild(hullGeo.get());
    addChild(hull.get());
    RcsMesh_destroy(mesh);
  }

  // Top cap
  {
    mesh = RcsMesh_createSphereSegment(1.0, seg/2, seg/2, 0.0, M_PI, 0.0, M_PI);
    osg::ref_ptr<osg::Geometry> topMesh = createGeometryFromMesh2(mesh);
    topMesh->setUseDisplayList(false);
    topMesh->setUseVertexBufferObjects(true);
    topMesh->setDataVariance(osg::Object::DYNAMIC);
    osg::ref_ptr<osg::Geode> topGeo = new osg::Geode;
    osg::StateSet* topSS = topGeo->getOrCreateStateSet();
    topSS->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
    topSS->setMode(GL_RESCALE_NORMAL, osg::StateAttribute::ON);
    topGeo->addDrawable(topMesh.get());
    this->top = new osg::PositionAttitudeTransform;
    top->addChild(topGeo.get());
    addChild(top.get());
    RcsMesh_destroy(mesh);
  }

  // Bottom cap
  {
    mesh = RcsMesh_createSphereSegment(1.0, seg/2, seg/2, M_PI, M_PI, 0.0, M_PI);
    osg::ref_ptr<osg::Geometry> bottomMesh = createGeometryFromMesh2(mesh);
    bottomMesh->setUseDisplayList(false);
    bottomMesh->setUseVertexBufferObjects(true);
    bottomMesh->setDataVariance(osg::Object::DYNAMIC);
    osg::ref_ptr<osg::Geode> bottomGeo = new osg::Geode;
    osg::StateSet* bottomSS = bottomGeo->getOrCreateStateSet();
    bottomSS->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
    bottomSS->setMode(GL_RESCALE_NORMAL, osg::StateAttribute::ON);
    bottomGeo->addDrawable(bottomMesh.get());
    this->bottom = new osg::PositionAttitudeTransform;
    bottom->addChild(bottomGeo.get());
    addChild(bottom.get());
    RcsMesh_destroy(mesh);
  }

  hull->setScale(osg::Vec3(radius, radius, length));
  top->setPosition(osg::Vec3(0.0, 0.0, length));
  bottom->setScale(osg::Vec3(radius, radius, radius));
  top->setScale(osg::Vec3(radius, radius, radius));
}

void CapsuleGeometry::initPrimitive()
{
  osg::ref_ptr<osg::Capsule> capsule = new osg::Capsule();
  capsule->setRadius(radius);
  capsule->setHeight(length);
  capsule->setCenter(osg::Vec3(0.0, 0.0, 0.5*length));
  osg::ref_ptr<osg::ShapeDrawable> shape = new osg::ShapeDrawable(capsule.get());
  osg::ref_ptr<osg::Geode> geode = new osg::Geode();
  geode->addDrawable(shape.get());
  addChild(geode.get());
}

CapsuleGeometry::~CapsuleGeometry()
{
}

void CapsuleGeometry::update(double r, double l)
{
  // Only update if the dimensions changed
  if ((radius == r) && (length == l))
  {
    return;
  }

  // Only update once constructed with meshes.
  if ((!top.valid()) || (!bottom.valid()) || (!hull.valid()))
  {
    return;
  }

  hull->setScale(osg::Vec3(r, r, l));

  if (length != l)
  {
    top->setPosition(osg::Vec3(0.0, 0.0, l));
  }

  if (radius != r)
  {
    bottom->setScale(osg::Vec3(r, r, r));
    top->setScale(osg::Vec3(r, r, r));
  }

  radius = r;
  length = l;
}

double CapsuleGeometry::getRadius() const
{
  return this->radius;
}

double CapsuleGeometry::getLength() const
{
  return this->length;
}

















CapsuleNode::CapsuleNode(const double ballPoint[3], double A_KI[3][3],
                         double radius, double length,
                         bool resizeable) : NodeBase()
{
  setName("CapsuleNode");
  this->capsule = new CapsuleGeometry(radius, length, resizeable);
  this->patPtr()->addChild(capsule.get());

  if (ballPoint != NULL)
  {
    setPosition(ballPoint);
  }

  if (A_KI != NULL)
  {
    setRotation(A_KI);
  }

  setWireframe(true);
}

void CapsuleNode::setLength(double l)
{
  capsule->update(capsule->getRadius(), l);
}

void CapsuleNode::setRadius(double r)
{
  capsule->update(r, capsule->getLength());
}


}   // namespace Rcs
