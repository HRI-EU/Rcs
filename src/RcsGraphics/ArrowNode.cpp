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

#include "ArrowNode.h"
#include "RcsViewer.h"   // Shadow cast masks are defined in RcsViewer
#include "Rcs_graphicsUtils.h"

#include <Rcs_Vec3d.h>
#include <Rcs_Mat3d.h>

#include <osg/Vec3d>
#include <osg/Geode>



/*******************************************************************************
 * Constructor.
 ******************************************************************************/
Rcs::ArrowNode::ArrowNode() :
  NodeBase(),
  originPtr(staticOrigin),
  directionPtr(staticDirection),
  offsetPtr(Vec3d_zeroVec()),
  scaleFactor(1.0),
  radius(0.005)
{
  init(0.0, radius, "BLUE");
}

/*******************************************************************************
 * Constructor.
 ******************************************************************************/
Rcs::ArrowNode::ArrowNode(const double* org,
                          const double* dir,
                          double scale,
                          const double* offset,
                          const std::string& color,
                          double radius_) :
  NodeBase(),
  offsetPtr(offset),
  scaleFactor(scale)
{
  this->originPtr = org ? org : this->staticOrigin;
  this->directionPtr = dir ? dir : this->staticDirection;
  this->radius = fabs(radius_);

  init(Vec3d_getLength(this->directionPtr), radius, color);

  makeDynamic();
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::ArrowNode::init(double length, double radius,
                          const std::string& color)
{
  Vec3d_setZero(this->staticOrigin);
  Vec3d_setUnitVector(this->staticDirection, 2);

  float coneHeight = 3.0*radius;
  osg::ref_ptr<osg::Geode> geode = new osg::Geode();
  osg::ref_ptr<osg::TessellationHints> hints = new osg::TessellationHints;
  hints->setDetailRatio(0.5f);
  length *= this->scaleFactor;

  // Sphere at arrow origin
  this->centerSphere = new osg::Sphere(osg::Vec3(0.0, 0.0, 0.0), 2.0*radius);
  osg::ref_ptr<osg::ShapeDrawable> shape1 = new osg::ShapeDrawable(centerSphere.get(), hints.get());
  //shape1->setColor(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f));
  shape1->setUseDisplayList(false);
  geode->addDrawable(shape1.get());

  // Arrow cylinder part
  this->cylZ = new osg::Cylinder(osg::Vec3(0.0, 0.0, length/2.0), radius, length);
  osg::ref_ptr<osg::ShapeDrawable> shape2 = new osg::ShapeDrawable(cylZ.get(), hints.get());
  //shape2->setColor(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f));
  shape2->setUseDisplayList(false);
  geode->addDrawable(shape2.get());

  // Cone tip
  this->coneZ =
    new osg::Cone(osg::Vec3(0.0f, 0.0f, length), 3.0 * radius, coneHeight);
  osg::ref_ptr<osg::ShapeDrawable> shape3 = new osg::ShapeDrawable(coneZ.get(), hints.get());
  //shape3->setColor(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f));
  shape3->setUseDisplayList(false);
  geode->addDrawable(shape3.get());

  setNodeMaterial(color, geode);

  // ArrowNodes neither cast nor receive shadows
  geode->setNodeMask(geode->getNodeMask() & ~CastsShadowTraversalMask);
  geode->setNodeMask(geode->getNodeMask() & ~ReceivesShadowTraversalMask);

  // Add the geode to the node
  this->patPtr()->addChild(geode.get());
}

/*******************************************************************************
 *
 ******************************************************************************/
const double* Rcs::ArrowNode::origin() const
{
  return this->originPtr;
}

/*******************************************************************************
 *
 ******************************************************************************/
const double* Rcs::ArrowNode::direction() const
{
  return this->directionPtr;
}

/*******************************************************************************
 *
 ******************************************************************************/
const double* Rcs::ArrowNode::offset() const
{
  return this->offsetPtr;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::ArrowNode::setOriginPtr(const double* org)
{
  this->originPtr = org ? org : Vec3d_zeroVec();
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::ArrowNode::setDirectionPtr(const double* dir)
{
  this->directionPtr = dir ? dir : Vec3d_ez();
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::ArrowNode::setOrigin(const double org[3])
{
  Vec3d_copy(this->staticOrigin, org);
  setOriginPtr(this->staticOrigin);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::ArrowNode::setDirection(const double dir[3])
{
  Vec3d_copy(this->staticDirection, dir);
  setDirectionPtr(this->staticDirection);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::ArrowNode::setArrowLength(double length)
{
  length *= this->scaleFactor;
  this->cylZ->setCenter(osg::Vec3(0.0f, 0.0f, length/2.0));
  this->cylZ->setHeight(length);
  this->coneZ->setCenter(osg::Vec3(0.0f, 0.0f, length));
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::ArrowNode::setRadius(double r)
{
  this->radius = r;
  this->centerSphere->setRadius(2.0*r);
  this->cylZ->setRadius(r);
  this->coneZ->setRadius(3.0*r);
  this->coneZ->setHeight(3.0*r);
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::ArrowNode::frameCallback()
{
  double A[3][3], dir[3], org[3];

  // Here we copy the direction and origin pointers, so that it is
  // ensured it doesn't change inside the Mat3d_fromVec() function.
  Vec3d_copy(dir, direction());
  Vec3d_normalizeSelf(dir);
  Vec3d_copy(org, origin());

  // add optional offset
  if (offset())
  {
    Vec3d_addSelf(org, offset());
  }

  if (getPosPtr())
  {
    Vec3d_addSelf(org, getPosPtr());
  }

  Mat3d_fromVec(A, dir, 2);

  if (getRotMatPtr())
  {
    const double* rm = getRotMatPtr();
    Mat3d_postMulSelf(A, (double (*)[3]) rm);
  }

  setTransformation(org, A);
  setArrowLength(Vec3d_getLength(dir));

  return false;
}
