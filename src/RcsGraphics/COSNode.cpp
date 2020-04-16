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

#include "COSNode.h"
#include "Rcs_graphicsUtils.h"

#include <Rcs_Mat3d.h>
#include <RcsViewer.h>

#include <osg/Geode>
#include <osg/ShapeDrawable>



/******************************************************************************

  \brief Constructor.

******************************************************************************/

Rcs::COSNode::COSNode(float scale, float lengthX, float lengthY,
                      float lengthZ) :
  NodeBase(), angMode(None)
{
  init(scale, lengthX, lengthY, lengthZ);
}


/******************************************************************************

  \brief Constructor.

******************************************************************************/

Rcs::COSNode::COSNode(const double* pos, float scale, float lengthX,
                      float lengthY,
                      float lengthZ) :
  NodeBase(), angMode(None)
{
  setPosPtr(pos);
  init(scale, lengthX, lengthY, lengthZ);
  makeDynamic();
}


/******************************************************************************

  \brief Constructor.

******************************************************************************/

Rcs::COSNode::COSNode(const double* pos, const double* rot, float scale,
                      AngularMode mode, float lengthX, float lengthY,
                      float lengthZ) :
  NodeBase(), angMode(mode)
{
  setPosPtr(pos);
  setRotMatPtr(rot);
  init(scale, lengthX, lengthY, lengthZ);
  makeDynamic();
}


/******************************************************************************

  \brief Constructor.

******************************************************************************/

Rcs::COSNode::COSNode(const HTr* A_CI, float scale, float lengthX,
                      float lengthY, float lengthZ) :
  NodeBase(), angMode(RotMat)
{
  setPosPtr(A_CI->org);
  setRotMatPtr((double*)A_CI->rot);
  init(scale, lengthX, lengthY, lengthZ);
  makeDynamic();
}


/******************************************************************************

  \brief See header.

******************************************************************************/

void Rcs::COSNode::init(float scale, float lengthX, float lengthY,
                        float lengthZ)
{
  setName("COSNode");

  float radius = 0.015f;
  float coneHeight = 0.1f;
  HTr A_KI;
  HTr_setIdentity(&A_KI);
  osg::Geode* geode = new osg::Geode();
  osg::TessellationHints* hints = new osg::TessellationHints;
  hints->setDetailRatio(0.2f);

  // Center sphere
  osg::Sphere* centerSphere =
    new osg::Sphere(osg::Vec3(0.0f, 0.0f, 0.0f), 2.0 * radius);
  osg::ShapeDrawable* shape1 = new osg::ShapeDrawable(centerSphere, hints);
  shape1->setColor(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
  geode->addDrawable(shape1);

  // z-axis
  osg::Cylinder* cylZ =
    new osg::Cylinder(osg::Vec3(0.0f, 0.0f, lengthZ / 2.), radius, lengthZ);
  osg::ShapeDrawable* shape2 = new osg::ShapeDrawable(cylZ, hints);
  shape2->setColor(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f));
  geode->addDrawable(shape2);

  // Cone z-axis
  osg::Cone* coneZ =
    new osg::Cone(osg::Vec3(0.0f, 0.0f, lengthZ), 3.0 * radius, coneHeight);
  osg::ShapeDrawable* shape3 = new osg::ShapeDrawable(coneZ, hints);
  shape3->setColor(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f));
  geode->addDrawable(shape3);

  // Cylinder x-axis
  osg::Cylinder* cylX =
    new osg::Cylinder(osg::Vec3(lengthX / 2.0f, 0.0f, 0.0f), radius, lengthX);
  Mat3d_setRotMatY(A_KI.rot, M_PI_2);
  A_KI.org[0] = 0.5;
  A_KI.org[1] = 0.0;
  A_KI.org[2] = 0.0;
  cylX->setRotation(QuatFromHTr(&A_KI));
  osg::ShapeDrawable* shape4 = new osg::ShapeDrawable(cylX, hints);
  shape4->setColor(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
  geode->addDrawable(shape4);

  // Cone x-axis
  osg::Cone* coneX =
    new osg::Cone(osg::Vec3(lengthX, 0.0f, 0.0f), 3.0 * radius, coneHeight);
  coneX->setRotation(QuatFromHTr(&A_KI));
  osg::ShapeDrawable* shape5 = new osg::ShapeDrawable(coneX, hints);
  shape5->setColor(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
  geode->addDrawable(shape5);

  // Cylinder y-axis
  osg::Cylinder* cylY =
    new osg::Cylinder(osg::Vec3(0.0f, lengthY / 2.0f, 0.0f), radius, lengthY);
  Mat3d_setRotMatX(A_KI.rot, -M_PI_2);
  cylY->setRotation(QuatFromHTr(&A_KI));
  osg::ShapeDrawable* shape6 = new osg::ShapeDrawable(cylY, hints);
  shape6->setColor(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
  geode->addDrawable(shape6);

  // Cone y-axis
  osg::Cone* coneY =
    new osg::Cone(osg::Vec3(0.0f, lengthY, 0.0f), 3.0 * radius, coneHeight);
  coneY->setRotation(QuatFromHTr(&A_KI));
  osg::ShapeDrawable* shape7 = new osg::ShapeDrawable(coneY, hints);
  shape7->setColor(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
  geode->addDrawable(shape7);

  // COSNodes neither cast nor receive shadows
  geode->setNodeMask(geode->getNodeMask() & ~CastsShadowTraversalMask);
  geode->setNodeMask(geode->getNodeMask() & ~ReceivesShadowTraversalMask);

  // Add the geode to the node
  this->patPtr()->addChild(geode);

  // Apply scaling factor
  this->patPtr()->setScale(osg::Vec3(scale, scale, scale));

  // Make node ignore material changes to avoid weird colouring when forced
  // parent's material.
  osg::ref_ptr<osg::StateSet> stateset = getOrCreateStateSet();
  stateset->setMode(GL_LIGHTING,
                    osg::StateAttribute::PROTECTED |
                    osg::StateAttribute::OFF);
}


/******************************************************************************

  \brief See header.

******************************************************************************/

Rcs::COSNode::AngularMode Rcs::COSNode::getAngularMode() const
{
  return this->angMode;
}


/******************************************************************************

  \brief See header.

******************************************************************************/

bool Rcs::COSNode::frameCallback()
{
  switch (getAngularMode())
  {
    case Rcs::COSNode::RotMat:
    {
      setTransformation(getPosPtr(),
                        (double(*)[3]) getRotMatPtr());
      break;
    }

    case Rcs::COSNode::Euler:
    {
      double A[3][3];
      Mat3d_fromEulerAngles(A, getRotMatPtr());
      setTransformation(getPosPtr(), A);
      break;
    }

    default:
      setPosition(getPosPtr());
  }

  return false;
}
