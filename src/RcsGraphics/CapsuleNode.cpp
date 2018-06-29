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

#include <osg/Geode>
#include <osg/ShapeDrawable>



/******************************************************************************

  \brief Constructors.

******************************************************************************/

Rcs::CapsuleNode::CapsuleNode(const double ballPoint[3], double A_KI[3][3],
                              const double radius, const double length,
                              bool resizeable) : NodeBase()
{
  this->capsule = new osg::Capsule();
  capsule->setRadius(radius);
  capsule->setHeight(length);
  capsule->setCenter(osg::Vec3(0.0, 0.0, length/2.0));
  osg::ShapeDrawable* shape = new osg::ShapeDrawable(capsule.get());

  if (resizeable==true)
  {
    shape->setUseDisplayList(false);
  }

  osg::Geode* geode = new osg::Geode();
  geode->addDrawable(shape);
  this->patPtr()->addChild(geode);

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

Rcs::CapsuleNode::CapsuleNode() : NodeBase()
{
  this->capsule = new osg::Capsule();
  capsule->setRadius(0.5);
  capsule->setHeight(1.0);
  capsule->setCenter(osg::Vec3(0.0, 0.0, 0.5));
  osg::ShapeDrawable* shape = new osg::ShapeDrawable(capsule.get());
  osg::Geode* geode = new osg::Geode();
  geode->addDrawable(shape);
  this->patPtr()->addChild(geode);
  setWireframe(true);
}

void Rcs::CapsuleNode::setHeight(double h)
{
  capsule->setHeight(h);
}

void Rcs::CapsuleNode::setRadius(double r)
{
  capsule->setRadius(r);
}

void Rcs::CapsuleNode::setCenter(double x, double y, double z)
{
  capsule->setCenter(osg::Vec3(x, y, z));
}
