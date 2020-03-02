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

#include "CylinderNode.h"

#include <osg/Geode>
#include <osg/ShapeDrawable>



/******************************************************************************

  \brief Constructors.

******************************************************************************/

Rcs::CylinderNode::CylinderNode(const double center[3], double A_BI[3][3],
                              const double radius, const double length,
                              bool resizeable) : NodeBase()
{
  this->cylinder = new osg::Cylinder();
  cylinder->setRadius(radius);
  cylinder->setHeight(length);
  cylinder->setCenter(osg::Vec3(center[0], center[1], center[2]));
  osg::ShapeDrawable* shape = new osg::ShapeDrawable(cylinder.get());

  if (resizeable==true)
  {
    shape->setUseDisplayList(false);
  }

  osg::Geode* geode = new osg::Geode();
  geode->addDrawable(shape);
  this->patPtr()->addChild(geode);

  if (A_BI != NULL)
  {
    setRotation(A_BI);
  }

  setWireframe(true);
}

Rcs::CylinderNode::CylinderNode() : NodeBase()
{
  this->cylinder = new osg::Cylinder();
  cylinder->setRadius(0.5);
  cylinder->setHeight(1.0);
  cylinder->setCenter(osg::Vec3(0.0, 0.0, 0.0));
  osg::ShapeDrawable* shape = new osg::ShapeDrawable(cylinder.get());
  osg::Geode* geode = new osg::Geode();
  geode->addDrawable(shape);
  this->patPtr()->addChild(geode);
  setWireframe(true);
}

void Rcs::CylinderNode::setHeight(double h)
{
  cylinder->setHeight(h);
}

void Rcs::CylinderNode::setRadius(double r)
{
  cylinder->setRadius(r);
}

void Rcs::CylinderNode::setCenter(double x, double y, double z)
{
  cylinder->setCenter(osg::Vec3(x, y, z));
}
