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

#include "SphereNode.h"

#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Geometry>
#include <osgUtil/SmoothingVisitor>



/******************************************************************************

  \brief Constructors.

******************************************************************************/

Rcs::SphereNode::SphereNode(const double pos[3],
                            const double radius,
                            bool resizeable) : NodeBase()
{
  init(pos, radius, resizeable);
}

Rcs::SphereNode::SphereNode() : NodeBase()
{
  init(NULL, 0.5, false);
}

void Rcs::SphereNode::init(const double pos[3],
                           const double radius,
                           bool resizeable)
{
  setName("SphereNode");
  this->sphere = new osg::Sphere();
  sphere->setRadius(radius);
  osg::ShapeDrawable* shape = new osg::ShapeDrawable(sphere.get());

  if (resizeable==true)
  {
    shape->setUseDisplayList(false);
  }

  osg::Geode* geode = new osg::Geode();
  geode->addDrawable(shape);
  this->patPtr()->addChild(geode);

  if (pos != NULL)
  {
    setPosition(pos);
  }

}

void Rcs::SphereNode::setRadius(double r)
{
  sphere->setRadius(r);
}
