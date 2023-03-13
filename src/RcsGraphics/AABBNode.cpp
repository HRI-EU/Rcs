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

#include "AABBNode.h"

#include <osg/Geode>
#include <osg/ShapeDrawable>



/*******************************************************************************
 *
 ******************************************************************************/
Rcs::AABBNode::AABBNode() : NodeBase(), aabbMinPtr(NULL), aabbMaxPtr(NULL)
{
  init();
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::AABBNode::AABBNode(const double aabbMin[3],
                        const double aabbMax[3]) :
  NodeBase(), aabbMinPtr(NULL), aabbMaxPtr(NULL)
{
  init();
  resize(aabbMin, aabbMax);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::AABBNode::init()
{
  setName("AABBNode");
  this->box = new osg::Box();
  osg::ref_ptr<osg::ShapeDrawable> shape = new osg::ShapeDrawable(box.get());
  shape->setUseDisplayList(false);
  osg::ref_ptr<osg::Geode> geode = new osg::Geode();
  geode->addDrawable(shape.get());
  this->patPtr()->addChild(geode.get());
  setWireframe(true);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::AABBNode::makeDynamic(const double aabbMin[3], const double aabbMax[3])
{
  this->aabbMinPtr = aabbMin;
  this->aabbMaxPtr = aabbMax;
  NodeBase::makeDynamic();
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::AABBNode::resize(const double aabbMin[3], const double aabbMax[3])
{
  double center[3], extents[3];
  for (int i = 0; i < 3; ++i)
  {
    center[i] = aabbMin[i] + 0.5 * (aabbMax[i] - aabbMin[i]);
    extents[i] = aabbMax[i] - aabbMin[i];
  }

  pat->setPosition(osg::Vec3(center[0], center[1], center[2]));
  pat->setScale(osg::Vec3(extents[0], extents[1], extents[2]));
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::AABBNode::frameCallback()
{
  resize(aabbMinPtr, aabbMaxPtr);
  return false;
}
