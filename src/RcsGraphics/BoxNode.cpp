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

#include "BoxNode.h"

#include <Rcs_Mat3d.h>

#include <osg/Geode>
#include <osg/ShapeDrawable>



/*******************************************************************************
 *
 ******************************************************************************/
Rcs::BoxNode::BoxNode() : NodeBase()
{
  this->box = new osg::Box();
  osg::ShapeDrawable* shape = new osg::ShapeDrawable(box);
  osg::Geode* geode = new osg::Geode();
  geode->addDrawable(shape);
  this->patPtr()->addChild(geode);
  setWireframe(true);
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::BoxNode::BoxNode(const double center[3], double A_BI[3][3],
                      double lx, double ly, double lz,
                      bool resizeable) : NodeBase()
{
  init(center, A_BI, lx, ly, lz, resizeable);
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::BoxNode::BoxNode(const double center[3], double lx, double ly, double lz,
                      bool resizeable) : NodeBase()
{
  double Id[3][3];
  Mat3d_setIdentity(Id);
  init(center, Id, lx, ly, lz, resizeable);
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::BoxNode::BoxNode(const double center[3], double A_BI[3][3],
                      const double ext[3], bool resizeable) : NodeBase()
{
  init(center, A_BI, ext[0], ext[1], ext[2], resizeable);
}

/*******************************************************************************
 * Initialization of graphics
 ******************************************************************************/
void Rcs::BoxNode::init(const double center[3], double A_BI[3][3],
                        double lx, double ly, double lz, bool resizeable)
{
  setName("BoxNode");
  this->box = new osg::Box();
  box->setHalfLengths(osg::Vec3(0.5*lx, 0.5*ly, 0.5*lz));

  osg::ShapeDrawable* shape = new osg::ShapeDrawable(box.get());

  if (resizeable==true)
  {
    shape->setUseDisplayList(false);
  }


  osg::Geode* geode = new osg::Geode();
  geode->addDrawable(shape);
  this->patPtr()->addChild(geode);
  setPosition(center);
  setRotation(A_BI);
  setWireframe(true);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BoxNode::resize(double lx, double ly, double lz)
{
  box->setHalfLengths(osg::Vec3(0.5*lx, 0.5*ly, 0.5*lz));
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BoxNode::resize(const double extents[3])
{
  resize(extents[0], extents[1], extents[2]);
}
