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

#include "SSRNode.h"

#include <osg/Geode>
#include <osg/ShapeDrawable>



/******************************************************************************

  \brief Constructors.

******************************************************************************/

Rcs::SSRNode::SSRNode(const double center[3], double A_KI[3][3],
                      const double extent[2], const double r,
                      bool resizeable) : NodeBase()
{
  osg::Geode* geode = new osg::Geode();
  osg::ref_ptr<osg::TessellationHints> hints = new osg::TessellationHints;
  hints->setDetailRatio(0.5f);

  double lx = extent[0];
  double ly = extent[1];

  // Side 1: Front y-direction
  osg::Capsule* cSSR1 =
    new osg::Capsule(osg::Vec3(-lx / 2.0, 0.0, 0.0), r, ly);
  cSSR1->setRotation(osg::Quat(osg::inDegrees(90.0f),
                               osg::Vec3(1.0f, 0.0f, 0.0f)));
  osg::ShapeDrawable* sdrC1 = new osg::ShapeDrawable(cSSR1, hints.get());
  if (resizeable==true)
  {
    sdrC1->setUseDisplayList(false);
  }
  geode->addDrawable(sdrC1);

  // Side 2: Back y-direction
  osg::Capsule* cSSR2 =
    new osg::Capsule(osg::Vec3(lx / 2.0, 0.0, 0.0), r, ly);
  cSSR2->setRotation(osg::Quat(osg::inDegrees(90.0f),
                               osg::Vec3(1.0f, 0.0f, 0.0f)));
  osg::ShapeDrawable* sdrC2 = new osg::ShapeDrawable(cSSR2, hints.get());
  if (resizeable==true)
  {
    sdrC2->setUseDisplayList(false);
  }
  geode->addDrawable(sdrC2);

  // Side 3: Right x-direction
  osg::Capsule* cSSR3 =
    new osg::Capsule(osg::Vec3(0.0, ly / 2.0, 0.0), r, lx);
  cSSR3->setRotation(osg::Quat(osg::inDegrees(90.0f),
                               osg::Vec3(0.0f, 1.0f, 0.0f)));
  osg::ShapeDrawable* sdrC3 = new osg::ShapeDrawable(cSSR3, hints.get());
  if (resizeable==true)
  {
    sdrC3->setUseDisplayList(false);
  }
  geode->addDrawable(sdrC3);

  // Side 4: Left x-direction
  osg::Capsule* cSSR4 =
    new osg::Capsule(osg::Vec3(0.0, -ly / 2.0, 0.0), r, lx);
  cSSR4->setRotation(osg::Quat(osg::inDegrees(90.0f),
                               osg::Vec3(0.0f, 1.0f, 0.0f)));
  osg::ShapeDrawable* sdrC4 = new osg::ShapeDrawable(cSSR4, hints.get());
  if (resizeable==true)
  {
    sdrC4->setUseDisplayList(false);
  }
  geode->addDrawable(sdrC4);

  // Box part
  osg::Box* bSSR =
    new osg::Box(osg::Vec3(0.0, 0.0, 0.0), lx, ly, 2.0 * r);
  osg::ShapeDrawable* sdrBox = new osg::ShapeDrawable(bSSR, hints.get());
  if (resizeable==true)
  {
    sdrBox->setUseDisplayList(false);
  }
  geode->addDrawable(sdrBox);
  this->patPtr()->addChild(geode);

  if (center != NULL)
  {
    setPosition(center);
  }

  if (A_KI != NULL)
  {
    setRotation(A_KI);
  }

  setWireframe(true);
}
