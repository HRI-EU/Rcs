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

#include "PPSSensorNode.h"
#include "Rcs_graphicsUtils.h"
#include "ArrowNode.h"

#include <Rcs_typedef.h>
#include <Rcs_Vec3d.h>
#include <Rcs_Mat3d.h>
#include <Rcs_macros.h>
#include <Rcs_resourcePath.h>

#include <osgText/Text>
#include <osg/Geode>
#include <osg/ShapeDrawable>



/*******************************************************************************
 * This array offsetNormal has as many columns as texels. It has 6 rows: Rows
 * 1-3 hold the texel position, rows 4-6 the corresponding texel normal vector.
 ******************************************************************************/
Rcs::PPSSensorNode::PPSSensorNode(const RcsSensor* pps, bool debug)
{
  RCHECK(pps->type==RCSSENSOR_PPS);

  char fontFile[256]="";
  bool fontFound = Rcs_getAbsoluteFileName("fonts/VeraMono.ttf", fontFile);

  if (fontFound==false)
  {
    RLOG(4, "Couldn't find font file - skipping texel numbering");
  }

  osg::ref_ptr<osg::Geode> geode = new osg::Geode();
  osg::ref_ptr<osg::Geode> textGeode = new osg::Geode();
  int id = 0;

  RCSSENSOR_TRAVERSE_TEXELS(pps)
  {
    // skip texels without proper normal
    double length = Vec3d_getLength(TEXEL->normal);
    if (fabs(length) < 0.1)
    {
      continue;
    }

    HTr relTrans;
    Vec3d_setZero(relTrans.org);
    Mat3d_fromVec(relTrans.rot, TEXEL->normal, 2);
    osg::ref_ptr<osg::Box> box = new osg::Box();
    box->setHalfLengths(osg::Vec3(0.5*TEXEL->extents[0], 0.5*TEXEL->extents[1],
                                  0.5*TEXEL->extents[2]));
    box->setCenter(osg::Vec3(TEXEL->position[0], TEXEL->position[1],
                             TEXEL->position[2]));
    box->setRotation(QuatFromHTr(&relTrans));
    osg::ref_ptr<osg::ShapeDrawable> shape = new osg::ShapeDrawable(box);
    geode->addDrawable(shape.get());

    if (debug==true)
    {
      const double edgeLength = Vec3d_getLength(TEXEL->extents);

      // Add a number on top of each texel
      if (fontFound==true)
      {
        osg::ref_ptr<osgText::Text> texelNumber = new osgText::Text();
        texelNumber->setCharacterSize(0.5*edgeLength);
        char a[64];
        sprintf(a, "%d", id);
        texelNumber->setText(std::string(a));
        texelNumber->setAlignment(osgText::Text::CENTER_CENTER);
        texelNumber->setFont(fontFile);

        double textPos[3];
        Vec3d_constMulAndAdd(textPos, TEXEL->position, TEXEL->normal,
                             1.5*TEXEL->extents[2]);
        texelNumber->setPosition(osg::Vec3(textPos[0], textPos[1], textPos[2]));
        texelNumber->setRotation(QuatFromHTr(&relTrans));
        texelNumber->setColor(colorFromString("RED"));
        textGeode->addDrawable(texelNumber.get());
      }

      // Show texel normals as arrows
      osg::ref_ptr<ArrowNode> an = new ArrowNode();
      an->setPosition(TEXEL->position);
      an->setRotation(relTrans.rot);
      an->setDirection(TEXEL->normal);
      an->setRadius(0.05*edgeLength);
      an->setArrowLength(3.0*edgeLength);
      an->setMaterial("BLUE");
      this->patPtr()->addChild(an.get());
    }

    id++;
  }

  setNodeMaterial("BLACK_RUBBER", geode.get());

  if (debug==true)
  {
    setNodeMaterial("RED", textGeode.get());
    setNodeMaterial("WHITE", geode.get());
    this->patPtr()->addChild(textGeode.get());
  }

  this->patPtr()->addChild(geode.get());

  makeDynamic(pps->body->A_BI->org, pps->body->A_BI->rot);
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::PPSSensorNode::~PPSSensorNode()
{
}
