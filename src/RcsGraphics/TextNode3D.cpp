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

#include "TextNode3D.h"

#include <Rcs_graphicsUtils.h>
#include <Rcs_resourcePath.h>

#include <osg/Geode>
#include <osg/ShapeDrawable>



Rcs::TextNode3D::TextNode3D(std::string text) : NodeBase()
{
  init(text);
}

Rcs::TextNode3D::TextNode3D(std::string text, const double pos[3]) : NodeBase()
{
  init(text);
  setPosition(pos);
}

void Rcs::TextNode3D::init(std::string text)
{
  osg::ref_ptr<osg::Geode> textGeode = new osg::Geode();
  addChild(textGeode.get());

  this->text3D = new osgText::Text;
  text3D->setCharacterSize(0.05);
  text3D->setText(text);
  text3D->setAlignment(osgText::Text::LEFT_CENTER);
  text3D->setAxisAlignment(osgText::Text::SCREEN);
  char fontFile[256] = "";
  bool fontFound = Rcs_getAbsoluteFileName("fonts/VeraMono.ttf", fontFile);
  if (fontFound)
  {
    text3D->setFont(fontFile);
  }

  textGeode->addDrawable(text3D.get());
}

void Rcs::TextNode3D::setMaterial(const std::string& material, double alpha)
{
  text3D->setColor(colorFromString(material.c_str()));
}

osg::ref_ptr<osgText::Text> Rcs::TextNode3D::getText()
{
  return this->text3D;
}
