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

#ifndef RCS_NODEBASE_H
#define RCS_NODEBASE_H

#include <osg/Switch>
#include <osg/PositionAttitudeTransform>
#include <osgGA/GUIEventHandler>

#include <string>


namespace Rcs
{
/*!
 * \ingroup RcsGraphics
 */
class NodeBase : public osg::Switch
{
public:

  NodeBase();
  virtual void show();
  virtual void hide();
  virtual bool toggle();
  virtual void setPosition(double x, double y, double z);
  virtual void setPosition(const double x[3]);
  virtual void setPosition(osg::Vec3 pos);
  virtual void setRotation(double A_KI[3][3]);
  virtual void setRotation(double eulerABC[3]);
  virtual void setRotation(osg::Quat att);
  virtual void setTransformation(const double x[3], double A_KI[3][3]);
  virtual void setWireframe(bool visible=true);
  virtual osg::ref_ptr<osg::PositionAttitudeTransform> patPtr() const;
  virtual void toggleWireframe();
  virtual void makeDynamic();
  virtual void makeDynamic(const double pos[3]);
  virtual void makeDynamic(double A_KI[3][3]);
  virtual void makeDynamic(const double pos[3], double A_KI[3][3]);
  virtual const double* getPosPtr() const;
  virtual const double* getRotMatPtr() const;
  virtual void setPosPtr(const double* pos);
  virtual void setRotMatPtr(const double* rot);
  virtual void setMaterial(const char* material, double alpha=1.0);
  virtual void setMaterial(const std::string& material, double alpha=1.0);
  virtual bool isVisible() const;
  virtual bool frameCallback();
  virtual bool eventCallback(const osgGA::GUIEventAdapter& ea,
                             osgGA::GUIActionAdapter& aa);

protected:

  osg::ref_ptr<osg::PositionAttitudeTransform> pat;
  bool showWireframe;
  const double* posPtr;
  const double* rmPtr;
  bool isDynamic;
};

}   // namespace Rcs

#endif // RCS_NODEBASE_H
