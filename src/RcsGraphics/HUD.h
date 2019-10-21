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





  This class is based on the OpenSceneGraph example osghud, which is licensed as

  OpenSceneGraph example, osghud.

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.

*******************************************************************************/

#ifndef RCS_HUD_H
#define RCS_HUD_H

#include <osgViewer/Viewer>
#include <osg/Geometry>
#include <osg/Camera>
#include <osgText/Text>
#include <osg/Geode>
#include <osg/Switch>

#include <string>
#include <sstream>



namespace Rcs
{

class HUD: public osg::Camera
{
  friend class HUDUpdateCallback;

public:

  HUD();
  HUD(const char* text_color);
  HUD(int llx, int lly, int sizeX, int sizeY, const char* text_color="WHITE");
  virtual ~HUD();

  void setText(const char* text);
  void setText(const std::string& text);
  void setText(const std::stringstream& text);

  void addText(const char* text);
  void addText(const std::string& text);
  void addText(const std::stringstream& text);

  void clearText();

  void show();
  void hide();
  bool toggle();

  virtual const char* className() const;

  /*! \brief Updates the font size of the text used in the HUD
   *
   *  \param[in] fontSize Font size in [pt] units.
   */
  virtual void setFontSize(int fontSize);

  /*! \brief Updates the color of the text used in the HUD.
   *
   *  \param[in] colorName Name of color to use. See Rcs_graphicsUtils.cpp for
   *                       available colors, or use the format: \#rrggbb
   */
  virtual void setColor(const char* colorName);
  void resize(int llx, int lly, int sizeX, int sizeY);
  void resize(int sizeX, int sizeY);

private:

  void init(int llx, int lly, int sizeX, int sizeY, const char* textColor);
  void resizeNoMutex(int llx, int lly, int sizeX, int sizeY);

  osg::ref_ptr<osg::Switch> switchNd;
  osg::ref_ptr<osg::Geometry> bgGeometry;
  osg::ref_ptr<osg::Geode> geode;
  osg::ref_ptr<osgText::Text> hudText;
  osg::ref_ptr<osg::Vec3Array> backgroundVertices;
  std::string _text;
  mutable OpenThreads::Mutex _mutex;
  bool textChanged;
  bool backgroundChanged;
  bool autoScale;
  int llx;
  int lly;
  int sizeX;
  int sizeY;
};

}   // namespace Rcs



#endif // RCS_HUD_H
