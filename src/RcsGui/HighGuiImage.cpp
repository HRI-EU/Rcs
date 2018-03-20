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

#ifdef USE_OPENCV

#include "HighGuiImage.h"

#include <Rcs_macros.h>
#include <Rcs_guiFactory.h>

#include <QPainter>
#include <QTimer>

namespace Rcs
{

typedef struct
{
  void* ptr[10];
} VoidPointerList;


static void* createHighGuiImage(void* arg)
{
  VoidPointerList* p = (VoidPointerList*) arg;
  RCHECK(arg);
  std::string name = *((std::string*) p->ptr[0]);

  HighGuiImage* w = new HighGuiImage(name);
  return w;
}

/******************************************************************************/

HighGuiImage::HighGuiImage(const std::string& name) :
  HighGuiWidget(name)
{
}

HighGuiImage::~HighGuiImage()
{
}

HighGuiImage* HighGuiImage::create(const std::string& name)
{
  VoidPointerList p;
  p.ptr[0] = (void*) &name;

  int handle = RcsGuiFactory_requestGUI(createHighGuiImage, &p);
  return (HighGuiImage*) RcsGuiFactory_getPointer(handle);
}

void HighGuiImage::setImage(const cv::Mat& image)
{
  switch (image.type())
  {
    case CV_8UC1:
      cvtColor(image, _tmp_image, CV_GRAY2RGB);
      break;
    case CV_8UC3:
      cvtColor(image, _tmp_image, CV_BGR2RGB);
      break;
    default:
      RFATAL("Type not handled: %d", image.type());
      break;
  }

  RCHECK(_tmp_image.isContinuous());

  lock();
  _image = QImage(_tmp_image.data, _tmp_image.cols, _tmp_image.rows,
                  3 * _tmp_image.cols, QImage::Format_RGB888);
  unlock();

  setFixedSize(_image.size());

  QWidget::update();
}

QSize HighGuiImage::sizeHint() const
{
  return _image.size();
}

QSize HighGuiImage::minimumSizeHint() const
{
  return _image.size();
}

void HighGuiImage::update()
{
  HighGuiWidget::update();
}

void HighGuiImage::paintEvent(QPaintEvent*)
{
  QPainter painter(this);
  lock();
  painter.drawImage(QPoint(0,0), _image);
  unlock();
  painter.end();
}

}

#endif
