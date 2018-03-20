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

#include "PPSWidget.h"

#include "Rcs_guiFactory.h"
#include "Rcs_macros.h"

#include <QtCore/QTimer>


#define LABEL_WIDTH 160

namespace Rcs
{
typedef struct
{
  void* ptr[10];
} VoidPointerList;

static void* ppsThreadFunc(void* arg)
{
  VoidPointerList* p = (VoidPointerList*) arg;
  RCHECK(arg);
  size_t* width    = (size_t*) p->ptr[0];
  size_t* height   = (size_t*) p->ptr[1];
  double* data     = (double*) p->ptr[2];
  const char* name = (const char*) p->ptr[3];

  PPSWidget* w = new PPSWidget(name, *width, *height, data);
  w->show();

  return w;
}

PPSWidget* PPSWidget::create(const size_t width, const size_t height, double* data, const char* name)
{
  VoidPointerList* p = new VoidPointerList;
  p->ptr[0] = (void*) &width;
  p->ptr[1] = (void*) &height;
  p->ptr[2] = (void*) data;
  p->ptr[3] = (void*) name;

  int handle = RcsGuiFactory_requestGUI(ppsThreadFunc, p);

  return (PPSWidget*) RcsGuiFactory_getPointer(handle);
}

PPSWidget::PPSWidget(const std::string& name, const size_t width, const size_t height, double* data, double scaling, double offset, bool palm):
  QWidget(),
  name(name),
  width(width),
  height(height),
  data(data),
  scaling(scaling),
  offset(offset),
  palm(palm),
  pixelSize(10)
{
  // main layout containing two lines
  this->main_layout = new QVBoxLayout(this);
  this->main_layout->setMargin(1);
  this->main_layout->setSpacing(1);
  this->setFocusPolicy(Qt::NoFocus);
  this->setStyleSheet(
    "QLabel { font-size: 14px; text-align: right; }"
  );

  //name label
  QLabel* name_label = new QLabel(QString(this->name.c_str()));
  name_label->setFixedWidth(LABEL_WIDTH);
  this->main_layout->addWidget(name_label);

  //label for pixmap
  this->PPS = new QLabel(QString(""));
  this->main_layout->addWidget(PPS);

  this->main_layout->addStretch();
  this->setLayout(this->main_layout);

  this->pixmap = new QPixmap(this->width * this->pixelSize, this->height * this->pixelSize);

  // already setting pixmap here to have the correct sizeHint() during construction of PPSGui
  this->PPS->setPixmap(*this->pixmap);


  QTimer* timer = new QTimer(this);

  connect(timer, SIGNAL(timeout()), SLOT(updateDisplay()));
  timer->start(100);
}

PPSWidget::~PPSWidget()
{
  delete this->pixmap;
}


void PPSWidget::updateDisplay()
{
  int counter = 0;

  this->pixmap->fill(QColor("transparent"));

  this->painter.begin(this->pixmap);

  for (size_t j = 0; j < this->height; j++)
  {
    for (size_t i = 0; i < this->width; i++)
    {
      //the palm does not have sensors in the 4 corners
      if ((this->palm) && (j==0 || j==this->height-1) && (i==0 || i==this->width-1))
      {
        continue;
      }

      // the user is to specify offset and scaling such that the range is [0..1]
      // 3*255 is the number of levels supported by this color map
      int r = (int)((this->data[counter++] + this->offset) * 3. * 255. * this->scaling);
      int g = 0;
      int b = 0;

      if (r < 0)
      {
        r = 0;
      }

      if (r > 255)
      {
        g = r - 255;
        r = 255;
      }

      if (g > 255)
      {
        b = g - 255;
        g = 255;
      }

      if (b > 255)
      {
        b = 255;
      }

      //RLOG(0, "[%d,%d]: r=%d g=%d b=%d", i, j, r, g, b);

      this->painter.setBrush(QBrush(QColor(r, g, b)));
      this->painter.setPen(QPen(Qt::NoPen));
      this->painter.drawRect(i * this->pixelSize, (this->height - j - 1) * this->pixelSize, this->pixelSize, this->pixelSize);
    }
  }

  this->painter.end();

  this->PPS->setPixmap(*this->pixmap);
}

}
