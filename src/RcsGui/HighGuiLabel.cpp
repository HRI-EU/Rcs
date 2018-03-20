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

#include "HighGuiLabel.h"
#include "Rcs_guiFactory.h"

#include <QLabel>
#include <QTimer>
#include <QVBoxLayout>

namespace Rcs
{

typedef struct
{
  void* ptr[10];
} VoidPointerList;


static void* createHighGuiLabel(void* arg)
{
  VoidPointerList* p = (VoidPointerList*) arg;
  std::string name = *((std::string*) p->ptr[0]);

  HighGuiLabel* w = new HighGuiLabel(name);
  return w;
}

/******************************************************************************/

HighGuiLabel::HighGuiLabel(const std::string& name) :
  HighGuiWidget(name),
  _new_label(false)
{
  QLayout* main_layout = new QVBoxLayout(this);

  _label = new QLabel(this);
  main_layout->addWidget(_label);
  main_layout->setMargin(0);
}

HighGuiLabel::~HighGuiLabel()
{
}

HighGuiLabel* HighGuiLabel::create(const std::string& name)
{
  VoidPointerList p;
  p.ptr[0] = (void*) &name;

  int handle = RcsGuiFactory_requestGUI(createHighGuiLabel, &p);
  return (HighGuiLabel*) RcsGuiFactory_getPointer(handle);
}

void HighGuiLabel::setLabel(const std::string& label)
{
  lock();
  _label_string = label;
  _new_label = true;
  unlock();
}

void HighGuiLabel::update()
{
  HighGuiWidget::update();

  lock();
  if (_new_label)
  {
    _label->setText(QString::fromStdString(_label_string));
    _new_label = false;
  }
  unlock();
}

}
