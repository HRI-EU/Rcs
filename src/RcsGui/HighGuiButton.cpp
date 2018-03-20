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

#include "HighGuiButton.h"

#include <Rcs_macros.h>
#include <Rcs_guiFactory.h>

#include <QPushButton>
#include <QVBoxLayout>

namespace Rcs
{

typedef struct
{
  void* ptr[10];
} VoidPointerList;


static void* createHighGuiButton(void* arg)
{
  VoidPointerList* p = (VoidPointerList*) arg;
  RCHECK(arg);
  std::string name = *((std::string*) p->ptr[0]);

  HighGuiButton* w = new HighGuiButton(name);
  return w;
}

/******************************************************************************/

HighGuiButton::HighGuiButton(const std::string& name) :
  HighGuiWidget(name)
{
  QLayout* main_layout = new QVBoxLayout(this);

  _button = new QPushButton(QString::fromStdString(name), this);
  _button->setCheckable(true);
  connect(_button, SIGNAL(clicked(bool)), SLOT(buttonClicked()));
  main_layout->addWidget(_button);
  main_layout->setMargin(0);
}

HighGuiButton::~HighGuiButton()
{
}

HighGuiButton* HighGuiButton::create(const std::string& name)
{
  VoidPointerList p;
  p.ptr[0] = (void*) &name;

  int handle = RcsGuiFactory_requestGUI(createHighGuiButton, &p);
  return (HighGuiButton*) RcsGuiFactory_getPointer(handle);
}

bool HighGuiButton::pressed(bool reset)
{
  if (_button->isChecked())
  {
    if (reset)
    {
      _button->setChecked(false);
    }

    return true;
  }

  return false;
}

void HighGuiButton::registerCallback(CallbackType cb_function)
{
  _cb_functions.push_back(cb_function);
}

void HighGuiButton::update()
{
  HighGuiWidget::update();
}

void HighGuiButton::buttonClicked()
{
  if (!_cb_functions.empty())
  {
    _button->setChecked(false);
  }

  for (CallbackList::iterator it = _cb_functions.begin();
       it != _cb_functions.end(); ++it)
  {
    (*it)();
  }
}

}
