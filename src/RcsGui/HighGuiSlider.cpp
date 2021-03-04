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

#include "HighGuiSlider.h"
#include "Rcs_guiFactory.h"

#include <Rcs_macros.h>
#include <Rcs_basicMath.h>

#include <QLabel>
#include <QSlider>
#include <QHBoxLayout>

#include <cmath>



namespace Rcs
{

typedef struct
{
  void* ptr[10];
} VoidPointerList;


static void* createHighGuiSlider(void* arg)
{
  VoidPointerList* p = (VoidPointerList*) arg;
  RCHECK(arg);
  std::string name = *((std::string*) p->ptr[0]);
  double min = *((double*) p->ptr[1]);
  double max = *((double*) p->ptr[2]);
  double step_size = *((double*) p->ptr[3]);
  Atomic<double>* feedback = (Atomic<double>*) p->ptr[4];

  HighGuiSlider* w = new HighGuiSlider(name, min, max, step_size, feedback);
  return w;
}

/******************************************************************************/

HighGuiSlider::HighGuiSlider(const std::string& name, double min, double max,
                             double step_size, Atomic<double>* feedback) :
  HighGuiWidget(name),
  _min(min),
  _max(max),
  _step_size(step_size),
  _last_value(min),
  _feedback_value(feedback)
{
  QLayout* main_layout = new QHBoxLayout(this);

  _label = new QLabel(this);
  _label->setText(QString::fromStdString(name));
  main_layout->addWidget(_label);

  _slider = new QSlider(Qt::Horizontal, this);
  _slider->setMinimum(round(_min / _step_size));
  _slider->setMaximum(round(_max / _step_size));
  _slider->setValue(_last_value);
  _slider->setMinimumWidth(100);
  main_layout->addWidget(_slider);

  _value_label = new QLabel(this);
  _value_label->setFixedWidth(40);
  _value_label->setNum(_last_value);
  main_layout->addWidget(_value_label);

  connect(_slider, SIGNAL(valueChanged(int)), this,
          SLOT(sliderValueChanged(int)));

  main_layout->setMargin(0);
}

HighGuiSlider::~HighGuiSlider()
{
}

HighGuiSlider* HighGuiSlider::create(const std::string& name, double min,
                                     double max, double step_size,
                                     Atomic<double>* feedback)
{
  VoidPointerList p;
  p.ptr[0] = (void*) &name;
  p.ptr[1] = (void*) &min;
  p.ptr[2] = (void*) &max;
  p.ptr[3] = (void*) &step_size;
  p.ptr[4] = (void*) feedback;

  int handle = RcsGuiFactory_requestGUI(createHighGuiSlider, &p);
  return (HighGuiSlider*) RcsGuiFactory_getPointer(handle);
}

void HighGuiSlider::update()
{
  HighGuiWidget::update();

  if (_feedback_value)
  {
    double val = *_feedback_value;

    if (val != _last_value)
    {
      _slider->setValue(round(val / _step_size));
    }
  }
}

double HighGuiSlider::getValue()
{
  lock();
  double value = _last_value;
  unlock();

  return value;
}

void HighGuiSlider::registerCallback(CallbackType cb_function)
{
  _cb_functions.push_back(cb_function);
}

void HighGuiSlider::sliderValueChanged(int value)
{
  lock();
  _last_value = value * _step_size;

  _value_label->setNum(_last_value);

  if (_feedback_value)
  {
    *_feedback_value = _last_value;
  }

  for (CallbackList::iterator it = _cb_functions.begin();
       it != _cb_functions.end(); ++it)
  {
    (*it)(_last_value);
  }

  unlock();
}


}
