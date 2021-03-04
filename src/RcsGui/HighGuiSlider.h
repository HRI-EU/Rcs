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

#ifndef HIGHGUISLIDER_H
#define HIGHGUISLIDER_H

#include "Atomic.hpp"
#include "HighGuiWidget.h"

#include <list>

class QSlider;
class QLabel;

namespace Rcs
{
class HighGuiSlider : public HighGuiWidget
{
  Q_OBJECT

public:
  typedef void (*CallbackType)(double);

public:
  HighGuiSlider(const std::string& name, double min, double max,
                double step_size, Atomic<double>* feedback);
  ~HighGuiSlider();

  static HighGuiSlider* create(const std::string& name, double min, double max,
                               double step_size, Atomic<double>* feedback);

  double getValue();

  void registerCallback(CallbackType cb_function);

protected Q_SLOTS:
  virtual void update();
  virtual void sliderValueChanged(int value);

protected:
  QSlider* _slider;
  QLabel* _label;
  QLabel* _value_label;
  double _min, _max, _step_size;
  double _last_value;
  Atomic<double>* _feedback_value;

  typedef std::list<CallbackType> CallbackList;
  CallbackList _cb_functions;
};

}

#endif   // HIGHGUISLIDER_H
