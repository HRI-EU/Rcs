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
#include "SliderCheck1Dof.h"

#include <Rcs_macros.h>
#include <Rcs_guiFactory.h>

#include <qglobal.h>
#include <qwt_slider.h>

#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QTimer>
#include <QtGlobal>



/******************************************************************************

  \brief Factory instantiation method for Qt thread.

******************************************************************************/

typedef struct
{
  void* ptr[10];
} VoidPointerList;


static void* sliderCheck1DGui(void* arg)
{
  VoidPointerList* p = (VoidPointerList*) arg;
  RCHECK(arg);
  double* q_des   = (double*)     p->ptr[0];
  double* q_curr  = (double*)     p->ptr[1];
  double* active  = (double*)     p->ptr[2];
  const char* name = (const char*) p->ptr[3];
  double* lb      = (double*)     p->ptr[4];
  double* zp      = (double*)     p->ptr[5];
  double* ub      = (double*)     p->ptr[6];
  double* ts      = (double*)     p->ptr[7];

  SliderCheck1Dof* w = new SliderCheck1Dof(q_des, q_curr, active, name, *lb, *zp, *ub, *ts);
  w->show();

  delete lb;
  delete zp;
  delete ub;
  delete ts;
  delete p;

  return w;
}

SliderCheck1Dof* SliderCheck1Dof::create(double* q_des,
                                         double* q_curr,
                                         double* active,
                                         const char* name,
                                         double lowerBound,
                                         double zeroPos,
                                         double upperBound,
                                         double ticSize)
{
  double* _lowerBound = new double;
  *_lowerBound = lowerBound;
  double* _zeroPos    = new double;
  *_zeroPos    = zeroPos;
  double* _upperBound = new double;
  *_upperBound = upperBound;
  double* _ticSize    = new double;
  *_ticSize    = ticSize;

  VoidPointerList* p = new VoidPointerList;
  p->ptr[0] = (void*) q_des;
  p->ptr[1] = (void*) q_curr;
  p->ptr[2] = (void*) active;
  p->ptr[3] = (void*) name;
  p->ptr[4] = (void*) _lowerBound;
  p->ptr[5] = (void*) _zeroPos;
  p->ptr[6] = (void*) _upperBound;
  p->ptr[7] = (void*) _ticSize;

  int handle = RcsGuiFactory_requestGUI(sliderCheck1DGui, p);

  return (SliderCheck1Dof*) RcsGuiFactory_getPointer(handle);
}



/******************************************************************************



******************************************************************************/

SliderCheck1Dof::SliderCheck1Dof(double* q_des_,
                                 double* q_curr_,
                                 double* active_,
                                 const char* name,
                                 double lowerBound_,
                                 double zeroPos_,
                                 double upperBound_,
                                 double ticSize) :
  QFrame(),
  zeroPos(zeroPos_),
  q_des(NULL),
  q_curr(NULL),
  active(NULL)
{
  this->q_des = q_des_;
  this->q_curr = q_curr_;
  this->active = active_;
  RCHECK(this->q_des);
  RCHECK(this->q_curr);
  RCHECK(this->active);

  // 25 Hz timer callback for q_curr updates
  QTimer* timer = new QTimer(this);

  // Name label
  QLabel* lb_name = name ? new QLabel(name) : NULL;

  // Palette for lcd numbers
  this->lcd1 = new QLCDNumber(6);
  QPalette palette = this->lcd1->palette();
#if QT_VERSION < QT_VERSION_CHECK(5, 13, 0)
  palette.setColor(QPalette::Normal, QPalette::Foreground, Qt::red);
  palette.setColor(QPalette::Normal, QPalette::Background, Qt::black);
  palette.setColor(QPalette::Normal, QPalette::Light, Qt::yellow);
  palette.setColor(QPalette::Normal, QPalette::Dark, Qt::darkYellow);
  palette.setColor(QPalette::Inactive, QPalette::Foreground, Qt::red);
  palette.setColor(QPalette::Inactive, QPalette::Background, Qt::black);
  palette.setColor(QPalette::Inactive, QPalette::Light, Qt::yellow);
  palette.setColor(QPalette::Inactive, QPalette::Dark, Qt::darkYellow);
#else

  palette.setColor(QPalette::Normal, QPalette::WindowText, Qt::red);
  palette.setColor(QPalette::Normal, QPalette::Window, Qt::black);
  palette.setColor(QPalette::Normal, QPalette::Light, Qt::yellow);
  palette.setColor(QPalette::Normal, QPalette::Dark, Qt::darkYellow);
  palette.setColor(QPalette::Inactive, QPalette::WindowText, Qt::red);
  palette.setColor(QPalette::Inactive, QPalette::Window, Qt::black);
  palette.setColor(QPalette::Inactive, QPalette::Light, Qt::yellow);
  palette.setColor(QPalette::Inactive, QPalette::Dark, Qt::darkYellow);

#endif

  // Command value
  this->lcd1->setAutoFillBackground(true);
  this->lcd1->setPalette(palette);
  this->lcd1->display(zeroPos);

  // Current value
  this->lcd2 = new QLCDNumber(6);
  this->lcd2->setAutoFillBackground(true);
  this->lcd2->setPalette(palette);
  this->lcd2->display(zeroPos);

  // Slider
  this->slider = new QwtSlider(this);
#if QWT_VERSION < 0x060102
  this->slider->setRange(lowerBound_,
                         upperBound_,
                         ticSize * 0.1, // Fraction of the interval length
                         10);   // Page size (?)
#else
  this->slider->setLowerBound(lowerBound_);
  this->slider->setUpperBound(upperBound_);
#endif
  this->slider->setValue(zeroPos);
  connect(this->slider, SIGNAL(valueChanged(double)), SLOT(desUpdate()));

  // Reset button
  this->button_reset = new QPushButton("Reset");
  this->button_reset->setFixedWidth(50);
  connect(this->button_reset, SIGNAL(clicked()), SLOT(desReset()));

  // Null button
  this->button_null = new QPushButton("Null");
  this->button_null->setFixedWidth(50);
  connect(this->button_null, SIGNAL(clicked()), SLOT(desNull()));

  // Activation check box
  this->check_active = new QCheckBox();
  if (*this->active < .5)
  {
    this->check_active->setChecked(true);
  }
  else
  {
    this->check_active->setChecked(false);
  }
  connect(check_active, SIGNAL(stateChanged(int)), SLOT(selectionUpdate(int)));

  QHBoxLayout* FormLayout = new QHBoxLayout(this);
  FormLayout->addWidget(this->check_active);
  if (lb_name)
  {
    FormLayout->addWidget(lb_name);
  }
  FormLayout->insertStretch(1);
  FormLayout->addWidget(this->lcd1);
  FormLayout->addWidget(this->lcd2);
  FormLayout->addWidget(this->button_null);
  FormLayout->addWidget(this->button_reset);
  FormLayout->addWidget(this->slider);

  setFrameStyle(QFrame::NoFrame | QFrame::Plain);
  setFixedHeight(40);

  timer->start(40);
  connect(timer, SIGNAL(timeout()), SLOT(updateControls()));
}



/******************************************************************************

  Slot function to reset slider to zero position
  The zero position has been given to the constructor in scaled units

******************************************************************************/

void SliderCheck1Dof::desNull()
{
  if (*this->active < .5)
  {
    *this->q_des = 0.0;
  }
}



/******************************************************************************

  Slot function to reset slider to zero position
  The zero position has been given to the constructor in scaled units

******************************************************************************/

void SliderCheck1Dof::desReset()
{
  if (*this->active < .5)
  {
    *this->q_des = this->zeroPos;
  }
}



/******************************************************************************



******************************************************************************/

void SliderCheck1Dof::desUpdate()
{
  if (*this->active < .5)
  {
    *this->q_des = this->slider->value();
  }
}

/******************************************************************************



******************************************************************************/

void SliderCheck1Dof::selectionUpdate(int checkBoxState)
{
  switch (checkBoxState)
  {
    case Qt::Unchecked:
      *this->active = 1.;
      break;

    case Qt::Checked:
      *this->active = 0.;
      break;

    default:
      RLOG(1, "Unknown check button state: %d", checkBoxState);
  }
}

/******************************************************************************
  \brief If the task is inactive, the command values will be set to the
         current values. This is convenient since activating any selection
         will not lead to a change of the movement.
******************************************************************************/
void SliderCheck1Dof::updateControls()
{
  if (*this->active < .5)
  {
    this->check_active->setChecked(true);
  }
  else
  {
    this->check_active->setChecked(false);
  }

  if (*this->active > .5)
  {
    *this->q_des = *this->q_curr;
  }

  this->slider->setValue(*this->q_des);
  this->lcd1->display(*this->q_des);
  this->lcd2->display(*this->q_curr);
}
