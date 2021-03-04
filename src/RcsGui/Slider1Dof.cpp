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

#include "Slider1Dof.h"

#include <Rcs_macros.h>
#include <Rcs_guiFactory.h>

#include <qglobal.h>
#include <qwt_slider.h>

#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QTimer>



/******************************************************************************

  \brief Factory instantiation method for Qt thread.

******************************************************************************/

typedef struct
{
  void* ptr[10];
} VoidPointerList;


static void* slider1DGui(void* arg)
{
  VoidPointerList* p = (VoidPointerList*) arg;
  RCHECK(arg);
  double* q_des        = (double*)          p->ptr[0];
  double* q_curr       = (double*)          p->ptr[1];
  const char* name     = (const char*)      p->ptr[2];
  double* lb           = (double*)          p->ptr[3];
  double* zp           = (double*)          p->ptr[4];
  double* ub           = (double*)          p->ptr[5];
  double* ts           = (double*)          p->ptr[6];
  pthread_mutex_t* mtx = (pthread_mutex_t*) p->ptr[7];

  Slider1Dof* w = new Slider1Dof(q_des, q_curr, name, *lb, *zp, *ub, *ts, mtx);
  w->show();

  delete lb;
  delete zp;
  delete ub;
  delete ts;
  delete p;

  return w;
}

Slider1Dof* Slider1Dof::create(double* q_des,
                               double* q_curr,
                               const char* name,
                               double lowerBound,
                               double zeroPos,
                               double upperBound,
                               double ticSize,
                               pthread_mutex_t* mutex)
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
  p->ptr[2] = (void*) name;
  p->ptr[3] = (void*) _lowerBound;
  p->ptr[4] = (void*) _zeroPos;
  p->ptr[5] = (void*) _upperBound;
  p->ptr[6] = (void*) _ticSize;
  p->ptr[7] = (void*) mutex;

  int handle = RcsGuiFactory_requestGUI(slider1DGui, p);

  return (Slider1Dof*) RcsGuiFactory_getPointer(handle);
}



/******************************************************************************



******************************************************************************/

Slider1Dof::Slider1Dof(double* q_des_,
                       double* q_curr_,
                       const char* name,
                       double lowerBound_,
                       double zeroPos_,
                       double upperBound_,
                       double ticSize,
                       pthread_mutex_t* mutex) :
  QFrame(),
  zeroPos(zeroPos_),
  q_des(NULL),
  q_curr(NULL),
  mtx(mutex)
{
  this->q_des = q_des_;
  this->q_curr = q_curr_;
  RCHECK(this->q_des);
  RCHECK(this->q_curr);

  // 25 Hz timer callback for q_curr updates
  QTimer* timer = new QTimer(this);

  // Name label
  QLabel* lb_name = name ? new QLabel(name) : NULL;

  // Palette for lcd numbers
  this->lcd1 = new QLCDNumber(6);
  QPalette palette = this->lcd1->palette();
  palette.setColor(QPalette::Normal, QPalette::Foreground, Qt::red);
  palette.setColor(QPalette::Normal, QPalette::Background, Qt::black);
  palette.setColor(QPalette::Normal, QPalette::Light, Qt::yellow);
  palette.setColor(QPalette::Normal, QPalette::Dark, Qt::darkYellow);
  palette.setColor(QPalette::Inactive, QPalette::Foreground, Qt::red);
  palette.setColor(QPalette::Inactive, QPalette::Background, Qt::black);
  palette.setColor(QPalette::Inactive, QPalette::Light, Qt::yellow);
  palette.setColor(QPalette::Inactive, QPalette::Dark, Qt::darkYellow);

  // Command value
  this->lcd1->setAutoFillBackground(true);
  this->lcd1->setPalette(palette);
  this->lcd1->display(zeroPos);

  // Current value
  this->lcd2 = new QLCDNumber(6);
  this->lcd2->setAutoFillBackground(true);
  this->lcd2->setPalette(palette);
  this->lcd2->display(zeroPos);
  connect(timer, SIGNAL(timeout()), SLOT(updateLcd2()));

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
  connect(this->slider, SIGNAL(valueChanged(double)), SLOT(updateLcd1()));

  // Reset button
  this->button_reset = new QPushButton("Reset");
  this->button_reset->setFixedWidth(50);
  connect(this->button_reset, SIGNAL(clicked()), SLOT(sliderReset()));

  // Null button
  this->button_null = new QPushButton("Null");
  this->button_null->setFixedWidth(50);
  connect(this->button_null, SIGNAL(clicked()), SLOT(sliderNull()));

  QHBoxLayout* FormLayout = new QHBoxLayout(this);
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
}



/******************************************************************************

  Slot function to reset slider to zero position
  The zero position has been given to the constructor in scaled units

******************************************************************************/

void Slider1Dof::sliderNull()
{
  this->slider->setValue(0.0);

  lock();
  *this->q_des = 0.0;
  unlock();
}



/******************************************************************************

  Slot function to reset slider to zero position
  The zero position has been given to the constructor in scaled units

******************************************************************************/

void Slider1Dof::sliderReset()
{
  this->slider->setValue(this->zeroPos);

  lock();
  *this->q_des = this->zeroPos;
  unlock();
}



/******************************************************************************



******************************************************************************/

void Slider1Dof::updateLcd1()
{
  this->lcd1->display(this->slider->value());

  lock();
  *this->q_des = this->slider->value();
  unlock();
}



/******************************************************************************



******************************************************************************/

void Slider1Dof::updateLcd2()
{
  lock();
  double value = *this->q_curr;
  unlock();

  this->lcd2->display(value);
}



/******************************************************************************



******************************************************************************/

void Slider1Dof::lock()
{
  if (this->mtx != NULL)
  {
    pthread_mutex_lock(this->mtx);
  }
}



/******************************************************************************



******************************************************************************/

void Slider1Dof::unlock()
{
  if (this->mtx != NULL)
  {
    pthread_mutex_unlock(this->mtx);
  }
}
