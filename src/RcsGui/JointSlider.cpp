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

#include "JointSlider.h"
#include "SimpleSlider.h"

#include <qwt_thermo.h>

#include <QApplication>
#include <QLabel>
#include <QHBoxLayout>
#include <QGroupBox>

#include <stdio.h>
#include <math.h>



/*******************************************************************************
 *
******************************************************************************/
JointSlider::JointSlider(double lower_bound,
                         double zero_pos,
                         double upper_bound,
                         double scaleFactor,
                         QWidget* parent,
                         const char* /*name*/)
  : QWidget(parent)
{
  char a[32];
  _scaleFactor = scaleFactor;
  _lb = (lower_bound - zero_pos) * _scaleFactor;
  _ub = (upper_bound - zero_pos) * _scaleFactor;
  _zp = 0.0;
  _dzp = zero_pos;

  if (fabs(_ub - _lb) < 1.0e-8)
  {
    _ub += 1.0e-8;
    _lb -= 1.0e-8;
  }

  QHBoxLayout* stateBoxLayout = new QHBoxLayout;

  // Label for lower bound joint value
  snprintf(a, 31, " %5.1f", lower_bound * _scaleFactor);
  QLabel* label_lb = new QLabel(a);
  label_lb->setFont(QFont("Helvetica", 8, QFont::Bold));
  label_lb->setFixedWidth(30);

  // Slider with command and current values
  sl = new SimpleSlider(lower_bound, zero_pos, upper_bound, scaleFactor);

  // Bar for left joint range
  t1 = new QwtThermo();
  t1->setFixedWidth((int)(-200.0 * _lb / (_ub - _lb)));
  t1->setBorderWidth(1);
  t1->setPipeWidth(4);
  t1->setFont(QFont("Helvetica", 10));
  // t1->setFillColor(QColor("green"));
  // t1->setAlarmColor(QColor("DarkRed"));
  t1->setAlarmLevel(0.25 * (_lb));
  t1->setOrientation(Qt::Horizontal, QwtThermo::NoScale);
  t1->setRange(_zp, _lb);
  t1->setValue(_lb);

  // Bar for right joint range
  t2 = new QwtThermo();
  t2->setFixedWidth((int)(200.0 * _ub / (_ub - _lb)));
  t2->setBorderWidth(1);
  t2->setPipeWidth(4);
  t2->setFont(QFont("Helvetica", 10));
  // t2->setFillColor(QColor("green"));
  // t2->setAlarmColor(QColor("DarkRed"));
  t2->setAlarmLevel(0.75 * _ub);
  t2->setOrientation(Qt::Horizontal, QwtThermo::NoScale);
  t2->setRange(_zp, _ub);
  t2->setValue(_zp);

  // Label for upper bound joint value
  snprintf(a, 31, " %5.1f", upper_bound * _scaleFactor);
  QLabel* label_ub = new QLabel(a);
  label_ub->setFont(QFont("Helvetica", 8, QFont::Bold));
  label_ub->setFixedWidth(30);

  stateBoxLayout->addWidget(label_lb, Qt::AlignLeft);
  stateBoxLayout->addWidget(sl, Qt::AlignLeft);
  stateBoxLayout->addWidget(t1, Qt::AlignLeft);
  stateBoxLayout->addWidget(t2, Qt::AlignLeft);
  stateBoxLayout->addWidget(label_ub, Qt::AlignLeft);

  // Add layout to widget
  // setFixedHeight(30);
  setLayout(stateBoxLayout);
}

/*******************************************************************************
 *
 ******************************************************************************/
void JointSlider::setValue(double val)
{

  double my_val = val - _dzp;

  if (my_val > 0.0)
  {
    t1->setValue(_lb * _scaleFactor);
    t2->setValue(my_val * _scaleFactor);
  }
  else
  {
    t1->setValue(_lb - my_val * _scaleFactor);
    t2->setValue(0.0);
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
double JointSlider::getSliderValue(void)
{
  return sl->getSliderValue();
}

/*******************************************************************************
 *
 ******************************************************************************/
void JointSlider::setSliderValue(double val)
{
  sl->setSliderValue(val);
}

/*******************************************************************************
 *
 ******************************************************************************/
QwtSlider* JointSlider::getSlider()
{
  return sl->getSlider();
}




