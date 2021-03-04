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

#include "SimpleSlider.h"

#include <qglobal.h>
#include <qwt_slider.h>

#include <QApplication>
#include <QPushButton>
#include <QLabel>
#include <QHBoxLayout>

#include <cmath>



/*******************************************************************************
 *
 ******************************************************************************/
SimpleSlider::SimpleSlider(double lowerBound,
                           double zeroPos_,
                           double upperBound,
                           double scaleFactor_,
                           QWidget* parent)
  : QWidget(parent), zeroPos(zeroPos_), scaleFactor(scaleFactor_)
{
  QHBoxLayout* sliderLayout = new QHBoxLayout;

  qwtslider = new QwtSlider(this);
  qwtslider->setFixedWidth(200);
#if QWT_VERSION < 0x060102
  qwtslider->setRange(lowerBound*scaleFactor, upperBound*scaleFactor,
                      0.01, // Fraction of the interval length
                      10); // Page size (?)
#else
  qwtslider->setLowerBound(round(lowerBound*scaleFactor));
  qwtslider->setUpperBound(round(upperBound*scaleFactor));
  qwtslider->setOrientation(Qt::Horizontal);
  qwtslider->setTotalSteps(round((upperBound-lowerBound)*scaleFactor));
#endif
  qwtslider->setValue(zeroPos*scaleFactor);

  QPushButton* button_reset = new QPushButton("Reset");
  button_reset->setFixedWidth(30);
  button_reset->setFixedHeight(15);
  connect(button_reset, SIGNAL(clicked()), SLOT(slider_reset()));

  QPushButton* button_null = new QPushButton("Null");
  button_null->setFixedWidth(30);
  button_null->setFixedHeight(15);
  connect(button_null, SIGNAL(clicked()), SLOT(slider_null()));

  sliderLayout->addWidget(button_null);
  sliderLayout->addWidget(qwtslider);
  sliderLayout->addWidget(button_reset);

  setLayout(sliderLayout);
}

/*******************************************************************************
 * Slot function to reset slider to zero
 ******************************************************************************/
void SimpleSlider::slider_null()
{
  setSliderValue(0.0);
}

/*******************************************************************************
 * Slot function to reset slider to the initial position
 * The initial position has been given to the constructor in scaled units
 ******************************************************************************/
void SimpleSlider::slider_reset()
{
  setSliderValue(this->zeroPos);
}

/*******************************************************************************
 * Return-value: Command-value in original units
 ******************************************************************************/
double SimpleSlider::getSliderValue()
{
  return qwtslider->value()/this->scaleFactor;
}

/*******************************************************************************
 *
 ******************************************************************************/
void SimpleSlider::setSliderValue(double value)
{
  qwtslider->setValue(this->scaleFactor*value);
}

/*******************************************************************************
 *
 ******************************************************************************/
QwtSlider* SimpleSlider::getSlider()
{
  return this->qwtslider;
}
