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

#include "LcdSlider.h"

#include <Rcs_macros.h>

#include <qglobal.h>
#include <qwt_slider.h>

#include <QHBoxLayout>
#include <QGridLayout>
#include <QTimer>
#include <QtGlobal>


/******************************************************************************



******************************************************************************/

LcdSlider::LcdSlider(double lowerBound,
                     double zeroPos_,
                     double upperBound,
                     double scaleFactor_,
                     double ticSize,
                     const char* name,
                     bool createThirdLcd,
                     bool pull2zero) :
  QFrame(),
  lcd_aux(NULL),
  zeroPos(zeroPos_),
  scaleFactor(scaleFactor_),
  sliderUpdateLcd1(false),
  mouseInteraction(false)
{
  init(lowerBound, zeroPos_, upperBound, scaleFactor_, ticSize, name,
       createThirdLcd, pull2zero);
}



/******************************************************************************



******************************************************************************/

void LcdSlider::init(double lowerBound,
                     double zeroPos_,
                     double upperBound,
                     double scaleFactor_,
                     double ticSize,
                     const char* name,
                     bool createAuxiliaryLcd,
                     bool pull2zero)
{
  // Name label
  this->lb_name = name ? new QLabel(name) : NULL;

  // Palette for lcd numbers
  this->lcd_cmd = new QLCDNumber(6);
  QPalette palette = this->lcd_cmd->palette();
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
  this->lcd_cmd->setAutoFillBackground(true);
  this->lcd_cmd->setPalette(palette);
  this->lcd_cmd->display(zeroPos * this->scaleFactor);

  // Current value
  this->lcd_curr = new QLCDNumber(6);
  this->lcd_curr->setAutoFillBackground(true);
  this->lcd_curr->setPalette(palette);
  this->lcd_curr->display(zeroPos * this->scaleFactor);

  // Auxiliary value
  if (createAuxiliaryLcd == true)
  {
    this->lcd_aux = new QLCDNumber(6);
    this->lcd_aux->setAutoFillBackground(true);
    this->lcd_aux->setPalette(palette);
    this->lcd_aux->display(0.0);
  }

  // Slider
  this->slider = new QwtSlider(this);

  double valueCurr = zeroPos*this->scaleFactor;
  double lb = lowerBound*this->scaleFactor;
  double ub = upperBound*this->scaleFactor;

  if (valueCurr < lb)
  {
    lb = valueCurr - 0.5*(ub-lb);
  }

  if (valueCurr > ub)
  {
    ub = valueCurr + 0.5*(ub-lb);
  }

#if QWT_VERSION < 0x060102
  this->slider->setRange(lb, ub,
                         ticSize * 0.1, // Fraction of the interval length
                         10);   // Page size (?)
#else
  this->slider->setLowerBound(lb);
  this->slider->setUpperBound(ub);
  //this->slider->setFixedWidth(400);
  this->slider->setPageSteps(10);
  this->slider->setOrientation(Qt::Horizontal);

  // 1cm for linear units, 1 deg for angular ones
  if (scaleFactor==1.0)
  {
    this->slider->setTotalSteps(100*round((ub-lb)));
  }
  else
  {
    this->slider->setTotalSteps(round((ub-lb)));
  }
#endif
  this->slider->setValue(valueCurr);
  connect(this->slider, SIGNAL(valueChanged(double)), SLOT(updateCmd()));

  // Reset button
  this->button_reset = new QPushButton("Reset");
  this->button_reset->setFixedWidth(50);
  connect(this->button_reset, SIGNAL(clicked()), SLOT(slider_reset()));

  // Null button
  this->button_null = new QPushButton("Null");
  this->button_null->setFixedWidth(50);
  connect(this->button_null, SIGNAL(clicked()), SLOT(slider_null()));

  QHBoxLayout* FormLayout = new QHBoxLayout(this);

  if (lb_name != NULL)
  {
    FormLayout->addWidget(lb_name);
  }
  //  FormLayout->insertStretch(1);
  FormLayout->addWidget(this->lcd_cmd);
  FormLayout->addWidget(this->lcd_curr);

  if (createAuxiliaryLcd == true)
  {
    FormLayout->addWidget(this->lcd_aux);
  }

  FormLayout->addWidget(this->button_null);
  FormLayout->addWidget(this->button_reset);
  FormLayout->addWidget(this->slider);
  //  this->setStyleSheet("QFrame { margin-top: 1px; margin-bottom: 1px; margin-left: 10px; margin-right: 3px; }");
  //  FormLayout->setMargin(2);
  FormLayout->setContentsMargins(10, 1, 4, 1);
  FormLayout->setSpacing(5);

  setFrameStyle(QFrame::NoFrame | QFrame::Plain);
  setFixedHeight(28);

  // 25 Hz timer callback for q_curr updates
  QTimer* timer = NULL;
  if (pull2zero == true && createAuxiliaryLcd == true)
  {
    connect(this->slider, SIGNAL(sliderPressed()), SLOT(startMouseInteraction()));
    connect(this->slider, SIGNAL(sliderReleased()), SLOT(stopMouseInteraction()));

    timer = new QTimer(this);
    timer->start(40);
    connect(timer, SIGNAL(timeout()), SLOT(updateControls()));
  }
}



/******************************************************************************

  Slot function to reset slider to zero position
  The zero position has been given to the constructor in scaled units

******************************************************************************/

void LcdSlider::slider_null()
{
  this->slider->setValue(0.0);
}



/******************************************************************************

  Slot function to reset slider to zero position
  The zero position has been given to the constructor in scaled units

******************************************************************************/

void LcdSlider::slider_reset()
{
  this->slider->setValue(this->zeroPos * this->scaleFactor);
}



/******************************************************************************

  return-value: Command-value in original units

******************************************************************************/

double LcdSlider::getSliderValue()
{
  return this->slider->value() / this->scaleFactor;
}



/******************************************************************************



******************************************************************************/

void LcdSlider::setSliderValue(double value)
{
  this->slider->setValue(this->scaleFactor * value);
}



/******************************************************************************



******************************************************************************/

void LcdSlider::updateCmd()
{
  if (this->sliderUpdateLcd1)
  {
    this->lcd_cmd->display(this->slider->value());
  }
  emit valueChanged(this->slider->value());
}



/******************************************************************************



******************************************************************************/

// void LcdSlider::setVisible(int visibility)
// {
//   if (!visibility)
//   {
//     hide();
//   }
//   else
//   {
//     show();
//   }
// }



/******************************************************************************



******************************************************************************/

void LcdSlider::setValueLcd1(double value)
{
  if (this->sliderUpdateLcd1)
  {
    RLOG(3, "Conflict: You are trying to overwrite a slider value");
  }
  this->lcd_cmd->display(this->scaleFactor * value);
}



/******************************************************************************



******************************************************************************/

void LcdSlider::setValueLcd2(double value)
{
  this->lcd_curr->display(this->scaleFactor * value);
}



/******************************************************************************



******************************************************************************/

void LcdSlider::setValueLcd3(double value)
{
  if (this->lcd_aux != NULL)
  {
    this->lcd_aux->display(value);
  }
  else
  {
    RLOG(4, "Setting value to non-existing auxiliary QLCDNumber");
  }

}



/******************************************************************************



******************************************************************************/

void LcdSlider::setActive(bool active)
{
  if (active == false)
  {
    this->lcd_cmd->hide();
    this->slider->hide();
    this->button_reset->hide();
    this->button_null->hide();
  }
  else
  {
    this->lcd_cmd->show();
    this->slider->show();
    this->button_reset->show();
    this->button_null->show();
  }

}

/******************************************************************************



******************************************************************************/

int LcdSlider::labelWidthHint()
{
  if (this->lb_name != NULL)
  {
    return this->lb_name->minimumSizeHint().width();
  }
  else
  {
    return 0;
  }
}

/******************************************************************************



******************************************************************************/

void LcdSlider::setLabelWidth(int width)
{
  if (this->lb_name != NULL)
  {
    this->lb_name->setFixedWidth(width);
  }
}

/******************************************************************************



******************************************************************************/

void LcdSlider::setLabel(const char* text)
{
  if (this->lb_name != NULL)
  {
    this->lb_name->setText(text);
  }
  else
  {
    this->lb_name = text ? new QLabel(text) : NULL;
  }
}


/******************************************************************************



******************************************************************************/

void LcdSlider::updateLcd1FromSlider(bool update)
{
  this->sliderUpdateLcd1 = update;
}

void LcdSlider::updateControls()
{
  if (this->mouseInteraction == false && this->lcd_aux != NULL && this->lcd_aux->value() > 0)
  {
    double q_des = getSliderValue();
    double diff2Zero = (this->zeroPos - q_des);
#if QWT_VERSION < 0x060102
    double minDecrease = .02*fabs(this->slider->maxValue() - this->slider->minValue())/this->scaleFactor;
#else
    double minDecrease = .02*fabs(this->slider->upperBound() - this->slider->lowerBound()) / this->scaleFactor;
#endif
    double step = .2*diff2Zero;

    if (fabs(step)<minDecrease)
    {
      step = copysign(minDecrease,step);
      if (fabs(step)>fabs(diff2Zero))
      {
        step = diff2Zero;
      }
    }

    q_des += step;

    setSliderValue(q_des);
  }
}

void LcdSlider::startMouseInteraction()
{
  this->mouseInteraction = true;
}

void LcdSlider::stopMouseInteraction()
{
  this->mouseInteraction = false;
}

void LcdSlider::setLowerBound(double value)
{
#if QWT_VERSION < 0x060102
  slider->setRange(value*this->scaleFactor, slider->maxValue());
#else
  slider->setLowerBound(value*this->scaleFactor);
#endif
}

void LcdSlider::setUpperBound(double value)
{
#if QWT_VERSION < 0x060102
  slider->setRange(slider->minValue(), value*this->scaleFactor);
#else
  slider->setUpperBound(value*this->scaleFactor);
#endif
}
