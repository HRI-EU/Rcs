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

#ifndef LCDSLIDER_H
#define LCDSLIDER_H

#include <QFrame>
#include <QLCDNumber>
#include <QPushButton>
#include <QLabel>




/*!
 * \defgroup LcdSliderFunctions Slider
 *
 *  Description: Slider class for GUI elements. <br>
 *
 *  Values lowerBound and upperBound are scaled with the given scaleFactor.
 *  The scale factor is meant to scale the range to human readable units, so
 *  that the GUI displays angles in degrees and translations in mm. The
 *  slider return value however returns the values WITHOUT the scale factor,
 *  since we never use degrees or mm inside our computation.
 *  The command values should also be in SI units. The scaling to human
 *  readable units is done within this class.
 */



class QwtSlider;

class LcdSlider: public QFrame
{
  Q_OBJECT

public:

  LcdSlider(double lowerBound  = -1.0,
            double zeroPos     = 0.0,
            double upperBound  = 1.0,
            double scaleFactor = 1.0,
            double ticSize     = 1.0,
            const char* name=NULL,
            bool createThirdLcd=false,
            bool pull2zero=false);

  /*!
   *  \brief Updates the first LCD from the slider value.
   */
  void updateLcd1FromSlider(bool update=true);

  /*!
   *  \brief Returns the slider value in SI-units (divided by the scale
   *         factor).
   */
  double getSliderValue();

  /*!
   *  \brief Sets the slider value in SI-units (multiplied by the scale
   *         factor).
   */
  void setSliderValue(double value);

  /*!
   *  \brief Displays the given value in the first, second and auxiliary LCD.
   *         If the auxiliary LCD hasn't been created, a warning is issued on
   *         debug level 4.
   */
  void setValueLcd1(double value);
  void setValueLcd2(double value);
  void setValueLcd3(double value);

  /*!
   *  \brief Shows or hides a lot of stuff according to being passive or
   *         active.
   */
  void setActive(bool active);

  /*!
   *  \brief Returns the size hint for the label width
   */
  int labelWidthHint();

  /*!
   *  \brief Sets the width of the name label if present
   */
  void setLabelWidth(int width);

  /*!
   *  \brief Sets the label text
   */
  void setLabel(const char* text);

  void setLowerBound(double value);
  void setUpperBound(double value);



signals:

  void valueChanged(double newValue);

  /* public slots: */

  /*   void setVisible(int visibility); */



private:

  void init(double lowerBound,
            double zeroPos,
            double upperBound,
            double scaleFactor,
            double ticSize,
            const char* name,
            bool createAuxiliaryLcd,
            bool pull2zero);
  QLabel* lb_name;
  QPushButton* button_reset;
  QPushButton* button_null;
  QLCDNumber* lcd_cmd;
  QLCDNumber* lcd_curr;
  QLCDNumber* lcd_aux;
  QwtSlider* slider;
  double zeroPos;
  double scaleFactor;
  bool sliderUpdateLcd1;

  bool mouseInteraction;


private slots:

  /*! \brief Sets the slider value to _zeroPos.
   */
  void slider_reset();

  /*!
   *  \brief Sets the slider value to 0.0.
   */
  void slider_null();

  /*! \brief Updates the slider value in the LCDNumber.
   */
  void updateCmd();

  void updateControls();
  void startMouseInteraction();
  void stopMouseInteraction();
};



#endif   // LCDSLIDER_H
