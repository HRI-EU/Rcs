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

#ifndef RCS_SIMPLESLIDER_H
#define RCS_SIMPLESLIDER_H

#include <QWidget>





/*!
 * \defgroup SliderFunctions Slider
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

class SimpleSlider: public QWidget
{
  Q_OBJECT

public:
  SimpleSlider(double lowerBound,
               double zeroPos,
               double upperBound,
               double scaleFactor = 1.0,
               QWidget* parent = 0);

  /*! \brief Returns the slider value in SI-units (divided by the scale
   *         factor).
   */
  double getSliderValue();

  /*! \brief Sets the slider value in SI-units (multiplied by the scale
   *         factor).
   */
  void setSliderValue(double value);


private:
  QwtSlider* qwtslider;
  double zeroPos;
  double scaleFactor;


private slots:

  /*! \brief Sets the slider value to _zeroPos.
   */
  void slider_reset();

  /*! \brief Sets the slider value to 0.0.
   */
  void slider_null();
};



#endif   // RCS_SIMPLESLIDER_H
