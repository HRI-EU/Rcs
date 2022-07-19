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

#ifndef SLIDER1DOF_H
#define SLIDER1DOF_H

#include "AsyncWidget.h"
#include <QFrame>
#include <QLCDNumber>
#include <QPushButton>

#include <pthread.h>
#include <string>


class SliderGui : public Rcs::AsyncWidget
{
public:
  SliderGui(double* q_des,
            double* q_curr,
            const char* title=NULL,
            double lowerBound=-1.0,
            double zeroPos=0.0,
            double upperBound=1.0,
            double ticSize=1.0,
            pthread_mutex_t* mutex=NULL);

  void construct();

protected:
  double* q_des;
  double* q_curr;
  std::string title;
  double lowerBound;
  double zeroPos;
  double upperBound;
  double ticSize;
  pthread_mutex_t* mutex;
};

class QwtSlider;

class Slider1Dof: public QFrame
{
  Q_OBJECT

public:
  static Slider1Dof* create(double* q_des,
                            double* q_curr,
                            const char* name = NULL,
                            double lowerBound  = -1.0,
                            double zeroPos     = 0.0,
                            double upperBound  = 1.0,
                            double ticSize     = 1.0,
                            pthread_mutex_t* mutex=NULL);

  Slider1Dof(double* q_des,
             double* q_curr,
             const char* name = NULL,
             double lowerBound  = -1.0,
             double zeroPos     = 0.0,
             double upperBound  = 1.0,
             double ticSize     = 1.0,
             pthread_mutex_t* mutex=NULL);

private:

  QPushButton* button_reset;
  QPushButton* button_null;
  QLCDNumber* lcd1;
  QLCDNumber* lcd2;
  QwtSlider* slider;
  double zeroPos;
  double* q_des;
  double* q_curr;
  pthread_mutex_t* mtx;
  void lock();
  void unlock();

private slots:

  void sliderReset();
  void sliderNull();
  void updateLcd1();
  void updateLcd2();

};

#endif   // SLIDER1DOF_H
