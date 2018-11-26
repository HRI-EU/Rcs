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

#ifndef CONTROLLERWIDGETBASE_H
#define CONTROLLERWIDGETBASE_H

#include "TaskWidget.h"

#include <ControllerBase.h>

#include <QScrollArea>


class LcdSlider;
class QCheckBox;
class QGroupBox;
class QLabel;


namespace Rcs
{

class ControllerWidgetBase: public QScrollArea
{
  Q_OBJECT
public:
  static void* controllerGuiBase(void* arg);
  static int create(ControllerBase* cntrl,
                    MatNd* a_des,
                    MatNd* x_des,
                    const MatNd* x_curr,
                    pthread_mutex_t* lock_=NULL,
                    bool showOnly = false);

  static int create(ControllerBase* cntrl,
                    MatNd* a_des,
                    MatNd* a_curr,
                    MatNd* x_des,
                    const MatNd* x_curr,
                    pthread_mutex_t* lock_=NULL,
                    bool showOnly = false);

  static bool destroy(int handle);

  ControllerWidgetBase();

  ControllerWidgetBase(ControllerBase* cntrl,
                       MatNd* a_des,
                       MatNd* a_curr,
                       MatNd* x_des,
                       const MatNd* x_curr,
                       pthread_mutex_t* lock_=NULL,
                       bool showOnly = false);

  virtual ~ControllerWidgetBase();

  void registerCallback(TaskWidget::TaskChangeCallback* callback);
  void reset(const MatNd* a_des, const MatNd* x_des);

protected slots:
  virtual void showActiveTasks(int checkBoxState);
  virtual void setActive(int status);
  virtual void displayAct();

protected:
  virtual QGroupBox* boxKinematicsInfo();
  virtual QGroupBox* boxControllerButtons();
  void lock();
  void unlock();
  ControllerBase* _controller;
  bool showOnly;
  pthread_mutex_t* mutex;
  std::vector<TaskWidget*> taskWidgets;

  QLabel* label_stats;
  QCheckBox* check_active_gui;
};

}

#endif // CONTROLLERWIDGETBASE_H
