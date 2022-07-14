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

#ifndef CONTROLLERWIDGETBASE_H
#define CONTROLLERWIDGETBASE_H

#include "AsyncWidget.h"
#include "TaskWidget.h"

#include <ControllerBase.h>

#include <QScrollArea>
#include <QGroupBox>
#include <QCheckBox>
#include <QLabel>



namespace Rcs
{
class ControllerGui : public Rcs::AsyncWidget
{
public:
  ControllerGui(ControllerBase* cntrl,
                MatNd* a_des,
                MatNd* x_des,
                const MatNd* x_curr,
                pthread_mutex_t* lock_ = NULL,
                bool showOnly = false);

  ControllerGui(ControllerBase* cntrl,
                MatNd* a_des,
                const MatNd* a_curr,
                MatNd* x_des,
                const MatNd* x_curr,
                pthread_mutex_t* lock_ = NULL,
                bool showOnly = false);

  void construct();
  void reset(const MatNd* a_des, const MatNd* x_des);

protected:
  ControllerBase* cntrl;
  MatNd* a_des;
  const MatNd* a_curr;
  MatNd* x_des;
  const MatNd* x_curr;
  pthread_mutex_t* lock;
  bool showOnly;
};

/*! \ingroup RcsGui
 *  \brief Gui to display and / or modify task-level control variables. This
 *         gui is instantiated with a ControllerBase class. It will go trough
 *         all its tasks and create a TaskWidget for each of them. If a task
 *         has not been assigned a name (default is unnamed task), then it
 *         will be skipped.
 *
 *         The widget will display and modify the arrays pointed to by a_des
 *         and x_des. It will store the pointers internally. In order to ensure
 *         no concurrent access, the class will lock the passed mutex when any
 *         changes are done to the arrays pointed to. You need to make sure
 *         to lock the pointer access on any other usage.
 *
 *         The Gui shold be instantiated through the factory create methods.
 *         These will take care that is runs in it's own thread. Therefore
 *         the constructors of the Qt class are not exposed publically. Here
 *         is an example:
 *
 *         \code
 *         pthread_mutex_t mtx;
 *         pthread_mutex_init(&mtx, NULL);
 *         ControllerBase* c = new ControllerBase(...);
 *         int hndl;
 *         hndl = ControllerWidgetBase::create(c, a_des, x_des, x_curr, &mtx);
 *         ...
 *         ControllerWidgetBase::destroy(hndl);
 *         \endcode
 *
 *         The detroy method does not need to be called if you call the
 *         RcsGuiFactory_shutdown() method.
 */
class ControllerWidgetBase: public QScrollArea
{
  Q_OBJECT
public:
  static int create(ControllerBase* cntrl,
                    MatNd* a_des,
                    MatNd* x_des,
                    const MatNd* x_curr,
                    pthread_mutex_t* lock_=NULL,
                    bool showOnly = false);

  static int create(ControllerBase* cntrl,
                    MatNd* a_des,
                    const MatNd* a_curr,
                    MatNd* x_des,
                    const MatNd* x_curr,
                    pthread_mutex_t* lock_=NULL,
                    bool showOnly = false);

  static bool destroy(int handle);

  void registerCallback(TaskWidget::TaskChangeCallback* callback);
  void reset(const MatNd* a_des, const MatNd* x_des);

protected slots:

  virtual void showActiveTasks(int checkBoxState);
  virtual void setActive(int status);
  virtual void displayAct();

public:

  ControllerWidgetBase(const ControllerBase* cntrl,
                       MatNd* a_des,
                       const MatNd* a_curr,
                       MatNd* x_des,
                       const MatNd* x_curr,
                       pthread_mutex_t* lock,
                       bool showOnly);

  virtual ~ControllerWidgetBase();
protected:

  /*! \brief We overwrite this with an empty function, otherwise the widget
   *         scrolls when we use te mouse wheel inside the canvas and not
   *         above the scroll bar. This is inconvenient, since we also want
   *         use the mouse wheel for scrolling the task sliders. This behaviour
   *         only exists for windows.
   */
  void wheelEvent(QWheelEvent* e);

  static void* controllerGuiBase(void* arg);
  virtual QGroupBox* boxKinematicsInfo();
  virtual QGroupBox* boxControllerButtons();
  void lock();
  void unlock();
  const ControllerBase* _controller;
  bool showOnly;
  pthread_mutex_t* mutex;
  std::vector<TaskWidget*> taskWidgets;

  QLabel* label_stats;
  QCheckBox* check_active_gui;
};

}

#endif // CONTROLLERWIDGETBASE_H
