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

#include "ControllerWidgetBase.h"
#include "TaskWidget.h"
#include "LcdSlider.h"

#include <Rcs_typedef.h>
#include <Rcs_macros.h>
#include <Rcs_guiFactory.h>

#include <QTimer>
#include <QGridLayout>
#include <QVBoxLayout>

#include <algorithm>

namespace Rcs
{

typedef struct
{
  void* ptr[10];
} VoidPointerList;



/*******************************************************************************
 * Thread function.
 ******************************************************************************/
void* ControllerWidgetBase::controllerGuiBase(void* arg)
{
  VoidPointerList* p = (VoidPointerList*) arg;
  RCHECK(arg);

  Rcs::ControllerBase* cntrl = (Rcs::ControllerBase*) p->ptr[0];
  MatNd* a_des                 = (MatNd*) p->ptr[1];
  MatNd* x_des                 = (MatNd*) p->ptr[2];
  const MatNd* x_curr          = (MatNd*) p->ptr[3];
  pthread_mutex_t* mutex       = (pthread_mutex_t*) p->ptr[4];
  bool* showOnly               = (bool*) p->ptr[5];
  MatNd* a_curr                = (MatNd*) p->ptr[6];

  delete p;

  ControllerWidgetBase* widget =
    new ControllerWidgetBase(cntrl, a_des, a_curr, x_des, x_curr, mutex, *showOnly);

  widget->show();

  delete showOnly;

  return widget;
}

/*******************************************************************************
 *
 ******************************************************************************/
int ControllerWidgetBase::create(ControllerBase* c,
                                 MatNd* a_des,
                                 MatNd* x_des,
                                 const MatNd* x_curr,
                                 pthread_mutex_t* mutex,
                                 bool showOnly)
{
  VoidPointerList* p = new VoidPointerList;
  p->ptr[0] = (void*) c;
  p->ptr[1] = (void*) a_des;
  p->ptr[2] = (void*) x_des;
  p->ptr[3] = (void*) x_curr;
  p->ptr[4] = (void*) mutex;

  bool* _showOnly = new bool;
  *_showOnly = showOnly;
  p->ptr[5] = (void*) _showOnly;
  p->ptr[6] = (void*) NULL;

  int handle =
    RcsGuiFactory_requestGUI(ControllerWidgetBase::controllerGuiBase, (void*) p);

  return handle;
}

/*******************************************************************************
 *
 ******************************************************************************/
int ControllerWidgetBase::create(ControllerBase* c,
                                 MatNd* a_des,
                                 const MatNd* a_curr,
                                 MatNd* x_des,
                                 const MatNd* x_curr,
                                 pthread_mutex_t* mutex,
                                 bool showOnly)
{
  VoidPointerList* p = new VoidPointerList;
  p->ptr[0] = (void*) c;
  p->ptr[1] = (void*) a_des;
  p->ptr[2] = (void*) x_des;
  p->ptr[3] = (void*) x_curr;
  p->ptr[4] = (void*) mutex;

  bool* _showOnly = new bool;
  *_showOnly = showOnly;
  p->ptr[5] = (void*) _showOnly;
  p->ptr[6] = (void*) a_curr;

  int handle =
    RcsGuiFactory_requestGUI(ControllerWidgetBase::controllerGuiBase, (void*) p);

  return handle;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool ControllerWidgetBase::destroy(int handle)
{
  return RcsGuiFactory_destroyGUI(handle);
}

/*******************************************************************************
 *
 ******************************************************************************/
ControllerWidgetBase::ControllerWidgetBase(const ControllerBase* cntrl,
                                           MatNd* a_des,
                                           MatNd* a_curr,
                                           MatNd* x_des,
                                           const MatNd* x_curr,
                                           pthread_mutex_t* mutex_,
                                           bool showOnly):
  QScrollArea(),
  _controller(cntrl),
  showOnly(showOnly),
  mutex(mutex_)
{
  int max_label_width = 0;

  QTimer* timer = new QTimer(this);
  setWindowTitle("Generic task space controller");

  // remove redundant frame
  setFrameShape(QFrame::NoFrame);

  // The layout for the overall task widget
  QVBoxLayout* taskLayout = new QVBoxLayout(this);

  // Widget for controller related sliders and some information
  QVBoxLayout* sliderLayout = new QVBoxLayout;
  sliderLayout->setMargin(1);
  sliderLayout->setSpacing(1);
  sliderLayout->addWidget(boxKinematicsInfo());
  sliderLayout->addWidget(boxControllerButtons());

  // Add layout to widget
  QGroupBox* gbox_controllerSliders = new QGroupBox(tr("Controller settings"), this);
  gbox_controllerSliders->setStyleSheet("QGroupBox { font-weight: bold; }");
  gbox_controllerSliders->setLayout(sliderLayout);
  taskLayout->addWidget(gbox_controllerSliders, 0, 0);

  unsigned int rowIndex = 0;

  for (size_t id = 0; id < _controller->getNumberOfTasks(); id++)
  {
    const Task* tsk = _controller->getTask(id);

    if (tsk->getName() != "Unnamed task")
    {
      TaskWidget* task_widget;

      bool showTask = false;

      for (size_t tskIdx = 0; tskIdx < tsk->getDim(); ++tskIdx)
      {
        const Task::Parameters& param = tsk->getParameter(tskIdx);
        if (param.maxVal - param.minVal > 0.0)
        {
          showTask = true;
        }
      }
      if (showTask)
      {
        if (a_curr == NULL)
        {
          task_widget = new TaskWidget(tsk,
                                       &a_des->ele[id],
                                       &x_des->ele[rowIndex],
                                       &x_curr->ele[rowIndex],
                                       mutex_, showOnly);
        }
        else
        {
          task_widget = new TaskWidget(tsk,
                                       &a_des->ele[id],
                                       &x_des->ele[rowIndex],
                                       &a_curr->ele[rowIndex],
                                       &x_curr->ele[rowIndex],
                                       mutex_, showOnly);
        }

        max_label_width = std::max(task_widget->getMaxLabelWidth(), max_label_width);
        this->taskWidgets.push_back(task_widget);
        taskLayout->addWidget(task_widget);
        connect(check_active_gui, SIGNAL(stateChanged(int)),
                task_widget, SLOT(setActive(int)));
        connect(timer, SIGNAL(timeout()), task_widget, SLOT(updateUnconstrainedControls()));
        connect(timer, SIGNAL(timeout()), task_widget, SLOT(displayAct()));
      }


    }   // if (showTask)

    rowIndex += _controller->getTask(id)->getDim();
  }

  // all lcd sliders have been added, let's adjust the label size
  RLOG(5, "max_label_width: %d", max_label_width);
  max_label_width += 5;

  for (size_t i=0; i<taskWidgets.size(); i++)
  {
    taskWidgets[i]->setLabelWidth(max_label_width);
  }

  taskLayout->addStretch();

  QWidget* scrollWidget = new QWidget(this);
  scrollWidget->setLayout(taskLayout);
  this->setWidget(scrollWidget);
  this->setWidgetResizable(true);

  connect(timer, SIGNAL(timeout()), SLOT(displayAct()));

  timer->start(100);
  setActive(showOnly ? Qt::Unchecked : Qt::Checked);

  const int width = 750;
  const int maxHeight = 1000;
  int height = widget()->sizeHint().height();
  resize(width, height < maxHeight ? height : maxHeight);

  showActiveTasks(Qt::Unchecked);

  RLOG(5, "ControllerWidgetBase generated");
}

/*******************************************************************************
 *
 ******************************************************************************/
ControllerWidgetBase::~ControllerWidgetBase()
{
  RLOG(5, "Destroying ControllerWidgetBase");
}

/*******************************************************************************
 *
 ******************************************************************************/
QGroupBox* ControllerWidgetBase::boxKinematicsInfo()
{
  // Elements are aligned horizontally
  QHBoxLayout* layout = new QHBoxLayout;

  // Label with kinematics information
  this->label_stats = new QLabel(this);
  this->label_stats->setFrameStyle(QFrame::Panel | QFrame::Sunken);

  layout->addWidget(this->label_stats, Qt::AlignLeft);
  layout->addStretch();

  QGroupBox* gbox = new QGroupBox(this);
  gbox->setLayout(layout);

  return gbox;
}

/*******************************************************************************
 *
 ******************************************************************************/
QGroupBox* ControllerWidgetBase::boxControllerButtons()
{
  // Elements are aligned horizontally
  QHBoxLayout* StatsLayout = new QHBoxLayout;
  StatsLayout->setAlignment(Qt::AlignLeft);

  // Check box to display active / all tasks
  QCheckBox* check_activeTasks = new QCheckBox("Show only active tasks", this);
  check_activeTasks->setChecked(true);
  StatsLayout->addWidget(check_activeTasks);

  // Check box to make GUI active or passive
  this->check_active_gui = new QCheckBox("GUI Active", this);
  check_active_gui->setChecked(!showOnly);
  StatsLayout->addWidget(this->check_active_gui);

  // Add layout to widget
  QGroupBox* gbox_controllerButtons = new QGroupBox(this);
  gbox_controllerButtons->setLayout(StatsLayout);

  // Connect functionality
  connect(this->check_active_gui,  SIGNAL(stateChanged(int)),
          SLOT(setActive(int)));
  connect(check_activeTasks, SIGNAL(stateChanged(int)),
          SLOT(showActiveTasks(int)));

  return gbox_controllerButtons;
}

/*******************************************************************************
 *
 ******************************************************************************/
void ControllerWidgetBase::displayAct()
{
  char a[256];

  lock();
  snprintf(a, 256, "Dofs: %d/%d  Tasks: %zu",
           _controller->getGraph()->nJ,
           _controller->getGraph()->dof,
           _controller->getTaskDim());
  unlock();

  this->label_stats->setText(a);
}

/*******************************************************************************
 *
 ******************************************************************************/
void ControllerWidgetBase::setActive(int checkBoxState)
{

  switch (checkBoxState)
  {
    case Qt::Unchecked:
      this->showOnly = true;
      break;

    case Qt::Checked:
      this->showOnly = false;
      break;

    default:
      RLOG(1, "Unknown check button state: %d", checkBoxState);
  }

}

/*******************************************************************************
 * Shows all task widgets of tasks with activation > 0.
 ******************************************************************************/
void ControllerWidgetBase::showActiveTasks(int checkBoxState)
{
  std::vector<TaskWidget*>::iterator it;

  for (it = taskWidgets.begin(); it != taskWidgets.end(); ++it)
  {
    if (checkBoxState == Qt::Checked)
    {
      if ((*it)->getActivation()==0.0)
      {
        (*it)->hide();
      }
    }
    else
    {
      (*it)->show();
    }
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void ControllerWidgetBase::lock()
{
  if (this->mutex != NULL)
  {
    pthread_mutex_lock(this->mutex);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void ControllerWidgetBase::unlock()
{
  if (this->mutex != NULL)
  {
    pthread_mutex_unlock(this->mutex);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void ControllerWidgetBase::registerCallback(TaskWidget::TaskChangeCallback* callback)
{
  for (size_t i=0; i<taskWidgets.size(); ++i)
  {
    taskWidgets[i]->registerCallback(callback);
  }
}

/*******************************************************************************
 * Reset with externally given activation and task vector. \todo: ax is missing
 ******************************************************************************/
void ControllerWidgetBase::reset(const MatNd* a, const MatNd* x)
{
  unsigned int rowIndex = 0;

  for (size_t id = 0; id < taskWidgets.size(); id++)
  {
    taskWidgets[id]->reset(&a->ele[id], &x->ele[rowIndex]);
    rowIndex += taskWidgets[id]->getDim();
  }

}

/*******************************************************************************
 * This overrides the behavior that the scroll area scrolls whenever the
 * mouse wheel is used inside it. This is a bit disturbing, since the sliders
 * inside the scroll area then cannot be scrolled accurately using the mouse
 * wheel. For some reason, this issue only exists on Windows.
 ******************************************************************************/
void ControllerWidgetBase::wheelEvent(QWheelEvent* e)
{
}


}   // namespace Rcs
