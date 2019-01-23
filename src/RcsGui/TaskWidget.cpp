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

#include "TaskWidget.h"
#include "LcdSlider.h"

#include <Rcs_macros.h>
#include <Rcs_VecNd.h>

#include <QGridLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QCheckBox>
#include <QLabel>
#include <QTimer>

#include <vector>
#include <algorithm>


using namespace Rcs;

/******************************************************************************
  \brief Constructor.
******************************************************************************/
TaskWidget::TaskWidget(const Task* task,
                       double* a_des_,
                       double* x_des_,
                       const double* x_curr_,
                       pthread_mutex_t* mutex_,
                       bool show_only_):
  QGroupBox(),
  a_des(a_des_),
  x_des(x_des_),
  ax_curr(NULL),
  x_curr(x_curr_),
  mutex(mutex_),
  show_only(show_only_),
  maxLabelWidth(0),
  dimTask(task->getDim())
{
  init(task);
  RLOG(5, "TaskWidget \"%s\" generated", task->getName().c_str());
}

/******************************************************************************
  \brief Constructor.
******************************************************************************/
TaskWidget::TaskWidget(const Task* task,
                       double* a_des_,
                       double* x_des_,
                       const double* ax_curr_,
                       const double* x_curr_,
                       pthread_mutex_t* mutex_,
                       bool show_only):
  QGroupBox(),
  a_des(a_des_),
  x_des(x_des_),
  ax_curr(ax_curr_),
  x_curr(x_curr_),
  mutex(mutex_),
  show_only(show_only),
  maxLabelWidth(0),
  dimTask(task->getDim())
{
  init(task);
  RLOG(5, "TaskWidget \"%s\" generated", task->getName().c_str());
}


/******************************************************************************
  \brief Destructor.
******************************************************************************/
TaskWidget::~TaskWidget()
{
}


/******************************************************************************
  \brief Default initializations.
******************************************************************************/
void TaskWidget::init(const Rcs::Task* task)
{
  RLOG(5, "Adding task %s", task->getName().c_str());

  // Box for the _dim task lines
  QVBoxLayout* mainGrid = new QVBoxLayout();
  mainGrid->setMargin(0);
  mainGrid->setSpacing(0);

  mainGrid->addWidget(createActivationBox(task));
  mainGrid->addWidget(createActivationSlider(task));

  // Sliders for task target values
  for (size_t i = 0; i < this->dimTask; i++)
  {
    mainGrid->addWidget(createTaskComponent(task, i, true));
  }

  setLayout(mainGrid);

  this->setStyleSheet("QCheckBox { font-weight: bold; }"
                      "QGroupBox { font-weight: bold; color: gray; }"
                      "QLabel { font-size: 12px; }"
                      "QProgressBar {border: 1px solid black; border-radius: 1px; "
                      "text-align: center; font-size: 11px; }"
                     );

  displayAct();
  setTarget();

  setActive(this->show_only ? Qt::Unchecked : Qt::Checked);
}



/*******************************************************************************
 *
******************************************************************************/
unsigned int TaskWidget::getDim() const
{
  return this->dimTask;
}


/******************************************************************************
  \brief Box for slider activations.
******************************************************************************/
QWidget* TaskWidget::createActivationBox(const Rcs::Task* task)
{
  QWidget* sub_box = new QWidget();

  // HBox for active, activation and lambda checkboxes
  QHBoxLayout* check_grid = new QHBoxLayout();
  check_grid->setAlignment(Qt::AlignLeft);
  sub_box->setLayout(check_grid);

  // Checkbox to activate task
  QString nameLabel = QString::fromStdString(task->getName()) +
                      QString(" [Task::") + QString::fromStdString(task->getClassName()) +
                      QString("]");
  this->check_activate = new QCheckBox(nameLabel);
  this->check_activate->setStyleSheet("QCheckBox { font-weight: bold; }");
  this->check_activate->setChecked(*a_des>0.0);
  this->check_activate->setToolTip("Toggles activation between zero and one");
  check_grid->addWidget(this->check_activate);
  connect(check_activate, SIGNAL(stateChanged(int)), SLOT(setConstraint()));

  check_grid->addStretch();

  // Checkbox to toggle activation slider
  this->check_activation = new QCheckBox("Show activation");
  this->check_activation->setChecked(false);
  check_grid->addWidget(this->check_activation);
  connect(this->check_activation, SIGNAL(stateChanged(int)),
          SLOT(toggleActivationSliders(int)));

  return sub_box;
}


/******************************************************************************
  \brief Slider to adjust the activation values.
******************************************************************************/
QWidget* TaskWidget::createActivationSlider(const Rcs::Task* task)
{
  this->activation_slider = new LcdSlider(0.0, *a_des, 1.0, 1.0, 0.01,
                                          "Activation");
  this->activation_slider->setToolTip("Continuously change activation between zero and one");
  this->maxLabelWidth = std::max(activation_slider->labelWidthHint(),
                                 maxLabelWidth);
  this->activation_slider->hide();
  this->activation_slider->updateLcd1FromSlider();
  connect(this->activation_slider, SIGNAL(valueChanged(double)),
          SLOT(setActivation(double)));

  return this->activation_slider;
}


/******************************************************************************
  \brief Task slider for one component.
******************************************************************************/
QWidget* TaskWidget::createTaskComponent(const Rcs::Task* task,
                                         unsigned int idx,
                                         bool withActivationLcd)
{
  Task::Parameters* param = task->getParameter(idx);
  double range = (param->maxVal - param->minVal) * param->scale_factor;
  double tick_size = 1.0;

  if (range > 0.0)
  {
    while (range < 1000.0)
    {
      tick_size *= 0.1;
      range *= 10.0;
    }
  }

  // Here we handle the case that x_curr is out of the range given by the
  // parameter struct. In this case, we set the center to x_curr, and set the
  // range limits to +/- the half range.
  double lowerBound = param->minVal;
  double upperBound = param->maxVal;

  if (this->x_curr[idx] > param->maxVal)
  {
    upperBound = this->x_curr[idx] + 0.5*(param->maxVal-param->minVal);
  }
  if (this->x_curr[idx] < param->minVal)
  {
    lowerBound = this->x_curr[idx] - 0.5*(param->maxVal-param->minVal);
  }

  LcdSlider* slider = new LcdSlider(lowerBound, this->x_curr[idx],
                                    upperBound, param->scale_factor,
                                    tick_size, param->name.c_str(),
                                    withActivationLcd,
                                    false);

  this->maxLabelWidth = std::max(slider->labelWidthHint(), maxLabelWidth);
  sliders.push_back(slider);
  connect(slider, SIGNAL(valueChanged(double)), SLOT(setTarget()));

  return slider;
}


/******************************************************************************
  \brief Shows and hides gui elements depending on if the GUI is active or
         passive. In passive mode, no GUI element that modifies the controller
         is shown.
******************************************************************************/
void TaskWidget::setActive(int checkBoxState)
{
  std::vector<LcdSlider*>::iterator it;
  switch (checkBoxState)
  {
    case Qt::Unchecked:
      this->show_only = true;
      this->activation_slider->setActive(false);
      for (it = this->sliders.begin(); it != this->sliders.end(); ++it)
      {
        (*it)->setActive(false);
      }
      break;

    case Qt::Checked:
      this->show_only = false;
      this->activation_slider->setActive(true);
      for (it = this->sliders.begin(); it != this->sliders.end(); ++it)
      {
        (*it)->setActive(true);
      }
      break;

    default:
      RLOG(1, "Unknown check button state: %d", checkBoxState);
  }

}


/******************************************************************************
  \brief If the task is inactive, the command values will be set to the
         current values. This is convenient since activating any task
         will not lead to a change of the movement.
******************************************************************************/
void TaskWidget::updateUnconstrainedControls()
{

  if (this->show_only)
  {
    return;
  }

  lock();
  if (*this->a_des==0.0)
  {
    VecNd_copy(x_des, x_curr, this->dimTask);
  }
  unlock();
}


/*******************************************************************************
 * Reset with externally given activation and task vector. \todo: ax is missing
 ******************************************************************************/
void TaskWidget::reset(const double* a, const double* x)
{
  lock();

  VecNd_copy(this->a_des, a, 1);
  VecNd_copy(this->x_des, x, this->dimTask);

  double activation = *this->a_des;
  this->check_activate->setChecked(activation > 0.0 ? true : false);
  this->activation_slider->setValueLcd2(activation);
  this->activation_slider->setSliderValue(activation);

  for (size_t i = 0; i < this->dimTask; i++)
  {
    this->sliders[i]->setValueLcd2(this->x_des[i]);
    this->sliders[i]->setValueLcd1(this->x_des[i]);
    this->sliders[i]->setSliderValue(this->x_des[i]);
  }

  unlock();
}

/*******************************************************************************
 * Displays task target and current values.
******************************************************************************/
void TaskWidget::displayAct()
{
  double* x_curr_tmp = RNSTALLOC(this->dimTask, double);
  double* x_des_tmp  = RNSTALLOC(this->dimTask, double);
  double activation;

  lock();
  activation = *this->a_des;
  VecNd_copy(x_curr_tmp, this->x_curr, this->dimTask);
  VecNd_copy(x_des_tmp, this->x_des, this->dimTask);
  unlock();

  // setChecked triggers setConstraint, which in turn sets the TaskActivation
  // to 1, even if we just move the slider slightly -> TaskActivation needs to
  // be reset
  this->check_activate->setChecked(activation>0.0);
  this->activation_slider->setValueLcd2(activation);
  this->activation_slider->setSliderValue(activation);

  for (size_t i = 0; i < this->dimTask; i++)
  {
    this->sliders[i]->setValueLcd2(x_curr_tmp[i]);
    this->sliders[i]->setValueLcd1(x_des_tmp[i]);
    this->sliders[i]->setSliderValue(x_des_tmp[i]);

    if (this->ax_curr != NULL)
    {
      this->sliders[i]->setValueLcd3(ax_curr[i]);
    }
  }

}

/******************************************************************************

*******************************************************************************/
int TaskWidget::getMaxLabelWidth()
{
  return this->maxLabelWidth;
}

/******************************************************************************

*******************************************************************************/
void TaskWidget::setLabelWidth(int width)
{
  this->activation_slider->setLabelWidth(width);
  for (size_t i=0; i<this->sliders.size(); i++)
  {
    this->sliders[i]->setLabelWidth(width);
  }
}

/******************************************************************************
  \brief Updates the target values when the slider is moved.
******************************************************************************/
void TaskWidget::setTarget()
{
  if (this->show_only)
  {
    return;
  }

  lock();
  double activation = *this->a_des;
  unlock();

  if (activation<=0.0)
  {
    return;
  }

  if (this->dimTask==0)
  {
    return;
  }

  double* target = RNSTALLOC(this->dimTask, double);

  for (unsigned int i = 0; i < this->dimTask; i++)
  {
    target[i] = this->sliders[i]->getSliderValue();
    this->sliders[i]->setValueLcd1(target[i]);
  }

  lock();
  VecNd_copy(this->x_des, target, this->dimTask);
  unlock();

  for (size_t i=0; i<callback.size(); ++i)
  {
    callback[i]->callback();
  }
}

/******************************************************************************
  \brief Returns the activation.
******************************************************************************/
double TaskWidget::getActivation()
{
  lock();
  double activation = *a_des;
  unlock();

  return activation;
}

/******************************************************************************
  \brief Sets the tasks active flag

         If we deactivate the task, the activation is set to zero and the
         corresponding slider is updated.
******************************************************************************/
void TaskWidget::setConstraint()
{
  if (this->show_only)
  {
    return;
  }

  if (this->check_activate->isChecked())
  {
    lock();
    *this->a_des = 1.0;
    unlock();

    this->activation_slider->setSliderValue(1.0);
  }
  else
  {
    lock();
    *this->a_des = 0.0;
    unlock();

    this->activation_slider->setSliderValue(0.0);
  }
}


/******************************************************************************
  \brief Changes the gain of the task controller. Since it is
         connected with the signal-slot mechanism, we have to put a mutex
         around it to avoid having this being called while the control
         computation is being carried out.
******************************************************************************/
void TaskWidget::setActivation(double activation)
{
  if (this->show_only)
  {
    return;
  }

  lock();
  *this->a_des = activation;
  unlock();
}


/******************************************************************************
  \brief Sets the visibility of the activation sliders.
******************************************************************************/
void TaskWidget::toggleActivationSliders(int checkBoxState)
{
  if (this->show_only)
  {
    return;
  }

  switch (checkBoxState)
  {
    case Qt::Unchecked:
      this->activation_slider->hide();
      break;

    case Qt::Checked:
      this->activation_slider->show();
      break;

    default:
      RLOG(1, "Unknown check button state: %d", checkBoxState);
  }

}

/******************************************************************************
  \brief Locks the mutex, if present.
******************************************************************************/
void TaskWidget::lock()
{
  if (this->mutex != NULL)
  {
    pthread_mutex_lock(this->mutex);
  }
}

/******************************************************************************
  \brief Unlocks the mutex, if present.
******************************************************************************/
void TaskWidget::unlock()
{
  if (this->mutex != NULL)
  {
    pthread_mutex_unlock(this->mutex);
  }
}


/*******************************************************************************
 *
 ******************************************************************************/
void TaskWidget::registerCallback(TaskChangeCallback* cb)
{
  callback.push_back(cb);
}
