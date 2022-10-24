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

#ifndef TASKWIDGET_H
#define TASKWIDGET_H

#include <Task.h>

#include <QGroupBox>
#include <QLCDNumber>

#include <pthread.h>

class LcdSlider;
class QCheckBox;


class TaskWidget: public QGroupBox
{
  Q_OBJECT
public:
  TaskWidget(const Rcs::Task* task,
             double* a_des, double* x_des,
             const double* x_curr,
             pthread_mutex_t* lock=NULL, bool show_only=false);

  TaskWidget(const Rcs::Task* task,
             double* a_des, double* x_des, const double* ax_curr,
             const double* x_curr,
             pthread_mutex_t* lock=NULL, bool show_only=false);

  virtual ~TaskWidget();

  /*! \brief This callback is called each time the desired task values are
   *         modified by the Gui. It is the case if
   *         - the activation is not zero and the task slider is moved
   *         - the activation is zero and the desired values differ from
   *           the curent ones.
   */
  class TaskChangeCallback
  {
  public:
    virtual ~TaskChangeCallback()
    {
    }
    virtual void callback() = 0;
  };

  void registerCallback(TaskChangeCallback* callback);
  unsigned int getDim() const;

  /*! \brief This is thread-safe.
   */
  double getActivation();
  void reset(const double* a_des, const double* x_des);

public slots:
  void setActive(int status);
  void setConstraint();
  void displayAct();

  /*! \brief If the task is inactive, the function computes its values and
   *         displays them in the LCDs. Further, the sliders are set to the
   *         computed values. The computation of the task values is embraced
   *         in the controller's mutex.
   */
  void updateUnconstrainedControls();
  int getMaxLabelWidth();
  void setLabelWidth(int width);

private slots:
  void setActivation(double weight);
  void setTarget();
  void toggleActivationSliders(int checkBoxState);

private:
  void init(const Rcs::Task* task);
  QWidget* createActivationBox(const Rcs::Task* task);
  QWidget* createActivationSlider(const Rcs::Task* task);
  QWidget* createTaskComponent(const Rcs::Task* task, unsigned int idx,
                               bool withActivationLcd=false);
  double* a_des;
  double* x_des;
  const double* ax_curr;
  const double* x_curr;
  pthread_mutex_t* mutex;
  bool show_only;
  QCheckBox* check_activate;
  QCheckBox* check_activation;
  LcdSlider* activation_slider;
  std::vector<LcdSlider*> sliders;
  int maxLabelWidth;
  unsigned int dimTask;

  void lock();
  void unlock();
  std::vector<TaskChangeCallback*> callback;
};

#endif // TASKWIDGET_H
