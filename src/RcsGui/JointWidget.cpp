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

#include "JointWidget.h"
#include "JointSlider.h"
#include "SimpleSlider.h"

#include <Rcs_typedef.h>
#include <Rcs_macros.h>
#include <Rcs_joint.h>
#include <Rcs_guiFactory.h>

#include <qwt_slider.h>

#include <QLabel>
#include <QGridLayout>
#include <QTimer>

#include <cstdio>
#include <cmath>


namespace Rcs
{


/*******************************************************************************
 * Factory instantiation method for Qt thread.
 ******************************************************************************/
typedef struct
{
  void* ptr[10];
} VoidPointerList;


void* JointWidget::stateGui(void* arg)
{
  VoidPointerList* p = (VoidPointerList*) arg;
  RCHECK(arg);
  RcsGraph* graph      = (RcsGraph*)        p->ptr[0];
  const RcsGraph* constGraph = (const RcsGraph*)        p->ptr[1];
  pthread_mutex_t* mtx = (pthread_mutex_t*) p->ptr[2];
  MatNd* q_des         = (MatNd*)           p->ptr[3];
  MatNd* q_curr        = (MatNd*)           p->ptr[4];
  bool* alwaysWriteToQ = (bool*)            p->ptr[5];
  bool* passive        = (bool*)            p->ptr[6];

  delete p;

  JointWidget* w = new JointWidget(graph, constGraph, mtx, q_des, q_curr,
                                   *alwaysWriteToQ, *passive);
  w->show();

  delete alwaysWriteToQ;
  delete passive;

  return w;
}

/*******************************************************************************
 *
 ******************************************************************************/
int JointWidget::create(RcsGraph* graph, pthread_mutex_t* graphLock,
                        MatNd* q_des, MatNd* q_curr,
                        bool alwaysWriteToQ, bool passive)
{
  bool* _alwaysWriteToQ = new bool;
  *_alwaysWriteToQ = alwaysWriteToQ;

  bool* _passive = new bool;
  *_passive = passive;

  VoidPointerList* p = new VoidPointerList;
  p->ptr[0] = (void*)graph;
  p->ptr[1] = (void*)graph;
  p->ptr[2] = (void*) graphLock;
  p->ptr[3] = (void*) q_des;
  p->ptr[4] = (void*) q_curr;
  p->ptr[5] = (void*) _alwaysWriteToQ;
  p->ptr[6] = (void*) _passive;

  int handle = RcsGuiFactory_requestGUI(stateGui, p);

  return handle;
}

/*******************************************************************************
 *
 ******************************************************************************/
int JointWidget::create(const RcsGraph* graph, pthread_mutex_t* graphLock,
                        MatNd* q_des, MatNd* q_curr,
                        bool alwaysWriteToQ, bool passive)
{
  bool* _alwaysWriteToQ = new bool;
  *_alwaysWriteToQ = alwaysWriteToQ;

  bool* _passive = new bool;
  *_passive = passive;

  VoidPointerList* p = new VoidPointerList;
  p->ptr[0] = (void*)NULL;
  p->ptr[1] = (void*)graph;
  p->ptr[2] = (void*)graphLock;
  p->ptr[3] = (void*)q_des;
  p->ptr[4] = (void*)q_curr;
  p->ptr[5] = (void*)_alwaysWriteToQ;
  p->ptr[6] = (void*)_passive;

  int handle = RcsGuiFactory_requestGUI(stateGui, p);

  return handle;
}

/*******************************************************************************
 * Static destroy method.
 ******************************************************************************/
bool JointWidget::destroy(int handle)
{
  return RcsGuiFactory_destroyGUI(handle);
}

/*******************************************************************************
 *
 ******************************************************************************/
JointWidget::JointWidget(RcsGraph* graph, const RcsGraph* constGraph,
                         pthread_mutex_t* graphLock,
                         MatNd* q_des, MatNd* q_curr, bool alwaysWriteToQ,
                         bool passive) :
  QScrollArea(),
  _graph(graph),
  _constGraph(constGraph),
  _togglestate(false),
  _alwaysWriteToQ(alwaysWriteToQ),
  _q_des(q_des ? q_des : constGraph->q),
  _q_curr(q_curr ? q_curr : constGraph->q),
  mutex(graphLock)
{
  RLOG(5, "Creating Widget with %d dof", _constGraph->dof);

  setWindowTitle("RCS Joint Control");

  QWidget* scrollWidget = new QWidget(this);

  if (_constGraph->dof > 0)
  {
    QPalette palette;
    palette.setColor(QPalette::Normal, QPalette::Foreground, Qt::red);
    palette.setColor(QPalette::Normal, QPalette::Background, Qt::black);
    palette.setColor(QPalette::Normal, QPalette::Light, Qt::yellow);
    palette.setColor(QPalette::Normal, QPalette::Dark, Qt::darkYellow);

    palette.setColor(QPalette::Inactive, QPalette::Foreground, Qt::red);
    palette.setColor(QPalette::Inactive, QPalette::Background, Qt::black);
    palette.setColor(QPalette::Inactive, QPalette::Light, Qt::yellow);
    palette.setColor(QPalette::Inactive, QPalette::Dark, Qt::darkYellow);

    //
    // Checkboxes for constraints
    //
    QGridLayout* constraintsLayout = new QGridLayout;

    int i = 0;
    RCSGRAPH_TRAVERSE_JOINTS(_constGraph)
    {
      RLOG(5, "Creating Widget for joint \"%s\"", JNT->name);

      QCheckBox* cc = new QCheckBox(JNT->name);
      cc->setChecked(JNT->constrained);
      check_constraints.push_back(cc);

      if (graph != NULL)
      {
        if (JNT->coupledTo == NULL)
        {
          connect(check_constraints[i], SIGNAL(clicked()),
                  SLOT(setConstraint()));
        }
      }
      else
      {
        check_constraints[i]->setEnabled(false);
      }

      QLCDNumber* lcd_cmd = new QLCDNumber(7);
      lcd_cmd->setAutoFillBackground(true);
      lcd_cmd->setFixedHeight(20);
      lcd_cmd->setPalette(palette);
      lcd_q_cmd.push_back(lcd_cmd);

      QLCDNumber* lcd_act = new QLCDNumber(7);
      lcd_act->setAutoFillBackground(true);
      lcd_act->setFixedHeight(20);
      lcd_act->setPalette(palette);
      lcd_q_act.push_back(lcd_act);

      // Here the joint scale
      double lb = JNT->q_min;
      double ub = JNT->q_max;
      double range = ub - lb;
      double qi = MatNd_get(_q_des, JNT->jointIndex, 0);
      double scaleFactor = RcsJoint_isRotation(JNT) ? 180.0/M_PI : 1000.0;

      JointSlider* jsl = new JointSlider(lb-0.1*range, qi, ub+0.1*range, scaleFactor);
      jsc_q.push_back(jsl);

      if (passive == false)
      {
        connect(jsc_q[i]->getSlider(), SIGNAL(valueChanged(double)),
                SLOT(setJoint()));
      }

      constraintsLayout->addWidget(check_constraints[i], i, 0,
                                   Qt::AlignLeft);
      constraintsLayout->addWidget(lcd_q_cmd[i], i, 1, Qt::AlignLeft);
      constraintsLayout->addWidget(lcd_q_act[i], i, 2, Qt::AlignLeft);
      constraintsLayout->addWidget(jsc_q[i],     i, 3, Qt::AlignLeft);

      i++;
    }   // for(i=0;i<_dof;i++)


    scrollWidget->setLayout(constraintsLayout);

  }   // if(_constGraph->dof>0)

  else
  {
    RLOG(4, "No joints in graph - skipping state widget");
    QLabel* label = new QLabel("no joints in graph");
    QFont font("Helvetica", 12, QFont::Bold);
    label->setFont(font);
    QHBoxLayout* layout = new QHBoxLayout;
    layout->addWidget(label);
    scrollWidget->setLayout(layout);
  }

  //
  // Put some scrollbars to the widget
  //
  scrollWidget->resize(scrollWidget->sizeHint());
  setWidget(scrollWidget);
  setWidgetResizable(true);

  //
  // 25 Hz timer callback
  //
  QTimer* _timer = new QTimer(this);
  connect(_timer, SIGNAL(timeout()), SLOT(displayAct()));
  _timer->start(40);

  RLOG(5, "JointWidget generated");
}

/*******************************************************************************
 *
 ******************************************************************************/
JointWidget::~JointWidget()
{
}

/*******************************************************************************
 *
 ******************************************************************************/
void JointWidget::toggle()
{
  _togglestate = !_togglestate;

  if (_togglestate)
  {
    show();
  }
  else
  {
    hide();
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void JointWidget::displayAct(void)
{
  size_t i = 0;
  char a[9];
  double scaleFactor;

  RCSGRAPH_TRAVERSE_JOINTS(_constGraph)
  {
    scaleFactor = RcsJoint_isRotation(JNT) ? 180.0 / M_PI : 1000.0;

    if (i<lcd_q_act.size())
    {
      snprintf(a, 8, "%5.1f", scaleFactor*_q_curr->ele[JNT->jointIndex]);
      lcd_q_act[i]->display(a);
    }

    if (i<lcd_q_cmd.size())
    {
      snprintf(a, 8, "%5.1f", scaleFactor*_q_des->ele[JNT->jointIndex]);
      lcd_q_cmd[i]->display(a);
    }

    if (i<jsc_q.size())
    {
      jsc_q[i]->setValue(_q_curr->ele[JNT->jointIndex]);
    }

    if ((_graph == NULL) && (i<check_constraints.size()))
    {
      check_constraints[i]->setChecked(JNT->constrained);
    }

    i++;
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
void JointWidget::setConstraint(void)
{
  size_t i = 0;
  lock();

  // Assign values from constraint check boxes
  RCHECK(_graph);
  RCSGRAPH_TRAVERSE_JOINTS(_graph)
  {
    if (i < check_constraints.size())
    {
      JNT->constrained = check_constraints[i]->isChecked();
    }
    i++;
  }

  unlock();
}

/*******************************************************************************
 * Assign values to _q_des from sliders
 ******************************************************************************/
void JointWidget::setJoint(void)
{
  size_t i = 0;
  lock();

  RCSGRAPH_TRAVERSE_JOINTS(_constGraph)
  {
    if ((JNT->constrained || _alwaysWriteToQ) && (!JNT->coupledTo) && (i < jsc_q.size()))
    {
      _q_des->ele[JNT->jointIndex] = jsc_q[i]->getSliderValue();
    }

    i++;
  }

  unlock();

  for (size_t i=0; i<callback.size(); ++i)
  {
    callback[i]->callback();
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
void JointWidget::lock() const
{
  if (this->mutex)
  {
    pthread_mutex_lock(this->mutex);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void JointWidget::unlock() const
{
  if (this->mutex)
  {
    pthread_mutex_unlock(this->mutex);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void JointWidget::reset(const MatNd* q)
{
  MatNd_copy(_q_des, q);
  MatNd_copy(_q_curr, q);

  for (unsigned int i=0; i<q->m; ++i)
  {
    jsc_q[i]->setValue(q->ele[i]);
    jsc_q[i]->setSliderValue(q->ele[i]);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void JointWidget::registerCallback(JointChangeCallback* cb)
{
  callback.push_back(cb);
}

/*******************************************************************************
 * This overrides the behavior that the scroll area scrolls whenever the
 * mouse wheel is used inside it. This is a bit disturbing, since the sliders
 * inside the scroll area then cannot be scrolled accurately using the mouse
 * wheel. For some reason, this issue only exists on Windows.
 ******************************************************************************/
void JointWidget::wheelEvent(QWheelEvent* e)
{
}

}   // namespace Rcs
