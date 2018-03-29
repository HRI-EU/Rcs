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

#include "JointWidget.h"
#include "JointSlider.h"

#include <Rcs_typedef.h>
#include <Rcs_macros.h>
#include <Rcs_joint.h>
#include <Rcs_guiFactory.h>

#include <qwt_slider.h>

#include <QApplication>
#include <QLabel>
#include <QLayout>
#include <QPushButton>
#include <QTabWidget>
#include <QGridLayout>

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


static void* stateGui(void* arg)
{
  VoidPointerList* p = (VoidPointerList*) arg;
  RCHECK(arg);
  RcsGraph* graph      = (RcsGraph*)        p->ptr[0];
  pthread_mutex_t* mtx = (pthread_mutex_t*) p->ptr[1];
  MatNd* q_des         = (MatNd*)           p->ptr[2];
  MatNd* q_curr        = (MatNd*)           p->ptr[3];
  bool* alwaysWriteToQ = (bool*)            p->ptr[4];
  bool* passive        = (bool*)            p->ptr[5];

  delete p;

  JointWidget* w = new JointWidget(graph, mtx, q_des, q_curr, *alwaysWriteToQ,
                                   *passive);
  w->show();

  delete alwaysWriteToQ;
  delete passive;

  return w;
}

JointWidget* JointWidget::create(RcsGraph* graph, pthread_mutex_t* graphLock,
                                 MatNd* q_des, MatNd* q_curr,
                                 bool alwaysWriteToQ, bool passive)
{
  bool* _alwaysWriteToQ = new bool;
  *_alwaysWriteToQ = alwaysWriteToQ;

  bool* _passive = new bool;
  *_passive = passive;

  VoidPointerList* p = new VoidPointerList;
  p->ptr[0] = (void*) graph;
  p->ptr[1] = (void*) graphLock;
  p->ptr[2] = (void*) q_des;
  p->ptr[3] = (void*) q_curr;
  p->ptr[4] = (void*) _alwaysWriteToQ;
  p->ptr[5] = (void*) _passive;

  int handle = RcsGuiFactory_requestGUI(stateGui, p);

  return (JointWidget*) RcsGuiFactory_getPointer(handle);
}

/*******************************************************************************
 *
 ******************************************************************************/
JointWidget::JointWidget(RcsGraph* graph, pthread_mutex_t* graphLock,
                         MatNd* q_des, MatNd* q_curr, bool alwaysWriteToQ,
                         bool passive) :
  QScrollArea(),
  _graph(graph),
  _togglestate(false),
  _alwaysWriteToQ(alwaysWriteToQ),
  _q_des(q_des ? q_des : graph->q),
  _q_curr(q_curr ? q_curr : graph->q),
  lcd_q_cmd(NULL),
  lcd_q_act(NULL),
  mutex(graphLock)
{
  RLOG(5, "Creating Widget with %d dof", graph->dof);

  setWindowTitle("RCS Joint Control");

  QWidget* scrollWidget = new QWidget(this);

  if (graph->dof > 0)
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

    lcd_q_cmd         = new QLCDNumber*[graph->dof];
    lcd_q_act         = new QLCDNumber*[graph->dof];
    jsc_q             = new JointSlider*[graph->dof];
    check_constraints = new QCheckBox*[graph->dof];

    for (unsigned int k=0; k<graph->dof; k++)
    {
      lcd_q_cmd[k]         = NULL;
      lcd_q_act[k]         = NULL;
      jsc_q[k]             = NULL;
      check_constraints[k] = NULL;
    }

    //
    // Checkboxes for constraints
    //
    QGridLayout* constraintsLayout = new QGridLayout;

    int i = 0;
    RCSGRAPH_TRAVERSE_JOINTS(graph)
    {
      RLOG(5, "Creating Widget for joint \"%s\"", JNT->name);

      check_constraints[i] = new QCheckBox(JNT->name);
      check_constraints[i]->setChecked(JNT->constrained);
      connect(check_constraints[i], SIGNAL(clicked()),
              SLOT(setConstraint()));

      if (JNT->coupledTo != NULL)
      {
        check_constraints[i]->setEnabled(false);
      }

      lcd_q_cmd[i] = new QLCDNumber(6);
      lcd_q_cmd[i]->setAutoFillBackground(true);
      lcd_q_cmd[i]->setFixedHeight(20);
      lcd_q_cmd[i]->setPalette(palette);
      lcd_q_act[i] = new QLCDNumber(6);
      lcd_q_act[i]->setAutoFillBackground(true);
      lcd_q_act[i]->setFixedHeight(20);
      lcd_q_act[i]->setPalette(palette);

      // Here the joint scale
      double lb = JNT->q_min;
      double ub = JNT->q_max;
      double qi = MatNd_get(_q_des, JNT->jointIndex, 0);

      double scaleFactor = 1.0;
      if (RcsJoint_isRotation(JNT))
      {
        scaleFactor = 180.0/M_PI;
      }
      else
      {
        scaleFactor = 1000.0;
      }

      jsc_q[i] = new JointSlider(lb, qi, ub, scaleFactor);

      constraintsLayout->addWidget(check_constraints[i], i, 0,
                                   Qt::AlignLeft);
      constraintsLayout->addWidget(lcd_q_cmd[i], i, 1, Qt::AlignLeft);
      constraintsLayout->addWidget(lcd_q_act[i], i, 2, Qt::AlignLeft);
      constraintsLayout->addWidget(jsc_q[i],     i, 3, Qt::AlignLeft);

      i++;
    }   // for(i=0;i<_dof;i++)


    scrollWidget->setLayout(constraintsLayout);

  }   // if(graph->dof>0)

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
  _timer = new QTimer(this);

  if (passive == false)
  {
    connect(_timer, SIGNAL(timeout()), SLOT(setJoint()));
  }

  connect(_timer, SIGNAL(timeout()), SLOT(displayAct()));
  _timer->start(40);

  RLOG(5, "JointWidget generated");
}

/*******************************************************************************
 *
 ******************************************************************************/
JointWidget::~JointWidget()
{
  if (_graph->dof > 0)
  {
    delete [] lcd_q_cmd;
    delete [] lcd_q_act;
    delete [] jsc_q;
    delete [] check_constraints;
  }

  killTimer(_timer->timerId());
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
  int i = 0;

  RCSGRAPH_TRAVERSE_JOINTS(_graph)
  {
    char a[256];

    double scaleFactor = 1.0;
    if (RcsJoint_isRotation(JNT))
    {
      scaleFactor = 180. / M_PI;
    }
    else
    {
      scaleFactor = 1000.0;
    }

    snprintf(a, 8, "%5.1f", scaleFactor * _q_curr->ele[JNT->jointIndex]);
    lcd_q_act[i]->display(a);

    snprintf(a, 8, "%5.1f", scaleFactor * _q_des->ele[JNT->jointIndex]);
    lcd_q_cmd[i]->display(a);

    // if (!JNT->hide)
    {
      jsc_q[i]->setValue(_q_curr->ele[JNT->jointIndex]);
    }

    i++;
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
void JointWidget::setConstraint(void)
{
  int i = 0;
  lock();

  // Assign values from constraint check boxes
  RCSGRAPH_TRAVERSE_JOINTS(_graph)
  {
    JNT->constrained = check_constraints[i++]->isChecked();
  }

  unlock();
}

/*******************************************************************************
 * Assign values do _q_des from sliders
 ******************************************************************************/
void JointWidget::setJoint(void)
{
  int i = 0;

  lock();

  RCSGRAPH_TRAVERSE_JOINTS(_graph)
  {
    if ((JNT->constrained || _alwaysWriteToQ) && (!JNT->coupledTo))
    {
      _q_des->ele[JNT->jointIndex] = jsc_q[i]->getSliderValue();
    }

    i++;
  }

  unlock();
}

/*******************************************************************************
 *
 ******************************************************************************/
void JointWidget::lock()
{
  if (this->mutex)
  {
    pthread_mutex_lock(this->mutex);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void JointWidget::unlock()
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
  //disconnect(_timer, SLOT(setJoint()));

  MatNd_copy(_q_des, q);
  MatNd_copy(_q_curr, q);

  for (unsigned int i=0; i<q->m; ++i)
  {
    jsc_q[i]->setValue(q->ele[i]);
    jsc_q[i]->setSliderValue(q->ele[i]);
  }

  //connect(_timer, SIGNAL(timeout()), SLOT(setJoint()));
}

}   // namespace Rcs
