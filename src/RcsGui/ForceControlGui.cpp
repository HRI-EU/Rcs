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

#include "Rcs_guiFactory.h"
#include "ForceControlGui.h"
#include "SliderCheck1Dof.h"

#include <Rcs_macros.h>

#include <QtGui/QVBoxLayout>



Rcs::ForceControlGui* Rcs::ForceControlGui::create(const Rcs::ControllerBase* controller,
                                                   MatNd* ft_des,
                                                   MatNd* s_des,
                                                   MatNd* ft_curr,
                                                   const char* title,
                                                   pthread_mutex_t* mutex)
{
  std::vector<Rcs::ForceControlGui::Entry> guiEntries;

  Rcs::ForceControlGui::populateFTguiList(guiEntries, controller, ft_des, ft_curr, s_des);
  int handle = RcsGuiFactory_requestGUI(Rcs::ForceControlGui::threadFunction, &guiEntries);

  return (Rcs::ForceControlGui*) RcsGuiFactory_getPointer(handle);
}

int Rcs::ForceControlGui::getTaskEntryByName(const Rcs::ControllerBase* controller, const char* name)
{
  for (size_t taskEntry = 0; taskEntry < controller->getNumberOfTasks(); taskEntry++)
  {
    if (STREQ(controller->getTaskName(taskEntry).c_str(), name))
    {
      return taskEntry;
    }
  }
  RFATAL("\'%s\' Task not found", name);
  return -1;
}

void Rcs::ForceControlGui::populateFTguiList(std::vector<Rcs::ForceControlGui::Entry>& guiEntries, const Rcs::ControllerBase* controller, MatNd* ft_des, MatNd* ft_task, MatNd* s)
{
  for (size_t id = 0; id < controller->getNumberOfTasks(); id++)
  {
    const Rcs::Task* tsk = controller->getTask(id);
    size_t curr_index = controller->getTaskArrayIndex(id);

    for (size_t i = 0; i < tsk->getDim(); i++)
    {
      Rcs::Task::Parameters* param = tsk->getParameter(i);


      guiEntries.push_back(
        Rcs::ForceControlGui::Entry(&ft_des->ele[curr_index+i], &ft_task->ele[curr_index+i], &s->ele[curr_index+i], param->name.c_str(), -20.0, 0.0, 20.0, 0.1));
    }
  }
}


void* Rcs::ForceControlGui::threadFunction(void* arg)
{
  RCHECK(arg);
  Rcs::ForceControlGui* gui = new Rcs::ForceControlGui((std::vector<ForceControlGui::Entry>*) arg);
  //    widget->move(600,0);
  gui->show();
  return gui;
}


Rcs::ForceControlGui::ForceControlGui(std::vector<Entry>* entries):
  QScrollArea()
{
  QString windowTitle("RCS ForceControl Viewer GUI");
  setWindowTitle(windowTitle);

  // The layout for the overall mp widget
  QVBoxLayout* main_layout = new QVBoxLayout();
  main_layout->setMargin(3);
  main_layout->setSpacing(1);


  for (std::vector<Entry>::iterator it = entries->begin(); it != entries->end(); ++it)
  {
    RLOG(2, "Adding entry %s", it->name);
    SliderCheck1Dof* widget = new SliderCheck1Dof(it->q_des, it->q_curr, it->active, it->name, it->lowerBound, it->zeroPos, it->upperBound, it->ticSize);
    main_layout->addWidget(widget);
  }

  //BOOST_FOREACH(ForceControlGui::Entry entry, *entries)
  //{
  //  RLOG(2, "Adding entry %s", entry.name);
  //  SliderCheck1Dof* widget = new SliderCheck1Dof(entry.q_des, entry.q_curr, entry.active, entry.name, entry.lowerBound, entry.zeroPos, entry.upperBound, entry.ticSize);
  //  main_layout->addWidget(widget);
  //}

  main_layout->addStretch();

  QWidget* scroll_widget = new QWidget(this);
  scroll_widget->setLayout(main_layout);
  this->setWidget(scroll_widget);
  this->setWidgetResizable(true);

  this->resize(650, 300);

  RLOG(5, "ForceControlGui generated");
}

Rcs::ForceControlGui::~ForceControlGui()
{
}
