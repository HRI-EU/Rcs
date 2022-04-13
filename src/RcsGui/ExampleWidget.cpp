/*******************************************************************************

  Copyright (c) 2022, Honda Research Institute Europe GmbH

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

#include "ExampleWidget.h"
#include "Rcs_guiFactory.h"
#include "Rcs_cmdLine.h"
#include "ExampleFactory.h"

#include <Rcs_macros.h>
#include <QPushButton>
#include <QVBoxLayout>
#include <QLabel>


namespace Rcs
{

typedef struct
{
  void* ptr[10];
} VoidPointerList;

static void* threadFunc(void* arg)
{
  VoidPointerList* p = (VoidPointerList*) arg;
  std::string* exampleName = (std::string*) p->ptr[0];
  int* argc = (int*) p->ptr[1];
  char** argv = (char**) p->ptr[2];

  ExampleWidget* w = new ExampleWidget(*exampleName, *argc, argv);
  w->show();

  delete argc;
  delete exampleName;
  delete p;

  return w;
}

int ExampleWidget::create(std::string name, int argc_, char** argv)
{
  VoidPointerList* p = new VoidPointerList;
  std::string* exampleName = new std::string(name);
  double* argc = new double;
  *argc = argc_;

  p->ptr[0] = (void*) exampleName;
  p->ptr[1] = (void*) argc;
  p->ptr[2] = (void*) argv;

  return RcsGuiFactory_requestGUI(threadFunc, p);
}

bool ExampleWidget::destroy(int handle)
{
  return RcsGuiFactory_destroyGUI(handle);
}

ExampleWidget::ExampleWidget(std::string name, int argc_, char** argv_) :
  example(NULL), exampleName(name), argc(argc_), argv(argv_)
{
  this->setStyleSheet("QCheckBox { font-weight: bold; }"
                      "QGroupBox { font-weight: bold; color: gray; border-radius: 1px; }"
                      "QLabel { font-size: 12px; }"
                      "QProgressBar {border: 1px solid black; border-radius: 1px; "
                      "text-align: center; font-size: 11px; }"
                      "QPushButton{font-size: 10px;font-family: Arial;color: rgb(255, 255, 255);background-color: rgb(38,56,76);}"
                     );
  initGui();
}

void ExampleWidget::initGui()
{
  QHBoxLayout* hbox = new QHBoxLayout(this);


  if (ExampleFactory::hasExample(exampleName))
  {
    QPushButton* bt_init = new QPushButton(QString::fromStdString(exampleName));
    connect(bt_init, SIGNAL(clicked()), SLOT(init()));
    hbox->addWidget(bt_init);

    QPushButton* bt_start = new QPushButton("Start");
    connect(bt_start, SIGNAL(clicked()), SLOT(start()));
    hbox->addWidget(bt_start);

    QPushButton* bt_stop = new QPushButton("Stop");
    connect(bt_stop, SIGNAL(clicked()), SLOT(stop()));
    hbox->addWidget(bt_stop);

    QPushButton* bt_destroy = new QPushButton("Delete");
    connect(bt_destroy, SIGNAL(clicked()), SLOT(destroy()));
    hbox->addWidget(bt_destroy);
  }
  else
  {
    QLabel* label = new QLabel(QString::fromStdString(exampleName+std::string(": not found")));
    hbox->addWidget(label);
  }
}

ExampleWidget::~ExampleWidget()
{
}

void ExampleWidget::init()
{
  RLOG(0, "Initializing example");
  example = ExampleFactory::create(exampleName, argc, argv);
}

static void* runThreadFunc(void* arg)
{
  ExampleBase* example = (ExampleBase*) arg;
  example->start();
  return NULL;
}

void ExampleWidget::start()
{
  RLOG(0, "Starting example");
  example->init(argc, argv);
  pthread_create(&runThread, NULL, &runThreadFunc, example);
}

void ExampleWidget::stop()
{
  RLOG(0, "Stopping example");
  example->stop();
  pthread_join(runThread, NULL);
  RLOG(0, "... stopped");
}

void ExampleWidget::destroy()
{
  // Calling pthread_join on a joined thread will immediately return
  stop();
  RLOG(0, "Deleting example");
  delete example;
}

}   // namespace Rcs
