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

#include "PPSGui.h"
#include "PPSWidget.h"

#include <Rcs_macros.h>
#include <Rcs_guiFactory.h>

#include <QHBoxLayout>



typedef struct
{
  void* ptr[10];
} VoidPointerList;


namespace Rcs
{

static void* ppsGui(void* arg)
{
  VoidPointerList* p = (VoidPointerList*) arg;
  RCHECK(p);

  std::vector<PPSGui::Entry>* entries = (std::vector<PPSGui::Entry>*)p->ptr[0];
  pthread_mutex_t* mutex = (pthread_mutex_t*) p->ptr[1];
  PPSGui* gui = new PPSGui(entries, mutex);
  gui->show();
  return gui;
}

PPSGui* PPSGui::create(std::vector<Rcs::PPSGui::Entry> ppsEntries,
                       pthread_mutex_t* mutex)
{
  VoidPointerList* p = new VoidPointerList;
  p->ptr[0] = (void*) &ppsEntries;
  p->ptr[1] = (void*) mutex;

  int handle = RcsGuiFactory_requestGUI(ppsGui, p);

  return (PPSGui*) RcsGuiFactory_getPointer(handle);
}


PPSGui::PPSGui(std::vector<Entry>* entries, pthread_mutex_t* mutex): QScrollArea()
{
  QString window_title("RCS PPS Viewer GUI");
  setWindowTitle(window_title);

  // The layout for the overall widget
  QHBoxLayout* mainLayout = new QHBoxLayout();
  mainLayout->setMargin(3);
  mainLayout->setSpacing(1);

  for (std::vector<Entry>::iterator it = entries->begin(); it != entries->end(); ++it)
  {
    RLOG(5, "Adding entry %s", it->name.c_str());
    PPSWidget* widget = new PPSWidget(it->name, it->width, it->height, it->data, it->scaling, it->offset, it->palm, mutex);
    mainLayout->addWidget(widget);
  }

  mainLayout->addStretch();

  QWidget* scrollWidget = new QWidget(this);
  scrollWidget->setLayout(mainLayout);
  this->setWidget(scrollWidget);
  this->setWidgetResizable(true);

  //  this->resize(600, 200);
  this->resize(this->widget()->sizeHint() + QSize(10, 20));

  RLOG(5, "PPSGui generated");
}

PPSGui::~PPSGui()
{
}



}
