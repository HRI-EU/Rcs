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

#include "PPSGui.h"
#include "PPSWidget.h"

#include <Rcs_macros.h>

#include <QtGui/QHBoxLayout>
#include <QtCore/QTimer>



namespace Rcs
{

void* ppsGui(void* arg)
{
  RCHECK(arg);
  PPSGui* gui = new PPSGui((std::vector<PPSGui::Entry>*)arg);
  //    widget->move(600,0);
  gui->show();
  return gui;
}


PPSGui::PPSGui(std::vector<Entry>* entries):
  QScrollArea()
{
  QString window_title("RCS PPS Viewer GUI");
  setWindowTitle(window_title);

  // The layout for the overall widget
  QHBoxLayout* main_layout = new QHBoxLayout();
  main_layout->setMargin(3);
  main_layout->setSpacing(1);
  // this timer will trigger the gui updates
  QTimer* timer = new QTimer(this);

  for (std::vector<Entry>::iterator it = entries->begin(); it != entries->end(); ++it)
  {
    RLOG(5, "Adding entry %s", it->name.c_str());
    PPSWidget* widget = new PPSWidget(it->name, it->width, it->height, it->data, it->scaling, it->offset, it->palm);
    main_layout->addWidget(widget);
    //connect(timer, SIGNAL(timeout()), widget, SLOT(updateDisplay()));
  }

  main_layout->addStretch();

  QWidget* scroll_widget = new QWidget(this);
  scroll_widget->setLayout(main_layout);
  this->setWidget(scroll_widget);
  this->setWidgetResizable(true);

  //  this->resize(600, 200);
  this->resize(this->widget()->sizeHint() + QSize(10, 20));

  //    connect(timer, SIGNAL(timeout()), SLOT(updateDisplay()) );
  timer->start(100);

  RLOG(5, "PPSGui generated");
}

PPSGui::~PPSGui()
{
}



}
