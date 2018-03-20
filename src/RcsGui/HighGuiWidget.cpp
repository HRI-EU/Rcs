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

#include "HighGuiWidget.h"
#include "Rcs_guiFactory.h"

#include <Rcs_macros.h>

#include <QTimer>

namespace Rcs
{

HighGuiWidget::HighGuiWidget(const std::string& name) :
  QWidget(),
  _name(name)
{
  setAttribute(Qt::WA_DeleteOnClose, false);
  setAttribute(Qt::WA_QuitOnClose, false);

  pthread_mutex_init(&_mutex, NULL);

  setWindowTitle(QString::fromStdString(_name));

  // 25 Hz timer callback for updating
  QTimer* timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), SLOT(update()));
  timer->start(40);
}

HighGuiWidget::~HighGuiWidget()
{
  RLOGS(5, "HighGuiWidget::~HighGuiWidget() for \"%s\"",
        windowTitle().toStdString().c_str());
  pthread_mutex_destroy(&_mutex);
}

std::string HighGuiWidget::getName() const
{
  return _name;
}


void HighGuiWidget::lock()
{
  pthread_mutex_lock(&_mutex);
}

void HighGuiWidget::unlock()
{
  pthread_mutex_unlock(&_mutex);
}

void HighGuiWidget::update()
{
}


}
