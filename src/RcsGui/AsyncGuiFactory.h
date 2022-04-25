/*******************************************************************************

  Copyright (c) Honda Research Institute Europe GmbH

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


#ifndef RCS_ASYNCGUIFACTORY_H
#define RCS_ASYNCGUIFACTORY_H

#include "AsyncWidget.h"

#include <vector>
#include <pthread.h>


namespace Rcs
{

class WidgetLauncher : public QObject
{
  Q_OBJECT

public:

  WidgetLauncher();
  ~WidgetLauncher();
  bool event(QEvent* ev);
  size_t numWidgets() const;

public slots:
  void onCloseWindow(QObject* obj);

private:
  std::vector<AsyncWidget*> asyncWidgets;
};


class AsyncGuiFactory
{
public:
  static int create();
  static int create(int argc_, char** argv_);
  static int destroy();
  static bool isGuiThread();
  static bool isThreadRunning();
  static WidgetLauncher* getLauncher();
  static QEvent::Type constructEvent;
  static QEvent::Type destroyEvent;
  static QEvent::Type resetEvent;

private:
  AsyncGuiFactory();
  static void* threadFunc(void*);
  static pthread_t myThread;
  static int argc;
  static char** argv;
  static WidgetLauncher* launcher;
  static bool threadRunning;
};


}   // namespace

#endif   // RCS_ASYNCGUIFACTORY_H
