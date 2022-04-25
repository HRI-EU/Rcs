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

#include "AsyncWidget.h"
#include "AsyncGuiFactory.h"

#include <Rcs_cmdLine.h>
#include <Rcs_macros.h>
#include <Rcs_timer.h>

#include <QApplication>
#include <QThread>

#include <cstdlib>



namespace Rcs
{

class AsyncWidgetEvent : public QEvent
{
public:
  AsyncWidgetEvent(AsyncWidget* widget_, QEvent::Type eventType)
    : QEvent(eventType), widget(widget_)
  {
  }

  ~AsyncWidgetEvent()
  {
    RLOG(0, "Deleting AsyncWidgetEvent");
  }

  AsyncWidget* widget;
};




int AsyncWidget::refCount = 0;

AsyncWidget::AsyncWidget() : launched(false), w(NULL)
{
  if (!QApplication::instance())
  {
    RLOG(0, "refCount is 0 - creating GuiFactory");
    AsyncGuiFactory::create();
  }

  refCount++;
  RLOG(0, "AsyncWidget created: refCount = %d", refCount);
}

AsyncWidget::~AsyncWidget()
{
  unlaunch();
  refCount--;
  RLOG(0, "AsyncWidget deleted: refCount = %d", refCount);

  if (refCount == 0)
  {
    RLOG(0, "Quitting QApplication with event");
    AsyncWidgetEvent* ae = new AsyncWidgetEvent(this, AsyncGuiFactory::resetEvent);

    if (!AsyncGuiFactory::isGuiThread())
    {
      QCoreApplication::postEvent(AsyncGuiFactory::getLauncher(), ae);

      while (AsyncGuiFactory::isThreadRunning())
      {
        RLOG(0, "Waiting util Gui factory thread finished");
        Timer_waitDT(0.1);
      }

    }
    else
    {
      RLOG(0, "Quitting QApplication straight away");
      AsyncGuiFactory::getLauncher()->event(ae);
      delete ae;
    }

    RLOG(0, "Gui factory thread finished");
  }

}

void AsyncWidget::setLaunched(bool isLaunched)
{
  launchMtx.lock();
  launched = isLaunched;
  launchMtx.unlock();
}

bool AsyncWidget::isLaunched() const
{
  bool isLaunched;
  launchMtx.lock();
  isLaunched = launched;
  launchMtx.unlock();

  return isLaunched;
}

// Emits an event that calls construct() from the Gui thread
void AsyncWidget::launch()
{
  AsyncWidgetEvent* ae = new AsyncWidgetEvent(this, AsyncGuiFactory::constructEvent);

  if (AsyncGuiFactory::isGuiThread())
  {
    RLOG(0, "launch(): Calling construct right away");
    AsyncGuiFactory::getLauncher()->event(ae);
    delete ae;
  }
  else
  {
    RLOG(0, "launch(): Constructing by posting event to Gui thread");
    QCoreApplication::postEvent(AsyncGuiFactory::getLauncher(), ae);
  }

  double t_launch = Timer_getSystemTime();
  while (!isLaunched())
  {
    Timer_usleep(10000);

    double duration = Timer_getSystemTime() - t_launch;
    if (duration > 3.0)
    {
      RLOG(0, "Waiting for launch: %.2f seconds", duration);
    }
  }
}

// Emits an event that calls destroy() from the Gui thread
void AsyncWidget::unlaunch()
{
  AsyncWidgetEvent* ae = new AsyncWidgetEvent(this, AsyncGuiFactory::destroyEvent);

  if (AsyncGuiFactory::isGuiThread())
  {
    RLOG(0, "unlaunch(): Calling destroy right away");
    AsyncGuiFactory::getLauncher()->event(ae);
    delete ae;
  }
  else
  {
    RLOG(0, "unlaunch(): Destroying by posting event to Gui thread");
    QCoreApplication::postEvent(AsyncGuiFactory::getLauncher(), ae);
  }

  double t_unlaunch = Timer_getSystemTime();
  while (isLaunched())
  {
    Timer_usleep(10000);

    double duration = Timer_getSystemTime() - t_unlaunch;
    if (duration > 3.0)
    {
      RLOG(0, "Waiting for unlaunch: %.2f seconds", duration);
    }
  }
}

// That's being called through the Gui thread
void AsyncWidget::destroy()
{
  if (!AsyncGuiFactory::isGuiThread())
  {
    RLOG(0, "WARNING: You must call this from the Gui thread using unlaunch()");
  }

  if (w)
  {
    RLOG_CPP(0, "Deleting widget " << w->objectName().toStdString());
    delete w;
    w = NULL;
  }
  else
  {
    RLOG(0, "Widget already destroyed");
  }
}

void AsyncWidget::setWidget(QWidget* widget)
{
  if (!AsyncGuiFactory::isGuiThread())
  {
    RLOG(0, "WARNING: Calling setWidget() from non-Gui thread");
  }

  w = widget;
}

} // namespace Rcs
