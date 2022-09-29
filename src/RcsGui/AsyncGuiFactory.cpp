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

/*

To implement this class, these things have been considerd:

- The Qt Gui thread is the thread that instantiates the QApplication. No call
  to any QObject-related function must be done before that.
- Qt Gui operations must be called from the Qt Gui thread.
- We realize this by defering the Gui construction to a construct() method, which
  is being called from the Gui thread. This is implemented in the AsyncWidget
  class. Passing it is done with the moveToThread method and QT's event system,
  the constructor arguments are held by the derived class.
- Classes with Q_OBJECT cannot be nested classes
- The QApplication instance cannot be destroyed in an atexit() function. This
  leads to error messages such as:
  QEventDispatcherWin32::wakeUp: Failed to post a message (Invalid window
  handle.) We therefore use a reference count to close the GuiFactory. This
  might lead to recreation of the Qt event loop several times though.
- Here are some good resources about what we are doing here:
  https://forum.qt.io/topic/124878/running-qapplication-exec-from-another-thread-qcoreapplication-qguiapplication/2

 */


/*
 Upon startup, we sometimes receive this error message:

  QCoreApplication::postEvent: Unexpected null receiver
 */


/*

[Rcs/src/RcsGui/AsyncWidget.cpp: launch(141)]: Check failed: wLauncher - Exiting

########## Stacktrace ##########
[Rcs/src/RcsGui/AsyncGuiFactory.cpp : threadFunc(230)]: Launcher assigned - after this, no crash should occur
  0: <Rcs::AsyncWidget::launch()+1039> RcsGui/AsyncWidget.cpp:141 (discriminator 1)
  1: <Rcs::ControllerGui::ControllerGui()+266> RcsGui/ControllerWidgetBase.cpp:63
  2: <Rcs::ExampleIK::initGuis()+1063> examples/ExampleInvKin.cpp:443
  3: <Rcs::ExampleBase::init()+1371> RcsCore/ExampleBase.cpp:65
  4: <NULL+94119907723698> bin/TestPoseGraph.cpp:806
  5: <main+452> bin/TestPoseGraph.cpp:851
################################


SIGABORT! Hey mgienger, go FIX your code!!!

########## Stacktrace ##########
  0: <NULL+94119907713225> RcsCore/SegFaultHandler.h:152
  1: <NULL+140128593196064> sigaction.c:?
addr2line: DWARF error: section .debug_info is larger than its filesize! (0x93ef57 vs 0x530ea0)
  2: <gsignal+203> ??:?
addr2line: DWARF error: section .debug_info is larger than its filesize! (0x93ef57 vs 0x530ea0)
  3: <abort+299> ??:?
  4: <Rcs::AsyncWidget::launch()+1044> RcsGui/AsyncWidget.cpp:141 (discriminator 1)
  5: <Rcs::ControllerGui::ControllerGui()+266> RcsGui/ControllerWidgetBase.cpp:63
  6: <Rcs::ExampleIK::initGuis()+1063> examples/ExampleInvKin.cpp:443
  7: <Rcs::ExampleBase::init()+1371> RcsCore/ExampleBase.cpp:65
  8: <NULL+94119907723698> bin/TestPoseGraph.cpp:806
  9: <main+452> bin/TestPoseGraph.cpp:851
################################







[Rcs/src/RcsGui/AsyncWidget.cpp: launch(141)]: Check failed: wLauncher - Exiting

########## Stacktrace ##########
[Rcs/src/RcsGui/AsyncGuiFactory.cpp : threadFunc(230)]: Launcher assigned - after this, no crash should occur
  0: <Rcs::AsyncWidget::launch()+1039> RcsGui/AsyncWidget.cpp:141 (discriminator 1)
  1: <Rcs::ControllerGui::ControllerGui()+266> RcsGui/ControllerWidgetBase.cpp:63
  2: <Rcs::ExampleIK::initGuis()+1063> examples/ExampleInvKin.cpp:443
  3: <Rcs::ExampleBase::init()+1371> RcsCore/ExampleBase.cpp:65
  4: <NULL+94576605325635> bin/TestPoseGraph.cpp:799
  5: <main+452> bin/TestPoseGraph.cpp:844
################################


SIGABORT! Hey mgienger, go FIX your code!!!

########## Stacktrace ##########
  0: <NULL+94576605315225> RcsCore/SegFaultHandler.h:152
  1: <NULL+140377886176288> sigaction.c:?
addr2line: DWARF error: section .debug_info is larger than its filesize! (0x93ef57 vs 0x530ea0)
  2: <gsignal+203> ??:?
addr2line: DWARF error: section .debug_info is larger than its filesize! (0x93ef57 vs 0x530ea0)
  3: <abort+299> ??:?
  4: <Rcs::AsyncWidget::launch()+1044> RcsGui/AsyncWidget.cpp:141 (discriminator 1)
  5: <Rcs::ControllerGui::ControllerGui()+266> RcsGui/ControllerWidgetBase.cpp:63
  6: <Rcs::ExampleIK::initGuis()+1063> examples/ExampleInvKin.cpp:443
  7: <Rcs::ExampleBase::init()+1371> RcsCore/ExampleBase.cpp:65
  8: <NULL+94576605325635> bin/TestPoseGraph.cpp:799
  9: <main+452> bin/TestPoseGraph.cpp:844
################################

 */

#include "AsyncGuiFactory.h"

#include <Rcs_cmdLine.h>
#include <Rcs_macros.h>
#include <Rcs_timer.h>

#include <QApplication>
#include <QThread>
#include <QWidget>

#include <cstdlib>


uint qGlobalPostedEventsCount();

void myMessageOutput(QtMsgType type,
                     const QMessageLogContext& context,
                     const QString& msg)
{
  QByteArray localMsg = msg.toLocal8Bit();
  const char* file = context.file ? context.file : "";
  const char* function = context.function ? context.function : "";
  switch (type)
  {
    case QtDebugMsg:
      fprintf(stderr, "Debug: %s (%s:%u, %s)\n",
              localMsg.constData(), file, context.line, function);
      break;
    case QtInfoMsg:
      fprintf(stderr, "Info: %s (%s:%u, %s)\n",
              localMsg.constData(), file, context.line, function);
      break;
    case QtWarningMsg:
      //fprintf(stderr, "xWarning: %s (%s:%u, %s)\n",
      //localMsg.constData(), file, context.line, function);
      break;
    case QtCriticalMsg:
      fprintf(stderr, "Critical: %s (%s:%u, %s)\n",
              localMsg.constData(), file, context.line, function);
      break;
    case QtFatalMsg:
      fprintf(stderr, "Fatal: %s (%s:%u, %s)\n",
              localMsg.constData(), file, context.line, function);
      break;
    default:
      fprintf(stderr, "No category: %s (%s:%u, %s)\n",
              localMsg.constData(), file, context.line, function);
  }
}


namespace Rcs
{

pthread_t AsyncGuiFactory::myThread;
int AsyncGuiFactory::argc = 0;
char** AsyncGuiFactory::argv = NULL;
WidgetLauncher* AsyncGuiFactory::launcher = NULL;
QEvent::Type AsyncGuiFactory::constructEvent = QEvent::None;
QEvent::Type AsyncGuiFactory::destroyEvent = QEvent::None;
QEvent::Type AsyncGuiFactory::resetEvent = QEvent::None;
bool AsyncGuiFactory::threadRunning = false;
bool AsyncGuiFactory::launcherRunning = false;

pthread_t guiThreadId;


int AsyncGuiFactory::create()
{
  Rcs::CmdLineParser argP;
  char** argv = NULL;
  int argc = argP.getArgs(&argv);

  if (argc == 0)
  {
    argc = 1;
    static char dummy[32] = "GuiFactory";
    static char* argvBuf[2];
    argvBuf[0] = dummy;
    argvBuf[1] = NULL;
    argv = argvBuf;
  }

  return create(argc, argv);
}

int AsyncGuiFactory::create(int argc_, char** argv_)
{
  if (QApplication::instance())
  {
    RLOG(1, "AsyncGuiFactory already running");
    return -1;
  }

  RLOG(5, "Creating AsyncGuiFactory thread");
  argc = argc_;
  argv = argv_;

  RLOG(5, "Construct event: %d   Destroy event: %d",
       constructEvent, destroyEvent);
  pthread_create(&myThread, NULL, AsyncGuiFactory::threadFunc, NULL);

  // Return only once the QApplication has completely been constructed.
  // Otherwise there might be issues when calling moveToThread for subsequent
  // widgets.
  RLOG(5, "Waiting for QApplication::instance()");
  while (QApplication::startingUp() && (!launcherRunning))
  {
    RLOG(5, "... starting up");
    Timer_usleep(10000);
  }

  RLOG_CPP(5, "Done: QApplication");

  return 0;
}

int AsyncGuiFactory::destroy()
{
  RLOG(5, "Destroying AsyncGuiFactory");
  // Uncomment his when there are warnings about non-stoppable timers from
  // another thread
  if (RcsLogLevel <= 0)
  {
    qInstallMessageHandler(myMessageOutput);
  }
  RCHECK(isGuiThread());

  QApplication::quit();

  while (QApplication::closingDown())
  {
    RLOG(5, "... shutting down");
    Timer_usleep(10000);
  }

  RLOG(5, "QApplication quit() done");
  pthread_join(myThread, NULL);
  RLOG(5, "myThread joined");
  return 0;
}

AsyncGuiFactory::AsyncGuiFactory()
{
}

void* AsyncGuiFactory::threadFunc(void*)
{
  // To be on the safe side, we register the event types from inside the
  // Gui thread.
  static bool initEvent = false;
  if (!initEvent)
  {
    constructEvent = static_cast<QEvent::Type>(QEvent::registerEventType());
    destroyEvent = static_cast<QEvent::Type>(QEvent::registerEventType());
    resetEvent = static_cast<QEvent::Type>(QEvent::registerEventType());
  }

  guiThreadId = pthread_self();
  threadRunning = true;

  RLOG(5, "threadFunc took off");
  QApplication app(argc, argv);
  app.setQuitOnLastWindowClosed(false);

  WidgetLauncher myLauncher;
  myLauncher.setObjectName("myLauncher");
  launcher = &myLauncher;
  launcherRunning = true;
  RLOG(0, "Launcher assigned - after this, no crash should occur");
  app.exec();

  RLOG_CPP(5, "Widgets: " << app.allWidgets().size());
  RLOG_CPP(5, "Event count: " << qGlobalPostedEventsCount());
  RLOG(5, "threadFunc exits, %zu widgets alive", myLauncher.numWidgets());

  threadRunning = false;
  launcher = NULL;

  return NULL;
}

bool AsyncGuiFactory::isGuiThread()
{
  if (pthread_equal(pthread_self(), guiThreadId) == 0)   // threads differ
  {
    return false;
  }

  return true;
}

bool AsyncGuiFactory::isThreadRunning()
{
  return threadRunning;
}

WidgetLauncher* AsyncGuiFactory::getLauncher()
{
  return launcher;
}




class AsyncWidgetEvent : public QEvent
{
public:
  AsyncWidgetEvent(AsyncWidget* widget_, QEvent::Type eventType)
    : QEvent(eventType)
    , widget(widget_)
  {
  }

  ~AsyncWidgetEvent()
  {
    RLOG(5, "Deleting AsyncWidgetEvent");
  }

  AsyncWidget* widget;
};





WidgetLauncher::WidgetLauncher() : QObject(NULL)
{
}

WidgetLauncher::~WidgetLauncher()
{
  RLOG_CPP(5, "Deleting GuiLauncher " << objectName().toStdString());
}

bool WidgetLauncher::event(QEvent* ev)
{
  RLOG_CPP(5, "Received event - event count: " << qGlobalPostedEventsCount());


  AsyncWidgetEvent* mev = dynamic_cast<AsyncWidgetEvent*>(ev);

  if (!mev)
  {
    RLOG(5, "Received NON-AsyncWidgetEvent");
    return false;
  }

  if (ev->type() == AsyncGuiFactory::constructEvent)
  {
    RLOG(5, "Received AsyncGuiFactory::constructEvent");

    if (!mev->widget->w)
    {
      RLOG(5, "Constructing and connecting onCloseWindow");
      mev->widget->construct();
      mev->widget->w->setAttribute(Qt::WA_DeleteOnClose);
      mev->widget->w->show();
      connect(mev->widget->w, SIGNAL(destroyed(QObject*)), this,
              SLOT(onCloseWindow(QObject*)));
      RLOG(5, "done");
      mev->widget->setLaunched(true);
      asyncWidgets.push_back(mev->widget);
      RLOG_CPP(5, "Widgets alive: " << asyncWidgets.size());
    }
    else
    {
      RLOG_CPP(5, "Widget " << mev->widget->w->objectName().toStdString()
               << " already up and running");
    }

    return true;
  }
  else if (ev->type() == AsyncGuiFactory::destroyEvent)
  {
    RLOG(5, "Received AsyncGuiFactory::destroyEvent");

    mev->widget->destroy();
    mev->widget->setLaunched(false);

    std::vector<AsyncWidget*>::iterator it = asyncWidgets.begin();

    while (it != asyncWidgets.end())
    {
      AsyncWidget* aw = *it;

      if (aw == mev->widget)
      {
        RLOG_CPP(5, "Removing AsyncWidget holding "
                 << aw->w->objectName().toStdString()
                 << " from queue");
        it = asyncWidgets.erase(it);
      }
      else
      {
        it++;
      }
    }

    RLOG_CPP(5, "Widgets alive: " << asyncWidgets.size());
    return true;
  }
  else if (ev->type() == AsyncGuiFactory::resetEvent)
  {
    RLOG(5, "Destroying GuiFactory");
    AsyncGuiFactory::destroy();
    return true;
  }

  return false;
}

void WidgetLauncher::onCloseWindow(QObject* obj)
{
  RLOG(5, "Catching close window event");
  std::vector<AsyncWidget*>::iterator it = asyncWidgets.begin();

  while (it != asyncWidgets.end())
  {
    AsyncWidget* aw = *it;

    RLOG_CPP(5, "aw->name:  " << aw->w->objectName().toStdString());
    RLOG_CPP(5, "obj->name: " << obj->objectName().toStdString());

    if (aw->w == obj)
    {
      RLOG_CPP(5, "Setting AsyncWidget's "
               << obj->objectName().toStdString() << " to NULL");
      disconnect(aw->w, SIGNAL(destroyed(QObject*)),
                 this, SLOT(onCloseWindow(QObject*)));
      aw->w = NULL;
      it = asyncWidgets.erase(it);
    }
    else
    {
      it++;
    }
  }

}

size_t WidgetLauncher::numWidgets() const
{
  return asyncWidgets.size();
}

} // namespace Rcs
