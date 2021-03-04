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

#include "GuiFactory.h"
#include "Rcs_guiFactory.h"

#include <Rcs_macros.h>

#include <QtGlobal>
#include <QApplication>
#include <QWidget>
#include <QTimer>
#include <QFont>

extern bool RCSGUIFACTORY_INIT;

// The implementation of this function is in RcsGuiFactory.c and only is
// needed here and nowhere else.
extern "C" {
  void RcsGuiFactory_update();
}



/*******************************************************************************
 * This runs the Qt event loop. It doesn't return and should be run in a
 * separate thread. This function needs to be here since it executes C++ Qt
 * stuff. If another QApplication is running, its event loop will be used and
 * no new QApplication is created.
 ******************************************************************************/
void* RcsGuiFactory_thread(void*)
{
  int argc = 1;
  char dummy[32] = "RcsGuiFactory_thread";
  char* argv[1];
  argv[0] = dummy;

  QApplication* app = NULL;
  GuiFactory* guiFactory = NULL;

  if (qApp != NULL)
  {
    RLOG(1, "QApplication seems to be running - not created");
  }
  else
  {
    app = new QApplication(argc, argv);
  }

  guiFactory = new GuiFactory();

  RCSGUIFACTORY_THREAD_VALID = true;

  if (app != NULL)
  {
    app->exec();
  }

  // app->exec() terminates after closing the last Qt widget. We therefore
  // clean up everything and set RCSGUIFACTORY_INIT to false, so that the next
  // created widget will restart the Qt thread.
  delete guiFactory;
  delete app;

  RLOG(5, "Leaving RcsGuiFactory_thread");
  RCSGUIFACTORY_THREAD_VALID = false;
  RCSGUIFACTORY_INIT = false;

  return (void*)NULL;
}

/*******************************************************************************
 * Creates the GUI factory thread. It needs to be "extern C" for name mangling
 * reasons.
 ******************************************************************************/

extern "C" {

  void RcsGuiFactory_create()
  {
    if (RCSGUIFACTORY_THREAD_VALID == true)
    {
      RLOG(1, "GuiFactory already running!");
      return;
    }

    // RCSGUIFACTORY_THREAD_VALID = true;
    pthread_create(&RCSGUIFACTORY_THREAD, NULL, RcsGuiFactory_thread, NULL);
  }

  bool RcsGuiFactory_setQtStyle(const char* style)
  {
    return false;
  }

  void RcsGuiFactory_stopApplication()
  {
    if (qApp != NULL)
    {
      RLOG(5, "Quitting QApplication of GuiFactory...");

      // Invoking slot, because calling qApp->quit() is not thread-safe
      QMetaObject::invokeMethod(qApp, "quit", Qt::BlockingQueuedConnection);
    }
  }

  bool RcsGuiFactory_deleteGUI(int handle)
  {
    QWidget* w = (QWidget*)RcsGuiFactory_getPointerDirectly(handle);

    if (!w)
    {
      RLOG(4, "No widget found for handle %d", handle);
      return false;
    }

    if (w->parent())
    {
      RLOG(4, "Widget with handle %d has parent - not deleting", handle);
      return false;
    }

    w->deleteLater();
    return true;
  }

}

/*******************************************************************************
 * GuiFactory class. It just creates a timer callback for its update function.
 ******************************************************************************/
GuiFactory::GuiFactory() : QObject()
{
  QTimer* timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), SLOT(update()));
  timer->start(40);
}

/*******************************************************************************
 * We only delete widgets without parent, since the Qt memory management is
 * taking care about child widgets.  We can delete all parent-less widgets
 * here, since the event loop is stopped at this point. The pointers of the
 * deleted instances are set to NULL subsequently in the
 * RcsGuiFactory_shutdown() method.
 ******************************************************************************/
GuiFactory::~GuiFactory()
{
  RLOG(5, "Destroying GuiFactory and all GUIs created...");

  for (int i = 0; i < MAX_GUIS; i++)
  {
    QWidget* w = (QWidget*)RcsGuiFactory_getPointerDirectly(i);
    if (w && (!w->parent()))
    {
      RLOG_CPP(5, "Deleting widget " << w->objectName().toStdString());
      delete w;
    }
  }

  RLOG(5, "Done destroying GuiFactory and all GUIs created...");
}

/*******************************************************************************
 * Calls the RcsGuiFactory_update() function, such checking for new GUI
 * requests.
 ******************************************************************************/
void GuiFactory::update()
{
  RcsGuiFactory_update();
}
