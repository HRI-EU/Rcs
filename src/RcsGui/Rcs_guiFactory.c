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

#include "Rcs_guiFactory.h"

#include <Rcs_macros.h>
#include <Rcs_timer.h>


// Some global variables
RCSGUI_API pthread_t RCSGUIFACTORY_THREAD;
RCSGUI_API bool RCSGUIFACTORY_THREAD_VALID = false;

// Some variables local to this file
static pthread_mutex_t RCSGUIFACTORY_MUTEX = PTHREAD_MUTEX_INITIALIZER;
bool RCSGUIFACTORY_INIT = false;
static void* RCSGUIFACTORY_ARG = NULL;
static RcsGuiCallbackFct RCSGUIFACTORY_CALLBACK;
static bool RCSGUIFACTORY_REQUEST_PENDING = false;
static bool RCSGUIFACTORY_DISABLE = false;

static void* RCSGUIFACTORY_HANDLE[MAX_GUIS];
static int RCSGUIFACTORY_CURRENT_GUI = 0;


// That's needed if this file is compiled with a c++ compiler. That's for
// instance the case for msvc, since the Microsoft C-compiler does not support
// the c99 standard, which makes it too hard to comply.
#ifdef __cplusplus
extern "C" {
#endif


// This is in GuiFactory.cpp and only is needed here and nowhere else. It is
// cast as "extern C" so that we can call it from here.
void RcsGuiFactory_create();
void RcsGuiFactory_stopApplication();
bool RcsGuiFactory_deleteGUI(int handle);



/******************************************************************************
  \brief See header.
******************************************************************************/
void RcsGuiFactory_shutdown()
{
  if (RCSGUIFACTORY_THREAD_VALID == true)
  {
    // Stopping qt application by calling quit() via
    // RcsGuiFactory_stopApplication()
    // Note: mutex must not be locked here, because otherwise the
    // QApplication may never quit, thus the program would hang

    //    printf("terminating GuiFactory thread...\n");
    RcsGuiFactory_stopApplication();
    pthread_join(RCSGUIFACTORY_THREAD, NULL);
    pthread_mutex_destroy(&RCSGUIFACTORY_MUTEX);
    //    printf("terminated GuiFactory thread\n");

    RCSGUIFACTORY_THREAD_VALID = false;
    memset(RCSGUIFACTORY_HANDLE, 0, MAX_GUIS*sizeof(void*));
  }
}



/******************************************************************************

  \brief See header.

******************************************************************************/

void* RcsGuiFactory_getPointer(int handle)
{
  int count = 0;
  RCHECK((handle >= 0) && (handle < MAX_GUIS));

  pthread_mutex_lock(&RCSGUIFACTORY_MUTEX);
  void* ptr = RCSGUIFACTORY_HANDLE[handle];
  pthread_mutex_unlock(&RCSGUIFACTORY_MUTEX);

  while (ptr == NULL)
  {
    Timer_usleep(100000);   // 0.1 sec
    if ((count++) > 300)
    {
      RLOG(1, "No valid pointer found for handle %d - returning NULL", handle);
      break;
    }

    pthread_mutex_lock(&RCSGUIFACTORY_MUTEX);
    ptr = RCSGUIFACTORY_HANDLE[handle];
    pthread_mutex_unlock(&RCSGUIFACTORY_MUTEX);
  }

  return ptr;
}




/******************************************************************************

  \brief See header.

******************************************************************************/

void* RcsGuiFactory_getPointerDirectly(int handle)
{
  RCHECK((handle >= 0) && (handle < MAX_GUIS));
  return RCSGUIFACTORY_HANDLE[handle];
}



/******************************************************************************

  \brief Default callback. It will be executed in each update step if no
         other callback has been registered.

******************************************************************************/

void* RcsGuiFactory_defaultCallbback(void* arg)
{
  (void) arg;   // This avoids unused parameter warnings
  return NULL;
}



/******************************************************************************

  \brief Initializes the GUI factory.

******************************************************************************/

static void RcsGuiFactory_init()
{
  pthread_mutex_lock(&RCSGUIFACTORY_MUTEX);
  if (RCSGUIFACTORY_INIT == false)
  {
    int i;
    for (i = 0; i < MAX_GUIS; i++)
    {
      RCSGUIFACTORY_HANDLE[i] = NULL;
    }
    RCSGUIFACTORY_CALLBACK    = RcsGuiFactory_defaultCallbback;
    RCSGUIFACTORY_INIT        = true;
    RcsGuiFactory_create();
  }
  pthread_mutex_unlock(&RCSGUIFACTORY_MUTEX);
}



/******************************************************************************

  \brief See header.

******************************************************************************/

int RcsGuiFactory_requestGUI(RcsGuiCallbackFct func, void* arg)
{
  int waitCycles = 0;

  // Do nothing if factory is disabled
  if (RCSGUIFACTORY_DISABLE == true)
  {
    return -1;
  }

  // Initialize factory if not yet done
  if (RCSGUIFACTORY_INIT == false)
  {
    RcsGuiFactory_init();
  }

  // If another GUI hasn't yet made it to be processed, we wait until update
  // has been called. We check if it hangs and get verbose after a while.
  while (RCSGUIFACTORY_REQUEST_PENDING)
  {
    waitCycles++;
    if (waitCycles > 100)
    {
      RLOG(1, "Hanging in while loop since %d cycles", waitCycles);
    }
    Timer_usleep(10000);
  }

  RLOG(5, "%d waitCycles", waitCycles);

  // The following operations need to be exclusive
  pthread_mutex_lock(&RCSGUIFACTORY_MUTEX);

  int return_handle = RCSGUIFACTORY_CURRENT_GUI;

  RCSGUIFACTORY_CALLBACK        = func;
  RCSGUIFACTORY_ARG             = arg;
  RCSGUIFACTORY_REQUEST_PENDING = true;

  // Release mutex
  pthread_mutex_unlock(&RCSGUIFACTORY_MUTEX);

  return return_handle;
}



/******************************************************************************

  \brief See header.

******************************************************************************/

void RcsGuiFactory_enable()
{
  // Initialize factory if not yet done
  if (!RCSGUIFACTORY_INIT)
  {
    RcsGuiFactory_init();
  }

  // Set disable flag
  pthread_mutex_lock(&RCSGUIFACTORY_MUTEX);
  RCSGUIFACTORY_DISABLE = false;
  pthread_mutex_unlock(&RCSGUIFACTORY_MUTEX);

  RLOG(5, "RcsGuiFactory is enabled");
}



/******************************************************************************

  \brief See header.

******************************************************************************/

void RcsGuiFactory_disable()
{
  // Initialize factory if not yet done
  if (!RCSGUIFACTORY_INIT) // RcsGuiFactory_init();
  {
    RCSGUIFACTORY_DISABLE = true;
    return;
  }

  // Set disable flag
  pthread_mutex_lock(&RCSGUIFACTORY_MUTEX);
  RCSGUIFACTORY_DISABLE = true;
  pthread_mutex_unlock(&RCSGUIFACTORY_MUTEX);

  RLOG(5, "RcsGuiFactory is disabled");
}



/******************************************************************************

  \brief This calls the global callback with the global argument. If a
         callback has been registered, it will be executed. After execution,
         the callback will be resetted to its default, and the pending flag
         will be resetted.
         This leads to a "one shot" call of the registered callback.

******************************************************************************/

void RcsGuiFactory_update()
{
  // Initialize factory if not yet done
  if (!RCSGUIFACTORY_INIT)
  {
    RcsGuiFactory_init();
  }

  // The following operations need to be exclusive
  pthread_mutex_lock(&RCSGUIFACTORY_MUTEX);

  // Call the callback function. The result pointer is stored in
  // the handle, so that we can return it from the request function.
  RCSGUIFACTORY_HANDLE[RCSGUIFACTORY_CURRENT_GUI]
    = RCSGUIFACTORY_CALLBACK(RCSGUIFACTORY_ARG);


  if (RCSGUIFACTORY_CALLBACK != RcsGuiFactory_defaultCallbback)
  {
    RCSGUIFACTORY_CURRENT_GUI++;
  }

  // Reset to defaults
  RCSGUIFACTORY_CALLBACK        = RcsGuiFactory_defaultCallbback;
  RCSGUIFACTORY_ARG             = NULL;
  RCSGUIFACTORY_REQUEST_PENDING = false;

  // Release mutex
  pthread_mutex_unlock(&RCSGUIFACTORY_MUTEX);
}

bool RcsGuiFactory_destroyGUI(int handle)
{
  pthread_mutex_lock(&RCSGUIFACTORY_MUTEX);
  bool success = RcsGuiFactory_deleteGUI(handle);

  if (!success)
  {
    RLOG(4, "Failed to delete Gui with handle %d", handle);
    pthread_mutex_unlock(&RCSGUIFACTORY_MUTEX);
    return false;
  }

  RCSGUIFACTORY_HANDLE[handle] = NULL;
  pthread_mutex_unlock(&RCSGUIFACTORY_MUTEX);

  return true;
}

#ifdef __cplusplus
}
#endif
