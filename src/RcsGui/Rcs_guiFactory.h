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


/*******************************************************************************

  Description: A factory to launch all kinds of GUIs. It mainly exists due
               to the Qt event loop mechanism. Here's what you have to do if
               you want your GUI to be handled with this factory:

               1. Create a function with the signature of below
                  RcsGuiCallbackFct. This function should instantiate your
                  GUI with a void pointer being cast to the parameter you
                  need to construct it.

               2. Launch it by calling:
                  RcsGuiFactory_requestGUI(myFunc, myParam);

                  That's it. If you instantiate a Qt GUI, it will be processed
                  by the Qt event loop. If you want to instantiate another
                  (non-Qt) GUI, please make sure it runs in its own thread.

                  This is written as a C interface, so that it's possible to
                  launch a Gui also from C code. This makes the functions a
                  little bit less comfortable. However, it's possible to create
                  a nice C++ singleton factory around this.

*******************************************************************************/

#ifndef RCS_GUIFACTORY_H
#define RCS_GUIFACTORY_H

#define MAX_GUIS (256)

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <pthread.h>

#if defined(_MSC_VER) && defined(WIN_DLL)
#    ifdef RCSGUI_EXPORTS
#        define RCSGUI_API __declspec(dllexport)
#    else
#        define RCSGUI_API __declspec(dllimport)
#    endif
#else
#    define RCSGUI_API
#endif

/*  \brief Actual variable is created in RcsGuiFactory.c
*/
extern RCSGUI_API pthread_t RCSGUIFACTORY_THREAD;
extern RCSGUI_API bool      RCSGUIFACTORY_THREAD_VALID;

/*  \brief An attempt to improve the shutting down of the Gui thread.
 *         Still, the QApplication must probably be finished cleanly.
*/
void RcsGuiFactory_shutdown();

/*  \brief C wrapper function signature for the GUI's constructor. In
*          this function, the GUI has to be constructed.
*/
typedef void* (*RcsGuiCallbackFct)(void*);

/*  \brief Requests a GUI from the factory. Callback func is the C-wrapper
*          around the GUI's constructor. The pointer arg points to the
*          argument which the C-wrapper function requires. It is passed
*          through.
*/
int RcsGuiFactory_requestGUI(RcsGuiCallbackFct func, void* arg);

/*  \brief Destroys the Gui with the given handle, and removes it out of
 *         the list of Guis.
*/
bool RcsGuiFactory_destroyGUI(int handle);

/*  \brief Globally disables GUI launching through the factory methods.
*/
void RcsGuiFactory_disable(void);

/*  \brief Globally enables GUI launching through the factory methods.
*/
void RcsGuiFactory_enable(void);

/*  \brief Returns the pointer returned by the thread. The function
 *         internally polls on the pointer for three seconds. If it
 *         doesn't exist, NULL is returned. If the
 *         corresponding function returns some class etc., this
 *         can be used from the calling thread context. The user is
 *         responsible that the operations done are thread-safe.
 */
void* RcsGuiFactory_getPointer(int handle);

/*  \brief Same as RcsGuiFactory_getPointer(), but without polling on the
 *         pointer.
 */
void* RcsGuiFactory_getPointerDirectly(int handle);

/*  \brief Sets the style of the Qt windows. Currently, the following
 *         styles are supported:
 *         -  NorwegianWoodStyle
 *         If the argument style is not supported, Qt's default style
 *         is used, and false is returned (True on success). The style
 *         is set globally for all widgets.
 */
bool RcsGuiFactory_setQtStyle(const char* style);



#ifdef __cplusplus
}
#endif

#endif   // RCS_GUIFACTORY_H
