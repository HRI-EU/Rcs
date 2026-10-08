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

#ifndef RCS_EXAMPLERUNNER_H
#define RCS_EXAMPLERUNNER_H

/*

[Rcs/src/RcsGui/ExampleGui.cpp : ExampleGui(62)]: Before launch
[Rcs/src/RcsGui/AsyncWidget.cpp : launch(148)]: launch(): Constructing by posting event to Gui thread
QCoreApplication::postEvent: Unexpected null receiver
[Rcs/src/RcsGui/AsyncWidget.cpp : ~AsyncWidgetEvent(61)]: Deleting AsyncWidgetEvent
[Rcs/src/RcsGui/AsyncWidget.cpp : launch(160)]: Waiting for launch: 3.00 seconds


 */



#include <Rcs_macros.h>
#include <Rcs_cmdLine.h>
#include <Rcs_timer.h>
#include <Rcs_resourcePath.h>
#include <ExampleFactory.h>
#include <ExampleGui.h>
#include <SegFaultHandler.h>
#include <CmdLineWidget.h>

#include <Rcs_graph.h>
#include <JointWidget.h>

#include <libxml/tree.h>

#include <iostream>
#include <csignal>


RCS_INSTALL_ERRORHANDLERS

Rcs::ExampleBase* example = NULL;

bool runLoop = true;


void* threadFunc7(void*)
{
  RcsGraph* graph = RcsGraph_createRandom(5, 3);
  Rcs::JointGui gui(graph);
  RPAUSE();
  return NULL;
}

/*******************************************************************************
 * Ctrl-C destructor. Tries to quit gracefully with the first Ctrl-C
 * press, then just exits.
 ******************************************************************************/
void quit(int /*sig*/)
{
  static int kHit = 0;
  runLoop = false;
  if (example)
  {
    example->stop();
  }
  fprintf(stderr, "Trying to exit gracefully - %dst attempt\n", kHit + 1);
  kHit++;

  if (kHit == 2)
  {
    fprintf(stderr, "Exiting without cleanup\n");
    exit(0);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
#if defined (NO_CONSOLE)
int main()
{
  int argc = 1;
  char* argv[2];
  argv[0] = "ExampleRunner";
  argv[1] = NULL;
#else
int main(int argc, char** argv)
{
#endif

  // Ctrl-C callback handler
  signal(SIGINT, quit);

  // This initialize the xml library and check potential mismatches between
  // the version it was compiled for and the actual shared library used.
  LIBXML_TEST_VERSION;

  // Parse command line arguments
  int mode = 1;
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-dl", &RcsLogLevel, "Debug level (default is 0)");
  argP.getArgument("-m", &mode, "Test mode");

  if (argP.hasArgument("-c") && argP.hasArgument("-e"))
  {
    mode = 2;
  }

  Rcs_addResourcePath("config");

  switch (mode)
  {
    // ==============================================================
    // Just print out some global information
    // ==============================================================
    case 0:
    {
      argP.print();
      Rcs_printResourcePath();
      Rcs::ExampleFactory::print();

      printf("\nHere's some useful testing modes:\n\n");
      printf("\t-m");
      printf("\t0   Print this message\n");
      printf("\t\t1   Example runner with Gui\n");
      printf("\t\t2   Example runner with specific example\n");
      break;
    }

    // ==============================================================
    // Example view Gui
    // ==============================================================
    case 1:
    {
      Rcs::ExampleGui gui(argc, argv);
      gui.waitUntilWidgetDeleted();
      break;
    }

    // ==============================================================
    // Programatic example test
    // ==============================================================
    case 2:
    {
      std::string categoryName = "Physics";
      std::string exampleName = "Gyro";
      argP.getArgument("-c", &categoryName, "Category name (default: %s)",
                       categoryName.c_str());
      argP.getArgument("-e", &exampleName, "Example name (default: %s)",
                       exampleName.c_str());

      // Under macOS, all windowing must happen on the main thread: the osg
      // viewer aborts with an NSInternalInconsistencyException if its graphics
      // context is created anywhere else. We therefore run the example on the
      // main thread there, so that the viewer's frame() call, which the
      // example issues from its step() method in the sequential sync mode,
      // ends up on the thread that Cocoa expects. The matching sync mode is
      // the default of ExampleBase. Pass "-mainThread false" to get the
      // threaded behaviour back.
      //
      // With -valgrind the example runs without graphics and without Guis.
      // There is then nothing that must live on the main thread, so we leave
      // the example in its own thread, as on any other platform.
#if defined (__APPLE__)
      bool mainThread = !argP.hasArgument("-valgrind");
#else
      bool mainThread = false;
#endif
      argP.getArgument("-mainThread", &mainThread, "Run the example on the "
                       "main thread rather than in its own thread (default: "
                       "%s)", mainThread ? "true" : "false");

      if (mainThread)
      {
        // Running on the main thread only has the desired effect if the
        // graphics are updated from there as well. That is what the sequential
        // sync mode does, which ExampleBase defaults to under macOS.
        //
        // This blocks until the example has finished. Note that runExample()
        // deletes the instance in the non-threaded case, therefore the global
        // example pointer is left alone here.
        Rcs::ExampleFactory::runExample(categoryName, exampleName,
                                        argc, argv, false);
      }
      else
      {
        example = Rcs::ExampleFactory::runExample(categoryName, exampleName,
                                                  argc, argv);

        while (example && example->isRunning())
        {
          Timer_waitDT(0.1);
        }

        Timer_waitDT(0.1);
        delete example;
      }
      break;
    }

    // ==============================================================
    // Command line Gui
    // ==============================================================
    case 3:
    {
      Rcs::ParameterCollection collection;
      int a = 4;
      std::string b = "Foo";
      collection.getArgument("a", &a, "This is a");
      collection.getArgument("b", &b, "This is b");

      Rcs::CmdLineGui gui(&collection);

      while (runLoop)
      {
        RLOG(0, "a = %d   b = %s", a, b.c_str());
        Timer_waitDT(1.0);
      }

      break;
    }

    // ==============================================================
    // JointGui
    // ==============================================================
    case 4:
    {
      RcsGraph* graph = RcsGraph_createRandom(5, 3);
      Rcs::JointGui gui(graph);
      gui.waitUntilWidgetDeleted();
      RcsGraph_destroy(graph);
      break;
    }

    // ==============================================================
    // Thread test
    // ==============================================================
    case 5:
    {
      RcsGraph* graph = RcsGraph_createRandom(5, 3);
      Rcs::JointGui gui(graph);
      RPAUSE();
      pthread_t myThread;
      pthread_create(&myThread, NULL, threadFunc7, NULL);
      RPAUSE();
      pthread_join(myThread, NULL);
      RcsGraph_destroy(graph);
      break;
    }

    // ==============================================================
    // That's it.
    // ==============================================================
    default:
    {
      RMSG("there is no mode %d", mode);
    }

  } // switch(mode)


  if (argP.hasArgument("-h"))
  {
    argP.print();
    Rcs_printResourcePath();
    Rcs::ExampleFactory::print();
  }

  xmlCleanupParser();

  fprintf(stderr, "Thanks for using the ExampleRunner\n");

  return 0;
}


#endif   // RCS_EXAMPLERUNNER_H
