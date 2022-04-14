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

#include <Rcs_macros.h>
#include <Rcs_cmdLine.h>
#include <Rcs_resourcePath.h>
#include <Rcs_guiFactory.h>
#include <ExampleFactory.h>
#include <ExampleGui.h>
#include <SegFaultHandler.h>

#include <QApplication>

#include <libxml/tree.h>

#include <iostream>
#include <csignal>

RCS_INSTALL_ERRORHANDLERS

bool runLoop = true;



/*******************************************************************************
 * Ctrl-C destructor. Tries to quit gracefully with the first Ctrl-C
 * press, then just exits.
 ******************************************************************************/
void quit(int /*sig*/)
{
  static int kHit = 0;
  runLoop = false;
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
int main(int argc, char** argv)
{
  // Ctrl-C callback handler
  signal(SIGINT, quit);

  // This initialize the xml library and check potential mismatches between
  // the version it was compiled for and the actual shared library used.
  LIBXML_TEST_VERSION;

  // Parse command line arguments
  int mode = 4;
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-dl", &RcsLogLevel, "Debug level (default is 0)");
  argP.getArgument("-m", &mode, "Test mode");

  if (argP.hasArgument("-h"))
  {
    mode = 0;
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
      break;
    }

    // ==============================================================
    // Example widget test
    // ==============================================================
    case 1:
    {
      std::string exampleName = "ExampleFK";
      argP.getArgument("-e", &exampleName, "Example name (default: %s)",
                       exampleName.c_str());
      int guiHandle = Rcs::ExampleWidget::create("My Category", exampleName, argc, argv);
      RPAUSE();
      Rcs::ExampleWidget::destroy(guiHandle);
      break;
    }

    // ==============================================================
    // ExampleGui
    // ==============================================================
    case 2:
    {
      int handle = Rcs::ExampleGui::create(argc, argv);
      RPAUSE();
      Rcs::ExampleGui::destroy(handle);
      break;
    }

    // ==============================================================
    // Programatic example test
    // ==============================================================
    case 3:
    {
      std::string categoryName = "RcsCore";
      std::string exampleName = "ExampleFK";
      argP.getArgument("-c", &categoryName, "Category name (default: %s)",
                       categoryName.c_str());
      argP.getArgument("-e", &exampleName, "Example name (default: %s)",
                       exampleName.c_str());
      Rcs::ExampleBase* example = Rcs::ExampleFactory::create(categoryName, exampleName, argc, argv);
      RCHECK_MSG(example, "Example %s unknown", exampleName.c_str());
      example->init(argc, argv);
      example->start();
      example->stop();
      delete example;
      break;
    }

    // ==============================================================
    // Tree view Gui
    // ==============================================================
    case 4:
    {
      int guiHandle = Rcs::TreeTest::create(argc, argv);
      RPAUSE();
      Rcs::TreeTest::destroy(guiHandle);
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


  RcsGuiFactory_shutdown();
  xmlCleanupParser();

  fprintf(stderr, "Thanks for using the ExampleRunner application\n");

  return 0;
}
