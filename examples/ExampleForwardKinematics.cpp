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
#include <Rcs_timer.h>
#include <Rcs_typedef.h>
#include <Rcs_parser.h>

#include <GraphNode.h>
#include <CapsuleNode.h>
#include <HUD.h>
#include <KeyCatcher.h>
#include <JointWidget.h>
#include <RcsViewer.h>
#include <Rcs_guiFactory.h>

#include <SegFaultHandler.h>

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
  RMSG("Starting Rcs...");

  const char* hgrDir = getenv("SIT");
  if (hgrDir != NULL)
  {
    std::string meshDir = std::string(hgrDir) + "/Data/RobotMeshes/1.0/data";
    Rcs_addResourcePath(std::string(meshDir).c_str());
  }

  int mode = 0;
  char xmlFileName[128] = "gScenario.xml";
  char directory[128] = "config/xml/DexBot";

  // Ctrl-C callback handler
  signal(SIGINT, quit);

  // This initialize the xml library and check potential mismatches between
  // the version it was compiled for and the actual shared library used.
  LIBXML_TEST_VERSION;

  // Parse command line arguments
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-dl", &RcsLogLevel, "Debug level (default is 0)");
  argP.getArgument("-m", &mode, "Test mode");
  argP.getArgument("-f", xmlFileName, "Configuration file name");
  argP.getArgument("-dir", directory, "Configuration file directory");
  bool simpleGraphics = argP.hasArgument("-simpleGraphics", "OpenGL without "
                                         "shadows and anti-aliasing)");

  // Add search path
  Rcs_addResourcePath("config");
  Rcs_addResourcePath(directory);

  // Initialize GUI and OSG mutex
  pthread_mutex_t graphLock;
  pthread_mutex_init(&graphLock, NULL);

  // Option without mutex for viewer
  pthread_mutex_t* mtx = &graphLock;
  if (argP.hasArgument("-nomutex", "Graphics without mutex"))
  {
    mtx = NULL;
  }



  switch (mode)
  {

    // ==============================================================
    // Just print out some global information
    // ==============================================================
    case 0:
    {
      printf("\n modes:\n\n");
      printf("\t-m");
      printf("\t0   Prints this message (default)\n");
      printf("\t\t1   Prints the graph to stdout, writes a dot file and "
             "shows\n\t\t    it with dotty\n");
      printf("\t\t2   Viewer + Joint Gui + Forward Kinematics\n");
      break;
    }

    // ==============================================================
    // Graph parsing and dot file output
    // ==============================================================
    case 1:
    {
      if (argP.hasArgument("-h"))
      {
        RMSG("Mode %d: \n\t- "
             "prints it's contents to the console\n\t- creates a dot-file "
             "RcsGraph.dot and\n\t- launches it with the dotty tool\n\n\tThe "
             "default xml file is \"%s\", the default directory is \"%s\"\n",
             mode, xmlFileName, directory);
        break;
      }


      RcsGraph* graph = RcsGraph_create(xmlFileName);
      RCHECK(graph);

      RMSG("Here's the forward tree:");
      RcsGraph_fprint(stderr, graph);
      RcsGraph_writeDotFile(graph, "RcsGraph.dot");

      double m = RcsGraph_mass(graph);
      RMSG("\nMass is %f [kg] = %f [N]", m, 9.81*m);

      RMSG("Writing graph to xml file");
      FILE* out = fopen("graph.xml", "w+");
      RCHECK(out);
      RcsGraph_fprintXML(out, graph);
      fclose(out);

      RcsGraph_destroy(graph);

      int err = system("dotty RcsGraph.dot&");

      if (err == -1)
      {
        RMSG("Couldn't start dot file viewer!");
      }
      break;
    }

    // ==============================================================
    // Forward kinematics
    // ==============================================================
    case 2:
    {
      Rcs::KeyCatcherBase::registerKey("q", "Quit");

      if (argP.hasArgument("-h"))
      {
        RMSG("Mode %d:\n\n\t- Creates a graph from an xml file\n\t"
             "- Creates a viewer (if option -valgrind is not set)\n\t"
             "- Creates a StateGui (if option -valgrind is not set)\n\t"
             "- Runs the forward kinematics in a loop\n\n\t"
             "The joints angles can be modified by the sliders\n", mode);
        break;
      }

      RcsGraph* graph = RcsGraph_create(xmlFileName);

      if (graph == NULL)
      {
        RMSG("Failed to create graph from file \"%s\" - exiting", xmlFileName);
        break;
      }

      // Make all joints constrained so that we can modify them with the Gui
      RCSGRAPH_TRAVERSE_JOINTS(graph)
      {
        JNT->constrained = true;
      }

      Rcs::Viewer viewer(!simpleGraphics, !simpleGraphics);
      osg::ref_ptr<Rcs::GraphNode> gn = new Rcs::GraphNode(graph);
      gn->toggleReferenceFrames();
      viewer.add(gn.get());

      osg::ref_ptr<Rcs::KeyCatcher> kc = new Rcs::KeyCatcher();
      viewer.add(kc.get());
      viewer.runInThread(mtx);

      Rcs::JointWidget::create(graph, mtx);


      while (runLoop)
      {
        pthread_mutex_lock(&graphLock);
        RcsGraph_setState(graph, NULL, NULL);
        pthread_mutex_unlock(&graphLock);

        if (kc->getAndResetKey('q'))
        {
          runLoop = false;
        }

        Timer_usleep(40000);
      }

      RcsGuiFactory_shutdown();
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


  if ((mode!=0) && argP.hasArgument("-h"))
  {
    argP.print();
    Rcs::KeyCatcherBase::printRegisteredKeys();
    Rcs_printResourcePath();
  }

  pthread_mutex_destroy(&graphLock);

  // Clean up global stuff. From the libxml2 documentation:
  // WARNING: if your application is multithreaded or has plugin support
  // calling this may crash the application if another thread or a plugin is
  // still using libxml2. It's sometimes very hard to guess if libxml2 is in
  // use in the application, some libraries or plugins may use it without
  // notice. In case of doubt abstain from calling this function or do it just
  // before calling exit() to avoid leak reports from valgrind !
  xmlCleanupParser();

  fprintf(stderr, "Thanks for using the Rcs libraries\n");

  return 0;
}
