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
#include <Rcs_math.h>
#include <Rcs_resourcePath.h>
#include <Rcs_timer.h>
#include <Rcs_typedef.h>
#include <Rcs_utils.h>
#include <IkSolverRMR.h>
#include <ViaPointSequence.h>
#include <JointWidget.h>
#include <Rcs_guiFactory.h>
#include <ControllerWidgetBase.h>
#include <DynamicDataPlot.h>
#include <AsyncWidget.h>
#include <MatNdWidget.h>
#include <PPSGui.h>
#include <SegFaultHandler.h>

#include <iostream>

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

class MatWidget : public Rcs::AsyncWidget
{
public:
  MatWidget(MatNd* mat_) : mat(mat_)
  {
    launch();
  }

  void construct()
  {
    setWidget(new Rcs::MatNdWidget(mat, mat, 0.0, 1.0, "Matrix", NULL));
  }

protected:
  MatNd* mat;
};

void testAsyncFactory(int argc, char** argv)
{
  {
    MatNd* mat = MatNd_create(3, 1);
    MatWidget mw(mat);
    MatWidget mw2(mat);
    MatWidget mw3(mat);
    RPAUSE();
    //mw3.destroy();
  }


  //Rcs::MyGuiFactory::create(argc, argv);
  {
    MatNd* mat = MatNd_create(3, 1);
    MatWidget mw(mat);
    MatWidget mw2(mat);
    MatWidget mw3(mat);

    //mw.launch();
    RPAUSE_MSG("Hit enter to unlaunch");

    //mw.unlaunch();
    mw.unlaunch();
    RPAUSE_MSG("Hit enter to launch");

    //MatWidget mw2(mat);
    mw.launch();
    RPAUSE_MSG("Hit enter to unlaunch");

    mw.unlaunch();
    mw.unlaunch();
    //RPAUSE_MSG("Hit enter to destroy facory");
  }





  //Rcs::MyGuiFactory::destroy();
  //RPAUSE_MSG("Hit enter to quit");
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
  int mode = 0;
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-dl", &RcsLogLevel, "Debug level (default is 0)");
  argP.getArgument("-m", &mode, "Test mode");

  // Initialize GUI and OSG mutex
  pthread_mutex_t graphLock;
  pthread_mutex_init(&graphLock, NULL);

  // Option without mutex for viewer
  pthread_mutex_t* mtx = &graphLock;
  if (argP.hasArgument("-nomutex", "Guis without mutex"))
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
      argP.print();

      printf("\nHere's some useful testing modes:\n\n");
      printf("\t-m");
      printf("\t0   Print this message\n");
      printf("\t\t1   MatNdWidget test\n");
      printf("\t\t2   JointWidget test\n");
      printf("\t\t3   DynamicDataPlot test\n");
      printf("\t\t4   DynamicDataPlot testwith ViaPointSequence\n");
      break;
    }

    // ==============================================================
    // MatNdWidget test
    // ==============================================================
    case 1:
    {
      unsigned int rows = 10, cols = 1;
      argP.getArgument("-rows", &rows, "Rows (default is %u)", rows);
      argP.getArgument("-cols", &cols, "Columns (default is %u)", cols);

      if (argP.hasArgument("-h"))
      {
        RMSG("Mode %d: Rcs -m %d\n", mode, mode);
        printf("\n\tMatNdWidget test\n");
        break;
      }

      MatNd* mat = MatNd_create(rows,cols);
      MatNd_setRandom(mat, -1.0, 1.0);
      Rcs::MatNdWidget::create(mat, NULL, mtx);

      while (runLoop==true)
      {
        pthread_mutex_lock(&graphLock);
        MatNd_printCommentDigits("mat", mat, 4);
        pthread_mutex_unlock(&graphLock);
        Timer_usleep(1000);
      }

      MatNd_destroy(mat);
      break;
    }

    // ==============================================================
    // JointWidget test
    // ==============================================================
    case 2:
    {
      char xmlFileName[128] = "gScenario.xml";
      char directory[128] = "config/xml/PPStest";
      argP.getArgument("-f", xmlFileName, "Configuration file name");
      argP.getArgument("-dir", directory, "Configuration file directory");
      bool writeQ1 = argP.hasArgument("-writeQ1", "Gui 1 always writes to q");
      bool writeQ2 = argP.hasArgument("-writeQ2", "Gui 2 always writes to q");
      bool passive1 = argP.hasArgument("-passive1", "Gui 1 passive");
      bool passive2 = argP.hasArgument("-passive2", "Gui 2 passive");


      Rcs_addResourcePath(directory);
      RcsGraph* graph = RcsGraph_create(xmlFileName);

      const RcsGraph* constGraph = graph;

      Rcs::JointWidget::create(graph, mtx, NULL, NULL, writeQ1, passive1);
      Rcs::JointWidget::create(constGraph, mtx, NULL, NULL, writeQ2, passive2);
      RPAUSE();

      break;
    }

    // ==============================================================
    // DynamicDataPlot test
    // ==============================================================
    case 3:
    {
      MatNd* mat = MatNd_create(1, 200);

      int handle = Rcs::DynamicDataPlot::create(mat, "Test", &graphLock);

      while (runLoop == true)
      {
        pthread_mutex_lock(&graphLock);
        MatNd_setRandom(mat, -1.0, 3.0);
        pthread_mutex_unlock(&graphLock);
        Timer_usleep(100000);
      }

      Rcs::DynamicDataPlot::destroy(handle);
      break;
    }

    // ==============================================================
    // DynamicDataPlot test with ViaPointSequence
    // ==============================================================
    case 4:
    {
      MatNd* viaDesc = MatNd_createFromString("0 0 0 0 7 , 0.9 1 0 0 1 , 1 1 0 0 7");
      Rcs::ViaPointSequence via(viaDesc);
      RCHECK(via.check());

      MatNd* traj_ = MatNd_create(4, 200);
      via.computeTrajectory(traj_);
      MatNd traj = MatNd_fromPtr(2, traj_->n, traj_->ele);

      int handle = Rcs::DynamicDataPlot::create(&traj, "Test", &graphLock);
      Rcs::MatNdWidget::create(viaDesc, -2.0, 2.0, "via", &graphLock);

      while (runLoop == true)
      {
        pthread_mutex_lock(&graphLock);
        via.init(viaDesc);
        via.computeTrajectory(traj_);
        pthread_mutex_unlock(&graphLock);
        Timer_usleep(100000);
      }

      Rcs::DynamicDataPlot::destroy(handle);
      break;
    }

    // ==============================================================
    // Asynchronous Gui factory
    // ==============================================================
    case 5:
    {
      testAsyncFactory(argc, argv);
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


  if ((mode!=0) && argP.hasArgument("-h", "Show help message"))
  {
    argP.print();
    Rcs_printResourcePath();
  }

  // Clean up global stuff. From the libxml2 documentation:
  // WARNING: if your application is multithreaded or has plugin support
  // calling this may crash the application if another thread or a plugin is
  // still using libxml2. It's sometimes very hard to guess if libxml2 is in
  // use in the application, some libraries or plugins may use it without
  // notice. In case of doubt abstain from calling this function or do it just
  // before calling exit() to avoid leak reports from valgrind !
  xmlCleanupParser();

  pthread_mutex_destroy(&graphLock);

  fprintf(stderr, "Thanks for using the Rcs libraries\n");

  return 0;
}
