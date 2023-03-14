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



/*! \page RcsExamples Rcs.cpp example program
 *
 *  The Rcs.cpp example program implements a number of different modes that
 *  show how to use the functionality of the libraries. The following command
 *  line arguments are common to all example modes. Each argument has a default
 *  value, that can be seen if running the program with the -h option. When
 *  calling Rcs -m \<mode\> -h, additionally the command line arguments of
 *  the specific mode will be printed to the console.
 *  - -dl:
 *     Sets the debug level of the overall library. As a convention, a
 *     debug level of 0 is silent. Debug levels between 1 and 4 will display
 *     warnings, the higher debug levels being more verbose. Debug levels above
 *     4 will tell you a lot of things that you might or might not be interested
 *     in. The example programs should not produce debug output on debug levels
 *     equal to or less than 4.
 *  - -m:
 *     Mode to select the example. The default mode is 0, which displays some
 *     information on usage.
 *  - -dir: Configuration file search directory.
 *  - -f: Configuration file
 *  - -simpleGraphics:
 *    Starts the graphics viewer with minimal settings (no anti-aliasing and
 *    shadows etc.). This is beneficial if the application is started on a
 *    remote computer, or the computer has a slow graphics card.
 *  - -nomutex:
 *    Disables mutex locking for the graphics viewer. This may result in some
 *    graphics artefacts, but does not compromise the calculation speed for
 *    computers with slow graphics cards.
 *  - -valgrind:
 *    Runs the algorithm for a fixed number of steps without launching Guis and
 *    graphics viewer. This allows to run memory checks without considering the
 *    non-relevant parts.
 *
 */



#include <Rcs_macros.h>
#include <Rcs_cmdLine.h>
#include <Rcs_math.h>
#include <Rcs_gradientTests.h>
#include <Rcs_resourcePath.h>
#include <Rcs_timer.h>
#include <Rcs_typedef.h>
#include <Rcs_kinematics.h>
#include <Rcs_joint.h>
#include <Rcs_shape.h>
#include <Rcs_utils.h>
#include <Rcs_utilsCPP.h>
#include <Rcs_filters.h>
#include <IkSolverRMR.h>
#include <SolverRAC.h>
#include <TaskFactory.h>
#include <TaskRegionFactory.h>
#include <PhysicsFactory.h>
#include <GraphNode.h>
#include <SphereNode.h>
#include <HUD.h>
#include <VertexArrayNode.h>
#include <KeyCatcher.h>
#include <JointWidget.h>
#include <RcsViewer.h>
#include <Rcs_graphicsUtils.h>
#include <Rcs_guiFactory.h>
#include <BodyPointDragger.h>
#include <ControllerWidgetBase.h>
#include <MatNdWidget.h>
#include <Rcs_mujocoParser.h>
#include <ExampleFactory.h>
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

/*******************************************************************************
 * A few pre-defined models
 ******************************************************************************/
static bool getModel(char* directory, char* xmlFileName)
{
  Rcs::CmdLineParser argP;

  if (!argP.hasArgument("-model"))
  {
    return false;
  }

  char model[64] = "";
  argP.getArgument("-model", model,
                   "Example models: Husky, DexBot, WAM, Humanoid");

  if (STREQ(model, "Husky"))
  {
    strcpy(directory, "config/xml/Husky");
    strcpy(xmlFileName, "husky-all.xml");
  }
  else if (STREQ(model, "DexBot"))
  {
    strcpy(directory, "config/xml/DexBot");
    strcpy(xmlFileName, "gScenario.xml");
  }
  else if (STREQ(model, "WAM"))
  {
    strcpy(directory, "config/xml/WAM");
    strcpy(xmlFileName, "gScenario.xml");
  }
  else if (STREQ(model, "Humanoid"))
  {
    strcpy(directory, "config/xml/GenericHumanoid");
    strcpy(xmlFileName, "gScenario.xml");
  }
  else
  {
    RMSG("Unknown model: %s", model);
    strcpy(directory, "");
    strcpy(xmlFileName, "");
    return false;
  }

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
int main(int argc, char** argv)
{
  int result = 0, mode = 0;
  char xmlFileName[128] = "", directory[128] = "";

  // Ctrl-C callback handler
  signal(SIGINT, quit);

  // This initialize the xml library and check potential mismatches between
  // the version it was compiled for and the actual shared library used.
  LIBXML_TEST_VERSION;

  // Parse command line arguments
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-dl", &RcsLogLevel, "Debug level (default is 0)");
  argP.getArgument("-m", &mode, "Test mode (default is %d)", mode);
  argP.getArgument("-f", xmlFileName, "Configuration file name");
  argP.getArgument("-dir", directory, "Configuration file directory");
  bool valgrind = argP.hasArgument("-valgrind",
                                   "Start without Guis and graphics");
  bool simpleGraphics = argP.hasArgument("-simpleGraphics", "OpenGL without fan"
                                         "cy stuff (shadows, anti-aliasing)");

  // Initialize GUI and OSG mutex
  pthread_mutex_t graphLock;
  pthread_mutex_init(&graphLock, NULL);

  // Option without mutex for viewer
  pthread_mutex_t* mtx = &graphLock;
  if (argP.hasArgument("-nomutex", "Graphics without mutex"))
  {
    mtx = NULL;
  }

  // Option to set locale - mainly for parsing tests
  if (argP.hasArgument("-locale", "Set locale"))
  {
    char localeStr[256] = "de_DE.utf8";
    argP.getArgument("-locale", localeStr);
    char* res = setlocale(LC_ALL, localeStr);
    if (res==NULL)
    {
      RLOG(1, "Failed to set locale \"%s\"", localeStr);
    }
    else
    {
      struct lconv* loc = localeconv();
      RLOG(1, "Locale successfully set to %s", res);
      RLOG(1, "Decimal character is %c", *(loc->decimal_point));
    }
  }

  const char* hgr = getenv("SIT");
  if (hgr != NULL)
  {
    std::string meshDir = std::string(hgr) +
                          std::string("/Data/RobotMeshes/1.0/data");
    Rcs_addResourcePath(meshDir.c_str());
  }

  Rcs_addResourcePath("config");

  RPAUSE_DL(5);

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
      printf("\t0   Prints this message (default)\n");
      printf("\t\t1   Prints the graph to stdout, writes a dot file and "
             "shows\n\t\t    it with dotty\n");
      printf("\t\t2   Viewer + StateGui + FK\n");
      printf("\t\t3   Gradient test for graphs\n");
      printf("\t\t4   Viewer + StateGui + FK + Physics\n");
      printf("\t\t5   Inverse kinematics + Viewer + Controller Gui\n");
      printf("\t\t6   Controller unit tests\n");
      printf("\t\t7   Null space convergence test\n");
      printf("\t\t8   Resolved acceleration controller test\n");
      printf("\t\t9   Depth first traversal test\n");
      printf("\t\t10  Task from string creation\n");
      printf("\t\t11  Jacobian re-projection test\n");

      REXEC(1)
      {
        RMSGS("SIT is \"%s\"",   getenv("SIT"));
        RMSGS("MAKEFILE_PLATFORM is \"%s\"", getenv("MAKEFILE_PLATFORM"));
        RMSGS("HOSTNAME is \"%s\"",          getenv("HOSTNAME"));
        Rcs_printResourcePath();
      }

      REXEC(2)
      {
        Rcs_printComputerStats(stdout);
      }

      REXEC(3)
      {
        Rcs::TaskFactory::printRegisteredTasks();
        Rcs::TaskRegionFactory::instance()->printRegisteredTaskRegions();
        Rcs::PhysicsFactory::print();
        RcsShape_fprintDistanceFunctions(stdout);
      }

      break;
    }

    // ==============================================================
    // Graph parsing and dot file output
    // ==============================================================
    case 1:
    {
      char dotFile[256] = "RcsGraph.dot";
      strcpy(xmlFileName, "gScenario.xml");
      strcpy(directory, "config/xml/DexBot");
      getModel(directory, xmlFileName);

      argP.getArgument("-f", xmlFileName, "Configuration file name (default "
                       "is \"%s\")", xmlFileName);
      argP.getArgument("-dir", directory, "Configuration file directory "
                       "(default is \"%s\")", directory);
      argP.getArgument("-dotFile", dotFile, "Dot file name");

      if (argP.hasArgument("-h"))
      {
        RMSG("Mode %d: Rcs -m %d -dir <graph-directory> -f "
             "<graph-file>\n\n\t- Creates a graph from an xml file\n\t- "
             "prints it's contents to the console\n\t- creates a dot-file "
             "RcsGraph.dot and\n\t- launches it with the dotty tool\n\n\t"
             "The default "
             "xml file is \"%s\", the default directory is \"%s\"\n",
             mode, mode, xmlFileName, directory);
        break;
      }

      Rcs_addResourcePath(directory);

      RcsGraph* graph = RcsGraph_create(xmlFileName);

      REXEC(1)
      {
        RMSG("Here's the forward tree:");
        RcsGraph_fprint(stderr, graph);
      }

      RMSG("Writing graph to dot file \"%s\"", dotFile);
      RcsGraph_writeDotFile(graph, dotFile);
      RcsGraph_writeDotFileDfsTraversal(graph, "RcsGraphDFS.dot");

      RMSG("Writing graph to xml file \"graph.xml\"");
      FILE* out = fopen("graph.xml", "w+");
      RcsGraph_fprintXML(out, graph);
      fclose(out);

      double mGraph = RcsGraph_mass(graph);
      RLOGS(0, "\nMass is %f [kg] = %f [N]", mGraph, 9.81*mGraph);

      RcsGraph_destroy(graph);

      REXEC(0)
      {
        if (valgrind==false)
        {
          std::string dottyCommand = "dotty " + std::string(dotFile) + "&";
          int err = system(dottyCommand.c_str());

          if (err == -1)
          {
            RMSG("Couldn't start dot file viewer!");
          }
        }
      }
      break;
    }

    // ==============================================================
    // Forward kinematics
    // ==============================================================
    case 2:
    {
      Rcs::ExampleFactory::runExample("Forward kinematics", "Dexbot",
                                      argc, argv, false);
      break;
    }

    // ==============================================================
    // Gradient tests for graphs
    // ==============================================================
    case 3:
    {
      int loopCount = 0, nIter = 1000;
      strcpy(xmlFileName, "LBR.xml");
      strcpy(directory, "config/xml/DexBot");
      argP.getArgument("-f", xmlFileName, "Configuration file name (default "
                       "is \"%s\")", xmlFileName);
      argP.getArgument("-dir", directory, "Configuration file directory "
                       "(default is \"%s\")", directory);
      argP.getArgument("-iter", &nIter, "Number of test iterations "
                       "(default is \"%d\")", nIter);

      if (argP.hasArgument("-h"))
      {
        RMSG("Mode %d: Rcs -m %d -dir <graph-directory> -f "
             "<graph-file>\n\n\t- Creates a graph from an xml file)\n\t"
             "- Runs a set of gradient tests on it\n\n\t"
             "On debug level 1 and higher, a viewer is launched, and more "
             "verbose information is printed to the console. The default "
             "xml file is \"%s\", the default directory is \"%s\"\n",
             mode, mode, xmlFileName, directory);
        break;
      }

      Rcs_addResourcePath(directory);

      RcsGraph* graph = RcsGraph_create(xmlFileName);
      RcsGraph_setState(graph, NULL, NULL);
      MatNd* q_test = MatNd_create(graph->dof, 1);

      if ((valgrind==0) && (RcsLogLevel>0))
      {
        Rcs::Viewer* viewer = new Rcs::Viewer(!simpleGraphics, !simpleGraphics);
        Rcs::GraphNode* gn = new Rcs::GraphNode(graph);
        gn->toggleReferenceFrames();
        viewer->add(gn);
        viewer->runInThread();
      }

      // Run tests with random state vectors
      while (runLoop)
      {
        RCSGRAPH_TRAVERSE_JOINTS(graph)
        {
          if (RcsJoint_isRotation(JNT))
          {
            MatNd_set(graph->q, JNT->jointIndex, 0,
                      Math_getRandomNumber(-M_PI_2, M_PI_2));
          }
          else
          {
            MatNd_set(graph->q, JNT->jointIndex, 0,
                      Math_getRandomNumber(-0.1, 0.1));
          }
        }

        MatNd_copy(q_test, graph->q);
        bool success = Rcs_gradientTestGraph(graph, q_test,
                                             RcsLogLevel>0?true:false);
        if (!success)
        {
          result++;
        }

        loopCount++;

        if (loopCount >= nIter)
        {
          runLoop = false;
        }
      }

      RcsGraph_destroy(graph);
      MatNd_destroy(q_test);
      break;
    }


    // ==============================================================
    // Run Scenario file with physics
    // ==============================================================
    case 4:
    {
      Rcs::ExampleFactory::runExample("Physics", "Dexbot", argc, argv, false);
      break;
    }

    // ==============================================================
    // Inverse kinematics
    // ==============================================================
    case 5:
    {
      Rcs::ExampleFactory::runExample("Inverse kinematics",
                                      "Dexbot with Task Interval",
                                      argc, argv, false);

      break;
    }


    // ==============================================================
    // Controller unit tests
    // ==============================================================
    case 6:
    {
      Rcs::KeyCatcherBase::registerKey("q", "Quit");
      Rcs::KeyCatcherBase::registerKey("p", "Toggle pause");

      int nTests = 1, loopCount = 0;
      strcpy(xmlFileName, "cAction.xml");
      strcpy(directory, "config/xml/GenericHumanoid");
      argP.getArgument("-f", xmlFileName);
      argP.getArgument("-dir", directory);
      argP.getArgument("-nTests", &nTests, "Number of test iterations (default"
                       " is %d)", nTests);
      bool pause = argP.hasArgument("-pause", "Pause after each iteration");
      bool skipGraphics = valgrind ||
                          argP.hasArgument("-noGraphics",
                                           "Skip graphics window");

      if (argP.hasArgument("-h"))
      {
        RMSG("Mode %d: Rcs -m %d\n", mode, mode);
        printf("\n\tController gradient tests\n");
        break;
      }

      Rcs_addResourcePath(directory);

      // Create controller
      Rcs::ControllerBase controller(xmlFileName);

      // Arrays for random q and q_dot vectors. The q_dot vector must not
      // comprise constrained dofs, since they would contribute to a
      // velocity component that is not part of the Jacobian projection.
      MatNd* q = MatNd_clone(controller.getGraph()->q);
      MatNd* q_dot = MatNd_create(controller.getGraph()->nJ, 1);

      // Create visualization
      Rcs::KeyCatcher* kc = NULL;
      if (skipGraphics==false)
      {
        Rcs::Viewer* v = new Rcs::Viewer(!simpleGraphics, !simpleGraphics);
        Rcs::GraphNode* gn = new Rcs::GraphNode(controller.getGraph());
        kc = new Rcs::KeyCatcher();
        v->add(gn);
        v->add(kc);
        v->runInThread(mtx);
      }


      if (pause==true)
      {
        RPAUSE_MSG("Hit enter to start endless loop");
      }

      // Endless loop
      while (runLoop == true)
      {
        MatNd_setRandom(q, -1.0, 1.0);
        MatNd_setRandom(q_dot, -1.0, 1.0);

        pthread_mutex_lock(&graphLock);
        RcsGraph_setState(controller.getGraph(), q, q_dot);
        pthread_mutex_unlock(&graphLock);

        bool success = controller.test(RcsLogLevel>1?true:false);

        if (success == false)
        {
          //RPAUSE_MSG("Test failed");
          result++;
        }

        if (kc && kc->getAndResetKey('q'))
        {
          runLoop = false;
        }
        else if (kc && kc->getAndResetKey('p'))
        {
          pause = !pause;
        }

        if ((valgrind==true) && (loopCount>=nTests))
        {
          runLoop = false;
        }

        if (pause==true)
        {
          RPAUSE_MSG("Hit enter to continue");
        }
        else
        {
          Timer_waitDT(0.01);
        }

        loopCount++;
      }

      MatNd_destroy(q);
      MatNd_destroy(q_dot);

      RLOGS(1, "%s testing controller gradients",
            (result==0) ? "SUCCESS" : "FAILURE");

      break;
    }


    // ==============================================================
    // Null space gradient test
    // ==============================================================
    case 7:
    {
      Rcs::KeyCatcherBase::registerKey("q", "Quit");
      Rcs::KeyCatcherBase::registerKey("t", "Run controller test");
      Rcs::KeyCatcherBase::registerKey("Space", "Toggle pause");
      Rcs::KeyCatcherBase::registerKey("a", "Change IK algorithm");
      Rcs::KeyCatcherBase::registerKey("p", "Print information to console");
      Rcs::KeyCatcherBase::registerKey("n", "Reset");

      int algo = 1, nTests = -1;
      unsigned int loopCount = 0, nIter = 10000;
      double alpha = 0.01, lambda = 1.0e-8, det = 0.0;
      double jlCost = 0.0, dJlCost = 0.0, eps=1.0e-5;
      strcpy(xmlFileName, "cAction.xml");
      strcpy(directory, "config/xml/DexBot");

      argP.getArgument("-iter", &nIter, "Number of iterations before next pose"
                       "(default is %u)", nIter);
      argP.getArgument("-nTests", &nTests, "Number of test iterations (default"
                       " is %d)", nTests);
      argP.getArgument("-algo", &algo, "IK algorithm: 0: left inverse, 1: "
                       "right inverse (default is %d)", algo);
      argP.getArgument("-alpha", &alpha,
                       "Null space scaling factor (default is %f)", alpha);
      argP.getArgument("-lambda", &lambda, "Regularization (default is %f)",
                       lambda);
      argP.getArgument("-f", xmlFileName);
      argP.getArgument("-dir", directory);
      argP.getArgument("-eps", &eps, "Small numerical treshold that is "
                       "acceptable as increase of the null space cost "
                       "(default is %f)", eps);
      bool pause = argP.hasArgument("-pause", "Pause after each iteration");

      Rcs_addResourcePath(directory);

      if (argP.hasArgument("-h"))
      {
        RMSG("Rcs -m %d\n", mode);
        printf("\n\tController nullspace gradient tests: The controller is "
               "initialized in a random pose. From there, the null space is "
               "iterated for a fixed number of steps (command line argument"
               " \"iter\"). In each iteration, it is checked if the cost "
               "function value is decreasing. If it is not the case (with a"
               " threshold of command line argument \"eps\"), the program is"
               " paused in the respective pose.\n");
        break;
      }

      // Create controller
      Rcs::ControllerBase controller(xmlFileName);
      Rcs::IkSolverRMR ikSolver(&controller);

      MatNd* dq_des  = MatNd_create(controller.getGraph()->dof, 1);
      MatNd* a_des   = MatNd_create(controller.getNumberOfTasks(), 1);
      MatNd* x_curr  = MatNd_create(controller.getTaskDim(), 1);
      MatNd* x_des   = MatNd_create(controller.getTaskDim(), 1);
      MatNd* dx_des  = MatNd_create(controller.getTaskDim(), 1);
      MatNd* dH      = MatNd_create(1, controller.getGraph()->nJ);

      controller.readActivationsFromXML(a_des);
      controller.computeX(x_curr);
      MatNd_copy(x_des, x_curr);

      // Create visualization
      Rcs::Viewer* v           = NULL;
      Rcs::KeyCatcher* kc      = NULL;
      Rcs::GraphNode* gn       = NULL;
      Rcs::HUD* hud            = NULL;
      Rcs::BodyPointDragger* dragger = NULL;
      char hudText[2056];

      if (valgrind==false)
      {
        v       = new Rcs::Viewer(!simpleGraphics, !simpleGraphics);
        kc      = new Rcs::KeyCatcher();
        gn      = new Rcs::GraphNode(controller.getGraph());
        hud     = new Rcs::HUD();
        dragger = new Rcs::BodyPointDragger();
        v->add(gn);
        v->add(hud);
        v->add(kc);
        v->add(dragger);
        v->runInThread(mtx);

        // Launch the activation widget
        std::vector<std::string> labels;
        Rcs::MatNdWidget* mw = Rcs::MatNdWidget::create(a_des, a_des,
                                                        0.0, 1.0, "activation",
                                                        &graphLock);
        for (size_t id=0; id<controller.getNumberOfTasks(); id++)
        {
          labels.push_back(controller.getTaskName(id));
        }
        mw->setLabels(labels);
      }



      // Endless loop
      while (runLoop == true)
      {
        pthread_mutex_lock(&graphLock);
        double dt = Timer_getTime();

        // Set state to random and compute null space cost and gradient
        if (loopCount%nIter==0)
        {
          RCSGRAPH_TRAVERSE_JOINTS(controller.getGraph())
          {
            controller.getGraph()->q->ele[JNT->jointIndex] =
              Math_getRandomNumber(JNT->q_min, JNT->q_max);
          }
        }

        RcsGraph_setState(controller.getGraph(), NULL, NULL);
        jlCost = controller.computeJointlimitCost();
        controller.computeJointlimitGradient(dH);
        MatNd_constMulSelf(dH, alpha);

        double dtIK = Timer_getTime();

        switch (algo)
        {
          case 0:
            det = ikSolver.solveLeftInverse(dq_des, dx_des, dH, a_des, lambda);
            break;

          case 1:
            det = ikSolver.solveRightInverse(dq_des, dx_des, dH, a_des, lambda);
            break;

          default:
            RFATAL("No such algorithm; %d", algo);
        }

        dtIK = Timer_getTime() - dtIK;

        MatNd_addSelf(controller.getGraph()->q, dq_des);
        RcsGraph_setState(controller.getGraph(), NULL, NULL);
        controller.computeX(x_curr);
        dt = Timer_getTime() - dt;

        dJlCost = -jlCost;
        jlCost = controller.computeJointlimitCost();
        dJlCost += jlCost;

        pthread_mutex_unlock(&graphLock);

        if (kc && kc->getAndResetKey('q'))
        {
          runLoop = false;
        }
        else if (kc && kc->getAndResetKey('a'))
        {
          algo++;
          if (algo>1)
          {
            algo = 0;
          }

          RLOGS(0, "Switching to IK algorithm %d", algo);
        }
        else if (kc && kc->getAndResetKey('t'))
        {
          RLOGS(0, "Running controller test");
          controller.test(true);
        }
        else if (kc && kc->getAndResetKey(' '))
        {
          pause = !pause;
          RMSG("Pause modus is %s", pause ? "ON" : "OFF");
        }
        else if (kc && kc->getAndResetKey('p'))
        {
          RMSG("Writing q to file \"q.dat\"");
          MatNd_toFile(controller.getGraph()->q, "q.dat");
        }
        else if (kc && kc->getAndResetKey('P'))
        {
          RMSG("Reading q from file \"q.dat\"");
          MatNd_fromFile(controller.getGraph()->q, "q.dat");
          RcsGraph_setState(controller.getGraph(), NULL, NULL);
        }
        else if (kc && kc->getAndResetKey('n'))
        {
          RMSG("Resetting");
          RcsGraph_setDefaultState(controller.getGraph());
        }

        sprintf(hudText, "%.1f %%: IK calculation: %.2f ms\ndof: %d nJ: %d "
                "nqr: %d nx: %zu\nJL-cost: %.6f dJL-cost: %.6f %s %s\n"
                "dt=%.2f us\nalgo: %d lambda:%g alpha: %g",
                fmod(100.0*((double)loopCount/nIter), 100.0),
                1.0e3*dt, controller.getGraph()->dof,
                controller.getGraph()->nJ, ikSolver.getInternalDof(),
                controller.getActiveTaskDim(a_des),
                jlCost, dJlCost,
                det==0.0?"SINGULAR":"",
                ((dJlCost > eps) && (MatNd_getNorm(dx_des) == 0.0)) ?
                "COST INCREASE" : "",
                dtIK*1.0e6, algo, lambda, alpha);

        if (hud != NULL)
        {
          hud->setText(hudText);
        }
        else
        {
          REXEC(3)
          {
            std::cout << hudText;
          }
        }

        if ((valgrind==true) && (loopCount>2*nIter))
        {
          runLoop = false;
        }

        if (pause==true)
        {
          RPAUSE();
        }


        if ((dJlCost > eps) && (MatNd_getNorm(dx_des) == 0.0))
        {
          RLOG(2, "COST INCREASE: %g", dJlCost);
          RPAUSE_DL(3);
          result++;
        }

        loopCount++;

        if (!valgrind)
        {
          Timer_usleep(1);
        }

      }



      // Clean up
      if (valgrind==false)
      {
        delete v;
        RcsGuiFactory_shutdown();
      }

      MatNd_destroy(dq_des);
      MatNd_destroy(a_des);
      MatNd_destroy(x_curr);
      MatNd_destroy(x_des);
      MatNd_destroy(dx_des);
      MatNd_destroy(dH);

      RLOGS(1, "%s testing null space projections",
            (result==0) ? "SUCCESS" : "FAILURE");

      break;
    }

    // ==============================================================
    // Resolved acceleration control
    // ==============================================================
    case 8:
    {
      Rcs::KeyCatcherBase::registerKey("q", "Quit");
      Rcs::KeyCatcherBase::registerKey("t", "Run controller test");
      Rcs::KeyCatcherBase::registerKey("Space", "Toggle pause");
      Rcs::KeyCatcherBase::registerKey("n", "Reset");
      Rcs::KeyCatcherBase::registerKey("o", "Set random pose");
      Rcs::KeyCatcherBase::registerKey("T", "RAC solver test");

      double lambda = 0.0;
      double jlCost = 0.0, dJlCost=0.0, tmc=0.1, dt=0.01, kp_nullspace=100.0;
      double kp = 100.0;
      bool pause = false;
      int guiHandle = -1;
      strcpy(xmlFileName, "cAction.xml");
      strcpy(directory, "config/xml/DexBot");

      argP.getArgument("-lambda", &lambda, "Regularization (default: %f",
                       lambda);
      argP.getArgument("-f", xmlFileName);
      argP.getArgument("-dir", directory);
      argP.getArgument("-tmc", &tmc, "Gui filter time constant ([0 ... 1], "
                       "small is smooth, default is %f)", tmc);
      argP.getArgument("-dt", &dt, "Sampling time constant (default: %f)", dt);
      argP.getArgument("-kp_nullspace", &kp_nullspace, "Null space gain "
                       "(default: %f)", kp_nullspace);
      double kd_nullspace = 2.0*sqrt(kp_nullspace);
      argP.getArgument("-kd_nullspace", &kd_nullspace, "Null space damping "
                       "(default: %f)", kd_nullspace);
      argP.getArgument("-kp", &kp, "Position gain (default: %f)", kp);
      double kd = 2.0*sqrt(kp);
      argP.getArgument("-kd", &kd, "Velocity gain (default: %f)", kd);
      bool ffwd = argP.hasArgument("-ffwd", "Feed forward only");

      Rcs_addResourcePath(directory);

      // Create controller
      Rcs::ControllerBase controller(xmlFileName);

      RCSGRAPH_TRAVERSE_JOINTS(controller.getGraph())
      {
        JNT->coupledToId = -1;
      }
      RcsGraph_setState(controller.getGraph(), NULL, NULL);


      Rcs::SolverRAC solver(&controller);

      unsigned int nx = controller.getTaskDim();
      MatNd* qpp_des = MatNd_create(controller.getGraph()->dof, 1);
      MatNd* qp_des  = MatNd_create(controller.getGraph()->dof, 1);
      MatNd* q_des   = MatNd_clone(controller.getGraph()->q);
      MatNd* a_des   = MatNd_create(controller.getNumberOfTasks(), 1);
      MatNd* x_curr  = MatNd_create(nx, 1);
      MatNd* xp_curr = MatNd_create(nx, 1);
      MatNd* x_des   = MatNd_create(nx, 1);
      MatNd* x_des_f = MatNd_create(nx, 1);
      MatNd* xp_des  = MatNd_create(nx, 1);
      MatNd* xpp_des = MatNd_create(nx, 1);
      MatNd* ax_des  = MatNd_create(nx, 1);
      MatNd* kpVec   = MatNd_create(nx, 1);
      MatNd* kdVec   = MatNd_create(nx, 1);
      MatNd* dH      = MatNd_create(1, controller.getGraph()->nJ);

      MatNd_setElementsTo(kpVec, kp);
      MatNd_setElementsTo(kdVec, kd);

      controller.readActivationsFromXML(a_des);
      controller.computeX(x_curr);
      MatNd_copy(x_des, x_curr);
      MatNd_copy(x_des_f, x_curr);

      Rcs::RampFilterND guiFilt(x_des->ele, tmc, 10.0, dt, x_des->m);

      // Create visualization
      Rcs::Viewer* v           = NULL;
      Rcs::KeyCatcher* kc      = NULL;
      Rcs::GraphNode* gn       = NULL;
      Rcs::HUD* hud            = NULL;
      Rcs::BodyPointDragger* dragger = NULL;
      char hudText[2056];

      if (argP.hasArgument("-h"))
      {
        printf("Resolved acceleration control test\n\n");
        break;
      }

      if (valgrind==false)
      {
        v       = new Rcs::Viewer(!simpleGraphics, !simpleGraphics);
        kc      = new Rcs::KeyCatcher();
        gn      = new Rcs::GraphNode(controller.getGraph());
        hud     = new Rcs::HUD();
        dragger = new Rcs::BodyPointDragger();
        v->add(gn);
        v->add(hud);
        v->add(kc);
        v->add(dragger);
        v->runInThread(mtx);

        // Launch the task widget
        if (ffwd == false)
        {
          guiHandle = Rcs::ControllerWidgetBase::create(&controller, a_des,
                                                        x_des, x_curr, mtx);
        }
        else
        {
          // Launch the task widget
          Rcs::MatNdWidget* mw = Rcs::MatNdWidget::create(xpp_des, x_curr,
                                                          -1000.0, 1000.0, "xpp_des",
                                                          mtx);

          std::vector<std::string> labels;
          for (size_t id=0; id<controller.getNumberOfTasks(); id++)
          {
            for (unsigned int j=0; j<controller.getTaskDim(id); j++)
              labels.push_back(controller.getTaskName(id) +
                               std::string(": ") +
                               controller.getTask(id)->getParameter(j).name);
          }

          mw->setLabels(labels);

          mw = Rcs::MatNdWidget::create(a_des, a_des,
                                        0.0, 1.0, "activation",
                                        &graphLock);
          labels.clear();
          for (size_t id=0; id<controller.getNumberOfTasks(); id++)
          {
            labels.push_back(controller.getTaskName(id));
          }
          mw->setLabels(labels);
        }


        if (argP.hasArgument("-jointWidget"))
        {
          Rcs::JointWidget::create(controller.getGraph(), mtx);
        }
      }

      unsigned int loopCount = 0;


      // Endless loop
      while (runLoop == true)
      {
        pthread_mutex_lock(&graphLock);
        double dt_compute = Timer_getTime();

        if (ffwd==false)
        {
          guiFilt.setTarget(x_des->ele);
          guiFilt.iterate(xpp_des->ele);
          guiFilt.getPosition(x_des_f->ele);
          guiFilt.getVelocity(xp_des->ele);
          controller.computeAx(ax_des, a_des, x_des_f,
                               xp_des, xpp_des, kpVec, kdVec);
        }
        else
        {
          controller.computeX(x_curr);
          controller.computeXp(xp_curr);
          controller.computeAx(ax_des, a_des, x_curr,
                               xp_curr, xpp_des, kpVec, kdVec);
        }

        controller.computeJointlimitGradient(dH);
        MatNd_constMulSelf(dH, kp_nullspace);

        // Joint speed damping: Kd(qp_des - qp) with qp_des = 0
        // MatNd* q_dot_ik = MatNd_create(1, controller.getGraph()->nJ);
        // RcsGraph_stateVectorToIK(controller.getGraph(),
        //                          controller.getGraph()->q_dot, q_dot_ik);
        // MatNd_transposeSelf(q_dot_ik);
        // MatNd_constMulAndAddSelf(dH, q_dot_ik, -kd_nullspace);


        dragger->addJointTorque(dH, controller.getGraph());
        solver.solve(qpp_des, a_des, ax_des, dH, lambda);

        MatNd_constMulAndAddSelf(qp_des, qpp_des, dt);
        MatNd_constMulAndAddSelf(q_des, qp_des, dt);
        RcsGraph_setState(controller.getGraph(), q_des, qp_des);
        controller.computeX(x_curr);
        dt_compute = Timer_getTime() - dt_compute;

        dJlCost = -jlCost;
        jlCost = controller.computeJointlimitCost();
        dJlCost += jlCost;

        pthread_mutex_unlock(&graphLock);

        if (kc && kc->getAndResetKey('q'))
        {
          runLoop = false;
        }
        else if (kc && kc->getAndResetKey('t'))
        {
          RLOGS(0, "Running controller test");
          controller.test(true);
        }
        else if (kc && kc->getAndResetKey(' '))
        {
          pause = !pause;
          RMSG("Pause modus is %s", pause ? "ON" : "OFF");
        }
        else if (kc && kc->getAndResetKey('n'))
        {
          RMSG("Resetting");
          RcsGraph_setDefaultState(controller.getGraph());
          MatNd_setZero(qp_des);
          MatNd_setZero(qpp_des);
          MatNd_copy(q_des, controller.getGraph()->q);
          MatNd_copy(qp_des, controller.getGraph()->q_dot);
          controller.computeX(x_des);
          MatNd_copy(x_des_f, x_des);
          MatNd_setZero(xp_des);
          MatNd_setZero(xpp_des);
          void* ptr = RcsGuiFactory_getPointer(guiHandle);
          Rcs::ControllerWidgetBase* cw =
            static_cast<Rcs::ControllerWidgetBase*>(ptr);
          cw->reset(a_des, x_des);
        }
        else if (kc && kc->getAndResetKey('o'))
        {
          RMSG("Setting to random state");
          MatNd_setRandom(q_des, -M_PI, M_PI);
          MatNd_setZero(qp_des);
          MatNd_setZero(qpp_des);
          RcsGraph_setState(controller.getGraph(), q_des, qp_des);
          controller.computeX(x_des);
          MatNd_copy(x_des_f, x_des);
          MatNd_setZero(xp_des);
          MatNd_setZero(xpp_des);
          void* ptr = RcsGuiFactory_getPointer(guiHandle);
          Rcs::ControllerWidgetBase* cw =
            static_cast<Rcs::ControllerWidgetBase*>(ptr);
          cw->reset(a_des, x_des);
        }
        else if (kc && kc->getAndResetKey('T'))
        {
          solver.test(a_des);
        }

        sprintf(hudText, "RAC calculation: %.2f ms\n"
                "nx: %d\nJL-cost: %.6f dJL-cost: %.6f"
                "\nlambda:%g alpha: %g tmc: %g",
                1.0e3*dt_compute, (int) controller.getActiveTaskDim(a_des),
                jlCost, dJlCost, lambda, kp_nullspace, tmc);

        if (hud != NULL)
        {
          hud->setText(hudText);
        }
        else
        {
          std::cout << hudText;
        }

        if ((valgrind==true) && (loopCount>10))
        {
          runLoop = false;
        }

        if (pause==true)
        {
          RPAUSE();
        }

        loopCount++;
        Timer_waitDT(dt);
      }



      // Clean up
      if (valgrind==false)
      {
        delete v;
        RcsGuiFactory_shutdown();
      }

      MatNd_destroy(qpp_des);
      MatNd_destroy(qp_des);
      MatNd_destroy(q_des);
      MatNd_destroy(a_des);
      MatNd_destroy(x_curr);
      MatNd_destroy(xp_curr);
      MatNd_destroy(x_des);
      MatNd_destroy(x_des_f);
      MatNd_destroy(xp_des);
      MatNd_destroy(xpp_des);
      MatNd_destroy(ax_des);
      MatNd_destroy(dH);
      break;
    }

    // ==============================================================
    // Depth first traversal test
    // ==============================================================
    case 9:
    {
      char dotFile[256], dotFileDfs[256], attachTo[256];
      strcpy(dotFile, "RcsGraph.dot");
      strcpy(attachTo, "");

      argP.getArgument("-dotFile", dotFile, "Dot file name");

      const char* fileExtension = strrchr(dotFile, '.');

      if (fileExtension==NULL)
      {
        snprintf(dotFileDfs, 256, "%s%s", dotFile, "DFS");
      }
      else
      {
        snprintf(dotFileDfs, 256, "%s%s%s", dotFile, "DFS", fileExtension);
      }

      argP.getArgument("-attachTo", attachTo, "Body to attach graph");

      if (!argP.hasArgument("-f"))
      {
        strcpy(xmlFileName, "WAM-arm-primitives.xml");
      }

      if (!argP.hasArgument("-dir"))
      {
        strcpy(directory, "config/xml/WAM");
      }

      if (argP.hasArgument("-h"))
      {
        break;
      }

      Rcs_addResourcePath(directory);

      RcsGraph* graph = RcsGraph_create(xmlFileName);
      RcsGraph* attachedGraph = RcsGraph_create(xmlFileName);
      RcsBody* attachementBody = RcsGraph_getBodyByName(graph, attachTo);

      HTr A;
      HTr_setIdentity(&A);
      A.org[1] = 0.5;
      Mat3d_rotateSelfAboutXYZAxis(A.rot, 0, 15.0*M_PI/180.0);
      Mat3d_rotateSelfAboutXYZAxis(A.rot, 2, 15.0*M_PI/180.0);
      Mat3d_rotateSelfAboutXYZAxis(A.rot, 1, 15.0*M_PI/180.0);

      bool success = RcsGraph_appendCopyOfGraph(graph, attachementBody,
                                                attachedGraph, "_2", &A);

      RCHECK(success);
      A.org[1] = 1.0;
      success = RcsGraph_appendCopyOfGraph(graph, attachementBody,
                                           attachedGraph, "_3", &A);
      RCHECK(success);
      A.org[1] = 1.5;
      success = RcsGraph_appendCopyOfGraph(graph, attachementBody,
                                           attachedGraph, "_4", &A);
      RCHECK(success);
      RCHECK(RcsGraph_check(graph, NULL, NULL));

      RcsGraph_writeDotFile(graph, dotFile);
      RcsGraph_writeDotFileDfsTraversal(graph, dotFileDfs);

      RcsGraph_destroy(graph);

      REXEC(0)
      {
        if (valgrind==false)
        {
          char osCmd[256];
          sprintf(osCmd, "dotty %s&", dotFile);
          int err = system(osCmd);

          if (err == -1)
          {
            RMSG("Couldn't start dot file viewer with command \"%s\"", osCmd);
          }

          sprintf(osCmd, "dotty %s&", dotFileDfs);

          err = system(osCmd);

          if (err == -1)
          {
            RMSG("Couldn't start dot file viewer with command \"%s\"", osCmd);
          }
        }
      }

      RcsGraph_destroy(attachedGraph);
      break;
    }

    // ==============================================================
    // Task from string test
    // ==============================================================
    case 10:
    {
      strcpy(xmlFileName, "gScenario.xml");
      strcpy(directory, "config/xml/DexBot");
      Rcs_addResourcePath(directory);

      RcsGraph* graph = RcsGraph_create(xmlFileName);
      RCHECK(graph);

      const char* descr =
        "<Task controlVariable=\"XYZ\" effector=\"PowerGrasp_L\" />";

      Rcs::Task* task = Rcs::TaskFactory::createTask(descr, graph);

      if (task == NULL)
      {
        RLOG((valgrind ? 2 : 0), "Can't create task \n\n%s\n\n", descr);
        result++;
      }
      else
      {
        if (!valgrind)
        {
          task->print();
        }
        delete task;
      }

      RcsGraph_destroy(graph);

      RLOGS(1, "%s testing task from string creation",
            (result==0) ? "SUCCESS" : "FAILURE");

      break;
    }

    // ==============================================================
    // Test for Jacobian re-projection
    // ==============================================================
    case 11:
    {
      strcpy(xmlFileName, "LBR.xml");
      strcpy(directory, "config/xml/DexBot");
      Rcs_addResourcePath(directory);
      RcsGraph* graph = RcsGraph_create(xmlFileName);

      double w = 1.0;
      argP.getArgument("-w", &w, "Weight");


      MatNd_setRandom(graph->q, -1.0, 1.0);
      MatNd_setElementsTo(graph->q, 1.0);
      RcsGraph_setState(graph, NULL, NULL);

      MatNd* J = MatNd_create(3, graph->nJ);
      MatNd* pinvJ = MatNd_create(graph->nJ, 3);
      MatNd* N = MatNd_create(graph->nJ, graph->nJ);
      MatNd* invW = MatNd_create(graph->nJ, 1);
      MatNd* F = MatNd_create(3, 1);
      MatNd* T = MatNd_create(graph->nJ, 1);
      MatNd* lambda = MatNd_create(1, 1);
      MatNd_setElementsTo(invW, w);
      MatNd_setElementsTo(T, 1.0);

      RcsBody* leafNode = NULL;
      RCSGRAPH_TRAVERSE_BODIES(graph)
      {
        leafNode = BODY;
      }

      Rcs::MatNdWidget::create(invW, invW, 0.0, 1.0, "weighting");
      Rcs::MatNdWidget::create(T, T, 0.0, 1.0, "torque");

      RcsGraph_worldPointJacobian(graph, leafNode, Vec3d_ez(), NULL, J);

      while (runLoop)
      {

        MatNd_rwPinv(pinvJ, J, invW, lambda);
        MatNd_transposeSelf(pinvJ);
        MatNd_mul(F, pinvJ, T);

        REXEC(0)
        {
          MatNd_printCommentDigits("J", J, 3);
          MatNd_printCommentDigits("pinvJ", pinvJ, 3);
          MatNd_printCommentDigits("T", T, 3);
          MatNd_printCommentDigits("F", F, 3);
          RMSG("F_res = %f", Vec3d_getLength(F->ele));
        }

        Timer_waitDT(0.05);
        //RPAUSE();
      }

      MatNd_destroy(J);
      MatNd_destroy(pinvJ);
      MatNd_destroy(N);
      MatNd_destroy(invW);
      MatNd_destroy(lambda);
      MatNd_destroy(T);
      MatNd_destroy(F);
      RcsGraph_destroy(graph);
      break;
    }


    // ==============================================================
    // Null space task re-projections test
    // ==============================================================
    case 12:
    {
      Rcs::KeyCatcherBase::registerKey("q", "Quit");
      Rcs::KeyCatcherBase::registerKey("t", "Run controller test");
      Rcs::KeyCatcherBase::registerKey("Space", "Toggle pause");
      Rcs::KeyCatcherBase::registerKey("n", "Reset");

      int nTests = -1;
      unsigned int loopCount = 0, nIter = 10000;
      double alpha = 0.05, lambda = 0.0, det = 0.0;
      double jlCost = 0.0, dJlCost = 0.0, eps=0*1.0e-5;
      strcpy(xmlFileName, "cAction.xml");
      strcpy(directory, "config/xml/DexBot");

      argP.getArgument("-iter", &nIter, "Number of iterations before next pose"
                       "(default is %u)", nIter);
      argP.getArgument("-nTests", &nTests, "Number of test iterations (default"
                       " is %d)", nTests);
      argP.getArgument("-alpha", &alpha,
                       "Null space scaling factor (default is %f)", alpha);
      argP.getArgument("-lambda", &lambda, "Regularization (default is %f)",
                       lambda);
      argP.getArgument("-f", xmlFileName);
      argP.getArgument("-dir", directory);
      argP.getArgument("-eps", &eps, "Small numerical treshold that is "
                       "acceptable as increase of the null space cost "
                       "(default is %f)", eps);
      bool pause = argP.hasArgument("-pause", "Pause after each iteration");
      bool projJ = argP.hasArgument("-projJ", "Projection: J dH^T");

      Rcs_addResourcePath(directory);

      if (argP.hasArgument("-h"))
      {
        break;
      }

      Rcs::ControllerBase controller(xmlFileName);
      Rcs::IkSolverRMR ikSolver(&controller);

      MatNd* dq_des  = MatNd_create(controller.getGraph()->dof, 1);
      MatNd* dq_ts   = MatNd_create(controller.getGraph()->dof, 1);
      MatNd* dq_ns   = MatNd_create(controller.getGraph()->dof, 1);
      MatNd* a_des   = MatNd_create(controller.getNumberOfTasks(), 1);
      MatNd* x_curr  = MatNd_create(controller.getTaskDim(), 1);
      MatNd* x_des   = MatNd_create(controller.getTaskDim(), 1);
      MatNd* dx_des  = MatNd_create(controller.getTaskDim(), 1);
      MatNd* dH      = MatNd_create(1, controller.getGraph()->nJ);

      controller.readActivationsFromXML(a_des);
      controller.computeX(x_curr);
      MatNd_copy(x_des, x_curr);

      // Create visualization
      Rcs::Viewer* v           = NULL;
      Rcs::KeyCatcher* kc      = NULL;
      Rcs::GraphNode* gn       = NULL;
      Rcs::HUD* hud            = NULL;
      Rcs::BodyPointDragger* dragger = NULL;
      char hudText[2056];

      if (valgrind==false)
      {
        v       = new Rcs::Viewer(!simpleGraphics, !simpleGraphics);
        kc      = new Rcs::KeyCatcher();
        gn      = new Rcs::GraphNode(controller.getGraph());
        hud     = new Rcs::HUD();
        dragger = new Rcs::BodyPointDragger();
        v->add(gn);
        v->add(hud);
        v->add(kc);
        v->add(dragger);
        v->runInThread(mtx);

        // Launch the activation widget
        std::vector<std::string> labels;
        Rcs::MatNdWidget* mw = Rcs::MatNdWidget::create(a_des, a_des,
                                                        0.0, 1.0, "activation",
                                                        &graphLock);
        for (size_t id=0; id<controller.getNumberOfTasks(); id++)
        {
          labels.push_back(controller.getTaskName(id));
        }
        mw->setLabels(labels);
      }



      // Endless loop
      while (runLoop == true)
      {
        pthread_mutex_lock(&graphLock);
        double dt = Timer_getTime();

        // Set state to random and compute null space cost and gradient
        if (loopCount>0 && loopCount%nIter==0)
        {
          RCSGRAPH_TRAVERSE_JOINTS(controller.getGraph())
          {
            controller.getGraph()->q->ele[JNT->jointIndex] =
              //Math_getRandomNumber(JNT->q_min, JNT->q_max);
              Math_getRandomNumber(JNT->q0-0.5*fabs(JNT->q0-JNT->q_min),
                                   JNT->q0+0.5*fabs(JNT->q_max-JNT->q0));
          }

        }

        RcsGraph_setState(controller.getGraph(), NULL, NULL);
        jlCost = controller.computeJointlimitCost();
        controller.computeJointlimitGradient(dH);
        MatNd_constMulSelf(dH, alpha);

        MatNd_setZero(dx_des);

        // Add task-space re-projection: dH^T J#
        if (!projJ)
        {
          double scaling = 1.0;// MatNd_getNorm(dq_ns);
          MatNd* invWq = MatNd_create(controller.getGraph()->dof, 1);
          RcsGraph_getInvWq(controller.getGraph(), invWq, RcsStateIK);
          MatNd_transposeSelf(invWq);
          MatNd_eleMulSelf(dH, invWq);

          MatNd* pinvJ = MatNd_create(controller.getGraph()->nJ,
                                      controller.getTaskDim());
          bool successPinv = ikSolver.computeRightInverse(pinvJ, a_des, lambda);
          RCHECK(successPinv);

          MatNd* dxProj = MatNd_create(1, controller.getTaskDim());
          MatNd_reshape(dxProj, 1, controller.getActiveTaskDim(a_des));
          MatNd_mul(dxProj, dH, pinvJ);
          MatNd_constMulSelf(dxProj, -scaling);
          MatNd_transposeSelf(dxProj);
          controller.decompressFromActiveSelf(dxProj, a_des);
          MatNd_addSelf(dx_des, dxProj);

          MatNd_destroy(dxProj);
          MatNd_destroy(pinvJ);
          MatNd_destroy(invWq);
        }   // End add task-space re-projection: dH J#
        else
        {
          // Add task-space re-projection: J dH^T
          double scaling = 1.0;// MatNd_getNorm(dq_ns);
          MatNd* J = MatNd_create(controller.getTaskDim(),
                                  controller.getGraph()->nJ);
          controller.computeJ(J, a_des);

          MatNd* dxProj = MatNd_create(1, controller.getTaskDim());
          MatNd_reshape(dxProj, controller.getActiveTaskDim(a_des), 1);
          MatNd_transposeSelf(dH);
          MatNd_mul(dxProj, J, dH);
          MatNd_constMulSelf(dxProj, -scaling);
          controller.decompressFromActiveSelf(dxProj, a_des);
          MatNd_addSelf(dx_des, dxProj);
          MatNd_transposeSelf(dH);

          MatNd_destroy(dxProj);
          MatNd_destroy(J);
        }
        // End add task-space re-projection: dH^T J#

        if (loopCount < 100)
        {
          MatNd_setZero(dx_des);
          MatNd_set(dx_des, 2, 0, 0.005);
        }

        double dtIK = Timer_getTime();
        det = ikSolver.solveRightInverse(dq_ts, dq_ns, dx_des, dH, a_des, lambda);
        MatNd_copy(dq_des, dq_ts);
        MatNd_addSelf(dq_des, dq_ns);

        dtIK = Timer_getTime() - dtIK;

        MatNd_addSelf(controller.getGraph()->q, dq_des);
        RcsGraph_setState(controller.getGraph(), NULL, NULL);
        controller.computeX(x_curr);
        dt = Timer_getTime() - dt;

        dJlCost = -jlCost;
        jlCost = controller.computeJointlimitCost();
        dJlCost += jlCost;

        REXEC(1)
        {
          VecNd_printComment("dx", dx_des->ele, 3);
          RLOG(1, "dJLCost: %f", dJlCost);
        }

        pthread_mutex_unlock(&graphLock);

        if (kc && kc->getAndResetKey('q'))
        {
          runLoop = false;
        }
        else if (kc && kc->getAndResetKey('t'))
        {
          RLOGS(0, "Running controller test");
          controller.test(true);
        }
        else if (kc && kc->getAndResetKey(' '))
        {
          pause = !pause;
          RMSG("Pause modus is %s", pause ? "ON" : "OFF");
        }
        else if (kc && kc->getAndResetKey('n'))
        {
          RMSG("Resetting");
          RcsGraph_setDefaultState(controller.getGraph());
        }

        sprintf(hudText, "%.1f %%: IK calculation: %.2f ms\ndof: %d nJ: %d "
                "nqr: %d nx: %zu\nJL-cost: %.6f dJL-cost: %.6f %s %s\n"
                "dt=%.2f uslambda:%g alpha: %g\nloopCount=%d",
                fmod(100.0*((double)loopCount/nIter), 100.0),
                1.0e3*dt, controller.getGraph()->dof,
                controller.getGraph()->nJ, ikSolver.getInternalDof(),
                controller.getActiveTaskDim(a_des),
                jlCost, dJlCost, det==0.0?"SINGULAR":"",
                ((dJlCost > eps) && (MatNd_getNorm(dx_des) == 0.0)) ?
                "COST INCREASE" : "",
                dtIK*1.0e6, lambda, alpha, loopCount);

        if (hud != NULL)
        {
          hud->setText(hudText);
        }
        else
        {
          REXEC(3)
          {
            std::cout << hudText;
          }
        }

        if ((valgrind==true) && (loopCount>2*nIter))
        {
          runLoop = false;
        }

        if (pause==true)
        {
          RPAUSE();
        }


        if ((dJlCost > eps) && (MatNd_getNorm(dx_des) == 0.0))
        {
          RLOG(2, "COST INCREASE: %g", dJlCost);
          RPAUSE_DL(3);
          result++;
        }

        loopCount++;

        if (!valgrind)
        {
          Timer_usleep(1);
        }

      }



      // Clean up
      if (valgrind==false)
      {
        delete v;
        RcsGuiFactory_shutdown();
      }

      MatNd_destroy(dq_des);
      MatNd_destroy(dq_ts);
      MatNd_destroy(dq_ns);
      MatNd_destroy(a_des);
      MatNd_destroy(x_curr);
      MatNd_destroy(x_des);
      MatNd_destroy(dx_des);
      MatNd_destroy(dH);

      RLOGS(1, "%s testing null space projections",
            (result==0) ? "SUCCESS" : "FAILURE");

      break;
    }


    // ==============================================================
    // Mujoco xml file converter test
    // ==============================================================
    case 13:
    {
      strcpy(xmlFileName, "config/xml/Examples/gHumanoidPendulum.xml");
      strcpy(directory, "config/xml/Examples");
      argP.getArgument("-f", xmlFileName, "Configuration file name (default "
                       "is \"%s\")", xmlFileName);
      argP.getArgument("-dir", directory, "Configuration file directory "
                       "(default is \"%s\")", directory);
      Rcs_addResourcePath(directory);

      RcsGraph* graph = RcsGraph_create(xmlFileName);
      bool success = RcsGraph_toMujocoFile("mujoco.xml", graph);
      RMSG("%s converting graph to mujoco.xml", success ? "Success" : "Failure");
      break;
    }


    // ==============================================================
    // Joint weighting per task IK
    // ==============================================================
    case 14:
    {
      strcpy(xmlFileName, "cJaco7.xml");
      strcpy(directory, "config/xml/Kinova");
      argP.getArgument("-f", xmlFileName, "Configuration file name (default "
                       "is \"%s\")", xmlFileName);
      argP.getArgument("-dir", directory, "Configuration file directory "
                       "(default is \"%s\")", directory);
      Rcs_addResourcePath(directory);

      Rcs::ControllerBase controller(xmlFileName);
      Rcs::Viewer viewer;
      viewer.add(new Rcs::GraphNode(controller.getGraph()));
      viewer.runInThread();

      const double dt = 0.01;
      const unsigned int nq = controller.getGraph()->nJ;
      const Rcs::Task* xyz = controller.getTask(0);
      const Rcs::Task* abc = controller.getTask(1);

      MatNd* dx = MatNd_create(6, 1);
      MatNd* x = MatNd_create(6, 1);
      MatNd* dq = MatNd_create(2*nq, 1);

      Rcs::MatNdGui dxGui(dx, x, -1.0, 1.0, "dx");

      while (runLoop)
      {

        // Weight matrices: Distal joints contribute stronger to orientations
        MatNd* Wq1 = MatNd_create(nq, 1);
        MatNd* Wq2 = MatNd_create(nq, 1);
        MatNd_setElementsTo(Wq1, 1.0);
        MatNd_setElementsTo(Wq2, 1.0);
        VecNd_setElementsTo(Wq1->ele, 1.0, 4);
        VecNd_setElementsTo(&Wq1->ele[4], 0.1, 3);
        VecNd_setElementsTo(Wq2->ele, 0.1, 4);
        VecNd_setElementsTo(&Wq1->ele[4], 1.0, 3);

        // Compute weighted Jacobians
        MatNd* J1 = MatNd_create(xyz->getDim(), nq);
        MatNd* J2 = MatNd_create(abc->getDim(), nq);
        xyz->computeJ(J1);
        abc->computeJ(J2);
        MatNd_postMulDiagSelf(J1, Wq1);
        MatNd_postMulDiagSelf(J2, Wq2);
        MatNd* JTJ1 = MatNd_create(nq, nq);
        MatNd* JTJ2 = MatNd_create(nq, nq);
        MatNd_sqrMulAtBA(JTJ1, J1, NULL);
        MatNd_sqrMulAtBA(JTJ2, J2, NULL);
        MatNd_addConstToDiag(JTJ1, 1.0);
        MatNd_addConstToDiag(JTJ2, -1.0);

        // The big matrix A
        MatNd* A = MatNd_create(2 * nq, 2 * nq);
        for (unsigned int i = 0; i < nq; ++i)
        {
          for (unsigned int j = 0; j < nq; ++j)
          {
            // Upper left
            double* dst = MatNd_getElePtr(A, i, j);
            *dst = MatNd_get(JTJ1, i, j);

            // Upper right
            dst = MatNd_getElePtr(A, i, j + nq);
            *dst = (i == j) ? -1.0 : 0.0;

            // Lower left
            dst = MatNd_getElePtr(A, i+nq, j);
            *dst = (i == j) ? 1.0 : 0.0;

            // Lower right
            dst = MatNd_getElePtr(A, i + nq, j+nq);
            *dst = MatNd_get(JTJ2, i, j);
          }
        }

        // The RHS vector b
        MatNd_transposeSelf(J1);
        MatNd_transposeSelf(J2);
        MatNd dxPos = MatNd_fromPtr(3, 1, dx->ele);
        MatNd dxOri = MatNd_fromPtr(3, 1, dx->ele+3);
        MatNd* b = MatNd_create(2 * nq, 1);
        MatNd bPos = MatNd_fromPtr(nq, 1, b->ele);
        MatNd bOri = MatNd_fromPtr(nq, 1, b->ele+nq);
        MatNd_mul(&bPos, J1, &dxPos);
        MatNd_mul(&bOri, J2, &dxOri);
        MatNd_constMulSelf(b, dt);

        // Solve it
        MatNd_addConstToDiag(A, 1.0e-8);
        double det = MatNd_gaussInverse(A, A);
        RLOG(1, "Determinant is %f", det);
        MatNd_mul(dq, A, b);

        MatNd dq1 = MatNd_fromPtr(nq, 1, dq->ele);
        MatNd dq2 = MatNd_fromPtr(nq, 1, dq->ele + nq);

        REXEC(2)
        {
          RLOG(2, "dq1    dq2");
          MatNd_printTwoArraysDiff(&dq1, &dq2, 5);
        }

        MatNd* dqGraph = MatNd_createLike(controller.getGraph()->q);
        MatNd_reshapeCopy(dqGraph, &dq1);
        RcsGraph_stateVectorFromIKSelf(controller.getGraph(), dqGraph);
        MatNd_addSelf(controller.getGraph()->q, dqGraph);
        RcsGraph_setState(controller.getGraph(), NULL, NULL);
        controller.computeX(x);

        // Clean up
        MatNd_destroyN(9, Wq1, Wq2, J1, J2, JTJ1, JTJ2, A, b, dqGraph);

        Timer_waitDT(dt);
      }
      viewer.stopUpdateThread();

      MatNd_destroyN(3, dx, x, dq);

      break;
    }



    // ==============================================================
    // Joint weighting per task IK - downprojected
    // ==============================================================
    case 15:
    {
      strcpy(xmlFileName, "cJaco7.xml");
      strcpy(directory, "config/xml/Kinova");
      argP.getArgument("-f", xmlFileName, "Configuration file name (default "
                       "is \"%s\")", xmlFileName);
      argP.getArgument("-dir", directory, "Configuration file directory "
                       "(default is \"%s\")", directory);
      Rcs_addResourcePath(directory);

      Rcs::ControllerBase controller(xmlFileName);
      Rcs::Viewer viewer;
      viewer.add(new Rcs::GraphNode(controller.getGraph()));
      viewer.runInThread();

      const double dt = 0.01;
      const unsigned int nq = controller.getGraph()->nJ;
      const Rcs::Task* xyz = controller.getTask(0);
      const Rcs::Task* abc = controller.getTask(1);

      MatNd* dx = MatNd_create(6, 1);
      MatNd* x = MatNd_create(6, 1);
      MatNd* dq = MatNd_create(2 * nq, 1);

      Rcs::MatNdGui dxGui(dx, x, -1.0, 1.0, "dx");

      while (runLoop)
      {

        // Weight matrices: Distal joints contribute stronger to orientations
        MatNd* Wq1 = MatNd_create(nq, 1);
        MatNd* Wq2 = MatNd_create(nq, 1);
        MatNd_setElementsTo(Wq1, 1.0);
        MatNd_setElementsTo(Wq2, 1.0);
        VecNd_setElementsTo(Wq1->ele, 1.0, 4);
        VecNd_setElementsTo(&Wq1->ele[4], 0.1, 3);
        VecNd_setElementsTo(Wq2->ele, 0.1, 4);
        VecNd_setElementsTo(&Wq1->ele[4], 1.0, 3);

        // Compute weighted Jacobians
        MatNd* J1 = MatNd_create(xyz->getDim(), nq);
        MatNd* J2 = MatNd_create(abc->getDim(), nq);
        xyz->computeJ(J1);
        abc->computeJ(J2);
        MatNd_postMulDiagSelf(J1, Wq1);
        MatNd_postMulDiagSelf(J2, Wq2);
        MatNd* JTJ1 = MatNd_create(nq, nq);
        MatNd* JTJ2 = MatNd_create(nq, nq);
        MatNd_sqrMulAtBA(JTJ1, J1, NULL);
        MatNd_sqrMulAtBA(JTJ2, J2, NULL);
        MatNd_addConstToDiag(JTJ1, 1.0);
        MatNd_addConstToDiag(JTJ2, -1.0);
        MatNd_preMulSelf(JTJ1, J1);
        MatNd_preMulSelf(JTJ2, J2);

        // The big matrix A
        MatNd* A = MatNd_create(6, 2 * nq);
        for (unsigned int col = 0; col < nq; ++col)
        {
          for (unsigned int row = 0; row < 3; ++row)
          {
            // Upper left
            double* dst = MatNd_getElePtr(A, row, col);
            *dst = MatNd_get(JTJ1, row, col);

            // Upper right
            dst = MatNd_getElePtr(A, row, col + nq);
            *dst = -MatNd_get(J1, row, col);

            // Lower left
            dst = MatNd_getElePtr(A, row+3, col);
            *dst = MatNd_get(J2, row, col);

            // Lower right
            dst = MatNd_getElePtr(A, row + 3, col + nq);
            *dst = MatNd_get(JTJ2, row, col);
          }
        }

        // The RHS vector b
        MatNd* JT1 = MatNd_create(J1->n, J1->m);
        MatNd* JT2 = MatNd_create(J2->n, J2->m);
        MatNd_transpose(JT1, J1);
        MatNd_transpose(JT2, J2);
        MatNd* JJT1 = MatNd_create(3, 3);
        MatNd* JJT2 = MatNd_create(3, 3);
        MatNd_mul(JJT1, J1, JT1);
        MatNd_mul(JJT2, J2, JT2);
        MatNd dxPos = MatNd_fromPtr(3, 1, dx->ele);
        MatNd dxOri = MatNd_fromPtr(3, 1, dx->ele + 3);
        MatNd* b = MatNd_create(6, 1);
        MatNd bPos = MatNd_fromPtr(3, 1, b->ele);
        MatNd bOri = MatNd_fromPtr(3, 1, b->ele + 3);
        MatNd_mul(&bPos, JJT1, &dxPos);
        MatNd_mul(&bOri, JJT2, &dxOri);

        // Solve it
        MatNd* buf = MatNd_clone(A);
        double det = MatNd_rwPinv(A, buf, NULL, NULL);
        MatNd_destroy(buf);
        RLOG(1, "Determinant is %f", det);
        MatNd_mul(dq, A, b);

        MatNd dq1 = MatNd_fromPtr(nq, 1, dq->ele);
        MatNd dq2 = MatNd_fromPtr(nq, 1, dq->ele + nq);

        REXEC(2)
        {
          RLOG(2, "dq1    dq2");
          MatNd_printTwoArraysDiff(&dq1, &dq2, 5);
        }

        MatNd_constMulSelf(dq, dt);

        MatNd* dqGraph = MatNd_createLike(controller.getGraph()->q);
        MatNd_reshapeCopy(dqGraph, &dq1);
        RcsGraph_stateVectorFromIKSelf(controller.getGraph(), dqGraph);
        MatNd_addSelf(controller.getGraph()->q, dqGraph);
        RcsGraph_setState(controller.getGraph(), NULL, NULL);
        controller.computeX(x);

        // Clean up
        MatNd_destroyN(13, Wq1, Wq2, J1, J2, JTJ1, JTJ2, JT1, JT2, JJT1, JJT2, A, b, dqGraph);

        Timer_waitDT(dt);
      }
      viewer.stopUpdateThread();

      MatNd_destroyN(3, dx, x, dq);

      break;
    }

    // ==============================================================
    // Joint weighting per task IK
    // ==============================================================
    case 16:
    {
      strcpy(xmlFileName, "cJaco7.xml");
      strcpy(directory, "config/xml/Kinova");
      argP.getArgument("-f", xmlFileName, "Configuration file name (default "
                       "is \"%s\")", xmlFileName);
      argP.getArgument("-dir", directory, "Configuration file directory "
                       "(default is \"%s\")", directory);
      Rcs_addResourcePath(directory);

      Rcs::ControllerBase controller(xmlFileName);
      Rcs::Viewer viewer;
      viewer.add(new Rcs::GraphNode(controller.getGraph()));
      viewer.runInThread();

      const double dt = 0.01;
      const unsigned int nq = controller.getGraph()->nJ;
      const Rcs::Task* xyz = controller.getTask(0);
      const Rcs::Task* abc = controller.getTask(1);

      MatNd* dx = MatNd_create(6+nq, 1);
      MatNd* x = MatNd_create(6, 1);
      MatNd* dq = MatNd_create(2*nq, 1);
      MatNd dx_gui = MatNd_fromPtr(6, 1, dx->ele);

      Rcs::MatNdGui dxGui(&dx_gui, x, -1.0, 1.0, "dx");

      while (runLoop)
      {

        // Weight matrices: Distal joints contribute stronger to orientations
        MatNd* Wq1 = MatNd_create(nq, 1);
        MatNd* Wq2 = MatNd_create(nq, 1);
        MatNd_setElementsTo(Wq1, 1.0);
        MatNd_setElementsTo(Wq2, 1.0);
        VecNd_setElementsTo(Wq1->ele, 1.0, 4);
        VecNd_setElementsTo(&Wq1->ele[4], 0.1, 3);
        VecNd_setElementsTo(Wq2->ele, 0.1, 4);
        VecNd_setElementsTo(&Wq1->ele[4], 1.0, 3);

        // Compute weighted Jacobians
        RCHECK(xyz->getDim() == 3);
        RCHECK(abc->getDim() == 3);
        MatNd* J1 = MatNd_create(xyz->getDim(), nq);
        MatNd* J2 = MatNd_create(abc->getDim(), nq);
        xyz->computeJ(J1);
        abc->computeJ(J2);
        MatNd_postMulDiagSelf(J1, Wq1);
        MatNd_postMulDiagSelf(J2, Wq2);

        // The big matrix A
        MatNd* A = MatNd_create(6+nq, 2*nq);

        // Jacobian J1 top left, J2 below top right
        for (unsigned int row = 0; row < 3; ++row)
        {
          for (unsigned int col = 0; col < nq; ++col)
          {
            MatNd_set(A, row, col, MatNd_get(J1, row, col));
            MatNd_set(A, row+3, col+nq, MatNd_get(J2, row, col));
          }
        }

        // Identities
        for (unsigned int i = 0; i < nq; ++i)
        {
          MatNd_set(A, i + 6, i, 1.0);
          MatNd_set(A, i + 6, i + nq, -1.0);
          //MatNd_set(A, i + 6, i, Wq1->ele[i]);
          //MatNd_set(A, i + 6, i + nq, -Wq2->ele[i]);
        }

        // IK
        MatNd* invA = MatNd_create(A->n, A->m);
        double det = MatNd_rwPinv(invA, A, NULL, NULL);
        RCHECK(det>0.0);
        MatNd_mul(dq, invA, dx);

        MatNd dq1 = MatNd_fromPtr(nq, 1, dq->ele);
        MatNd dq2 = MatNd_fromPtr(nq, 1, dq->ele + nq);

        REXEC(3)
        {
          RLOG(3, "A");
          MatNd_printCommentDigits("A", A, 4);
        }

        REXEC(2)
        {
          RLOG(2, "dq1    dq2");
          MatNd_printTwoArraysDiff(&dq1, &dq2, 5);
        }

        REXEC(1)
        {
          MatNd* dx_test = MatNd_create(3, 1);
          MatNd_mul(dx_test, J1, &dq1);
          MatNd dx_act = MatNd_fromPtr(3, 1, dx->ele);
          //MatNd_subSelf(dx_test, &dx_act);
          //MatNd_printCommentDigits("dx_err_pos", dx_test, 16);
          RLOG(2, "pos");
          MatNd_printTwoArraysDiff(dx_test, &dx_act, 12);

          MatNd_mul(dx_test, J2, &dq1);
          dx_act = MatNd_fromPtr(3, 1, dx->ele+3);
          //MatNd_subSelf(dx_test, &dx_act);
          //MatNd_printCommentDigits("dx_err_ori", dx_test, 16);
          RLOG(2, "ori");
          MatNd_printTwoArraysDiff(dx_test, &dx_act, 12);

          MatNd_destroy(dx_test);
        }

        MatNd* dqGraph = MatNd_createLike(controller.getGraph()->q);
        MatNd_reshapeCopy(dqGraph, &dq1);
        MatNd_constMulSelf(dqGraph, dt);
        RcsGraph_stateVectorFromIKSelf(controller.getGraph(), dqGraph);
        MatNd_addSelf(controller.getGraph()->q, dqGraph);
        RcsGraph_setState(controller.getGraph(), NULL, NULL);
        controller.computeX(x);

        // Clean up
        MatNd_destroyN(7, Wq1, Wq2, J1, J2, A, invA, dqGraph);

        Timer_waitDT(dt);
      }
      viewer.stopUpdateThread();

      MatNd_destroyN(3, dx, x, dq);

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
    Rcs_printResourcePath();
    Rcs::KeyCatcherBase::printRegisteredKeys();
    argP.print();
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

  if (!valgrind)
  {
    RLOG(0, "Thanks for using the Rcs libraries\n");
  }

  return Math_iClip(result, 0, 255);
}
