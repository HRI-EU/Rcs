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
#include <Rcs_geometry.h>
#include <Rcs_gradientTests.h>
#include <Rcs_resourcePath.h>
#include <Rcs_BVHParser.h>
#include <Rcs_timer.h>
#include <Rcs_sensor.h>
#include <Rcs_typedef.h>
#include <Rcs_graphParser.h>
#include <Rcs_kinematics.h>
#include <Rcs_dynamics.h>
#include <Rcs_joint.h>
#include <Rcs_body.h>
#include <Rcs_shape.h>
#include <Rcs_utils.h>
#include <Rcs_utilsCPP.h>
#include <Rcs_filters.h>
#include <IkSolverConstraintRMR.h>
#include <SolverRAC.h>
#include <TaskFactory.h>
#include <TaskRegionFactory.h>
#include <PhysicsFactory.h>
#include <KineticSimulation.h>
#include <PhysicsNode.h>
#include <GraphNode.h>
#include <SphereNode.h>
#include <HUD.h>
#include <VertexArrayNode.h>
#include <PPSSensorNode.h>
#include <KeyCatcher.h>
#include <JointWidget.h>
#include <RcsViewer.h>
#include <Rcs_graphicsUtils.h>
#include <Rcs_guiFactory.h>
#include <BodyPointDragger.h>
#include <ControllerWidgetBase.h>
#include <MatNdWidget.h>
#include <TargetSetter.h>
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

/*******************************************************************************
 * A few pre-defined models
 ******************************************************************************/
static bool getModel(char* directory, char* xmlFileName)
{
  Rcs::CmdLineParser argP;

  if (!argP.hasArgument("-model",
                        "Example models: Husky, DexBot, WAM, Humanoid"))
  {
    return false;
  }

  char model[64] = "";
  argP.getArgument("-model", model);

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
  argP.getArgument("-m", &mode, "Test mode (default is 0)");
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

      RMSG("Writing graph to dot file \"RcsGraph.dot\"");
      RcsGraph_writeDotFile(graph, "RcsGraph.dot");
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
      Rcs::KeyCatcherBase::registerKey("a", "Change body attachement");
      Rcs::KeyCatcherBase::registerKey("C", "Toggle COM display");
      Rcs::KeyCatcherBase::registerKey("d", "Print dot file");
      Rcs::KeyCatcherBase::registerKey("f", "Write graph to file");
      Rcs::KeyCatcherBase::registerKey("j", "Create JointWidget");
      Rcs::KeyCatcherBase::registerKey("l", "Reload graph file");
      Rcs::KeyCatcherBase::registerKey("m", "Set q to model state");
      Rcs::KeyCatcherBase::registerKey("p", "Print information to console");
      Rcs::KeyCatcherBase::registerKey("q", "Quit");
      Rcs::KeyCatcherBase::registerKey("W", "Merge bodies");
      Rcs::KeyCatcherBase::registerKey("x", "Rewind bvh file");
      Rcs::KeyCatcherBase::registerKey("e", "Remove body under mouse");
      Rcs::KeyCatcherBase::registerKey("J", "Remove joints of body by name");
      Rcs::KeyCatcherBase::registerKey("E", "Remove all joints of graph");
      Rcs::KeyCatcherBase::registerKey("T", "Load trajectory file");
      Rcs::KeyCatcherBase::registerKey("X", "Make monolithic");
      Rcs::KeyCatcherBase::registerKey("S", "Scale graph");
      Rcs::KeyCatcherBase::registerKey("b", "Boxify graph");
      Rcs::KeyCatcherBase::registerKey("B", "Capsulify graph");
      Rcs::KeyCatcherBase::registerKey("H", "Toggle HUD");

      double dtSim = 0.0, dtStep = 0.04;
      int fwdKinType = 0;
      char hudText[512] = "", comRef[64] = "";
      char dotFile[256] = "RcsGraph.dot";
      char bgColor[64] = "LIGHT_GRAYISH_GREEN";
      std::string fKinBdyName;
      strcpy(xmlFileName, "gScenario.xml");
      strcpy(directory, "config/xml/DexBot");
      getModel(directory, xmlFileName);

      argP.getArgument("-dt", &dtStep, "Animation time step (default is %f)",
                       dtStep);
      argP.getArgument("-dotFile", dotFile, "Dot file name");
      argP.getArgument("-f", xmlFileName, "Configuration file name (default"
                       " is \"%s\")", xmlFileName);
      argP.getArgument("-dir", directory, "Configuration file directory "
                       "(default is \"%s\")", directory);
      argP.getArgument("-comRef", comRef, "Reference body for COM (default is "
                       "root)");
      argP.getArgument("-bgColor", bgColor, "Background color (default is "
                       "\"%s\")", bgColor);
      argP.getArgument("-fKin", &fwdKinType, "Forward kinematics: 0: all dof,"
                       "1: sub tree, 2: body only (default is %d)", fwdKinType);
      argP.getArgument("-fKinBdy", &fKinBdyName, "Forward kinematics start "
                       "body (default is none)");
      bool testCopy = argP.hasArgument("-copy", "Test graph copying");
      bool resizeable = argP.hasArgument("-resizeable", "Adjust visualization "
                                         "of shapes dynamically");
      bool editMode = argP.hasArgument("-edit", "Start in xml edit mode "
                                       "(no Qt Gui)");
      bool playBVH = argP.hasArgument("-bvh", "Play bvh file");
      bool noHud = argP.hasArgument("-noHud", "Don't show HUD");
      bool randomGraph = argP.hasArgument("-random", "Create randomized graph");
      bool shapifyReplace = argP.hasArgument("-shapifyReplace", "True: Replace "
                                             "shapes when using boxify / capsul"
                                             "ify, default: add enclosing box /"
                                             " capsule as additional shape"
                                             " to bodies");

      Rcs_addResourcePath(directory);

      if (argP.hasArgument("-h"))
      {
        RMSG("Mode %d: Rcs -m %d -dir <graph-directory> -f "
             "<graph-file>\n\n\t- Creates a graph from an xml file\n\t"
             "- Creates a viewer (if option -valgrind is not set)\n\t"
             "- Creates a JointWidget (if option -valgrind is not set)\n\t"
             "- Runs the forward kinematics in a loop\n\n\t"
             "The joints angles can be modified by the sliders\n",
             mode, mode);
        printf("\n\tForward kinematics:\n");
        printf("\t-dir\t<directory=%s>   Configuration file directory\n",
               directory);
        printf("\t-f\t<configFile=%s>   Configuration file\n",
               xmlFileName);
        printf("\t-edit\t   Don't launch JointWidget. Otherwise, reloading"
               " doesn't work with the widget being active.\n");
        printf("\n");
        printf("\n");
        printf("\tExamples: Rcs -m 2\n");
        printf("\t-dir config/xml/LWR -f lbr_iiwa_14_r820.urdf\n");
        printf("\t-dir config/xml/DarwinOP -f robotis_op.urdf\n");
        printf("\t-dir config/xml/Husky -f dual_arm_husky_original.urdf\n");
        printf("\t-dir config/xml/Valkyrie -f valkyrie_sim.urdf\n");
        printf("\n");
        break;
      }

      RcsGraph* graph = NULL;

      if (randomGraph)
      {
        graph = RcsGraph_createRandom(30, 5);
      }
      else
      {
        graph = RcsGraph_create(xmlFileName);
      }

      REXEC(5)
      {
        RLOG(0, "Original traversal:");
        int count1 = 0;
        RCSGRAPH_TRAVERSE_BODIES(graph)
        {
          printf("%d: %s - %d: %s\n", count1, BODY->name,
                 graph->bodies[count1].id, graph->bodies[count1].name);
          RcsBody* body = BODY;
          RLOG(0, "name=%s id=%d parent=%d prev=%d next=%d first=%d last=%d",
               body->name, body->id, body->parentId, body->prevId, body->nextId,
               body->firstChildId, body->lastChildId);
          count1++;
        }
        RLOG(0, "Done original traversal");

        RLOG(0, "New traversal:");
        count1 = 0;
        for (unsigned int i=0; i<graph->nBodies; ++i)
        {
          printf("%d: %s\n", i, graph->bodies[i].name);
        }

        RLOG(0, "Newest traversal:");
        count1 = 0;
        RCSGRAPH_FOREACH_BODY(graph)
        {
          printf("%d: %s\n", count1++, BODY->name);
        }
      }

      if (graph == NULL)
      {
        RMSG("Failed to create graph from file \"%s\" - exiting", xmlFileName);
        break;
      }

      char bvhFile[256];
      strcpy(bvhFile, graph->cfgFile);
      argP.getArgument("-bvhFile", bvhFile, "BVH file "
                       "(default is \"%s\")", bvhFile);

      MatNd* bvhTraj = NULL;
      unsigned int bvhIdx = 0;
      if (playBVH && String_hasEnding(bvhFile, ".bvh", false))
      {
        bvhTraj = RcsGraph_createTrajectoryFromBVHFile(graph, bvhFile,
                                                       &dtStep, 0.01,
                                                       M_PI/180.0);

        if (bvhTraj && (bvhTraj->n!=graph->dof))
        {
          RLOG(1, "Mismatch in bvh array dimensions: found %d columns, but "
               "graph has %d dof", bvhTraj->n, graph->dof);
          MatNd_destroy(bvhTraj);
          bvhTraj = NULL;
        }
        else if (!bvhTraj)
        {
          bvhTraj = MatNd_createFromFile(bvhFile);
        }
      }

      if (testCopy==true)
      {
        RcsGraph* graph2 = graph;
        double t_copy = Timer_getSystemTime();
        graph = RcsGraph_clone(graph2);
        t_copy = Timer_getSystemTime() - t_copy;
        RMSG("Cloning graph took %.3f msec", t_copy*1.0e3);
        t_copy = Timer_getSystemTime();
        RcsGraph_copy(graph, graph2);
        t_copy = Timer_getSystemTime() - t_copy;
        RMSG("Copying graph took %.3f msec", t_copy*1.0e3);
        t_copy = Timer_getSystemTime();
        RcsGraph_setState(graph2, NULL, NULL);
        t_copy = Timer_getSystemTime() - t_copy;
        RMSG("Forward kinematics took %.3f msec", t_copy*1.0e3);
        RcsGraph_destroy(graph2);
      }

      if (testCopy==true)
      {
        const int nIter = 1000;
        RcsGraph* graph2 = RcsGraph_clone(graph);

        double t1 = Timer_getTime();
        for (int i=0; i<nIter; ++i)
        {
          RcsGraph_copy(graph2, graph);
        }
        t1 = Timer_getTime() - t1;


        double t2 = Timer_getTime();
        for (int i=0; i<nIter; ++i)
        {
          RcsGraph_setState(graph2, graph->q, graph->q_dot);
        }
        t2 = Timer_getTime() - t2;

        RLOG(0, "Copying took %f msec  forward kinematics took %f msec",
             1000.0*t1/nIter, 1000.0*t2/nIter);

        RcsGraph_destroy(graph2);
      }

      const RcsBody* comBase = RcsGraph_getBodyByName(graph, comRef);

      int guiHandle = -1;
      unsigned int loopCount = 0;
      double mass = 0.0, Id[3][3], r_com[3];
      Mat3d_setIdentity(Id);
      Vec3d_setZero(r_com);

      Rcs::KeyCatcher* kc = NULL;
      Rcs::GraphNode* gn  = NULL;
      Rcs::SphereNode* comNd = NULL;
      Rcs::HUD* hud = NULL;
      Rcs::Viewer* viewer = NULL;

      if (!valgrind)
      {
        viewer = new Rcs::Viewer(!simpleGraphics, !simpleGraphics);
        viewer->setBackgroundColor(bgColor);
        gn = new Rcs::GraphNode(graph, resizeable);
        gn->toggleReferenceFrames();
        viewer->add(gn);

        comNd = new Rcs::SphereNode(r_com, 0.05);
        comNd->makeDynamic(r_com);
        comNd->setMaterial("RED");
        comNd->toggleWireframe();
        comNd->hide();
        viewer->add(comNd);

        char xmlFile2[64] = "";
        argP.getArgument("-f2", xmlFile2, "Optional second graph file (default"
                         "is empty)");
        if (strlen(xmlFile2) > 0)
        {
          RcsGraph* graph2 = RcsGraph_create(xmlFile2);
          RCHECK(graph2);
          viewer->add(new Rcs::GraphNode(graph2, resizeable));
        }

        if (!noHud)
        {
          hud = new Rcs::HUD();
          viewer->add(hud);
        }

        kc = new Rcs::KeyCatcher();
        viewer->add(kc);

        RCSGRAPH_FOREACH_SENSOR(graph)
        {
          if (SENSOR->type==RCSSENSOR_PPS)
          {
            bool debug = RcsLogLevel > 0 ? true : false;
            viewer->add(new Rcs::PPSSensorNode(SENSOR, graph, debug));
          }
        }

        viewer->runInThread(mtx);
        if ((editMode==false) && (bvhTraj==NULL))
        {
          guiHandle = Rcs::JointWidget::create(graph, mtx);

          // We constrain all joints here so that we can drag them conveniently
          // with the sliders.
          RCSGRAPH_TRAVERSE_JOINTS(graph)
          {
            JNT->constrained = true;
          }

        }
      }

      REXEC(5)
      {
        RPAUSE_MSG("Hit enter to start kinematics computation loop");
      }



      while (runLoop)
      {
        pthread_mutex_lock(&graphLock);

        if (valgrind)
        {
          RLOG(1, "Starting step");
        }

        if (graph != NULL)
        {
          if (bvhTraj!=NULL)
          {
            MatNd row = MatNd_getRowViewTranspose(bvhTraj, bvhIdx);
            MatNd_copy(graph->q, &row);

            bvhIdx++;
            if (bvhIdx >= bvhTraj->m)
            {
              bvhIdx = 0;
            }
          }

          dtSim = Timer_getSystemTime();

          switch (fwdKinType)
          {
            case 0:
              RcsGraph_setState(graph, NULL, NULL);
              break;

            case 1:
            {
              RcsBody* fkBdy = RcsGraph_getBodyByName(graph, fKinBdyName.c_str());
              RcsGraph_computeBodyKinematics(graph, fkBdy, NULL, NULL, true);
            }
            break;

            case 2:
            {
              RcsBody* fkBdy = RcsGraph_getBodyByName(graph, fKinBdyName.c_str());
              RcsGraph_computeBodyKinematics(graph, fkBdy, NULL, NULL, false);
            }
            break;

            default:
              RFATAL("No forward kinematics mode %d", fwdKinType);
          }

          dtSim = Timer_getSystemTime() - dtSim;
          if (comBase != NULL)
          {
            mass = RcsGraph_COG_Body(graph, comBase, r_com);
          }
          else
          {
            mass = RcsGraph_COG(graph, r_com);
          }
        }

        if (valgrind)
        {
          RLOG(1, "Finished step");

          if (valgrind==true && loopCount>10)
          {
            runLoop = false;
          }
        }

        pthread_mutex_unlock(&graphLock);

        if (kc != NULL)
        {
          if (kc->getAndResetKey('q'))
          {
            runLoop = false;
          }
          else if (kc->getAndResetKey('H'))
          {
            if (hud)
            {
              hud->toggle();
            }
          }
          else if (kc->getAndResetKey('S'))
          {
            RMSG("Changing scale factor");
            printf("Enter scaling factor: ");
            double scaleFactor;
            std::cin >> scaleFactor;

            bool collisionVisible = gn->collisionModelVisible();
            bool graphicsVisible = gn->graphicsModelVisible();
            bool physicsVisible = gn->physicsModelVisible();
            bool framesVisible = gn->referenceFramesVisible();
            bool ghostVisible = gn->getGhostMode();
            bool wireframeVisible = gn->getWireframe();
            viewer->removeInternal(gn);

            pthread_mutex_lock(&graphLock);
            RcsGraph_scale(graph, scaleFactor);
            gn = new Rcs::GraphNode(graph);
            gn->toggleReferenceFrames();
            gn->displayGraphicsModel(graphicsVisible);
            gn->displayPhysicsModel(physicsVisible);
            gn->displayCollisionModel(collisionVisible);
            gn->displayReferenceFrames(framesVisible);
            gn->setGhostMode(ghostVisible);
            gn->showWireframe(wireframeVisible);
            pthread_mutex_unlock(&graphLock);
            viewer->add(gn);
          }
          else if (kc->getAndResetKey('m'))
          {
            std::string mdlState;
            RMSG("Changing model state. Options are:\n");
            std::vector<std::string> ms = RcsGraph_getModelStateNames(graph);
            for (size_t i=0; i<ms.size(); ++i)
            {
              printf("\t%s\n", ms[i].c_str());
            }
            printf("\nEnter name of model state: ");
            std::cin >> mdlState;
            pthread_mutex_lock(&graphLock);
            bool ok = RcsGraph_setModelStateFromXML(graph, mdlState.c_str(), 0);
            pthread_mutex_unlock(&graphLock);
            RMSG("%s changing model state to %s",
                 ok ? "SUCCEEDED" : "FAILED", mdlState.c_str());
          }
          else if (kc->getAndResetKey('j'))
          {
            RMSGS("Creating JointWidget");
            guiHandle = Rcs::JointWidget::create(graph, &graphLock);
          }
          else if (kc->getAndResetKey('e'))
          {
            Rcs::BodyNode* bNd =
              viewer->getBodyNodeUnderMouse<Rcs::BodyNode*>();
            if (bNd==NULL)
            {
              RMSG("No BodyNode found under mouse");
              continue;
            }
            RMSG("Removing body \"%s\" under mouse", bNd->body()->name);
            pthread_mutex_lock(&graphLock);
            bool ok = RcsGraph_removeBody(graph, bNd->body()->name, NULL, 0);
            if (ok)
            {
              ok = gn->removeBodyNode(bNd);
            }
            pthread_mutex_unlock(&graphLock);
            RMSG("%s removing body", ok ? "SUCCEESS" : "FAILURE");
          }
          else if (kc->getAndResetKey('J'))
          {
            std::string bodyName;
            RMSG("Removing all joints of a body");
            printf("Enter body name: ");
            std::cin >> bodyName;
            RcsBody* bdy = RcsGraph_getBodyByName(graph, bodyName.c_str());
            bool success = RcsBody_removeJoints(bdy, graph);
            RMSG("Removing all joints of a body %s %s", bodyName.c_str(),
                 success ? "SUCCEEDED" : "FAILED");
          }
          else if (kc->getAndResetKey('E'))
          {
            RMSG("Removing all joints of graph");
            pthread_mutex_lock(&graphLock);
            RCSGRAPH_TRAVERSE_BODIES(graph)
            {
              bool success = RcsBody_removeJoints(BODY, graph);
              RLOG(1, "Removing all joints of a body %s %s", BODY->name,
                   success ? "SUCCEEDED" : "FAILED");
            }
            pthread_mutex_unlock(&graphLock);
          }
          else if (kc->getAndResetKey('C'))
          {
            RMSGS("Toggling COM");
            comNd->toggle();
          }
          else if (kc->getAndResetKey('f'))
          {
            RMSG("Writing graph to xml file");
            FILE* out = fopen("graph.txt", "w+");
            RcsGraph_fprint(out, graph);
            fclose(out);
            out = fopen("graph.xml", "w+");
            RcsGraph_fprintXML(out, graph);
            fclose(out);
          }
          else if (kc->getAndResetKey('d'))
          {
            RMSGS("Writing dot file");
            RcsGraph_writeDotFile(graph, dotFile);
            char osCmd[256];
            sprintf(osCmd, "dotty %s&", dotFile);
            int err = system(osCmd);

            if (err == -1)
            {
              RMSG("Couldn't start dot file viewer!");
            }
          }
          else if (kc->getAndResetKey('a'))
          {
            std::string parentName, childName;
            RMSG("Changing body attachement");
            printf("Enter body to attach (child): ");
            std::cin >> childName;
            printf("Enter attachement (parent) body: ");
            std::cin >> parentName;

            RcsBody* child = RcsGraph_getBodyByName(graph, childName.c_str());
            RcsBody* parent = RcsGraph_getBodyByName(graph, parentName.c_str());
            RLOG(0, "Attaching \"%s\" (%s) to \"%s\" (%s)",
                 child ? child->name : "NULL", childName.c_str(),
                 parent ? parent->name : "NULL", parentName.c_str());

            bool success = RcsBody_attachToBodyId(graph, child?child->id:-1, parent?parent->id:-1);
            RMSG("%s changing attachement", success ? "SUCCESS" : "FAILURE");

            RcsGraph_fprintJointRecursion(stdout, graph, parentName.c_str());
            RcsGraph_toXML("graph.xml", graph);
            RcsGraph_writeDotFile(graph, dotFile);
            RCHECK(RcsGraph_check(graph, NULL, NULL));


            REXEC(1)
            {
              char osCmd[256];
              sprintf(osCmd, "dotty %s&", dotFile);
              int err = system(osCmd);

              if (err == -1)
              {
                RMSG("Couldn't start dot file viewer!");
              }
            }


          }
          else if (kc->getAndResetKey('p'))
          {
            if (graph != NULL)
            {
              RMSG("Here's the forward tree:");
              RcsGraph_fprint(stderr, graph);
              RLOGS(0, "m=%f   r_com=%f %f %f",
                    mass, r_com[0], r_com[1], r_com[2]);
              RcsGraph_fprintModelState(stdout, graph, graph->q);
            }
          }
          else if (kc->getAndResetKey('b'))
          {
            RMSG("Boxifying graph ...");
            pthread_mutex_lock(&graphLock);
            viewer->removeInternal(gn);
            RCSGRAPH_TRAVERSE_BODIES(graph)
            {
              bool success = RcsBody_boxify(BODY, RCSSHAPE_COMPUTE_GRAPHICS +
                                            RCSSHAPE_COMPUTE_PHYSICS, shapifyReplace);
              RLOG(1, "%s boxifying body %s", success ? "SUCCESS" : "FAILURE",
                   BODY->name);
            }
            gn = new Rcs::GraphNode(graph);
            viewer->addInternal(gn);
            pthread_mutex_unlock(&graphLock);
            RMSG("... done boxifying graph");
          }
          else if (kc->getAndResetKey('B'))
          {
            RMSG("Capsulifying graph ...");
            pthread_mutex_lock(&graphLock);
            viewer->removeInternal(gn);
            RCSGRAPH_TRAVERSE_BODIES(graph)
            {
              bool success = RcsBody_capsulify(BODY, RCSSHAPE_COMPUTE_GRAPHICS,
                                               shapifyReplace);
              RLOG(1, "%s capsulifying body %s", success ? "SUCCESS" : "FAILURE",
                   BODY->name);
            }
            gn = new Rcs::GraphNode(graph);
            viewer->addInternal(gn);
            pthread_mutex_unlock(&graphLock);
            RMSG("... done boxifying graph");
          }
          else if (kc->getAndResetKey('l'))
          {
            RMSG("Reloading GraphNode from %s", xmlFileName);

            if (guiHandle != -1)
            {
              bool success = RcsGuiFactory_destroyGUI(guiHandle);
              RLOG(0, "%s destroyed Gui", success ? "Successfully" : "Not");
            }

            bool collisionVisible = gn->collisionModelVisible();
            bool graphicsVisible = gn->graphicsModelVisible();
            bool physicsVisible = gn->physicsModelVisible();
            bool depthVisible = gn->depthModelVisible();
            bool framesVisible = gn->referenceFramesVisible();
            bool ghostVisible = gn->getGhostMode();
            bool wireframeVisible = gn->getWireframe();
            pthread_mutex_lock(&graphLock);
            viewer->removeInternal(gn);
            viewer->removeInternal("PPSSensorNode");
            gn = NULL;
            RcsGraph_destroy(graph);
            graph = RcsGraph_create(xmlFileName);
            comBase = RcsGraph_getBodyByName(graph, comRef);

            if (graph != NULL)
            {
              gn = new Rcs::GraphNode(graph);
              gn->toggleReferenceFrames();
              gn->displayGraphicsModel(graphicsVisible);
              gn->displayPhysicsModel(physicsVisible);
              gn->displayCollisionModel(collisionVisible);
              gn->displayDepthModel(depthVisible);
              gn->displayReferenceFrames(framesVisible);
              gn->setGhostMode(ghostVisible);
              gn->showWireframe(wireframeVisible);
              pthread_mutex_unlock(&graphLock);
              viewer->add(gn);
              pthread_mutex_lock(&graphLock);
            }
            else
            {
              RLOG(1, "Couldn't create graph - skipping osg "
                   "node");
            }
            pthread_mutex_unlock(&graphLock);
            RMSG("... done");
          }
          else if (kc->getAndResetKey('W'))
          {
            RMSG("Merging bodies. Here are the options:");
            int nBodiesMergeable = 0;
            RCSGRAPH_TRAVERSE_BODIES(graph)
            {
              if (BODY->jntId==-1 && BODY->id!=graph->rootId)
              {
                printf("   %s\n", BODY->name);
                nBodiesMergeable++;
              }
            }

            if (nBodiesMergeable>0)
            {
              std::string name;
              printf("Enter body to merge: ");
              std::cin >> name;
              pthread_mutex_lock(&graphLock);
              bool success = RcsBody_mergeWithParent(graph, name.c_str());
              RMSG("%s merging body %s", success ? "SUCCEEDED" : "FAILED",
                   name.c_str());
              if (success==true)
              {
                FILE* fd = fopen("merged.xml", "w+");
                RCHECK(fd);
                RcsGraph_fprintXML(fd, graph);
                fclose(fd);
              }
              viewer->removeInternal(gn);
              gn = new Rcs::GraphNode(graph);
              gn->toggleReferenceFrames();
              pthread_mutex_unlock(&graphLock);
              viewer->add(gn);
            }
            else
            {
              RMSG("There are no more bodies to merge");
            }
          }
          else if (kc->getAndResetKey('x'))
          {
            RMSG("Rewinding BVH file");
            bvhIdx = 0;
          }
          else if (kc->getAndResetKey('T'))
          {
            RMSG("Loading trajectory file");
            std::string tFile;
            printf("Enter file name: ");
            std::cin >> tFile;

            MatNd* newTraj = MatNd_createFromFile(tFile.c_str());

            if ((newTraj!=NULL) && (newTraj->n==graph->dof))
            {
              MatNd_destroy(bvhTraj);
              bvhTraj = newTraj;
              bvhIdx = 0;
            }
            else
            {
              RLOG(1, "Failed to read trajectory file \"%s\"", tFile.c_str());
            }

          }
          else if (kc->getAndResetKey('X'))
          {
            RMSG("Merging graph to a single rigid body");
            pthread_mutex_lock(&graphLock);
            RMSG("Removing all joints of graph");
            RCSGRAPH_TRAVERSE_BODIES(graph)
            {
              bool success = RcsBody_removeJoints(BODY, graph);
              RMSG("Removing all joints of a body %s %s", BODY->name,
                   success ? "SUCCEEDED" : "FAILED");
            }

            RcsBody* rootBdy = RCSBODY_BY_ID(graph, graph->rootId);
            RcsBody* first = RcsBody_depthFirstTraversalGetNextById(graph, rootBdy);

            while (first)
            {
              bool success = RcsBody_mergeWithParent(graph, first->name);
              RMSG("%s to merge body %s with root",
                   success ? "SUCCEEDED" : "FAILED", first->name);
              first = RcsBody_depthFirstTraversalGetNextById(graph, rootBdy);
              RMSG("Next body to merge: \"%s\"", first ? first->name : "NULL");
              RPAUSE();
            }

            viewer->removeInternal(gn);
            gn = new Rcs::GraphNode(graph);
            gn->toggleReferenceFrames();
            pthread_mutex_unlock(&graphLock);

            viewer->add(gn);
          }
        }   // KeyCatcher

        sprintf(hudText, "Graph \"%s\"\nDof: %d nJ: %d\n"
                "Forward kinematics step: %.1f ms",
                graph->cfgFile, graph->dof, graph->nJ, dtSim*1000.0);

        if (bvhTraj!=NULL)
        {
          char a[256];
          sprintf(a, "BVH row %d (from %d)\n", bvhIdx, bvhTraj->m);
          strcat(hudText, a);
        }

        if (hud != NULL)
        {
          hud->setText(hudText);
        }
        else
        {
          std::cout << hudText;
        }

        REXEC(6)
        {
          RPAUSE();
        }

        Timer_waitDT(dtStep);
        loopCount++;
      }
      if (!valgrind)
      {
        RcsGuiFactory_shutdown();
        delete viewer;
      }
      RcsGraph_destroy(graph);
      MatNd_destroy(bvhTraj);
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
      Rcs::KeyCatcherBase::registerKey("q", "Quit");
      Rcs::KeyCatcherBase::registerKey("p", "Reset physics");
      Rcs::KeyCatcherBase::registerKey("o", "Set physics to random state");
      Rcs::KeyCatcherBase::registerKey("Space", "Toggle pause");
      Rcs::KeyCatcherBase::registerKey("S", "Print out sensors and simulation");
      Rcs::KeyCatcherBase::registerKey("j", "Disable joint limits");
      Rcs::KeyCatcherBase::registerKey("J", "Enable joint limits");
      Rcs::KeyCatcherBase::registerKey("u", "Toggle gravity compensation");
      Rcs::KeyCatcherBase::registerKey("W", "Create joint widget");
      Rcs::KeyCatcherBase::registerKey("m", "Change physics parameters");
      Rcs::KeyCatcherBase::registerKey("e", "Remove body under mouse");
      Rcs::KeyCatcherBase::registerKey("k", "Shoot sphere");
      Rcs::KeyCatcherBase::registerKey("D", "Show dot file of graph");
      Rcs::KeyCatcherBase::registerKey("a", "Deactivate body under mouse");
      Rcs::KeyCatcherBase::registerKey("A", "Activate body under mouse");
      Rcs::KeyCatcherBase::registerKey("Q", "Write current q to model_state");
      Rcs::KeyCatcherBase::registerKey("t", "Run physics test for step");

      double dt = 0.005, tmc = 0.01, damping = 2.0, shootMass = 1.0;
      char hudText[2056] = "";
      char physicsEngine[32] = "Bullet";
      std::string integrator = "Fehlberg";
      char physicsCfg[128] = "config/physics/physics.xml";
      char bgColor[64] = "LIGHT_GRAYISH_GREEN";
      strcpy(xmlFileName, "gScenario.xml");
      strcpy(directory, "config/xml/DexBot");
      bool pause = argP.hasArgument("-pause", "Hit key for each iteration");
      bool posCntrl = argP.hasArgument("-posCntrl",
                                       "Enforce position control");
      bool skipGui = argP.hasArgument("-skipGui",
                                      "No joint angle command Gui");
      bool skipControl = argP.hasArgument("-skipControl",
                                          "No commands considered in physics");
      bool disableCollisions = argP.hasArgument("-disableCollisions",
                                                "Disable collisions between"
                                                " all rigid bodies");
      bool disableJointLimits = argP.hasArgument("-disableJointLimits",
                                                 "Disable all joint limits");
      bool testCopy = argP.hasArgument("-copy", "Test physics copying");
      bool withPPS = argP.hasArgument("-pps", "Launch PPS widgets");
      bool gravComp = argP.hasArgument("-gravComp", "Apply gravity compensation"
                                       " to torque joints");
      bool resizeable = argP.hasArgument("-resizeable", "Adjust visualization "
                                         "of shapes dynamically");
      bool syncHard = argP.hasArgument("-syncHard", "Try to sync with wall "
                                       "clock time as hard as possible");
      bool seqSim = argP.hasArgument("-sequentialPhysics", "Physics simulation "
                                     "step alternating with viewer's frame()");
      argP.getArgument("-physics_config", physicsCfg, "Configuration file name"
                       " for physics (default is %s)", physicsCfg);
      argP.getArgument("-physicsEngine", physicsEngine,
                       "Physics engine (default is \"%s\")", physicsEngine);
      argP.getArgument("-dt", &dt, "Simulation time step (default is %f)", dt);
      argP.getArgument("-tmc", &tmc, "Gui filter, smaller is softer (default"
                       " is: %f)", tmc);
      argP.getArgument("-damping", &damping,
                       "Joint torque damping (default is %f)", damping);
      argP.getArgument("-f", xmlFileName, "Configuration file name (default "
                       "is \"%s\")", xmlFileName);
      argP.getArgument("-dir", directory, "Configuration file directory "
                       "(default is \"%s\")", directory);
      argP.getArgument("-shootMass", &shootMass, "Mass of shooting ball"
                       "(default is \"%f\")", shootMass);
      argP.getArgument("-bgColor", bgColor, "Background color (default is "
                       "\"%s\")", bgColor);
      argP.getArgument("-i", &integrator, "Integrator for Newton-Euler "
                       "simulatio (default is \"%s\")", integrator.c_str());
      getModel(directory, xmlFileName);

      if (argP.hasArgument("-h"))
      {
        RMSG("Mode %d: Rcs -m %d -dir <graph-directory> -f "
             "<graph-file>\n\n\t- Creates a graph from an xml file\n\t"
             "- Creates a viewer (if option -valgrind is not set)\n\t"
             "- Creates a StateGui (if option -valgrind is not set)\n\t"
             "- Runs the forward kinematics in a physics enabled loop\n\n\t"
             "The joints angles can be modified by the sliders\n",
             mode, mode);
        Rcs::PhysicsFactory::print();
        break;
      }

      Rcs_addResourcePath(directory);

      RcsGraph* graph = RcsGraph_create(xmlFileName);
      RCHECK(graph);

      if (posCntrl == true)
      {
        RCSGRAPH_TRAVERSE_JOINTS(graph)
        {
          if ((JNT->ctrlType == RCSJOINT_CTRL_VELOCITY) ||
              (JNT->ctrlType == RCSJOINT_CTRL_TORQUE))
          {
            JNT->ctrlType = RCSJOINT_CTRL_POSITION;
          }
        }
        RcsGraph_setState(graph, NULL, NULL);
      }

      Rcs::PhysicsBase* sim = Rcs::PhysicsFactory::create(physicsEngine,
                                                          graph, physicsCfg);

      if (sim==NULL)
      {
        Rcs::PhysicsFactory::print();
        RFATAL("Couldn't create physics engine \"%s\"", physicsEngine);
      }

      if (testCopy==true)
      {
        RcsGraph* graph2 = graph;
        graph = RcsGraph_clone(graph2);

        Rcs::PhysicsBase* sim2 = sim->clone(graph);
        delete sim;
        sim = sim2;

        RcsGraph_destroy(graph2);
      }

      sim->setParameter(Rcs::PhysicsBase::Simulation, integrator.c_str(),
                        "Integrator", 0.0);
      if (disableCollisions==true)
      {
        sim->disableCollisions();
      }

      // remember initial state for resetting simulation
      MatNd* q0 = MatNd_clone(graph->q);
      MatNd* q_des = MatNd_clone(graph->q);
      MatNd* q_des_f = MatNd_clone(graph->q);
      MatNd* q_curr = MatNd_clone(graph->q);
      MatNd* q_dot_curr = MatNd_create(graph->dof, 1);
      MatNd* T_gravity = MatNd_create(graph->dof, 1);
      MatNd* T_curr = MatNd_create(graph->dof, 1);
      RcsGraph_computeGravityTorque(graph, T_gravity);
      MatNd_constMulSelf(T_gravity, -1.0);

      // Viewer and Gui
      Rcs::KeyCatcher* kc = NULL;
      Rcs::Viewer* viewer = NULL;
      Rcs::HUD* hud = NULL;
      Rcs::JointWidget* jw = NULL;
      Rcs::PhysicsNode* simNode = NULL;

      if (valgrind==false)
      {
        viewer = new Rcs::Viewer(!simpleGraphics, !simpleGraphics);
        viewer->setBackgroundColor(bgColor);
        simNode = new Rcs::PhysicsNode(sim, resizeable);
        viewer->add(simNode);
        hud = new Rcs::HUD();
        viewer->add(hud);
        kc = new Rcs::KeyCatcher();
        viewer->add(kc);

        if (seqSim==false)
        {
          viewer->runInThread(mtx);
        }
        else
        {
          simNode->setDebugDrawer(true);
        }

        if (skipGui==false)
        {
          int guiHandle = Rcs::JointWidget::create(graph, mtx, q_des, q_curr);
          void* ptr = RcsGuiFactory_getPointer(guiHandle);
          jw = static_cast<Rcs::JointWidget*>(ptr);
        }

        if (withPPS==true)
        {
          sim->setEnablePPS(true);
          std::vector<Rcs::PPSGui::Entry> ppsEntries;
          double scaling = 1.0;

          RCSGRAPH_FOREACH_SENSOR(graph)
          {
            if (SENSOR->type==RCSSENSOR_PPS)
            {
              ppsEntries.push_back(Rcs::PPSGui::Entry(SENSOR->name,
                                                      SENSOR->rawData->m,
                                                      SENSOR->rawData->n,
                                                      SENSOR->rawData->ele,
                                                      scaling));
              bool debug = RcsLogLevel > 0 ? true : false;
              viewer->add(new Rcs::PPSSensorNode(SENSOR, graph, debug));
            }
          }

          Rcs::PPSGui::create(ppsEntries, mtx);
        }
      }

      Timer_setZero();
      Timer* timer = Timer_create(dt);
      unsigned int loopCount = 0;
      bool bodyAdded = false;

      while (runLoop)
      {
        if (pause==true)
        {
          RPAUSE_MSG("Hit enter to continue iteration %u", loopCount);
        }

        pthread_mutex_lock(&graphLock);

        //////////////////////////////////////////////////////////////
        // Keycatcher
        /////////////////////////////////////////////////////////////////
        if (kc && kc->getAndResetKey('q'))
        {
          RMSGS("Quitting run loop");
          runLoop = false;
        }
        else if (kc && kc->getAndResetKey('D'))
        {
          RMSGS("Writing dot file");
          RcsGraph_writeDotFile(graph, "graph.dot");
          char osCmd[256];
          sprintf(osCmd, "dotty graph.dot&");
          int err = system(osCmd);

          if (err == -1)
          {
            RMSG("Couldn't start dot file viewer!");
          }
        }
        else if (kc && kc->getAndResetKey('W'))
        {
          RMSGS("Creating JointWidget");
          int guiHandle = Rcs::JointWidget::create(graph, mtx, q_des, q_curr);
          void* ptr = RcsGuiFactory_getPointer(guiHandle);
          jw = static_cast<Rcs::JointWidget*>(ptr);
        }
        else if (kc && kc->getAndResetKey('Q'))
        {
          RcsGraph_fprintModelState(stdout, graph, graph->q);
        }
        else if (kc && kc->getAndResetKey('t'))
        {
          sim->check();
        }
        else if (kc && kc->getAndResetKey('k'))
        {
          RLOG(1, "RcsGraph_addBody");

          // Create a new body in the camera position.
          HTr A_camI;
          viewer->getCameraTransform(&A_camI);
          RcsBody* bdy = RcsBody_createBouncingSphere(graph, A_camI.org,
                                                      shootMass, 0.05);

          // Calculate initial velocity vector from eye point to mouse tip.
          double I_mouseCoords[3];
          viewer->getMouseTip(I_mouseCoords);
          Vec3d_sub(bdy->x_dot, I_mouseCoords, A_camI.org);
          Vec3d_normalizeSelf(bdy->x_dot);
          Vec3d_constMulSelf(bdy->x_dot, 20.0);

          RLOG(1, "Adding body to simulation");
          bool ok = sim->addBody(graph, bdy);

          if (ok)
          {
            RLOG(1, "Adding body to graph");

            MatNd* arrBuf[7];
            arrBuf[0] = q_curr;
            arrBuf[1] = q_dot_curr;
            arrBuf[2] = q_des;
            arrBuf[3] = q_des_f;
            arrBuf[4] = q0;
            arrBuf[5] = T_gravity;
            arrBuf[6] = T_curr;
            ok = RcsGraph_addBodyDofs(graph, NULL, bdy, arrBuf, 7);

            RLOG(1, "Adding body to graphics");
            simNode->addBodyNode(bdy);
            simNode->updateTransformPointers();
          }

          RMSG("%s adding body \"%s\"",
               ok ? "SUCCEEDED" : "FAILED", bdy->name);

          bodyAdded = true;
        }
        else if (kc && kc->getAndResetKey('p'))
        {
          RMSGS("Resetting physics");
          MatNd_setZero(q_dot_curr);
          RcsGraph_setState(graph, q0, q_dot_curr);
          sim->reset(q0);
          MatNd_copy(q_des, graph->q);
          MatNd_copy(q_des_f, graph->q);
          if (jw != NULL)
          {
            if (bodyAdded==true)
            {
              RMSGS("Resetting physics after adding bodies only works if "
                    "there is no JointWidget running. Please click it away "
                    "before resetting the physics, or start the program with "
                    " the command line option \"-skipGui\"");
            }
            else
            {
              jw->reset(graph->q);
            }
          }
        }
        else if (kc && kc->getAndResetKey('o'))
        {
          RMSGS("Resetting physics");
          MatNd_setZero(q_dot_curr);
          MatNd_copy(graph->q, q0);
          MatNd* q_rnd = MatNd_create(graph->dof, 1);
          MatNd_setRandom(q_rnd, -RCS_DEG2RAD(25.0), RCS_DEG2RAD(25.0));
          MatNd_addSelf(graph->q, q_rnd);
          RcsGraph_setState(graph, NULL, q_dot_curr);
          MatNd_destroy(q_rnd);
          sim->reset();
          MatNd_copy(q_des, graph->q);
          MatNd_copy(q_des_f, graph->q);
          if (jw != NULL)
          {
            jw->reset(graph->q);
          }
        }
        else if (kc && kc->getAndResetKey('j'))
        {
          RMSGS("Disabling joint limits");
          sim->setJointLimits(false);
        }
        else if (kc && kc->getAndResetKey('J'))
        {
          RMSGS("Enabling joint limits");
          sim->setJointLimits(true);
        }
        else if (kc && kc->getAndResetKey('u'))
        {
          gravComp = !gravComp;
          RMSGS("Gravity compensation is %s", gravComp ? "ON" : "OFF");
        }
        else if (kc && kc->getAndResetKey('e'))
        {
          viewer->unlock();
          Rcs::BodyNode* bNd = viewer->getBodyNodeUnderMouse<Rcs::BodyNode*>();
          viewer->lock();
          if (bNd == NULL)
          {
            RMSG("No BodyNode found under mouse");
            continue;
          }
          std::string name = std::string(bNd->body()->name);

          RMSG("Removing body \"%s\" under mouse", name.c_str());
          bool ok = sim->removeBody(name.c_str());

          if (ok)
          {
            ok = simNode->removeBodyNode(name.c_str()) && ok;

            MatNd* arrBuf[7];
            arrBuf[0] = q_curr;
            arrBuf[1] = q_dot_curr;
            arrBuf[2] = q_des;
            arrBuf[3] = q_des_f;
            arrBuf[4] = q0;
            arrBuf[5] = T_gravity;
            arrBuf[6] = T_curr;

            ok = RcsGraph_removeBody(graph, name.c_str(), arrBuf, 7) && ok;
          }
          RMSG("%s removing body \"%s\"", ok ? "SUCCEEDED" : "FAILED",
               name.c_str());
        }
        else if (kc && kc->getAndResetKey('a'))
        {
          viewer->unlock();
          Rcs::BodyNode* bNd = viewer->getBodyNodeUnderMouse<Rcs::BodyNode*>();
          viewer->lock();
          if (bNd == NULL)
          {
            RMSG("No BodyNode found under mouse");
            continue;
          }
          std::string name = std::string(bNd->body()->name);

          RMSG("Deactivating body \"%s\" under mouse", name.c_str());
          bool ok = sim->deactivateBody(name.c_str());
          if (ok)
          {
            bNd->setGhostMode(true, "WHITE");
          }
          RMSG("%s deactivating body \"%s\"", ok ? "SUCCEEDED" : "FAILED",
               name.c_str());
        }
        else if (kc && kc->getAndResetKey('A'))
        {
          viewer->unlock();
          Rcs::BodyNode* bNd = viewer->getBodyNodeUnderMouse<Rcs::BodyNode*>();
          viewer->lock();
          if (bNd == NULL)
          {
            RMSG("No BodyNode found under mouse");
            continue;
          }
          std::string name = std::string(bNd->body()->name);

          RMSG("Activating body \"%s\" under mouse", name.c_str());
          HTr A_BI;
          HTr_copy(&A_BI, bNd->getTransformPtr());
          A_BI.org[2] += 0.2;
          bool ok = sim->activateBody(name.c_str(), &A_BI);
          if (ok)
          {
            bNd->setGhostMode(false);
          }

          RMSG("%s activating body \"%s\"", ok ? "SUCCEEDED" : "FAILED",
               name.c_str());
        }
        else if (kc && kc->getAndResetKey('m'))
        {
          RMSGS("Enter physics parameter:");
          int categoryInt;
          std::string type, name;
          double value;
          printf("Enter category (0: Simulation 1: Material 2: Body): ");
          std::cin >> categoryInt;
          printf("Enter type: ");
          std::cin >> type;
          printf("Enter name: ");
          std::cin >> name;
          printf("Enter value: ");
          std::cin >> value;
          Rcs::PhysicsBase::ParameterCategory category;
          category = (Rcs::PhysicsBase::ParameterCategory) categoryInt;
          bool pSuccess = sim->setParameter(category, name.c_str(),
                                            type.c_str(), value);
          RMSGS("%s physics parameters",
                pSuccess ? "Successfully applied" : "Failed to apply");
        }
        else if (kc && kc->getAndResetKey(' '))
        {
          pause = !pause;
          Timer_setTo(timer, sim->time());
          RMSG("Pause modus is %s", pause ? "ON" : "OFF");
        }
        else if (kc && kc->getAndResetKey('l'))
        {
          RMSG("Reloading GraphNode from %s", xmlFileName);
          double t_reload = Timer_getSystemTime();
          int displayMode = simNode->getDisplayMode();
          viewer->removeInternal(simNode);
          simNode = NULL;
          double t_reload2 = Timer_getSystemTime();
          RcsGraph_destroy(graph);
          graph = RcsGraph_create(xmlFileName);
          if (graph != NULL)
          {
            if (posCntrl == true)
            {
              RCSGRAPH_TRAVERSE_JOINTS(graph)
              {
                if ((JNT->ctrlType == RCSJOINT_CTRL_VELOCITY) ||
                    (JNT->ctrlType == RCSJOINT_CTRL_TORQUE))
                {
                  JNT->ctrlType = RCSJOINT_CTRL_POSITION;
                }
              }
              RcsGraph_setState(graph, NULL, NULL);
            }

            delete sim;
            sim = Rcs::PhysicsFactory::create(physicsEngine, graph,
                                              physicsCfg);
            if (disableCollisions==true)
            {
              sim->disableCollisions();
            }

            if (disableJointLimits == true)
            {
              sim->disableJointLimits();
            }
            t_reload2 = Timer_getSystemTime() - t_reload2;

            REXEC(1)
            {
              sim->print();
            }

            MatNd_resizeCopy(&q0, graph->q);
            MatNd_resizeCopy(&q_des, graph->q);
            MatNd_resizeCopy(&q_des_f, graph->q);
            MatNd_resizeCopy(&q_curr, graph->q);
            MatNd_resizeCopy(&q_dot_curr, graph->q_dot);
            T_curr = MatNd_realloc(T_curr, graph->dof, 1);
            MatNd_setZero(T_curr);
            T_gravity = MatNd_realloc(T_gravity, graph->dof, 1);
            RcsGraph_computeGravityTorque(graph, T_gravity);
            MatNd_constMulSelf(T_gravity, -1.0);
            simNode = new Rcs::PhysicsNode(sim);
            simNode->setDisplayMode(displayMode);
            pthread_mutex_unlock(&graphLock);
            viewer->add(simNode);
            pthread_mutex_lock(&graphLock);
          }
          else
          {
            RLOG(1, "Couldn't create graph - skipping osg "
                 "node and physics simulation");
          }
          t_reload = Timer_getSystemTime() - t_reload;
          RMSG("... took %.2f msec (%.2f msec simulation only)",
               1000.0*t_reload, 1000.0*t_reload2);
        }
        else if (kc && kc->getAndResetKey('S'))
        {
          RMSGS("Printing out sensors");

          RCSGRAPH_FOREACH_SENSOR(graph)
          {
            RcsSensor_fprint(stdout, SENSOR);
          }

          sim->print();
          RcsGraph_toXML("gSim.xml", sim->getGraph());
        }   // if (kc && ...)

        if (valgrind)
        {
          RLOG(1, "Step");
        }




        //////////////////////////////////////////////////////////////
        // Compute control input
        /////////////////////////////////////////////////////////////////
        if (gravComp==true)
        {
          // Update state in case the graph changed due to Gui input
          RcsGraph_setState(graph, NULL, NULL);

          // Velocity damping
          MatNd* M_damp = MatNd_clone(q_dot_curr);
          MatNd* MM = MatNd_create(graph->nJ, graph->nJ);
          RcsGraph_stateVectorToIKSelf(graph, M_damp);
          MatNd_constMulSelf(M_damp, -damping);
          RcsGraph_computeMassMatrix(graph, MM);
          MatNd_preMulSelf(M_damp, MM);

          // Gravity compensation
          RcsGraph_computeGravityTorque(graph, T_gravity);
          MatNd_constMulSelf(T_gravity, -1.0);
          MatNd_addSelf(T_gravity, M_damp);

          MatNd_destroy(M_damp);
          MatNd_destroy(MM);
        }
        else
        {
          MatNd_reshapeAndSetZero(T_gravity, graph->dof, 1);
        }

        // Desired joint angles from Gui
        for (unsigned int i=0; i<graph->dof; i++)
        {
          q_des_f->ele[i] = (1.0-tmc)*q_des_f->ele[i] + tmc*q_des->ele[i];
        }

        sim->setControlInput(q_des_f, NULL, T_gravity);

        //////////////////////////////////////////////////////////////
        // call physics simulation and read new current state
        //////////////////////////////////////////////////////////////

        double dtSim = Timer_getTime();
        sim->simulate(dt, graph, NULL, NULL, !skipControl);
        sim->getJointAngles(q_curr);
        REXEC(5)
        {
          sim->getJointTorque(T_curr);
          MatNd_printCommentDigits("T_curr", T_curr, 4);
        }
        dtSim = Timer_getTime() - dtSim;


        //////////////////////////////////////////////////////////////
        // Forward kinematics
        //////////////////////////////////////////////////////////////
        RcsGraph_setState(graph, q_curr, q_dot_curr);

        pthread_mutex_unlock(&graphLock);

        if (seqSim==true)
        {
          viewer->frame();
        }

        if (valgrind)
        {
          RLOG(1, "Step");
        }

        snprintf(hudText, 2056,
                 "[%s]: Sim-step: %.1f ms\nSim time: %.1f (%.1f) sec\n"
                 "Bodies: %d   Joints: %d\n"
                 "Gravity compensation: %s\nDisplaying %s",
                 sim->getClassName(), dtSim*1000.0, sim->time(),
                 Timer_get(timer),
                 sim->getGraph()->nBodies, sim->getGraph()->dof,
                 gravComp ? "ON" : "OFF",
                 simNode ? simNode->getDisplayModeStr() : "nothing");
        Rcs::KineticSimulation* kSim = dynamic_cast<Rcs::KineticSimulation*>(sim);
        if (kSim)
        {
          char neText[128];
          if (kSim->getIntegrator()=="Euler")
          {
            snprintf(neText, 128, "\nIntegrator: Euler   Energy: %.4f",
                     kSim->getEnergy());
          }
          else if (kSim->getIntegrator()=="Fehlberg")
          {
            snprintf(neText, 128, "\nIntegrator: Fehlberg   Energy: %.4f   step: %f",
                     kSim->getEnergy(), kSim->getAdaptedDt());
          }
          strcat(hudText, neText);
        }

        if (hud != NULL)
        {
          hud->setText(hudText);
        }
        else
        {
          std::cout << hudText;
        }

        if (syncHard)
        {
          Timer_wait(timer);
        }
        else
        {
          Timer_waitNoCatchUp(timer);
          Timer_usleep(1);
        }

        loopCount++;

        if ((loopCount>10) && (valgrind==true))
        {
          runLoop = false;
        }

      }

      Timer_destroy(timer);
      if (!valgrind)
      {
        RcsGuiFactory_shutdown();
        delete viewer;
      }
      MatNd_destroy(q0);
      MatNd_destroy(q_des);
      MatNd_destroy(q_des_f);
      MatNd_destroy(q_curr);
      MatNd_destroy(q_dot_curr);
      MatNd_destroy(T_gravity);
      MatNd_destroy(T_curr);

      delete sim;
      RcsGraph_destroy(graph);
      break;
    }

    // ==============================================================
    // Inverse kinematics
    // ==============================================================
    case 5:
    {
      Rcs::KeyCatcherBase::registerKey("q", "Quit");
      Rcs::KeyCatcherBase::registerKey("t", "Run controller test");
      Rcs::KeyCatcherBase::registerKey(" ", "Toggle pause");
      Rcs::KeyCatcherBase::registerKey("a", "Change IK algorithm");
      Rcs::KeyCatcherBase::registerKey("d", "Write q-vector to q.dat");
      Rcs::KeyCatcherBase::registerKey("D", "Set q-vector from file q.dat");
      Rcs::KeyCatcherBase::registerKey("n", "Reset to default state");
      Rcs::KeyCatcherBase::registerKey("C", "Toggle closest point lines");
      Rcs::KeyCatcherBase::registerKey("o", "Toggle distance calculation");
      Rcs::KeyCatcherBase::registerKey("m", "Manipulability null space");
      Rcs::KeyCatcherBase::registerKey("e", "Remove body under mouse");
      Rcs::KeyCatcherBase::registerKey("E", "Link generic body");
      Rcs::KeyCatcherBase::registerKey("v", "Write current q to model_state");
      Rcs::KeyCatcherBase::registerKey("f", "Toggle physics feedback");
      Rcs::KeyCatcherBase::registerKey("p", "Print to console and file");
      Rcs::KeyCatcherBase::registerKey("H", "Toggle HUD");
      Rcs::KeyCatcherBase::registerKey("k", "Toggle GraphNode");
      Rcs::KeyCatcherBase::registerKey("S", "Reset physics");

      int algo = 0;
      double alpha = 0.05, lambda = 1.0e-8, tmc = 0.1, dt = 0.01, dt_calc = 0.0;
      double jlCost = 0.0, dJlCost = 0.0, clipLimit = DBL_MAX;
      double scaleDragForce = 0.01;
      bool calcDistance = true;
      strcpy(xmlFileName, "cAction.xml");
      strcpy(directory, "config/xml/DexBot");
      char effortBdyName[256] = "";
      std::string physicsEngine;
      char physicsCfg[128] = "config/physics/physics.xml";

      argP.getArgument("-algo", &algo, "IK algorithm: 0: left inverse, 1: "
                       "right inverse (default is %d)", algo);
      argP.getArgument("-alpha", &alpha,
                       "Null space scaling factor (default is %g)", alpha);
      argP.getArgument("-lambda", &lambda, "Regularization (default is %g)",
                       lambda);
      argP.getArgument("-f", xmlFileName);
      argP.getArgument("-dir", directory);
      argP.getArgument("-tmc", &tmc, "Filter time constant for sliders");
      argP.getArgument("-dt", &dt, "Sampling time interval (default: %f)", dt);
      argP.getArgument("-clipLimit", &clipLimit, "Clip limit for dx (default "
                       "is DBL_MAX)");
      argP.getArgument("-staticEffort", effortBdyName,
                       "Body to map static effort");
      argP.getArgument("-physics_config", physicsCfg, "Configuration file name"
                       " for physics (default is %s)", physicsCfg);
      argP.getArgument("-physicsEngine", &physicsEngine, "Physics engine "
                       "(default is \"%s\")", physicsEngine.c_str());
      argP.getArgument("-scaleDragForce", &scaleDragForce, "Scale factor for"
                       " mouse dragger (default is \"%f\")", scaleDragForce);
      bool ffwd = argP.hasArgument("-ffwd", "Feed-forward dx only");
      bool skipGui = argP.hasArgument("-skipGui", "No GUIs, only viewer");
      bool pause = argP.hasArgument("-pause", "Pause after each iteration");
      bool launchJointWidget = argP.hasArgument("-jointWidget",
                                                "Launch JointWidget");
      bool manipulability = argP.hasArgument("-manipulability",
                                             "Manipulability criterion in "
                                             "null space");
      bool cAvoidance = argP.hasArgument("-ca", "Collision avoidance in "
                                         "null space");
      bool constraintIK = argP.hasArgument("-constraintIK", "Use constraint IK"
                                           " solver");
      bool initToQ0 = argP.hasArgument("-setDefaultStateFromInit", "Set the "
                                       "joint center defaults from the initial"
                                       " state");
      bool testCopying = argP.hasArgument("-copy", "Test copying");
      bool noHud = argP.hasArgument("-noHud", "Don't show HUD");
      bool posCntrl = argP.hasArgument("-posCntrl",
                                       "Enforce position control with physics");

      Rcs_addResourcePath(directory);

      if (argP.hasArgument("-h"))
      {
        printf("Resolved motion rate control test\n\n");
        Rcs::ControllerBase::printUsage(xmlFileName);
        break;
      }

      // Create controller
      Rcs::ControllerBase controller(xmlFileName);

      if (testCopying)
      {
        Rcs::ControllerBase tmp(controller);
        controller = tmp;
      }

      if (initToQ0)
      {
        MatNd* q_init = MatNd_createLike(controller.getGraph()->q);
        RcsGraph_getInitState(controller.getGraph(), q_init);
        RcsGraph_changeDefaultState(controller.getGraph(), q_init);
      }


      Rcs::IkSolverRMR* ikSolver = NULL;

      if (constraintIK==true)
      {
        ikSolver = new Rcs::IkSolverConstraintRMR(&controller);
      }
      else
      {
        ikSolver = new Rcs::IkSolverRMR(&controller);
      }

      MatNd* dq_des    = MatNd_create(controller.getGraph()->dof, 1);
      MatNd* q_dot_des = MatNd_create(controller.getGraph()->dof, 1);
      MatNd* a_des     = MatNd_create(controller.getNumberOfTasks(), 1);
      MatNd* x_curr    = MatNd_create(controller.getTaskDim(), 1);
      MatNd* x_des     = MatNd_create(controller.getTaskDim(), 1);
      MatNd* x_des_f   = MatNd_create(controller.getTaskDim(), 1);
      MatNd* dx_des    = MatNd_create(controller.getTaskDim(), 1);
      MatNd* dH        = MatNd_create(1, controller.getGraph()->nJ);

      controller.readActivationsFromXML(a_des);
      controller.computeX(x_curr);
      MatNd_copy(x_des, x_curr);
      MatNd_copy(x_des_f, x_curr);

      // Body for static effort null space gradient
      const RcsBody* effortBdy = RcsGraph_getBodyByName(controller.getGraph(),
                                                        effortBdyName);
      MatNd* F_effort = NULL;
      MatNd_fromStack(F_effort, 4, 1);
      MatNd F_effort3 = MatNd_fromPtr(3, 1, F_effort->ele);

      // Physics engine
      Rcs::PhysicsBase* sim = NULL;
      RcsGraph* simGraph = NULL;
      bool physicsFeedback = false;

      if (Rcs::PhysicsFactory::hasEngine(physicsEngine.c_str()))
      {
        simGraph = RcsGraph_clone(controller.getGraph());

        if (posCntrl)
        {
          RCSGRAPH_TRAVERSE_JOINTS(simGraph)
          {
            if ((JNT->ctrlType == RCSJOINT_CTRL_VELOCITY) ||
                (JNT->ctrlType == RCSJOINT_CTRL_TORQUE))
            {
              JNT->ctrlType = RCSJOINT_CTRL_POSITION;
            }
          }
          RcsGraph_setState(simGraph, NULL, NULL);
        }

        sim = Rcs::PhysicsFactory::create(physicsEngine.c_str(),
                                          simGraph, physicsCfg);

        if (sim==NULL)
        {
          Rcs::PhysicsFactory::print();
          RLOG_CPP(1, "Couldn't create physics \"" << physicsEngine << "\"");
          RcsGraph_destroy(simGraph);
          simGraph = NULL;
        }
      }


      // Create visualization
      Rcs::Viewer* v           = NULL;
      osg::ref_ptr<Rcs::KeyCatcher> kc;
      osg::ref_ptr<Rcs::GraphNode> gn;
      osg::ref_ptr<Rcs::PhysicsNode> simNode;
      osg::ref_ptr<Rcs::HUD> hud;
      osg::ref_ptr<Rcs::BodyPointDragger> dragger;
      osg::ref_ptr<Rcs::VertexArrayNode> cn;
      char hudText[2056];

      if (valgrind==false)
      {
        v       = new Rcs::Viewer(!simpleGraphics, !simpleGraphics);
        kc      = new Rcs::KeyCatcher();
        gn      = new Rcs::GraphNode(controller.getGraph());

        if (!noHud)
        {
          hud     = new Rcs::HUD();
          v->add(hud);
        }

        dragger = new Rcs::BodyPointDragger();
        dragger->scaleDragForce(scaleDragForce);
        v->add(gn.get());
        v->add(kc.get());
        v->add(dragger.get());

        if (sim)
        {
          simNode = new Rcs::PhysicsNode(sim);
          gn->setGhostMode(true, "RED");
          v->add(simNode.get());
        }

        if (controller.getCollisionMdl() != NULL)
        {
          cn = new Rcs::VertexArrayNode(controller.getCollisionMdl()->cp,
                                        osg::PrimitiveSet::LINES, "RED");
          cn->toggle();
          v->add(cn.get());
        }

        v->runInThread(mtx);

        // Launch the task widget
        if (ffwd == false)
        {
          if (!skipGui)
          {
            if (algo==0 && lambda>0.0)
            {
              Rcs::ControllerWidgetBase::create(&controller, a_des,
                                                ikSolver->getCurrentActivation(), x_des,
                                                x_curr, mtx);
            }
            else
            {
              Rcs::ControllerWidgetBase::create(&controller, a_des,
                                                x_des, x_curr, mtx);
            }
          }
        }
        else
        {
          if (!skipGui)
          {
            // Launch the task widget
            Rcs::MatNdWidget* mw = Rcs::MatNdWidget::create(dx_des, x_curr,
                                                            -1.0, 1.0, "dx",
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
        }


        if (launchJointWidget==true)
        {
          Rcs::JointWidget::create(controller.getGraph(), mtx);
        }

        if (effortBdy != NULL)
        {
          std::vector<std::string> labels;
          Rcs::MatNdWidget* mw = Rcs::MatNdWidget::create(F_effort, F_effort,
                                                          -1.0, 1.0,
                                                          "F_effort", mtx);
          labels.push_back("Fx");
          labels.push_back("Fy");
          labels.push_back("Fz");
          labels.push_back("gain");
          mw->setLabels(labels);
        }
      }

      unsigned int loopCount = 0;


      // Endless loop
      while (runLoop == true)
      {
        pthread_mutex_lock(&graphLock);

        dt_calc = Timer_getTime();

        if (ffwd==false)
        {

          for (unsigned int i=0; i<x_des_f->m; i++)
          {
            x_des_f->ele[i] = tmc*x_des->ele[i] +
                              (1.0-tmc)*x_des_f->ele[i];
          }

          controller.computeDX(dx_des, x_des_f);
          MatNd clipArr = MatNd_fromPtr(1, 1, &clipLimit);
          MatNd_saturateSelf(dx_des, &clipArr);
        }

        controller.computeJointlimitGradient(dH);

        if (calcDistance==true)
        {
          controller.computeCollisionCost();
        }

        if (cAvoidance==true)
        {
          MatNd* dH_ca = MatNd_create(1, controller.getGraph()->dof);
          controller.getCollisionGradient(dH_ca);
          RcsGraph_limitJointSpeeds(controller.getGraph(), dH_ca,
                                    1.0, RcsStateIK);
          MatNd_constMulSelf(dH_ca, 0.01);
          MatNd_addSelf(dH, dH_ca);
        }

        if (manipulability)
        {
          MatNd_setZero(dH);
          controller.computeManipulabilityGradient(dH, a_des);
          MatNd_constMulSelf(dH, 100.0);
        }

        if (effortBdy != NULL)
        {
          MatNd* W_ef = MatNd_create(controller.getGraph()->dof, 1);
          RCSGRAPH_TRAVERSE_JOINTS(controller.getGraph())
          {
            W_ef->ele[JNT->jointIndex] = 1.0/JNT->maxTorque;
          }

          RcsGraph_stateVectorToIKSelf(controller.getGraph(), W_ef);
          MatNd* effortGrad = MatNd_create(1, controller.getGraph()->nJ);
          RcsGraph_staticEffortGradient(controller.getGraph(), effortBdy,
                                        &F_effort3, W_ef, NULL, effortGrad);
          MatNd_destroy(W_ef);
          MatNd_constMulSelf(effortGrad, 1000.0*MatNd_get(F_effort, 3, 0));
          MatNd_addSelf(dH, effortGrad);

          MatNd_destroy(effortGrad);
        }

        MatNd_constMulSelf(dH, alpha);

        if (valgrind==false)
        {
          dragger->addJointTorque(dH, controller.getGraph());
        }

        switch (algo)
        {
          case 0:
            ikSolver->solveLeftInverse(dq_des, dx_des, dH, a_des, lambda);
            break;

          case 1:
            ikSolver->solveRightInverse(dq_des, dx_des, dH, a_des, lambda);
            break;

          default:
            RFATAL("No such algorithm; %d", algo);
        }

        MatNd_constMul(q_dot_des, dq_des, 1.0/dt);

        MatNd_addSelf(controller.getGraph()->q, dq_des);
        RcsGraph_setState(controller.getGraph(), NULL, q_dot_des);
        bool poseOK = controller.checkLimits();
        controller.computeX(x_curr);

        if (sim)
        {
          sim->setControlInput(controller.getGraph()->q, q_dot_des, NULL);
          sim->simulate(dt, simGraph);
          RcsGraph_setState(simGraph, NULL, NULL);
          if (physicsFeedback)
          {
            RcsGraph_setState(controller.getGraph(), simGraph->q,
                              simGraph->q_dot);
          }
          else
          {
            RcsGraph* dstGraph = controller.getGraph();
            const RcsGraph* srcGraph = sim->getGraph();
            RCHECK(dstGraph->nSensors==srcGraph->nSensors);

            for (unsigned int i=0; i<dstGraph->nSensors; ++i)
            {
              RCHECK(dstGraph->sensors[i].type==srcGraph->sensors[i].type);
              RcsSensor_copy(&dstGraph->sensors[i], &srcGraph->sensors[i]);
            }
          }
        }

        dJlCost = -jlCost;
        jlCost = controller.computeJointlimitCost();
        dJlCost += jlCost;

        dt_calc = Timer_getTime() - dt_calc;

        // Compute inside mutex, otherwise clicking activation boxes in the Gui
        // can lead to crashes.
        double manipIdx = controller.computeManipulabilityCost(a_des);
        double staticEff = RcsGraph_staticEffort(controller.getGraph(),
                                                 effortBdy, &F_effort3,
                                                 NULL, NULL);

        pthread_mutex_unlock(&graphLock);

        if (kc && kc->getAndResetKey('q'))
        {
          runLoop = false;
        }
        else if (kc && kc->getAndResetKey('H'))
        {
          if (hud.valid())
          {
            hud->toggle();
          }
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
          pthread_mutex_lock(&graphLock);
          controller.test(true);
          pthread_mutex_unlock(&graphLock);
        }
        else if (kc && kc->getAndResetKey(' '))
        {
          pause = !pause;
          RMSG("Pause modus is %s", pause ? "ON" : "OFF");
        }
        else if (kc && kc->getAndResetKey('d'))
        {
          RMSG("Writing q to file \"q.dat\"");
          MatNd* q_deg = MatNd_clone(controller.getGraph()->q);
          VecNd_constMulSelf(q_deg->ele,180.0/M_PI,q_deg->m);
          MatNd_toFile(q_deg, "q.dat");
          MatNd_destroy(q_deg);
        }
        else if (kc && kc->getAndResetKey('D'))
        {
          bool success = MatNd_fromFile(controller.getGraph()->q, "q.dat");
          RMSG("%s read q from file \"q.dat\"",
               success ? "Successfully" : "Failed to");
          if (success)
          {
            RcsGraph_setState(controller.getGraph(), NULL, NULL);
          }
        }
        else if (kc && kc->getAndResetKey('n'))
        {
          RMSG("Resetting");
          RcsGraph_setDefaultState(controller.getGraph());
        }
        else if (kc && kc->getAndResetKey('k'))
        {
          if (gn)
          {
            RMSG("Toggling GraphNode");
            gn->toggle();
          }
        }
        else if (kc && kc->getAndResetKey('C') && cn)
        {
          RMSG("Toggle closest points visualization");
          cn->toggle();
        }
        else if (kc && kc->getAndResetKey('o'))
        {
          calcDistance = !calcDistance;
          RMSG("Distance calculation is %s", calcDistance ? "ON" : "OFF");
        }
        else if (kc && kc->getAndResetKey('m'))
        {
          manipulability = !manipulability;
          RMSG("Manipulation index nullspace is %s",
               manipulability ? "ON" : "OFF");
        }
        else if (kc && kc->getAndResetKey('e'))
        {
          Rcs::BodyNode* bNd = v->getBodyNodeUnderMouse<Rcs::BodyNode*>();
          if (bNd == NULL)
          {
            RMSG("No BodyNode found under mouse");
            continue;
          }
          std::string name = std::string(bNd->body()->name);

          RMSG("Removing body \"%s\" under mouse", name.c_str());
          pthread_mutex_lock(&graphLock);
          bool ok = true;

          if (sim)
          {
            RLOG(0, "Removing from simulator");
            ok = sim->removeBody(name.c_str());
          }

          if (ok)
          {
            MatNd* arrBuf[2];
            arrBuf[0] = dq_des;
            arrBuf[1] = q_dot_des;

            RLOG(0, "Removing from graph");
            ok = RcsGraph_removeBody(controller.getGraph(), name.c_str(),
                                     arrBuf, 2) && ok;

            if (ok && simNode)
            {
              RLOG(0, "Removing from simNode");
              ok = simNode->removeBodyNode(name.c_str()) && ok;
            }

            if (ok && gn)
            {
              RLOG(0, "Removing from GraphNode");
              ok = gn->removeBodyNode(name.c_str()) && ok;
            }

          }
          pthread_mutex_unlock(&graphLock);
          RMSG("%s removing body \"%s\"", ok ? "SUCCEEDED" : "FAILED",
               name.c_str());
        }
        else if (kc && kc->getAndResetKey('E'))
        {
          std::string name;
          RMSG("Linking GenericBody");
          printf("Enter body to link against: ");
          std::cin >> name;

          RcsBody* lb = RcsGraph_linkGenericBody(controller.getGraph(),
                                                 0, name.c_str());
          RMSG("Linked against \"%s\"", lb ? lb->name : "NULL");
        }
        else if (kc && kc->getAndResetKey('v'))
        {
          RcsGraph_fprintModelState(stdout, controller.getGraph(),
                                    controller.getGraph()->q);
        }
        else if (kc && kc->getAndResetKey('p'))
        {
          controller.print();
          controller.toXML("cAction.xml");
        }
        else if (kc && kc->getAndResetKey('f'))
        {
          physicsFeedback = !physicsFeedback;
          RMSG("Physics feedback is %s", physicsFeedback ? "ON" : "OFF");
        }
        else if (kc && kc->getAndResetKey('S'))
        {
          if (sim)
          {
            RMSG("Resetting simulation");
            sim->reset(controller.getGraph()->q);
          }
        }

        char timeStr[64] = "";
        if (1.0e6*dt_calc>10000000.0)   // show seconds
        {
          snprintf(timeStr, 64, "%.1f s", dt_calc);
        }
        else if (1.0e6*dt_calc>10000.0)   // show milliseconds
        {
          snprintf(timeStr, 64, "%.1f ms", 1.0e3*dt_calc);
        }
        else
        {
          snprintf(timeStr, 64, "%.1f us", 1.0e6*dt_calc);
        }

        snprintf(hudText, 2056,
                 "IK calculation: %s\ndof: %d nJ: %d "
                 "nqr: %d nx: %d\nJL-cost: %.6f dJL-cost: %.6f %s %s"
                 "\nalgo: %d lambda:%g alpha: %g tmc: %.3f\n"
                 "Manipulability index: %.6f\n"
                 "Static effort: %.6f\n"
                 "Robot pose %s",
                 timeStr, controller.getGraph()->dof,
                 controller.getGraph()->nJ, ikSolver->getInternalDof(),
                 (int) controller.getActiveTaskDim(a_des),
                 jlCost, dJlCost,
                 ikSolver->getDeterminant()==0.0?"SINGULAR":"",
                 ((dJlCost > 1.0e-8) && (MatNd_getNorm(dx_des) == 0.0)) ?
                 "COST INCREASE" : "",
                 algo, lambda, alpha, tmc, manipIdx, staticEff,
                 poseOK ? "VALID" : "VIOLATES LIMITS");

        // snprintf(hudText, 2056,
        //          "IK calculation: %s\ndof: %d nJ: %d "
        //          "nqr: %d nx: %d\nJL-cost: %.6f",
        //          timeStr, controller.getGraph()->dof,
        //          controller.getGraph()->nJ, ikSolver->getInternalDof(),
        //          (int) controller.getActiveTaskDim(a_des),
        //          jlCost);

        if (hud.valid())
        {
          hud->setText(hudText);
        }
        else
        {
          RLOG_CPP(1, "Hud text: " << hudText);
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
        Timer_waitDT(0.01);
      }



      // Clean up
      if (valgrind==false)
      {
        delete v;
        RcsGuiFactory_shutdown();
      }

      MatNd_destroy(dq_des);
      MatNd_destroy(q_dot_des);
      MatNd_destroy(a_des);
      MatNd_destroy(x_curr);
      MatNd_destroy(x_des);
      MatNd_destroy(x_des_f);
      MatNd_destroy(dx_des);
      MatNd_destroy(dH);

      delete ikSolver;

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
      double alpha = 0.01, lambda = 1.0e-8;
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
      Rcs::ControllerBase controller(xmlFileName, true);
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
            ikSolver.solveLeftInverse(dq_des, dx_des, dH, a_des, lambda);
            break;

          case 1:
            ikSolver.solveRightInverse(dq_des, dx_des, dH, a_des, lambda);
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
                ikSolver.getDeterminant()==0.0?"SINGULAR":"",
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
      double alpha = 0.05, lambda = 0.0;
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
        ikSolver.solveRightInverse(dq_ts, dq_ns, dx_des, dH, a_des, lambda);
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
                jlCost, dJlCost,
                ikSolver.getDeterminant()==0.0?"SINGULAR":"",
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
    // Bit mask test
    // ==============================================================
    case 16:
    {
      unsigned int mask = 0;

      mask += 1 << 1;
      mask += 1 << 4;

      RMSG("Mask is %d", mask);
      Math_printBinaryVector(mask);

      for (unsigned int i=0; i<7; ++i)
      {
        RMSG("Bit %d is %s", i, Math_isBitSet(mask, i) ? "SET" : "CLEAR");
      }
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
