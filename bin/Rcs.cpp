/*******************************************************************************

  Copyright (c) 2017, Honda Research Institute Europe GmbH.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.

  3. All advertising materials mentioning features or use of this software
     must display the following acknowledgement: This product includes
     software developed by the Honda Research Institute Europe GmbH.

  4. Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER "AS IS" AND ANY EXPRESS OR
  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#include <Rcs_URDFParser.h>
#include <Rcs_macros.h>
#include <Rcs_cmdLine.h>
#include <Rcs_math.h>
#include <Rcs_geometry.h>
#include <Rcs_gradientTests.h>
#include <Rcs_resourcePath.h>
#include <Rcs_timer.h>
#include <Rcs_sensor.h>
#include <Rcs_typedef.h>
#include <Rcs_kinematics.h>
#include <Rcs_dynamics.h>
#include <Rcs_joint.h>
#include <Rcs_body.h>
#include <Rcs_shape.h>
#include <Rcs_utils.h>
#include <IkSolverRMR.h>
#include <SolverRAC.h>
#include <TaskFactory.h>
#include <PhysicsFactory.h>
#include <PhysicsNode.h>
#include <GraphNode.h>
#include <FTSensorNode.h>
#include <CapsuleNode.h>
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
#include <TargetSetter.h>
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

  if (!argP.hasArgument("-model", "Example models: Husky, DexBot"))
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
  RMSG("Starting Rcs...");
  int mode = 0, simpleGraphics = 0;
  char xmlFileName[128] = "", directory[128] = "";
  char physicsCfg[128] = "config/physics/vortex.xml";

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
  argP.getArgument("-physics_config", physicsCfg,
                   "Configuration file name for physics (default is %s)",
                   physicsCfg);
  argP.getArgument("-dir", directory, "Configuration file directory");
  bool valgrind = argP.hasArgument("-valgrind",
                                   "Start without Guis and graphics");
  simpleGraphics = argP.hasArgument("-simpleGraphics", "OpenGL without "
                                    "fancy stuff (shadows, anti-aliasing)");

  // Initialize GUI and OSG mutex
  pthread_mutex_t graphLock;
  pthread_mutex_init(&graphLock, NULL);

  // Option without mutex for viewer
  pthread_mutex_t* mtx = &graphLock;
  if (argP.hasArgument("-nomutex", "Graphics without mutex"))
  {
    mtx = NULL;
  }

  runLoop = true;

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
      printf("\t\t9   MatNdWidget test\n");
      printf("\t\t12  Distance function test\n");
      printf("\t\t13  Depth first traversal test\n");

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
        Rcs::TaskFactory::instance()->printRegisteredTasks();
        Rcs::PhysicsFactory::print();
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
        RMSG("Mode %d: Rcs.exe -m %d -dir <graph-directory> -f "
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

      RMSG("Here's the forward tree:");
      RcsGraph_fprint(stderr, graph);
      RcsGraph_writeDotFile(graph, "RcsGraph.dot");

      RMSG("Writing graph to xml file");
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
      Rcs::KeyCatcherBase::registerKey("L", "Reload graph file (force). If "
                                       "the JointWidget is active the "
                                       "application might crash.");
      Rcs::KeyCatcherBase::registerKey("m", "Set q to model state");
      Rcs::KeyCatcherBase::registerKey("p", "Print information to console");
      Rcs::KeyCatcherBase::registerKey("q", "Quit");
      Rcs::KeyCatcherBase::registerKey("W", "Merge bodies");

      double dtSim = 0.0;
      char hudText[512] = "", comRef[64] = "";
      char dotFile[256] = "RcsGraph.dot";
      strcpy(xmlFileName, "gScenario.xml");
      strcpy(directory, "config/xml/DexBot");
      getModel(directory, xmlFileName);

      argP.getArgument("-dotFile", dotFile, "Dot file name");
      argP.getArgument("-f", xmlFileName, "Configuration file name (default"
                       " is \"%s\")", xmlFileName);
      argP.getArgument("-dir", directory, "Configuration file directory "
                       "(default is \"%s\")", directory);
      argP.getArgument("-comRef", comRef, "Reference body for COM (default is "
                       "root)");
      bool testCopy = argP.hasArgument("-copy", "Test graph copying");
      bool resizeable = argP.hasArgument("-resizeable", "Adjust visualization of shapes dynamically");
      bool editMode = argP.hasArgument("-edit", "Start in xml edit mode "
                                       "(no Qt Gui)");

      Rcs_addResourcePath(directory);

      if (argP.hasArgument("-h"))
      {
        RMSG("Mode %d: Rcs.exe -m %d -dir <graph-directory> -f "
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

      RcsGraph* graph = RcsGraph_create(xmlFileName);

      if (graph == NULL)
      {
        RMSG("Failed to create graph from file \"%s\" - exiting",
             xmlFileName);
        break;
      }

      if (testCopy==true)
      {
        RcsGraph* graph2 = graph;
        double t_copy = Timer_getSystemTime();
        graph = RcsGraph_clone(graph2);
        t_copy = Timer_getSystemTime() - t_copy;
        RcsGraph_destroy(graph2);
        RMSG("Copying graph took %.3f msec", t_copy*1.0e3);
      }

      const RcsBody* comBase = RcsGraph_getBodyByName(graph, comRef);

      RCSGRAPH_TRAVERSE_JOINTS(graph)
      {
        JNT->constrained = true;
      }

      unsigned int loopCount = 0;
      double mass = 0.0, Id[3][3], r_com[3];
      Mat3d_setIdentity(Id);
      Vec3d_setZero(r_com);

      Rcs::KeyCatcher* kc = NULL;
      Rcs::GraphNode* gn  = NULL;
      Rcs::CapsuleNode* comNd = NULL;
      Rcs::HUD* hud = NULL;
      Rcs::Viewer* viewer = NULL;

      if (!valgrind)
      {
        viewer = new Rcs::Viewer(!simpleGraphics, !simpleGraphics);
        gn = new Rcs::GraphNode(graph, resizeable);
        gn->toggleReferenceFrames();
        viewer->add(gn);

        comNd = new Rcs::CapsuleNode(r_com, Id, 0.05, 0.0);
        comNd->makeDynamic(r_com);
        comNd->setMaterial("RED");
        viewer->add(comNd);

        hud = new Rcs::HUD();
        viewer->add(hud);

        kc = new Rcs::KeyCatcher();
        viewer->add(kc);
        viewer->runInThread(mtx);
        if (editMode == false)
        {
          Rcs::JointWidget::create(graph, mtx);
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
          dtSim = Timer_getSystemTime();
          RcsGraph_setState(graph, NULL, NULL);
          dtSim = Timer_getSystemTime() - dtSim;
          if (comBase != NULL)
          {
            mass = RcsGraph_COG_Body(comBase, r_com);
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
          else if (kc->getAndResetKey('S'))
          {
            double scaleFactor;
            RMSG("Changing scale factor");
            printf("Enter scaling factor: ");
            std::cin >> scaleFactor;
            RcsGraph_scale(graph, scaleFactor);
          }
          else if (kc->getAndResetKey('m'))
          {
            std::string mdlState;
            RMSG("Changing model state");
            printf("Enter name of model state: ");
            std::cin >> mdlState;
            bool ok = RcsGraph_setStateFromXML(graph, mdlState.c_str(), 0);
            RMSG("%s changing model state to %s",
                 ok ? "SUCCEEDED" : "FAILED", mdlState.c_str());
          }
          else if (kc->getAndResetKey('j'))
          {
            RMSGS("Creating JointWidget");
            Rcs::JointWidget::create(graph, &graphLock);
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
            RMSG("Changing body childName");
            printf("Enter body to attach (parent): ");
            std::cin >> parentName;
            printf("Enter attachement (child) body: ");
            std::cin >> childName;

            RcsBody* child = RcsGraph_getBodyByName(graph, parentName.c_str());
            RcsBody* parent = RcsGraph_getBodyByName(graph, childName.c_str());
            RLOG(0, "Attaching \"%s\" (%s) to \"%s\" (%s)",
                 child ? child->name : "NULL", childName.c_str(),
                 parent ? parent->name : "NULL", parentName.c_str());

            HTr A_KV;
            HTr_setIdentity(&A_KV);
            HTr_invTransform(&A_KV, parent ? parent->A_BI : HTr_identity(),
                             child ? child->A_BI : HTr_identity());
            if (child && child->rigid_body_joints == true)
            {
              RcsJoint* jPtr = child->jnt;
              while (jPtr->next != NULL)
              {
                MatNd_set(graph->q, jPtr->jointIndex, 0, 0.0);
                jPtr = jPtr->next;
              }
            }
            bool success = RcsBody_attachToBody(graph, child, parent, &A_KV);
            RMSG("%s changing body attachement",
                 success ? "SUCCESS" : "FAILURE");

            RcsGraph_fprintJointRecursion(stdout, graph, parentName.c_str());
          }
          else if (kc->getAndResetKey('p'))
          {
            if (graph != NULL)
            {
              RLOGS(0, "m=%f   r_com=%f %f %f",
                    mass, r_com[0], r_com[1], r_com[2]);
            }
          }
          else if (kc->getAndResetKey('l'))
          {
            if (!argP.hasArgument("-edit"))
            {
              RLOG(0, "Please start this mode with option "
                   "\"-edit\", otherwise reloading will not "
                   "work. You can also close the Gui and reload "
                   "with key \"L\"");
            }
            else
            {
              RMSG("Reloading GraphNode");
              pthread_mutex_lock(&graphLock);
              viewer->removeNode(gn);
              gn = NULL;
              RcsGraph_destroy(graph);
              graph = RcsGraph_create(xmlFileName);
              comBase = RcsGraph_getBodyByName(graph, comRef);

              if (graph != NULL)
              {
                gn = new Rcs::GraphNode(graph);
                gn->toggleReferenceFrames();
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
            }
          }
          else if (kc->getAndResetKey('L'))
          {
            RMSG("Reloading GraphNode from %s", xmlFileName);
            pthread_mutex_lock(&graphLock);
            viewer->removeNode(gn);
            gn = NULL;
            RcsGraph_destroy(graph);
            graph = RcsGraph_create(xmlFileName);
            comBase = RcsGraph_getBodyByName(graph, comRef);

            if (graph != NULL)
            {
              gn = new Rcs::GraphNode(graph);
              gn->toggleReferenceFrames();
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
            RCSGRAPH_TRAVERSE_BODIES(graph)
            {
              if (BODY->jnt==NULL)
              {
                printf("   %s\n", BODY->name);
              }
            }

            std::string bdyName;
            printf("Enter body to merge: ");
            std::cin >> bdyName;
            pthread_mutex_lock(&graphLock);
            bool success = RcsBody_mergeWithParent(graph, bdyName.c_str());
            RMSG("%s merging body %s", success ? "SUCCEEDED" : "FAILED",
                 bdyName.c_str());
            pthread_mutex_unlock(&graphLock);
            viewer->removeNode(gn);
            gn = NULL;
            gn = new Rcs::GraphNode(graph);
            gn->toggleReferenceFrames();
            //pthread_mutex_unlock(&graphLock);
            viewer->add(gn);
            //pthread_mutex_lock(&graphLock);
            //pthread_mutex_unlock(&graphLock);
          }
        }   // KeyCatcher

        sprintf(hudText, "Graph \"%s\"\nDof: %d nJ: %d\n"
                "Forward kinematics step: %.1f ms",
                graph->xmlFile, graph->dof, graph->nJ, dtSim*1000.0);

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

        Timer_usleep(40000);
        loopCount++;
      }
      if (!valgrind)
      {
        RcsGuiFactory_shutdown();
        delete viewer;
      }
      RcsGraph_destroy(graph);
      break;
    }

    // ==============================================================
    // Gradient tests for graphs
    // ==============================================================
    case 3:
    {
      strcpy(xmlFileName, "LBR.xml");
      strcpy(directory, "config/xml/DexBot");
      argP.getArgument("-f", xmlFileName, "Configuration file name (default "
                       "is \"%s\")", xmlFileName);
      argP.getArgument("-dir", directory, "Configuration file directory "
                       "(default is \"%s\")", directory);

      if (argP.hasArgument("-h"))
      {
        RMSG("Mode %d: Rcs.exe -m %d -dir <graph-directory> -f "
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
        Rcs_gradientTestGraph(graph, q_test, true);
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
      Rcs::KeyCatcherBase::registerKey("Space", "Toggle pause");
      Rcs::KeyCatcherBase::registerKey("S", "Print out joint torques");
      Rcs::KeyCatcherBase::registerKey("j", "Disable joint limits");
      Rcs::KeyCatcherBase::registerKey("J", "Enable joint limits");
      Rcs::KeyCatcherBase::registerKey("g", "Toggle gravity compensation");

      double dt = 0.005, tmc = 0.01;
      double damping = 2.0;
      char hudText[2056] = "";
      char physicsEngine[32] = "Bullet";
      strcpy(xmlFileName, "gScenario.xml");
      strcpy(directory, "config/xml/DexBot");
      bool pause = argP.hasArgument("-pause", "Hit key for each iteration");
      bool posCntrl = argP.hasArgument("-posCntrl",
                                       "Enforce position control");
      bool skipGui = argP.hasArgument("-skipGui",
                                      "No joint angle command Gui");
      bool disableCollisions = argP.hasArgument("-disableCollisions",
                                                "Disable collisions between"
                                                " all rigid bodies");
      bool testCopy = argP.hasArgument("-copy", "Test physics copying");
      bool gravComp = argP.hasArgument("-gravComp",
                                       "Apply gravity compensation to torque joints");
      argP.getArgument("-physicsEngine", physicsEngine,
                       "Physics engine (default is \"%s\")", physicsEngine);
      argP.getArgument("-dt", &dt, "Simulation time step (default is %f)",
                       dt);
      argP.getArgument("-tmc", &tmc, "Gui filter, smaller is softer (default"
                       " is: %f)",
                       tmc);
      argP.getArgument("-damping", &damping,
                       "Joint torque damping (default is %f)", damping);
      argP.getArgument("-f", xmlFileName,
                       "Configuration file name (default is \"%s\")",
                       xmlFileName);
      argP.getArgument("-dir", directory,
                       "Configuration file directory (default is \"%s\")",
                       directory);
      getModel(directory, xmlFileName);

      if (argP.hasArgument("-h"))
      {
        RMSG("Mode %d: Rcs.exe -m %d -dir <graph-directory> -f "
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

      // physics simulation
      Rcs::PhysicsBase* sim = Rcs::PhysicsFactory::create(physicsEngine, graph,
                                                          physicsCfg);
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
      RcsGraph_computeGravityTorque(graph, T_gravity);
      MatNd_constMulSelf(T_gravity, -1.0);

      std::vector<RcsSensor*> jointTorqueSensor;

      RCSGRAPH_TRAVERSE_SENSORS(graph)
      {
        if (SENSOR->type == RCSSENSOR_JOINT_TORQUE)
        {
          jointTorqueSensor.push_back(SENSOR);
        }
      }

      RLOGS(5, "Found %zu joint torque sensors", jointTorqueSensor.size());

      // Viewer and Gui
      Rcs::KeyCatcher* kc = NULL;
      Rcs::Viewer* viewer = NULL;
      Rcs::HUD* hud = NULL;
      Rcs::JointWidget* jw = NULL;

      if (valgrind==false)
      {
        viewer = new Rcs::Viewer(!simpleGraphics, !simpleGraphics);
        Rcs::PhysicsNode* simNode = new Rcs::PhysicsNode(sim);
        viewer->add(simNode);
        hud = new Rcs::HUD();
        viewer->add(hud);
        kc = new Rcs::KeyCatcher();
        viewer->add(kc);
        viewer->runInThread(mtx);

        if (skipGui==false)
        {
          jw = Rcs::JointWidget::create(graph, mtx, q_des, graph->q);
        }
      }






      Timer* timer = Timer_create(dt);
      unsigned int loopCount = 0;

      while (runLoop)
      {

        if (pause==true)
        {
          RPAUSE_MSG("Hit enter to continue iteration %u", loopCount);
        }

        //////////////////////////////////////////////////////////////
        // Keycatcher
        /////////////////////////////////////////////////////////////////
        if (kc && kc->getAndResetKey('q'))
        {
          RMSGS("Quitting run loop");
          runLoop = false;
        }
        else if (kc && kc->getAndResetKey('p'))
        {
          RMSGS("Resetting physics");
          MatNd_setZero(q_dot_curr);
          pthread_mutex_lock(&graphLock);
          RcsGraph_setState(graph, q0, q_dot_curr);
          sim->reset();
          MatNd_copy(q_des, graph->q);
          MatNd_copy(q_des_f, graph->q);
          jw->reset(graph->q);
          pthread_mutex_unlock(&graphLock);
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
        else if (kc && kc->getAndResetKey('g'))
        {
          gravComp = !gravComp;
          RMSGS("Gravity compensation is %s", gravComp ? "ON" : "OFF");
        }
        else if (kc && kc->getAndResetKey(' '))
        {
          pause = !pause;
          RMSG("Pause modus is %s", pause ? "ON" : "OFF");
        }
        else if (kc && kc->getAndResetKey('S'))
        {
          RMSGS("Printing out joint torques");
          for (size_t i=0; i<jointTorqueSensor.size(); i++)
          {
            double jntTorque = jointTorqueSensor[i]->rawData->ele[0];
            RLOGS(0, "[%s]: Joint torque: %f",
                  jointTorqueSensor[i]->name, jntTorque);
          }

        }   // if (kc && ...)

        if (valgrind)
        {
          RLOG(1, "Step");
        }

        pthread_mutex_lock(&graphLock);



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

        // Dsired joint angles from Gui
        for (unsigned int i=0; i<graph->dof; i++)
        {
          q_des_f->ele[i] = (1.0-tmc)*q_des_f->ele[i] + tmc*q_des->ele[i];
        }

        if (gravComp==false)
        {
          MatNd_setZero(T_gravity);
        }

        sim->setControlInput(q_des_f, NULL, T_gravity);

        //////////////////////////////////////////////////////////////
        // call physics simulation and read new current state
        //////////////////////////////////////////////////////////////

        double dtSim = Timer_getTime();
        sim->simulate(dt, q_curr, q_dot_curr, NULL, NULL, true);
        dtSim = Timer_getTime() - dtSim;


        //////////////////////////////////////////////////////////////
        // Forward kinematics
        //////////////////////////////////////////////////////////////
        RcsGraph_setState(graph, q_curr, q_dot_curr);

        pthread_mutex_unlock(&graphLock);

        if (valgrind)
        {
          RLOG(1, "Step");
        }

        sprintf(hudText, "[%s]: Sim-step: %.1f ms\nSim time: %.1f sec\n"
                "Gravity compensation: %s",
                sim->getClassName(), dtSim*1000.0, sim->time(),
                gravComp ? "ON" : "OFF");

        if (hud != NULL)
        {
          hud->setText(hudText);
        }
        else
        {
          std::cout << hudText;
        }


        Timer_waitNoCatchUp(timer);

        loopCount++;

        if (loopCount>10 && (valgrind==true))
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
      Rcs::KeyCatcherBase::registerKey("T", "Run controller test");
      Rcs::KeyCatcherBase::registerKey("p", "Toggle pause");
      Rcs::KeyCatcherBase::registerKey("a", "Change IK algorithm");
      Rcs::KeyCatcherBase::registerKey("d", "Write q-vector to q.dat");
      Rcs::KeyCatcherBase::registerKey("D", "Set q-vector from file q.dat");
      Rcs::KeyCatcherBase::registerKey("n", "Reset to default state");
      Rcs::KeyCatcherBase::registerKey("C", "Toggle closest point lines");
      Rcs::KeyCatcherBase::registerKey("o", "Toggle distance calculation");
      Rcs::KeyCatcherBase::registerKey("m", "Manipulability null space");
      Rcs::KeyCatcherBase::registerKey("e", "Link generic body");

      int algo = 0;
      double alpha = 0.05, lambda = 1.0e-8, tmc = 0.1, dt = 0.01, dt_calc = 0.0;
      double jlCost = 0.0, dJlCost = 0.0;
      bool calcDistance = true;
      strcpy(xmlFileName, "cAction.xml");
      strcpy(directory, "config/xml/DexBot");
      char effortBdyName[256] = "";

      argP.getArgument("-algo", &algo, "IK algorithm: 0: left inverse, 1: "
                       "left inverse (Nakamura), 2: right inverse, 3: "
                       "experimental, 4: Solving according to Klein/Huang");
      argP.getArgument("-alpha", &alpha,
                       "Null space scaling factor (default is 0.05)");
      argP.getArgument("-lambda", &lambda, "Regularization");
      argP.getArgument("-f", xmlFileName);
      argP.getArgument("-dir", directory);
      argP.getArgument("-tmc", &tmc, "Filter time constant for sliders");
      argP.getArgument("-dt", &dt, "Sampling time interval");
      argP.getArgument("-staticEffort", effortBdyName, "Body to map static effort");
      bool ffwd = argP.hasArgument("-ffwd", "Feed-forward dx only");
      bool pause = argP.hasArgument("-pause", "Pause after each iteration");
      bool launchJointWidget = argP.hasArgument("-jointWidget",
                                                "Launch JointWidget");
      bool manipulability = argP.hasArgument("-manipulability",
                                             "Manipulability criterion in "
                                             "null space");

      if (argP.hasArgument("-h"))
      {
        printf("Resolved motion rate control test\n\n");
        break;
      }

      Rcs_addResourcePath(directory);

      // Create controller
      Rcs::ControllerBase controller(xmlFileName, true);
      Rcs::IkSolverRMR ikSolver(&controller);

      MatNd* dq_des  = MatNd_create(controller.getGraph()->dof, 1);
      MatNd* q_dot_des  = MatNd_create(controller.getGraph()->dof, 1);
      MatNd* a_des   = MatNd_create(controller.getNumberOfTasks(), 1);
      MatNd* x_curr  = MatNd_create(controller.getTaskDim(), 1);
      MatNd* x_dot_curr = MatNd_create(controller.getTaskDim(), 1);
      MatNd* x_des   = MatNd_create(controller.getTaskDim(), 1);
      MatNd* x_des_f = MatNd_create(controller.getTaskDim(), 1);
      MatNd* dx_des  = MatNd_create(controller.getTaskDim(), 1);
      MatNd* dH      = MatNd_create(1, controller.getGraph()->nJ);

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


      // Create visualization
      Rcs::Viewer* v           = NULL;
      Rcs::KeyCatcher* kc      = NULL;
      Rcs::GraphNode* gn       = NULL;
      Rcs::HUD* hud            = NULL;
      Rcs::BodyPointDragger* dragger = NULL;
      Rcs::VertexArrayNode* cn = NULL;
      char hudText[2056];

      if (valgrind==false)
      {
        v       = new Rcs::Viewer(!simpleGraphics, !simpleGraphics);
        kc      = new Rcs::KeyCatcher();
        gn      = new Rcs::GraphNode(controller.getGraph());
        hud     = new Rcs::HUD();
        dragger = new Rcs::BodyPointDragger();
        dragger->scaleDragForce(0.01);
        v->add(gn);
        v->add(hud);
        v->add(kc);
        v->add(dragger);

        if (controller.getCollisionMdl() != NULL)
        {
          cn = new Rcs::VertexArrayNode(controller.getCollisionMdl()->cp,
                                        osg::PrimitiveSet::LINES, "RED");
          cn->toggle();
          v->add(cn);
        }

        v->runInThread(mtx);

        // Launch the task widget
        if (ffwd == false)
        {
          Rcs::ControllerWidgetBase::create(&controller, a_des, x_des,
                                            x_curr, mtx);
        }
        else
        {
          // Launch the task widget
          MatNdWidget* mw = MatNdWidget::create(dx_des, x_curr,
                                                -1.0, 1.0, "dx",
                                                mtx);

          std::vector<std::string> labels;
          for (size_t id=0; id<controller.getNumberOfTasks(); id++)
          {
            for (unsigned int j=0; j<controller.getTaskDim(id); j++)
              labels.push_back(controller.getTaskName(id) +
                               std::string(": ") +
                               controller.getTask(id)->getParameter(j)->name);
          }

          mw->setLabels(labels);

          mw = MatNdWidget::create(a_des, a_des,
                                   0.0, 1.0, "activation",
                                   &graphLock);
          labels.clear();
          for (size_t id=0; id<controller.getNumberOfTasks(); id++)
          {
            labels.push_back(controller.getTaskName(id));
          }
          mw->setLabels(labels);
        }


        if (launchJointWidget==true)
        {
          Rcs::JointWidget::create(controller.getGraph(), mtx);
        }

        if (effortBdy != NULL)
        {
          std::vector<std::string> labels;
          MatNdWidget* mw = MatNdWidget::create(F_effort, F_effort,
                                                -1.0, 1.0, "F_effort", mtx);
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
          double clipLimit = 0.1;
          MatNd clipArr = MatNd_fromPtr(1, 1, &clipLimit);
          MatNd_saturateSelf(dx_des, &clipArr);
        }

        controller.computeJointlimitGradient(dH);

        if (calcDistance==true)
        {
          controller.computeCollisionCost();
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
            ikSolver.solveLeftInverse(dq_des, dx_des, dH, a_des, lambda);
            break;

          case 1:
            ikSolver.solveRightInverse(dq_des, dx_des, dH, a_des, lambda);
            break;

          default:
            RFATAL("No such algorithm; %d", algo);
        }

        MatNd_constMul(q_dot_des, dq_des, 1.0/dt);

        MatNd_addSelf(controller.getGraph()->q, dq_des);
        RcsGraph_setState(controller.getGraph(), NULL, q_dot_des);
        controller.computeX(x_curr);
        //controller.computeXp(x_dot_curr);

        dJlCost = -jlCost;
        jlCost = controller.computeJointlimitCost();
        dJlCost += jlCost;

        dt_calc = Timer_getTime() - dt_calc;

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
        else if (kc && kc->getAndResetKey('T'))
        {
          RLOGS(0, "Running controller test");
          controller.test(true);
        }
        else if (kc && kc->getAndResetKey('p'))
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
          RMSG("%s read q from file \"q.dat\"", success ? "Successfully" : "Failed to");
          RcsGraph_setState(controller.getGraph(), NULL, NULL);
        }
        else if (kc && kc->getAndResetKey('n'))
        {
          RMSG("Resetting");
          RcsGraph_setDefaultState(controller.getGraph());
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
          std::string bdyName;
          RMSG("Linking GenericBody");
          printf("Enter body to link against: ");
          std::cin >> bdyName;

          RcsBody* lb = RcsGraph_linkGenericBody(controller.getGraph(),
                                                 0, bdyName.c_str());
          RMSG("Linked against \"%s\"", lb ? lb->name : "NULL");
        }

        sprintf(hudText, "IK calculation: %.1f us\ndof: %d nJ: %d "
                "nqr: %d nx: %d\nJL-cost: %.6f dJL-cost: %.6f %s %s"
                "\nalgo: %d lambda:%g alpha: %g tmc: %.3f\n"
                "Manipulability index: %.6f\n"
                "Static effort: %.6f",
                1.0e6*dt_calc, controller.getGraph()->dof,
                ikSolver.nq, ikSolver.nqr,
                (int) controller.getActiveTaskDim(a_des),
                jlCost, dJlCost,
                ikSolver.getDeterminant()==0.0?"SINGULAR":"",
                ((dJlCost > 1.0e-8) && (MatNd_getNorm(dx_des) == 0.0)) ?
                "COST INCREASE" : "",
                algo, lambda, alpha, tmc,
                controller.computeManipulabilityCost(a_des),
                RcsGraph_staticEffort(controller.getGraph(),
                                      effortBdy, &F_effort3, NULL, NULL));

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
      MatNd_destroy(x_dot_curr);
      MatNd_destroy(x_des);
      MatNd_destroy(x_des_f);
      MatNd_destroy(dx_des);
      MatNd_destroy(dH);
      break;
    }


    // ==============================================================
    // Controller unit tests
    // ==============================================================
    case 6:
    {
      Rcs::KeyCatcherBase::registerKey("q", "Quit");
      Rcs::KeyCatcherBase::registerKey("p", "Toggle pause");

      strcpy(xmlFileName, "cAction.xml");
      strcpy(directory, "config/xml/GenericHumanoid");
      argP.getArgument("-f", xmlFileName);
      argP.getArgument("-dir", directory);
      bool pause = argP.hasArgument("-pause", "Pause after each iteration");
      bool skipGraphics = valgrind ||
                          argP.hasArgument("-noGraphics",
                                           "Skip graphics window");

      if (argP.hasArgument("-h"))
      {
        RMSG("Mode %d: Rcs.exe -m %d\n", mode, mode);
        printf("\n\tController gradient tests:\n");
        argP.print();
        Rcs::KeyCatcherBase::printRegisteredKeys();
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
          RPAUSE_MSG("Test failed");
        }

        if (kc && kc->getAndResetKey('q'))
        {
          runLoop = false;
        }
        else if (kc && kc->getAndResetKey('p'))
        {
          pause = !pause;
        }

        if (valgrind==true)
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

      }

      MatNd_destroy(q);
      MatNd_destroy(q_dot);

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

      int algo = 1;
      double alpha = 0.05;
      double lambda = 1.0e-8;
      double jlCost = 0.0, dJlCost = 0.0, eps=1.0e-5;
      strcpy(xmlFileName, "cAction.xml");
      strcpy(directory, "config/xml/DexBot");

      argP.getArgument("-algo", &algo);
      argP.getArgument("-alpha", &alpha);
      argP.getArgument("-lambda", &lambda);
      argP.getArgument("-f", xmlFileName);
      argP.getArgument("-dir", directory);
      argP.getArgument("-eps", &eps);
      bool pause = argP.hasArgument("-pause");

      Rcs_addResourcePath(directory);

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
        hud     = new Rcs::HUD(0,0,500,160);
        dragger = new Rcs::BodyPointDragger();
        v->add(gn);
        v->add(hud);
        v->add(kc);
        v->add(dragger);
        v->setCameraHomePosition(osg::Vec3d(2.5,  1.0, 1.8),
                                 osg::Vec3d(0.0, -0.2, 0.8),
                                 osg::Vec3d(0.0, 0.05, 1.0));
        v->runInThread(mtx);

        // Launch the activation widget
        std::vector<std::string> labels;
        MatNdWidget* mw = MatNdWidget::create(a_des, a_des,
                                              0.0, 1.0, "activation",
                                              &graphLock);
        for (size_t id=0; id<controller.getNumberOfTasks(); id++)
        {
          labels.push_back(controller.getTaskName(id));
        }
        mw->setLabels(labels);
      }

      unsigned int loopCount = 0;


      // Endless loop
      while (runLoop == true)
      {
        pthread_mutex_lock(&graphLock);
        double dt = Timer_getTime();

        // Set state to random and compute null space cost and gradient
        if (loopCount%10000==0)
        {
          MatNd_setRandom(controller.getGraph()->q, -M_PI, M_PI);
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

        sprintf(hudText, "IK calculation: %.2f ms\ndof: %d nJ: %d "
                "nqr: %d nx: %zu\nJL-cost: %.6f dJL-cost: %.6f %s %s\n"
                "dt=%.2f us\nalgo: %d lambda:%g alpha: %g",
                1.0e3*dt, controller.getGraph()->dof,
                ikSolver.nq, ikSolver.nqr,
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


        if ((dJlCost > eps) && (MatNd_getNorm(dx_des) == 0.0))
        {
          RPAUSE_MSG("COST INCREASE: %g", dJlCost);
        }

        loopCount++;
        Timer_usleep(1);
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
      Rcs::KeyCatcherBase::registerKey("p", "Print information to console");
      Rcs::KeyCatcherBase::registerKey("n", "Reset");
      Rcs::KeyCatcherBase::registerKey("o", "Set random pose");

      double lambda = 1.0e-8;
      double jlCost = 0.0, dJlCost=0.0, tmc=1.0, dt=0.005, kp_nullspace=0.5;
      double kp = 1.0;
      bool pause = false;
      strcpy(xmlFileName, "cAction.xml");
      strcpy(directory, "config/xml/DexBot");

      argP.getArgument("-lambda", &lambda, "Regularization (default: %f",
                       lambda);
      argP.getArgument("-f", xmlFileName);
      argP.getArgument("-dir", directory);
      argP.getArgument("-tmc", &tmc, "Gui filter time constant ([0 ... 1], "
                       "small is smooth, default is %f)", tmc);
      argP.getArgument("-dt", &dt);
      argP.getArgument("-kp_nullspace", &kp_nullspace);
      argP.getArgument("-kp", &kp);
      double kd = 2.0*sqrt(kp);
      argP.getArgument("-kd", &kd);
      bool ffwd = argP.hasArgument("-ffwd", "Feed forward only");

      Rcs_addResourcePath(directory);

      // Create controller
      Rcs::ControllerBase controller(xmlFileName);

      RCSGRAPH_TRAVERSE_JOINTS(controller.getGraph())
      {
        JNT->coupledTo = NULL;
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
      MatNd* xp_des_ik  = MatNd_create(nx, 1);
      MatNd* xpp_des_ik  = MatNd_create(nx, 1);
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
          Rcs::ControllerWidgetBase::create(&controller, a_des,
                                            x_des, x_curr, mtx);
        }
        else
        {
          // Launch the task widget
          MatNdWidget* mw = MatNdWidget::create(xpp_des, x_curr,
                                                -1000.0, 1000.0, "xpp_des",
                                                mtx);

          std::vector<std::string> labels;
          for (size_t id=0; id<controller.getNumberOfTasks(); id++)
          {
            for (unsigned int j=0; j<controller.getTaskDim(id); j++)
              labels.push_back(controller.getTaskName(id) +
                               std::string(": ") +
                               controller.getTask(id)->getParameter(j)->name);
          }

          mw->setLabels(labels);

          mw = MatNdWidget::create(a_des, a_des,
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
          MatNd_constMul(xpp_des, xp_des, -1.0);
          MatNd_constMul(xp_des, x_des_f, -1.0);

          for (unsigned int i=0; i<nx; i++)
          {
            x_des_f->ele[i] = tmc*x_des->ele[i] +
                              (1.0-tmc)*x_des_f->ele[i];
          }

          MatNd_addSelf(xp_des, x_des_f);
          MatNd_constMulSelf(xp_des, 1.0/dt);
          controller.computeDXp(xp_des_ik, xp_des);

          MatNd_addSelf(xpp_des, xp_des);
          MatNd_constMulSelf(xpp_des, 1.0/dt);
          controller.computeFfXpp(xpp_des_ik, xpp_des);

          controller.computeAx(ax_des, a_des, x_des_f,
                               xp_des_ik, xpp_des_ik, kpVec, kdVec);
        }
        else
        {
          controller.computeX(x_curr);
          controller.computeXp(xp_curr);
          controller.computeAx(ax_des, a_des, x_curr,
                               xp_curr, xpp_des, kpVec, kdVec);
        }

        controller.computeJointlimitGradient(dH);
        MatNd_constMulSelf(dH, -kp_nullspace);

        dragger->addJointTorque(dH, controller.getGraph());
        solver.solve(qpp_des, a_des, ax_des, dH, 0.0);

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
          MatNd_setZero(qp_des);
          MatNd_setZero(qpp_des);
          MatNd_copy(q_des, controller.getGraph()->q);
          MatNd_copy(qp_des, controller.getGraph()->q_dot);
          controller.computeX(x_des);
          MatNd_copy(x_des_f, x_des);
          MatNd_setZero(xp_des);
          MatNd_setZero(xpp_des);
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
        }

        sprintf(hudText, "IK calculation: %.1f us\n"
                "nx: %d\nJL-cost: %.6f dJL-cost: %.6f"
                "\nlambda:%g alpha: %g tmc: %g",
                1.0e6*dt_compute, (int) controller.getActiveTaskDim(a_des),
                jlCost, dJlCost,
                lambda, kp_nullspace, tmc);

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
        Timer_waitDT(0.01);
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
      MatNd_destroy(xp_des_ik);
      MatNd_destroy(xpp_des_ik);
      MatNd_destroy(xpp_des);
      MatNd_destroy(ax_des);
      MatNd_destroy(dH);
      break;




    }

    // ==============================================================
    // Distance function test
    // ==============================================================
    case 12:
    {
      Rcs::CmdLineParser argP(argc, argv);
      int shapeType1 = RCSSHAPE_SSL;
      int shapeType2 = RCSSHAPE_SSL;
      char textLine[2056] = "";

      argP.getArgument("-t1", &shapeType1, "Shape type for shape 1");
      argP.getArgument("-t2", &shapeType2, "Shape type for shape 2");

      if (argP.hasArgument("-h"))
      {
        RcsShape_fprintDistanceFunctions(stdout);
        break;
      }

      RcsBody* b1 = RALLOC(RcsBody);
      b1->A_BI = HTr_create();
      b1->Inertia = HTr_create();
      Vec3d_setRandom(b1->A_BI->org, -0.2, -0.1);
      b1->shape = RNALLOC(2, RcsShape*);
      b1->shape[0] = RcsShape_createRandomShape(shapeType1);
      RCHECK(b1->shape[0]);
      b1->shape[1] = NULL;

      RcsBody* b2 = RALLOC(RcsBody);
      b2->A_BI = HTr_create();
      b2->Inertia = HTr_create();
      Vec3d_setRandom(b1->A_BI->org, 0.1, 0.2);
      b2->shape = RNALLOC(2, RcsShape*);
      b2->shape[0] = RcsShape_createRandomShape(shapeType2);
      RCHECK(b2->shape[0]);
      b2->shape[1] = NULL;

      double I_closestPts[6];
      VecNd_setZero(I_closestPts, 6);
      double* cp0 = &I_closestPts[0];
      double* cp1 = &I_closestPts[3];
      double n01[3];
      Vec3d_setZero(n01);

      // Graphics
      Rcs::HUD* hud = NULL;
      Rcs::Viewer* viewer = NULL;
      Rcs::KeyCatcher* kc = NULL;

      if (!valgrind)
      {
        viewer = new Rcs::Viewer(!simpleGraphics, !simpleGraphics);

        // HUD
        hud = new Rcs::HUD();
        viewer->add(hud);

        // BodyNodes
        Rcs::BodyNode* bNd1 = new Rcs::BodyNode(b1);
        Rcs::BodyNode* bNd2 = new Rcs::BodyNode(b2);
        bNd1->setGhostMode(true, "RED");
        bNd2->setGhostMode(true, "GREEN");
        viewer->add(bNd1);
        viewer->add(bNd2);

        // TargetSetters
        bool sphTracker = b1->shape[0]->type==RCSSHAPE_POINT ? false : true;
        Rcs::TargetSetter* ts1 =
          new Rcs::TargetSetter(b1->A_BI->org, b1->A_BI->rot, 0.5, sphTracker);
        viewer->add(ts1);
        viewer->add(ts1->getHandler());
        sphTracker = b2->shape[0]->type==RCSSHAPE_POINT ? false : true;
        Rcs::TargetSetter* ts2 =
          new Rcs::TargetSetter(b2->A_BI->org, b2->A_BI->rot, 0.5, sphTracker);
        viewer->add(ts2);
        viewer->add(ts2->getHandler());

        // VertexArrayNode for distance
        Rcs::VertexArrayNode* cpLine = new Rcs::VertexArrayNode(I_closestPts, 2);
        cpLine->setColor("GREEN");
        cpLine->setPointSize(2.0);
        viewer->add(cpLine);

        Rcs::CapsuleNode* sphereCP0 = new Rcs::CapsuleNode(cp0, NULL, 0.015, 0.0);
        sphereCP0->makeDynamic(cp0);
        sphereCP0->setMaterial("RED");
        viewer->add(sphereCP0);

        Rcs::CapsuleNode* sphereCP1 = new Rcs::CapsuleNode(cp1, NULL, 0.015, 0.0);
        sphereCP1->makeDynamic(cp1);
        sphereCP1->setMaterial("RED");
        viewer->add(sphereCP1);

        // ArrowNode for normal vector
        Rcs::ArrowNode* normalArrow = new Rcs::ArrowNode(cp0, n01, 0.2);
        viewer->add(normalArrow);

        // KeyCatcher
        kc = new Rcs::KeyCatcher();
        viewer->add(kc);
        viewer->runInThread(mtx);
      }


      while (runLoop)
      {
        pthread_mutex_lock(&graphLock);
        double dt = Timer_getTime();
        double dist;

        dist = RcsShape_distance(b1->shape[0], b2->shape[0],
                                 b1->A_BI, b2->A_BI, cp0, cp1, n01);

        dt = Timer_getTime() - dt;
        pthread_mutex_unlock(&graphLock);

        std::stringstream hudText;
        sprintf(textLine, "Distance: D = % 3.1f mm took %3.2f usec\n",
                dist*1000.0, dt*1.0e6);
        hudText << textLine;
        if (hud)
        {
          hud->setText(hudText);
        }
        else
        {
          std::cout << hudText;
        }

        if (kc && kc->getAndResetKey('q'))
        {
          runLoop = false;
        }

        Timer_usleep(1000);
        RPAUSE_DL(4);

        if (valgrind)
        {
          runLoop = false;
        }

      } // while runLoop


      // Clean up
      RcsBody_destroy(b1);
      RcsBody_destroy(b2);

      if (viewer)
      {
        delete viewer;
      }

      break;
    }

    // ==============================================================
    // Depth first traversal test
    // ==============================================================
    case 13:
    {
      char dotFile[256], dotFileDfs[256], attachTo[256];
      strcpy(dotFile, "RcsGraph.dot");
      strcpy(attachTo, "");

      argP.getArgument("-dotFile", dotFile, "Dot file name");

      const char* fileExtension = strrchr(dotFile, '.');

      if (fileExtension==NULL)
      {
        strcpy(dotFileDfs, dotFile);
        strcat(dotFileDfs, "DFS");
      }
      else
      {
        snprintf(dotFileDfs, strlen(dotFile)-strlen(fileExtension)+1, dotFile);
        strcat(dotFileDfs, "DFS");
        strcat(dotFileDfs, fileExtension);
      }

      argP.getArgument("-attachTo", attachTo, "Body to attach graph");

      if (!argP.hasArgument("-f"))
      {
        strcpy(xmlFileName, "WAM-only.xml");
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
      RCHECK(RcsGraph_check(graph)==0);

      RcsGraph_writeDotFile(graph, "RcsGraph.dot");
      RcsGraph_writeDotFileDfsTraversal(graph, dotFile);

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
    // Test for Jacobian re-projection
    // ==============================================================
    case 14:
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

      MatNdWidget::create(invW, invW, 0.0, 1.0, "weighting");
      MatNdWidget::create(T, T, 0.0, 1.0, "torque");

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
    // Test for valid rotation matrices
    // ==============================================================
    case 15:
    {

      RLOG(0, "Trace 0: phi = %f", 180.0/M_PI*Math_acos(0.5 * (0.0 - 1.0)));
      RLOG(0, "Trace 1: phi = %f", 180.0/M_PI*Math_acos(0.5 * (1.0 - 1.0)));
      RLOG(0, "Trace 2: phi = %f", 180.0/M_PI*Math_acos(0.5 * (2.0 - 1.0)));
      RLOG(0, "Trace 3: phi = %f", 180.0/M_PI*Math_acos(0.5 * (3.0 - 1.0)));

      RLOG(0, "Trace 0.5: phi = %f", 180.0/M_PI*Math_acos(0.5 * (0.5 - 1.0)));
      RLOG(0, "Trace 1.5: phi = %f", 180.0/M_PI*Math_acos(0.5 * (1.5 - 1.0)));
      RLOG(0, "Trace 2.5: phi = %f", 180.0/M_PI*Math_acos(0.5 * (2.5 - 1.0)));

      RLOG(0, "Trace 1/3: phi = %f", 180.0/M_PI*Math_acos(0.5 * (1.0/3.0 - 1.0)));
      RLOG(0, "Trace 2/3: phi = %f", 180.0/M_PI*Math_acos(0.5 * (2.0/3.0 - 1.0)));
      // 0 .33 .66 1 1.33 1.66 2 2.33 2.66 3



      // double v[3], rm[3][3], scaling=0.0;
      // MatNd vm = MatNd_fromPtr(3, 1, v);
      // Vec3d_setZero(v);
      // argP.getArgument("-a", &v[0], "x-angle");
      // argP.getArgument("-b", &v[1], "y-angle");
      // argP.getArgument("-c", &v[2], "z-angle");
      // argP.getArgument("-s", &scaling, "scaling");
      // v[0] = M_PI*scaling;

      // MatNdWidget::create(&vm, &vm, -3.5, 3.5, "a-b-c");


      // while (runLoop)
      // {
      //   Mat3d_fromEulerAngles(rm, v);
      //   double trace = Mat3d_trace(rm);

      //   //RMSG("a=%f   b=%f   c=%f   trace=%f", v[0], v[1], v[2], trace);
      //   RMSG("trace = %f", trace);
      //   Mat3d_printCommentDigits("rm", rm, 6);
      //   Timer_waitDT(0.1);
      // }

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
    // 2D convex polygon
    // ==============================================================
    case 17:
    {
      double radius = 0.25;
      double height = 1.0;
      argP.getArgument("-radius", &radius, "Radius (default is %g)", radius);
      argP.getArgument("-height", &height, "Height (default is %g)", height);

      double poly[4][2];
      poly[0][0] = -radius;
      poly[0][1] = -0.5*height;
      poly[1][0] =  radius;
      poly[1][1] = -0.5*height;
      poly[2][0] =  radius;
      poly[2][1] =  0.5*height;
      poly[3][0] = -radius;
      poly[3][1] =  0.5*height;
      poly[4][0] = -radius;
      poly[4][1] = -0.5*height;
      MatNd polyArr = MatNd_fromPtr(5, 2, &poly[0][0]);

      Rcs::VertexArrayNode* vn = new Rcs::VertexArrayNode(&polyArr, osg::PrimitiveSet::LINE_STRIP);

      Rcs::Viewer* viewer = new Rcs::Viewer(!simpleGraphics, !simpleGraphics);
      viewer->add(vn);

      double pt[3], ang[3], Id[3][3];
      Mat3d_setIdentity(Id);
      Vec3d_setZero(ang);
      Vec3d_setZero(pt);
      Rcs::TargetSetter* ts = new Rcs::TargetSetter(pt, ang, 0.5, false);
      viewer->add(ts);

      Rcs::CapsuleNode* pn = new Rcs::CapsuleNode(pt, Id, 0.025, 0.0);
      pn->setMaterial("GREEN");
      pn->makeDynamic(pt);
      pn->setWireframe(false);
      viewer->add(pn);

      double cpPoly[3], nPoly[3];
      Vec3d_setZero(cpPoly);
      Vec3d_setZero(nPoly);
      Rcs::CapsuleNode* cn = new Rcs::CapsuleNode(cpPoly, Id, 0.025, 0.0);
      cn->makeDynamic(cpPoly);
      cn->setMaterial("RED");
      cn->setWireframe(false);
      viewer->add(cn);

      Rcs::ArrowNode* an = new Rcs::ArrowNode(cpPoly, nPoly, 0.2);
      viewer->add(an);


      char hudText[512] = "";
      Rcs::HUD* hud = new Rcs::HUD();
      viewer->add(hud);

      viewer->runInThread(mtx);

      while (runLoop)
      {
        pthread_mutex_lock(&graphLock);
        double d = Math_distPointConvexPolygon2D(pt, poly, 4, cpPoly, nPoly);
        sprintf(hudText, "d = %f", d);
        hud->setText(hudText);
        pthread_mutex_unlock(&graphLock);
        Timer_waitDT(0.01);
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
    Rcs::KeyCatcherBase::printRegisteredKeys();
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

  fprintf(stderr, "Thanks for using the Rcs libraries\n");

#if defined (_MSC_VER)
  if ((mode==0) || argP.hasArgument("-h"))
  {
    RPAUSE();
  }
#endif

  return 0;
}

