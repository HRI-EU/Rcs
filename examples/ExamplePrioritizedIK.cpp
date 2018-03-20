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


/*! \page Inverse_kinematics_example Inverse kinematics example
 *
 *  The source code of this example is in file example_prioritizedIK.cpp
 *
 *  \image html example_prioritizedIK.png
 *  This example has 2 modes (see command line option -m).
 *
 *  In mode 0, an inverse kinematics controller is instantiated,
 *  and a corresponding Gui allows to modify the task variables. Each task
 *  variable can be changed in its value, and in its activation. The check box
 *  allows to switch it on or off. The joint limit cost is projected into the
 *  last null space, it's magnitude is fixed in the code. This mode also has
 *  a mouse dragger. When pressing \"Shift\" and drag a point of the model
 *  with the mouse pointer, a virtual force is calculated according to the
 *  mouse force, and projected into the null space of the movement. Pressing
 *  \"Shift\" and \"Ctrl\" at the same time will create a larger force
 *  magnitude.
 *
 *  In mode 1, the same controller is instantiated. However, the Gui sliders
 *  now assign a velocity to the task variables, which will be integrated
 *  (without error feedback). This mode therefore allows to make sure that
 *  the controller equations work correctly: Assigning a velocity in one
 *  task variable should have no effect on the others.
 *
 *  The default command line arguments are:
 *  - -m \<0\>
 *  - -dir \<config/xml/DexBot\>
 *  - -f \<cAction.xml\>
 *  - -simpleGraphics
 *    Starts the graphics viewer with minimal settings (no anti-aliasing and
 *    shadows etc.). This is beneficial if the application is strted on a
 *    remote computer, or the computer has a slow graphics card.
 *  - -nomutex
 *    Disables mutex locking for the graphics viewer. This may result in some
 *    graphics artefacts, but does not compromise the calculation speed for
 *    computers with slow graphics cards.
 *  - -valgrind
 *    Runs the algorithm for a fixed number of steps without launching Guis and
 *    graphics viewer. This allows to run memory checks without considering the
 *    non-relevant parts.
 *
 *  For mode 0, an additional command line arguments is:
 *  - -plastic
 *    If this command line argument is given, the arm joint center positions
 *    of the left arm are set to the current pose if the mouse dragger is used.
 *    This makes the pose so-to-say \"plastic\". This mode can be toggled in
 *    the viewer window by pressing \"P\".
 *  - -algo \<0\>
 *    The example can select between 2 inverse kinematics algorithms: with algo
 *    being 0, a prioritized inverse kinematics algorithm is used. For algo
 *    equal 1, a standard resolved motion rate controller is selected. The
 *    algorithms can be switched by pressing the "a" key in the viewer window.
 *
 *  For mode 1, an additional command line arguments is:
 *  - -q2x
 *    Enables the joint space to task space re-projection.
 *    This mode can be toggled in the viewer window by pressing \"x\".
 *
 */

#include <ControllerBase.h>
#include <ControllerWidgetBase.h>
#include <IkSolverPrioRMR.h>

#include <Rcs_typedef.h>
#include <Rcs_macros.h>
#include <Rcs_timer.h>
#include <Rcs_resourcePath.h>
#include <Rcs_cmdLine.h>
#include <Rcs_math.h>
#include <Rcs_kinematics.h>
#include <Rcs_joint.h>

#include <RcsViewer.h>
#include <GraphNode.h>
#include <HUD.h>
#include <KeyCatcher.h>
#include <BodyPointDragger.h>
#include <Rcs_guiFactory.h>
#include <JointWidget.h>
#include <SegFaultHandler.h>

#include <MatNdWidget.h>

#include <libxml/tree.h>

#include <iostream>
#include <signal.h>

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
  int mode = 0, algo = 0, simpleGraphics = 0;
  bool pause = false;
  char xmlFileName[128] = "cAction.xml";
  char directory[128] = "config/xml/DexBot";

  // Ctrl-C callback handler
  signal(SIGINT, quit);

  // This initialize the xml library and check potential mismatches between
  // the version it was compiled for and the actual shared library used.
  LIBXML_TEST_VERSION

  // Parse command line arguments
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-dl", &RcsLogLevel, "Set Rcs log level");
  argP.getArgument("-m", &mode);
  argP.getArgument("-f", xmlFileName, "Controller config file name");
  argP.getArgument("-dir", directory, "Config file directory");
  bool valgrind = argP.hasArgument("-valgrind", "Start without Guis");
  bool noCpldJnts = argP.hasArgument("-noCoupledJoints", "Remove all joint couplings");
  simpleGraphics = argP.hasArgument("-simpleGraphics", "No fancy OpenGL");

  Rcs::KeyCatcherBase::registerKey("q", "Quit program");
  Rcs::KeyCatcherBase::registerKey(" ", "Toggle pause");

  // Initialize GUI and OSG mutex
  pthread_mutex_t graphLock;
  pthread_mutex_init(&graphLock, NULL);

  // Option without mutex for viewer
  pthread_mutex_t* mtx = &graphLock;
  if (argP.hasArgument("-nomutex"), "Don't sync viewer updates")
  {
    mtx = NULL;
  }

  Rcs_addResourcePath("config");
  Rcs_addResourcePath(directory);

  const char* hgr = getenv("SIT");
  if (hgr != NULL)
  {
    std::string meshDir = std::string(hgr) +
                          std::string("/Data/RobotMeshes/1.0/data");
    Rcs_addResourcePath(meshDir.c_str());
  }

  if ((!argP.hasArgument("-m")) && (argP.hasArgument("-h")))
  {
    mode = 2;
  }



  switch (mode)
  {

    // ==============================================================
    // IK with feed back
    // ==============================================================
    case 0:
    {
      Rcs::KeyCatcherBase::registerKey("a", "Switch between inverse "
                                       "kinematics algorithms: prioriti"
                                       "zed IK and resolved motion rate"
                                       " control");
      Rcs::KeyCatcherBase::registerKey("P", "Toggle \"plastic\" mode for "
                                       "left arm");
      bool plastic = argP.hasArgument("-plastic");
      argP.getArgument("-algo", &algo, "Algrithm: 0: PrioIK, 1: RMR");

      // Create controller
      Rcs::ControllerBase controller(xmlFileName, true);

      if (noCpldJnts==true)
      {
        RCSGRAPH_TRAVERSE_JOINTS(controller.getGraph())
        {
          JNT->coupledTo = NULL;
        }
        RcsGraph_setState(controller.getGraph(), NULL, NULL);
        RCHECK(RcsGraph_countCoupledJoints(controller.getGraph()) == 0);
      }


      Rcs::IkSolverPrioRMR ikSolver(&controller);

      MatNd* dq_1    = MatNd_create(controller.getGraph()->dof, 1);
      MatNd* dq_2    = MatNd_create(controller.getGraph()->dof, 1);
      MatNd* dq_2c   = MatNd_create(controller.getGraph()->dof, 1);
      MatNd* dq_3    = MatNd_create(controller.getGraph()->dof, 1);
      MatNd* dq_des  = MatNd_create(controller.getGraph()->dof, 1);
      MatNd* a_des   = MatNd_create(controller.getNumberOfTasks(), 1);
      MatNd* x_curr  = MatNd_create(controller.getTaskDim(), 1);
      MatNd* x_des   = MatNd_create(controller.getTaskDim(), 1);
      MatNd* x_des_f = MatNd_create(controller.getTaskDim(), 1);
      MatNd* dx_des  = MatNd_create(controller.getTaskDim(), 1);
      MatNd* dH      = MatNd_create(1, controller.getGraph()->nJ);
      MatNd* dH_jl   = MatNd_create(1, controller.getGraph()->nJ);
      MatNd* dH_coll = MatNd_create(1, controller.getGraph()->nJ);

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
        v->setCameraHomePosition(osg::Vec3d(2.5,  1.0, 1.8),
                                 osg::Vec3d(0.0, -0.2, 0.8),
                                 osg::Vec3d(0.0, 0.05, 1.0));
        v->runInThread(mtx);

        // Launch the task widget
        Rcs::ControllerWidgetBase::create(&controller, a_des, x_des,
                                          x_curr, &graphLock);



        if (argP.hasArgument("-jointWidget", "Launch joint widget"))
        {
          Rcs::JointWidget::create(controller.getGraph(), &graphLock);
        }
      }

      if (argP.hasArgument("-h"))
      {
        Rcs::CmdLineParser::print();
        Rcs::KeyCatcherBase::printRegisteredKeys();
        break;
      }

      unsigned int loopCount = 0;


      // Endless loop
      while (runLoop == true)
      {
        pthread_mutex_lock(&graphLock);
        double dt = Timer_getTime();

        for (unsigned int i=0; i<x_des->m; ++i)
        {
          x_des_f->ele[i] = 0.99*x_des_f->ele[i] + 0.01*x_des->ele[i];
        }
        controller.computeDX(dx_des, x_des_f);

        controller.computeJointlimitGradient(dH_jl);
        controller.computeCollisionGradient(dH_coll);

        // double s_jl = MatNd_scaleSelfToScalar(dH_jl, 1.0);
        // double s_coll = MatNd_scaleSelfToScalar(dH_coll, 1.0);

        MatNd_add(dH, dH_jl, dH_coll);
        // double s_ns = MatNd_scaleSelfToScalar(dH, 1.0);

        // RLOGS(1, "Scaling down null space: jl %f   coll %f   all %f",
        //       s_jl, s_coll, s_ns);

        if (valgrind==false)
        {
          dragger->addJointTorque(dH, controller.getGraph());
        }

        MatNd_constMulSelf(dH, 1.0e-2);

        ikSolver.solve(dq_des, dx_des, dH, a_des, 0.0);

        REXEC(10)
        {
          RLOGS(0, "dq   dh   %f   %f   ratio %f",
                MatNd_getNorm(dq_3),
                MatNd_getNorm(dH),
                MatNd_getNorm(dq_3)/MatNd_getNorm(dH));
          MatNd_reshape(dH, dH->n, dH->m);
          MatNd_printTwoArrays(dq_3, dH, 12);
          MatNd_reshape(dH, dH->n, dH->m);
        }

        MatNd_addSelf(controller.getGraph()->q, dq_des);
        RcsGraph_setState(controller.getGraph(), NULL, NULL);
        controller.computeX(x_curr);
        dt = Timer_getTime() - dt;
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
          RMSG("Switching to algorithm %s", algo==0?"PrioIK":"RMR");
        }
        else if (kc && kc->getAndResetKey('P'))
        {
          plastic = !plastic;
          RMSG("Switching plastic mode %s", plastic ? "on" : "off");
          if (plastic == false)
          {
            RCSGRAPH_TRAVERSE_JOINTS(controller.getGraph())
            {
              JNT->q0 = JNT->q_init;
            }
          }
        }
        else if (kc && kc->getAndResetKey(' '))
        {
          pause = !pause;
          RMSG("Pause modus is %s", pause ? "ON" : "OFF");
        }

        sprintf(hudText, "IK calculation: %.2f ms\ndof: %d nJ: %d\n"
                "nx1: %d nx2: %d\n"
                "IK-algorithm: %s\nJL-cost: %.12f",
                1.0e3*dt, controller.getGraph()->dof,
                controller.getGraph()->nJ,
                (int) ikSolver.getTaskDimForPriority(0, a_des),
                (int) ikSolver.getTaskDimForPriority(1, a_des),
                algo==0 ? "PrioIK" : "RMR",
                controller.computeJointlimitCost());

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

      MatNd_destroy(dq_1);
      MatNd_destroy(dq_2);
      MatNd_destroy(dq_2c);
      MatNd_destroy(dq_3);
      MatNd_destroy(dq_des);
      MatNd_destroy(a_des);
      MatNd_destroy(x_curr);
      MatNd_destroy(x_des);
      MatNd_destroy(x_des_f);
      MatNd_destroy(dx_des);
      MatNd_destroy(dH);
      MatNd_destroy(dH_jl);
      MatNd_destroy(dH_coll);
      break;
    }


    // ==============================================================
    // IK with ffwd velocities
    // ==============================================================
    case 1:
    {
      Rcs::KeyCatcherBase::registerKey("x", "Toggle joint space to task "
                                       "space re-projetion");
      Rcs::KeyCatcherBase::registerKey("o", "Set random state");

      bool enable_q2x = argP.hasArgument("-q2x", "Enable task-space "
                                         "re-projection");
      double lambda = 0.0;
      argP.getArgument("-lambda", &lambda, "Regularization for IK");

      // Create controller
      Rcs::ControllerBase controller(xmlFileName, true);
      Rcs::IkSolverPrioRMR ikSolver(&controller);

      unsigned int nx  = controller.getTaskDim();
      unsigned int nx1 = ikSolver.getTaskDimForPriority(1);
      unsigned int nx2 = ikSolver.getTaskDimForPriority(2);

      MatNd* dx2_prj = MatNd_create(1, nx);
      MatNd_reshape(dx2_prj, 1, nx2);
      MatNd* dq_des  = MatNd_create(controller.getGraph()->dof, 1);
      MatNd* dq_1    = MatNd_create(controller.getGraph()->dof, 1);
      MatNd* dq_2    = MatNd_create(controller.getGraph()->dof, 1);
      MatNd* dq_3    = MatNd_create(controller.getGraph()->dof, 1);
      MatNd* dq_2c   = MatNd_create(controller.getGraph()->dof, 1);
      MatNd* a_des   = MatNd_create(controller.getNumberOfTasks(), 1);
      MatNd* x_curr  = MatNd_create(nx, 1);
      MatNd* x_des   = MatNd_create(nx, 1);
      MatNd* dx_gui  = MatNd_create(nx, 1);
      MatNd* dx_des  = MatNd_create(nx, 1);
      MatNd* dH      = MatNd_create(1, controller.getGraph()->nJ);
      MatNd* dH_jl   = MatNd_create(1, controller.getGraph()->nJ);
      MatNd* dH_coll = MatNd_create(1, controller.getGraph()->nJ);
      MatNd* J2N1_tp = MatNd_create(controller.getGraph()->nJ, nx);

      controller.readActivationsFromXML(a_des);
      controller.computeX(x_curr);
      MatNd_copy(x_des, x_curr);

      // Create visualization
      Rcs::Viewer* v           = NULL;
      Rcs::KeyCatcher* kc      = NULL;
      Rcs::GraphNode* gn       = NULL;
      Rcs::HUD* hud            = NULL;
      char hudText[2056];

      if (valgrind==false)
      {
        v     = new Rcs::Viewer(!simpleGraphics, !simpleGraphics);
        kc    = new Rcs::KeyCatcher();
        gn    = new Rcs::GraphNode(controller.getGraph());
        hud   = new Rcs::HUD(0,0,500,160);
        v->add(gn);
        v->add(hud);
        v->add(kc);
        v->setCameraHomePosition(osg::Vec3d(2.5,  1.0, 1.8),
                                 osg::Vec3d(0.0, -0.2, 0.8),
                                 osg::Vec3d(0.0, 0.05, 1.0));
        v->runInThread(mtx);

        // Launch the dx widget
        MatNdWidget* mw = MatNdWidget::create(dx_gui, x_curr,
                                              -1.0, 1.0, "dx",
                                              &graphLock);

        std::vector<std::string> labels;
        for (size_t id=0; id<controller.getNumberOfTasks(); id++)
        {
          for (unsigned int j=0; j<controller.getTaskDim(id); j++)
          {
            labels.push_back(controller.getTaskName(id) +
                             std::string(": ") +
                             controller.getTask(id)->getParameter(j)->name);
          }
        }

        mw->setLabels(labels);

        // Launch the widget for the activations
        mw = MatNdWidget::create(a_des, a_des,
                                 0.0, 1.0, "activations", &graphLock);

        labels.clear();
        for (size_t id=0; id<controller.getNumberOfTasks(); id++)
        {
          labels.push_back(controller.getTaskName(id));
        }

        mw->setLabels(labels);
      }

      if (argP.hasArgument("-h"))
      {
        Rcs::CmdLineParser::print();
        Rcs::KeyCatcherBase::printRegisteredKeys();
        break;
      }


      unsigned int loopCount = 0;
      double jlCost = DBL_MAX, jlCost_prev = DBL_MAX;

      // Endless loop
      while (runLoop == true)
      {
        pthread_mutex_lock(&graphLock);

        // Copy the desired dx from the widget
        MatNd_copy(dx_des, dx_gui);

        // Add joint space to task space re-projection
        VecNd_subSelf(&dx_des->ele[nx1], dx2_prj->ele, nx2);

        // Start time measurement
        double dt = Timer_getTime();

        // Compute null space gradients. We scale it down so that the
        // max. absolute value doesn't get larger than a threshold.
        // This avoids linearization errors.
        controller.computeJointlimitGradient(dH);
        MatNd_scaleSelfToScalar(dH, 0.01);


        // Solve inverse kinematics and compute projection terms
        bool success = ikSolver.solve(dq_des, dx_des, dH, a_des, 0.0);

        // Here we compute the null space gradient projection back to
        // the policy space. It will be added to the dx vector in the
        // next step, which introduces a delay of 1 time step. Since
        // the changes in posture are small, we can accept it. To be
        // perfectly correct, we would need to compute the projection
        // matrices with the current state. This just requires
        // re-arranging the computation steps.
        MatNd_setZero(dx2_prj);

        if (enable_q2x == true)
        {
          MatNd_reshapeCopy(J2N1_tp, ikSolver.J2N1);
          MatNd_postMulDiagSelf(J2N1_tp, ikSolver.invWq);
          MatNd_transposeSelf(J2N1_tp);
          MatNd_reshape(dx2_prj, dH->m, J2N1_tp->n);
          MatNd_mul(dx2_prj, dH, J2N1_tp);
        }

        // Here we call the forward kinematics, since it doesn't make
        // the coupled joints consistent. This allows to check if the
        // equations work correctly.
        MatNd_addSelf(controller.getGraph()->q, dq_des);
        RcsGraph_computeForwardKinematics(controller.getGraph(),
                                          NULL, NULL);
        controller.computeX(x_curr);
        dt = Timer_getTime() - dt;
        pthread_mutex_unlock(&graphLock);

        // Catch viewer key presses
        if (kc && kc->getAndResetKey('q'))
        {
          runLoop = false;
        }
        else if (kc && kc->getAndResetKey(' '))
        {
          pause = !pause;
          RMSG("Pause modus is %s", pause ? "ON" : "OFF");
        }
        else if (kc && kc->getAndResetKey('x'))
        {
          enable_q2x = !enable_q2x;
          RMSG("enable_q2x modus is %s", enable_q2x ? "ON" : "OFF");
        }
        else if (kc && kc->getAndResetKey('o'))
        {
          RLOG(1, "Setting new random state");

          // Set joints to random state
          RCSGRAPH_TRAVERSE_JOINTS(controller.getGraph())
          {
            double offset = 0.0;
            if (RcsJoint_isTranslation(JNT))
            {
              offset = Math_getRandomNumber(-0.1, 0.1);
            }
            else
            {
              offset = Math_getRandomNumber(-M_PI, M_PI);
            }

            MatNd_set(controller.getGraph()->q, JNT->jointIndex, 0,
                      JNT->q0 + offset);
          }

          RcsGraph_setState(controller.getGraph(), NULL, NULL);
          jlCost_prev = DBL_MAX;
        }

        jlCost = controller.computeJointlimitCost();

        // Compare the consecutive costs. They must decrease, unless
        // there is a priority 1 policy applied. A small value is added
        // in the comparison to avoid numerical effects close to the
        // minimum.
        if ((jlCost > jlCost_prev+1.0e-8) && (enable_q2x == true))
        {
          RLOG(0, "JL-cost increase: diff is %.16f\ndH is",
               jlCost-jlCost_prev);
          MatNd_printTranspose(dH);
        }

        // Remember cost for next step
        jlCost_prev = jlCost;

        // Print out information to the head-up display
        sprintf(hudText, "IK calculation: %.2f ms\ndof: %d nJ: %d nx1: %d nx2: %d\n"
                "JL-cost: %.6f\nQ2X projection is %s",
                1.0e3*dt, controller.getGraph()->dof,
                controller.getGraph()->nJ, ikSolver.nx1, ikSolver.nx2,
                jlCost, enable_q2x ? "ON" : "OFF");

        if (success==false)
        {
          strcat(hudText, "\nSINGULAR!!!");
        }

        if (hud != NULL)
        {
          hud->setText(hudText);
        }
        else
        {
          std::cout << hudText;
        }

        // Early exit if valgrind option is set
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
      MatNd_destroy(dq_1);
      MatNd_destroy(dq_2);
      MatNd_destroy(dq_2c);
      MatNd_destroy(dq_3);
      MatNd_destroy(a_des);
      MatNd_destroy(x_curr);
      MatNd_destroy(x_des);
      MatNd_destroy(dx_des);
      MatNd_destroy(dx_gui);
      MatNd_destroy(dH);
      MatNd_destroy(dH_jl);
      MatNd_destroy(dH_coll);
      MatNd_destroy(J2N1_tp);
      MatNd_destroy(dx2_prj);
      break;
    }



    case 2:
    {
      Rcs::CmdLineParser::print();
      Rcs::KeyCatcherBase::printRegisteredKeys();
      printf("Mode 0: IK with feedback\n");
      printf("Mode 1: IK with ffwd velocities\n");
      break;
    }



    default:
    {
      RMSG("there is no mode %d", mode);
    }

  }   // switch(mode)



  xmlCleanupParser();
  fprintf(stderr, "Thanks for using the Rcs libraries\n");

  return 0;
}
