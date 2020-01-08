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

#include <RcsViewer.h>
#include <GraphNode.h>
#include <HUD.h>
#include <BodyPointDragger.h>
#include <JointWidget.h>
#include <MatNdWidget.h>
#include <KeyCatcher.h>
#include <PhysicsNode.h>
#include <HighGui.h>

#include <ControllerBase.h>
#include <PhysicsFactory.h>
#include <Rcs_macros.h>
#include <Rcs_cmdLine.h>
#include <Rcs_math.h>
#include <Rcs_guiFactory.h>
#include <Rcs_resourcePath.h>
#include <Rcs_timer.h>
#include <Rcs_typedef.h>
#include <Rcs_kinematics.h>
#include <Rcs_dynamics.h>
#include <Rcs_filters.h>
#include <Rcs_joint.h>
#include <Rcs_body.h>
#include <Rcs_utils.h>
#include <Rcs_parser.h>
#include <SegFaultHandler.h>

#include <iostream>


RCS_INSTALL_SEGFAULTHANDLER

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
  int mode = 0;
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-dl", &RcsLogLevel, "Debug level (default is 0)");
  argP.getArgument("-m", &mode, "Test mode");
  bool pause = argP.hasArgument("-pause", "Hit key for each iteration");
  bool valgrind = argP.hasArgument("-valgrind", "Without Guis and graphics");
  bool simpleGraphics = argP.hasArgument("-simpleGraphics", "OpenGL without "
                                         "shadows and anti-aliasing");

  // Initialize GUI and OSG mutex
  pthread_mutex_t graphLock;
  pthread_mutex_init(&graphLock, NULL);

  // Add search path for meshes etc.
  const char* hgr = getenv("SIT");
  if (hgr != NULL)
  {
    std::string meshDir = std::string(hgr) +
                          std::string("/Data/RobotMeshes/1.0/data");
    Rcs_addResourcePath(meshDir.c_str());
  }

  Rcs_addResourcePath("config");



  switch (mode)
  {

    // ==============================================================
    // Direct dynamics simulation using Newton-Euler equations
    // ==============================================================
    case 0:
    {
      Rcs::KeyCatcherBase::registerKey("e", "Toggle integrator");
      Rcs::KeyCatcherBase::registerKey("q", "Quit");
      Rcs::KeyCatcherBase::registerKey("d", "Scale down damping by 0.5");
      Rcs::KeyCatcherBase::registerKey("D", "Scale down damping by 2.0");
      Rcs::KeyCatcherBase::registerKey("o", "Toggle gravity compensation");
      Rcs::KeyCatcherBase::registerKey("h", "Toggle Coriolis compensation");
      Rcs::KeyCatcherBase::registerKey("t", "Set state to default");
      Rcs::KeyCatcherBase::registerKey("p", "Toggle pause");

      char xmlFileName[128] = "LBR.xml", directory[128] = "config/xml/DexBot";
      char integrator[32] = "Euler", hudText[4096] = "";
      double damping = 1.0, E = 0.0, dt = 0.001;
      bool gCancel = true, hCancel = false;

      argP.getArgument("-dt", &dt, "Integration step (default is %f sec)", dt);
      argP.getArgument("-damping", &damping, "Damping: default is %f", damping);
      argP.getArgument("-i", integrator, "Integrator: Euler (default) or "
                       "Fehlberg");
      argP.getArgument("-f", xmlFileName, "Configuration file name (default"
                       " is \"%s\")", xmlFileName);
      argP.getArgument("-dir", directory, "Configuration file directory "
                       "(default is \"%s\")", directory);

      Rcs_addResourcePath(directory);

      if (argP.hasArgument("-h"))
      {
        RMSG("Mode %d:\n\n\t"
             "ExampleKinetics -m %d -dir <%s> -f <%s>"
             " -damping <damping-value=%g> -dt <integration step=%g>"
             " -i <%s (Euler, Fehlberg)>"
             "\n\n\t- Creates a graph from an xml file)\n\t"
             "- Computes iteratively the direct dynamics and \n\t"
             "  integrates it over time using a Runge-Kutta Fehlberg \n\t"
             "  integrator with step size adaptation \n\t"
             "- Damping: acts as external force on dof velocities\n\t"
             "  (value of 0: energy conservation)\n\t",
             mode, mode, xmlFileName, directory, damping, dt, integrator);
        break;
      }


      RcsGraph* graph = RcsGraph_create(xmlFileName);
      RCHECK(graph);

      osg::ref_ptr<Rcs::BodyPointDragger> dragger;
      osg::ref_ptr<Rcs::HUD> hud;
      osg::ref_ptr<Rcs::KeyCatcher> kc;
      Rcs::Viewer* viewer            = NULL;
      if (!valgrind)
      {
        viewer = new Rcs::Viewer(!simpleGraphics, !simpleGraphics);
        Rcs::GraphNode* gn = new Rcs::GraphNode(graph);
        dragger = new Rcs::BodyPointDragger();
        hud = new Rcs::HUD();
        kc  = new Rcs::KeyCatcher();
        gn->toggleReferenceFrames();
        viewer->add(gn);
        viewer->add(dragger.get());
        viewer->add(hud.get());
        viewer->add(kc.get());
        viewer->runInThread(&graphLock);
      }

      double time = 0.0, dt_opt = dt;
      int nSteps = 1, n = graph->nJ;

      argP.getArgument("-dt", &dt);
      MatNd* z = MatNd_create(2*n, 1);
      VecNd_copy(z->ele, graph->q->ele, n);
      MatNd qp = MatNd_fromPtr(n, 1, &z->ele[n]);
      MatNd* F_ext = MatNd_create(n, 1);
      graph->userData = (void*) F_ext;
      MatNd* err = MatNd_create(2 * n, 1);
      for (int i = 0; i < n; i++)
      {
        err->ele[i]     = 1.0e-2;
        err->ele[i + n] = 1.0e-3;
      }

      Timer* timer = Timer_create(dt);
      double t0 = Timer_getTime();

      while (runLoop)
      {
        if (pause==true)
        {
          RPAUSE();
        }

        // Gravity compensation
        MatNd* MM        = MatNd_create(n, n);
        MatNd* F_gravity = MatNd_create(n, 1);
        MatNd* h         = MatNd_create(n, 1);
        MatNd* Fi        = MatNd_create(n, 1);
        E = RcsGraph_computeKineticTerms(graph, MM, h, F_gravity);

        // Joint speed damping: M Kv(qp_des - qp)
        MatNd_mul(Fi, MM, &qp);
        MatNd_constMulSelf(Fi, -damping);
        MatNd_addSelf(F_ext, Fi);

        if (gCancel)
        {
          MatNd_subSelf(F_ext, F_gravity);
          if (hCancel)
          {
            MatNd_addSelf(F_ext, h);
          }
        }

        MatNd_destroy(F_gravity);
        MatNd_destroy(h);
        MatNd_destroy(MM);
        MatNd_destroy(Fi);
        // End gravity compensation



        double dtCompute = Timer_getTime();

        pthread_mutex_lock(&graphLock);

        if (STREQ(integrator, "Fehlberg"))
        {
          nSteps = integration_t1_t2(Rcs_directDynamicsIntegrationStep,
                                     (void*) graph, 2*n, time, time+dt,
                                     &dt_opt, z->ele, z->ele, err->ele);
        }
        else if (STREQ(integrator, "Euler"))
        {
          integration_euler(Rcs_directDynamicsIntegrationStep,
                            (void*) graph, 2 * n, dt, z->ele, z->ele);
        }
        else
        {
          RFATAL("Unknonw integrator: \"%s\"", integrator);
        }

        pthread_mutex_unlock(&graphLock);

        dtCompute = Timer_getTime() - dtCompute;
        time += dt;

        sprintf(hudText, "Direct dynamics simulation\nTime: %.3f (%.3f)   "
                "dof: %d\ndt: %.3f dt_opt: %.3f\n%d steps took %.1f msec"
                "\n[%s]   Energy: %.3f Damping: %.1f\n"
                "G-comp: %d h-comp: %d\n",
                time, Timer_getTime() - t0, n, dt, dt_opt, nSteps,
                dtCompute*1.0e3, integrator, E, damping, gCancel, hCancel);

        if (kc.valid())
        {
          if (kc->getAndResetKey('q'))
          {
            runLoop = false;
          }
          else if (kc->getAndResetKey('d'))
          {
            damping *= 0.5;
            RLOGS(1, "Damping is %g", damping);
          }
          else if (kc->getAndResetKey('D'))
          {
            damping *= 2.0;
            if (damping == 0.0)
            {
              damping = 0.1;
            }
            RLOGS(1, "Damping is %g", damping);
          }
          else if (kc->getAndResetKey('o'))
          {
            gCancel = !gCancel;
            RLOGS(1, "gravity compensation is %d", gCancel);
          }
          else if (kc->getAndResetKey('h'))
          {
            hCancel = !hCancel;
            RLOGS(1, "h-vector compensation is %d", hCancel);
          }
          else if (kc->getAndResetKey('t'))
          {
            RLOGS(1, "Resetting state");
            RcsGraph_setDefaultState(graph);
            MatNd_setZero(z);
            VecNd_copy(z->ele, graph->q->ele, n);
          }
          else if (kc && kc->getAndResetKey('p'))
            {
              pause = !pause;
              RLOGS(1, "Pause modus is %s", pause ? "ON" : "OFF");
            }
          else if (kc->getAndResetKey('e'))
          {
            if (STREQ(integrator, "Euler"))
              {
                strcpy(integrator, "Fehlberg");
              }
            else
              {
                strcpy(integrator, "Euler");
              }
            RLOGS(1, "Integrator is %s", integrator);
          }
        } // if(kc)

        MatNd_setZero(F_ext);

        if (dragger.valid())
        {
          // Map external mouse force to joints: M = J^T F
          dragger->getJointTorque(F_ext, graph);
          MatNd_constMulSelf(F_ext, -10.0);

          if (hud.valid())
          {
          hud->setText(hudText);
        }
        }
        else
        {
          std::cout << hudText;
        }

        Timer_wait(timer);

        if ((valgrind==true) && (time>10.0*dt))
          {
            runLoop = false;
          }
      }

      MatNd_destroy(err);
      MatNd_destroy(z);
      MatNd_destroy(F_ext);
      RcsGraph_destroy(graph);
      Timer_destroy(timer);

      delete viewer;
      break;
    }

    // ==============================================================
    // Joint space inverse dynamics
    // ==============================================================
    case 1:
    {
      Rcs::KeyCatcherBase::registerKey("p", "Toggle pause");
      Rcs::KeyCatcherBase::registerKey("q", "Quit");
      Rcs::KeyCatcherBase::registerKey("o", "Reset system");

      char xmlFileName[128] = "LBR.xml", directory[128] = "config/xml/DexBot";
      char physicsEngine[32] = "Bullet";
      char pCfg[128] = "config/physics/vortex.xml";
      char hudText[2048] = "";
      double dt = 0.001, tmc = 0.1, vmax = 1.0;
 
      argP.getArgument("-dt", &dt, "Integration step (default is %f sec)", dt);
      argP.getArgument("-tmc", &tmc, "Slider filter time constant (default is"
                       " %f)", tmc);
      argP.getArgument("-vmax", &vmax, "Slider max. filter velocity (default "
                       "is %f)", vmax);
      argP.getArgument("-f", xmlFileName, "Configuration file name (default"
                       " is \"%s\")", xmlFileName);
      argP.getArgument("-dir", directory, "Configuration file directory "
                       "(default is \"%s\")", directory);
      argP.getArgument("-physics_config", pCfg, "Configuration file name"
                       " for physics (default is %s)", pCfg);
      argP.getArgument("-physicsEngine", physicsEngine,
                       "Physics engine (default is \"%s\")", physicsEngine);
      bool plot = argP.hasArgument("-plot", "Plot joint torques in HighGui");
      bool noJointLimits = argP.hasArgument("-noJointLimits", "No joint "
                                            "limits");
      bool noCollisions = argP.hasArgument("-noCollisions", "No collisions");

      double kp = 100.0;
      argP.getArgument("-kp", &kp, "Position gain (default is %f)", kp);
      double kd = 0.5*sqrt(4.0*kp);
      argP.getArgument("-kd", &kd, "Velocity gain (default is asymptotically"
                       " damped: %f)", kd);


      Rcs_addResourcePath(directory);

      if (argP.hasArgument("-h"))
      {
        break;
      }

      RcsGraph* graph = RcsGraph_create(xmlFileName);
      RCHECK(graph);
      MatNd* q_gui    = MatNd_clone(graph->q);
      MatNd* q_des    = MatNd_create(graph->dof,1);
      MatNd* qp_des   = MatNd_create(graph->dof,1);
      MatNd* qpp_des  = MatNd_create(graph->dof,1);
      MatNd* T_des    = MatNd_create(graph->dof,1);
      MatNd* M        = MatNd_create(graph->dof,graph->dof);
      MatNd* g        = MatNd_create(graph->dof,1);
      MatNd* h        = MatNd_create(graph->dof,1);
      MatNd* aq       = MatNd_create(graph->dof,1);
      MatNd* qp_ik    = MatNd_create(1, graph->dof);
      MatNd* T_limit  = MatNd_create(graph->dof,1);
      RcsGraph_getTorqueLimits(graph, T_limit, RcsStateFull);

      if (plot==true)
        {
          const double maxTorqueOfAll = MatNd_maxAbsEle(T_limit);
          Rcs::HighGui::configurePlot("Plot 1", 1, 5.0/dt, 
                                      -maxTorqueOfAll, maxTorqueOfAll);
        }

      MatNd* q_curr = graph->q;
      MatNd* qp_curr = graph->q_dot;

      Rcs::RampFilterND filt(q_gui->ele, tmc, vmax, dt, graph->dof);



      // Create physics simulation
      Rcs::PhysicsBase* sim = Rcs::PhysicsFactory::create(physicsEngine, 
                                                          graph, pCfg);
      RCHECK(sim);

      if (noCollisions)
      {
        RLOG(4, "Disabling collisions");
        sim->disableCollisions();
      }

      if (noJointLimits)
      {
        RLOG(4, "Disabling joint limits");
        sim->disableJointLimits();
      }


      // Viewer and Gui
      Rcs::Viewer* viewer = NULL;
      osg::ref_ptr<Rcs::HUD> hud;
      osg::ref_ptr<Rcs::KeyCatcher> kc;
      MatNdWidget* gui = NULL;

      if (!valgrind)
      {
        viewer = new Rcs::Viewer(!simpleGraphics, !simpleGraphics);
        osg::ref_ptr<Rcs::PhysicsNode> simNode = new Rcs::PhysicsNode(sim);
        viewer->add(simNode.get());
        Rcs::GraphNode* gnDes = new Rcs::GraphNode(sim->getGraph());
        gnDes->setGhostMode(true, "RED");
        viewer->add(gnDes);
        hud = new Rcs::HUD();
        viewer->add(hud.get());
        kc = new Rcs::KeyCatcher();
        viewer->add(kc.get());
        viewer->runInThread(&graphLock);

        gui = MatNdWidget::create(q_gui, q_curr, -3.0, 3.0, 
                                        "Joint angles", &graphLock);
       }




      Timer* rtClock = Timer_create(dt);

      // Endless loop
      while (runLoop)
        {
          if (pause==true)
            {
              RPAUSE();
            }

          double t_start = Timer_getSystemTime();

          pthread_mutex_lock(&graphLock);

          ////////////////////////////////////////////////////////////
          // Compute desired reference motion
          ////////////////////////////////////////////////////////////
          MatNd_reshape(qp_des, graph->dof, 1);
          MatNd_reshape(qpp_des, graph->dof, 1);
          filt.setTarget(q_gui->ele);
          filt.iterate(qpp_des->ele);
          filt.getPosition(q_des->ele);
          filt.getVelocity(qp_des->ele);

          ////////////////////////////////////////////////////////////
          // Compute inverse dynamics
          ////////////////////////////////////////////////////////////
#if 0
          // Mass matrix, gravity load and h-vector
          RcsGraph_computeKineticTerms(graph, M, h, g);

          // aq = -kp*(q-q_des)
          MatNd_reshape(aq, graph->dof, 1);
          MatNd_sub(aq, q_curr, q_des);
          RcsGraph_stateVectorToIKSelf(graph, aq);
          MatNd_constMulSelf(aq, -kp);

          // aq = aq  -kd*qp_curr + kd*qp_des
          RcsGraph_stateVectorToIK(graph, graph->q_dot, qp_ik);
          RcsGraph_stateVectorToIKSelf(graph, qp_des);
          RcsGraph_stateVectorToIKSelf(graph, qpp_des);
          MatNd_constMulAndAddSelf(aq, qp_ik, -kd);
          MatNd_constMulAndAddSelf(aq, qp_des, kd);

          // aq = aq + qpp_des
          MatNd_addSelf(aq, qpp_des);

          // Set the speed of the kinematic and constrained joints to zero
          RCSGRAPH_TRAVERSE_JOINTS(graph)
          {
            if ((JNT->ctrlType!=RCSJOINT_CTRL_TORQUE) ||
                (JNT->constrained==true))
              {
                MatNd_set(aq, JNT->jacobiIndex, 0, 0.0);
              }
          }

          // Add gravity and coriolis compensation: u += h + Fg
          // u = M*aq + h + g
          MatNd_reshape(T_des, graph->nJ, 1);
          MatNd_mul(T_des, M, aq);   // Tracking error
          MatNd_subSelf(T_des, h);   // Cancellation of coriolis forces
          MatNd_subSelf(T_des, g);   // Cancellation of gravity forces
#else
        Rcs::ControllerBase::computeInvDynJointSpace(T_des, graph, q_des, kp);
#endif

          // Check for torque limit violations
          unsigned int torqueLimitsViolated = 0;
          RcsGraph_stateVectorFromIKSelf(graph, T_des);
          for (unsigned int i=0;i<T_des->m; ++i)
            {
              if (fabs(T_des->ele[i]) > T_limit->ele[i])
                {
                  torqueLimitsViolated++;
                  RcsJoint* jidx = RcsGraph_getJointByIndex(graph, i,
                                                            RcsStateFull);
                  RLOG(1, "Torque limit violation at index %d (%s): %f > %f", 
                       i, jidx ? jidx->name : "NULL",
                       fabs(T_des->ele[i]), T_limit->ele[i]);
                }
            }
          double t_dyn = Timer_getSystemTime();
          
          ////////////////////////////////////////////////////////////
          // Simulate
          ////////////////////////////////////////////////////////////
          sim->setControlInput(q_des, qp_des, T_des);
          sim->simulate(dt, q_curr, qp_curr, NULL, NULL, true);
          RcsGraph_setState(graph, q_curr, qp_curr);

          pthread_mutex_unlock(&graphLock);
          double t_sim = Timer_getSystemTime();
          
          ////////////////////////////////////////////////////////////
          // Show some data in plots
          ////////////////////////////////////////////////////////////
          if (plot==true)
            {
              Rcs::HighGui::showPlot("Plot 1", 1, T_des->ele, graph->dof);
            }

          //////////////////////////////////////////////////////////////
          // Keycatcher and hud output
          /////////////////////////////////////////////////////////////////
        if (kc.valid())
        {
          if (kc->getAndResetKey('q'))
            {
              RMSGS("Quitting run loop");
              runLoop = false;
            }
          else if (kc->getAndResetKey('p'))
            {
              pause = !pause;
              RLOGS(1, "Pause modus is %s", pause ? "ON" : "OFF");
            }
          else if (kc->getAndResetKey('o'))
            {
              RMSGS("Resetting physics");
              pthread_mutex_lock(&graphLock);
              RcsGraph_setDefaultState(graph);
              sim->reset();
              gui->reset(graph->q);
              MatNd_copy(q_gui, graph->q);
              MatNd_copy(q_des, graph->q);
              filt.init(graph->q->ele);
              pthread_mutex_unlock(&graphLock);
          }
            }

          sprintf(hudText, "Time: %.3f (real: %.3f) dt: %.1f msec\ndt_dyn: "
                  "%.3f msec "
                  "dt_sim: %.3f msec\n%u torque limits violated\n", 
                  sim->time(), Timer_get(rtClock), 1000.0*dt,
                  1000.0*(t_dyn-t_start), 1000.0*(t_sim-t_start),
                  torqueLimitsViolated);

        if (hud.valid())
            {
              hud->setText(hudText);
            }
          else
            {
              std::cout << hudText;
              if (sim->time() > 10.0*dt)
                {
                  runLoop = false;
                }
            }


          Timer_wait(rtClock);

        }   // while(runLoop)

      RcsGuiFactory_shutdown();
      delete sim;

      RcsGraph_destroy(graph);
      Timer_destroy(rtClock);
      MatNd_destroy(q_gui);
      MatNd_destroy(q_des);
      MatNd_destroy(qp_des);
      MatNd_destroy(qpp_des);
      MatNd_destroy(T_des);
      MatNd_destroy(M);
      MatNd_destroy(g);
      MatNd_destroy(h);
      MatNd_destroy(aq);
      MatNd_destroy(qp_ik);
      MatNd_destroy(T_limit);
      delete viewer;

      break;
    }

    default:
    {
      RMSG("there is no mode %d", mode);
    }

  } // switch(mode)

  if (argP.hasArgument("-h"))
    {
      Rcs_printResourcePath();
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

  fprintf(stderr, "Thanks for using the Rcs libraries\n");

  return 0;
}
