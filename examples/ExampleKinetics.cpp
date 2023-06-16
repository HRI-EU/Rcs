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

#include "ExampleKinetics.h"

#include <GraphNode.h>
#include <JointWidget.h>
#include <PhysicsNode.h>
#include <HighGui.h>

#include <ExampleFactory.h>
#include <ControllerBase.h>
#include <PhysicsFactory.h>
#include <Rcs_macros.h>
#include <Rcs_cmdLine.h>
#include <Rcs_math.h>
#include <Rcs_resourcePath.h>
#include <Rcs_timer.h>
#include <Rcs_typedef.h>
#include <Rcs_kinematics.h>
#include <Rcs_dynamics.h>
#include <Rcs_joint.h>
#include <Rcs_body.h>
#include <Rcs_utils.h>
#include <Rcs_parser.h>

#include <iostream>



namespace Rcs
{

/*******************************************************************************
 * Direct dynamics simulation using Newton-Euler equations
 ******************************************************************************/
RCS_REGISTER_EXAMPLE(ExampleKinetics, "Kinetics", "Direct dynamics");

ExampleKinetics::ExampleKinetics(int argc, char** argv) : ExampleBase(argc, argv)
{
  Rcs::KeyCatcherBase::registerKey("e", "Toggle integrator");
  Rcs::KeyCatcherBase::registerKey("q", "Quit");
  Rcs::KeyCatcherBase::registerKey("d", "Scale down damping by 0.5");
  Rcs::KeyCatcherBase::registerKey("D", "Scale down damping by 2.0");
  Rcs::KeyCatcherBase::registerKey("o", "Toggle gravity compensation");
  Rcs::KeyCatcherBase::registerKey("h", "Toggle Coriolis compensation");
  Rcs::KeyCatcherBase::registerKey("t", "Set state to default");
  Rcs::KeyCatcherBase::registerKey("p", "Toggle pause");

  // Initialize GUI and OSG mutex
  pthread_mutex_init(&graphLock, NULL);
  viewer = NULL;
  hudText[0] = '\0';
  damping = 1.0;
  E = 0.0;
  dt = 0.001;
  dt_opt = dt;
  time = 0.0;
  t0 = 0.0;
  gCancel = true;
  hCancel = false;
  valgrind = false;
  pause = false;
  simpleGraphics = false;
  graph = NULL;
  timer = Timer_create(dt);
  z = NULL;
  F_ext = NULL;
  err = NULL;
}

ExampleKinetics::~ExampleKinetics()
{
  delete viewer;
  RcsGraph_destroy(graph);
  pthread_mutex_destroy(&graphLock);
  Timer_destroy(timer);
  MatNd_destroy(z);
  MatNd_destroy(F_ext);
  MatNd_destroy(err);
  Rcs_removeResourcePath(directory.c_str());
}

bool ExampleKinetics::initParameters()
{
  xmlFileName = "LBR.xml";
  directory = "config/xml/DexBot";
  integrator = "Euler";
  return true;
}

bool ExampleKinetics::parseArgs(Rcs::CmdLineParser* parser)
{
  parser->getArgument("-dt", &dt, "Integration step (default is %f sec)", dt);
  parser->getArgument("-damping", &damping, "Damping: default is %f", damping);
  parser->getArgument("-i", &integrator, "Integrator: Euler (default) or "
                      "Fehlberg");
  parser->getArgument("-f", &xmlFileName, "Configuration file name (default"
                      " is \"%s\")", xmlFileName.c_str());
  parser->getArgument("-dir", &directory, "Configuration file directory "
                      "(default is \"%s\")", directory.c_str());
  parser->getArgument("-pause", &pause, "Hit key for each iteration");
  parser->getArgument("-valgrind", &valgrind, "Without Guis and graphics");
  parser->getArgument("-simpleGraphics", &simpleGraphics, "OpenGL without "
                      "shadows and anti-aliasing");

  Timer_setDT(timer, dt);
  dt_opt = dt;

  return true;
}

bool ExampleKinetics::initAlgo()
{
  Rcs_addResourcePath(directory.c_str());

  graph = RcsGraph_create(xmlFileName.c_str());
  RCHECK(graph);

  const int n = graph->nJ;

  z = MatNd_create(2 * graph->dof, 1);
  MatNd_reshape(z, 2 * n, 1);
  MatNd zq = MatNd_fromPtr(n, 1, z->ele);
  RcsGraph_stateVectorToIK(graph, graph->q, &zq);

  F_ext = MatNd_create(graph->dof, 1);
  MatNd_reshape(F_ext, n, 1);

  err = MatNd_create(2 * graph->dof, 1);
  MatNd_reshape(err, n, 1);

  for (int i = 0; i < n; i++)
  {
    err->ele[i] = 1.0e-2;
    err->ele[i + n] = 1.0e-3;
  }

  t0 = Timer_getTime();

  return true;
}

bool ExampleKinetics::initGraphics()
{
  if (valgrind)
  {
    return true;
  }

  viewer = new Rcs::Viewer(!simpleGraphics, !simpleGraphics);
  Rcs::GraphNode* gn = new Rcs::GraphNode(graph);
  dragger = new Rcs::BodyPointDragger();
  hud = new Rcs::HUD();
  kc = new Rcs::KeyCatcher();
  gn->toggleReferenceFrames();
  viewer->add(gn);
  viewer->add(dragger.get());
  viewer->add(hud.get());
  viewer->add(kc.get());
  viewer->runInThread(&graphLock);

  return true;
}

void ExampleKinetics::handleKeys()
{
  if (!kc.valid())
  {
    return;
  }

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
    MatNd zq = MatNd_fromPtr(graph->nJ, 1, z->ele);
    RcsGraph_stateVectorToIK(graph, graph->q, &zq);
  }
  else if (kc && kc->getAndResetKey('p'))
  {
    pause = !pause;
    RLOGS(1, "Pause modus is %s", pause ? "ON" : "OFF");
  }
  else if (kc->getAndResetKey('e'))
  {
    if (integrator == "Euler")
    {
      integrator = "Fehlberg";
    }
    else
    {
      integrator = "Euler";
    }
    RLOG_CPP(1, "Integrator is " << integrator);
  }

}

std::string ExampleKinetics::help()
{
  std::string helpMsg;
  char txt[512];
  snprintf(txt, 512, "Example that creates a graph from an xml file\n\t"
           "- Computes iteratively the direct dynamics and \n\t"
           "  integrates it over time using an Euler or Runge-Kutta Fehlberg"
           " \n\t  integrator with step size adaptation \n\t"
           "- Damping: acts as external force on dof velocities\n\t"
           "  (value of 0: energy conservation)\n\t");

  helpMsg += ExampleBase::help();
  helpMsg += std::string(txt);
  return helpMsg;
}

void ExampleKinetics::step()
{
  if (pause == true)
  {
    RPAUSE();
  }

  DirDynParams params;
  params.graph = graph;
  params.F_ext = F_ext;

  // Gravity compensation
  const int n = graph->nJ;
  MatNd* MM = MatNd_create(n, n);
  MatNd* F_gravity = MatNd_create(n, 1);
  MatNd* h = MatNd_create(n, 1);
  MatNd* Fi = MatNd_create(n, 1);

  E = RcsGraph_computeKineticTerms(graph, NULL, MM, h, F_gravity);

  // Joint speed damping: M Kv(qp_des - qp)
  MatNd qp = MatNd_fromPtr(n, 1, &z->ele[n]);
  MatNd_mul(Fi, MM, &qp);
  MatNd_constMulSelf(Fi, -damping);
  MatNd_copy(F_ext, Fi);

  if (gCancel)
  {
    MatNd_subSelf(F_ext, F_gravity);
    if (hCancel)
    {
      MatNd_addSelf(F_ext, h);
    }
  }

  MatNd_destroyN(4, F_gravity, h, MM, Fi);
  // End gravity compensation

  if (dragger.valid())
  {
    // Map external mouse force to joints: M = J^T F
    MatNd* T_dragger = MatNd_create(graph->nJ, 1);
    dragger->getJointTorque(T_dragger, graph);
    MatNd_constMulSelf(T_dragger, -10.0);
    MatNd_addSelf(F_ext, T_dragger);
    MatNd_destroy(T_dragger);
  }


  double dtCompute = Timer_getTime();
  int nSteps = 1;

  pthread_mutex_lock(&graphLock);

  if (integrator == "Fehlberg")
  {
    nSteps = integration_t1_t2(Rcs_directDynamicsIntegrationStep,
                               (void*)&params, 2 * n, time, time + dt,
                               &dt_opt, z->ele, z->ele, err->ele);
  }
  else if (integrator == "Euler")
  {
    integration_euler(Rcs_directDynamicsIntegrationStep,
                      (void*)&params, 2 * n, dt, z->ele, z->ele);
  }
  else
  {
    RFATAL("Unknonw integrator: \"%s\"", integrator.c_str());
  }

  pthread_mutex_unlock(&graphLock);

  dtCompute = Timer_getTime() - dtCompute;
  time += dt;

  sprintf(hudText, "Direct dynamics simulation\nTime: %.3f (%.3f)   "
          "dof: %d\ndt: %.3f dt_opt: %.3f\n%d steps took %.1f msec"
          "\n[%s]   Energy: %.3f Damping: %.1f\nG-comp: %d h-comp: %d\n",
          time, Timer_getTime() - t0, n, dt, dt_opt, nSteps,
          dtCompute * 1.0e3, integrator.c_str(), E, damping, gCancel, hCancel);


  MatNd_setZero(F_ext);

  if (hud.valid())
  {
    hud->setText(hudText);
  }
  else
  {
    std::cout << hudText;
  }

  Timer_wait(timer);

  if ((valgrind == true) && (time > 10.0 * dt))
  {
    runLoop = false;
  }

}
















/*******************************************************************************
 * Joint space inverse dynamics
 ******************************************************************************/
RCS_REGISTER_EXAMPLE(ExampleJointSpaceInvDyn, "Kinetics", "Joint space inverse dynamics");

ExampleJointSpaceInvDyn::ExampleJointSpaceInvDyn(int argc, char** argv) : ExampleBase(argc, argv)
{
  Rcs::KeyCatcherBase::registerKey("p", "Toggle pause");
  Rcs::KeyCatcherBase::registerKey("q", "Quit");
  Rcs::KeyCatcherBase::registerKey("o", "Reset system");

  dt = 0.001;
  tmc = 0.1;
  vmax = 1.0;
  kp = 100.0;
  kd = 0.5 * sqrt(4.0 * kp);

  graph = NULL;
  q_gui = NULL;
  q_des = NULL;
  qp_des = NULL;
  qpp_des = NULL;
  T_des = NULL;
  M = NULL;
  g = NULL;
  h = NULL;
  aq = NULL;
  qp_ik = NULL;
  T_limit = NULL;

  sim = NULL;

  pause = false;
  valgrind = false;
  simpleGraphics = false;
  plot = false;
  noJointLimits = false;
  noCollisions = false;

  viewer = NULL;
  gui = NULL;
  filt = NULL;

  rtClock = NULL;

  pthread_mutex_init(&graphLock, NULL);
}

ExampleJointSpaceInvDyn::~ExampleJointSpaceInvDyn()
{
  pthread_mutex_destroy(&graphLock);
  Rcs_removeResourcePath(directory.c_str());
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

  delete sim;
  delete viewer;
  delete gui;
  delete filt;

  Timer_destroy(rtClock);
}

bool ExampleJointSpaceInvDyn::initParameters()
{
  xmlFileName = "LBR.xml";
  directory = "config/xml/DexBot";
  physicsEngine = "Bullet";
  physicsConfig = "config/physics/vortex.xml";
  return true;
}

bool ExampleJointSpaceInvDyn::parseArgs(CmdLineParser* parser)
{
  parser->getArgument("-f", &xmlFileName, "Configuration file name (default"
                      " is \"%s\")", xmlFileName.c_str());
  parser->getArgument("-dir", &directory, "Configuration file directory "
                      "(default is \"%s\")", directory.c_str());
  parser->getArgument("-physicsEngine", &physicsEngine,
                      "Physics engine (default is \"%s\")", physicsEngine.c_str());
  parser->getArgument("-physics_config", &physicsConfig, "Configuration file name"
                      " for physics (default is %s)", physicsConfig.c_str());
  parser->getArgument("-dt", &dt, "Integration step (default is %f sec)", dt);
  parser->getArgument("-tmc", &tmc, "Slider filter time constant (default is"
                      " %f)", tmc);
  parser->getArgument("-vmax", &vmax, "Slider max. filter velocity (default "
                      "is %f)", vmax);
  parser->getArgument("-kp", &kp, "Position gain (default is %f)", kp);
  kd = 0.5 * sqrt(4.0 * kp);
  parser->getArgument("-kd", &kd, "Velocity gain (default is asymptotically"
                      " damped: %f)", kd);
  parser->getArgument("-pause", &pause, "Hit key for each iteration");
  parser->getArgument("-valgrind", &valgrind, "Without Guis and graphics");
  parser->getArgument("-simpleGraphics", &simpleGraphics, "OpenGL without "
                      "shadows and anti-aliasing");
  parser->getArgument("-plot", &plot, "Plot joint torques in HighGui");
  parser->getArgument("-noJointLimits", &noJointLimits, "No joint "
                      "limits");
  parser->getArgument("-noCollisions", &noCollisions, "No collisions");

  return true;
}

bool ExampleJointSpaceInvDyn::initAlgo()
{
  Rcs_addResourcePath(directory.c_str());
  graph = RcsGraph_create(xmlFileName.c_str());
  RCHECK(graph);
  q_gui = MatNd_clone(graph->q);
  q_des = MatNd_create(graph->dof, 1);
  qp_des = MatNd_create(graph->dof, 1);
  qpp_des = MatNd_create(graph->dof, 1);
  T_des = MatNd_create(graph->dof, 1);
  M = MatNd_create(graph->dof, graph->dof);
  g = MatNd_create(graph->dof, 1);
  h = MatNd_create(graph->dof, 1);
  aq = MatNd_create(graph->dof, 1);
  qp_ik = MatNd_create(1, graph->dof);
  T_limit = MatNd_create(graph->dof, 1);
  RcsGraph_getTorqueLimits(graph, T_limit, RcsStateFull);

  sim = Rcs::PhysicsFactory::create(physicsEngine.c_str(), graph,
                                    physicsConfig.c_str());
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

  filt = new Rcs::RampFilterND(q_gui->ele, tmc, vmax, dt, graph->dof);

  rtClock = Timer_create(dt);

  return true;
}

bool ExampleJointSpaceInvDyn::initGraphics()
{
  if (valgrind)
  {
    return true;
  }

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

  return true;
}

bool ExampleJointSpaceInvDyn::initGuis()
{
  if (valgrind)
  {
    return true;
  }

  if (plot == true)
  {
    const double maxTorqueOfAll = MatNd_maxAbsEle(T_limit);
    RLOG(0, "Creating HighGui");
    Rcs::HighGui::configurePlot("Plot 1", 1, 5.0 / dt,
                                -maxTorqueOfAll, maxTorqueOfAll);
    RLOG(0, "Done creating HighGui");
  }

  gui = new Rcs::MatNdGui(q_gui, graph->q, -3.0, 3.0,
                          "Joint angles", &graphLock);

  return true;
}

void ExampleJointSpaceInvDyn::handleKeys()
{
  if (!kc.valid())
  {
    return;
  }

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
    filt->init(graph->q->ele);
    pthread_mutex_unlock(&graphLock);
  }

}

void ExampleJointSpaceInvDyn::step()
{
  if (pause == true)
  {
    RPAUSE();
  }

  MatNd* q_curr = graph->q;
  MatNd* qp_curr = graph->q_dot;

  double t_start = Timer_getSystemTime();

  pthread_mutex_lock(&graphLock);

  ////////////////////////////////////////////////////////////
  // Compute desired reference motion
  ////////////////////////////////////////////////////////////
  MatNd_reshape(qp_des, graph->dof, 1);
  MatNd_reshape(qpp_des, graph->dof, 1);
  filt->setTarget(q_gui->ele);
  filt->iterate(qpp_des->ele);
  filt->getPosition(q_des->ele);
  filt->getVelocity(qp_des->ele);

  ////////////////////////////////////////////////////////////
  // Compute inverse dynamics
  ////////////////////////////////////////////////////////////
  Rcs::ControllerBase::computeInvDynJointSpace(T_des, graph, q_des, kp);

  // Check for torque limit violations
  unsigned int torqueLimitsViolated = 0;
  RcsGraph_stateVectorFromIKSelf(graph, T_des);
  for (unsigned int i = 0; i < T_des->m; ++i)
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
  if (plot == true)
  {
    RLOG(10, "Start showplot");
    Rcs::HighGui::showPlot("Plot 1", 1, T_des->ele, graph->dof);
    RLOG(10, "Done showplot");
  }

  char hudText[512] = "";

  snprintf(hudText, 512, "Time: %.3f (real: %.3f) dt: %.1f msec\ndt_dyn: "
           "%.3f msec "
           "dt_sim: %.3f msec\n%u torque limits violated\n",
           sim->time(), Timer_get(rtClock), 1000.0 * dt,
           1000.0 * (t_dyn - t_start), 1000.0 * (t_sim - t_start),
           torqueLimitsViolated);

  if (hud.valid())
  {
    hud->setText(hudText);
  }
  else
  {
    std::cout << hudText;
    if (sim->time() > 10.0 * dt)
    {
      runLoop = false;
    }
  }


  Timer_wait(rtClock);
}


}   // namespace
