/*******************************************************************************

  Copyright (c) 2022, Honda Research Institute Europe GmbH

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

#include "ExamplePhysics.h"

#include <Rcs_resourcePath.h>
#include <Rcs_cmdLine.h>
#include <Rcs_macros.h>
#include <Rcs_typedef.h>
#include <Rcs_dynamics.h>
#include <Rcs_sensor.h>
#include <Rcs_body.h>
#include <Rcs_utils.h>
#include <Rcs_graphParser.h>
#include <Rcs_kinematics.h>
#include <Rcs_BVHParser.h>
#include <Rcs_math.h>
#include <Rcs_timer.h>
#include <Rcs_utilsCPP.h>
#include <ExampleFactory.h>
#include <PhysicsFactory.h>
#include <KineticSimulation.h>

#include <RcsViewer.h>
#include <KeyCatcher.h>
#include <GraphNode.h>
#include <SphereNode.h>
#include <HUD.h>
#include <PPSGui.h>
#include <PPSSensorNode.h>

#include <sstream>



void ExampleInfo()
{
  Rcs::ExampleFactory::print();
}

namespace Rcs
{

static ExampleFactoryRegistrar<ExamplePhysics> ExamplePhysics_("Physics", "Dexbot");


ExamplePhysics::ExamplePhysics(int argc, char** argv) : ExampleBase(argc, argv)
{
  pthread_mutex_init(&graphLock, NULL);
  mtx = &graphLock;
  dt = 0.005;
  tmc = 0.01;
  damping = 2.0;
  shootMass = 1.0;
  Vec3d_setZero(gVec);
  hudText[0] = '\0';
  pause = false;
  posCntrl = false;
  skipGui = false;
  skipControl = false;
  disableCollisions = false;
  disableJointLimits = false;
  testCopy = false;
  withPPS = false;
  gravComp = false;
  resizeable = false;
  syncHard = false;
  seqSim = false;
  valgrind = false;
  simpleGraphics = false;
  bodyAdded = false;
  nomutex = false;

  loopCount = 0;

  graph = NULL;
  sim = NULL;

  q0 = NULL;
  q_des = NULL;
  q_des_f = NULL;
  q_curr = NULL;
  q_dot_curr = NULL;
  T_gravity = NULL;
  T_curr = NULL;

  kc = NULL;
  viewer = NULL;
  hud = NULL;
  simNode = NULL;

  jGui = NULL;

  timer = NULL;
}

ExamplePhysics::~ExamplePhysics()
{
  RLOG(5, "ExamplePhysics destructor");
  clear();
  pthread_mutex_destroy(&graphLock);
}

void ExamplePhysics::clear()
{
  RLOG(5, "Calling clear()");
  Timer_destroy(timer);
  timer = NULL;

  if (!valgrind)
  {
    delete jGui;
    jGui = NULL;
    delete viewer;
    viewer = NULL;
  }

  MatNd_destroy(q0);
  MatNd_destroy(q_des);
  MatNd_destroy(q_des_f);
  MatNd_destroy(q_curr);
  MatNd_destroy(q_dot_curr);
  MatNd_destroy(T_gravity);
  MatNd_destroy(T_curr);

  q0 = NULL;
  q_des = NULL;
  q_des_f = NULL;
  q_curr = NULL;
  q_dot_curr = NULL;
  T_gravity = NULL;
  T_curr = NULL;

  delete sim;
  sim = NULL;
  RcsGraph_destroy(graph);
  graph = NULL;

  Rcs_removeResourcePath(directory.c_str());

  RLOG(5, "Done clear()");
}

bool ExamplePhysics::initParameters()
{
  Vec3d_set(gVec, 0.0, 0.0, -RCS_GRAVITY);
  physicsEngine = "Bullet";
  integrator = "Fehlberg";
  physicsCfg = "config/physics/physics.xml";
  bgColor = "LIGHT_GRAYISH_GREEN";
  xmlFileName = "gScenario.xml";
  directory = "config/xml/DexBot";

  return true;
}

bool ExamplePhysics::parseArgs(CmdLineParser* argP)
{
  argP->getArgument("-nomutex", &nomutex, "Graphics without mutex");

  argP->getArgument("-pause", &pause, "Hit key for each iteration");
  argP->getArgument("-posCntrl", &posCntrl, "Enforce position control");
  argP->getArgument("-skipGui", &skipGui, "No joint angle command Gui");
  argP->getArgument("-skipControl", &skipControl,
                    "No commands considered in physics");
  argP->getArgument("-disableCollisions", &disableCollisions,
                    "Disable collisions between all rigid bodies");
  argP->getArgument("-disableJointLimits", &disableJointLimits,
                    "Disable all joint limits");
  argP->getArgument("-copy", &testCopy, "Test physics copying");
  argP->getArgument("-pps", &withPPS, "Launch PPS widgets");
  argP->getArgument("-gravComp", &gravComp, "Apply gravity compensation"
                    " to torque joints");
  argP->getArgument("-resizeable", &resizeable, "Adjust visualization "
                    "of shapes dynamically");
  argP->getArgument("-syncHard", &syncHard, "Try to sync with wall "
                    "clock time as hard as possible");
  argP->getArgument("-sequentialPhysics", &seqSim, "Physics simulation "
                    "step alternating with viewer's frame()");
  argP->getArgument("-valgrind", &valgrind, "Start without Guis and graphics");
  argP->getArgument("-simpleGraphics", &simpleGraphics, "OpenGL without fancy"
                    " stuff (shadows, anti-aliasing)");
  argP->getArgument("-physics_config", &physicsCfg, "Configuration file name"
                    " for physics (default is %s)", physicsCfg.c_str());
  argP->getArgument("-physicsEngine", &physicsEngine,
                    "Physics engine (default is \"%s\")", physicsEngine.c_str());
  argP->getArgument("-dt", &dt, "Simulation time step (default is %f)", dt);
  argP->getArgument("-tmc", &tmc, "Gui filter, smaller is softer (default"
                    " is: %f)", tmc);
  argP->getArgument("-damping", &damping,
                    "Joint torque damping (default is %f)", damping);
  argP->getArgument("-f", &xmlFileName, "Configuration file name (default "
                    "is \"%s\")", xmlFileName.c_str());
  argP->getArgument("-dir", &directory, "Configuration file directory "
                    "(default is \"%s\")", directory.c_str());
  argP->getArgument("-shootMass", &shootMass, "Mass of shooting ball"
                    "(default is \"%f\")", shootMass);
  argP->getArgument("-bgColor", &bgColor, "Background color (default is "
                    "\"%s\")", bgColor.c_str());
  argP->getArgument("-i", &integrator, "Integrator for Newton-Euler "
                    "simulation (default is \"%s\")", integrator.c_str());
  argP->getArgument("-gx", &gVec[0], "Gravity x (default is %f)", gVec[0]);
  argP->getArgument("-gy", &gVec[1], "Gravity y (default is %f)", gVec[1]);
  argP->getArgument("-gz", &gVec[2], "Gravity z (default is %f)", gVec[2]);

  return true;
}

std::string ExamplePhysics::help()
{
  std::stringstream s;
  s << "\tPhysics simulation test\n\n";
  s << "\tHere are a few examples:\n";
  s << "\t-dir config/xml/PPStest\n";
  s << "\t-skipGui -dir config/xml/Examples -f gGyro.xml -physicsEngine NewtonEuler -gz 0\n";
  s << "\t-skipGui -dir config/xml/Examples -f gGyro.xml -physicsEngine Bullet -gz 0\n";
  s << "\t-f config/xml/Examples/gHumanoidPendulum.xml -physicsEngine NewtonEuler -skipGui\n";
  s << "\t-dir config/xml/Examples/ -f cSitToStand.xml -physicsEngine NewtonEuler -skipGui\n";
  s << "\t-dir config/xml/Examples/ -f gSoftPhysics.xml -physicsEngine SoftBullet\n";
  s << "\t-dir config/xml/Examples/ -f cSoftPhysicsIK.xml -physicsEngine SoftBullet\n";
  s << "\t-dir config/xml/Examples/ -f gSoftShirtPerson.xml -physicsEngine SoftBullet\n";
  s << "\t-dir config/xml/WAM -f gScenario.xml -gc -damping 3 -physicsEngine NewtonEuler -gravComp\n";
  s << "\t-dir config/xml/Examples/ -f cWeldConstraint.xml -physicsEngine NewtonEuler -skipGui\n\n";

  s << PhysicsFactory::printToString();
  s << Rcs::getResourcePaths();
  s << Rcs::CmdLineParser::printToString();
  s << Rcs::RcsGraph_printUsageToString(xmlFileName);
  return s.str();
}

bool ExamplePhysics::initAlgo()
{
  if (nomutex)
  {
    mtx = NULL;
  }
  Rcs_addResourcePath(directory.c_str());

  graph = RcsGraph_create(xmlFileName.c_str());
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

  sim = PhysicsFactory::create(physicsEngine.c_str(), graph, physicsCfg.c_str());

  if (sim == NULL)
  {
    Rcs::PhysicsFactory::print();
    RFATAL("Couldn't create physics engine \"%s\"", physicsEngine.c_str());
  }

  if (testCopy == true)
  {
    RcsGraph* graph2 = graph;
    graph = RcsGraph_clone(graph2);
    RCHECK(graph);
    Rcs::PhysicsBase* sim2 = sim->clone(graph);
    delete sim;
    sim = sim2;

    RcsGraph_destroy(graph2);
  }

  sim->setParameter(Rcs::PhysicsBase::Simulation, integrator.c_str(),
                    "Integrator", 0.0);
  sim->setGravity(gVec);
  if (disableCollisions == true)
  {
    sim->disableCollisions();
  }

  // remember initial state for resetting simulation
  q0 = MatNd_clone(graph->q);
  q_des = MatNd_clone(graph->q);
  q_des_f = MatNd_clone(graph->q);
  q_curr = MatNd_clone(graph->q);
  q_dot_curr = MatNd_create(graph->dof, 1);
  T_gravity = MatNd_create(graph->dof, 1);
  T_curr = MatNd_create(graph->dof, 1);
  RcsGraph_computeGravityTorque(graph, NULL, T_gravity);
  MatNd_constMulSelf(T_gravity, -1.0);

  timer = Timer_create(dt);

  return true;
}

bool ExamplePhysics::initGraphics()
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

  // Viewer and Gui
  if (valgrind)
  {
    return true;
  }

  viewer = new Rcs::Viewer(!simpleGraphics, !simpleGraphics);
  viewer->setBackgroundColor(bgColor);
  simNode = new Rcs::PhysicsNode(sim, resizeable);
  viewer->add(simNode);
  hud = new Rcs::HUD();
  viewer->add(hud);
  kc = new Rcs::KeyCatcher();
  viewer->add(kc);

  if (seqSim == false)
  {
    viewer->runInThread(mtx);
  }
  else
  {
    simNode->setDebugDrawer(true);
  }

  if (withPPS == true)
  {
    sim->setEnablePPS(true);
    std::vector<Rcs::PPSGui::Entry> ppsEntries;
    double scaling = 1.0;

    RCSGRAPH_FOREACH_SENSOR(graph)
    {
      if (SENSOR->type == RCSSENSOR_PPS)
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

  return true;
}

bool ExamplePhysics::initGuis()
{
  if (skipGui == false)
  {
    jGui = new JointGui(graph, mtx, q_des, q_curr);

    //int guiHandle = Rcs::JointWidget::create(graph, mtx, q_des, q_curr);
    //void* ptr = RcsGuiFactory_getPointer(guiHandle);
    //jw = static_cast<Rcs::JointWidget*>(ptr);
  }

  return true;
}

void ExamplePhysics::step()
{
  if (pause == true)
  {
    RPAUSE_MSG("Hit enter to continue iteration %zu", loopCount);
  }

  pthread_mutex_lock(&graphLock);

  if (valgrind)
  {
    RLOG(1, "Step");
  }

  //////////////////////////////////////////////////////////////
  // Compute control input
  /////////////////////////////////////////////////////////////////
  if (gravComp == true)
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
    RcsGraph_computeGravityTorque(graph, NULL, T_gravity);
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
  MatNd* qp_des_f = MatNd_createLike(q_des_f);
  for (unsigned int i = 0; i < graph->dof; i++)
  {
    double q_prev = q_des_f->ele[i];
    q_des_f->ele[i] = (1.0 - tmc) * q_des_f->ele[i] + tmc * q_des->ele[i];
    qp_des_f->ele[i] = (q_des_f->ele[i] - q_prev) / dt;
  }

  sim->setControlInput(q_des_f, qp_des_f, T_gravity);
  MatNd_destroy(qp_des_f);

  //////////////////////////////////////////////////////////////
  // call physics simulation and read new current state
  //////////////////////////////////////////////////////////////

  double dtSim = Timer_getTime();
  sim->simulate(dt, graph, NULL, NULL, !skipControl);
  sim->getJointAngles(q_curr);
  REXEC(6)
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

  if (seqSim == true)
  {
    viewer->frame();
  }

  snprintf(hudText, 2056,
           "%s\n"
           "[%s]: Sim-step: %.1f ms\nSim time: %.1f (%.1f) sec\n"
           "Bodies: %d   Joints: %d\n"
           "Gravity compensation: %s\nDisplaying %s",
           sim->getGraph()->cfgFile,
           sim->getClassName(), dtSim * 1000.0, sim->time(),
           Timer_get(timer),
           sim->getGraph()->nBodies, sim->getGraph()->dof,
           gravComp ? "ON" : "OFF",
           simNode ? simNode->getDisplayModeStr() : "nothing");
  Rcs::KineticSimulation* kSim = dynamic_cast<Rcs::KineticSimulation*>(sim);
  if (kSim)
  {
    char neText[128] = "";
    if (kSim->getIntegrator() == "Euler")
    {
      snprintf(neText, 128, "\nIntegrator: Euler   Energy: %.4f",
               kSim->getEnergy());
    }
    else if (kSim->getIntegrator() == "Fehlberg")
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

  if ((loopCount > 10) && (valgrind == true))
  {
    runLoop = false;
  }
}

void ExamplePhysics::handleKeys()
{
  if (!kc)
  {
    return;
  }


  //////////////////////////////////////////////////////////////
  // Keycatcher
  /////////////////////////////////////////////////////////////////
  if (kc->getAndResetKey('q'))
  {
    RMSGS("Quitting run loop");
    runLoop = false;
  }
  else if (kc->getAndResetKey('D'))
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
  else if (kc->getAndResetKey('W'))
  {
    RMSGS("Creating JointWidget");
    jGui = new JointGui(graph, mtx, q_des, q_curr);
    //int guiHandle = Rcs::JointWidget::create(graph, mtx, q_des, q_curr);
    //void* ptr = RcsGuiFactory_getPointer(guiHandle);
    //jw = static_cast<Rcs::JointWidget*>(ptr);
  }
  else if (kc->getAndResetKey('Q'))
  {
    RcsGraph_fprintModelState(stdout, graph, graph->q, NULL, 0);
  }
  else if (kc->getAndResetKey('t'))
  {
    sim->check();
  }
  else if (kc->getAndResetKey('k'))
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
  else if (kc->getAndResetKey('p'))
  {
    RMSGS("Resetting physics");
    MatNd_setZero(q_dot_curr);
    RcsGraph_setState(graph, q0, q_dot_curr);
    sim->reset(q0);
    MatNd_copy(q_des, graph->q);
    MatNd_copy(q_des_f, graph->q);
    if (jGui != NULL)
    {
      if (bodyAdded == true)
      {
        RMSGS("Resetting physics after adding bodies only works if "
              "there is no JointWidget running. Please click it away "
              "before resetting the physics, or start the program with "
              " the command line option \"-skipGui\"");
      }
      else
      {
        //Gui->reset(graph->q);
      }
    }
  }
  else if (kc->getAndResetKey('o'))
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
    //if (jw != NULL) \todo
    //{
    //  jw->reset(graph->q);
    //}
  }
  else if (kc->getAndResetKey('j'))
  {
    RMSGS("Disabling joint limits");
    sim->setJointLimits(false);
  }
  else if (kc->getAndResetKey('J'))
  {
    RMSGS("Enabling joint limits");
    sim->setJointLimits(true);
  }
  else if (kc->getAndResetKey('u'))
  {
    gravComp = !gravComp;
    RMSGS("Gravity compensation is %s", gravComp ? "ON" : "OFF");
  }
  else if (kc->getAndResetKey('e'))
  {
    viewer->unlock();
    Rcs::BodyNode* bNd = viewer->getBodyNodeUnderMouse<Rcs::BodyNode*>();
    viewer->lock();
    if (bNd == NULL)
    {
      RMSG("No BodyNode found under mouse");
    }
    else
    {
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
  }
  else if (kc->getAndResetKey('a'))
  {
    viewer->unlock();
    Rcs::BodyNode* bNd = viewer->getBodyNodeUnderMouse<Rcs::BodyNode*>();
    viewer->lock();
    if (bNd == NULL)
    {
      RMSG("No BodyNode found under mouse");
    }
    else
    {
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
  }
  else if (kc->getAndResetKey('A'))
  {
    viewer->unlock();
    Rcs::BodyNode* bNd = viewer->getBodyNodeUnderMouse<Rcs::BodyNode*>();
    viewer->lock();
    if (bNd == NULL)
    {
      RMSG("No BodyNode found under mouse");
    }
    else
    {
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
  }
  else if (kc->getAndResetKey('m'))
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
    category = (Rcs::PhysicsBase::ParameterCategory)categoryInt;
    bool pSuccess = sim->setParameter(category, name.c_str(),
                                      type.c_str(), value);
    RMSGS("%s physics parameters",
          pSuccess ? "Successfully applied" : "Failed to apply");
  }
  else if (kc->getAndResetKey(' '))
  {
    pause = !pause;
    Timer_setTo(timer, sim->time());
    RMSG("Pause modus is %s", pause ? "ON" : "OFF");
  }
  else if (kc->getAndResetKey('l'))
  {
    RMSG("Reloading GraphNode from %s", xmlFileName.c_str());
    double t_reload = Timer_getSystemTime();
    int displayMode = simNode->getDisplayMode();
    viewer->removeInternal(simNode);
    simNode = NULL;
    double t_reload2 = Timer_getSystemTime();
    RcsGraph_destroy(graph);
    graph = RcsGraph_create(xmlFileName.c_str());
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
      sim = PhysicsFactory::create(physicsEngine.c_str(), graph,
                                   physicsCfg.c_str());
      RCHECK(sim);
      if (disableCollisions == true)
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

      MatNd_resizeCopy(q0, graph->q);
      MatNd_resizeCopy(q_des, graph->q);
      MatNd_resizeCopy(q_des_f, graph->q);
      MatNd_resizeCopy(q_curr, graph->q);
      MatNd_resizeCopy(q_dot_curr, graph->q_dot);
      T_curr = MatNd_realloc(T_curr, graph->dof, 1);
      MatNd_setZero(T_curr);
      T_gravity = MatNd_realloc(T_gravity, graph->dof, 1);
      RcsGraph_computeGravityTorque(graph, NULL, T_gravity);
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
         1000.0 * t_reload, 1000.0 * t_reload2);
  }
  else if (kc->getAndResetKey('S'))
  {
    RMSGS("Printing out sensors");

    RCSGRAPH_FOREACH_SENSOR(graph)
    {
      RcsSensor_fprint(stdout, SENSOR);
    }

    sim->print();
    RcsGraph_toXML(sim->getGraph(), "gSim.xml");
  }

}






// Rcs -m 4 -skipGui -dir config/xml/Examples -f gGyro.xml -physicsEngine NewtonEuler -gz 0
static ExampleFactoryRegistrar<ExamplePhysics_Gyro> ExamplePhysics_Gyro_("Physics", "Gyro");

ExamplePhysics_Gyro::ExamplePhysics_Gyro(int argc, char** argv) : ExamplePhysics(argc, argv)
{
}

// Here we overwrite the parseArgs() method, since the bools are overwritten in it. Otherwise,
// the skipGui flag cannot be changed.
bool ExamplePhysics_Gyro::initParameters()
{
  ExamplePhysics::initParameters();
  xmlFileName = "gGyro.xml";
  directory = "config/xml/Examples";
  physicsEngine = "NewtonEuler";
  skipGui = true;
  Vec3d_setZero(gVec);

  return true;
}






// Rcs -m 4 -dir config/xml/Examples/ -f gSoftPhysics.xml -physicsEngine SoftBullet
static ExampleFactoryRegistrar<ExamplePhysics_SoftBullet> ExamplePhysics_SoftBullet_("Physics", "Soft Bullet");

ExamplePhysics_SoftBullet::ExamplePhysics_SoftBullet(int argc, char** argv) : ExamplePhysics(argc, argv)
{
}

bool ExamplePhysics_SoftBullet::initParameters()
{
  ExamplePhysics::initParameters();
  xmlFileName = "gSoftPhysics.xml";
  directory = "config/xml/Examples";
  physicsEngine = "SoftBullet";

  return true;
}






// Rcs -m 4 -dir config/xml/Examples/ -f cSitToStand.xml -physicsEngine NewtonEuler -skipGui
static ExampleFactoryRegistrar<ExamplePhysics_SitToStand> ExamplePhysics_SitToStand_("Physics", "Sit-to-stand");

ExamplePhysics_SitToStand::ExamplePhysics_SitToStand(int argc, char** argv) : ExamplePhysics(argc, argv)
{
}

bool ExamplePhysics_SitToStand::initParameters()
{
  ExamplePhysics::initParameters();
  xmlFileName = "cSitToStand.xml";
  directory = "config/xml/Examples";
  physicsEngine = "NewtonEuler";

  return true;
}






// Rcs -m 4 -f config/xml/Examples/gHumanoidPendulum.xml -physicsEngine NewtonEuler -skipGui
static ExampleFactoryRegistrar<ExamplePhysics_HumanoidPendulum> ExamplePhysics_HumanoidPendulum_("Physics", "Humanoid pendulum");

ExamplePhysics_HumanoidPendulum::ExamplePhysics_HumanoidPendulum(int argc, char** argv) : ExamplePhysics(argc, argv)
{
}

bool ExamplePhysics_HumanoidPendulum::initParameters()
{
  ExamplePhysics::initParameters();
  xmlFileName = "gHumanoidPendulum.xml";
  directory = "config/xml/Examples";
  physicsEngine = "Mujoco";
  skipGui = true;

  return true;
}






// Rcs -m 4 -f config/xml/PPStest -f gScenario.xml -physicsEngine NewtonEuler -skipGui
static ExampleFactoryRegistrar<ExamplePhysics_PPStest> ExamplePhysics_PPStest("Physics", "Pressure sensors");

ExamplePhysics_PPStest::ExamplePhysics_PPStest(int argc, char** argv) : ExamplePhysics(argc, argv)
{
}

bool ExamplePhysics_PPStest::initParameters()
{
  ExamplePhysics::initParameters();
  xmlFileName = "gScenario.xml";
  directory = "config/xml/PPStest";
  physicsEngine = "Mujoco";
  skipGui = true;

  return true;
}





// Rcs -m 4 -f config/xml/Examples -f cWeldConstraint.xml -physicsEngine NewtonEuler -skipGui
static ExampleFactoryRegistrar<ExamplePhysics_WeldNewtonEuler> ExamplePhysics_WeldNewtonEuler("Physics", "Chain with weld constraints");

ExamplePhysics_WeldNewtonEuler::ExamplePhysics_WeldNewtonEuler(int argc, char** argv) : ExamplePhysics(argc, argv)
{
}

bool ExamplePhysics_WeldNewtonEuler::initParameters()
{
  ExamplePhysics::initParameters();
  xmlFileName = "cWeldConstraint.xml";
  directory = "config/xml/Examples";
  physicsEngine = "NewtonEuler";
  skipGui = true;

  return true;
}

}   // namespace Rcs
