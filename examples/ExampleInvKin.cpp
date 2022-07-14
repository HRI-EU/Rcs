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

#include "ExampleInvKin.h"

#include <Rcs_resourcePath.h>
#include <Rcs_cmdLine.h>
#include <Rcs_macros.h>
#include <Rcs_typedef.h>
#include <Rcs_body.h>
#include <Rcs_utils.h>
#include <Rcs_kinematics.h>
#include <Rcs_math.h>
#include <Rcs_timer.h>
#include <Rcs_sensor.h>
#include <Rcs_utilsCPP.h>
#include <Rcs_graphParser.h>
#include <ExampleFactory.h>
#include <IkSolverConstraintRMR.h>
#include <PhysicsFactory.h>



namespace Rcs
{

static ExampleFactoryRegistrar<ExampleIK> ExampleIK_("Inverse kinematics", "Dexbot with Task Interval");


ExampleIK::ExampleIK(int argc, char** argv) : ExampleBase(argc, argv),
  valgrind(false), simpleGraphics(false), nomutex(false), testLocale(false),
  mtx(NULL), algo(1), alpha(0.05), lambda(1.0e-8), tmc(0.1),
  dt(0.01), dt_calc(0.0), jlCost(0.0), dJlCost(0.0), clipLimit(DBL_MAX),
  det(0.0), scaleDragForce(0.01), calcDistance(false),
  ffwd(false), skipGui(false), pause(false), launchJointWidget(false),
  manipulability(false), cAvoidance(false),
  constraintIK(false), initToQ0(false), testCopying(false), noHud(false),
  posCntrl(false),
  controller(NULL), ikSolver(NULL),
  dq_des(NULL), q_dot_des(NULL), a_des(NULL), x_curr(NULL), x_physics(NULL),
  x_des(NULL), x_des_f(NULL), dx_des(NULL), dH(NULL),
  effortBdy(NULL), F_effort(NULL),
  sim(NULL), simController(NULL), simGraph(NULL), physicsFeedback(false),
  v(NULL), cGui(NULL), effortGui(NULL), dxGui(NULL), activationGui(NULL),
  jGui(NULL), loopCount(0)
{
  pthread_mutex_init(&graphLock, NULL);
  Vec3d_setZero(r_com);
  hudText[0] = '\0';
  Vec3d_setZero(r_com);
}

ExampleIK::~ExampleIK()
{
  clear();
  pthread_mutex_destroy(&graphLock);
}

void ExampleIK::clear()
{
  delete v;
  v = NULL;
  delete cGui;
  cGui = NULL;
  delete effortGui;
  effortGui = NULL;
  delete dxGui;
  dxGui = NULL;
  delete activationGui;
  activationGui = NULL;
  delete jGui;
  jGui = NULL;

  MatNd_destroy(dq_des);
  MatNd_destroy(q_dot_des);
  MatNd_destroy(a_des);
  MatNd_destroy(x_curr);
  MatNd_destroy(x_physics);
  MatNd_destroy(x_des);
  MatNd_destroy(x_des_f);
  MatNd_destroy(dx_des);
  MatNd_destroy(dH);
  dq_des = NULL;
  q_dot_des = NULL;
  a_des = NULL;
  x_curr = NULL;
  x_physics = NULL;
  x_des = NULL;
  x_des_f = NULL;
  dx_des = NULL;
  dH = NULL;

  delete simController;   // It's safe even if simController is NULL
  simController = NULL;
  delete ikSolver;
  ikSolver = NULL;

  Rcs_removeResourcePath(directory.c_str());
}

void ExampleIK::initParameters()
{
  xmlFileName = "cAction.xml";
  directory = "config/xml/DexBot";
  physicsCfg = "config/physics/physics.xml";
  integrator = "Fehlberg";
  localeStr = "de_DE.utf8";
}

void ExampleIK::parseArgs(CmdLineParser* argP)
{
  argP->getArgument("-valgrind", &valgrind, "Start without Guis and graphics");
  argP->getArgument("-simpleGraphics", &simpleGraphics, "OpenGL without fancy "
                    "stuff (shadows, anti-aliasing)");
  argP->getArgument("-nomutex", &nomutex, "Graphics without mutex");
  argP->getArgument("-i", &integrator, "Integrator for Newton-Euler "
                    "simulation (default is \"%s\")", integrator.c_str());
  argP->getArgument("-algo", &algo, "IK algorithm: 0: left inverse, 1: "
                    "right inverse (default is %d)", algo);
  argP->getArgument("-alpha", &alpha,
                    "Null space scaling factor (default is %g)", alpha);
  argP->getArgument("-lambda", &lambda, "Regularization (default is %g)",
                    lambda);
  argP->getArgument("-f", &xmlFileName, "Configuration file (default is %s)",
                    xmlFileName.c_str());
  argP->getArgument("-dir", &directory, "Configuration file directory "
                    "(default is %s)", directory.c_str());
  argP->getArgument("-tmc", &tmc, "Filter time constant for sliders: 1 is "
                    "unfiltered (default: %f)", tmc);
  argP->getArgument("-dt", &dt, "Sampling time interval (default: %f)", dt);
  argP->getArgument("-clipLimit", &clipLimit, "Clip limit for dx (default "
                    "is DBL_MAX)");
  argP->getArgument("-staticEffort", &effortBdyName,
                    "Body to map static effort (default: none)");
  argP->getArgument("-physics_config", &physicsCfg, "Configuration file name"
                    " for physics (default is %s)", physicsCfg.c_str());
  argP->getArgument("-physicsEngine", &physicsEngine, "Physics engine "
                    "(default is \"%s\")", physicsEngine.c_str());
  argP->getArgument("-scaleDragForce", &scaleDragForce, "Scale factor for"
                    " mouse dragger (default is \"%f\")", scaleDragForce);
  argP->getArgument("-ffwd", &ffwd, "Feed-forward dx only");
  argP->getArgument("-skipGui", &skipGui, "No GUIs, only viewer");
  argP->getArgument("-pause", &pause, "Pause after each iteration");
  argP->getArgument("-jointWidget", &launchJointWidget, "Launch JointWidget");
  argP->getArgument("-manipulability", &manipulability, "Manipulability "
                    "criterion in null space");
  argP->getArgument("-ca", &cAvoidance, "Collision avoidance in null space");
  argP->getArgument("-constraintIK", &constraintIK, "Use constraint IK solver");
  argP->getArgument("-setDefaultStateFromInit", &initToQ0, "Set the "
                    "joint center defaults from the initial state");
  argP->getArgument("-copy", &testCopying, "Test copying");
  argP->getArgument("-noHud", &noHud, "Don't show HUD");
  argP->getArgument("-posCntrl", &posCntrl, "Enforce position control "
                    "with physics");

  // Option to set locale - for parsing tests
  argP->getArgument("-testLocale", &testLocale, "Test locale");
  argP->getArgument("-locale", &localeStr, "Locale to be tested (default: %s)",
                    localeStr.c_str());
}

std::string ExampleIK::help()
{
  std::stringstream s;
  s << "  Resolved motion rate control test\n\n";
  s << "  Here are a few examples:\n";
  s << "  -dir config/xml/DexBot -f cAction.xml\n";
  s << "  -f config/xml/Examples/cContactGrasping.xml -algo 1 -lambda 0.001 -alpha 0\n";
  s << "  -f config/xml/Examples/cDistanceTask.xml\n";
  s << "  -f config/xml/Examples/cNormalAlign.xml -algo 1 -alpha 0.01 -lambda 0 -scaleDragForce 0.001\n";
  s << "  -f config/xml/Examples/cFace.xml\n";
  s << "  -f config/xml/Examples/cSoftPhysicsIK.xml -physicsEngine SoftBullet\n";
  s << "  -dir config/xml/Examples/ -f cSitToStand.xml -physicsEngine NewtonEuler -algo 1 -lamba 0 -dt 0.01\n";
  s << "  -dir config/xml/Dressing -f cRoboSleeve.xml -algo 1 -physicsEngine SoftBullet -dt 0.002\n";
  s << "  -dir config/xml/AvatarSkeleton -f cOpenSimWholeBody.xml -physicsEngine NewtonEuler -algo 1 -lambda 0 -i Euler\n\n";
  s << ControllerBase::printUsageToString(xmlFileName);
  s << Rcs::getResourcePaths();
  s << Rcs::CmdLineParser::printToString();
  s << Rcs::RcsGraph_printUsageToString(xmlFileName);
  return s.str();
}

bool ExampleIK::initAlgo()
{
  Rcs_addResourcePath(directory.c_str());

  if (nomutex)
  {
    mtx = NULL;
  }

  if (testLocale)
  {
    char* res = setlocale(LC_ALL, localeStr.c_str());
    if (res == NULL)
    {
      RLOG(1, "Failed to set locale \"%s\"", localeStr.c_str());
    }
    else
    {
      struct lconv* loc = localeconv();
      RLOG(1, "Locale successfully set to %s", res);
      RLOG(1, "Decimal character is %c", *(loc->decimal_point));
    }
  }

  controller = new ControllerBase(xmlFileName.c_str());

  if (testCopying)
  {
    Rcs::ControllerBase tmp(*controller);
    *controller = tmp;
  }

  if (initToQ0)
  {
    MatNd* q_init = MatNd_createLike(controller->getGraph()->q);
    RcsGraph_getInitState(controller->getGraph(), q_init);
    RcsGraph_changeDefaultState(controller->getGraph(), q_init);
  }

  if (constraintIK == true)
  {
    ikSolver = new Rcs::IkSolverConstraintRMR(controller);
  }
  else
  {
    ikSolver = new Rcs::IkSolverRMR(controller);
  }

  dq_des = MatNd_create(controller->getGraph()->dof, 1);
  q_dot_des = MatNd_create(controller->getGraph()->dof, 1);
  a_des = MatNd_create(controller->getNumberOfTasks(), 1);
  x_curr = MatNd_create(controller->getTaskDim(), 1);
  x_physics = MatNd_create(controller->getTaskDim(), 1);
  x_des = MatNd_create(controller->getTaskDim(), 1);
  x_des_f = MatNd_create(controller->getTaskDim(), 1);
  dx_des = MatNd_create(controller->getTaskDim(), 1);
  dH = MatNd_create(1, controller->getGraph()->nJ);

  controller->readActivationsFromXML(a_des);
  controller->computeX(x_curr);
  MatNd_copy(x_des, x_curr);
  MatNd_copy(x_des_f, x_curr);
  MatNd_copy(x_physics, x_curr);

  // Body for static effort null space gradient
  effortBdy = RcsGraph_getBodyByName(controller->getGraph(),
                                     effortBdyName.c_str());
  F_effort = MatNd_create(4, 1);   // 4-th element is gain

  // Overall COM
  Vec3d_setZero(r_com);

  // Physics engine
  if (PhysicsFactory::hasEngine(physicsEngine.c_str()))
  {
    simController = new Rcs::ControllerBase(*controller);
    simGraph = simController->getGraph();//RcsGraph_clone(controller.getGraph());

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

    sim = PhysicsFactory::create(physicsEngine.c_str(),
                                 simGraph, physicsCfg.c_str());

    if (sim == NULL)
    {
      Rcs::PhysicsFactory::print();
      RLOG_CPP(1, "Couldn't create physics \"" << physicsEngine << "\"");
      RcsGraph_destroy(simGraph);
      simGraph = NULL;
    }
    else
    {
      sim->setParameter(Rcs::PhysicsBase::Simulation, integrator.c_str(),
                        "Integrator", 0.0);
    }
  }

  return true;
}

void ExampleIK::initGraphics()
{
  Rcs::KeyCatcherBase::registerKey("q", "Quit");
  Rcs::KeyCatcherBase::registerKey("t", "Run controller test");
  Rcs::KeyCatcherBase::registerKey(" ", "Toggle pause");
  Rcs::KeyCatcherBase::registerKey("a", "Change IK algorithm");
  Rcs::KeyCatcherBase::registerKey("d", "Write q-vector to q.dat");
  Rcs::KeyCatcherBase::registerKey("D", "Set q-vector from file q.dat");
  Rcs::KeyCatcherBase::registerKey("n", "Reset to default state");
  Rcs::KeyCatcherBase::registerKey("C", "Toggle closest point lines and COM");
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

  if (valgrind)
  {
    return;
  }




  v = new Rcs::Viewer(!simpleGraphics, !simpleGraphics);
  kc = new Rcs::KeyCatcher();
  gn = new Rcs::GraphNode(controller->getGraph());

  if (!noHud)
  {
    hud = new Rcs::HUD();
    v->add(hud);
  }
  comNd = new Rcs::SphereNode(r_com, 0.05);
  comNd->makeDynamic(r_com);
  comNd->setMaterial("RED");
  v->add(comNd);

  dragger = new Rcs::BodyPointDragger();
  dragger->scaleDragForce(scaleDragForce);
  v->add(gn.get());
  v->add(kc.get());
  v->add(dragger.get());

  if (sim)
  {
    simNode = new Rcs::PhysicsNode(sim);
    gn->setGhostMode(true, "RED");
    gn->hide();
    v->add(simNode.get());
  }

  if (controller->getCollisionMdl() != NULL)
  {
    cn = new Rcs::VertexArrayNode(controller->getCollisionMdl()->cp,
                                  osg::PrimitiveSet::LINES, "RED");
    cn->toggle();
    v->add(cn.get());
  }

  v->runInThread(mtx);
}

void ExampleIK::initGuis()
{
  if (valgrind || skipGui)
  {
    return;
  }


  // Launch the task widget
  if (ffwd == false)
  {
    if (!skipGui)
    {
      if ((algo == 0) && (lambda > 0.0))
      {
        cGui = new ControllerGui(controller, a_des,
                                 ikSolver->getCurrentActivation(),
                                 x_des, x_curr, mtx);
      }
      else
      {
        if (sim)
        {
          // If mode 5 runs with a simulator, the GUI displays the
          // current values from physics.
          cGui = new ControllerGui(controller, a_des,
                                   x_des, x_physics, mtx);
        }
        else
        {
          cGui = new ControllerGui(controller, a_des,
                                   x_des, x_curr, mtx);
        }
      }
    }
  }
  else
  {
    if (!skipGui)
    {
      // Launch the task widget
      dxGui = new MatNdGui(dx_des, x_curr, -1.0, 1.0, "dx", mtx);

      std::vector<std::string> labels;
      for (size_t id = 0; id < controller->getNumberOfTasks(); id++)
      {
        for (unsigned int j = 0; j < controller->getTaskDim(id); j++)
          labels.push_back(controller->getTaskName(id) +
                           std::string(": ") +
                           controller->getTask(id)->getParameter(j).name);
      }

      dxGui->setLabels(labels);

      activationGui = new MatNdGui(a_des, a_des, 0.0, 1.0, "activation", &graphLock);
      labels.clear();
      for (size_t id = 0; id < controller->getNumberOfTasks(); id++)
      {
        labels.push_back(controller->getTaskName(id));
      }
      activationGui->setLabels(labels);
    }
  }


  if (launchJointWidget == true)
  {
    jGui = new JointGui(controller->getGraph(), mtx);
  }

  if (effortBdy)
  {
    std::vector<std::string> labels;
    effortGui = new MatNdGui(F_effort, F_effort, -1.0, 1.0, "F_effort", mtx);
    labels.push_back("Fx");
    labels.push_back("Fy");
    labels.push_back("Fz");
    labels.push_back("gain");
    effortGui->setLabels(labels);
  }

}

void ExampleIK::step()
{
  pthread_mutex_lock(&graphLock);
  RcsGraph_COG(controller->getGraph(), r_com);

  dt_calc = Timer_getTime();

  if (ffwd == false)
  {

    for (unsigned int i = 0; i < x_des_f->m; i++)
    {
      x_des_f->ele[i] = tmc * x_des->ele[i] +
                        (1.0 - tmc)*x_des_f->ele[i];
    }

    controller->computeDX(dx_des, x_des_f);
    MatNd clipArr = MatNd_fromPtr(1, 1, &clipLimit);
    MatNd_saturateSelf(dx_des, &clipArr);
  }

  controller->computeJointlimitGradient(dH);

  if (calcDistance == true)
  {
    controller->computeCollisionCost();
  }

  if (cAvoidance == true)
  {
    MatNd* dH_ca = MatNd_create(1, controller->getGraph()->dof);
    controller->getCollisionGradient(dH_ca);
    RcsGraph_limitJointSpeeds(controller->getGraph(), dH_ca,
                              1.0, RcsStateIK);
    MatNd_constMulSelf(dH_ca, 0.01);
    MatNd_addSelf(dH, dH_ca);
  }

  if (manipulability)
  {
    MatNd_setZero(dH);
    controller->computeManipulabilityGradient(dH, a_des);
    MatNd_constMulSelf(dH, 100.0);
  }

  if (effortBdy != NULL)
  {
    MatNd* W_ef = MatNd_create(controller->getGraph()->dof, 1);
    RCSGRAPH_TRAVERSE_JOINTS(controller->getGraph())
    {
      W_ef->ele[JNT->jointIndex] = 1.0 / JNT->maxTorque;
    }

    RcsGraph_stateVectorToIKSelf(controller->getGraph(), W_ef);
    MatNd* effortGrad = MatNd_create(1, controller->getGraph()->nJ);
    MatNd F_effort3 = MatNd_fromPtr(3, 1, F_effort->ele);
    RcsGraph_staticEffortGradient(controller->getGraph(), effortBdy,
                                  &F_effort3, W_ef, NULL, effortGrad);
    MatNd_destroy(W_ef);
    MatNd_constMulSelf(effortGrad, 1000.0*MatNd_get(F_effort, 3, 0));
    MatNd_addSelf(dH, effortGrad);

    MatNd_destroy(effortGrad);
  }

  MatNd_constMulSelf(dH, alpha);

  if (valgrind == false)
  {
    dragger->addJointTorque(dH, controller->getGraph());
  }

  switch (algo)
  {
    case 0:
      det = ikSolver->solveLeftInverse(dq_des, dx_des, dH, a_des, lambda);
      break;

    case 1:
      det = ikSolver->solveRightInverse(dq_des, dx_des, dH, a_des, lambda);
      break;

    default:
      RFATAL("No such algorithm; %d", algo);
  }

  MatNd_constMul(q_dot_des, dq_des, 1.0 / dt);

  MatNd_addSelf(controller->getGraph()->q, dq_des);
  RcsGraph_setState(controller->getGraph(), NULL, q_dot_des);
  bool poseOK = controller->checkLimits();
  controller->computeX(x_curr);

  if (sim)
  {
    sim->setControlInput(controller->getGraph()->q, q_dot_des, NULL);
    sim->simulate(dt, simGraph);
    RcsGraph_setState(simGraph, simGraph->q, simGraph->q_dot);
    simController->computeX(x_physics);
    if (physicsFeedback)
    {
      RcsGraph_setState(controller->getGraph(), simGraph->q, simGraph->q_dot);
    }

    else
    {
      RcsGraph* dstGraph = controller->getGraph();
      const RcsGraph* srcGraph = sim->getGraph();
      RCHECK(dstGraph->nSensors == srcGraph->nSensors);

      for (unsigned int i = 0; i < dstGraph->nSensors; ++i)
      {
        RCHECK(dstGraph->sensors[i].type == srcGraph->sensors[i].type);
        RcsSensor_copy(&dstGraph->sensors[i], &srcGraph->sensors[i]);
      }
    }
  }

  dJlCost = -jlCost;
  jlCost = controller->computeJointlimitCost();
  dJlCost += jlCost;

  dt_calc = Timer_getTime() - dt_calc;

  // Compute inside mutex, otherwise clicking activation boxes in the Gui
  // can lead to crashes.
  MatNd F_effort3 = MatNd_fromPtr(3, 1, F_effort->ele);
  double manipIdx = controller->computeManipulabilityCost(a_des);
  double staticEff = RcsGraph_staticEffort(controller->getGraph(),
                                           effortBdy, &F_effort3,
                                           NULL, NULL);
  pthread_mutex_unlock(&graphLock);


  char timeStr[64] = "";
  if (dt_calc > 10.0)   // show seconds
  {
    snprintf(timeStr, 64, "%.1f s", dt_calc);
  }
  else if (dt_calc > 0.0001)   // show milliseconds
  {
    snprintf(timeStr, 64, "%.3f ms", 1.0e3*dt_calc);
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
           timeStr, controller->getGraph()->dof,
           controller->getGraph()->nJ, ikSolver->getInternalDof(),
           (int)controller->getActiveTaskDim(a_des),
           jlCost, dJlCost,
           det == 0.0 ? "SINGULAR" : "",
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

  if ((valgrind == true) && (loopCount > 10))
  {
    runLoop = false;
  }

  if (pause == true)
  {
    RPAUSE();
  }

  loopCount++;
  Timer_waitDT(0.01);
}

void ExampleIK::handleKeys()
{
  if (!kc.valid())
  {
    return;
  }

  if (kc->getAndResetKey('q'))
  {
    runLoop = false;
  }
  else if (kc->getAndResetKey('H'))
  {
    if (hud.valid())
    {
      hud->toggle();
    }
  }
  else if (kc->getAndResetKey('a'))
  {
    algo++;
    if (algo > 1)
    {
      algo = 0;
    }

    RLOGS(0, "Switching to IK algorithm %d", algo);
  }
  else if (kc->getAndResetKey('t'))
  {
    RLOGS(0, "Running controller test");
    pthread_mutex_lock(&graphLock);
    controller->test(true);
    pthread_mutex_unlock(&graphLock);
  }
  else if (kc->getAndResetKey(' '))
  {
    pause = !pause;
    RMSG("Pause modus is %s", pause ? "ON" : "OFF");
  }
  else if (kc->getAndResetKey('d'))
  {
    RMSG("Writing q to file \"q.dat\"");
    MatNd* q_deg = MatNd_clone(controller->getGraph()->q);
    VecNd_constMulSelf(q_deg->ele, 180.0 / M_PI, q_deg->m);
    MatNd_toFile(q_deg, "q.dat");
    MatNd_destroy(q_deg);
  }
  else if (kc->getAndResetKey('D'))
  {
    bool success = MatNd_fromFile(controller->getGraph()->q, "q.dat");
    RMSG("%s read q from file \"q.dat\"",
         success ? "Successfully" : "Failed to");
    if (success)
    {
      RcsGraph_setState(controller->getGraph(), NULL, NULL);
    }
  }
  else if (kc->getAndResetKey('n'))
  {
    RMSG("Resetting");
    RcsGraph_setDefaultState(controller->getGraph());
    controller->computeX(x_curr);
    MatNd_copy(x_des, x_curr);
    MatNd_copy(x_des_f, x_curr);

    if (sim)
    {
      sim->reset(controller->getGraph()->q);
      MatNd_copy(x_physics, x_curr);
    }

    cGui->reset(a_des, x_curr);
  }
  else if (kc->getAndResetKey('k') && gn)
  {
    RMSG("Toggling GraphNode");
    gn->toggle();
  }
  else if (kc->getAndResetKey('C') && cn)
  {
    RMSG("Toggle closest points visualization");
    cn->toggle();
    comNd->toggle();
  }
  else if (kc->getAndResetKey('o'))
  {
    calcDistance = !calcDistance;
    RMSG("Distance calculation is %s", calcDistance ? "ON" : "OFF");
  }
  else if (kc->getAndResetKey('m'))
  {
    manipulability = !manipulability;
    RMSG("Manipulation index nullspace is %s",
         manipulability ? "ON" : "OFF");
  }
  else if (kc->getAndResetKey('e'))
  {
    Rcs::BodyNode* bNd = v->getBodyNodeUnderMouse<Rcs::BodyNode*>();
    if (bNd == NULL)
    {
      RMSG("No BodyNode found under mouse");
    }
    else
    {
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
        ok = RcsGraph_removeBody(controller->getGraph(), name.c_str(),
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
  }
  else if (kc->getAndResetKey('E'))
  {
    std::string name;
    RMSG("Linking GenericBody");
    printf("Enter body to link against: ");
    std::cin >> name;

    RcsBody* lb = RcsGraph_linkGenericBody(controller->getGraph(),
                                           0, name.c_str());
    RMSG("Linked against \"%s\"", lb ? lb->name : "NULL");
  }
  else if (kc->getAndResetKey('v'))
  {
    RcsGraph_fprintModelState(stdout, controller->getGraph(),
                              controller->getGraph()->q);
  }
  else if (kc->getAndResetKey('p'))
  {
    controller->print();
    controller->toXML("cAction.xml");
  }
  else if (kc->getAndResetKey('f'))
  {
    physicsFeedback = !physicsFeedback;
    RMSG("Physics feedback is %s", physicsFeedback ? "ON" : "OFF");
  }
  else if (kc->getAndResetKey('S'))
  {
    if (sim)
    {
      RMSG("Resetting simulation");
      sim->reset(controller->getGraph()->q);
    }
  }
}



static ExampleFactoryRegistrar<ExampleIK_ContactGrasping> ExampleIK_ContactGrasping_("Inverse kinematics", "Contact Grasping");

ExampleIK_ContactGrasping::ExampleIK_ContactGrasping(int argc, char** argv) : ExampleIK(argc, argv)
{
}

void ExampleIK_ContactGrasping::initParameters()
{
  ExampleIK::initParameters();
  xmlFileName = "cContactGrasping.xml";
  directory = "config/xml/Examples";
  algo = 1;
  lambda = 0.001;
  alpha = 0.0;
}


static ExampleFactoryRegistrar<ExampleIK_OSimWholeBody> ExampleIK_OSimWholeBody_("Inverse kinematics", "OpenSim whole-body");

ExampleIK_OSimWholeBody::ExampleIK_OSimWholeBody(int argc, char** argv) : ExampleIK(argc, argv)
{
}

void ExampleIK_OSimWholeBody::initParameters()
{
  ExampleIK::initParameters();
  xmlFileName = "cOpenSimWholeBody.xml";
  directory = "config/xml/AvatarSkeleton";
  algo = 1;
  lambda = 0.0;
  physicsEngine = "NewtonEuler";
  integrator = "Euler";
}


static ExampleFactoryRegistrar<ExampleIK_AssistiveDressing> ExampleIK_AssistiveDressing_("Inverse kinematics", "Assistive dressing");

ExampleIK_AssistiveDressing::ExampleIK_AssistiveDressing(int argc, char** argv) : ExampleIK(argc, argv)
{
}

void ExampleIK_AssistiveDressing::initParameters()
{
  ExampleIK::initParameters();
  xmlFileName = "cRoboSleeve.xml";
  directory = "config/xml/Dressing";
  algo = 1;
  dt = 0.002;
  physicsEngine = "SoftBullet";
}


static ExampleFactoryRegistrar<ExampleIK_StaticEffort> ExampleIK_StaticEffort_("Inverse kinematics", "Static effort");

ExampleIK_StaticEffort::ExampleIK_StaticEffort(int argc, char** argv) : ExampleIK(argc, argv)
{
}

void ExampleIK_StaticEffort::initParameters()
{
  ExampleIK::initParameters();
  effortBdyName = "sdh-base_R";
  algo = 1;
}

bool ExampleIK_StaticEffort::initAlgo()
{
  bool success = ExampleIK::initAlgo();
  MatNd_set(F_effort, 2, 0, -1.0);   // Downwards force
  success = RcsGraph_getModelStateFromXML(controller->getGraph()->q, controller->getGraph(), "StaticEffort", 0) && success;
  RcsGraph_setState(controller->getGraph(), NULL, NULL);
  controller->computeX(x_curr);
  MatNd_copy(x_des, x_curr);
  MatNd_copy(x_des_f, x_curr);
  MatNd_copy(x_physics, x_curr);
  return success;
}

std::string ExampleIK_StaticEffort::help()
{
  std::stringstream s;
  s << "  Static effort test:\n\n";
  s << "  The Gui shows the 3 force compoents that are applied to the right hand.\n";
  s << "  The 4th slider (gain) allows to adjust the amplification of the static effort\n";
  s << "  gradient to the null space of the system. A positive gain should decrease the\n";
  s << "  static effort, a negative value should increase it. The static effort is\n";
  s << "  displayed in the HUD\n\n";
  s << ControllerBase::printUsageToString(xmlFileName);
  s << Rcs::getResourcePaths();
  s << Rcs::CmdLineParser::printToString();
  s << Rcs::RcsGraph_printUsageToString(xmlFileName);
  return s.str();
}


}   // namespace Rcs
