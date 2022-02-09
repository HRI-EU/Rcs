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
#include <Rcs_math.h>
#include <TaskFactory.h>
#include <IkSolverRMR.h>
#include <PhysicsNode.h>
#include <HUD.h>
#include <KeyCatcher.h>
#include <RcsViewer.h>
#include <Rcs_guiFactory.h>
#include <ControllerWidgetBase.h>
#include <SegFaultHandler.h>

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
static inline double computeBlendingPolynomial(double s)
{
  s = Math_clip(s, 0.0, 1.0);

  return 6.0*pow(s,5) - 15.0*pow(s, 4) + 10.0*pow(s, 3);
}

/*******************************************************************************
 *
 ******************************************************************************/
static void testBlendingLeftInverse(int argc, char** argv)
{
  // Parse command line arguments
  size_t nSteps = 600;
  double alpha = 0.05, lambda = 1.0e-8, dt = 0.01;
  std::string blending = "Binary";
  std::string xmlFileName = "cPlanarArm7D.xml";
  std::string directory = "config/xml/Examples";
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-f", &xmlFileName, "Configuration file name");
  argP.getArgument("-dir", &directory, "Configuration file directory");
  argP.getArgument("-blending", &blending, "Blending mode (default is %s)",
                   blending.c_str());
  argP.getArgument("-alpha", &alpha,
                   "Null space scaling factor (default is %f)", alpha);
  argP.getArgument("-lambda", &lambda, "Regularization (default is %f)",
                   lambda);
  argP.getArgument("-nSteps", &nSteps, "Number of computation steps (default"
                   " is %d)", nSteps);
  bool randomizeQ0 = argP.hasArgument("-randomize");
  Rcs_addResourcePath("config");
  Rcs_addResourcePath(directory.c_str());

  // Initialize mutex that takes care that Gui, viewer and control loop can
  // run concurrently.
  pthread_mutex_t mtx;
  pthread_mutex_init(&mtx, NULL);

  // Create controller and Inverse Kinematics solver
  Rcs::ControllerBase controller(xmlFileName);

  if (randomizeQ0)
  {
    MatNd_setRandom(controller.getGraph()->q, -M_PI, M_PI);
    RcsGraph_setState(controller.getGraph(), NULL, NULL);
  }

  Rcs::IkSolverRMR ikSolver(&controller);
  bool success = ikSolver.setActivationBlending(blending);
  RCHECK(success);

  MatNd* dq_des  = MatNd_create(controller.getGraph()->dof, 1);
  MatNd* a_des   = MatNd_create(controller.getNumberOfTasks(), 1);
  MatNd* a_blend = MatNd_create(controller.getNumberOfTasks(), 1);
  MatNd* x_curr  = MatNd_create(controller.getTaskDim(), 1);
  MatNd* x_des   = MatNd_create(controller.getTaskDim(), 1);
  MatNd* dx_des  = MatNd_create(controller.getTaskDim(), 1);
  MatNd* dH      = MatNd_create(1, controller.getGraph()->nJ);
  MatNd_setElementsTo(dx_des, 1.0);

  MatNd* plotMat = MatNd_create(nSteps, 3+7+2);
  plotMat->m = 0;

  // We assign the values from the xml file to the activation vector. If a
  // task is tagged with active="true", the activation is 1, otherwise 0.
  controller.readActivationsFromXML(a_des);
  MatNd_setZero(a_des);
  MatNd_setZero(a_blend);

  // Computes the task kinematics for the current configuration of the model.
  // We initialize it before the Gui is created, so that the sliders will be
  // initialized with the actual values of our model.
  controller.computeX(x_curr);
  MatNd_copy(x_des, x_curr);

  // Launch the task widget in its own thread. The Gui will
  // only write to the arrays if it can grab the mutex.
  // Rcs::ControllerWidgetBase::create(&controller, a_des, x_des, x_curr, &mtx);


  // while (runLoop == true)
  for (size_t i=0; i<nSteps; ++i)
  {
    //pthread_mutex_lock(&mtx);
    //controller.computeDX(dx_des, x_des);
    // controller.computeJointlimitGradient(dH);
    // MatNd_constMulSelf(dH, alpha);
    ikSolver.solveLeftInverse(dq_des, dx_des, dH, a_blend, lambda);
    // MatNd_addSelf(controller.getGraph()->q, dq_des);
    // RcsGraph_setState(controller.getGraph(), NULL, NULL);
    // controller.computeX(x_curr);
    //pthread_mutex_unlock(&mtx);

    // MatNd_printCommentDigits("dq", dq_des, 6);

    // This outputs some information text in the HUD.
    // sprintf(hudText, "dof: %d nJ: %d nx: %zu lambda:%g alpha: %g\n"
    //         "Control is %s",
    //         controller.getGraph()->dof, controller.getGraph()->nJ,
    //         controller.getActiveTaskDim(a_des), lambda, alpha,
    //         ffwd ? "feed-forward" : "feed-back");
    // hud->setText(hudText);


    if (i<100)
    {
      a_des->ele[0] = Math_clip(a_des->ele[0]+dt, 0.0, 1.0);
    }

    if (i>=200 && i<300)
    {
      a_des->ele[0] = Math_clip(a_des->ele[0]-dt, 0.0, 1.0);
      a_des->ele[1] = Math_clip(a_des->ele[1]+dt, 0.0, 1.0);
    }

    if (i>=400 && i<500)
    {
      a_des->ele[0] = Math_clip(a_des->ele[0]+dt, 0.0, 1.0);
      a_des->ele[1] = Math_clip(a_des->ele[1]-dt, 0.0, 1.0);
    }


    a_blend->ele[0] = computeBlendingPolynomial(a_des->ele[0]);
    a_blend->ele[1] = computeBlendingPolynomial(a_des->ele[1]);

    plotMat->m++;
    double* row = MatNd_getRowPtr(plotMat, plotMat->m-1);
    row[0] = i*dt;
    row[1] = a_blend->ele[0];
    row[2] = a_blend->ele[1];
    VecNd_copy(&row[3], dq_des->ele, 7);
    row[10] = VecNd_mean(ikSolver.getCurrentActivation()->ele, 3);
    row[11] = VecNd_mean(&ikSolver.getCurrentActivation()->ele[3], 7);
  }

  MatNd_toFile(plotMat, "blending.dat");

  const char* gpCmd = "plot \"blending.dat\" u 1:2 w l title \"a1\", \"blending.dat\" u 1:3 w l title \"a2\", \"blending.dat\" u 1:4 w lp title \"qp1\", \"blending.dat\" u 1:5 w lp title \"qp2\", \"blending.dat\" u 1:6 w lp title \"qp3\", \"blending.dat\" u 1:7 w lp title \"qp4\", \"blending.dat\" u 1:8 w lp title \"qp5\", \"blending.dat\" u 1:9 w lp title \"qp6\", \"blending.dat\" u 1:10 w lp title \"qp7\"";
  RLOG(0, "Gnuplot command 1:\n\n%s", gpCmd);

  gpCmd = "plot \"blending.dat\" u 1:2 w l title \"a1\", \"blending.dat\" u 1:3 w l title \"a2\", \"blending.dat\" u 1:11 w lp title \"a_{1,curr}\", \"blending.dat\" u 1:12 w lp title \"a_{2,curr}\"";
  RLOG(0, "Gnuplot command 2:\n\n%s", gpCmd);

  // Clean up
  MatNd_destroy(dq_des);
  MatNd_destroy(a_des);
  MatNd_destroy(a_blend);
  MatNd_destroy(x_des);
  MatNd_destroy(dx_des);
  MatNd_destroy(dH);
  MatNd_destroy(plotMat);
  RcsGuiFactory_shutdown();
  xmlCleanupParser();
  pthread_mutex_destroy(&mtx);
}

/*******************************************************************************
 *
 ******************************************************************************/
static void testRandomTasks(int argc, char** argv)
{
  RcsGraph* graph = RcsGraph_createRandom(30, 5);
  Rcs::ControllerBase controller(graph);
  Rcs::Task* task = Rcs::TaskFactory::createRandomTask("XYZ", graph);
  RLOG(0, "Build task of type %s", task ? task->getClassName().c_str(): "NULL");
  if (task)
  {
    controller.add(task);
  }
  task = Rcs::TaskFactory::createRandomTask("ABC", graph);
  RLOG(0, "Build task of type %s", task ? task->getClassName().c_str(): "NULL");
  if (task)
  {
    controller.add(task);
  }
  task = Rcs::TaskFactory::createRandomTask("X", graph);
  RLOG(0, "Build task of type %s", task ? task->getClassName().c_str(): "NULL");
  if (task)
  {
    controller.add(task);
  }
  task = Rcs::TaskFactory::createRandomTask("Y", graph);
  RLOG(0, "Build task of type %s", task ? task->getClassName().c_str(): "NULL");
  if (task)
  {
    controller.add(task);
  }
  task = Rcs::TaskFactory::createRandomTask("Joint", graph);
  RLOG(0, "Build task of type %s", task ? task->getClassName().c_str(): "NULL");
  if (task)
  {
    controller.add(task);
  }

  controller.toXML("RandomController.xml");
}

/*******************************************************************************
 *
 ******************************************************************************/
int main(int argc, char** argv)
{
  RMSG("Starting Rcs...");
  int mode = 0;

  // Ctrl-C callback handler
  signal(SIGINT, quit);

  // This initialize the xml library and check potential mismatches between
  // the version it was compiled for and the actual shared library used.
  LIBXML_TEST_VERSION;

  // Parse command line arguments
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-dl", &RcsLogLevel, "Debug level (default is 0)");
  argP.getArgument("-m", &mode, "Test mode");

  Rcs_addResourcePath("config");


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
      printf("\t\t1   Test blending with left inverse\n");
      printf("\t\t2   Test random task creation\n");
      break;
    }

    case 1:
    {
      testBlendingLeftInverse(argc, argv);
      break;
    }

    case 2:
    {
      testRandomTasks(argc, argv);
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

  xmlCleanupParser();

  fprintf(stderr, "Thanks for using the TestTasks program\n");

  return 0;
}
