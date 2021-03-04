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
#include <IkSolverRMR.h>
#include <PhysicsNode.h>
#include <HUD.h>
#include <KeyCatcher.h>
#include <RcsViewer.h>
#include <Rcs_guiFactory.h>
#include <ControllerWidgetBase.h>
#include <PhysicsFactory.h>
#include <SegFaultHandler.h>

#include <iostream>

RCS_INSTALL_ERRORHANDLERS


int main(int argc, char** argv)
{
  // That's what KeyCatcherBase::printRegisteredKeys() will show
  Rcs::KeyCatcherBase::registerKey("q", "Quit");
  Rcs::KeyCatcherBase::registerKey("f", "Toggle feed-back / feed-forward mode");

  // Parse command line arguments
  double alpha = 0.05, lambda = 1.0e-8, dt = 0.01;
  std::string xmlFileName = "cSoftPhysicsIK.xml";
  std::string directory = "config/xml/Examples";
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-dl", &RcsLogLevel, "Debug level (default is 0)");
  argP.getArgument("-f", &xmlFileName, "Configuration file name");
  argP.getArgument("-dir", &directory, "Configuration file directory");
  argP.getArgument("-alpha", &alpha,
                   "Null space scaling factor (default is %f)", alpha);
  argP.getArgument("-lambda", &lambda, "Regularization (default is %f)",
                   lambda);

  // We add these paths to our search paths so that xml, texture and font
  // files can be found.
  Rcs_addResourcePath("config");
  Rcs_addResourcePath(directory.c_str());

  // If the program is started with the -h option, we print out lots of
  // information and quit.
  if (argP.hasArgument("-h"))
  {
    printf("Inverse Kinematics test\n\n");
    Rcs::KeyCatcherBase::printRegisteredKeys();
    argP.print();
    Rcs_printResourcePath();
    Rcs::ControllerBase::printUsage(xmlFileName);
    return 0;
  }

  // Initialize mutex that takes care that Gui, viewer and control loop can
  // run concurrently.
  pthread_mutex_t mtx;
  pthread_mutex_init(&mtx, NULL);

  // Create controller and Inverse Kinematics solver
  Rcs::ControllerBase controller(xmlFileName, true);
  Rcs::IkSolverRMR ikSolver(&controller);

  MatNd* dq_des  = MatNd_create(controller.getGraph()->dof, 1);
  MatNd* a_des   = MatNd_create(controller.getNumberOfTasks(), 1);
  MatNd* x_curr  = MatNd_create(controller.getTaskDim(), 1);
  MatNd* x_des   = MatNd_create(controller.getTaskDim(), 1);
  MatNd* dx_des  = MatNd_create(controller.getTaskDim(), 1);
  MatNd* dH      = MatNd_create(1, controller.getGraph()->nJ);

  // We assign the values from the xml file to the activation vector. If a
  // task is tagged with active="true", the activation is 1, otherwise 0.
  controller.readActivationsFromXML(a_des);

  // Computes the task kinematics for the current configuration of the model.
  // We initialize it before the Gui is created, so that the sliders will be
  // initialized with the actual values of our model.
  controller.computeX(x_curr);
  MatNd_copy(x_des, x_curr);

  // Create a physics simulator with our graph model.
  Rcs::PhysicsBase* sim = Rcs::PhysicsFactory::create("SoftBullet", controller.getGraph(),
                                                      "config/physics/physics.xml");
  RCHECK(sim);

  // Create visualization. We add a key catcher that allows us to determine
  // which key was pressed in the graphics window. The HUD allows us to
  // display some text, and the GraphNode shows the 3d model. We add a
  // PhysicsNode for the pose coming out of the IK, and for the pose
  // showing the result from the physics simulator.
  Rcs::KeyCatcherBase::registerKey("q", "Quit");
  Rcs::KeyCatcherBase::registerKey("f", "Toggle feed forward / feed back IK");
  bool ffwd = true;
  char hudText[512] = "";
  Rcs::Viewer viewer;
  osg::ref_ptr<Rcs::KeyCatcher> kc = new Rcs::KeyCatcher();
  osg::ref_ptr<Rcs::HUD> hud = new Rcs::HUD();
  viewer.add(hud.get());
  viewer.add(kc.get());
  viewer.add(new Rcs::PhysicsNode(sim));

  // The viewer is started in its own thread. It will only update the graphics
  // if it gets the mutex. This is how we avoid graphics update during
  // kinematics calculations.
  viewer.runInThread(&mtx);

  // Launch the task widget in its own thread. The Gui will
  // only write to the arrays if it can grab the mutex.
  Rcs::ControllerWidgetBase::create(&controller, a_des, x_des, x_curr, &mtx);


  // Here is the main loop for the IK calculation
  bool runLoop = true;

  while (runLoop == true)
  {
    // Everything inside the mutex lock and unlock directives never runs
    // concurrently with the Gui and viewer updates.
    pthread_mutex_lock(&mtx);

    // Based on the x_des array (comes out of the Gui) and the current
    // configuration (forward kinematics of the model), we compute a task
    // space error dx_des that the IK should correct.
    controller.computeDX(dx_des, x_des);

    // Since we have a redundant system (dim(dx)<dim(dq)), there is a null
    // space. We compute a gradient dH that we project into it.
    controller.computeJointlimitGradient(dH);
    MatNd_constMulSelf(dH, alpha);

    // Here we solve the Inverse Kinematics. The array dq_des is the joint space
    // displacement that corrects for dx_des. The scalar lambda is a small
    // regularization value that makes the solution more numerically robust.
    ikSolver.solveRightInverse(dq_des, dx_des, dH, a_des, lambda);

    // Here we add the result dq_des from the IK to the q-vector of the model...
    MatNd_addSelf(controller.getGraph()->q, dq_des);

    // ... and compute the forward kinematics. After this, all steps that might
    // possibly interfere with the Gui and viwer are completed, and we can
    // release the mutex.
    RcsGraph_setState(controller.getGraph(), NULL, NULL);

    // Set the joint state after the IK step as desired values for the
    // simulator. We only consider position-controlled joints here.
    sim->setControlInput(controller.getGraph()->q, NULL, NULL);

    // Simulation steps. Here we consider two options:
    // Feed-forward: The resulting real joint angles are ignored, and the
    // nominal angles from the IK are integrated. That's pretty robust, but
    // requires a low-level control that tracks the joint angles very well.
    // Feed-back: We feed the real physical angles into the IK loop. This
    // allows to close the loop around real measurements. It needs to be
    // taken care that the control keeps stability.
    if (ffwd)
    {
      sim->simulate(dt, NULL, NULL, NULL, NULL, true);
    }
    else
    {
      sim->simulate(dt, controller.getGraph());
      RcsGraph_setState(controller.getGraph(), NULL, NULL);
    }

    // Compute the current task vector. This is not really needed, we do it only
    // to update the values in the Gui.
    controller.computeX(x_curr);

    // After this, all steps that might possibly interfere with the Gui and
    // viewer are completed, and we can release the mutex.
    pthread_mutex_unlock(&mtx);

    // In case the q-key is pressed in the graphics window, we quit the loop.
    if (kc->getAndResetKey('q'))
    {
      runLoop = false;
    }
    // Toggle between feed-back and feed-forward mode on button f.
    else if (kc->getAndResetKey('f'))
    {
      ffwd = !ffwd;
    }

    // This outputs some information text in the HUD.
    sprintf(hudText, "dof: %d nJ: %d nx: %zu lambda:%g alpha: %g\n"
            "Control is %s",
            controller.getGraph()->dof, controller.getGraph()->nJ,
            controller.getActiveTaskDim(a_des), lambda, alpha,
            ffwd ? "feed-forward" : "feed-back");
    hud->setText(hudText);

    // Give te other threads a bit time to do their stuff.
    Timer_waitDT(dt);
  }


  // Stop viewer and Gui before we clean up
  viewer.stopUpdateThread();
  RcsGuiFactory_shutdown();

  // Clean up
  delete sim;
  MatNd_destroy(dq_des);
  MatNd_destroy(a_des);
  MatNd_destroy(x_des);
  MatNd_destroy(dx_des);
  MatNd_destroy(dH);
  xmlCleanupParser();
  pthread_mutex_destroy(&mtx);

  RLOG(0, "Thanks for using the ExampleIK program\n");

  return 0;
}
