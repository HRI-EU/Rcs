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

#ifndef RCS_EXAMPLEIK_H
#define RCS_EXAMPLEIK_H

#include <ExampleBase.h>
#include <IkSolverRMR.h>
#include <PhysicsBase.h>

#include <RcsViewer.h>
#include <KeyCatcher.h>
#include <GraphNode.h>
#include <PhysicsNode.h>
#include <HUD.h>
#include <BodyPointDragger.h>
#include <VertexArrayNode.h>
#include <SphereNode.h>
#include <ControllerWidgetBase.h>
#include <MatNdWidget.h>
#include <JointWidget.h>

#include <pthread.h>


namespace Rcs
{

class ExampleIK : public ExampleBase
{
public:
  ExampleIK(int argc, char** argv);
  virtual ~ExampleIK();
  virtual void initParameters();
  virtual void parseArgs(CmdLineParser* parser);
  virtual bool initAlgo();
  virtual void initGraphics();
  virtual void initGuis();
  virtual void step();
  virtual void handleKeys();
  virtual std::string help();
  virtual void clear();

protected:
  bool valgrind, simpleGraphics, nomutex, testLocale;
  pthread_mutex_t graphLock;
  pthread_mutex_t* mtx;
  std::string localeStr;
  int algo;
  double alpha, lambda, tmc, dt, dt_calc, jlCost, dJlCost, clipLimit, det,
         scaleDragForce;
  bool calcDistance;
  std::string xmlFileName, directory, effortBdyName, physicsEngine, physicsCfg,
      integrator;
  bool ffwd, skipGui, pause, launchJointWidget, manipulability, cAvoidance,
       constraintIK, initToQ0, testCopying, noHud, posCntrl;
  ControllerBase* controller;
  IkSolverRMR* ikSolver;
  MatNd* dq_des, *q_dot_des, *a_des, *x_curr, *x_physics, *x_des, *x_des_f,
         *dx_des, *dH;
  const RcsBody* effortBdy;
  MatNd* F_effort;
  double r_com[3];

  PhysicsBase* sim;
  ControllerBase* simController;
  RcsGraph* simGraph;
  bool physicsFeedback;

  Viewer* v;
  osg::ref_ptr<KeyCatcher> kc;
  osg::ref_ptr<GraphNode> gn;
  osg::ref_ptr<PhysicsNode> simNode;
  osg::ref_ptr<HUD> hud;
  osg::ref_ptr<BodyPointDragger> dragger;
  osg::ref_ptr<VertexArrayNode> cn;
  osg::ref_ptr<SphereNode> comNd;
  char hudText[2056];
  ControllerGui* cGui;
  MatNdGui* effortGui;
  MatNdGui* dxGui;
  MatNdGui* activationGui;
  JointGui* jGui;
  unsigned int loopCount;
};


class ExampleIK_ContactGrasping : public ExampleIK
{
public:
  ExampleIK_ContactGrasping(int argc, char** argv);
  virtual void initParameters();
};

class ExampleIK_OSimWholeBody : public ExampleIK
{
public:
  ExampleIK_OSimWholeBody(int argc, char** argv);
  virtual void initParameters();
};

class ExampleIK_AssistiveDressing : public ExampleIK
{
public:
  ExampleIK_AssistiveDressing(int argc, char** argv);
  virtual void initParameters();
};

class ExampleIK_StaticEffort : public ExampleIK
{
public:
  ExampleIK_StaticEffort(int argc, char** argv);
  virtual void initParameters();
  virtual bool initAlgo();
  virtual std::string help();
};

}   // namespace

#endif   // RCS_EXAMPLEIK_H
