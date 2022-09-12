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

#ifndef RCS_EXAMPLEPHYSICS_H
#define RCS_EXAMPLEPHYSICS_H

#include <ExampleBase.h>
#include <Rcs_graph.h>
#include <Rcs_timer.h>

#include <RcsViewer.h>
#include <KeyCatcher.h>
#include <GraphNode.h>
#include <SphereNode.h>
#include <HUD.h>
#include <JointWidget.h>
#include <PhysicsNode.h>

#include <pthread.h>



extern "C" {
  void ExampleInfo();
}

namespace Rcs
{

class ExamplePhysics : public ExampleBase
{
public:
  ExamplePhysics(int argc, char** argv);
  virtual ~ExamplePhysics();
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
  pthread_mutex_t graphLock;
  pthread_mutex_t* mtx;
  double dt, tmc, damping, shootMass;
  double gVec[3];
  char hudText[2056];
  std::string physicsEngine;
  std::string integrator;
  std::string physicsCfg;
  std::string xmlFileName;
  std::string directory;
  std::string bgColor;
  bool pause, posCntrl, skipGui, skipControl, disableCollisions,
       disableJointLimits, testCopy, withPPS, gravComp, resizeable,
       syncHard, seqSim, valgrind, simpleGraphics, bodyAdded, nomutex;
  size_t loopCount;
  RcsGraph* graph;
  PhysicsBase* sim;

  MatNd* q0, *q_des, *q_des_f, *q_curr, *q_dot_curr, *T_gravity, *T_curr;

  KeyCatcher* kc;
  Viewer* viewer;
  HUD* hud;
  PhysicsNode* simNode;

  JointGui* jGui;

  Timer* timer;
};

class ExamplePhysics_Gyro : public ExamplePhysics
{
public:
  ExamplePhysics_Gyro(int argc, char** argv);
  virtual void initParameters();
};

class ExamplePhysics_SoftBullet : public ExamplePhysics
{
public:
  ExamplePhysics_SoftBullet(int argc, char** argv);
  virtual void initParameters();
};

class ExamplePhysics_SitToStand : public ExamplePhysics
{
public:
  ExamplePhysics_SitToStand(int argc, char** argv);
  virtual void initParameters();
};

class ExamplePhysics_HumanoidPendulum : public ExamplePhysics
{
public:
  ExamplePhysics_HumanoidPendulum(int argc, char** argv);
  virtual void initParameters();
};

class ExamplePhysics_PPStest : public ExamplePhysics
{
public:
  ExamplePhysics_PPStest(int argc, char** argv);
  virtual void initParameters();
};
}   // namespace

#endif   // RCS_EXAMPLEPHYSICS_H
