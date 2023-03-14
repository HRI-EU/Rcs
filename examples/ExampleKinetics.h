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

#ifndef RCS_EXAMPLEKINETICS_H
#define RCS_EXAMPLEKINETICS_H

#include <ExampleBase.h>
#include <RcsViewer.h>
#include <HUD.h>
#include <BodyPointDragger.h>
#include <KeyCatcher.h>
#include <MatNdWidget.h>
#include <Rcs_graph.h>
#include <Rcs_timer.h>
#include <PhysicsBase.h>
#include <Rcs_filters.h>

#include <pthread.h>


namespace Rcs
{

class ExampleKinetics : public ExampleBase
{
public:

  pthread_mutex_t graphLock;
  osg::ref_ptr<Rcs::BodyPointDragger> dragger;
  osg::ref_ptr<Rcs::HUD> hud;
  osg::ref_ptr<Rcs::KeyCatcher> kc;
  Rcs::Viewer* viewer;

  std::string xmlFileName, directory, integrator;
  char hudText[4096];
  double damping, E, dt, dt_opt, time, t0;
  bool gCancel, hCancel;
  bool valgrind, pause, simpleGraphics;

  RcsGraph* graph;
  Timer* timer;

  MatNd* z;
  MatNd* F_ext;
  MatNd* err;

  ExampleKinetics(int argc, char** argv);
  virtual ~ExampleKinetics();
  virtual bool initParameters();
  virtual bool parseArgs(CmdLineParser* parser);
  virtual bool initAlgo();
  virtual bool initGraphics();
  virtual void step();
  virtual void handleKeys();
  virtual std::string help();
};

class ExampleJointSpaceInvDyn : public ExampleBase
{
public:

  pthread_mutex_t graphLock;
  std::string xmlFileName, directory, physicsEngine, physicsConfig;
  double dt, tmc, vmax, kp, kd;
  bool pause, valgrind, simpleGraphics, plot, noJointLimits, noCollisions;

  RcsGraph* graph;
  MatNd* q_gui;
  MatNd* q_des;
  MatNd* qp_des;
  MatNd* qpp_des;
  MatNd* T_des;
  MatNd* M;
  MatNd* g;
  MatNd* h;
  MatNd* aq;
  MatNd* qp_ik;
  MatNd* T_limit;
  Rcs::PhysicsBase* sim;

  Rcs::Viewer* viewer;
  osg::ref_ptr<Rcs::HUD> hud;
  osg::ref_ptr<Rcs::KeyCatcher> kc;
  Rcs::MatNdGui* gui;

  Rcs::RampFilterND* filt;

  Timer* rtClock;

  ExampleJointSpaceInvDyn(int argc, char** argv);
  virtual ~ExampleJointSpaceInvDyn();
  virtual bool initParameters();
  virtual bool parseArgs(CmdLineParser* parser);
  virtual bool initAlgo();
  virtual bool initGraphics();
  virtual bool initGuis();
  virtual void step();
  virtual void handleKeys();
};

}   // namespace

#endif   // RCS_EXAMPLEKINETICS_H
