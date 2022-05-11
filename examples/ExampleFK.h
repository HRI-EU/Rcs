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

#ifndef RCS_EXAMPLEFK_H
#define RCS_EXAMPLEFK_H

#include <ExampleBase.h>
#include <Rcs_graph.h>

#include <RcsViewer.h>
#include <KeyCatcher.h>
#include <GraphNode.h>
#include <SphereNode.h>
#include <HUD.h>
#include <JointWidget.h>

#include <pthread.h>


namespace Rcs
{

class ExampleFK : public ExampleBase
{
public:
  ExampleFK(int argc, char** argv);
  virtual ~ExampleFK();
  virtual void initParameters();
  virtual void clear();
  virtual void parseArgs(CmdLineParser* parser);
  virtual bool initAlgo();
  virtual void initGraphics();
  virtual void initGuis();
  virtual void step();
  virtual std::string help();
  virtual void handleKeys();

protected:
  bool valgrind;
  bool simpleGraphics;
  std::string xmlFileName;
  std::string directory;
  double dtSim, dtStep;
  int fwdKinType;
  char hudText[512];
  std::string comRef;
  std::string dotFile;
  std::string bgColor;
  std::string fKinBdyName;
  bool testCopy;
  bool resizeable;
  bool editMode;
  bool playBVH;
  bool noHud;
  bool randomGraph;
  bool shapifyReplace;
  bool noMutex;
  bool helpMsg;
  RcsGraph* graph;
  MatNd* bvhTraj;
  pthread_mutex_t graphLock;
  pthread_mutex_t* mtx;

  osg::ref_ptr<Rcs::KeyCatcher> kc;
  osg::ref_ptr<Rcs::GraphNode> gn;
  osg::ref_ptr<Rcs::SphereNode> comNd;
  osg::ref_ptr<Rcs::HUD> hud;
  Rcs::Viewer* viewer;
  JointGui* jGui;

  int guiHandle;
  unsigned int loopCount;
  double mass, Id[3][3], r_com[3];
  unsigned int bvhIdx;
  const RcsBody* comBase;
};

class ExampleFK_Octree : public ExampleFK
{
public:
  ExampleFK_Octree(int argc, char** argv);
  virtual void initParameters();
};

}   // namespace

#endif   // RCS_EXAMPLEFK_H
