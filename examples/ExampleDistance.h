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

#ifndef RCS_EXAMPLEDISTANCE_H
#define RCS_EXAMPLEDISTANCE_H

#include <ExampleBase.h>
#include <Rcs_graph.h>

#include <RcsViewer.h>
#include <HUD.h>
#include <KeyCatcher.h>

#include <pthread.h>


namespace Rcs
{

class ExampleDistance : public ExampleBase
{
public:
  ExampleDistance(int argc, char** argv);
  virtual ~ExampleDistance();
  virtual bool parseArgs(CmdLineParser* parser);
  virtual bool initAlgo();
  virtual bool initGraphics();
  virtual void step();
  virtual std::string help();
  virtual void handleKeys();

protected:
  int shapeType1;
  int shapeType2;
  char textLine[2056];
  bool valgrind;
  bool simpleGraphics;
  bool nomutex;
  pthread_mutex_t graphLock;
  pthread_mutex_t* mtx;

  RcsGraph* graph;
  RcsBody* b1;
  RcsBody* b2;
  RcsShape* sh1;
  RcsShape* sh2;
  double I_closestPts[6];
  double* cp0;
  double* cp1;
  double n01[3];

  Rcs::Viewer* viewer;
  osg::ref_ptr<HUD> hud;
  osg::ref_ptr<KeyCatcher> kc;
};


}   // namespace

#endif   // RCS_EXAMPLEDISTANCE_H
