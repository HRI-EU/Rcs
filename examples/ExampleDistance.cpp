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

#include "ExampleDistance.h"

#include <ExampleFactory.h>
#include <Rcs_resourcePath.h>
#include <Rcs_cmdLine.h>
#include <Rcs_macros.h>
#include <Rcs_typedef.h>
#include <Rcs_body.h>
#include <Rcs_shape.h>
#include <Rcs_math.h>
#include <Rcs_timer.h>
#include <Rcs_utilsCPP.h>

#include <TargetSetter.h>
#include <ArrowNode.h>
#include <BodyNode.h>
#include <VertexArrayNode.h>
#include <SphereNode.h>



namespace Rcs
{

static ExampleFactoryRegistrar<ExampleDistance> ExampleDistance("Geometry", "Distance");


ExampleDistance::ExampleDistance(int argc, char** argv) : ExampleBase(argc, argv)
{
  pthread_mutex_init(&graphLock, NULL);
  shapeType1 = RCSSHAPE_SSL;
  shapeType2 = RCSSHAPE_SSL;
  textLine[0] = '\0';
  valgrind = false;
  simpleGraphics = false;
  nomutex = false;
  mtx = &graphLock;

  graph = NULL;
  b1 = NULL;
  b2 = NULL;
  sh1 = NULL;
  sh2 = NULL;
  VecNd_setZero(I_closestPts, 6);
  cp0 = &I_closestPts[0];
  cp1 = &I_closestPts[3];
  Vec3d_set(n01, 0.0, 0.0, 0.25);

  viewer = NULL;
}

ExampleDistance::~ExampleDistance()
{
  clear();
  delete viewer;
  RcsGraph_destroy(graph);
  pthread_mutex_destroy(&graphLock);
}

bool ExampleDistance::parseArgs(CmdLineParser* argP)
{
  argP->getArgument("-t1", &shapeType1, "Shape type for shape 1");
  argP->getArgument("-t2", &shapeType2, "Shape type for shape 2");
  argP->getArgument("-valgrind", &valgrind, "Start without Guis and graphics");
  argP->getArgument("-simpleGraphics", &simpleGraphics, "OpenGL without fancy "
                    "stuff (shadows, anti-aliasing)");
  argP->getArgument("-nomutex", &nomutex, "Graphics without mutex");

  return true;
}

std::string ExampleDistance::help()
{
  std::stringstream s;
  s << RcsShape_distanceFunctionsToString();
  return s.str();
}

bool ExampleDistance::initAlgo()
{
  if (nomutex)
  {
    mtx = NULL;
  }

  graph = RALLOC(RcsGraph);
  RcsGraph_insertGraphBody(graph, -1);
  RcsGraph_insertGraphBody(graph, -1);
  b1 = &graph->bodies[0];
  b2 = &graph->bodies[1];

  Vec3d_setRandom(b1->A_BI.org, -0.2, -0.1);
  sh1 = RcsBody_appendShape(b1);
  RcsShape_initRandom(sh1, shapeType1);

  Vec3d_setRandom(b2->A_BI.org, 0.1, 0.2);
  sh2 = RcsBody_appendShape(b2);
  RcsShape_initRandom(sh2, shapeType2);

  return true;
}

bool ExampleDistance::initGraphics()
{
  if (valgrind)
  {
    return true;
  }

  viewer = new Rcs::Viewer(!simpleGraphics, !simpleGraphics);

  // HUD
  hud = new Rcs::HUD();
  viewer->add(hud.get());

  kc = new Rcs::KeyCatcher();
  viewer->add(kc.get());

  // BodyNodes
  Rcs::BodyNode* bNd1 = new Rcs::BodyNode(b1, graph, 1.0, false);
  Rcs::BodyNode* bNd2 = new Rcs::BodyNode(b2, graph, 1.0, false);
  bNd1->setGhostMode(true, "RED");
  bNd2->setGhostMode(true, "GREEN");
  viewer->add(bNd1);
  viewer->add(bNd2);

  // TargetSetters
  bool sphTracker = b1->shapes[0].type == RCSSHAPE_POINT ? false : true;
  Rcs::TargetSetter* ts1 =
    new Rcs::TargetSetter(b1->A_BI.org, b1->A_BI.rot, 0.5, sphTracker);
  viewer->add(ts1);
  viewer->add(ts1->getHandler());
  sphTracker = b2->shapes[0].type == RCSSHAPE_POINT ? false : true;
  Rcs::TargetSetter* ts2 =
    new Rcs::TargetSetter(b2->A_BI.org, b2->A_BI.rot, 0.5, sphTracker);
  viewer->add(ts2);
  viewer->add(ts2->getHandler());

  // VertexArrayNode for distance
  Rcs::VertexArrayNode* cpLine =
    new Rcs::VertexArrayNode(I_closestPts, 2);
  cpLine->setColor("GREEN");
  cpLine->setPointSize(2.0);
  viewer->add(cpLine);

  Rcs::SphereNode* sphereCP0 = new Rcs::SphereNode(cp0, 0.015);
  sphereCP0->makeDynamic(cp0);
  sphereCP0->setMaterial("RED");
  viewer->add(sphereCP0);

  Rcs::SphereNode* sphereCP1 = new Rcs::SphereNode(cp1, 0.015);
  sphereCP1->makeDynamic(cp1);
  sphereCP1->setMaterial("GREEN");
  viewer->add(sphereCP1);

  // ArrowNode for normal vector
  Rcs::ArrowNode* normalArrow = new Rcs::ArrowNode(cp0, n01, 0.2);
  viewer->add(normalArrow);

  viewer->runInThread(mtx);

  return true;
}

void ExampleDistance::step()
{
  double dt, dist;
  char buf[512];

  pthread_mutex_lock(&graphLock);
  dt = Timer_getTime();
  dist = RcsShape_distance(sh1, sh2, &b1->A_BI, &b2->A_BI, cp0, cp1, n01);
  dt = Timer_getTime() - dt;
  pthread_mutex_unlock(&graphLock);

  sprintf(buf, "Distance: D = % 3.2f mm took %3.2f usec\n",
          dist * 1000.0, dt * 1.0e6);
  strcpy(textLine, buf);
  sprintf(buf, "cp1: %.5f %.5f %.5f   cp2: %.5f %.5f %.5f\n",
          cp0[0], cp0[1], cp0[2], cp1[0], cp1[1], cp1[2]);
  strcat(textLine, buf);
  sprintf(buf, "normal: %.5f %.5f %.5f (len: %.5f)\n",
          n01[0], n01[1], n01[2], Vec3d_getLength(n01));
  strcat(textLine, buf);
  if (hud.valid())
  {
    hud->setText(textLine);
  }
  else
  {
    std::cout << textLine;
  }

  Timer_usleep(1000);
  RPAUSE_DL(4);

  if (valgrind)
  {
    runLoop = false;
  }
}

void ExampleDistance::handleKeys()
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

}


}   // namespace Rcs
