/*******************************************************************************

  Copyright (c) Honda Research Institute Europe GmbH

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

#include "ExampleFK.h"

#include <Rcs_resourcePath.h>
#include <Rcs_cmdLine.h>
#include <Rcs_macros.h>
#include <Rcs_typedef.h>
#include <Rcs_body.h>
#include <Rcs_shape.h>
#include <Rcs_utils.h>
#include <Rcs_graphParser.h>
#include <Rcs_kinematics.h>
#include <Rcs_BVHParser.h>
#include <Rcs_math.h>
#include <Rcs_timer.h>
#include <Rcs_utilsCPP.h>
#include <ExampleFactory.h>
#include <Rcs_collisionModel.h>

#include <RcsViewer.h>
#include <KeyCatcher.h>
#include <GraphNode.h>
#include <SphereNode.h>
#include <HUD.h>
#include <PPSGui.h>
#include <PPSSensorNode.h>
#include <VertexArrayNode.h>
#include <AABBNode.h>
#include <CmdLineWidget.h>

#include <sstream>


namespace Rcs
{

/*******************************************************************************
 *
 ******************************************************************************/
RCS_REGISTER_EXAMPLE(ExampleFK, "Forward kinematics", "Dexbot");

ExampleFK::ExampleFK(int argc, char** argv) : ExampleBase(argc, argv)
{
  valgrind = false;
  simpleGraphics = false;
  dtSim = 0.0;
  dtStep = 0.04;
  fwdKinType = 0;
  hudText[0] = '\0';
  testCopy = false;
  resizeable = false;
  editMode = false;
  playBVH = false;
  noHud = false;
  updateHud = true;
  randomGraph = false;
  shapifyReplace = false;
  noMutex = false;
  helpMsg = false;
  graph = NULL;
  bvhTraj = NULL;
  viewer = NULL;
  jGui = NULL;
  loopCount = 0;
  mass = 0.0;
  Mat3d_setIdentity(Id);
  Vec3d_setZero(r_com);
  bvhIdx = 0;
  comBase = NULL;
  pthread_mutex_init(&graphLock, NULL);
  mtx = &graphLock;
}

ExampleFK::~ExampleFK()
{
  clear();

  RLOG(1, "Destroying graphLock");
  pthread_mutex_destroy(&graphLock);
  RLOG(1, "Done destructor");
}

void ExampleFK::clear()
{
  if (!valgrind)
  {
    RLOG(1, "Deleting viewer");
    delete viewer;
    viewer = NULL;
    RLOG(1, "Deleting JointWidget");
    delete jGui;
    jGui = NULL;
    RLOG(1, "Done deleting JointWidget");
  }

  RLOG(1, "Destroying graph etc.");
  RcsGraph_destroy(graph);
  graph = NULL;
  MatNd_destroy(bvhTraj);
  bvhTraj = NULL;

  Rcs_removeResourcePath(directory.c_str());
  RLOG(1, "Done clear()");
}

bool ExampleFK::initParameters()
{
  xmlFileName = "gScenario.xml";
  directory = "config/xml/DexBot";
  dotFile = "RcsGraph.dot";
  bgColor = "LIGHT_GRAYISH_GREEN";

  return true;
}

bool ExampleFK::parseArgs(CmdLineParser* argP)
{
  argP->getArgument("-nomutex", &noMutex, "Graphics without mutex");
  argP->getArgument("-valgrind", &valgrind, "Start without Guis and graphics");
  argP->getArgument("-simpleGraphics", &simpleGraphics, "OpenGL without fancy"
                    " stuff (shadows, anti-aliasing)");

  argP->getArgument("-dt", &dtStep, "Animation time step (default is %f)",
                    dtStep);
  argP->getArgument("-dotFile", &dotFile, "Dot file name");
  argP->getArgument("-f", &xmlFileName, "Configuration file name (default"
                    " is \"%s\")", xmlFileName.c_str());
  argP->getArgument("-dir", &directory, "Configuration file directory "
                    "(default is \"%s\")", directory.c_str());
  argP->getArgument("-comRef", &comRef, "Reference body for COM (default is "
                    "root)");
  argP->getArgument("-bgColor", &bgColor, "Background color (default is "
                    "\"%s\")", bgColor.c_str());
  argP->getArgument("-fKin", &fwdKinType, "Forward kinematics: 0: all dof,"
                    "1: sub tree, 2: body only (default is %d)", fwdKinType);
  argP->getArgument("-fKinBdy", &fKinBdyName, "Forward kinematics start "
                    "body (default is none)");
  argP->getArgument("-aabb", &aabbBdyName, "AABB root body (default is none)");
  argP->getArgument("-copy", &testCopy, "Test graph copying");
  argP->getArgument("-resizeable", &resizeable, "Adjust visualization "
                    "of shapes dynamically");
  argP->getArgument("-edit", &editMode, "Start in xml edit mode (no Qt Gui)");
  argP->getArgument("-bvh", &playBVH, "Play bvh file");
  argP->getArgument("-noHud", &noHud, "Don't show HUD");
  argP->getArgument("-random", &randomGraph, "Create randomized graph");
  argP->getArgument("-shapifyReplace", &shapifyReplace, "True: Replace shapes "
                    "when using boxify / capsulify, default: add enclosing box"
                    " / capsule as additional shape to bodies");

  argP->getArgument("-h", &helpMsg, "Print help message to console");

  return true;
}

std::string ExampleFK::help()
{
  std::stringstream s;
  s << "  Forward kineamtics test\n\n";
  s << "  Here are a few examples:\n";
  s << "\t-dir config/xml/LWR -f lbr_iiwa_14_r820.urdf\n";
  s << "\t-dir config/xml/DarwinOP -f robotis_op.urdf\n";
  s << "\t-dir config/xml/Husky -f dual_arm_husky_original.urdf\n";
  s << "\t-dir config/xml/Valkyrie -f valkyrie_sim.urdf\n\n";
  s << Rcs::getResourcePaths();
  s << Rcs::CmdLineParser::printToString();
  s << Rcs::RcsGraph_printUsageToString(xmlFileName);
  return s.str();
}

bool ExampleFK::initAlgo()
{
  Rcs_addResourcePath(directory.c_str());

  // Option without mutex for viewer
  if (noMutex)
  {
    mtx = NULL;
  }

  if (helpMsg)
  {
    help();
  }

  if (randomGraph)
  {
    graph = RcsGraph_createRandom(30, 5);
  }
  else
  {
    graph = RcsGraph_create(xmlFileName.c_str());
  }

  if (graph == NULL)
  {
    RMSG_CPP("Failed to create graph from file " << xmlFileName << " - exiting");
    return false;
  }

  char bvhFile[256];
  strcpy(bvhFile, graph->cfgFile);
  Rcs::CmdLineParser argP;
  argP.getArgument("-bvhFile", bvhFile, "BVH file "
                   "(default is \"%s\")", bvhFile);

  bvhTraj = NULL;
  bvhIdx = 0;
  if (playBVH && String_hasEnding(bvhFile, ".bvh", false))
  {
    bvhTraj = RcsGraph_createTrajectoryFromBVHFile(graph, bvhFile,
                                                   &dtStep, 0.01,
                                                   M_PI / 180.0);

    if (bvhTraj && (bvhTraj->n != graph->dof))
    {
      RLOG(1, "Mismatch in bvh array dimensions: found %d columns, but "
           "graph has %d dof", bvhTraj->n, graph->dof);
      MatNd_destroy(bvhTraj);
      bvhTraj = NULL;
    }
    else if (!bvhTraj)
    {
      bvhTraj = MatNd_createFromFile(bvhFile);
    }
  }

  if (testCopy == true)
  {
    RcsGraph* graph2 = graph;
    double t_copy = Timer_getSystemTime();
    graph = RcsGraph_clone(graph2);
    t_copy = Timer_getSystemTime() - t_copy;
    RMSG("Cloning graph took %.3f msec", t_copy*1.0e3);
    t_copy = Timer_getSystemTime();
    RcsGraph_copy(graph, graph2);
    t_copy = Timer_getSystemTime() - t_copy;
    RMSG("Copying graph took %.3f msec", t_copy*1.0e3);
    t_copy = Timer_getSystemTime();
    RcsGraph_setState(graph2, NULL, NULL);
    t_copy = Timer_getSystemTime() - t_copy;
    RMSG("Forward kinematics took %.3f msec", t_copy*1.0e3);
    RcsGraph_destroy(graph2);
  }

  if (testCopy == true)
  {
    const int nIter = 1000;
    RcsGraph* graph2 = RcsGraph_clone(graph);

    double t1 = Timer_getTime();
    for (int i = 0; i < nIter; ++i)
    {
      RcsGraph_copy(graph2, graph);
    }
    t1 = Timer_getTime() - t1;


    double t2 = Timer_getTime();
    for (int i = 0; i < nIter; ++i)
    {
      RcsGraph_setState(graph2, graph->q, graph->q_dot);
    }
    t2 = Timer_getTime() - t2;

    RLOG(0, "Copying took %f msec  forward kinematics took %f msec",
         1000.0*t1 / nIter, 1000.0*t2 / nIter);

    RcsGraph_destroy(graph2);
  }

  comBase = RcsGraph_getBodyByName(graph, comRef.c_str());

  return true;
}

bool ExampleFK::initGraphics()
{
  Rcs::KeyCatcherBase::registerKey("a", "Change body attachement");
  Rcs::KeyCatcherBase::registerKey("C", "Toggle COM display");
  Rcs::KeyCatcherBase::registerKey("d", "Print dot file");
  Rcs::KeyCatcherBase::registerKey("f", "Write graph to file");
  Rcs::KeyCatcherBase::registerKey("j", "Create JointWidget");
  Rcs::KeyCatcherBase::registerKey("l", "Reload graph file");
  Rcs::KeyCatcherBase::registerKey("m", "Set q to model state");
  Rcs::KeyCatcherBase::registerKey("p", "Print information to console");
  Rcs::KeyCatcherBase::registerKey("q", "Quit");
  Rcs::KeyCatcherBase::registerKey("W", "Merge bodies");
  Rcs::KeyCatcherBase::registerKey("x", "Rewind bvh file");
  Rcs::KeyCatcherBase::registerKey("e", "Remove body under mouse");
  Rcs::KeyCatcherBase::registerKey("J", "Remove joints of body by name");
  Rcs::KeyCatcherBase::registerKey("E", "Remove all joints of graph");
  Rcs::KeyCatcherBase::registerKey("T", "Load trajectory file");
  Rcs::KeyCatcherBase::registerKey("X", "Make monolithic");
  Rcs::KeyCatcherBase::registerKey("S", "Scale graph");
  Rcs::KeyCatcherBase::registerKey("b", "Boxify graph");
  Rcs::KeyCatcherBase::registerKey("B", "Capsulify graph");
  Rcs::KeyCatcherBase::registerKey("H", "Toggle HUD");

  if (valgrind)
  {
    return true;
  }

  viewer = new Rcs::Viewer(!simpleGraphics, !simpleGraphics);
  viewer->setBackgroundColor(bgColor);
  gn = new Rcs::GraphNode(graph, resizeable);
  gn->toggleReferenceFrames();
  viewer->add(gn);

  comNd = new Rcs::SphereNode(r_com, 0.05);
  comNd->makeDynamic(r_com);
  comNd->setMaterial("RED");
  comNd->toggleWireframe();
  comNd->hide();
  viewer->add(comNd);

  char xmlFile2[64] = "";
  Rcs::CmdLineParser argP;
  argP.getArgument("-f2", xmlFile2, "Optional second graph file (default"
                   "is empty)");
  if (strlen(xmlFile2) > 0)
  {
    RcsGraph* graph2 = RcsGraph_create(xmlFile2);
    RCHECK(graph2);
    viewer->add(new Rcs::GraphNode(graph2, resizeable));
  }

  if (!noHud)
  {
    hud = new Rcs::HUD();
    viewer->add(hud);
  }

  kc = new Rcs::KeyCatcher();
  viewer->add(kc);

  RCSGRAPH_FOREACH_SENSOR(graph)
  {
    if (SENSOR->type == RCSSENSOR_PPS)
    {
      bool debug = RcsLogLevel > 0 ? true : false;
      viewer->add(new Rcs::PPSSensorNode(SENSOR, graph, debug));
    }
  }
  if (!aabbBdyName.empty())
  {
    const RcsBody* aabbRoot = RcsGraph_getBodyByName(graph, aabbBdyName.c_str());
    if (!aabbRoot)
    {
      RLOG(1, "AABB root body \"%s\" not found in graph", aabbBdyName.c_str());
    }
    else
    {
      aabbNd = new Rcs::BoxNode(Vec3d_zeroVec(), 1.0, 1.0, 1.0, true);
      viewer->add(aabbNd.get());
    }
  }
  viewer->runInThread(mtx);

  return true;
}

bool ExampleFK::initGuis()
{
  if (valgrind)
  {
    return true;
  }

  if ((editMode == false) && (bvhTraj == NULL))
  {

    // We constrain all joints here so that we can drag them conveniently
    // with the sliders.
    RCSGRAPH_TRAVERSE_JOINTS(graph)
    {
      JNT->constrained = true;
    }

    jGui = new JointGui(graph, mtx);
  }

  return true;
}

void ExampleFK::step()
{
  pthread_mutex_lock(&graphLock);

  if (valgrind)
  {
    RLOG(1, "Starting step");
  }

  if (bvhTraj != NULL)
  {
    MatNd row = MatNd_getRowViewTranspose(bvhTraj, bvhIdx);
    MatNd_copy(graph->q, &row);

    bvhIdx++;
    if (bvhIdx >= bvhTraj->m)
    {
      bvhIdx = 0;
    }
  }

  dtSim = Timer_getSystemTime();

  switch (fwdKinType)
  {
    case 0:
      RcsGraph_setState(graph, NULL, NULL);
      break;

    case 1:
    {
      RcsBody* fkBdy = RcsGraph_getBodyByName(graph, fKinBdyName.c_str());
      RcsGraph_computeBodyKinematics(graph, fkBdy, NULL, NULL, true);
    }
    break;

    case 2:
    {
      RcsBody* fkBdy = RcsGraph_getBodyByName(graph, fKinBdyName.c_str());
      RcsGraph_computeBodyKinematics(graph, fkBdy, NULL, NULL, false);
    }
    break;

    default:
      RFATAL("No forward kinematics mode %d", fwdKinType);
  }

  dtSim = Timer_getSystemTime() - dtSim;
  if (comBase != NULL)
  {
    mass = RcsGraph_COG_Body(graph, comBase, r_com);
  }
  else
  {
    mass = RcsGraph_COG(graph, r_com);
  }

  double t_aabb = 0.0;

  if (aabbNd.valid())
  {
    const RcsBody* aabbBdy = RcsGraph_getBodyByName(graph, aabbBdyName.c_str());
    RCHECK(aabbBdy);   // Has already been checked in initGraphics()
    double xyzMin[3], xyzMax[3];
    t_aabb = Timer_getTime();
    int nAABB = RcsGraph_computeSubTreeAABB(graph, aabbBdy->id,
                                            RCSSHAPE_COMPUTE_DISTANCE,
                                            xyzMin, xyzMax, NULL);
    t_aabb = Timer_getTime() - t_aabb;
    if (nAABB > 0)
    {
      double center[3], extents[3];
      for (int i = 0; i < 3; ++i)
      {
        center[i] = xyzMin[i] + 0.5 * (xyzMax[i] - xyzMin[i]);
        extents[i] = xyzMax[i] - xyzMin[i];
      }
      aabbNd->setPosition(center);
      aabbNd->resize(extents);
    }
  }

  if (valgrind)
  {
    RLOG(1, "Finished step");

    if (valgrind == true && loopCount > 10)
    {
      runLoop = false;
    }
  }

  pthread_mutex_unlock(&graphLock);

  sprintf(hudText, "Graph \"%s\"\nDof: %d nJ: %d\n"
          "Forward kinematics step: %.1f ms",
          graph->cfgFile, graph->dof, graph->nJ, dtSim*1000.0);

  if (bvhTraj != NULL)
  {
    char a[256];
    snprintf(a, 256, "\nBVH row %d (from %d)", bvhIdx, bvhTraj->m);
    strcat(hudText, a);
  }

  if (updateHud)
  {
    if (hud)
    {
      hud->setText(hudText);
    }
    else
    {
      std::cout << hudText;
    }
  }

  REXEC(6)
  {
    RPAUSE();
  }

  Timer_waitDT(dtStep);
  loopCount++;
}

void ExampleFK::handleKeys()
{
  if (!kc)
  {
    return;
  }

  if (kc->getAndResetKey('q'))
  {
    runLoop = false;
  }
  else if (kc->getAndResetKey('H'))
  {
    if (hud)
    {
      hud->toggle();
    }
  }
  else if (kc->getAndResetKey('S'))
  {
    RMSG("Changing scale factor");
    printf("Enter scaling factor: ");
    double scaleFactor;
    std::cin >> scaleFactor;

    bool collisionVisible = gn->collisionModelVisible();
    bool graphicsVisible = gn->graphicsModelVisible();
    bool physicsVisible = gn->physicsModelVisible();
    bool framesVisible = gn->referenceFramesVisible();
    bool ghostVisible = gn->getGhostMode();
    bool wireframeVisible = gn->getWireframe();
    viewer->removeInternal(gn);

    pthread_mutex_lock(&graphLock);
    RcsGraph_scale(graph, scaleFactor);
    gn = new Rcs::GraphNode(graph);
    gn->toggleReferenceFrames();
    gn->displayGraphicsModel(graphicsVisible);
    gn->displayPhysicsModel(physicsVisible);
    gn->displayCollisionModel(collisionVisible);
    gn->displayReferenceFrames(framesVisible);
    gn->setGhostMode(ghostVisible);
    gn->showWireframe(wireframeVisible);
    pthread_mutex_unlock(&graphLock);
    viewer->add(gn);
  }
  else if (kc->getAndResetKey('m'))
  {
    std::string mdlState;
    RMSG("Changing model state. Options are:\n");
    std::vector<std::string> ms = RcsGraph_getModelStateNames(graph);
    ms.push_back("Zero");
    for (size_t i = 0; i < ms.size(); ++i)
    {
      printf("\t%s\n", ms[i].c_str());
    }
    printf("\nEnter name of model state: ");
    std::cin >> mdlState;
    pthread_mutex_lock(&graphLock);
    bool ok = true;

    if (mdlState == "Zero")
    {
      MatNd_setZero(graph->q);
      RcsGraph_setState(graph, NULL, NULL);
    }
    else
    {
      ok = RcsGraph_setModelStateFromXML(graph, mdlState.c_str(), 0);
    }

    pthread_mutex_unlock(&graphLock);
    RMSG("%s changing model state to %s",
         ok ? "SUCCEEDED" : "FAILED", mdlState.c_str());
  }
  else if (kc->getAndResetKey('j'))
  {
    RMSGS("Creating JointWidget");
    jGui = new JointGui(graph, &graphLock);
  }
  else if (kc->getAndResetKey('e'))
  {
    Rcs::BodyNode* bNd = viewer->getBodyNodeUnderMouse<Rcs::BodyNode*>();
    if (bNd == NULL)
    {
      RMSG("No BodyNode found under mouse");
    }
    else
    {
      RMSG("Removing body \"%s\" under mouse", bNd->body()->name);
      pthread_mutex_lock(&graphLock);
      bool ok = RcsGraph_removeBody(graph, bNd->body()->name, NULL, 0);
      if (ok)
      {
        ok = gn->removeBodyNode(bNd);
      }
      pthread_mutex_unlock(&graphLock);
      RMSG("%s removing body", ok ? "SUCCEESS" : "FAILURE");
    }
  }
  else if (kc->getAndResetKey('J'))
  {
    std::string bodyName;
    RMSG("Removing all joints of a body");
    printf("Enter body name: ");
    std::cin >> bodyName;
    RcsBody* bdy = RcsGraph_getBodyByName(graph, bodyName.c_str());
    if (bdy)
    {
      bool success = RcsBody_removeJoints(bdy, graph);
      RMSG("Removing all joints of a body %s %s", bodyName.c_str(),
           success ? "SUCCEEDED" : "FAILED");
    }
    else
    {
      RMSG("Body %s not found in graph", bodyName.c_str());
    }
  }
  else if (kc->getAndResetKey('E'))
  {
    RMSG("Removing all joints of graph");
    pthread_mutex_lock(&graphLock);
    RCSGRAPH_TRAVERSE_BODIES(graph)
    {
      bool success = RcsBody_removeJoints(BODY, graph);
      RLOG(1, "Removing all joints of a body %s %s", BODY->name,
           success ? "SUCCEEDED" : "FAILED");
    }
    pthread_mutex_unlock(&graphLock);
  }
  else if (kc->getAndResetKey('C'))
  {
    RMSGS("Toggling COM");
    comNd->toggle();
  }
  else if (kc->getAndResetKey('f'))
  {
    RMSG("Writing graph to xml file");
    FILE* out = fopen("graph.txt", "w+");
    RCHECK(out);
    RcsGraph_fprint(out, graph);
    fclose(out);
    out = fopen("graph.xml", "w+");
    RCHECK(out);
    RcsGraph_fprintXML(out, graph);
    fclose(out);
  }
  else if (kc->getAndResetKey('d'))
  {
    RMSGS("Writing dot file");
    RcsGraph_writeDotFile(graph, dotFile.c_str());
    char osCmd[256];
    sprintf(osCmd, "dotty %s&", dotFile.c_str());
    int err = system(osCmd);

    if (err == -1)
    {
      RMSG("Couldn't start dot file viewer!");
    }
  }
  else if (kc->getAndResetKey('a'))
  {
    std::string parentName, childName;
    RMSG("Changing body attachement");
    printf("Enter body to attach (child): ");
    std::cin >> childName;
    printf("Enter attachement (parent) body: ");
    std::cin >> parentName;

    RcsBody* child = RcsGraph_getBodyByName(graph, childName.c_str());
    RcsBody* parent = RcsGraph_getBodyByName(graph, parentName.c_str());
    RLOG(0, "Attaching \"%s\" (%s) to \"%s\" (%s)",
         child ? child->name : "NULL", childName.c_str(),
         parent ? parent->name : "NULL", parentName.c_str());

    bool success = RcsBody_attachToBodyId(graph, child ? child->id : -1, parent ? parent->id : -1);
    RMSG("%s changing attachement", success ? "SUCCESS" : "FAILURE");

    RcsGraph_fprintJointRecursion(stdout, graph, parentName.c_str());
    RcsGraph_toXML(graph, "graph.xml");
    RcsGraph_writeDotFile(graph, dotFile.c_str());
    RCHECK(RcsGraph_check(graph, NULL, NULL));


    REXEC(1)
    {
      char osCmd[256];
      sprintf(osCmd, "dotty %s&", dotFile.c_str());
      int err = system(osCmd);

      if (err == -1)
      {
        RMSG("Couldn't start dot file viewer!");
      }
    }


  }
  else if (kc->getAndResetKey('p'))
  {
    if (graph != NULL)
    {
      RMSG("Here's the forward tree:");
      RcsGraph_fprint(stderr, graph);
      RLOGS(0, "m=%f   r_com=%f %f %f",
            mass, r_com[0], r_com[1], r_com[2]);
      RcsGraph_fprintModelState(stdout, graph, graph->q, NULL, 0);
    }
  }
  else if (kc->getAndResetKey('b'))
  {
    RMSG("Boxifying graph ...");
    pthread_mutex_lock(&graphLock);
    viewer->removeInternal(gn);
    RCSGRAPH_TRAVERSE_BODIES(graph)
    {
      bool success = RcsBody_boxify(BODY, RCSSHAPE_COMPUTE_GRAPHICS +
                                    RCSSHAPE_COMPUTE_PHYSICS, shapifyReplace);
      RLOG(1, "%s boxifying body %s", success ? "SUCCESS" : "FAILURE",
           BODY->name);
    }
    gn = new Rcs::GraphNode(graph);
    viewer->addInternal(gn);
    pthread_mutex_unlock(&graphLock);
    RMSG("... done boxifying graph");
  }
  else if (kc->getAndResetKey('B'))
  {
    RMSG("Capsulifying graph ...");
    pthread_mutex_lock(&graphLock);
    viewer->removeInternal(gn);
    RCSGRAPH_TRAVERSE_BODIES(graph)
    {
      bool success = RcsBody_capsulify(BODY, RCSSHAPE_COMPUTE_GRAPHICS,
                                       shapifyReplace);
      RLOG(1, "%s capsulifying body %s", success ? "SUCCESS" : "FAILURE",
           BODY->name);
    }
    gn = new Rcs::GraphNode(graph);
    viewer->addInternal(gn);
    pthread_mutex_unlock(&graphLock);
    RMSG("... done boxifying graph");
  }
  else if (kc->getAndResetKey('l'))
  {
    RMSG("Reloading GraphNode from %s", xmlFileName.c_str());

    delete jGui;
    jGui = NULL;
    RLOG(0, "JointGui deleted");

    bool collisionVisible = gn->collisionModelVisible();
    bool graphicsVisible = gn->graphicsModelVisible();
    bool physicsVisible = gn->physicsModelVisible();
    bool depthVisible = gn->depthModelVisible();
    bool framesVisible = gn->referenceFramesVisible();
    bool ghostVisible = gn->getGhostMode();
    bool wireframeVisible = gn->getWireframe();
    pthread_mutex_lock(&graphLock);
    viewer->removeInternal(gn);
    viewer->removeInternal("PPSSensorNode");
    gn = NULL;
    RcsGraph_destroy(graph);
    graph = RcsGraph_create(xmlFileName.c_str());
    comBase = RcsGraph_getBodyByName(graph, comRef.c_str());

    if (graph != NULL)
    {
      gn = new Rcs::GraphNode(graph);
      gn->toggleReferenceFrames();
      gn->displayGraphicsModel(graphicsVisible);
      gn->displayPhysicsModel(physicsVisible);
      gn->displayCollisionModel(collisionVisible);
      gn->displayDepthModel(depthVisible);
      gn->displayReferenceFrames(framesVisible);
      gn->setGhostMode(ghostVisible);
      gn->showWireframe(wireframeVisible);
      pthread_mutex_unlock(&graphLock);
      viewer->add(gn);
      pthread_mutex_lock(&graphLock);
    }
    else
    {
      RLOG(1, "Couldn't create graph - skipping osg "
           "node");
    }
    pthread_mutex_unlock(&graphLock);
    RMSG("... done");
  }
  else if (kc->getAndResetKey('W'))
  {
    RMSG("Merging bodies. Here are the options:");
    int nBodiesMergeable = 0;
    RCSGRAPH_TRAVERSE_BODIES(graph)
    {
      if (BODY->jntId == -1 && BODY->id != graph->rootId)
      {
        printf("   %s\n", BODY->name);
        nBodiesMergeable++;
      }
    }

    if (nBodiesMergeable > 0)
    {
      std::string name;
      printf("Enter body to merge: ");
      std::cin >> name;
      pthread_mutex_lock(&graphLock);
      bool success = RcsBody_mergeWithParent(graph, name.c_str());
      RMSG("%s merging body %s", success ? "SUCCEEDED" : "FAILED",
           name.c_str());
      if (success == true)
      {
        FILE* fd = fopen("merged.xml", "w+");
        RCHECK(fd);
        RcsGraph_fprintXML(fd, graph);
        fclose(fd);
      }
      viewer->removeInternal(gn);
      gn = new Rcs::GraphNode(graph);
      gn->toggleReferenceFrames();
      pthread_mutex_unlock(&graphLock);
      viewer->add(gn);
    }
    else
    {
      RMSG("There are no more bodies to merge");
    }
  }
  else if (kc->getAndResetKey('x'))
  {
    RMSG("Rewinding BVH file");
    bvhIdx = 0;
  }
  else if (kc->getAndResetKey('T'))
  {
    RMSG("Loading trajectory file");
    std::string tFile;
    printf("Enter file name: ");
    std::cin >> tFile;

    MatNd* newTraj = MatNd_createFromFile(tFile.c_str());

    if ((newTraj != NULL) && (newTraj->n == graph->dof))
    {
      MatNd_destroy(bvhTraj);
      bvhTraj = newTraj;
      bvhIdx = 0;
    }
    else
    {
      RLOG(1, "Failed to read trajectory file \"%s\"", tFile.c_str());
    }

  }
  else if (kc->getAndResetKey('X'))
  {
    RMSG("Merging graph to a single rigid body");
    pthread_mutex_lock(&graphLock);
    RMSG("Removing all joints of graph");
    RCSGRAPH_TRAVERSE_BODIES(graph)
    {
      bool success = RcsBody_removeJoints(BODY, graph);
      RMSG("Removing all joints of a body %s %s", BODY->name,
           success ? "SUCCEEDED" : "FAILED");
    }

    RcsBody* rootBdy = RcsGraph_getRootBody(graph);
    RcsBody* first = RcsBody_depthFirstTraversalGetNextById(graph, rootBdy);

    while (first)
    {
      bool success = RcsBody_mergeWithParent(graph, first->name);
      RMSG("%s to merge body %s with root",
           success ? "SUCCEEDED" : "FAILED", first->name);
      first = RcsBody_depthFirstTraversalGetNextById(graph, rootBdy);
      RMSG("Next body to merge: \"%s\"", first ? first->name : "NULL");
      RPAUSE();
    }

    viewer->removeInternal(gn);
    gn = new Rcs::GraphNode(graph);
    gn->toggleReferenceFrames();
    pthread_mutex_unlock(&graphLock);

    viewer->add(gn);
  }
}



/*******************************************************************************
 *
 ******************************************************************************/
RCS_REGISTER_EXAMPLE(ExampleFK_Octree, "Forward kinematics", "Octree");

ExampleFK_Octree::ExampleFK_Octree(int argc, char** argv) : ExampleFK(argc, argv)
{
}

bool ExampleFK_Octree::initParameters()
{
  ExampleFK::initParameters();
  xmlFileName = "gOctree.xml";
  directory = "config/xml/Examples";

  return true;
}



/*******************************************************************************
 *
 ******************************************************************************/
RCS_REGISTER_EXAMPLE(ExampleFK_Below, "Forward kinematics", "Below");

ExampleFK_Below::ExampleFK_Below(int argc, char** argv) : ExampleFK(argc, argv)
{
  Vec3d_setZero(belowPt);
}

bool ExampleFK_Below::initGraphics()
{
  ExampleFK::initGraphics();

  gn->displayReferenceFrames(false);

  belowNd = new Rcs::SphereNode(belowPt, 0.05);
  belowNd->makeDynamic(belowPt);
  belowNd->setMaterial("RED");
  viewer->add(belowNd);

  return true;
}

bool ExampleFK_Below::initParameters()
{
  ExampleFK::initParameters();
  xmlFileName = "gScenario.xml";
  directory = "config/xml/PPStest";
  editMode = true;
  updateHud = false;   // Class writes its own text into the Hud
  belowBdy = "Sphere";

  return true;
}

bool ExampleFK_Below::initAlgo()
{
  ExampleFK::initAlgo();
  RCSGRAPH_FOREACH_BODY(graph)
  {
    RCSBODY_TRAVERSE_SHAPES(BODY)
    {
      RcsShape_setComputeType(SHAPE, RCSSHAPE_COMPUTE_DISTANCE, true);
    }
  }

  const RcsBody* bb = RcsGraph_getBodyByName(graph, belowBdy.c_str());
  RCHECK(bb);
  RCSBODY_TRAVERSE_SHAPES(bb)
  {
    RcsShape_setComputeType(SHAPE, RCSSHAPE_COMPUTE_DISTANCE, false);
  }

  return true;
}

void ExampleFK_Below::step()
{
  ExampleFK::step();

  const RcsBody* bb = RcsGraph_getBodyByName(graph, belowBdy.c_str());

  if (!bb)
  {
    RLOG(0, "Below-body %s not found", belowBdy.c_str());
    return;
  }


  double direction[3], distance;
  Vec3d_set(direction, 0.0, 0.0, -1.0);
  pthread_mutex_lock(&graphLock);
  const RcsBody* closest = RcsBody_closestInDirection(graph, bb->A_BI.org, direction, belowPt, &distance);
  pthread_mutex_unlock(&graphLock);

  snprintf(hudText, 256, "Body below %s is %s \nd=%f   pt=[%f %f %f]\n",
           bb->name, closest ? closest->name : "NULL", distance,
           belowPt[0], belowPt[1], belowPt[2]);
  if (hud)
  {
    hud->setText(hudText);
  }
  else
  {
    std::cout << hudText;
  }
}

std::string ExampleFK_Below::help()
{
  std::stringstream s;
  s << "  Raycast test\n\n";
  s << "  Here is how it works:\n";
  s << "\tWhen pressing the TAB-key, the green sphere can be dragged with \n";
  s << "\tthe TargetSetter. The red sphere below is indicating the \n";
  s << "\tintersection from the sphere's frame origin, casted vertically\n";
  s << "\tdownwards. In this example, all shapes regardless of their compute \n";
  s << "\ttype are considered.\n\n";
  s << Rcs::getResourcePaths();
  s << Rcs::CmdLineParser::printToString();
  s << Rcs::RcsGraph_printUsageToString(xmlFileName);
  return s.str();
}



/*******************************************************************************
 *
 ******************************************************************************/
RCS_REGISTER_EXAMPLE(ExampleFK_Broadphase, "Forward kinematics", "Broadphase");

ExampleFK_Broadphase::ExampleFK_Broadphase(int argc, char** argv) :
  ExampleFK(argc, argv), bp(NULL), cMdl(NULL), distanceThreshold(0.0),
  t_broadphase(0.0), t_narrowphase(0.0)
{
}

ExampleFK_Broadphase::~ExampleFK_Broadphase()
{
  // We delete the base class's viewer and gui  before deleting memory, since
  // otherwise some classes in different threads refer to invalid memory.
  clear();

  RcsBroadPhase_destroy(bp);
  RcsCollisionModel_destroy(cMdl);
}

bool ExampleFK_Broadphase::initParameters()
{
  ExampleFK::initParameters();
  xmlFileName = "gBroadphase.xml";
  directory = "config/xml/SmileActions";
  treeBodies = "j2s7s300_link_2_left j2s7s300_link_2_right";

  return true;
}

bool ExampleFK_Broadphase::initAlgo()
{
  ExampleFK::initAlgo();

  bp = RcsBroadPhase_create(graph, distanceThreshold);
  std::vector<std::string> treeBdyVec = Rcs::String_split(treeBodies, " ");
  for (size_t i = 0; i < treeBdyVec.size(); ++i)
  {
    RcsBroadPhase_addTreeByName(bp, treeBdyVec[i].c_str());
  }
  RcsBroadPhase_updateBoundingVolumes(bp);

  cMdl = RcsBroadPhase_createNarrowPhase(bp);

  return true;
}

void ExampleFK_Broadphase::step()
{
  bool updateHudPrev = updateHud;
  updateHud = false;
  ExampleFK::step();
  updateHud = updateHudPrev;

  double t_bp = Timer_getSystemTime();
  RcsBroadPhase_updateBoundingVolumes(bp);
  int nb = RcsBroadPhase_computeNarrowPhase(bp, cMdl);
  t_bp = Timer_getSystemTime() - t_bp;
  pthread_mutex_lock(&graphLock);
  double t_np = Timer_getSystemTime();
  RcsCollisionModel_compute(cMdl);
  t_np = Timer_getSystemTime() - t_np;
  pthread_mutex_unlock(&graphLock);

  t_broadphase = (t_broadphase>0.0) ? 0.99*t_broadphase + 0.01*t_bp : t_bp;
  t_narrowphase = (t_narrowphase>0.0) ? 0.99*t_narrowphase + 0.01*t_np : t_np;

  snprintf(hudText, 256, "%d of %d possible pairs\nBroad phase took %.3f msec\n"
           "Narrow phase took %.3f msec\nCompression is %.1f%%",
           cMdl->nPairs, nb, 1.0e3*t_broadphase, 1.0e3*t_narrowphase,
           100.0-100.0*cMdl->nPairs/nb);

  if (hud)
  {
    hud->setText(hudText);
  }
  else
  {
    std::cout << hudText;
  }

  REXEC(3)
  {
    RcsCollisionModel_fprint(stdout, cMdl);
  }

}

bool ExampleFK_Broadphase::initGraphics()
{
  Rcs::KeyCatcherBase::registerKey("k", "Toggle broadphase visualization");
  if (valgrind)
  {
    return true;
  }

  ExampleFK::initGraphics();
  bpNode = new osg::Switch();
  viewer->add(bpNode.get());

  gn->displayCollisionModel(true);
  gn->displayGraphicsModel(false);

#if 0
  for (unsigned int i = 0; i < bp->nBodies; ++i)
  {
    osg::ref_ptr<osg::PositionAttitudeTransform> pat;
    pat = new osg::PositionAttitudeTransform();
    pat->setPosition(osg::Vec3(bp->bodies[i].sphereCenter[0],
                               bp->bodies[i].sphereCenter[1],
                               bp->bodies[i].sphereCenter[2]));

    osg::ref_ptr<Rcs::SphereNode> sn;
    sn = new Rcs::SphereNode(Vec3d_zeroVec(), bp->bodies[i].sphereRadius);
    sn->makeDynamic(graph->bodies[bp->bodies[i].id].A_BI.org);
    sn->toggleWireframe();
    pat->addChild(sn.get());
    viewer->add(pat.get());
  }
#else
  for (unsigned int i = 0; i < bp->nBodies; ++i)
  {
    osg::ref_ptr<Rcs::AABBNode> sn = new Rcs::AABBNode();
    sn->makeDynamic(bp->bodies[i].aabbMin, bp->bodies[i].aabbMax);
    bpNode->addChild(sn.get());
  }
  for (unsigned int i = 0; i < bp->nTrees; ++i)
  {
    osg::ref_ptr<Rcs::AABBNode> sn = new Rcs::AABBNode();
    sn->makeDynamic(bp->trees[i].aabbMin, bp->trees[i].aabbMax);
    bpNode->addChild(sn.get());
    for (unsigned int j = 0; j < bp->trees[i].nBodies; ++j)
    {
      osg::ref_ptr<Rcs::AABBNode> sn = new Rcs::AABBNode();
      sn->makeDynamic(bp->trees[i].bodies[j].aabbMin,
                      bp->trees[i].bodies[j].aabbMax);
      bpNode->addChild(sn.get());
    }

  }
#endif

  osg::ref_ptr<Rcs::VertexArrayNode> cn;
  cn = new Rcs::VertexArrayNode(cMdl->cp, osg::PrimitiveSet::LINES, "RED");
  viewer->add(cn.get());


  return true;
}

void ExampleFK_Broadphase::handleKeys()
{
  if (!kc)
  {
    return;
  }

  ExampleFK::handleKeys();

  if (kc->getAndResetKey('k'))
  {
    bool visible = bpNode.get()->getValue(0);
    if (visible)
    {
      bpNode->setAllChildrenOff();
    }
    else
    {
      bpNode->setAllChildrenOn();
    }
  }

}

bool ExampleFK_Broadphase::parseArgs(CmdLineParser* argP)
{
  argP->getArgument("-distanceThreshold", &distanceThreshold,
                    "AABB distance threshold (default is 0)");
  argP->getArgument("-treeBodies", &treeBodies, "Space-separated strings of "
                    "tree bodies to be checked against rigid bodies (default"
                    " is \"%s\")", treeBodies.c_str());

  return ExampleFK::parseArgs(argP);
}


}   // namespace Rcs
