/*******************************************************************************

  Copyright (c) 2017, Honda Research Institute Europe GmbH.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.

  3. All advertising materials mentioning features or use of this software
     must display the following acknowledgement: This product includes
     software developed by the Honda Research Institute Europe GmbH.

  4. Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER "AS IS" AND ANY EXPRESS OR
  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#include <BulletSoftSimulation.h>
#include <BulletHelpers.h>
#include <BulletDebugDrawer.h>
#include <Rcs_macros.h>
#include <Rcs_cmdLine.h>
#include <Rcs_math.h>
#include <Rcs_resourcePath.h>
#include <Rcs_timer.h>
#include <Rcs_typedef.h>
#include <RcsViewer.h>
#include <MeshNode.h>
#include <SegFaultHandler.h>

#include <libxml/tree.h>

#include <iostream>
#include <csignal>
#include <map>

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


/******************************************************************************
 * SoftBullet simulation and graphics update example
 *****************************************************************************/
static void test_softBody(int argc, char** argv)
{
  double dt = 0.005;
  char xmlFileName[128] = "gSoftPhysics.xml";
  char directory[128] = "config/xml/Examples";
  char cfg[128] = "config/physics/physics.xml";
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-f", xmlFileName, "Configuration file name");
  argP.getArgument("-dir", directory, "Configuration file directory");
  argP.getArgument("-physics_config", cfg, "Configuration file name for "
                   "physics (default is %s)", cfg);
  argP.getArgument("-dt", &dt, "Simulation time step (default is %f)", dt);
  Rcs_addResourcePath(directory);

  // Create a kinematic graph from the given xml file.
  RcsGraph* graph = RcsGraph_create(xmlFileName);

  // Create a soft bullet simulation of the graph. Since in this example, we
  // keep things very basic, we don't use the visualization classes based on
  // the graph. These assume that all geometries are parented by the shapes.
  // Here instead we use the simulated soft vertices which we convert into
  // world coordinates. That's why we use sim->transformVerticesToWorld().
  Rcs::BulletSoftSimulation sim;
  sim.initialize(graph, cfg);
  sim.transformVerticesToWorld();

  // Create visualizations for OpenSceneGraph graphics. The viewer is the top-
  // level window. We add a few visualization nodes to it.
  Rcs::Viewer viewer;

  // The debug drawer draws a set of lines that make up the geometries of all
  // physics entities. It is computed inside Bullet. Therefore we need to add
  // it also to the simulator. The debug drawer also shows all Bullet rigid
  // bodies. You can spot them as wire frame objects only, since we don't
  // add a MeshNode for them.
  Rcs::BulletDebugDrawer* debugDrawer = new Rcs::BulletDebugDrawer();
  debugDrawer->setDebugMode(btIDebugDraw::DBG_DrawWireframe);
  viewer.add(debugDrawer);
  sim.setDebugDrawer(debugDrawer);

  // In the following, we create a MeshNode for each soft body, and add it
  // to the viewer. Since later, we update all meshes from the simulation,
  // we create a simple convenience map that maps the MeshNode to the mesh
  // data that has been updated inside Bullet.
  std::map<osg::ref_ptr<Rcs::MeshNode>, RcsMeshData*> meshMap;

  // Walk through all bodies of the graph. Each traversed body can be
  // referenced by the variable BODY.
  RCSGRAPH_TRAVERSE_BODIES(sim.getGraph())
  {
    // Walk through all shapes of the body. Each traversed shape can be
    // referenced by the variable SHAPE.
    RCSBODY_TRAVERSE_SHAPES(BODY)
    {
      // We skip the shape if it is not labelled as a soft physics shape. In
      // the xml file, there must be a softPhysics="true" to make the shape
      // being simulated in the soft body simulation.
      if ((SHAPE->computeType & RCSSHAPE_COMPUTE_SOFTPHYSICS) == 0)
      {
        continue;
      }

      // If we found a mesh for the shape, we create a MeshNode and add it
      // to the map. Otherwise, we emit a warning on the default debug level.
      RcsMeshData* mesh = (RcsMeshData*)SHAPE->userData;
      if (mesh)
      {
        RLOG(0, "Found mesh for body %s", BODY->name);
        osg::ref_ptr<Rcs::MeshNode> mn;
        mn = new Rcs::MeshNode(mesh->vertices, mesh->nVertices,
                               mesh->faces, mesh->nFaces);
        viewer.add(mn.get());
        meshMap[mn] = mesh;
      }
      else
      {
        RLOG(0, "Failed to add mesh for body %s", BODY->name);
      }
    }
  }


  // End-less loop that simulates and updates the visualization. Can be
  // stopped with Ctrl-C: The signal is caught by quit() which sets
  // runLoop to false.
  while (runLoop)
  {
    // Performs one simulation step for the given time increment
    sim.step(dt);

    // Traverse all mesh nodes and update the vertices from the
    // simulator.
    std::map<osg::ref_ptr<Rcs::MeshNode>, RcsMeshData*>::iterator it;
    for (it = meshMap.begin(); it != meshMap.end(); it++)
    {
      osg::ref_ptr<Rcs::MeshNode> mn = it->first;
      RcsMeshData* msh = it->second;
      mn->setMesh(msh->vertices, msh->nVertices, msh->faces, msh->nFaces);
    }

    // Update graphics
    viewer.frame();

    // Wait a little bit to let the CPU breathe
    Timer_usleep(1000);

    // On debug level 5, we wait for an enter key here. The debug level can be
    // set through the command line argument "-dl 5", or by pressing the number
    // key 5 in the viewer window.
    RPAUSE_DL(5);
  }


  // Clean up heap memory.
  RcsGraph_destroy(graph);
}

/******************************************************************************
 *
 *****************************************************************************/
static void test_mesh(int argc, char** argv)
{
  std::string meshFile;
  std::string outFile = "reducedMesh.tri";

  const char* hgr = getenv("SIT");

  if (hgr != NULL)
  {
    meshFile = std::string(hgr) + std::string("/Data/RobotMeshes/1.0/data/") +
               std::string("iiwa_description/meshes/iiwa14/visual/link_1.stl");
  }

  Rcs::CmdLineParser argP;
  argP.getArgument("-f", &meshFile, "Name of mesh file (default is %s)",
                   meshFile.c_str());
  argP.getArgument("-o", &outFile, "Name of output file (default is %s)",
                   outFile.c_str());

  if (argP.hasArgument("-h", "Show help message"))
  {
    Rcs_printResourcePath();
    argP.print();
    return;
  }

  Rcs::convertMesh(outFile.c_str(), meshFile.c_str());
}

/*******************************************************************************
 *
 ******************************************************************************/
int main(int argc, char** argv)
{
  // Ctrl-C callback handler
  signal(SIGINT, quit);

  // This initialize the xml library and check potential mismatches between
  // the version it was compiled for and the actual shared library used.
  LIBXML_TEST_VERSION;

  // Parse command line arguments
  int mode = 0;
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
      printf("\t0   Prints this message (default)\n");
      printf("\t\t1   Mesh conversion to convex hull\n");
      printf("\t\t2   Bullet soft body simulation with native meshes\n");
      break;
    }

    case 1:
    {
      test_mesh(argc, argv);
      break;
    }

    case 2:
    {
      test_softBody(argc, argv);
      break;
    }

    default:
    {
      RMSG("there is no mode %d", mode);
    }

  } // switch(mode)

  // Clean up global stuff
  xmlCleanupParser();

  fprintf(stderr, "Thanks for using the ExampleBullet app\n");

  return 0;
}
