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

#include <RcsViewer.h>
#include <Rcs_cmdLine.h>
#include <Rcs_macros.h>
#include <Rcs_timer.h>
#include <Rcs_resourcePath.h>
#include <Rcs_math.h>
#include <Rcs_mesh.h>
#include <Rcs_parser.h>

#include <COSNode.h>
#include <ArrowNode.h>
#include <VertexArrayNode.h>
#include <GraphNode.h>
#include <KeyCatcher.h>
#include <Rcs_graphicsUtils.h>
#include <Rcs_utils.h>
#include <MeshNode.h>

#include <osg/Node>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>

#include <csignal>

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
 * Test for VertexArrayNode
 ******************************************************************************/
static void testVertexArrayNode()
{
  Rcs::CmdLineParser argP;

  // Initialize GUI and OSG mutex
  pthread_mutex_t mtx;
  pthread_mutex_init(&mtx, NULL);

  MatNd* rndMat = MatNd_create(1000, 3);
  MatNd_setRandom(rndMat, -0.5, 0.5);

  char oglMode[32] = "Points";
  argP.getArgument("-oglMode", oglMode, "OpenGL mode: Points, Lines ...");

  if (argP.hasArgument("-h"))
  {
    return;
  }


  osg::ref_ptr<Rcs::VertexArrayNode> vn;

  if (STREQ(oglMode, "Points"))
  {
    vn = new Rcs::VertexArrayNode(osg::PrimitiveSet::POINTS);
  }
  else if (STREQ(oglMode, "Lines"))
  {
    vn = new Rcs::VertexArrayNode(osg::PrimitiveSet::LINES);
  }

  RCHECK(vn.valid());

  vn->setPoints(rndMat);
  vn->setPointSize(5.0);

  Rcs::Viewer* viewer = new Rcs::Viewer();
  viewer->add(new Rcs::COSNode());
  viewer->add(vn.get());
  viewer->runInThread(&mtx);

  RMSG("Hit any key to quit");

  while (!Rcs_kbhit())
  {
    pthread_mutex_lock(&mtx);
    MatNd_setRandom(rndMat, -0.5, 0.5);
    pthread_mutex_unlock(&mtx);
    Timer_waitDT(0.01);
  }

  delete viewer;
  MatNd_destroy(rndMat);
}

/*******************************************************************************
 * Test for MeshNode
 ******************************************************************************/
static void testMeshNode()
{
  char meshFile[256] = "/hri/sit/latest/Data/RobotMeshes/1.0/data/Schunk/SDH_Gehaeuse_x.tri";
  Rcs::CmdLineParser argP;
  argP.getArgument("-f", meshFile, "Mesh file (default is %s)", meshFile);

  Rcs::MeshNode* mn = new Rcs::MeshNode(meshFile);



  Rcs::Viewer* viewer = new Rcs::Viewer();
  viewer->add(mn);
  viewer->runInThread();

  RPAUSE();

  delete viewer;
}

/*******************************************************************************
 * Test for viewer setCamera functions
 ******************************************************************************/
static void testCameraTransform()
{
  Rcs::KeyCatcherBase::registerKey("q", "Quit");
  Rcs::KeyCatcherBase::registerKey("a", "Print out current camera transform");
  Rcs::KeyCatcherBase::registerKey("b", "Set camera transform to itself");
  Rcs::KeyCatcherBase::registerKey("+", "Shift camera transform to right");
  Rcs::KeyCatcherBase::registerKey("-", "Shift camera transform to left");
  Rcs::KeyCatcherBase::registerKey("e", "Rotate camera transform about z");
  Rcs::KeyCatcherBase::registerKey("E", "Align camera transform horizontally");
  Rcs::KeyCatcherBase::registerKey("Z", "Increase field of view");
  Rcs::KeyCatcherBase::registerKey("z", "Decrease field of view");

  // Parse command line arguments
  char xmlFileName[128] = "gScenario.xml";
  char directory[128] = "config/xml/GenericHumanoid";
  Rcs::CmdLineParser argP;
  argP.getArgument("-f", xmlFileName, "Configuration file name");
  argP.getArgument("-dir", directory, "Configuration file directory");

  pthread_mutex_t mtx;
  pthread_mutex_init(&mtx, NULL);

  Rcs_addResourcePath("config");
  Rcs_addResourcePath(directory);

  RcsGraph* graph = RcsGraph_create(xmlFileName);
  Rcs::Viewer* viewer = new Rcs::Viewer();
  Rcs::KeyCatcher* kc = new Rcs::KeyCatcher();
  Rcs::COSNode* cn = new Rcs::COSNode();

  viewer->add(new Rcs::GraphNode(graph));
  viewer->add(cn);
  viewer->add(kc);
  viewer->runInThread(&mtx);

  HTr A_CI;   // World to camera transform

  while (runLoop)
  {

    pthread_mutex_lock(&mtx);

    if (kc->getAndResetKey('q'))
    {
      runLoop = false;
    }
    else if (kc->getAndResetKey('a'))
    {
      viewer->getCameraTransform(&A_CI);
      RMSGS("Camera transform A_CI:");
      HTr_fprint(stdout, &A_CI);
      cn->setPosition(A_CI.org);
      cn->setRotation(A_CI.rot);
    }
    else if (kc->getAndResetKey('b'))
    {
      viewer->getCameraTransform(&A_CI);
      RMSGS("Camera transform A_CI:");
      HTr_fprint(stdout, &A_CI);

      viewer->setCameraTransform(&A_CI);
      HTr A_CI2;
      viewer->getCameraTransform(&A_CI2);
      RMSGS("Camera transform after setting:");
      HTr_fprint(stdout, &A_CI2);
    }
    else if (kc->getAndResetKey('+'))
    {
      viewer->getCameraTransform(&A_CI);
      RMSGS("Camera transform A_CI:");
      HTr_fprint(stdout, &A_CI);
      A_CI.org[1] += 0.01;
      viewer->setCameraTransform(&A_CI);
      viewer->getCameraTransform(&A_CI);
      RMSGS("Camera transform after setting:");
      HTr_fprint(stdout, &A_CI);
    }
    else if (kc->getAndResetKey('-'))
    {
      viewer->getCameraTransform(&A_CI);
      RMSGS("Camera transform A_CI:");
      HTr_fprint(stdout, &A_CI);
      A_CI.org[1] -= 0.01;
      viewer->setCameraTransform(&A_CI);
      viewer->getCameraTransform(&A_CI);
      RMSGS("Camera transform after setting:");
      HTr_fprint(stdout, &A_CI);
    }
    else if (kc->getAndResetKey('E'))
    {
      HTr_setIdentity(&A_CI);
      A_CI.org[0] = -2.0;
      A_CI.org[2] =  1.0;
      viewer->setCameraTransform(&A_CI);
      viewer->getCameraTransform(&A_CI);
      RMSGS("Camera transform after setting:");
      HTr_fprint(stdout, &A_CI);
    }
    else if (kc->getAndResetKey('e'))
    {
      viewer->getCameraTransform(&A_CI);
      RMSGS("Camera transform A_CI:");
      HTr_fprint(stdout, &A_CI);
      Mat3d_rotateSelfAboutXYZAxis(A_CI.rot, 2, 5.0*M_PI/180.0);
      viewer->setCameraTransform(&A_CI);
      viewer->getCameraTransform(&A_CI);
      RMSGS("Camera transform after setting:");
      HTr_fprint(stdout, &A_CI);
    }
    else if (kc->getAndResetKey('Z'))
    {
      double fov = viewer->getFieldOfView();
      RMSGS("Field of view: %f", fov);
      viewer->setFieldOfView(fov + 5.0);
      fov = viewer->getFieldOfView();
      RMSGS("Field of view after setting: %f", fov);
    }
    else if (kc->getAndResetKey('z'))
    {
      double fov = viewer->getFieldOfView();
      RMSGS("Field of view: %f", fov);
      viewer->setFieldOfView(fov - 5.0);
      fov = viewer->getFieldOfView();
      RMSGS("Field of view after setting: %f", fov);
    }

    pthread_mutex_unlock(&mtx);

    Timer_waitDT(0.01);
  }

  delete viewer;
  RcsGraph_destroy(graph);
  pthread_mutex_destroy(&mtx);
}

/*******************************************************************************
 * Test for viewer setCamera functions
 ******************************************************************************/
static void testArrowNode()
{
  double org[3], dir[3], radius = 0.1, length = 1.0;
  Vec3d_setZero(org);
  Vec3d_setUnitVector(dir, 2);


  Rcs::CmdLineParser argP;
  argP.getArgument("-x", &org[0], "X-position of arrow origin");
  argP.getArgument("-y", &org[1], "Y-position of arrow origin");
  argP.getArgument("-z", &org[2], "Z-position of arrow origin");
  argP.getArgument("-r", &radius, "Radius of arrow");
  argP.getArgument("-l", &length, "Length of arrow");

  Rcs::ArrowNode* an1 = new Rcs::ArrowNode();
  an1->setPosition(org);
  an1->setDirection(dir);
  an1->setRadius(radius);
  an1->setArrowLength(length);



  Rcs::Viewer* viewer = new Rcs::Viewer();
  viewer->add(an1);
  viewer->add(new Rcs::COSNode());
  viewer->runInThread();

  RPAUSE();

  delete viewer;
}

/*******************************************************************************
 * Select test modes
 ******************************************************************************/
int main(int argc, char** argv)
{
  int mode = -1;

  // Ctrl-C callback handler
  signal(SIGINT, quit);

  // Parse command line arguments
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-dl", &RcsLogLevel, "Debug level (default is 0)");
  argP.getArgument("-m", &mode, "Test mode");

  switch (mode)
  {
    case -1:
      printf("\nHere's some useful testing modes:\n\n");
      printf("\t-m");
      printf("\t-1   Prints this message (default)\n");
      printf("\t\t0    Test viewer with VertexArrayNode\n");
      printf("\t\t1    Test MeshNode\n");
      printf("\t\t2    Test camera transformation\n");
      printf("\t\t3    Test ArrowNode\n");
      break;

    case 0:
      testVertexArrayNode();
      break;

    case 1:
      testMeshNode();
      break;

    case 2:
      testCameraTransform();
      if (argP.hasArgument("-repeat"))
      {
        runLoop = true;
        testCameraTransform();
      }
      break;

    case 3:
      testArrowNode();
      break;

    default:
      RFATAL("No mode %d", mode);
  }


  if (argP.hasArgument("-h"))
  {
    argP.print();
  }

  xmlCleanupParser();
  fprintf(stderr, "Thanks for using the ViewerTest\n");

  return 0;
}
