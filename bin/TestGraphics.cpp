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
#include <Rcs_typedef.h>

#include <COSNode.h>
#include <ArrowNode.h>
#include <VertexArrayNode.h>
#include <GraphNode.h>
#include <KeyCatcher.h>
#include <Rcs_utils.h>
#include <MeshNode.h>

#include <osgGA/TrackballManipulator>
#include <osgDB/Registry>
#include <osgFX/Cartoon>

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

/******************************************************************************
 *
 *****************************************************************************/
static void showMesh(const RcsMeshData* mesh)
{
  Rcs::CmdLineParser argP;
  bool valgrind = argP.hasArgument("-valgrind");

  if (valgrind || (mesh==NULL))
  {
    return;
  }

  REXEC(1)
  {
    RcsMesh_print(mesh);
  }
  Rcs::MeshNode* mn = new Rcs::MeshNode(mesh->vertices, mesh->nVertices,
                                        mesh->faces, mesh->nFaces);
  Rcs::Viewer* viewer = new Rcs::Viewer();
  viewer->add(mn);
  viewer->add(new Rcs::COSNode());
  viewer->runInThread();

  RPAUSE();

  delete viewer;
}

/******************************************************************************
 *
 *****************************************************************************/
static bool test_ssrMesh()
{
  double extents[3];
  Vec3d_set(extents, 1.0, 0.5, 0.2);
  unsigned int segments = 16;

  Rcs::CmdLineParser argP;
  argP.getArgument("-x", &extents[0], "X-dimension (default is %f)",
                   extents[0]);
  argP.getArgument("-y", &extents[1], "Y-dimension (default is %f)",
                   extents[1]);
  argP.getArgument("-z", &extents[2], "Z-dimension (default is %f)",
                   extents[2]);
  argP.getArgument("-segments", &segments, "Segments (default is %d)",
                   segments);

  if (argP.hasArgument("-h"))
  {
    return true;
  }

  RcsMeshData* mesh = RcsMesh_createSSR(extents, segments);

  showMesh(mesh);
  RcsMesh_destroy(mesh);

  return true;
}

/******************************************************************************
 *
 *****************************************************************************/
static bool test_coneMesh()
{
  double radius = 0.5, height = 1.0;
  unsigned int segments = 32;

  Rcs::CmdLineParser argP;
  argP.getArgument("-r", &radius, "Radius (default is %f)", radius);
  argP.getArgument("-height", &height, "Heigth (default is %f)", height);
  argP.getArgument("-segments", &segments, "Segments (default is %d)",
                   segments);

  if (argP.hasArgument("-h"))
  {
    return true;
  }

  RcsMeshData* mesh = RcsMesh_createCone(radius, 1.0, segments);

  showMesh(mesh);
  RcsMesh_destroy(mesh);

  return true;
}

/******************************************************************************
 *
 *****************************************************************************/
static bool test_torusMesh()
{
  double radius = 0.5, height = 0.1;
  unsigned int rSegments = 16, tSegments = 16;

  Rcs::CmdLineParser argP;
  argP.getArgument("-r", &radius, "Radius (default is %f)", radius);
  argP.getArgument("-height", &height, "Heigth (default is %f)", height);
  argP.getArgument("-radial", &rSegments, "Radial segments (default is %d)",
                   rSegments);
  argP.getArgument("-tubular", &tSegments, "Tubular segments (default is %d)",
                   tSegments);

  if (argP.hasArgument("-h"))
  {
    return true;
  }

  RcsMeshData* mesh = RcsMesh_createTorus(radius, height, rSegments, tSegments);

  showMesh(mesh);
  RcsMesh_destroy(mesh);

  return true;
}

/******************************************************************************
 *
 *****************************************************************************/
static bool test_capsuleMesh()
{
  double radius = 0.5, height = 1.0;
  unsigned int segments = 16;

  Rcs::CmdLineParser argP;
  argP.getArgument("-r", &radius, "Radius (default is %f)", radius);
  argP.getArgument("-height", &height, "Heigth (default is %f)", height);
  argP.getArgument("-segments", &segments, "Segments (default is %d)",
                   segments);

  if (argP.hasArgument("-h"))
  {
    return true;
  }

  RcsMeshData* mesh = RcsMesh_createCapsule(radius, 1.0, segments);

  showMesh(mesh);
  RcsMesh_destroy(mesh);

  return true;
}

/******************************************************************************
 *
 *****************************************************************************/
static bool test_sphereMesh()
{
  double radius = 0.5;
  unsigned int segments = 16;

  Rcs::CmdLineParser argP;
  argP.getArgument("-r", &radius, "Radius (default is %f)", radius);
  argP.getArgument("-segments", &segments, "Segments (default is %d)",
                   segments);

  if (argP.hasArgument("-h"))
  {
    return true;
  }

  RcsMeshData* mesh = RcsMesh_createSphere(radius, segments);

  showMesh(mesh);
  RcsMesh_destroy(mesh);

  return true;
}

/******************************************************************************
 *
 *****************************************************************************/
static bool test_sphereSegmentMesh()
{
  double radius = 0.5;
  unsigned int heightSegments = 16;
  unsigned int widthSegments = 32;
  double phiStart = 0.0;
  double phiLength = 360.0;
  double thetaStart = 0.0;
  double thetaLength = 180.0;

  Rcs::CmdLineParser argP;
  argP.getArgument("-r", &radius, "Radius (default is %f)", radius);
  argP.getArgument("-heightSegments", &heightSegments, "Height segments "
                   "(default is %d)", heightSegments);
  argP.getArgument("-widthSegments", &widthSegments, "Width segments (default"
                   "is %d)", widthSegments);
  argP.getArgument("-phiStart", &phiStart, "Phi start [deg](default is %f)",
                   phiStart);
  argP.getArgument("-phiLength", &phiLength, "Phi length [deg](default is %f)",
                   phiLength);
  argP.getArgument("-thetaStart", &thetaStart, "Theta start [deg](default "
                   "is %f)", thetaStart);
  argP.getArgument("-thetaLength", &thetaLength, "Theta length [deg](default"
                   "is %f)", thetaLength);

  if (argP.hasArgument("-h"))
  {
    return true;
  }

  phiStart*= M_PI/180.0;
  phiLength*= M_PI/180.0;
  thetaStart*= M_PI/180.0;
  thetaLength*= M_PI/180.0;

  RcsMeshData* mesh = RcsMesh_createSphereSegment(radius, heightSegments,
                                                  widthSegments, phiStart,
                                                  phiLength, thetaStart,
                                                  thetaLength);

  showMesh(mesh);
  RcsMesh_destroy(mesh);

  return true;
}

/******************************************************************************
 *
 *****************************************************************************/
static bool test_cylinderMesh()
{
  double radius = 0.5;
  double height = 1.0;
  unsigned int radialSegments = 16;
  unsigned int heightSegments = 4;
  double angleAround = 360.0;

  Rcs::CmdLineParser argP;
  argP.getArgument("-r", &radius, "Radius (default is %f)", radius);
  argP.getArgument("-h", &height, "Height (default is %f)", height);
  argP.getArgument("-angleAround", &angleAround, "Angle around [deg](default"
                   "is %f)", angleAround);
  argP.getArgument("-radialSegments", &radialSegments, "Radial segments "
                   "(default is %d)", radialSegments);
  argP.getArgument("-heightSegments", &heightSegments, "Height segments "
                   "(default is %d)", heightSegments);

  if (argP.hasArgument("-h"))
  {
    return true;
  }

  angleAround *= M_PI/180.0;

  RcsMeshData* mesh = RcsMesh_createCylinder(radius, height, 32);

  showMesh(mesh);
  RcsMesh_destroy(mesh);

  return true;
}

/******************************************************************************
 *
 *****************************************************************************/
static bool test_cylinderHullMesh()
{
  double radius = 0.5;
  double height = 1.0;
  unsigned int radialSegments = 16;
  unsigned int heightSegments = 4;
  double angleAround = 360.0;

  Rcs::CmdLineParser argP;
  argP.getArgument("-r", &radius, "Radius (default is %f)", radius);
  argP.getArgument("-h", &height, "Height (default is %f)", height);
  argP.getArgument("-angleAround", &angleAround, "Angle around [deg](default "
                   "is %f)", angleAround);
  argP.getArgument("-radialSegments", &radialSegments, "Radial segments "
                   "(default is %d)", radialSegments);
  argP.getArgument("-heightSegments", &heightSegments, "Height segments "
                   "(default is %d)", heightSegments);

  if (argP.hasArgument("-h"))
  {
    return true;
  }

  angleAround *= M_PI/180.0;

  RcsMeshData* mesh = RcsMesh_createCylinderHull(radius, 0.5*radius, height,
                                                 radialSegments, heightSegments,
                                                 angleAround);

  showMesh(mesh);
  RcsMesh_destroy(mesh);

  return true;
}

/******************************************************************************
 *
 *****************************************************************************/
static bool test_meshify()
{
  char xmlFileName[128] = "gScenario.xml";
  char directory[128] = "config/xml/DexBot";
  double scale = 1.0;
  int computeType = RCSSHAPE_COMPUTE_GRAPHICS;
  Rcs::CmdLineParser argP;
  argP.getArgument("-f", xmlFileName, "Configuration file name");
  argP.getArgument("-dir", directory, "Configuration file directory");
  argP.getArgument("-scale", &scale, "Scaling factor, default is %f", scale);
  argP.getArgument("-computeType", &computeType, "Compute type: 1 distance, "
                   "2 physics, 4 graphics, default is %d", computeType);

  Rcs_addResourcePath("config");
  Rcs_addResourcePath(directory);
  RcsGraph* graph = RcsGraph_create(xmlFileName);
  RcsMeshData* allMesh = RcsGraph_meshify(graph, scale, computeType);
  bool success = allMesh ? true : false;

  if (success)
  {
    RcsMesh_toFile(allMesh, "AllMesh.stl");
    RcsMesh_destroy(allMesh);
  }

  RcsGraph_destroy(graph);

  return success;
}

/*******************************************************************************
* Test for native viewer
******************************************************************************/
static void testOsgViewer()
{
  Rcs::CmdLineParser argP;

  // Rotate loaded file nodes to standard coordinate conventions
  // (z: up, x: forward)
  osgDB::ReaderWriter::Options* options = new osgDB::ReaderWriter::Options;
  options->setOptionString("noRotation");
  osgDB::Registry::instance()->setOptions(options);

  osgViewer::Viewer* viewer = new osgViewer::Viewer();

  osg::ref_ptr<osgGA::TrackballManipulator> trackball = new osgGA::TrackballManipulator();
  viewer->setCameraManipulator(trackball.get());

  osg::ref_ptr<osg::Group> rootnode;

  // Cartoon mode
  if (argP.hasArgument("-cartoon", "Test Cartoon mode"))
  {
    rootnode = new osgFX::Cartoon;
  }
  else
  {
    rootnode = new osg::Group;
  }

  rootnode->addChild(new Rcs::COSNode());

  // Light grayish blue universe
  if (argP.hasArgument("-clearNode", "Test ClearNode"))
  {
    osg::ref_ptr<osg::ClearNode> clearNode = new osg::ClearNode;
    clearNode->setClearColor(osg::Vec4(0.8, 0.8, 0.95, 1.0));
    rootnode->addChild(clearNode.get());
  }

  // Same as above, but with setting camera background explicitely
  if (argP.hasArgument("-clearColor", "Test background color"))
  {
    viewer->getCamera()->setClearColor(osg::Vec4(0.8, 0.8, 0.95, 1.0));
  }

  // Disable small feature culling to avoid problems with drawing single points
  // as they have zero bounding box size
  if (argP.hasArgument("-smallFeatures", "Test small feature culling"))
  {
    RLOG(1, "Test small feature culling enabled");
    viewer->getCamera()->setCullingMode(viewer->getCamera()->getCullingMode() &
                                        ~osg::CullSettings::SMALL_FEATURE_CULLING);
  }

  // Light model: We switch off the default viewer light, and configure two
  // light sources. The sunlight shines down from 10m. Another light source
  // moves with the camera, so that there are no dark spots whereever
  // the mouse manipulator moves to.
  if (argP.hasArgument("-antialias", "Test anti-aliasing"))
  {
    RLOG(1, "Anti-aliasing test enabled");

    // Set anti-aliasing
    osg::ref_ptr<osg::DisplaySettings> ds = new osg::DisplaySettings;
    ds->setNumMultiSamples(4);
    viewer->setDisplaySettings(ds.get());
  }

  osg::ref_ptr<osg::LightSource> sunlight;

  if (argP.hasArgument("-light", "Test light"))
  {
    RLOG(1, "Light test enabled");

    // Disable default light
    rootnode->getOrCreateStateSet()->setMode(GL_LIGHT0, osg::StateAttribute::OFF);

    // Light source that moves with the camera
    osg::ref_ptr<osg::LightSource> cameraLight = new osg::LightSource;
    cameraLight->getLight()->setLightNum(1);
    //cameraLight->getLight()->setPosition(osg::Vec4(0.0, 0.0, 10.0, 1.0));
    cameraLight->getLight()->setSpecular(osg::Vec4(1.0, 1.0, 1.0, 1.0));
    rootnode->addChild(cameraLight.get());
    rootnode->getOrCreateStateSet()->setMode(GL_LIGHT1, osg::StateAttribute::ON);

    // Light source that shines down
    sunlight = new osg::LightSource;
    sunlight->getLight()->setLightNum(2);
    //sunlight->getLight()->setPosition(osg::Vec4(0.0, 0.0, 10.0, 1.0));
    rootnode->addChild(sunlight.get());
    rootnode->getOrCreateStateSet()->setMode(GL_LIGHT2, osg::StateAttribute::ON);
  }

  if (argP.hasArgument("-shadow", "Test shadows"))
  {
    RLOG(1, "Shadow test enabled");
    // Shadow map scene. We use the sunlight to case shadows.
    osg::ref_ptr<osgShadow::ShadowedScene> shadowScene = new osgShadow::ShadowedScene;
    osg::ref_ptr<osgShadow::ShadowMap> sm = new osgShadow::ShadowMap;
    sm->setTextureSize(osg::Vec2s(2048, 2048));
    if (sunlight.valid())
    {
      sm->setLight(sunlight->getLight());
    }
    sm->setPolygonOffset(osg::Vec2(-0.7, 0.0));
    sm->setAmbientBias(osg::Vec2(0.7, 0.3));   // values need to sum up to 1.0

    shadowScene->setShadowTechnique(sm.get());
    shadowScene->addChild(rootnode.get());
    //shadowScene->setReceivesShadowTraversalMask(ReceivesShadowTraversalMask);
    //shadowScene->setCastsShadowTraversalMask(CastsShadowTraversalMask);
    viewer->setSceneData(shadowScene.get());
  }
  else
  {
    viewer->setSceneData(rootnode.get());
  }


  if (argP.hasArgument("-cullThread", "Test osgViewer::Viewer::CullDrawThreadPerContext"))
  {
    // Change the threading model. The default threading model is
    // osgViewer::Viewer::CullThreadPerCameraDrawThreadPerContext.
    // This leads to problems with multi-threaded updates (HUD).
    viewer->setThreadingModel(osgViewer::Viewer::CullDrawThreadPerContext);
  }


  if (argP.hasArgument("-graph", "Add GraphNode"))
  {
    RcsGraph* graph = RcsGraph_create("config/xml/DexBot/LBR.xml");
    RCHECK(graph);
    osg::ref_ptr<Rcs::GraphNode> gn = new Rcs::GraphNode(graph);
    rootnode->addChild(gn.get());
  }


  if (argP.hasArgument("-body", "Add BodyNode"))
  {
    RcsGraph* graph = RcsGraph_create("config/xml/DexBot/LBR.xml");
    RCHECK(graph);
    osg::ref_ptr<Rcs::BodyNode> gn = new Rcs::BodyNode(graph->root);
    rootnode->addChild(gn.get());
  }

  if (argP.hasArgument("-h"))
  {
    delete viewer;
    return;
  }


  viewer->setUpViewInWindow(12, 38, 640, 480);
  viewer->realize();
  viewer->run();

  delete viewer;
}

/*******************************************************************************
 * Test for VertexArrayNode
 ******************************************************************************/
static void testVertexArrayNode()
{
  Rcs::CmdLineParser argP;

  char oglMode[32] = "Points";
  argP.getArgument("-oglMode", oglMode, "OpenGL mode: Points, Lines ...");

  if (argP.hasArgument("-h"))
  {
    return;
  }

  pthread_mutex_t mtx;
  pthread_mutex_init(&mtx, NULL);

  MatNd* rndMat = MatNd_create(1000, 3);
  MatNd_setRandom(rndMat, -0.5, 0.5);

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
  pthread_mutex_destroy(&mtx);
}

/*******************************************************************************
 * Test for MeshNode
 ******************************************************************************/
static void testMeshNode()
{
  char meshFile[256] = "";
  Rcs::CmdLineParser argP;
  argP.getArgument("-f", meshFile, "Mesh file (default is %s)", meshFile);

  if (argP.hasArgument("-h"))
  {
    return;
  }

  Rcs::MeshNode* mn = NULL;

  if (strlen(meshFile)>0)
  {
    mn = new Rcs::MeshNode(meshFile);
  }
  else
  {
    RcsMeshData* mesh = RcsMesh_createTorus(0.5, 0.25, 32, 32);
    mn = new Rcs::MeshNode(mesh->vertices, mesh->nVertices,
                           mesh->faces, mesh->nFaces);
    RcsMesh_destroy(mesh);
  }


  Rcs::Viewer* viewer = new Rcs::Viewer();
  viewer->add(mn);
  viewer->add(new Rcs::COSNode());
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
      printf("\t\t4    Test nodes with native osgViewer\n");
      printf("\t\t5    Test mesh creation from graph\n");
      printf("\t\t6    Test cylinder hull mesh\n");
      printf("\t\t7    Test cylinder mesh\n");
      printf("\t\t8    Test sphere segment mesh\n");
      printf("\t\t9    Test sphere mesh\n");
      printf("\t\t10   Test capsule mesh\n");
      printf("\t\t11   Test torus mesh\n");
      printf("\t\t12   Test cone mesh\n");
      printf("\t\t13   Test sphere swept rectangle mesh\n");
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

    case 4:
      testOsgViewer();
      break;

    case 5:
    {
      test_meshify();
      break;
    }

    case 6:
    {
      test_cylinderHullMesh();
      break;
    }

    case 7:
    {
      test_cylinderMesh();
      break;
    }

    case 8:
    {
      test_sphereSegmentMesh();
      break;
    }

    case 9:
    {
      test_sphereMesh();
      break;
    }

    case 10:
    {
      test_capsuleMesh();
      break;
    }

    case 11:
    {
      test_torusMesh();
      break;
    }

    case 12:
    {
      test_coneMesh();
      break;
    }

    case 13:
    {
      test_ssrMesh();
      break;
    }

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
