/*******************************************************************************

  Copyright (c) 2017, Honda Research Institute Europe GmbH

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

#include <RcsViewer.h>
#include <COSNode.h>
#include <ArrowNode.h>
#include <SphereNode.h>
#include <VertexArrayNode.h>
#include <GraphNode.h>
#include <KeyCatcher.h>
#include <Rcs_utils.h>
#include <MeshNode.h>
#include <BoxNode.h>
#include <TextNode3D.h>
#include <Rcs_graphicsUtils.h>
#include <DepthRenderer.h>
#include <PPSGui.h>
#include <HighGui.h>

#include <MatNdWidget.h>
#include <Rcs_guiFactory.h>

#include <Rcs_cmdLine.h>
#include <Rcs_macros.h>
#include <Rcs_timer.h>
#include <Rcs_resourcePath.h>
#include <Rcs_math.h>
#include <Rcs_mesh.h>
#include <Rcs_parser.h>
#include <Rcs_typedef.h>

#include <osgGA/TrackballManipulator>
#include <osgDB/Registry>
#include <osgFX/Cartoon>
#include <osgViewer/ViewerEventHandlers>

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
  std::string outFile;
  bool valgrind = argP.hasArgument("-valgrind");
  argP.getArgument("-f", &outFile);

  if (valgrind || (mesh==NULL))
  {
    return;
  }

  REXEC(1)
  {
    RcsMesh_print(mesh);
  }

  if (!outFile.empty())
  {
    RcsMesh_toFile(mesh, outFile.c_str());
  }

  if (argP.hasArgument("-h"))
  {
    return;
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

  RcsMeshData* mesh = RcsMesh_createSSR(extents, segments);

  showMesh(mesh);
  RcsMesh_destroy(mesh);

  return true;
}

/******************************************************************************
 *
 *****************************************************************************/
static bool test_pyramidMesh()
{
  double extents[3];
  Vec3d_setElementsTo(extents, 1.0);

  Rcs::CmdLineParser argP;
  argP.getArgument("-x", &extents[0], "X-dimension (default is %f)",
                   extents[0]);
  argP.getArgument("-y", &extents[1], "Y-dimension (default is %f)",
                   extents[1]);
  argP.getArgument("-z", &extents[2], "Z-dimension (default is %f)",
                   extents[2]);

  RcsMeshData* mesh = RcsMesh_createPyramid(extents[0], extents[1], extents[2]);

  showMesh(mesh);
  RcsMesh_destroy(mesh);

  return true;
}

/******************************************************************************
 *
 *****************************************************************************/
static bool test_frustumMesh()
{
  double fovx = RCS_RAD2DEG(M_PI_2);
  double fovy = RCS_RAD2DEG(M_PI_2);
  double h = 1.0;

  Rcs::CmdLineParser argP;
  argP.getArgument("-fovx", &fovx, "Field of view x (default: %f deg)", fovx);
  argP.getArgument("-fovy", &fovy, "Field of view y (default: %f deg)", fovy);
  argP.getArgument("-z", &h, "Height (default is %f)", h);

  fovx = RCS_DEG2RAD(fovx);
  fovy = RCS_DEG2RAD(fovy);

  RcsMeshData* mesh = RcsMesh_createFrustum(fovx, fovy, h);

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
  double radiusBottom = 0.5;
  double radiusTop = 0.25;
  double height = 1.0;
  unsigned int radialSegments = 16;
  unsigned int heightSegments = 4;
  double angleAround = 360.0;

  Rcs::CmdLineParser argP;
  argP.getArgument("-r1", &radiusBottom, "Radius bottom (default is %f)",
                   radiusBottom);
  argP.getArgument("-r2", &radiusTop, "Radius top (default is %f)", radiusTop);
  argP.getArgument("-height", &height, "Height (default is %f)", height);
  argP.getArgument("-angleAround", &angleAround, "Angle around [deg](default "
                   "is %f)", angleAround);
  argP.getArgument("-radialSegments", &radialSegments, "Radial segments "
                   "(default is %d)", radialSegments);
  argP.getArgument("-heightSegments", &heightSegments, "Height segments "
                   "(default is %d)", heightSegments);

  angleAround *= M_PI/180.0;

  RcsMeshData* mesh = RcsMesh_createCylinderHull(radiusBottom, radiusTop,
                                                 height, radialSegments,
                                                 heightSegments, angleAround);
  RMSGS("Mesh has %d vertices and %d facecs", mesh->nVertices, mesh->nFaces);
  int nDuplicates = RcsMesh_compressVertices(mesh, 1.0e-8);
  RLOG(0, "Reduced mesh by %d duplicates", nDuplicates);
  RMSGS("Reduced mesh has %d vertices and %d facecs",
        mesh->nVertices, mesh->nFaces);
  RcsMesh_toFile(mesh, "reducedMesh.stl");

  RcsMeshData* delauny = RcsMesh_fromVertices(mesh->vertices, mesh->nVertices);

  if (delauny)
  {
    nDuplicates = RcsMesh_compressVertices(delauny, 1.0e-8);
    RLOG(0, "Delaunay mesh has now less %d duplicates", nDuplicates);
    RcsMesh_toFile(delauny, "CylinderHullDelaunay.stl");
    RcsMesh_destroy(delauny);
  }

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
  viewer->setCameraManipulator(new osgGA::TrackballManipulator());

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

  if (argP.hasArgument("-frame", "Add coordinate system to origin"))
  {
    rootnode->addChild(new Rcs::COSNode());
  }

  if (argP.hasArgument("-primitives", "Add 3 osg shape primitives"))
  {
    osg::ref_ptr<osg::ShapeDrawable> s1 = new osg::ShapeDrawable;
    s1->setShape(new osg::Box(osg::Vec3(-3.0f, 0.0f, 0.0f), 2.0f, 2.0f, 1.0f));
    osg::ref_ptr<osg::ShapeDrawable> s2 = new osg::ShapeDrawable;
    s2->setShape(new osg::Cone(osg::Vec3(0.0f, 0.0f, 0.0f), 1.0f, 1.0f));
    s2->setColor(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
    osg::ref_ptr<osg::ShapeDrawable> s3 = new osg::ShapeDrawable;
    s3->setShape(new osg::Sphere(osg::Vec3(3.0f, 0.0f, 0.0f), 1.0f));
    s3->setColor(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f));
    osg::ref_ptr<osg::Geode> root = new osg::Geode;
    root->addDrawable(s1.get());
    root->addDrawable(s2.get());
    root->addDrawable(s3.get());
    rootnode->addChild(root.get());
  }

  // Light grayish blue universe
  if (argP.hasArgument("-clearNode", "Test ClearNode"))
  {
    osg::ref_ptr<osg::ClearNode> clearNode = new osg::ClearNode;
    clearNode->setClearColor(osg::Vec4(0.8, 0.8, 0.95, 1.0));
    rootnode->addChild(clearNode.get());
  }

  // Window size handler toggles fullscreen with F10
  if (argP.hasArgument("-windowSizeHandler",
                       "Test WindowSizeHandler (toggles fullscreen with F10)"))
  {
    osg::ref_ptr<osgViewer::WindowSizeHandler> wsh;
    wsh = new osgViewer::WindowSizeHandler;
    wsh->setKeyEventToggleFullscreen(osgGA::GUIEventAdapter::KEY_F10);
    viewer->addEventHandler(wsh.get());
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
    osg::Camera* cam = viewer->getCamera();
    cam->setCullingMode(cam->getCullingMode() &
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
    char xmlFileName[256] = "LBR.xml";
    char directory[256] = "config/xml/DexBot";
    argP.getArgument("-f", xmlFileName, "Configuration file name");
    argP.getArgument("-dir", directory, "Configuration file directory");
    Rcs_addResourcePath(directory);
    RcsGraph* graph = RcsGraph_create(xmlFileName);
    RCHECK(graph);
    osg::ref_ptr<Rcs::GraphNode> gn = new Rcs::GraphNode(graph);
    rootnode->addChild(gn.get());
  }


  if (argP.hasArgument("-body", "Add BodyNode"))
  {
    RcsGraph* graph = RcsGraph_create("config/xml/DexBot/LBR.xml");
    RCHECK(graph);
    RcsBody* rootBdy = RcsGraph_getRootBody(graph);
    osg::ref_ptr<Rcs::BodyNode> gn = new Rcs::BodyNode(rootBdy, graph);
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
* Test for Rcs viewer
******************************************************************************/
static void testRcsViewer()
{
  int loopCount = 0;
  Rcs::Viewer* viewer = new Rcs::Viewer();
  viewer->setTitle("Window title before realize()");
  viewer->add(new Rcs::COSNode());
  viewer->runInThread();

  while (runLoop)
  {
    Timer_waitDT(1.0);
    char newTitle[256];
    snprintf(newTitle, 256, "Window title %d", loopCount++);
    viewer->setTitle(newTitle);
  }

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
  Rcs::KeyCatcherBase::registerKey("q", "Quit");
  Rcs::KeyCatcherBase::registerKey("p", "Print mesh to console and file");

  std::string meshFile, outfile="outmesh.stl";
  Rcs::CmdLineParser argP;
  argP.getArgument("-f", &meshFile, "Mesh file (default is %s)",
                   meshFile.c_str());
  argP.getArgument("-outFile", &outfile, "Mesh file to be written (default "
                   "is %s)", outfile.c_str());
  bool withScaling = argP.hasArgument("-scaling", "Slider for dynamic scaling");
  bool withAABB = argP.hasArgument("-aabb", "Show axis-aligned bounding box");

  if (argP.hasArgument("-h"))
  {
    return;
  }

  osg::ref_ptr<Rcs::MeshNode> mn;
  RcsMeshData* mesh = NULL;
  RcsMeshData* cpyOfMesh = NULL;

  if (!meshFile.empty())
  {
    RMSG("Loading mesh file \"%s\"", meshFile.c_str());
    mesh = RcsMesh_createFromFile(meshFile.c_str());
    mn = new Rcs::MeshNode(mesh);
  }
  else
  {
    mesh = RcsMesh_createTorus(0.5, 0.25, 32, 32);
    mn = new Rcs::MeshNode(mesh->vertices, mesh->nVertices,
                           mesh->faces, mesh->nFaces);
  }


  Rcs::Viewer* viewer = new Rcs::Viewer();
  viewer->add(mn.get());
  viewer->add(new Rcs::COSNode());
  osg::ref_ptr<Rcs::KeyCatcher> kc = new Rcs::KeyCatcher();
  viewer->add(kc.get());
  if (withAABB)
  {
    double xyzMin[3], xyzMax[3], center[3], extents[3];
    RcsMesh_computeAABB(mesh, xyzMin, xyzMax);
    RLOG(1, "aabb: min: %.4f %.4f %.4f   max: %.4f %.4f %.4f",
         xyzMin[0], xyzMin[1], xyzMin[2], xyzMax[0], xyzMax[1], xyzMax[2]);
    Vec3d_sub(extents, xyzMax, xyzMin);
    Vec3d_constMulAndAdd(center, xyzMin, extents, 0.5);
    osg::ref_ptr<Rcs::BoxNode> aabb = new Rcs::BoxNode(center, extents[0], extents[1], extents[2]);
    viewer->add(aabb.get());
  }

  double scale = 1.0;
  MatNd scaleArr = MatNd_fromPtr(1, 1, &scale);

  if (withScaling)
  {
    Rcs::MatNdWidget::create(&scaleArr, -2.0, 2.0, "Scale");
  }

  while (runLoop)
  {
    if (withScaling && (scale!=1.0))
    {
      RcsMesh_destroy(cpyOfMesh);
      cpyOfMesh = RcsMesh_clone(mesh);
      RCHECK(cpyOfMesh);
      RcsMesh_scale(cpyOfMesh, scale);
      mn->setMesh(cpyOfMesh->vertices, cpyOfMesh->nVertices,
                  cpyOfMesh->faces, cpyOfMesh->nFaces);
    }

    viewer->frame();

    if (kc->getAndResetKey('q'))
    {
      runLoop = false;
    }
    else if (kc->getAndResetKey('p'))
    {
      RMSGS("Printing mesh to file %s", outfile.c_str());
      RcsMesh_print(cpyOfMesh ? cpyOfMesh : mesh);
      bool success = RcsMesh_toFile(cpyOfMesh ? cpyOfMesh : mesh, outfile.c_str());
      RMSGS("%s", success ? "Success" : "Failed");
    }

    Timer_usleep(1000);
  }

  RcsMesh_destroy(mesh);
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
  Rcs::KeyCatcherBase::registerKey("T", "Increase field of view");
  Rcs::KeyCatcherBase::registerKey("t", "Decrease field of view");

  // Parse command line arguments
  char xmlFileName[128] = "gScenario.xml";
  char directory[128] = "config/xml/GenericHumanoid";
  Rcs::CmdLineParser argP;
  argP.getArgument("-f", xmlFileName, "Configuration file name");
  argP.getArgument("-dir", directory, "Configuration file directory");

  if (argP.hasArgument("-h"))
  {
    return;
  }

  pthread_mutex_t mtx;
  pthread_mutex_init(&mtx, NULL);

  Rcs_addResourcePath(directory);

  RcsGraph* graph = RcsGraph_create(xmlFileName);
  Rcs::Viewer* viewer = new Rcs::Viewer();
  osg::ref_ptr<Rcs::KeyCatcher> kc = new Rcs::KeyCatcher();
  osg::ref_ptr<Rcs::COSNode> cn = new Rcs::COSNode();

  viewer->add(new Rcs::GraphNode(graph));
  viewer->add(cn.get());
  viewer->add(kc.get());
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
    else if (kc->getAndResetKey('T'))
    {
      double fov = viewer->getFieldOfView();
      RMSGS("Field of view: %f deg", RCS_RAD2DEG(fov));
      viewer->setFieldOfView(fov + RCS_DEG2RAD(5.0));
      fov = viewer->getFieldOfView();
      RMSGS("Field of view after setting: %f deg", RCS_RAD2DEG(fov));
    }
    else if (kc->getAndResetKey('t'))
    {
      double fov = viewer->getFieldOfView();
      RMSGS("Field of view: %f deg", RCS_RAD2DEG(fov));
      viewer->setFieldOfView(fov - RCS_DEG2RAD(5.0));
      fov = viewer->getFieldOfView();
      RMSGS("Field of view after setting: %f deg", RCS_RAD2DEG(fov));
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
 * Test for 3d text
 ******************************************************************************/
static void testText3D()
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

  osg::ref_ptr<Rcs::TextNode3D> textNd = new Rcs::TextNode3D("My text");
  textNd->setMaterial("RED");


  Rcs::Viewer* viewer = new Rcs::Viewer();
  viewer->add(new Rcs::COSNode());
  viewer->add(textNd.get());
  viewer->runInThread();

  RPAUSE();

  delete viewer;
}

/*******************************************************************************
 * FindChildrenOfType
 ******************************************************************************/
bool testFindChildrenOfType()
{
  Rcs::CmdLineParser argP;
  char xmlFileName[256] = "gScenario.xml";
  char directory[256] = "config/xml/DexBot";
  argP.getArgument("-f", xmlFileName, "Configuration file name (default"
                   " is \"%s\")", xmlFileName);
  argP.getArgument("-dir", directory, "Configuration file directory "
                   "(default is \"%s\")", directory);
  Rcs_addResourcePath(directory);
  Rcs_addResourcePath("config");

  RcsGraph* graph = RcsGraph_create(xmlFileName);

  if (graph == NULL)
  {
    RMSG("Failed to create graph from file \"%s\" - exiting",
         xmlFileName);
    return false;
  }

  osg::ref_ptr<Rcs::GraphNode> gn = new Rcs::GraphNode(graph);
  std::vector<Rcs::BodyNode*> bodyNodes;
  bodyNodes = Rcs::findChildrenOfType<Rcs::BodyNode>(gn.get());

  for (size_t i=0; i<bodyNodes.size(); ++i)
  {
    RLOG_CPP(0, "BodyNode[" << i << "] = " << bodyNodes[i]->getName());
  }

  return true;
}

/*******************************************************************************
 * Depth rendering test
 ******************************************************************************/
// The below comment is for astyle, since otherwise it screws up the formatting
// *INDENT-OFF*
#define MULTI_LINE_STRING(a) #a
const char* depthExampleGraph =
  MULTI_LINE_STRING(
<Graph >
<Body name="Box" rigid_body_joints="3 0 0 0 0 0" >
   <Shape type="BOX" extents="1 1 1" transform="0.5 0 0 0 0 0"
graphics="true" />
</Body>
</Graph>
);
// *INDENT-ON*

void testDepthRenderer()
{
  std::string xmlFileName;
  const char* cfgFilePtr = depthExampleGraph;
  char directory[128] = "config/xml/Examples";
  unsigned int width = 640, height = 480;

  Rcs::CmdLineParser argP;
  argP.getArgument("-f", &xmlFileName, "Configuration file name "
                   "(default is %s)", xmlFileName.c_str());
  argP.getArgument("-dir", directory, "Configuration file directory "
                   "(default is %s)", directory);
  argP.getArgument("-width", &width, "Image width (default is %d)", width);
  argP.getArgument("-height", &height, "Image height (default is %d)", height);
  Rcs_addResourcePath(directory);

  if (!xmlFileName.empty())
  {
    cfgFilePtr = xmlFileName.c_str();
  }

  if (argP.hasArgument("-h"))
  {
    return;
  }

  RcsGraph* graph = RcsGraph_create(cfgFilePtr);
  RCHECK(graph);
  osg::ref_ptr<Rcs::GraphNode> gn = new Rcs::GraphNode(graph);
  Rcs::Viewer* viewer = new Rcs::Viewer();
  viewer->setCameraTransform(HTr_identity());
  viewer->add(gn.get());

  osg::ref_ptr<Rcs::DepthRenderer> zRenderer;
  zRenderer = new Rcs::DepthRenderer(width, height);
  zRenderer->setCameraTransform(HTr_identity());
  zRenderer->addNode(gn.get());

  double* data = new double[width*height];
  std::vector<Rcs::PPSGui::Entry> pps;
  pps.push_back(Rcs::PPSGui::Entry("Depth image", width, height, data, 0.1));
  Rcs::PPSGui::create(pps);
  const std::vector<std::vector<float>>& zImage = zRenderer->getDepthImageRef();

  Rcs::MatNdWidget::create(graph->q, -10.0, 10.0, "q");

  // These come from a Kinect v2 calbration
  const int windowId = 0;
  Rcs::Atomic<double> fx = 6.5746697810243404e+002;
  Rcs::Atomic<double> cx = 3.1950000000000000e+002;
  Rcs::Atomic<double> fy = 6.5746697810243404e+002;
  Rcs::Atomic<double> cy = 2.3950000000000000e+002;
  Rcs::Atomic<double> near = 0.3;
  Rcs::Atomic<double> far = 10.0;
  zRenderer->setProjectionFromFocalParams(fx, fy, cx, cy, near, far);
  Rcs::HighGui::showSlider("fx", windowId, 0.0, 1000.0, 10.0, &fx);
  Rcs::HighGui::showSlider("fy", windowId, 0.0, 1000.0, 10.0, &fy);
  Rcs::HighGui::showSlider("cx", windowId, 0.0, 1000.0, 10.0, &cx);
  Rcs::HighGui::showSlider("cy", windowId, 0.0, 1000.0, 10.0, &cy);
  Rcs::HighGui::showSlider("near", windowId, 0.01, 20.0, 0.01, &near);
  Rcs::HighGui::showSlider("far", windowId, 0.01, 100.0, 0.01, &far);



  while (runLoop)
  {
    RcsGraph_setState(graph, NULL, NULL);
    viewer->frame();
    HTr camTrf;
    viewer->getCameraTransform(&camTrf);
    zRenderer->setCameraTransform(&camTrf);
    zRenderer->setProjectionFromFocalParams(fx, fy, cx, cy, near, far);
    double t_render = Timer_getSystemTime();
    zRenderer->frame();
    t_render = Timer_getSystemTime() - t_render;
    RLOG(0, "Rendering took %.1f msec", 1000.0*t_render);

    // Update the pixel widget
    for (size_t i=0; i<height; ++i)
    {
      for (size_t j=0; j<width; ++j)
      {
        data[i*width+j] = zImage[i][j];
      }
    }

    Timer_waitDT(0.1);
  }

  delete viewer;
  RcsGuiFactory_shutdown();
  RcsGraph_destroy(graph);
}

/*******************************************************************************
 * Changing colors in GraphNode
 ******************************************************************************/
void testDynamicColoring()
{
  RcsGraph* graph = RcsGraph_create("config/xml/DexBot/LBR.xml");
  RCHECK(graph);

  Rcs::Viewer viewer;
  osg::ref_ptr<Rcs::GraphNode> gn = new Rcs::GraphNode(graph);
  viewer.add(gn.get());

  std::vector<Rcs::BodyNode*> bdyNodes = gn->getBodyNodes();

  while (runLoop)
  {

    for (size_t i=0; i<bdyNodes.size(); ++i)
    {
      setNodeMaterial("RANDOM", bdyNodes[i]);
    }

    viewer.frame();
    Timer_waitDT(0.5);
  }

  RcsGraph_destroy(graph);
}

/*******************************************************************************
 * Changing shape size in GraphNode
 ******************************************************************************/
void test_dynamicShapeResizing()
{
  double x = 0.2;
  Rcs::CmdLineParser argP;
  std::string xmlFileName = "gShapes.xml";
  std::string directory = "config/xml/Examples";
  argP.getArgument("-f", &xmlFileName, "Configuration file name "
                   "(default is %s)", xmlFileName.c_str());
  argP.getArgument("-dir", &directory, "Configuration file directory "
                   "(default is %s)", directory);
  bool native = argP.hasArgument("-native", "osg::Capsule test only");
  bool pause = argP.hasArgument("-pause", "Pause before each frame");
  bool valgrind = argP.hasArgument("-valgrind", "Stop after 10 frames");
  Rcs_addResourcePath(directory.c_str());

  RcsGraph* graph = RcsGraph_create(xmlFileName.c_str());
  RCHECK(graph);

  Rcs::Viewer viewer;
  viewer.setCameraTransform(18.0, 26.0, 10.0, 0.2, -0.3, -2.4);
  osg::ref_ptr<osg::Capsule> capsule;
  osg::ref_ptr <osg::ShapeDrawable> sd;
  osg::ref_ptr<osg::Geode> geode;

  if (native)
  {
    osg::ref_ptr <osg::PositionAttitudeTransform> pat;
    pat = new osg::PositionAttitudeTransform;
    geode = new osg::Geode();
    capsule = new osg::Capsule(osg::Vec3(0.0, 0.0, 0.0), x, x);

    capsule->setDataVariance(osg::Object::DYNAMIC);

    sd = new osg::ShapeDrawable(capsule);
    sd->setUseDisplayList(true);
    sd->setUseVertexBufferObjects(false);
    sd->setDataVariance(osg::Object::DYNAMIC);
    geode->addDrawable(sd.get());
    pat->addChild(geode.get());

    viewer.add(pat.get());
  }
  else
  {
    viewer.add(new Rcs::GraphNode(graph));
  }

  double time = 0.0, dt = 0.01;

  std::vector<std::vector<double>> xyz;

  RCSGRAPH_TRAVERSE_BODIES(graph)
  {
    RCSBODY_TRAVERSE_SHAPES(BODY)
    {
      xyz.push_back(std::vector<double>(SHAPE->extents, SHAPE->extents+3));
    }
  }



  while (runLoop)
  {

    int idx = 0;
    RCSGRAPH_TRAVERSE_BODIES(graph)
    {
      RCSBODY_TRAVERSE_SHAPES(BODY)
      {
        Vec3d_constMul(SHAPE->extents, xyz[idx++].data(), fabs(cos(time)));
      }
    }

    x = fabs(cos(time));

    if (native)
    {
      sd->dirtyBound();
      sd->dirtyDisplayList();
      geode->dirtyBound();
      capsule->setRadius(x);
      capsule->setHeight(x);
      //sd->dirtyGLObjects();
      RLOG(0, "r=%f displayList=%s   VBO=%s", x,
           sd->getUseDisplayList()?"TRUE":"FALSE",
           sd->getUseVertexBufferObjects() ? "TRUE" : "FALSE");
    }

    if (pause)
    {
      RPAUSE();
    }
    viewer.frame();
    time += dt;
    Timer_waitDT(dt);
    RPAUSE_DL(5);

    if (valgrind && (time>10.0*dt))
    {
      runLoop = false;
    }
  }

  RcsGraph_destroy(graph);
}

/*******************************************************************************
 *
 ******************************************************************************/
void test_meshNormals()
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

  RcsMeshData* mesh = RcsMesh_createSSR(extents, segments);
  //RcsMeshData* mesh = RcsMesh_createSphere(0.25, segments);
  //RcsMeshData* mesh = RcsMesh_createBox(extents);



  Rcs::MeshNode* mn = new Rcs::MeshNode(mesh->vertices, mesh->nVertices,
                                        mesh->faces, mesh->nFaces);

  double* normals = RcsMesh_createNormalArray(mesh);

  MatNd* nVecs = MatNd_create(2*mesh->nVertices, 3);

  for (unsigned int i=0; i<mesh->nVertices; ++i)
  {
    double* p0 = MatNd_getRowPtr(nVecs, 2*i);
    double* p1 = MatNd_getRowPtr(nVecs, 2*i+1);

    const double* vi = &mesh->vertices[3*i];
    const double* ni = &normals[3*i];

    // p0 is vertex position
    Vec3d_copy(p0, vi);
    Vec3d_constMulAndAdd(p1, vi, ni, 0.2);
  }


  Rcs::Viewer* viewer = new Rcs::Viewer();
  viewer->add(mn);
  viewer->add(new Rcs::COSNode());
  viewer->add(new Rcs::VertexArrayNode(nVecs));
  viewer->runInThread();

  RPAUSE();

  delete viewer;

  MatNd_destroy(nVecs);
  RcsMesh_destroy(mesh);
}

/*******************************************************************************
 *
 ******************************************************************************/
void test_sphereNode()
{
  Rcs::SphereNode* sn = new Rcs::SphereNode();
  Rcs::Viewer* viewer = new Rcs::Viewer();
  viewer->add(sn);
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

  Rcs_addResourcePath("config");

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
      printf("\t\t14   Test 3d text\n");
      printf("\t\t15   Test findChildrenOfType() function\n");
      printf("\t\t16   Test depth rendering\n");
      printf("\t\t17   Test RcsViewer setTitle() method\n");
      printf("\t\t18   Test Setting colors in GraphNode\n");
      printf("\t\t19   Test pyramid mesh\n");
      printf("\t\t20   Test frustum mesh\n");
      printf("\t\t21   Test dynamic shape resizing\n");
      printf("\t\t22   Test mesh normal computation\n");
      printf("\t\t23   Test SphereNode\n");
      printf("\n");
      printf("\t\tYou can write the meshes to a file with -f\n");
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

    case 14:
    {
      testText3D();
      break;
    }

    case 15:
    {
      testFindChildrenOfType();
      break;
    }

    case 16:
    {
      testDepthRenderer();
      break;
    }

    case 17:
      testRcsViewer();
      break;

    case 18:
      testDynamicColoring();
      break;

    case 19:
    {
      test_pyramidMesh();
      break;
    }

    case 20:
    {
      test_frustumMesh();
      break;
    }

    case 21:
    {
      test_dynamicShapeResizing();
      break;
    }

    case 22:
    {
      test_meshNormals();
      break;
    }

    case 23:
    {
      test_sphereNode();
      break;
    }

    default:
      RFATAL("No mode %d", mode);
  }


  if (argP.hasArgument("-h"))
  {
    Rcs_printResourcePath();
    Rcs::KeyCatcherBase::printRegisteredKeys();
    argP.print();
  }

  xmlCleanupParser();
  fprintf(stderr, "Thanks for using the ViewerTest\n");

  return 0;
}
