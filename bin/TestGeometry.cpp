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

#include <Rcs_macros.h>
#include <Rcs_cmdLine.h>
#include <Rcs_math.h>
#include <Rcs_geometry.h>
#include <Rcs_resourcePath.h>
#include <Rcs_timer.h>
#include <Rcs_typedef.h>
#include <Rcs_parser.h>
#include <Rcs_body.h>
#include <Rcs_shape.h>
#include <Rcs_utils.h>
#include <GraphNode.h>
#include <SphereNode.h>
#include <ArrowNode.h>
#include <CapsuleNode.h>
#include <HUD.h>
#include <VertexArrayNode.h>
#include <KeyCatcher.h>
#include <RcsViewer.h>
#include <Rcs_guiFactory.h>
#include <Slider1Dof.h>
#include <MatNdWidget.h>
#include <TargetSetter.h>
#include <SegFaultHandler.h>

#include <pthread.h>

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

/*******************************************************************************
 * Random plausibility tests
 ******************************************************************************/
static bool testDistanceRandomly()
{
  int iterations = 10, st1 = -1, st2 = -1;
  Rcs::CmdLineParser argP;
  argP.getArgument("-iter", &iterations, "Number of iterations");
  argP.getArgument("-t1", &st1, "Shape type for shape 1 (Default: all shapes)");
  argP.getArgument("-t2", &st2, "Shape type for shape 2 (Default: all shapes)");
  bool skipPenetrations = argP.hasArgument("-skipPenetrations",
                                           "Ignore cases with penetrations");
  bool showWindow = argP.hasArgument("-graphics",
                                     "Show graphics window for failure cases");

  if (argP.hasArgument("-h"))
  {
    RcsShape_fprintDistanceFunctions(stdout);
    return false;
  }

  std::vector<RCSSHAPE_TYPE> sTypes;
  sTypes.push_back(RCSSHAPE_SSL);
  sTypes.push_back(RCSSHAPE_SSR);
  sTypes.push_back(RCSSHAPE_MESH);
  sTypes.push_back(RCSSHAPE_BOX);
  sTypes.push_back(RCSSHAPE_CYLINDER);
  sTypes.push_back(RCSSHAPE_SPHERE);
  sTypes.push_back(RCSSHAPE_CONE);
  sTypes.push_back(RCSSHAPE_TORUS);
  sTypes.push_back(RCSSHAPE_POINT);

  bool success = true;
  const double eps = 1.0e-5;

  for (int i = 0; i < iterations; ++i)
  {
    RLOG_CPP(2, "Iteration " << i << " of " << iterations);

    int rndIdx1 = st1;
    int rndIdx2 = st2;

    if (rndIdx1 == -1)
    {
      rndIdx1 = sTypes[Math_getRandomInteger(0, sTypes.size() - 1)];
    }

    if (rndIdx2 == -1)
    {
      rndIdx2 = sTypes[Math_getRandomInteger(0, sTypes.size() - 1)];
    }

    RcsShape* s1 = RcsShape_createRandomShape(rndIdx1);
    RcsShape* s2 = RcsShape_createRandomShape(rndIdx2);
    RCHECK(s1);
    RCHECK(s2);
    RLOG_CPP(2, "Testing " << RcsShape_name(s1->type) << " against "
             << RcsShape_name(s2->type));

    HTr A_BI1, A_BI2;
    Vec3d_setRandom(A_BI1.org, -0.1, 0.1);
    Mat3d_setRandomRotation(A_BI1.rot);
    Vec3d_setRandom(A_BI2.org, -0.1, 0.1);
    Mat3d_setRandomRotation(A_BI2.rot);

    double cp1[3], cp2[3], n12[3];
    HTr A_C1I, A_C2I;
    HTr_transform(&A_C1I, &A_BI1, &s1->A_CB);
    HTr_transform(&A_C2I, &A_BI2, &s2->A_CB);

    double d = RcsShape_distance(s1, s2, &A_BI1, &A_BI2, cp1, cp2, n12);

    if (skipPenetrations && (d<0.0))
    {
      continue;
    }

    double testPt1[3], testPt2[3], tmp[3];
    double dtest1 = RcsShape_distanceToPoint(s1, &A_BI1, cp1, testPt1, tmp);
    double dtest2 = RcsShape_distanceToPoint(s2, &A_BI2, cp2, testPt2, tmp);

    bool success_i = true;

    // Test 1: contact points must be on shape's surface
    if (fabs(dtest1 > eps) || fabs(dtest2 > eps))
    {
      success_i = false;
      success = false;
      RLOG_CPP(1, "Testing " << RcsShape_name(s1->type) << " against "
               << RcsShape_name(s2->type) << " with d=" << d);
      RLOG(1, "Distance test 1 %d failed: %f %f", i, dtest1, dtest2);
    }

    // Test 2: Distance query from point cp2 to shape 1 (and the other way
    //         around) must deiver the same contact point cp1.
    RcsShape_distanceToPoint(s1, &A_BI1, cp2, testPt1, tmp);
    RcsShape_distanceToPoint(s2, &A_BI2, cp1, testPt2, tmp);

    if ((d>0.0) && ((Vec3d_distance(cp1, testPt1) > eps) ||
                    (Vec3d_distance(cp1, testPt1) > eps)))
    {
      success_i = false;
      success = false;
      RLOG_CPP(1, "Testing " << RcsShape_name(s1->type) << " against "
               << RcsShape_name(s2->type) << " with d=" << d);
      RLOG(1, "Distance test 2 %d failed", i);
    }

    if ((!success_i) && showWindow)
    {
      Rcs::Viewer viewer;
      osg::ref_ptr<Rcs::SphereNode> sp1 = new Rcs::SphereNode(cp1, 0.01);
      osg::ref_ptr<Rcs::SphereNode> sp2 = new Rcs::SphereNode(cp2, 0.01);
      sp1->setMaterial("RED");
      sp2->setMaterial("GREEN");
      viewer.add(sp1);
      viewer.add(sp2);

      RcsBody b1;
      memset(&b1, 0, sizeof(RcsBody));
      b1.shape = RNALLOC(2, RcsShape*);
      b1.shape[0] = s1;
      b1.shape[1] = NULL;
      b1.A_BI = A_BI1;

      osg::ref_ptr<Rcs::BodyNode> coll1 = new Rcs::BodyNode(&b1);
      coll1->setMaterial("RED");
      coll1->displayCollisionNode(true);
      coll1->displayGraphicsNode(false);
      coll1->displayPhysicsNode(false);

      RcsBody b2;
      memset(&b2, 0, sizeof(RcsBody));
      b2.shape = RNALLOC(2, RcsShape*);
      b2.shape[0] = s2;
      b2.shape[1] = NULL;
      b2.A_BI = A_BI2;

      osg::ref_ptr<Rcs::BodyNode> coll2 = new Rcs::BodyNode(&b2);
      coll2->setMaterial("BLUE");
      coll2->displayCollisionNode(true);
      coll2->displayGraphicsNode(false);
      coll2->displayPhysicsNode(false);
      viewer.add(coll1);
      viewer.add(coll2);

      viewer.runInThread();
      RPAUSE();
    }

    RLOG(2, "Distance test %s: %f %f", success_i ? "SUCCEEDED" : "FAILED",
         dtest1, dtest2);

    RcsShape_destroy(s1);
    RcsShape_destroy(s2);
  }

  RMSG("Distance test reported %s", success ? "SUCCESS" : "FAILURE");

  return success;
}


/*******************************************************************************
 * Threaded distance function test
 ******************************************************************************/
static void* threadFunc(void* arg)
{
  bool* result = new bool;
  *result = testDistanceRandomly();
  return result;
}

static bool testDistanceThreaded()
{
  const size_t maxThreads = 32;
  pthread_t threads[maxThreads];
  bool success = true;
  int iterations = 10, nThreads = 1;
  Rcs::CmdLineParser argP;
  argP.getArgument("-iter", &iterations, "Number of iterations");
  argP.getArgument("-nThreads", &nThreads, "Number of threads");

  if (argP.hasArgument("-h"))
  {
    RcsShape_fprintDistanceFunctions(stdout);
    return false;
  }

  nThreads = Math_iClip(nThreads, 0, maxThreads);

  RLOG_CPP(0, "Launching " << nThreads << " threads with " << iterations
           << " iterations");

  for (int i=0; i< nThreads; ++i)
  {
    int res = pthread_create(&threads[i], NULL, threadFunc, &iterations);
    RCHECK_MSG(res == 0, "Failure launching thread number %d", i);
  }

  for (int i = 0; i < nThreads; ++i)
  {
    bool* result_i;
    pthread_join(threads[i], (void**)&result_i);
    success = *result_i && success;
    delete result_i;
  }

  RLOG(0, "Joined %d threads, test was a great %s", nThreads,
       success ? "SUCCESS" : "FAILURE");

  return success;
}

/*******************************************************************************
 * Distance function test
 ******************************************************************************/
static void testDistance(int argc, char** argv)
{
  Rcs::CmdLineParser argP(argc, argv);

  int shapeType1 = RCSSHAPE_SSL;
  int shapeType2 = RCSSHAPE_SSL;
  char textLine[2056] = "";

  argP.getArgument("-t1", &shapeType1, "Shape type for shape 1");
  argP.getArgument("-t2", &shapeType2, "Shape type for shape 2");
  bool valgrind = argP.hasArgument("-valgrind",
                                   "Start without Guis and graphics");
  bool simpleGraphics = argP.hasArgument("-simpleGraphics", "OpenGL without fan"
                                         "cy stuff (shadows, anti-aliasing)");


  // Initialize GUI and OSG mutex
  pthread_mutex_t graphLock;
  pthread_mutex_init(&graphLock, NULL);

  // Option without mutex for viewer
  pthread_mutex_t* mtx = &graphLock;
  if (argP.hasArgument("-nomutex", "Graphics without mutex"))
  {
    mtx = NULL;
  }

  if (argP.hasArgument("-h"))
  {
    RcsShape_fprintDistanceFunctions(stdout);
    pthread_mutex_destroy(&graphLock);
    return;
  }

  RcsBody* b1 = RcsBody_create();
  Vec3d_setRandom(b1->A_BI.org, -0.2, -0.1);
  RcsBody_addShape(b1, RcsShape_createRandomShape(shapeType1));

  RcsBody* b2 = RcsBody_create();
  Vec3d_setRandom(b1->A_BI.org, 0.1, 0.2);
  RcsBody_addShape(b2, RcsShape_createRandomShape(shapeType2));

  double I_closestPts[6];
  VecNd_setZero(I_closestPts, 6);
  double* cp0 = &I_closestPts[0];
  double* cp1 = &I_closestPts[3];
  double n01[3];
  Vec3d_set(n01, 0.0, 0.0, 0.25);

  // Graphics
  Rcs::Viewer* viewer = NULL;
  osg::ref_ptr<Rcs::HUD> hud;
  osg::ref_ptr<Rcs::KeyCatcher> kc;

  if (!valgrind)
  {
    viewer = new Rcs::Viewer(!simpleGraphics, !simpleGraphics);

    // HUD
    hud = new Rcs::HUD();
    viewer->add(hud.get());

    // BodyNodes
    Rcs::BodyNode* bNd1 = new Rcs::BodyNode(b1, 1.0, false);
    Rcs::BodyNode* bNd2 = new Rcs::BodyNode(b2, 1.0, false);
    bNd1->setGhostMode(true, "RED");
    bNd2->setGhostMode(true, "GREEN");
    viewer->add(bNd1);
    viewer->add(bNd2);

    // TargetSetters
    bool sphTracker = b1->shape[0]->type==RCSSHAPE_POINT ? false : true;
    Rcs::TargetSetter* ts1 =
      new Rcs::TargetSetter(b1->A_BI.org, b1->A_BI.rot, 0.5, sphTracker);
    viewer->add(ts1);
    viewer->add(ts1->getHandler());
    sphTracker = b2->shape[0]->type==RCSSHAPE_POINT ? false : true;
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

    Rcs::CapsuleNode* sphereCP0 =
      new Rcs::CapsuleNode(cp0, NULL, 0.015, 0.0);
    sphereCP0->makeDynamic(cp0);
    sphereCP0->setMaterial("RED");
    viewer->add(sphereCP0);

    Rcs::CapsuleNode* sphereCP1 =
      new Rcs::CapsuleNode(cp1, NULL, 0.015, 0.0);
    sphereCP1->makeDynamic(cp1);
    sphereCP1->setMaterial("RED");
    viewer->add(sphereCP1);

    // ArrowNode for normal vector
    Rcs::ArrowNode* normalArrow = new Rcs::ArrowNode(cp0, n01, 0.2);
    viewer->add(normalArrow);

    // KeyCatcher
    kc = new Rcs::KeyCatcher();
    viewer->add(kc.get());
    viewer->runInThread(mtx);
  }


  while (runLoop)
  {
    pthread_mutex_lock(&graphLock);
    double dt = Timer_getTime();
    double dist;

    dist = RcsShape_distance(b1->shape[0], b2->shape[0],
                             &b1->A_BI, &b2->A_BI, cp0, cp1, n01);

    dt = Timer_getTime() - dt;
    pthread_mutex_unlock(&graphLock);

    std::stringstream hudText;
    sprintf(textLine, "Distance: D = % 3.1f mm took %3.2f usec\n",
            dist*1000.0, dt*1.0e6);
    hudText << textLine;
    if (hud.valid())
    {
      hud->setText(hudText);
    }
    else
    {
      std::cout << hudText.str();
    }

    if (kc.valid() && kc->getAndResetKey('q'))
    {
      runLoop = false;
    }

    Timer_usleep(1000);
    RPAUSE_DL(4);

    if (valgrind)
    {
      runLoop = false;
    }

  } // while runLoop


  // Clean up
  RcsBody_destroy(b1);
  RcsBody_destroy(b2);

  if (!valgrind)
  {
    delete viewer;
  }

  pthread_mutex_destroy(&graphLock);
}

/*******************************************************************************
 * 2D polygon test
 ******************************************************************************/
static void testPolygon(int argc, char** argv)
{
  double radius = 0.25;
  double height = 1.0;
  std::string polyFileName;

  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-radius", &radius, "Radius (default is %g)", radius);
  argP.getArgument("-height", &height, "Height (default is %g)", height);
  bool simpleGraphics = argP.hasArgument("-simpleGraphics", "OpenGL without fan"
                                         "cy stuff (shadows, anti-aliasing)");
  argP.getArgument("-f", &polyFileName, "Polygon file name");

  // Initialize GUI and OSG mutex
  pthread_mutex_t graphLock;
  pthread_mutex_init(&graphLock, NULL);

  // Option without mutex for viewer
  pthread_mutex_t* mtx = &graphLock;
  if (argP.hasArgument("-nomutex", "Graphics without mutex"))
  {
    mtx = NULL;
  }

  if (argP.hasArgument("-h"))
  {
    pthread_mutex_destroy(&graphLock);
    return;
  }

  MatNd* polyArr = NULL;
  double (*poly)[2];
  unsigned int nVertices = 0;
  int vidx = 0;

  if (polyFileName.empty())
  {
    polyArr = MatNd_create(5, 2);
    poly = (double (*)[2])polyArr->ele;
    poly[0][0] = -radius;
    poly[0][1] = -0.5*height;
    poly[1][0] =  radius;
    poly[1][1] = -0.5*height;
    poly[2][0] =  radius;
    poly[2][1] =  0.5*height;
    poly[3][0] = -radius;
    poly[3][1] =  0.5*height;
    poly[4][0] = -radius;
    poly[4][1] = -0.5*height;
    nVertices = 5;
    RCHECK(Math_checkPolygon2D(poly, nVertices-1));
  }
  else
  {
    polyArr = MatNd_createFromFile(polyFileName.c_str());
    RCHECK(polyArr);
    poly = (double (*)[2])polyArr->ele;
    RCHECK(polyArr->n==2);
    RCHECK(polyArr->m > 2);
    MatNd_printCommentDigits("Polygon", polyArr, 3);

    for (unsigned int i=0; i<polyArr->m; ++i)
    {
      const double* row_i = MatNd_getRowPtr(polyArr, i);
      poly[i][0] = row_i[0];
      poly[i][1] = row_i[1];
    }

    nVertices = polyArr->m;
    //RCHECK(Math_checkPolygon2D(poly, nVertices));
  }

  MatNd_reshape(polyArr, polyArr->m-1, polyArr->n);


  Rcs::VertexArrayNode* vn =
    new Rcs::VertexArrayNode(polyArr, osg::PrimitiveSet::LINE_LOOP);

  Rcs::Viewer* viewer = new Rcs::Viewer(!simpleGraphics, !simpleGraphics);
  viewer->setBackgroundColor("DARKRED");
  viewer->add(vn);

  // Visualize original polygon as points
  MatNd polyArr2 = MatNd_fromPtr(nVertices, 2, &poly[0][0]);
  Rcs::VertexArrayNode* vn2 =
    new Rcs::VertexArrayNode(&polyArr2, osg::PrimitiveSet::POINTS);
  vn2->setPointSize(15);
  viewer->add(vn2);

  // Visualize re-sampled polygon
  // const unsigned int nvOut = 8*nVertices;
  // double polyR[nvOut][2];
  // Math_resamplePolygon2D(polyR, nvOut, poly, nVertices);
  // MatNd polyArr3 = MatNd_fromPtr(nvOut, 2, &polyR[0][0]);
  // Rcs::VertexArrayNode* vn3 =
  //   new Rcs::VertexArrayNode(&polyArr3, osg::PrimitiveSet::POINTS);
  // vn3->setPointSize(25);
  // vn3->setMaterial("YELLOW");
  // viewer->add(vn3);

  double pt[3], ang[3], Id[3][3];
  Mat3d_setIdentity(Id);
  Vec3d_setZero(ang);
  Vec3d_setZero(pt);
  Rcs::TargetSetter* ts = new Rcs::TargetSetter(pt, ang, 0.5, false);
  viewer->add(ts);

  Rcs::CapsuleNode* pn = new Rcs::CapsuleNode(pt, Id, 0.025, 0.0);
  pn->setMaterial("GREEN");
  pn->makeDynamic(pt);
  pn->setWireframe(false);
  viewer->add(pn);

  double cpPoly[3], nPoly[3];
  Vec3d_setZero(cpPoly);
  Vec3d_setZero(nPoly);
  Rcs::CapsuleNode* cn = new Rcs::CapsuleNode(cpPoly, Id, 0.025, 0.0);
  cn->makeDynamic(cpPoly);
  cn->setMaterial("RED");
  cn->setWireframe(false);
  viewer->add(cn);

  double s = 0.0;
  Slider1Dof::create(&s, &s, "s", -0.1, 0.0, 1.1, 0.01, mtx);
  double ps[3];
  Vec3d_set(ps, poly[0][0], poly[0][1], 0.0);
  Rcs::CapsuleNode* sn = new Rcs::CapsuleNode(ps, Id, 0.03, 0.0);
  sn->setMaterial("BLUE");
  sn->makeDynamic(ps);
  sn->setWireframe(false);
  viewer->add(sn);

  Rcs::ArrowNode* an = new Rcs::ArrowNode(cpPoly, nPoly, 0.2);
  viewer->add(an);

  char hudText[512] = "";
  Rcs::HUD* hud = new Rcs::HUD();
  viewer->add(hud);

  osg::ref_ptr<Rcs::KeyCatcher> kc = new Rcs::KeyCatcher();
  viewer->add(kc.get());

  viewer->runInThread(mtx);

  while (runLoop)
  {
    pthread_mutex_lock(&graphLock);
    double d = Math_distPointConvexPolygon2D(pt, poly, nVertices, cpPoly, nPoly);
    Math_interpolatePolygon2D(ps, poly, nVertices, s);
    // ps[0] = poly[vidx][0];
    // ps[1] = poly[vidx][1];
    int res = Math_pointInsideOrOnPolygon2D(pt, poly, nVertices);
    sprintf(hudText, "d = %f   s = %f   ps=%f %f\n%s",
            d, s, ps[0], ps[1],
            res%2==0?"Outside" : "Inside");
    hud->setText(hudText);
    pthread_mutex_unlock(&graphLock);
    if (kc->getAndResetKey('q'))
    {
      runLoop = false;
    }
    else if (kc->getAndResetKey('+'))
    {
      vidx = Math_iClip(vidx+1, 0, nVertices-1);
    }
    else if (kc->getAndResetKey('-'))
    {
      vidx = Math_iClip(vidx-1, 0, nVertices-1);
    }

    Timer_waitDT(0.01);
  }

  delete viewer;
  RcsGuiFactory_shutdown();
  pthread_mutex_destroy(&graphLock);
}

/*******************************************************************************
 * 2D ray - line segment intersection test
 ******************************************************************************/
static inline const char* IntersectReturnStr(unsigned int iType)
{
  static char res[][64] = {"no intersection",
                           "intersection on the line segment",
                           "intersection with first vertex point",
                           "intersection with second vertex point",
                           "co-linear line segment and ray",
                           "Wrong argument"
                          };

  iType = Math_iClip(iType, 0, 5);
  return res[iType];
}

static void testRayLinesegIntersection2D(int argc, char** argv)
{
  Rcs::CmdLineParser argP(argc, argv);
  bool simpleGraphics = argP.hasArgument("-simpleGraphics", "OpenGL without fan"
                                         "cy stuff (shadows, anti-aliasing)");

  // Initialize GUI and OSG mutex
  pthread_mutex_t graphLock;
  pthread_mutex_init(&graphLock, NULL);

  // Option without mutex for viewer
  pthread_mutex_t* mtx = &graphLock;
  if (argP.hasArgument("-nomutex", "Graphics without mutex"))
  {
    mtx = NULL;
  }

  if (argP.hasArgument("-h"))
  {
    pthread_mutex_destroy(&graphLock);
    return;
  }

  double segVtx[6], rayVtx[6], intersectPt[3];
  double* rayPt0 = &rayVtx[0];
  double* rayPt1 = &rayVtx[3];
  double* segPt0 = &segVtx[0];
  double* segPt1 = &segVtx[3];

  // Normal
  Vec3d_set(segPt0, -0.5, -0.1, 0.0);
  Vec3d_set(segPt1,  0.5, 0.1, 0.0);

  // Co-linear
  // Vec3d_set(segPt0,  0.0, 0.1, 0.0);
  // Vec3d_set(segPt1,  0.0, 0.8, 0.0);

  // Vertex 0
  // Vec3d_set(segPt0,  0.0, 0.1, 0.0);
  // Vec3d_set(segPt1,  0.1, 0.8, 0.0);

  // Vertex 1
  // Vec3d_set(segPt0,  0.1, 0.1, 0.0);
  // Vec3d_set(segPt1,  0.0, 0.8, 0.0);


  Vec3d_setZero(rayPt0);
  Vec3d_set(rayPt1,  0.0, 1.0, 0.0);
  Vec3d_setZero(intersectPt);

  Rcs::Viewer* viewer = new Rcs::Viewer(!simpleGraphics, !simpleGraphics);
  viewer->setBackgroundColor("DARKRED");
  Rcs::TargetSetter* ts0 = new Rcs::TargetSetter(segPt0);
  Rcs::TargetSetter* ts1 = new Rcs::TargetSetter(segPt1);
  viewer->add(ts0);
  viewer->add(ts0->getHandler());
  viewer->add(ts1);
  viewer->add(ts1->getHandler());

  Rcs::CapsuleNode* intrsctNd =
    new Rcs::CapsuleNode(intersectPt, NULL, 0.015, 0.0);
  intrsctNd->makeDynamic(intersectPt);
  intrsctNd->setWireframe(false);
  intrsctNd->setMaterial("RED");
  viewer->add(intrsctNd);

  Rcs::VertexArrayNode* segNd = new Rcs::VertexArrayNode(segVtx, 2);
  viewer->add(segNd);

  Rcs::VertexArrayNode* rayNd = new Rcs::VertexArrayNode(rayVtx, 2);
  rayNd->setMaterial("BLUE");
  viewer->add(rayNd);
  Rcs::HUD* hud = new Rcs::HUD();
  viewer->add(hud);
  osg::ref_ptr<Rcs::KeyCatcher> kc = new Rcs::KeyCatcher();
  viewer->add(kc.get());
  viewer->runInThread(mtx);

  char hudText[512] = "";

  while (runLoop)
  {

    pthread_mutex_lock(&graphLock);
    double rayDir[3];
    Vec3d_sub(rayDir, rayPt1, rayPt0);
    Vec3d_normalizeSelf(rayDir);
    int res = Math_intersectRayLineseg2D(rayPt0, rayDir, segPt0, segPt1, intersectPt);
    if (res==0)
    {
      intrsctNd->hide();
    }
    else
    {
      intrsctNd->show();
    }


    pthread_mutex_unlock(&graphLock);

    sprintf(hudText, "res = %s (%d)\n", IntersectReturnStr(res), res);

    if (res>0)
    {
      char tmp[256];
      sprintf(tmp, "intersect = %.3f %.3f\n",
              intersectPt[0], intersectPt[1]);
      strcat(hudText, tmp);
    }

    hud->setText(hudText);

    if (kc->getAndResetKey('q'))
    {
      runLoop = false;
    }
    Timer_waitDT(0.01);

  }

  delete viewer;
  pthread_mutex_destroy(&graphLock);
}

/*******************************************************************************
 *
 ******************************************************************************/
int main(int argc, char** argv)
{
  RMSG("Starting Rcs...");
  int mode = 0;

  // Ctrl-C callback handler
  signal(SIGINT, quit);

  // This initialize the xml library and check potential mismatches between
  // the version it was compiled for and the actual shared library used.
  LIBXML_TEST_VERSION;

  // Parse command line arguments
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
      printf("\t0   Print this message\n");
      printf("\t\t1   Distance function test\n");
      printf("\t\t2   2D polygon test\n");
      printf("\t\t3   2D ray - line segment intersection test\n");
      printf("\t\t4   Random distance test\n");
      printf("\t\t5   Threaded random distance test\n");
      break;
    }

    case 1:
    {
      testDistance(argc, argv);
      break;
    }

    case 2:
    {
      testPolygon(argc, argv);
      break;
    }

    case 3:
    {
      testRayLinesegIntersection2D(argc, argv);
      break;
    }

    case 4:
    {
      testDistanceRandomly();
      break;
    }

    case 5:
    {
      testDistanceThreaded();
      break;
    }

    // ==============================================================
    // That's it.
    // ==============================================================
    default:
    {
      RMSG("there is no mode %d", mode);
    }

  } // switch(mode)


  if ((mode!=0) && argP.hasArgument("-h", "Show help message"))
  {
    Rcs::KeyCatcherBase::printRegisteredKeys();
    argP.print();
    Rcs_printResourcePath();
  }

  xmlCleanupParser();

  fprintf(stderr, "Thanks for using the Rcs libraries\n");

  return 0;
}
