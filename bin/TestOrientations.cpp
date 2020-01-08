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
#include <Rcs_timer.h>
#include <Rcs_cmdLine.h>
#include <Rcs_resourcePath.h>
#include <EulerAngles.h>
#include <SegFaultHandler.h>

#include <RcsViewer.h>
#include <COSNode.h>
#include <KeyCatcher.h>
#include <HUD.h>
#include <TargetSetter.h>
#include <ArrowNode.h>

#include <Slider1Dof.h>
#include <MatNdWidget.h>
#include <Rcs_guiFactory.h>

#include <signal.h>

RCS_INSTALL_ERRORHANDLERS

#define D2R (M_PI/180.0)
#define R2D (180.0/M_PI)
#define V3ANG(x) (x)[0]*R2D, (x)[1]*R2D, (x)[2]*R2D
#define V3(x) (x)[0], (x)[1], (x)[2]

/*******************************************************************************

  \brief Ctrl-C destructor. Tries to quit gracefully with the first Ctrl-C
         press, then just exits.

*******************************************************************************/

bool runLoop = true;

static void quit(int /*sig*/)
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

  \brief Euler angles using Ken Shoemake implementation. For the
         agnular order, see EulerAngles.h. More details on Euler angles:
         http://en.wikipedia.org/wiki/Euler_angles

*******************************************************************************/

void Mat3d_fromEulerAngles_Shoemake(double A_KI[3][3], const double ea[3],
                                    int eulerOrder)
{
  HMatrix R;
  EulerAngles inAngs = {0, 0, 0, static_cast<double>(eulerOrder)};
  inAngs.x = ea[0];
  inAngs.y = ea[1];
  inAngs.z = ea[2];
  Eul_ToHMatrix(inAngs, R);

  for (int i=0; i<3; i++)
  {
    for (int j=0; j<3; j++)
    {
      A_KI[j][i] = R[i][j];
    }
  }

}



/*******************************************************************************

  \brief Displays the Euler angles.

*******************************************************************************/

static void testEulerAngles(int argc, char** argv)
{
  printf("Euler angles test. With command line option -transposed, the Euler\n"
         "angles around the world axes are computed. Otherwise, it's the\n"
         "serial Euler angles. To quit, press \"q\" in the viewer window.\n"
         "Switch do debul level 1 for verbose output on the console.\n");

  Rcs::CmdLineParser argP(argc, argv);
  bool transposed = argP.hasArgument("-transposed");

  pthread_mutex_t OSGLock;
  pthread_mutex_init(&OSGLock, NULL);

  double pos[3], eaDeg[3], eaRad[3], eaCurr[3], A[3][3];
  Vec3d_setZero(pos);
  Vec3d_setZero(eaDeg);
  Vec3d_setZero(eaRad);
  Vec3d_setZero(eaCurr);
  Mat3d_setIdentity(A);

  Rcs::Viewer v;
  osg::ref_ptr<Rcs::COSNode> cn0   = new Rcs::COSNode(pos);
  osg::ref_ptr<Rcs::COSNode> cn    = new Rcs::COSNode(pos, (double*) A);
  osg::ref_ptr<Rcs::KeyCatcher> kc = new Rcs::KeyCatcher();

  v.add(cn.get());
  v.add(cn0.get());
  v.add(kc.get());
  v.runInThread(&OSGLock);

  Slider1Dof::create(&eaDeg[0], &eaCurr[0], "Alpha", -190.0, 0.0, 190.0, 0.1);
  Slider1Dof::create(&eaDeg[1], &eaCurr[1], "Beta",  -190.0, 0.0, 190.0, 0.1);
  Slider1Dof::create(&eaDeg[2], &eaCurr[2], "Gamma", -190.0, 0.0, 190.0, 0.1);

  // Endless loop
  while (runLoop)
  {
    pthread_mutex_lock(&OSGLock);
    Vec3d_constMul(eaRad, eaDeg, D2R);
    Mat3d_fromEulerAngles(A, eaRad);
    //Mat3d_fromEulerAngles_Shoemake(A, eaRad, EulOrdXYZs);

    if (transposed==true)
    {
      Mat3d_transposeSelf(A);
    }

    REXEC(1)
    {
      double H[3][3];
      Mat3d_getEulerVelocityMatrix(H, eaRad);
      RMSG("Euler velocity matrix: dot(ea) = H I_omega");
      Mat3d_fprint(stderr, H);
    }

    Mat3d_toEulerAngles(eaCurr, A);
    Vec3d_constMulSelf(eaCurr, R2D);

    pthread_mutex_unlock(&OSGLock);

    if (kc->getAndResetKey('q'))
    {
      runLoop = false;
    }

    Timer_usleep(10000);

  }   // while(runLoop)

  pthread_mutex_destroy(&OSGLock);
}



/*******************************************************************************

  \brief .

*******************************************************************************/

static void testOmega(int argc, char** argv)
{
  Rcs_addResourcePath("config");

  Rcs::CmdLineParser argP(argc, argv);

  pthread_mutex_t OSGLock;
  pthread_mutex_init(&OSGLock, NULL);

  char txt[2048];
  double pos[3], omega[3], A[3][3], A_prev[3][3], angVel[3], eulErr[3], axErr[3];
  Vec3d_setZero(pos);
  Vec3d_setZero(omega);
  Mat3d_setIdentity(A);

  bool refWorld = true;
  bool pause = false;

  Rcs::Viewer v;
  osg::ref_ptr<Rcs::COSNode> cn0   = new Rcs::COSNode(pos);
  osg::ref_ptr<Rcs::COSNode> cn    = new Rcs::COSNode(pos, (double*) A);
  osg::ref_ptr<Rcs::KeyCatcher> kc = new Rcs::KeyCatcher();
  osg::ref_ptr<Rcs::HUD> hud       = new Rcs::HUD(0, 0, 700, 250);

  v.add(cn.get());
  v.add(cn0.get());
  v.add(kc.get());
  v.add(hud.get());
  v.runInThread(&OSGLock);

  Slider1Dof::create(&omega[0], &omega[0], "omega_x", -1.0, 0.0, 1.0, 0.01);
  Slider1Dof::create(&omega[1], &omega[1], "omega_y", -1.0, 0.0, 1.0, 0.01);
  Slider1Dof::create(&omega[2], &omega[2], "omega_z", -1.0, 0.0, 1.0, 0.01);

  // Endless loop
  while (runLoop)
  {

    pthread_mutex_lock(&OSGLock);
    Mat3d_copy(A_prev, A);
    Mat3d_rotateOmegaSelf(A, omega, refWorld);
    Mat3d_getOmega(A, A_prev, angVel);
    Mat3d_getEulerError(eulErr, A_prev, A);

    double angle = Mat3d_getAxisAngle(axErr, A, A_prev);
    Vec3d_constMulSelf(axErr, angle);


    sprintf(txt, "Keys:\nt   toggle world / body reference\n"
            "r   reset rotation matrix\n"
            "p   toggle pause\n"
            "omega (*10):       %+.2f   %+.2f   %+.2f\n"
            "Euler  error (*10): %+.4f   %+.4f   %+.4f   norm: %g\n"
            "Ax-ang error (*10): %+.4f   %+.4f   %+.4f   norm: %g\n"
            "Reference frame is %s   Diff. angle is %.3f deg",
            10.0*angVel[0], 10.0*angVel[1], 10.0*angVel[2],
            10.0*eulErr[0], 10.0*eulErr[1], 10.0*eulErr[2], Vec3d_getLength(eulErr),
            10.0*axErr[0], 10.0*axErr[1], 10.0*axErr[2], Vec3d_getLength(axErr),
            refWorld ? "World" : "Body", (180.0/M_PI)*Mat3d_diffAngle(A_prev, A));
    hud->setText(txt);

    if (Mat3d_isValid(A) == false)
    {
      strcat(txt, "\nInvalid rotation matrix!!!");
      Mat3d_fprint(stderr, A);
    }

    pthread_mutex_unlock(&OSGLock);

    if (kc->getAndResetKey('q'))
    {
      runLoop = false;
    }
    else if (kc->getAndResetKey('r'))
    {
      Mat3d_setIdentity(A);
    }
    else if (kc->getAndResetKey('t'))
    {
      refWorld = !refWorld;
      RMSG("Reference frame is %s", refWorld ? "world frame" : "body frame");
    }
    else if (kc->getAndResetKey('p'))
    {
      pause = !pause;
      RMSG("Pause mode is %s", pause ? "ON" : "OFF");
    }

    if (pause == true)
    {
      RPAUSE();
    }

    Timer_usleep(10000);

  }   // while(runLoop)

  pthread_mutex_destroy(&OSGLock);
}



/*******************************************************************************
 * Axis angle test
 ******************************************************************************/
static void testAxisAngleLocalFrame(int argc, char** argv)
{
  Rcs::KeyCatcherBase::registerKey("q", "Quit");
  Rcs::KeyCatcherBase::registerKey("a", "Update rotation axis");
  Rcs::KeyCatcherBase::registerKey("r", "Reset rotation matrices");
  Rcs::KeyCatcherBase::registerKey("t", "Oppose rotation matrices by 180 deg.");
  Rcs::KeyCatcherBase::registerKey("V", "Increase step size");
  Rcs::KeyCatcherBase::registerKey("v", "Decrease step size");
  Rcs::KeyCatcherBase::registerKey("+", "Step towards desired frame");
  Rcs::KeyCatcherBase::registerKey("-", "Step away from desired frame");

  double stepSize = 1.0;
  bool runLoop = true;
  char textLine[1024] = "";

  // Initialize mutex
  pthread_mutex_t mtx;
  pthread_mutex_init(&mtx, NULL);

  Rcs::Viewer viewer;

  // KeyCatcher
  osg::ref_ptr<Rcs::KeyCatcher> kc = new Rcs::KeyCatcher();
  viewer.add(kc.get());

  // HUD
  Rcs::HUD* hud = new Rcs::HUD(0, 0, 700, 200);
  viewer.add(hud);

  // Coordinate systems
  double x_avg[3], A_1I[3][3];
  Mat3d_setIdentity(A_1I);
  Vec3d_setZero(x_avg);
  osg::ref_ptr<Rcs::COSNode> cnAvg = new Rcs::COSNode(x_avg, (double*)A_1I);
  viewer.add(cnAvg.get());

  double x1[3], A_2I[3][3];
  Mat3d_setIdentity(A_2I);
  Vec3d_set(x1, 0.0, -1.5, 0.0);
  osg::ref_ptr<Rcs::COSNode> cn1 = new Rcs::COSNode(x1, (double*)A_2I);
  viewer.add(cn1.get());

  // TargetSetter for coordinate systems
  Rcs::TargetSetter* ts1 = new Rcs::TargetSetter(x1, A_2I);
  viewer.add(ts1);
  viewer.add(ts1->getHandler());

  // Axis angle arrows
  double axis[3], angle = 0;
  Vec3d_setZero(axis);
  osg::ref_ptr<Rcs::ArrowNode> an1 = new Rcs::ArrowNode(Vec3d_zeroVec(), axis);
  viewer.add(an1.get());

  // Run viewer
  viewer.runInThread(&mtx);



  while (runLoop)
  {

    strcpy(textLine, " ");

    if (kc->getAndResetKey('q'))
    {
      runLoop = false;
    }
    else if (kc->getAndResetKey('r'))
    {
      Mat3d_setIdentity(A_1I);
      Mat3d_setIdentity(A_2I);
    }
    else if (kc->getAndResetKey('t'))
    {
      double ea[3];
      Vec3d_set(ea, M_PI, 0.0, 0.0);
      Mat3d_setIdentity(A_1I);
      Mat3d_fromEulerAngles(A_2I, ea);
    }
    else if (kc->getAndResetKey('a'))
    {
      double A_21[3][3];   // Rotation from avg to des
      Mat3d_mulTranspose(A_21, A_2I, A_1I);
      angle = Mat3d_getAxisAngleSelf(axis, A_21);
    }
    else if (kc->getAndResetKey('+'))
    {
      double A_21[3][3];   // Rotation from avg to des
      Mat3d_mulTranspose(A_21, A_2I, A_1I);
      angle = Mat3d_getAxisAngleSelf(axis, A_21);

      double omega[3], diff = Mat3d_diffAngle(A_1I, A_2I);
      Vec3d_constMul(omega, axis, diff*stepSize);
      Mat3d_rotateOmegaSelf(A_1I, omega, false); // false means about local axes
    }
    else if (kc->getAndResetKey('-'))
    {
      double omega[3];
      Vec3d_constMul(omega, axis, -10.0*D2R);
      Mat3d_rotateOmegaSelf(A_1I, omega, true);
    }
    else if (kc->getAndResetKey('V'))
    {
      stepSize = Math_clip(stepSize+0.1, 0.0, 1.0);
    }
    else if (kc->getAndResetKey('v'))
    {
      stepSize = Math_clip(stepSize-0.1, 0.0, 1.0);
    }

    sprintf(textLine, "diff angle: %.2f\n"
            "step size: %.1f", R2D*angle, stepSize);

    hud->setText(textLine);

    Timer_waitDT(0.01);

    REXEC(4)
    {
      RPAUSE();
    }

  }   // while(runLoop)

  pthread_mutex_destroy(&mtx);
}



/*******************************************************************************
 * Axis angle test
 ******************************************************************************/
static void testAxisAngleWorldFrame(int argc, char** argv)
{
  Rcs::KeyCatcherBase::registerKey("q", "Quit");
  Rcs::KeyCatcherBase::registerKey("a", "Update rotation axis");
  Rcs::KeyCatcherBase::registerKey("r", "Reset rotation matrices");
  Rcs::KeyCatcherBase::registerKey("t", "Oppose rotation matrices by 180 deg.");
  Rcs::KeyCatcherBase::registerKey("V", "Increase step size");
  Rcs::KeyCatcherBase::registerKey("v", "Decrease step size");
  Rcs::KeyCatcherBase::registerKey("+", "Step towards desired frame");
  Rcs::KeyCatcherBase::registerKey("-", "Step away from desired frame");

  double stepSize = 1.0;
  bool runLoop = true;
  char textLine[1024] = "";

  // Initialize mutex
  pthread_mutex_t mtx;
  pthread_mutex_init(&mtx, NULL);

  Rcs::Viewer* viewer = new Rcs::Viewer();

  // KeyCatcher
  osg::ref_ptr<Rcs::KeyCatcher> kc = new Rcs::KeyCatcher();
  viewer->add(kc.get());

  // HUD
  osg::ref_ptr<Rcs::HUD> hud = new Rcs::HUD(0, 0, 700, 200);
  viewer->add(hud.get());

  // Coordinate systems
  double x_avg[3], A_1I[3][3];
  Mat3d_setIdentity(A_1I);
  Vec3d_setZero(x_avg);
  osg::ref_ptr<Rcs::COSNode> cnAvg = new Rcs::COSNode(x_avg, (double*)A_1I);
  viewer->add(cnAvg.get());

  double x1[3], A_2I[3][3];
  Mat3d_setIdentity(A_2I);
  Vec3d_set(x1, 0.0, -1.5, 0.0);
  osg::ref_ptr<Rcs::COSNode> cn1 = new Rcs::COSNode(x1, (double*)A_2I);
  viewer->add(cn1.get());

  // TargetSetter for coordinate systems
  Rcs::TargetSetter* ts1 = new Rcs::TargetSetter(x1, A_2I);
  viewer->add(ts1);
  viewer->add(ts1->getHandler());

  // Axis angle arrows
  double axis[3], angle = 0;
  Vec3d_setZero(axis);
  osg::ref_ptr<Rcs::ArrowNode> an1 = new Rcs::ArrowNode(Vec3d_zeroVec(), axis);
  viewer->add(an1.get());

  // Run viewer
  viewer->runInThread(&mtx);



  while (runLoop)
  {

    strcpy(textLine, " ");

    if (kc->getAndResetKey('q'))
    {
      runLoop = false;
    }
    else if (kc->getAndResetKey('r'))
    {
      Mat3d_setIdentity(A_1I);
      Mat3d_setIdentity(A_2I);
    }
    else if (kc->getAndResetKey('t'))
    {
      double ea[3];
      Vec3d_set(ea, M_PI, 0.0, 0.0);
      Mat3d_setIdentity(A_1I);
      Mat3d_fromEulerAngles(A_2I, ea);
    }
    else if (kc->getAndResetKey('a'))
    {
      angle = Mat3d_getAxisAngle(axis, A_2I, A_1I);
    }
    else if (kc->getAndResetKey('+'))
    {
      angle = Mat3d_getAxisAngle(axis, A_2I, A_1I);

      double omega[3];
      Vec3d_constMul(omega, axis, angle*stepSize);
      Mat3d_rotateOmegaSelf(A_1I, omega, true); // false means about local axes
    }
    else if (kc->getAndResetKey('-'))
    {
      double omega[3];
      Vec3d_constMul(omega, axis, -10.0*D2R);
      Mat3d_rotateOmegaSelf(A_1I, omega, true);
    }
    else if (kc->getAndResetKey('V'))
    {
      stepSize = Math_clip(stepSize+0.1, 0.0, 1.0);
    }
    else if (kc->getAndResetKey('v'))
    {
      stepSize = Math_clip(stepSize-0.1, 0.0, 1.0);
    }

    sprintf(textLine, "diff angle: %.2f\n"
            "step size: %.1f", R2D*angle, stepSize);

    hud->setText(textLine);

    Timer_waitDT(0.01);

    REXEC(4)
    {
      RPAUSE();
    }

  }   // while(runLoop)

  delete viewer;
  pthread_mutex_destroy(&mtx);
}




/*******************************************************************************

  \brief Test to average a set of rotations

*******************************************************************************/
#define N_COS (4)

static void testRotationAverage(int argc, char** argv, bool useRotationAxes = false)
{
  Rcs::KeyCatcherBase::registerKey("q", "Quit");
  Rcs::KeyCatcherBase::registerKey("r", "Reset rotation matrices");

  double eul[N_COS][3], avgEul[3];
  char textLine[1024] = "";
  char hudText[2048]  = "";

  Vec3d_setZero(avgEul);

  // Initialize mutex
  pthread_mutex_t mtx;
  pthread_mutex_init(&mtx, NULL);

  Rcs::Viewer* viewer = new Rcs::Viewer();

  // KeyCatcher
  osg::ref_ptr<Rcs::KeyCatcher> kc = new Rcs::KeyCatcher();
  viewer->add(kc.get());

  // HUD
  osg::ref_ptr<Rcs::HUD> hud = new Rcs::HUD(0, 0, 700, 200);
  viewer->add(hud.get());

  // Coordinate system of averaged rotation
  double x1[3], A_1I[3][3];
  Mat3d_setIdentity(A_1I);
  Vec3d_setZero(x1);
  osg::ref_ptr<Rcs::COSNode> cnAvg = new Rcs::COSNode(x1, (double*)A_1I);
  viewer->add(cnAvg.get());


  // Coordinate systems of desired rotations
  double x2[N_COS][3], A_2I[N_COS][3][3];

  for (unsigned int i=0; i<N_COS; i++)
  {
    int halfI = i/2;
    double sign = (i%2==0) ? -1.0 : 1.0;
    Mat3d_setIdentity(A_2I[i]);
    Vec3d_set(x2[i], 0.0, sign*(halfI+1)*1.1, 0.0);
    osg::ref_ptr<Rcs::COSNode> cn = new Rcs::COSNode(x2[i], (double*)A_2I[i], 0.75);
    viewer->add(cn.get());

    // TargetSetter for coordinate systems
    Rcs::TargetSetter* ts = new Rcs::TargetSetter(x2[i], A_2I[i]);
    viewer->add(ts);
    viewer->add(ts->getHandler());
  }

  MatNd* weight = MatNd_create(N_COS, 1);
  MatNd_setElementsTo(weight, 1.0);
  MatNdWidget::create(weight, weight, 0.0, 1.0, "weight", &mtx);



  // Run viewer
  viewer->runInThread(&mtx);

  bool runLoop = true;
  bool isAvgSafe = true;

  while (runLoop)
  {
    pthread_mutex_lock(&mtx);

    // Compute desired Euler angles from draggers
    for (unsigned int i=0; i<N_COS; i++)
    {
      Mat3d_toEulerAngles(eul[i], A_2I[i]);
    }

    // Compute mean
    if (useRotationAxes == true)
    {
      Math_weightedMeanRotationAxes(avgEul, eul, weight->ele, N_COS);
    }
    else
    {
      isAvgSafe = Math_weightedMeanEulerAngles(avgEul, eul, weight->ele, N_COS);
    }

    // Update data for visualization
    Mat3d_fromEulerAngles(A_1I, avgEul);

    pthread_mutex_unlock(&mtx);



    // HUD output
    double sumDiffAng = 0.0;
    strcpy(hudText, "");

    for (unsigned int i=0; i<N_COS; i++)
    {
      sprintf(textLine, "err[%u]: %.2f   (%.2f   %.2f   %.2f)\n", i,
              R2D*Mat3d_diffAngle(A_1I, A_2I[i]),
              R2D*Vec3d_diffAngle(A_1I[0], A_2I[i][0]),
              R2D*Vec3d_diffAngle(A_1I[1], A_2I[i][1]),
              R2D*Vec3d_diffAngle(A_1I[2], A_2I[i][2]));
      strcat(hudText, textLine);
      sumDiffAng += Mat3d_diffAngle(A_1I, A_2I[i]);
    }
    sprintf(textLine, "sum diff: %f\n", sumDiffAng);
    strcat(hudText, textLine);

    if (isAvgSafe == false)
    {
      sprintf(textLine, "Averaging is UNSSAFE\n");
      strcat(hudText, textLine);
    }

    hud->setText(hudText);



    // Catch some key events
    if (kc->getAndResetKey('q'))
    {
      runLoop = false;
    }
    else if (kc->getAndResetKey('r'))
    {
      for (unsigned int i=0; i<N_COS; i++)
      {
        Mat3d_setIdentity(A_2I[i]);
      }
    }



    Timer_waitDT(0.01);

    REXEC(4)
    {
      RPAUSE();
    }

  }   // while(runLoop)


  // Clean up
  MatNd_destroy(weight);

  delete viewer;
  pthread_mutex_destroy(&mtx);
}



/*******************************************************************************

  \brief Euler angle policy with decomposition into tangential and normal
         error components

*******************************************************************************/

static void testEulerPolicy(int argc, char** argv)
{
  double A_pi_tm1[3][3], A_des_t[3][3], A_curr_t[3][3], a, b, c, stepSize = 1.0;
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-stepSize", &stepSize);
  stepSize *= D2R;

  if (argP.hasArgument("-rnd"))
  {
    a = Math_getRandomNumber(-stepSize, stepSize);
    b = Math_getRandomNumber(-stepSize, stepSize);
    c = Math_getRandomNumber(-stepSize, stepSize);
    RLOGS(0, "EA_base: %f   %f   %f", a*R2D, b*R2D, c*R2D);
    Mat3d_fromEulerAngles2(A_pi_tm1, a, b, c);
    a = Math_getRandomNumber(-stepSize, stepSize);
    b = Math_getRandomNumber(-stepSize, stepSize);
    c = Math_getRandomNumber(-stepSize,stepSize);
    RLOGS(0, "EA_des: %f   %f   %f", a*R2D, b*R2D, c*R2D);
    Mat3d_fromEulerAngles2(A_des_t, a, b, c);
    a = Math_getRandomNumber(-stepSize, stepSize);
    b = Math_getRandomNumber(-stepSize, stepSize);
    c = Math_getRandomNumber(-stepSize, stepSize);
    RLOGS(0, "EA_curr: %f   %f   %f", a*R2D, b*R2D, c*R2D);
    Mat3d_fromEulerAngles2(A_curr_t, a, b, c);
  }
  else
  {
    Mat3d_fromEulerAngles2(A_pi_tm1, 0.0*D2R, 0.0*D2R, 0.0*D2R);
    Mat3d_fromEulerAngles2(A_des_t, 1.0*D2R, 0.0*D2R, 0.0*D2R);
    Mat3d_fromEulerAngles2(A_curr_t, 0.5*D2R, 0.5*D2R, 0.0*D2R);
  }

  RLOGS(0, "A_pi_tm1");
  Mat3d_fprint(stderr, A_pi_tm1);

  RLOGS(0, "A_des_t");
  Mat3d_fprint(stderr, A_des_t);

  RLOGS(0, "A_curr_t");
  Mat3d_fprint(stderr, A_curr_t);


  // Compute the current and desired policies
  double phi, pi_des[3], pi_curr[3];

  // Desired policy
  phi = Mat3d_getAxisAngle(pi_des, A_des_t, A_pi_tm1);
  Vec3d_constMulSelf(pi_des, phi);
  RLOGS(0, "pi_des: %f   %f   %f", V3(pi_des));

  // Current policy
  phi = Mat3d_getAxisAngle(pi_curr, A_curr_t, A_pi_tm1);
  Vec3d_constMulSelf(pi_curr, phi);
  RLOGS(0, "pi_curr: %f   %f   %f", V3(pi_curr));

  // Compute the difference between desired and current policy
  double dpi[3];
  Vec3d_sub(dpi, pi_des, pi_curr);
  RLOGS(0, "dpi: %f   %f   %f", V3(dpi));

  // Compute the projected pi (pi_curr on pi_des)
  double pi_prj[3];
  Vec3d_setZero(pi_prj);
  double len = Vec3d_sqrLength(pi_des);

  if (len > 0.0)
  {
    Vec3d_constMul(pi_prj, pi_des,
                   Vec3d_innerProduct(pi_curr, pi_des)/len);
  }

  // Split is up into tangential and normal components
  double e_t[3], e_n[3], e_full[3];
  Vec3d_sub(e_t, pi_des, pi_prj);
  Vec3d_sub(e_n, pi_prj, pi_curr);

  Vec3d_add(e_full, e_t, e_n);

  RLOGS(0, "e = %f   %f   %f   dpi = %f   %f   %f",
        V3(e_full), V3(dpi));
  Vec3d_subSelf(e_full, dpi);
  RLOGS(0, "err = %g", Vec3d_getLength(e_full));





  // Test: We decompose the rotation A_des_t = A_curr_t + pi
  // It turns out that it suffers from linearization errors for large
  // angular differences. But for small displacements, it is acceptable.
  if (0)
  {
    double A_test[3][3];
    Mat3d_copy(A_test, A_curr_t);
    RLOGS(0, "Before policy step: Phi = %f",
          R2D*Mat3d_diffAngle(A_test, A_des_t));

    Mat3d_rotateOmegaSelf(A_test, dpi, true);
    RLOGS(0, "After policy step: Phi = %f",
          R2D*Mat3d_diffAngle(A_test, A_des_t));

    // RCHECK(Mat3d_isValid(A_test));
    // Mat3d_printCommentDigits("A_test", A_test, 16);
    // Mat3d_printCommentDigits("A_des_t", A_des_t, 16);

    // Mat3d_subSelf(A_test, A_des_t);
    // Mat3d_printCommentDigits("A_test - A_des_t", A_test, 16);
  }





  // Test: Same as above, but we decompose the policy into tangential and
  // normal components.
  if (1)
  {
    double A_test[3][3];
    Mat3d_copy(A_test, A_curr_t);
    RLOGS(0, "Before policy step: Phi = %f",
          R2D*Mat3d_diffAngle(A_test, A_des_t));

    Mat3d_rotateOmegaSelf(A_test, e_t, true);
    Mat3d_rotateOmegaSelf(A_test, e_n, true);
    RLOGS(0, "After policy step: Phi = %f",
          R2D*Mat3d_diffAngle(A_test, A_des_t));

    // RCHECK(Mat3d_isValid(A_test));
    // Mat3d_printCommentDigits("A_test", A_test, 16);
    // Mat3d_printCommentDigits("A_des_t", A_des_t, 16);

    // Mat3d_subSelf(A_test, A_des_t);
    // Mat3d_printCommentDigits("A_test - A_des_t", A_test, 16);
  }










  // if(0)
  // {// Test:
  //   double A_test[3][3], omega_test[3], phi_test;
  //   phi_test = Mat3d_getAxisAngle(omega_test, A_curr_t, A_pi_tm1);
  //   Vec3d_constMulSelf(omega_test, phi_test);
  //   Mat3d_copy(A_test, A_pi_tm1);
  //   Mat3d_rotateOmegaSelf(A_test, omega_test, true);

  //   Mat3d_printCommentDigits("A_test", A_test, 16);

  //   RLOGS(0, "A_curr_t");
  //   Mat3d_fprint(stderr, A_curr_t);

  //   RLOGS(0, "A_test - A_curr_t");
  //   Mat3d_subSelf(A_test, A_curr_t);
  //   Mat3d_fprint(stderr, A_test);
  // }

}



/*******************************************************************************

  \brief Displays the Euler angles.

*******************************************************************************/

int main(int argc, char** argv)
{
  // Ctrl-C callback handler
  signal(SIGINT, quit);

  int mode = 0;
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-m", &mode);
  argP.getArgument("-dl", &RcsLogLevel);

  // For colors, fonts etc.
  Rcs_addResourcePath("config");

  switch (mode)
  {
    case 0:
    {
      fprintf(stderr, "\n\tHere's some useful testing modes:\n\n");
      fprintf(stderr, "\n\t-m");
      fprintf(stderr, "\t0   Prints this message (default)\n");
      fprintf(stderr, "\t\t1   Euler angles test\n");
      fprintf(stderr, "\t\t2   Omega test\n");
      fprintf(stderr, "\t\t3   Axis angle test (world coordinates)\n");
      fprintf(stderr, "\t\t4   Axis angle test (local coordinates)\n");
      fprintf(stderr, "\t\t5   Rotation matrix averaging test\n");
      fprintf(stderr, "\t\t6   Rotation matrix averaging test (using axis angle averaging)\n");
      break;
    }

    case 1:
      testEulerAngles(argc, argv);
      break;

    case 2:
      testOmega(argc, argv);
      break;

    case 3:
      testAxisAngleWorldFrame(argc, argv);
      break;

    case 4:
      testAxisAngleLocalFrame(argc, argv);
      break;

    case 5:
      testRotationAverage(argc, argv, false);
      break;

    case 6:
      testRotationAverage(argc, argv, true);
      break;

    case 7:
      testEulerPolicy(argc, argv);
      break;

    default:
      RFATAL("Mode %d not yet implemented", mode);
  }

  RcsGuiFactory_shutdown();

  fprintf(stderr, "Thanks for using the Rcs library\n");

  return 0;
}
