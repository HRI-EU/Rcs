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

#include <SegFaultHandler.h>

#include <Rcs_math.h>
#include <Rcs_mathTests.h>
#include <Rcs_macros.h>
#include <Rcs_cmdLine.h>
#include <Rcs_utils.h>
#include <Rcs_timer.h>

#include <stdlib.h>
#include <string>


RCS_INSTALL_ERRORHANDLERS


static bool testMode(int mode, int argc, char** argv)
{
  int numTests = 1;
  bool success = true;

  // To initialize random seed
  Math_getRandomNumber(-1.0, 1.0);

  // Parse command line arguments
  Rcs::CmdLineParser argP;
  argP.getArgument("-numTests", &numTests, "Number of repetitions");


  for (int i=0; i<numTests; i++)
  {
    switch (mode)
    {
      // ==============================================================
      // Just print out some global information
      // ==============================================================
      case 0:
      {
        fprintf(stderr, "\n\tHere's some useful testing modes:\n\n");
        fprintf(stderr, "\t-dl\t<0...>   Sets the debug level ");
        fprintf(stderr, "(0 is default)\n\t-dim");
        fprintf(stderr, "\t<0...>  Dimension of arrays ");
        fprintf(stderr, "(5 is default)\n\t-m");
        fprintf(stderr, "\t-1  Runs all tests\n");
        fprintf(stderr, "\t\t0   Prints this message (default)\n");
        fprintf(stderr, "\t\t1   Tests Euler Angles functions\n");
        fprintf(stderr, "\t\t2   Tests simple matrix functions\n");
        fprintf(stderr, "\t\t3   Tests linear algebra functions\n");
        fprintf(stderr, "\t\t4   Tests derivative functions\n");
        fprintf(stderr, "\t\t5   Tests HTr functions\n");
        fprintf(stderr, "\t\t6   Tests basic math functions\n");
        fprintf(stderr, "\t\t7   Tests curve fitting functions\n");
        fprintf(stderr, "\t\t8   Tests vector projection\n");
        fprintf(stderr, "\t\t9   Tests 3x3 matrix orthogonalisation\n");
        fprintf(stderr, "\t\t10  Tests Eigenvalues for 3d matrices\n");
        fprintf(stderr, "\t\t11  Tests isnan and isfinite functions\n");
        fprintf(stderr, "\t\t12  Tests the Woodbury Matrix Identity\n");
        fprintf(stderr, "\t\t13  Tests random number generators\n");
        fprintf(stderr, "\t\t14  Tests Axis Angle conversion\n");
        fprintf(stderr, "\t\t15  Tests Axis Angle interpolation\n");
        fprintf(stderr, "\t\t16  Tests Miller inversion algorithm\n");
        fprintf(stderr, "\t\t17  Tests minimum rotation angle\n");
        fprintf(stderr, "\t\t18  Test line search\n");
        fprintf(stderr, "\t\t19  Tests minimum jerk trajectory\n");
        fprintf(stderr, "\t\t20  Tests 5th order poly min. jerk trajectory\n");
        fprintf(stderr, "\t\t21  Tests minimum jerk interpolation\n");
        fprintf(stderr, "\t\t22  Test numerical issues\n");
        fprintf(stderr, "\t\t23  Test 1-dimensional filters\n");
        fprintf(stderr, "\t\t24  Test n-dimensional filters\n");
        fprintf(stderr, "\t\t25  Tests moving man filter\n");
        fprintf(stderr, "\t\t26  Test Dynamic Time Warping\n");
        fprintf(stderr, "\t\t27  Test Mat3d functions\n");
        fprintf(stderr, "\t\t28  Test ViaPointSequence\n");
        fprintf(stderr, "\t\t29  Test polynomial root finding\n");
        fprintf(stderr, "\t\t30  Test ViaPointSequence plotting\n");
        fprintf(stderr, "\t\t31  Test ViaPointTrajectory1D\n");
        fprintf(stderr, "\t\t32  Test Quaternion conversion\n");
        fprintf(stderr, "\t\t33  Test Eigen3 linear algebra functions\n");
        fprintf(stderr, "\t\t34  Test SLERP against matrix clip\n");
        fprintf(stderr, "\t\t35  Test StackVec class\n");
        fprintf(stderr, "\t\t36  Tests interpolation algorithms\n");
        fprintf(stderr, "\t\t37  Tests plane fitting of 3d points\n");
        break;
      }

      case 1:
        success = testEulerAnglesFunctions(argc, argv) && success;
        break;
      case 2:
        success = testSimpleMatrixFunctions(argc, argv) && success;
        break;
      case 3:
        success = testLinearAlgebraFunctions(argc, argv) && success;
        break;
      case 4:
        success = testDerivatives(argc, argv) && success;
        break;
      case 5:
        success = testHTr(argc, argv) && success;
        break;
      case 6:
        success = testBasicMath(argc, argv) && success;
        break;
      case 7:
        success = testCurveFitting(argc, argv) && success;
        break;
      case 8:
        success = testVectorProjection(argc, argv) && success;
        break;
      case 9:
        success = testOrthogonalization3x3(argc, argv) && success;
        break;
      case 10:
        success = testEigenvalues3x3(argc, argv) && success;
        break;
      case 11:
        success = testFiniteNan() && success;
        break;
      case 12:
        success = testWoodburyIdenity(argc, argv) && success;
        break;
      case 13:
        success = testRnd(argc, argv) && success;
        break;
      case 14:
        success = testAxisAngleConversion(argc, argv) && success;
        break;
      case 15:
        success = testAxisAngleInterpolation(argc, argv) && success;
        break;
      case 16:
        success = testMillerInversion(argc, argv) && success;
        break;
      case 17:
        success = testMinimumRotationAngle(argc, argv) && success;
        break;
      case 18:
        success = testLinesearch(argc, argv) && success;
        break;
      case 19:
        success = testMinJerkTrj(argc, argv) && success;
        break;
      case 20:
        success = testMinJerkTrjPoly(argc, argv) && success;
        break;
      case 21:
        success = testArcLengthInterpolation(argc, argv) && success;
        break;
      case 22:
        success = testNumerics(argc, argv) && success;
        break;
      case 23:
        success = testFilters1D(argc, argv) && success;
        break;
      case 24:
        success = testFiltersND(argc, argv) && success;
        break;
      case 25:
        success = testMovingMeanFilter(argc, argv) && success;
        break;
      case 26:
        success = testDTW(argc, argv) && success;
        break;
      case 27:
        success = testMat3dFunctions(argc, argv) && success;
        break;
      case 28:
        success = testViaPointSequence(argc, argv) && success;
        break;
      case 29:
        success = testPolynomialRootFinding(argc, argv) && success;
        break;
      case 30:
        success = testViaPointSequencePlotter(argc, argv) && success;
        break;
      case 31:
        success = testViaPointTrajectory1D(argc, argv) && success;
        break;
      case 32:
        success = testQuaternionConversion(argc, argv) && success;
        break;
      case 33:
        success = testFunctionsEigen3(argc, argv) && success;
        break;
      case 34:
        success = testSlerp(argc, argv) && success;
        break;
      case 35:
        success = testStackVec(argc, argv) && success;
        break;
      case 36:
        success = testInterpolation(argc, argv) && success;
        break;
      case 37:
        success = testPlaneFit3d(argc, argv) && success;
        break;
      default:
        RMSGS("there is no mode %d", mode);

    }   // switch(mode)

  }   // for (int i=0; i<numTests; i++)


  return success;
}


/*******************************************************************************
 *
 ******************************************************************************/
int main(int argc, char** argv)
{
  int result = 0, mode = 0;

  // Parse command line arguments
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-dl", &RcsLogLevel, "Set debug level (default is %d)",
                   RcsLogLevel);
  argP.getArgument("-m", &mode, "Set test mode (default is %d)", mode);

  bool success = true;

  if (mode==-1)
  {
    for (int i=1; i<=18; ++i)
    {
      bool success_i = testMode(i, argc, argv);
      if (!success_i)
      {
        result++;
      }

      success = success_i && success;
    }
  }
  else
  {
    success = testMode(mode, argc, argv);
  }

  if (argP.hasArgument("-h"))
  {
    argP.print();
  }
  else
  {
    RLOGS(1, "Math tests %s", success ? "SUCCEEDED" : "FAILED");
  }

  return Math_iClip(result, 0, 255);
}
