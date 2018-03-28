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


int main(int argc, char** argv)
{
  int mode = 0, numTests = 1;
  bool success = true;

  // To initialize random seed
  Math_getRandomNumber(-1.0, 1.0);

  // Parse command line arguments
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-dl", &RcsLogLevel, "Set debug level");
  argP.getArgument("-m", &mode, "Set test mode");
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
        fprintf(stderr, "\t0   Prints this message (default)\n");
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
        fprintf(stderr, "\t\t17  Tests interpolation algorithms\n");
        fprintf(stderr, "\t\t18  Tests moving man filter\n");
        fprintf(stderr, "\t\t19  Tests minimum jerk trajectory\n");
        fprintf(stderr, "\t\t20  Tests 5th order poly min. jerk trajectory\n");
        fprintf(stderr, "\t\t21  Tests minimum jerk interpolation\n");
        fprintf(stderr, "\t\t22  Test numerical issues\n");
        fprintf(stderr, "\t\t23  Test 1-dimensional filters\n");
        fprintf(stderr, "\t\t24  Test n-dimensional filters\n");
        fprintf(stderr, "\t\t25  Test line search\n");
        fprintf(stderr, "\t\t26  Test Dynamic Time Warping\n");
        fprintf(stderr, "\t\t27  Test Mat3d functions\n");
        fprintf(stderr, "\t\t28  Test ViaPointSequence\n");
        fprintf(stderr, "\t\t29  Test polynomial root finding\n");
        fprintf(stderr, "\t\t30  Test ViaPointSequence plotting\n");
        fprintf(stderr, "\t\t31  Test ViaPointTrajectory1D\n");
        fprintf(stderr, "\t\t32  Test Quaternion conversion\n");
        fprintf(stderr, "\t\t33  Test Eigen3 linear algebra functions\n");
        break;
      }

      case 1:
        success = testEulerAnglesFunctions(argc, argv);
        break;
      case 2:
        success = testSimpleMatrixFunctions(argc, argv);
        break;
      case 3:
        success = testLinearAlgebraFunctions(argc, argv);
        break;
      case 4:
        success = testDerivatives(argc, argv);
        break;
      case 5:
        success = testHTr(argc, argv);
        break;
      case 6:
        success = testBasicMath(argc, argv);
        break;
      case 7:
        success = testCurveFitting(argc, argv);
        break;
      case 8:
        success = testVectorProjection(argc, argv);
        break;
      case 9:
        success = testOrthogonalization3x3(argc, argv);
        break;
      case 10:
        success = testEigenvalues3x3(argc, argv);
        break;
      case 11:
        success = testFiniteNan();
        break;
      case 12:
        success = testWoodburyIdenity(argc, argv);
        break;
      case 13:
        success = testRnd(argc, argv);
        break;
      case 14:
        success = testAxisAngleConversion(argc, argv);
        break;
      case 15:
        success = testAxisAngleInterpolation(argc, argv);
        break;
      case 16:
        success = testMillerInversion(argc, argv);
        break;
      case 17:
        success = testInterpolation(argc, argv);
        break;
      case 18:
        success = testMovingMeanFilter(argc, argv);
        break;
      case 19:
        success = testMinJerkTrj(argc, argv);
        break;
      case 20:
        success = testMinJerkTrjPoly(argc, argv);
        break;
      case 21:
        success = testArcLengthInterpolation(argc, argv);
        break;
      case 22:
        success = testNumerics(argc, argv);
        break;
      case 23:
        success = testFilters1D(argc, argv);
        break;
      case 24:
        success = testFiltersND(argc, argv);
        break;
      case 25:
        success = testLinesearch(argc, argv);
        break;
      case 26:
        success = testDTW(argc, argv);
        break;
      case 27:
        success = testMat3dFunctions(argc, argv);
        break;
      case 28:
        success = testViaPointSequence(argc, argv);
        break;
      case 29:
        success = testPolynomialRootFinding(argc, argv);
        break;
      case 30:
        success = testViaPointSequencePlotter(argc, argv);
        break;
      case 31:
        success = testViaPointTrajectory1D(argc, argv);
        break;
      case 32:
        success = testQuaternionConversion(argc, argv);
        break;
      case 33:
        success = testFunctionsEigen3(argc, argv);
        break;
      default:
        RMSGS("there is no mode %d", mode);

    }   // switch(mode)

  }   // for (int i=0; i<numTests; i++)


  if (argP.hasArgument("-h"))
  {
    argP.print();
  }

  printf("Test %s\n", success ? "SUCCEEDED" : "FAILED");

  return 0;
}
