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

#ifndef RCS_MATHTESTS_H
#define RCS_MATHTESTS_H



#ifdef __cplusplus
extern "C" {
#endif

bool testEulerAnglesFunctions(int argc, char** argv);
bool testSimpleMatrixFunctions(int argc, char** argv);
bool testLinearAlgebraFunctions(int argc, char** argv);
bool testDerivatives(int argc, char** argv);
bool testHTr(int argc, char** argv);
bool testBasicMath(int argc, char** argv);
bool testCurveFitting(int argc, char** argv);
bool testVectorProjection(int argc, char** argv);
bool testOrthogonalization3x3(int argc, char** argv);
bool testFiniteNan();
bool testWoodburyIdenity(int argc, char** argv);
bool testRnd(int argc, char** argv);
bool testAxisAngleConversion(int argc, char** argv);
bool testAxisAngleInterpolation(int argc, char** argv);
bool testMillerInversion(int argc, char** argv);
bool testInterpolation(int argc, char** argv);
bool testMovingMeanFilter(int argc, char** argv);
bool testMinJerkTrj(int argc, char** argv);
bool testMinJerkTrjPoly(int argc, char** argv);
bool testArcLengthInterpolation(int argc, char** argv);
bool testNumerics(int argc, char** argv);
bool testFilters1D(int argc, char** argv);
bool testFiltersND(int argc, char** argv);
bool testMinimumRotationAngle(int argc, char** argv);
bool testLinesearch(int argc, char** argv);
bool testDTW(int argc, char** argv);
bool testMat3dFunctions(int argc, char** argv);
bool testViaPointSequence(int argc, char** argv);
bool testPolynomialRootFinding(int argc, char** argv);
bool testEigenvalues3x3(int argc, char** argv);
bool testViaPointSequencePlotter(int argc, char** argv);
bool testViaPointTrajectory1D(int argc, char** argv);
bool testQuaternionConversion(int argc, char** argv);
bool testViaPointGradient(int argc, char** argv);
bool testFunctionsEigen3(int argc, char** argv);
bool testSlerp(int argc, char** argv);
bool testStackVec(int argc, char** argv);


#ifdef __cplusplus
}
#endif

#endif   // RCS_MATHTESTS_H
