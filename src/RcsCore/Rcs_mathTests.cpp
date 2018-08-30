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

#include "Rcs_mathTests.h"
#include "Rcs_math.h"
#include "Rcs_quaternion.h"
#include "Rcs_filters.h"
#include "ViaPointSequence.h"
#include "Rcs_macros.h"
#include "Rcs_cmdLine.h"
#include "Rcs_utils.h"
#include "Rcs_timer.h"

#include <iostream>
#include <limits>
#include <cfloat>



/*******************************************************************************
 * ABAt = A * B * A^T (Naiive implementation)
 ******************************************************************************/
static void MatNd_sqrMulABAt_simple(MatNd* ABAt, const MatNd* A, const MatNd* B)
{
  MatNd* At = NULL, *BAt = NULL;

  // No weighting matrix
  if (B == NULL)
  {
    MatNd_create2(At, A->n, A->m);
    MatNd_transpose(At, A);
    MatNd_mul(ABAt, A, At);
    MatNd_destroy(At);
  }
  // n x 1 weighting vector
  else if (B->n == 1)
  {
    MatNd_create2(BAt, A->n, A->m);
    MatNd_transpose(BAt, A);
    MatNd_preMulDiagSelf(BAt, B);
    MatNd_mul(ABAt, A, BAt);
    MatNd_destroy(BAt);
  }
  // Square weighting matrix
  else if (B->n == B->m)
  {
    MatNd_create2(At, A->n, A->m);
    MatNd_create2(BAt, B->m, At->n);
    MatNd_transpose(At, A);    // A^T
    MatNd_mul(BAt, B, At);     // B * A^T
    MatNd_mul(ABAt, A, BAt);   // A * B * A^T
    MatNd_destroy(BAt);
    MatNd_destroy(At);
  }

}

/*******************************************************************************
 * Euler angles test: Create a 3x3 matrix A1 from 3 Euler angles (xyz order),
 * calculate the Euler angles from this matrix, and compare the resulting
 * matrix A2 with the matrix A1.
 ******************************************************************************/
bool testEulerAnglesFunctions(int argc, char** argv)
{
  bool success = true;
  double ea0[3], ea1[3], A_KI[3][3], A_test[3][3], A_diff[3][3];
  Vec3d_setRandom(ea0, -720.0, 720.0);

  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-a", &ea0[0]);
  argP.getArgument("-b", &ea0[1]);
  argP.getArgument("-c", &ea0[2]);

  Vec3d_constMulSelf(ea0, M_PI/180.0);
  Mat3d_fromEulerAngles(A_KI, ea0);
  Mat3d_toEulerAngles(ea1, A_KI);
  Mat3d_fromEulerAngles(A_test, ea1);
  RLOG(2, "in:  %f   %f   %f",
       ea0[0] * (180.0 / M_PI), ea0[1] * (180.0 / M_PI),
       ea0[2] * (180.0 / M_PI));
  RLOG(2, "out: %f   %f   %f",
       ea1[0] * (180.0 / M_PI), ea1[1] * (180.0 / M_PI),
       ea1[2] * (180.0 / M_PI));

  Mat3d_sub(A_diff, A_test, A_KI);
  const double errMax = 1.0e-8;
  double err = VecNd_getLength((double*) A_diff, 9);

  if (err>errMax)
  {
    REXEC(2)
    {
      Mat3d_printCommentDigits("A_KI", A_KI, 4);
      Mat3d_printCommentDigits("A_test", A_test, 4);
      Mat3d_printCommentDigits("A_diff", A_diff, 4);
    }
    success = false;
  }

  RLOGS(1, "%s", success ? "SUCCESS" : "FAILURE");
  RLOG(2, "Error is %g (<%g)", err, errMax);

  return success;
}

/*******************************************************************************
 * Test for simple vector-matrix functions:
 * MatNd_copyColumns
 * MatNd_postMulDiagSelf
 * MatNd_insertColumns
 * MatNd_fromString
 * MatNd_toFile
 * MatNd_fromFile
 ******************************************************************************/
bool testSimpleMatrixFunctions(int argc, char** argv)
{
  bool success = true;

  // MatNd_copyColumns
  {
    // Copy column 0, 4 and 1 from src into dst.
    RLOGS(2, "**************************************");
    RLOGS(2, "Test for MatNd_copyColumns");
    RLOGS(2, "**************************************");
    int rows = 5;
    MatNd* src = MatNd_create(rows, 6);
    MatNd_setRandom(src, 0.0, 1.0);
    MatNd* dst = MatNd_create(rows, 6);
    int cols[4] = { 0, 4, 1, -1 };
    MatNd_copyColumns(dst, src, cols);

    REXEC(2)
    {
      MatNd_printCommentDigits("src", src, 3);
      MatNd_printCommentDigits("dst", dst, 3);
    }

    // Calculate difference of columns
    double sqrErr = 0.0;
    for (int col=0; col<3; col++)
    {
      for (int row=0; row<rows; row++)
      {
        double srcEle = MatNd_get(src, row, cols[col]);
        double dstEle = MatNd_get(dst, row, col);
        sqrErr += pow(srcEle-dstEle, 2);
      }

    }

    if (sqrErr>0.0)
    {
      success = false;
      RLOG(2, "Test for MatNd_copyColumns failed: sqrErr = %g", sqrErr);
    }

    MatNd_destroy(src);
    MatNd_destroy(dst);
  }

  // MatNd_postMulDiagSelf
  {
    RLOGS(2, "**************************************");
    RLOGS(2, "Test for MatNd_postMulDiagSelf");
    RLOGS(2, "**************************************");
    MatNd* A = MatNd_create(2, 3);
    char matrixA[512] = "1 2 3 , 4 5 6";
    MatNd_fromString(A, matrixA);

    MatNd* B = MatNd_create(3, 1);
    char matrixB[512] = "10 , 20 , 30";
    MatNd_fromString(B, matrixB);

    REXEC(2)
    {
      RMSGS("A is");
      MatNd_print(A);
      RMSGS("B is");
      MatNd_print(B);
    }

    MatNd_postMulDiagSelf(A, B);

    REXEC(2)
    {
      RMSGS("Result: A is");
      MatNd_print(A);

      RMSGS("Should be:");
      RMSGS("A              B            Result      ");
      RMSGS("1 2 3          10           10 40 90");
      RMSGS("4 5 6          20           40 100 180");
      RMSGS("               30");
    }

    MatNd* expectedResult = MatNd_create(2, 3);
    char res[512] = "10 40 90 , 40 100 180";
    MatNd_fromString(expectedResult, res);

    if (MatNd_isEqual(expectedResult, A, 0.0) == false)
    {
      success = false;
      RLOG(2, "Test for MatNd_postMulDiagSelf failed");
    }

    MatNd_destroy(A);
    MatNd_destroy(B);
    MatNd_destroy(expectedResult);
  }

  // MatNd_insertColumns
  {
    RLOGS(2, "**************************************");
    RLOGS(2, "Test for MatNd_insertColumns");
    RLOGS(2, "**************************************");
    int rows = 10;
    MatNd* A = MatNd_create(rows, 5);
    for (unsigned int i = 0; i < A->n; i++)
      for (unsigned int j = 0; j < A->m; j++)
      {
        MatNd_set(A, j, i, i);
      }
    MatNd* A_org = MatNd_clone(A);

    MatNd* B = MatNd_create(rows, 5);
    for (unsigned int i = 0; i < B->n; i++)
      for (unsigned int j = 0; j < B->m; j++)
      {
        MatNd_set(B, j, i, i + 10);
      }

    REXEC(2)
    {
      MatNd_printCommentDigits("A", A_org, 3);
      MatNd_printCommentDigits("B", B, 3);
      RMSGS("Insert columns 1-3 of B after column 1 of A. Please note that "
            "all indices start from 0");
    }

    MatNd_insertColumns(A, 1, B, 1, 3);

    REXEC(2)
    {
      MatNd_printCommentDigits("A", A, 3);
    }


    // Calculate difference of columns
    double sqrErr = 0.0;

    for (int col=0; col<2; col++)
    {
      for (int row=0; row<rows; row++)
      {
        double srcEle = MatNd_get(A_org, row, col);
        double dstEle = MatNd_get(A, row, col);
        sqrErr += pow(srcEle-dstEle, 2);
      }
    }

    for (int col=2; col<5; col++)
    {
      for (int row=0; row<rows; row++)
      {
        double srcEle = MatNd_get(B, row, col-1);
        double dstEle = MatNd_get(A, row, col);
        sqrErr += pow(srcEle-dstEle, 2);
      }
    }

    for (int col=5; col<8; col++)
    {
      for (int row=0; row<rows; row++)
      {
        double srcEle = MatNd_get(A_org, row, col-3);
        double dstEle = MatNd_get(A, row, col);
        sqrErr += pow(srcEle-dstEle, 2);
      }
    }


    if (sqrErr>0.0)
    {
      success = false;
      RLOG(2, "Test for MatNd_insertColumns failed: sqrErr = %g", sqrErr);
    }



    MatNd_destroy(A);
    MatNd_destroy(A_org);
    MatNd_destroy(B);
  }

  // MatNd_fromString
  // MatNd_toFile
  // MatNd_fromFile
  {
    RLOGS(2, "**************************************");
    RLOGS(2, "Test for MatNd_fromString");
    RLOGS(2, "         MatNd_toFile");
    RLOGS(2, "         MatNd_fromFile");
    RLOGS(2, "**************************************");
    MatNd* A = MatNd_create(10, 10);
    MatNd* B = MatNd_create(10, 10);
    char matrix[512] = "1 2 3 4 , 5 6 7 8 , 9 10 11 12 , 13 14 15 16";
    MatNd_fromString(A, matrix);

#if defined (_MSC_VER)
    const char* fileName = "C:\\Temp\\deleteme.dat";
#else
    const char* fileName = "/tmp/deleteme.dat";
#endif

    REXEC(2)
    {
      RMSGS("A is");
      MatNd_print(A);
    }

    RLOG(2, "Writing A to %s", fileName);
    MatNd_toFile(A, fileName);

    RLOG(2, "Reading B from %s", fileName);
    MatNd_fromFile(B, fileName);

    REXEC(2)
    {
      RMSGS("B is");
      MatNd_print(B);
    }

    if (MatNd_isEqual(A, B, 0.0) == false)
    {
      REXEC(2)
      {
        RMSGS("[Test for MatNd_fromString, MatNd_toFile, "
              "MatNd_fromFile]: A and B are not equal:");
        MatNd_print(A);
        MatNd_print(B);
      }

      success = false;
    }

    MatNd_destroy(A);
    MatNd_destroy(B);

    RLOG(2, "Deleting %s", fileName);
    char sysStr[256];
#if defined (_MSC_VER)
    strcpy(sysStr, "del ");
#else
    strcpy(sysStr, "/bin/rm ");
#endif
    strcat(sysStr, fileName);
    int err = system(sysStr);

    if (err == -1)
    {
      RLOG(2, "Couldn't delete %s - please do manually", fileName);
      success = false;
    }

  }

  // MatNd_appendToFile
  {
    RLOGS(2, "**************************************");
    RLOGS(2, "Test for MatNd_appendToFile");
    RLOGS(2, "**************************************");
    MatNd* A = MatNd_create(5, 2);

#if defined (_MSC_VER)
    const char* fileName = "C:\\Temp\\deleteme2.dat";
#else
    const char* fileName = "/tmp/deleteme2.dat";
#endif

    for (unsigned int i=0; i<A->m; i++)
    {
      for (unsigned int j=0; j<A->n; j++)
      {
        MatNd_set(A, i, j, i*A->n+j);
      }

    }

    REXEC(2)
    {
      RMSGS("A is");
      MatNd_print(A);
    }

    RLOG(2, "Writing A to %s", fileName);
    MatNd_toFile(A, fileName);

    RLOG(2, "Appending A to %s", fileName);
    MatNd_appendToFile(A, fileName);

    MatNd* B = MatNd_createFromFile(fileName);
    RCHECK(B);

    REXEC(2)
    {
      MatNd_printCommentDigits("A appended to itself is", B, 3);
    }

    RLOG(2, "Deleting %s", fileName);
    char sysStr[256];
#if defined (_MSC_VER)
    strcpy(sysStr, "del ");
#else
    strcpy(sysStr, "/bin/rm ");
#endif
    strcat(sysStr, fileName);
    int err = system(sysStr);

    if (err == -1)
    {
      RLOG(2, "Couldn't delete %s - please do manually", fileName);
      success = false;
    }

    MatNd_destroy(A);
    MatNd_destroy(B);
  }

  // MatNd_sqrMulABAt
  {
    RLOGS(2, "**************************************");
    RLOGS(2, "Test for MatNd_sqrMulABAt");
    RLOGS(2, "**************************************");
    int m=30, n=10, iter=1;
    Rcs::CmdLineParser argP(argc, argv);
    argP.getArgument("-rows", &m, "Number of rows (Default is 30)");
    argP.getArgument("-cols", &n, "Number of columns (Default is 10)");
    argP.getArgument("-iter", &iter, "Number of iterations");
    bool bNull = argP.hasArgument("-BNULL", "Weight matrix is NULL");

    MatNd* A = MatNd_create(m, n);
    MatNd_setRandom(A, -1.0, 1.0);

    MatNd* B = NULL;

    if (bNull==false)
    {
      B = MatNd_create(n, 1);
      MatNd_setRandom(B, -1.0, 1.0);
    }

    MatNd* ABAt1 = MatNd_create(m, m);
    MatNd* ABAt2 = MatNd_create(m, m);

    double dt1 = Timer_getTime();
    for (int i=0; i<iter; i++)
    {
      MatNd_sqrMulABAt_simple(ABAt1, A, B);
    }
    dt1 = (1.0/iter)*(Timer_getTime() - dt1);

    double dt2 = Timer_getTime();
    for (int i=0; i<iter; i++)
    {
      MatNd_sqrMulABAt(ABAt2, A, B);
    }
    dt2 = (1.0/iter)*(Timer_getTime() - dt2);

    double msqError = MatNd_msqError(ABAt1, ABAt2);

    RLOG(2, "dt1=%.4f usec   dt2=%.4f usec   err=%g",
         1.0e6*dt1, 1.0e6*dt2, msqError);

    REXEC(3)
    {
      MatNd_printTwoArraysDiff(ABAt1, ABAt2, 3);
    }

    if (msqError > 1.0e-8)
    {
      RLOG(2, "MatNd_sqrMulABAt has error of %g (> 1.0e-8)", msqError);
      success = false;
    }

    MatNd_destroy(A);
    MatNd_destroy(B);
    MatNd_destroy(ABAt1);
    MatNd_destroy(ABAt2);
  }

  RLOGS(1, "%s", success ? "SUCCESS" : "FAILURE");

  return success;
}

/*******************************************************************************
 * Linear algebra tests
 ******************************************************************************/
bool testLinearAlgebraFunctions(int argc, char** argv)
{
  bool success = true;
  int dim = 5;

  // Parse command line arguments
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-dim", &dim, "Set default dimansionality");
  bool testChol = argP.hasArgument("-chol", "Test Cholesky functions only");
  bool testGS = argP.hasArgument("-gaussSeidl", "Test Gauss-Seidl "
                                 "functions");
  bool testInverse = argP.hasArgument("-inverse", "Test matrix inversion");
  bool testSolve = argP.hasArgument("-solve", "Test solving linear equation "
                                    "systems with all algorithms");
  bool testAll = (!testChol) && (!testGS) && (!testInverse) && (!testSolve);

  // MatNd_choleskyDecomposition
  if (testAll || testChol)
  {
    RLOGS(2, "**************************************");
    RLOGS(2, "Test for MatNd_choleskyDecomposition");
    RLOGS(2, "**************************************");
    int n = dim;   // dimension of square matrix
    MatNd* J      = MatNd_create(3 * n, n);
    MatNd* A      = MatNd_create(n, n);
    MatNd* L      = MatNd_create(n, n);
    MatNd* LT     = MatNd_create(n, n);
    MatNd* A_test = MatNd_create(n, n);

    MatNd_setRandom(J, -1.0, 1.0);
    MatNd_sqrMulAtBA(A, J, NULL);

    bool isSymmetric = MatNd_isSymmetric(A, 1.0e-12);
    if (isSymmetric==false)
    {
      REXEC(2)
      {
        MatNd_printCommentDigits("A", A, 6);
        RLOGS(2, "A is not symmetric");
        success = false;
      }
    }

    Timer_setZero();
    double det = MatNd_choleskyDecomposition(L, A);
    double dt = Timer_getTime();
    RCHECK(det);

    // Test A = L LT
    MatNd_transpose(LT, L);
    MatNd_mul(A_test, L, LT);

    double errNorm = MatNd_msqError(A, A_test);
    const double maxErr = 1.0e-12;

    if (errNorm < maxErr)
    {
      RLOGS(2, "SUCCESS for Cholesky decomposition after %.4f msec: error is "
            "%g (<%g)", 1.0e3*dt, errNorm, maxErr);
    }
    else
    {
      success = false;
      RMSGS("FAILURE for Cholesky decompositionafter %.2f msec: error is %g"
            " (>%g)", 1.0e3*dt, errNorm, maxErr);

      REXEC(3)
      {
        MatNd_printCommentDigits("A", A, 5);
        MatNd_printCommentDigits("L", L, 5);
        MatNd_printCommentDigits("L*transpose(L)", A_test, 5);
      }

    }

    MatNd_destroy(J);
    MatNd_destroy(A);
    MatNd_destroy(L);
    MatNd_destroy(LT);
    MatNd_destroy(A_test);
  }

  // MatNd_choleskySolve
  if (testAll || testChol)
  {
    // Create a square symmetric positiv definite matrix A with
    // random values
    RLOGS(2, "**************************************");
    RLOGS(2, "Test for MatNd_choleskySolve");
    RLOGS(2, "**************************************");
    int n = dim;
    int rhsCols = 2;
    argP.getArgument("-rhsDim", &rhsCols,
                     "Number of columns of the right hand side vector");
    MatNd* L = MatNd_create(n, 2 * n);
    MatNd_setRandom(L, -1.0, 1.0);
    MatNd* A = MatNd_create(n, n);
    MatNd_sqrMulABAt(A, L, NULL);
    MatNd_destroy(L);

    // Create a random vector b
    MatNd* b = MatNd_create(n, rhsCols);
    MatNd_setRandom(b, -1.0, 1.0);

    // Solve Ax = b for x
    MatNd* x  = MatNd_create(n, rhsCols);
    Timer_setZero();
    double det = MatNd_choleskySolve(x, A, b);
    double dt = Timer_getTime();
    RCHECK(det);

    // Test: Ax = b
    MatNd* Ax  = MatNd_create(n, rhsCols);
    MatNd* err = MatNd_create(n, rhsCols);
    MatNd_mul(Ax, A, x);
    MatNd_sub(err, Ax, b);

    double errNorm = MatNd_getNorm(err);
    const double maxErr = 1.0e-12;

    if (errNorm < maxErr)
    {
      RLOGS(2, "SUCCESS for Cholesky solve after %.2f msec: error is %g (<%g)",
            1.0e3*dt, errNorm, maxErr);
    }
    else
    {
      success = false;
      RMSGS("FAILURE for Cholesky solve after %.2f msec: error is %g (>%g)",
            1.0e3*dt, errNorm, maxErr);

      REXEC(3)
      {
        MatNd_printCommentDigits("Error", err, 8);
        MatNd_printCommentDigits("Ax", Ax, 8);
        MatNd_printCommentDigits("b", b, 8);
      }
    }

    // Clean up
    MatNd_destroy(A);
    MatNd_destroy(x);
    MatNd_destroy(b);
    MatNd_destroy(Ax);
    MatNd_destroy(err);
  }

  // MatNd_gaussSeidelSolve
  if (testAll || testGS)
  {
    // Create a square symmetric positiv definite matrix A with
    // random values
    RLOGS(2, "**************************************");
    RLOGS(2, "Test for MatNd_gaussSeidelSolve");
    RLOGS(2, "**************************************");
    int n = dim;
    MatNd* L = MatNd_create(n, 2 * n);
    MatNd_setRandom(L, -1.0, 1.0);
    MatNd* A = MatNd_create(n, n);
    MatNd_sqrMulABAt(A, L, NULL);
    MatNd_destroy(L);

    // Create a random vector b
    MatNd* b = MatNd_create(n, 1);
    MatNd_setRandom(b, -1.0, 1.0);

    // Solve Ax = b for x
    MatNd* x  = MatNd_create(n, 1);
    Timer_setZero();
    int nSteps = MatNd_gaussSeidelSolve(x, A, b);
    double dt = Timer_getTime();

    // Test: Ax = b
    MatNd* Ax  = MatNd_create(n, 1);
    MatNd* err = MatNd_create(n, 1);
    MatNd_mul(Ax, A, x);
    MatNd_sub(err, Ax, b);

    RLOGS(2, "Gauss-Seidel took %d steps (%.1f usec)", nSteps, 1.0e6*dt);

    double errNorm = MatNd_getNorm(err);
    const double maxErr = 1.0e-6;

    if (errNorm < maxErr)
    {
      RLOGS(2, "SUCCESS for Gauss-Seidl solve: error is %g (<%g)",
            errNorm, maxErr);
    }
    else
    {
      success = false;
      RMSGS("FAILURE for Gauss-Seidl solve: error is %g (>=%g)",
            errNorm, maxErr);

      REXEC(3)
      {
        MatNd_printCommentDigits("Error", err, 8);
        MatNd_printCommentDigits("Ax", Ax, 8);
        MatNd_printCommentDigits("b", b, 8);
        RMSGS("Error norm is %g", MatNd_getNorm(err));
      }
    }

    // Clean up
    MatNd_destroy(A);
    MatNd_destroy(x);
    MatNd_destroy(b);
    MatNd_destroy(Ax);
    MatNd_destroy(err);
  }

  // MatNd_choleskyInverse:
  // Here's some test code based on inv(A)*A = E:
  if (testAll || testChol)
  {
    RLOGS(2, "**************************************");
    RLOGS(2, "Test for MatNd_choleskyInverse");
    RLOGS(2, "**************************************");
    int n = dim;
    MatNd* sqrtA  = MatNd_create(n, 2 * n);
    MatNd* A      = MatNd_create(n, n);
    MatNd* A_inv  = MatNd_create(n, n);
    MatNd* A_invA = MatNd_create(n, n);

    MatNd_setRandom(sqrtA, -1.0, 1.0);
    MatNd_sqrMulABAt(A, sqrtA, NULL);

    double t0 = Timer_getTime();
    double det = MatNd_choleskyInverse(A_inv, A);
    double t1 = Timer_getTime();
    MatNd_mul(A_invA, A_inv, A);

    if ((RcsLogLevel>2) || ((RcsLogLevel>1) && (dim<10)))
    {
      MatNd_printCommentDigits("A", A, 5);
      MatNd_printCommentDigits("inv(A)", A_inv, 5);
      MatNd_printCommentDigits("A*inv(A) = E", A_invA, 5);
    }

    if (MatNd_isIdentity(A_invA, 1.0e-8))
    {
      RLOGS(2, "SUCCESS for Cholesky inverse: A*inv(A) = I");
    }
    else
    {
      success = false;
      REXEC(3)
      {
        RMSGS("FAILURE for Cholesky inverse: A*inv(A) != I");
        MatNd_printCommentDigits("A", A, 5);
        MatNd_printCommentDigits("inv(A)", A_inv, 5);
        MatNd_printCommentDigits("A*inv(A) = I", A_invA, 5);
      }
    }

    RLOGS(2, "det = %g, inversion took %g usec", det, 1.0e6 * (t1 - t0));

    MatNd_destroy(sqrtA);
    MatNd_destroy(A);
    MatNd_destroy(A_inv);
    MatNd_destroy(A_invA);
  }


  RLOGS(1, "%s", success ? "SUCCESS" : "FAILURE");

  return success;
}

/*******************************************************************************
 * Derivative tests
 ******************************************************************************/
static void test_f(double* f, const double* x, void* none)
{
  *f = 0.0;
  for (int i = 0; i < 5; i++)
  {
    *f += i * x[i] * x[i];
  }
}

static void test_df(double* df, const double* x, void* none)
{
  for (int i = 0; i < 5; i++)
  {
    df[i] = 2.*i * x[i];
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
static void test_LocalRotMat(double* f, const double* x, void* params)
{
  double rm[3][3];
  int* index = (int*) params;
  Mat3d_setElementaryRotation(rm, *index, x[0]);
  memcpy(f, rm, 9 * sizeof(double));
}

static void test_dLocalRotMat(double* f, const double* x, void* params)
{
  double rm[3][3];
  int* index = (int*) params;
  Mat3d_dAdq(rm, *index, x[0]);
  memcpy(f, rm, 9 * sizeof(double));
}

/*******************************************************************************
 * Rotation matrix derivative wrt. rotation angle test:
 *       A3(a,b)       = A1(a)*A2(b)
 *        d(A3)/d(a,b)  = dA1(a)*A2(b) + A1(a)*dA2(b)
 ******************************************************************************/
static void test_LocalRotMatRel(double* f, const double* x, void* params)
{
  double rm1[3][3], rm2[3][3];

  MatNd A1 = MatNd_fromPtr(3, 3, (double*) rm1);
  MatNd A2 = MatNd_fromPtr(3, 3, (double*) rm2);
  MatNd A3 = MatNd_fromPtr(3, 3, f);

  Mat3d_setRotMatX(rm1, x[0]);
  Mat3d_setRotMatY(rm2, x[0]);

  MatNd_mul(&A3, &A1, &A2);
}

static void test_dLocalRotMatRel(double* f, const double* x, void* params)
{
  HTr rm1, drm1, rm2, drm2;
  double buf1[9], buf2[9];

  MatNd A1 = MatNd_fromPtr(3, 3, (double*) rm1.rot);
  MatNd A2 = MatNd_fromPtr(3, 3, (double*) rm2.rot);
  MatNd dA1 = MatNd_fromPtr(3, 3, (double*) drm1.rot);
  MatNd dA2 = MatNd_fromPtr(3, 3, (double*) drm2.rot);
  MatNd term1 = MatNd_fromPtr(3, 3, buf1);
  MatNd term2 = MatNd_fromPtr(3, 3, buf2);
  MatNd res = MatNd_fromPtr(3, 3, f);

  Mat3d_setRotMatX(rm1.rot, x[0]);
  Mat3d_dAdq(drm1.rot, 0, x[0]);

  Mat3d_setRotMatY(rm2.rot, x[0]);
  Mat3d_dAdq(drm2.rot, 1, x[0]);

  MatNd_mul(&term1, &dA1, &A2);
  MatNd_mul(&term2, &A1, &dA2);
  MatNd_add(&res, &term1, &term2);
}

/*******************************************************************************
 * Axis angle partial derivative wrt. Euler angles.
 ******************************************************************************/
static void test_AxisAngle(double* f, const double* x, void* params)
{
  double A_KI[3][3];

  Mat3d_fromEulerAngles(A_KI, x);
  *f = Mat3d_diffAngleSelf(A_KI);
}

static void test_dAxisAngleDEuler(double* f, const double* x, void* params)
{
  Mat3d_dDiffAngleDEuler(f, x);
}

/*******************************************************************************
 * Gradient test function.
 ******************************************************************************/
bool testDerivatives(int argc, char** argv)
{
  // Parse command line arguments
  int iter = 10;
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-iter", &iter, "Number of iterations");

  bool success = true;
  bool verbose = (RcsLogLevel>1) ? true : false;
  double eps = 1.0e-2;
  argP.getArgument("-eps", &eps, "Permissable gradient error (Default is"
                   " 0.01, which is 1 percent)");

  RLOG(2, "Testing gradients");

  MatNd* x = MatNd_create(3, 3);
  for (int i = 0; i < iter; i++)
  {
    // Random state vector
    MatNd_setRandom(x, -1.0, 1.0);

    // Random number between 0 and 2
    int rndXYZ = (int)(3.0 * (rand() / (RAND_MAX + 1.0)));
    RCHECK((rndXYZ >= 0) && (rndXYZ < 3));

    REXEC(2)
    {
      fprintf(stderr, "Test:                    ");
    }
    success = success && Rcs_testGradient(test_f, test_df, NULL, x->ele,
                                          5, 1, eps, verbose);

    REXEC(2)
    {
      fprintf(stderr, "Local Rotmat gradient:   ");
    }
    success = success && Rcs_testGradient(test_LocalRotMat, test_dLocalRotMat,
                                          &rndXYZ, x->ele, 1, 9, eps, verbose);

    REXEC(2)
    {
      fprintf(stderr, "Local Rotmat grad. (rel):");
    }
    success = success && Rcs_testGradient(test_LocalRotMatRel,
                                          test_dLocalRotMatRel,
                                          NULL, x->ele, 1, 9, eps, verbose);

    REXEC(2)
    {
      fprintf(stderr, "Axis angle wrt. Eul. ang:");
    }
    success = success && Rcs_testGradient(test_AxisAngle,
                                          test_dAxisAngleDEuler,
                                          NULL, x->ele, 3, 1, eps, verbose);

    REXEC(2)
    {
      if (success==false)
      {
        RPAUSE_MSG("FAILURE");
      }
      else
      {
        RMSGS("All gradient tests succeeded\n");
      }
    }

    //Timer_usleep(1000);

  }   // while(true)

  MatNd_destroy(x);

  RLOGS(1, "%s", success ? "SUCCESS" : "FAILURE");

  return success;
}

/*******************************************************************************
 * HTr transform & invTransform testing
 ******************************************************************************/
bool testHTr(int argc, char** argv)
{
  // create & set
  HTr* A_1I = HTr_create();
  HTr_setIdentity(A_1I);
  Vec3d_set(A_1I->org, 1.0, 0.0, 0.0);
  Mat3d_setRotMatZ(A_1I->rot, -45.0 * (M_PI / 180.));

  HTr* A_2I = HTr_create();
  HTr_setIdentity(A_2I);
  Vec3d_set(A_2I->org, 0.0, 1.0, 0.0);
  Mat3d_setRotMatZ(A_2I->rot, -30.0 * (M_PI / 180.));

  HTr* A_21 = HTr_create();
  HTr_setIdentity(A_21);
  Vec3d_set(A_21->org, -1.0 / 0.707107, 0.0, 0.0);
  Mat3d_setRotMatZ(A_21->rot, 15.0 * (M_PI / 180.));

  fprintf(stderr, "\nA1I:\n");
  HTr_fprint(stderr, A_1I);
  fprintf(stderr, "\nA2I:\n");
  HTr_fprint(stderr, A_2I);
  fprintf(stderr, "\nA21:\n");
  HTr_fprint(stderr, A_21);
  fprintf(stderr, "\n");

  // transform
  HTr_transform(A_2I, A_1I, A_21);
  fprintf(stderr, "\nTrans A_2I:\n");
  HTr_fprint(stderr, A_2I);

  // inv_transform
  HTr_invTransform(A_21, A_1I, A_2I);
  fprintf(stderr, "\nInvTrans A_21:\n");
  HTr_fprint(stderr, A_21);

  // transform
  HTr_transform(A_2I, A_1I, A_21);
  fprintf(stderr, "\nTrans A_2I:\n");
  HTr_fprint(stderr, A_2I);

  // cleanup
  RFREE(A_1I);
  RFREE(A_2I);
  RFREE(A_21);

  return true;
}

/*******************************************************************************
 * Basic math functions test.
 ******************************************************************************/
bool testBasicMath(int argc, char** argv)
{
  printf ( "lround (2.3) = %ld\n", lround(2.3));
  printf ( "lround (3.8) = %ld\n", lround(3.8));
  printf ( "lround (-2.3) = %ld\n", lround(-2.3));
  printf ( "lround (-3.8) = %ld\n", lround(-3.8));

  MatNd* sqrMat  = MatNd_create(3, 6);
  MatNd* Mat     = MatNd_create(3, 3);
  MatNd_setRandom(sqrMat, -1.0, 1.0);
  MatNd_sqrMulABAt(Mat, sqrMat, NULL);

  double A[3][3], invA[3][3], invASelf[3][3], Identity[3][3];

  VecNd_copy((double*) A, sqrMat->ele, 9);
  VecNd_copy((double*) invASelf, sqrMat->ele, 9);

  RMSGS("A:");
  Mat3d_fprint(stderr, A);

  Mat3d_inverse(invA, A);
  RMSGS("inv(A) after Mat3d_inverse():");
  Mat3d_fprint(stderr, invA);

  RMSGS("Test: A*inv(A) = I");
  Mat3d_mul(Identity, A, invA);
  Mat3d_fprint(stderr, Identity);

  Mat3d_inverseSelf(invASelf);
  RMSGS("inv(A) after Mat3d_inverseSelf():");
  Mat3d_fprint(stderr, invASelf);

  RMSGS("Test: A*inv(A) = I");
  Mat3d_mul(Identity, A, invASelf);
  Mat3d_fprint(stderr, Identity);

  MatNd_destroy(sqrMat);
  MatNd_destroy(Mat);

  // Test MatNd_scaleSelf
  const int dim = 10;
  MatNd* B = MatNd_create(dim, 1);
  MatNd* limit = MatNd_create(dim,1);

  for (int i=0; i<dim; i++)
  {
    MatNd_set(B, i, 0, i);
    MatNd_set(limit, i, 0, 0.5);
  }

  MatNd_printCommentDigits("limit", limit, 6);
  MatNd_printCommentDigits("B before scaling", B, 6);
  double scale = MatNd_scaleSelf(B, limit);
  MatNd_printCommentDigits("B after scaling", B, 6);
  RMSGS("scale factor is %f", scale);

  // Test VecNd sorting
  double vec_src[dim], vec_dst[dim];
  VecNd_setRandom(vec_src, -100.0, 100.0, dim);
  VecNd_sort(vec_dst, vec_src, dim);
  VecNd_printComment("Vector before sorting", vec_src, dim);
  VecNd_printComment("Vector after sorting", vec_dst, dim);

  // Test VecNd reverting
  for (int i=0; i<dim; ++i)
  {
    vec_src[i] = i;
    vec_dst[i] = i;
  }
  VecNd_reverseSelf(vec_dst, dim);
  VecNd_printComment("Vector before reverting", vec_src, dim);
  VecNd_printComment("Vector after reverting", vec_dst, dim);

  // Test cylinder coordinates conversion
  double radialDist, azimuth, height, p[3], p2[3];
  Vec3d_setRandom(p, -1.0, 1.0);

  Math_Cart2Cyl(p, &radialDist, &azimuth, &height);
  Math_Cyl2Cart(radialDist, azimuth, height, p2);
  RLOG(0, "Cylinder coordinates conversion: %f %f   %f %f   err=%f",
       p[0], p2[0], p[1], p2[1], VecNd_sqrDiff(p, p2, 2));

  // cleanup
  MatNd_destroy(B);
  MatNd_destroy(limit);

  return true;
}

/*******************************************************************************
 * Curve fitting tests.
 ******************************************************************************/
bool testCurveFitting(int argc, char** argv)
{
  RMSGS("Testing linear fit");
  Rcs::CmdLineParser argP(argc, argv);
  int n = 100;
  argP.getArgument("-n", &n);

  MatNd* data = MatNd_create(n, 2);
  MatNd_setRandom(data, -1.0, 1.0);
  for (unsigned int i=0; i<data->m; i++)
  {
    MatNd_set(data, i, 0, i);
    MatNd_set(data, i, 1, 20.0 +
              Math_getRandomNumber(-1.0, 1.0) + 10.0*sin(i/50.0));
  }
  // MatNd_printCommentDigits("data", data, 6);

  double A, B;
  MatNd_lineFit2D(&A, &B, data);

  double A2, B2, C2;
  MatNd_parabolicFit2D(&A2, &B2, &C2, data);

  RMSGS("A=%f   B=%f", A, B);
  RMSGS("A2=%f   B2=%f   C2=%f", A2, B2, C2);
  MatNd_toFile(data, "data.dat");

  MatNd* fitLine = MatNd_create(n,4);
  for (int i=0; i<n; i++)
  {
    MatNd_set(fitLine, i, 0, MatNd_get(data, i, 0));
    MatNd_set(fitLine, i, 1, MatNd_get(data, i, 1));
    MatNd_set(fitLine, i, 2, A*i+B);
    MatNd_set(fitLine, i, 3, A2*i*i+B2*i+C2);
  }
  MatNd_toFile(fitLine, "/tmp/fitLine.dat");

  RMSGS("To show the results: Start gnuplot and type:\n"
        "plot \"/tmp/fitLine.dat\" u 1:2 title \"data\" w lp,"
        " \"/tmp/fitLine.dat\" u 1:3 title \"line fit\" w l, "
        "\"/tmp/fitLine.dat\" u 1:4 title \"parabolic fit\"  w l\n");

  MatNd_destroy(data);
  MatNd_destroy(fitLine);

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool testVectorProjection(int argc, char** argv)
{
  // Parse command line arguments
  int nq = 5;
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-dim", &nq, "Set default dimansionality");

  MatNd* a = MatNd_create(nq, 1);
  MatNd* b = MatNd_create(nq, 1);
  MatNd* c = MatNd_create(nq, 1);

  MatNd_setRandom(a, -1.0, 1.0);
  MatNd_setRandom(b, -1.0, 1.0);

  MatNd_printCommentDigits("a", a, 6);
  MatNd_printCommentDigits("b", b, 6);
  RMSGS("diff-angle is %f", (180./M_PI)*MatNd_diffAngle(a,b));

  MatNd_vectorProjection(c, a, b);
  MatNd_printCommentDigits("c", c, 6);
  RMSGS("diff-angle c to a is %f", (180./M_PI)*MatNd_diffAngle(c,a));
  RMSGS("diff-angle c to b is %f", (180./M_PI)*MatNd_diffAngle(c,b));

  MatNd_destroy(a);
  MatNd_destroy(b);
  MatNd_destroy(c);

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
static bool testGramSchmidt(double M[3][3])
{
  RMSGS("\n\n\nPertubed matrix:");
  Mat3d_fprint(stdout, M);

  RMSGS("Angle x-y: %f", Vec3d_diffAngle(M[0], M[1])*180.0/M_PI);
  RMSGS("Angle x-z: %f", Vec3d_diffAngle(M[0], M[2])*180.0/M_PI);
  RMSGS("Angle y-z: %f", Vec3d_diffAngle(M[1], M[2])*180.0/M_PI);

  RMSGS("Length x: %f", Vec3d_getLength(M[0]));
  RMSGS("Length y: %f", Vec3d_getLength(M[1]));
  RMSGS("Length z: %f", Vec3d_getLength(M[2]));

  RMSGS("\n\n\nNormalized matrix:");
  Timer_setZero();
  bool success = Mat3d_orthonormalizeSelf(M);
  double dt = Timer_getTime();

  if (success==false)
  {
    RMSGS("\n\n\n***** Failed to orthonormalize matrix !!!\n\n\n");
  }

  Mat3d_fprint(stdout, M);

  double det = Mat3d_determinant(M);
  RMSGS("Took %f usec, determinant: %g", dt*1.0e6, det);

  RMSGS("Angle x-y: %f", Vec3d_diffAngle(M[0], M[1])*180.0/M_PI);
  RMSGS("Angle x-z: %f", Vec3d_diffAngle(M[0], M[2])*180.0/M_PI);
  RMSGS("Angle y-z: %f", Vec3d_diffAngle(M[1], M[2])*180.0/M_PI);

  RMSGS("Length x: %f", Vec3d_getLength(M[0]));
  RMSGS("Length y: %f", Vec3d_getLength(M[1]));
  RMSGS("Length z: %f", Vec3d_getLength(M[2]));

  return success;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool testOrthogonalization3x3(int argc, char** argv)
{
  bool success = true;

  double M1[3][3], M2[3][3];
  Mat3d_setIdentity(M1);
  Mat3d_addRandom(M1, -0.1, 0.1);
  Mat3d_copy(M2, M1);

  if (testGramSchmidt(M1) == false)
  {
    success = false;
  }

  return success;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool testFiniteNan()
{
  double mat[3][3], vec[3];
  bool isFin, success = true;

#if !defined(_MSC_VER)
  double infVal = 3.0/0.0;
#else
  double infVal = std::numeric_limits<double>::infinity();
#endif

  RLOG(1, "Vec3d functions");

  Vec3d_set(vec, 1.0, 2.0, 3.0);
  isFin = Vec3d_isFinite(vec);

  if (isFin==false)
  {
    success = false;
  }

  RLOG(1, "Vector: %.1f   %.1f   %.1f\tisfinite: %s",
       vec[0], vec[1], vec[2], isFin==true?"true":"false");

  Vec3d_set(vec, 1.0, 2.0, infVal);
  isFin = Vec3d_isFinite(vec);

  if (isFin==true)
  {
    success = false;
  }

  RLOG(1, "Vector: %.1f   %.1f   %.1f\tisfinite: %s",
       vec[0], vec[1], vec[2], isFin==true?"true":"false");




  RLOG(1, "VecNd functions");

  Vec3d_set(vec, 1.0, 2.0, 3.0);
  isFin = Vec3d_isFinite(vec);

  if (isFin==false)
  {
    success = false;
  }

  RLOG(1, "Vector: %.1f   %.1f   %.1f\tisfinite: %s",
       vec[0], vec[1], vec[2], isFin==true?"true":"false");

  Vec3d_set(vec, 1.0, 2.0, infVal);
  isFin = VecNd_isFinite(vec, 3);

  if (isFin==true)
  {
    success = false;
  }

  RLOG(1, "Vector: %.1f   %.1f   %.1f\tisfinite: %s",
       vec[0], vec[1], vec[2], isFin==true?"true":"false");




  RMSGS("Mat3d functions");
  Mat3d_setIdentity(mat);

  isFin = Mat3d_isFinite(mat);

  if (isFin==false)
  {
    success = false;
  }

  REXEC(1)
  {
    Mat3d_fprint(stderr, mat);
    RMSGS("Matrix - isfinite: %s", isFin==true?"true":"false");
  }

  mat[0][2] = infVal;

  isFin = Mat3d_isFinite(mat);

  if (isFin==true)
  {
    success = false;
  }

  REXEC(1)
  {
    Mat3d_fprint(stderr, mat);
    RMSGS("Matrix - isfinite: %s", isFin==true?"true":"false");
  }

  return success;
}

/*******************************************************************************
 * Test of the Woodbury Identity for positive definite matrices lambda
 * and Wx. See matrixcookook.pdf for details.
 ******************************************************************************/
bool testWoodburyIdenity(int argc, char** argv)
{
  RMSGS("\n\n**************************************************************"
        "\n  Test for Woodbury Matrix Idenity\n"
        "  Options:\n"
        "    -rows   <number of Jacobian rows>\n"
        "    -cols   <number of Jacobian columns>\n"
        "    -digits <Digits after the dot in the matrix console output>\n"
        "**************************************************************\n");

  int m = 4, n = 20, digits = 6;
  unsigned int i;

  // Parse command line arguments
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-rows", &m);
  argP.getArgument("-cols", &n);
  argP.getArgument("-digits", &digits);

  MatNd* J  = MatNd_create(m, n);
  MatNd* Jt = MatNd_create(n, m);
  MatNd* lambda = MatNd_create(n, 1);
  MatNd* invLambda = MatNd_create(n, 1);
  MatNd* Wx = MatNd_create(m, 1);
  MatNd* invWx = MatNd_create(m, 1);

  MatNd_setRandom(J, -1.0, 1.0);
  MatNd_transpose(Jt, J);

  MatNd_setRandom(lambda, 1.0e-8, 1.0e-6);
  for (i=0; i<lambda->size; i++)
  {
    invLambda->ele[i] = 1.0/lambda->ele[i];
  }

  MatNd_setRandom(Wx, 0.4, 0.6);
  for (i=0; i<Wx->size; i++)
  {
    invWx->ele[i] = 1.0/Wx->ele[i];
  }

  MatNd* pinvJ1 = MatNd_create(n, m);
  double det1 = MatNd_rwPinv(pinvJ1, J, invLambda, invWx);

  MatNd* pinvJ2 = MatNd_create(n, m);
  double det2 = MatNd_rwPinv2(pinvJ2, J, Wx, lambda);

  RMSGS("\n\nJ1# = invLambda J^T (J invLambda J^T + invWx)^-1\t"
        "J2# = (J^T Wx J + lambda)^-1 J^T Wx\t diff\n");
  MatNd_printTwoArraysDiff(pinvJ1, pinvJ2, digits);
  RMSGS("det1 = %g   det2 = %g", det1, det2);

  MatNd_destroy(J);
  MatNd_destroy(Jt);
  MatNd_destroy(lambda);
  MatNd_destroy(invLambda);
  MatNd_destroy(Wx);
  MatNd_destroy(invWx);
  MatNd_destroy(pinvJ1);
  MatNd_destroy(pinvJ2);

  return true;
}

/*******************************************************************************
 * Outputs random integers and doubles.
 ******************************************************************************/
bool testRnd(int argc, char** argv)
{
  int iLower = -10, iUpper = 10;

  RMSGS("\n\n****************************************************************"
        "\n  Test for random number functions\n"
        "  Options:\n"
        "    -lower   <lower limit=%d>\n"
        "    -upper   <upper limit=%d>\n"
        "****************************************************************\n",
        iLower, iUpper);

  // Parse command line arguments
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-lower", &iLower);
  argP.getArgument("-upper", &iUpper);

  fprintf(stderr, "Lower integer value: %d\n", iLower);
  fprintf(stderr, "Upper integer value: %d\n\n", iUpper);

  for (int i=0; i<10; i++)
  {
    fprintf(stderr, "Random integer[%d] = %d\n",
            i, Math_getRandomInteger(iLower, iUpper));
  }

  fprintf(stderr, "\n");

  for (int i=0; i<10; i++)
  {
    fprintf(stderr, "Random double[%d] = %.12f\n",
            i, Math_getRandomNumber(iLower, iUpper));
  }

  fprintf(stderr, "\n");

  return true;
}

/*******************************************************************************
 * Axis angle test: Tests conversion from and to axis angle representation
 ******************************************************************************/
bool testAxisAngleConversion(int argc, char** argv)
{
  bool success = true;

  int its = 100;
  double err, ang, ax[3], A1[3][3], A2[3][3];
  double maxErr = 0.0;
  double errLimit = 1.0e-8;

  // Parse command line arguments
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-its", &its, "Number of iterations (default is 100)");
  argP.getArgument("-errLimit", &errLimit,
                   "Max. error to pass test (default is 1.0e-8)");

  if (argP.hasArgument("-h"))
  {
    argP.print();
    return true;
  }



  // ==============================================================
  // Test 1: FromAxisAngle(AxisAngle(A)) = A
  // ==============================================================
  for (int i=0; i<its; i++)
  {
    Mat3d_setRandomRotation(A1);
    ang = Mat3d_getAxisAngleSelf(ax, A1);
    Mat3d_fromAxisAngle(A2, ax, ang);
    Mat3d_subSelf(A2, A1);

    err = VecNd_getLength((double*) A2, 9);

    if (err>maxErr)
    {
      maxErr = err;
    }

    if (err > errLimit)
    {
      RLOG(1, "Err: %g", err);
      Mat3d_printCommentDigits("A1", A1, 5);
      Mat3d_printCommentDigits("A2", A2, 5);
      Mat3d_printCommentDigits("Diff", A2, 5);
      success = false;
    }
    else
    {
      RLOG(1, "Axis: %.2f %.2f %.2f angle: %.1f err: %g",
           ax[0], ax[1], ax[2], ang*180.0/M_PI, err);
    }

  }

  RMSGS("Test 1: %s for %d axis angle tests: Max. error = %g < %g",
        maxErr>errLimit ? "FAILURE" : "SUCCESS", its, maxErr, errLimit);



  // ==============================================================
  // Test 2: A = rotateAxisAngle(Identity, AxisAngle(A))
  // ==============================================================
  for (int i=0; i<its; i++)
  {
    Mat3d_setRandomRotation(A1);
    ang = Mat3d_getAxisAngleSelf(ax, A1);

    Mat3d_setIdentity(A2);
    Mat3d_rotateAxisAngleSelf(A2, ax, ang);
    Mat3d_subSelf(A2, A1);

    err = VecNd_getLength((double*) A2, 9);

    if (err>maxErr)
    {
      maxErr = err;
    }

    if (err > errLimit)
    {
      RLOG(1, "Err: %g", err);
      Mat3d_printCommentDigits("A1", A1, 5);
      Mat3d_printCommentDigits("A2", A2, 5);
      Mat3d_printCommentDigits("Diff", A2, 5);
      success = false;
    }
    else
    {
      RLOG(1, "Axis: %.2f %.2f %.2f angle: %.1f err: %g",
           ax[0], ax[1], ax[2], ang*180.0/M_PI, err);
    }

  }

  RMSGS("Test 2: %s for %d axis angle tests: Max. error = %g < %g",
        maxErr>errLimit ? "FAILURE" : "SUCCESS", its, maxErr, errLimit);



  // ==============================================================
  // Test 3: Test fat and rotate axis angle functions
  // ==============================================================
  for (int i=0; i<its; i++)
  {
    Mat3d_setRandomRotation(A1);
    Mat3d_setRandomRotation(A2);
    ang = Mat3d_getAxisAngle(ax, A2, A1);

    double A2_test[3][3];
    Mat3d_rotateAxisAngle(A2_test, ax, ang, A1);
    Mat3d_subSelf(A2, A2_test);

    err = VecNd_getLength((double*) A2, 9);

    if (err>maxErr)
    {
      maxErr = err;
    }

    if (err > errLimit)
    {
      RLOG(1, "Err: %g", err);
      Mat3d_printCommentDigits("A1", A1, 5);
      Mat3d_printCommentDigits("A2", A2, 5);
      Mat3d_printCommentDigits("Diff", A2, 5);
      success = false;
    }
    else
    {
      RLOG(1, "Axis: %.2f %.2f %.2f angle: %.1f err: %g",
           ax[0], ax[1], ax[2], ang*180.0/M_PI, err);
    }

  }

  RMSGS("Test 3: %s for %d axis angle tests: Max. error = %g < %g",
        maxErr>errLimit ? "FAILURE" : "SUCCESS", its, maxErr, errLimit);



  // ==============================================================
  // Test 4: Compare axis angle and omega rotation functions
  // ==============================================================
  for (int i=0; i<its; i++)
  {
    Mat3d_setRandomRotation(A1);
    Mat3d_setRandomRotation(A2);
    ang = Mat3d_getAxisAngle(ax, A2, A1);

    double A2_test[3][3];
    Mat3d_rotateAxisAngle(A2_test, ax, ang, A1);

    // Omega test
    double omega[3], A2_test2[3][3];
    Mat3d_copy(A2_test2, A1);
    Vec3d_constMul(omega, ax, ang);
    Mat3d_rotateOmegaSelf(A2_test2, omega, true);

    Mat3d_sub(A2, A2_test, A2_test2);

    err = VecNd_getLength((double*) A2, 9);

    if (err>maxErr)
    {
      maxErr = err;
    }

    if (err > errLimit)
    {
      RLOG(1, "Err: %g", err);
      Mat3d_printCommentDigits("A1", A1, 5);
      Mat3d_printCommentDigits("A2", A2, 5);
      Mat3d_printCommentDigits("Diff", A2, 5);
      success = false;
    }
    else
    {
      RLOG(1, "Axis: %.2f %.2f %.2f angle: %.1f err: %g",
           ax[0], ax[1], ax[2], ang*180.0/M_PI, err);
    }

  }

  RMSGS("Test 4: %s for %d axis angle tests: Max. error = %g < %g",
        maxErr>errLimit ? "FAILURE" : "SUCCESS", its, maxErr, errLimit);



  // ==============================================================
  // Test 5: Compare Slerp and axis angle rotation functions
  // ==============================================================
  for (int i=0; i<its; i++)
  {
    const double t_slerp = 0.25;

    Mat3d_setRandomRotation(A1);
    Mat3d_setRandomRotation(A2);
    ang = Mat3d_getAxisAngle(ax, A2, A1);

    double A2_test[3][3];
    Mat3d_rotateAxisAngle(A2_test, ax, ang, A1);

    // Rotate about axis angle
    double omega[3], A3[3][3];
    Mat3d_copy(A3, A1);
    Vec3d_constMul(omega, ax, t_slerp*ang);
    Mat3d_rotateOmegaSelf(A3, omega, true);

    // Slerp
    double A4[3][3];
    Mat3d_slerp(A4, A1, A2, t_slerp);


    Mat3d_sub(A2, A4, A3);

    err = VecNd_getLength((double*) A2, 9);

    if (err>maxErr)
    {
      maxErr = err;
    }

    if (err > errLimit)
    {
      RLOG(1, "Err: %g", err);
      Mat3d_printCommentDigits("A1", A1, 5);
      Mat3d_printCommentDigits("A2", A2, 5);
      Mat3d_printCommentDigits("Diff", A2, 5);
      success = false;
    }
    else
    {
      RLOG(1, "Axis: %.2f %.2f %.2f angle: %.1f err: %g",
           ax[0], ax[1], ax[2], ang*180.0/M_PI, err);
    }

  }

  RMSGS("Test 5: %s for %d axis angle tests: Max. error = %g < %g",
        maxErr>errLimit ? "FAILURE" : "SUCCESS", its, maxErr, errLimit);



  return success;
}

/*******************************************************************************
 * Axis angle test: Distribution of axis angle error on trajectory
 ******************************************************************************/
bool testAxisAngleInterpolation(int argc, char** argv)
{
  const int maxT = 100;
  int T = 10;

  // Parse command line arguments
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-T", &T, "Number of time steps");
  T = Math_iClip(T, 0, maxT);

  if (argP.hasArgument("-h"))
  {
    argP.print();
    return true;
  }

  bool success = true;
  double A[maxT+1][3][3], A_des[3][3];
  double om[maxT][3], phi_des[3];
  double angle, angle_i, axis[3], deltaOm[3], deltaOm_i[3], deltaOm_temp[3];

  // Set the desired rotation
  Vec3d_setRandom(phi_des, -10.0*M_PI, 10.0*M_PI);
  Mat3d_fromEulerAngles(A_des, phi_des);

  // Initialize the initial policy
  for (int i=0; i<T; i++)
  {
    Vec3d_setRandom(om[i], -10.0*M_PI/180.0, 10.0*M_PI/180.0);
  }

  // Initialize the kinematics
  Mat3d_setIdentity(A[0]);
  for (int i=1; i<=T; i++)
  {
    Mat3d_copy(A[i], A[i-1]);
    Mat3d_rotateOmegaSelf(A[i], om[i-1], true);
  }

  // Compute the axis angle error between desired and last rotation
  angle = Mat3d_getAxisAngle(axis, A_des, A[T]);
  angle_i = angle/((double)T);
  Vec3d_constMul(deltaOm, axis, angle);
  Vec3d_constMul(deltaOm_i, axis, angle_i);

  RLOG(2, "Axis = %f   %f   %f   Angle=%f [deg]",
       axis[0], axis[1], axis[2], angle*180.0/M_PI);
  RLOG(3, "dOmega   = %f  %f  %f", deltaOm[0], deltaOm[1], deltaOm[2]);
  RLOG(3, "dOmega_i = %f  %f  %f", deltaOm_i[0], deltaOm_i[1], deltaOm_i[2]);

  //  // Distribute it on each policy step
  //  for(int i=0;i<T;i++)
  //    {
  //      Vec3d_addSelf(om[i], deltaOm_i);
  //    }


  // Compute the kinematics
  Mat3d_setIdentity(A[0]);
  for (int i=1; i<=T; i++)
  {
    Mat3d_copy(A[i], A[i-1]);
    //      Mat3d_rotateOmegaSelf(A[i], om[i-1], true);
    //      Mat3d_rotateOmegaSelf(A[i], deltaOm_i, true);

    Vec3d_constMul(deltaOm_temp, deltaOm, -(i-1)/((double)T));
    Mat3d_rotateOmegaSelf(A[i], deltaOm_temp, true);

    Mat3d_rotateOmegaSelf(A[i], om[i-1], true);

    Vec3d_constMul(deltaOm_temp, deltaOm, i/((double)T));
    Mat3d_rotateOmegaSelf(A[i], deltaOm_temp, true);
  }



  // Calculate error
  double A_diff[3][3];
  Mat3d_sub(A_diff, A_des, A[T]);

  double errNorm = VecNd_getLength((double*) A_diff, 9);

  if (errNorm > 1.0e-8)
  {
    Mat3d_printCommentDigits("A_des", A_des, 4);
    Mat3d_printCommentDigits("A_last", A[T], 4);
    Mat3d_printCommentDigits("A_diff", A_diff, 8);
    RFATAL("Error norm is %g", errNorm);
  }
  else
  {
    RLOG(1, "Success for angles [%.1f %.1f %.1f] deg! Error norm is %g",
         phi_des[0]*180.0/M_PI, phi_des[1]*180.0/M_PI,
         phi_des[2]*180.0/M_PI, errNorm);
  }


  return success;
}

/*******************************************************************************
 * Test of the Miller matrix inversion for a sum of symmetric matrices.
 *
 * See: Kenneth S. Miller: On the Inverse of the Sum of Matrices,
 *      Mathematics Magazine, Vol. 54, No. 2 (Mar., 1981), pp. 67-72
 *
 ******************************************************************************/
bool testMillerInversion(int argc, char** argv)
{
  RMSGS("\n\n**************************************************************\n"
        "  Test for Miller Matrix inversion\n"
        "  Options:\n"
        "    -rows   <number of Jacobian rows>\n"
        "    -cols   <number of Jacobian columns>\n"
        "    -lambda <Ridge factor>\n"
        "    -digits <Digits after the dot in the matrix console output>\n"
        "**************************************************************\n");

  int m = 4, n = 20, digits = 6;
  const double errMax = 1.0e-8;
  double lambda0 = 1.0e-8;
  bool success = true;

  // Parse command line arguments
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-rows", &m);
  argP.getArgument("-cols", &n);
  argP.getArgument("-digits", &digits);
  argP.getArgument("-lambda", &lambda0);

  MatNd* J  = MatNd_create(m, n);
  MatNd* lambda = MatNd_create(n, 1);
  MatNd* Wx = MatNd_create(m, 1);

  MatNd_setRandom(J, -1.0, 1.0);
  MatNd_setElementsTo(lambda, lambda0);
  MatNd_setRandom(Wx, 0.4, 0.6);

  MatNd* pinvJ1 = MatNd_create(n, m);
  double dt1 = Timer_getTime();
  MatNd_MillerPinv(pinvJ1, J, Wx, lambda);
  dt1 = Timer_getTime() - dt1;

  MatNd* pinvJ2 = MatNd_create(n, m);
  double dt2 = Timer_getTime();
  MatNd_rwPinv2(pinvJ2, J, Wx, lambda);
  dt2 = Timer_getTime() - dt2;

  double err = MatNd_sqrDistance(pinvJ1, pinvJ2);

  RMSGS("\n\nJ1# = invLambda J^T (J invLambda J^T + invWx)^-1\t"
        "J2# = (J^T Wx J + lambda)^-1 J^T Wx\t diff\n");
  MatNd_printTwoArraysDiff(pinvJ1, pinvJ2, digits);
  printf("Miller inversion took %.3f usec, Cholesky inversion took %.3f "
         "usec\n", 1.0e6*dt1, 1.0e6*dt2);
  printf("%s: Error is %g\n", err < errMax ? "SUCCESS" : "FAILURE", err);


  if (err>errMax)
  {
    success = false;
  }

  MatNd_destroy(J);
  MatNd_destroy(lambda);
  MatNd_destroy(Wx);
  MatNd_destroy(pinvJ1);
  MatNd_destroy(pinvJ2);

  return success;
}

/*******************************************************************************
 * Test of interpolation algorithms.
 ******************************************************************************/
bool testInterpolation(int argc, char** argv)
{
  RMSGS("\n\n**************************************************************\n"
        "  Test for data interpolation. The time index is the row index.\n"
        "  Options:\n"
        "    -subSteps   <number of subdivision steps>\n"
        "    -rows       <number of samples>\n"
        "**************************************************************\n");

  // Parse command line arguments
  int subSteps = 3, rows = 10;
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-subSteps", &subSteps);
  argP.getArgument("-rows", &rows);

  RCHECK_MSG((subSteps>0) && (subSteps<rows), "subSteps: %d   rows: %d",
             subSteps, rows);

  MatNd* src = MatNd_create(rows,3);
  MatNd* interpolateLin = MatNd_create(subSteps*src->m,3);
  MatNd* interpolatePoly = MatNd_create(subSteps*src->m,3);
  MatNd_setRandom(src, 0.0, 1.0);
  MatNd_interpolateFifthOrder(interpolatePoly, src, subSteps);
  MatNd_interpolateLinear(interpolateLin, src, subSteps);

  MatNd* compare = MatNd_create(interpolateLin->m, 4);

  for (unsigned int i=0; i<compare->m; i++)
  {
    MatNd_set(compare, i, 0, i);
    MatNd_set(compare, i, 1, MatNd_get(src, i/subSteps, 0));
    MatNd_set(compare, i, 2, MatNd_get(interpolateLin, i, 0));
    MatNd_set(compare, i, 3, MatNd_get(interpolatePoly, i, 0));
  }

  MatNd_toFile(compare, "/tmp/out.dat");

  const char* gpCmd = "set grid\nplot \"/tmp/out.dat\" u 1:2 w lp title "
                      "\"raw\", \"/tmp/out.dat\" u 1:3 w lp title \"linear\""
                      ", \"/tmp/out.dat\" u 1:4 w "
                      "lp title \"5th order polynomial\"\n";
  FILE* outDat = fopen("/tmp/postpro.gnu", "w+");
  RCHECK(outDat);
  fprintf(outDat, gpCmd);
  fflush(outDat);
  fclose(outDat);

  int err = system("/usr/bin/gnuplot -persist /tmp/postpro.gnu");

  if (err == -1)
  {
    RMSGS("Couldn't start gnuplot");
  }
  else
  {
    RMSGS("/usr/bin/gnuplot -persist /tmp/postpro.gnu");
  }

  MatNd_destroy(compare);
  MatNd_destroy(src);
  MatNd_destroy(interpolateLin);
  MatNd_destroy(interpolatePoly);

  return true;
}

/*******************************************************************************
 * Test of interpolation algorithms.
 ******************************************************************************/
bool testMovingMeanFilter(int argc, char** argv)
{
  RMSGS("\n\n**************************************************************\n"
        "  Test for data interpolation. The time index is the row index.\n"
        "  Options:\n"
        "    -subSteps   <number of subdivision steps>\n"
        "    -rows       <number of samples>\n"
        "**************************************************************\n");

  // Parse command line arguments
  unsigned int subSteps = 10, rows = 100;
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-subSteps", &subSteps);
  argP.getArgument("-rows", &rows);

  RCHECK((subSteps>0) && (subSteps<rows));


  MatNd* src = MatNd_create(rows,3);
  MatNd* filt = MatNd_create(rows,3);
  MatNd_setRandom(src, 0.0, 1.0);
  MatNd_filterMovingMean(filt, src, subSteps);

  MatNd* compare = MatNd_create(rows, 3);

  for (unsigned int i=0; i<rows; i++)
  {
    MatNd_set(compare, i, 0, i);
    MatNd_set(compare, i, 1, MatNd_get(src, i, 0));
    MatNd_set(compare, i, 2, MatNd_get(filt, i, 0));
  }

  MatNd_toFile(compare, "/tmp/out.dat");

  const char* gpCmd = "set grid\nplot \"/tmp/out.dat\" u 1:2 w l title"
                      " \"raw\", \"/tmp/out.dat\" u 1:3 w lp "
                      "title \"filtered\"\n";
  FILE* outDat = fopen("/tmp/postpro.gnu", "w+");
  RCHECK(outDat);
  fprintf(outDat, gpCmd);
  fflush(outDat);
  fclose(outDat);

  int err = system("/usr/bin/gnuplot -persist /tmp/postpro.gnu");

  if (err == -1)
  {
    RMSGS("Couldn't start gnuplot");
  }
  else
  {
    RMSGS("/usr/bin/gnuplot -persist /tmp/postpro.gnu");
  }

  MatNd_destroy(compare);
  MatNd_destroy(src);
  MatNd_destroy(filt);

  return true;
}

/*******************************************************************************
 * Test of minimum jerk trajectory.
 ******************************************************************************/
bool testMinJerkTrj(int argc, char** argv)
{
  RMSGS("\n\n**************************************************************\n"
        "  Test for minimum jerk trajectory: Morasso & Mussa-Ivaldi model.\n"
        "  Options:\n"
        "    -T   <number of time steps>\n"
        "**************************************************************\n");

  // Parse command line arguments
  int T = 100;
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-T", &T);

  MatNd* s     = MatNd_create(T,1);
  MatNd* s_dot = MatNd_create(T,1);
  MatNd_computeMinimumJerkTrajectory(s, s_dot);

  MatNd* plot = MatNd_create(T, 3);

  for (int i=0; i<T; i++)
  {
    MatNd_set(plot, i, 0, i);
    MatNd_set(plot, i, 1, MatNd_get(s, i, 0));
    MatNd_set(plot, i, 2, MatNd_get(s_dot, i, 0));
  }

  MatNd_toFile(plot, "/tmp/testMinJerkTrj.dat");

  const char* gpCmd = "set grid\nset title \"Morasso & Mussa-Ivaldi model\"\n"
                      "plot \"/tmp/testMinJerkTrj.dat\" u 1:2 w l "
                      "title \"s\", \"/tmp/testMinJerkTrj.dat\" u 1:3 w lp "
                      "title \"s_dot\"\n";
  FILE* outDat = fopen("/tmp/postpro.gnu", "w+");
  RCHECK(outDat);
  fprintf(outDat, gpCmd);
  fflush(outDat);
  fclose(outDat);

  int err = system("/usr/bin/gnuplot -persist /tmp/postpro.gnu");

  if (err == -1)
  {
    RMSGS("Couldn't start gnuplot");
  }
  else
  {
    RMSGS("/usr/bin/gnuplot -persist /tmp/postpro.gnu");
  }

  REXEC(1)
  {
    RMSGS("\ntime step\ts\ts_dot");
    MatNd_printDigits(plot, 6);
  }

  MatNd_destroy(plot);
  MatNd_destroy(s);
  MatNd_destroy(s_dot);

  return true;
}

/*******************************************************************************
 * Test of minimum jerk trajectory.
 ******************************************************************************/
bool testMinJerkTrjPoly(int argc, char** argv)
{
  RMSGS("\n\n****************************************************************"
        "\n  Test for minimum jerk trajectory: 5th order polynomial.\n"
        "  Options:\n"
        "    -T   <number of time steps>\n"
        "****************************************************************\n");

  // Parse command line arguments
  int T = 100;
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-T", &T);

  MatNd* plot = MatNd_create(T, 3);
  double len_sdot = 0.0;

  for (int i=0; i<T; i++)
  {
    double t = (double) i/(T-1);

    double a3 = 10.0, a4 = -15.0, a5 = 6.0;

    double s = a3*pow(t,3) + a4*pow(t,4) + a5*pow(t,5);
    double s_dot = 3.0*a3*pow(t,3) + 4.0*a4*pow(t,4) + 5.0*a5*pow(t,5);
    //double s_ddot = 6.0*a3*pow(t,3) + 12.0*a4*pow(t,4) + 20.0*a5*pow(t,5);
    len_sdot += fabs(s_dot);

    MatNd_set(plot, i, 0, t);
    MatNd_set(plot, i, 1, s);
    MatNd_set(plot, i, 2, s_dot);
  }

  for (int i=0; i<T; i++)
  {
    MatNd_set(plot, i, 2, MatNd_get(plot, i, 2)/len_sdot);
  }

  MatNd_toFile(plot, "/tmp/testMinJerkTrj2.dat");

  const char* gpCmd = "set grid\nset title \"5th order polynomial\"\n"
                      "plot \"/tmp/testMinJerkTrj2.dat\" u 1:2 w l "
                      "title \"s\", \"/tmp/testMinJerkTrj2.dat\" u 1:3 "
                      "w lp title \"s_dot\"\n";
  FILE* outDat = fopen("/tmp/postpro2.gnu", "w+");
  RCHECK(outDat);
  fprintf(outDat, gpCmd);
  fflush(outDat);
  fclose(outDat);

  int err = system("/usr/bin/gnuplot -persist /tmp/postpro2.gnu");

  if (err == -1)
  {
    RMSGS("Couldn't start gnuplot");
  }
  else
  {
    RMSGS("/usr/bin/gnuplot -persist /tmp/postpro2.gnu");
  }

  REXEC(1)
  {
    RMSGS("\ntime step\ts\ts_dot");
    MatNd_printDigits(plot, 6);
  }

  MatNd_destroy(plot);

  return true;
}

/*******************************************************************************
 * Test of arc length interpolation
 ******************************************************************************/
bool testArcLengthInterpolation(int argc, char** argv)
{

  // Parse command line arguments
  int T = 40;
  int dim = 1;
  char fileName[256] = "";
  MatNd* x = NULL;
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-T", &T, "Number of time steps");
  argP.getArgument("-f", fileName, "MatNd file name with 1 column of "
                   "input data (T is ignored)");

  if (argP.hasArgument("-h"))
  {
    RMSGS("\n\n**********************************************************\n"
          "  Test for arc length interpolation.\n"
          "**********************************************************\n");
    argP.print();
    return true;
  }

  if (strlen(fileName)>0)
  {
    RLOG(1, "Reading data from file \"%s\"", fileName);
    x = MatNd_createFromFile(fileName);
    RCHECK_MSG(x, "File \"%s\" could not be opened", fileName);
    T = x->m;
    dim = x->n;
  }
  else
  {
    RLOG(1, "Setting sinusoidal trajectory");
    dim = 1;
    x = MatNd_create(T, dim);

    // Create input array
    for (int i=0; i<T; i++)
    {
      double t = (double)i/(T-1);
      x->ele[i] = sin(M_PI*t);
      //x->ele[i] = t;
    }
  }

  MatNd* plot = MatNd_create(T, 5);


  // Compute minimum jerk parameters. We use a model from Moussa-Ivaldi et al.
  MatNd* s_des = MatNd_create(T+1, 1);
  MatNd_computeMinimumJerkTrajectory(s_des, NULL);




  // Plot array:
  // Column 0: time
  // Column 1: x
  // Column 2: s_des
  for (int i=0; i<T; i++)
  {
    double t = (double)i/T;
    MatNd_set(plot, i, 0, t);
    MatNd_set(plot, i, 1, MatNd_get(x, i, 0));
  }




  // Plot array:
  // Column 3: x(s_des)
  for (int j=0; j<T; j++)
  {
    MatNd plotEle = MatNd_fromPtr(1, 1, MatNd_getElePtr(plot, j, 3));
    double t_s = MatNd_interpolateArcLength(&plotEle, NULL, x,
                                            s_des->ele[j]);
    MatNd_set(plot, j, 2, t_s);
    RLOG(1, "t(%.3f) = %.3f", s_des->ele[j], t_s);
  }







  MatNd_toFile(plot, "/tmp/testArcLengthInterpolation2.dat");

  const char* gpCmd =
    "set grid\nplot \"/tmp/testArcLengthInterpolation2.dat\" u 1:2 w lp "
    "title \"x\", \"/tmp/testArcLengthInterpolation2.dat\" u 3:4 w p title"
    " \"x_mj\"\n";
  FILE* outDat = fopen("/tmp/postpro.gnu", "w+");
  RCHECK(outDat);
  fprintf(outDat, gpCmd);
  fflush(outDat);
  fclose(outDat);

  int err = system("/usr/bin/gnuplot -persist /tmp/postpro.gnu");

  if (err == -1)
  {
    RMSGS("Couldn't start gnuplot");
  }
  else
  {
    RMSGS("/usr/bin/gnuplot -persist /tmp/postpro.gnu");
  }



  MatNd_destroy(s_des);
  MatNd_destroy(plot);
  MatNd_destroy(x);

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool testNumerics(int argc, char** argv)
{
  bool success = true;

  double sv = 0.99;
  double sv_prev = sv;
  unsigned int iter = 0;

  do
  {
    RLOG(1, "sv=%g   eval=%g", sv, fabs(sv*(1.0/sv)-1.0));
    sv_prev = sv;
    sv *= 0.5;
    iter++;
  }
  while (fabs(sv*(1.0/sv)-1.0)<1.0e-8);

  sv = sv_prev;

  RLOG(1, "Found smallest delta after %u iterations: %g error=%g DBL_MIN=%g"
       " delta*(1.0/delta): %g",
       iter, sv, fabs(sv*(1.0/sv)-1.0), DBL_MIN, sv*(1.0/sv));

  RLOG(1, "(%g > 0.0) = %s", sv, sv > 0.0 ? "TRUE" : "FALSE");
  RLOG(1, "(DBL_MIN/10.0) > 0.0 = %s", (DBL_MIN/10.) > 0.0 ? "TRUE" : "FALSE");


  // DBL_MIN, DBL_MAX, std::numeric_limits<double>::infinity()
  RLOG(1, "DBL_MIN = %g", DBL_MIN);
  RLOG(1, "DBL_MAX = %g", DBL_MAX);
  RLOG(1, "std::numeric_limits<double>::infinity() = %g",
       std::numeric_limits<double>::infinity());

  if (DBL_MAX < std::numeric_limits<double>::infinity())
  {
    RMSG("DBL_MAX < infinity (%g)", std::numeric_limits<double>::infinity());
  }
  else if (DBL_MAX > std::numeric_limits<double>::infinity())
  {
    RMSG("DBL_MAX > infinity (%g)", std::numeric_limits<double>::infinity());
  }
  else
  {
    RMSG("DBL_MAX = infinity (%g)", std::numeric_limits<double>::infinity());
  }


  // Check comparison against infinity
  double value = Math_getRandomNumber(-1.0e12, 1.0e12);

  if (value < std::numeric_limits<double>::infinity())
  {
    RMSG("%g < infinity (%g)", value, std::numeric_limits<double>::infinity());
  }
  else if (value > std::numeric_limits<double>::infinity())
  {
    RMSG("%g > infinity (%g)", value, std::numeric_limits<double>::infinity());
  }
  else
  {
    RMSG("%g = infinity (%g)", value, std::numeric_limits<double>::infinity());
  }

  return success;
}

/*******************************************************************************
 * Test of 1-dimensional filters
 ******************************************************************************/
bool testFilters1D(int argc, char** argv)
{
  RMSGS("\n\n**************************************************************\n"
        "  Test for 2nd order LPF."
        "  Options:\n"
        "    -steps   <number of filter steps>\n"
        "    -dt      <sampling time interval>\n"
        "    -tmc     <filter time constant>\n"
        "    -x0      <initial state>\n"
        "    -x1      <target state>\n"
        "**************************************************************\n");

  // Parse command line arguments
  int steps = 1000;
  double dt = 0.005;
  double tmc = 0.5;
  double x0 = 0.0;
  double x1 = 1.0;
  double vmax = 0.5;
  unsigned int medianSamples = 100;
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-steps", &steps, "Number of samples (default: %d)", steps);
  argP.getArgument("-dt", &dt, "Sampling time in secs (default: %f)", dt);
  argP.getArgument("-tmc", &tmc, "Time constant (default: %f)", tmc);
  argP.getArgument("-x0", &x0, "Initial value (default: %f)", x0);
  argP.getArgument("-x1", &x1, "Target value (default: %f)", x1);
  argP.getArgument("-vmax", &vmax, "Max. velocity for ramp filter (default: "
                   "%f)", vmax);
  argP.getArgument("-medianSamples", &medianSamples, "Horizon of median filter"
                   " (default: %d)", medianSamples);

  MatNd* plot = MatNd_create(steps+1, 5);
  MatNd_setElementsTo(plot, x0);

  Rcs::SecondOrderLPF1D sfilt(x0, tmc, dt);
  sfilt.setTarget(x1);

  Rcs::RampFilter1D rfilt(x0, tmc, vmax, dt);
  rfilt.setTarget(x1);

  Rcs::Ramp1D rmp(x0, 0.5*vmax, dt);
  rmp.setTarget(x1);

  Rcs::MedianFilter1D mfilt(medianSamples, x0);

  for (int i=0; i<steps; i++)
  {
    sfilt.iterate();
    rfilt.iterate();
    rmp.iterate();
    double median = mfilt.filt();
    mfilt.addSample(x1);
    MatNd_set(plot, i+1, 0, sfilt.getPosition());
    MatNd_set(plot, i+1, 1, rfilt.getRamp());
    MatNd_set(plot, i+1, 2, rfilt.getPosition());
    MatNd_set(plot, i+1, 3, rmp.getPosition());
    MatNd_set(plot, i+1, 4, median);
  }

  MatNd_toFile(plot, "/tmp/out.dat");

  const char* gpCmd = "set grid\nplot \"/tmp/out.dat\" u 1 w l title"
                      " \"2nd order filter\", \"/tmp/out.dat\" u 2 w l"
                      " title \"ramp\", \"/tmp/out.dat\" u 3 w l"
                      " title \"filtered ramp\", \"/tmp/out.dat\" u 4 w l "
                      "title \"linear ramp\", \"/tmp/out.dat\" u 5 w l title "
                      "\"median filter\"\n";
  FILE* outDat = fopen("/tmp/postpro.gnu", "w+");
  RCHECK(outDat);
  fprintf(outDat, gpCmd);
  fflush(outDat);
  fclose(outDat);

  int err = system("/usr/bin/gnuplot -persist /tmp/postpro.gnu");

  if (err == -1)
  {
    RMSGS("Couldn't start gnuplot");
  }
  else
  {
    RMSGS("/usr/bin/gnuplot -persist /tmp/postpro.gnu");
  }

  MatNd_destroy(plot);

  return true;
}

/*******************************************************************************
 * Test of n-dimensional filters
 ******************************************************************************/
bool testFiltersND(int argc, char** argv)
{
  RMSGS("\n\n**************************************************************\n"
        "  Test for 2nd order LPF."
        "  Options:\n"
        "    -steps   <number of filter steps>\n"
        "    -dt      <sampling time interval>\n"
        "    -tmc     <filter time constant>\n"
        "    -dim     <filter dimension>\n"
        "**************************************************************\n");

  // Parse command line arguments
  unsigned int dim = 3;
  unsigned int steps = 1000;
  double dt = 0.005;
  double tmc = 0.5;
  double vmax = 2.0;
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-steps", &steps);
  argP.getArgument("-dt", &dt);
  argP.getArgument("-tmc", &tmc);
  argP.getArgument("-vmax", &vmax);
  argP.getArgument("-dim", &dim);

  RCHECK(dim<1000);

  MatNd* plot = MatNd_create(steps+1, 3*dim);

  MatNd* x0 = MatNd_create(dim, 1);
  MatNd_setRandom(x0, -1.0, 1.0);

  MatNd* x1 = MatNd_create(dim, 1);
  MatNd_setRandom(x1, 2.0, 3.0);

  Rcs::SecondOrderLPFND sfilt(x0->ele, tmc, dt, dim);
  sfilt.setTarget(x1->ele);

  Rcs::RampFilterND rfilt(x0->ele, tmc, vmax, dt, dim);
  rfilt.setTarget(x1->ele);

  for (unsigned int i=0; i<steps; i++)
  {
    sfilt.iterate();
    rfilt.iterate();

    for (unsigned int j=0; j<dim; j++)
    {
      MatNd_set(plot, i+1, j,       sfilt.getPosition(j));
      MatNd_set(plot, i+1, dim+j,   rfilt.getRamp(j));
      MatNd_set(plot, i+1, 2*dim+j, rfilt.getPosition(j));
    }

  }

  MatNd_toFile(plot, "/tmp/out.dat");

  char gpCmd[4096];
  char tmp[256];
  unsigned int idxCount = 0;

  strcpy(gpCmd, "set grid\nplot ");

  for (unsigned int i=0; i<dim; i++)
  {
    sprintf(tmp, "\"/tmp/out.dat\" u %u w l title \"2nd order filter[%u]\", ",
            i+1, i);
    strcat(gpCmd, tmp);
    sprintf(tmp, "\"/tmp/out.dat\" u %u w l title \"ramp[%u]\", ",
            dim+i+1, i);
    strcat(gpCmd, tmp);

    if (i != dim-1)
    {
      sprintf(tmp, "\"/tmp/out.dat\" u %u w l title \"ramp filter[%u]\", ",
              2*dim+i+1, i);
    }
    else
    {
      sprintf(tmp, "\"/tmp/out.dat\" u %u w l title \"ramp filter[%u]\"",
              2*dim+i+1, i);
    }

    strcat(gpCmd, tmp);
    idxCount++;
  }

  FILE* outDat = fopen("/tmp/postpro.gnu", "w+");
  RCHECK(outDat);
  fprintf(outDat, gpCmd);
  fflush(outDat);
  fclose(outDat);

  int err = system("/usr/bin/gnuplot -persist /tmp/postpro.gnu");

  if (err == -1)
  {
    RMSGS("Couldn't start gnuplot");
  }
  else
  {
    RMSGS("/usr/bin/gnuplot -persist /tmp/postpro.gnu");
  }

  MatNd_destroy(plot);

  return true;
}

/*******************************************************************************
 * Test of minimum rotation angle computation
 ******************************************************************************/
bool testMinimumRotationAngle(int argc, char** argv)
{
  // Start orientation
  double A_SI[3][3];
  Mat3d_setRandomRotation(A_SI);

  // Target orientation
  double A_TI[3][3];
  Mat3d_setRandomRotation(A_TI);

  // Transformation from start to target
  double A_TS[3][3];
  Mat3d_mulTranspose(A_TS, A_TI, A_SI);

  // Get axis angle for relative transformation
  double axis[3];
  double angle = Mat3d_getAxisAngleSelf(axis, A_TS);

  // Now we want to find the minimum angle of rotation for the axis of the
  // relative transformation. If the method works correctly, the estimated angle should
  // be equal to the angle of the relative transformation.
  double angle_min = Mat3d_getMinimumRotationAngle(A_SI, A_TI, axis);
  if (fabs(angle_min - angle) < 1e-5)
  {
    return true;
  }

  return false;
}

/*******************************************************************************
 * Test of line search functions
 ******************************************************************************/
static double lineSearch_func(double* x, void* data)
{
  return 3. + .5 * pow(x[0] - 2., 4);
}

static double lineSearch_dfunc(double* x, void* data)
{
  return 4.*.5 * (x[0] - 2.);
}

// f(1) = 3.0 + 0.5 * pow(-11, 4);
// f(3) = 3.0 + 0.5 * pow(1, 4);
// Minimum is f(2) = 3
static double lineSearch_cost(double* x, void* data)
{
  return 3.0 + 0.5 * pow(x[0] - 2.0, 4);
}

static void lineSearch_grad(double* dfdx, const double* x)
{
  dfdx[0] = 4.0*0.5*(x[0] - 2.0);
}

bool testLinesearch2(int argc, char** argv)
{
  RLOG(1, "**************************************");
  RLOG(1, "Test for MatNd_lineSearch");
  RLOG(1, "**************************************");

  Rcs::CmdLineParser argP(argc, argv);
  int nIter = 1;
  double alpha = 0.9;
  argP.getArgument("-iter", &nIter, "Max. number of iterations");
  argP.getArgument("-alpha", &alpha, "Initial alpha");

  bool success = true;
  MatNd* x    = MatNd_create(1, 1);
  MatNd* dfdx = MatNd_create(1, 1);
  MatNd_setRandom(x, -100.0, 100.0);
  x->ele[0] = 1.0;

  double cost0 = lineSearch_cost(x->ele, NULL);
  double cost = cost0;
  lineSearch_grad(dfdx->ele, x->ele);

  RLOG(1, "Initial cost is %g, initial x is %g", cost0, x->ele[0]);



  for (int i=0; i<nIter; i++)
  {
    cost = MatNd_lineSearchArmijo(x, dfdx, lineSearch_cost, NULL, &alpha);
    RLOG(1, "Min. cost is %.12f, min. x is %.12f, alpha is %g", cost, x->ele[0], alpha);
    lineSearch_grad(dfdx->ele, x->ele);
  }

  if (cost > cost0)
  {
    RMSGS("FAILURE for line search: cost > cost0: %g > %g",
          cost, cost0);
    success = false;
  }
  else
  {
    RLOG(1, "SUCCESS for line search: cost < cost0: %g <= %g",
         cost, cost0);
  }

  MatNd_destroy(x);
  MatNd_destroy(dfdx);

  return success;
}

bool testLinesearch(int argc, char** argv)
{
  Rcs::CmdLineParser argP(argc, argv);
  double maxStep = 1.0;
  argP.getArgument("-maxStep", &maxStep, "Max. step size");
  if (argP.hasArgument("-new"))
  {
    return testLinesearch2(argc, argv);
  }

  RLOG(1, "**************************************");
  RLOG(1, "Test for MatNd_lineSearch");
  RLOG(1, "**************************************");

  bool success = true;
  MatNd* x    = MatNd_create(1, 1);
  MatNd* dfdx = MatNd_create(1, 1);
  MatNd_setRandom(x, -100.0, 100.0);
  x->ele[0] = 1.0;
  dfdx->ele[0] = lineSearch_dfunc(&x->ele[0], NULL);
  double cost0 = lineSearch_func(x->ele, NULL);

  RLOG(1, "Initial x is %g", x->ele[0]);
  RLOG(1, "Initial cost is %g", cost0);

  int nEval = 0;
  double cost = MatNd_lineSearchSelf(x, dfdx, lineSearch_func, NULL,
                                     &nEval, maxStep, NULL);

  RLOG(1, "Did %d evaluations", nEval);
  RLOG(1, "Min. cost is %g, min. x is %g", cost, x->ele[0]);

  if (cost > cost0)
  {
    RMSGS("FAILURE for line search: cost > cost0: %g > %g",
          cost, cost0);
    success = false;
  }
  else
  {
    RLOG(1, "SUCCESS for line search: cost < cost0: %g < %g",
         cost, cost0);
  }

  MatNd_destroy(x);
  MatNd_destroy(dfdx);

  return success;
}

/*******************************************************************************
 * Test of Dynamic Time Warping
 ******************************************************************************/
bool testDTW(int argc, char** argv)
{
  typedef struct
  {
    char name[256];
    FILE* fd;
    long lineCount;
  } DataFile;

  DataFile datFile[16];
  MatNd* seg[16];
  HTr A;
  double tmp, ea[3];
  char a[256];
  char dataDir[256];
  strcpy(dataDir, "config/data/dtw/pouring/");

  for (int i=0; i<16; i++)
  {
    sprintf(datFile[i].name, "%sseg%d.txt", dataDir, i+1);
    RLOG(3, "Opening file \"%s\"", datFile[i].name);
    datFile[i].lineCount = File_getLineCount(datFile[i].name);
    datFile[i].fd = fopen(datFile[i].name, "r");
    seg[i] = MatNd_create(datFile[i].lineCount,7);
  }

  // Assign arrays
  for (int i=0; i<16; i++)
  {
    for (int j=0; j<datFile[i].lineCount; j++)
    {
      int nItemsRead;

      // time step
      nItemsRead = fscanf(datFile[i].fd, "%lf", &tmp);
      RCHECK(nItemsRead==1);
      MatNd_set(seg[i], j, 0, tmp);

      // pos x
      nItemsRead = fscanf(datFile[i].fd, "%lf", &A.org[1]);
      RCHECK(nItemsRead==1);
      MatNd_set(seg[i], j, 1, A.org[1]);

      // pos y
      nItemsRead = fscanf(datFile[i].fd, "%lf", &A.org[2]);
      RCHECK(nItemsRead==1);
      MatNd_set(seg[i], j, 2, A.org[2]);

      // pos z
      nItemsRead = fscanf(datFile[i].fd, "%lf", &A.org[0]);
      RCHECK(nItemsRead==1);
      MatNd_set(seg[i], j, 3, A.org[0]);

      // Euler angles
      nItemsRead = fscanf(datFile[i].fd, "%lf %lf %lf %lf %lf %lf %lf %lf %lf",
                          &A.rot[0][0], &A.rot[0][1], &A.rot[0][2],
                          &A.rot[1][0], &A.rot[1][1], &A.rot[1][2],
                          &A.rot[2][0], &A.rot[2][1], &A.rot[2][2]);
      RCHECK(nItemsRead==9);

      Mat3d_toEulerAngles(ea, A.rot);
      MatNd_set(seg[i], j, 4, ea[0]);
      MatNd_set(seg[i], j, 5, ea[1]);
      MatNd_set(seg[i], j, 6, ea[2]);

      for (int k=0; k<12; k++)
      {
        nItemsRead = fscanf(datFile[i].fd, "%lf", &tmp);
        RCHECK(nItemsRead==1);
      }
    }

    sprintf(a, "seg%d.arr", i);
    MatNd_toFile(seg[i], a);
  }

  // DTW
  MatNd* warped[15];
  MatNd* weight = MatNd_create(7,1);
  MatNd_setElementsTo(weight, 1.0);
  MatNd_set(weight, 0, 0, 0.0);

  for (int i=1; i<16; i++)
  {
    RLOG(3, "Warping array %d", i);
    warped[i-1] = MatNd_create(seg[0]->m, seg[0]->n);
    MatNd_DTW(warped[i-1], seg[0], seg[i], weight);
    sprintf(a, "warped%d.arr", i-1);
    MatNd_toFile(warped[i-1], a);
  }

  // Clean up
  for (int i=0; i<16; i++)
  {
    RLOG(3, "Closing file \"%s\"", datFile[i].name);
    fclose(datFile[i].fd);
    MatNd_destroy(seg[i]);
    if (i!=15)
    {
      MatNd_destroy(warped[i]);
    }
  }
  MatNd_destroy(weight);

  char gpCmd[256];
#if defined (_MSC_VER)
  sprintf(gpCmd, "wgnuplot.exe -persist %sdtw.gnu", dataDir);
#else
  sprintf(gpCmd, "gnuplot -persist %sdtw.gnu", dataDir);
#endif

  int err = system(gpCmd);

  if (err == -1)
  {
    RLOG(1, "Couldn't run \"%s\"", gpCmd);
  }
  else
  {
    RLOG(3, gpCmd);
  }

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool testMat3dFunctions(int argc, char** argv)
{
  bool success = true;


  // Create a square symmetric positiv definite matrix A with
  // random values
  RLOGS(2, "**************************************");
  RLOGS(2, "Test for Mat3d_fromVec()");
  RLOGS(2, "**************************************");
  {
    double A[3][3], v[3];
    Vec3d_setRandom(v, -1.0, 1.0);
    int idx = Math_getRandomInteger(0,2);
    Mat3d_fromVec(A, v, idx);

    if (!Mat3d_isValid(A))
    {
      success = false;

      REXEC(1)
      {
        RMSG("A is invalid:");
        Mat3d_fprint(stderr, A);
      }
    }
    else
    {
      REXEC(3)
      {
        RMSG("Success for Mat3d_fromVec()");
        RMSG("idx = %d   vec = %f %f %f", idx, v[0], v[1], v[2]);
        Mat3d_fprint(stderr, A);
      }
    }
  }



  return success;
}

/*******************************************************************************
 * Test of Via point sequence
 ******************************************************************************/
bool testViaPointSequence2(int argc, char** argv)
{
  char viaFileName[256] = "";

  // Parse command line arguments
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-f", viaFileName, "Via point file name");

  MatNd* viaDesc = NULL;

  if (strlen(viaFileName)>0)
  {
    viaDesc = MatNd_createFromFile(viaFileName);
  }
  else
  {
    viaDesc = MatNd_createFromString("0 0 0 0 7 , 1 1 1 0 1 , 2 2 1 0 1 , 3 3 2 0 1 , 4 3 2 0 1 , 5 3 2 0 1 , 6 3 2 0 1 , 7 4 0 0 7");
  }

  RCHECK(viaDesc);
  MatNd_printCommentDigits("viaDesc", viaDesc, 2);

  Rcs::ViaPointSequence via(viaDesc);
  via.print();
  RCHECK(via.check());
  via.gnuplot();

  MatNd_destroy(viaDesc);

  return true;
}

bool testViaPointSequence(int argc, char** argv)
{
  // Parse command line arguments
  double dt = 0.01;
  unsigned int nSteps = 4;
  double dt_shift = 0.1;
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-dt", &dt, "Sampling time for plotting");
  argP.getArgument("-nSteps", &nSteps, "Number of shifts");
  argP.getArgument("-dt_shift", &dt_shift, "Shift time");

  MatNd* viaDesc = MatNd_createFromString("0 0 0 0 7 , 0.5 2 0 0 1 , 1 1 0 0 7 , 1.1 1 0 0 7");
  RCHECK(viaDesc);
  MatNd_printCommentDigits("ViaDesc", viaDesc, 4);

  MatNd* traj = MatNd_create(4, round(1.5/dt)+1);
  Rcs::ViaPointSequence via(viaDesc);
  RCHECK(via.check());
  via.computeTrajectory(traj, 0.0, 1.5, dt);

  double xt, xt_dot, xt_ddot;


  MatNd* plot = MatNd_create(nSteps+1, traj->n);
  MatNd_copyRow(plot, 0, traj, 1);



  for (unsigned int i=1; i<=nSteps; ++i)
  {
    MatNd* viaDesc_i = MatNd_create(4, 5);

    // Shift first point
    via.computeTrajectoryPoint(xt, xt_dot, xt_ddot, i*dt_shift);
    MatNd_set(viaDesc_i, 0, 0, i*dt_shift);
    MatNd_set(viaDesc_i, 0, 1, xt);
    MatNd_set(viaDesc_i, 0, 2, xt_dot);
    MatNd_set(viaDesc_i, 0, 3, xt_ddot);
    MatNd_set(viaDesc_i, 0, 4, 7);

    // Version 1: Keep via point at the original time (it is getting closer to
    //            the current time)
    VecNd_copy(MatNd_getRowPtr(viaDesc_i, 1), MatNd_getRowPtr(viaDesc, 1), 5);

    // Version 2: Keep via point position at original time step
    // VecNd_copy(MatNd_getRowPtr(viaDesc_i, 1), MatNd_getRowPtr(viaDesc, 1), 5);
    // MatNd_addToEle(viaDesc_i, 1, 0, i*dt_shift);

    // Version 3: Take via point position from previous one-step-ahead time
    // via.computeTrajectoryPoint(xt, xt_dot, xt_ddot, 0.5+i*dt_shift);
    // MatNd_set(viaDesc_i, 1, 0, 0.5+i*dt_shift);
    // MatNd_set(viaDesc_i, 1, 1, xt);
    // MatNd_set(viaDesc_i, 1, 2, xt_dot);
    // MatNd_set(viaDesc_i, 1, 3, xt_ddot);
    // MatNd_set(viaDesc_i, 1, 4, 1);

    // Keep end point
    VecNd_copy(MatNd_getRowPtr(viaDesc_i, 2), MatNd_getRowPtr(viaDesc, 2), 5);

    // Add end point at end of horizon
    VecNd_copy(MatNd_getRowPtr(viaDesc_i, 3), MatNd_getRowPtr(viaDesc, 2), 5);
    MatNd_set(viaDesc_i, 3, 0, 1.0+i*dt_shift);

    Rcs::ViaPointSequence via2(viaDesc_i);
    RCHECK(via2.check());
    MatNd_printCommentDigits("ViaDesc_i", viaDesc_i, 4);

    MatNd* traj1 = MatNd_clone(traj);
    via2.computeTrajectory(traj1, 0.0, 1.5, dt);

    MatNd_copyRow(plot, i, traj1, 1);
    MatNd_destroy(viaDesc_i);
  }

  MatNd_transposeSelf(plot);
  MatNd_gnuplot("plot", plot);

  MatNd_destroy(viaDesc);
  MatNd_destroy(plot);

  return true;
}

bool testViaPointSequence_new(int argc, char** argv)
{
  // Parse command line arguments
  double dt = 0.01;
  unsigned int nSteps = 4;
  double dt_shift = 0.1;
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-dt", &dt, "Sampling time for plotting");
  argP.getArgument("-nSteps", &nSteps, "Number of shifts");
  argP.getArgument("-dt_shift", &dt_shift, "Shift time");

  MatNd* viaDesc = MatNd_createFromString("0 0 0 0 7 , 0.5 2 0 0 1 , 1 1 0 0 7 , 1.1 1 0 0 7");
  RCHECK(viaDesc);
  MatNd_printCommentDigits("ViaDesc", viaDesc, 4);

  MatNd* traj = MatNd_create(4, round(1.5/dt)+1);
  Rcs::ViaPointSequence via(viaDesc);
  RCHECK(via.check());
  via.computeTrajectory(traj, 0.0, 1.5, dt);

  double xt, xt_dot, xt_ddot;


  MatNd* plot = MatNd_create(nSteps+1, traj->n);
  MatNd_copyRow(plot, 0, traj, 1);



  for (unsigned int i=1; i<=nSteps; ++i)
  {
    MatNd* viaDesc_i = MatNd_create(5, 5);

    // Keep first point
    MatNd_copyRow(viaDesc_i, 0, viaDesc, 0);

    // Shift first point
    via.computeTrajectoryPoint(xt, xt_dot, xt_ddot, i*dt_shift);
    MatNd_set(viaDesc_i, 1, 0, i*dt_shift);
    MatNd_set(viaDesc_i, 1, 1, xt);
    MatNd_set(viaDesc_i, 1, 2, xt_dot);
    MatNd_set(viaDesc_i, 1, 3, xt_ddot);
    MatNd_set(viaDesc_i, 1, 4, 7);

    // Version 1: Keep via point at the original time (it is getting closer to
    //            the current time): YES
    // VecNd_copy(MatNd_getRowPtr(viaDesc_i, 2), MatNd_getRowPtr(viaDesc, 1), 5);

    // Version 2: Keep via point position at original time step: NO
    // VecNd_copy(MatNd_getRowPtr(viaDesc_i, 2), MatNd_getRowPtr(viaDesc, 1), 5);
    // MatNd_addToEle(viaDesc_i, 1, 0, i*dt_shift);

    // Version 3: Take via point position from previous one-step-ahead time
    double t_via = 0.5+i*dt_shift;
    via.computeTrajectoryPoint(xt, xt_dot, xt_ddot, t_via);
    MatNd_set(viaDesc_i, 2, 0, t_via);
    MatNd_set(viaDesc_i, 2, 1, xt);
    MatNd_set(viaDesc_i, 2, 2, xt_dot);
    MatNd_set(viaDesc_i, 2, 3, xt_ddot);
    MatNd_set(viaDesc_i, 2, 4, 1);

    // Keep end point
    MatNd_copyRow(viaDesc_i, 3, viaDesc, 2);

    // Add end point at end of horizon
    MatNd_copyRow(viaDesc_i, 4, viaDesc, 3);
    MatNd_set(viaDesc_i, 4, 0, 1.0+i*dt_shift);

    Rcs::ViaPointSequence via2(viaDesc_i);
    RCHECK(via2.check());
    MatNd_printCommentDigits("ViaDesc_i", viaDesc_i, 4);

    MatNd* traj1 = MatNd_clone(traj);
    via2.computeTrajectory(traj1, 0.0, 1.5, dt);

    MatNd_copyRow(plot, i, traj1, 1);
    MatNd_destroy(viaDesc_i);
  }

  MatNd_transposeSelf(plot);
  MatNd_gnuplot("plot", plot);

  MatNd_destroy(viaDesc);
  MatNd_destroy(plot);

  return true;
}

/*******************************************************************************
 * Test of polynomial root finding
 ******************************************************************************/
bool testPolynomialRootFinding(int argc, char** argv)
{
  const unsigned int maxDegree = 3;
  bool success = true;
  double c[maxDegree+1], roots[maxDegree];
  VecNd_setZero(c, maxDegree+1);
  VecNd_setZero(roots, maxDegree);

  // Parse command line arguments
  int iter = 10;
  double lowerLimit = -1000.0;
  double upperLimit =  1000.0;
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-iter", &iter, "Number of test iterations");
  argP.getArgument("-lowerLimit", &lowerLimit,
                   "Lower limit for random polynomial coefficients (default"
                   " is %f)", lowerLimit);
  argP.getArgument("-upperLimit", &upperLimit,
                   "Upper limit for random polynomial coefficients (default"
                   " is %f)", upperLimit);

  for (int i=0; i<iter; ++i)
  {
    int degree = Math_getRandomInteger(1, maxDegree);
    VecNd_setZero(roots, maxDegree);
    VecNd_setZero(c, maxDegree+1);
    VecNd_setRandom(c, lowerLimit, upperLimit, degree+1);

#if 0
    // Disc=0, p!=0
    // D=0, C=1, A=1: 4=b^2 => b=2
    c[0] = 0;
    c[1] = 1;
    c[2] = 2;
    c[3] = 1;

    // D=0, C=2, A=1: 4=b^2 => b=sqrt(8)
    c[0] = 0;
    c[1] = 2;
    c[2] = sqrt(8.0);
    c[3] = 1;

    // Both p & q = 0
    // 3 A C = B^2
    // 2 B^3 - 9 A B C + 27 A^2 D = 0
    // A = 1, D=1: 3C = B*B   B*C = 9
    c[0] = 1;
    c[1] = 3;
    c[2] = 3;
    c[3] = 1;
#endif

    RLOG(5, "Testing polynomial of degree %d with coefficients", degree);
    REXEC(5)
    {
      VecNd_print(c, degree+1);
    }

    int nRoots = Math_findPolynomialRoots(roots, c, degree);

    REXEC(5)
    {
      RLOG(5, "Found %d roots", nRoots);
      if (nRoots > 0)
      {
        VecNd_print(roots, nRoots);
      }
    }

    for (int j=0; j<nRoots; ++j)
    {
      double y = Math_computePolynomial(roots[j], c, degree);
      RLOG(4, "Evaluating root %d: %f", j, y);
      if (fabs(y)>1.0e-8)
      {
        success = false;
        RLOG(1, "Failure for order %d: %g", degree, y);
        REXEC(2)
        {
          VecNd_printComment("Coefficients", c, degree+1);
          VecNd_printComment("Roots", roots, nRoots);
          RPAUSE();
        }
      }
      else
      {
        RLOG(1, "Success for order %d: %f", degree, y);
      }
    }

  }

  return success;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool testEigenvalues3x3(int argc, char** argv)
{
  bool success = true;
  double det, A[3][3], lambda[3], V[3][3];
  Mat3d_setRandom(A, 10.0, 20.0);

  // Make matrix symmetric
  A[1][0] = A[0][1];
  A[2][0] = A[0][2];
  A[2][1] = A[1][2];

  Mat3d_getEigenVectors(V, lambda, A);
  det = Mat3d_determinant(A);


  // Check: determinant is product of all eigenvalues
  double err = fabs(lambda[0]*lambda[1]*lambda[2] - det);

  if (err > 1.0e-8)
  {
    RLOG(1, "Eigenvalues test failed - error is %g", err);
    REXEC(4)
    {
      Mat3d_printCommentDigits("A", A, 8);
      RLOG(4, "Eigenvalues: %g %g %g", lambda[0], lambda[1], lambda[2]);
      RLOG(4, "det(A) = %g   PI(lambda) = %g   err = %g",
           det, lambda[0]*lambda[1]*lambda[2], err);
    }
    success = false;
  }

  RLOG(1, "%s: Error is %g", success ? "SUCCESS" : "FAILURE", err);

  return success;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool testViaPointSequencePlotter(int argc, char** argv)
{
  bool success = true;
  double dt = 0.01;
  unsigned int nSteps = 4;
  int flag = 1;
  int testMode = 0;
  double dt_shift = 0.01, pos, vel, acc;
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-dt", &dt, "Sampling time for plotting");
  argP.getArgument("-nSteps", &nSteps, "Number of shifts");
  argP.getArgument("-dt_shift", &dt_shift, "Shift time");
  argP.getArgument("-flag", &flag, "Flag for via trajectory selection");
  argP.getArgument("-t", &testMode,
                   "0: Keep time, 1: Move with time, 2: keep time at t0=0");
  bool skipPause = argP.hasArgument("-skipPause",
                                    "Skip enter key after each step");

  MatNd* viaDesc = MatNd_createFromString("0 0 0 0 7 , 0.5 2 0 0 1 , 1 1 0 0 7");
  viaDesc = MatNd_realloc(viaDesc, viaDesc->m, viaDesc->n+1);
  MatNd_reshape(viaDesc, viaDesc->m, viaDesc->n-1);

  Rcs::ViaPointSequence via(viaDesc);
  RCHECK(via.check());

  Rcs::ViaPointSequencePlotter plotter;
  plotter.enableFixedAxes(via, true);

  while (true)
  {
    plotter.plot2(via, via.t0(), 1.5/*via.t1()*/, dt, flag);


    // Mode 0: Via point stays with time
    if (testMode==0)
    {
      via.computeTrajectoryPoint(pos, vel, acc, via.t0() + dt_shift);
      MatNd_set(via.viaDescr, 0, 0, via.t0() + dt_shift);
      MatNd_set(via.viaDescr, 0, 1, pos);
      MatNd_set(via.viaDescr, 0, 2, vel);
      MatNd_set(via.viaDescr, 0, 3, acc);
    }
    // Mode 1: Via point shifted with time
    else if (testMode==1)
    {
      via.computeTrajectoryPoint(pos, vel, acc, via.t0() + dt_shift);
      MatNd_set(via.viaDescr, 0, 0, via.t0() + dt_shift);
      MatNd_set(via.viaDescr, 0, 1, pos);
      MatNd_set(via.viaDescr, 0, 2, vel);
      MatNd_set(via.viaDescr, 0, 3, acc);

      // Intermediate via point
      MatNd_addToEle(via.viaDescr, 1, 0, dt_shift);
      pos = via.computeTrajectoryPos(MatNd_get(via.viaDescr, 1, 0));
      MatNd_set(via.viaDescr, 1, 1, pos);
    }
    // Mode 2: Via point shifted with time, everything moves back
    else if (testMode==2)
    {
      // Set first via point to state at t0+dt
      via.computeTrajectoryPoint(pos, vel, acc, via.t0() + dt_shift);
      MatNd_set(via.viaDescr, 0, 1, pos);
      MatNd_set(via.viaDescr, 0, 2, vel);
      MatNd_set(via.viaDescr, 0, 3, acc);

      // Shift second via point back in time, but keep position
      MatNd_addToEle(via.viaDescr, 1, 0, -dt_shift);

      if (via.t1() < MatNd_get(viaDesc, 2, 0))
      {
        via.viaDescr = MatNd_realloc(via.viaDescr, 4, via.viaDescr->n);
        MatNd_copyRow(via.viaDescr, 3, viaDesc, 2);
        RLOG(0, "Adding via point");
      }


      if (via.viaDescr->m>2)
      {
        MatNd_addToEle(via.viaDescr, 2, 0, -dt_shift);
      }

      if (MatNd_get(via.viaDescr, 1, 0) <= MatNd_get(via.viaDescr, 0, 0))
      {
        MatNd_deleteRow(via.viaDescr, 1);
        RLOG(0, "Removing via point");
      }

    }
    else
    {
      RFATAL("No mode %d", testMode);
    }

    // Re-compute polynomial
    via.init(via.viaDescr);

    REXEC(1)
    {
      MatNd_printCommentDigits("Via descriptor", via.viaDescr, 3);
    }

    if (skipPause==false)
    {
      RPAUSE_MSG("Hit enter to shift via point");
    }

    if (Rcs_kbhit())
    {
      getchar();
      double t, x, xp, xpp;
      int flag;
      printf("Enter time: ");
      std::cin >> t;
      printf("Enter x: ");
      std::cin >> x;
      printf("Enter xp: ");
      std::cin >> xp;
      printf("Enter xpp: ");
      std::cin >> xpp;
      printf("Enter flag: ");
      std::cin >> flag;
      via.addViaPoint(t, x, xp, xpp, flag);
      via.init(via.viaDescr);
      RPAUSE_MSG("Hit enter to continue");
    }

    Timer_waitDT(0.01);
  }

  return success;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool testViaPointTrajectory1D(int argc, char** argv)
{
  bool success = true;
  RFATAL("Fix me");
  //   double horizon = 1.0;
  //   int flag = 1;
  //   double dt = 0.01;
  //   Rcs::CmdLineParser argP(argc, argv);
  //   argP.getArgument("-horizon", &horizon, "Receeding horizon length [sec]");
  //   argP.getArgument("-flag", &flag, "Flag for via trajectory selection");
  //   argP.getArgument("-dt", &dt, "Sampling time for plotting");

  //   Rcs::ViaPointTrajectory1D via(1.0, horizon);
  //   RCHECK(via.check());
  //   via.addConstraint(0.5, 2.0, 0.0, 0.0, 1);
  //   via.addConstraint(0.25, -2.0, 0.0, 0.0, 7);
  //   via.addConstraint(0.75, -1.0, 0.0, 0.0, 1);
  //   via.addConstraint(1.0, 5.0, 0.0, 0.0, 7);
  //   via.addConstraint(1.95, 5.0, 0.0, 0.0, 7);
  //   via.initFromConstraints();
  //   std::cout << via << std::endl;

  //   Rcs::ViaPointSequencePlotter plotter;
  //   const Rcs::ViaPointSequence* viaSeq = via.getViaSequence();
  //   plotter.enableFixedAxes(*viaSeq, true);

  //   while (true)
  //   {
  //     plotter.plot2(*viaSeq, -0.2, horizon+0.2, dt, flag);
  //     via.step(dt);
  //     std::cout << via << std::endl;
  //     RPAUSE();
  //   }

  return success;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool testQuaternionConversion(int argc, char** argv)
{
  bool success = true;

  {
    RLOGS(2, "**************************************");
    RLOGS(2, "Test quaternion - rotation matrix");
    RLOGS(2, "**************************************");

    double q[4], A_BI[3][3], A_BI2[3][3], err[3][3];

    Mat3d_setRandomRotation(A_BI);
    Quat_fromRotationMatrix(q, A_BI);
    Quat_toRotationMatrix(A_BI2, q);
    Mat3d_sub(err, A_BI2, A_BI);

    double frobNorm = Mat3d_getFrobeniusnorm(err);

    if (frobNorm>1.0e-8)
    {
      success = false;

      REXEC(1)
      {
        REXEC(2)
        {
          Mat3d_printCommentDigits("A_BI", A_BI, 6);
          VecNd_printComment("q from rotation matrix", q, 4);
          Mat3d_printCommentDigits("A_BI from quaternion", A_BI2, 6);
        }
        Mat3d_printCommentDigits("Error", err, 12);
      }
    }
  }

  {
    RLOGS(2, "**************************************");
    RLOGS(2, "Test quaternion - Euler angles");
    RLOGS(2, "**************************************");

    double q[4], q2[4], ea[3], rm[3][3], err[4];
    Vec3d_setRandom(ea, -3.0*M_PI, 3.0*M_PI);

    Quat_fromEulerAngles(q, ea);

    Mat3d_fromEulerAngles(rm, ea);
    Mat3d_toQuaternion(q2, rm);

    if (q[0]<0.0)
    {
      VecNd_constMulSelf(q, -1.0, 4);
    }

    if (q2[0]<0.0)
    {
      VecNd_constMulSelf(q2, -1.0, 4);
    }

    VecNd_sub(err, q, q2, 4);
    double norm = VecNd_sqrLength(err, 4);


    if (norm>1.0e-8)
    {
      success = false;

      REXEC(1)
      {
        RMSG("norm: %g", norm);
        VecNd_printTwoArraysDiff(q, q2, 4);
      }
    }
  }

  return success;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool testViaPointGradient(int argc, char** argv)
{
  bool success = true;
  double dt = 0.01;
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-dt", &dt, "Sampling time for gradient discretisation");
  bool plotMe = argP.hasArgument("-plot", "Plot trajectory with gnuplot");

  if (dt <= 0.0)
  {
    RLOG(1, "Invalid dt: %f - must be >0.0", dt);
    return false;
  }

  Rcs::ViaPointSequence viaSeq("0 0 0 0 7 , 0.25 2 0 0 1 , 0.5 3 0 0 1 , 0.75 4 0 0 1 , 1 1 0 0 7");

  if (viaSeq.check()==false)
  {
    RLOG(1, "Invalid via point descriptor - couldn't initialize sequence");
    return false;
  }

  double nSteps = (viaSeq.t1()-viaSeq.t0())/dt;
  MatNd* dxdvia = MatNd_create(1, (unsigned int)nSteps+1);
  MatNd* dxdvia2 = MatNd_create(1, (unsigned int)nSteps+1);
  viaSeq.gradientDxDvia(dxdvia, 2, viaSeq.t0(), viaSeq.t1(), dt);
  viaSeq.gradientDxDvia_a(dxdvia2, 2, viaSeq.t0(), viaSeq.t1(), dt);
  MatNd_printTwoArraysDiff(dxdvia, dxdvia2, 8);

  if (plotMe==true)
  {
    MatNd* plotArr = MatNd_create(dxdvia->m, 2);
    MatNd_copyColumn(plotArr, 0, dxdvia, 0);
    MatNd_copyColumn(plotArr, 1, dxdvia2, 0);
    MatNd_gnuplot("Gradients", plotArr);

    REXEC(2)
    {
      viaSeq.gnuplot(dt, 1);
    }
  }

  return success;
}

/*******************************************************************************
 * Eigen3 linear algebra tests
 ******************************************************************************/
#if defined (USE_EIGEN3)

#include "Rcs_eigen.h"

bool testFunctionsEigen3(int argc, char** argv)
{
  bool success = true;
  int dim = 5;

  // Parse command line arguments
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-dim", &dim, "Set default dimansionality");

  // MatNd_SVD
  {
    RLOGS(2, "**************************************");
    RLOGS(2, "Test for MatNd_SVD");
    RLOGS(2, "**************************************");
    int m = 3, n = 5;
    double eps = 1.0e-8;

    argP.getArgument("-rows", &m, "Rows for SVD test");
    argP.getArgument("-cols", &n, "Columns for SVD test");
    argP.getArgument("-eps", &eps, "Smallest singular value");

    MatNd* J = MatNd_create(m, n);
    MatNd_setRandom(J, -1.0, 1.0);

    MatNd* U = MatNd_create(m, n);
    MatNd* S = MatNd_create(n, 1);
    MatNd* V = MatNd_create(n, n);

    // Perform singular value decomposition
    int rank = MatNd_SVD(U, S, V, J, eps);
    V->n=rank;

    // Reconstruct J
    MatNd* UxS    = MatNd_create(U->m, S->m);
    MatNd* diagS  = MatNd_create(S->m, S->m);
    MatNd* VT     = MatNd_create(V->n, V->m);
    MatNd* UxSxVT = MatNd_create(UxS->m, VT->n);

    MatNd_setDiag(diagS, S);
    MatNd_mul(UxS, U, diagS);
    MatNd_transpose(VT, V);
    MatNd_mul(UxSxVT, UxS, VT);

    double rmsErr = MatNd_msqError(J, UxSxVT);
    const double maxErr = 1.0e-12;

    int rankTest = MatNd_rank(J, eps);
    if (rankTest != rank)
    {
      RLOGS(2, "FAILURE for MatNd_rank(): Result doesn't match SVD: %d != %d",
            rankTest, rank);
      success = false;
    }

    if (rmsErr < maxErr)
    {
      RLOGS(2, "SUCCESS for SVD: RMS error is %g (<%g), rank of J is %d",
            rmsErr, maxErr, rank);
    }
    else
    {
      success = false;
      RMSGS("FAILURE for SVD: RMS error is %g (>=%g), rank of J is %d",
            rmsErr, maxErr, rank);
    }

    if ((rmsErr>=maxErr) || (RcsLogLevel>1) || ((RcsLogLevel>0)&&(dim<10)))
    {
      MatNd_printCommentDigits("U", U, 4);
      MatNd_printComment("S", S);
      MatNd_printCommentDigits("V", V, 4);
      MatNd_printCommentDigits("J", J, 4);
      MatNd_printCommentDigits("Reconstruction (UxSxVT)", UxSxVT, 4);
    }

    MatNd_destroy(U);
    MatNd_destroy(S);
    MatNd_destroy(V);

    MatNd_destroy(UxS);
    MatNd_destroy(diagS);
    MatNd_destroy(VT);
    MatNd_destroy(UxSxVT);
    MatNd_destroy(J);
  }

  // MatNd_svdSolve
  {
    RLOGS(2, "**************************************");
    RLOGS(2, "Test for MatNd_svdSolve");
    RLOGS(2, "**************************************");
    // Create a square symmetric positiv definite matrix A with
    // random values
    int n = dim;
    MatNd* A = MatNd_create(n, n);
    MatNd_setRandom(A, -1.0, 1.0);

    // Create a random vector b
    MatNd* b = MatNd_create(n, 1);
    MatNd_setRandom(b, -1.0, 1.0);

    // Solve Ax = b for x
    MatNd* x  = MatNd_create(n, 1);
    double det = MatNd_SVDSolve(x, A, b);
    RLOGS(2, "Determinant is %g", det);

    // Test: Ax = b
    MatNd* Ax  = MatNd_create(n, 1);
    MatNd* err = MatNd_create(n, 1);
    MatNd_mul(Ax, A, x);
    MatNd_sub(err, Ax, b);

    double errNorm = MatNd_getNorm(err);
    const double maxErr = 1.0e-12;

    if (errNorm < maxErr)
    {
      RLOGS(2, "SUCCESS for SVD solve: error norm is %g (<%g)",
            errNorm, maxErr);
    }
    else
    {
      success = false;
      RMSGS("FAILURE for SVD solve: Error norm is %g (>1e-12)", errNorm);
    }


    if (((errNorm>=maxErr) && (RcsLogLevel>2)) ||
        ((RcsLogLevel>0)&&(dim<10)))
    {
      MatNd_printCommentDigits("Error", err, 8);
      MatNd_printCommentDigits("Ax", Ax, 8);
      MatNd_printCommentDigits("b", b, 8);
    }

    // Clean up
    MatNd_destroy(A);
    MatNd_destroy(x);
    MatNd_destroy(b);
    MatNd_destroy(Ax);
    MatNd_destroy(err);
  }

  // MatNd_inverse: inv = V diag(S^-1) U^T
  // Here's some test code based on inv(A)*A = E:
  {
    RLOGS(2, "**************************************");
    RLOGS(2, "Test for MatNd_SVDInverse");
    RLOGS(2, "**************************************");
    int n = dim;
    MatNd* A      = MatNd_create(n, n);
    MatNd* A_inv  = MatNd_create(n, n);
    MatNd* A_invA = MatNd_create(n, n);
    MatNd_setRandom(A, -1.0, 1.0);

    double t0 = Timer_getTime();
    double det = MatNd_SVDInverse(A_inv, A);
    double t1 = Timer_getTime();
    MatNd_mul(A_invA, A_inv, A);

    RLOGS(2, "det = %g, inversion took %g usec", det, 1.0e6 * (t1 - t0));

    if (MatNd_isIdentity(A_invA, 1.0e-8))
    {
      RLOGS(2, "SUCCESS for SVD inverse: A*inv(A) = I");
    }
    else
    {
      success = false;
      RMSGS("FAILURE for SVD inverse: A*inv(A) != I");
      MatNd_printCommentDigits("A", A, 5);
      MatNd_printCommentDigits("inv(A)", A_inv, 5);
      MatNd_printCommentDigits("A*inv(A) = I", A_invA, 5);
    }

    if ((RcsLogLevel>1) || ((RcsLogLevel>0) && (dim<10)))
    {
      MatNd_printCommentDigits("A", A, 5);
      MatNd_printCommentDigits("inv(A)", A_inv, 5);
      MatNd_printCommentDigits("A*inv(A) = E", A_invA, 5);
    }

    MatNd_destroy(A);
    MatNd_destroy(A_inv);
    MatNd_destroy(A_invA);
  }

  // MatNd_QRDecomposition
  {
    RLOGS(2, "**************************************");
    RLOGS(2, "Test for MatNd_QRDecomposition");
    RLOGS(2, "**************************************");
    int m = dim;
    int n = 3*dim;
    argP.getArgument("-rows", &m, "Number of rows");
    argP.getArgument("-cols", &n, "Number of columns");
    MatNd* J      = MatNd_create(m, n);
    MatNd* Q      = MatNd_create(m, m);
    MatNd* QtQ    = MatNd_create(m, m);
    MatNd* R      = MatNd_create(m, n);
    MatNd* J_test = MatNd_create(m, n);

    MatNd_setRandom(J, -1.0, 1.0);

    Timer_setZero();
    MatNd_QRDecomposition(Q, R, J);
    double dt = Timer_getTime();

    // Test J = Q R
    MatNd_mul(J_test, Q, R);

    // Test Q^T Q = I
    MatNd_sqrMulAtBA(QtQ, Q, NULL);
    bool isQorthogonal = MatNd_isIdentity(QtQ, 1.0e-8);

    double errNorm = MatNd_msqError(J, J_test);
    const double maxErr = 1.0e-12;

    if ((errNorm < maxErr) && (isQorthogonal==true))
    {
      RLOGS(2, "SUCCESS for QR decomposition after %.2f msec: error is "
            "%g (<%g)", 1.0e3*dt, errNorm, maxErr);
    }
    else
    {
      success = false;
      RMSGS("FAILURE for QR decompositionafter %.2f msec: error is %g"
            " (>%g)", 1.0e3*dt, errNorm, maxErr);
    }

    if ((errNorm>=maxErr) || (isQorthogonal==false) || (RcsLogLevel>1) ||
        ((RcsLogLevel>0)&&(dim<10)))
    {
      MatNd_printCommentDigits("J", J, 5);
      MatNd_printCommentDigits("Q", Q, 5);
      MatNd_printCommentDigits("R", R, 5);
      MatNd_printCommentDigits("QtQ", QtQ, 5);
      MatNd_printCommentDigits("Q*R", J_test, 5);
    }

    MatNd_destroy(J);
    MatNd_destroy(Q);
    MatNd_destroy(QtQ);
    MatNd_destroy(R);
    MatNd_destroy(J_test);
  }


  return success;
}
#else

bool testFunctionsEigen3(int argc, char** argv)
{
  RMSG("Eigen3 not supported");
  return false;
}

#endif
