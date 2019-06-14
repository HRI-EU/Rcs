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

/*!
 *  Misc:
 *
 *  - How to cast a 2-d array to a pointer:
 *      double rm[3][3];
 *      double (*R_VI)[3][3] = rm;
 */

#include "Rcs_Mat3d.h"
#include "Rcs_MatNd.h"
#include "Rcs_VecNd.h"
#include "Rcs_Vec3d.h"
#include "Rcs_basicMath.h"
#include "Rcs_macros.h"

#include <float.h>



/*******************************************************************************
 * See header.
 ******************************************************************************/
double Mat3d_trace(double A[3][3])
{
  return A[0][0]+A[1][1]+A[2][2];
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Mat3d_setIdentity(double A[3][3])
{
  A[0][0] =  1.0;
  A[0][1] =  0.0;
  A[0][2] =  0.0;

  A[1][0] =  0.0;
  A[1][1] =  1.0;
  A[1][2] =  0.0;

  A[2][0] =  0.0;
  A[2][1] =  0.0;
  A[2][2] =  1.0;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Mat3d_setRandom(double A[3][3], double lower, double upper)
{
  int i, j;

  for (i=0; i<3; i++)
  {
    for (j=0; j<3; j++)
    {
      A[i][j] = Math_getRandomNumber(lower, upper);
    }
  }
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Mat3d_addRandom(double A[3][3], double lower, double upper)
{
  int i, j;

  for (i=0; i<3; i++)
    for (j=0; j<3; j++)
    {
      A[i][j] += Math_getRandomNumber(lower, upper);
    }
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Mat3d_addDiag(double A[3][3], double diag[3])
{
  A[0][0] += diag[0];
  A[1][1] += diag[1];
  A[2][2] += diag[2];
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Mat3d_addConstToDiag(double A[3][3], double value)
{
  A[0][0] += value;
  A[1][1] += value;
  A[2][2] += value;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Mat3d_setZero(double A[3][3])
{
  A[0][0] = 0.0;
  A[0][1] = 0.0;
  A[0][2] = 0.0;

  A[1][0] = 0.0;
  A[1][1] = 0.0;
  A[1][2] = 0.0;

  A[2][0] = 0.0;
  A[2][1] = 0.0;
  A[2][2] = 0.0;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Mat3d_setRotMatX(double A[3][3], double angle)
{
  double sa = sin(angle);
  double ca = cos(angle);

  A[0][0] =  1.0;
  A[0][1] =  0.0;
  A[0][2] =  0.0;

  A[1][0] =  0.0;
  A[1][1] =  ca;
  A[1][2] =  sa;

  A[2][0] =  0.0;
  A[2][1] = -sa;
  A[2][2] =  ca;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Mat3d_setRotMatY(double A[3][3], double angle)
{
  double sa = sin(angle);
  double ca = cos(angle);

  A[0][0] =  ca;
  A[0][1] =  0.0;
  A[0][2] = -sa;

  A[1][0] =  0.0;
  A[1][1] =  1.0;
  A[1][2] =  0.0;

  A[2][0] =  sa;
  A[2][1] =  0.0;
  A[2][2] =  ca;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Mat3d_setRotMatZ(double A[3][3], double angle)
{
  double sa = sin(angle);
  double ca = cos(angle);

  A[0][0] =  ca;
  A[0][1] =  sa;
  A[0][2] =  0.0;

  A[1][0] = -sa;
  A[1][1] =  ca;
  A[1][2] =  0.0;

  A[2][0] =  0.0;
  A[2][1] =  0.0;
  A[2][2] =  1.0;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Mat3d_setElementaryRotation(double A[3][3], unsigned int dir, double angle)
{
  switch (dir)
  {
    case 0:
      Mat3d_setRotMatX(A, angle);
      break;
    case 1:
      Mat3d_setRotMatY(A, angle);
      break;
    case 2:
      Mat3d_setRotMatZ(A, angle);
      break;
    default:
      RFATAL("Index out of range: %d should be within [0...2]", dir);
  }

}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Mat3d_copy(double dst[3][3], double src[3][3])
{
  dst[0][0] = src[0][0];
  dst[0][1] = src[0][1];
  dst[0][2] = src[0][2];

  dst[1][0] = src[1][0];
  dst[1][1] = src[1][1];
  dst[1][2] = src[1][2];

  dst[2][0] = src[2][0];
  dst[2][1] = src[2][1];
  dst[2][2] = src[2][2];
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Mat3d_fromArray(double dst[3][3], double src[9])
{
  dst[0][0] = src[0];
  dst[0][1] = src[1];
  dst[0][2] = src[2];

  dst[1][0] = src[3];
  dst[1][1] = src[4];
  dst[1][2] = src[5];

  dst[2][0] = src[6];
  dst[2][1] = src[7];
  dst[2][2] = src[8];
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Mat3d_toArray(double dst[9], double src[3][3])
{
  dst[0] = src[0][0];
  dst[1] = src[0][1];
  dst[2] = src[0][2];

  dst[3] = src[1][0];
  dst[4] = src[1][1];
  dst[5] = src[1][2];

  dst[6] = src[2][0];
  dst[7] = src[2][1];
  dst[8] = src[2][2];
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Mat3d_sub(double C[3][3], double A[3][3], double B[3][3])
{
  C[0][0] = A[0][0] - B[0][0];
  C[0][1] = A[0][1] - B[0][1];
  C[0][2] = A[0][2] - B[0][2];

  C[1][0] = A[1][0] - B[1][0];
  C[1][1] = A[1][1] - B[1][1];
  C[1][2] = A[1][2] - B[1][2];

  C[2][0] = A[2][0] - B[2][0];
  C[2][1] = A[2][1] - B[2][1];
  C[2][2] = A[2][2] - B[2][2];
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Mat3d_subSelf(double dst[3][3], double src[3][3])
{
  dst[0][0] -= src[0][0];
  dst[0][1] -= src[0][1];
  dst[0][2] -= src[0][2];

  dst[1][0] -= src[1][0];
  dst[1][1] -= src[1][1];
  dst[1][2] -= src[1][2];

  dst[2][0] -= src[2][0];
  dst[2][1] -= src[2][1];
  dst[2][2] -= src[2][2];
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Mat3d_addSelf(double dst[3][3], double src[3][3])
{
  dst[0][0] += src[0][0];
  dst[0][1] += src[0][1];
  dst[0][2] += src[0][2];

  dst[1][0] += src[1][0];
  dst[1][1] += src[1][1];
  dst[1][2] += src[1][2];

  dst[2][0] += src[2][0];
  dst[2][1] += src[2][1];
  dst[2][2] += src[2][2];
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Mat3d_constMulSelf(double dst[3][3], double value)
{
  dst[0][0] *= value;
  dst[0][1] *= value;
  dst[0][2] *= value;

  dst[1][0] *= value;
  dst[1][1] *= value;
  dst[1][2] *= value;

  dst[2][0] *= value;
  dst[2][1] *= value;
  dst[2][2] *= value;
}

/*******************************************************************************
 * Transforms the inertia tensor given in coordinate system K into
 * coordinate system I:
 *
 * I_Inertia = A_IK * K_Inertia * A_IK^T = A_KI^T * K_Inertia * A_KI
 ******************************************************************************/
void Mat3d_similarityTransform(double I_Inertia[3][3], double A_KI[3][3],
                               double K_Inertia[3][3])
{
  double tmp[3][3];
  Mat3d_mul(tmp, K_Inertia, A_KI);           // K_I*A_KI
  Mat3d_transposeMul(I_Inertia, A_KI, tmp);  // A_IK*K_I*A_KI
}

/*******************************************************************************
 * A_KI = A_KV * A_VI
 ******************************************************************************/
void Mat3d_mul(double A_KI[3][3], double A_KV[3][3], double A_VI[3][3])
{
  int i, j;

  for (i = 0; i < 3; i++)
    for (j = 0; j < 3; j++)
      A_KI[i][j] =
        A_KV[i][0] * A_VI[0][j] +
        A_KV[i][1] * A_VI[1][j] +
        A_KV[i][2] * A_VI[2][j];
}

/*******************************************************************************
 * A = B * A
 ******************************************************************************/
void Mat3d_preMulSelf(double A[3][3], double B[3][3])
{
  double Atmp[3][3];
  Mat3d_copy(Atmp, A);
  Mat3d_mul(A, B, Atmp);
}

/*******************************************************************************
 * A = A * B
 ******************************************************************************/
void Mat3d_postMulSelf(double A[3][3], double B[3][3])
{
  double Atmp[3][3];
  Mat3d_copy(Atmp, A);
  Mat3d_mul(A, Atmp, B);
}

/*******************************************************************************
 * A_KI = (A_KV)^T * A_VI
 ******************************************************************************/
void Mat3d_transposeMul(double A_KI[3][3], double A_KV[3][3],
                        double A_VI[3][3])
{
  int i, j;

  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 3; j++)
    {
      A_KI[i][j] =
        A_KV[0][i] * A_VI[0][j] +
        A_KV[1][i] * A_VI[1][j] +
        A_KV[2][i] * A_VI[2][j];
    }
  }
}

/*******************************************************************************
 * A_21 = A_2I * (A_1I)^T
 ******************************************************************************/
void Mat3d_mulTranspose(double A_21[3][3], double A_2I[3][3], double A_1I[3][3])
{
  int i, j;

  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 3; j++)
    {
      A_21[i][j] =
        A_2I[i][0] * A_1I[j][0] +
        A_2I[i][1] * A_1I[j][1] +
        A_2I[i][2] * A_1I[j][2];
    }
  }
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Mat3d_isIdentity(double A[3][3])
{
  int i, j;

  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 3; j++)
    {
      if (A[i][j] != ((i == j) ? 1.0 : 0.0))
      {
        return false;
      }
    }
  }

  return true;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Mat3d_isZero(double A[3][3])
{
  int i, j;

  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 3; j++)
    {
      if (A[i][j] != 0.0)
      {
        return false;
      }
    }
  }

  return true;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Mat3d_isSymmetric(double A[3][3], double eps)
{
  if (fabs(A[0][1]-A[1][0])>eps)
  {
    return false;
  }

  if (fabs(A[0][2]-A[2][0])>eps)
  {
    return false;
  }

  if (fabs(A[1][2]-A[2][1])>eps)
  {
    return false;
  }


  return true;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Mat3d_isDiagonal(double A[3][3], double eps)
{
  if (fabs(A[0][1])>eps)
  {
    return false;
  }

  if (fabs(A[0][2])>eps)
  {
    return false;
  }

  if (fabs(A[1][0])>eps)
  {
    return false;
  }

  if (fabs(A[1][2])>eps)
  {
    return false;
  }

  if (fabs(A[2][0])>eps)
  {
    return false;
  }

  if (fabs(A[2][1])>eps)
  {
    return false;
  }

  return true;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Mat3d_transpose(double dst[3][3], double src[3][3])
{
  dst[0][0] = src[0][0];
  dst[0][1] = src[1][0];
  dst[0][2] = src[2][0];

  dst[1][0] = src[0][1];
  dst[1][1] = src[1][1];
  dst[1][2] = src[2][1];

  dst[2][0] = src[0][2];
  dst[2][1] = src[1][2];
  dst[2][2] = src[2][2];
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Mat3d_transposeSelf(double A[3][3])
{
  double tmp = A[0][1];
  A[0][1] = A[1][0];
  A[1][0] = tmp;

  tmp = A[0][2];
  A[0][2] = A[2][0];
  A[2][0] = tmp;

  tmp = A[1][2];
  A[1][2] = A[2][1];
  A[2][1] = tmp;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
double Mat3d_inverse(double invA[3][3], double A[3][3])
{
  double det = Mat3d_determinant(A);

  // Reset matrix if not invertible
  if (det == 0.0)
  {
    Mat3d_setZero(invA);
    return det;
  }

  invA[0][0] = (A[1][1] * A[2][2] - A[2][1] * A[1][2]) / det;
  invA[1][0] = -(A[1][0] * A[2][2] - A[1][2] * A[2][0]) / det;
  invA[2][0] = (A[1][0] * A[2][1] - A[2][0] * A[1][1]) / det;
  invA[0][1] = -(A[0][1] * A[2][2] - A[0][2] * A[2][1]) / det;
  invA[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) / det;
  invA[2][1] = -(A[0][0] * A[2][1] - A[2][0] * A[0][1]) / det;
  invA[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) / det;
  invA[1][2] = -(A[0][0] * A[1][2] - A[1][0] * A[0][2]) / det;
  invA[2][2] = (A[0][0] * A[1][1] - A[1][0] * A[0][1]) / det;

  return det;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
double Mat3d_rwPinv(double J_pinv[3][3],
                    double J[3][3],
                    double invW[3],
                    double lambda[3])
{
  double det;

  double WJt[3][3], JWJt[3][3], invJWJt[3][3];

  // WJt = W*J^T
  Mat3d_transpose(WJt, J);

  if (invW != NULL)
  {
    double invWmat[3][3];
    Mat3d_setDiag(invWmat, invW);
    Mat3d_preMulSelf(WJt, invWmat);
  }

  // JWJt = J*W*J^T
  Mat3d_mul(JWJt, J, WJt);

  // Adding regularization
  if (lambda != NULL)
  {
    Mat3d_addDiag(JWJt, lambda);
  }

  // Inverse (J W J^T)^-1
  det = Mat3d_inverse(invJWJt, JWJt);

  // Compute pseudoinverse
  if (det > 0.0)
  {
    Mat3d_mul(J_pinv, WJt, invJWJt);
  }
  else
  {
    Mat3d_setZero(J_pinv);
  }

  return det;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
double Mat3d_rwPinv2(double J_pinv[3][3],
                     double J[3][3],
                     double Wx[3],
                     double lambda[3])
{
  double det;

  double JtW[3][3], JtWJ[3][3], invJtWJ[3][3];

  // J^T Wx
  Mat3d_transpose(JtW, J);

  if (Wx != NULL)
  {
    double Wmat[3][3];
    Mat3d_setDiag(Wmat, Wx);
    Mat3d_postMulSelf(JtW, Wmat);
  }

  // JtWJ = J^T*W*J
  Mat3d_mul(JtWJ, JtW, J);

  // Regularization
  if (lambda != NULL)
  {
    Mat3d_addDiag(JtWJ, lambda);
  }

  // Inverse (J^T W J)^-1
  det = Mat3d_inverse(invJtWJ, JtWJ);

  // Post-multiplication part: inv(J^T*W*J) * (J^T*W)
  if (det > 0.0)
  {
    Mat3d_mul(J_pinv, invJtWJ, JtW);
  }
  else
  {
    Mat3d_setZero(J_pinv);
  }

  return det;
}

/*******************************************************************************
 *
 * In-place inverse of a 3 x 3 matrix.
 *
 * | a11 a12 a13 |-1
 * | a21 a22 a23 |    =
 * | a31 a32 a33 |
 *
 *         |   a33a22-a32a23  -(a33a12-a32a13)   a23a12-a22a13  |
 * 1/det * | -(a33a21-a31a23)   a33a11-a31a13  -(a23a11-a21a13) |
 *         |   a32a21-a31a22  -(a32a11-a31a12)   a22a11-a21a12  |
 *
 ******************************************************************************/
double Mat3d_inverseSelf(double A[3][3])
{
  double det = Mat3d_determinant(A);

  if (det==0.0)
  {
    Mat3d_setZero(A);
    return 0.0;
  }

  double a11 = A[0][0];
  double a12 = A[0][1];
  double a13 = A[0][2];
  double a21 = A[1][0];
  double a22 = A[1][1];
  double a23 = A[1][2];
  double a31 = A[2][0];
  double a32 = A[2][1];
  double a33 = A[2][2];

  A[0][0] =  a33*a22-a32*a23;
  A[0][1] =  -(a33*a12-a32*a13);
  A[0][2] =  a23*a12-a22*a13;

  A[1][0] =  -(a33*a21-a31*a23);
  A[1][1] =  a33*a11-a31*a13;
  A[1][2] =  -(a23*a11-a21*a13);

  A[2][0] =  a32*a21-a31*a22;
  A[2][1] =  -(a32*a11-a31*a12);
  A[2][2] =  a22*a11-a21*a12;

  Mat3d_constMulSelf(A, 1.0/det);

  return det;
}

/*******************************************************************************
 *
 *         Computes the angular velocity omega that drives the rotation
 *         matrix of A_prev to the one of A. Omega is represented in the
 *         frame of A. It's implemented according to text books as
 *
 *         tilde(omega) = - dot(A) * transpose(A)
 *
 *                         0   -wz   wy
 *         tilde(omega) =  wz   0   -wx
 *                        -wy   wx   0
 *
 *         More efficiently:
 *
 *         tilde(omega) = - dot(A) * transpose(A)
 *                      = [dot(A) A^T]^T
 *                      = [ (A_des - A) A^T ]^T
 *                      = A (A_des - A)^T
 *                      = A A_des^T - A A^T
 *                      = A A_des^T - I
 *
 ******************************************************************************/
void Mat3d_getOmega(double A[3][3], double A_prev[3][3], double omega[3])
{
  omega[0] = A_prev[2][0]*A[1][0] + A_prev[2][1]*A[1][1] + A_prev[2][2]*A[1][2];
  omega[1] = A_prev[0][0]*A[2][0] + A_prev[0][1]*A[2][1] + A_prev[0][2]*A[2][2];
  omega[2] = A_prev[1][0]*A[0][0] + A_prev[1][1]*A[0][1] + A_prev[1][2]*A[0][2];
}

/*******************************************************************************
 * A_KI = ElementaryRotation(axis, theta) * A_KI
 ******************************************************************************/
void Mat3d_rotateSelfAboutXYZAxis(double A[3][3], unsigned int axis, double th)
{
  double Al[3][3], tmp[3][3];

  Mat3d_copy(tmp, A);

  switch (axis)
  {
    case 0:
      Mat3d_setRotMatX(Al, th);
      A[1][0] = Al[1][1] * tmp[1][0] + Al[1][2] * tmp[2][0];
      A[1][1] = Al[1][1] * tmp[1][1] + Al[1][2] * tmp[2][1];
      A[1][2] = Al[1][1] * tmp[1][2] + Al[1][2] * tmp[2][2];
      A[2][0] = Al[2][1] * tmp[1][0] + Al[2][2] * tmp[2][0];
      A[2][1] = Al[2][1] * tmp[1][1] + Al[2][2] * tmp[2][1];
      A[2][2] = Al[2][1] * tmp[1][2] + Al[2][2] * tmp[2][2];
      break;

    case 1:
      Mat3d_setRotMatY(Al, th);
      A[0][0] = Al[0][0] * tmp[0][0] + Al[0][2] * tmp[2][0];
      A[0][1] = Al[0][0] * tmp[0][1] + Al[0][2] * tmp[2][1];
      A[0][2] = Al[0][0] * tmp[0][2] + Al[0][2] * tmp[2][2];
      A[2][0] = Al[2][0] * tmp[0][0] + Al[2][2] * tmp[2][0];
      A[2][1] = Al[2][0] * tmp[0][1] + Al[2][2] * tmp[2][1];
      A[2][2] = Al[2][0] * tmp[0][2] + Al[2][2] * tmp[2][2];
      break;

    case 2:
      Mat3d_setRotMatZ(Al, th);
      A[0][0] = Al[0][0] * tmp[0][0] + Al[0][1] * tmp[1][0];
      A[0][1] = Al[0][0] * tmp[0][1] + Al[0][1] * tmp[1][1];
      A[0][2] = Al[0][0] * tmp[0][2] + Al[0][1] * tmp[1][2];
      A[1][0] = Al[1][0] * tmp[0][0] + Al[1][1] * tmp[1][0];
      A[1][1] = Al[1][0] * tmp[0][1] + Al[1][1] * tmp[1][1];
      A[1][2] = Al[1][0] * tmp[0][2] + Al[1][1] * tmp[1][2];
      break;

    default:
      RFATAL("Wrong index: %d - should be [0 ... 2]", axis);
  }
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Mat3d_rotateAxisAngle(double A_BI[3][3], const double I_axis[3],
                           const double angle, double A_AI[3][3])
{
  // Rotate world axis into body A-frame
  double A_axis[3];
  Vec3d_rotate(A_axis, A_AI, I_axis);

  double A_BA[3][3];
  Mat3d_fromAxisAngle(A_BA, A_axis, angle);
  Mat3d_mul(A_BI, A_BA, A_AI);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Mat3d_rotateAxisAngleSelf(double A_KI[3][3], const double I_axis[3],
                               const double angle)
{
  double A[3][3];
  Mat3d_copy(A, A_KI);
  Mat3d_rotateAxisAngle(A_KI, I_axis, angle, A);
}

/*******************************************************************************
 *
 * The same can be done by successive unit-rotations, like e.g.:
 *
 * double Ax[3][3], Ay[3][3], Az[3][3];
 *
 * Mat3d_setRotMatX(Ax, omega[0]);
 * Mat3d_setRotMatY(Ay, omega[1]);
 * Mat3d_setRotMatZ(Az, omega[2]);
 *
 * if (worldFrame==true) // A = A * Ax * Ay * Az
 *   {
 *     Mat3d_postMulSelf(A_KI, Ax);
 *     Mat3d_postMulSelf(A_KI, Ay);
 *     Mat3d_postMulSelf(A_KI, Az);
 *   }
 * else // A = Az * Ay * Ax * A
 *   {
 *     Mat3d_preMulSelf(A_KI, Ax);
 *     Mat3d_preMulSelf(A_KI, Ay);
 *     Mat3d_preMulSelf(A_KI, Az);
 *   }
 *
 ******************************************************************************/
void Mat3d_rotateOmegaSelf(double A_KI[3][3], const double omega[3],
                           bool worldFrame)
{
  double axis[3], angle;

  angle = Vec3d_normalize(axis, omega);

  if (angle == 0.0)
  {
    return;
  }

  if (worldFrame==true)
  {
    Vec3d_rotateSelf(axis, A_KI);
  }

  double A[3][3];
  Mat3d_fromAxisAngle(A, axis, angle);
  Mat3d_preMulSelf(A_KI, A);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
//! \todo Really exit hard?
double Mat3d_getMinimumRotationAngle(double A_SI[3][3], double A_TI[3][3],
                                     double axis[3])
{
  // Compute relative transformation from x to mean of distribution
  double A_TS[3][3];
  Mat3d_mulTranspose(A_TS, A_TI, A_SI);

  // Pre-compute values needed for solution
  double a0 = A_TS[0][0] - A_TS[0][0]*axis[0]*axis[0] -
              A_TS[0][1]*axis[1]*axis[0] - A_TS[0][2]*axis[2]*axis[0];
  double b0 = A_TS[0][1]*axis[2] - A_TS[0][2]*axis[1];
  double a1 = -A_TS[1][0]*axis[0]*axis[1] + A_TS[1][1] -
              A_TS[1][1]*axis[1]*axis[1] - A_TS[1][2]*axis[2]*axis[1];
  double b1 = -A_TS[1][0]*axis[2] + A_TS[1][2]*axis[0];
  double a2 = -A_TS[2][0]*axis[0]*axis[2] - A_TS[2][1]*axis[1]*axis[2] +
              A_TS[2][2] - A_TS[2][2]*axis[2]*axis[2];
  double b2 = A_TS[2][0]*axis[1] - A_TS[2][1]*axis[0];

  double t0 = a0 + a1 + a2;
  double t1 = b0 + b1 + b2;

  // Compute minimum angle
  double t = atan2(t1, t0);
  RCHECK(Math_isFinite(t));
  return t;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Mat3d_toEulerAngles(double ea[3], double A_KI[3][3])
{
  double cy = sqrt(A_KI[0][0] * A_KI[0][0] + A_KI[1][0] * A_KI[1][0]);

  if (cy > 16.0 * FLT_EPSILON)
  {
    ea[0] = -atan2(A_KI[2][1], A_KI[2][2]);
    ea[1] = -atan2(-A_KI[2][0], cy);
    ea[2] = -atan2(A_KI[1][0], A_KI[0][0]);
  }
  else
  {
    ea[0] = -atan2(-A_KI[1][2], A_KI[1][1]);
    ea[1] = -atan2(-A_KI[2][0], cy);
    ea[2] = 0.0;
  }
}

/*******************************************************************************
 * eap = H * I_omega
 ******************************************************************************/
void Mat3d_getEulerVelocityMatrix(double H[3][3], const double ea[3])
{
  double sa = sin(ea[0]);
  double ca = cos(ea[0]);
  double sb = sin(ea[1]);
  double cb = cos(ea[1]);

  H[0][0] = 1.0;
  H[0][1] = sa * sb / cb;
  H[0][2] = -ca * sb / cb;

  H[1][0] = 0.0;
  H[1][1] = ca;
  H[1][2] = sa;

  H[2][0] = 0.0;
  H[2][1] = -sa / cb;
  H[2][2] = ca / cb;
}

/*******************************************************************************
 * I_omega = H * eap
 ******************************************************************************/
void Mat3d_getInverseEulerVelocityMatrix(double invH[3][3], const double ea[3])
{
  double sa = sin(ea[0]);
  double ca = cos(ea[0]);
  double sb = sin(ea[1]);
  double cb = cos(ea[1]);

  invH[0][0] = 1.0;
  invH[0][1] = 0.0;
  invH[0][2] = sb;
  invH[1][0] = 0.0;
  invH[1][1] = ca;
  invH[1][2] = -sa*cb;
  invH[2][0] = 0.0;
  invH[2][1] = sa;
  invH[2][2] = ca*cb;
}

/*******************************************************************************
 * d(H^-1)/dt
 ******************************************************************************/
void Mat3d_getInverseEulerVelocityMatrixDerivative(double dInvH[3][3],
                                                   const double ea[3],
                                                   const double eap[3])
{
  double sa = sin(ea[0]);
  double ca = cos(ea[0]);
  double sb = sin(ea[1]);
  double cb = cos(ea[1]);

  dInvH[0][0] = 0.0;
  dInvH[0][1] = 0.0;
  dInvH[0][2] = eap[1]*cb;
  dInvH[1][0] = 0.0;
  dInvH[1][1] = -eap[0]*sa;
  dInvH[1][2] = -eap[0]*ca*cb + eap[1]*sa*sb;
  dInvH[2][0] = 0.0;
  dInvH[2][1] = eap[0]*ca;
  dInvH[2][2] = -eap[0]*sa*cb - eap[1]*ca*sb;
}

/*******************************************************************************
 * e = 0.5 * (n x nd + s x sd + a x ad)
 ******************************************************************************/
void Mat3d_getEulerError(double e[3], double A_act[3][3], double A_des[3][3])
{
  const double* n, *nd, *s, *sd, *a, *ad;
  double tmp[3];

  // Desired coordinate axes
  nd = &A_des[0][0];
  sd = &A_des[1][0];
  ad = &A_des[2][0];

  // Current coordinate axes
  n = &A_act[0][0];
  s = &A_act[1][0];
  a = &A_act[2][0];

  // Feedback error e = 0.5 * (n x nd + s x sd + a x ad)
  Vec3d_crossProduct(e, n, nd);
  Vec3d_crossProduct(tmp, s, sd);
  Vec3d_addSelf(e, tmp);
  Vec3d_crossProduct(tmp, a, ad);
  Vec3d_addSelf(e, tmp);
  Vec3d_constMulSelf(e, 0.5);
}

/*******************************************************************************
 *
 * Euler error partial derivative wrt. x_des = (alpha beta gamma)^T
 *
 * de/dx = 0.5* (n x dx(nd) + s x dx(sd) + a x dx(ad))
 *
 * with n being the 1st row of the A_KI rotation matrix, s the second and
 * a the third one. Index d stands for desired. Result grad is 3 x 3.
 *
 ******************************************************************************/
void Mat3d_dEulerErrorDx(double grad[3][3], const double x_des[3],
                         double A_KI[3][3])
{
  double dxndBuf[9], dxsdBuf[9], dxadBuf[9];
  MatNd dxnd = MatNd_fromPtr(3, 3, dxndBuf);
  MatNd dxsd = MatNd_fromPtr(3, 3, dxsdBuf);
  MatNd dxad = MatNd_fromPtr(3, 3, dxadBuf);
  MatNd dedx = MatNd_fromPtr(3, 3, (double*) grad);

  double sa = sin(x_des[0]);
  double ca = cos(x_des[0]);
  double sb = sin(x_des[1]);
  double cb = cos(x_des[1]);
  double sg = sin(x_des[2]);
  double cg = cos(x_des[2]);

  // dx(nd)
  dxnd.ele[0] = 0.0;
  dxnd.ele[1] = -sb * cg;
  dxnd.ele[2] = -cb * sg;

  dxnd.ele[3] = -sa * sg - ca * sb * cg;
  dxnd.ele[4] = sa * cb * cg;
  dxnd.ele[5] = ca * cg - sa * sb * sg;

  dxnd.ele[6] = ca * sg + sa * sb * cg;
  dxnd.ele[7] = -ca * cb * cg;
  dxnd.ele[8] = sa * cg + ca * sb * sg;

  // dx(sd)
  dxsd.ele[0] = 0.0;
  dxsd.ele[1] = sb * sg;
  dxsd.ele[2] = -cb * cg;

  dxsd.ele[3] = -sa * cg - ca * sb * sg;
  dxsd.ele[4] = -sa * cb * sg;
  dxsd.ele[5] = -ca * sg - sa * sb * cg;

  dxsd.ele[6] = ca * cg - sa * sb * sg;
  dxsd.ele[7] = ca * cb * sg;
  dxsd.ele[8] = -sa * sg - ca * sb * cg;

  // dx(ad)
  dxad.ele[0] = 0.0;
  dxad.ele[1] = cb;
  dxad.ele[2] = 0.0;

  dxad.ele[3] = -ca * cb;
  dxad.ele[4] = sa * sb;
  dxad.ele[5] = 0.0;

  dxad.ele[6] = -sa * cb;
  dxad.ele[7] = -ca * sb;
  dxad.ele[8] = 0.0;

  // Actual axis directions
  const double* n = A_KI[0];
  const double* s = A_KI[1];
  const double* a = A_KI[2];

  // n x dx(nd) + s x dx(sd) + a x dx(ad)
  MatNd_columnCrossProductSelf(&dxnd, n);
  MatNd_columnCrossProductSelf(&dxsd, s);
  MatNd_columnCrossProductSelf(&dxad, a);

  // Result
  MatNd_add(&dedx, &dxnd, &dxsd);
  MatNd_addSelf(&dedx, &dxad);
  MatNd_constMulSelf(&dedx, 0.5);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Mat3d_fromEulerAngles(double A_KI[3][3], const double ea[3])
{
  double sa = sin(ea[0]);
  double ca = cos(ea[0]);
  double sb = sin(ea[1]);
  double cb = cos(ea[1]);
  double sg = sin(ea[2]);
  double cg = cos(ea[2]);

  A_KI[0][0] = cb * cg;
  A_KI[0][1] = ca * sg + sa * sb * cg;
  A_KI[0][2] = sa * sg - ca * sb * cg;

  A_KI[1][0] = -cb * sg;
  A_KI[1][1] = ca * cg - sa * sb * sg;
  A_KI[1][2] = sa * cg + ca * sb * sg;

  A_KI[2][0] = sb;
  A_KI[2][1] = -sa * cb;
  A_KI[2][2] = ca * cb;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Mat3d_fromEulerAngles2(double A_KI[3][3], double a, double b, double c)
{
  double ea[3];
  Vec3d_set(ea, a, b, c);
  Mat3d_fromEulerAngles(A_KI, ea);
}

/*******************************************************************************
 * The rotation matrix is constructed according to the axis-angle
 * representation, see e.g. http://en.wikipedia.org/wiki/Rotation_matrix
 *                                 #Rotation_matrix_from_axis_and_angle
 ******************************************************************************/
void Mat3d_fromAxisAngle(double A_KI[3][3], const double axis_[3],
                         const double angle)
{
  double axis[3], s, c, u, vx, vy, vz;

  double len = Vec3d_normalize(axis, axis_);

  // If the axis length is 0, we return the identity matrix
  if (len==0.0)
  {
    Mat3d_setIdentity(A_KI);
    return;
  }

  vx = axis[0];
  vy = axis[1];
  vz = axis[2];

  s = sin(angle);
  c = cos(angle);
  u = 1.0 - c;

  A_KI[0][0] = vx*vx*u + c;
  A_KI[1][0] = vy*vx*u-vz*s;
  A_KI[2][0] = vz*vx*u + vy*s;

  A_KI[0][1] = vx*vy*u + vz*s;
  A_KI[1][1] = vy*vy*u + c;
  A_KI[2][1] = vz*vy*u - vx*s;

  A_KI[0][2] = vx*vz*u - vy*s;
  A_KI[1][2] = vy*vz*u + vx*s;
  A_KI[2][2] = vz*vz*u + c;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
double Mat3d_getAxisAngle(double axis[3], double A_2I[3][3], double A_1I[3][3])
{
  double phi, A_21[3][3];

  Mat3d_mulTranspose(A_21, A_2I, A_1I);

  phi = Mat3d_getAxisAngleSelf(axis, A_21);

  // The axis is now in the 1-frame. We need to rotate it into the I-frame
  Vec3d_transRotateSelf(axis, A_1I);

  return phi;
}

/*******************************************************************************
 * Determinant of a 3x3 rotation matrix:
 * det = a11(a33a22-a32a23)-a21(a33a12-a32a13)+a31(a23a12-a22a13)
 ******************************************************************************/
double Mat3d_determinant(double A[3][3])
{
  double det =
    A[0][0] * (A[1][1] * A[2][2] - A[2][1] * A[1][2]) -
    A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
    A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);

  return det;
}

/*******************************************************************************
 * Returns false if any matrix element is not finite, true otherwise.
 ******************************************************************************/
bool Mat3d_isFinite(double A[3][3])
{
  int i, j;

  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 3; j++)
    {
      if (Math_isFinite(A[i][j])==0)
      {
        return false;
      }
    }
  }

  return true;
}

/*******************************************************************************
 * Checks orthonormality (A' * A = I) and right-handedness (det(A) = 1).
 *        That's the sufficient conditions for a pure rotation matrix.
 ******************************************************************************/
bool Mat3d_isValid(double A[3][3])
{
  double err, eps = 1.0 * (M_PI / 180.); // 1 degree

  // Invalid if A is NULL
  if (!A)
  {
    RLOG(4, "Transformation is NULL");
    return false;
  }

  // Invalid if any matrix element is not finite
  if (Mat3d_isFinite(A) == false)
  {
    REXEC(4)
    {
      RLOG(4, "Matrix contains infinite elements:");
      Mat3d_fprint(stderr, A);
    }
    return false;
  }

  // Orthogonality and right-handed checks

  // ex ey'
  err = fabs(A[0][0] * A[1][0] + A[0][1] * A[1][1] + A[0][2] * A[1][2]);
  if (err > eps)
  {
    RLOG(4, "ex ey': err =%f", err);
    return false;
  }

  // ex ez'
  err = fabs(A[0][0] * A[2][0] + A[0][1] * A[2][1] + A[0][2] * A[2][2]);
  if (err > eps)
  {
    RLOG(4, "ex ez': err =%f", err);
    return false;
  }

  // ey ez'
  err = fabs(A[1][0] * A[2][0] + A[1][1] * A[2][1] + A[1][2] * A[2][2]);
  if (err > eps)
  {
    RLOG(4, "ey ez': err =%f", err);
    return false;
  }

  // ex ex'
  err = fabs(A[0][0] * A[0][0] + A[0][1] * A[0][1] + A[0][2] * A[0][2] - 1.0);
  if (err > eps)
  {
    RLOG(4, "ex ex': err = %f", err);
    return false;
  }

  // ey ey'
  err = fabs(A[1][0] * A[1][0] + A[1][1] * A[1][1] + A[1][2] * A[1][2] - 1.0);
  if (err > eps)
  {
    RLOG(4, "ey ey': err =%f", err);
    return false;
  }

  // ez ez'
  err = fabs(A[2][0] * A[2][0] + A[2][1] * A[2][1] + A[2][2] * A[2][2] - 1.0);
  if (err > eps)
  {
    RLOG(4, "ez ez': err =%f", err);
    return false;
  }

  // Right-handedness: Determinant is 1 (Left-handed: -1)
  err = fabs(Mat3d_determinant(A) - 1.0);
  if (err > eps)
  {
    RLOG(4, "det(R) = 1: %f", err);
    return false;
  }

  return true;
}

/*******************************************************************************
 *
 *        Elementary rotation matrices:    Their derivatives:
 *
 *               1   0     0                      0    0    0
 *        Ax =   0   cos   sin             dAx =  0   -sin  cos
 *               0  -sin   cos                    0   -cos -sin
 *
 *               cos   0   -sin                  -sin   0    -cos
 *        Ay =   0     1    0              dAy =  0     0     0
 *               sin   0    cos                   cos   0    -sin
 *
 *               cos   sin  0                    -sin   cos   0
 *        Az =  -sin   cos  0              dAz = -cos  -sin   0
 *               0     0    1                     0     0     0
 *
 ******************************************************************************/
void Mat3d_dAdq(double dAdq[3][3], unsigned int dir, double q)
{
  double sq = sin(q);
  double cq = cos(q);

  Mat3d_setZero(dAdq);

  switch (dir)
  {
    case 0:   // X
      dAdq[1][1] = -sq;
      dAdq[1][2] =  cq;
      dAdq[2][1] = -cq;
      dAdq[2][2] = -sq;
      break;

    case 1:   // Y
      dAdq[0][0] = -sq;
      dAdq[0][2] = -cq;
      dAdq[2][0] =  cq;
      dAdq[2][2] = -sq;
      break;

    case 2:   // Z
      dAdq[0][0] = -sq;
      dAdq[0][1] =  cq;
      dAdq[1][0] = -cq;
      dAdq[1][1] = -sq;
      break;

    default:
      RFATAL("Wrong index: %d - should be [0 ... 2]", dir);
  }

}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Mat3d_slerp(double A[3][3], double A_1I[3][3], double A_2I[3][3],
                 double t)
{
  double A_21[3][3], phi;

  // Relative transform A_21 = A_2I * A_I1
  Mat3d_mulTranspose(A_21, A_2I, A_1I);

  // Axis angle between A_1I and A_2I
  phi = Math_acos((A_21[0][0] + A_21[1][1] + A_21[2][2] - 1.0) / 2.0);

  // If relative rotation is zero, we just copy A_1I
  if (phi == 0.0)
  {
    Mat3d_copy(A, A_1I);
    return;
  }

  double A_12[3][3], n[3], N[3][3], N2[3][3], expAt[3][3], A_I1[3][3];

  Mat3d_transpose(A_12, A_21);

  // N = (1/(2*sin(phi)))*(A_12-A_12^T)
  Mat3d_sub(N, A_12, A_21);
  Mat3d_constMulSelf(N, 1.0 / (2.0 * sin(phi)));

  // Axis vector n comes out of N
  Vec3d_set(n, N[2][1], N[0][2], N[1][0]);

  // N^2 = n^T n - E
  Vec3d_outerProductSelf(N2, n);
  Mat3d_addConstToDiag(N2, -1.0);

  // exp(phi*t) = E + N*sin(phi*t) + N^2*(1-cos(phi*t)
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      expAt[i][j] = ((i == j) ? 1.0 : 0.0) +
                    N[i][j] * sin(phi * t) + N2[i][j] * (1.0 - cos(phi * t));
    }
  }

  // Project into I-coordinates
  Mat3d_transpose(A_I1, A_1I);
  Mat3d_mul(A, A_I1, expAt);
  Mat3d_transposeSelf(A);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
double Mat3d_clip(double A_clipped[3][3], double A_from[3][3], double A_to[3][3],
                double maxAngle)
{
  const double phi = Mat3d_diffAngle(A_from, A_to);

  if (phi <= maxAngle)
    {
      Mat3d_copy(A_clipped, A_to);
      return 1.0;
    }

  // Compute orientation error e
  double e[3], A_err[3][3];

  if (phi<1.0*(M_PI/180.0))   // Less than 1 deg error: Use Euler error
  {
    Mat3d_getEulerError(e, A_from, A_to);
  }
  else   // Larger error: Use axis angle error
  {
    Mat3d_getAxisAngle(e, A_to, A_from);
    Vec3d_constMulSelf(e, phi);
  }

  // Error e is in world coords, lets transform it into the local frame
  Vec3d_rotateSelf(e, A_from);
  Vec3d_constMulSelf(e, maxAngle/phi);
  Mat3d_fromEulerAngles(A_err, e);
  Mat3d_mul(A_clipped, A_err, A_from);

  return maxAngle/phi;
}

/*******************************************************************************
 *
 *         0    -rz    ry
 *  ~r =   rz    0    -rx
 *        -ry    rx    0
 *
 ******************************************************************************/
void Mat3d_skew(double rSkew[3][3], const double r[3])
{
  rSkew[0][0] =  0.0;
  rSkew[0][1] = -r[2];
  rSkew[0][2] =  r[1];
  rSkew[1][0] =  r[2];
  rSkew[1][1] =  0.0;
  rSkew[1][2] = -r[0];
  rSkew[2][0] = -r[1];
  rSkew[2][1] =  r[0];
  rSkew[2][2] =  0.0;
}

/*******************************************************************************
 *
 *           0    rz   -ry
 *  ~r^T =  -rz   0     rx
 *           ry  -rx    0
 *
 ******************************************************************************/
void Mat3d_transposeSkew(double rSkewTp[3][3], const double r[3])
{
  rSkewTp[0][0] =  0.0;
  rSkewTp[0][1] =  r[2];
  rSkewTp[0][2] = -r[1];
  rSkewTp[1][0] = -r[2];
  rSkewTp[1][1] =  0.0;
  rSkewTp[1][2] =  r[0];
  rSkewTp[2][0] =  r[1];
  rSkewTp[2][1] = -r[0];
  rSkewTp[2][2] =  0.0;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Mat3d_fprint(FILE* fd, double A[3][3])
{
  fprintf(fd, "\t%.6f\t%.6f\t%.6f\n", A[0][0], A[0][1], A[0][2]);
  fprintf(fd, "\t%.6f\t%.6f\t%.6f\n", A[1][0], A[1][1], A[1][2]);
  fprintf(fd, "\t%.6f\t%.6f\t%.6f\n", A[2][0], A[2][1], A[2][2]);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Mat3d_printCommentDigits(const char* text, double mat[3][3],
                              unsigned int digits)
{
  unsigned int i, j;
  char formatStr[16];

  if (text != NULL)
  {
    fprintf(stderr, "%s\n", text);
  }

  snprintf(formatStr, 16, "%%+5.%uf ", digits);

  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 3; j++)
    {
      fprintf(stderr, formatStr, mat[i][j]);
    }
    fprintf(stderr, "\n");
  }
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Mat3d_printFormatted(const char* text, const char* format, double M[3][3])
{
  unsigned int i, j;

  if (text != NULL)
  {
    fprintf(stderr, "%s\n", text);
  }

  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 3; j++)
    {
      fprintf(stderr, format, M[i][j]);
    }
    fprintf(stderr, "\n");
  }
}

/*******************************************************************************
 * A_21 = A_2I*A_1I^T
 ******************************************************************************/
double Mat3d_diffAngle(double A1I[3][3], double A2I[3][3])
{
  double A_21[3][3];
  Mat3d_transposeMul(A_21, A2I, A1I);
  return Mat3d_diffAngleSelf(A_21);
}

/*******************************************************************************
 *
 *  The result is between [0 ... 180deg], which means that the inner part of
 *  the bracket is between [-1 ... 1]: 0.5*(trace(A)-1)
 *  => [-2 ... 2]: trace(A)-1
 *  => [-1 ... 3]: trace(A)
 *  For trace(A) =  3: diff =   0 deg
 *  For trace(A) = -1: diff = 180 deg
 *
 ******************************************************************************/
double Mat3d_diffAngleSelf(double A[3][3])
{
  return Math_acos(0.5 * (A[0][0] + A[1][1] + A[2][2] - 1.0));
}

/*******************************************************************************
 * There is a gradient test function in the test executable.
 ******************************************************************************/
void Mat3d_dDiffAngleDEuler(double dAAdEA[3], const double a[3])
{
  double cosPhi, term1;

  double sa = sin(a[0]);
  double ca = cos(a[0]);
  double sb = sin(a[1]);
  double cb = cos(a[1]);
  double sg = sin(a[2]);
  double cg = cos(a[2]);

  double a00 = cb * cg;
  double a11 = ca * cg - sa * sb * sg;
  double a22 = ca * cb;

  cosPhi = 0.5 * (a00 + a11 + a22 - 1.0);

  if (cosPhi==1.0)
  {
    term1 = 0.0;
  }
  else
  {
    term1 = -1.0 / (2.0 * sqrt(1.0 - cosPhi * cosPhi));
  }


  double dDa = -sa * cg - ca * sb * sg - sa * cb;
  double dDb = -sb * cg - sa * cb * sg - ca * sb;
  double dDg = -cb * sg - ca * sg - sa * sb * cg;

  dAAdEA[0] = dDa * term1;
  dAAdEA[1] = dDb * term1;
  dAAdEA[2] = dDg * term1;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Mat3d_setDiag(double A[3][3], const double v[3])
{
  for (unsigned int i = 0; i < 3; i++)
  {
    A[i][i] = v[i];
  }
}

/*******************************************************************************
 *
 * Here's a pedantic check, which can be added to the end of the
 * function:
 *
 * REXEC(0)
 * {
 *   if (!Mat3d_isValid(A))
 *   {
 *     RcsLogLevel = 4;
 *     RLOG(0, "A is");
 *     Mat3d_fprint(stderr, A);
 *     RLOG(0, "length of A[2] is %.16f  sqrLength = %.16f  sqrt = %.16f",
 *           Vec3d_getLength(A[2]), sqrLength , sqrt(sqrLength));
 *     RLOG(0, "index=%d invLen=%g v=%g %g %g",
 *          dir, 1.0 / sqrt(sqrLength), v[0], v[1], v[2]);
 *     RCHECK_MSG(Mat3d_isValid(A), "This must not happen!");
 *   }
 * }
 *
 ******************************************************************************/
void Mat3d_fromVec(double A[3][3], const double v[3], int dir)
{
  double sqrLength = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];

  if (sqrLength == 0.0)
  {
    Mat3d_setIdentity(A);
    return;
  }

  switch (dir)
  {
    case 0:
    {
      Vec3d_constMul(A[0], v, 1.0 / sqrt(sqrLength));  // x-axis
      Vec3d_orthonormalVec(A[1], A[0]);                // y-axis
      Vec3d_crossProduct(A[2], A[0], A[1]);            // z-axis
      break;
    }
    case 1:
    {
      Vec3d_constMul(A[1], v, 1.0 / sqrt(sqrLength));  // y-axis
      Vec3d_orthonormalVec(A[2], A[1]);                // z-axis
      Vec3d_crossProduct(A[0], A[1], A[2]);            // x-axis
      break;
    }
    case 2:
    {
      Vec3d_constMul(A[2], v, 1.0 / sqrt(sqrLength));  // z-axis
      Vec3d_orthonormalVec(A[1], A[2]);                // y-axis
      Vec3d_crossProduct(A[0], A[1], A[2]);            // x-axis
      break;
    }
    default:
      RFATAL("Direction index must be [0...2], but is %d", dir);
  }

}

/*******************************************************************************
 * See header.
 ******************************************************************************/
double Mat3d_maxAbsEle(double A[3][3])
{
  double res = fabs(A[0][0]);

  for (unsigned int i = 0; i < 3; ++i)
  {
    for (unsigned int j = 0; j < 3; ++j)
    {
      double ele_ij = fabs(A[i][j]);
      if (ele_ij > res)
      {
        res = ele_ij;
      }
    }
  }

  return res;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
double Mat3d_getFrobeniusnorm(double A[3][3])
{
  return sqrt(A[0][0]*A[0][0] +
              A[0][1]*A[0][1] +
              A[0][2]*A[0][2] +
              A[1][0]*A[1][0] +
              A[1][1]*A[1][1] +
              A[1][2]*A[1][2] +
              A[2][0]*A[2][0] +
              A[2][1]*A[2][1] +
              A[2][2]*A[2][2]);
}

/*******************************************************************************
 * Test: A*v = lambda*v
 ******************************************************************************/
bool Mat3d_testEigenbasis(double A[3][3], double lambda[3], double V[3][3],
                          double eps)
{
  // Test 1: determinant is product of all eigenvalues
  double det = Mat3d_determinant(A);
  double err = fabs(lambda[0]*lambda[1]*lambda[2] - det);

  if (err > eps)
  {
    REXEC(4)
    {
      RMSG("Eigenvalues test failed - error is %g", err);
      Mat3d_printCommentDigits("A", A, 8);
      RMSG("Eigenvalues: %g %g %g", lambda[0], lambda[1], lambda[2]);
      RMSG("det(A) = %g   lambda0*lambda1*lambda2 = %g   err = %g",
           det, lambda[0]*lambda[1]*lambda[2], err);
    }
    return false;
  }

  // Test 2: A*v = lambda*v
  double ev_tp[3][3];
  Mat3d_transpose(ev_tp, V);

  for (unsigned int i=0; i<3; ++i)
  {
    double tmp1[3], tmp2[3];
    Vec3d_rotate(tmp1, A, ev_tp[i]);
    Vec3d_constMul(tmp2, ev_tp[i], lambda[i]);
    Vec3d_subSelf(tmp1, tmp2);
    if (Vec3d_getLength(tmp1)>eps)
    {
      REXEC(4)
      {
        RLOG(4, "Test A*v = lambda*v failed with err = %g )> %g)",
             Vec3d_getLength(tmp1), eps);
        RLOG(4, "lambda=%g", lambda[i]);
        Mat3d_printFormatted("A", "%g\t", A);
        Vec3d_printFormatted("v", "%g", ev_tp[i]);
      }
      return false;
    }
  }

  // Test 2: V is orthonormal and right-handed
  return Mat3d_isValid(V);
}

/*******************************************************************************
 *  \ingroup RcsMat3dFunctions
 *  \brief Sorts the Eigenvectors according to their Eigenvalue magnitudes.
 *         The Eigenvectors are assumed to be the columns of V. They will be
 *         modified to build a right-handed coordinate system.
 *
 *  \param[in,out] d Eigen values to be ordered
 *  \param[in,out] V Eigenvectors in columns of V, corresponding eigenvalues
 *                   in d
 *  \param[in] increasing If true, Eigenvectors and Eigenvalues will be
 *                        sorted in increasing order, otherwise in decreasing
 *                        order.
 ******************************************************************************/
static void Mat3d_sortEigenbasis(double lambda[3], double V[3][3],
                                 bool increasing)
{

  if (increasing == true)
  {
    if (lambda[0] > lambda[1])
    {
      Math_dSwap(&lambda[0], &lambda[1]);
      Math_dSwap(&V[0][0], &V[0][1]);
      Math_dSwap(&V[1][0], &V[1][1]);
      Math_dSwap(&V[2][0], &V[2][1]);
    }

    if (lambda[1] > lambda[2])
    {
      Math_dSwap(&lambda[1], &lambda[2]);
      Math_dSwap(&V[0][1], &V[0][2]);
      Math_dSwap(&V[1][1], &V[1][2]);
      Math_dSwap(&V[2][1], &V[2][2]);
    }

    if (lambda[0] > lambda[1])
    {
      Math_dSwap(&lambda[0], &lambda[1]);
      Math_dSwap(&V[0][0], &V[0][1]);
      Math_dSwap(&V[1][0], &V[1][1]);
      Math_dSwap(&V[2][0], &V[2][1]);
    }
  }
  else
  {
    if (lambda[0] < lambda[1])
    {
      Math_dSwap(&lambda[0], &lambda[1]);
      Math_dSwap(&V[0][0], &V[0][1]);
      Math_dSwap(&V[1][0], &V[1][1]);
      Math_dSwap(&V[2][0], &V[2][1]);
    }

    if (lambda[1] < lambda[2])
    {
      Math_dSwap(&lambda[1], &lambda[2]);
      Math_dSwap(&V[0][1], &V[0][2]);
      Math_dSwap(&V[1][1], &V[1][2]);
      Math_dSwap(&V[2][1], &V[2][2]);
    }

    if (lambda[0] < lambda[1])
    {
      Math_dSwap(&lambda[0], &lambda[1]);
      Math_dSwap(&V[0][0], &V[0][1]);
      Math_dSwap(&V[1][0], &V[1][1]);
      Math_dSwap(&V[2][0], &V[2][1]);
    }
  }

  // If the Eigenbasis is not right-handed, then the vector perpendicular on
  // the first two Eigenbasis components and the vector corresponding to the
  // third Eigenbasis component are anti-parallel. It means that their dot
  // product is -1 instead of +1. This is what we check and correct here.
  Mat3d_transposeSelf(V);
  double orthoVec[3];
  Vec3d_crossProduct(orthoVec, V[0], V[1]);

  if (Vec3d_innerProduct(orthoVec, V[2]) < 0.0)
  {
    Vec3d_constMulSelf(V[2], -1.0);
  }
  Mat3d_transposeSelf(V);
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Mat3d_compareEigenbasis(double lambda1_[3], double ev1_[3][3],
                             double lambda2_[3], double ev2_[3][3],
                             double eps)
{
  double lambda1[3], ev1[3][3], lambda2[3], ev2[3][3];

  Vec3d_copy(lambda1, lambda1_);
  Vec3d_copy(lambda2, lambda2_);
  Mat3d_copy(ev1, ev1_);
  Mat3d_copy(ev2, ev2_);

  Mat3d_sortEigenbasis(lambda1, ev1, true);
  Mat3d_sortEigenbasis(lambda2, ev2, true);

  // First we check if the Eigenvalues are equal
  if (!Vec3d_isEqual(lambda1, lambda2, eps))
  {
    RLOGS(4, "Mismatch in Eigenvalues:");
    Vec3d_printTwoVectorsDiff(lambda1, lambda2, 12);
    return false;
  }

  // If there is duplicate Eigenvalues, we skip the Eigenvector comparison,
  // since there are infinity possible solutions.
  if (Vec3d_multiplicity(lambda1, eps) != 1)
  {
    return true;
  }


  // Now we check if the Eigenvectors are parallel or anti-parallel
  Mat3d_transposeSelf(ev1);
  Mat3d_transposeSelf(ev2);

  for (unsigned int i=0; i<3; ++i)
  {
    Vec3d_normalizeSelf(ev1[i]);
    Vec3d_normalizeSelf(ev2[i]);
    double dotE = fabs(Vec3d_innerProduct(ev1[i], ev2[i]));
    double isParallel = fabs(dotE-1.0);

    if (isParallel > eps)
    {
      REXEC(4)
      {
        RMSGS("Eigenvectors %d are not parallel: %g", i, dotE);
        RMSGS("Angle between them is %g degrees",
              (180.0/M_PI)*Vec3d_diffAngle(ev1[i], ev2[i]));
        Vec3d_printTwoVectorsDiff(ev1[i], ev2[i], 8);
      }
      return false;
    }
  }

  return true;
}

/*******************************************************************************
 * Given a real symmetric 3x3 matrix A, compute the Eigenvector for a given
 * Eigenvalue.
 *
 * This function follows the explanations on
 * https://en.wikipedia.org/wiki/Eigenvalue_algorithm and
 * A Robust Eigensolver for 3 x 3 Symmetric Matrices (David Eberly,
 * Geometric Tools, LLC).
 *
 * In this implementation, it's not needed to check all combinations of the
 * matrix (A - lambda*I) to find linear independent vectors. If we
 * check 2 out of 3, then this set must contain a linear independent basis
 * vector.
 ******************************************************************************/
static bool calcEVi(double v[3], double A[3][3], double lambda)
{
  double AmlI[3][3], crossIJ[3][3], dotIJ[3], len = 0.0;

  Mat3d_transpose(AmlI, A);
  Mat3d_addConstToDiag(AmlI, -lambda);

  Vec3d_crossProduct(crossIJ[0], AmlI[0], AmlI[1]);
  Vec3d_crossProduct(crossIJ[1], AmlI[0], AmlI[2]);
  Vec3d_crossProduct(crossIJ[2], AmlI[1], AmlI[2]);

  dotIJ[0] = Vec3d_innerProduct(crossIJ[0], crossIJ[0]);
  dotIJ[1] = Vec3d_innerProduct(crossIJ[1], crossIJ[1]);
  dotIJ[2] = Vec3d_innerProduct(crossIJ[2], crossIJ[2]);

  int maxDot = Vec3d_indexMax(dotIJ);
  len = Vec3d_normalize(v, crossIJ[maxDot]);

  if (len==0.0)
  {
    return false;
  }

  return true;
}

/*******************************************************************************
 * For symmetric 3x3 matrices, there is a simple solution to the Eigenvalue
 * problem by an affine transformation. The below code follows the
 * explanations from: https://en.wikipedia.org/wiki/Eigenvalue_algorithm
 ******************************************************************************/
bool Mat3d_getEigenVectors(double V[3][3], double lambda[3], double A[3][3])
{
  // If the Eigenbasis cannot be determined, it is an identity matrix and
  // Eigenvalues that are 0.
  Mat3d_setIdentity(V);
  Vec3d_setZero(lambda);

  if ((Mat3d_isZero(A)==true) || (Mat3d_isSymmetric(A, 1.0e-8)==false))
  {
    return false;
  }

  // If A is diagonal, it already is the Eigenbasis
  if (Mat3d_isDiagonal(A, 1.0e-8)==true)
  {
    Vec3d_set(lambda, A[0][0], A[1][1], A[2][2]);
    Mat3d_setIdentity(V);
    Mat3d_sortEigenbasis(lambda, V, true);
    return true;
  }

  // q = trace(A)/3
  double q = (A[0][0]+A[1][1]+A[2][2])/3.0;

  // p = sqrt(trace((A-qI)^2)/6)
  double AmqI[3][3], sqrAmqI[3][3];
  Mat3d_copy(AmqI, A);
  Mat3d_addConstToDiag(AmqI, -q);
  Mat3d_mul(sqrAmqI, AmqI, AmqI);
  double p = sqrt(Mat3d_trace(sqrAmqI)/6.0);

  // B = (A-qI)/p and phi = acos(0.5*det(B))/3
  Mat3d_constMulSelf(AmqI, 1.0/p);
  double phi = Math_acos(0.5*Mat3d_determinant(AmqI))/3.0;

  // beta = 2*cos(phi + 2*pi*k/3) with k=0,1,2
  double beta[3];
  beta[0] = 2.0*cos(phi);
  beta[1] = 2.0*cos(phi + (2.0*M_PI/3.0));
  beta[2] = 2.0*cos(phi + (4.0*M_PI/3.0));

  // Back-substitution: lambda = p*beta + q
  lambda[0] = p*beta[0] + q;
  lambda[1] = p*beta[1] + q;
  lambda[2] = p*beta[2] + q;

  // From here on, we compute the Eigenvectors. We need to check for duplicate
  // Eigenvalues. For that case, we select the non-duplicate one and span an
  // orthogonal basis on it. If there are no duplicates, we select two
  // arbitrary eigenvalues and complete the third one according to the right
  // hand rule. The case of an Eigenvalue of multiplicy 3 is implicitely
  // handled: The basis is decided to be the identity matrix, and the
  // Eigenvalues have been computed above.
  unsigned int multiplicity = Vec3d_multiplicity(lambda, 1.0e-6);

  if (multiplicity==1) // 3 different Eigenvalues
  {
    calcEVi(V[0], A, lambda[0]);
    calcEVi(V[1], A, lambda[1]);
    Vec3d_crossProduct(V[2], V[0], V[1]);
  }
  else if (multiplicity==2) // 2 different Eigenvalues
  {
    if (fabs(lambda[0]-lambda[1])<1.0e-6)       // Duplicate x and y
    {
      calcEVi(beta, A, lambda[2]);
      Mat3d_fromVec(V, beta, 2);
    }
    else if (fabs(lambda[1]-lambda[2])<1.0e-6)  // Duplicate y and z
    {
      calcEVi(beta, A, lambda[0]);
      Mat3d_fromVec(V, beta, 0);
    }
    else if (fabs(lambda[0]-lambda[2])<1.0e-6)  // Duplicate x and z
    {
      calcEVi(beta, A, lambda[1]);
      Mat3d_fromVec(V, beta, 1);
    }
  }

  // Transpose to move Eigenvectors in columns, and sort in increasing order
  Mat3d_transposeSelf(V);
  Mat3d_sortEigenbasis(lambda, V, true);

  return Mat3d_testEigenbasis(A, lambda, V, 1.0e-6);
}
