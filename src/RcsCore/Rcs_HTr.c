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

#include "Rcs_math.h"
#include "Rcs_macros.h"
#include "Rcs_utils.h"

#include <float.h>


#define HTR_TOSTRING_MAXSIZE (12 * 30)


static HTr Identity = { {0., 0., 0.} ,
  { {1., 0., 0.}, {0., 1., 0.}, {0., 0., 1.} }
};

/*******************************************************************************
 *
 ******************************************************************************/
HTr* HTr_create()
{
  HTr* self = RALLOC(HTr);
  RCHECK(self);
  HTr_setIdentity(self);
  return self;
}

/*******************************************************************************
 *
 ******************************************************************************/
HTr* HTr_clone(const HTr* src)
{
  HTr* self;

  if (src == NULL)
  {
    return NULL;
  }

  self = RALLOC(HTr);
  RCHECK(self);
  HTr_copy(self, src);
  return self;
}

/*******************************************************************************
 *
 ******************************************************************************/
void HTr_copy(HTr* dst, const HTr* src)
{
  memmove(dst, src, sizeof(HTr));
}

/*******************************************************************************
 *
 ******************************************************************************/
void HTr_transpose(HTr* A_12, const HTr* A_21)
{
  Mat3d_transpose(A_12->rot, (double (*)[3]) A_21->rot);

  Vec3d_setZero(A_12->org);
  Vec3d_subSelf(A_12->org, A_21->org);
  Vec3d_rotateSelf(A_12->org, (double (*)[3]) A_21->rot);
}

/*******************************************************************************
 *
 ******************************************************************************/
void HTr_setIdentity(HTr* self)
{
  self->org[0] =  0.0;
  self->org[1] =  0.0;
  self->org[2] =  0.0;

  Mat3d_setIdentity(self->rot);
}

/*******************************************************************************
 *
 ******************************************************************************/
void HTr_setZero(HTr* self)
{
  memset(self, 0, 12 * sizeof(double));
}

/*******************************************************************************
 *
 ******************************************************************************/
bool HTr_isValid(const HTr* A)
{
  int i;
  bool isValid = false;

  if (A == NULL)
  {
    return false;
  }

  isValid = Mat3d_isValid((double(*)[3]) A->rot);

  if (isValid == false)
  {
    return false;
  }

  for (i = 0; i < 3; i++)
  {
    if (!Math_isFinite(A->org[i]))
    {
      RLOG(4, "A->org[%d] not finite: %g", i, A->org[i]);
      return false;
    }

  }

  return isValid;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool HTr_isIdentity(const HTr* A)
{
  int i;

  for (i = 0; i < 3; i++)
  {
    if (A->org[i] != 0.0)
    {
      return false;
    }
  }

  return Mat3d_isIdentity((double(*)[3]) A->rot);
}

/*******************************************************************************
 *
 ******************************************************************************/
void HTr_slerp(HTr* A, const HTr* A_1I, const HTr* A_2I, double t)
{
  int i;

  RCHECK(HTr_isValid(A_1I));
  RCHECK(HTr_isValid(A_2I));

  // Rotation part
  Mat3d_slerp(A->rot, (double(*)[3]) A_1I->rot, (double(*)[3]) A_2I->rot, t);

  // Translation part
  for (i = 0; i < 3; i++)
  {
    A->org[i] = A_1I->org[i] + t * (A_2I->org[i] - A_1I->org[i]);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void HTr_fprint(FILE* fd, const HTr* A)
{
  if (A == NULL)
  {
    fprintf(fd, "HTr is NULL");
    return;
  }

  fprintf(fd, "\t%.6f\t%.6f\t%.6f\tr\t%.6f\n",
          A->rot[0][0], A->rot[0][1], A->rot[0][2], A->org[0]);
  fprintf(fd, "\t%.6f\t%.6f\t%.6f\t\t%.6f\n",
          A->rot[1][0], A->rot[1][1], A->rot[1][2], A->org[1]);
  fprintf(fd, "\t%.6f\t%.6f\t%.6f\t\t%.6f\n",
          A->rot[2][0], A->rot[2][1], A->rot[2][2], A->org[2]);
}

/*******************************************************************************
 * Transformation from body to world coordinates
 *
 * A_2I   = A_21 * A_1I
 * I_r_2 = I_r_1 + A_I1 * 1_r_12
 ******************************************************************************/
void HTr_transform(HTr* A_2I, const HTr* A_1I, const HTr* A_21)
{
  Mat3d_mul(A_2I->rot, (double(*)[3]) A_21->rot, (double(*)[3]) A_1I->rot);
  Vec3d_transform(A_2I->org, A_1I, A_21->org);
}

/*******************************************************************************
 * See HTr_transform()
 ******************************************************************************/
void HTr_transformSelf(HTr* A_2I /* in as A_1I */, const HTr* A_21)
{
  HTr A_1I;
  HTr_copy(&A_1I, A_2I);
  HTr_transform(A_2I, &A_1I, A_21);
}

/*******************************************************************************
 * A_21   = A_2I * (A_1I)^T
 * 1_r_12 = A_1I * (I_r_I2 - I_r_I1)
 ******************************************************************************/
void HTr_invTransform(HTr* A_21, const HTr* A_1I, const HTr* A_2I)
{
  Mat3d_mulTranspose(A_21->rot, (double(*)[3]) A_2I->rot,
                     (double(*)[3]) A_1I->rot);
  Vec3d_invTransform(A_21->org, A_1I, A_2I->org);
}

/*******************************************************************************
 *
 ******************************************************************************/
const HTr* HTr_identity()
{
  return &Identity;
}

/*******************************************************************************
 *
 ******************************************************************************/
void HTr_toString(char* str, const HTr* A)
{
  RCHECK(str);
  RCHECK(A);

  char buf[12][32];
  const unsigned int maxDigits = 8;

  snprintf(str, HTR_TOSTRING_MAXSIZE,
           "%31s %31s %31s %31s %31s %31s %31s %31s %31s %31s %31s %31s",
           String_fromDouble(buf[0], A->org[0], maxDigits),
           String_fromDouble(buf[1], A->org[1], maxDigits),
           String_fromDouble(buf[2], A->org[2], maxDigits),
           String_fromDouble(buf[3], A->rot[0][0], maxDigits),
           String_fromDouble(buf[4], A->rot[0][1], maxDigits),
           String_fromDouble(buf[5], A->rot[0][2], maxDigits),
           String_fromDouble(buf[6], A->rot[1][0], maxDigits),
           String_fromDouble(buf[7], A->rot[1][1], maxDigits),
           String_fromDouble(buf[8], A->rot[1][2], maxDigits),
           String_fromDouble(buf[9], A->rot[2][0], maxDigits),
           String_fromDouble(buf[10], A->rot[2][1], maxDigits),
           String_fromDouble(buf[11], A->rot[2][2], maxDigits));
}

/*******************************************************************************
 *
 ******************************************************************************/
bool HTr_fromString(HTr* A, const char* str)
{
  if ((A == NULL) || (str == NULL))
  {
    return false;
  }

  double x[12];

  bool success = String_toDoubleArray_l(str, x, 12);

  if (success == true)
  {
    Vec3d_copy(A->org, &x[0]);
    Vec3d_copy(A->rot[0], &x[3]);
    Vec3d_copy(A->rot[1], &x[6]);
    Vec3d_copy(A->rot[2], &x[9]);
  }

  return success;
}

/*******************************************************************************
 * Constructs a HTr from 2 points, so that p1-p2 will become the unit z-axis
 * of the rotation matrix. The x- and y-axis are computed just somehow to be
 * orthogonal. The HTrs' origin will be set to p1.
 ******************************************************************************/
void HTr_from2Points(HTr* A_KI, const double p1[3], const double p2[3])
{
  double tmp[3], length;
  double* ex = A_KI->rot[0], *ey = A_KI->rot[1], *ez = A_KI->rot[2];
  Vec3d_set(A_KI->org, p1[0], p1[1], p1[2]);

  // Create the unit z-axis from p1 to p2
  Vec3d_sub(ez, p2, p1);
  length = Vec3d_normalizeSelf(ez);

  // If points coincide, we choose the identity matrix
  if (length==0.0)
  {
    Mat3d_setIdentity(A_KI->rot);
    return;
  }

  // Determine the y-axis  to be orthogonal to ez and tmp.
  Vec3d_set(tmp, 0.0, 0.0, 1.0);

  // In the (unlikely) case that ez and tmp almost coincide we switch to
  // a different rotation axis: the x-axis
  if ((fabs(Vec3d_diffAngle(ez,tmp))<1.0e-5) ||
      (fabs(Vec3d_diffAngle(ez,tmp)-M_PI)<1.0e-5))
  {
    Vec3d_set(tmp, 1.0 , 0.0 , 0.0);
  }

  Vec3d_crossProduct(ey, ez, tmp);
  Vec3d_normalizeSelf(ey);
  Vec3d_crossProduct(ex, ey, ez);
  Vec3d_normalizeSelf(ex);

  return;
}

/*******************************************************************************
 *
 ******************************************************************************/
void HTr_toVector(double vec[12], const HTr* A_KI)
{
  RCHECK(vec);
  RCHECK(A_KI);

  Vec3d_copy(&vec[0], A_KI->org);
  vec[3] = A_KI->rot[0][0];
  vec[4] = A_KI->rot[0][1];
  vec[5] = A_KI->rot[0][2];
  vec[6] = A_KI->rot[1][0];
  vec[7] = A_KI->rot[1][1];
  vec[8] = A_KI->rot[1][2];
  vec[9] = A_KI->rot[2][0];
  vec[10] = A_KI->rot[2][1];
  vec[11] = A_KI->rot[2][2];
}

/*******************************************************************************
 *
 ******************************************************************************/
void HTr_fromVector(HTr* A_KI, const double vec[12])
{
  RCHECK(vec);
  RCHECK(A_KI);

  Vec3d_copy(A_KI->org, &vec[0]);
  A_KI->rot[0][0] = vec[3];
  A_KI->rot[0][1] = vec[4];
  A_KI->rot[0][2] = vec[5];
  A_KI->rot[1][0] = vec[6];
  A_KI->rot[1][1] = vec[7];
  A_KI->rot[1][2] = vec[8];
  A_KI->rot[2][0] = vec[9];
  A_KI->rot[2][1] = vec[10];
  A_KI->rot[2][2] = vec[11];
}

/*******************************************************************************
 *
 ******************************************************************************/
void HTr_from6DVector(HTr* A, const double x[6])
{
  RCHECK(A);
  RCHECK(x);

  Vec3d_copy(A->org, &x[0]);
  Mat3d_fromEulerAngles(A->rot, &x[3]);
}

/*******************************************************************************
 *
 ******************************************************************************/
void HTr_to6DVector(double x[6], const HTr* A)
{
  RCHECK(x);
  RCHECK(A);

  Vec3d_copy(&x[0], A->org);
  Mat3d_toEulerAngles(&x[3], (double (*)[3]) A->rot);
}

/*******************************************************************************
 * Pole direction, see Springer Handbook of Robotics, pp. 16., table 1.4.
 ******************************************************************************/
bool HTr_computeChaslesPole(const HTr* A,
                            const HTr* A_prev,
                            double polePt[3],
                            double poleDirection[3],
                            double* v_pol,
                            double* om_pol)
{
  HTr A_21, A_12;
  const HTr* A_1I = A_prev, *A_2I = A;

  HTr_invTransform(&A_21, A_1I, A_2I);
  Mat3d_transpose(A_12.rot, A_21.rot);
  double l[3], *p = &A_21.org[0];

  l[0] = A_12.rot[2][1] - A_12.rot[1][2];
  l[1] = A_12.rot[0][2] - A_12.rot[2][0];
  l[2] = A_12.rot[1][0] - A_12.rot[0][1];

  // Rotation angle
  double phi = Math_dsign(Vec3d_innerProduct(l,p)) *
               fabs(Math_acos(0.5*(A_12.rot[0][0]+A_12.rot[1][1]+A_12.rot[2][2]-1.0)));

  double sinPhi = sin(phi);

  // No relative rotation - pole direction is undefined
  if (sinPhi == 0.0)
  {
    return false;
  }

  // Linear displacement
  double h = Vec3d_innerProduct(l,p)/(2.0*phi*sinPhi);

  // Point on pole axis (in body coordinates)
  double rho[3], EmRT[3][3], Id[3][3];
  Mat3d_setIdentity(Id);
  Mat3d_sub(EmRT, Id, A_21.rot);
  Vec3d_rotate(rho, EmRT, p);
  Vec3d_constMulSelf(rho, 1.0/(2.0*(1.0-cos(phi))));

  // Normalized rotation axis (in body coordinates)
  double om_hat[3];
  Vec3d_constMul(om_hat, l, 1.0/(2.0*sinPhi));

  // Normalization failed - pole direction is undefined
  if (fabs(Vec3d_sqrLength(om_hat)-1.0)>1.0e-8)
  {
    return false;
  }

  // Pole axis direction in body coordinates
  // Vec3d_copy(poleDirection, om_hat);

  // Pole axis direction in world coordinates
  Vec3d_transRotate(poleDirection, (double (*)[3]) A->rot, om_hat);

  // Point on pole axis in body coordinates
  // Vec3d_copy(polePt, rho);

  // Point on pole axis in world coordinates
  Vec3d_transform(polePt, A, rho);

  // Speed along pole axis
  if (v_pol)
  {
    *v_pol = h;
  }

  // Angular velocity around pole axis
  if (om_pol)
  {
    *om_pol = phi;
  }

  // Test: pole direction must be perpendicular to pole vector
  REXEC(5)
  {
    bool success = false;

    // Test: If we have both linear and angular body velocities, the pole
    //       direction must be perpendicular to pole vector
    if ((Vec3d_getLength(rho)>1.0e-8) && (Vec3d_getLength(om_hat)>1.0e-8))
    {
      double ang = Vec3d_diffAngle(om_hat, rho);
      success = fabs(ang-M_PI_2)<1.0e-6 ? true : false;
      RCHECK_MSG(success, "%s: angle between r_QP and pole direction is %.1f "
                 "(must be 90.0 deg)", success ? "SUCCESS" : "FAILURE",
                 (180.0/M_PI)*Vec3d_diffAngle(om_hat, rho));
    }

    // Test: If we have an angular body velocities, the pole direction vector
    //       must be of inut length.
    success = fabs(Vec3d_sqrLength(om_hat)-1.0)<1.0e-8 ? true : false;
    RCHECK_MSG(success, "%s: pole direction vector norm is %.8f "
               "(must be 1)", success ? "SUCCESS" : "FAILURE",
               Vec3d_sqrLength(om_hat));
  }

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
void HTr_copyOrRecreate(HTr** dst, const HTr* src)
{
  if (src==NULL)
  {
    if (*dst != NULL)
    {
      RFREE(*dst);
      *dst = NULL;
    }
    return;
  }

  if (*dst==NULL)
  {
    *dst = HTr_clone(src);
  }
  else
  {
    HTr_copy(*dst, src);
  }

}
