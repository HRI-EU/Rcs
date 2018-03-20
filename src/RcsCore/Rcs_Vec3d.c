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

#include "Rcs_Vec3d.h"
#include "Rcs_Mat3d.h"
#include "Rcs_basicMath.h"
#include "Rcs_macros.h"


static double zeroVec[3]  = { 0.0, 0.0, 0.0 };
static double unitVecX[3] = { 1.0, 0.0, 0.0 };
static double unitVecY[3] = { 0.0, 1.0, 0.0 };
static double unitVecZ[3] = { 0.0, 0.0, 1.0 };



/*******************************************************************************
 *
 ******************************************************************************/
void Vec3d_printTwoVectorsDiff(const double v1[3], const double v2[3],
                               int digits)
{
  char formatStr[64];

  snprintf(formatStr, 63, "%%+.%df ", digits);

  for (unsigned int i = 0; i < 3; i++)
  {
    fprintf(stderr, formatStr, v1[i]);

    fprintf(stderr, "   ");

    fprintf(stderr, formatStr, v2[i]);

    fprintf(stderr, "   ");

    fprintf(stderr, formatStr, v1[i] - v2[i]);

    fprintf(stderr, "\n");
  }
  fprintf(stderr, "\n");
}

/*******************************************************************************
 *
 ******************************************************************************/
void Vec3d_printFormatted(const char* text, const char* format,
                          const double self[3])
{
  if (text != NULL)
  {
    fprintf(stderr, "%s\n", text);
  }

  for (unsigned int i = 0; i < 3; ++i)
  {
    fprintf(stderr, format, self[i]);
    fprintf(stderr, "\n");
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void Vec3d_add(double v3[3], const double v1[3], const double v2[3])
{
  v3[0] = v1[0] + v2[0];
  v3[1] = v1[1] + v2[1];
  v3[2] = v1[2] + v2[2];
}

/*******************************************************************************
 *
 ******************************************************************************/
void Vec3d_addSelf(double v2[3], const double v1[3])
{
  v2[0] += v1[0];
  v2[1] += v1[1];
  v2[2] += v1[2];
}

/*******************************************************************************
 *
 ******************************************************************************/
double Vec3d_getPolarPhi(const double z[3])
{
  return Math_acos(z[2]);
}

/*******************************************************************************
 *
 ******************************************************************************/
double Vec3d_getPolarTheta(const double z[3])
{
  double xpr[3], r0, theta;

  Vec3d_set(xpr, z[0], z[1], 0.0);
  r0 = sqrt(xpr[0] * xpr[0] + xpr[1] * xpr[1]);

  if (Vec3d_getLength(xpr) > 1.0e-8) // Theta valid
  {
    theta = Math_dsign(z[1]) * Math_acos(z[0] / r0);
  }
  else
  {
    theta = 0.0;
  }

  return theta;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Vec3d_getPolarAngles(double polarAngles[2], const double z[3])
{
  polarAngles[0] = Vec3d_getPolarPhi(z);
  polarAngles[1] = Vec3d_getPolarTheta(z);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Vec3d_getPolarAxis(double axis[3], double phi, double theta)
{
  double sinPhi = sin(phi);
  axis[0] = cos(theta) * sinPhi;
  axis[1] = sin(theta) * sinPhi;
  axis[2] = cos(phi);
}

/*******************************************************************************
 * Returns the spherical velocity matrix dphi/domega. We handle the
 * denominator-zero cases with a simple clip.
 ******************************************************************************/
void Vec3d_getPolarVelocityMatrix(double H[2][3], const double r[3])
{
  double x = r[0];
  double y = r[1];
  double z = r[2];
  double r0 = sqrt(x * x + y * y);

  if (r0 < 1.0e-8)
  {
    r0 = 1.0e-8;
  }

  double sqr_r0 = r0 * r0;

  H[0][0] = -y / r0;
  H[0][1] =  x / r0;
  H[0][2] =  0.0;

  H[1][0] = -(x * z) / sqr_r0;
  H[1][1] = -(y * z) / sqr_r0;
  H[1][2] =  1.0;
}

/*******************************************************************************
 * Calculates the angular displacement about the x- and y-axis in order
 * to align the A_BI's z-axis (third row) with the desired Polar angles.
 ******************************************************************************/
void Vec3d_getPolarError(double om_xy[2], const double polarDes[2],
                         double A_BI[3][3])
{
  // Compute current and desired polar axes
  double phi, a_des[3], *a_curr = A_BI[2];
  Vec3d_getPolarAxis(a_des, polarDes[0], polarDes[1]);
  phi = Vec3d_diffAngle(a_curr, a_des);

  // Set feedback error to 0.0 and return if target axis is reached
  if (phi==0.0)
  {
    om_xy[0] = 0.0;
    om_xy[1] = 0.0;
    return;
  }

  // In the following, we construct a coordinate frame S with z-axis as
  // current z-axis, y-axis as the rotation axis that rotates the z-axis
  // onto the target axis, and x-axis according to the right hand rule.
  // In this frame, the rotation about the y-axis will move the z-axis
  // in the plane spanned by z-axis and target axis.
  double A_SR[3][3];
  Vec3d_copy(A_SR[2], a_curr);
  Vec3d_crossProduct(A_SR[1], a_curr, a_des);
  Vec3d_normalizeSelf(A_SR[1]);
  Vec3d_crossProduct(A_SR[0], A_SR[1], A_SR[2]);

  // Rotation matrix A_ES from the S-frame into the effector frame
  double A_ES[3][3];
  Mat3d_mulTranspose(A_ES, A_BI, A_SR);

  // In the S-frame, a rotation about the y-axis will move the
  // current polar axis towards the desired polar axis according
  // to the shortest distance on the unit sphere.
  double s_omega[3];
  Vec3d_set(s_omega, 0.0, phi, 0.0);

  // Here we rotate the angular velocity vector in the S-frame into the
  // effector frame and copy the x- and y-components into the result. We
  // then have the effector-fixed angular velocities.
  double e_omega[3];
  Vec3d_rotate(e_omega, A_ES, s_omega);

  // Assign x- and y-components
  om_xy[0] = e_omega[0];
  om_xy[1] = e_omega[1];
}

/*******************************************************************************
 * Calculates the angular displacement in order to align the A_BI's
 * direction-axis (direction-th row) with the desired Polar angles.
 * This is the general function for Vec3d_getPolarError().
 ******************************************************************************/
void Vec3d_getPolarErrorFromDirection(double om_xy[2],
                                      const double polarDes[2],
                                      double A_BI[3][3],
                                      int direction)
{
  RCHECK_MSG((direction>=0) && (direction<=2),
             "Direction index must be 0, 1 or 2, but is %d", direction);

  // Compute current and desired polar axes. We assume that the current axis
  // is of unit length.
  double a_des[3], *a_curr = A_BI[direction];
  Vec3d_getPolarAxis(a_des, polarDes[0], polarDes[1]);

  // Compute the intermediate angle between the axes.
  double cosPhi = Vec3d_innerProduct(a_curr, a_des);

  // In the following, we construct a coordinate frame S with z-axis as
  // current z-axis, y-axis as the rotation axis that rotates the z-axis
  // onto the target axis, and x-axis according to the right hand rule.
  // In this frame, the rotation about the y-axis will move the z-axis
  // in the plane spanned by z-axis and target axis.
  double A_SR[3][3];
  Vec3d_copy(A_SR[2], a_curr);
  Vec3d_crossProduct(A_SR[1], a_curr, a_des);

  // We need to make sure that the current and desired axis are not
  // parallel. This can be determined by checking the length of the
  // rotation axis A_SR[1].
  double lengthRot = Vec3d_getLength(A_SR[1]);

  // If the length is 0, the intermediate angle can be 0 or 180 degrees.
  if (lengthRot==0.0)
  {
    // It is 0 degrees if the cosine is 1. In this case, the polar angle
    // error is also 0.0
    if (cosPhi>0.0)
    {
      om_xy[0] = 0.0;
      om_xy[1] = 0.0;
      return;
    }
    // ... and 180 degrees if the cosine is -1. In this case, we can
    // choose any rotation axis that is perpendicular to a_curr (=a_des).
    else
    {
      // Create any vector perpendicular to a_curr. We simply take a row
      // of the rotation matrix A_BI that is not the direction index. It
      // therefore is already normalized.
      switch (direction)
      {
        case 0:
          Vec3d_crossProduct(A_SR[1], a_curr, A_BI[1]);
          break;
        case 1:
          Vec3d_crossProduct(A_SR[1], a_curr, A_BI[2]);
          break;
        case 2:
          Vec3d_crossProduct(A_SR[1], a_curr, A_BI[0]);
          break;
      }

    }
  }
  // Otherwise we simply normalize it.
  else
  {
    Vec3d_constMulSelf(A_SR[1], 1.0/lengthRot);
  }

  // Calculate x-axis according to right hand rule
  Vec3d_crossProduct(A_SR[0], A_SR[1], A_SR[2]);

  // Rotation matrix A_ES from the S-frame into the effector frame
  double A_ES[3][3];
  Mat3d_mulTranspose(A_ES, A_BI, A_SR);

  // In the S-frame, a rotation about the y-axis will move the
  // current polar axis towards the desired polar axis according
  // to the shortest distance on the unit sphere.
  double s_omega[3];
  double phi = Math_acos(cosPhi);
  Vec3d_set(s_omega, 0.0, phi, 0.0);

  // Here we rotate the angular velocity vector in the S-frame into the
  // effector frame and copy the x- and y-components into the result. We
  // then have the effector-fixed angular velocities.
  double e_omega[3];
  Vec3d_rotate(e_omega, A_ES, s_omega);

  // Assign angular velocity components according to the direction vector.
  switch (direction)
  {
    case 0: // X rotation is free: copy y and z components
      om_xy[0] = e_omega[1];
      om_xy[1] = e_omega[2];
      break;
    case 1: // Y rotation is free: copy x and z components
      om_xy[0] = e_omega[0];
      om_xy[1] = e_omega[2];
      break;
    case 2: // Z rotation is free: copy x and y components
      om_xy[0] = e_omega[0];
      om_xy[1] = e_omega[1];
      break;
  }
}

/*******************************************************************************
 * K_r = A_KI I_r
 ******************************************************************************/
void Vec3d_rotate(double K_r[3], double A_KI[3][3], const double I_r[3])
{
  K_r[0] = A_KI[0][0] * I_r[0] + A_KI[0][1] * I_r[1] + A_KI[0][2] * I_r[2];
  K_r[1] = A_KI[1][0] * I_r[0] + A_KI[1][1] * I_r[1] + A_KI[1][2] * I_r[2];
  K_r[2] = A_KI[2][0] * I_r[0] + A_KI[2][1] * I_r[1] + A_KI[2][2] * I_r[2];
}

/*******************************************************************************
 * K_r = A_KI * I_r
 ******************************************************************************/
void Vec3d_rotateSelf(double vec[3], double A_KI[3][3])
{
  double tmp[3];
  Vec3d_rotate(tmp, A_KI, vec);
  Vec3d_copy(vec, tmp);
}

/*******************************************************************************
 * I_r = transpose(A_KI) * K_r
 ******************************************************************************/
void Vec3d_transRotate(double I_r[3], double A_KI[3][3], const double K_r[3])
{
  I_r[0] = A_KI[0][0] * K_r[0] + A_KI[1][0] * K_r[1] + A_KI[2][0] * K_r[2];
  I_r[1] = A_KI[0][1] * K_r[0] + A_KI[1][1] * K_r[1] + A_KI[2][1] * K_r[2];
  I_r[2] = A_KI[0][2] * K_r[0] + A_KI[1][2] * K_r[1] + A_KI[2][2] * K_r[2];
}

/*******************************************************************************
 * I_r = transpose(A_KI) * K_r
 ******************************************************************************/
void Vec3d_transRotateSelf(double vec[3], double A_KI[3][3])
{
  double tmp[3];
  Vec3d_transRotate(tmp, A_KI, vec);
  Vec3d_copy(vec, tmp);
}

/*******************************************************************************
 * The rotation matrix is constructed according to the axis-angle
 * representation, see e.g. http://en.wikipedia.org/wiki/Rotation_matrix
 *                                 #Rotation_matrix_from_axis_and_angle
 ******************************************************************************/
void Vec3d_rotateAboutAxis(double erg[3],
                           const double vec[3],
                           const double axis[3],
                           const double angle)
{
  double A[3][3];
  Mat3d_fromAxisAngle(A, axis, angle);
  Vec3d_rotate(erg, A, vec);
}

/*******************************************************************************
 *
 ******************************************************************************/
double Vec3d_distance(const double* p1, const double* p2)
{
  return sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) +
              (p1[1] - p2[1]) * (p1[1] - p2[1]) +
              (p1[2] - p2[2]) * (p1[2] - p2[2]));
}

/*******************************************************************************
 *
 ******************************************************************************/
double Vec3d_sqrDistance(const double* p1, const double* p2)
{
  return ((p1[0] - p2[0]) * (p1[0] - p2[0]) +
          (p1[1] - p2[1]) * (p1[1] - p2[1]) +
          (p1[2] - p2[2]) * (p1[2] - p2[2]));
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Vec3d_isEqual(const double v1[3], const double v2[3], double eps)
{
  if (fabs(v1[0] - v2[0]) > eps)
  {
    return false;
  }

  if (fabs(v1[1] - v2[1]) > eps)
  {
    return false;
  }

  if (fabs(v1[2] - v2[2]) > eps)
  {
    return false;
  }

  return true;
}

/*******************************************************************************
 * Returns false if any element is not finite, true otherwise.
 ******************************************************************************/
bool Vec3d_isFinite(const double vec[3])
{
  int i;

  for (i = 0; i < 3; i++)
  {
    if (Math_isFinite(vec[i])==0)
    {
      return false;
    }
  }

  return true;
}

/*******************************************************************************
 * Transforms vector I_r into the coordinates of the K-frame:
 * k_r_kp = A_KI * (I_r_Ip - I_r_Ik)
 ******************************************************************************/
void Vec3d_invTransform(double k_r[3], const HTr* A_KI, const double I_r[3])
{
  double I_r_IK[3];
  Vec3d_sub(I_r_IK, I_r, A_KI->org);
  Vec3d_rotate(k_r, (double(*)[3]) A_KI->rot, I_r_IK);
}

/*******************************************************************************
 * See Vec3d_invTransform()
 ******************************************************************************/
void Vec3d_invTransformSelf(double I_r[3], const HTr* A_KI)
{
  double K_r[3];
  Vec3d_sub(K_r, I_r, A_KI->org);
  Vec3d_rotate(I_r, (double(*)[3]) A_KI->rot, K_r);
}

/*******************************************************************************
 * I_r_Ip = I_r_K  + (A_KI)^T * K_r_kp
 ******************************************************************************/
void Vec3d_transform(double I_r[3], const HTr* A_KI, const double K_r[3])
{
  Vec3d_transRotate(I_r, (double(*)[3]) A_KI->rot, K_r);   // I_r_kp
  Vec3d_addSelf(I_r, A_KI->org);                           // I_r_kp + I_r_K
}

/*******************************************************************************
 * I_r_Ip = I_r  + (A_KI)^T * K_r_kp
 ******************************************************************************/
void Vec3d_transformSelf(double I_r[3], const HTr* A_KI)
{
  Vec3d_transRotateSelf(I_r, (double(*)[3]) A_KI->rot);   // I_r_kp
  Vec3d_addSelf(I_r, A_KI->org);                          // I_r_kp + I_r_K
}

/*******************************************************************************
 *
 ******************************************************************************/
void Vec3d_crossProduct(double v3[3], const double v1[3], const double v2[3])
{
  v3[0] = v1[1] * v2[2] - v2[1] * v1[2];
  v3[1] = - (v1[0] * v2[2] - v2[0] * v1[2]);
  v3[2] = v1[0] * v2[1] - v2[0] * v1[1];
}

/*******************************************************************************
 *
 ******************************************************************************/
void Vec3d_constMul(double v2[3], const double v1[3], double c)
{
  v2[0] = v1[0] * c;
  v2[1] = v1[1] * c;
  v2[2] = v1[2] * c;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Vec3d_constAdd(double v2[3], const double v1[3], double c)
{
  v2[0] = v1[0] + c;
  v2[1] = v1[1] + c;
  v2[2] = v1[2] + c;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Vec3d_constMulAndAdd(double v3[3], const double v1[3],
                          const double v2[3], double c)
{
  v3[0] = v1[0] + v2[0] * c;
  v3[1] = v1[1] + v2[1] * c;
  v3[2] = v1[2] + v2[2] * c;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Vec3d_constMulAndAddSelf(double v2[3], const double v1[3], double c)
{
  v2[0] += v1[0] * c;
  v2[1] += v1[1] * c;
  v2[2] += v1[2] * c;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Vec3d_addAndConstMul(double v3[3], const double v1[3],
                          const double v2[3], double c)
{
  v3[0] = (v1[0] + v2[0]) * c;
  v3[1] = (v1[1] + v2[1]) * c;
  v3[2] = (v1[2] + v2[2]) * c;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Vec3d_constMulSelf(double v[3], double c)
{
  v[0] *= c;
  v[1] *= c;
  v[2] *= c;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Vec3d_constAddSelf(double v[3], double c)
{
  v[0] += c;
  v[1] += c;
  v[2] += c;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Vec3d_sub(double v3[3], const double v1[3], const double v2[3])
{
  v3[0] = v1[0] - v2[0];
  v3[1] = v1[1] - v2[1];
  v3[2] = v1[2] - v2[2];
}

/*******************************************************************************
 *
 ******************************************************************************/
void Vec3d_subSelf(double v2[3], const double v1[3])
{
  v2[0] -= v1[0];
  v2[1] -= v1[1];
  v2[2] -= v1[2];
}

/*******************************************************************************
 *
 ******************************************************************************/
void Vec3d_set(double vector[3], double x, double y, double z)
{
  vector[0] = x;
  vector[1] = y;
  vector[2] = z;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Vec3d_setElementsTo(double vector[3], const double value)
{
  vector[0] = value;
  vector[1] = value;
  vector[2] = value;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Vec3d_setZero(double vector[3])
{
  vector[0] = 0.0;
  vector[1] = 0.0;
  vector[2] = 0.0;
}

/*******************************************************************************
 *
 ******************************************************************************/
double Vec3d_normalize(double dst[3], const double src[3])
{
  double len = sqrt(src[0]*src[0] + src[1]*src[1] + src[2]*src[2]);

  if (len > 0.0)
  {
    double c = 1.0 / len;
    dst[0] = src[0]*c;
    dst[1] = src[1]*c;
    dst[2] = src[2]*c;
  }

  return len;
}

/*******************************************************************************
 *
 ******************************************************************************/
double Vec3d_normalizeSelf(double vec[3])
{
  double len = sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);

  if (len > 0.0)
  {
    double c = 1.0 / len;
    vec[0] *= c;
    vec[1] *= c;
    vec[2] *= c;
  }

  return len;
}

/*******************************************************************************
 *
 ******************************************************************************/
double Vec3d_diffAngle(const double v1[3], const double v2[3])
{
  double lv1 = Vec3d_getLength(v1);
  double lv2 = Vec3d_getLength(v2);

  if ((lv1 == 0.0) || (lv2 == 0.0))
  {
    return 0.0;
  }

  return Math_acos(Vec3d_innerProduct(v1, v2)/(lv1*lv2));
}

/*******************************************************************************
 *
 ******************************************************************************/
void Vec3d_copy(double dst[3], const double src[3])
{
  dst[0] = src[0];
  dst[1] = src[1];
  dst[2] = src[2];
}

/*******************************************************************************
 *
 ******************************************************************************/
double Vec3d_innerProduct(const double v1[3], const double v2[3])
{
  return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
}

/*******************************************************************************
 *
 ******************************************************************************/
void Vec3d_outerProduct(double A[3][3], const double u[3], const double v[3])
{
  A[0][0] = u[0]*v[0];
  A[0][1] = u[0]*v[1];
  A[0][2] = u[0]*v[2];

  A[1][0] = u[1]*v[0];
  A[1][1] = u[1]*v[1];
  A[1][2] = u[1]*v[2];

  A[2][0] = u[2]*v[0];
  A[2][1] = u[2]*v[1];
  A[2][2] = u[2]*v[2];
}

/*******************************************************************************
 *
 ******************************************************************************/
void Vec3d_outerProductSelf(double A[3][3], const double u[3])
{
  A[0][0] = u[0]*u[0];
  A[0][1] = u[0]*u[1];
  A[0][2] = u[0]*u[2];

  A[1][0] = A[0][1];
  A[1][1] = u[1]*u[1];
  A[1][2] = u[1]*u[2];

  A[2][0] = A[0][2];
  A[2][1] = A[1][2];
  A[2][2] = u[2]*u[2];
}

/*******************************************************************************
 *
 ******************************************************************************/
double Vec3d_getLength(const double v[3])
{
  return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

/*******************************************************************************
 *
 ******************************************************************************/
double Vec3d_sqrLength(const double v[3])
{
  return v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
}

/*******************************************************************************
 *
 * Computes the intersection point between a line and a plane. For
 * this, we solve the following equation system:
 *
 * r_line + lambda*v = r_plane + a*e_px + b*e_py
 *
 * with: r_line       = vector to line start point
 *       v            = line direction vector (not required normalized)
 *       r_plane      = a point on the plane
 *       e_px         = direction vector in plane
 *       e_py         = perpendicular to e_px, also in plane
 *       lambda, a, b = parameters to solve for:
 *
 * (lambda a b)^T = (-v e_px e_py)^-1 (r_line-r_plane)
 *
 * The intersection point is computed as
 *
 *       p = r_line + lambda*v
 *
 ******************************************************************************/
bool Vec3d_computePlaneLineIntersection(double intersection[3],
                                        const double linePt[3],
                                        const double lineDir[3],
                                        const double planePt[3],
                                        const double planeNormal[3])
{
  int i;
  double det, eqs[3][3], eqs_inv[3][3], param[3], dr[3], plane[3][3];

  // Compute two axes that lie within the plane. The normal direction is
  // the z-axis, the x- and y elements of the transform are within the plane.
  Mat3d_fromVec(plane, planeNormal, 2);

  // Solve the linear equation system for the search parameters lambda, a, b
  for (i = 0; i < 3; i++)
  {
    eqs[i][0] = -lineDir[i];
    eqs[i][1] =  plane[0][i];
    eqs[i][2] =  plane[1][i];
  }

  det = Mat3d_inverse(eqs_inv, eqs);

  // No intersection
  if (det == 0.0)
  {
    return false;
  }

  // Compute the parameter vector param = (lambda a b)^T
  Vec3d_sub(dr, linePt, planePt);
  Vec3d_rotate(param, eqs_inv, dr);

  // intersection = linePt + lambda*lineDir
  Vec3d_constMulAndAdd(intersection, linePt, lineDir, param[0]);

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Vec3d_slerp(double c[3], const double a[3], const double b[3], double t)
{
  double A_1I[3][3];
  double A_2I[3][3];
  double A[3][3];

  Mat3d_fromEulerAngles(A_1I, a);
  Mat3d_fromEulerAngles(A_2I, b);
  Mat3d_slerp(A, A_1I, A_2I, t);
  Mat3d_toEulerAngles(c, A);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Vec3d_lerp(double c[3], const double a[3], const double b[3], double t)
{
  for (int i=0; i<3; ++i)
  {
    c[i] = (1.0-t)*a[i] + t*b[i];
  }
}

/*******************************************************************************
 * I_r_IV += (A_VI)^T V_r_VK
 ******************************************************************************/
void Vec3d_transMulAndAddSelf(double I_r_IP[3], double A_KI[3][3],
                              const double K_r_KP[3])
{
  double I_r_KP[3];
  Vec3d_transRotate(I_r_KP, A_KI, K_r_KP);  // I_r_KP = (A_KI)^T * K_r_KP
  Vec3d_addSelf(I_r_IP, I_r_KP);            // I_r_IP = I_r_KP + I_r_IK
}

/*******************************************************************************
 * I_r_IK = I_r_IV + (A_VI)^T V_r_VK
 ******************************************************************************/
void Vec3d_transMulAndAdd(double I_r_IK[3], const double I_r_IV[3],
                          double A_VI[3][3], const double V_r_VK[3])
{
  double I_r_VK[3];
  Vec3d_transRotate(I_r_VK, A_VI, V_r_VK);  // I_r_VK = (A_VI)^T * V_r_VK
  Vec3d_add(I_r_IK, I_r_IV, I_r_VK);        // I_r_IK = I_r_IV + I_r_VK
}

/*******************************************************************************
 * If vec's length is 0, (0 0 1) will be copied to ortho. If vec is not aligned
 * with (0 0 1), then the normalized outer product vector with (0 0 1) is
 * copied to ortho. If vec is aligned with (0 0 1), then the normalized outer
 * product vector with (1 0 0) is copied to ortho.
 *
 * Here's a pedantic check, which can be added to the end of the function:
 *
 * REXEC(0)
 * {
 *   double diffAng = Vec3d_diffAngle(vec, ortho);
 *   if (fabs(diffAng) - M_PI_2 > 1.0e-8)
 *   {
 *     RMSG("vec is       [%g   %g   %g]", vec[0], vec[1], vec[2]);
 *     RMSG("norm(vec) is [%g   %g   %g]",
 *          normalizedVec[0], normalizedVec[1], normalizedVec[2]);
 *     RMSG("tmp is       [%g   %g   %g]", tmp[0], tmp[1], tmp[2]);
 *     RMSG("ortho is     [%g   %g   %g]", ortho[0], ortho[1], ortho[2]);
 *     RFATAL("Delta angle is %.16f, but must be 90.0 degrees",
 *            diffAng * (180. / M_PI));
 *   }
 * }
 ******************************************************************************/
void Vec3d_orthonormalVec(double ortho[3], const double vec[3])
{
  double cosPhi, tmp[3], sqrLength, normalizedVec[3];

  // If the squared length of vec is 0, we copy the (1 0 0) vector to ortho.
  sqrLength = vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2];

  if (sqrLength == 0.0)
  {
    Vec3d_set(ortho, 1.0, 0.0, 0.0);
    return;
  }

  // We take the z-direction and compute the angle between it and the
  // normalized vec. The resulting vector tmp will be in the x-y plane.
  Vec3d_constMul(normalizedVec, vec, 1.0 / sqrt(sqrLength));
  Vec3d_set(tmp, 0.0, 0.0, 1.0);
  cosPhi = Vec3d_innerProduct(normalizedVec, tmp);

  // In the unlikely case that vec coincides with the z-axis, we take the x-
  // axis. This must work, because vec cannot coincide with the x- and z-axis
  // at the same time.
  if (cosPhi * cosPhi > 0.99) // closer than approx. 5.7 deg
  {
    Vec3d_set(tmp, 1.0, 0.0, 0.0);
  }

  Vec3d_crossProduct(ortho, normalizedVec, tmp);
  Vec3d_normalizeSelf(ortho);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Vec3d_setRandom(double vec[3], double lower, double upper)
{
  vec[0] = Math_getRandomNumber(lower, upper);
  vec[1] = Math_getRandomNumber(lower, upper);
  vec[2] = Math_getRandomNumber(lower, upper);
}

/*******************************************************************************
 * Following the explanations of
 * http://www.flipcode.com/archives/Random_Unit_Vectors.shtml :
 * 1. Create a plane with random height within [-1 ; 1] intersecting a
 *    unit sphere leading to a circle
 * 2. Creating a random angle [0 ; 360 deg] around this circle
 * 3. This point is the tip of a random unit vector.
 ******************************************************************************/
void Vec3d_setRandomUnitVector(double vec[3])
{
  vec[2] = Math_getRandomNumber(-1.0, 1.0);

  double a = Math_getRandomNumber(0.0, 2.0*M_PI);
  double r = sqrt(1.0 - vec[2]*vec[2]);

  vec[0] = r * cos(a);
  vec[1] = r * sin(a);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Vec3d_setUnitVector(double vec[3], int dir)
{
  switch (dir)
  {
    case 0:
      Vec3d_set(vec, 1.0, 0.0, 0.0);
      break;
    case 1:
      Vec3d_set(vec, 0.0, 1.0, 0.0);
      break;
    case 2:
      Vec3d_set(vec, 0.0, 0.0, 1.0);
      break;
    default:
      RFATAL("Direction must be within [0:2], but is %d", dir);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
const double* Vec3d_zeroVec()
{
  return &zeroVec[0];
}

/*******************************************************************************
 *
 ******************************************************************************/
const double* Vec3d_ex()
{
  return &unitVecX[0];
}

/*******************************************************************************
 *
 ******************************************************************************/
const double* Vec3d_ey()
{
  return &unitVecY[0];
}

/*******************************************************************************
 *
 ******************************************************************************/
const double* Vec3d_ez()
{
  return &unitVecZ[0];
}

/*******************************************************************************
 *
 ******************************************************************************/
const double* Vec3d_unitVector(int dim)
{
  switch (dim)
  {
    case 0:
      return unitVecX;
      break;
    case 1:
      return unitVecY;
      break;
    case 2:
      return unitVecZ;
      break;
    default:
      RFATAL("Direction must be within [0:2], but is %d", dim);
  }

  return NULL;
}

/*******************************************************************************
 *
 ******************************************************************************/
double Vec3d_constSaturateSelf(double* self, const double limit)
{
  if (limit <= 0.0)
  {
    Vec3d_setZero(self);
    return 0.0;
  }

  double scale = 1.0;
  double len = Vec3d_getLength(self);

  if (len > limit)
  {
    scale = limit / len;
    self[0] *= scale;
    self[1] *= scale;
    self[2] *= scale;
  }

  return scale;
}

/*******************************************************************************
 *
 ******************************************************************************/
int Vec3d_indexMax(const double x[3])
{
  unsigned int res = 0;

  if (x[1] > x[res])
  {
    res = 1;
  }

  if (x[2] > x[res])
  {
    res = 2;
  }

  return res;
}

/*******************************************************************************
 *
 ******************************************************************************/
unsigned int Vec3d_multiplicity(const double v[3], double eps)
{
  unsigned int multiplicity = 1;

  if (fabs(v[0]-v[1])<eps)   // 0 and 1 are the same
  {
    if (fabs(v[0]-v[2])<eps)   // 0, 1 and 2 are the same
    {
      multiplicity = 3;
    }
    else   // 0 and 1 are the same, 2 is different
    {
      multiplicity = 2;
    }
  }
  else   // 0 and 1 are different
  {
    if (fabs(v[0]-v[2])<eps)   // 0 and 2 are the same, 1 is different
    {
      multiplicity = 2;
    }
    else
    {
      multiplicity = 1;   // 0, 1 and 2 are different
    }
  }

  return multiplicity;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Vec3d_swap(double v1[3], double v2[3])
{
  Math_dSwap(&v1[0], &v2[0]);
  Math_dSwap(&v1[1], &v2[1]);
  Math_dSwap(&v1[2], &v2[2]);
}
