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

#include "Rcs_quaternion.h"
#include "Rcs_basicMath.h"
#include "Rcs_Mat3d.h"
#include "Rcs_VecNd.h"
#include "Rcs_macros.h"



/*******************************************************************************
 *
 ******************************************************************************/
void Quat_conjugate(double q_conj[4], const double q[4])
{
  q_conj[0] =  q[0];
  q_conj[1] = -q[1];
  q_conj[2] = -q[2];
  q_conj[3] = -q[3];
}

/*******************************************************************************
 *
 ******************************************************************************/
void Quat_conjugateSelf(double q[4])
{
  q[0] =  q[0];
  q[1] = -q[1];
  q[2] = -q[2];
  q[3] = -q[3];
}

/*******************************************************************************
 *
 ******************************************************************************/
void Quat_mul(double q[4], const double q1[4], const double q2[4])
{
  q[1] =  q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2] + q1[0]*q2[1];
  q[2] = -q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1] + q1[0]*q2[2];
  q[3] =  q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0] + q1[0]*q2[3];
  q[0] = -q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3] + q1[0]*q2[0];
}

/*******************************************************************************
 *
 ******************************************************************************/
double Quat_dot(const double q1[4], const double q2[4])
{
  return q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3];
}

/*******************************************************************************
 *
 ******************************************************************************/
double Quat_diffAngle(const double q1[4], const double q2[4])
{
  return Math_acos(Quat_dot(q1, q2));
}

/*******************************************************************************
 * Relative rotation between two quaternions from q1 to q2: q =  q1^-1 * q2
 ******************************************************************************/
void Quat_relativeRotation(double q[4], const double q1[4], const double q2[4])
{
  double q_inv1[4];
  Quat_conjugate(q_inv1, q1);
  Quat_mul(q, q_inv1, q2);
}

/*******************************************************************************
 *
 ******************************************************************************/
const double* Quat_identity()
{
  static double q_identity[4] = {1.0, 0.0, 0.0, 0.0};

  return q_identity;
}

/*******************************************************************************
 *
 ******************************************************************************/
double Quat_normalizeSelf(double q[4])
{
  double n = sqrt(Quat_dot(q, q));

  if (n > 0.0)
  {
    q[0] /= n;
    q[1] /= n;
    q[2] /= n;
    q[3] /= n;
    return n;
  }

  return n;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Quat_toRotationMatrix(double A_BI[3][3], const double q[4])
{
  const double qw = q[0];
  const double qx = q[1];
  const double qy = q[2];
  const double qz = q[3];

  A_BI[0][0] = 1.0 - 2.0f*qy*qy - 2.0f*qz*qz;
  A_BI[1][0] = 2.0*qx*qy - 2.0f*qz*qw;
  A_BI[2][0] = 2.0*qx*qz + 2.0f*qy*qw;

  A_BI[0][1] = 2.0*qx*qy + 2.0f*qz*qw;
  A_BI[1][1] = 1.0 - 2.0f*qx*qx - 2.0f*qz*qz;
  A_BI[2][1] = 2.0*qy*qz - 2.0f*qx*qw;

  A_BI[0][2] = 2.0*qx*qz - 2.0f*qy*qw;
  A_BI[1][2] = 2.0*qy*qz + 2.0f*qx*qw;
  A_BI[2][2] = 1.0 - 2.0f*qx*qx - 2.0f*qy*qy;
}

/*******************************************************************************
 * This function follows the divide-and-conquer strategy of the article
 * "Converting a Rotation Matrix to a Quaternion" by Mike Day, Insomniac Games.
 *
 *  qw = q[0]
 *  qx = q[1]
 *  qy = q[2]
 *  qz = q[3]
 ******************************************************************************/
bool Quat_fromRotationMatrix(double q_[4], double rm[3][3])
{
  double t = 0.0;
  double q[4];

  if (rm[2][2] < 0.0)
  {
    if (rm[0][0] > rm[1][1])
    {
      t = 1.0 + rm[0][0] - rm[1][1] - rm[2][2];
      q[1] = t;
      q[2] = rm[0][1]+rm[1][0];
      q[3] = rm[2][0]+rm[0][2];
      q[0] = rm[1][2]-rm[2][1];
    }
    else
    {
      t = 1.0 - rm[0][0] + rm[1][1] - rm[2][2];
      q[1] = rm[0][1]+rm[1][0];
      q[2] = t;
      q[3] = rm[1][2]+rm[2][1];
      q[0] = rm[2][0]-rm[0][2];
    }
  }
  else
  {
    if (rm[0][0] < -rm[1][1])
    {
      t = 1.0 - rm[0][0] - rm[1][1] + rm[2][2];
      q[1] = rm[2][0]+rm[0][2];
      q[2] = rm[1][2]+rm[2][1];
      q[3] = t;
      q[0] = rm[0][1]-rm[1][0];
    }
    else
    {
      t = 1.0 + rm[0][0] + rm[1][1] + rm[2][2];
      q[1] = rm[1][2]-rm[2][1];
      q[2] = rm[2][0]-rm[0][2];
      q[3] = rm[0][1]-rm[1][0];
      q[0] = t;
    }
  }

  if (t<=0.0)
  {
    REXEC(1)
    {
      RMSG("Failed converting quaternion from rotation matrix: t = %g", t);
      RMSG("Mat3d is %s", Mat3d_isValid(rm) ? "VALID" : "INVALID");
      Mat3d_printFormatted("Rotation matrix", "%g ", rm);
    }
    return false;
  }

  VecNd_constMul(q_, q, 0.5/sqrt(t), 4);

  return true;
}

/*******************************************************************************
 * This function follows the article
 * "Euler Angles, Quaternions, and Transformation Matrices (NASA 1977).
 *
 *  qw = q[0]
 *  qx = q[1]
 *  qy = q[2]
 *  qz = q[3]
 ******************************************************************************/
void Quat_fromEulerAngles(double q[4], const double ea[3])
{
  const double halfX = 0.5*ea[0];
  const double halfY = 0.5*ea[1];
  const double halfZ = 0.5*ea[2];

  const double s1 = sin(halfX);
  const double s2 = sin(halfY);
  const double s3 = sin(halfZ);
  const double c1 = cos(halfX);
  const double c2 = cos(halfY);
  const double c3 = cos(halfZ);

  q[0] = -s1*s2*s3 + c1*c2*c3;
  q[1] =  s1*c2*c3 + s2*s3*c1;
  q[2] = -s1*s3*c2 + s2*c1*c3;
  q[3] =  s1*s2*c3 + s3*c1*c2;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Quat_toEulerAngles(double ea[3], const double q[4])
{
  double rm[3][3];

  Quat_toRotationMatrix(rm, q);
  Mat3d_toEulerAngles(ea, rm);
}
