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

#ifndef RCS_QUATERNION_H
#define RCS_QUATERNION_H


#ifdef __cplusplus
extern "C" {
#endif


#include "Rcs_MatNd.h"
#include "Rcs_HTr.h"


/*!
 * \defgroup RcsQuaternionFunctions Quaternion functions
 */


/*! \ingroup RcsQuaternionFunctions
 *  \brief Conjugates a quaternion: q_conj = [q.w -q.x -q.y -q.z]
 *
 *  \param[out]  q_conj   Conjugated quaternion
 *  \param[in]   q        Original quaternion
 */
void Quat_conjugate(double q_conj[4], const double q[4]);

/*! \ingroup RcsQuaternionFunctions
 *  \brief In-place conjugation of a quaternion. See \ref Quat_conjugate().
 *
 *  \param[in,out]  q    Quaternion to be conjugated
 */
void Quat_conjugateSelf(double q[4]);

/*! \ingroup RcsQuaternionFunctions
 *  \brief Quaternion multiplication: q = q1 * q2.
 *
 *  \param[out]  q    Result of q1*q2
 *  \param[in]   q1   Start quaternion
 *  \param[in]   q2   Target quaternion
 */
void Quat_mul(double q[4], const double q1[4], const double q2[4]);

/*! \ingroup RcsQuaternionFunctions
 *  \brief Quaternion dot product: Sum of element-wise multiplied elements.
 *
 *  \param[in]   q1   Start quaternion
 *  \param[in]   q2   Target quaternion
 *
 *  \return Quaternion dot product
 */
double Quat_dot(const double q1[4], const double q2[4]);

/*! \ingroup RcsQuaternionFunctions
 *  \brief Shortest angle on unit sphere between two quaternions.
 *
 *  \param[in]   q1   Start quaternion
 *  \param[in]   q2   Target quaternion
 *
 *  \return Shortest angle on unit sphere between two quaternions.
 */
double Quat_diffAngle(const double q1[4], const double q2[4]);

/*! \ingroup RcsQuaternionFunctions
 *  \brief Relative quaternion q =  q1^-1 * q2
 *
 *  \param[out]  q    Relative quaternion
 *  \param[in]   q1   Start quaternion
 *  \param[in]   q2   Target quaternion
 */
void Quat_relativeRotation(double q[4], const double q1[4], const double q2[4]);

/*! \ingroup RcsQuaternionFunctions
 *  \brief Identity quaternion q = [1 0 0 0]
 *
 *  \return Identity quaternion
 */
const double* Quat_identity();

/*! \ingroup RcsQuaternionFunctions
 *  \brief Normalizes a quaternion in-place. This fails if its length is 0. In
 *         this case, the quaternion remains unchanged.
 *
 *  \return Lenth of the original quaternion.
 */
double Quat_normalizeSelf(double q[4]);

/*! \ingroup RcsQuaternionFunctions
 *  \brief Converts the quaternion q [qw qx qy qz] to a rotation matrix in
 *         row-major form. This function always succeeds. It expects a
 *         normalized quaternion as input. This is not checked.
 *
 *  \param[out]  A_BI   Rotation matrix in row-major form
 *  \param[in]   q      Quaternion in ordering [qw qx qy qz]
 */
void Quat_toRotationMatrix(double A_BI[3][3], const double q[4]);

/*! \ingroup RcsQuaternionFunctions
 *  \brief Converts a rotation matrix in row-major form to a quaternion
 *         q [qw qx qy qz]. This function may fail if the rotation matrix
 *         is invalid. In this case, the function returns false and leaves
 *         the target argument q unchanged.
 *
 *  \param[out]  q      Quaternion in ordering [qw qx qy qz]
 *  \param[in]   rm     Rotation matrix in row-major form
 *
 *  \return True for success, false otherwise.
 */
bool Quat_fromRotationMatrix(double q[4], double rm[3][3]);

/*! \ingroup RcsQuaternionFunctions
 *  \brief Converts a set of Euler angles (x-y-z order) to a quaternion
 *         q [qw qx qy qz]. This function always succeeds.
 *
 *  \param[out]  q      Quaternion in ordering [qw qx qy qz]
 *  \param[in]   ea     Euler angles
 */
void Quat_fromEulerAngles(double q[4], const double ea[3]);

/*! \ingroup RcsQuaternionFunctions
 *  \brief Converts a quaternion q [qw qx qy qz] to a set of Euler angles
 *         (x-y-z order). This function always succeeds. It expects a
 *         normalized quaternion as input. This is not checked.
 *
 *  \param[out] ea     Euler angles in x-y-z order
 *  \param[in]  q      Normalized quaternion in ordering [qw qx qy qz]
 */
void Quat_toEulerAngles(double ea[3], const double q[4]);



#ifdef __cplusplus
}
#endif

#endif   // RCS_QUATERNION_H
