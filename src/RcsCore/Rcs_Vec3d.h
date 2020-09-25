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

#ifndef RCS_VEC3D_H
#define RCS_VEC3D_H


#include "Rcs_HTr.h"


#ifdef __cplusplus
extern "C" {
#endif



/*!
 * \defgroup RcsVec3dFunctions Vec3d: 3-dimensional vector functions
 */



/*! \ingroup RcsVec3dFunctions
 *  \brief v3 = v1 + v2
 */
void Vec3d_add(double v3[3], const double v1[3], const double v2[3]);

/*! \ingroup RcsVec3dFunctions
 *  \brief v2 = v2 + v1
 */
void Vec3d_addSelf(double v2[3], const double v1[3]);

/*! \ingroup RcsVec3dFunctions
 *  \brief v3 = v1 - v2
 */
void Vec3d_sub(double v3[3], const double v1[3], const double v2[3]);

/*! \ingroup RcsVec3dFunctions
 *  \brief v2 = v2 - v1
 */
void Vec3d_subSelf(double v2[3], const double v1[3]);

/*! \ingroup RcsVec3dFunctions
 *  \brief \f$ \mathbf{_K r = A_{KI} \; _I r}\f$
 */
void Vec3d_rotate(double K_r[3], double A_KI[3][3], const double I_r[3]);

/*! \ingroup RcsVec3dFunctions
 *  \brief \f$ \mathbf{_K r = A_{KI} \; _I r}\f$
 */
void Vec3d_rotateSelf(double I_r[3], double A_KI[3][3]);

/*! \ingroup RcsVec3dFunctions
 *  \brief \f$ \mathbf{_I r = A_{KI}^T \; _K r}\f$
 */
void Vec3d_transRotate(double I_r[3], double A_KI[3][3],
                       const double K_r[3]);

/*! \ingroup RcsVec3dFunctions
 *  \brief \f$ \mathbf{_I r = A_{KI}^T \; _K r}\f$
 */
void Vec3d_transRotateSelf(double K_r[3], double A_KI[3][3]);

/*! \ingroup RcsVec3dFunctions
 *  \brief Rotates vector vec around the given axis with the given angle.
 */
void Vec3d_rotateAboutAxis(double dst[3],
                           const double src[3],
                           const double axis[3],
                           const double angle);

/*! \ingroup RcsVec3dFunctions
 *  \brief Returns the Euclidean distance of points p1 and p2.
 */
double Vec3d_distance(const double* p1, const double* p2);

/*! \ingroup RcsVec3dFunctions
 *  \brief Returns the squared distance of points p1 and p2.
 */
double Vec3d_sqrDistance(const double* p1, const double* p2);

/*! \ingroup RcsVec3dFunctions
 *  \brief Returns true if the vectors are element-wise equal with a
 *         tolerance less than eps.
 */
bool Vec3d_isEqual(const double v1[3], const double v2[3], double eps);

/*! \ingroup RcsVec3dFunctions
 *  \brief Returns false if any element is not finite, true otherwise.
 */
bool Vec3d_isFinite(const double vec[3]);

/*! \ingroup RcsVec3dFunctions
 *  \brief Computes the polar angle phi from the given 3-dimensional
 *         z-vector. It is assumed to have unit-length. No checking
 *         is done.
 */
double Vec3d_getPolarPhi(const double z[3]);

/*! \ingroup RcsVec3dFunctions
 *  \brief Computes the polar angle theta from the given 3-dimensional
 *         z-vector. It is assumed to have unit-length. No checking is
 *         done.
 */
double Vec3d_getPolarTheta(const double z[3]);

/*! \ingroup RcsVec3dFunctions
 *  \brief Sets the vectors elements to [x y z].
 */
void Vec3d_set(double vector[3], double x, double y, double z);

/*! \ingroup RcsVec3dFunctions
 *  \brief Sets the vectors elements to the given value.
 */
void Vec3d_setElementsTo(double vector[3], const double value);

/*! \ingroup RcsVec3dFunctions
 *  \brief Sets the 3 elements to 0.
 */
void Vec3d_setZero(double vector[3]);

/*! \ingroup RcsVec3dFunctions
 *  \brief v2 = v1 * c.
 */
void Vec3d_constMul(double v2[3], const double v1[3], double c);

/*! \ingroup RcsVec3dFunctions
 *  \brief v2 = = v1 + c
 */
void Vec3d_constAdd(double v2[3], const double v1[3], double c);

/*! \ingroup RcsVec3dFunctions
 *  \brief v3 = v1 + v2 * c.
 */
void Vec3d_constMulAndAdd(double v3[3], const double v1[3],
                          const double v2[3], double c);

/*! \ingroup RcsVec3dFunctions
 *  \brief v2 += v1 * c.
 */
void Vec3d_constMulAndAddSelf(double v2[3], const double v1[3], double c);

/*! \ingroup RcsVec3dFunctions
 *  \brief v3 = c * (v1 + v2).
 */
void Vec3d_addAndConstMul(double v3[3], const double v1[3],
                          const double v2[3], double c);

/*! \ingroup RcsVec3dFunctions
 *  \brief Multiplies all elements with scalar c.
 */
void Vec3d_constMulSelf(double v1[3], double c);

/*! \ingroup RcsVec3dFunctions
 *  \brief Adds a scalar c to all elements.
 */
void Vec3d_constAddSelf(double v1[3], double c);

/*! \ingroup RcsVec3dFunctions
 *  \brief Computes the cross product v3 = v1 x v2.
 */
void Vec3d_crossProduct(double v3[3], const double v1[3],
                        const double v2[3]);

/*! \ingroup RcsVec3dFunctions
 *  \brief Returns the length of the vector.
 */
double Vec3d_getLength(const double v[3]);

/*! \ingroup RcsVec3dFunctions
 *  \brief Returns the squared length of the vector.
 */
double Vec3d_sqrLength(const double v[3]);

/*! \ingroup RcsVec3dFunctions
 *  \brief Copies src to dst.
 */
void Vec3d_copy(double dst[3], const double src[3]);

/*! \ingroup RcsVec3dFunctions
 *  \brief Copies the normalized vector src to dst. The length of the
 *         original vector is returned. If it is 0, no normalization
 *         takes place, and dst remains unchanged.
 */
double Vec3d_normalize(double dst[3], const double src[3]);

/*! \ingroup RcsVec3dFunctions
 *  \brief Normalizes the vector to unit length. The length of the original
 *         vector is returned. If it is 0, no normalization takes place.
 */
double Vec3d_normalizeSelf(double vec[3]);

/*! \ingroup RcsVec3dFunctions
 *  \brief Returns the angle between v1 and v2. The vectors don't need to be
 *         normalized. If one of v1 or v2 is of zero length, the function
 *         returns 0. The angle will always be in the range of [0 : M_PI].
 */
double Vec3d_diffAngle(const double v1[3], const double v2[3]);

/*! \ingroup RcsVec3dFunctions
 *  \brief Returns the inner product of v1 and v2.
 */
double Vec3d_innerProduct(const double v1[3], const double v2[3]);

/*! \ingroup RcsVec3dFunctions
 *  \brief Computes the outer product of u and v: A = u v^T.
 */
void Vec3d_outerProduct(double A[3][3], const double u[3], const double v[3]);

/*! \ingroup RcsVec3dFunctions
 *  \brief Computes the outer product of u: A = u u^T.
 */
void Vec3d_outerProductSelf(double A[3][3], const double u[3]);

/*! \ingroup RcsVec3dFunctions
 *  \brief Computes the polar angles from the given 3-dimensional
 *         vector vec. It is assumed to have unit-length.
 */
void Vec3d_getPolarAngles(double polarAngles[2], const double vec[3]);

/*! \ingroup RcsVec3dFunctions
 *  \brief Computes the (unit) polar axis from angles phi and theta.
 */
void Vec3d_getPolarAxis(double axis[3], double phi, double theta);

/*! \ingroup RcsVec3dFunctions
 *  \brief Returns the spherical velocity matrix
 *         \f$
 *         \mathbf{ H = \frac{\partial \Phi}{\partial \omega}  }
 *         \f$.
 *         Vector r must be of unit-length, otherwise the result is wrong.
 *         The matrix has a singularity for r having x- and y-components
 *         being zero, which means the axis is pointing into the positive or
 *         negative z-direction.
 *         \f[
 *         \mbox{$\left( \begin{array}{c}
 *                       { \dot{\varphi} } \\ [0.2em]
 *                       { \dot{\theta} }
 *                       \end{array} \right)$}
 *         =
 *         \mathbf{H \omega}
 *         \f]
 */
void Vec3d_getPolarVelocityMatrix(double H[2][3], const double r[3]);

/*! \ingroup RcsVec3dFunctions
 *  \brief Calculates the angular displacement about the x- and y-axis in
 *         order to align the A_BI's z-axis (third row) with the desired
 *         Polar angles.
 */
void Vec3d_getPolarError(double om_xy[2], const double polarDes[2],
                         double A_BI[3][3]);

/*! \ingroup RcsVec3dFunctions
 *  \brief Calculates the angular displacement in order to align the A_BI's
 *         direction-axis (direction-th row of A_BI) with the desired Polar
 *         angles.
 */
void Vec3d_getPolarErrorFromDirection(double om_xy[2],
                                      const double polarDes[2],
                                      double A_BI[3][3], int direction);

/*! \ingroup RcsVec3dFunctions
 *  \brief Transforms vector \f$ \mathbf{_I r_{IP}} \f$ into the coordinates
 *         of the K-frame: <br>
 *         \f$ \mathbf{_K r_{KP} = A_{KI} \; (_I r_{IP} - _I r_{IK})}\f$
 *         <br>
 *         with \f$ \mathbf{ _I r_{IK}  } \f$ being the origin of the
 *         K-frame, in I-coordinates.
 */
void Vec3d_invTransform(double K_r_KP[3], const HTr* A_KI,
                        const double I_r_IP[3]);

/*! \ingroup RcsVec3dFunctions
 *  \brief See Vec3d_invTransform().
 */
void Vec3d_invTransformSelf(double r[3], const HTr* A_KI);

/*! \ingroup RcsVec3dFunctions
 *  \brief Transforms vector K_r into the coordinates of the I-frame:
 *         I_r_IP = I_r_IK  + (A_KI)^T * K_r_KP
 *         with K_r_KP being the vector from the K-frame to the point P,
 *         in K-coordinates. Vector I_r_IK is the origin of A_KI.
 *         <br>
 *         This does the same:
 *         Vec3d_transform(I_r_IP, A_KI, K_r_KP);
 *         Vec3d_transMulAndAdd(I_r_IP, A_KI->org, A_KI->rot, K_r_KP);
 *
 */
void Vec3d_transform(double I_r_IP[3], const HTr* A_KI,
                     const double K_r_KP[3]);

/*! \ingroup RcsVec3dFunctions
 *  \brief \f$ \mathbf{_I r_{IP} = _I r_{IK} + A_{KI}^T \; _K r_{KP}} \f$
 */
void Vec3d_transMulAndAdd(double I_r_IP[3], const double I_r_IK[3],
                          double A_KI[3][3], const double K_r_KP[3]);

/*! \ingroup RcsVec3dFunctions
 *  \brief \f$ \mathbf{_I r_{IP} += A_{KI}^T \; _K r_{KP}} \f$
 */
void Vec3d_transMulAndAddSelf(double I_r_IP[3], double A_KI[3][3],
                              const double K_r_KP[3]);

/*! \ingroup RcsVec3dFunctions
 *  \brief See Vec3d_transform().
 */
void Vec3d_transformSelf(double I_r_IP[3], const HTr* A_KI);

/*! \ingroup RcsVec3dFunctions
 *  \brief Computes the intersection of a line and a plane. The line is
 *         represented by a 3-D point and its 3-D direction vector. The
 *         plane is represented by a 3-D point lying in the plane, and the
 *         3-D plane normal. The function returns true if an intersection
 *         point exists, false otherwise. In the latter case, argument
 *         intersection is unchanged.
 *
 *  \param[out] intersection Intersection point
 *  \param[in] linePt Point on the line
 *  \param[in] lineDir Direction vector of the line. Must be normalized.
 *  \param[in] planePt Point on the plane
 *  \param[in] planeNormal Normal on the plane. Must be normalized.
 */
bool Vec3d_computePlaneLineIntersection(double intersection[3],
                                        const double linePt[3],
                                        const double lineDir[3],
                                        const double planePt[3],
                                        const double planeNormal[3]);

/*! \ingroup RcsVec3dFunctions
 *  \brief Interpolation of Euler orientations using Mat3d_slerp. Value
 *         t is the interpolation factor. A value between [0...1]
 *         interpolates between a and b, but t can also be negative or
 *         larger than t.
 *  \see Mat3d_slerp()
 */
void Vec3d_slerp(double c[3], const double a[3], const double b[3], double t);

/*! \ingroup RcsVec3dFunctions
 *  \brief Linear interpolation of two vectors: c = (1-t)*a + t*b
 */
void Vec3d_lerp(double c[3], const double a[3], const double b[3], double t);

/*! \ingroup RcsVec3dFunctions
 *  \brief Computes an orthogonal and normalized vector of vec. The result
 *         will always be a vector in the xy-plane. If vec points (almost)
 *         to (0 0 1), the result will be the x-axis. If vec's length is
 *         0, ortho will be (1 0 0).
 */
void Vec3d_orthonormalVec(double ortho[3], const double vec[3]);

/*! \ingroup RcsVec3dFunctions
 *  \brief Copies 3 random values within the range between lower and upper
 *         into the vector.
 */
void Vec3d_setRandom(double vec[3], double lower, double upper);

/*! \ingroup RcsVec3dFunctions
 *  \brief Creates a unit vector with random direction.
 */
void Vec3d_setRandomUnitVector(double vec[3]);

/*! \ingroup RcsVec3dFunctions
 *  \brief Sets vec to (1 0 0) for dir = 0, (0 1 0) for dir = 1 or (0 0 1)
 *         for dir = 2. If dir is anything else, the function will exit with
 *         a fatal error.
 */
void Vec3d_setUnitVector(double vec[3], int dir);

/*! \ingroup RcsVec3dFunctions
 *  \brief Returns a pointer to a vector [0 0 0]. This returns a vector to
 *         a static variable. Please don't change this vector, since this
 *         affects every caller to this function.
 */
const double* Vec3d_zeroVec();

/*! \ingroup RcsVec3dFunctions
 *  \brief Returns a pointer to a vector [1 0 0]. This returns a vector to
 *         a static variable. Please don't change this vector, since this
 *         affects every caller to this function.
 */
const double* Vec3d_ex();

/*! \ingroup RcsVec3dFunctions
 *  \brief Returns a pointer to a vector [0 1 0]. This returns a vector to
 *         a static variable. Please don't change this vector, since this
 *         affects every caller to this function.
 */
const double* Vec3d_ey();

/*! \ingroup RcsVec3dFunctions
 *  \brief Returns a pointer to a vector [0 0 1]. This returns a vector to
 *         a static variable. Please don't change this vector, since this
 *         affects every caller to this function.
 */
const double* Vec3d_ez();

/*! \ingroup RcsVec3dFunctions
 *  \brief Returns a pointer to the unit vector according to the value dim.
 *         If dim is not within [0...2], the function exits with a fatal
 *         error. This returns a vector to a static variable. Please don't
 *         change this vector, since this affects every caller to this
 *         function.
 */
const double* Vec3d_unitVector(int dim);

/*! \ingroup RcsVec3dFunctions
 *  \brief Scales the vector so that its length does not exceed the limit
 *         value. If limit is smaller or equal to zero, self is set to zero.
 *         The function returns the scaling factor.
 */
double Vec3d_constSaturateSelf(double* self, const double limit);

/*! \ingroup RcsVec3dFunctions
 *  \brief Prints two vectors and their difference next to each other.
 */
void Vec3d_printTwoVectorsDiff(const double v1[3], const double v2[3],
                               int digits);

/*! \ingroup RcsVec3dFunctions
 *  \brief Prints the vector with a comment and a given format string to
 *         stderr. If comment is NULL, it will be skipped.
 */
void Vec3d_printFormatted(const char* text, const char* format,
                          const double self[3]);

/*! \ingroup RcsVec3Functions
 *  \brief Returns the index of the entry with the highest value of the vector.
 *         If two values have the same value, the lower index is returned.
 */
unsigned int Vec3d_indexMax(const double x[3]);

/*! \ingroup RcsVec3Functions
 *  \brief Returns the multiplicity of vector elements:
 *         - Three identical values: multiplicity = 3
 *         - Two identical values: multiplicity = 2
 *         - All values different: multiplicity = 1
 *         Values are considered to be different if they differ more than eps.
 */
unsigned int Vec3d_multiplicity(const double v[3], double eps);

/*! \ingroup RcsVec3Functions
 *  \brief Swaps the memory of v1 and v2.
 */
void Vec3d_swap(double v1[3], double v2[3]);


#ifdef __cplusplus
}
#endif

#endif   // RCS_VEC3D_H
