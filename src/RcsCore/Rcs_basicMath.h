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

#ifndef RCS_BASICMATH_H
#define RCS_BASICMATH_H


#ifdef __cplusplus
extern "C" {
#endif


#include "Rcs_MatNd.h"
#include "Rcs_HTr.h"


/*!
 * \defgroup RcsBasicMathFunctions Basic math functions
 */


/**
 * @name BasicFunctions
 *
 * Basic functions
 */

///@{

/*! \ingroup RcsBasicMathFunctions
 *  \brief Returns the corresponding angle in ]-180, 180] degrees.
 */
double Math_fmodAngle(double angle);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Returns the smallest of 3 double values.
 */
double Math_fmin3(double x, double y, double z);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Returns the largest of 3 double values.
 */
double Math_fmax3(double x, double y, double z);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Returns x clipped to the range [min , max].
 */
int Math_iClip(int x, int min, int max);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Returns x clipped to the range [min , max].
 */
double Math_clip(double x, double min, double max);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Returns 1.0 for val>=0, -1.0 else.
 */
double Math_dsign(double val);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Returns the acos of c, clipped to [-1:1].
 */
double Math_acos(double c);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Returns an linearly interpolated value from a data vector
 *
 *  e.g., Math_interpolateLinear(5.3, data, n) returns
 *  a[5] + 0.3 * (a[6] - a[5])
 *
 *  If x < 0, data[0] is returned. If x > n-1, data[x-1] is returned.
 */
double Math_interpolateLinear(double x, const double* data, unsigned int n);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Swaps the values pointed to by p1 and p2.
 */
void Math_dSwap(double* p1, double* p2);

/*! \ingroup RcsBasicMathFunctions
  *  \brief Calculates the binomial coefficient "n choose k"
  *
  *  \param[in] n  total number of elements
  *  \param[in] k  number of elements to pick
  */
unsigned int Math_NchooseK(unsigned int n, unsigned int k);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Transforms the inertia tensor with respect to the body origin to
 *         the center of mass position.
 *
 *  \param[out] I_com  Inertia tensor with COM reference point, still
 *                     represented in the bodie's frame of reference.
 *  \param[in]  I_bdy  Inertia tensor with body reference point, represented
 *                     in the bodie's frame of reference.
 *  \param[in]  b_r_bc Vector from body origin to COM, represented
 *                     in the bodie's frame of reference.
 *  \param[in]  m      Body mass
 */
void Math_transformInertiaTensorBdyToCom(double I_com[3][3],
                                         double I_bdy[3][3],
                                         const double b_r_bc[3],
                                         double m);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Adds the Steiner term resluting from the given com and
 *         mass to the inertia tensor.
 *
 *  \param[in,out] Inertia  Inertia tenso
 *  \param[in]  r_com       Vector to COM, represented in the inertia tensor's
 *                          frame of reference.
 *  \param[in]  m           Mass
 */
void Math_addSteinerToInertia(double Inertia[3][3],
                              const double r_com[3],
                              double m);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Returns the "likelihood" that a scalar x is belongs to Gaussian
 *  defined by a scalar mu and scalar stddev
 */
double Math_scalarGaussian(double stddev, double mu, double x);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Normalized version of Math_scalarGaussian, gives the Gaussian PDF
 */
double Math_normalizedScalarGaussian(double stddev, double mu, double x);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Returns the "likelihood" that x is belongs to Gaussian defined by
 *         mu and sigma. If array sigma is square, its inverse is computed
 *         using the Cholesky decomposition. If it cannot be inverted (e.g.
 *         sigma is not symmetric), the function exits fatally.
 *         If array sigma is a column
 *         vector, it is interpreted as the main diagonal of sigma, and its
 *         inverse is computed by the reciprocal of its elements. In the
 *         latter case, if a diagonal element is zero, its inverse is set to
 *         DBL_MAX.
 *         If dim is smaller or equal MATND_MAX_STACK_VECTOR_SIZE doubles,
 *         no heap memory will be allocated.
 *
 *  NOTE: this is not the Gaussian probability density function, for this the
 *        result would need to be scaled with
 *        1 / ((2 * PI)^(dim/2) * det(Sigma)^(1/2))
 *        see Math_normalizedGaussian
 */
double Math_gaussian(const MatNd* sigma, const double* mu, const double* x,
                     unsigned int dim);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Normalized version of Math_gaussian, gives the Gaussian PDF
 */
double Math_normalizedGaussian(const MatNd* sigma, const double* mu,
                               const double* x, unsigned int dim);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Mahalanobis distance
 *         dist = sqrt((x-mu)' Sigma^-1 (x-mu))
 */
double Math_mahalanobisDistance(const double* x, const double* mu,
                                const MatNd* sigma, unsigned int dim);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Linear Sigmoid function
 */
double Math_sigmoidLinear(double value, double startRise, double endRise);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Exponential Sigmoid function
 */
double Math_sigmoidExponential(double value, double steepness);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Sine Sigmoid function
 */
double Math_sigmoidSine(double value, double startRise, double endRise);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Filters the weighted mean Euler angles of a set of n Euler angles.
 *         The function updates avgEul so that it makes a minimization step
 *         towards minimizing the weighted square cost of angular differences
 *         between the rotation matrices corresponding to avgEul and desEul.
 *         This function does not suffer from representation issues of the
 *         Euler angles (jumps etc.), since it uses the underlying rotation
 *         matrices.
 *
 *  \param[in,out] avgEul  Euler angles in x-y-z order to be updated
 *  \param[in] desEul      n vectors of desired Euler angles
 *  \param[in] weight      n doubles (or NULL) of weights describing to how
 *                         strong the corresponding Euler angles should be
 *                         reached
 *  \param[in] n           number of desired Euler angle vectors
 */
bool Math_weightedMeanEulerAngles(double avgEul[3], double desEul[][3],
                                  const double weight[], unsigned int n);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Averages a set of Euler angles. The function updates avgEul
 *         by computing the axis angle represention of all angles,
 *         resulting in a set of axes a_i and a set of angles phi_i.
 *         Next, the function computes the (weighted) mean of phi_i * a_i.
 *         The resulting angle is two times the norm of the resulting vector,
 *         while the normalized vector represents the axis of rotation.
 *         The resulting representation is then transformed back to the
 *         Euler space.
 *         Implicitely, the function is computing the mean of the rotations
 *         in the log-space of the rotations represented as set of quaternions.
 *
 *  \param[in,out] avgEul  Euler angles in x-y-z order to be updated
 *  \param[in] desEul      n vectors of desired Euler angles
 *  \param[in] weight      n doubles (or NULL) of weights describing to how
 *                         strong the corresponding Euler angles should be
 *                         reached
 *  \param[in] n           number of desired Euler angle vectors
 */
void Math_weightedMeanRotationAxes(double avgEul[3], double desEul[][3],
                                   const double weight[], unsigned int n);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Finite difference approximation.
 *
 *  \param[out] dfdq Numerical derivative
 *  \param[in]  f    Function pointer to cost function
 *  \param[in]  data User pointer to underlying struct (NULL if none)
 *  \param[in]  q    State vector
 *  \param[in]  eps  Finite difference step size (should be small, e.g. 1e-8)
 *  \return Relative approximation error, see source code for details.
 */
void Math_finiteDifferenceDerivative(MatNd* dfdq,
                                     void (*f)(MatNd*, const MatNd*, void*),
                                     void* data,
                                     const MatNd* q,
                                     double eps);

///@}





/**
 * @name SpecialNumbers
 *
 * Special numbers
 */

///@{

/*! \ingroup RcsBasicMathFunctions
 *  \brief Returnd infinity (portable version)
 */
double Math_infinity();

/*! \ingroup RcsBasicMathFunctions
 *  \brief Returns true if the value is finite, false otherwise.
 *
 *  \param[in] value Value to be checked.
 */
bool Math_isFinite(double value);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Returns true if the value is infinite.
 *
 *  \param[in] value Value to be checked.
 */
bool Math_isINF(double value);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Returns true if the value is NAN.
 *
 *  \param[in] value Value to be checked.
 */
bool Math_isNAN(double value);

///@}




/**
 * @name BitwiseOperations
 *
 * Bitwise operations
 */

///@{

/*! \ingroup RcsBasicMathFunctions
 *  \brief Returns true if the bit at position bitNo is 1, false otherwise.
 *         Argument bitNo must be within [0 ... 31], otherwise the function
 *         exits fatally.
 */
bool Math_isBitSet(unsigned int value, unsigned int bitNo);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Returns true if the bit at position bitNo is 0, false otherwise.
 *         Argument bitNo must be within [0 ... 31], otherwise the function
 *         exits fatally.
 */
bool Math_isBitClear(unsigned int value, unsigned int bitNo);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Converts 32-bit integer value into a string with the binary
 *         representation and prints it to the console.
 */
void Math_printBinaryVector(int b);

///@}





/**
 * @name RandomNumbers
 *
 * Random numbers
 */

///@{

/*! \ingroup RcsBasicMathFunctions
 *  \brief Returns a uniformly distributed random number within the interval
 *         given by lower and upper: lower <= result < upper. The random
 *         numbers are seeded with the computer clock, so that they do not
 *         give reproduceable results. Lower must be <= upper, otherwise the
 *         function will exit fatally.
 */
double Math_getRandomNumber(double lower, double upper);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Returns a uniformly distributed random number within the interval
 *         given by lower and upper: lower <= result <= upper. The random
 *         numbers are seeded with the computer clock, so that they do not
 *         give reproduceable results. Lower must be < upper, otherwise the
 *         function will exit fatally.
 */
int Math_getRandomInteger(int lower, int upper);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Calls \ref Math_srand48 using a double, which is assumed to be
 *  a time in seconds, as a seed.
 */
void Math_srand48Time(double time);

/*! \ingroup RcsBasicMathFunctions
 *  \brief See srand48(). This function exists for OS compatibility.
 */
void Math_srand48(long int seed);


///@}





/**
 * @name CreationAndDestruction
 *
 * Creation and destruction
 */

///@{

/*! \ingroup RcsBasicMathFunctions
 *  \brief Polynomial root finder for polynomials with arbitrary degree.
 *
 *         This function finds the real roots for a polynomial of the
 *         form y = c[0] + c[1] x + c[2] x^2 + ... + c[n] x^n
 *         The function implements analytic solutions up to degree 3.
 *         Everything above will be determined numerically, and therefore is
 *         not particularly performant and robust.
 *
 *  \param[out] roots The roots of the polynomial. It must have memory for
 *                    \param degree double values. The function writes the
 *                    number of degree roots. If there are roots with
 *                    multiplicity, they are written with the same values. For
 *                    instance if a degree 3 polynomial has a triple root for 1,
 *                    root[0] = root[1] = root[2] = 1, and the return value
 *                    is 1.
 *  \param[in] c      degree+1 coefficients of the polynomial
 *  \param[in] degree Degree of the polynomial. It corresponds to the highest
 *                    exponent
 *  \return Number of determined roots on success, -1 on failure. For
 *          polynomials up to order 3 (cubic), the function always succeeds.
 */
int Math_findPolynomialRoots(double* roots, const double* c,
                             unsigned int degree);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Evaluates the polynomial equation
 *         y = c[0] + c[1] x + c[2] x^2 + ... + c[n] x^n
 *
 *  \param[in] x   Value that is being evaluated
 *  \param[in] c   Polynomial coefficients (one more than degree)
 *  \param[in] degree   Order of the polynomial (1=linear, 2=quadratic ...)
 *  \return Value of the polynomial for the given x
 */
double Math_computePolynomial(double x, const double* c, int degree);


///@}





/**
 * @name Distance
 *
 * Distance functions
 */

///@{

/*! \ingroup RcsBasicMathFunctions
 *  \brief This function returns the squared distance between a point and a
 *         line, and computes the closest points. Vector lineDir
 *         must be of unit length, otherwise the result is wrong. This
 *         is not cheked.
 *
 *  \param[in]  pt Point coordinates
 *  \param[in]  linePt Start point on line segment
 *  \param[in]  lineDir Normalized direction of the line segment.
 *  \param[out] cpLine Closest point on the line segment.
 *  \return Squared distance between point and line segment.
 */
double Math_sqrDistPointLine(const double pt[3],
                             const double linePt[3],
                             const double lineDir[3],
                             double cpLine[3]);

/*! \ingroup RcsBasicMathFunctions
 *  \brief This function returns the squared distance between a point and a
 *         line segment, and computes the closest points. Vector lineDir
 *         must be of unit length, otherwise the result is wrong. This
 *         is not cheked.
 *
 *  \param[in]  pt Point coordinates
 *  \param[in]  segPt Start point on line segment
 *  \param[in]  segDir Normalized direction of the line segment.
 *  \param[in]  segLength Length of the line segment
 *  \param[out] cpSeg Closest point on the line segment.
 *  \return Squared distance between point and line segment.
 */
double Math_sqrDistPointLineseg(const double pt[3],
                                const double segPt[3],
                                const double segDir[3],
                                const double segLength,
                                double cpSeg[3]);

/*! \ingroup RcsBasicMathFunctions
 *  \brief This function returns the squared distance between a point and a
 *         line segment, and computes the closest points. Vector lineDir
 *         must be of unit length.
 *
 *  \param[in]  pt Point coordinates
 *  \param[in]  capsulePt Start point on line segment of the capsule
 *  \param[in]  capsuleDir Normalized direction of the capsule.
 *  \param[in]  capsuleLength Length of the capsule
 *  \param[in]  capsuleRadius Radius of the capsule
 *  \param[out] cpCapsule Closest point on the capsule. If it is NULL,
 *              it will be ignored.
 *  \param[out] nPtCapsule Unit length normal from the point to the capsule.
 *              If the point lies on the line segment, nPtLine will be set to
 *              zero. If it is NULL, it will be ignored.
 *  \return Signed distance between point and capsule.
 */
double Math_distPointCapsule(const double pt[3],
                             const double capsulePt[3],
                             const double capsuleDir[3],
                             const double capsuleLength,
                             const double capsuleRadius,
                             double cpCapsule[3],
                             double nPtCapsule[3]);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Returns the distance and closest points between a point and a convex
 *         polygon in 2D.
 *  \param[in]  point      Query Point
 *  \param[in]  polygon    Polygon vertices in ordered counter-clockwise
 *  \param[in]  nVertices  Number of polygon vertices. Must be > 0, otherwise
 *                         the function exits with a fatal error.
 *  \param[out] cpPoly     Closest point on the polygon. If it is NULL, it will
 *                         be ignored.
 *  \param[out] nPoly      Normal vector that depenetrates the point from the
 *                         polygon. It is of unit length. If it is NULL, it will
 *                         be ignored.
 *  \return Distance is positive if point is outside the polygon and negative
 *          if inside.
 */
double Math_distPointConvexPolygon2D(const double point[2],
                                     const double polygon[][2],
                                     unsigned int nVertices,
                                     double cpPoly[2],
                                     double nPoly[2]);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Returns the distance and closest points between a point and a convex
 *         polygon in 3D. The polygon is assumed to be in the x-y plane of the
 *         frame A_PI. Its z-direction is the polygon normal. The polygon
 *         is assumed to be "filled".
 *
 *  \param[in]  point       Query Point
 *  \param[in]  polygon     Polygon vertices in ordered counter-clockwise
 *  \param[in]  nVertices   Number of polygon vertices. Must be > 0, otherwise
 *                          the function exits with a fatal error.
 *  \param[out] I_cpPoly    Closest point on the polygon in world coordinates.
 *                          If it is NULL, it will be ignored.
 *  \param[out] I_nPoly     Normal vector that depenetrates the point from the
 *                          polygon, in world coordinates. It is of unit length.
 *                          If it is NULL, it will be ignored.
 *  \return Squared distance between point and polygon.
 */
double Math_sqrDistPointConvexPolygon(const double I_pt[3],
                                      const HTr* A_PI,
                                      const double poly[][2],
                                      unsigned int nVertices,
                                      double I_cpPoly[3],
                                      double I_nPoly[3]);

/*! \ingroup RcsBasicMathFunctions
 *  \brief This function returns the squared distance between two line
 *         segments, and computes the closest points.
 *
 *  \param[in]  segPt0 Start point of the first line segment
 *  \param[in]  segDir0 Direction of the first line segment
 *  \param[in]  segLength0 Length of the second line segment
 *  \param[in]  segPt1 Start point of the second line segment
 *  \param[in]  segDir1 Direction of the second line segment
 *  \param[in]  segLength1 Length of the second line segment
 *  \param[out] cp0 Closest point on the line. If it is NULL, it will be
 *              ignored.
 *  \param[out] cp1 Closest point on the line segment. If it is NULL, it will
 *              be ignored.
 *  \return Squared distance of closest points.
 */
double Math_sqrDistLinesegLineseg(const double segPt0[3],
                                  const double segDir0[3],
                                  const double segLength0,
                                  const double segPt1[3],
                                  const double segDir1[3],
                                  const double segLength1,
                                  double cp0[3],
                                  double cp1[3]);

/*! \ingroup RcsBasicMathFunctions
 *  \brief This function returns the distance between two capsules, and
 *         computes the closest points. The distance is negative when the
 *         capsules penetrate.
 *
 *  \param[in]  linePt0 Start point of the first capsule
 *  \param[in]  lineDir0 Normalized direction vector of the first capsule
 *  \param[in]  l0 Length of the first capsule (distance between ball ends)
 *  \param[in]  r0 Radius of the first capsule
 *  \param[in]  linePt1 Start point of the second capsule
 *  \param[in]  lineDir1 Normalized direction vector of the second capsule
 *  \param[in]  l1 Length of the second capsule (distance between ball ends)
 *  \param[in]  r1 Radius of the second capsule
 *  \param[out] cp0 Closest point on the first capsule. If it is NULL, it
 *              will be ignored.
 *  \param[out] cp1 Closest point on the second capsule. If it is NULL, it
 *              will be ignored.
 *  \param[out] n01  Unit length normal from the closest point of the first
 *              to the second capsule. If the closest points coincide, the
 *              vector n01 is set to zero. If it is NULL, it will be ignored.
 *  \return Signed distance between closest points.
 */
double Math_distCapsuleCapsule(const double linePt0[3],
                               const double lineDir0[3],
                               const double l0,
                               const double r0,
                               const double linePt1[3],
                               const double lineDir1[3],
                               const double l1,
                               const double r1,
                               double cp0[3],
                               double cp1[3],
                               double n01[3]);
///@}





/**
 * @name Quaternios
 *
 * Quaternions are represented as [w x y z].
 */

///@{

/*! \ingroup RcsBasicMathFunctions
 *  \brief Conjugates a quaternion: q_conj = [q.w -q.x -q.y -q.z]
 *
 *  \param[out]  q_conj   Conjugated quaternion
 *  \param[in]   q        Original quaternion
 */
void Quat_conjugate(double q_conj[4], const double q[4]);

/*! \ingroup RcsBasicMathFunctions
 *  \brief In-place conjugation of a quaternion. See \ref Quat_conjugate().
 *
 *  \param[in,out]  q    Quaternion to be conjugated
 */
void Quat_conjugateSelf(double q[4]);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Quaternion multiplication: q = q1 * q2.
 *
 *  \param[out]  q    Result of q1*q2
 *  \param[in]   q1   Start quaternion
 *  \param[in]   q2   Target quaternion
 */
void Quat_mul(double q[4], const double q1[4], const double q2[4]);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Quaternion dot product: Sum of element-wise multiplied elements.
 *
 *  \param[in]   q1   Start quaternion
 *  \param[in]   q2   Target quaternion
 *
 *  \return Quaternion dot product
 */
double Quat_dot(const double q1[4], const double q2[4]);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Shortest angle on unit sphere between two quaternions.
 *
 *  \param[in]   q1   Start quaternion
 *  \param[in]   q2   Target quaternion
 *
 *  \return Shortest angle on unit sphere between two quaternions.
 */
double Quat_diffAngle(const double q1[4], const double q2[4]);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Relative quaternion q =  q1^-1 * q2
 *
 *  \param[out]  q    Relative quaternion
 *  \param[in]   q1   Start quaternion
 *  \param[in]   q2   Target quaternion
 */
void Quat_relativeRotation(double q[4], const double q1[4], const double q2[4]);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Identity quaternion q = [1 0 0 0]
 *
 *  \return Identity quaternion
 */
const double* Quat_identity();

/*! \ingroup RcsBasicMathFunctions
 *  \brief Normalizes a quaternion in-place. This fails if its length is 0. In
 *         this case, the quaternion remains unchanged.
 *
 *  \return Lenth of the original quaternion.
 */
double Quat_normalizeSelf(double q[4]);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Converts the quaternion q [qw qx qy qz] to a rotation matrix in
 *         row-major form.
 *
 *  \param[out]  A_BI   Rotation matrix in row-major form
 *  \param[in]   q      Quaternion in ordering [qw qx qy qz]
 */
void Quat_toRotationMatrix(double A_BI[3][3], double q[4]);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Converts a rotation matrix in row-major form to a quaternion
 *         q [qw qx qy qz].
 *
 *  \param[out]  q      Quaternion in ordering [qw qx qy qz]
 *  \param[in]   A_BI   Rotation matrix in row-major form
 */
void Quat_fromRotationMatrix(double q[4], double rm[3][3]);

///@}



#ifdef __cplusplus
}
#endif

#endif   // RCS_BASICMATH_H
