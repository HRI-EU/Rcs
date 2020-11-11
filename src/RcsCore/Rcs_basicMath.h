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

/*! \ingroup RcsBasicMathFunctions
 *  \brief Converts cylinder to cartesian coordinates:
 *
 *   / x \       / r      \
 *   | y |  = f  | phi    |
 *   \ z /       \ height /
 *
 *  \param[in]  radialDist   Radial distance from cylinder axis
 *  \param[in]  azimuth      Azimuth angle (between x-axis and p)
 *  \param[in]  height       Cylinder height coordinate (equals p[2])
 *  \param[out] p            Cartesian coordinates.
 */
void Math_Cyl2Cart(const double radialDist, const double azimuth,
                   const double height, double p[3]);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Converts cartesian to cylinder coordinates:
 *
 *   / r      \     / x \
 *   | phi    | = f | y |
 *   \ height /     \ z /
 *
 *  \param[in] p              Cartesian coordinates. The length of the vector
 *                            must not be zero. It is not checked.
 *  \param[out]  radialDist   Radial distance from cylinder axis
 *  \param[out]  azimuth      Azimuth angle (between x-axis and p)
 *  \param[out]  height       Cylinder height coordinate (equals p[2])
 */
void Math_Cart2Cyl(const double p[3], double* radialDist, double* azimuth,
                   double* height);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Gradient of cylinder coordinates wrt. to cartesian coordinates:
 *
 *              / dr/dx        dr/dy         dr/dz     \
 *  dCyldCart = | dphi/dx      dpi/dy       dpi/dz     |
 *              \ dheight/dx   dheight/dy   dheight/dz /
 *
 *  \param[out] dCyldCart   Gradient as above
 *  \param[in]  cart        Cartesian coordinates. The length of the vector
 *                          must not be zero. It is not checked.
 */
void Math_dCyldCart(double dCyldCart[3][3], const double cart[3]);

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
 *         give reproduceable results. Lower must be < upper, otherwise the
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
 *  \brief Returns a random true or false.
 */
bool Math_getRandomBool();

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
 * @name PolynomialRoots
 *
 * Polynomial root finding
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





#ifdef __cplusplus
}
#endif

#endif   // RCS_BASICMATH_H
