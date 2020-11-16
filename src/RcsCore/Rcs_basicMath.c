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

#include "Rcs_basicMath.h"
#include "Rcs_Mat3d.h"
#include "Rcs_VecNd.h"
#include "Rcs_Vec3d.h"
#include "Rcs_timer.h"
#include "Rcs_macros.h"


// Global flag indicating whether the random number generator has already
// been initialized.
static int randomNumberGeneratorInit = false;



/*******************************************************************************
 * Find the corresponding angle in ]-180, 180] degrees
 ******************************************************************************/
double Math_fmodAngle(double angle)
{
  double result = fmod(angle, 2.0*M_PI);

  if (result > M_PI)
  {
    result = result - 2.0*M_PI;
  }
  else if (result <= -M_PI)
  {
    result = result + 2.0*M_PI;
  }

  return result;
}

/*******************************************************************************
 * See header
 ******************************************************************************/
double Math_fmin3(double x, double y, double z)
{
  double min = x;

  if (y < min)
  {
    min = y;
  }

  if (z < min)
  {
    min = z;
  }

  return min;
}

/*******************************************************************************
 * See header
 ******************************************************************************/
double Math_fmax3(double x, double y, double z)
{
  double max = x;

  if (y > max)
  {
    max = y;
  }

  if (z > max)
  {
    max = z;
  }

  return max;
}

/*******************************************************************************
 * See header
 ******************************************************************************/
bool Math_isBitSet(unsigned int value, unsigned int bitNo)
{
  RCHECK_MSG(bitNo<32, "Bit %d not in range [0 : 31]", bitNo);

  if (((value>>bitNo)&0x01) == 0x01)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/*******************************************************************************
 * See header
 ******************************************************************************/
void Math_printBinaryVector(int b)
{
  char a[33];
  int i;

  strcpy(a,"");

  for (i=31; i>=0; i--)
  {
    if (((b>>i)&0x0001) == 1)
    {
      strcat(a,"1");
    }
    else
    {
      strcat(a,"0");
    }
  }

  fprintf(stderr,"%s\n",a);
}

/*******************************************************************************
 * See header
 ******************************************************************************/
bool Math_isBitClear(unsigned int value, unsigned int bitNo)
{
  bool isBitClear = !Math_isBitSet(value, bitNo);

  return isBitClear;
}

/*******************************************************************************
 * See header
 ******************************************************************************/
int Math_iClip(int val, int lower, int upper)
{
  if (val < lower)
  {
    return lower;
  }
  else if (val > upper)
  {
    return upper;
  }
  else
  {
    return val;
  }
}

/*******************************************************************************
 * See header
 ******************************************************************************/
double Math_clip(double x, double min, double max)
{
  if (x < min)
  {
    return min;
  }
  else if (x > max)
  {
    return max;
  }
  else
  {
    return x;
  }
}

/*******************************************************************************
 * See header
 ******************************************************************************/
double Math_dsign(double val)
{
  return val >= 0.0 ? 1.0 : -1.0;
}

/*******************************************************************************
 * See header
 ******************************************************************************/
double Math_acos(double c)
{
  if (c > 1.0)
  {
    c = 1.0;
  }
  else if (c < -1.0)
  {
    c = -1.0;
  }

  return acos(c);
}

/*******************************************************************************
 * See header. The man pages state:
 *
 * The drand48() and erand48() functions return nonnegative,
 * double-precision, floating-point values uniformly distributed
 * over the range of y values such that 0 <= y < 1.0.
 *
 * That means that the lower value is inside the interval, while the
 * upper limit is not.
 ******************************************************************************/
double Math_getRandomNumber(double lower, double upper)
{
  double ele;
  double range = upper - lower;

  RCHECK_MSG(range >= 0.0, "Lower is larger than upper!");

  if (randomNumberGeneratorInit==false)
  {
    Math_srand48Time(Timer_getTime());
  }

  ele = range * drand48() + lower;

  // Pedantic check (can be removed when we trust the rand() function)
  RCHECK_MSG((ele >= lower) && (ele <= upper), "lower: %g ele: %g upper:%g   "
             "This must never happen!!!", lower, ele, upper);

  return ele;
}

/*******************************************************************************
 * See header
 ******************************************************************************/
int Math_getRandomInteger(int lower, int upper)
{
  RCHECK_MSG(upper - lower >= 0, "Lower is larger or equal to upper!");

  if (lower == upper)
  {
    return lower;
  }

  if (randomNumberGeneratorInit==false)
  {
    Math_srand48Time(Timer_getTime());
  }

  double ele = drand48();
  ele *= upper - lower + 1.0;
  ele += lower - 0.5;

  int return_temp = lround(ele);

  // pedantic checks for numeric/rounding errors (especially if original ele
  // is close to 1)
  if (return_temp < lower)
  {
    return_temp = lower;
  }
  if (return_temp > upper)
  {
    return_temp = upper;
  }

  return return_temp;
}

/*******************************************************************************
 * See header
 ******************************************************************************/
bool Math_getRandomBool()
{
  return Math_getRandomInteger(0, 1) == 0 ? true : false;
}

/*******************************************************************************
 * Since the srand function requres a long unsigned int, it happens for large
 * system times that the seed is always the same. Therefore, here we just
 * consider the values after the comma, and multiply it by 1000.
 ******************************************************************************/
void Math_srand48Time(double time)
{
  double msec = time - trunc(time);
  Math_srand48((long)(1.0e3*msec));
}

/*******************************************************************************
 * See header
 ******************************************************************************/
void Math_srand48(long int seed)
{
#if defined(_MSC_VER)
  srand(seed);
#else
  srand48(seed);
#endif
  randomNumberGeneratorInit = true;
}

/*******************************************************************************
 *
******************************************************************************/
void Math_Cart2Cyl(const double p[3], double* radialDist, double* azimuth,
                   double* height)
{
  *radialDist = sqrt(p[0]*p[0]+p[1]*p[1]);
  *height = p[2];
  *azimuth = atan2(p[1],p[0]);
}

/*******************************************************************************
 *
******************************************************************************/
void Math_Cyl2Cart(const double radialDist, const double azimuth,
                   const double height, double p[3])
{
  p[0] = radialDist*cos(azimuth);
  p[1] = radialDist*sin(azimuth);
  p[2] = height;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Math_dCyldCart(double dCyldCart[3][3], const double cart[3])
{
  const double r = sqrt(cart[0]*cart[0]+cart[1]*cart[1]);
  const double r2 = r*r;
  dCyldCart[0][0] =  cart[0]/r;
  dCyldCart[0][1] =  cart[1]/r;
  dCyldCart[0][2] =  0.0;
  dCyldCart[1][0] = -cart[1]/r2;
  dCyldCart[1][1] =  cart[0]/r2;
  dCyldCart[1][2] =  0.0;
  dCyldCart[2][0] =  0.0;
  dCyldCart[2][1] =  0.0;
  dCyldCart[2][2] =  1.0;
}

/*******************************************************************************
 * See header
 ******************************************************************************/
double Math_interpolateLinear(double x, const double* data, unsigned int n)
{
  if (x <= 0.0)
  {
    return data[0];
  }
  else if (x >= n - 1)
  {
    return data[n-1];
  }
  else
  {
    unsigned int j = (unsigned int) x;
    return data[j] + (x - j) * (data[j+1] - data[j]);
  }
}

/*******************************************************************************
 * See header
 ******************************************************************************/
double Math_infinity()
{
#if defined(_MSC_VER)
  return std::numeric_limits<double>::infinity();
#else
  return INFINITY;
#endif
}

/*******************************************************************************
 * See header
 ******************************************************************************/
void Math_dSwap(double* p1, double* p2)
{
  double temp = *p1;
  *p1=*p2;
  *p2=temp;
}

/*******************************************************************************
 * See header
 ******************************************************************************/
unsigned int Math_NchooseK(unsigned int n, unsigned int k)
{
  RCHECK_MSG(n >= k, "n: %d k: %d", n, k);
  unsigned int useK = k;
  if (k >= n/2)
  {
    useK = n-k;
  }
  unsigned int nMinusK = n-useK;

  double binCoeff = 1.;

  unsigned int i;
  for (i=1; i<=useK; i++)
  {
    binCoeff *= (double)(nMinusK+i)/(double)i;
  }
  return (unsigned int)binCoeff;
}

/*******************************************************************************
 * See header
 ******************************************************************************/
void Math_transformInertiaTensorBdyToCom(double I_com[3][3],
                                         double I_bdy[3][3],
                                         const double b_r_bc[3],
                                         double m)
{
  double steiner[3][3];
  Mat3d_setZero(steiner);
  Math_addSteinerToInertia(steiner, b_r_bc, m);
  Mat3d_sub(I_com, I_bdy, steiner);
}

/*******************************************************************************
 * See header
 ******************************************************************************/
void Math_addSteinerToInertia(double Inertia[3][3], const double r_com[3],
                              double m)
{
  // Principal components
  Inertia[0][0] += m*(r_com[1]*r_com[1] + r_com[2]*r_com[2]);
  Inertia[1][1] += m*(r_com[0]*r_com[0] + r_com[2]*r_com[2]);
  Inertia[2][2] += m*(r_com[0]*r_com[0] + r_com[1]*r_com[1]);

  // Deviation components
  Inertia[0][1] -= m*(r_com[0]*r_com[1]);
  Inertia[1][0] -= m*(r_com[0]*r_com[1]);

  Inertia[0][2] -= m*(r_com[0]*r_com[2]);
  Inertia[2][0] -= m*(r_com[0]*r_com[2]);

  Inertia[1][2] -= m*(r_com[1]*r_com[2]);
  Inertia[2][1] -= m*(r_com[1]*r_com[2]);
}

/*******************************************************************************
 * The isfinite() macro returns a nonzero value if x is finite: not plus or
 * minus infinity, and not NaN
 ******************************************************************************/
bool Math_isFinite(double value)
{
  return (isfinite(value) != 0 ? true : false);
}

/*******************************************************************************
 * See header
 ******************************************************************************/
bool Math_isINF(double value)
{
  return (!isfinite(value));
}

/*******************************************************************************
 * The isnan macro returns a nonzero value if x is NaN
 ******************************************************************************/
bool Math_isNAN(double value)
{
  return (isnan(value) != 0 ? true : false);
}

/*******************************************************************************
 * c0 + c1*x = 0 => x = -c0/c1
 ******************************************************************************/
static int Math_findLinearRoot(double* roots, const double* c)
{
  if (c[1]==0.0)
  {
    return 0;
  }
  else
  {
    roots[0] = -c[0]/c[1];
    return 1;
  }

}

/*******************************************************************************
 * y = ax^2 + bx + c => x12 = -b +/- sqrt(b^2-4ac)/2a
 ******************************************************************************/
static int Math_findQuadraticRoots(double* roots, double a, double b, double c)
{
  double discriminant = b*b - 4.0*a*c;

  if (discriminant < 0.0)
  {
    return 0;
  }
  else if (discriminant == 0.0)
  {
    roots[0] = -b/(2.0*a);
    roots[1] = roots[0];
    return 1;
  }
  else
  {
    double term1 = sqrt(discriminant);
    roots[0] = (-b - term1)/(2.0*a);
    roots[1] = (-b + term1)/(2.0*a);
    return 2;
  }

}

/*******************************************************************************
 * This follows the explanaions of Wikipedia:
 * https://de.wikipedia.org/wiki/Cardanische_Formeln
 * (except for the wrong sign for the case discriminant < 0)
 ******************************************************************************/
static int Math_findCubicRoots(double* roots,
                               double A, double B, double C, double D)
{
  // First we check if there is a cubic term so that we can bring the
  // polynomial to standard form. If there isn't, it's a quadratic polynomial
  // or of even lower order and we are done.
  if (A == 0.0)
  {
    return Math_findQuadraticRoots(roots, B, C, D);
  }

  int nRoots = 0;

  // Original form: A x^3 + B x^2 + C x + D = 0
  // Standard form:   x^3 + a x^2 + b x + c = 0
  double a = B/A;
  double b = C/A;
  double c = D/A;

  // Reduced form: z^3 + p z + q = 0 with x = z - a/3
  double p = b - a*a/3.0;
  double q = 2.0*a*a*a/27.0 - a*b/3.0 + c;

  // Discriminant = 0: One or two real roots
  double discriminant = pow(q/2.0,2) + pow(p/3.0, 3);

  const double almostZero = Math_clip(pow(q/2.0,2)*1.0e-12, 1.0e-12, 1.0e-3);

  if (fabs(discriminant) < almostZero)
    //if (discriminant == 0.0)
  {
    RLOGS(0, "Discriminant = 0: %g < %g, b=%g",
          fabs(discriminant), almostZero, b);

    if (p==0.0)   // triple root for z=0, only possible if p = q = 0
    {
      RLOG(0, "Case A: p=%g", p);
      roots[0] = -a/3.0;
      roots[1] = roots[0];
      roots[2] = roots[0];
      nRoots = 1;
    }
    else
    {
      RLOG(0, "Case B: p=%g (should not be 0)", p);
      roots[0] = 3.0*q/p - B/(3.0*A);
      roots[1] = -3.0*q/(2.0*p) - B/(3.0*A);
      roots[2] = roots[1];
      nRoots = 2;
    }
  }
  // Discriminant < 0: Three real roots
  else if (discriminant < 0.0)
  {
    RLOG(0, "Case C: discriminant=%g (<0)", discriminant);
    double term1 = 2.0*sqrt(-p/3.0);
    double term2 = Math_acos(-1.5*(q/p)*sqrt(-3.0/p))/3.0;
    double term3 = a/3.0;
    roots[0] = -term1*cos(term2) - term3;
    roots[1] =  term1*cos(term2 + M_PI/3.0) - term3;
    roots[2] =  term1*cos(term2 - M_PI/3.0) - term3;
    nRoots = 3;
  }
  // Discriminant > 0: Exactly one real root
  else
  {
    RLOG(0, "Case D: discriminant=%g (>0)", discriminant);
    double sqrtDis = sqrt(discriminant);
    double u = cbrt(-0.5*q + sqrtDis);
    double v = cbrt(-0.5*q - sqrtDis);
    roots[0] = u + v - B/(3.0*A);
    roots[1] = roots[0];
    roots[2] = roots[0];
    nRoots = 1;
  }

  return nRoots;
}

/*******************************************************************************
 * y = c[0] + c[1] x + c[2] x^2 + ... + c[n] x^n
 ******************************************************************************/
int Math_findPolynomialRoots(double* roots, const double* c,
                             unsigned int degree)
{
  switch (degree)
  {
    case 1:
      return Math_findLinearRoot(roots, c);
      break;

    case 2:
      return Math_findQuadraticRoots(roots, c[2], c[1], c[0]);
      break;

    case 3:
      return Math_findCubicRoots(roots, c[3], c[2], c[1], c[0]);
      break;

    default:
      RLOG(1, "Root finding for degree %d not yet implemented", degree);
  }

  return -1;
}

/*******************************************************************************
 * y = c[0] + c[1] x + c[2] x^2 + ... + c[n] x^n
 ******************************************************************************/
double Math_computePolynomial(double x, const double* c, int degree)
{
  // y = c[0] + c[1] x + c[2] x^2 + ... + c[n] x^n
  double y = c[0];

  for (int i=1; i<=degree; ++i)
  {
    y += c[i] * pow(x, i);
  }

  return y;
}

/*******************************************************************************
 * See header
 ******************************************************************************/
double Math_scalarGaussian(double stddev, double mu, double x)
{
  double exponent = -0.5 * pow((x - mu) / stddev, 2);
  return exp(exponent);
}

/*******************************************************************************
 * See header
 ******************************************************************************/
double Math_normalizedScalarGaussian(double stddev, double mu, double x)
{
  return Math_scalarGaussian(stddev, mu, x) / sqrt(2. * M_PI * pow(stddev, 2));
}

/*******************************************************************************
 * See header
 ******************************************************************************/
double Math_generalizedSquaredInterpointDistance(const MatNd* sigma,
                                                 const double* mu,
                                                 const double* x,
                                                 unsigned int dim,
                                                 double* determinant)
{
  MatNd* invSigma = NULL;
  double det = 0.0;

  if (sigma->n == 1)
  {
    RCHECK(sigma->m == dim);
    MatNd_create2(invSigma, dim, 1);
    det = MatNd_inverseDiag(invSigma, sigma);
  }
  else
  {
    RCHECK((sigma->m == dim) && (sigma->n == dim));
    MatNd_create2(invSigma, dim, dim);
    det = MatNd_choleskyInverse(invSigma, sigma);
    RCHECK(!Math_isNAN(det));
  }

  // distance = (x-mu)' Sigma^-1 (x-mu)
  MatNd* diff_x_mu = NULL;
  MatNd_create2(diff_x_mu, dim, 1);
  VecNd_sub(diff_x_mu->ele, x, mu, dim);

  double distance = 0.0;
  MatNd distanceMat = MatNd_fromPtr(1, 1, &distance);
  MatNd_sqrMulAtBA(&distanceMat, diff_x_mu, invSigma);

  //clean up
  MatNd_destroy(invSigma);
  MatNd_destroy(diff_x_mu);

  if (determinant != NULL)
  {
    *determinant = det;
  }

  return distance;
}

/*******************************************************************************
 * See header
 ******************************************************************************/
double Math_gaussianCommon(const MatNd* sigma,
                           const double* mu,
                           const double* x,
                           unsigned int dim,
                           bool normalize)
{
  double det;
  double exponent =
    Math_generalizedSquaredInterpointDistance(sigma, mu, x, dim, &det);

  // normalizing constant
  double normalizer = 1.0;
  if (normalize == true)
  {
    RCHECK(det != 0.);
    normalizer = sqrt(pow(2. * M_PI, (int) dim) * fabs(det));
  }

  return exp(-0.5 * exponent) / normalizer;
}

/*******************************************************************************
 * See header
 ******************************************************************************/
double Math_gaussian(const MatNd* sigma,
                     const double* mu,
                     const double* x,
                     unsigned int dim)
{
  return Math_gaussianCommon(sigma, mu, x, dim, false);
}

/*******************************************************************************
 * See header
 ******************************************************************************/
double Math_normalizedGaussian(const MatNd* sigma,
                               const double* mu,
                               const double* x,
                               unsigned int dim)
{
  return Math_gaussianCommon(sigma, mu, x, dim, true);
}

/*******************************************************************************
 * See header
 ******************************************************************************/
double Math_mahalanobisDistance(const double* x,
                                const double* mu,
                                const MatNd* sigma,
                                unsigned int dim)
{
  return sqrt(Math_generalizedSquaredInterpointDistance(sigma, mu, x, dim, NULL));
}

/*******************************************************************************
 * See header
 ******************************************************************************/
double Math_sigmoidLinear(double value, double staHTrise, double endRise)
{
  RCHECK(endRise >= staHTrise);

  if (value <= staHTrise)
  {
    return 0.0;
  }
  else if (value > endRise)
  {
    return 1.0;
  }
  else
  {
    return (value - staHTrise) / (endRise - staHTrise) ;
  }
}

/*******************************************************************************
 * See header
 ******************************************************************************/
double Math_sigmoidExponential(double value, double steepness)
{
  return 1.0 / (1.0 + exp(-steepness * value));
}

/*******************************************************************************
 * See header
 ******************************************************************************/
double Math_sigmoidSine(double value, double staHTrise, double endRise)
{
  RCHECK(endRise >= staHTrise);

  if (value <= staHTrise)
  {
    return 0.0;
  }
  else if (value > endRise)
  {
    return 1.0;
  }
  else
  {
    return (0.5 + 0.5*sin(M_PI*((value-staHTrise)/(endRise-staHTrise) - 0.5)));
  }
}

/*******************************************************************************
 * Weighted mean Euler angle calculation
 ******************************************************************************/
bool Math_weightedMeanEulerAngles(double avgEul[3], double desEul[][3],
                                  const double weight[], unsigned int n)
{
  const double stepSize = 1.0*1.0/n;
  const double warnAngle = 150.0/180.0*M_PI;
  double angle, axis[3], omega_i[3], omega[3];
  double A_1I[3][3], A_2I[3][3], A_21[3][3];
  bool isRangeSafe = true;

  Vec3d_setZero(omega);

  // Compute current averaged rotation matrix
  Mat3d_fromEulerAngles(A_1I, avgEul);

  // Loop over all desired euler angle vectors
  for (unsigned int i=0; i<n; i++)
  {
    // Compute desired rotation matrices
    Mat3d_fromEulerAngles(A_2I, desEul[i]);
    Mat3d_mulTranspose(A_21, A_2I, A_1I);

    // Compute axis-angle representation for relative rotation (in average
    // frame coordinates)
    angle = Mat3d_getAxisAngleSelf(axis, A_21);

    if (angle>warnAngle)
    {
      //RLOGS(0, "WARNING: Angle %d is %.2f", i, angle*180.0/M_PI);
      isRangeSafe = false;
    }

    // Compute corresponding angular velocity (in average frame coordinates)
    if (weight==NULL)
    {
      Vec3d_constMul(omega_i, axis, angle*stepSize);
    }
    else
    {
      Vec3d_constMul(omega_i, axis, angle*stepSize*weight[i]);
    }

    // Integrate angular velocities. They are all in the same frame, so we
    // simply add them together
    Vec3d_addSelf(omega, omega_i);
  }

  // Rotate current averaged rotation matrix around local (body) frame
  Mat3d_rotateOmegaSelf(A_1I, omega, false);

  // Compute new averaged Euler angles
  Mat3d_toEulerAngles(avgEul, A_1I);

  return isRangeSafe;
}

/*******************************************************************************
 * Weighted mean rotation axis calculation
 ******************************************************************************/
void Math_weightedMeanRotationAxes(double avgEul[3], double desEul[][3],
                                   const double weight[], unsigned int n)
{
  Vec3d_setZero(avgEul);
  double weight_sum = 0.0;
  double u[3];
  Vec3d_setZero(u);

  // Iterate each desired rotation
  for (size_t idx = 0; idx < n; idx++)
  {
    // Compute rotation matrix
    double A_KI[3][3];
    Mat3d_fromEulerAngles(A_KI, desEul[idx]);

    // Get axis angle representation
    double axis[3];
    double angle = Mat3d_getAxisAngleSelf(axis, A_KI);

    // Convert axis angle to quaternion representation
    double theta = angle/2.0;
    Vec3d_constMulSelf(axis, theta);
    if (weight != NULL)
    {
      Vec3d_constMulSelf(axis, weight[idx]);
      weight_sum += weight[idx];
    }
    Vec3d_addSelf(u, axis);
  }

  // Normalize u
  if (weight != NULL)
  {
    Vec3d_constMulSelf(u, 1.0/weight_sum);
  }
  else
  {
    Vec3d_constMulSelf(u, 1.0/n);
  }
  double u_norm = Vec3d_normalizeSelf(u);

  // Transform back by taking exponential of quaternion
  double angle = 2*u_norm;
  double A_KI[3][3];
  Mat3d_fromAxisAngle(A_KI, u, angle);

  // Extract euler angles
  Mat3d_toEulerAngles(avgEul, A_KI);
}

/*******************************************************************************
 * See header
 ******************************************************************************/
void Math_finiteDifferenceDerivative(MatNd* dfdq,
                                     void (*f)(MatNd*, const MatNd*, void*),
                                     void* data,
                                     const MatNd* q,
                                     double eps)
{
  RCHECK_MSG(dfdq->m>0, "dfdq: %d x %d", dfdq->m, dfdq->n);
  RCHECK_MSG(q->n==1, "State has more than 1 column: %d x %d", q->m, q->n);
  RCHECK_MSG(q->m==dfdq->n, "Wrong gradient size: dfdq->m=%d dfdq->n=%d "
             "x->m=%d x->n=%d", dfdq->m, dfdq->n, q->m, q->n);

  size_t dimQ = q->m;
  size_t dimX = dfdq->m;

  MatNd* x  = MatNd_create(dimX, 1);
  MatNd* dq = MatNd_create(dimQ, 1);
  MatNd* dx = MatNd_create(dimX, 1);

  MatNd_reshapeAndSetZero(dfdq, dimX, dimQ);


  // Evaluate function at initial state
  f(x, q, data);


  // Numerical approximation for each element of q
  for (size_t i = 0; i < dimQ; i++)
  {
    MatNd_copy(dq, q);
    MatNd_addToEle(dq, i, 0, eps);
    f(dx, dq, data);
    MatNd_subSelf(dx, x);
    MatNd_constMulSelf(dx, 1.0/eps);
    MatNd_copyColumn(dfdq, i, dx, 0);
  }


  // Reset data to the initial state
  f(x, q, data);


  // Clean up
  MatNd_destroy(x);
  MatNd_destroy(dx);
  MatNd_destroy(dq);
}
