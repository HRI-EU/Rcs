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
  RCHECK_MSG(upper - lower > 0, "Lower is larger or equal to upper!");

  if (randomNumberGeneratorInit==false)
  {
    Math_srand48Time(Timer_getTime());
  }

  double ele = drand48();
  ele *= upper - lower + 1.0;
  ele += lower - 0.5;

  int return_temp = round(ele);

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
  NLOG(0, "seed: %ld", seed);
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
 *
 * The squared distance between point and line is
 *
 * f = 0.5 (segPt + s*segDir - pt)^T (segPt + s*segDir - pt)
 *
 * with s being the line parameter in the range [0 ... segLength]
 *
 * The derivative df/ds is zero for the minimum distance, resulting in
 *
 * df/ds = (segPt + s*segDir - pt)^T * segDir := 0
 *       = segPt^T segDir + s*segDir^T segDir - pt^T segDir
 *
 * Solving for s yields
 *
 *      (pt^T  - segPt)^T segDir
 * s = ___________________________
 *       segDir^T segDir
 *
 ******************************************************************************/
double Math_sqrDistPointLine(const double pt[3],
                             const double segPt[3],
                             const double segDir[3],
                             double cpLine[3])
{
  double s, diff[3];
  Vec3d_sub(diff, pt, segPt);
  s = Vec3d_innerProduct(diff, segDir) / Vec3d_innerProduct(segDir, segDir);
  Vec3d_constMulAndAdd(cpLine, segPt, segDir, s);

  return Vec3d_sqrDistance(pt, cpLine);
}

/*******************************************************************************
 * Same as Math_sqrDistPointLine(), except that s is clipped to the extents
 ******************************************************************************/
double Math_sqrDistPointLineseg(const double pt[3],
                                const double segPt[3],
                                const double segDir[3],
                                const double segLength,
                                double cpLine[3])
{
  double s, diff[3];
  Vec3d_sub(diff, pt, segPt);
  s = Vec3d_innerProduct(diff, segDir) / Vec3d_innerProduct(segDir, segDir);
  s = Math_clip(s, 0, segLength);
  Vec3d_constMulAndAdd(cpLine, segPt, segDir, s);

  return Vec3d_sqrDistance(pt, cpLine);
}

/*******************************************************************************
 * See header
 ******************************************************************************/
double Math_distPointCapsule(const double pt[3],
                             const double capsulePt[3],
                             const double capsuleDir[3],
                             const double capsuleLength,
                             const double capsuleRadius,
                             double cpCapsule[3],
                             double nPtLine[3])
{
  double sqrDist = Math_sqrDistPointLineseg(pt, capsulePt, capsuleDir,
                                            capsuleLength, cpCapsule);

  // Shift capsule point onto capsule hull. Here we need to consider the
  // degenerate case of the point lying on the capsule axis. In this case, the
  // normal vector is zero length, and we can select any radial direction. We
  // decide for one of them.
  if (sqrDist == 0.0)
  {
    Vec3d_orthonormalVec(nPtLine, capsuleDir);
  }
  else
  {
    Vec3d_sub(nPtLine, cpCapsule, pt);
    Vec3d_normalizeSelf(nPtLine);
  }

  Vec3d_constMulAndAddSelf(cpCapsule, nPtLine, -capsuleRadius);

  return sqrt(sqrDist) - capsuleRadius;
}

/*******************************************************************************
 *
 ******************************************************************************/
static inline bool Math_pointLeftorOnLine2D(const double p[2],
                                            const double l0[2],
                                            const double l1[2])
{
  double det = (l1[0]-l0[0])*(p[1]-l0[1]) - (l1[1]-l0[1])*(p[0]-l0[0]);

  NLOG(0, "[%.3f %.3f] is %s of [%.2f %.2f %.2f %.2f]",
       p[0], p[1],
       (det<=0.0) ? "LEFT" : "RIGHT",
       l0[0], l0[1], l1[0], l1[1]);

  return (det<=0.0) ? false : true;
}

static bool Math_pointInsideOrOnConvexPolygon2D(const double pt[2],
                                                const double polygon[][2],
                                                unsigned int nVertices)
{
  for (unsigned int i=0; i<nVertices; ++i)
  {
    const double* lPt0 = polygon[i];
    const double* lPt1 = polygon[(i+1)%nVertices];

    if (Math_pointLeftorOnLine2D(pt, lPt0, lPt1) == false)
    {
      return false;
    }
  }

  return true;
}

static double Math_sqrDistPointLineseg2D(const double point[2],
                                         const double linePt0[2],
                                         const double linePt1[2],
                                         double cpLine[2])
{
  double pt3D[3], segPt[3], segDir[3], cp[3];

  Vec3d_set(pt3D, point[0], point[1], 0.0);
  Vec3d_set(segPt, linePt0[0], linePt0[1], 0.0);
  Vec3d_set(segDir, linePt1[0]-linePt0[0], linePt1[1]-linePt0[1], 0.0);
  double segLength = Vec3d_normalizeSelf(segDir);

  double d = Math_sqrDistPointLineseg(point, segPt, segDir, segLength, cp);

  cpLine[0] = cp[0];
  cpLine[1] = cp[1];

  return d;
}

static void Math_centroidConvexPolygon2D(double centroid[2],
                                         const double poly[][2],
                                         unsigned int nVertices)
{
  centroid[0] = 0.0;
  centroid[1] = 0.0;

  for (unsigned int i=0; i<nVertices; ++i)
  {
    centroid[0] += poly[i][0];
    centroid[1] += poly[i][1];
  }

  centroid[0] /= nVertices;
  centroid[1] /= nVertices;
}

double Math_distPointConvexPolygon2D(const double pt[2],
                                     const double poly[][2],
                                     unsigned int nVertices,
                                     double cpPoly[2],
                                     double nPoly[2])
{
  RCHECK_MSG(nVertices>0, "A polygon needs more than 0 vertices");

  double distance, cpTmp[2];
  double* cp = cpPoly ? cpPoly : cpTmp;

  if (nVertices == 1)
  {
    distance = sqrt(VecNd_sqrDiff(pt, poly[0], 2));
    cp[0] = poly[0][0];
    cp[1] = poly[0][1];

    // From the infinity solutions for the normal, we set it to [0 1].
    if (nPoly!=NULL)
    {
      nPoly[0] = 0.0;
      nPoly[1] = 1.0;
    }
  }
  else
  {
    // Iterate over all line segments and find closest distance.
    distance = Math_infinity();

    for (unsigned int i = 0; i < nVertices; i++)
    {
      const double* lPt0 = poly[i];
      const double* lPt1 = poly[(i+1)%nVertices];

      double tmp[2];
      double d_i = Math_sqrDistPointLineseg2D(pt, lPt0, lPt1, tmp);

      if (d_i < distance)
      {
        distance = d_i;
        cp[0] = tmp[0];
        cp[1] = tmp[1];

        if (nPoly!=NULL)
        {
          // Normal vector in counter-clockwise direction: [ -ay ax ]
          // and therefore in clockwise direction:  [ ay -ax ]
          double ax = lPt1[0]-lPt0[0];
          double ay = lPt1[1]-lPt0[1];
          nPoly[0] = ay;
          nPoly[1] = -ax;
          double len = VecNd_normalizeSelf(nPoly, 2);

          // If the segment points coincide, the segment length is 0 and we
          // fail to normalize the normal. In this case, we set the normal to
          // the connection of the polygon centroid to the degenerate segment
          // point. This might still fail, e.g. if the polygon consists of sevreal
          // coinciding points. In this case, we give up and exit with a fatal
          // error.
          if (len==0.0)
          {
            double centroid[2];
            Math_centroidConvexPolygon2D(centroid, poly, nVertices);
            VecNd_sub(nPoly, lPt0, centroid, 2);
            len = VecNd_normalizeSelf(nPoly, 2);
            if (len==0.0)
            {
              for (unsigned int j=0; j<nVertices; ++j)
              {
                fprintf(stderr, "%f %f\n", poly[j][0], poly[j][1]);
              }
              RFATAL("The polygon is pretty degenerate, please check it.");
            }
          }
        }   // (nPoly!=NULL)

      }
    }

    if (Math_pointInsideOrOnConvexPolygon2D(pt, poly, nVertices))
    {
      distance = -sqrt(distance);
      RLOG(1, "INSIDE");
    }
    else
    {
      distance = sqrt(distance);
      RLOG(1, "OUTSIDE");
    }
  }

  return distance;
}

/*******************************************************************************
 *
 ******************************************************************************/
double Math_sqrDistPointConvexPolygon(const double I_pt[3],
                                      const HTr* A_PI,
                                      const double poly[][2],
                                      unsigned int nVertices,
                                      double I_cpPoly[3],
                                      double I_nPoly[3])
{
  double sqrDist3D;

  // Transform point into the frame of reference of the polygon: Vertices lie
  // in the x-y plane, the plane normal is the z-axis of A_PI.
  double P_pt[3];
  Vec3d_invTransform(P_pt, A_PI, I_pt);

  // Compute the closest point and distance for the 2D projection.
  double P_cp[3];
  P_cp[2] = 0.0;
  double d2D = Math_distPointConvexPolygon2D(P_pt, poly, nVertices, P_cp, NULL);

  // If the projected point is outside the polygon, we keep the closest point
  // as the one at the polygon boundary, and set the z-component to 0.
  if (d2D >= 0.0)
  {
    sqrDist3D = Vec3d_sqrDistance(P_cp, P_pt);
    Vec3d_transform(I_cpPoly, A_PI, P_cp);
    Vec3d_sub(I_nPoly, I_pt, I_cpPoly);
    Vec3d_normalizeSelf(I_nPoly);
  }
  // If it is inside, we modify the closest point to be the projection of the
  // point on the polygon plane. This results in a distance to a "filled", and
  // not a "wired" polygon. The distance is ust the "height" (z-component) of
  // P_pt.
  else
  {
    sqrDist3D = P_pt[2]*P_pt[2];
    P_cp[0] = P_pt[0];
    P_cp[1] = P_pt[1];
    Vec3d_transform(I_cpPoly, A_PI, P_cp);
    Vec3d_sub(I_nPoly, I_pt, I_cpPoly);
    Vec3d_normalizeSelf(I_nPoly);
  }

  // Project the normal back to the "world" coordinates

  return sqrDist3D;
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
void Quat_toRotationMatrix(double A_BI[3][3], double q[4])
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
void Quat_fromRotationMatrix(double q[4], double rm[3][3])
{
  double t = 0.0;

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

  RCHECK(t>0.0);
  VecNd_constMulSelf(q, 0.5/sqrt(t), 4);
}

/*******************************************************************************
 *
 * The squared distance between two lines is
 *
 * f = 0.5 (segPt1 + s1*segDir1 - (segPt2 + s2*segDir2))^T (segPt1 + s1*segDir1 - (segPt2 + s2*segDir2))
 *   = 0.5 (segPt1 + s1*segDir1 - segPt2 - s2*segDir2)^T (segPt1 + s1*segDir1 - segPt2 - s2*segDir2)
 *
 * with s being the line parameter in the range [0 ... segLength]
 *
 * The derivative df/ds is zero for the minimum distance, resulting in
 *
 * df/ds1 = (segPt1 + s1*segDir1 - (segPt2 + s2*segDir2))^T * segDir1
 *        = sp1^T*sd1 + sd1^T*sd1*s1 - sp2^T*sd1 - sd2^T*sd1*s2
 *        = sd1^T*sd1*s1 - sd2^T*sd1*s2 + sp1^T*sd1 - sp2^T*sd1
 *        = sd1^T*sd1*s1 - sd2^T*sd1*s2 + (sp1-sp2)^T*sd1
 *
 * df/ds2 = -(segPt1 + s1*segDir1 - segPt2 - s2*segDir2)^T * segDir2
 *        = (-sp1 -s1*sd1 + sp2 + s2*sd2)^T *sd2
 *        = -sp1^T*sd2 - s1*sd1^T*sd2 + sp2^T*sd2 + s2*sd2^T*sd2
 *        = -sd1^T*sd2*s1 + sd2^T*sd2*s2 + (sp2-sp1)^T*sd2
 *        = -sd1^T*sd2*s1 + sd2^T*sd2*s2 - (sp1-sp2)^T*sd2
 *
 * Simple linear equation system:
 *
 *    /  sd1^T*sd1   -sd2^T*sd1 \  / s1 \     / -(sp1-sp2)^T*sd1 \
 *    |                         |  |    |  =  |                  |
 *    \ -sd1^T*sd2    sd2^T*sd2 /  \ s2 /     \  (sp1-sp2)^T*sd2 /
 *
 * We can write: A*s=b
 *
 * The inverse of a 2 x 2 matrix is
 *
 * | a11 a12 |-1             |  a22 -a12 |
 * | a21 a22 |    =  1/det * | -a21  a11 |
 *
 * with det  =  a11*a22 - a12*a21
 *
 ******************************************************************************/
double Math_inverse2D(double invA[2][2], double A[2][2])
{
  double det = A[0][0] * A[1][1] - A[0][1] * A[1][0];

  if (det != 0.0)
  {
    invA[0][0] =  A[1][1] / det;
    invA[0][1] = -A[0][1] / det;
    invA[1][0] = -A[1][0] / det;
    invA[1][1] =  A[0][0] / det;
  }

  return det;
}

double Math_sqrDistLineLine(const double lp1[3],
                            const double ld1[3],
                            const double lp2[3],
                            const double ld2[3],
                            double s[2])
{
  double A[2][2], invA[2][2], b[2];

  A[0][0] =  Vec3d_innerProduct(ld1, ld1);
  A[0][1] = -Vec3d_innerProduct(ld2, ld1);
  A[1][0] =  A[0][1];
  A[1][1] =  Vec3d_innerProduct(ld2, ld2);

  double lp1mlp2[3];
  Vec3d_sub(lp1mlp2, lp1, lp2);
  b[0] = -Vec3d_innerProduct(lp1mlp2, ld1);
  b[1] =  Vec3d_innerProduct(lp1mlp2, ld2);

  double det = Math_inverse2D(invA, A);

  if (det!=0.0)   // Lines are parallel or anti-parallel if the determinant is 0
  {
    s[0] = invA[0][0]*b[0] + invA[0][1]*b[1];
    s[1] = invA[1][0]*b[0] + invA[1][1]*b[1];
  }
  else
  {
    s[0] = 0.0;
    s[1] = 0.0;
  }

  return det;
}

double Math_sqrDistLinesegLineseg(const double segPt0[3],
                                  const double segDir0[3],
                                  const double segLength0,
                                  const double segPt1[3],
                                  const double segDir1[3],
                                  const double segLength1,
                                  double cp0_[3],
                                  double cp1_[3])
{
  // 0: Line0 - line1
  double s[2];
  double det = Math_sqrDistLineLine(segPt0, segDir0, segPt1, segDir1, s);
  Vec3d_constMulAndAdd(cp0_, segPt0, segDir0, Math_clip(s[0], 0.0, segLength0));
  Vec3d_constMulAndAdd(cp1_, segPt1, segDir1, Math_clip(s[1], 0.0, segLength1));
  double dLineLine = Vec3d_sqrDistance(cp0_, cp1_);

  // Early exit if s is within length of both segments.
  if ((s[0]>=0.0) && (s[0]<=segLength0) && (s[1]>=0.0) && (s[1]<=segLength1))
  {
    if (det == 0.0)
    {
      RLOG(1, "PARALLEL - implement special treatment: s0=%.1f s1=%.1f", s[0], s[1]);
    }
    else
    {
      RLOG(1, "Early exit");
    }
    return dLineLine;
  }

  double d[9], cp0[9][3], cp1[9][3];
  d[0] = dLineLine;
  Vec3d_copy(cp0[0], cp0_);
  Vec3d_copy(cp0[1], cp1_);

  double linePt0B[3], linePt1B[3];
  Vec3d_constMulAndAdd(linePt1B, segPt1, segDir1, segLength1);
  Vec3d_constMulAndAdd(linePt0B, segPt0, segDir0, segLength0);


  // 1: Line0 - linePt1A
  d[1] = Math_sqrDistPointLineseg(segPt1, segPt0, segDir0, segLength0, cp0[1]);
  Vec3d_copy(cp1[1], segPt1);

  // 2: Line0 - linePt1B
  d[2] = Math_sqrDistPointLineseg(linePt1B, segPt0, segDir0, segLength0, cp0[2]);
  Vec3d_copy(cp1[2], linePt1B);

  // 3: LinePt0A - line1
  d[3] = Math_sqrDistPointLineseg(segPt0, segPt1, segDir1, segLength1, cp1[3]);
  Vec3d_copy(cp0[3], segPt0);

  // 4: LinePt0A - linePt1A
  d[4] = Vec3d_sqrDistance(segPt0, segPt1);
  Vec3d_copy(cp0[4], segPt0);
  Vec3d_copy(cp1[4], segPt1);

  // 5: LinePt0A - linePt1B
  d[5] = Vec3d_sqrDistance(segPt0, linePt1B);
  Vec3d_copy(cp0[5], segPt0);
  Vec3d_copy(cp1[5], linePt1B);

  // 6: LinePt0B - line1
  d[6] = Math_sqrDistPointLineseg(segPt0, segPt1, segDir1, segLength1, cp1[6]);
  Vec3d_copy(cp0[6], segPt0);

  // 7: LinePt0B - linePt1A
  d[7] = Vec3d_sqrDistance(linePt0B, segPt1);
  Vec3d_copy(cp0[7], linePt0B);
  Vec3d_copy(cp1[7], segPt1);

  // 8: LinePt0B - linePt1B
  d[8] = Vec3d_sqrDistance(linePt0B, linePt1B);
  Vec3d_copy(cp0[8], linePt0B);
  Vec3d_copy(cp1[8], linePt1B);

  int minIdx = VecNd_indexMin(&d[1], 8);   // Line-line already returned

  Vec3d_copy(cp0_, cp0[minIdx]);
  Vec3d_copy(cp1_, cp1[minIdx]);

  return d[minIdx];
}


/*******************************************************************************
 * See header.
 ******************************************************************************/
double Math_distCapsuleCapsule(const double linePt0[3],
                               const double lineDir[3],
                               const double l0,
                               const double r0,
                               const double linePt1[3],
                               const double lineDir1[3],
                               const double l1,
                               const double r1,
                               double cp0_[3],
                               double cp1_[3],
                               double n01_[3])
{
  double cpTmp0[3], cpTmp1[3], nTmp01[3], sqrLinesegDist, capsuleDist;

  double* cp0 = (cp0_==NULL) ? cpTmp0 : cp0_;
  double* cp1 = (cp1_==NULL) ? cpTmp1 : cp1_;
  double* n01 = (n01_==NULL) ? nTmp01 : n01_;

  sqrLinesegDist = Math_sqrDistLinesegLineseg(linePt0, lineDir, l0,
                                              linePt1, lineDir1, l1,
                                              cp0, cp1);

  Vec3d_sub(n01, cp1, cp0);
  Vec3d_normalizeSelf(n01);

  // Consider the radius of the capsules for the distance
  capsuleDist = sqrt(sqrLinesegDist) - (r0 + r1);

  // Normalized direction vector from closest connection line0 -> line1
  if ((cp0_!=NULL) || (cp1_!=NULL) || (n01_!=NULL))
  {
    Vec3d_sub(n01, cp1, cp0);
    Vec3d_normalizeSelf(n01);
    Vec3d_constMulAndAddSelf(cp0, n01, r0);
    Vec3d_constMulAndAddSelf(cp1, n01, -r1);
  }

  return capsuleDist;
}
