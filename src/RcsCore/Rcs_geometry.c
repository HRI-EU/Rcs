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

#include "Rcs_geometry.h"
#include "Rcs_Vec3d.h"
#include "Rcs_VecNd.h"
#include "Rcs_basicMath.h"
#include "Rcs_macros.h"



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

  s = Vec3d_innerProduct(diff, segDir) / Vec3d_sqrLength(segDir);
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
                                                double polygon[][2],
                                                unsigned int nVertices)
{
  if (nVertices==0)
  {
    if (pt[0]==0.0 && pt[1]==0.0)
    {
      return true;
    }

    return false;
  }

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

  double d = Math_sqrDistPointLineseg(pt3D, segPt, segDir, segLength, cp);

  if (cpLine != NULL)
  {
    cpLine[0] = cp[0];
    cpLine[1] = cp[1];
  }

  return d;
}

static void Math_centroidConvexPolygon2D(double centroid[2],
                                         double poly[][2],
                                         unsigned int nVertices)
{
  centroid[0] = 0.0;
  centroid[1] = 0.0;

  for (unsigned int i=0; i<nVertices; ++i)
  {
    centroid[0] += poly[i][0];
    centroid[1] += poly[i][1];
  }

  if (nVertices>0.0)
  {
  centroid[0] /= nVertices;
  centroid[1] /= nVertices;
}
}

bool Math_checkPolygon2D(double polygon[][2], unsigned int nVertices)
{
  double centroid[2];
  Math_centroidConvexPolygon2D(centroid, polygon, nVertices);
  return Math_pointInsideOrOnConvexPolygon2D(centroid, polygon, nVertices);
}

double Math_distPointConvexPolygon2D(const double pt[2],
                                     double poly[][2],
                                     unsigned int nVertices,
                                     double cpPoly[2],
                                     double nPoly[2])
{
  RCHECK_MSG(nVertices>0, "A polygon needs more than 0 vertices");

  if (nVertices == 1)
  {
    if (cpPoly!=NULL)
    {
      cpPoly[0] = poly[0][0];
      cpPoly[1] = poly[0][1];
    }

    // From the infinity solutions for the normal, we set it to [0 1].
    if (nPoly!=NULL)
    {
      nPoly[0] = 0.0;
      nPoly[1] = 1.0;
    }

    return sqrt(VecNd_sqrDiff(pt, poly[0], 2));
  }

  // Iterate over all line segments and find closest distance.
  double cpTmp[2], tmp[2];
  double* cp = cpPoly ? cpPoly : cpTmp;
  bool inside = Math_pointInsideOrOnConvexPolygon2D(pt, poly, nVertices);
  double ptInsideSign = (inside==true) ? -1.0 : 1.0;
  double distance = 1.0e8;//Math_infinity();

  for (unsigned int i = 0; i < nVertices; i++)
  {
    const double* lPt0 = poly[i];
    const double* lPt1 = poly[(i+1)%nVertices];
    double d_i = Math_sqrDistPointLineseg2D(pt, lPt0, lPt1, tmp);

    if (d_i < distance)
    {
      distance = d_i;
      cp[0] = tmp[0];
      cp[1] = tmp[1];
    }
  }

  if (nPoly!=NULL)
  {
    VecNd_sub(nPoly, pt, cp, 2);
    VecNd_normalizeSelf(nPoly, 2);
    VecNd_constMulSelf(nPoly, ptInsideSign, 2);
  }

  return ptInsideSign*sqrt(distance);
}


/*******************************************************************************
 *
 ******************************************************************************/
double Math_sqrDistPointConvexPolygon(const double I_pt[3],
                                      const HTr* A_PI,
                                      double poly[][2],
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

/*******************************************************************************
 * See header.
 ******************************************************************************/
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
      double p01[3];
      Vec3d_sub(p01, segPt1, segPt0);
      double a = Vec3d_innerProduct(p01, segDir0);

      if (a>=0.0)
      {
        if (a>=segLength0)   // End point seg0 - start point s1
        {

        }
        else   // Positive overlap
        {
        }
      }
      else
      {
      }
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

/*******************************************************************************
 *
******************************************************************************/
void Math_Cart2Cyl(const double p[3], double* radialDist, double* azimuth,
                   double* height)
{
  *radialDist = sqrt(p[0]*p[0]+p[1]*p[1]);
  *height = p[2];

  if ((p[0]==0.0) && (p[1]==0.0))
  {
    *azimuth = 0.0;
  }
  else if (p[0]>=0.0)
  {
    *azimuth = asin(p[1]/(*radialDist));
  }
  else
  {
    *azimuth = M_PI - asin(p[1]/(*radialDist));
  }
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
 * Computes the distance between a point and a spinning polygon. The polygon's
 * transformation is given by A_PI. Vector I_cp holds the closest point of the
 * distance query. It may be NULL.
 ******************************************************************************/
double Math_distPointSpinningPolygon(const double I_pt[3],
                                     const HTr* A_PI,
                                     double poly[][2],
                                     unsigned int nVertices,
                                     double I_cp[3])
{
  RCHECK(Math_checkPolygon2D(poly, nVertices));

  // Transform point into polygon frame
  double P_pt[3];
  Vec3d_invTransform(P_pt, A_PI, I_pt);

  // Transform into cylinder coordinates
  double pt[2], angle;
  Math_Cart2Cyl(P_pt, &pt[0], &angle, &pt[1]);

  double P_cp2D[2];
  double distance = Math_distPointConvexPolygon2D(pt, poly, nVertices,
                                                  P_cp2D, NULL);

  if (I_cp != NULL)
  {
    // Transform back to Cartesian coordinates
    Math_Cyl2Cart(P_cp2D[0], angle, P_cp2D[1], I_cp);

    // Transform back into global reference frame
    Vec3d_transformSelf(I_cp, A_PI);
  }

  return distance;
}

/*******************************************************************************
 * Computes the distance between a point and a cone. The cone's transformation
 * is given by A_cone and its dimensions by height and radius. Vectors I_cp
 * holds the closest point of the distance query. It may be NULL.
 ******************************************************************************/
double Math_distPointCone(const double I_pt[3],
                          const HTr* A_CI,
                          double height,
                          double radius,
                          double I_cp[3])
{
  double poly[3][2];
  poly[0][0] = -radius;
  poly[0][1] = 0.0;
  poly[1][0] =  radius;
  poly[1][1] = 0.0;
  poly[2][0] =  0.0;
  poly[2][1] = height;

  return Math_distPointSpinningPolygon(I_pt, A_CI, poly, 3, I_cp);
}

/*******************************************************************************
 * Computes the distance between a point and a cylinder. The cylinder's
 * transformation is given by A_CI and its dimensions by height and radius.
 * Vector I_cp holds the closest point of the distance query. It may be NULL.
 ******************************************************************************/
double Math_distPointCylinder(const double I_pt[3],
                              const HTr* A_CI,
                              double height,
                              double radius,
                              double I_cp[3])
{
  double poly[4][2];
  poly[0][0] = -radius;
  poly[0][1] = -0.5*height;
  poly[1][0] =  radius;
  poly[1][1] = -0.5*height;
  poly[2][0] =  radius;
  poly[2][1] =  0.5*height;
  poly[3][0] = -radius;
  poly[3][1] =  0.5*height;

  return Math_distPointSpinningPolygon(I_pt, A_CI, poly, 4, I_cp);
}
