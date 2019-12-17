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

#include <limits.h>



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
 *
 ******************************************************************************/
double Math_distPointPlane(const double pt[3],
                           const double planePt[3],
                           const double planeNormal[3],
                           double cpPlane[3])
{
  double dist, diff[3];

  Vec3d_sub(diff, pt, planePt);
  dist = Vec3d_innerProduct(diff, planeNormal);

  if (cpPlane != NULL)
  {
    Vec3d_constMulAndAdd(cpPlane, pt, planeNormal, -dist);
  }

  return dist;
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

/*******************************************************************************
 *
 ******************************************************************************/
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

/*******************************************************************************
 *
 ******************************************************************************/
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

/*******************************************************************************
 *
 ******************************************************************************/
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

/*******************************************************************************
 *
 ******************************************************************************/
bool Math_checkPolygon2D(double polygon[][2], unsigned int nVertices)
{
  double centroid[2];
  Math_centroidConvexPolygon2D(centroid, polygon, nVertices);
  return Math_pointInsideOrOnConvexPolygon2D(centroid, polygon, nVertices);
}

/*******************************************************************************
 *
 ******************************************************************************/
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
  //bool inside = Math_pointInsideOrOnConvexPolygon2D(pt, poly, nVertices);
  int res = Math_pointInsideOrOnPolygon2D(pt, poly, nVertices);
  bool inside = res%2!=0;
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
double Math_lengthPolygon2D(double polygon[][2], unsigned int nVertices)
{
  double len = 0.0;

  for (unsigned int i=0; i<nVertices; ++i)
  {
    const unsigned int iNext = (i==nVertices-1) ? 0 : i+1;
    const double dx = polygon[iNext][0]-polygon[i][0];
    const double dy = polygon[iNext][1]-polygon[i][1];
    len += sqrt(dx*dx+dy*dy);
  }

  return len;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Math_interpolatePolygon2D(double res[2], double polygon[][2],
                               unsigned int nVertices, double s)
{
  if ((s<=0.0) || (s>=1.0))
    {
      res[0] = polygon[0][0];
      res[1] = polygon[0][1];
    }

  const double len = Math_lengthPolygon2D(polygon, nVertices);

  double len_s = 0.0;

  for (unsigned int i=0; i<nVertices; ++i)
  {
    const unsigned int iNext = (i==nVertices-1) ? 0 : i+1;
    const double dx = polygon[iNext][0]-polygon[i][0];
    const double dy = polygon[iNext][1]-polygon[i][1];
    const double s0 = len_s/len;
    len_s += sqrt(dx*dx+dy*dy);
    const double s1 = len_s/len;

    if ((s>s0) && (s<=s1))
      {
        const double ds = (s - s0)/(s1 - s0);
        res[0] = polygon[i][0] + ds*dx;
        res[1] = polygon[i][1] + ds*dy;
        return;
      }
  }

  // In case we get here, it's due to numerical issues and we are at the end.
  res[0] = polygon[0][0];
  res[1] = polygon[0][1];
}

/*******************************************************************************
 * Trivial but slow
 ******************************************************************************/
void Math_resamplePolygon2D(double polyOut[][2], unsigned int nvOut,
                            double polyIn[][2], unsigned int nvIn)
{
  const double ds = 1.0/nvOut;
  double s = 0.0;

  for (unsigned int i=0; i<nvOut; ++i)
  {
    Math_interpolatePolygon2D(polyOut[i], polyIn, nvIn, s);
    s += ds;
  }
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
static double Math_inverse2D(double invA[2][2], double A[2][2])
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
double Math_sqrDistLinesegLineseg_old(const double segPt0[3],
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

/******************************************************************************
 *  Adapted from: Wildmagic library (version 5.8)
 *  Geometric Tools LLC, Redmond WA 98052
 *  Copyright (c) 1998-2015
 *  Distributed under the Boost Software License, Version 1.0.
 *  http://www.boost.org/LICENSE_1_0.txt
 *  http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
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
  const double zeroTol = 1.0e-8;
  double cpTmp0[3], cpTmp1[3], diff[3], segCntr0[3], segCntr1[3];
  double s0, s1, sqrDist, extDet0, extDet1, tmpS0, tmpS1;
  double hExtent0 = 0.5*segLength0;
  double hExtent1 = 0.5*segLength1;
  double* cp0 = (cp0_==NULL) ? cpTmp0 : cp0_;
  double* cp1 = (cp1_==NULL) ? cpTmp1 : cp1_;

  Vec3d_constMulAndAdd(segCntr0, segPt0, segDir0, hExtent0);
  Vec3d_constMulAndAdd(segCntr1, segPt1, segDir1, hExtent1);

  // Call point-lineseg functions if length is 0
  if (segLength0==0.0)
  {
    sqrDist = Math_sqrDistPointLineseg(segPt0, segPt1, segDir1, segLength1,
                                       cp1_);
    Vec3d_copy(cp0, segPt0);
    return sqrDist;
  }

  Vec3d_sub(diff, segCntr0, segCntr1);

  double a01 = -Vec3d_innerProduct(segDir0, segDir1);
  double b0  =  Vec3d_innerProduct(diff, segDir0);
  double b1  = -Vec3d_innerProduct(diff, segDir1);
  double c   =  Vec3d_sqrLength(diff);
  double det = fabs(1.0 - a01*a01);

  if (det >= zeroTol)
  {
    // Segments are not parallel.
    s0 = a01*b1 - b0;
    s1 = a01*b0 - b1;
    extDet0 = hExtent0*det;
    extDet1 = hExtent1*det;

    if (s0 >= -extDet0)
    {
      if (s0 <= extDet0)
      {
        if (s1 >= -extDet1)
        {
          if (s1 <= extDet1)  // region 0 (interior)
          {
            // Minimum at interior points of segments.
            double invDet = 1.0/det;
            s0 *= invDet;
            s1 *= invDet;
            sqrDist = s0*(s0 + a01*s1 + 2.0*b0) +
                      s1*(a01*s0 + s1 + 2.0*b1) + c;
          }
          else  // region 3 (side)
          {
            s1 = hExtent1;
            tmpS0 = -(a01*s1 + b0);
            if (tmpS0 < -hExtent0)
            {
              s0 = -hExtent0;
              sqrDist = s0*(s0 - 2.0*tmpS0) +
                        s1*(s1 + (2.0)*b1) + c;
            }
            else if (tmpS0 <= hExtent0)
            {
              s0 = tmpS0;
              sqrDist = -s0*s0 + s1*(s1 + 2.0*b1) + c;
            }
            else
            {
              s0 = hExtent0;
              sqrDist = s0*(s0 - 2.0*tmpS0) + s1*(s1 + 2.0*b1) + c;
            }
          }
        }
        else  // region 7 (side)
        {
          s1 = -hExtent1;
          tmpS0 = -(a01*s1 + b0);
          if (tmpS0 < -hExtent0)
          {
            s0 = -hExtent0;
            sqrDist = s0*(s0 - 2.0*tmpS0) + s1*(s1 + 2.0*b1) + c;
          }
          else if (tmpS0 <= hExtent0)
          {
            s0 = tmpS0;
            sqrDist = -s0*s0 + s1*(s1 + 2.0*b1) + c;
          }
          else
          {
            s0 = hExtent0;
            sqrDist = s0*(s0 - 2.0*tmpS0) + s1*(s1 + 2.0*b1) + c;
          }
        }
      }
      else
      {
        if (s1 >= -extDet1)
        {
          if (s1 <= extDet1)  // region 1 (side)
          {
            s0 = hExtent0;
            tmpS1 = -(a01*s0 + b1);
            if (tmpS1 < -hExtent1)
            {
              s1 = -hExtent1;
              sqrDist = s1*(s1 - 2.0*tmpS1) + s0*(s0 + 2.0*b0) + c;
            }
            else if (tmpS1 <= hExtent1)
            {
              s1 = tmpS1;
              sqrDist = -s1*s1 + s0*(s0 + 2.0*b0) + c;
            }
            else
            {
              s1 = hExtent1;
              sqrDist = s1*(s1 - 2.0*tmpS1) + s0*(s0 + 2.0*b0) + c;
            }
          }
          else  // region 2 (corner)
          {
            s1 = hExtent1;
            tmpS0 = -(a01*s1 + b0);
            if (tmpS0 < -hExtent0)
            {
              s0 = -hExtent0;
              sqrDist = s0*(s0 - 2.0*tmpS0) + s1*(s1 + 2.0*b1) + c;
            }
            else if (tmpS0 <= hExtent0)
            {
              s0 = tmpS0;
              sqrDist = -s0*s0 + s1*(s1 + 2.0*b1) + c;
            }
            else
            {
              s0 = hExtent0;
              tmpS1 = -(a01*s0 + b1);
              if (tmpS1 < -hExtent1)
              {
                s1 = -hExtent1;
                sqrDist = s1*(s1 - 2.0*tmpS1) +
                          s0*(s0 + 2.0*b0) + c;
              }
              else if (tmpS1 <= hExtent1)
              {
                s1 = tmpS1;
                sqrDist = -s1*s1 + s0*(s0 + 2.0*b0) + c;
              }
              else
              {
                s1 = hExtent1;
                sqrDist = s1*(s1 - 2.0*tmpS1) +
                          s0*(s0 + 2.0*b0) + c;
              }
            }
          }
        }
        else  // region 8 (corner)
        {
          s1 = -hExtent1;
          tmpS0 = -(a01*s1 + b0);
          if (tmpS0 < -hExtent0)
          {
            s0 = -hExtent0;
            sqrDist = s0*(s0 - 2.0*tmpS0) + s1*(s1 + 2.0*b1) + c;
          }
          else if (tmpS0 <= hExtent0)
          {
            s0 = tmpS0;
            sqrDist = -s0*s0 + s1*(s1 + 2.0*b1) + c;
          }
          else
          {
            s0 = hExtent0;
            tmpS1 = -(a01*s0 + b1);
            if (tmpS1 > hExtent1)
            {
              s1 = hExtent1;
              sqrDist = s1*(s1 - 2.0*tmpS1) + s0*(s0 + 2.0*b0) + c;
            }
            else if (tmpS1 >= -hExtent1)
            {
              s1 = tmpS1;
              sqrDist = -s1*s1 + s0*(s0 + 2.0*b0) + c;
            }
            else
            {
              s1 = -hExtent1;
              sqrDist = s1*(s1 - 2.0*tmpS1) + s0*(s0 + 2.0*b0) + c;
            }
          }
        }
      }
    }
    else
    {
      if (s1 >= -extDet1)
      {
        if (s1 <= extDet1)  // region 5 (side)
        {
          s0 = -hExtent0;
          tmpS1 = -(a01*s0 + b1);
          if (tmpS1 < -hExtent1)
          {
            s1 = -hExtent1;
            sqrDist = s1*(s1 - 2.0*tmpS1) + s0*(s0 + 2.0*b0) + c;
          }
          else if (tmpS1 <= hExtent1)
          {
            s1 = tmpS1;
            sqrDist = -s1*s1 + s0*(s0 + 2.0*b0) + c;
          }
          else
          {
            s1 = hExtent1;
            sqrDist = s1*(s1 - 2.0*tmpS1) + s0*(s0 + 2.0*b0) + c;
          }
        }
        else  // region 4 (corner)
        {
          s1 = hExtent1;
          tmpS0 = -(a01*s1 + b0);
          if (tmpS0 > hExtent0)
          {
            s0 = hExtent0;
            sqrDist = s0*(s0 - 2.0*tmpS0) + s1*(s1 + 2.0*b1) + c;
          }
          else if (tmpS0 >= -hExtent0)
          {
            s0 = tmpS0;
            sqrDist = -s0*s0 + s1*(s1 + 2.0*b1) + c;
          }
          else
          {
            s0 = -hExtent0;
            tmpS1 = -(a01*s0 + b1);
            if (tmpS1 < -hExtent1)
            {
              s1 = -hExtent1;
              sqrDist = s1*(s1 - 2.0*tmpS1) + s0*(s0 + 2.0*b0) + c;
            }
            else if (tmpS1 <= hExtent1)
            {
              s1 = tmpS1;
              sqrDist = -s1*s1 + s0*(s0 + 2.0*b0) + c;
            }
            else
            {
              s1 = hExtent1;
              sqrDist = s1*(s1 - 2.0*tmpS1) + s0*(s0 + 2.0*b0) + c;
            }
          }
        }
      }
      else   // region 6 (corner)
      {
        s1 = -hExtent1;
        tmpS0 = -(a01*s1 + b0);
        if (tmpS0 > hExtent0)
        {
          s0 = hExtent0;
          sqrDist = s0*(s0 - 2.0*tmpS0) + s1*(s1 + 2.0*b1) + c;
        }
        else if (tmpS0 >= -hExtent0)
        {
          s0 = tmpS0;
          sqrDist = -s0*s0 + s1*(s1 + 2.0*b1) + c;
        }
        else
        {
          s0 = -hExtent0;
          tmpS1 = -(a01*s0 + b1);
          if (tmpS1 < -hExtent1)
          {
            s1 = -hExtent1;
            sqrDist = s1*(s1 - 2.0*tmpS1) + s0*(s0 + 2.0*b0) + c;
          }
          else if (tmpS1 <= hExtent1)
          {
            s1 = tmpS1;
            sqrDist = -s1*s1 + s0*(s0 + 2.0*b0) + c;
          }
          else
          {
            s1 = hExtent1;
            sqrDist = s1*(s1 - 2.0*tmpS1) + s0*(s0 + 2.0*b0) + c;
          }
        }
      }
    }
  }
  else
  {
    // The segments are parallel.  The average b0 term is designed to
    // ensure symmetry of the function.  That is, dist(seg0,seg1) and
    // dist(seg1,seg0) should produce the same number.
    double e0pe1 = hExtent0 + hExtent1;
    double sign = (a01 > 0.0 ? -1.0 : 1.0);
    double b0Avr = (0.5)*(b0 - sign*b1);
    double lambda = -b0Avr;

    if (lambda < -e0pe1)
    {
      lambda = -e0pe1;
    }
    else if (lambda > e0pe1)
    {
      lambda = e0pe1;
    }

    s1 = -sign*lambda*hExtent1/e0pe1;
    s0 = lambda + sign*s1;
    sqrDist = lambda*(lambda + 2.0*b0Avr) + c;
  }

  if ((cp0_!=NULL) || (cp1_!=NULL))
  {
    Vec3d_constMulAndAdd(cp0, segCntr0, segDir0, s0);
    Vec3d_constMulAndAdd(cp1, segCntr1, segDir1, s1);
  }

  // Account for numerical round-off errors.
  return (sqrDist < 0.0) ? 0.0 : sqrDist;
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

double Math_distPointBox(const double I_p[3],
                         const HTr* A_box,
                         const double extents[3],
                         double cpBox[3],
                         double nBox[3])
{
  int inside = 0;
  double sgn = 1.0, halfExt[3];
  Vec3d_constMul(halfExt, extents, 0.5);
  Vec3d_setZero(nBox);

  // Transform point into box frame
  Vec3d_invTransform(cpBox, A_box, I_p);

  // If one component of the point is outside the box, project it onto box face.
  for (int i = 0; i < 3; ++i)
  {
    if (cpBox[i] < -halfExt[i])
    {
      cpBox[i] = -halfExt[i];
      nBox[i] = -1.0;
    }
    else if (cpBox[i] > halfExt[i])
    {
      cpBox[i] = halfExt[i];
      nBox[i] = 1.0;
    }
    else
    {
      inside++;
    }
  }

  // Handle case when point is contained in box:  Find the closest face.
  if (inside == 3)
  {
    sgn = -1.0;
    double sgn_i[3], tmp[3];

    for (int i = 0; i < 3; ++i)
    {
      double d1 = halfExt[i] - cpBox[i];
      double d2 = halfExt[i] + cpBox[i];
      tmp[i] = fmin(d1, d2);
      sgn_i[i] = (d1 < d2) ? 1.0 : -1.0;
    }

    int idx = VecNd_indexMin(tmp, 3);
    cpBox[idx] = sgn_i[idx] * halfExt[idx];

    // In the case the point is contained, the closest point will always lie on
    // a face, and never on an edge or corner. Therefore, the normal will be an
    // elementary vector in the box frame. We need to rotate this into the
    // world frame.
    nBox[idx] = sgn_i[idx];
    Vec3d_transRotateSelf(nBox, (double(*)[3]) A_box->rot);
  }

  // Transform point back into world frame
  Vec3d_transformSelf(cpBox, A_box);

  // If the point lies outside the box, we do have a finite delta in the closest
  // points (see < and > operators instead of <= and >= for checking this). We
  // therefore can assume that the normalization of the difference between the
  // closest points always works.
  if (inside < 3)
  {
    Vec3d_sub(nBox, I_p, cpBox);
    Vec3d_normalizeSelf(nBox);
  }

  return sgn*Vec3d_distance(cpBox, I_p);
}

/******************************************************************************
 *  Adapted from: Wildmagic library (version 5.10): Wm5Vector2.inl
 *  Geometric Tools LLC, Redmond WA 98052
 *  Copyright (c) 1998-2015
 *  Distributed under the Boost Software License, Version 1.0.
 *  http://www.boost.org/LICENSE_1_0.txt
 *  http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
 ******************************************************************************/
static inline double dotPerp(const double a[2], const double b[2])
{
  // Returns Cross((x,y,0),(V.x,V.y,0)) = x*V.y - y*V.x
  return a[0]*b[1] - a[1]*b[0];
}

#define INTERSECT_NONE     (0)
#define INTERSECT_NORMAL   (1)
#define INTERSECT_VERTEX0  (2)
#define INTERSECT_VERTEX1  (3)
#define INTERSECT_COLINEAR (4)

/******************************************************************************
 *  Adapted from: Wildmagic library (version 5.10): IntrRay2Segment2.inl
 *  Geometric Tools LLC, Redmond WA 98052
 *  Copyright (c) 1998-2015
 *  Distributed under the Boost Software License, Version 1.0.
 *  http://www.boost.org/LICENSE_1_0.txt
 *  http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
 ******************************************************************************/
static int IntrRay2Segment2_Classify(const double rayOrigin[2],
                                     const double rayDir[2],
                                     const double segCenter[2],
                                     const double segDir[2],
                                     double s[2],
                                     double diff[2],
                                     double diffN[2])
{
  // The intersection of two lines is a solution to P0+s0*D0 = P1+s1*D1.
  // Rewrite this as s0*D0 - s1*D1 = P1 - P0 = Q.  If D0.Dot(Perp(D1)) = 0,
  // the lines are parallel.  Additionally, if Q.Dot(Perp(D1)) = 0, the
  // lines are the same.  If D0.Dot(Perp(D1)) is not zero, then
  //   s0 = Q.Dot(Perp(D1))/D0.Dot(Per(D1))
  // produces the point of intersection.  Also,
  //   s1 = Q.Dot(Perp(D0))/D0.Dot(Perp(D1))
  const double almostZero = 1.0e-8;
  double originDiff[2];
  originDiff[0] = segCenter[0] - rayOrigin[0];
  originDiff[1] = segCenter[1] - rayOrigin[1];

  if (diff)
  {
    diff[0] = originDiff[0];
    diff[1] = originDiff[1];
  }

  double D0DotPerpD1 = dotPerp(rayDir, segDir);

  if (fabs(D0DotPerpD1) > almostZero)
  {
    // Lines intersect in a single point.
    if (s)
    {
      double invD0DotPerpD1 = 1.0/D0DotPerpD1;
      double diffDotPerpD0 = dotPerp(originDiff, rayDir);
      double diffDotPerpD1 = dotPerp(originDiff, segDir);
      s[0] = diffDotPerpD1*invD0DotPerpD1;
      s[1] = diffDotPerpD0*invD0DotPerpD1;
    }

    return INTERSECT_NORMAL;
  }

  // Lines are parallel.
  VecNd_normalizeSelf(originDiff, 2);
  if (diffN)
  {
    diffN[0] = originDiff[0];
    diffN[1] = originDiff[1];
  }

  double diffNDotPerpD1 = dotPerp(originDiff, segDir);
  if (fabs(diffNDotPerpD1) <= almostZero)
  {
    // Lines are colinear.
    return INTERSECT_COLINEAR;
  }

  // Lines are parallel, but distinct.
  return INTERSECT_NONE;
}

/******************************************************************************
 *  Adapted from: Wildmagic library (version 5.10): IntrRay2Segment2.inl
 *  Geometric Tools LLC, Redmond WA 98052
 *  Copyright (c) 1998-2015
 *  Distributed under the Boost Software License, Version 1.0.
 *  http://www.boost.org/LICENSE_1_0.txt
 *  http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
 ******************************************************************************/
static int IntrRay2Segment2_Find(const double rayOrigin[2],
                                 const double rayDir[2],
                                 const double segCenter[2],
                                 const double segDir[2],
                                 double segLength,
                                 double intersectPt[2])
{
  const double almostZero = 1.0e-8;
  double s[2] = { 0.0, 0.0 };
  segLength *= 0.5; // half extents

  int intersectionType = IntrRay2Segment2_Classify(rayOrigin, rayDir,
                                                   segCenter, segDir,
                                                   s, NULL, NULL);

  if (intersectionType != INTERSECT_NORMAL)
  {
    return intersectionType;
  }

  // Test whether the line-line intersection is on the ray and on the
  // segment.
  if ((s[0] >= 0.0) && (fabs(s[1])<=segLength+almostZero))
  {
    if (fabs(s[1]+segLength)<=almostZero)
    {
      intersectionType = INTERSECT_VERTEX0;
    }
    else if (fabs(s[1]-segLength)<=almostZero)
    {
      intersectionType = INTERSECT_VERTEX1;
    }

    if (intersectPt)
    {
      intersectPt[0] = rayOrigin[0] + s[0]*rayDir[0];
      intersectPt[1] = rayOrigin[1] + s[0]*rayDir[1];
    }
  }
  else
  {
    intersectionType = INTERSECT_NONE;
  }

  return intersectionType;
}

int Math_intersectRayLineseg2D(const double rayOrigin[2],
                               const double rayDir[2],
                               const double segPt0[2],
                               const double segPt1[2],
                               double intersectPt[2])
{
  double segCenter[2], segDir[2];

  for (int i=0; i<2; ++i)
  {
    segCenter[i] = 0.5*(segPt0[i]+segPt1[i]);
    segDir[i] = segPt1[i] - segPt0[i];
  }
  const double segLength = VecNd_normalizeSelf(segDir, 2);

  int res = IntrRay2Segment2_Find(rayOrigin, rayDir,
                                  segCenter, segDir, segLength,
                                  intersectPt);

  if (res == INTERSECT_COLINEAR)
  {
    // Here we need to check if both vertex points are "behind" the ray
    // origin. In this case, we have colinearity, but no intersection.
    // vtx0 = rayPt + s*rayDir
    // s = (vtx0[idx]-rayPt[idx)/rayDir[1]
    // We determine idx as the index of the larger absolute value of
    // rayDir to avoid division by zero.
    const int idx = fabs(rayDir[0]) > fabs(rayDir[1]) ? 0 : 1;
    const double sv0 = (segPt0[idx]-rayOrigin[idx])/rayDir[idx];
    const double sv1 = (segPt1[idx]-rayOrigin[idx])/rayDir[idx];

    if ((sv0<0.0) && (sv1<0.0))
    {
      res = INTERSECT_NONE;
    }

  }

  return res;
}

/*******************************************************************************
 *
 ******************************************************************************/
int Math_pointInsideOrOnPolygon2D(const double pt[2],
                                  double polygon[][2],
                                  unsigned int nVertices)
{
  if (nVertices<1)
  {
    return -1;
  }

  // Ensure that first and last vertices don't coincide
  if ((polygon[0][0]==polygon[nVertices-1][0]) &&
      (polygon[0][1]==polygon[nVertices-1][1]))
  {
    nVertices--;
  }

  const unsigned int maxIter = 10;
  double rayDir[2] = { 1.0, 0.0 };

  for (unsigned int iter=0; iter<maxIter; ++iter)
  {
    unsigned int iCount = 0;
    bool success = true;

    for (unsigned int i=0; i<nVertices; ++i)
    {
      const unsigned int iNext = (i==nVertices-1) ? 0 : i+1;
      int res = Math_intersectRayLineseg2D(pt, rayDir, polygon[i],
                                           polygon[iNext], NULL);
      switch (res)
      {
        case 0:   // Nothing to do
          break;
        case 1:   // Increment count for each detected intersection
          iCount++;
          break;
        case 2: // Currently ignore degenerate intersections and loop again
        case 3:
        case 4:
          success = false;
          break;
        default:
          RFATAL("Unknown result: %d", res);
      }

    }   // for (unsigned int i=0; i<nVertices; ++i)

    if (success == true)
    {
      return iCount;
    }

    // In the unlikely case we hit a vertex or even got a colinear polygon
    // segment, we set a new random ray direction. Its values are set in an
    // interval not containing 0, so that we never (even if it is extremely
    // unlikely) get a zero vector.
    VecNd_setRandom(rayDir, 0.1, 1.0, 2);
    VecNd_normalizeSelf(rayDir, 2);
    RLOG(0, "Iteration %d failed", iter);

  }   // for (unsigned int iter=0;iter<maxIter; ++iter)

  // If we get here, we iterated maxIter times with different random ray
  // directions without getting a valid solution. That's extremely unlikely,
  // but bad luck in this case.
  return -1;
}

/*******************************************************************************
 *
 ******************************************************************************/
double Math_polyVertexDistance(double polygon[][2],
                               unsigned int nVertices,
                               unsigned int idx1,
                               unsigned int idx2)
{
  RCHECK_MSG(idx1<nVertices, "idx1: %d   nVertices: %d", idx1, nVertices);
  RCHECK_MSG(idx2<nVertices, "idx2: %d   nVertices: %d", idx2, nVertices);

  if (idx1==idx2 || nVertices<2)
  {
    return 0.0;
  }

  double dx, dy, len = 0.0;

  if (idx2 > idx1)
  {
    for (unsigned int i=idx1; i<idx2-1; ++i)
    {
      dx = polygon[i+1][0]-polygon[i][0];
      dy = polygon[i+1][1]-polygon[i][1];
      len += sqrt(dx*dx+dy*dy);
    }
  }
  else   // Wrap around first vertex
  {
    // Loop until last vertex
    for (unsigned int i=idx1; i<nVertices-2; ++i)
    {
      dx = polygon[i+1][0]-polygon[i][0];
      dy = polygon[i+1][1]-polygon[i][1];
      len += sqrt(dx*dx+dy*dy);
    }

    // Transistion between last and first vertex
    dx = polygon[nVertices-1][0]-polygon[0][0];
    dy = polygon[nVertices-1][1]-polygon[0][1];
    len += sqrt(dx*dx+dy*dy);

    // Loop from first vertex to final one
    if (idx2!=0)
    {
      for (unsigned int i=0; i<idx2-1; ++i)
      {
        dx = polygon[i+1][0]-polygon[i][0];
        dy = polygon[i+1][1]-polygon[i][1];
        len += sqrt(dx*dx+dy*dy);
      }
    }
  }

  return len;
}
