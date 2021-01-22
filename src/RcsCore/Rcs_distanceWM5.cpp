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

#include "Rcs_distanceWM5.h"
#include "Rcs_macros.h"

#if defined (USE_WM5)

#include "Rcs_shape.h"
#include "Rcs_typedef.h"
#include "Rcs_math.h"

#include <Wm5DistSegment3Box3.h>
#include <Wm5DistSegment3Segment3.h>
#include <Wm5DistSegment2Segment2.h>
#include <Wm5DistSegment3Rectangle3.h>
#include <Wm5DistRectangle3Rectangle3.h>
#include <Wm5DistPoint3Circle3.h>
#include <Wm5DistPoint3Segment3.h>
#include <Wm5DistPoint2Segment2.h>
#include <Wm5DistPoint3Rectangle3.h>
#include <Wm5DistPoint3Line3.h>
#include <Wm5DistLine3Circle3.h>
#include <Wm5IntrPlane3Plane3.h>
#include <Wm5PolynomialRoots.h>
#include <Wm5DistCircle3Circle3.h>

#include <limits>


typedef Wm5::Vector3d Vec3;


/*******************************************************************************
 * Computes the distance between point pt and a line defined by two points
 * linePt1 and linePt2. The function copies the closest point to pt that lies
 * on the line into it. The function returns the distance between point and
 * line.
 ******************************************************************************/
static inline double Rcs_distancePointLine(const double pt[3],
                                           const double linePt1[3],
                                           const double linePt2[3],
                                           double* closestLinePt)
{
  Vec3 vPt(pt[0], pt[1], pt[2]);
  Vec3 vl1 = Vec3(linePt1[0], linePt1[1], linePt1[2]);
  Vec3 vl2 = Vec3(linePt2[0], linePt2[1], linePt2[2]);
  Vec3 lineDir = vl2 - vl1;
  double len = lineDir.Normalize();

  if (len==0.0)
  {
    if (closestLinePt)
    {
      Vec3d_copy(closestLinePt, linePt1);
    }

    return 0.0;
  }

  Wm5::Line3<double> line(vl1, lineDir);

  Wm5::DistPoint3Line3d dist(vPt, line);
  double d = dist.Get();

  if (closestLinePt)
  {
    Vec3 clp = vl1 + lineDir * dist.GetLineParameter();
    Vec3d_set(closestLinePt, clp.X(), clp.Y(), clp.Z());
  }

  return d;
}

/*******************************************************************************
 * Computes the distance between a point and a line segment. The line segment
 * is defined by the points p1 and p2. Vectors cp0 and cp1 hold the closest
 * points of the distance query.
 ******************************************************************************/
static inline double Rcs_distancePointLineseg(const double point[3],
                                              const double linePt0[3],
                                              const double linePt1[3],
                                              double cp0[3],
                                              double cp1[3])
{
  Vec3 pt(point[0], point[1], point[2]);

  Wm5::Segment3d line(Vec3(linePt0[0], linePt0[1], linePt0[2]),
                      Vec3(linePt1[0], linePt1[1], linePt1[2]));

  Wm5::DistPoint3Segment3d dist(pt, line);

  double distance = dist.Get();

  if (cp0)
  {
    Vec3 vcp0 = dist.GetClosestPoint0();
    cp0[0] = vcp0[0];
    cp0[1] = vcp0[1];
    cp0[2] = vcp0[2];
  }

  if (cp1)
  {
    Vec3 vcp1 = dist.GetClosestPoint1();
    cp1[0] = vcp1[0];
    cp1[1] = vcp1[1];
    cp1[2] = vcp1[2];
  }

  return distance;
}

/*******************************************************************************
 * Computes the distance between a point and a circle. The circle's
 * transformation is given by A_c and its dimension by radius. Vectors
 * cp0 and cp1 hold the closest points of the distance query. They may be NULL.
 ******************************************************************************/
static double Rcs_distancePointCircle(const double p[3],
                                      const HTr* A_c,
                                      double radius,
                                      double cp0[3],
                                      double cp1[3])
{
  Vec3 pt(p[0], p[1], p[2]);

  Wm5::Circle3d circle(Vec3(A_c->org[0], A_c->org[1], A_c->org[2]),
                       Vec3(A_c->rot[0][0], A_c->rot[0][1], A_c->rot[0][2]),
                       Vec3(A_c->rot[1][0], A_c->rot[1][1], A_c->rot[1][2]),
                       Vec3(A_c->rot[2][0], A_c->rot[2][1], A_c->rot[2][2]),
                       radius);


  Wm5::DistPoint3Circle3d dist(pt, circle);

  double distance = dist.Get();

  // there is a special case that we have to handle first:
  // if the point is exactly on the normal of the torus, there are infite
  // closest points on the circle, vcp1 will then be set to (MAX_REAL,
  // MAX_REAL, MAX_REAL) we have to handle that (we just choose a specific
  // point on the torus)
  Vec3 vcp0 = dist.GetClosestPoint0();
  Vec3 vcp1 = dist.GetClosestPoint1();

  if (vcp1[0] == Wm5::Math<double>::MAX_REAL ||
      vcp1[1] == Wm5::Math<double>::MAX_REAL ||
      vcp1[2] == Wm5::Math<double>::MAX_REAL)
  {
    double pt[3] = {radius, 0.0, 0.0};
    double pt_t[3];
    Vec3d_transform(pt_t, A_c, pt);

    vcp1 = Vec3(pt_t[0], pt_t[1],pt_t[2]);
  }


  if (cp0)
  {
    cp0[0] = vcp0[0];
    cp0[1] = vcp0[1];
    cp0[2] = vcp0[2];
  }

  if (cp1)
  {
    cp1[0] = vcp1[0];
    cp1[1] = vcp1[1];
    cp1[2] = vcp1[2];
  }

  return distance;
}

/*******************************************************************************
 * Computes the distance between a line segment and a box. The line segment
 * is defined by the points p1 and p2. The box's transformation is given by
 * A_box, its extents are the length of each side of the box. Vectors cp0 and
 * cp1 hold the closest points of the distance query. They may be NULL.
 ******************************************************************************/
static double Rcs_distanceLinesegBox(const double linePt0[3],
                                     const double linePt1[3],
                                     const HTr* A_box,
                                     const double extents[3],
                                     double cp0[3],
                                     double cp1[3])
{
  Wm5::Segment3d line(Vec3(linePt0[0], linePt0[1], linePt0[2]),
                      Vec3(linePt1[0], linePt1[1], linePt1[2]));

  Wm5::Box3d box(Vec3(A_box->org[0], A_box->org[1], A_box->org[2]),
                 Vec3(A_box->rot[0][0], A_box->rot[0][1], A_box->rot[0][2]),
                 Vec3(A_box->rot[1][0], A_box->rot[1][1], A_box->rot[1][2]),
                 Vec3(A_box->rot[2][0], A_box->rot[2][1], A_box->rot[2][2]),
                 0.5 * extents[0], 0.5 * extents[1], 0.5 * extents[2]);

  Wm5::DistSegment3Box3d dist(line, box);
  double distance = dist.GetSquared();

  // In rare cases, the squared distance gets slightly negative for numerical
  // reasons. Here we correct it.
  if (distance<0.0)
  {
    distance = 0.0;
  }
  else
  {
    distance = sqrt(distance);
  }

  if (cp0)
  {
    Vec3 vcp0 = dist.GetClosestPoint0();
    cp0[0] = vcp0[0];
    cp0[1] = vcp0[1];
    cp0[2] = vcp0[2];
  }

  if (cp1)
  {
    Vec3 vcp1 = dist.GetClosestPoint1();
    cp1[0] = vcp1[0];
    cp1[1] = vcp1[1];
    cp1[2] = vcp1[2];
  }

  return distance;
}

/*******************************************************************************
 * Computes the distance between a line segment and a circle. The line segment
 * is defined by the points p1 and p2. The circle's transformation is given by
 * A_circle and its dimension by radius. Vectors cp0 and cp1 hold the closest
 * points of the distance query. They may be NULL.
 *
 * Important: This method uses Wm5 methods that seem to dynamically
 *            allocate memory. Before calling this method, make
 *            sure to call Wm5::Memory::Initialize() !!!
 ******************************************************************************/
static double Rcs_distanceLinesegCircle(const double linePt0[3],
                                        const double linePt1[3],
                                        const HTr* A_c,
                                        double radius,
                                        double cp0[3],
                                        double cp1[3])
{
  // At first we check the point circle distance for the line segment
  // we need it if no closest point is being found on the line segment
  double cp0_[3], cp1_[3];
  double min_dist = Rcs_distancePointCircle(linePt0, A_c, radius, cp0_, cp1_);

  double tmp0[3],tmp1[3];
  double dist = Rcs_distancePointCircle(linePt1, A_c, radius, tmp0, tmp1);

  if (dist < min_dist)
  {
    min_dist = dist;
    Vec3d_copy(cp0_, tmp0);
    Vec3d_copy(cp1_, tmp1);
  }

  // Next, we check for all solutions of the line circle distance
  // if it is on the line segment and has a closer distance than the current
  // min_dist then that's the closest point for us
  Wm5::Circle3d circle(Vec3(A_c->org[0], A_c->org[1], A_c->org[2]),
                       Vec3(A_c->rot[0][0], A_c->rot[0][1], A_c->rot[0][2]),
                       Vec3(A_c->rot[1][0], A_c->rot[1][1], A_c->rot[1][2]),
                       Vec3(A_c->rot[2][0], A_c->rot[2][1], A_c->rot[2][2]),
                       radius);

  // line defined by two points, create a normalized direction vector
  Vec3 pt0(linePt0[0], linePt0[1], linePt0[2]);
  Vec3 pt1(linePt1[0], linePt1[1], linePt1[2]);
  Vec3 dir = pt1 - pt0;

  // we do not normalize the direction vector, so later we know that a factor
  // between 0.0 and 1.0 for the point is on the segment
  Wm5::Line3d line(pt0, dir);

  Vec3 diff = line.Origin - circle.Center;
  double diffSqrLen = diff.SquaredLength();
  double MdM = line.Direction.SquaredLength();
  double DdM = diff.Dot(line.Direction);
  double NdM = circle.Normal.Dot(line.Direction);
  double DdN = diff.Dot(circle.Normal);

  double a0 = DdM;
  double a1 = MdM;
  double b0 = DdM - NdM*DdN;
  double b1 = MdM - NdM*NdM;
  double c0 = diffSqrLen - DdN*DdN;
  double c1 = b0;
  double c2 = b1;
  double rsqr = circle.Radius*circle.Radius;

  double a0sqr = a0*a0;
  double a1sqr = a1*a1;
  double twoA0A1 = 2.0*a0*a1;
  double b0sqr = b0*b0;
  double b1Sqr = b1*b1;
  double twoB0B1 = 2.0*b0*b1;
  double twoC1 = 2.0*c1;

  // The minimum point B+t*M occurs when t is a root of the quartic
  // equation whose coefficients are defined below.
  Wm5::Polynomial1d poly(4);
  poly[0] = a0sqr*c0 - b0sqr*rsqr;
  poly[1] = twoA0A1*c0 + a0sqr*twoC1 - twoB0B1*rsqr;
  poly[2] = a1sqr*c0 + twoA0A1*twoC1 + a0sqr*c2 - b1Sqr*rsqr;
  poly[3] = a1sqr*twoC1 + twoA0A1*c2;
  poly[4] = a1sqr*c2;

  Wm5::PolynomialRootsd polyroots(Wm5::Math<double>::ZERO_TOLERANCE);
  polyroots.FindB(poly, 6);
  int count = polyroots.GetCount();
  const double* roots = polyroots.GetRoots();

  for (int i = 0; i < count; ++i)
  {
    // check if the point is on the line segment
    if (roots[i] >= 0.0 && roots[i] <= 1.0)
    {
      // Compute distance from P(t) to circle.
      Vec3 P = line.Origin + roots[i]*line.Direction;
      Wm5::DistPoint3Circle3d query(P, circle);
      dist = query.Get();
      if (dist < min_dist)
      {
        min_dist = dist;
        Vec3d_copy(cp0_, query.GetClosestPoint0());
        Vec3d_copy(cp1_, query.GetClosestPoint1());
      }
    }
  }

  // there is a special case that we have to handle:
  // if the SSL is exactly on the normal of the circle, there are infinite
  // closest points on the torus, vcp1 will then be set to
  // (MAX_REAL, MAX_REAL, MAX_REAL)
  // we have to handle that (we just choose a specific point on the torus)
  if (cp1_[0] == Wm5::Math<double>::MAX_REAL ||
      cp1_[1] == Wm5::Math<double>::MAX_REAL ||
      cp1_[2] == Wm5::Math<double>::MAX_REAL)
  {
    double pt[3] = {radius, 0.0, 0.0};
    Vec3d_transform(cp1_, A_c, pt);
  }

  // the point is on the segment
  if (cp0)
  {
    Vec3d_copy(cp0, cp0_);
  }

  if (cp1)
  {
    Vec3d_copy(cp1, cp1_);
  }


  return min_dist;
}

/*******************************************************************************
 * Computes the distance between two line segments. Line segment 1 is defined
 * by the points p00 and p01. Line segment 2 is determined by p10 and p11.
 * Vectors cp0 and cp1 hold the closest points of the distance query.
 ******************************************************************************/
static double Rcs_distanceLinesegLineseg(const double p00[3],
                                         const double p01[3],
                                         const double p10[3],
                                         const double p11[3],
                                         double cp0[3],
                                         double cp1[3])
{
  Wm5::Segment3d l1(Vec3(p00[0], p00[1], p00[2]), Vec3(p01[0], p01[1], p01[2]));
  Wm5::Segment3d l2(Vec3(p10[0], p10[1], p10[2]), Vec3(p11[0], p11[1], p11[2]));

  Wm5::DistSegment3Segment3d dist(l1, l2);
  double distance = dist.Get();

  if (cp0)
  {
    Vec3 vcp0 = dist.GetClosestPoint0();
    cp0[0] = vcp0[0];
    cp0[1] = vcp0[1];
    cp0[2] = vcp0[2];
  }

  if (cp1)
  {
    Vec3 vcp1 = dist.GetClosestPoint1();
    cp1[0] = vcp1[0];
    cp1[1] = vcp1[1];
    cp1[2] = vcp1[2];
  }

  return distance;
}

/*******************************************************************************
 * Computes the distance between two capsules. The function is almost the same
 * as distanceLinesegLineseg(), except that the radius of the capsules is
 * considered. Please note that this leads to ill-posed cases, such as one
 * capsule being fully contained in the other one. Vector n is the unit normal
 * on the surface of the first capsule.
 ******************************************************************************/
static double Rcs_distanceCapsuleCapsule(const double linePt00[3],
                                         const double linePt01[3],
                                         double d0,
                                         const double linePt10[3],
                                         const double linePt11[3],
                                         double d1,
                                         double cp0[3],
                                         double cp1[3],
                                         double n[3])
{
  double cpSurf0[3], cpSurf1[3];
  double distance = Rcs_distanceLinesegLineseg(linePt00, linePt01,
                                               linePt10, linePt11, cp0, cp1);

  // Normalized direction vector from closest connection line0 -> line1
  Vec3d_sub(n, cp1, cp0);
  Vec3d_normalizeSelf(n);

  // Surface points of capsules
  for (int i = 0; i < 3; i++)
  {
    cpSurf0[i] = cp0[i] + 0.5 * d0 * n[i];
    cpSurf1[i] = cp1[i] - 0.5 * d1 * n[i];
  }

  Vec3d_copy(cp0, cpSurf0);
  Vec3d_copy(cp1, cpSurf1);

  // Consider the radius of the capsules for the distance
  return distance - 0.5 * (d0 + d1);
}

/*******************************************************************************
 * Computes the distance between a point and a plane segment. The plane
 * segment is defined by transformation A_plane. The origin is the  centroid
 * of the plane, the transforms z-axis points toward the plane normal
 * direction. Scalars x and y specify the extents of the plane segment in the
 * respective direction. Vectors cp0 and cp1 hold the closest points of the
 * distance query.
 ******************************************************************************/
static double Rcs_distancePointPlaneseg(const double p[3],
                                        const HTr* A_plane,   // z is normal
                                        double x,
                                        double y,
                                        double cp0[3],
                                        double cp1[3])
{
  Vec3 point(Vec3(p[0], p[1], p[2]));

  Vec3 cntr(A_plane->org[0], A_plane->org[1], A_plane->org[2]);
  Vec3 ax(A_plane->rot[0][0], A_plane->rot[0][1], A_plane->rot[0][2]);
  Vec3 ay(A_plane->rot[1][0], A_plane->rot[1][1], A_plane->rot[1][2]);

  Wm5::Rectangle3d rect(cntr, ax, ay, 0.5 * x, 0.5 * y);


  Wm5::DistPoint3Rectangle3d dist(point, rect);

  double distance = dist.Get();

  if (cp0)
  {
    Vec3 vcp0 = dist.GetClosestPoint0();
    cp0[0] = vcp0[0];
    cp0[1] = vcp0[1];
    cp0[2] = vcp0[2];
  }

  if (cp1)
  {
    Vec3 vcp1 = dist.GetClosestPoint1();
    cp1[0] = vcp1[0];
    cp1[1] = vcp1[1];
    cp1[2] = vcp1[2];
  }

  return distance;
}

/*******************************************************************************
 * Computes the distance between a line segment and a plane segment. The line
 * segment is defined by the points p1 and p2. The plane segment is defined by
 * transformation A_plane. The origin is the centroid of the plane, the
 * transforms z-axis points toward the plane normal direction. Scalars x and
 * y specify the extents of the plane segment in the respective direction.
 * Vectors cp0 and cp1 hold the closest points of the distance query.
 ******************************************************************************/
static double Rcs_distanceLinesegPlaneseg(const double p00[3],
                                          const double p01[3],
                                          const HTr* A_plane,   // z is normal
                                          double x,
                                          double y,
                                          double cp0[3],
                                          double cp1[3])
{
  Wm5::Segment3d line(Vec3(p00[0], p00[1], p00[2]),
                      Vec3(p01[0], p01[1], p01[2]));

  Vec3 cntr(A_plane->org[0], A_plane->org[1], A_plane->org[2]);
  Vec3 ax(A_plane->rot[0][0], A_plane->rot[0][1], A_plane->rot[0][2]);
  Vec3 ay(A_plane->rot[1][0], A_plane->rot[1][1], A_plane->rot[1][2]);

  Wm5::Rectangle3d rect(cntr, ax, ay, 0.5 * x, 0.5 * y);

  Wm5::DistSegment3Rectangle3d dist(line, rect);
  double distance = dist.Get();

  if (cp0)
  {
    Vec3 vcp0 = dist.GetClosestPoint0();
    cp0[0] = vcp0[0];
    cp0[1] = vcp0[1];
    cp0[2] = vcp0[2];
  }

  if (cp1)
  {
    Vec3 vcp1 = dist.GetClosestPoint1();
    cp1[0] = vcp1[0];
    cp1[1] = vcp1[1];
    cp1[2] = vcp1[2];
  }

  return distance;
}

/*******************************************************************************
 * Computes the distance between a capsule and a sphere-swept rectangle. The
 * function is almost the same as distanceLinesegPlaneseg(), except that the
 * radius of the shapes is considered. Please note that this can lead to
 * ill-posed cases, such as the capsule being fully contained in the SSR.
 ******************************************************************************/
static double Rcs_distanceCapsuleSSR(const double linePt0[3],
                                     const double linePt1[3],
                                     double dSSL,
                                     const HTr* A_plane,   // z is normal
                                     const double extents[3],
                                     double cp0[3],
                                     double cp1[3],
                                     double n[3])
{
  double distance;
  if (Vec3d_distance(linePt0, linePt1) == 0.0)
  {
    distance = Rcs_distancePointPlaneseg(linePt0, A_plane,
                                         extents[0], extents[1], cp0, cp1);
  }
  else
  {
    distance = Rcs_distanceLinesegPlaneseg(linePt0, linePt1, A_plane,
                                           extents[0], extents[1], cp0, cp1);
  }

  // Project the closest line point to the surface of the capsule
  double cpSurf0[3], cpSurf1[3];

  // Normalized direction vector from closest connection SSL -> SSR
  Vec3d_sub(n, cp1, cp0);
  Vec3d_normalizeSelf(n);

  // Surface points
  for (int i = 0; i < 3; i++)
  {
    cpSurf0[i] = cp0[i] + 0.5 * dSSL * n[i];
    cpSurf1[i] = cp1[i] - 0.5 * extents[2] * n[i];
  }

  Vec3d_copy(cp0, cpSurf0);
  Vec3d_copy(cp1, cpSurf1);

  return distance - 0.5 * (dSSL + extents[2]);
}

/*******************************************************************************
 * Computes the distance between a capsule and a box. The function is almost
 * the same as distanceLinesegBox(), except that the radius of the capsule is
 * considered. Please note that this leads to ill-posed cases, such as one
 * capsule being fully contained in the box.
 ******************************************************************************/
static double Rcs_distanceCapsuleBox(const double linePt0[3],
                                     const double linePt1[3],
                                     double dCapsule,
                                     const HTr* A_box,
                                     const double extents[3],
                                     double cpc[3],
                                     double cpb[3],
                                     double n[3])
{
  double cpl[3], d;

  d = Rcs_distanceLinesegBox(linePt0, linePt1, A_box, extents, cpl, cpb);

  // Normal vector on capsule
  Vec3d_sub(n, cpb, cpl);
  Vec3d_normalizeSelf(n);

  // Point on capsule surface: cpc = cpl + 0.5*dCapsule*n
  Vec3d_constMulAndAdd(cpc, cpl, n, 0.5 * dCapsule);

  return d - 0.5 * dCapsule;
}

/*******************************************************************************
 * Computes the distance between a capsule and a torus. The function is almost
 * the same as distanceLinesegCircle(), except that the radius of the capsule
 * and the torus are considered. Please note that this leads to ill-posed
 * cases, such as one capsule being fully contained in the torus.
 ******************************************************************************/
static double Rcs_distanceCapsuleTorus(const double linePt0[3],
                                       const double linePt1[3],
                                       double dCapsule,
                                       const HTr* A_circle,
                                       double radius,
                                       double thickness,
                                       double cp0[3],
                                       double cp1[3],
                                       double n[3])
{
  double cpl[3], cpt[3];

  double d = Rcs_distanceLinesegCircle(linePt0, linePt1, A_circle, radius,
                                       cpl, cpt);

  // Normal vector on capsule
  Vec3d_sub(n, cpt, cpl);
  Vec3d_normalizeSelf(n);

  // Point on capsule surface: cp0 = cpl + 0.5*dCapsule*n
  Vec3d_constMulAndAdd(cp0, cpl, n, 0.5 * dCapsule);

  // Point on torus surface: cp1 = cpt - 0.5*thickness*n
  Vec3d_constMulAndAdd(cp1, cpt, n, -0.5 * thickness);

  return d - 0.5 * dCapsule - 0.5 * thickness;
}

/*******************************************************************************
 * Computes the distance between two plane segments. They are defined by their
 * transformation A_plane. The origin is the centroid of the plane segment,
 * the transforms z-axis points toward the plane normal direction. Scalars x
 * and y specify the extents of the plane segment in the respective direction.
 * Vectors cp0 and cp1 hold the closest points of the distance query.
 ******************************************************************************/
static double Rcs_distancePlanesegPlaneseg(const HTr* A_plane1,
                                           double x1,
                                           double y1,
                                           const HTr* A_plane2,
                                           double x2,
                                           double y2,
                                           double cp0[3],
                                           double cp1[3])
{
  Vec3 cntr1(A_plane1->org[0],  A_plane1->org[1],    A_plane1->org[2]);
  Vec3 ax1(A_plane1->rot[0][0], A_plane1->rot[0][1], A_plane1->rot[0][2]);
  Vec3 ay1(A_plane1->rot[1][0], A_plane1->rot[1][1], A_plane1->rot[1][2]);

  Wm5::Rectangle3d rect1(cntr1, ax1, ay1, 0.5 * x1, 0.5 * y1);

  Vec3 cntr2(A_plane2->org[0],  A_plane2->org[1],    A_plane2->org[2]);
  Vec3 ax2(A_plane2->rot[0][0], A_plane2->rot[0][1], A_plane2->rot[0][2]);
  Vec3 ay2(A_plane2->rot[1][0], A_plane2->rot[1][1], A_plane2->rot[1][2]);

  Wm5::Rectangle3d rect2(cntr2, ax2, ay2, 0.5 * x2, 0.5 * y2);

  Wm5::DistRectangle3Rectangle3d dist(rect1, rect2);
  double distance = dist.Get();

  if (cp0)
  {
    Vec3 vcp0 = dist.GetClosestPoint0();
    cp0[0] = vcp0[0];
    cp0[1] = vcp0[1];
    cp0[2] = vcp0[2];
  }

  if (cp1)
  {
    Vec3 vcp1 = dist.GetClosestPoint1();
    cp1[0] = vcp1[0];
    cp1[1] = vcp1[1];
    cp1[2] = vcp1[2];
  }

  return distance;
}

/*******************************************************************************
 * Computes the distance between a plane segment and a box. The plane segment
 * is defined by transformation A_plane. The origin is the centroid of the
 * plane, the transforms z-axis points toward the plane normal direction.
 * Scalars x and y specify the extents of the plane segment in the respective
 * direction. The box' center is in the origin of transformation A_box, its
 * local edges are aligned with the transforms coordinate axes. The edge
 * length of each side is given in array extents. Vectors cp0 and cp1 hold
 * the closest points of the distance query.
 *
 * The implementation of this function is not very efficient. It tests all
 * combinations of the plane segments defining the boxes shapes against the
 * other plane segment. This results in 6 plane segment - plane segment tests.
 ******************************************************************************/
static double Rcs_distancePlanesegBox(const HTr* A_plane,
                                      double x,
                                      double y,
                                      const HTr* A_box,
                                      const double extents[3],
                                      double cp0[3],
                                      double cp1[3])
{
  const double* ex = A_box->rot[0];
  const double* ey = A_box->rot[1];
  const double* ez = A_box->rot[2];
  double cpl0[3], cpl1[3];
  double d_min = std::numeric_limits<double>::infinity();
  double d     = std::numeric_limits<double>::infinity();
  HTr A_bp;
  HTr_copy(&A_bp, A_box);



  // Rectangle for box bottom side
  for (int i = 0; i < 3; i++)
  {
    A_bp.org[i] = A_box->org[i] - 0.5 * ez[i] * extents[2];
  }

  d = Rcs_distancePlanesegPlaneseg(A_plane, x, y, &A_bp,
                                   extents[0], extents[1], cpl0, cpl1);
  if (d < d_min)
  {
    d_min = d;
    if (cp0)
    {
      Vec3d_copy(cp0, cpl0);
    }
    if (cp1)
    {
      Vec3d_copy(cp1, cpl1);
    }
  }


  // Rectangle for box top side
  for (int i = 0; i < 3; i++)
  {
    A_bp.org[i] = A_box->org[i] + 0.5 * ez[i] * extents[2];
  }

  d = Rcs_distancePlanesegPlaneseg(A_plane, x, y, &A_bp,
                                   extents[0], extents[1], cpl0, cpl1);
  if (d < d_min)
  {
    d_min = d;
    if (cp0)
    {
      Vec3d_copy(cp0, cpl0);
    }
    if (cp1)
    {
      Vec3d_copy(cp1, cpl1);
    }
  }



  // Rectangle for box left side
  for (int i = 0; i < 3; i++)
  {
    A_bp.org[i] = A_box->org[i] + 0.5 * ey[i] * extents[1];
    A_bp.rot[0][i] =  ex[i];
    A_bp.rot[1][i] = -ez[i];
    A_bp.rot[2][i] =  ey[i];
  }

  d = Rcs_distancePlanesegPlaneseg(A_plane, x, y, &A_bp,
                                   extents[0], extents[2], cpl0, cpl1);
  if (d < d_min)
  {
    d_min = d;
    if (cp0)
    {
      Vec3d_copy(cp0, cpl0);
    }
    if (cp1)
    {
      Vec3d_copy(cp1, cpl1);
    }
  }



  // Rectangle for box right side
  for (int i = 0; i < 3; i++)
  {
    A_bp.org[i] = A_box->org[i] - 0.5 * ey[i] * extents[1];
    A_bp.rot[0][i] =  ex[i];
    A_bp.rot[1][i] =  ez[i];
    A_bp.rot[2][i] = -ey[i];
  }

  d = Rcs_distancePlanesegPlaneseg(A_plane, x, y, &A_bp,
                                   extents[0], extents[2], cpl0, cpl1);
  if (d < d_min)
  {
    d_min = d;
    if (cp0)
    {
      Vec3d_copy(cp0, cpl0);
    }
    if (cp1)
    {
      Vec3d_copy(cp1, cpl1);
    }
  }



  // Rectangle for box near front side
  for (int i = 0; i < 3; i++)
  {
    // double offs[3];
    A_bp.org[i] = A_box->org[i] - 0.5 * ex[i] * extents[0];
    A_bp.rot[0][i] =  ez[i];
    A_bp.rot[1][i] =  ey[i];
    A_bp.rot[2][i] = -ex[i];
  }

  d = Rcs_distancePlanesegPlaneseg(A_plane, x, y, &A_bp,
                                   extents[2], extents[1], cpl0, cpl1);
  if (d < d_min)
  {
    d_min = d;
    if (cp0)
    {
      Vec3d_copy(cp0, cpl0);
    }
    if (cp1)
    {
      Vec3d_copy(cp1, cpl1);
    }
  }



  // Rectangle for box far front side
  for (int i = 0; i < 3; i++)
  {
    A_bp.org[i] = A_box->org[i] + 0.5 * ex[i] * extents[0];
    A_bp.rot[0][i] =  ez[i];
    A_bp.rot[1][i] = -ey[i];
    A_bp.rot[2][i] =  ex[i];
  }

  d = Rcs_distancePlanesegPlaneseg(A_plane, x, y, &A_bp,
                                   extents[2], extents[1], cpl0, cpl1);
  if (d < d_min)
  {
    d_min = d;
    if (cp0)
    {
      Vec3d_copy(cp0, cpl0);
    }
    if (cp1)
    {
      Vec3d_copy(cp1, cpl1);
    }
  }

  return d_min;
}

/*******************************************************************************
 * Computes the distance between a SSR and a box. The function is almost the
 * same as distancePlanesegBox(), except that the radius of the SSR is
 * considered. Please note that this leads to ill-posed cases, such as the SSR
 * being fully contained in the box.
 ******************************************************************************/
static double Rcs_distanceSSRBox(const HTr* A_ssr,
                                 const double extentsSSR[3],
                                 const HTr* A_box,
                                 const double extentsBox[3],
                                 double cpSSR[3],
                                 double cpBox[3],
                                 double n[3])
{
  double cpRect[3], r_SSR = extentsSSR[2], d;

  d = Rcs_distancePlanesegBox(A_ssr, extentsSSR[0], extentsSSR[1],
                              A_box, extentsBox, cpRect, cpBox);

  // Normal vector on SSR
  Vec3d_sub(n, cpBox, cpRect);
  Vec3d_normalizeSelf(n);

  // Point on SSR surface: cpSSR = cpRect + r_SSR*n
  Vec3d_constMulAndAdd(cpSSR, cpRect, n, r_SSR);

  return d - r_SSR;
}

/*******************************************************************************
 * Computes the distance between two boxes. The box centers are in the origin
 * of their transformation A_box, the local edges are aligned with the
 * transforms coordinate axes. The edge length of each side is given in array
 * extents. Vectors cp0 and cp1 hold the closest points of the distance query.
 *
 * The implementation of this function is not very efficient. It simply tests
 * all combinations of the plane segments defining the boxes shapes against
 * each other. This results in 36 plane segment - plane segment tests. On this
 * level, it might be more efficient to employ more advanced algorithms such
 * as for instance GJK.
 ******************************************************************************/
static double Rcs_distanceBoxBox(const HTr* A_box1,
                                 const double extents1[3],
                                 const HTr* A_box2,
                                 const double extents2[3],
                                 double cp0[3],
                                 double cp1[3])
{
  const double* ex = A_box1->rot[0];
  const double* ey = A_box1->rot[1];
  const double* ez = A_box1->rot[2];
  double cpl0[3] = {0.0, 0.0, 0.0}, cpl1[3] = {0.0, 0.0, 0.0};
  double d_min = std::numeric_limits<double>::infinity();
  double d     = std::numeric_limits<double>::infinity();
  HTr A_bp1;
  HTr_copy(&A_bp1, A_box1);



  // Rectangle for box bottom side
  for (int i = 0; i < 3; i++)
  {
    A_bp1.org[i] = A_box1->org[i] - 0.5 * ez[i] * extents1[2];
  }

  d = Rcs_distancePlanesegBox(&A_bp1, extents1[0], extents1[1],
                              A_box2, extents2, cpl0, cpl1);
  if (d < d_min)
  {
    d_min = d;
    if (cp0)
    {
      Vec3d_copy(cp0, cpl0);
    }
    if (cp1)
    {
      Vec3d_copy(cp1, cpl1);
    }
  }

  // Rectangle for box top side
  for (int i = 0; i < 3; i++)
  {
    A_bp1.org[i] = A_box1->org[i] + 0.5 * ez[i] * extents1[2];
  }

  d = Rcs_distancePlanesegBox(&A_bp1, extents1[0], extents1[1],
                              A_box2, extents2, cpl0, cpl1);
  if (d < d_min)
  {
    d_min = d;
    if (cp0)
    {
      Vec3d_copy(cp0, cpl0);
    }
    if (cp1)
    {
      Vec3d_copy(cp1, cpl1);
    }
  }



  // Rectangle for box left side
  for (int i = 0; i < 3; i++)
  {
    A_bp1.org[i] = A_box1->org[i] + 0.5 * ey[i] * extents1[1];
    A_bp1.rot[0][i] =  ex[i];
    A_bp1.rot[1][i] = -ez[i];
    A_bp1.rot[2][i] =  ey[i];
  }

  d = Rcs_distancePlanesegBox(&A_bp1, extents1[0], extents1[2],
                              A_box2, extents2, cpl0, cpl1);
  if (d < d_min)
  {
    d_min = d;
    if (cp0)
    {
      Vec3d_copy(cp0, cpl0);
    }
    if (cp1)
    {
      Vec3d_copy(cp1, cpl1);
    }
  }



  // Rectangle for box right side
  for (int i = 0; i < 3; i++)
  {
    A_bp1.org[i] = A_box1->org[i] - 0.5 * ey[i] * extents1[1];
    A_bp1.rot[0][i] =  ex[i];
    A_bp1.rot[1][i] =  ez[i];
    A_bp1.rot[2][i] = -ey[i];
  }

  d = Rcs_distancePlanesegBox(&A_bp1, extents1[0], extents1[2],
                              A_box2, extents2, cpl0, cpl1);
  if (d < d_min)
  {
    d_min = d;
    if (cp0)
    {
      Vec3d_copy(cp0, cpl0);
    }
    if (cp1)
    {
      Vec3d_copy(cp1, cpl1);
    }
  }



  // Rectangle for box near front side
  for (int i = 0; i < 3; i++)
  {
    // double offs[3];
    A_bp1.org[i] = A_box1->org[i] - 0.5 * ex[i] * extents1[0];
    A_bp1.rot[0][i] =  ez[i];
    A_bp1.rot[1][i] =  ey[i];
    A_bp1.rot[2][i] = -ex[i];
  }

  d = Rcs_distancePlanesegBox(&A_bp1, extents1[2], extents1[1],
                              A_box2, extents2, cpl0, cpl1);
  if (d < d_min)
  {
    d_min = d;
    if (cp0)
    {
      Vec3d_copy(cp0, cpl0);
    }
    if (cp1)
    {
      Vec3d_copy(cp1, cpl1);
    }
  }



  // Rectangle for box far front side
  for (int i = 0; i < 3; i++)
  {
    A_bp1.org[i] = A_box1->org[i] + 0.5 * ex[i] * extents1[0];
    A_bp1.rot[0][i] =  ez[i];
    A_bp1.rot[1][i] = -ey[i];
    A_bp1.rot[2][i] =  ex[i];
  }

  d = Rcs_distancePlanesegBox(&A_bp1, extents1[2], extents1[1],
                              A_box2, extents2, cpl0, cpl1);
  if (d < d_min)
  {
    d_min = d;
    if (cp0)
    {
      Vec3d_copy(cp0, cpl0);
    }
    if (cp1)
    {
      Vec3d_copy(cp1, cpl1);
    }
  }

  return d_min;
}

/*******************************************************************************
 *
******************************************************************************/
bool Rcs_computePlanePlaneIntersection(const double p1[3],
                                       const double n1[3],
                                       const double p2[3],
                                       const double n2[3],
                                       double origin[3],
                                       double direction[3])
{
  // Degenerate case: normal vector has zero length
  double normN1[3], normN2[3];
  Vec3d_copy(normN1, n1);
  double l1 = Vec3d_normalizeSelf(normN1);
  if (l1 == 0.0)
  {
    RLOG(4, "Length of normal vector 1 is 0");
    return false;
  }

  Vec3d_copy(normN2, n2);
  double l2 = Vec3d_normalizeSelf(normN2);
  if (l2 == 0.0)
  {
    RLOG(4, "Length of normal vector 2 is 0");
    return false;
  }

  // N is specified, c = Dot(N,P) where P is on the plane
  Wm5::Plane3d  plane1(Vec3(normN1[0], normN1[1], normN1[2]),
                       Vec3(p1[0], p1[1], p1[2]));
  Wm5::Plane3d  plane2(Vec3(normN2[0], normN2[1], normN2[2]),
                       Vec3(p2[0], p2[1], p2[2]));

  Wm5::IntrPlane3Plane3d intrsec(plane1, plane2);

  if (!intrsec.Test())
  {
    RLOG(4, "No intersection found - planes seem to be parallel:\n"
         "n1 = %f   %f   %f\nn2 = %f   %f   %f\n"
         "p1 = %f   %f   %f\np2 = %f   %f   %f",
         normN1[0], normN1[1], normN1[2], normN2[0], normN2[1], normN2[2],
         p1[0], p1[1], p1[2], p2[0], p2[1], p2[2]);
    return false;
  }

  intrsec.Find();
  Wm5::Line3d sec =  intrsec.GetIntersectionLine();

  Vec3 linePt  = sec.Origin;
  Vec3 lineDir = sec.Direction;

  origin[0] = linePt.X();
  origin[1] = linePt.Y();
  origin[2] = linePt.Z();

  direction[0] = lineDir.X();
  direction[1] = lineDir.Y();
  direction[2] = lineDir.Z();

  return true;
}

/*******************************************************************************
 *
******************************************************************************/
double Rcs_distancePointLineseg2D(const double point[2],
                                  const double linePt0[2],
                                  const double linePt1[2],
                                  double cp0[2],
                                  double cp1[2])
{
  Wm5::Vector2d pt(point[0], point[1]);

  Wm5::Segment2d line(Wm5::Vector2d(linePt0[0], linePt0[1]),
                      Wm5::Vector2d(linePt1[0], linePt1[1]));

  Wm5::DistPoint2Segment2d dist(pt, line);

  double distance = dist.Get();

  if (cp0)
  {
    Wm5::Vector2d vcp0 = dist.GetClosestPoint0();
    cp0[0] = vcp0[0];
    cp0[1] = vcp0[1];
  }

  if (cp1)
  {
    Wm5::Vector2d vcp1 = dist.GetClosestPoint1();
    cp1[0] = vcp1[0];
    cp1[1] = vcp1[1];
  }

  return distance;
}

/*******************************************************************************
 *
******************************************************************************/
double Rcs_distanceLinesegLineseg2D(const double p00[2],
                                    const double p01[2],
                                    const double p10[2],
                                    const double p11[2],
                                    double cp0[2],
                                    double cp1[2])
{
  Wm5::Segment2d l1(Wm5::Vector2d(p00[0], p00[1]),
                    Wm5::Vector2d(p01[0], p01[1]));
  Wm5::Segment2d l2(Wm5::Vector2d(p10[0], p10[1]),
                    Wm5::Vector2d(p11[0], p11[1]));

  Wm5::DistSegment2Segment2d dist(l1, l2);
  double distance = dist.Get();

  if (cp0)
  {
    Wm5::Vector2d vcp0 = dist.GetClosestPoint0();
    cp0[0] = vcp0[0];
    cp0[1] = vcp0[1];
  }

  if (cp1)
  {
    Wm5::Vector2d vcp1 = dist.GetClosestPoint1();
    cp1[0] = vcp1[0];
    cp1[1] = vcp1[1];
  }

  return distance;
}

/*******************************************************************************
 *
******************************************************************************/
inline double sqr(double x)
{
  return x*x;
}

/*******************************************************************************
 *
******************************************************************************/
void Rcs_Cart2Cyl(const double p[3], double& radialDist, double& azimuth,
                  double& height)
{
  radialDist = sqrt(sqr(p[0])+sqr(p[1]));
  height = p[2];

  if (p[0]==0. && p[1]==0.)
  {
    azimuth = 0.;
  }
  else if (p[0]>=0)
  {
    azimuth = asin(p[1]/radialDist);
  }
  else
  {
    azimuth = M_PI - asin(p[1]/radialDist);
  }
}

/*******************************************************************************
 *
******************************************************************************/
void Rcs_Cyl2Cart(const double radialDist, const double azimuth,
                  const double height, double p[3])
{
  p[0] = radialDist*cos(azimuth);
  p[1] = radialDist*sin(azimuth);
  p[2] = height;
}

/*******************************************************************************
 *
******************************************************************************/
void Rcs_copyPointIfCloser2D(const double distance_temp,
                             const double cp0_2D_temp[2],
                             const double cp1_2D_temp[2],
                             double& distance,
                             double cp0_2D[2],
                             double cp1_2D[2])
{
  if (distance>distance_temp)
  {
    distance = distance_temp;
    if (cp0_2D && cp0_2D_temp)
    {
      VecNd_copy(cp0_2D, cp0_2D_temp, 2);
    }
    if (cp1_2D && cp1_2D_temp)
    {
      VecNd_copy(cp1_2D, cp1_2D_temp, 2);
    }
  }
}

/*******************************************************************************
 * Computes the distance between a point and a cylinder. The cylinder's
 * transformation is given by A_cylinder and its dimensions by height and
 * radius. Vectors cp0 and cp1 hold the closest points of the distance query.
 * They may be NULL.
 ******************************************************************************/
static double Rcs_distancePointCylinder(const double p[3],
                                        const HTr* A_cylinder,
                                        double height,
                                        double radius,
                                        double cp0_[3],
                                        double cp1_[3])
{
  double buf0[3], buf1[3];
  double* cp0 = cp0_ ? cp0_ : buf0;
  double* cp1 = cp1_ ? cp1_ : buf1;

  //transform into cylinder ref frame.
  double pTrafo[3];
  Vec3d_invTransform(pTrafo, A_cylinder, p);

  //transform into cylinder coordinates
  double pt[2];
  double angle;
  Rcs_Cart2Cyl(pTrafo, pt[0], angle, pt[1]);

  double p0[2] = {0, -.5*height}; //bottom center
  double p1[2] = {radius, -.5*height}; //bottom rim
  double p2[2] = {radius, .5*height}; //top rim
  double p3[2] = {0, .5*height}; //top center

  double cp1_2D[2];
  double distance =
    Rcs_distancePointLineseg2D(pt, p1, p2, NULL, cp1_2D); //prefer side

  double cp1_2D_temp[2];
  double distance_temp;

  distance_temp = Rcs_distancePointLineseg2D(pt, p0, p1, NULL, cp1_2D_temp);
  Rcs_copyPointIfCloser2D(distance_temp, NULL, cp1_2D_temp, distance, NULL,
                          cp1_2D);

  distance_temp = Rcs_distancePointLineseg2D(pt, p2, p3, NULL, cp1_2D_temp);
  Rcs_copyPointIfCloser2D(distance_temp, NULL, cp1_2D_temp, distance, NULL,
                          cp1_2D);

  //transform back to Cartesian coordinates
  Vec3d_copy(cp0, p);
  Rcs_Cyl2Cart(cp1_2D[0], angle, cp1_2D[1], cp1);

  //transform back into global ref frame
  Vec3d_transformSelf(cp1, A_cylinder);

  return distance;
}


/*******************************************************************************
 * Computes the distance between a point and a cone. The cone's transformation
 * is given by A_cone and its dimensions by height and radius. Vectors cp0 and
 * cp1 hold the closest points of the distance query. They may be NULL.
 ******************************************************************************/
// static double Rcs_distancePointCone(const double p[3],
//                                     const HTr* A_cone,
//                                     double height,
//                                     double radius,
//                                     double cp0_[3],
//                                     double cp1_[3])
// {
//   double buf0[3], buf1[3];
//   double* cp0 = cp0_ ? cp0_ : buf0;
//   double* cp1 = cp1_ ? cp1_ : buf1;

//   //transform into cone ref frame.
//   double pTrafo[3];
//   Vec3d_invTransform(pTrafo, A_cone, p);

//   //transform into cylinder coordinates
//   double pt[2];
//   double angle;
//   Rcs_Cart2Cyl(pTrafo, pt[0], angle, pt[1]);

//   double p0[2] = { 0, 0 }; //base
//   double p1[2] = { radius, 0 }; //rim
//   double p2[2] = { 0, height }; //tip

//   double cp1_2D[2];
//   double distance =
//     Rcs_distancePointLineseg2D(pt, p0, p1, NULL, cp1_2D); //prefer bottom

//   double cp1_2D_temp[2];
//   double distance_temp = Rcs_distancePointLineseg2D(pt, p1, p2, NULL,
//                                                     cp1_2D_temp);
//   Rcs_copyPointIfCloser2D(distance_temp, NULL, cp1_2D_temp, distance, NULL,
//                           cp1_2D);

//   //transform back to Cartesian coordinates
//   Vec3d_copy(cp0, p);
//   Rcs_Cyl2Cart(cp1_2D[0], angle, cp1_2D[1], cp1);

//   //transform back into global ref frame
//   Vec3d_transformSelf(cp1, A_cone);

//   return distance;
// }

/*!
 * \brief Recursive helper function to decide if a point lies inside convex
 *        polygon in O(log N) time
 * \param pt Query point
 * \param polygon Polygon vertices in ordered counter-clockwise
 * \param N Number of polygon vertices
 * \param i0 Bisect index 1
 * \param i1 Bisect index 2
 *
 * Implemenation assimilated from GeometricTools
 */
bool Rcs_containedPoint2DConvexPolygon2DRecursive(const double pt[2],
                                                  const double polygon[][2],
                                                  int N,
                                                  int i0,
                                                  int i1)
{
  double nx, ny, dx, dy;

  int diff = i1 - i0;
  if (diff == 1 || (diff < 0 && diff + N == 1))
  {
    nx = polygon[i1][1] - polygon[i0][1];
    ny = polygon[i0][0] - polygon[i1][0];
    dx = pt[0] - polygon[i0][0];
    dy = pt[1] - polygon[i0][1];
    return nx*dx + ny*dy <= 0.0;
  }

  // Bisect the index range.
  int mid;
  if (i0 < i1)
  {
    mid = (i0 + i1) >> 1;
  }
  else
  {
    mid = ((i0 + i1 + N) >> 1);
    if (mid >= N)
    {
      mid -= N;
    }
  }

  // Determine which side of the splitting line contains the point.
  nx = polygon[mid][1] - polygon[i0][1];
  ny = polygon[i0][0] - polygon[mid][0];
  dx = pt[0] - polygon[i0][0];
  dy = pt[1] - polygon[i0][1];
  if (nx*dx + ny*dy > 0.0)
  {
    // P potentially in <V(i0),V(i0+1),...,V(mid-1),V(mid)>
    return Rcs_containedPoint2DConvexPolygon2DRecursive(pt, polygon, N, i0, mid);
  }
  else
  {
    // P potentially in <V(mid),V(mid+1),...,V(i1-1),V(i1)>
    return Rcs_containedPoint2DConvexPolygon2DRecursive(pt, polygon, N, mid, i1);
  }
}

/******************************************************************************

  \brief See header.

******************************************************************************/

bool Rcs_containedPoint2DConvexPolygon2D(const double pt[2],
                                         const double polygon[][2],
                                         unsigned int N)
{
  RCHECK_MSG(N >= 1, "A polygon needs at least 1 vertex");

  if (N == 1)
  {
    return (pt[0] == polygon[0][0]) && (pt[1] == polygon[0][1]);
  }
  if (N == 2)
  {
    return Rcs_distancePointLineseg2D(pt, polygon[0], polygon[1], NULL, NULL)
           == 0.0;
  }
  else
  {
    return Rcs_containedPoint2DConvexPolygon2DRecursive(pt, polygon, N, 0, 0);
  }
}

/*******************************************************************************
 *
******************************************************************************/
double Rcs_distancePoint2DConvexPolygon2D(const double pt[2],
                                          const double polygon[][2],
                                          unsigned int N,
                                          double cp0_[2],
                                          double cp1_[2])
{
  RCHECK_MSG(N >= 1, "A polygon needs at least 1 vertex");

  double distance = std::numeric_limits<double>::infinity();
  double cpTmp0[2], cpTmp1[2];
  double* cp0 = cp0_ ? cp0_ : cpTmp0;
  double* cp1 = cp1_ ? cp1_ : cpTmp1;
  VecNd_setElementsTo(cp0, distance, 2);
  VecNd_setElementsTo(cp1, distance, 2);

  if (N == 1)
  {
    distance = sqrt(VecNd_sqrDiff(pt, polygon[0], 2));
    VecNd_copy(cp0, pt, 2);
    VecNd_copy(cp1, polygon[0], 2);
  }
  else if (N == 2)
  {
    distance = Rcs_distancePointLineseg2D(pt, polygon[0], polygon[1], cp0, cp1);
  }
  else
  {
    // Naive implementation: Iterate over all line segments and find closest
    // distance
    for (unsigned int i = 0; i < N; i++)
    {
      double cp0_temp[2], cp1_temp[2];
      double dist = Rcs_distancePointLineseg2D(pt, polygon[i], polygon[(i+1)%N],
                                               cp0_temp, cp1_temp);

      if (dist < distance)
      {
        distance = dist;
        VecNd_copy(cp0, cp0_temp, 2);
        VecNd_copy(cp1, cp1_temp, 2);
      }
    }

    // if the point is inside the polygon, a negative distance is returned
    if (Rcs_containedPoint2DConvexPolygon2D(pt, polygon, N))
    {
      distance = -distance;
    }
  }

  return distance;
}

/*******************************************************************************
 *
******************************************************************************/
double Rcs_distancePoint3DConvexPolygon2D(const double pt[3],
                                          const double polygon[][2],
                                          unsigned int N,
                                          const HTr* A,
                                          double cp0[3],
                                          double cp1[3])
{
  RCHECK_MSG(N >= 1, "A polygon needs at least 1 vertex");

  // first transform point to polygon reference frame
  double pt_P[3];
  Vec3d_invTransform(pt_P, A, pt);

  // find closest point on plane
  double plane_pt[3];
  bool ret = Vec3d_computePlaneLineIntersection(plane_pt, pt_P, Vec3d_ez(),
                                                A->org, Vec3d_ez());

  RCHECK_MSG(ret, "No intersection point found");

  // solve problem in 2D
  double cp1_[3];
  Vec3d_setZero(cp1_);
  Rcs_distancePoint2DConvexPolygon2D(plane_pt, polygon, N, NULL, cp1_);

  // transform closest points back
  Vec3d_transformSelf(cp1_, A);

  // fill optional result vector
  if (cp0)
  {
    Vec3d_copy(cp0, pt);
  }

  // fill optional result vector
  if (cp1)
  {
    Vec3d_copy(cp1, cp1_);
  }

  // return distance
  return Vec3d_distance(pt, cp1_);
}

/*******************************************************************************
 *
******************************************************************************/
double Rcs_distancePointSqrtParabolaSeg2D(const double p[2],
                                          const double coeff[3],
                                          const double ep1[2],
                                          const double ep2[2],
                                          double cp0[2],
                                          double cp1[2])
{
  // The minimum point occurs at a root of the quartic
  // equation whose coefficients are defined below.
  Wm5::Polynomial1d poly(4);

  poly[4] = sqr(coeff[2]+1.)*coeff[2];
  poly[3] = (coeff[2]+1.)*(coeff[1]-2.*p[1])*coeff[2] + sqr(coeff[2]+1.)*coeff[1];
  poly[2] = sqr(coeff[1]/2.-p[1])*coeff[2] +
            (coeff[2]+1)*(coeff[1]-2.*p[1])*coeff[1] + sqr(coeff[2]+1.)*coeff[0] - sqr(coeff[2])*sqr(p[0]);
  poly[1] = sqr(coeff[1]/2.-p[1])*coeff[1] + (coeff[2]+1)*(coeff[1]-2.*p[1])*coeff[0] - coeff[2]*coeff[1]*sqr(p[0]);
  poly[0] = sqr(coeff[1]/2.-p[1])*coeff[0] - sqr(coeff[1])/4.*sqr(p[0]);

  Wm5::PolynomialRootsd polyroots(Wm5::Math<double>::ZERO_TOLERANCE);
  //  polyroots.FindB(poly, 6);
  polyroots.FindA(poly[0], poly[1], poly[2], poly[3], poly[4]);
  int count = polyroots.GetCount();
  const double* roots = polyroots.GetRoots();

  double distance = std::numeric_limits<double>::infinity();
  double limits[2] = {std::min(ep1[1], ep2[1]), std::max(ep1[1], ep2[1])};

  for (int i = 0; i < count; ++i)
  {
    if (roots[i]>=limits[0] && roots[i]<=limits[1])
    {
      double rootValue = sqrt(coeff[2]*sqr(roots[i]) + coeff[1]*roots[i] + coeff[0]);
      double dist = sqrt(sqr(rootValue-p[0])+sqr(roots[i]-p[1]));

      if (distance > dist)
      {
        distance = dist;
        if (cp1)
        {
          cp1[0] = rootValue;
          cp1[1] = roots[i];
        }
      }
    }
  }

  //check endpoints (always have to do that as above we could have picked a max)
  for (int i = 0; i < 2; ++i)
  {
    // TODO: This sometimes throws floating point exceptions
    // double sqrVal = coeff[2]*sqr(limits[i]) + coeff[1]*limits[i] + coeff[0];
    // RCHECK_MSG(sqrVal>=0.0, "sqrVal = %g coeff=%g %g %g limit=%g", sqrVal, coeff[0], coeff[1], coeff[2], limits[i]);
    double limitsValue = sqrt(coeff[2]*sqr(limits[i]) + coeff[1]*limits[i] + coeff[0]);
    double dist = sqrt(sqr(limitsValue-p[0])+sqr(limits[i]-p[1]));

    if (distance > dist)
    {
      distance = dist;
      if (cp1)
      {
        cp1[0] = limitsValue;
        cp1[1] = limits[i];
      }
    }
  }

  if (cp0)
  {
    VecNd_copy(cp0, p, 2);
  }

  return distance;
}


/******************************************************************************

  \brief Helper Function.

******************************************************************************/

double Rcs_distanceLinesegSqrtParabolaEndpoints2D(const double pl1[2],
                                                  const double pl2[2],
                                                  const double pp1[2],
                                                  const double pp2[2],
                                                  const double coeff[3],
                                                  double cp0[3],
                                                  double cp1[3])
{
  double cp0_temp[2];
  double cp1_temp[2];
  double distance_temp;
  double distance = std::numeric_limits<double>::infinity();

  // curve endpoints
  for (int i = 0; i < 2; ++i)
  {
    if (i==0)
    {
      VecNd_copy(cp1_temp, pp1, 2);
    }
    else
    {
      VecNd_copy(cp1_temp, pp2, 2);
    }

    distance_temp = Rcs_distancePointLineseg2D(cp1_temp, pl1, pl2, NULL, cp0_temp);
    Rcs_copyPointIfCloser2D(distance_temp, cp0_temp, cp1_temp, distance, cp0, cp1);
  }

  // line endpoints
  for (int i = 0; i < 2; ++i)
  {
    if (i==0)
    {
      VecNd_copy(cp0_temp, pl1, 2);
    }
    else
    {
      VecNd_copy(cp0_temp, pl2, 2);
    }

    distance_temp = Rcs_distancePointSqrtParabolaSeg2D(cp0_temp, coeff, pp1, pp2, NULL, cp1_temp);
    Rcs_copyPointIfCloser2D(distance_temp, cp0_temp, cp1_temp, distance, cp0, cp1);
  }

  return distance;
}

/*******************************************************************************
 * Computes the distance between a line segment and a cylinder. The line
 * segment is defined by the points p1 and p2. The cylinder's transformation
 * is given by A_cylinder and its dimension by height and radius. Vectors cp0
 * and cp1 hold the closest points of the distance query. They may be NULL.
 ******************************************************************************/
static double Rcs_distanceLinesegCylinder(const double p1[3],
                                          const double p2[3],
                                          const HTr* A_cylinder,
                                          double height,
                                          double radius,
                                          double cp0_[3],
                                          double cp1_[3])
{
  double buf0[3], buf1[3];
  double* cp0 = cp0_ ? cp0_ : buf0;
  double* cp1 = cp1_ ? cp1_ : buf1;

  //transform into cone ref frame.
  double p1Trafo[3];
  Vec3d_invTransform(p1Trafo, A_cylinder, p1);
  double p2Trafo[3];
  Vec3d_invTransform(p2Trafo, A_cylinder, p2);

  //transform into cylinder coordinates
  double pl1[2];
  double al1;
  Rcs_Cart2Cyl(p1Trafo, pl1[0], al1, pl1[1]);

  double pl2[2];
  double al2;
  Rcs_Cart2Cyl(p2Trafo, pl2[0], al2, pl2[1]);

  // cylinder points in cylinder coordinates
  double pc0[2] = {0, -.5*height}; //bottom center
  double pc1[2] = {radius, -.5*height}; //bottom rim
  double pc2[2] = {radius, .5*height}; //top rim
  double pc3[2] = {0, .5*height}; //top center

  //coefficients for line => sqrt(parabola) in terms of d (position in % between the two endpoints)
  double lineCoeffD[3];
  lineCoeffD[2] = sqr(p2Trafo[0]-p1Trafo[0]) + sqr(p2Trafo[1]-p1Trafo[1]);
  lineCoeffD[1] = 2.*(p1Trafo[0]*(p2Trafo[0]-p1Trafo[0]) + p1Trafo[1]*(p2Trafo[1]-p1Trafo[1]));
  lineCoeffD[0] = sqr(p1Trafo[0]) + sqr(p1Trafo[1]);

  //coefficients for line => sqrt(parabola) in terms of z
  double lineCoeffZ[3];
  if (fabs(p2Trafo[2]-p1Trafo[2])>0.0)
  {
    lineCoeffZ[2] = lineCoeffD[2]/sqr(p2Trafo[2]-p1Trafo[2]);
    lineCoeffZ[1] = lineCoeffD[1]/(p2Trafo[2]-p1Trafo[2]) - 2.*lineCoeffD[2]*p1Trafo[2]/sqr(p2Trafo[2]-p1Trafo[2]);
    lineCoeffZ[0] = lineCoeffD[0] - lineCoeffD[1]*p1Trafo[2]/(p2Trafo[2]-p1Trafo[2]) +
                    lineCoeffD[2]*sqr(p1Trafo[2]/(p2Trafo[2]-p1Trafo[2]));
  }

  double sideCoeff = radius;

  double cp0_2D[2];
  double cp1_2D[2];
  double distance = std::numeric_limits<double>::infinity();

  bool vertex = false;

  double vertexD = -lineCoeffD[1]/(2.*lineCoeffD[2]);
  double vertexP[2];
  if (vertexD>0. && vertexD<1.)
  {
    vertexP[0] = sqrt(lineCoeffD[2]*sqr(vertexD) + lineCoeffD[1]*vertexD + lineCoeffD[0]);
    vertexP[1] = pl1[1] + vertexD*(pl2[1]-pl1[1]);
    vertex = true;
  }

  // if transformed line is a line => distanceLinesegLineseg
  // this happens if the line is parallel or perpendicular to the base
  if (pl1[1]==pl2[1] || al1==al2)
  {
    //if it is a horizontal line we need to take into account the vertex as potential end-point
    if (pl1[1]==pl2[1])
    {
      if (vertex)
      {
        if ((vertexP[0]<pl1[0] && pl1[0]<=pl2[0]) || (vertexP[0]>pl1[0] && pl1[0]>=pl2[0]))
        {
          pl1[0] = vertexP[0];
        }
        else if ((vertexP[0]<pl2[0] && pl2[0]<pl1[0]) || (vertexP[0]>pl2[0] && pl2[0]>pl1[0]))
        {
          pl2[0] = vertexP[0];
        }
      }
    }

    distance = Rcs_distanceLinesegLineseg2D(pl1, pl2, pc1, pc2, cp0_2D, cp1_2D); //prefer side

    double cp0_2D_temp[2];
    double cp1_2D_temp[2];
    double distance_temp;

    distance_temp = Rcs_distanceLinesegLineseg2D(pl1, pl2, pc0, pc1, cp0_2D_temp, cp1_2D_temp); //bottom
    Rcs_copyPointIfCloser2D(distance_temp, cp0_2D_temp, cp1_2D_temp, distance, cp0_2D, cp1_2D);

    distance_temp = Rcs_distanceLinesegLineseg2D(pl1, pl2, pc2, pc3, cp0_2D_temp, cp1_2D_temp); //top
    Rcs_copyPointIfCloser2D(distance_temp, cp0_2D_temp, cp1_2D_temp, distance, cp0_2D, cp1_2D);
  }
  else //line is sqrt(parabola)
  {
    bool intersection = false;

    //test intersection side
    Wm5::Polynomial1d poly(2);

    poly[2] = lineCoeffD[2];
    poly[1] = lineCoeffD[1];
    poly[0] = lineCoeffD[0] - sqr(sideCoeff);

    Wm5::PolynomialRootsd polyroots(Wm5::Math<double>::ZERO_TOLERANCE);
    polyroots.FindA(poly[0], poly[1], poly[2]);
    int count = polyroots.GetCount();
    const double* roots = polyroots.GetRoots();

    for (int i = 0; i < count; ++i)
    {
      if (roots[i]>=0. && roots[i]<=1.)
      {
        double intersectionH = pc1[1] + roots[i]*(pc2[1]-pc1[1]);
        if (intersectionH>=pl1[1] && intersectionH<=pl2[1])
        {
          intersection = true;
          distance = 0.;
          cp0_2D[0] = cp1_2D[0] = sideCoeff;
          cp0_2D[1] = cp1_2D[1] = intersectionH;
          break;
        }
      }
    }

    //test intersection bottom
    if (!intersection)
    {
      double intersectionD = (pc0[1]-pl1[1])/(pl2[1]-pl1[1]);
      if (intersectionD>=0. && intersectionD<=1.)
      {
        double intersectionR = sqrt(lineCoeffD[2]*sqr(intersectionD) + lineCoeffD[1]*intersectionD + lineCoeffD[0]);
        if (intersectionR>=pc3[0] && intersectionR<=pc2[0])
        {
          intersection = true;
          distance = 0.;
          cp0_2D[0] = cp1_2D[0] = intersectionR;
          cp0_2D[1] = cp1_2D[1] = pc0[1];
        }
      }
    }

    //test intersection top
    if (!intersection)
    {
      double intersectionD = (pc3[1]-pl1[1])/(pl2[1]-pl1[1]);
      if (intersectionD>=0. && intersectionD<=1.)
      {
        double intersectionR = sqrt(lineCoeffD[2]*sqr(intersectionD) + lineCoeffD[1]*intersectionD + lineCoeffD[0]);
        if (intersectionR>=pc0[0] && intersectionR<=pc1[0])
        {
          intersection = true;
          distance = 0.;
          cp0_2D[0] = cp1_2D[0] = intersectionR;
          cp0_2D[1] = cp1_2D[1] = pc3[1];
        }
      }
    }

    if (!intersection)
    {
      //prefer side
      //enough to check endpoints and vertex
      distance = Rcs_distanceLinesegSqrtParabolaEndpoints2D(pc1, pc2, pl1, pl2, lineCoeffZ, cp1_2D, cp0_2D);

      double cp0_2D_temp[2];
      double cp1_2D_temp[2];
      double distance_temp;

      if (vertex)
      {
        distance_temp = Rcs_distancePointLineseg2D(vertexP, pc1, pc2, NULL, cp1_2D_temp);
        Rcs_copyPointIfCloser2D(distance_temp, vertexP, cp1_2D_temp, distance, cp0_2D, cp1_2D);
      }

      //min distance bottom
      //as this line is along the radius axis, checking the endpoints is enough
      distance_temp = Rcs_distanceLinesegSqrtParabolaEndpoints2D(pc0, pc1, pl1, pl2, lineCoeffZ,
                                                                 cp1_2D_temp, cp0_2D_temp);
      Rcs_copyPointIfCloser2D(distance_temp, cp0_2D_temp, cp1_2D_temp, distance, cp0_2D, cp1_2D);

      //min distance top
      //as this line is along the radius axis, checking the endpoints is enough
      distance_temp = Rcs_distanceLinesegSqrtParabolaEndpoints2D(pc2, pc3, pl1, pl2, lineCoeffZ,
                                                                 cp1_2D_temp, cp0_2D_temp);
      Rcs_copyPointIfCloser2D(distance_temp, cp0_2D_temp, cp1_2D_temp, distance, cp0_2D, cp1_2D);
    }
  }

  //find point on line in Cartesian coordinates
  if (cp0_ || cp1_)
  {
    // find % position between endpoints of line
    double minD = 0.0;

    if (pl1[1]==pl2[1])
    {
      Wm5::Polynomial1d poly(2);

      poly[2] = lineCoeffD[2];
      poly[1] = lineCoeffD[1];
      poly[0] = lineCoeffD[0] - sqr(cp0_2D[0]);

      Wm5::PolynomialRootsd polyroots(Wm5::Math<double>::ZERO_TOLERANCE);
      polyroots.FindA(poly[0], poly[1], poly[2]);
      int count = polyroots.GetCount();
      const double* roots = polyroots.GetRoots();

      for (int i = 0; i < count; ++i)
      {
        //ensure that between 2 end-points
        if (roots[i]>=0. && roots[i]<=1.)
        {
          minD = roots[i];
          break;
        }
      }
    }
    else
    {
      minD = (cp0_2D[1]-pl1[1])/(pl2[1]-pl1[1]);
    }

    double cp0_temp[3] = { p1Trafo[0]+minD* (p2Trafo[0] - p1Trafo[0]),
                           p1Trafo[1]+minD* (p2Trafo[1] - p1Trafo[1]),
                           cp0_2D[1]
                         };

    //transform back to Cartesian coordinates
    Vec3d_copy(cp0, cp0_temp);

    //transform back into global ref frame
    Vec3d_transformSelf(cp0, A_cylinder);

    // calculate angle of point on line, than find point on cone with same angle
    double angle;
    Rcs_Cart2Cyl(cp0_temp, cp0_2D[0], angle, cp0_2D[1]);

    Rcs_Cyl2Cart(cp1_2D[0], angle, cp1_2D[1], cp1);

    //transform back into global ref frame
    Vec3d_transformSelf(cp1, A_cylinder);
  }

  return distance;
}

/*******************************************************************************
 * Computes the distance between a line segment and a cone. The line segment
 * is defined by the points p1 and p2. The cone's transformation is given by
 * A_cone and its dimension by height and radius. Vectors cp0 and cp1 hold the
 * closest points of the distance query. They may be NULL.
 ******************************************************************************/
static double Rcs_distanceLinesegCone(const double p1[3],
                                      const double p2[3],
                                      const HTr* A_cone,
                                      double height,
                                      double radius,
                                      double cp0_[3],
                                      double cp1_[3])
{
  double buf0[3], buf1[3];
  double* cp0 = cp0_ ? cp0_ : buf0;
  double* cp1 = cp1_ ? cp1_ : buf1;

  //transform into cone ref frame.
  double p1Trafo[3];
  Vec3d_invTransform(p1Trafo, A_cone, p1);
  double p2Trafo[3];
  Vec3d_invTransform(p2Trafo, A_cone, p2);

  //transform into cylinder coordinates
  double pl1[2];
  double al1;
  Rcs_Cart2Cyl(p1Trafo, pl1[0], al1, pl1[1]);

  double pl2[2];
  double al2;
  Rcs_Cart2Cyl(p2Trafo, pl2[0], al2, pl2[1]);

  // cone points in cylinder coordinates
  double pc0[2] = {0, -.25*height}; //base
  double pc1[2] = {radius, -.25*height}; //rim
  double pc2[2] = {0, .75*height}; //tip

  //coefficients for line => sqrt(parabola) in terms of d (position in % between the two endpoints)
  double lineCoeffD[3];
  lineCoeffD[2] = sqr(p2Trafo[0]-p1Trafo[0]) + sqr(p2Trafo[1]-p1Trafo[1]);
  lineCoeffD[1] = 2.*(p1Trafo[0]*(p2Trafo[0]-p1Trafo[0]) +
                      p1Trafo[1]*(p2Trafo[1]-p1Trafo[1]));
  lineCoeffD[0] = sqr(p1Trafo[0]) + sqr(p1Trafo[1]);

  //coefficients for line => sqrt(parabola) in terms of z
  double lineCoeffZ[3];
  Vec3d_setZero(lineCoeffZ);
  if (fabs(p2Trafo[2]-p1Trafo[2])>0.)
  {
    lineCoeffZ[2] = lineCoeffD[2]/sqr(p2Trafo[2]-p1Trafo[2]);
    lineCoeffZ[1] = lineCoeffD[1]/(p2Trafo[2]-p1Trafo[2]) -
                    2.*lineCoeffD[2]*p1Trafo[2]/sqr(p2Trafo[2]-p1Trafo[2]);
    lineCoeffZ[0] = lineCoeffD[0] - lineCoeffD[1]*p1Trafo[2]/(p2Trafo[2] - p1Trafo[2]) +
                    lineCoeffD[2]*sqr(p1Trafo[2]/(p2Trafo[2]-p1Trafo[2]));
  }

  double sideCoeffZ[2] = {0.0, 0.0};
  if (fabs(pc2[1]-pc1[1])>0)
  {
    sideCoeffZ[1] = (pc2[0]-pc1[0])/(pc2[1]-pc1[1]);
    sideCoeffZ[0] = pc1[0] - pc1[1]*sideCoeffZ[1];
  }

  double cp0_2D[2];
  double cp1_2D[2];
  double distance = std::numeric_limits<double>::infinity();

  // if transformed line is a line => distanceLinesegLineseg
  // this happens if the line is parallel to the base or start and end angles
  // are the same
  if (pl1[1]==pl2[1] || al1==al2)
  {
    //if it is a horizontal line we need to take into account the vertex as
    // potential end-point
    if (pl1[1]==pl2[1])
    {
      //calculate vertex
      double vertexD = -lineCoeffD[1]/(2.*lineCoeffD[2]);
      if (vertexD>0. && vertexD<1.)
      {
        double vertexValue = sqrt(lineCoeffD[2]*sqr(vertexD) +
                                  lineCoeffD[1]*vertexD + lineCoeffD[0]);
        if ((vertexValue<pl1[0] && pl1[0]<=pl2[0]) || (vertexValue>pl1[0] && pl1[0]>=pl2[0]))
        {
          pl1[0] = vertexValue;
        }
        else if ((vertexValue<pl2[0] && pl2[0]<pl1[0]) || (vertexValue>pl2[0] && pl2[0]>pl1[0]))
        {
          pl2[0] = vertexValue;
        }
      }
    }

    distance = Rcs_distanceLinesegLineseg2D(pl1, pl2, pc0, pc1, cp0_2D, cp1_2D); //prefer bottom

    double cp0_2D_temp[2];
    double cp1_2D_temp[2];
    double distance_temp = Rcs_distanceLinesegLineseg2D(pl1, pl2, pc1, pc2, cp0_2D_temp, cp1_2D_temp);
    Rcs_copyPointIfCloser2D(distance_temp, cp0_2D_temp, cp1_2D_temp, distance, cp0_2D, cp1_2D);
  }
  else //line is sqrt(parabola)
  {
    bool intersection = false;
    double limits[2] = {std::min(pl1[1], pl2[1]), std::max(pl1[1], pl2[1])};

    //test intersection bottom
    double intersectionD = (pc0[1]-pl1[1])/(pl2[1]-pl1[1]);
    // on line?
    if (intersectionD>=0. && intersectionD<=1.)
    {
      double intersectionValue = sqrt(lineCoeffD[2]*sqr(intersectionD) + lineCoeffD[1]*intersectionD + lineCoeffD[0]);
      // on cone?
      if (intersectionValue>=pc0[0] && intersectionValue<=pc1[0])
      {
        intersection = true;
        distance = 0.;
        cp0_2D[0] = cp1_2D[0] = intersectionValue;
        cp0_2D[1] = cp1_2D[1] = pc0[1];
      }
    }

    //test intersection side
    if (!intersection && pc1[1]!=pc2[1])
    {
      Wm5::Polynomial1d poly(2);

      poly[2] = lineCoeffZ[2] - sqr(sideCoeffZ[1]);
      poly[1] = lineCoeffZ[1] - 2.*sideCoeffZ[1]*sideCoeffZ[0];
      poly[0] = lineCoeffZ[0] - sqr(sideCoeffZ[0]);

      Wm5::PolynomialRootsd polyroots(Wm5::Math<double>::ZERO_TOLERANCE);
      polyroots.FindA(poly[0], poly[1], poly[2]);
      int count = polyroots.GetCount();
      const double* roots = polyroots.GetRoots();

      for (int i = 0; i < count; ++i)
      {
        if (roots[i]>=limits[0] && roots[i]<=limits[1] && roots[i]>=pc1[1] && roots[i]<=pc2[1])
        {
          intersection = true;
          distance = 0.;
          cp0_2D[0] = cp1_2D[0] = sqrt(lineCoeffZ[2]*sqr(roots[i]) + lineCoeffZ[1]*roots[i] + lineCoeffZ[0]);
          cp0_2D[1] = cp1_2D[1] = roots[i];
          break;
        }
      }
    }

    if (!intersection)
    {
      //min distance bottom
      //as this line is along the radius axis, checking the endpoints is enough
      distance = Rcs_distanceLinesegSqrtParabolaEndpoints2D(pc0, pc1, pl1, pl2, lineCoeffZ, cp1_2D, cp0_2D);
      //printf("dist: %f cp0_2D %f %f cp1_2D %f %f\n",distance,cp0_2D[0],cp0_2D[1],cp1_2D[0],cp1_2D[1]);

      //min distance side
      if (pc1[1]!=pc2[1]) //degenerate cone = disk
      {
        double cp0_2D_temp[2];
        double cp1_2D_temp[2];
        double distance_temp;

        //check same slope candidates
        Wm5::Polynomial1d poly(2);

        poly[2] = sqr(lineCoeffZ[2]) - lineCoeffZ[2]*sqr(sideCoeffZ[1]);
        poly[1] = lineCoeffZ[2]*lineCoeffZ[1] - lineCoeffZ[1]*sqr(sideCoeffZ[1]);
        poly[0] = sqr(lineCoeffZ[1])/4. - lineCoeffZ[0]*sqr(sideCoeffZ[1]);

        Wm5::PolynomialRootsd polyroots(Wm5::Math<double>::ZERO_TOLERANCE);
        polyroots.FindA(poly[0], poly[1], poly[2]);
        int count = polyroots.GetCount();
        const double* roots = polyroots.GetRoots();

        //printf("limits %f %f\n",limits[0],limits[1]);
        for (int i = 0; i < count; ++i)
        {
          //printf("roots[i] %f\n",roots[i]);
          //on line?
          if (roots[i]>=limits[0] && roots[i]<=limits[1])
          {
            cp0_2D_temp[0] = sqrt(lineCoeffZ[2]*sqr(roots[i]) +
                                  lineCoeffZ[1]*roots[i] + lineCoeffZ[0]);
            cp0_2D_temp[1] = roots[i];

            //find closest point on cone (as it is checking the segment not necessarily the one with the same slope)
            distance_temp = Rcs_distancePointLineseg2D(cp0_2D_temp, pc1, pc2, NULL, cp1_2D_temp);
            Rcs_copyPointIfCloser2D(distance_temp, cp0_2D_temp, cp1_2D_temp, distance, cp0_2D, cp1_2D);
          }
        }

        //check endpoints
        distance_temp = Rcs_distanceLinesegSqrtParabolaEndpoints2D(pc1, pc2, pl1, pl2, lineCoeffZ,
                                                                   cp1_2D_temp, cp0_2D_temp);
        Rcs_copyPointIfCloser2D(distance_temp, cp0_2D_temp, cp1_2D_temp, distance, cp0_2D, cp1_2D);
      } // check side
    } // no intersection
  } // !LinesegLineseg

  //find point on line in Cartesian coordinates
  if (cp0_ || cp1_)
  {
    // find % position between endpoints of line
    double minD = 0.0;

    if (pl1[1]==pl2[1])
    {
      Wm5::Polynomial1d poly(2);

      poly[2] = lineCoeffD[2];
      poly[1] = lineCoeffD[1];
      poly[0] = lineCoeffD[0] - sqr(cp0_2D[0]);

      Wm5::PolynomialRootsd polyroots(Wm5::Math<double>::ZERO_TOLERANCE);
      polyroots.FindA(poly[0], poly[1], poly[2]);
      int count = polyroots.GetCount();
      const double* roots = polyroots.GetRoots();

      for (int i = 0; i < count; ++i)
      {
        //ensure that between 2 end-points
        if (roots[i]>=0. && roots[i]<=1.)
        {
          minD = roots[i];
          break;
        }
      }
    }
    else
    {
      minD = (cp0_2D[1]-pl1[1])/(pl2[1]-pl1[1]);
    }

    double cp0_temp[3] = { p1Trafo[0]+minD* (p2Trafo[0] - p1Trafo[0]),
                           p1Trafo[1]+minD* (p2Trafo[1] - p1Trafo[1]),
                           cp0_2D[1]
                         };

    //transform back to Cartesian coordinates
    if (cp0_)
    {
      Vec3d_copy(cp0, cp0_temp);

      //transform back into global ref frame
      Vec3d_transformSelf(cp0, A_cone);
    }

    if (cp1_)
    {
      // calculate angle of point on line, than find point on cone with
      // same angle
      double angle;
      Rcs_Cart2Cyl(cp0_temp, cp0_2D[0], angle, cp0_2D[1]);

      Rcs_Cyl2Cart(cp1_2D[0], angle, cp1_2D[1], cp1);

      //transform back into global ref frame
      Vec3d_transformSelf(cp1, A_cone);
    }
  }

  return distance;
}




// bool Rcs_intersectionPlaneCylinder(const double planePt[3],
//           const double planeNormal[3],
//           const double cylinderPt[3],
//           const double cylinderDir[3],
//           double radius)
// {
//   Wm5::Plane3d plane(Vec3(planeNormal[0], planeNormal[1], planeNormal[2]),
//         Vec3(planePt[0], planePt[1], planePt[2]) );

//   Wm5::Line3d cylLine(Vec3(cylinderPt[0], cylinderPt[1], cylinderPt[2]),
//          Vec3(cylinderDir[0], cylinderDir[1], cylinderDir[2]) );

//   Wm5::Cylinder3d cylinder(cylLine, radius, 0.5*Vec3d_getLength(cylinderDir));

//   Wm5::IntrPlane3Cylinder3d intrsec(plane, cylinder);

//   bool isIntersecting = intrsec.Test();

//   RLOG(0, "planePt: %f %f %f", planePt[0], planePt[1], planePt[2]);
//   RLOG(0, "planeDir: %f %f %f", planeNormal[0], planeNormal[1], planeNormal[2]);
//   RLOG(0, "cylinderPt: %f %f %f", cylinderPt[0], cylinderPt[1], cylinderPt[2]);
//   RLOG(0, "cylinderDir: %f %f %f", cylinderDir[0], cylinderDir[1], cylinderDir[2]);
//   RLOG(0, "Raduis: %f   Intersection: %d", radius, isIntersecting);

//   return isIntersecting;
// }



/*******************************************************************************
 * Computes the distance between two torus primitives.
 ******************************************************************************/
static double RcsShape_closestTorusToTorus(const RcsShape* t1,
                                           const RcsShape* t2,
                                           const HTr* A_t1,
                                           const HTr* A_t2,
                                           double I_cp1[3],
                                           double I_cp2[3],
                                           double I_n12[3])
{
  Wm5::Circle3d c1(Vec3(A_t1->org[0], A_t1->org[1], A_t1->org[2]),
                   Vec3(A_t1->rot[0][0], A_t1->rot[0][1], A_t1->rot[0][2]),
                   Vec3(A_t1->rot[1][0], A_t1->rot[1][1], A_t1->rot[1][2]),
                   Vec3(A_t1->rot[2][0], A_t1->rot[2][1], A_t1->rot[2][2]),
                   t1->extents[0]);

  Wm5::Circle3d c2(Vec3(A_t2->org[0], A_t2->org[1], A_t2->org[2]),
                   Vec3(A_t2->rot[0][0], A_t2->rot[0][1], A_t2->rot[0][2]),
                   Vec3(A_t2->rot[1][0], A_t2->rot[1][1], A_t2->rot[1][2]),
                   Vec3(A_t2->rot[2][0], A_t2->rot[2][1], A_t2->rot[2][2]),
                   t2->extents[0]);

  Wm5::DistCircle3Circle3d dist(c1, c2);

  double distance = dist.Get();

  I_cp1[0] = dist.GetClosestPoint0()[0];
  I_cp1[1] = dist.GetClosestPoint0()[1];
  I_cp1[2] = dist.GetClosestPoint0()[2];

  I_cp2[0] = dist.GetClosestPoint1()[0];
  I_cp2[1] = dist.GetClosestPoint1()[1];
  I_cp2[2] = dist.GetClosestPoint1()[2];

  Vec3d_sub(I_n12, I_cp2, I_cp1);
  Vec3d_normalizeSelf(I_n12);

  Vec3d_constMulAndAddSelf(I_cp1, I_n12, 0.5*t1->extents[2]);
  Vec3d_constMulAndAddSelf(I_cp2, I_n12, -0.5*t2->extents[2]);

  return distance - 0.5*t1->extents[2] - 0.5*t2->extents[2];
}

/*******************************************************************************
 * Computes the distance between a SSL and a cone shape primitives.
 ******************************************************************************/
static double RcsShape_closestSSLToCone(const RcsShape* ssl,
                                        const RcsShape* cone,
                                        const HTr* A_ssl,
                                        const HTr* A_cone,
                                        double I_cp1[3],
                                        double I_cp2[3],
                                        double I_n[3])
{
  // Compute ball points in world coordinates
  double K_bp[3], I_bp[3];
  Vec3d_set(K_bp, 0.0, 0.0, ssl->extents[2]);
  Vec3d_transform(I_bp, A_ssl, K_bp);

  double dist = Rcs_distanceLinesegCone(A_ssl->org, I_bp, A_cone,
                                        cone->extents[2], cone->extents[0],
                                        I_cp1, I_cp2);

  // Normal vector on sphere
  Vec3d_sub(I_n, I_cp2, I_cp1);
  Vec3d_normalizeSelf(I_n);

  // Point on sphere surface: cp_sphere = cp_point + radius_sphere*n
  Vec3d_constMulAndAddSelf(I_cp1, I_n, ssl->extents[0]);

  return dist;
}

/*******************************************************************************
 * Computes the distance between a SSL and a cone shape primitives.
 ******************************************************************************/
static inline double RcsShape_closestConeToSSL(const RcsShape* cone,
                                               const RcsShape* ssl,
                                               const HTr* A_cone,
                                               const HTr* A_ssl,
                                               double I_cp2[3],
                                               double I_cp1[3],
                                               double I_n[3])
{
  double dist = RcsShape_closestSSLToCone(ssl, cone, A_ssl, A_cone,
                                          I_cp1, I_cp2, I_n);

  // Revert the normal, because we are calling the reverse method
  Vec3d_constMulSelf(I_n, -1.0);

  return dist;
}

/*******************************************************************************
 * Computes the distance between a sphere and a cone shape primitives.
 ******************************************************************************/
// static double RcsShape_closestSphereToCone(const RcsShape* sphere,
//                                            const RcsShape* cone,
//                                            const HTr* A_sphere,
//                                            const HTr* A_cone,
//                                            double I_cp1[3],
//                                            double I_cp2[3],
//                                            double I_n[3])
// {
//   double dist = Rcs_distancePointCone(A_sphere->org, A_cone, cone->extents[2],
//                                       cone->extents[0], I_cp1, I_cp2);

//   // Normal vector on sphere
//   Vec3d_sub(I_n, I_cp2, I_cp1);
//   Vec3d_normalizeSelf(I_n);

//   // Point on sphere surface: cp_sphere = cp_point + radius_sphere*n
//   Vec3d_constMulAndAddSelf(I_cp1, I_n, sphere->extents[0]);

//   return dist - sphere->extents[0];
// }

/*******************************************************************************
 * Computes the distance between a cone and a sphere shape primitives.
 ******************************************************************************/
// static double RcsShape_closestConeToSphere(const RcsShape* cone,
//                                            const RcsShape* sphere,
//                                            const HTr* A_cone,
//                                            const HTr* A_sphere,
//                                            double I_cp2[3],
//                                            double I_cp1[3],
//                                            double I_n[3])
// {
//   double dist = RcsShape_closestSphereToCone(sphere, cone, A_sphere, A_cone,
//                                              I_cp1, I_cp2, I_n);
//   // revert the normal, because we are calling the reverse method
//   Vec3d_constMulSelf(I_n, -1.0);
//   return dist;
// }

/*******************************************************************************
 * Computes the distance between a SSL and a cylinder shape primitives.
 ******************************************************************************/
static double RcsShape_closestSSLToCylinder(const RcsShape* ssl,
                                            const RcsShape* cylinder,
                                            const HTr* A_ssl,
                                            const HTr* A_cylinder,
                                            double I_cp1[3],
                                            double I_cp2[3],
                                            double I_n[3])
{
  // Compute ball points in world coordinates
  double K_bp[3], I_bp[3];
  Vec3d_set(K_bp, 0.0, 0.0, ssl->extents[2]);
  Vec3d_transform(I_bp, A_ssl, K_bp);

  double dist = Rcs_distanceLinesegCylinder(A_ssl->org, I_bp, A_cylinder,
                                            cylinder->extents[2],
                                            cylinder->extents[0],
                                            I_cp1, I_cp2);

  // Normal vector on sphere
  Vec3d_sub(I_n, I_cp2, I_cp1);
  Vec3d_normalizeSelf(I_n);

  // Point on sphere surface: cp_sphere = cp_point + radius_sphere*n
  Vec3d_constMulAndAddSelf(I_cp1, I_n, ssl->extents[0]);

  return dist;
}

/*******************************************************************************
 * Computes the distance between a SSL and a cylinder shape primitives.
 ******************************************************************************/
static inline double RcsShape_closestCylinderToSSL(const RcsShape* cylinder,
                                                   const RcsShape* ssl,
                                                   const HTr* A_cylinder,
                                                   const HTr* A_ssl,
                                                   double I_cp2[3],
                                                   double I_cp1[3],
                                                   double I_n[3])
{
  double dist = RcsShape_closestSSLToCylinder(ssl, cylinder, A_ssl, A_cylinder,
                                              I_cp1, I_cp2, I_n);
  // revert the normal, because we are calling the reverse method
  Vec3d_constMulSelf(I_n, -1.0);
  return dist;
}

/*******************************************************************************
 * Computes the distance between a sphere and a cylinder shape primitives.
 ******************************************************************************/
static double RcsShape_closestSphereToCylinder(const RcsShape* sphere,
                                               const RcsShape* cylinder,
                                               const HTr* A_sphere,
                                               const HTr* A_cylinder,
                                               double I_cp1[3],
                                               double I_cp2[3],
                                               double I_n[3])
{
  RLOG(0, "AAA");
  double dist = Rcs_distancePointCylinder(A_sphere->org, A_cylinder,
                                          cylinder->extents[2],
                                          cylinder->extents[0],
                                          I_cp1, I_cp2);

  // Normal vector on sphere
  Vec3d_sub(I_n, I_cp2, I_cp1);
  Vec3d_normalizeSelf(I_n);

  // Point on sphere surface: cp_sphere = cp_point + radius_sphere*n
  Vec3d_constMulAndAddSelf(I_cp1, I_n, sphere->extents[0]);

  return dist-sphere->extents[0];
}

/*******************************************************************************
 * Computes the distance between a cylinder and a sphere shape primitives.
 ******************************************************************************/
static double RcsShape_closestCylinderToSphere(const RcsShape* cylinder,
                                               const RcsShape* sphere,
                                               const HTr* A_cylinder,
                                               const HTr* A_sphere,
                                               double I_cp2[3],
                                               double I_cp1[3],
                                               double I_n[3])
{
  double dist = RcsShape_closestSphereToCylinder(sphere, cylinder, A_sphere,
                                                 A_cylinder, I_cp1, I_cp2, I_n);
  // revert the normal, because we are calling the reverse method
  Vec3d_constMulSelf(I_n, -1.0);
  return dist;
}

/*******************************************************************************
 * Computes the distance between a point and a torus shape primitives.
 ******************************************************************************/
static double RcsShape_closestPointToTorus(const RcsShape* pt,
                                           const RcsShape* torus,
                                           const HTr* A_pt,
                                           const HTr* A_torus,
                                           double I_cp1[3],
                                           double I_cp2[3],
                                           double I_n[3])
{
  double tRadius = torus->extents[0];
  double tThickness = torus->extents[2];
  double d = Rcs_distancePointCircle(A_pt->org,  A_torus, tRadius, I_cp1, I_cp2);
  Vec3d_sub(I_n, I_cp2, I_cp1);
  Vec3d_normalizeSelf(I_n);
  Vec3d_constMulAndAddSelf(I_cp2, I_n, -0.5*tThickness);
  return d - 0.5*tThickness;
}

/*******************************************************************************
 * Computes the distance between a torus and a point shape primitives.
 ******************************************************************************/
static double RcsShape_closestTorusToPoint(const RcsShape* torus,
                                           const RcsShape* pt,
                                           const HTr* A_torus,
                                           const HTr* A_pt,
                                           double I_cp1[3],
                                           double I_cp2[3],
                                           double I_n[3])
{
  double d = RcsShape_closestPointToTorus(pt, torus, A_pt, A_torus,
                                          I_cp2, I_cp1, I_n);
  Vec3d_constMulSelf(I_n, -1.0);
  return d;
}

/*******************************************************************************
 * Computes the distance between a sphere and a torus shape primitives.
 ******************************************************************************/
static double RcsShape_closestSphereToTorus(const RcsShape* sphere,
                                            const RcsShape* torus,
                                            const HTr* A_sphere,
                                            const HTr* A_torus,
                                            double I_cp1[3],
                                            double I_cp2[3],
                                            double I_n[3])
{
  double tRadius = torus->extents[0];
  double tThickness = torus->extents[2];
  double d = Rcs_distancePointCircle(A_sphere->org,  A_torus, tRadius,
                                     I_cp1, I_cp2);
  Vec3d_sub(I_n, I_cp2, I_cp1);
  Vec3d_normalizeSelf(I_n);
  Vec3d_constMulAndAddSelf(I_cp1, I_n, sphere->extents[0]);
  Vec3d_constMulAndAddSelf(I_cp2, I_n, -0.5*tThickness);
  return d - 0.5*tThickness;
}

/*******************************************************************************
 * Computes the distance between a torus and a sphere shape primitives.
 ******************************************************************************/
static double RcsShape_closestTorusToSphere(const RcsShape* torus,
                                            const RcsShape* sphere,
                                            const HTr* A_torus,
                                            const HTr* A_sphere,
                                            double I_cp1[3],
                                            double I_cp2[3],
                                            double I_n[3])
{
  double d = RcsShape_closestSphereToTorus(sphere, torus, A_sphere, A_torus,
                                           I_cp2, I_cp1, I_n);
  Vec3d_constMulSelf(I_n, -1.0);
  return d;
}

/*******************************************************************************
 * Computes the distance between a SSL and a torus shape primitives.
 ******************************************************************************/
static double RcsShape_closestSSLToTorus(const RcsShape* ssl,
                                         const RcsShape* torus,
                                         const HTr* A_ssl,
                                         const HTr* A_torus,
                                         double I_cp1[3],
                                         double I_cp2[3],
                                         double I_n[3])
{
  // Compute ball points in world coordinates
  double K_bp[3], I_bp[3];
  Vec3d_set(K_bp, 0.0, 0.0, ssl->extents[2]);
  Vec3d_transform(I_bp, A_ssl, K_bp);

  return Rcs_distanceCapsuleTorus(A_ssl->org, I_bp, 2.0 * ssl->extents[0],
                                  A_torus, torus->extents[0],
                                  torus->extents[2],
                                  I_cp1, I_cp2, I_n);
}

/*******************************************************************************
 * Computes the distance between a SSL and a torus shape primitives.
 ******************************************************************************/
static double RcsShape_closestTorusToSSL(const RcsShape* torus,
                                         const RcsShape* ssl,
                                         const HTr* A_torus,
                                         const HTr* A_ssl,
                                         double I_cp2[3],
                                         double I_cp1[3],
                                         double I_n[3])
{
  double dist = RcsShape_closestSSLToTorus(ssl, torus, A_ssl, A_torus,
                                           I_cp1, I_cp2, I_n);

  // revert the normal, because we are calling the reverse method
  Vec3d_constMulSelf(I_n, -1.0);
  return dist;
}

/*******************************************************************************
 * Computes the distance between a SSR and a box shape primitives sh1
 * and sh2 that are associated with transformations A_bxI.
 ******************************************************************************/
static double RcsShape_closestSSRToBox(const RcsShape* s1,
                                       const RcsShape* s2,
                                       const HTr* A_S1I,
                                       const HTr* A_S2I,
                                       double cp1[3],
                                       double cp2[3],
                                       double I_n[3])
{
  return Rcs_distanceSSRBox(A_S1I, s1->extents, A_S2I, s2->extents, cp1, cp2,
                            I_n);
}

/*******************************************************************************
 * Other way around than RcsShape_closestSSRToBox()
 ******************************************************************************/
static double RcsShape_closestBoxToSSR(const RcsShape* s1,
                                       const RcsShape* s2,
                                       const HTr* A_b1I,
                                       const HTr* A_b2I,
                                       double cp1[3],
                                       double cp2[3],
                                       double I_n[3])
{
  double dist = RcsShape_closestSSRToBox(s2, s1, A_b2I, A_b1I, cp2, cp1, I_n);

  // revert the normal, because we are calling the reverse method
  Vec3d_constMulSelf(I_n, -1.0);
  return dist;
}

/*******************************************************************************
 * Computes the distance between two SSR shape primitives sh1 and sh2
 * that are associated with transformations A_bxI.
 ******************************************************************************/
static double RcsShape_closestSSRToSSR(const RcsShape* s1,
                                       const RcsShape* s2,
                                       const HTr* A_S1I,
                                       const HTr* A_S2I,
                                       double I_cp1[3],
                                       double I_cp2[3],
                                       double I_n12[3])
{
  double distance = Rcs_distancePlanesegPlaneseg(A_S1I, s1->extents[0],
                                                 s1->extents[1], A_S2I,
                                                 s2->extents[0], s2->extents[1],
                                                 I_cp1, I_cp2);

  // Normalized direction vector from closest connection (1-->2))
  Vec3d_sub(I_n12, I_cp2, I_cp1);
  Vec3d_normalizeSelf(I_n12);

  // Surface points
  for (int i = 0; i < 3; i++)
  {
    I_cp1[i] += 0.5 * s1->extents[2] * I_n12[i];
    I_cp2[i] -= 0.5 * s2->extents[2] * I_n12[i];
  }

  return distance - 0.5 * (s1->extents[2] + s2->extents[2]);
}

/*******************************************************************************
 * Computes the distance between a SSL and box shape primitives sh1 and sh2
 * that are associated with transformations A_bxI.
 ******************************************************************************/
double RcsShape_closestSSLToBox(const RcsShape* s1,
                                const RcsShape* s2,
                                const HTr* A_S1I,
                                const HTr* A_S2I,
                                double I_cp1[3],
                                double I_cp2[3],
                                double I_n[3])
{
  // SSL: Compute ball points in world coordinates (I_pt1 and I_pt2)
  double K_bp[3], I_bp[3];
  Vec3d_set(K_bp, 0.0, 0.0, s1->extents[2]);
  Vec3d_transform(I_bp, A_S1I, K_bp);

  return Rcs_distanceCapsuleBox(A_S1I->org, I_bp, 2.0*s1->extents[0],
                                A_S2I, s2->extents, I_cp1, I_cp2, I_n);
}

/*******************************************************************************
 * Computes the distance between a SSL and box shape primitives sh1 and
 * sh2 that are associated with transformations A_bxI.
 ******************************************************************************/
static inline double RcsShape_closestBoxToSSL(const RcsShape* s1,
                                              const RcsShape* s2,
                                              const HTr* A_b1I,
                                              const HTr* A_b2I,
                                              double cp1[3],
                                              double cp2[3],
                                              double I_n[3])
{
  double dist = RcsShape_closestSSLToBox(s2, s1, A_b2I, A_b1I, cp2, cp1, I_n);

  // revert the normal, because we are calling the reverse method
  Vec3d_constMulSelf(I_n, -1.0);
  return dist;
}

/*******************************************************************************
 * Computes the distance between two box shape primitives sh1 and sh2 that are
 * associated with transformations A_bxI.
 ******************************************************************************/
static double RcsShape_closestBoxToBox(const RcsShape* s1,
                                       const RcsShape* s2,
                                       const HTr* A_S1I,
                                       const HTr* A_S2I,
                                       double cp1[3],
                                       double cp2[3],
                                       double I_n[3])
{
  double d = Rcs_distanceBoxBox(A_S1I, s1->extents, A_S2I, s2->extents,
                                cp1, cp2);
  Vec3d_sub(I_n, cp2, cp1);
  Vec3d_normalizeSelf(I_n);

  if (d < 0.0)
  {
    Vec3d_constMulSelf(I_n, -1.0);
  }

  return d;
}

/*******************************************************************************
 * SSL to SSR distance computation.
 ******************************************************************************/
static double RcsShape_closestSSLToSSR(const RcsShape* ssl,
                                       const RcsShape* ssr,
                                       const HTr* A_sslI,
                                       const HTr* A_ssrI,
                                       double cpSSL[3],
                                       double cpSSR[3],
                                       double I_n[3])
{
  // SSL: Compute ball point in world coordinates
  double K_pt2[3], I_pt2[3];
  Vec3d_set(K_pt2, 0.0, 0.0, ssl->extents[2]);
  Vec3d_transform(I_pt2, A_sslI, K_pt2);

  return Rcs_distanceCapsuleSSR(A_sslI->org, I_pt2, 2.0 * ssl->extents[0],
                                A_ssrI, ssr->extents, cpSSL, cpSSR, I_n);
}

/*******************************************************************************
 * SSR to SSL distance computation.
 ******************************************************************************/
static double RcsShape_closestSSRToSSL(const RcsShape* ssr,
                                       const RcsShape* ssl,
                                       const HTr* A_ssrI,
                                       const HTr* A_sslI,
                                       double cpSSR[3],
                                       double cpSSL[3],
                                       double I_n[3])
{
  double dist = RcsShape_closestSSLToSSR(ssl, ssr, A_sslI, A_ssrI,
                                         cpSSL, cpSSR, I_n);

  // revert the normal, because we are calling the reverse method
  Vec3d_constMulSelf(I_n, -1.0);
  return dist;
}

/*******************************************************************************
 * Computes the distance between two SSL shape primitives.
 ******************************************************************************/
static double RcsShape_closestSSLToSSL(const RcsShape* ssl1,
                                       const RcsShape* ssl2,
                                       const HTr* A_ssl1I,
                                       const HTr* A_ssl2I,
                                       double I_cp1[3],
                                       double I_cp2[3],
                                       double I_n[3])
{
  // Compute ball points in world coordinates
  double I_bp1[3], I_bp2[3], K_bp1[3], K_bp2[3];
  Vec3d_set(K_bp1, 0.0, 0.0, ssl1->extents[2]);
  Vec3d_set(K_bp2, 0.0, 0.0, ssl2->extents[2]);
  Vec3d_transform(I_bp1, A_ssl1I, K_bp1);
  Vec3d_transform(I_bp2, A_ssl2I, K_bp2);

  return Rcs_distanceCapsuleCapsule(A_ssl1I->org, I_bp1, 2.0 * ssl1->extents[0],
                                    A_ssl2I->org, I_bp2, 2.0 * ssl2->extents[0],
                                    I_cp1, I_cp2, I_n);
}

/*******************************************************************************
 * Computes the distance between a point and a SSR.
 ******************************************************************************/
static double RcsShape_closestPointToSSR(const RcsShape* pt,
                                         const RcsShape* ssr,
                                         const HTr* A_ptI,
                                         const HTr* A_ssrI,
                                         double I_cp1[3],
                                         double I_cp2[3],
                                         double I_n12[3])
{
  double d = Rcs_distancePointPlaneseg(A_ptI->org, A_ssrI,
                                       ssr->extents[0], ssr->extents[1],
                                       I_cp1, I_cp2);

  Vec3d_sub(I_n12, I_cp2, I_cp1);
  Vec3d_normalizeSelf(I_n12);

  // Surface point
  for (int i = 0; i < 3; i++)
  {
    I_cp2[i] -= 0.5 * ssr->extents[2]*I_n12[i];
  }

  return d - 0.5*ssr->extents[2];
}

/*******************************************************************************
 * SSR to Point distance computation.
 ******************************************************************************/
static inline double RcsShape_closestSSRToPoint(const RcsShape* ssr,
                                                const RcsShape* pt,
                                                const HTr* A_ssrI,
                                                const HTr* A_ptI,
                                                double cpSSR[3],
                                                double cpPt[3],
                                                double I_n[3])
{
  double dist = RcsShape_closestPointToSSR(pt, ssr, A_ptI, A_ssrI,
                                           cpPt, cpSSR, I_n);

  // revert the normal, because we are calling the reverse method
  Vec3d_constMulSelf(I_n, -1.0);

  return dist;
}

/*******************************************************************************
 * Add Bullet distance functions to the array of distance functions
 ******************************************************************************/
static bool setWildMagicDistanceFunctions()
{
  bool success = true;

  // SSL
  if (RcsShape_getDistanceFunction(RCSSHAPE_SSL, RCSSHAPE_SSL)==NULL)
  {
    success = RcsShape_setDistanceFunction(RCSSHAPE_SSL, RCSSHAPE_SSL,
                                           RcsShape_closestSSLToSSL) && success;
  }

  success = RcsShape_setDistanceFunction(RCSSHAPE_SSL, RCSSHAPE_SSR,
                                         RcsShape_closestSSLToSSR) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_SSL, RCSSHAPE_BOX,
                                         RcsShape_closestSSLToBox) && success;
  //success = RcsShape_setDistanceFunction(RCSSHAPE_SSL, RCSSHAPE_CYLINDER,
  //                                       RcsShape_closestSSLToCylinder)
  //          && success;
  //success = RcsShape_setDistanceFunction(RCSSHAPE_SSL, RCSSHAPE_CONE,
  //                                       RcsShape_closestSSLToCone) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_SSL, RCSSHAPE_TORUS,
                                         RcsShape_closestSSLToTorus) && success;

  // SSR
  success = RcsShape_setDistanceFunction(RCSSHAPE_SSR, RCSSHAPE_SSL,
                                         RcsShape_closestSSRToSSL) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_SSR, RCSSHAPE_SSR,
                                         RcsShape_closestSSRToSSR) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_SSR, RCSSHAPE_BOX,
                                         RcsShape_closestSSRToBox) && success;
  //if (RcsShape_getDistanceFunction(RCSSHAPE_SSR, RCSSHAPE_POINT)==NULL)
  //{
  //  success = RcsShape_setDistanceFunction(RCSSHAPE_SSR, RCSSHAPE_POINT,
  //                                         RcsShape_closestSSRToPoint) && success;
  //}

  // BOX
  success = RcsShape_setDistanceFunction(RCSSHAPE_BOX, RCSSHAPE_SSL,
                                         RcsShape_closestBoxToSSL) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_BOX, RCSSHAPE_SSR,
                                         RcsShape_closestBoxToSSR) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_BOX, RCSSHAPE_BOX,
                                         RcsShape_closestBoxToBox) && success;
  //success = RcsShape_setDistanceFunction(RCSSHAPE_BOX, RCSSHAPE_SPHERE,
  //                                       RcsShape_closestBoxToSSL) && success;

  // CYLINDER
  //success = RcsShape_setDistanceFunction(RCSSHAPE_CYLINDER, RCSSHAPE_SSL,
  //                                       RcsShape_closestCylinderToSSL)
  //          && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_CYLINDER, RCSSHAPE_SPHERE,
                                         RcsShape_closestCylinderToSphere)
            && success;
  //success = RcsShape_setDistanceFunction(RCSSHAPE_CYLINDER, RCSSHAPE_POINT,
  //                                       RcsShape_closestCylinderToSphere)
  //          && success;

  // SPHERE
  //success = RcsShape_setDistanceFunction(RCSSHAPE_SPHERE, RCSSHAPE_BOX,
  //                                       RcsShape_closestSSLToBox) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_SPHERE, RCSSHAPE_CYLINDER,
                                         RcsShape_closestSphereToCylinder)
            && success;
  //success = RcsShape_setDistanceFunction(RCSSHAPE_SPHERE, RCSSHAPE_CONE,
  //                                       RcsShape_closestSphereToCone)
  //          && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_SPHERE, RCSSHAPE_TORUS,
                                         RcsShape_closestSphereToTorus) && success;

  // CONE
  //success = RcsShape_setDistanceFunction(RCSSHAPE_CONE, RCSSHAPE_SSL,
  //                                       RcsShape_closestConeToSSL) && success;
  //success = RcsShape_setDistanceFunction(RCSSHAPE_CONE, RCSSHAPE_SPHERE,
  //                                       RcsShape_closestConeToSphere)
  //          && success;
  //success = RcsShape_setDistanceFunction(RCSSHAPE_CONE, RCSSHAPE_POINT,
  //                                       RcsShape_closestConeToSphere)
  //          && success;

  // TORUS
  success = RcsShape_setDistanceFunction(RCSSHAPE_TORUS, RCSSHAPE_SSL,
                                         RcsShape_closestTorusToSSL) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_TORUS, RCSSHAPE_SPHERE,
                                         RcsShape_closestTorusToSphere) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_TORUS, RCSSHAPE_TORUS,
                                         RcsShape_closestTorusToTorus) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_TORUS, RCSSHAPE_POINT,
                                         RcsShape_closestTorusToPoint) && success;

  // Point
  //if (RcsShape_getDistanceFunction(RCSSHAPE_POINT, RCSSHAPE_SSR)==NULL)
  //{
  //  success = RcsShape_setDistanceFunction(RCSSHAPE_POINT, RCSSHAPE_SSR,
  //                                         RcsShape_closestPointToSSR) && success;
  //}

  //success = RcsShape_setDistanceFunction(RCSSHAPE_POINT, RCSSHAPE_CYLINDER,
  //                                       RcsShape_closestSphereToCylinder)
  //          && success;
  //success = RcsShape_setDistanceFunction(RCSSHAPE_POINT, RCSSHAPE_CONE,
  //                                       RcsShape_closestSphereToCone)
  //          && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_POINT, RCSSHAPE_TORUS,
                                         RcsShape_closestPointToTorus) && success;

  NLOG(5, "%s WM5 distance functions",
       success ? "SUCCESFULLY added" : "FAILED to add");

  return success;
}

// This is being called before main()
static bool distanceInitialized = setWildMagicDistanceFunctions();

#else // USE_WM5


bool Rcs_containedPoint2DConvexPolygon2D(const double pt[2],
                                         const double polygon[][2],
                                         unsigned int N)
{
  RLOG(4, "Rcs_containedPoint2DConvexPolygon2D: WM5 library not available");
  return false;
}

double Rcs_distancePoint2DConvexPolygon2D(const double pt[2],
                                          const double polygon[][2],
                                          unsigned int N,
                                          double cp0[2],
                                          double cp1[2])
{
  RLOG(4, "Rcs_distancePoint2DConvexPolygon2D: WM5 library not available");
  return 0.0;
}


double Rcs_distancePoint3DConvexPolygon2D(const double pt[3],
                                          const double polygon[][2],
                                          unsigned int N,
                                          const HTr* A,
                                          double cp0[3],
                                          double cp1[3])
{
  RLOG(4, "Rcs_distancePoint3DConvexPolygon2D: WM5 library not available");
  return 0.0;
}

bool Rcs_computePlanePlaneIntersection(const double p1[3],
                                       const double n1[3],
                                       const double p2[3],
                                       const double n2[3],
                                       double origin[3],
                                       double direction[3])
{
  RLOG(4, "Rcs_computePlanePlaneIntersection: WM5 library not available");
  return false;
}

#endif // USE_WM5
