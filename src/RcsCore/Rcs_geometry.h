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

#ifndef RCS_GEOMETRY_H
#define RCS_GEOMETRY_H


#ifdef __cplusplus
extern "C" {
#endif


#include "Rcs_HTr.h"


/*!
 * \defgroup RcsBasicMathFunctions Basic math functions
 */






/**
 * @name Distance2D
 *
 * Distance and polygon functions in 2D
 */

///@{

/*! \ingroup RcsBasicMathFunctions
 *  \brief Returns the distance and closest points between a point and a convex
 *         polygon in 2D.
 *  \param[in]  point      Query Point
 *  \param[in]  polygon    Polygon vertices in ordered counter-clockwise. The
 *                         polygon is assumed to be opened, the last vertex
 *                         will be considered to be connected to the first one.
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
                                     double polygon[][2],
                                     unsigned int nVertices,
                                     double cpPoly[2],
                                     double nPoly[2]);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Checks if a given 2D polygon is valid. The function checks if the
 *         polygon centroid lies within the edges spanned by the polygon
 *         vertices. If that's not the case, the ordering of the vertices is
 *         not counter-clockwise.
 *
 *  \param[in]  polygon    Polygon vertices
 *  \param[in]  nVertices  Number of polygon vertices.
 *  \return True if valid, false otherwise.
 */
bool Math_checkPolygon2D(double polygon[][2], unsigned int nVertices);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Returns the arc length of a polygon. The function walks through the
 *         (ordered) vertices and adds their segment lengths. It is assumed that
 *         vertices are sorted according to their neighborhood. The last polygon
 *         vertex will be considered to be connected to the first one.
 *
 *  \param[in]  polygon    Polygon vertices
 *  \param[in]  nVertices  Number of polygon vertices.
 *  \return Length of the polygon curve.
 */
double Math_lengthPolygon2D(double polygon[][2], unsigned int nVertices);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Continuous interpolation of a polygon. The variable s defines the
 *         interolation value. It is assumed to be 0 <= s <= 1. A value of
 *         0 gets the first vertex, a value of 1 as well. A value of 0.5 gets
 *         the polygon coordinate at 0.5 of the polygon lenght.
 *
 *  \param[out] res        Interpolation point on polygon curve.
 *  \param[in]  polygon    Polygon vertices
 *  \param[in]  nVertices  Number of polygon vertices.
 *  \param[in]  s          Interpolation value. The function internally clips
 *                         it to a range of [0...1].
 */
void Math_interpolatePolygon2D(double res[2], double polygon[][2],
                               unsigned int nVertices, double s);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Continuous interpolation of a polygon. The variable s defines the
 *         interolation value. It is assumed to be 0 <= s <= 1. A value of
 *         0 gets the first vertex, a value of 1 as well. A value of 0.5 gets
 *         the polygon coordinate at 0.5 of the polygon lenght.
 *
 *  \param[out] polyOut  Re-sampled polygon
 *  \param[in]  nvOut    Number of polygon vertices of the resampled polygon
 *  \param[in]  polyIn   Polygon vertices of original polygon
 *  \param[in]  nvIn     Number of polygon vertices of the incoming polygon
 */
void Math_resamplePolygon2D(double polyOut[][2], unsigned int nvOut,
                            double polyIn[][2], unsigned int nvIn);

/*! \ingroup RcsBasicMathFunctions
 *  \brief This function computes the intersection between a ray and a line
 *         segment in 2 dimensions.
 *
 *  \param[in]  rayOrigin Start point of ray
 *  \param[in]  rayDir    Ray direction vector. It must be of unit lengh,
 *                        otherwise the result is undefined.
 *  \param[in]  segPt0    First point of line segment
 *  \param[in]  segPt1    Second point of line segment
 *  \param[out] intersectPt Point of intersection. If it is NULL, it is ignored.
 *                          If the result is not of type 1, 2 or 3, the
 *                          argument intersectPt remains unchanged.
 *  \return Result type:
 *          - 0: no intersection
 *          - 1: intersection on the line segment
 *          - 2: intersection with first vertex point
 *          - 3: intersection with second vertex point
 *          - 4: co-linear line segment and ray (infinity intersections)
 */
int Math_intersectRayLineseg2D(const double rayOrigin[2],
                               const double rayDir[2],
                               const double segPt0[2],
                               const double segPt1[2],
                               double intersectPt[2]);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Computes the number of polygon intersections of a ray starting out
 *         from the query point with some random direction. In case the number
 *         of polygon intersections is even, the point is outside the polygon.
 *         In case it is odd, it is inside. This method is valid for any kind
 *         of 2d polygon, also non-convex ones. It is known as ray-casting
 *         algorithm (https://en.wikipedia.org/wiki/Point_in_polygon).
 *
 *  \param[in] pt        Query point to be checked
 *  \param[in] polygon   Polygon vertices. The function can deal with vertex
 *                       arrays where the last vertex is the same as the first.
 *  \param[in] nVertices Number of polygon vertices of the polygon. Must be >=1.
 *  \return Even number for outside, odd number for inside, -1 for no solution.
 *          In the unlikely case of -1, the polygon is probably degenerated.
 */
int Math_pointInsideOrOnPolygon2D(const double pt[2],
                                  double polygon[][2],
                                  unsigned int nVertices);

/*! \ingroup RcsBasicMathFunctions
 *  \brief Computes the distance between two polygon vertices along the
 *         polygon's outline. The distance will be calculated along increasing
 *         indices. If the index idx2 is less than idx1, the computation will
 *         correctly wrap around the initial vertex. The function will exit
 *         fatally if idx1 aor idx2 are not less than nVertices.
 *
 *  \param[out] polygon    Polygon vertices.
 *  \param[in]  nVertices  Number of polygon vertices
 *  \param[in]  idx1       Start vertex
 *  \param[in]  idx2       End vertex
 *  \return Distance between the two vertices.
 */
double Math_polyVertexDistance(double polygon[][2],
                               unsigned int nVertices,
                               unsigned int idx1,
                               unsigned int idx2);

///@}




/**
 * @name Distance3D
 *
 * Distance functions in 3D
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
  *  \brief This function returns the signed distance between a point and a
  *         plane, and computes the closest plane point.
  *
  *  \param[in]  pt Point coordinates
  *  \param[in]  planePt Arbitrary point on the plane
  *  \param[in]  planeNormal Normal vector of the plane
  *  \param[out] cpPlane Closest point on the plane. If it is NULL,
  *              it will be ignored.
  *  \return Signed distance of the point to the plane according to the plane
  *          normal.
  */
double Math_distPointPlane(const double pt[3],
                           const double planePt[3],
                           const double planeNormal[3],
                           double cpPlane[3]);

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
 *         polygon in 3D. The polygon is assumed to be in the x-y plane of the
 *         frame A_PI. Its z-direction is the polygon normal. The polygon
 *         is assumed to be "filled".
 *
 *  \param[in]  I_pt        Query Point in world coordinates
 *  \param[in]  A_PI        Transformation from world to polygon frame
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
                                      double polygon[][2],
                                      unsigned int nVertices,
                                      double I_cpPoly[3],
                                      double I_nPoly[3]);

/*! \ingroup RcsBasicMathFunctions
 *  \brief This function returns the squared distance between two line
 *         segments, and computes the closest points.
 *
 *  Adapted from: Wildmagic library (version 5.8)
 *  Geometric Tools LLC, Redmond WA 98052
 *  Copyright (c) 1998-2015
 *  Distributed under the Boost Software License, Version 1.0.
 *  http://www.boost.org/LICENSE_1_0.txt
 *  http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
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

/*! \ingroup RcsBasicMathFunctions
 *  \brief This function returns the distance between a point and a cone,
 *         and computes the closest points. The distance is negative when the
 *         point penetrates the cone.
 *
 *  \param[in]  point Arbitrary 3d point
 *  \param[in]  A_cone Transformation of the cone
 *  \param[in]  height Cone height
 *  \param[in]  radius Cone radius
 *  \param[out] cpCone Closest point on the cone. If it is NULL, it will be
 *              ignored.
 *  \return Signed distance of closest points.
 */
double Math_distPointCone(const double point[3],
                          const HTr* A_cone,
                          double height,
                          double radius,
                          double cpCone[3]);

/*! \ingroup RcsBasicMathFunctions
 *  \brief This function returns the distance between a point and a cylinder,
 *         and computes the closest points. The distance is negative when the
 *         point penetrates the cylinder.
 *
 *  \param[in]  point Arbitrary 3d point
 *  \param[in]  A_cyl Transformation of the cylinder
 *  \param[in]  height Cylinder height
 *  \param[in]  radius Cylinder radius
 *  \param[out] cpCyl Closest point on the cylinder. If it is NULL, it will
 *              be ignored.
 *  \return Signed distance of closest points.
 */
double Math_distPointCylinder(const double point[3],
                              const HTr* A_cyl,
                              double height,
                              double radius,
                              double cpCyl[3]);

/*! \ingroup RcsBasicMathFunctions
 *  \brief This function returns the distance between a point and a box, and
 *         computes the closest points. The distance is negative when the
 *         point penetrates the box.
 *
 *  \param[in]  point Arbitrary 3d point
 *  \param[in]  A_box Transformation of the box
 *  \param[in]  extents Side lengths of the box.
 *  \param[out] cpBox Closest point on the box. If it is NULL, it will be
 *              ignored.
 *  \param[out] nBox Unit length normal from the point to the box. If the
 *              closest points coincide, the vector normal is set to zero. If
 *              it is NULL, it will be ignored.
 *  \return Signed distance of closest points.
 */
double Math_distPointBox(const double point[3],
                         const HTr* A_box,
                         const double extents[3],
                         double cpBox[3],
                         double nBox[3]);


///@}






#ifdef __cplusplus
}
#endif

#endif   // RCS_GEOMETRY_H
