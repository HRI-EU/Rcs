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

#ifndef RCS_DISTANCEWM5_H
#define RCS_DISTANCEWM5_H

#include "Rcs_HTr.h"

#include <stdlib.h>



#ifdef __cplusplus
extern "C" {
#endif

/*!
 * \defgroup DistanceFunctions Distance functions
 */



/*!
 * \brief Returns true if the query point lies within the convex polygon (2D)
 * \param pt Query point
 * \param polygon Polygon vertices in ordered counter-clockwise
 * \param N Number of polygon vertices
 * \return
 */
bool Rcs_containedPoint2DConvexPolygon2D(const double pt[2],
                                         const double polygon[][2],
                                         unsigned int N);

/*!
 * \ingroup DistanceFunctions
 * \brief Returns the distance and closest points between a point and a convex
 *        polygon
 * \param pt Query Point
 * \param polygon Polygon vertices in ordered counter-clockwise
 * \param N Number of polygon vertices
 * \param cp0 Will be set to pt (can be NULL)
 * \param cp1 Set to closest point, if query point is outside the polygon
 *            (can be NULL)
 * \return
 *
 * Distance is positive if point is outside the polygon and negative if
 * inside.
 */
double Rcs_distancePoint2DConvexPolygon2D(const double pt[2],
                                          const double polygon[][2],
                                          unsigned int N,
                                          double cp0[2],
                                          double cp1[2]);


/*!
 * \ingroup DistanceFunctions
 * \brief Returns the distance between a 3D point and a 2D convex polygon
 * \param pt Query point (3D)
 * \param polygon Polygon vertices in ordered counter-clockwise (2D)
 * \param N Number of polygon vertices
 * \param A Reference frame of the polygon
 * \param cp0 Will be set to pt (can be NULL)
 * \param cp1 Set to closest point, if query point is outside the polygon
 *            (can be NULL)
 * \return
 */
double Rcs_distancePoint3DConvexPolygon2D(const double pt[3],
                                          const double polygon[][2],
                                          unsigned int N,
                                          const HTr* A,
                                          double cp0[3],
                                          double cp1[3]);

/*! \ingroup DistanceFunctions
 *  \brief Computes the intersection between two planes. The planes are
 *         defined by a plane point p and a plane normal n. The function
 *         returns false if no intersection line could be found. In this
 *         case, origin and direction are unchanged. If an intersection line
 *         was found, vector origin will be a point on the intersection line,
 *         and direction will be a normalized direction vector.
 */
bool Rcs_computePlanePlaneIntersection(const double p1[3],
                                       const double n1[3],
                                       const double p2[3],
                                       const double n2[3],
                                       double origin[3],
                                       double direction[3]);

/*! \ingroup DistanceFunctions
 *  \brief Returns true if the Wildmagic 5 library has been compiled in, and
 *         the above functions have an implementation. Returns false otherwise.
 */
bool Rcs_hasWM5();


#ifdef __cplusplus
}
#endif



#endif   // RCS_DISTANCEWM5_H
