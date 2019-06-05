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

#ifndef RCS_INTERSECTIONWM5_H
#define RCS_INTERSECTIONWM5_H

#include "Rcs_HTr.h"
#include "Rcs_body.h"

#include <stdlib.h>



#ifdef __cplusplus
extern "C" {
#endif


/*! \ingroup DistanceFunctions
 *  \brief Computes the intersection between a line given by a point and a
 *         direction and a box. The function returns true if both intersect,
 *         and stores the closest intersection point in closestLinePt (if it
 *         is not NULL).
 */
bool Rcs_intersectionLineBox(const double linePt[3],
                             const double lineDir[3],
                             const HTr* A_box,
                             const double extentsBox[3],
                             double* closestLinePt);

/*! \ingroup DistanceFunctions
 *  \brief Computes the intersection between a line given by a point and a
 *         direction and a cylinder. The function returns true if both
 *         intersect, and stores the closest intersection point in
 *         closestLinePt (if it is not NULL).
 */
bool Rcs_intersectionLineCylinder(const double linePt[3],
                                  const double lineDir[3],
                                  const HTr* A_cyl,
                                  double height,
                                  double radius,
                                  double* closestLinePt);

/*! \ingroup DistanceFunctions
 *  \brief Computes the intersection between a line given by a point and a
 *         direction and a SSL/capsule. The function returns true if both
 *         intersect, and stores the closest intersection point in
 *         closestLinePt (if it is not NULL).
 */
bool Rcs_intersectionLineSSL(const double linePt[3],
                             const double lineDir[3],
                             const HTr* A_cap,
                             double height,
                             double radius,
                             double* closestLinePt);

/*! \ingroup DistanceFunctions
 *  \brief Computes the intersection between a line given by a point and a
 *         direction and a sphere. The function returns true if both
 *         intersect, and stores the closest intersection point in
 *         closestLinePt (if it is not NULL).
 */
bool Rcs_intersectionLineSphere(const double linePt[3],
                                const double lineDir[3],
                                const HTr* A_sph,
                                double radius,
                                double* closestLinePt);

/*! \ingroup DistanceFunctions
 *  \brief Computes the intersection between a line given by a point and a
 *         direction and a torus. The function returns true if both intersect,
 *         and stores the closest intersection point in closestLinePt (if it
 *         is not NULL).
 */
bool Rcs_intersectionLineTorus(const double linePt[3],
                               const double lineDir[3],
                               const HTr* A_tor,
                               double height,
                               double radius,
                               double* closestLinePt);

/*! \ingroup DistanceFunctions
 *  \brief Computes the intersection between a line given by a point and a
 *         direction and a cone. The function returns true if both
 *         intersect, and stores the closest intersection point in
 *         closestLinePt (if it is not NULL).
 */
bool Rcs_intersectionLineCone(const double linePt[3],
                              const double lineDir[3],
                              const HTr* A_cone,
                              double height,
                              double radius,
                              double* closestLinePt);

/*! \ingroup DistanceFunctions
 *  \brief Computes the intersection between a line given by a point and a
 *         direction and a SSR. The function returns true if both intersect,
 *         and stores the closest intersection point in closestLinePt (if it
 *         is not NULL).
 */
bool Rcs_intersectionLineSSR(const double linePt[3],
                             const double lineDir[3],
                             const HTr* A_ssr,
                             const double extentsSSR[3],
                             double* closestLinePt);

/*! \ingroup DistanceFunctions
 *  \brief Computes the intersection between a plane given by a point and a
 *         direction and a cylinder. The function returns true if both
 *         intersect.
 */
bool Rcs_intersectionPlaneCylinder(const double planePt[3],
                                   const double planeNormal[3],
                                   const double cylinderPt[3],
                                   const double cylinderDir[3],
                                   double radius);

/*! \ingroup DistanceFunctions
 *  \brief Computes a minimum oriented bounding box that encloses the
 *         given points.
 */
bool Rcs_computeOrientedBox(HTr* A_box, double extents[3],
                            const double* points,
                            unsigned int nPoints);


#ifdef __cplusplus
}
#endif



#endif   // RCS_INTERSECTIONWM5_H
