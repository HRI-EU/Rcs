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

#ifndef RCS_BULLETDISTANCE_H
#define RCS_BULLETDISTANCE_H

#include <Rcs_graph.h>


#ifdef __cplusplus
extern "C" {
#endif

bool testBulletDistance(size_t iterations);
bool testBulletDistanceThreaded(size_t nThreads, size_t iterations);

/*! \brief Returns the distance of s1 and s2. The closest points (in world
 *         coordinates) will be copied to I_cp1 and I_cp2. The unit normal
 *         vector (in world coordinates) of s1 (pointing away from the
 *         surface) is copied to I_n12. The following shapes can be computed:
 *         - Point
 *         - SSL
 *         - Cone
 *         - Sphere
 *         - Cylinder
 *         - Box
 *         - SSR
 *         - Mesh (convex only)
 *
 *  \param[in]  s1        First shape
 *  \param[in]  s2        Second shape
 *  \param[in]  A_S1I     Transformation from world (I) to the first shape's
 *                        frame. The shape frame might have an additional
 *                        relative transform that is represented in
 *                        RcsShape::A_CB. The caller is respnsible to compute
 *                        the correct transform.
 *  \param[in]  A_S2I     Transformation from world (I) to the secon shape's
 *                        frame. The shape frame might have an additional
 *                        relative transform that is represented in
 *                        RcsShape::A_CB. The caller is respnsible to compute
 *                        the correct transform.
 *  \param[in]  I_cp1     Closest point on shape 1 in world coordinates. Must
 *                        not be NULL.
 *  \param[out] I_cp2     Closest point on shape 2 in world coordinates. Must
 *                        not be NULL.
 *  \param[out] I_n12     Unit normal vector from shape1 towards shape2. Must
 *                        not be NULL.
 */
double RcsShape_distanceBullet(const RcsShape* s1, const RcsShape* s2,
                               const HTr* A_S1I, const HTr* A_S2I,
                               double I_cp1[3], double I_cp2[3],
                               double I_n12[3]);

#ifdef __cplusplus
}
#endif


#endif // RCS_BULLETDISTANCE_H
