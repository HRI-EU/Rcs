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

#ifndef RCS_OCTREE_H
#define RCS_OCTREE_H

#include "Rcs_graph.h"


#ifdef __cplusplus
extern "C" {
#endif

/*!
 * \defgroup RcsOctreeFunctions Octree distance functions
 *
 */

/*! \ingroup RcsOctreeFunctions
 *  \brief Traverses all shapes of the graph. If a shape is of the type
 *         RCSSHAPE_OCTREE, the function checks for the mesh pointer. If
 *         one or more of them is NULL, the function returns false. If all
 *         octree shapes have a non-NULL mesh pointer, the function
 *         returns true.
 */
bool RcsGraph_checkOctreeConsistency(const RcsGraph* graph);

/*! \ingroup RcsOctreeFunctions
 *  \brief Creates an Octomap class instance and returns the pointer to it,
 *         cast to a void pointer. The function internally creates an
 *         instance of the octomap class. Please check the documentation for
 *         more information..
 */
void* Rcs_loadOctree(const char* fileName);

/*! \ingroup RcsOctreeFunctions
 * \brief Destroys an Octomap class instance from a given void pointer
 * \param octomap Pointer to Octomap class instance
 */
void Rcs_destroyOctree(void* octomap);

/*! \ingroup RcsOctreeFunctions
 *  \brief Calculates the bounding box of the octree.
 */
void Rcs_getOctreeBoundingBox(void* octomap, double xyz_min[3],
                              double xyz_max[3]);

/*! \ingroup RcsOctreeFunctions
 *  \brief Calculates the volume of the octree.
 */
double Rcs_getOctreeVolume(void* octomap);


#ifdef __cplusplus
}
#endif
#endif // RCS_OCTREE_H
