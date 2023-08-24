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

#ifndef RCS_COLLISIONMODEL_H
#define RCS_COLLISIONMODEL_H

#include "Rcs_graph.h"

#include <libxml/tree.h>


#ifdef __cplusplus
extern "C" {
#endif


/*!
 * \defgroup RcsCollisionMdlFunctions Collision Model
 *
 *           Collision model. The results of the computation are stored in
 *           the arrays cp and n1. Here is how they are organized:
 *           Closest point array cp holds the closest points of the pairs in
 *           alternating order as below. The array is [nBodies x 3]
 *
 *           <br>[ x y z ]   of pair 0 body 0
 *           <br>[ x y z ]   of pair 0 body 1
 *           <br>[   .   ]
 *           <br>[   .   ]
 *           <br>[   .   ]
 *           <br>[ x y z ]   of pair n-1 body 0
 *           <br>[ x y z ]   of pair n-1 body 1
 *
 *           Normal array cp holds the normalized surface normal of the first
 *           body of each pairas below. The array is [nPairs x 3]
 *
 *           [ x y z ]   of pair 0 body 0
 *           <br>[ x y z ]   of pair 1 body 0
 *           <br>[   .   ]
 *           <br>[   .   ]
 *           <br>[   .   ]
 *           <br>[ x y z ]   of pair n-1 body 0
 *
 */


/*! \ingroup RcsCollisionMdlFunctions
 *  \brief Creates an empty collision model.
 */
RcsCollisionMdl* RcsCollisionModel_create(const RcsGraph* graph);

/*! \ingroup RcsCollisionMdlFunctions
 *  \brief Creates a collision model from a XML node pointer.
 */
RcsCollisionMdl* RcsCollisionModel_createFromXML(const RcsGraph* graph,
                                                 xmlNodePtr node);

/*! \ingroup RcsCollisionMdlFunctions
 *  \brief Computes all proximities of the collision model and copies the
 *         results to the arrays cp and n1.
 */
void RcsCollisionModel_compute(RcsCollisionMdl* self);

/*! \ingroup RcsCollisionMdlFunctions
 *  \brief Prints collision model to a file descriptor.
 */
void RcsCollisionModel_fprint(FILE* fd, const RcsCollisionMdl* self);

/*! \ingroup RcsCollisionMdlFunctions
 *  \brief Frees all memory allocated bythe collision model.
 */
void RcsCollisionModel_destroy(RcsCollisionMdl* self);

/*! \ingroup RcsCollisionMdlFunctions
 *  \brief Makes a deep copy of the collision model. If self is NULL, the
 *         function returns NULL.
 */
RcsCollisionMdl* RcsCollisionModel_clone(const RcsCollisionMdl* self,
                                         const RcsGraph* newGraph);

/*! \ingroup RcsCollisionMdlFunctions
 *  \brief Appends another collision model.
 */
bool RcsCollisionModel_append(RcsCollisionMdl* self,
                              const RcsCollisionMdl* other,
                              const char* suffix);

/*! \ingroup RcsCollisionMdlFunctions
 *  \brief Returns the minimum distance inside the collision model. If self
 *         is NULL, or the model contains no pairs, DBL_MAX will be returned.
 */
double RcsCollisionMdl_getMinDist(const RcsCollisionMdl* self);

/*! \ingroup RcsCollisionMdlFunctions
 *  \brief Same as \ref RcsCollisionMdl_getMinDist(), but copies the pairs
 *         index into pairIdx. In case of failute (empty collision model,
 *         self or self->pair is NULL), the function returns the largest
 *         possible double value (DBL_MAX), and leaves pairIdx unchanged.
 */
double RcsCollisionMdl_getMinDistPair(const RcsCollisionMdl* self,
                                      int* pairIdx);

/*! \ingroup RcsCollisionMdlFunctions
 *  \brief Returns the collision cost, summed over all pairs. This function
 *         reads the values of distances etc. from the data structure. In
 *         order to be up to date, the RcsCollisionMdl_compute() function
 *         has to be called before.
 */
double RcsCollisionMdl_cost(const RcsCollisionMdl* self);

/*! \ingroup RcsCollisionMdlFunctions
 *  \brief Returns the collision gradient, summed over all pairs. In
 *         order to be up to date, the RcsCollisionMdl_compute() function
 *         has to be called before.
 */
void RcsCollisionMdl_gradient(const RcsCollisionMdl* self, MatNd* grad);

/*! \ingroup RcsCollisionMdlFunctions
 *  \brief Returns true if all parameters of the collision models are the
 *         same. The underlying graph may be different. Value eps is the
 *         permissable error for values, such as distances or closest
 *         point coordinates.
 */
bool RcsCollisionMdl_isEqual(const RcsCollisionMdl* self,
                             const RcsCollisionMdl* other,
                             double eps);

/*! \ingroup RcsCollisionMdlFunctions
 *  \brief Prints information of all pairs that have a distance of equal or
 *         less than argument distanceThreshold. All pointer arguments
 *         may be NULL.
 */
void RcsCollisionModel_fprintCollisions(FILE* fd, const RcsCollisionMdl* self,
                                        double distanceThreshold);

/*! \ingroup RcsCollisionMdlFunctions
 *  \brief Prints the collision model xml representation to the given file
 *         descriptor. The arguments are assumed to be not NULL.
 *
 *  \param[in] out     File to write to.
 *  \param[in] self    Collision model to write.
 *
 *  \return Number of errors encountered. Errors are reported on debug level 1.
 */
int RcsCollisionModel_fprintXML(FILE* out, const RcsCollisionMdl* self);

#ifdef __cplusplus
}
#endif

#endif   // RCS_COLLISIONMODEL_H
