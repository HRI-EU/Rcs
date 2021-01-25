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
 *  \brief Given a RcsCollisionMdl data structure and two bodies b1 and b2,
 *         this function sets the pointers cp1, cp2, n1 and d to point to the
 *         memory within the collision model. If the body combination b1 - b2
 *         does not exist (the function also checks for b2 - b1), the
 *         pointers remain unchanged and false is returned. In case of
 *         success, true is returned.
 *
 *         Here's an example:
\verbatim
           double *cpR=NULL, *cpE=NULL, *n=NULL, *d=NULL;
           bool success =
             RcsCollisionMdl_getPointers(cMdl, refBdy, &cpR,
                                         effector, &cpE, &n, &d);
           RCHECK(success);
           RMSG("cpR = [%g %g %g]", cpR[0], cpR[1], cpR[2]);
           RMSG("cpE = [%g %g %g]", cpE[0], cpE[1], cpE[2]);
           RMSG("n   = [%g %g %g]", n[0], n[1], n[2]);
           RMSG("distance = %g", *d);
\endverbatim
 */
bool RcsCollisionMdl_getPointers(const RcsCollisionMdl* self,
                                 const RcsBody* b1, const double** cp1,
                                 const RcsBody* b2, const double** cp2,
                                 const double** n1, const double** d);

/*! \ingroup RcsCollisionMdlFunctions
 *  \brief Returns the minimum distance inside the collision model. If self
 *         is NULL, DBL_MAX will be returned.
 */
double RcsCollisionMdl_getMinDist(const RcsCollisionMdl* self);

/*! \ingroup RcsCollisionMdlFunctions
 *  \brief Same as \ref RcsCollisionMdl_getMinDist(), but copies the pairs
 *         index into pairIdx.
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
 *  \brief Removes the collision pair from the model, and updates all
 *         indices and pointers. If the pair is not found, the function
 *         returns false. The pair will be deleted. if the collision
 *         model has no pairs, the function returns false.
 */
bool RcsCollisionMdl_removePair(RcsCollisionMdl* self,
                                const RcsPair* pair);

/*! \ingroup RcsCollisionMdlFunctions
 *  \brief Sets the weight value of the given body pair to the given value.
 *         The bodies do not need to be in the same order as in the pair. If
 *         several pairs with the same body combinations exist in the model,
 *         the first one is used. If no pair is found, the function returns
 *         false, true otherwise. If one or both bodies don't exist in the
 *         graph of the collision model, the function returns false.
 */
bool RcsCollisionMdl_setPairWeightByName(RcsCollisionMdl* self,
                                         const char* bdy1, const char* bdy2,
                                         double weight);

/*! \ingroup RcsCollisionMdlFunctions
 *  \brief Prints the collision model to the console. If self or fd is NULL,
 *         the function issues a warning on debug level 1 and returns.
 */
//void RcsPair_printCollisionModel(FILE* fd, RcsPair** pPtr);

/*! \ingroup RcsCollisionMdlFunctions
 *  \brief Prints information of all pairs that have a distance of equal or
 *         less than argument distanceThreshold. All pointer arguments
 *         may be NULL.
 */
void RcsCollisionModel_fprintCollisions(FILE* fd, const RcsCollisionMdl* self,
                                        double distanceThreshold);

/*! \ingroup RcsCollisionMdlFunctions
 *  \brief Prints the joint data to a file descriptor. All arguments may
 *         be NULL.
 */
//void RcsPair_fprint(FILE* out, const RcsPair* self, const RcsCollisionMdl* cmdl);

/*! \ingroup RcsCollisionMdlFunctions
 *  \brief Returns true if all parameters of the pairss are the
 *         same. The underlying graph may be different. Value eps is the
 *         permissable error for values, such as distances or closest
 *         point coordinates.
 */
bool RcsPair_isEqual(const RcsPair* p1, const RcsPair* p2,
                     const RcsGraph* g1, const RcsGraph* g2, double eps);


#ifdef __cplusplus
}
#endif

#endif   // RCS_COLLISIONMODEL_H
