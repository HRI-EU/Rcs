/*******************************************************************************

  Copyright (c) Honda Research Institute Europe GmbH

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

#ifndef RCS_BROADPHASE_H
#define RCS_BROADPHASE_H

#include "Rcs_graph.h"

/*!
* \defgroup RcsBroadphaseFunctions Collision detection broadphase
*
*           The broadphase is a computation step that determines a (narrow
*           phase) collision model from a set of collideable bodies of a
*           graph. While theoretically every object can collide with every
*           other object, in many applications it makes sense to check a set
*           of articulated kinematic trees (e.g. robot arms or legs) against
*           a set of non-articulated obstacles. This is what the RcsBroadPhase
*           is doing. You need to instantiate a broadphase structure
*
*           RcsBroadPhase* bp = RcsBroadPhase_create(graph, distThreshold);
*
*           and add the kinematic chains to be tested against all obstacles:
*
*           RcsBroadPhase_addTreeByName(bp, "treeRoot1");
*           RcsBroadPhase_addTreeByName(bp, "treeRoot2");
*           RcsBroadPhase_addTreeByName(bp, "treeRoot3");
*
*           where treeRoot is the body from which a subtree to be tested
*           descends (e.g. if a robot arm is to be tested, the shoulder link
*           is the one to be passed as treeRoot. It and all its children will
*           be checked).
*
*           A few implementation details:
*
*           The geometric analysis is computed in the function
*           RcsBroadPhase_updateBoundingVolumes() which does the following:
*           For each tree, an enclosing axis-aligned bounding box (AABB) as
*           well as the AABB for each of its bodies is computed. These
*           AABBs constitute to the set A that is going to be checked against
*           a set B: All bodies that have rigid_body_dofs are added to the
*           set B.
*
*           The pruning is done in the function
*           RcsBroadPhase_computeNarrowPhase(bp, cMdl)
*           Here, all tree AABBs are compared against all obstacle AABBs with
*           a separating axes test. In case a separating axis is found, the
*           pair is not considered. In case there is no separating axis, each
*           AABB from the tree bodies is compared against the obstacle. All
*           potentially colliding pairs are added to the passed collision
*           model, which then can be computed in the narrow phase.
*
*/


/*! \ingroup RcsBroadphaseFunctions
 *  This structure models the broadphase geometry of a rigid body. The body
 *  is referred to with its unique id. The struct contains geometry
 *  information about a bounding sphere and an axis-aligned bounding box.
 *  The bounding sphere only needs to be computed once, while the size of
 *  the AABB depends on the orientation of the body and thus needs to be
 *  updated in each time step. Both geometries are computed such that the
 *  bodie's collision geometry is fully contained. For bodies that have
 *  no collideable geometry attached, the flag hasAABB is set to false.
 *  The geometry updates are all computed in the function
 *  RcsBroadPhase_updateBoundingVolumes()
 */
struct _RcsBroadPhaseBdy
{
  int id;
  double sphereRadius;
  double sphereCenter[3];
  // Run-time data
  bool hasAABB;
  double aabbMin[3];
  double aabbMax[3];
};

typedef struct _RcsBroadPhaseBdy RcsBroadPhaseBdy;
/*! \ingroup RcsBroadphaseFunctions
 *  This structure models a kinematic tree of bodies that are considered as
 *  one collideable entity. You could for instance imagine a robot arm to
 *  be such a tree, starting at its root joint, ending at its end effector.
 *  This structure models the root link of such a tree with the startBdyId.
 *  It also maintains an array of RcsBroadPhaseBdy constituing to the tree,
 *  and a bounding box that encloses the whole tree. A tree can be added to
 *  the broadphase with the RcsBroadPhase_addTree() function. This will be
 *  tested against all rigid bodies in the graph. The array bodies will be
 *  recomputed in each iteration, so that structural kinematic changes (e.g.
 *  attaching an object to an end effector) are consistently considered at
 *  run-time.
 */
struct _RcsBroadPhaseTree
{
  int startBdyId;

  // Run-time data
  RcsBroadPhaseBdy* bodies;
  unsigned int nBodies;
  bool hasAABB;
  double aabbMin[3];
  double aabbMax[3];
};

typedef struct _RcsBroadPhaseTree RcsBroadPhaseTree;

/*! \ingroup RcsBroadphaseFunctions
 *  This structure models the broadphase computation step. It contains a set
 *  of RcsBroadPhaseTree trees and a set of RcsBroadPhaseBdy, both with their
 *  AABB representation. These gemoetries will be checked for potential
 *  collisions using the separate axes theorem (SAT). Potentially collideable
 *  bodies can then be added to a narrow phase collision model. This is done
 *  in the function RcsBroadPhase_createNarrowPhase().
 */

struct _RcsBroadPhase
{
  const RcsGraph* graph;
  RcsBroadPhaseTree* trees;
  unsigned int nTrees;
  RcsBroadPhaseBdy* bodies;
  unsigned int nBodies;
  double distanceThreshold;
};

typedef struct _RcsBroadPhase RcsBroadPhase;

#ifdef __cplusplus
extern "C" {
#endif



/*! \ingroup RcsBroadphaseFunctions
 *  \brief Creates a broadphase structure and allocates all internal memory.
 *         The arrays trees and bodies are sized to match all graph bodies.
 *         This is very conservative, so that no reallocations are needed
 *         during run time.
 *
 *  \param graph             Kinematic representation for the broadphase.
 *  \param distanceThreshold Distance threshold to be added to the bounds
 *                           of the enclosing AABBs
 *  \return Pre-allocated and updated broadphase structure.
 */
RcsBroadPhase* RcsBroadPhase_create(const RcsGraph* graph,
                                    double distanceThreshold);

/*! \ingroup RcsBroadphaseFunctions
 *  \brief Clears and deletes all memory
 *
 *  \param bp    Broadphase to be deleted.
 */
void RcsBroadPhase_destroy(RcsBroadPhase* bp);

/*! \ingroup RcsBroadphaseFunctions
 *  \brief Adds a RcsBroadPhaseTree to the broadphase
 *
 *  \param bp    Broadphase to be extended with tree.
 *  \param bdyId Root body of kinematic tree to be added
 *  \return true for success, false otherwise (e.g. invalid body id). A log
 *          message will be written to the console on debug level 1 in case
 *          of failure.
 */
bool RcsBroadPhase_addTree(RcsBroadPhase* bp, int bdyId);

/*! \ingroup RcsBroadphaseFunctions
 *  \brief Same as RcsBroadPhase_addTree with body name as argument
 *
 *  \param bp      Broadphase to be extended with tree.
 *  \param bdyName Root body name of kinematic tree to be added
 *  \return true for success, false otherwise (e.g. body not found). A log
 *          message will be written to the console on debug level 1 in case
 *          of failure.
 */
bool RcsBroadPhase_addTreeByName(RcsBroadPhase* bp, const char* bdyName);

/*! \ingroup RcsBroadphaseFunctions
 *  \brief Computes the bounding shapes of all trees and all bodies, and stores
 *         the results in the respective structures.
 *
 *  \param bp    Broadphase to be updated.
 */
void RcsBroadPhase_updateBoundingVolumes(RcsBroadPhase* bp);

/*! \ingroup RcsBroadphaseFunctions
 *  \brief Compares all trees against all rigid bodies using a SAT test. All
 *         object pairs that potentially collide are added to the narrow
 *         phase collision model cMdl. If a tree is found to collide against
 *         an object, all its bodies are subsequently tested against the body,
 *         which is kind of a mini-broadphase in itself.
 *
 *  \param bp      Broadphase to be considered.
 *  \param cMdl    Narrow phase collsion model to be updated.
 *  \return Number of potentially collideable pairs for statistics.
 */
int RcsBroadPhase_computeNarrowPhase(const RcsBroadPhase* bp,
                                     RcsCollisionMdl* cMdl);

/*! \ingroup RcsBroadphaseFunctions
 *  \brief Sames a RcsBroadPhase_computeNarrowPhase, except for not updating
 *         but creating a narrow phase collsion model.
 *
 *  \param bp      Broadphase to be considered.
 *  \return Narrow phase collsion model corresponding to bp.
 */
RcsCollisionMdl* RcsBroadPhase_createNarrowPhase(const RcsBroadPhase* bp);


#ifdef __cplusplus
}
#endif

#endif   // RCS_BROADPHASE_H
