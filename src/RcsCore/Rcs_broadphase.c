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

#include "Rcs_broadphase.h"
#include "Rcs_typedef.h"
#include "Rcs_body.h"
#include "Rcs_shape.h"
#include "Rcs_collisionModel.h"
#include "Rcs_Vec3d.h"
#include "Rcs_macros.h"

#include <float.h>


/*******************************************************************************
 * returns true if no collision is possible.
 ******************************************************************************/
static bool RcsBroadPhaseBdy_SAT(const RcsBroadPhase* bp, int bpBodyId,
                                 const double xyzMin[3], const double xyzMax[3])
{
  RcsBroadPhaseBdy* bdy = &bp->bodies[bpBodyId];

#if 0
  const double bdyR = bdy->sphereRadius;
  const double* bdyPos = bp->graph->bodies[bdy->id].A_BI.org;

  if ((bdyPos[0] - bdyR > xyzMax[0]) || (bdyPos[0] + bdyR < xyzMin[0]))
  {
    return true;
  }

  if ((bdyPos[1] - bdyR > xyzMax[1]) || (bdyPos[1] + bdyR < xyzMin[1]))
  {
    return true;
  }
  if ((bdyPos[2] - bdyR > xyzMax[2]) || (bdyPos[2] + bdyR < xyzMin[2]))
  {
    return true;
  }
#else

  if ((bdy->aabbMin[0] > xyzMax[0]) || (bdy->aabbMax[0] < xyzMin[0]))
  {
    return true;
  }
  if ((bdy->aabbMin[1] > xyzMax[1]) || (bdy->aabbMax[1] < xyzMin[1]))
  {
    return true;
  }
  if ((bdy->aabbMin[2] > xyzMax[2]) || (bdy->aabbMax[2] < xyzMin[2]))
  {
    return true;
  }

#endif

  return false;
}

/*******************************************************************************
 *
 ******************************************************************************/
RcsBroadPhase* RcsBroadPhase_create(const RcsGraph* graph,
                                    double distanceThreshold)
{
  RcsBroadPhase* bp = RALLOC(RcsBroadPhase);
  bp->graph = graph;
  bp->trees = RNALLOC(bp->graph->nBodies, RcsBroadPhaseTree);
  bp->bodies = RNALLOC(bp->graph->nBodies, RcsBroadPhaseBdy);
  bp->distanceThreshold = distanceThreshold;
  RcsBroadPhase_updateBoundingVolumes(bp);
  return bp;
}

/*******************************************************************************
 *
 ******************************************************************************/
void RcsBroadPhase_destroy(RcsBroadPhase* bp)
{
  RFREE(bp->bodies);
  RFREE(bp->trees->bodies);
  RFREE(bp->trees);
  RFREE(bp);
}

/*******************************************************************************
 *
 ******************************************************************************/
bool RcsBroadPhase_addTree(RcsBroadPhase* bp, int bdyId)
{
  if (bdyId==-1)
  {
    RLOG(1, "Can't add chain with body id -1");
    return false;
  }

  if (bp->nTrees >= bp->graph->nBodies)
  {
    RLOG(1, "Max number of trees (%d >= %d) exceeded",
         bp->nTrees, bp->graph->nBodies);
    return false;
  }

  memset(&bp->trees[bp->nTrees], 0, sizeof(RcsBroadPhaseTree));
  bp->trees[bp->nTrees].startBdyId = bdyId;
  bp->trees[bp->nTrees].bodies = RNALLOC(bp->graph->nBodies, RcsBroadPhaseBdy);
  bp->nTrees++;

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool RcsBroadPhase_addTreeByName(RcsBroadPhase* bp, const char* bdyName)
{
  const RcsBody* bdy = RcsGraph_getBodyByName(bp->graph, bdyName);

  if (!bdy)
  {
    RLOG(1, "Body \"%s\" not found in broad phase graph",
         bdyName ? bdyName : "NULL");
    return false;
  }

  return RcsBroadPhase_addTree(bp, bdy->id);
}

/*******************************************************************************
 *
 ******************************************************************************/
void RcsBroadPhase_updateBoundingVolumes(RcsBroadPhase* bp)
{
  // Compute AABB's of all rigid bodies. We do this completely from scratch so
  // that potential changes in the body parameters are reflected in each step.
  bp->nBodies = 0;
  for (unsigned int i = 0; i < bp->graph->nBodies; ++i)
  {
    const RcsBody* bdy_i = &bp->graph->bodies[i];

    if ((!bdy_i->rigid_body_joints) || (bdy_i->id == -1) ||
        (RcsBody_numDistanceShapes(bdy_i) == 0))
    {
      continue;
    }

    double sMin[3], sMax[3];
    bool hasAABB = RcsGraph_computeBodyAABB(bp->graph, bdy_i->id,
                                            RCSSHAPE_COMPUTE_DISTANCE,
                                            sMin, sMax, NULL);

    if (!hasAABB)
    {
      continue;
    }

    RcsBroadPhaseBdy* bb = &bp->bodies[bp->nBodies];
    bb->id = bdy_i->id;
    bb->sphereRadius = 0.5 * Vec3d_distance(bb->aabbMin, bb->aabbMax);
    Vec3d_add(bb->sphereCenter, bb->aabbMin, bb->aabbMax);
    Vec3d_constMulSelf(bb->sphereCenter, 0.5);
    bb->hasAABB = hasAABB;
    Vec3d_constAddSelf(sMin, -bp->distanceThreshold);
    Vec3d_constAddSelf(sMax,  bp->distanceThreshold);
    Vec3d_copy(bb->aabbMin, sMin);
    Vec3d_copy(bb->aabbMax, sMax);
    bp->nBodies++;
  }

  // Compute all chains
  for (unsigned int c = 0; c < bp->nTrees; ++c)
  {
    RcsBroadPhaseTree* tree = &bp->trees[c];
    tree->hasAABB = false;
    double xyzMin[3], xyzMax[3];
    Vec3d_set(xyzMin, DBL_MAX, DBL_MAX, DBL_MAX);
    Vec3d_set(xyzMax, -DBL_MAX, -DBL_MAX, -DBL_MAX);
    tree->nBodies = 0;

    RCSBODY_TRAVERSE_BODIES(bp->graph, &bp->graph->bodies[tree->startBdyId])
    {
      double sMin[3], sMax[3];
      bool hasAABB = RcsGraph_computeBodyAABB(bp->graph, BODY->id,
                                              RCSSHAPE_COMPUTE_DISTANCE,
                                              sMin, sMax, NULL);

      if (hasAABB)
      {
        tree->hasAABB = true;
        Vec3d_constAddSelf(sMin, -bp->distanceThreshold);
        Vec3d_constAddSelf(sMax,  bp->distanceThreshold);

        for (int i = 0; i < 3; ++i)
        {
          xyzMin[i] = fmin(sMin[i], xyzMin[i]);
          xyzMax[i] = fmax(sMax[i], xyzMax[i]);
        }
        RcsBroadPhaseBdy* treeBdy = &tree->bodies[tree->nBodies];
        treeBdy->id = BODY->id;
        treeBdy->sphereRadius = 0.5 * Vec3d_distance(sMin, sMax);
        Vec3d_add(treeBdy->sphereCenter, sMin, sMax);
        Vec3d_constMulSelf(treeBdy->sphereCenter, 0.5);
        Vec3d_copy(treeBdy->aabbMin, sMin);
        Vec3d_copy(treeBdy->aabbMax, sMax);
        tree->nBodies++;
      }

    }

    if (tree->hasAABB)
    {
      Vec3d_copy(tree->aabbMin, xyzMin);
      Vec3d_copy(tree->aabbMax, xyzMax);
    }
  }   // for (unsigned int c = 0; c < bp->nTrees; ++c)

}

/*******************************************************************************
 *
 ******************************************************************************/
int RcsBroadPhase_computeNarrowPhase(const RcsBroadPhase* bp,
                                     RcsCollisionMdl* cMdl)
{
  // For statistics: nNaiive is the worst case combination of all tree bodies
  // against all rigid bodies.
  int nNaiive = 0;

  // Memorize number of model pairs to determine if reallocation is needed
  const unsigned int nPairs = cMdl->nPairs;
  cMdl->nPairs = 0;

  // Perform broadphase calculation using SAT
  for (unsigned int c = 0; c < bp->nTrees; ++c)
  {
    if (!bp->trees[c].hasAABB)
    {
      continue;
    }

    const RcsBroadPhaseTree* tree = &bp->trees[c];
    nNaiive += tree->nBodies * bp->nBodies;

    for (unsigned int i = 0; i < bp->nBodies; ++i)
    {
      // Perform separating axes test. If an axis is overlapping, the
      // pair will be added to the narrow phase.
      if (RcsBroadPhaseBdy_SAT(bp, i, tree->aabbMin, tree->aabbMax))
      {
        RLOG(1, "Found SAT in tree: skipping %s - %s",
             bp->graph->bodies[bp->bodies[i].id].name,
             bp->graph->bodies[tree->startBdyId].name);
        continue;
      }

      // Add all combinations of the tree to the model
      for (unsigned int j=0; j<tree->nBodies; ++j)
      {
        // Skip these that don't collide
        if (RcsBroadPhaseBdy_SAT(bp, i, tree->bodies[j].aabbMin,
                                 tree->bodies[j].aabbMax))
        {
          RLOG(1, "Found SAT in subtree: skipping %s - %s",
               bp->graph->bodies[bp->bodies[i].id].name,
               bp->graph->bodies[tree->bodies[j].id].name);
          continue;
        }

        // We only grow and never shring the narrow phase collision model to
        // minimize the number of memory reallocations.
        if (cMdl->nPairs>=nPairs)
        {
          cMdl->pair = RREALLOC(cMdl->pair, cMdl->nPairs+1, RcsPair);
          RCHECK_MSG(cMdl->pair, "Failed to allocate memory for %d pairs",
                     cMdl->nPairs + 1);
        }
        RcsPair* lastPair = &cMdl->pair[cMdl->nPairs];
        memset(lastPair, 0, sizeof(RcsPair));
        lastPair->b1 = tree->bodies[j].id;
        lastPair->b2 = bp->bodies[i].id;
        lastPair->weight = 1.0;
        lastPair->cp1 = 2*cMdl->nPairs;
        lastPair->cp2 = 2*cMdl->nPairs+1;
        lastPair->n1  = cMdl->nPairs;
        cMdl->nPairs++;
      }

    }

  }   // for (unsigned int c = 0; c < bp->nTrees; ++c)

  // Update arrays for closest points and normals
  MatNd_realloc(cMdl->cp, 2*cMdl->nPairs, 3);
  MatNd_realloc(cMdl->n1, cMdl->nPairs, 3);

  return nNaiive;
}

/*******************************************************************************
 *
 ******************************************************************************/
RcsCollisionMdl* RcsBroadPhase_createNarrowPhase(const RcsBroadPhase* bp)
{
  RcsCollisionMdl* cMdl = RALLOC(RcsCollisionMdl);
  cMdl->graph = bp->graph;
  cMdl->pair = RNALLOC(1, RcsPair); // So that it is not NULL
  cMdl->cp = MatNd_create(1, 3);
  cMdl->n1 = MatNd_create(1, 3);
  cMdl->sMixtureCost = RCS_DISTANCE_MIXTURE_COST;
  cMdl->penetrationSlope = RCS_DISTANCE_PENETRATION_SLOPE;

  RcsBroadPhase_computeNarrowPhase(bp, cMdl);

  return cMdl;
}
