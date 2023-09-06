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
#include "Rcs_parser.h"
#include "Rcs_macros.h"

#include <float.h>



/*******************************************************************************
 * Separating Axes test: returns true if no collision is possible.
 ******************************************************************************/
static inline bool SAT(const double xyzMin1[3], const double xyzMax1[3],
                       const double xyzMin2[3], const double xyzMax2[3])
{

  if ((xyzMin1[0] > xyzMax2[0]) || (xyzMax1[0] < xyzMin2[0]))
  {
    return true;
  }

  if ((xyzMin1[1] > xyzMax2[1]) || (xyzMax1[1] < xyzMin2[1]))
  {
    return true;
  }

  if ((xyzMin1[2] > xyzMax2[2]) || (xyzMax1[2] < xyzMin2[2]))
  {
    return true;
  }

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
RcsBroadPhase* RcsBroadPhase_createFromXML(const RcsGraph* graph,
                                           xmlNodePtr node)
{
  if (!isXMLNodeName(node, "BroadPhase"))
  {
    RLOG(1, "XML tag in broadphase is not \"BroadPhase\" - returning NULL");
    return NULL;
  }

  double distanceThreshold = 0.0;
  getXMLNodePropertyDouble(node, "DistanceThreshold", &distanceThreshold);

  RcsBroadPhase* bp = RcsBroadPhase_create(graph, distanceThreshold);

  xmlNodePtr child = node->children;

  // Count collision bodies
  while (child)
  {

    if (isXMLNodeName(child, "Tree"))
    {
      char treeRoot[RCS_MAX_NAMELEN] = "";
      if (getXMLNodePropertyStringN(child, "root", treeRoot, RCS_MAX_NAMELEN))
      {
        const RcsBody* rootBdy = RcsGraph_getBodyByName(graph, treeRoot);
        if (rootBdy)
        {
          RcsBroadPhase_addTree(bp, rootBdy->id);
        }
        else
        {
          RLOG(1, "Broadphase root %s not found", treeRoot);
        }
      }

    }
    child = child->next;

  } // while(lnode)

  return bp;
}

/*******************************************************************************
 *
 ******************************************************************************/
void RcsBroadPhase_destroy(RcsBroadPhase* bp)
{
  if (!bp)
  {
    return;
  }

  RFREE(bp->bodies);

  for (unsigned int i=0; i<bp->nTrees; ++i)
  {
    RcsBroadPhaseTree* tree_i = &bp->trees[i];
    RFREE(tree_i->bodies);
  }

  RFREE(bp->trees);
  RFREE(bp);
}

/*******************************************************************************
 *
 ******************************************************************************/
RcsBroadPhase* RcsBroadPhase_clone(const RcsBroadPhase* src,
                                   const RcsGraph* newGraph)
{
  if (src == NULL)
  {
    return NULL;
  }

  const unsigned int nBodies = src->graph->nBodies;
  RcsBroadPhase* dst = RALLOC(RcsBroadPhase);

  dst->graph = newGraph ? newGraph : src->graph;

  // Allocate trees
  dst->nTrees = src->nTrees;
  dst->trees = RNALLOC(nBodies, RcsBroadPhaseTree);

  for (unsigned int i=0; i<dst->nTrees; ++i)
  {
    const RcsBroadPhaseTree* srcTree = &src->trees[i];
    RcsBroadPhaseTree* dstTree = &dst->trees[i];
    memcpy(dstTree, srcTree, sizeof(RcsBroadPhaseTree));
    // Array of bodies needs to be recreated
    dstTree->bodies = RNALLOC(nBodies, RcsBroadPhaseBdy);
    memcpy(dstTree->bodies, srcTree->bodies, dstTree->nBodies*sizeof(RcsBroadPhaseBdy));
  }

  dst->nBodies = src->nBodies;
  dst->bodies = RNALLOC(nBodies, RcsBroadPhaseBdy);
  memcpy(dst->bodies, src->bodies, dst->nBodies*sizeof(RcsBroadPhaseBdy));

  dst->distanceThreshold = src->distanceThreshold;

  return dst;
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
static bool RcsBroadPhase_isBodyInTree(RcsBroadPhase* bp, int bodyId)
{

  for (unsigned int i = 0; i < bp->nTrees; ++i)
  {
    const RcsBroadPhaseTree* tree = &bp->trees[i];

    for (unsigned int j = 0; j < tree->nBodies; ++j)
    {
      const RcsBroadPhaseBdy* bdy = &tree->bodies[j];

      if (bdy->id==bodyId)
      {
        return true;
      }
    }
  }

  return false;
}

/*******************************************************************************
 *
 ******************************************************************************/
void RcsBroadPhase_updateBoundingVolumes(RcsBroadPhase* bp)
{
  // Compute all chains. This is the first step, since we populate an array of
  // ids of the chain bodies. In the collection of the bodies, these are not
  // considered, so that we don't have an object showing up both in a chain
  // and in the body collections.
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
      RcsBroadPhaseBdy* treeBdy = &tree->bodies[tree->nBodies];
      treeBdy->hasAABB = RcsGraph_computeBodyAABB(bp->graph, BODY->id,
                                                  RCSSHAPE_COMPUTE_DISTANCE,
                                                  sMin, sMax, NULL);

      if (treeBdy->hasAABB)
      {
        tree->hasAABB = true;
        Vec3d_constAddSelf(sMin, -bp->distanceThreshold);
        Vec3d_constAddSelf(sMax,  bp->distanceThreshold);

        for (int i = 0; i < 3; ++i)
        {
          xyzMin[i] = fmin(sMin[i], xyzMin[i]);
          xyzMax[i] = fmax(sMax[i], xyzMax[i]);
        }

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

    if (RcsBroadPhase_isBodyInTree(bp, bdy_i->id))
    {
      //RLOG(2, "Excluding %s", RCSBODY_NAME_BY_ID(bp->graph, bdy_i->id));
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

}

/*******************************************************************************
 * t1 t2 t3 t4
 *
 * t1 t2
 * t1 t3
 * t1 t4
 * t2 t3
 * t2 t4
 * t3 t4
 *
 ******************************************************************************/
static int RcsBroadPhase_computeTreeTreeNarrowPhase(const RcsBroadPhase* bp,
                                                    RcsCollisionMdl* cMdl,
                                                    unsigned int pairCapacity)
{
  // For statistics: nNaiive is the worst case combination of all tree bodies
  // against all rigid bodies.
  int nNaiive = 0;

  // This is the "n" of n-choose-k (k is 2)
  int n = bp->nTrees;

  if (n == 0)
  {
    return nNaiive;
  }

  // Dynamically allocate memory for the 2D array to store pairs
  int* values = RNALLOC(n, int);
  int(*pairs)[2] = (int(*)[2])values;
  int pairCount = 0;

  // Generate and store pairs in the array
  for (int i = 1; i <= n; i++)
  {
    for (int j = i + 1; j <= n; j++)
    {
      pairs[pairCount][0] = i-1;
      pairs[pairCount][1] = j-1;
      pairCount++;
    }
  }

  // Print the generated pairs
  NLOG(1, "%d Generated pairs:", pairCount);
  for (int i = 0; i < pairCount; ++i)
  {
    NLOG(1, "(%d, %d)", pairs[i][0], pairs[i][1]);
    RCHECK(pairs[i][0] < (int)bp->nTrees);
    RCHECK(pairs[i][1] < (int)bp->nTrees);
    const RcsBroadPhaseTree* tree1 = &bp->trees[pairs[i][0]];
    const RcsBroadPhaseTree* tree2 = &bp->trees[pairs[i][1]];

    nNaiive += tree1->nBodies*tree2->nBodies;

    // First separating axes test around overall trees. If an axis is
    // overlapping, the tree will be further checked below for addition
    // to the narrow phase.
    if ((!tree1->hasAABB))
    {
      NLOG(1, "Tree %d has no AABB: skipping trees %d - %d", pairs[i][0], pairs[i][0], pairs[i][1]);
      continue;
    }

    if ((!tree2->hasAABB))
    {
      NLOG(1, "Tree %d has no AABB: skipping trees %d - %d", pairs[i][1], pairs[i][0], pairs[i][1]);
      continue;
    }

    if (SAT(tree1->aabbMin, tree1->aabbMax, tree2->aabbMin, tree2->aabbMax))
    {
      NLOG(1, "Found SAT in tree-tree: skipping trees %d - %d", pairs[i][0], pairs[i][1]);
      continue;
    }


    // Add all bodies of the tree1 to the overall tree2
    for (unsigned int j = 0; j < tree1->nBodies; ++j)
    {
      const RcsBroadPhaseBdy* treeBdy1 = &tree1->bodies[j];

      //if (!treeBdy1->hasAABB)
      //{
      //  RLOG(1, "Tree-body %d (%s) of tree %d has no AABB",
      //       j, RCSBODY_NAME_BY_ID(bp->graph, treeBdy1->id), pairs[i][0]);
      //  continue;
      //}

      // The tree-bodie's AABB has not necessarily been computed. If not, we
      // assume the worst case and add all combinations to the narrow phase.
      if (treeBdy1->hasAABB && SAT(treeBdy1->aabbMin, treeBdy1->aabbMax,
                                   tree2->aabbMin, tree2->aabbMax))
      {
        NLOG(1, "Found SAT in body-tree: skipping body %d - tree %d", j, pairs[i][1]);
        continue;
      }

      // Here we have a collision of treeBdy1 with the tree2. We break down
      // tree2 into its individual bodies.
      for (unsigned int k = 0; k < tree2->nBodies; ++k)
      {
        const RcsBroadPhaseBdy* treeBdy2 = &tree2->bodies[k];

        //if (!treeBdy2->hasAABB)
        //{
        //  RLOG(1, "Tree-body %d (%s) of tree %d has no AABB",
        //       k, RCSBODY_NAME_BY_ID(bp->graph, treeBdy2->id), pairs[i][1]);
        //  continue;
        //}

        // The tree-bodie's AABB has not necessarily been computed. If not, we
        // assume the worst case and add all combinations to the narrow phase.
        if (treeBdy2->hasAABB && SAT(treeBdy1->aabbMin, treeBdy1->aabbMax,
                                     treeBdy2->aabbMin, treeBdy2->aabbMax))
        {
          NLOG(1, "Found SAT in body-body: skipping bodies %d - %d", k, j);
          continue;
        }

        // Ending up here give us the narrow phase pair [treeBdy1,treeBdy2]
        // that must be added to the narrow phase.
        // We only grow and never shrink the narrow phase collision model to
        // minimize the number of memory reallocations.
        if (cMdl->nPairs >= pairCapacity)
        {
          cMdl->pair = RREALLOC(cMdl->pair, cMdl->nPairs + 1, RcsPair);
          RCHECK_MSG(cMdl->pair, "Failed to allocate memory for %d pairs",
                     cMdl->nPairs + 1);
          NLOG(1, "REALLOC");
        }
        RcsPair* lastPair = &cMdl->pair[cMdl->nPairs];
        memset(lastPair, 0, sizeof(RcsPair));
        lastPair->b1 = treeBdy1->id;
        lastPair->b2 = treeBdy2->id;
        lastPair->weight = 1.0;
        lastPair->cp1 = 2 * cMdl->nPairs;
        lastPair->cp2 = 2 * cMdl->nPairs + 1;
        lastPair->n1 = cMdl->nPairs;
        cMdl->nPairs++;
        NLOG(1, "Adding pair %s - %s",
             RCSBODY_NAME_BY_ID(bp->graph, treeBdy1->id),
             RCSBODY_NAME_BY_ID(bp->graph, treeBdy2->id));

      }   // for (unsigned int k = 0; k < tree2->nBodies; ++k)

    }   // for (unsigned int j = 0; j < tree1->nBodies; ++j)

  }   // for (int i = 0; i < pairCount; ++i)


  // Update arrays for closest points and normals
  MatNd_realloc(cMdl->cp, 2 * cMdl->nPairs, 3);
  MatNd_realloc(cMdl->n1, cMdl->nPairs, 3);

  // Free allocated memory for tree pairs
  RFREE(values);

  return nNaiive;
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
      const RcsBroadPhaseBdy* bdy_i = &bp->bodies[i];

      // Perform separating axes test. If an axis is overlapping, the
      // pair will be further checked below for addition to the narrow phase.
      if (SAT(bdy_i->aabbMin, bdy_i->aabbMax, tree->aabbMin, tree->aabbMax))
      {
        NLOG(1, "Found SAT in tree: skipping %s - %s",
             bp->graph->bodies[bp->bodies[i].id].name,
             bp->graph->bodies[tree->startBdyId].name);
        continue;
      }

      // Add all combinations of the tree to the model
      for (unsigned int j=0; j<tree->nBodies; ++j)
      {
        const RcsBroadPhaseBdy* treeBdy = &tree->bodies[j];

        // Skip these that don't collide
        if (SAT(bdy_i->aabbMin, bdy_i->aabbMax, treeBdy->aabbMin, treeBdy->aabbMax))
        {
          NLOG(1, "Found SAT in subtree: skipping %s - %s",
               bp->graph->bodies[bp->bodies[i].id].name,
               bp->graph->bodies[tree->bodies[j].id].name);
          continue;
        }

        // We only grow and never shrink the narrow phase collision model to
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
        lastPair->dThreshold = bp->distanceThreshold;
        cMdl->nPairs++;
        NLOG(4, "Adding pair %s - %s",
             RCSBODY_NAME_BY_ID(bp->graph, tree->bodies[j].id),
             RCSBODY_NAME_BY_ID(bp->graph, bp->bodies[i].id));
      }

    }

  }   // for (unsigned int c = 0; c < bp->nTrees; ++c)

  // Update arrays for closest points and normals
  MatNd_realloc(cMdl->cp, 2*cMdl->nPairs, 3);
  MatNd_realloc(cMdl->n1, cMdl->nPairs, 3);

  //REXEC(0)
  {
    nNaiive += RcsBroadPhase_computeTreeTreeNarrowPhase(bp, cMdl, nPairs);
  }

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
  cMdl->cp = MatNd_create(0, 3);
  cMdl->n1 = MatNd_create(0, 3);
  cMdl->sMixtureCost = RCS_DISTANCE_MIXTURE_COST;
  cMdl->penetrationSlope = RCS_DISTANCE_PENETRATION_SLOPE;

  RcsBroadPhase_computeNarrowPhase(bp, cMdl);

  return cMdl;
}

/*******************************************************************************
 *
 ******************************************************************************/
void RcsBroadPhase_fprint(FILE* fd, const RcsBroadPhase* bp)
{
  if (!bp)
  {
    fprintf(fd, "Broadphase is NULL\n");
    return;
  }

  fprintf(fd, "Broadphase with %u trees and %u bodies:\n",
          bp->nTrees, bp->nBodies);
  fprintf(fd, "Distance threshold: %f\n", bp->distanceThreshold);

  for (unsigned int i = 0; i < bp->nBodies; ++i)
  {
    const RcsBroadPhaseBdy* bdy = &bp->bodies[i];
    fprintf(fd, "Body %u: id: %d (%s)\n",
            i, bdy->id, RCSBODY_NAME_BY_ID(bp->graph, bdy->id));
  }

  for (unsigned int i = 0; i < bp->nTrees; ++i)
  {
    const RcsBroadPhaseTree* tree = &bp->trees[i];

    fprintf(fd, "Tree %u: root-body id: %d nBodies: %d\n",
            i, tree->startBdyId, tree->nBodies);

    for (unsigned int j = 0; j < tree->nBodies; ++j)
    {
      const RcsBroadPhaseBdy* bdy = &tree->bodies[j];
      fprintf(fd, "Tree-body %u: id: %d (%s)\n",
              j, bdy->id, RCSBODY_NAME_BY_ID(bp->graph, bdy->id));
    }
  }



}
