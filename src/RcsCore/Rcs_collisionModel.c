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

#include "Rcs_collisionModel.h"
#include "Rcs_graphParser.h"
#include "Rcs_typedef.h"
#include "Rcs_body.h"
#include "Rcs_math.h"
#include "Rcs_macros.h"

#include <float.h>



/*******************************************************************************
 * Reads the collision model from the XML file.
 ******************************************************************************/
RcsCollisionMdl* RcsCollisionModel_createFromXML(const RcsGraph* graph,
                                                 xmlNodePtr node)
{

  if (isXMLNodeName(node, "CollisionModel") == false)
  {
    REXEC(1)
    {
      char wrongTag[512];
      bool xmlPtrValid = getXMLNodeName(node, wrongTag);

      if (!xmlPtrValid)
      {
        RLOG(1, "xmlNodePtr is NULL - skipping model");
      }
      else
      {
        RLOG(1, "Tag has name \"%s\" and not \"CollisionPair\""
             " - skipping model", wrongTag);
      }
    }

    return NULL;
  }


  xmlNodePtr lnode = node->children;
  int nBodies = 0;

  // Count collision bodies
  while (lnode)
  {

    if (isXMLNodeName(lnode, "CollisionPair"))
    {
      if (getXMLNodeProperty(lnode, "body1"))
      {
        nBodies++;
      }
      if (getXMLNodeProperty(lnode, "body2"))
      {
        nBodies++;
      }
    }
    lnode = lnode->next;

  } // while(lnode)

  NLOG(0, "Found %d bodies", nBodies);
  RCHECK_MSG(nBodies % 2 == 0, "Uneven number of collideable bodies");

  if (nBodies==0)
  {
    return NULL;
  }

  RcsCollisionMdl* self = RALLOC(RcsCollisionMdl);
  self->graph = graph;
  self->nPairs = nBodies / 2;
  self->pair = RNALLOC(self->nPairs, RcsPair);
  self->cp = MatNd_create(nBodies, 3);         // closest points
  self->n1 = MatNd_create(self->nPairs, 3);    // normals

  // Load collision model

  // Set defaults
  double defaultThreshold = 0.1;
  self->sMixtureCost = 0.0;   // was 1000.0
  self->penetrationSlope = 10.0;
  getXMLNodePropertyDouble(node, "DistanceThreshold", &defaultThreshold);
  getXMLNodePropertyDouble(node, "threshold", &defaultThreshold);
  getXMLNodePropertyDouble(node, "MixtureCost", &self->sMixtureCost);
  getXMLNodePropertyDouble(node, "PenetrationSlope", &self->penetrationSlope);

  // Parse through pair entries
  char bdyName[RCS_MAX_NAMELEN];
  int pairIdx = 0;
  lnode = node->children;

  while (lnode)
  {

    if (isXMLNodeName(lnode, "CollisionPair"))
    {
      // body1
      RcsBody* bdy1 = NULL;
      bdyName[0] = '\0';
      if (getXMLNodePropertyStringN(lnode, "body1", bdyName, 64))
      {
        bdy1 = RcsGraph_getBodyByName(graph, bdyName);
      }

      if (bdy1 == NULL)
      {
        RLOG(4, "Body 1 \"%s\" doesn't exist - skipping pair %d", bdyName, pairIdx);
        lnode = lnode->next;
        continue;
      }

      // body2
      RcsBody* bdy2 = NULL;
      bdyName[0] = '\0';
      if (getXMLNodePropertyStringN(lnode, "body2", bdyName, 64))
      {
        bdy2 = RcsGraph_getBodyByName(graph, bdyName);
      }

      if (bdy2 == NULL)
      {
        RLOG(4, "Body 2 \"%s\" doesn't exist - skipping pair %d", bdyName, pairIdx);
        lnode = lnode->next;
        continue;
      }

      // collision pair
      self->pair[pairIdx].dThreshold = defaultThreshold;
      self->pair[pairIdx].weight = 1.0;
      self->pair[pairIdx].b1 = bdy1->id;
      self->pair[pairIdx].b2 = bdy2->id;
      self->pair[pairIdx].cp1 = 2*pairIdx;
      self->pair[pairIdx].cp2 = 2*pairIdx+1;
      self->pair[pairIdx].n1  = pairIdx;

      getXMLNodePropertyDouble(lnode, "DistanceThreshold",
                               &self->pair[pairIdx].dThreshold);
      getXMLNodePropertyDouble(lnode, "threshold",
                               &self->pair[pairIdx].dThreshold);
      getXMLNodePropertyDouble(lnode, "weight", &self->pair[pairIdx].weight);

      RLOG(5, "Collision pair %d is \"%s\" - \"%s\"\n"
           "\tDistance threshold =  %g\n\tDistance weight is %g",
           self->nPairs, bdy1->name, bdy2->name,
           self->pair[pairIdx].dThreshold, self->pair[pairIdx].weight);

      pairIdx++;

    }   // if(isXMLNodeName(lnode, "CollisionPair"))

    lnode = lnode->next;

  } // while(lnode)

  // Adjust dimensions of matrices for closest points and normals, in case some
  // bodies have not been found in the graph.
  MatNd_reshape(self->cp, 2*self->nPairs, 3);
  MatNd_reshape(self->n1, self->nPairs, 3);

  return self;
}

/*******************************************************************************
 *
 ******************************************************************************/
void RcsCollisionModel_compute(RcsCollisionMdl* self)
{
  if (self == NULL)
  {
    NLOG(0, "Collision model is NULL - skip computing");
    return;
  }

  if (self->pair == NULL)
  {
    NLOG(0, "No pair found in collision model - skip computing");
    return;
  }

  for (unsigned int i=0; i<self->nPairs; ++i)
  {
    RcsPair* PAIR = &self->pair[i];
    const RcsBody* b1 = RCSBODY_BY_ID(self->graph, PAIR->b1);
    const RcsBody* b2 = RCSBODY_BY_ID(self->graph, PAIR->b2);
    NLOG(0, "Computing distance between \"%s\" and \"%s\"", b1->name, b2->name);

    double* cp1 = MatNd_getRowPtr(self->cp, PAIR->cp1);
    double* cp2 = MatNd_getRowPtr(self->cp, PAIR->cp2);
    double* n1 = MatNd_getRowPtr(self->cp, PAIR->n1);
    PAIR->distance = RcsBody_distance(b1, b2, cp1, cp2, n1);
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
void RcsCollisionModel_fprintCollisions(FILE* fd, const RcsCollisionMdl* self,
                                        double distanceThreshold)
{
  if (fd == NULL)
  {
    RLOG(3, "You are trying to print a collision model to a NULL file "
         "descriptor - ignoring");
    return;
  }

  fprintf(fd, "[%s: %s(%d)]: ", __FILE__, __FUNCTION__, __LINE__);

  if (self == NULL)
  {
    fprintf(fd, "The collision model is NULL\n");
    return;
  }

  if (self->nPairs == 0)
  {
    fprintf(fd, "No pairs in collision model!\n");
    return;
  }


  fprintf(fd, "The following pairs have a distance below %f:\n",
          distanceThreshold);


  for (unsigned int i=0; i<self->nPairs; ++i)
  {
    const RcsPair* pair = &self->pair[i];

    if (pair->distance<=distanceThreshold)
    {
      const RcsBody* b1 = RCSBODY_BY_ID(self->graph, pair->b1);
      const RcsBody* b2 = RCSBODY_BY_ID(self->graph, pair->b2);

      fprintf(stderr, "\t[%d]   %s", i, b1 ? b1->name : "NULL");
      fprintf(stderr, "\t\t%s", b2 ? b2->name : "NULL");
      fprintf(stderr, "\tdistance = %.6f   ", pair->distance);
      const double* cp1 = MatNd_getRowPtr(self->cp, pair->cp1);
      const double* cp2 = MatNd_getRowPtr(self->cp, pair->cp2);
      fprintf(stderr, "cp1=%.4f %.4f %.4f", cp1[0], cp1[1], cp1[2]);
      fprintf(stderr, "   cp2=%.4f %.4f %.4f\n", cp2[0], cp2[1], cp2[2]);
    }

  }

}

/*******************************************************************************
 *
 ******************************************************************************/
void RcsCollisionModel_fprint(FILE* fd, const RcsCollisionMdl* self)
{
  if (fd == NULL)
  {
    RLOG(3, "You are trying to print a collision model to a NULL file "
         "descriptor - ignoring");
    return;
  }

  fprintf(fd, "[%s: %s(%d)]: ", __FILE__, __FUNCTION__, __LINE__);

  if (self == NULL)
  {
    fprintf(fd, "The collision model is NULL\n");
    return;
  }

  fprintf(fd, "Collision model parameters:\n");
  fprintf(fd, "\tMixture cost: %f\n", self->sMixtureCost);
  fprintf(fd, "\tPenetration slope: %f\n\n", self->penetrationSlope);

  if (self->nPairs > 0)
  {
    fprintf(fd, "The collision model consists of\n");

    for (unsigned int i=0; i<self->nPairs; ++i)
    {
      const RcsPair* pair = &self->pair[i];
      const RcsBody* b1 = RCSBODY_BY_ID(self->graph, pair->b1);
      const RcsBody* b2 = RCSBODY_BY_ID(self->graph, pair->b2);

      fprintf(fd, "\t[%d]   %s", i, b1 ? b1->name : "NULL");
      fprintf(fd, "\t\t%s", b2 ? b2->name : "NULL");
      fprintf(fd, "\n\tdistance = %.6f   weight=%.3f   dThreshold=%.3f\n",
              pair->distance, pair->weight, pair->dThreshold);
    }
  }
  else
  {
    fprintf(fd, "No pairs in collision model!\n");
  }

}

/*******************************************************************************
 * It is safe to call free on NULL
 ******************************************************************************/
void RcsCollisionModel_destroy(RcsCollisionMdl* self)
{
  if (self)
  {
    RFREE(self->pair);
    MatNd_destroy(self->cp);
    MatNd_destroy(self->n1);
    RFREE(self);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
RcsCollisionMdl* RcsCollisionModel_clone(const RcsCollisionMdl* src,
                                         const RcsGraph* newGraph)
{
  if (src == NULL)
  {
    return NULL;
  }

  RcsCollisionMdl* dst = RALLOC(RcsCollisionMdl);
  memcpy(dst, src, sizeof(RcsCollisionMdl));

  // Clone pointer data structures
  dst->cp = MatNd_clone(src->cp);
  dst->n1 = MatNd_clone(src->n1);
  dst->graph = newGraph ? newGraph : src->graph;
  dst->pair = NULL;

  if (dst->nPairs > 0)
  {
    dst->pair = RNALLOC(src->nPairs, RcsPair);
    memcpy(dst->pair, src->pair, src->nPairs*sizeof(RcsPair));
  }

  return dst;
}

/*******************************************************************************
 *
 ******************************************************************************/
double RcsCollisionMdl_getMinDist(const RcsCollisionMdl* self)
{
  return RcsCollisionMdl_getMinDistPair(self, NULL);
}

/*******************************************************************************
 *
 ******************************************************************************/
double RcsCollisionMdl_getMinDistPair(const RcsCollisionMdl* self, int* pairIdx)
{
  double d = DBL_MAX;

  if (self == NULL || self->pair == NULL)
  {
    return d;
  }

  for (unsigned int i=0; i<self->nPairs; ++i)
  {
    RcsPair* PAIR = &self->pair[i];

    if (PAIR->distance < d)
    {
      d = PAIR->distance;
      if (pairIdx)
      {
        *pairIdx = i;
      }
    }

  }

  return d;
}

/*******************************************************************************
 *
 ******************************************************************************/
double RcsCollisionMdl_cost(const RcsCollisionMdl* self)
{
  if ((self==NULL) || (self->pair==NULL))
  {
    return 0.0;
  }

  const double s = self->penetrationSlope;
  double cost = 0.0;

  for (unsigned int i=0; i<self->nPairs; ++i)
  {
    const RcsPair* PAIR = &self->pair[i];
    const double dLimit = PAIR->dThreshold;
    const double d  = PAIR->distance;
    const double w  = PAIR->weight;

    if (d > dLimit)
    {
      continue;
    }

    if (d <= 0.0)
    {
      cost += w*(-2.0*s*d/dLimit + s);
    }
    else
    {
      cost += w*(s / (dLimit*dLimit))*(d-dLimit)*(d-dLimit);
    }

    // Inverse exponential center distance
    if (self->sMixtureCost > 0.0)
    {
      const RcsBody* b1 = RCSBODY_BY_ID(self->graph, PAIR->b1);
      const RcsBody* b2 = RCSBODY_BY_ID(self->graph, PAIR->b2);
      const double* center1 = b1 ? b1->A_BI.org : Vec3d_zeroVec();
      const double* center2 = b2 ? b2->A_BI.org : Vec3d_zeroVec();
      double dCenters = Vec3d_distance(center1, center2);

      if (d <= 0.0)
      {
        cost += w*self->sMixtureCost*exp(-dCenters);
      }
      else
      {
        cost += w*self->sMixtureCost*(1.0-d/dLimit)*exp(-dCenters);
      }
    }   // if(self->sMixtureCost > 0.0)

  }

  return cost;
}

/*******************************************************************************
 *
 ******************************************************************************/
void RcsCollisionMdl_gradient(const RcsCollisionMdl* self, MatNd* grad)
{
  if (self == NULL)
  {
    return;
  }

  MatNd_reshapeAndSetZero(grad, 1, self->graph->nJ);

  if (self->pair == NULL)
  {
    return;
  }

  const double s = self->penetrationSlope;
  MatNd* dDpdq = NULL;
  MatNd_create2(dDpdq, grad->m, grad->n);
  MatNd* dDcdq = NULL;
  MatNd_create2(dDcdq, grad->m, grad->n);
  MatNd_reshapeAndSetZero(grad, 1, self->graph->nJ);

  for (unsigned int i=0; i<self->nPairs; ++i)
  {
    const RcsPair* PAIR = &self->pair[i];
    const double dLimit = PAIR->dThreshold;
    const double d  = PAIR->distance;

    // If the distance is larger than the threshold, the gradient is zero.
    if (d > dLimit)
    {
      continue;
    }

    const double w  = PAIR->weight;
    const RcsBody* b1 = RCSBODY_BY_ID(self->graph, PAIR->b1);
    const RcsBody* b2 = RCSBODY_BY_ID(self->graph, PAIR->b2);
    bool repelling = (d >= 0.0) ? true : false;
    double dgpDdp;

    double* cp1 = MatNd_getRowPtr(self->cp, PAIR->cp1);
    double* cp2 = MatNd_getRowPtr(self->cp, PAIR->cp2);
    double* n1 = MatNd_getRowPtr(self->cp, PAIR->n1);
    RcsBody_distanceGradient(self->graph, b1, b2, repelling, cp1, cp2, n1, dDpdq);

    if (d < 0)
    {
      dgpDdp = -2.0*s/dLimit;
    }
    else
    {
      dgpDdp = (2.0*s / (dLimit*dLimit)) * (d-dLimit);
    }

    MatNd_constMulAndAddSelf(grad, dDpdq, dgpDdp*w);



    // Mixture gradient
    if (self->sMixtureCost > 0.0)
    {
      double I_p1_center[3], I_p2_center[3], dgcDdc, dgcDdp, dCenters;

      // Partial derivative of cost with respect to dCenters
      dCenters = RcsBody_centerDistance(b1, b2, NULL, NULL,
                                        I_p1_center, I_p2_center);

      RcsBody_distanceGradient(self->graph, b1, b2, true,
                               I_p1_center, I_p2_center, NULL, dDcdq);

      // dgc/ddc
      if (d < 0)
      {
        dgcDdc = -self->sMixtureCost*exp(-dCenters);
      }
      else
      {
        dgcDdc = -self->sMixtureCost*(1.0-d/dLimit)*exp(-dCenters);
      }

      MatNd_constMulAndAddSelf(grad, dDcdq, dgcDdc*w);

      // dgc/ddp
      if (d < 0)
      {
        dgcDdp = 0.0;
      }
      else
      {
        dgcDdp = self->sMixtureCost*(-1.0/dLimit)*exp(-dCenters);
      }

      MatNd_constMulAndAddSelf(grad, dDpdq, dgcDdp*w);

    }   // if(self->sMixtureCost > 0.0)

  }   // RCSPAIR_TRAVERSE(self->pair)

  MatNd_destroy(dDpdq);
  MatNd_destroy(dDcdq);
}

/*******************************************************************************
 *
 ******************************************************************************/
bool RcsCollisionMdl_isEqual(const RcsCollisionMdl* self,
                             const RcsCollisionMdl* other,
                             double eps)
{
  bool isEqual = true;

  if (self->nPairs != other->nPairs)
  {
    RLOGS(1, "Different number of pairs: %d!=%d", self->nPairs, other->nPairs);
    isEqual = false;
  }

  if (fabs(self->sMixtureCost - other->sMixtureCost) > eps)
  {
    RLOGS(1, "Mixture cost differs: %g != %g",
          self->sMixtureCost, other->sMixtureCost);
    isEqual = false;
  }

  if (fabs(self->penetrationSlope - other->penetrationSlope) > eps)
  {
    RLOGS(1, "Penetration slope differs: %g != %g",
          self->penetrationSlope, other->penetrationSlope);
    isEqual = false;
  }

  if (MatNd_isEqual(self->cp, other->cp, eps) == false)
  {
    RLOGS(2, "Collision model mismatch in closest point array");
    REXEC(4)
    {
      MatNd_printTwoArraysDiff(self->cp, other->cp, 6);
    }
    isEqual = false;
  }

  if (MatNd_isEqual(self->n1, other->n1, eps) == false)
  {
    RLOGS(2, "Collision model mismatch in closest point normals array");
    REXEC(4)
    {
      MatNd_printTwoArraysDiff(self->n1, other->n1, 6);
    }
    isEqual = false;
  }

  if (memcmp(self->pair, other->pair, self->nPairs*sizeof(RcsPair)) != 0)
  {
    RLOGS(2, "Difference in the pairs");
    isEqual = false;
  }

  return isEqual;
}
