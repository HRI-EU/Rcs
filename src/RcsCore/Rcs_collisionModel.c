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

    return NULL;
  }

  RcsCollisionMdl* self = RALLOC(RcsCollisionMdl);
  self->graph = graph;

  char body[64];
  xmlNodePtr lnode = node->children;
  int nBodies = 0, nPairs = 0;

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
  nPairs = nBodies / 2;


  // Allocate collision model (Last pointer is NULL)
  if (nPairs > 0)
  {
    self->pair = RNALLOC(nPairs + 1, RcsPair*);
  }
  else
  {
    RFREE(self);
    return NULL;
  }

  RcsPair** pair = self->pair;

  // Allocation of matrices for closest points and normals
  self->cp = MatNd_create(nBodies, 3);
  self->n1 = MatNd_create(nPairs, 3);



  // Load collision model

  // defaults
  double defaultThreshold = 0.1;
  self->sMixtureCost = 0.0;   // was 1000.0
  self->penetrationSlope = 10.0;
  getXMLNodePropertyDouble(node, "DistanceThreshold", &defaultThreshold);
  getXMLNodePropertyDouble(node, "threshold", &defaultThreshold);
  getXMLNodePropertyDouble(node, "MixtureCost", &self->sMixtureCost);
  getXMLNodePropertyDouble(node, "PenetrationSlope", &self->penetrationSlope);
  NLOG(0, "Distance threshold is %g", defaultThreshold);


  // Parse through pair entries
  nPairs = 0;
  lnode = node->children;

  while (lnode)
  {

    if (isXMLNodeName(lnode, "CollisionPair"))
    {
      // body1
      RcsBody* bdy1 = NULL;
      if (getXMLNodePropertyStringN(lnode, "body1", body, 64))
      {
        bdy1 = RcsGraph_getBodyByName(graph, body);
        if (bdy1 == NULL)
        {
          RLOG(4, "Body 1 \"%s\" doesn't exist", body);
        }
      }

      // body2
      RcsBody* bdy2 = NULL;
      if (getXMLNodePropertyStringN(lnode, "body2", body, 64))
      {
        bdy2 = RcsGraph_getBodyByName(graph, body);
        if (bdy2 == NULL)
        {
          RLOG(4, "Body 2 \"%s\" doesn't exist", body);
        }
      }

      // If one of the bodies does not exist, we put the array index to the
      // previous position and do not make a collision pair;
      if ((bdy1 == NULL) || (bdy2 == NULL))
      {
        lnode = lnode->next;
        continue;
      }

      // collision pair
      pair[nPairs] = RALLOC(RcsPair);
      pair[nPairs]->dThreshold = defaultThreshold;
      pair[nPairs]->weight = 1.0;
      pair[nPairs]->b1 = bdy1;
      pair[nPairs]->b2 = bdy2;

      pair[nPairs]->cp1 = MatNd_getRowPtr(self->cp, 2 * nPairs);
      pair[nPairs]->cp2 = MatNd_getRowPtr(self->cp, 2 * nPairs + 1);
      pair[nPairs]->n1  = MatNd_getRowPtr(self->n1, nPairs);

      getXMLNodePropertyDouble(lnode, "DistanceThreshold",
                               &(pair[nPairs])->dThreshold);
      getXMLNodePropertyDouble(lnode, "threshold",
                               &(pair[nPairs])->dThreshold);
      getXMLNodePropertyDouble(lnode, "weight", &(pair[nPairs])->weight);

      RLOG(5, "Collision pair %d is \"%s\" - \"%s\"\n"
           "\tDistance threshold =  %g\n\tDistance weight is %g",
           nPairs, pair[nPairs]->b1->name, pair[nPairs]->b2->name,
           pair[nPairs]->dThreshold, pair[nPairs]->weight);

      nPairs++;

    }   // if(isXMLNodeName(lnode, "CollisionPair"))

    lnode = lnode->next;

  } // while(lnode)

  // Adjust dimensions of matrices for closest points and normals, in case some
  // bodies have not been found in the graph.
  nBodies = 2*nPairs;
  MatNd_reshape(self->cp, nBodies, 3);
  MatNd_reshape(self->n1, nPairs, 3);

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

  RCSPAIR_TRAVERSE(self->pair)
  {
    NLOG(0, "Computing distance between \"%s\" and \"%s\"",
         PAIR->b1->name, PAIR->b2->name);
    PAIR->distance = RcsBody_distance(PAIR->b1, PAIR->b2,
                                      PAIR->cp1, PAIR->cp2, PAIR->n1);
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

  RcsPair** pPtr = self->pair;

  if (pPtr == NULL)
  {
    fprintf(fd, "No pairs in collision model!\n");
    return;
  }


  fprintf(fd, "The following pairs have a distance below %f:\n",
          distanceThreshold);


  int k = 0;
  while (*pPtr)
  {

    if ((*pPtr)->distance<=distanceThreshold)
    {
      fprintf(stderr, "\t[%d]   %s", k, (*pPtr)->b1 ? (*pPtr)->b1->name : "NULL");

      if ((*pPtr)->b1 && STRNEQ((*pPtr)->b1->name, "GenericBody", 11))
      {
        RcsBody* gPtr = (RcsBody*)(*pPtr)->b1->extraInfo;
        fprintf(stderr, " (points to \"%s\")", gPtr ? gPtr->name : "NULL");
      }

      fprintf(stderr, "\t\t%s", (*pPtr)->b2 ? (*pPtr)->b2->name : "NULL");

      if ((*pPtr)->b2 && STRNEQ((*pPtr)->b2->name, "GenericBody", 11))
      {
        RcsBody* gPtr = (RcsBody*)(*pPtr)->b2->extraInfo;
        fprintf(stderr, " (points to \"%s\")", gPtr ? gPtr->name : "NULL");
      }

      fprintf(stderr, "\tdistance = %.6f   ", (*pPtr)->distance);
      const double* cp1 = (*pPtr)->cp1;
      const double* cp2 = (*pPtr)->cp2;
      fprintf(stderr, "cp1=%.4f %.4f %.4f", cp1[0], cp1[1], cp1[2]);
      fprintf(stderr, "   cp2=%.4f %.4f %.4f\n", cp2[0], cp2[1], cp2[2]);
    }

    pPtr++;
    k++;
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

  RcsPair** pPtr = self->pair;

  if (pPtr != NULL)
  {
    fprintf(fd, "The collision model consists of\n");

    int k = 0;
    while (*pPtr)
    {
      fprintf(fd, "\t[%d]   %s", k, (*pPtr)->b1 ? (*pPtr)->b1->name : "NULL");

      if ((*pPtr)->b1 && STRNEQ((*pPtr)->b1->name, "GenericBody", 11))
      {
        RcsBody* gPtr = (RcsBody*)(*pPtr)->b1->extraInfo;
        fprintf(fd, " (points to \"%s\")", gPtr ? gPtr->name : "NULL");
      }

      fprintf(fd, "\t\t%s", (*pPtr)->b2 ? (*pPtr)->b2->name : "NULL");

      if ((*pPtr)->b2 && STRNEQ((*pPtr)->b2->name, "GenericBody", 11))
      {
        RcsBody* gPtr = (RcsBody*)(*pPtr)->b2->extraInfo;
        fprintf(fd, " (points to \"%s\")", gPtr ? gPtr->name : "NULL");
      }

      fprintf(fd, "\n\tdistance = %.6f   weight=%.3f   dThreshold=%.3f\n",
              (*pPtr)->distance, (*pPtr)->weight, (*pPtr)->dThreshold);

      pPtr++;
      k++;
    }
  }
  else
  {
    fprintf(fd, "No pairs in collision model!\n");
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
void RcsCollisionModel_destroy(RcsCollisionMdl* self)
{
  int i = 0;

  if (self == NULL)
  {
    NLOG(3, "Collision model is NULL - skip destroying");
    return;
  }

  if (self->pair != NULL)
  {
    while (self->pair[i])
    {
      RLOG(5, "Deleting collision pair %d: \"%s\" - \"%s\"",
           i, self->pair[i]->b1->name, self->pair[i]->b2->name);
      RFREE(self->pair[i]);
      self->pair[i] = NULL;
      i++;
    }
    RFREE(self->pair);
    self->pair = NULL;

    MatNd_destroy(self->cp);
    MatNd_destroy(self->n1);
  }

  RFREE(self);
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

  // Global parameters
  dst->sMixtureCost = src->sMixtureCost;
  dst->penetrationSlope = src->penetrationSlope;

  // Clone arrays for closest points and normals
  dst->cp = MatNd_clone(src->cp);
  dst->n1 = MatNd_clone(src->n1);

  // Copy graph pointer or set new one
  if (newGraph != NULL)
  {
    dst->graph = newGraph;
  }
  else
  {
    dst->graph = src->graph;
  }

  // Copy collision pair list
  if (src->pair != NULL)
  {
    // Count number of pairs in src
    int nCPairs = 0;
    while (src->pair[nCPairs])
    {
      nCPairs++;
    }

    // Copy pair list
    dst->pair = RNALLOC(nCPairs + 1, RcsPair*);
    RCHECK(dst->pair);

    for (int i = 0; i < nCPairs; i++)
    {
      dst->pair[i] = RALLOC(RcsPair);
      dst->pair[i]->b1 = NULL;
      dst->pair[i]->b2 = NULL;

      if (src->pair[i]->b1 != NULL)
      {
        dst->pair[i]->b1 =
          RcsGraph_getBodyByName(dst->graph, src->pair[i]->b1->name);
      }

      if (src->pair[i]->b2 != NULL)
      {
        dst->pair[i]->b2 =
          RcsGraph_getBodyByName(dst->graph, src->pair[i]->b2->name);
      }

      dst->pair[i]->graph = dst->graph;
      dst->pair[i]->weight = src->pair[i]->weight;
      dst->pair[i]->dThreshold = src->pair[i]->dThreshold;
      dst->pair[i]->distance = src->pair[i]->distance;
      dst->pair[i]->cp1 = MatNd_getRowPtr(dst->cp, 2*i);
      dst->pair[i]->cp2 = MatNd_getRowPtr(dst->cp, 2*i+1);
      dst->pair[i]->n1  = MatNd_getRowPtr(dst->n1, i);
    }

  }   // if(src->pair)

  return dst;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool RcsCollisionMdl_getPointers(const RcsCollisionMdl* self,
                                 const RcsBody* b1, const double** cp1,
                                 const RcsBody* b2, const double** cp2,
                                 const double** n1, const double** d)
{
  bool success = false;

  if (self == NULL)
  {
    NLOG(3, "Collision model is NULL");
    return false;
  }

  if (self->pair == NULL)
  {
    NLOG(3, "Collision model has no pairs!");
    return false;
  }

  if (b1 == NULL)
  {
    RLOG(3, "Body 1 is NULL");
    return false;
  }

  if (b2 == NULL)
  {
    RLOG(3, "Body 2 is NULL");
    return false;
  }

  RLOG(5, "Getting distance pointers between \"%s\" and \"%s\"",
       b1 ? b1->name : "NULL", b2 ? b2->name : "NULL");

  RCSPAIR_TRAVERSE(self->pair)
  {
    if ((PAIR->b1 == b1) && (PAIR->b2 == b2))
    {
      NLOG(5, "Getting distance pointers between \"%s\" and \"%s\""
           " in correct order", PAIR->b1->name, PAIR->b2->name);
      success = true;
      *cp1 = (PAIR->cp1);
      *cp2 = (PAIR->cp2);
    }
    else if ((PAIR->b1 == b2) && (PAIR->b2 == b1))
    {
      NLOG(5, "Getting distance pointers between \"%s\" and \"%s\""
           " in reverse order", PAIR->b1->name, PAIR->b2->name);
      *cp1 = (PAIR->cp2);
      *cp2 = (PAIR->cp1);
    }
    else
    {
      continue;
    }

    *d = &PAIR->distance;
    *n1  = (PAIR->n1);
    RCHECK(*cp1);
    RCHECK(*cp2);
    RCHECK(*n1);
    success = true;
  }

  return success;
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

  int count = 0;
  RCSPAIR_TRAVERSE(self->pair)
  {
    if (PAIR->distance < d)
    {
      d = PAIR->distance;
      if (pairIdx)
      {
        *pairIdx = count;
      }
    }

    count++;
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

  RCSPAIR_TRAVERSE(self->pair)
  {
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
      const double* center1 = PAIR->b1 ? PAIR->b1->A_BI->org : Vec3d_zeroVec();
      const double* center2 = PAIR->b2 ? PAIR->b2->A_BI->org : Vec3d_zeroVec();
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

  RCSPAIR_TRAVERSE(self->pair)
  {
    const double dLimit = PAIR->dThreshold;
    const double d  = PAIR->distance;
    const double w  = PAIR->weight;

    // If the distance is larger than the threshold, the gradient is zero.
    if (d > dLimit)
    {
      continue;
    }

    bool repelling = (d >= 0.0) ? true : false;
    double dgpDdp;

    RcsBody_distanceGradient(self->graph, PAIR->b1, PAIR->b2, repelling,
                             PAIR->cp1, PAIR->cp2, PAIR->n1, dDpdq);

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
      dCenters = RcsBody_centerDistance(PAIR->b1, PAIR->b2, NULL, NULL,
                                        I_p1_center, I_p2_center);

      RcsBody_distanceGradient(self->graph, PAIR->b1, PAIR->b2, true,
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

  // Count pairs of self
  unsigned int nPairs1 = 0;

  if (self->pair != NULL)
  {
    for (RcsPair** pPtr = self->pair; *pPtr; pPtr++)
    {
      nPairs1++;
    }
  }

  // Count pairs of other
  unsigned int nPairs2 = 0;

  if (other->pair != NULL)
  {
    for (RcsPair** pPtr = other->pair; *pPtr; pPtr++)
    {
      nPairs2++;
    }
  }

  if (nPairs1 != nPairs2)
  {
    RLOGS(1, "Models have different number of pairs: %d != %d",
          nPairs1, nPairs2);
    isEqual = false;
  }

  // Check if pairs are the same (only if the same number of pairs exists)
  if (nPairs1 == nPairs2)
  {
    for (unsigned int i=0; i<nPairs1; i++)
    {
      RcsPair* p1 = self->pair[i];
      RcsPair* p2 = other->pair[i];
      bool isPairEqual = RcsPair_isEqual(p1, p2, eps);

      if (isPairEqual==false)
      {
        isEqual = false;
      }
      else
      {
        NLOGS(1, "Pair \"%s\" - \"%s\" is equal",
              p1->b1 ? p1->b1->name : "NULL",
              p1->b2 ? p1->b2->name : "NULL");
      }

    }

  }


  return isEqual;
}

/*******************************************************************************
 *
 ******************************************************************************/
void RcsPair_fprint(FILE* fd, const RcsPair* self)
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
    fprintf(fd, "The pair is NULL\n");
    return;
  }

  fprintf(fd, "Pair \"%s\" - \"%s\":\n",
          self->b1 ? self->b1->name : "NULL",
          self->b2 ? self->b2->name : "NULL");
  fprintf(fd, "\tdThreshold = %.6f ", self->dThreshold);
  fprintf(fd, "weight = %.6f ", self->weight);
  fprintf(fd, "d = %.6f\n", self->distance);
  fprintf(fd, "\tcp1 = %.6f %.6f %.6f\n", self->cp1[0], self->cp1[1], self->cp1[2]);
  fprintf(fd, "\tcp2 = %.6f %.6f %.6f\n", self->cp2[0], self->cp2[1], self->cp2[2]);
  fprintf(fd, "\tn1  = %.6f %.6f %.6f\n", self->n1[0],  self->n1[1],  self->n1[2]);
}

/*******************************************************************************
 *
 ******************************************************************************/
void RcsPair_printCollisionModel(FILE* fd, RcsPair** pPtr)
{
  if (!fd)
  {
    RLOG(1, "You are trying to print a collision model to a NULL file "
         "descriptor - ignoring");
    return;
  }

  fprintf(fd, "[%s: %s(%d)]: ", __FILE__, __FUNCTION__, __LINE__);

  if (pPtr)
  {
    fprintf(fd, "The collision model consists of\n");

    int k = 0;
    while (*pPtr)
    {
      fprintf(fd, "\t[%d]   %s", k, (*pPtr)->b1->name);

      if (STRNEQ((*pPtr)->b1->name, "GenericBody", 11))
      {
        RcsBody* gPtr = (RcsBody*)(*pPtr)->b1->extraInfo;
        fprintf(fd, " (points to \"%s\")", gPtr ? gPtr->name : "NULL");
      }

      fprintf(fd, "\t\t%s", (*pPtr)->b2->name);

      if (STRNEQ((*pPtr)->b2->name, "GenericBody", 11))
      {
        RcsBody* gPtr = (RcsBody*)(*pPtr)->b2->extraInfo;
        fprintf(fd, " (points to \"%s\")", gPtr ? gPtr->name : "NULL");
      }

      fprintf(fd, "\n");

      pPtr++;
      k++;
    }
  }
  else
  {
    fprintf(fd, "No collision defined!\n");
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
bool RcsPair_isEqual(const RcsPair* p1, const RcsPair* p2, double eps)
{
  bool isPairEqual = true;

  const char* nameP11 = p1->b1 ? p1->b1->name : "NULL";
  const char* nameP12 = p1->b2 ? p1->b2->name : "NULL";
  const char* nameP21 = p2->b1 ? p2->b1->name : "NULL";
  const char* nameP22 = p2->b2 ? p2->b2->name : "NULL";

  if (!STREQ(nameP11, nameP21))
  {
    RLOGS(2, "Names of first bodies differ: \"%s\" != \"%s\"", nameP11, nameP21);
    isPairEqual = false;
  }

  if (!STREQ(nameP12, nameP22))
  {
    RLOGS(2, "Names of second bodies differ: \"%s\" != \"%s\"", nameP12, nameP22);
    isPairEqual = false;
  }

  if (fabs(p1->dThreshold - p2->dThreshold) > eps)
  {
    RLOGS(2, "Pair \"%s\" - \"%s\": Distance threshold differs: %g != %g",
          nameP11, nameP12, p1->dThreshold, p2->dThreshold);
    isPairEqual = false;
  }

  if (fabs(p1->weight - p2->weight) > eps)
  {
    RLOGS(2, "Pair \"%s\" - \"%s\": Distance weight differs: %g != %g",
          nameP11, nameP12, p1->weight, p2->weight);
    isPairEqual = false;
  }

  if (fabs(p1->distance - p2->distance) > eps)
  {
    RLOGS(2, "Pair \"%s\" - \"%s\": Distance differs: %g != %g",
          nameP11, nameP12, p1->distance, p2->distance);
    isPairEqual = false;
  }

  if (VecNd_isEqual(p1->cp1, p2->cp1, 3, eps)==false)
  {
    RLOGS(2, "Pair \"%s\" - \"%s\": cp1 differ: [%f %f %f] != [%f %f %f] err = %g",
          nameP11, nameP12,
          p1->cp1[0], p1->cp1[1], p1->cp1[2],
          p2->cp1[0], p2->cp1[1], p2->cp1[2],
          Vec3d_sqrDistance(p1->cp1, p2->cp1));
    isPairEqual = false;
  }

  if (VecNd_isEqual(p1->cp2, p2->cp2, 3, eps)==false)
  {
    RLOGS(2, "Pair \"%s\" - \"%s\": cp2 differ: [%f %f %f] != [%f %f %f] err = %g",
          nameP11, nameP12,
          p1->cp2[0], p1->cp2[1], p1->cp2[2],
          p2->cp2[0], p2->cp2[1], p2->cp2[2],
          Vec3d_sqrDistance(p1->cp2, p2->cp2));
    isPairEqual = false;

    RLOGS(3, "Pair \"%s\" - \"%s\": d1 = %f d(cp1) = %f    d2 = %f d(cp2) = %f",
          nameP11, nameP12,
          p1->distance, Vec3d_distance(p1->cp1, p1->cp2),
          p2->distance, Vec3d_distance(p2->cp1, p2->cp2));
  }

  if (VecNd_isEqual(p1->n1, p2->n1, 3, eps)==false)
  {
    RLOGS(2, "Pair \"%s\" - \"%s\": normals differ: [%f %f %f] != [%f %f %f] err = %g",
          nameP11, nameP12,
          p1->n1[0], p1->n1[1], p1->n1[2],
          p2->n1[0], p2->n1[1], p2->n1[2],
          Vec3d_sqrDistance(p1->n1, p2->n1));
    isPairEqual = false;
  }


  return isPairEqual;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool RcsCollisionMdl_removePair(RcsCollisionMdl* self,
                                const RcsPair* pair)
{

  if (self->pair == NULL)
  {
    RLOGS(2, "Can't remove pair \"%s\" - \"%s\" from collision model: "
          "Has no pairs",
          pair->b1 ? pair->b1->name : "NULL",
          pair->b2 ? pair->b2->name : "NULL");
    return false;
  }

  // Find matching pair and its index
  int nPairs = 0, pairIdx = -1;

  RCSPAIR_TRAVERSE(self->pair)
  {
    if (PAIR == pair)
    {
      pairIdx = nPairs;
    }

    nPairs++;
  }

  if (pairIdx == -1)
  {
    RLOGS(2, "Pair \"%s\" - \"%s\" not found in collision model",
          pair->b1 ? pair->b1->name : "NULL",
          pair->b2 ? pair->b2->name : "NULL");
    return false;
  }


  // Remove the detected pair
  RFREE(self->pair[pairIdx]);


  // Last pair needs special treatment: The pointers don't need to
  // be shifted, it is enough to just remove the last pair and to
  // resize the closest points and normals arrays.
  if (pairIdx == nPairs-1)
  {
    self->pair[pairIdx] = NULL;
  }
  else
  {
    // Pair at the beginning or in the middle found
    for (int i=pairIdx; i<nPairs-1; i++)
    {
      self->pair[i] = self->pair[i+1];

      self->pair[i]->cp1 = MatNd_getRowPtr(self->cp, 2*i);
      Vec3d_copy(self->pair[i]->cp1, MatNd_getRowPtr(self->cp, 2*i+2));

      self->pair[i]->cp2 = MatNd_getRowPtr(self->cp, 2*i+1);
      Vec3d_copy(self->pair[i]->cp2, MatNd_getRowPtr(self->cp, 2*i+3));

      self->pair[i]->n1 = MatNd_getRowPtr(self->n1, i);
      Vec3d_copy(self->pair[i]->n1, MatNd_getRowPtr(self->n1, i+1));
    }

    self->pair[nPairs-1] = NULL;
  }

  // Shrink the closest point and normal arrays to the correct size
  MatNd_reshape(self->cp, 2*(nPairs-1), 3);
  MatNd_reshape(self->n1, nPairs-1, 3);

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool RcsCollisionMdl_setPairWeightByName(RcsCollisionMdl* self,
                                         const char* bdy1, const char* bdy2,
                                         double weight)
{

  if ((self==NULL) || (self->pair==NULL) || (bdy1==NULL) || (bdy2==NULL))
  {
    RLOGS(4, "NULL argument found - doing nothing");
    return false;
  }

  RcsBody* b1 = RcsGraph_getBodyByName(self->graph, bdy1);

  if (b1 == NULL)
  {
    RLOGS(4, "Body \"%s\" not found in the collision mdoel",
          bdy1 ? bdy1 : "NULL");
    return false;
  }

  RcsBody* b2 = RcsGraph_getBodyByName(self->graph, bdy1);

  if (b2 == NULL)
  {
    RLOGS(4, "Body \"%s\" not found in the collision mdoel",
          bdy2 ? bdy2 : "NULL");
    return false;
  }

  RCSPAIR_TRAVERSE(self->pair)
  {
    if (((b1==PAIR->b1) && (b2==PAIR->b2)) ||
        ((b1==PAIR->b2) && (b2==PAIR->b1)))
    {
      PAIR->weight = weight;
    }
  }

  return true;
}
