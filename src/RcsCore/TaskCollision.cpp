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

#include "TaskCollision.h"
#include "TaskFactory.h"

#include <Rcs_typedef.h>
#include <Rcs_macros.h>
#include <Rcs_parser.h>
#include <Rcs_utils.h>
#include <Rcs_body.h>


static Rcs::TaskFactoryRegistrar<Rcs::TaskCollision> registrar("Collision");


/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::TaskCollision::TaskCollision(const std::string& className_,
                                  xmlNode* node,
                                  RcsGraph* graph_,
                                  int dim):
  TaskGenericIK(className_, node, graph_, dim), pair(NULL)
{
  getParameter(0)->setParameters(0.0, 1.0, 1.0, "sum(sqr(d))");

  // Parse XML file:
  char bdyName[256];

  // default distance threshold
  double defaultThreshold = 0.1;
  getXMLNodePropertyDouble(node, "DistanceThreshold", &defaultThreshold);
  getXMLNodePropertyDouble(node, "threshold", &defaultThreshold);
  NLOG(0, "Distance threshold is %g", defaultThreshold);

  // Count collision pairs and allocate memory
  xmlNodePtr child = node->children;
  int nPairs = 0;

  while (child)
  {
    if (isXMLNodeName(child, "CollisionPair"))
    {
      nPairs++;
    }
    child = child->next;
  }

  RCHECK_MSG(nPairs > 0, "Task \"%s\" has no collision pairs",
             getName().c_str());
  this->pair = new RcsPair*[nPairs + 1];
  memset(this->pair, 0, (nPairs + 1)*sizeof(RcsPair*));

  // Traverse collision pairs and assign settings
  child = node->children;
  nPairs = 0;

  while (child)
  {

    if (isXMLNodeName(child, "CollisionPair"))
    {
      this->pair[nPairs] = new RcsPair;
      memset(this->pair[nPairs], 0, sizeof(RcsPair));

      // body1
      int len = getXMLNodePropertyStringN(child, "body1", bdyName, 256);
      RCHECK_MSG(len, "\"body1\" not found in pair %d", nPairs);
      RcsBody* b = RcsGraph_getBodyByName(this->graph, bdyName);
      RCHECK_MSG(b, "Body \"%s\" doesn't exist!", bdyName);
      this->pair[nPairs]->b1 = b;

      // body2
      len = getXMLNodePropertyStringN(child, "body2", bdyName, 256);
      RCHECK_MSG(len, "\"body2\" not found in pair %d", nPairs);
      b = RcsGraph_getBodyByName(this->graph, bdyName);
      RCHECK_MSG(b, "Body \"%s\" doesn't exist!", bdyName);
      this->pair[nPairs]->b2 = b;

      // other pair parameters
      this->pair[nPairs]->dThreshold = defaultThreshold;
      this->pair[nPairs]->weight = 1.0;
      getXMLNodePropertyDouble(child, "DistanceThreshold",
                               &this->pair[nPairs]->dThreshold);
      getXMLNodePropertyDouble(child, "threshold",
                               &this->pair[nPairs]->dThreshold);
      getXMLNodePropertyDouble(child, "weight",
                               &this->pair[nPairs]->weight);
      REXEC(1)
      {
        RMSG("Pair %d: \"%s\" - \"%s\"", nPairs,
             this->pair[nPairs]->b1->name,
             this->pair[nPairs]->b2->name);
        RMSG("DistanceThreshold = %g", this->pair[nPairs]->dThreshold);
        RMSG("Distance weight is %g", this->pair[nPairs]->weight);
      }

      nPairs++;
    }   // if(isXMLNodeName(child, "CollisionPair"))

    child = child->next;

  } // while(child)

}

/*******************************************************************************
 * Copy constructor doing deep copying
 ******************************************************************************/
Rcs::TaskCollision::TaskCollision(const TaskCollision& copyFromMe):
  TaskGenericIK(copyFromMe), pair(NULL)
{
  copyCollisionModel(copyFromMe.pair, graph);
}

/*******************************************************************************
 * Copy constructor doing deep copying
 ******************************************************************************/
Rcs::TaskCollision::TaskCollision(const TaskCollision& copyFromMe,
                                  RcsGraph* newGraph):
  TaskGenericIK(copyFromMe, newGraph), pair(NULL)
{
  copyCollisionModel(copyFromMe.pair, newGraph);
}

/*******************************************************************************
 * Copy collision model
 ******************************************************************************/
void Rcs::TaskCollision::copyCollisionModel(RcsPair** srcPair,
                                            const RcsGraph* newGraph)
{
  if (srcPair == NULL)
  {
    return;
  }

  // Count number of pairs
  unsigned int nPairs = 0;
  for (RcsPair** pPtr = srcPair; *pPtr; (void)*pPtr++)
  {
    nPairs++;
  }

  // Allocate memory
  this->pair = new RcsPair*[nPairs + 1];
  memset(this->pair, 0, (nPairs + 1)*sizeof(RcsPair*));

  // Clone pairs
  nPairs = 0;
  RCSPAIR_TRAVERSE(srcPair)
  {
    this->pair[nPairs] = new RcsPair;
    memset(this->pair[nPairs], 0, sizeof(RcsPair));
    this->pair[nPairs]->weight = PAIR->weight;
    this->pair[nPairs]->dThreshold = PAIR->dThreshold;
    this->pair[nPairs]->distance = PAIR->distance;
    if (newGraph)
    {
      this->pair[nPairs]->graph = newGraph;
      this->pair[nPairs]->b1 = RcsGraph_getBodyByName(newGraph, PAIR->b1->name);
      this->pair[nPairs]->b2 = RcsGraph_getBodyByName(newGraph, PAIR->b2->name);
    }
    else
    {
      this->pair[nPairs]->graph = PAIR->graph;
      this->pair[nPairs]->b1 = PAIR->b1;
      this->pair[nPairs]->b2 = PAIR->b2;
    }
    nPairs++;
  }
}

/*******************************************************************************
 * Destructor
 ******************************************************************************/
Rcs::TaskCollision::~TaskCollision()
{
  // Delete all collision pairs
  if (this->pair)
  {
    int i = 0;
    while (this->pair[i])
    {
      delete this->pair[i++];
    }
    delete [] this->pair;
  }
}

/*******************************************************************************
 * Clone function
 ******************************************************************************/
Rcs::TaskCollision* Rcs::TaskCollision::clone(RcsGraph* newGraph) const
{
  return new TaskCollision(*this, newGraph);
}

/*******************************************************************************
 * Computes the sum of the squared distances of the collision model.
 ******************************************************************************/
void Rcs::TaskCollision::computeX(double* x_res) const
{
  *x_res = 0.0;

  RCSPAIR_TRAVERSE(this->pair)
  {
    *x_res += PAIR->weight * RcsBody_collisionCost(PAIR->b1, PAIR->b2,
                                                   PAIR->dThreshold);
  }

}

/*******************************************************************************
 * Computes current task Jacobian to parameter \param jacobian
 ******************************************************************************/
void Rcs::TaskCollision::computeJ(MatNd* dH) const
{
  MatNd_reshape(dH, this->graph->nJ, 1);
  MatNd_setZero(dH);
  MatNd* dHi = NULL;
  MatNd_create2(dHi, dH->m, dH->n);
  double status;

  RCSPAIR_TRAVERSE(this->pair)
  {
    RcsBody_collisionGradient(this->graph, PAIR->b1, PAIR->b2,
                              PAIR->dThreshold, dHi, &status);
    MatNd_constMulSelf(dHi, PAIR->weight);   // HACK
    MatNd_addSelf(dH, dHi);
  }

  // Apply the weight factor for each joint
  RCSGRAPH_TRAVERSE_JOINTS(this->graph)
  {
    if (JNT->jacobiIndex != -1)
    {
      dH->ele[JNT->jacobiIndex] *= JNT->weightCA;
    }
  }

  MatNd_destroy(dHi);
  MatNd_reshape(dH, 1, this->graph->nJ);
}

/*******************************************************************************
 * \brief see header
 ******************************************************************************/
void Rcs::TaskCollision::computeH(MatNd* hessian) const
{
  RFATAL("Hessian computation for task \"%s\" not yet implemented!",
         getName().c_str());
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::TaskCollision::isValid(xmlNode* node, const RcsGraph* graph)
{
  return Rcs::Task::isValid(node, graph, "Collision");
}
