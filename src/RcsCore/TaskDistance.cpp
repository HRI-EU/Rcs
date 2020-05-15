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

#include "TaskDistance.h"
#include "TaskFactory.h"
#include "Rcs_typedef.h"
#include "Rcs_macros.h"
#include "Rcs_parser.h"
#include "Rcs_body.h"
#include "Rcs_Vec3d.h"


static Rcs::TaskFactoryRegistrar<Rcs::TaskDistance> registrar1("Distance");


/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::TaskDistance::TaskDistance(const std::string& className_,
                                xmlNode* node,
                                RcsGraph* _graph,
                                int dim):
  TaskGenericIK(className_, node, _graph, dim), gainDX(1.0)
{
  if (getClassName()=="Distance")
  {
    double guiMax = 1.0, guiMin = -1.0;
    getXMLNodePropertyDouble(node, "guiMax", &guiMax);
    getXMLNodePropertyDouble(node, "guiMin", &guiMin);
    resetParameter(Parameters(guiMin, guiMax, 1.0, "Distance [m]"));
  }

  // The gainDX scales the position error coming out of computeDX. It is
  // sometimes needed to scale it down to avoid jitter due to the contact
  // normal updates.
  getXMLNodePropertyDouble(node, "gainDX", &this->gainDX);
}

/*******************************************************************************
 * Copy constructor doing deep copying
 ******************************************************************************/
Rcs::TaskDistance::TaskDistance(const TaskDistance& copyFromMe,
                                RcsGraph* newGraph):
  TaskGenericIK(copyFromMe, newGraph),
  gainDX(copyFromMe.gainDX)
{
}

/*******************************************************************************
 * Constructor based on body pointers
 ******************************************************************************/
Rcs::TaskDistance::TaskDistance(RcsGraph* graph_,
                                const RcsBody* effector,
                                const RcsBody* refBdy) :
  TaskGenericIK(), gainDX(1.0)

{
  int nQueries = RcsBody_getNumDistanceQueries(effector, refBdy);
  RCHECK(graph_);
  RCHECK(effector);
  RCHECK(refBdy);
  RCHECK_MSG(nQueries>0, "The body pair %s - %s has no distance query. Did "
             "you include any shape with enabled distance flag?",
             effector->name, refBdy->name);

  this->graph = graph_;
  setClassName("Distance");
  setName("Distance " + std::string(effector ? effector->name : "NULL") + "-"
          + std::string(refBdy ? refBdy->name : "NULL"));
  setEffector(effector);
  setRefBody(refBdy);
  setRefFrame(refFrame ? refFrame : refBdy);
  setDim(1);
  resetParameter(Parameters(-1.0, 1.0, 1.0, "Distance [m]"));
}

/*******************************************************************************
 * Destructor
 ******************************************************************************/
Rcs::TaskDistance::~TaskDistance()
{
}

/*******************************************************************************
 * Clone function
 ******************************************************************************/
Rcs::TaskDistance* Rcs::TaskDistance::clone(RcsGraph* newGraph) const
{
  return new Rcs::TaskDistance(*this, newGraph);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskDistance::computeX(double* x_res) const
{
  x_res[0] = RcsBody_distance(this->refBody, this->ef, NULL, NULL, NULL);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskDistance::computeDX(double* dx_ik,
                                  const double* x_des,
                                  const double* x_curr) const
{
  TaskGenericIK::computeDX(dx_ik, x_des, x_curr);
  dx_ik[0] *= gainDX;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskDistance::computeJ(MatNd* jacobian) const
{
  double cpEf[3], cpRef[3], nRE[3];
  RcsBody_distance(this->refBody, this->ef, cpRef, cpEf, nRE);
  RcsBody_distanceGradient(graph, this->refBody, this->ef, true,
                           cpRef, cpEf, nRE, jacobian);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskDistance::computeH(MatNd* hessian) const
{
  double cpEf[3], cpRef[3], nRE[3];
  RcsBody_distance(this->refBody, this->ef, cpRef, cpEf, nRE);
  MatNd_reshapeAndSetZero(hessian, graph->nJ, graph->nJ);
  RcsBody_distanceHessian(graph, this->refBody, this->ef, true,
                          cpRef, cpEf, hessian->ele);
}

/*******************************************************************************
 * See header
 ******************************************************************************/
bool Rcs::TaskDistance::testHessian(bool verbose)
{
  RLOG(4, "Skipping Hessian test for task \"%s\"", getName().c_str());
  return true;
}

/*******************************************************************************
 * This task required an effector and a reference body.
 ******************************************************************************/
bool Rcs::TaskDistance::isValid(xmlNode* node, const RcsGraph* graph)
{
  bool success = Rcs::Task::isValid(node, graph, "Distance");
  success = Rcs::TaskDistance::hasDistanceFunction(node, graph) && success;

  return success;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::TaskDistance::hasDistanceFunction(xmlNode* node,
                                            const RcsGraph* graph)
{
  bool success = true;
  char taskName[256] = "Unnamed task";
  getXMLNodePropertyStringN(node, "name", taskName, 256);

  // Check if there is a distance function called between effector and refBdy
  char name1[265] = "", name2[265] = "";
  getXMLNodePropertyStringN(node, "effector", name1, 256);
  getXMLNodePropertyStringN(node, "refBdy", name2, 256);
  getXMLNodePropertyStringN(node, "refBody", name2, 256);
  const RcsBody* b1 = RcsGraph_getBodyByName(graph, name1);
  const RcsBody* b2 = RcsGraph_getBodyByName(graph, name2);

  if (b1 == NULL)
  {
    RLOG(1, "Effector \"%s\" of task \"%s\" doesn't exist!", name1, taskName);
    success = false;
  }
  else if (b2 == NULL)
  {
    RLOG(1, "Ref-body \"%s\" of task \"%s\" doesn't exist!", name2, taskName);
    success = false;
  }
  else if (RcsBody_getNumDistanceQueries(b1, b2) == 0)
  {
    RLOG(1, "Task \"%s\": No distance function between \"%s\" and \"%s\"!",
         taskName, name1, name2);
    success = false;
  }

  return success;
}
