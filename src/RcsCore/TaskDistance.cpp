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
#include "Rcs_kinematics.h"


static Rcs::TaskFactoryRegistrar<Rcs::TaskDistance> registrar1("Distance");


/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::TaskDistance::TaskDistance(const std::string& className_,
                                xmlNode* node,
                                RcsGraph* _graph,
                                int dim):
  TaskGenericIK(className_, node, _graph, dim)
{
  getParameter(0)->name.assign("Distance [m]");
}

/*******************************************************************************
 * Copy constructor doing deep copying
 ******************************************************************************/
Rcs::TaskDistance::TaskDistance(const TaskDistance& copyFromMe,
                                RcsGraph* newGraph):
  TaskGenericIK(copyFromMe, newGraph)
{
}

/*******************************************************************************
 * Constructor based on body pointers
 ******************************************************************************/
//! \todo Memory leak when derieved class calls params.clear()
Rcs::TaskDistance::TaskDistance(RcsGraph* graph_,
                                const RcsBody* effector,
                                const RcsBody* refBdy) : TaskGenericIK()
{
  this->graph = graph_;
  setClassName("Distance");
  setName("Distance " + std::string(effector ? effector->name : "NULL") + "-"
          + std::string(refBdy ? refBdy->name : NULL));
  setEffector(effector);
  setRefBody(refBdy);
  setRefFrame(refFrame ? refFrame : refBdy);
  setDim(1);
  std::vector<Parameters*>& params = getParameters();
  params.clear();
  params.push_back(new Task::Parameters(-1.0, 1.0, 1.0, "Distance [m]"));
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
void Rcs::TaskDistance::computeJ(MatNd* jacobian) const
{
  double cpEf[3], cpRef[3], nRE[3];
  RcsBody_distance(this->refBody, this->ef, cpRef, cpEf, nRE);
  RcsBody_distanceGradient(graph, this->refBody, this->ef, true,
                           cpRef, cpEf, nRE, jacobian);
  MatNd_reshape(jacobian, 1, graph->nJ); // \todo(MG): Fix in RcsBody_distanceGradient() function
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
  std::vector<std::string> classNameVec;
  classNameVec.push_back(std::string("Distance"));

  bool success = Rcs::Task::isValid(node, graph, classNameVec);

  char taskName[256] = "Unnamed task";
  getXMLNodePropertyStringN(node, "name", taskName, 256);

  // This task requires an effector and a reference body
  if (getXMLNodeProperty(node, "effector")==false)
  {
    RLOG(3, "Task \"%s\" requires \"effector\", none found", taskName);
    success = false;
  }

  if ((getXMLNodeProperty(node, "refBdy")==false) &&
      (getXMLNodeProperty(node, "refBody")==false))
  {
    RLOG(3, "Task \"%s\" requires \"refBdy\" or \"refBody\", none found",
         taskName);
    success = false;
  }

  return success;
}
