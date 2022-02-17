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

#include "TaskJointLimit.h"
#include "TaskFactory.h"
#include "Rcs_typedef.h"
#include "Rcs_macros.h"
#include "Rcs_parser.h"
#include "Rcs_kinematics.h"
#include "Rcs_basicMath.h"


static Rcs::TaskFactoryRegistrar<Rcs::TaskJointLimit> registrar("JointLimit");


/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::TaskJointLimit::TaskJointLimit(const std::string& className_,
                                    xmlNode* node,
                                    const RcsGraph* _graph,
                                    int dim):
  TaskGenericIK(className_, node, _graph, dim),
  borderRatio(0.0)
{
  // re-initialize parameters
  if (getClassName()=="JointLimit")
  {
    resetParameter(Parameters(0.0, 1.0, 1.0, "sum(sqr(d))"));
  }

  // Border ratio (default is 0)
  getXMLNodePropertyDouble(node, "borderRatio", &this->borderRatio);
  this->borderRatio = Math_clip(this->borderRatio, 0.0, 1.0);
}

/*******************************************************************************
 * Clone function
 ******************************************************************************/
Rcs::TaskJointLimit* Rcs::TaskJointLimit::clone(const RcsGraph* newGraph) const
{
  TaskJointLimit* task =  new TaskJointLimit(*this);
  task->setGraph(newGraph);
  return task;
}

/*******************************************************************************
 * Computes the sum of the squared distances of the collision model.
 ******************************************************************************/
void Rcs::TaskJointLimit::computeX(double* x_res) const
{
  *x_res = RcsGraph_jointLimitBorderCost(this->graph, this->borderRatio,
                                         RcsStateIK);
}

/*******************************************************************************
 * Computes current task Jacobian to parameter \param jacobian
 ******************************************************************************/
void Rcs::TaskJointLimit::computeJ(MatNd* dH) const
{
  MatNd_reshape(dH, this->graph->nJ, 1);
  RcsGraph_jointLimitBorderGradient(this->graph, dH, this->borderRatio,
                                    RcsStateIK);
  MatNd_reshape(dH, 1, this->graph->nJ);
}

/*******************************************************************************
 * Computes current task Hessian
 ******************************************************************************/
void Rcs::TaskJointLimit::computeH(MatNd* H) const
{
  RcsGraph_jointLimitBorderHessian(this->graph, H, this->borderRatio,
                                   RcsStateIK);
}

/*******************************************************************************
 * This task is always valid
 ******************************************************************************/
bool Rcs::TaskJointLimit::isValid(xmlNode* node, const RcsGraph* graph)
{
  bool success = Rcs::Task::isValid(node, graph, "JointLimit");

  double borderRatio = 0.0;

  getXMLNodePropertyDouble(node, "borderRatio", &borderRatio);

  if ((borderRatio<0.0) || (borderRatio>1.0))
  {
    RLOG(3, "Border ratio out of range: %f - should be within [0...1]",
         borderRatio);
    success = false;
  }

  return success;
}
