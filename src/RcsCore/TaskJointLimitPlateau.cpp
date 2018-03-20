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

#include "TaskJointLimitPlateau.h"
#include "TaskFactory.h"
#include "Rcs_typedef.h"
#include "Rcs_macros.h"
#include "Rcs_parser.h"
#include "Rcs_kinematics.h"



static Rcs::TaskFactoryRegistrar<Rcs::TaskJointLimitPlateau> registrar("JointLimitPlateau");




/******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::TaskJointLimitPlateau::TaskJointLimitPlateau(const std::string& className,
                                                  xmlNode* node,
                                                  RcsGraph* _graph,
                                                  int dim):
  TaskGenericIK(className, node, _graph, dim),
  borderRatio(0.05)
{
  getParameter(0)->setParameters(0.0, 1.0, 1.0, "sum(sqr(d))");
}

/*******************************************************************************
 * Copy constructor doing deep copying
 ******************************************************************************/
Rcs::TaskJointLimitPlateau::TaskJointLimitPlateau(const Rcs::TaskJointLimitPlateau& copyFromMe,
                                                  RcsGraph* newGraph):
  TaskGenericIK(copyFromMe, newGraph)
{
  this->borderRatio = copyFromMe.borderRatio;
}

/*******************************************************************************
 * Destructor
 ******************************************************************************/
Rcs::TaskJointLimitPlateau::~TaskJointLimitPlateau()
{
}

/*******************************************************************************
 * Clone function
 ******************************************************************************/
Rcs::TaskJointLimitPlateau* Rcs::TaskJointLimitPlateau::clone(RcsGraph* newGraph) const
{
  return new Rcs::TaskJointLimitPlateau(*this, newGraph);
}

/*******************************************************************************
 * \brief Computes the sum of the squared distances of the collision
 *        model.
 ******************************************************************************/
void Rcs::TaskJointLimitPlateau::computeX(double* x_res) const
{
  *x_res = RcsGraph_jointLimitCostPlateau(this->graph, this->borderRatio,
                                          RcsStateIK);
}


/*******************************************************************************
 * \brief Computes current task Jacobian to parameter \param jacobian
 ******************************************************************************/
void Rcs::TaskJointLimitPlateau::computeJ(MatNd* dH) const
{
  MatNd_reshape(dH, this->graph->nJ, 1);
  RcsGraph_jointLimitGradientPlateau(this->graph, dH, this->borderRatio,
                                     RcsStateIK);
  MatNd_reshape(dH, 1, this->graph->nJ);
}

/****************************************************************************
 * \brief see header
 ***************************************************************************/
void Rcs::TaskJointLimitPlateau::computeH(MatNd* hessian) const
{
  RFATAL("Hessian computation for task \"%s\" not yet implemented!",
         getName().c_str());
}

/*******************************************************************************
 * \brief
 ******************************************************************************/
double Rcs::TaskJointLimitPlateau::getBorderRatio() const
{
  return this->borderRatio;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::TaskJointLimitPlateau::isValid(xmlNode* node, const RcsGraph* graph)
{
  return Rcs::Task::isValid(node, graph, "JointLimitPlateau");
}
