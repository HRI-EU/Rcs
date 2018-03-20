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

#include "TaskVelocity1D.h"
#include "TaskFactory.h"
#include "Rcs_typedef.h"
#include "Rcs_macros.h"
#include "Rcs_parser.h"
#include "Rcs_utils.h"
#include "Rcs_VecNd.h"
#include "Rcs_kinematics.h"



static Rcs::TaskFactoryRegistrar<Rcs::TaskVelocity1D> registrar1("Xd");
static Rcs::TaskFactoryRegistrar<Rcs::TaskVelocity1D> registrar2("Yd");
static Rcs::TaskFactoryRegistrar<Rcs::TaskVelocity1D> registrar3("Zd");



/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::TaskVelocity1D::TaskVelocity1D(const std::string& taskType,
                                    xmlNode* node,
                                    RcsGraph* _graph,
                                    int dim):
  TaskPosition1D(taskType, node, _graph, dim),
  velocity_des_temp(0.0)
{

  if (taskType=="Xd")
  {
    getParameter(0)->setParameters(-10., 10., 1.0, "X Velocity [m/s]");
    this->index = 0;
  }
  else if (taskType=="Yd")
  {
    getParameter(0)->setParameters(-1.0, 1.0, 1.0, "Y Velocity [m/s]");
    this->index = 1;
  }
  else if (taskType=="Zd")
  {
    getParameter(0)->setParameters(-1.0, 1.0, 1.0, "Z Velocity [m/s]");
    this->index = 2;
  }

}

/*******************************************************************************
 * Copy constructor doing deep copying
 ******************************************************************************/
Rcs::TaskVelocity1D::TaskVelocity1D(const TaskVelocity1D& copyFromMe,
                                    RcsGraph* newGraph):
  TaskPosition1D(copyFromMe, newGraph),
  velocity_des_temp(copyFromMe.velocity_des_temp)
{
}

/*******************************************************************************
 * Destructor
 ******************************************************************************/
Rcs::TaskVelocity1D::~TaskVelocity1D()
{
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::TaskVelocity1D* Rcs::TaskVelocity1D::clone(RcsGraph* newGraph) const
{
  return new Rcs::TaskVelocity1D(*this, newGraph);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskVelocity1D::computeX(double* x_res) const
{
  double temp;
  computeXp(&temp);
  *x_res = temp;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskVelocity1D::computeDX(double* dx, const double* x_des) const
{
  const_cast<Rcs::TaskVelocity1D*>(this)->velocity_des_temp = x_des[0];
  *dx = 0.;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskVelocity1D::computeDXp(double* dxp_res,
                                     const double* desired_vel) const
{
  const_cast<double*>(desired_vel)[0] = this->velocity_des_temp;
  Rcs::TaskPosition1D::computeDXp(dxp_res, desired_vel);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::TaskVelocity1D::isValid(xmlNode* node, const RcsGraph* graph)
{
  std::vector<std::string> classNameVec;
  classNameVec.push_back(std::string("Xd"));
  classNameVec.push_back(std::string("Yd"));
  classNameVec.push_back(std::string("Zd"));

  bool success = Rcs::Task::isValid(node, graph, classNameVec);

  return success;
}
