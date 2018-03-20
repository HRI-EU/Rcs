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

#include "TaskOmega3D.h"
#include "TaskFactory.h"
#include "Rcs_typedef.h"
#include "Rcs_macros.h"
#include "Rcs_kinematics.h"
#include "Rcs_VecNd.h"
#include "Rcs_Vec3d.h"
#include "Rcs_parser.h"


static Rcs::TaskFactoryRegistrar<Rcs::TaskOmega3D> registrar("ABCd");

/*******************************************************************************
 * Constructor based on graph and effectors.
 ******************************************************************************/
Rcs::TaskOmega3D::TaskOmega3D(RcsGraph* graph, const RcsBody* effector,
                              const RcsBody* refBdy, const RcsBody* refFrame):
  TaskGenericIK()
{
  this->graph = graph;
  setDim(3);
  setClassName("ABCd");

  setEffector(effector);
  setRefBody(refBdy);
  setRefFrame(refFrame ? refFrame : refBdy);

  getParameters().clear();
  getParameters().push_back(new Parameters(-M_PI, M_PI, (180.0/M_PI), "Ad [deg/sec]"));
  getParameters().push_back(new Parameters(-M_PI, M_PI, (180.0/M_PI), "Bd [deg/sec]"));
  getParameters().push_back(new Parameters(-M_PI, M_PI, (180.0/M_PI), "Cd [deg/sec]"));

  Vec3d_setZero(this->omega_des_temp);
}

/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::TaskOmega3D::TaskOmega3D(const std::string& className_,
                              xmlNode* node,
                              RcsGraph* _graph):
  TaskGenericIK(className_, node, _graph, 3)
{

  getParameter(0)->setParameters(-M_PI, M_PI, (180.0/M_PI), "Ad [deg/sec]");
  getParameter(1)->setParameters(-M_PI, M_PI, (180.0/M_PI), "Bd [deg/sec]");
  getParameter(2)->setParameters(-M_PI, M_PI, (180.0/M_PI), "Cd [deg/sec]");

  // Other tasks inherit from this. Therefore we can't do any hard checking
  // on classNames etc.

  // init temp vec
  Vec3d_setZero(this->omega_des_temp);
}

/*******************************************************************************
 * Copy constructor doing deep copying
 ******************************************************************************/
Rcs::TaskOmega3D::TaskOmega3D(const TaskOmega3D& copyFromMe,
                              RcsGraph* newGraph):
  TaskGenericIK(copyFromMe, newGraph)
{
  Vec3d_copy(this->omega_des_temp, copyFromMe.omega_des_temp);
}

/*******************************************************************************
 * Destructor
 ******************************************************************************/
Rcs::TaskOmega3D::~TaskOmega3D()
{
}

/*******************************************************************************
 * Clone function
 ******************************************************************************/
Rcs::TaskOmega3D* Rcs::TaskOmega3D::clone(RcsGraph* newGraph) const
{
  return new Rcs::TaskOmega3D(*this, newGraph);
}

/*******************************************************************************
 * Omega Jacobian.
 ******************************************************************************/
void Rcs::TaskOmega3D::computeJ(MatNd* jacobian) const
{
  RcsGraph_3dOmegaJacobian(this->graph, this->ef, this->refBody,
                           this->refFrame, jacobian);
}

/*******************************************************************************
 * Omega Jacobian.
 ******************************************************************************/
void Rcs::TaskOmega3D::computeH(MatNd* hessian) const
{
  RcsGraph_3dOmegaHessian(this->graph, this->ef, this->refBody, this->refFrame,
                          hessian);
}

/*******************************************************************************
 * Computes the current value of the task variable: XYZ Euler angles
 ******************************************************************************/
void Rcs::TaskOmega3D::computeX(double* x_res) const
{
  double temp[3];
  this->computeXp(temp);
  Vec3d_copy(x_res, temp);
}

/*******************************************************************************
 * Computes the delta in task space for the differential kinematics. That's a
 * bit hacky. Shouldn't we better have a function to set the desired velocities?
 ******************************************************************************/
void Rcs::TaskOmega3D::computeDX(double* dx, const double* x_des) const
{
  //  Vec3d_copy(const_cast<TaskOmega3D*>(this)->omega_des_temp, x_des);
  //  Vec3d_setZero(dx);
  Vec3d_copy(dx, x_des);
}


/*******************************************************************************
 * Computes the velocity error
 ******************************************************************************/
void Rcs::TaskOmega3D::computeDXp(double* dxp_res,
                                  const double* desired_vel) const
{
  //  Vec3d_copy(const_cast<double*>(desired_vel), this->omega_des_temp);
  Rcs::TaskGenericIK::computeDXp(dxp_res, desired_vel);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::TaskOmega3D::isValid(xmlNode* node, const RcsGraph* graph)
{
  std::vector<std::string> classNameVec;
  classNameVec.push_back(std::string("ABCd"));

  bool success = Rcs::Task::isValid(node, graph, classNameVec);

  return success;
}
