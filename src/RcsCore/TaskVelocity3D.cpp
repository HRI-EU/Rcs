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

#include "TaskVelocity3D.h"
#include "TaskFactory.h"
#include "Rcs_typedef.h"
#include "Rcs_macros.h"
#include "Rcs_parser.h"
#include "Rcs_utils.h"
#include "Rcs_VecNd.h"
#include "Rcs_Vec3d.h"
#include "Rcs_kinematics.h"



static Rcs::TaskFactoryRegistrar<Rcs::TaskVelocity3D> registrar("XYZd");



/*******************************************************************************
 * Constructor based on graph and effectors.
 ******************************************************************************/
Rcs::TaskVelocity3D::TaskVelocity3D(RcsGraph* graph,
                                    const RcsBody* effector,
                                    const RcsBody* refBdy,
                                    const RcsBody* refFrame,
                                    float scaleFactor):
  TaskPosition3D(graph, effector, refBdy, refFrame)
{
  setClassName("XYZd");

  resetParameter(Parameters(-10., 10., scaleFactor, "X Velocity [m/s]"));
  addParameter(Parameters(-1.0, 1.0, scaleFactor, "Y Velocity [m/s]"));
  addParameter(Parameters(-1.0, 1.0, scaleFactor, "Z Velocity [m/s]"));

  Vec3d_setZero(velocity_des_temp);
}

/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::TaskVelocity3D::TaskVelocity3D(const std::string& taskType,
                                    xmlNode* node,
                                    RcsGraph* _graph,
                                    int dim,
                                    float scaleFactor):
  TaskPosition3D(taskType, node, _graph, dim)
{

  if (getClassName()=="XYZd")
  {
    resetParameter(Parameters(-10., 10., scaleFactor, "X Velocity [m/s]"));
    addParameter(Parameters(-1.0, 1.0, scaleFactor, "Y Velocity [m/s]"));
    addParameter(Parameters(-1.0, 1.0, scaleFactor, "Z Velocity [m/s]"));
  }

  Vec3d_setZero(velocity_des_temp);
}

/*******************************************************************************
 * Copy constructor doing deep copying
 ******************************************************************************/
Rcs::TaskVelocity3D::TaskVelocity3D(const TaskVelocity3D& copyFromMe,
                                    RcsGraph* newGraph):
  TaskPosition3D(copyFromMe, newGraph)
{
  Vec3d_copy(velocity_des_temp, copyFromMe.velocity_des_temp);
}

/*******************************************************************************
 * Destructor
 ******************************************************************************/
Rcs::TaskVelocity3D::~TaskVelocity3D()
{
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::TaskVelocity3D* Rcs::TaskVelocity3D::clone(RcsGraph* newGraph) const
{
  return new Rcs::TaskVelocity3D(*this, newGraph);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskVelocity3D::computeX(double* x_res) const
{
  double temp[3];
  computeXp(temp);
  Vec3d_copy(x_res, temp);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskVelocity3D::computeDX(double* dx, const double* x_des) const
{
  //  Vec3d_copy(const_cast<Rcs::TaskVelocity3D*>(this)->velocity_des_temp, x_des);
  //  Vec3d_setZero(dx);
  Vec3d_copy(dx, x_des);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskVelocity3D::computeDXp(double* dxp_res,
                                     const double* desired_vel) const
{
  //  Vec3d_copy(const_cast<double*>(desired_vel), this->velocity_des_temp);
  Rcs::TaskPosition3D::computeDXp(dxp_res, desired_vel);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::TaskVelocity3D::isValid(xmlNode* node, const RcsGraph* graph)
{
  std::vector<std::string> classNameVec;
  classNameVec.push_back(std::string("XYZd"));

  bool success = Rcs::Task::isValid(node, graph, classNameVec);

  return success;
}
