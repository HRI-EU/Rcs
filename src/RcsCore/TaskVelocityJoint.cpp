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

#include "TaskVelocityJoint.h"
#include "TaskFactory.h"
#include "Rcs_typedef.h"
#include "Rcs_macros.h"
#include "Rcs_parser.h"
#include "Rcs_utils.h"
#include "Rcs_VecNd.h"
#include "Rcs_joint.h"
#include "Rcs_kinematics.h"



static Rcs::TaskFactoryRegistrar<Rcs::TaskVelocityJoint> registrar("Jointd");



/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::TaskVelocityJoint::TaskVelocityJoint(const std::string& className,
                                          xmlNode* node,
                                          const RcsGraph* _graph,
                                          int dim):
  TaskJoint(className, node, _graph, dim)
{
  // re-initialize parameters
  if (getClassName()=="Jointd")
  {
    if (RcsJoint_isTranslation(getJoint()) == true)
    {
      resetParameter(Parameters(-1.0, 1.0, 1.0, "Vel. [m]"));
    }
    else
    {
      resetParameter(Parameters(-90.0, 90.0, (180.0/M_PI), "Vel. [deg/s]"));
    }
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::TaskVelocityJoint* Rcs::TaskVelocityJoint::clone(const RcsGraph* newGraph) const
{
  TaskVelocityJoint* task = new Rcs::TaskVelocityJoint(*this);
  task->setGraph(newGraph);
  return task;
}

/*******************************************************************************
 * Task variable is the joint velocity
 ******************************************************************************/
void Rcs::TaskVelocityJoint::computeX(double* x_res) const
{
  x_res[0] = MatNd_get(this->graph->q_dot, this->jointId, 0);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskVelocityJoint::computeDX(double* dx, const double* x_des) const
{
  *dx = *x_des;
}

/*******************************************************************************
 * TODO: Do we need it? I think it does the same as the parent class
 ******************************************************************************/
void Rcs::TaskVelocityJoint::computeDXp(double* delta_q_dot,
                                        const double* q_dot_des) const
{
  double q_dot_curr = MatNd_get(this->graph->q_dot, this->jointId, 0);
  *delta_q_dot = *q_dot_des - q_dot_curr;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::TaskVelocityJoint::isValid(xmlNode* node, const RcsGraph* graph)
{
  bool success = Rcs::Task::isValid(node, graph, "Jointd");

  char taskName[RCS_MAX_NAMELEN];
  strcpy(taskName, "unnamed task");
  getXMLNodePropertyStringN(node, "name", taskName, RCS_MAX_NAMELEN);

  // Check if joint exists
  char tag[RCS_MAX_NAMELEN] = "";
  int len = getXMLNodePropertyStringN(node, "jnt", tag, RCS_MAX_NAMELEN);

  if (len==0)
  {
    RLOG(3, "Task \"%s\": No tag \"jnt\" found in xml file", taskName);
    success = false;
  }

  RcsJoint* joint = RcsGraph_getJointByName(graph, tag);

  if (joint==NULL)
  {
    RLOG(3, "Task \"%s\": Joint \"%s\" not found", taskName, tag);
    success = false;
  }

  return success;
}
