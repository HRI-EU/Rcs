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

#include "TaskJoint.h"
#include "TaskFactory.h"
#include "Rcs_typedef.h"
#include "Rcs_macros.h"
#include "Rcs_parser.h"
#include "Rcs_joint.h"
#include "Rcs_VecNd.h"


static Rcs::TaskFactoryRegistrar<Rcs::TaskJoint> registrar("Joint");


/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::TaskJoint::TaskJoint(const std::string& className,
                          xmlNode* node,
                          RcsGraph* _graph,
                          int dim):
  TaskGenericIK(className, node, _graph, dim),
  jointId(-1), refJointId(-1), refGain(1.0)
{
  // Parse XML file
  char msg[RCS_MAX_NAMELEN] = "";

  // joint
  getXMLNodePropertyStringN(node, "jnt", msg, RCS_MAX_NAMELEN);
  const RcsJoint* joint = RcsGraph_getJointByName(this->graph, msg);
  RCHECK_MSG(joint, "Joint \"%s\" doesn't exist!", msg);
  this->jointId = joint->id;

  // We require the joint to be unconstrained. Otherwise, we'll have a zero
  // Jacobian.
  RCHECK_MSG(joint->constrained==false, "Joint \"%s\" is constrained", msg);

  // ref-joint
  if (getXMLNodeProperty(node, "refJnt"))
  {
    getXMLNodePropertyStringN(node, "refJnt", msg, RCS_MAX_NAMELEN);
    const RcsJoint* refJoint = RcsGraph_getJointByName(this->graph, msg);
    RCHECK_MSG(refJoint, "Joint \"%s\" doesn't exist!", msg);
    this->refJointId = refJoint->id;

    // We require the joint to be unconstrained. Otherwise, we'll have a zero
    // Jacobian.
    RCHECK_MSG(refJoint->constrained==false, "Ref-joint \"%s\" is "
               "constrained", msg);

    // Coupling factor between joints, default is 1
    getXMLNodePropertyDouble(node, "refGain", &this->refGain);
  }



  // re-initialize parameters
  if (getClassName() == "Joint")
  {
    bool isRot = RcsJoint_isRotation(joint);
    double xml2SI = isRot ? M_PI / 180.0 : 1.0;

    // Convert to deg / m units before xml parsing, since the values in the xml
    // file are degrees and meters.
    double guiMax = joint->q_max / xml2SI;
    double guiMin = joint->q_min / xml2SI;
    getXMLNodePropertyDouble(node, "guiMax", &guiMax);
    getXMLNodePropertyDouble(node, "guiMin", &guiMin);

    // Convert from xml units back to SI units.
    guiMax *= xml2SI;
    guiMin *= xml2SI;

    bool hide = false;
    getXMLNodePropertyBoolString(node, "hide", &hide);
    if (hide)
    {
      guiMin = 0.0;
      guiMax = 0.0;
    }
    std::string label = std::string(joint->name) + (isRot ? " [deg]" : " [m]");
    resetParameter(Parameters(guiMin, guiMax, 1.0 / xml2SI, label));
  }

}

/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::TaskJoint::TaskJoint(const RcsJoint* joint,
                          const RcsJoint* refJoint,
                          xmlNode* node,
                          RcsGraph* _graph,
                          double refGain_):
  TaskGenericIK("Joint", node, _graph, 1),
  jointId(joint ? joint->id : -1),
  refJointId(refJoint ? refJoint->id : -1),
  refGain(refGain_)
{
  RCHECK(joint);

  // We require the joint to be unconstrained. Otherwise, we'll have a zero
  // Jacobian.
  RCHECK_MSG(joint->constrained==false, "Joint \"%s\" is constrained",
             joint->name);

  // re-initialize parameters
  if (RcsJoint_isTranslation(joint) == true)
  {
    std::string label = std::string(joint->name) + " [m]";
    resetParameter(Parameters(joint->q_min, joint->q_max, 1.0, label));
  }
  else
  {
    std::string label = std::string(joint->name) + " [deg]";
    resetParameter(Parameters(joint->q_min, joint->q_max, (180.0/M_PI), label));
  }
}

/*******************************************************************************
 *
******************************************************************************/
Rcs::TaskJoint::TaskJoint(RcsGraph* graph_, const RcsJoint* joint,
                          const RcsJoint* refJoint, double refGain_) :
  TaskGenericIK(),
  jointId(joint ? joint->id : -1),
  refJointId(refJoint ? refJoint->id : -1),
  refGain(refGain_)
{
  // We require the joint to be unconstrained. Otherwise, we'll have a zero
  // Jacobian.
  RCHECK(joint);
  RCHECK_MSG(joint->constrained == false, "Joint \"%s\" is constrained",
             joint->name);

  this->graph = graph_;
  setClassName("Joint");
  setName("Jnt " + std::string(joint->name));
  setDim(1);

  // re-initialize parameters. That's all done a bit weird and needs revision.
  if (RcsJoint_isTranslation(joint) == true)
  {
    std::string label = std::string(joint->name) + " [m]";
    resetParameter(Task::Parameters(joint->q_min, joint->q_max, 1.0, label));
  }
  else
  {
    std::string label = std::string(joint->name) + " [deg]";
    resetParameter(Task::Parameters(joint->q_min, joint->q_max, 180.0/M_PI, label));
  }

}

/*******************************************************************************
 * Copy constructor doing deep copying
 ******************************************************************************/
Rcs::TaskJoint::TaskJoint(const TaskJoint& copyFromMe, RcsGraph* newGraph):
  TaskGenericIK(copyFromMe, newGraph),
  jointId(copyFromMe.jointId),
  refJointId(copyFromMe.refJointId),
  refGain(copyFromMe.refGain)
{
}

/*******************************************************************************
 * Destructor
 ******************************************************************************/
Rcs::TaskJoint::~TaskJoint()
{
}

/*******************************************************************************
 * Clone function
 ******************************************************************************/
Rcs::TaskJoint* Rcs::TaskJoint::clone(RcsGraph* newGraph) const
{
  return new Rcs::TaskJoint(*this, newGraph);
}

/*******************************************************************************
 * Computes the current value of the task variable: Copies the current joint
 * angle to the task variable.
 ******************************************************************************/
void Rcs::TaskJoint::computeX(double* x_res) const
{
  x_res[0] = MatNd_get(this->graph->q, this->jointId, 0);

  if (this->refJointId != -1)
  {
    x_res[0] += refGain*MatNd_get(this->graph->q, this->refJointId, 0);
  }
}

/*******************************************************************************
 * Computes the current value of the task variable: Copies the current joint
 * angle to the task variable.
 ******************************************************************************/
void Rcs::TaskJoint::computeXp(double* x_dot_res) const
{
  x_dot_res[0] = MatNd_get(graph->q_dot, this->jointId, 0);

  if (this->refJointId != -1)
  {
    x_dot_res[0] += refGain*MatNd_get(graph->q_dot, this->refJointId, 0);
  }
}

/*******************************************************************************
 * Computes current task Jacobian to parameter jacobian
 * This produces a jacobian with only one row of zeros,
 * with the column corresponding to the dof of the joint set to 1.
 ******************************************************************************/
void Rcs::TaskJoint::computeJ(MatNd* jacobian) const
{
  MatNd_reshapeAndSetZero(jacobian, 1, this->graph->nJ);
  MatNd_set(jacobian, 0, getJoint()->jacobiIndex,  1.0);

  if (this->refJointId != -1)
  {
    MatNd_set(jacobian, 0, getRefJoint()->jacobiIndex, refGain);
  }
}

/*******************************************************************************
 * Computes current task Hessian to parameter hessian. It is a zero matrix.
 ******************************************************************************/
void Rcs::TaskJoint::computeH(MatNd* hessian) const
{
  MatNd_reshapeAndSetZero(hessian, this->graph->nJ, this->graph->nJ);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
const RcsJoint* Rcs::TaskJoint::getJoint() const
{
  return &graph->joints[this->jointId];
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
int Rcs::TaskJoint::getJointIndex() const
{
  return getJoint()->jointIndex;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
const RcsJoint* Rcs::TaskJoint::getRefJoint() const
{
  return &graph->joints[this->refJointId];
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
double Rcs::TaskJoint::getRefGain() const
{
  return this->refGain;
}

/*******************************************************************************
 * Since those joints are mostly kinematic, we scale the gains to make sure
 * they don't get into oscillations.
 ******************************************************************************/
void Rcs::TaskJoint::computeAX(double* a_res,
                               double* integral_x,
                               const double* x_des,
                               const double* x_dot_des,
                               const double* x_ddot_des,
                               const double* S_des,
                               const double a_des,
                               const double kp,
                               const double kd,
                               const double ki) const
{
  double vel_des = 0.0;
  Rcs::TaskGenericIK::computeAX(a_res, integral_x, x_des, &vel_des, NULL,
                                S_des, a_des, 0.2*kp, kd, ki);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskJoint::print() const
{
  TaskGenericIK::print();

  const RcsJoint* joint = getJoint();

  if (joint)
  {
    printf("Joint: \"%s\" (type: %s value: %f)",
           joint->name, RcsJoint_typeName(joint->type),
           getGraph()->q->ele[joint->jointIndex]);
  }

  const RcsJoint* refJoint = getRefJoint();

  if (refJoint)
  {
    printf(" refJoint: \"%s\" (type: %s value: %f)",
           refJoint->name, RcsJoint_typeName(refJoint->type),
           getGraph()->q->ele[refJoint->jointIndex]);
  }

  if (refGain != 1.0)
  {
    printf(" refGain: %g", refGain);
  }

  printf("\n");

}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskJoint::setJoint(const RcsJoint* jnt)
{
  RCHECK(jnt);
  this->jointId = jnt->id;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskJoint::setRefJoint(const RcsJoint* jnt)
{
  this->refJointId = jnt ? jnt->id : -1;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskJoint::setRefGain(double gain)
{
  this->refGain = gain;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::TaskJoint::isValid(xmlNode* node, const RcsGraph* graph)
{
  bool success = Rcs::Task::isValid(node, graph, "Joint");

  char taskName[256];
  strcpy(taskName, "unnamed task");
  getXMLNodePropertyStringN(node, "name", taskName, 256);

  // Check if joint exists
  char tag[256] = "";
  int len = getXMLNodePropertyStringN(node, "jnt", tag, 256);

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

  // We require the joint to be unconstrained. Otherwise, we'll have a zero
  // Jacobian.
  if (joint && joint->constrained==true)
  {
    RLOG(3, "Task \"%s\": Joint \"%s\" is constrained",
         taskName, joint->name);
    success = false;
  }

  return success;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskJoint::toXMLBody(FILE* out) const
{
  if (getJoint())
  {
    fprintf(out, " jnt=\"%s\"", getJoint()->name);
  }

  if (getRefJoint())
  {
    fprintf(out, " refJnt=\"%s\"", getRefJoint()->name);
  }

  if (getRefGain())
  {
    fprintf(out, " refGain=\"%g\"", getRefGain());
  }

}
