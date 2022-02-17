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

#include "TaskPositionTarget3D.h"
#include "TaskFactory.h"
#include "Rcs_typedef.h"
#include "Rcs_body.h"
#include "Rcs_macros.h"
#include "Rcs_parser.h"
#include "Rcs_utils.h"
#include "Rcs_math.h"
#include "Rcs_kinematics.h"

namespace Rcs
{

static TaskFactoryRegistrar<TaskPositionTarget3D> registrar("XYZ_TARGET");


/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
TaskPositionTarget3D::TaskPositionTarget3D(const std::string& className_,
                                           xmlNode* node,
                                           const RcsGraph* _graph,
                                           int dim):
  TaskPosition3D(className_, node, _graph, dim)
{
}

/*******************************************************************************
 * Clone function
 ******************************************************************************/
TaskPositionTarget3D* TaskPositionTarget3D::clone(const RcsGraph* newGraph) const
{
  TaskPositionTarget3D* task = new TaskPositionTarget3D(*this);
  task->setGraph(newGraph);
  return task;
}

/*******************************************************************************
 * The 3-d task variable is (distance 0 0)^T
 ******************************************************************************/
void TaskPositionTarget3D::computeX(double* I_r) const
{
  HTr A_goal = computeGoalRotation();
  Vec3d_set(I_r, Vec3d_getLength(A_goal.org), 0.0, 0.0);
}


/*******************************************************************************
 * Computes the delta in task space for the differential kinematics
 ******************************************************************************/
void TaskPositionTarget3D::computeDX(double* dx,
                                     const double* x_des,
                                     const double* x_curr) const
{
  double d_des = x_des[0];

  if (d_des < 0.0)
  {
    d_des = 0.0;
  }

  dx[0] = d_des - x_curr[0];
  dx[1] = 0.0;
  dx[2] = 0.0;
}

/*******************************************************************************
 * Computes the current velocity in task space:
 *
 * ref_xp = A_ref-I * (I_xp_ef - I_xp_ref)
 ******************************************************************************/
void TaskPositionTarget3D::computeXp_ik(double* xp_res) const
{
  TaskPosition3D::computeXp_ik(xp_res);
}

/*******************************************************************************
 * Rotate Jacobian into frame that points from effector to refBdy.
 * Construct a transform with origin being the refBdy, and the rotation matrix
 * with x-axis pointing from end-effector to refBdy. All coordinates are
 * represented in world coordinates.
 ******************************************************************************/
void TaskPositionTarget3D::computeJ(MatNd* jacobian) const
{
  HTr A_goal = computeGoalRotation();
  RcsGraph_bodyPointJacobian(graph, getEffector(), NULL, A_goal.rot, jacobian);
}

/*******************************************************************************
 *
 ******************************************************************************/
HTr TaskPositionTarget3D::computeGoalRotation() const
{
  HTr A_goal;
  Vec3d_sub(A_goal.org, getEffector()->A_BI.org, getRefBody()->A_BI.org);
  Mat3d_fromVec(A_goal.rot, A_goal.org, 0);

  return A_goal;
}

/*******************************************************************************
 * Computes current task Hessian to parameter hessian. See
 * RcsGraph_3dPosHessian() for details.
 ******************************************************************************/
void TaskPositionTarget3D::computeH(MatNd* hessian) const
{
  RLOG(0, "Fixme");
  TaskPosition3D::computeH(hessian);
}

/*******************************************************************************
 * Function wrapper for task kinematics, used in the finite difference
 * Jacobian approximation.
 ******************************************************************************/
static void calcTaskKinematics(MatNd* x, const MatNd* q, void* params)
{
  TaskPositionTarget3D* task = (TaskPositionTarget3D*)params;
  RcsGraph* graph = RcsGraph_clone(task->getGraph());
  RcsGraph_setState(graph, q, NULL);

  TaskPositionTarget3D* taskCpy = task->clone(graph);
  taskCpy->computeX(x->ele);

  delete taskCpy;
  RcsGraph_destroy(graph);
}

/*******************************************************************************
 * See header
 ******************************************************************************/
bool TaskPositionTarget3D::testJacobian(double errorLimit,
                                        double delta,
                                        bool relativeError,
                                        bool verbose) const
{
  MatNd* J = MatNd_create(3, getGraph()->nJ);
  MatNd* J_fd = MatNd_create(3, getGraph()->nJ);
  computeJ(J);
  Math_finiteDifferenceDerivative(J_fd, calcTaskKinematics, (void*)this,
                                  getGraph()->q, delta);

  MatNd_setRowZero(J, 1);
  MatNd_setRowZero(J, 2);

  REXEC(2)
  {
    MatNd_printCommentDigits("J", J, 3);
    MatNd_printCommentDigits("J_fd", J_fd, 3);
  }

  MatNd_subSelf(J, J_fd);
  double err = MatNd_maxAbsEle(J);
  bool success = err<errorLimit;

  if (verbose)
  {
    RLOG(1, "%s: Max. error is %g", success ? "SUCCESS" : "FAILURE", err);
  }

  MatNd_destroy(J);
  MatNd_destroy(J_fd);

  return success;
}

/*******************************************************************************
 * Function wrapper for task Jacobian, used in the finite difference
 * Hessian approximation.
 ******************************************************************************/
static void calcTaskJacobian(MatNd* J, const MatNd* q, void* params)
{
  TaskPositionTarget3D* task = (TaskPositionTarget3D*)params;
  RcsGraph* graph = RcsGraph_clone(task->getGraph());
  RcsGraph_setState(graph, q, NULL);

  TaskPositionTarget3D* taskCpy = task->clone(graph);
  taskCpy->computeJ(J);
  MatNd_reshape(J, 3*taskCpy->getGraph()->nJ, 1);
  delete taskCpy;
  RcsGraph_destroy(graph);
}

bool TaskPositionTarget3D::testHessian(bool verbose) const
{
  return true;
  MatNd* H = MatNd_create(3*getGraph()->nJ, getGraph()->nJ);
  MatNd* H_fd = MatNd_create(3*getGraph()->nJ, getGraph()->nJ);
  computeH(H);

  // This makes a Hessian that is aligned the following way in ths memory:
  // J is m x n
  // q is n x 1
  //
  // dJ00/dq0 dJ00/dq1 dJ00/dqn
  // dJ01/dq0 dJ01/dq1 dJ01/dqn
  // dJ02/dq0 dJ02/dq1 dJ02/dqn
  // ...      ...      ...
  // dJmn/dq0 dJmn/dq1 dJmn/dqn
  //
  Math_finiteDifferenceDerivative(H_fd, calcTaskJacobian, (void*)this,
                                  getGraph()->q, 1.0e-8);




  // MatNd_reshape(H_fd, getGraph()->nJ, 3*getGraph()->nJ);
  // MatNd_transposeSelf(H_fd);

  REXEC(2)
  {
    MatNd_printCommentDigits("H", H, 3);
    MatNd_printCommentDigits("H_fd", H_fd, 3);
  }

  MatNd_subSelf(H, H_fd);
  double err = MatNd_maxAbsEle(H);
  bool success = err<1.0e-6;

  if (verbose)
  {
    RLOG(1, "%s: Max. error is %g", success ? "SUCCESS" : "FAILURE", err);
  }

  MatNd_destroy(H);
  MatNd_destroy(H_fd);

  return success;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool TaskPositionTarget3D::isValid(xmlNode* node, const RcsGraph* graph)
{
  bool success = Task::isValid(node, graph, "XYZ_TARGET");

  char tag[RCS_MAX_NAMELEN] = "";
  char taskName[RCS_MAX_NAMELEN];

  strcpy(taskName, "unnamed task");
  getXMLNodePropertyStringN(node, "name", taskName, RCS_MAX_NAMELEN);


  unsigned int len = getXMLNodePropertyStringN(node, "refFrame", tag,
                                               RCS_MAX_NAMELEN);

  if (len > 0)
  {
    RLOG(1, "Task \"%s\": Must not have a refFrame", taskName);
    success = false;
  }

  len = getXMLNodePropertyStringN(node, "refBdy", tag, RCS_MAX_NAMELEN);
  len += getXMLNodePropertyStringN(node, "refBody", tag, RCS_MAX_NAMELEN);

  if (len==0)
  {
    RLOG(1, "Task \"%s\": Must have a refBdy or refBody attribute", taskName);
    return false;
  }

  const RcsBody* refBdy = RcsGraph_getBodyByName(graph, tag);

  if (refBdy == NULL)
  {
    RLOG(1, "Task \"%s\": Couldn't find reference body with name \"%s\"",
         taskName, tag);
    return false;
  }

  len = getXMLNodePropertyStringN(node, "effector", tag, RCS_MAX_NAMELEN);

  if (len == 0)
  {
    RLOG(1, "Task \"%s\": Must have an effector attribute", taskName);
    return false;
  }

  if (RcsBody_isArticulated(graph, refBdy))
  {
    RLOG(1, "Task \"%s\": Goal body \"%s\" is articulated - not supported",
         taskName, tag);
    return false;
  }

  return success;
}

}   // namespace Rcs
