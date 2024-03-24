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

/*
This task has 2 dimensions:
1. The distance between effector and the z-axis of the refBdy
2. The angle between the effector's x-z plane intersecting the refBdy's x-y
   plane, and the line connecting the effector with the refBdy's z-axis
*/
#include "TaskTangentCircle.h"
#include "TaskFactory.h"
#include "Rcs_typedef.h"
#include "Rcs_macros.h"
#include "Rcs_parser.h"
#include "Rcs_stlParser.h"
#include "Rcs_math.h"
#include "Rcs_kinematics.h"


namespace Rcs
{
// register task at the task factory
REGISTER_TASK(TaskTangentCircle, "TangentCircle");


/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
TaskTangentCircle::TaskTangentCircle(const std::string& className,
                                     xmlNode* node,
                                     const RcsGraph* _graph,
                                     int dim) :
  Task(className, node, _graph, dim)
{
  if (getClassName() == "TangentCircle")
  {
    //resetParameter(Parameters(0, 2.5, 1.0, "Radius [m]"));
    //addParameter(Parameters(-2.0 * M_PI, 2.0 * M_PI, 180.0 / M_PI, "Phi [deg]"));
    resetParameter(Parameters(-2.0 * M_PI, 2.0 * M_PI, 180.0 / M_PI, "Phi [deg]"));
  }
}

/*******************************************************************************
 * Clone function
 ******************************************************************************/
TaskTangentCircle* TaskTangentCircle::clone(const RcsGraph* newGraph) const
{
  TaskTangentCircle* task = new TaskTangentCircle(*this);
  task->setGraph(newGraph);
  return task;
}

/*******************************************************************************
 *
 ******************************************************************************/
void TaskTangentCircle::computeX(double* x) const
{
  const RcsBody* ef = getEffector();
  const RcsBody* refBody = getRefBody();
  //  const RcsBody* refFrame = getRefFrame();

  // Transform effector into refBdy frame: A_ER from (R)ef-body to (E)ffector
  HTr A_ER;
  computeRelativeRotationMatrix(A_ER.rot, ef, refBody);
  Vec3d_invTransform(A_ER.org, &refBody->A_BI, ef->A_BI.org);

  // First dimension is distance to z-axis.
  x[0] = VecNd_getLength(A_ER.org, 2);   // Just length of x-y projection

  // Second dimension is angle between  the effector's x-z plane intersecting
  // the refBdy's x-y plane, and the line connecting the effector with the
  // refBdy's z-axis.
  double dir_des[3];
  Vec3d_set(dir_des, -A_ER.org[0], -A_ER.org[1], 0.0);

  double dir_curr[3];
  int dirIdx = (fabs(A_ER.org[0]) < fabs(A_ER.org[2])) ? 0 : 2;
  Vec3d_set(dir_curr, A_ER.rot[dirIdx][0], A_ER.rot[dirIdx][1], 0.0);

  // Signed angle between two direction vectors.
  const double dot = VecNd_innerProduct(dir_des, dir_curr, 2); // dot product
  const double det = dir_des[0] * dir_curr[1] - dir_curr[0] * dir_des[1];      // Determinant
  x[0] = atan2(det, dot);
}

/*******************************************************************************
 * Computes the Polar angle velocity: phi_dot = H*R_om
 ******************************************************************************/
void TaskTangentCircle::computeXp(double* phip) const
{
}

/*******************************************************************************
 * Computes the Polar angle acceleration
 ******************************************************************************/
void TaskTangentCircle::computeXpp(double* phipp, const MatNd* q_ddot) const
{
  RLOG(4, "Implement TaskTangentCircle::computeXpp");
  VecNd_setZero(phipp, getDim());
}

/*******************************************************************************
 * Computes the Polar angle velocity delta
 ******************************************************************************/
void TaskTangentCircle::computeDXp(double* dOmega,
                                   const double* phip_des) const
{
  RFATAL("Not yet implemented");
}

/*******************************************************************************
 *
 *******************************************************************************/
void TaskTangentCircle::computeJ(MatNd* jacobian) const
{
  const RcsBody* ef = getEffector();
  const RcsBody* refBody = getRefBody();
  const RcsBody* refFrame = getRefFrame();

  MatNd* J_pos = NULL;
  MatNd_create2(J_pos, 3, graph->nJ);
  RcsGraph_3dPosJacobian(this->graph, ef, refBody, refFrame, J_pos);

  HTr A_ER;
  computeRelativeRotationMatrix(A_ER.rot, ef, refBody);
  Vec3d_invTransform(A_ER.org, &refBody->A_BI, ef->A_BI.org);

  const double* XYZ = A_ER.org;
  double dCyldCart[3][3];
  MatNd trafo = MatNd_fromPtr(3, 3, &dCyldCart[0][0]);
  Math_dCyldCart(dCyldCart, XYZ);

  MatNd_preMulSelf(J_pos, &trafo);   // First row is radial direction
  MatNd_copyRow(jacobian, 0, J_pos, 0);


  RcsGraph_3dOmegaJacobian(graph, ef, refBody, refFrame, J_pos);
  MatNd_constMulSelf(J_pos, -1.0);
  MatNd_copyRow(jacobian, 0, J_pos, 2);

  REXEC(1)
  {
    HTr_printComment("A_ER", &A_ER);
    Mat3d_printCommentDigits("dCyldCart", dCyldCart, 3);
    MatNd_printCommentDigits("J", jacobian, 2);
  }
  MatNd_destroy(J_pos);
}

/*******************************************************************************
 *
 ******************************************************************************/
void TaskTangentCircle::computeH(MatNd* hessian) const
{
  RFATAL("Not yet implemented");
}

/*******************************************************************************
 *  Computes the delta in task space for the differential kinematics
 ******************************************************************************/
void TaskTangentCircle::computeDX(double* dx, const double* x_des) const
{
  double x_curr[2];
  computeX(x_curr);
  VecNd_sub(dx, x_des, x_curr, getDim());
  dx[0] *= -1.0;
}

/*******************************************************************************
 *  Due to the lack of 1 dof in the task representation, this function cannot be
 *  implemented.
 ******************************************************************************/
void TaskTangentCircle::computeDX(double* dx_ik,
                                  const double* x_des,
                                  const double* x_curr) const
{
  RFATAL("Not yet implemented");
}

/*******************************************************************************
 *
 ******************************************************************************/
void TaskTangentCircle::integrateXp_ik(double* x_res, const double* x,
                                       const double* x_dot, double dt) const
{
  RFATAL("Not yet implemented");
}

/*******************************************************************************
 * Computes the acceleration in velocity / acceleration coordinates from the
 * accelerations in task coordinates. The velocity / acceleration level is
 * represented in body-fixed angular velocities (w_dot), the task level in Polar
 * coordinates (phi_ddot). The conversion is done as follows:
 *
 * w = H^-1*phi_dot
 * w_dot = d(H^-1)/dt phi_dot + (H^-1) phi_ddot
 ******************************************************************************/
void TaskTangentCircle::computeFfXpp(double* x_ddot_res,
                                     const double* desired_acc) const
{
  RFATAL("Implement me");
}

/*******************************************************************************
 * Transforms force from Jacobi coordinates to task coordinates.
 ******************************************************************************/
void TaskTangentCircle::forceTrafo(double* ft_task) const
{
  RFATAL("Implement me");
}

/*******************************************************************************
 * Transforms the selection into the Jacobian coordinates
*******************************************************************************/
void TaskTangentCircle::selectionTrafo(double* S_des_trafo,
                                       const double* S_des) const
{
  RFATAL("Implement me");
}

/*******************************************************************************
 * Finite difference tests with a bit different error limits.
 ******************************************************************************/
bool TaskTangentCircle::test(bool verbose) const
{
  bool success = true;
  RFATAL("Not yet implemented");
  return success;
}

/*******************************************************************************
 * We require effector and refBdy
 ******************************************************************************/
bool TaskTangentCircle::isValid(xmlNode* node, const RcsGraph* graph)
{
  bool success = Task::isValid(node, graph, "TangentCircle");

  std::string bdyName = getXMLNodePropertySTLString(node, "effector");

  if (bdyName.empty())
  {
    RLOG(3, "Task \"TaskTangentCircle\": effector not specified");
    success = false;
  }

  if (!RcsGraph_getBodyByName(graph, bdyName.c_str()))
  {
    RLOG_CPP(3, "Task \"TaskTangentCircle\": effector '" << bdyName
             << "' not found in graph");
    success = false;
  }

  bdyName = getXMLNodePropertySTLString(node, "refBdy");

  if (bdyName.empty())
  {
    RLOG(3, "Task \"TaskTangentCircle\": refBdy not specified");
    success = false;
  }

  if (!RcsGraph_getBodyByName(graph, bdyName.c_str()))
  {
    RLOG_CPP(3, "Task \"TaskTangentCircle\": refBdy '" << bdyName
             << "' not found in graph");
    success = false;
  }


  return success;
}

}   // namespace Rcs
