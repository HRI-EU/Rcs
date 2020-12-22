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

#include "TaskPositionTarget3D.h"
#include "TaskFactory.h"
#include "Rcs_typedef.h"
#include "Rcs_body.h"
#include "Rcs_macros.h"
#include "Rcs_parser.h"
#include "Rcs_utils.h"
#include "Rcs_math.h"
#include "Rcs_kinematics.h"


static Rcs::TaskFactoryRegistrar<Rcs::TaskPositionTarget3D> registrar("XYZ_TARGET");


/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::TaskPositionTarget3D::TaskPositionTarget3D(const std::string& className_,
                                                xmlNode* node,
                                                RcsGraph* _graph,
                                                int dim):
  TaskPosition3D(className_, node, _graph, dim)
{
  this->goalBdy = createBody();
  this->refFrame = this->goalBdy;
}

/*******************************************************************************
 * Copy constructor doing deep copying
 ******************************************************************************/
Rcs::TaskPositionTarget3D::TaskPositionTarget3D(const TaskPositionTarget3D& src):
  TaskPosition3D(src)
{
  this->goalBdy = createBody();
  this->refFrame = this->goalBdy;
}

/*******************************************************************************
 * Copy constructor doing deep copying
 ******************************************************************************/
Rcs::TaskPositionTarget3D::TaskPositionTarget3D(const TaskPositionTarget3D& src,
                                                RcsGraph* newGraph):
  TaskPosition3D(src, newGraph)
{
  this->goalBdy = createBody();
  this->refFrame = this->goalBdy;
}

/*******************************************************************************
 * Destructor
 ******************************************************************************/
Rcs::TaskPositionTarget3D::~TaskPositionTarget3D()
{
  RcsBody_destroy(this->goalBdy);
}

/*******************************************************************************
 * Clone function
 ******************************************************************************/
Rcs::TaskPositionTarget3D* Rcs::TaskPositionTarget3D::clone(RcsGraph* newGraph) const
{
  TaskPositionTarget3D* cpy = new TaskPositionTarget3D(*this, newGraph);
  cpy->updateRefBody();

  return cpy;
}

/*******************************************************************************
 * Set refBody transformation to point from reference towards effector
 ******************************************************************************/
void Rcs::TaskPositionTarget3D::updateRefBody() const
{
  // Origin is in reference body
  const double* refPos = refBody ? refBody->A_BI.org : Vec3d_zeroVec();
  Vec3d_copy(goalBdy->A_BI.org, refPos);

  // Orientation (x-axis of frame) points towards effector
  double dir[3];
  Vec3d_sub(dir, this->ef->A_BI.org, refPos);
  Mat3d_fromVec(goalBdy->A_BI.rot, dir, 0);
}

/*******************************************************************************
 * Computes the current value of the task variable
 *
 *  1. Local effector reference vector is added to effector origin:
 *
 *       I_r = A_IK * K_ef_r
 *
 *  2. If a reference body exists, subtract its origin and rotate
 *     the result into the reference bodies's basis:
 *
 *       ref_r = A_ref-I * (I_r - I_r_refBdy)
 ******************************************************************************/
void Rcs::TaskPositionTarget3D::computeX(double* I_r) const
{
  updateRefBody();

  double pos[3];
  TaskPosition3D::computeX(pos);

  Vec3d_set(I_r, Vec3d_getLength(pos), 0.0, 0.0);
}


/*******************************************************************************
 * Computes the delta in task space for the differential kinematics
 ******************************************************************************/
void Rcs::TaskPositionTarget3D::computeDX(double* dx,
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
void Rcs::TaskPositionTarget3D::computeXp_ik(double* xp_res) const
{
  updateRefBody();
  TaskPosition3D::computeXp_ik(xp_res);
}

/*******************************************************************************
 * Computes current task Jacobian to parameter jacobian. See
 * RcsGraph_3dPosJacobian() for details.
 ******************************************************************************/
void Rcs::TaskPositionTarget3D::computeJ(MatNd* jacobian) const
{
  updateRefBody();
  TaskPosition3D::computeJ(jacobian);
}

/*******************************************************************************
 * Computes current task Hessian to parameter hessian. See
 * RcsGraph_3dPosHessian() for details.
 ******************************************************************************/
void Rcs::TaskPositionTarget3D::computeH(MatNd* hessian) const
{
  updateRefBody();
  TaskPosition3D::computeH(hessian);
}

/*******************************************************************************
 * Computes current task Hessian to parameter hessian. See
 * RcsGraph_3dPosHessian() for details.
 ******************************************************************************/
RcsBody* Rcs::TaskPositionTarget3D::createBody()
{
  RcsBody* body = RcsBody_create();

  snprintf(body->bdyName, RCS_MAX_NAMELEN, "%s", "TaskPositionTarget3D::refFrame");

  return body;
}

/*******************************************************************************
 * Function wrapper for task kinematics, used in the finite difference
 * Jacobian approximation.
 ******************************************************************************/
static void calcTaskKinematics(MatNd* x, const MatNd* q, void* params)
{
  Rcs::TaskPositionTarget3D* task = (Rcs::TaskPositionTarget3D*)params;
  RcsGraph* graph = RcsGraph_clone(task->getGraph());
  RcsGraph_setState(graph, q, NULL);

  Rcs::TaskPositionTarget3D* taskCpy = task->clone(graph);
  taskCpy->computeX(x->ele);

  delete taskCpy;
  RcsGraph_destroy(graph);
}

/*******************************************************************************
 * See header
 ******************************************************************************/
bool Rcs::TaskPositionTarget3D::testJacobian(double errorLimit,
                                             double delta,
                                             bool relativeError,
                                             bool verbose)
{
  MatNd* J = MatNd_create(3, getGraph()->nJ);
  MatNd* J_fd = MatNd_create(3, getGraph()->nJ);
  computeJ(J);
  Math_finiteDifferenceDerivative(J_fd, calcTaskKinematics, this,
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
  Rcs::TaskPositionTarget3D* task = (Rcs::TaskPositionTarget3D*)params;
  RcsGraph* graph = RcsGraph_clone(task->getGraph());
  RcsGraph_setState(graph, q, NULL);

  Rcs::TaskPositionTarget3D* taskCpy = task->clone(graph);
  taskCpy->computeJ(J);
  MatNd_reshape(J, 3*taskCpy->getGraph()->nJ, 1);
  delete taskCpy;
  RcsGraph_destroy(graph);
}

bool Rcs::TaskPositionTarget3D::testHessian(bool verbose)
{
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
  Math_finiteDifferenceDerivative(H_fd, calcTaskJacobian, this,
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
bool Rcs::TaskPositionTarget3D::isValid(xmlNode* node, const RcsGraph* graph)
{
  bool success = Rcs::Task::isValid(node, graph, "XYZ_TARGET");

  char tag[256] = "";
  unsigned int len = getXMLNodePropertyStringN(node, "refFrame", tag, 256);

  if (len>0)
  {
    REXEC(3)
    {
      char taskName[256];
      strcpy(taskName, "unnamed task");
      getXMLNodePropertyStringN(node, "name", taskName, 256);
      RMSG("Task \"%s\": Must not have a refFrame", taskName);
    }
    success = false;
  }

  return success;
}
