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

#include "TaskPolarTarget2D.h"
#include "TaskFactory.h"
#include "Rcs_typedef.h"
#include "Rcs_macros.h"
#include "Rcs_parser.h"
#include "Rcs_math.h"
#include "Rcs_kinematics.h"
#include "Rcs_joint.h"


// register task at the task factory
static Rcs::TaskFactoryRegistrar<Rcs::TaskPolarTarget2D> registrar("POLAR_TARGET");



/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::TaskPolarTarget2D::TaskPolarTarget2D(const std::string& className,
                                          xmlNode* node,
                                          RcsGraph* _graph,
                                          int dim) :
  Task(className, node, _graph, dim), direction(2)
{
  getParameter(0)->setParameters(-M_PI, M_PI, (180.0/M_PI), "Phi [deg]");

  if (getDim() >= 2)
  {
    getParameter(1)->setParameters(-M_PI, M_PI, (180.0/M_PI), "Theta [deg]");
  }

  // Parse axis direction (should be X, Y or Z)
  char text[256] = "Z";
  getXMLNodePropertyStringN(node, "axisDirection", text, 256);

  if (STRCASEEQ(text, "X"))
  {
    this->direction = 0;
  }
  else if (STRCASEEQ(text, "Y"))
  {
    this->direction = 1;
  }
  else if (STRCASEEQ(text, "Z"))
  {
    this->direction = 2;
  }

  this->polarDes[0] = 0.0;
  this->polarDes[1] = 0.0;
}

/*******************************************************************************
 * Copy constructor doing deep copying
 ******************************************************************************/
Rcs::TaskPolarTarget2D::TaskPolarTarget2D(const TaskPolarTarget2D& copyFromMe,
                                          RcsGraph* newGraph):
  Task(copyFromMe, newGraph), direction(copyFromMe.direction)
{
  this->polarDes[0] = copyFromMe.polarDes[0];
  this->polarDes[1] = copyFromMe.polarDes[1];
}

/*******************************************************************************
 * Destructor
 ******************************************************************************/
Rcs::TaskPolarTarget2D::~TaskPolarTarget2D()
{
}

/*******************************************************************************
 * Clone function
 ******************************************************************************/
Rcs::TaskPolarTarget2D* Rcs::TaskPolarTarget2D::clone(RcsGraph* newGraph) const
{
  return new Rcs::TaskPolarTarget2D(*this, newGraph);
}

/*******************************************************************************
  Computes the current value of the task variable: Polar angles
*******************************************************************************/
void Rcs::TaskPolarTarget2D::computeX(double* x_curr) const
{
  // Compute the desired polar axis
  double a_des[3];
  Vec3d_getPolarAxis(a_des, this->polarDes[0], this->polarDes[1]);

  // Compute the current polar axis
  double A_ER[3][3];
  computeRelativeRotationMatrix(A_ER, this->ef, this->refBody);
  const double* a_curr = A_ER[this->direction];

  x_curr[0] = Vec3d_diffAngle(a_curr, a_des);
  x_curr[1] = 0.0;
}

/*******************************************************************************
 * Computes the delta in task space for the differential kinematics
 ******************************************************************************/
void Rcs::TaskPolarTarget2D::computeDX(double* dx_ik,
                                       const double* x_des,
                                       const double* x_curr) const
{
  double phi_des = Math_clip(x_des[0], 0.0, M_PI);
  dx_ik[0] = x_curr[0] - phi_des;
  dx_ik[1] = 0.0;
}

/*******************************************************************************
 * Computes the Polar angle velocity
 ******************************************************************************/
void Rcs::TaskPolarTarget2D::computeXp(double* phip) const
{
  RFATAL("Implement me");
}

/*******************************************************************************
 * Computes the feed forward acceleration given a desired acceleration. The
 * argument x_ddot_res is expected to be in Jacobi-coordinate. In this
 * implementation, it's just a copy operation. But in some derieved classes,
 * it needs to be overwritten to account for the different coordinates of the
 * position and velocity levels.
 ******************************************************************************/
void Rcs::TaskPolarTarget2D::computeFfXpp(double* x_ddot_res,
                                          const double* desired_acc) const
{
  RFATAL("Implement me");
}

/*******************************************************************************
 * Computes the Polar angle velocity delta
 ******************************************************************************/
void Rcs::TaskPolarTarget2D::computeDXp(double* dOmega,
                                        const double* phip_des) const
{
  RFATAL("Implement me");
}

/*******************************************************************************
 * Computes the angular velocity to rotate the current polar axis on the goal
 * axis with the shortest path.
 ******************************************************************************/
void Rcs::TaskPolarTarget2D::computeXp_ik(double* phip) const
{
  // Compute the current angular velocity in direction of the desired Polar
  // axis. The sideways component it 0 per definition.
  double R_om[3], A_SR[3][3];
  computeOmega(R_om);
  computeSlerpFrame(A_SR);
  Vec3d_rotateSelf(R_om, A_SR);   // The y-component is what we need

  phip[0] = R_om[1];
  phip[1] = R_om[0];   // TODO: Check if negative sign is needed. But we probably need this component for damping.
}

/*******************************************************************************
 * We need to go in the negative velocity direction here.
 ******************************************************************************/
void Rcs::TaskPolarTarget2D::integrateXp_ik(double* x_res,
                                            const double* x,
                                            const double* x_dot,
                                            double dt) const
{
  VecNd_constMulAndAdd(x_res, x, x_dot, -dt, getDim());
}

/*******************************************************************************
 * Computes the Polar angle acceleration
 ******************************************************************************/
void Rcs::TaskPolarTarget2D::computeXpp(double* phi_ddot,
                                        const MatNd* q_ddot) const
{
  RFATAL("Implement TaskPolarTarget2D::computeXpp");
}

/*******************************************************************************
 * See header. Here we may use fixed indices for the z-axis, since the
 * direction index is already considered in the Slerp frame.
 ******************************************************************************/
void Rcs::TaskPolarTarget2D::computeJ(MatNd* jacobian) const
{
  // Reshape to correct dimensions
  MatNd_reshape(jacobian, 2, this->graph->nJ);

  // Compute the angular velocity Jacobian
  MatNd* JR2 = NULL;
  MatNd_create2(JR2, 3, this->graph->nJ);
  RcsGraph_3dOmegaJacobian(this->graph, this->ef, this->refBody,
                           this->refFrame, JR2);

  // Compute S-frame
  double A_SR[3][3];
  computeSlerpFrame(A_SR);

  // Rotate Jacobian into S-frame
  MatNd_rotateSelf(JR2, A_SR);

  // In the S-frame, the y-component corresponds to the intermediate angle
  // between current and desired Polar axis. The z-axis is invariant, since
  // it represents the rotation about the Polar axis.
  MatNd_copyRow(jacobian, 0, JR2, 1);
  MatNd_copyRow(jacobian, 1, JR2, 0);

  MatNd_destroy(JR2);
}

/*******************************************************************************
 * See header. Here we may use fixed indices for the z-axis, since the
 * direction index is already considered in the Slerp frame.
 ******************************************************************************/
void Rcs::TaskPolarTarget2D::computeH(MatNd* hessian) const
{
  int nq = this->graph->nJ;
  int nn = nq*nq;
  MatNd* H_omega = NULL;
  MatNd_create2(H_omega, 3*nq, nq);

  RcsGraph_3dOmegaHessian(this->graph, this->ef, this->refBody, this->refFrame,
                          H_omega);

  // Compute S-frame
  double A_SR[3][3];
  computeSlerpFrame(A_SR);

  // Project Hessian into S-frame
  for (int j = 0; j < nq; j++)
  {
    for (int k = 0; k < nq; k++)
    {
      double col[3];
      int idx_x = j * nq + k;
      int idx_y = nn + j * nq + k;
      int idx_z = 2 * nn + j * nq + k;
      col[0] = H_omega->ele[idx_x];
      col[1] = H_omega->ele[idx_y];
      col[2] = H_omega->ele[idx_z];
      Vec3d_rotateSelf(col, A_SR);
      H_omega->ele[idx_x] += col[0];
      H_omega->ele[idx_y] += col[1];
      H_omega->ele[idx_z] += col[2];
    }
  }

  MatNd_reshape(H_omega, 3*nq, nq);
  MatNd_reshape(hessian, 2*nq, nq);

  // In the S-frame, the y-component corresponds to the intermediate angle
  // between current and desired Polar axis. The z-axis is invariant, since
  // it represents the rotation about the Polar axis.
  MatNd_copyRows(hessian, 0, H_omega, nq, nq);
  MatNd_copyRows(hessian, nq, H_omega, 0, nq);

  MatNd_destroy(H_omega);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskPolarTarget2D::setTarget(const double polarTarget[2])
{
  setTarget(polarTarget[0], polarTarget[1]);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskPolarTarget2D::setTarget(double phi, double theta)
{
  this->polarDes[0] = phi;
  this->polarDes[1] = theta;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskPolarTarget2D::computeSlerpFrame(double A_SR[3][3]) const
{
  // Compute current Polar axis
  double A_ER[3][3];
  computeRelativeRotationMatrix(A_ER, this->ef, this->refBody);
  double* a_curr = A_ER[this->direction];

  // Compute the desired polar axis
  double a_des[3];
  Vec3d_getPolarAxis(a_des, this->polarDes[0], this->polarDes[1]);

  // In the following, we construct a coordinate frame S with z-axis as
  // current z-axis, y-axis as the rotation axis that rotates the z-axis
  // onto the target axis, and x-axis according to the right hand rule.
  // In this frame, the rotation about the y-axis will move the z-axis
  // in the plane spanned by z-axis and target axis.
  Vec3d_copy(A_SR[2], a_curr);
  Vec3d_crossProduct(A_SR[1], a_curr, a_des);

  // We need to make sure that the current and desired axis are not
  // parallel. This can be determined by checking the length of the
  // rotation axis A_SR[1].
  double lengthRot = Vec3d_getLength(A_SR[1]);

  // If the length is 0, the intermediate angle can be 0 or 180 degrees.
  // We construct a coordinate frame with z-axis being the current Polar
  // axis. From the other components, we can select any combination.
  if (lengthRot==0.0)
  {
    Mat3d_fromVec(A_SR, a_curr, 2);
  }
  else
  {
    // Normalize rotation axis
    Vec3d_constMulSelf(A_SR[1], 1.0/lengthRot);

    // Calculate x-axis according to right hand rule
    Vec3d_crossProduct(A_SR[0], A_SR[1], A_SR[2]);
  }

}

/*******************************************************************************
 * Transforms force from Jacobi coordinates to task coordinates.
 ******************************************************************************/
void Rcs::TaskPolarTarget2D::forceTrafo(double* ft_task) const
{
  RFATAL("Implement me");
}

/*******************************************************************************
 * Transforms the selection into the Jacobian coordinates
*******************************************************************************/
void Rcs::TaskPolarTarget2D::selectionTrafo(double* S_des_trafo,
                                            const double* S_des) const
{
  RFATAL("Implement me");
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::TaskPolarTarget2D::isValid(xmlNode* node, const RcsGraph* graph)
{
  bool success = Rcs::Task::isValid(node, graph, "POLAR_TARGET");


  // Check if axis direction is X, Y or Z
  char text[256] = "Z";
  getXMLNodePropertyStringN(node, "axisDirection", text, 256);

  if ((!STRCASEEQ(text, "X")) &&
      (!STRCASEEQ(text, "Y")) &&
      (!STRCASEEQ(text, "Z")))
  {
    success =  false;

    REXEC(3)
    {
      char taskName[256] = "unnamed task";
      getXMLNodePropertyStringN(node, "name", taskName, 256);
      RMSG("Task \"%s\": Axis direction not [0...2]: %s",
           taskName, text);
    }
  }

  return success;
}
