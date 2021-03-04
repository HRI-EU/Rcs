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

#include "TaskPolar2D.h"
#include "TaskFactory.h"
#include "Rcs_typedef.h"
#include "Rcs_macros.h"
#include "Rcs_parser.h"
#include "Rcs_math.h"
#include "Rcs_kinematics.h"


// register task at the task factory
static Rcs::TaskFactoryRegistrar<Rcs::TaskPolar2D> registrar("POLAR");



/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::TaskPolar2D::TaskPolar2D(const std::string& className,
                              xmlNode* node,
                              RcsGraph* _graph,
                              int dim) :
  Task(className, node, _graph, dim), direction(2)
{
  if (getClassName()=="POLAR")
  {
    double guiMax[2], guiMin[2];
    guiMin[0] = -M_PI;
    guiMin[1] = -M_PI;
    guiMax[0] = M_PI;
    guiMax[1] = M_PI;
    getXMLNodePropertyVecN(node, "guiMax", guiMax, 2);
    getXMLNodePropertyVecN(node, "guiMin", guiMin, 2);

    bool hide = false;
    getXMLNodePropertyBoolString(node, "hide", &hide);
    if (hide)
    {
      VecNd_setZero(guiMin, 2);
      VecNd_setZero(guiMax, 2);
    }

    resetParameter(Parameters(guiMin[0], guiMax[0], (180.0/M_PI), "Phi [deg]"));
    addParameter(Parameters(guiMin[1], guiMax[1], (180.0/M_PI), "Theta [deg]"));
  }

  // Parse axis direction (should be X, Y or Z)
  char text[16] = "Z";
  getXMLNodePropertyStringN(node, "axisDirection", text, 16);

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

}

/*******************************************************************************
 * Copy constructor doing deep copying
 ******************************************************************************/
Rcs::TaskPolar2D::TaskPolar2D(const TaskPolar2D& copyFromMe,
                              RcsGraph* newGraph):
  Task(copyFromMe, newGraph), direction(copyFromMe.direction)
{
}

/*******************************************************************************
 *  Destructor
 ******************************************************************************/
Rcs::TaskPolar2D::~TaskPolar2D()
{
}

/*******************************************************************************
 * Clone function
 ******************************************************************************/
Rcs::TaskPolar2D* Rcs::TaskPolar2D::clone(RcsGraph* newGraph) const
{
  return new Rcs::TaskPolar2D(*this, newGraph);
}

/*******************************************************************************
 * Computes the current value of the task variable: Polar angles
 ******************************************************************************/
void Rcs::TaskPolar2D::computeX(double* polarAngles) const
{
  double A_ER[3][3];
  computeRelativeRotationMatrix(A_ER, getEffector(), getRefBody());
  Vec3d_getPolarAngles(polarAngles, A_ER[this->direction]);
}

/*******************************************************************************
 * Computes the Polar angle velocity: phi_dot = H*R_om
 ******************************************************************************/
void Rcs::TaskPolar2D::computeXp(double* phip) const
{
  double R_om[3], A_ER[3][3], H[2][3];

  computeOmega(R_om);
  computeRelativeRotationMatrix(A_ER, getEffector(), getRefBody());
  Vec3d_getPolarVelocityMatrix(H, A_ER[this->direction]);

  // TODO: Do we need to account for the direction vector here? Plase check.
  phip[0] = H[0][0]*R_om[0] + H[0][1]*R_om[1] + H[0][2]*R_om[2];
  phip[1] = H[1][0]*R_om[0] + H[1][1]*R_om[1] + H[1][2]*R_om[2];
}

/*******************************************************************************
 * Computes the Polar angle acceleration
 ******************************************************************************/
void Rcs::TaskPolar2D::computeXpp(double* phipp, const MatNd* q_ddot) const
{
  RLOG(4, "Implement TaskPolar2D::computeXpp");
  VecNd_setZero(phipp, getDim());
}

/*******************************************************************************
 * Computes the Polar angle velocity delta
 ******************************************************************************/
void Rcs::TaskPolar2D::computeDXp(double* dOmega,
                                  const double* phip_des) const
{
  double E_om[3], A_ER[3][3], H[2][3], invH[3][2], E_om_des[3], lambda = 0.001;
  MatNd arrH      = MatNd_fromPtr(2, 3, &H[0][0]);
  MatNd arrInvH   = MatNd_fromPtr(3, 2, &invH[0][0]);
  MatNd arrLambda = MatNd_fromPtr(1, 1, &lambda);

  // Compute the current angular velocity in the end effector frame
  computeRelativeRotationMatrix(A_ER, getEffector(), getRefFrame());
  computeOmega(E_om);
  Vec3d_rotateSelf(E_om, A_ER);

  // Compute the desired angular velocity in the end effector frame. This is
  // currently done by inverting the Polar velocity matrix. However, it should
  // be possible to write it down analytically.
  Vec3d_getPolarVelocityMatrix(H, A_ER[this->direction]);
  MatNd_rwPinv2(&arrInvH, &arrH, NULL, &arrLambda);
  E_om_des[0] = invH[0][0]*phip_des[0] + invH[0][1]*phip_des[1];
  E_om_des[1] = invH[1][0]*phip_des[0] + invH[1][1]*phip_des[1];
  E_om_des[2] = invH[2][0]*phip_des[0] + invH[2][1]*phip_des[1];
  Vec3d_rotateSelf(E_om_des, A_ER);

  // The result is the difference of the first and second component (x and y)
  if (this->direction != 2)
  {
    RFATAL("TODO: Do we need to account for the direction vector here?");
  }

  dOmega[0] = E_om_des[0] - E_om[0];
  dOmega[1] = E_om_des[1] - E_om[1];
}

/*******************************************************************************

  This Jacobian relates the body-fixed x- and y-components of the
  angular velocity (with respect to a reference body) to the
  state velocity vector. Index 2 denotes the body to be controlled
  wrt. index 1. Index 1 is having identity values if no reference body
  is given. Index 0 denotes an inertial reference frame. The 2-fixed
  angular velocity wrt. body 1 is

  2_w12 = A_21 A_1I (I_w02 - I_w01)

  The differential kinematics give

  2_JR12 = A_21 A_1I (I_JR2 - I_JR1)

  with JR being the rotation Jacobians of the respective body. To
  control the axis direction of the 2-fixed z-axis, we only need the
  x and y component, so that the resulting Jacobian is

  J = 2_JR12 with only rows x,y

*******************************************************************************/
void Rcs::TaskPolar2D::computeJ(MatNd* jacobian) const
{
  MatNd* JR2 = NULL;
  MatNd_create2(JR2, 3, this->graph->nJ);
  RcsGraph_3dOmegaJacobian(this->graph, getEffector(), getRefBody(),
                           getEffector(), JR2);
  MatNd_reshape(jacobian, 2, this->graph->nJ);

  switch (this->direction)
  {
    case 0: // X rotation is free
      MatNd_copyRow(jacobian, 0, JR2, 1);
      MatNd_copyRow(jacobian, 1, JR2, 2);
      break;

    case 1: // Y rotation is free
      MatNd_copyRow(jacobian, 0, JR2, 0);
      MatNd_copyRow(jacobian, 1, JR2, 2);
      break;

    case 2: // Z rotation is free
      MatNd_reshape(JR2, 2, this->graph->nJ);
      MatNd_copy(jacobian, JR2);
      break;
  }

  MatNd_destroy(JR2);
}

/*******************************************************************************
 * The relative angular velocity is:
 *      1_om_12 = A_1I (I_om_2 - I_om_1)
 *
 * The Jacobian is:
 *      1_J_12 = d(A_1I)/dq (I_om_2 - I_om_1) + A_1I (I_J_2 - I_J_1)
 *             = A_1I (I_J_2 - I_J_1)
 *
 * The Hessian is:
 *      1_H_12 = d(A_1I)/dq(I_J_2 - I_J_1) + A_1I (I_H_2 - I_H_1)
 ******************************************************************************/
void Rcs::TaskPolar2D::computeH(MatNd* hessian) const
{
  int nq = this->graph->nJ;
  MatNd* H_omega = NULL;
  MatNd_create2(H_omega, 3*nq, nq);

  RcsGraph_3dOmegaHessian(this->graph, getEffector(), getRefBody(),
                          getEffector(), H_omega);

  MatNd_reshape(H_omega, 3*nq, nq);
  MatNd_reshape(hessian, 2*nq, nq);

  switch (this->direction)
  {
    case 0: // X rotation is free: copy the slices of y and z to H
      MatNd_copyRows(hessian, 0, H_omega, nq, nq);
      MatNd_copyRows(hessian, nq, H_omega, 2*nq, nq);
      break;

    case 1: // Y rotation is free: copy the slices of x and z to H
      MatNd_copyRows(hessian, 0, H_omega, 0, nq);
      MatNd_copyRows(hessian, nq, H_omega, 2*nq, nq);
      break;

    case 2: // Z rotation is free: copy the slices of x and y to H
      MatNd_reshape(H_omega, 2*nq, nq);
      MatNd_copy(hessian, H_omega);
      break;
  }

  MatNd_destroy(H_omega);
}

/*******************************************************************************
 *  Computes the delta in task space for the differential kinematics
 ******************************************************************************/
void Rcs::TaskPolar2D::computeDX(double* dx_ik, const double* polar_des) const
{
  double A_ER[3][3];
  computeRelativeRotationMatrix(A_ER, getEffector(), getRefBody());
  Vec3d_getPolarErrorFromDirection(dx_ik, polar_des, A_ER, this->direction);
}

/*******************************************************************************
 *  Due to the lack of 1 dof in the task representation, this function cannot be
 *  implemented.
 ******************************************************************************/
void Rcs::TaskPolar2D::computeDX(double* dx_ik,
                                 const double* x_des,
                                 const double* x_curr) const
{
  RFATAL("Not yet implemented");
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskPolar2D::integrateXp_ik(double* x_res, const double* x,
                                      const double* x_dot, double dt) const
{
  RFATAL("Not yet implemented");
}

/*******************************************************************************
 * Finite difference tests with a bit different error limits.
 ******************************************************************************/
bool Rcs::TaskPolar2D::test(bool verbose)
{
  bool success = true;
  bool success_i;
  bool relativeError = false;
  double errorLimit = 5.0e-3;
  double delta = 1.0e-4;

  success_i = testJacobian(errorLimit, delta, relativeError, verbose);
  success = success && success_i;

  success_i = testHessian(verbose);
  success = success && success_i;

  return success;
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
void Rcs::TaskPolar2D::computeFfXpp(double* x_ddot_res,
                                    const double* desired_acc) const
{
  RFATAL("Implement me");
}

/*******************************************************************************
 * Transforms force from Jacobi coordinates to task coordinates.
 ******************************************************************************/
void Rcs::TaskPolar2D::forceTrafo(double* ft_task) const
{
  RFATAL("Implement me");
}

/*******************************************************************************
 * Transforms the selection into the Jacobian coordinates
*******************************************************************************/
void Rcs::TaskPolar2D::selectionTrafo(double* S_des_trafo,
                                      const double* S_des) const
{
  RFATAL("Implement me");
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskPolar2D::toXMLBody(FILE* out) const
{
  Task::toXMLBody(out);

  switch (direction)
  {
    case 0:
      fprintf(out, " axisDirection=\"X\"");
      break;

    case 1:
      fprintf(out, " axisDirection=\"Y\"");
      break;

    default:
      break;
  }

}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::TaskPolar2D::isValid(xmlNode* node, const RcsGraph* graph)
{
  bool success = Rcs::Task::isValid(node, graph, "POLAR");


  // Check if axis direction is X, Y or Z
  char text[16] = "Z";
  getXMLNodePropertyStringN(node, "axisDirection", text, 16);

  if ((!STRCASEEQ(text, "X")) &&
      (!STRCASEEQ(text, "Y")) &&
      (!STRCASEEQ(text, "Z")))
  {
    success = false;

    REXEC(3)
    {
      char taskName[256] = "unnamed task";
      getXMLNodePropertyStringN(node, "name", taskName, 256);
      RMSG("Task \"%s\": Axis direction not [0...2]: %s", taskName, text);
    }
  }

  return success;
}
