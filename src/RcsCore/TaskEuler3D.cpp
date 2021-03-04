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

#include "TaskEuler3D.h"
#include "TaskFactory.h"
#include "Rcs_typedef.h"
#include "Rcs_macros.h"
#include "Rcs_parser.h"
#include "Rcs_utils.h"
#include "Rcs_math.h"
#include "Rcs_kinematics.h"


static Rcs::TaskFactoryRegistrar<Rcs::TaskEuler3D> registrar("ABC");


/*******************************************************************************
 *
 ******************************************************************************/
Rcs::TaskEuler3D::TaskEuler3D(const std::string& className_,
                              xmlNode* node,
                              RcsGraph* _graph,
                              int dim):
  Rcs::Task(className_, node, _graph, dim)
{
  if (getClassName()=="ABC")
  {
    resetParameter(Task::Parameters(-M_PI, M_PI, (180.0/M_PI), "A [deg]"));
    addParameter(Task::Parameters(-M_PI, M_PI, (180.0/M_PI), "B [deg]"));
    addParameter(Task::Parameters(-M_PI, M_PI, (180.0/M_PI), "C [deg]"));
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::TaskEuler3D::TaskEuler3D(const TaskEuler3D& copyFromMe,
                              RcsGraph* newGraph):
  Rcs::Task(copyFromMe, newGraph)
{
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::TaskEuler3D::TaskEuler3D(RcsGraph* graph_,
                              const RcsBody* effector,
                              const RcsBody* refBdy,
                              const RcsBody* refFrame): Task()
{
  this->graph = graph_;
  setClassName("ABC");
  setDim(3);
  setEffector(effector);
  setRefBody(refBdy);
  setRefFrame(refFrame ? refFrame : refBdy);
  resetParameter(Task::Parameters(-M_PI, M_PI, (180.0/M_PI), "A [deg]"));
  addParameter(Task::Parameters(-M_PI, M_PI, (180.0/M_PI), "B [deg]"));
  addParameter(Task::Parameters(-M_PI, M_PI, (180.0/M_PI), "C [deg]"));
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::TaskEuler3D::~TaskEuler3D()
{
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::TaskEuler3D* Rcs::TaskEuler3D::clone(RcsGraph* newGraph) const
{
  return new Rcs::TaskEuler3D(*this, newGraph);
}

/*******************************************************************************
 * Computes the XYZ Euler angles
 ******************************************************************************/
void Rcs::TaskEuler3D::computeX(double* x_res) const
{
  computeEulerAngles(x_res, getEffector(), getRefBody());
}

/*******************************************************************************
 * Computes the current Euler angles velocity. These can get excessively
 * large in singular configurations.
 ******************************************************************************/
void Rcs::TaskEuler3D::computeXp(double* eap) const
{
  double omega[3], ea[3], H[3][3];
  computeOmega(omega);
  computeX(ea);
  Mat3d_getEulerVelocityMatrix(H, ea);
  Vec3d_rotate(eap, H, omega);
}

/*******************************************************************************
 * Computes the current angular velocity with respect to the refBdy
 ******************************************************************************/
void Rcs::TaskEuler3D::computeXp_ik(double* omega) const
{
  computeOmega(omega);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskEuler3D::integrateXp_ik(double* x_res, const double* x,
                                      const double* omega, double dt) const
{
  double A_CI[3][3];   // Rotation matrix for current state
  Mat3d_fromEulerAngles(A_CI, x);

  double delta[3];
  Vec3d_constMul(delta, omega, dt);
  Mat3d_rotateOmegaSelf(A_CI, delta, true);
  Mat3d_toEulerAngles(x_res, A_CI);
}

/*******************************************************************************
 * Computes the current Euler angles accelerations
 *
 * app = H (omega_dot - d(H^-1)/dt ap)
 ******************************************************************************/
void Rcs::TaskEuler3D::computeXpp(double* eapp, const MatNd* q_ddot) const
{
  // Angular accelerations: omega_dot = Jdot q_dot + J q_ddot
  Rcs::Task::computeXpp_ik(eapp, q_ddot);

  double ea[3], eap[3];
  this->computeX(ea);
  this->computeXp(eap);

  // d(invH)/dt
  double dInvH[3][3];
  Mat3d_getInverseEulerVelocityMatrixDerivative(dInvH, ea, eap);

  double H[3][3];
  Mat3d_getEulerVelocityMatrix(H, ea);

  Vec3d_rotateSelf(eap, dInvH); // d(H^-1)/dt ap
  Vec3d_subSelf(eapp, eap); // (wp - d(H^-1)/dt ap)
  Vec3d_rotateSelf(eapp, H); // H (wp - d(H^-1)/dt ap)
}

/*******************************************************************************
 *  Computes the desired angular accelerations from given angular Euler angle
 *  accelerations:
 *
 *  wp = d(H^-1)/dt ap + (H^-1) app
 *     = d(H^-1)/dt H om +  (H^-1) app
 ******************************************************************************/
void Rcs::TaskEuler3D::computeFfXpp(double* omegap_des,
                                    const double* eapp_des) const
{
  double ea[3], eap[3];
  this->computeX(ea);
  this->computeXp(eap);

  // I_om = invH*eap
  double invH[3][3];
  Mat3d_getInverseEulerVelocityMatrix(invH, ea);

  // d(invH)/dt
  double dInvH[3][3];
  Mat3d_getInverseEulerVelocityMatrixDerivative(dInvH, ea, eap);

  // ompp_des = invH*eapp + dInvH*eap
  double tmp[3];
  Vec3d_rotate(tmp, invH, eapp_des);
  Vec3d_rotate(omegap_des, dInvH, eap);
  Vec3d_addSelf(omegap_des, tmp);

  // Check this!!!
  Vec3d_setZero(omegap_des);
}

/*******************************************************************************
 * Angular velocity Jacobian. This is not the geometric Jacobian, since we
 * compute differential gains for the IK in order to get around singularity
 * issues. For this reason, this Jacobian can be considered as "safe" (It is
 * always non-singular), and such differs from the standard formulations (for
 * instance used in the element-wise Kardan Angles Jacobian as implemented
 * here).
 *
 * ref_JR = A_ref-I * (I_JR,ef - I_JR,ref)
 ******************************************************************************/
void Rcs::TaskEuler3D::computeJ(MatNd* jacobian) const
{
  RcsGraph_3dOmegaJacobian(this->graph, getEffector(), getRefBody(),
                           getRefFrame(), jacobian);
}

/*******************************************************************************
 * see RcsGraph_3dOmegaHessian();
 ******************************************************************************/
void Rcs::TaskEuler3D::computeH(MatNd* hessian) const
{
  RcsGraph_3dOmegaHessian(this->graph, getEffector(), getRefBody(),
                          getRefFrame(), hessian);
}

/*******************************************************************************
 * Computes the delta in task space for the differential kinematics. The current
 * state is taken from the underlying graph. In order to avoid numerical
 * issues, we use the Euler error approximation in the near-to-linear range
 * of the orientation error (1 degree turned out to be fine). Using the axis-
 * angle error for tiny values leads to problems due to distorted rotation axes
 * etc. For larger orientation errors, we use the axis-angle error, since it
 * does not do any linearization.
 ******************************************************************************/
void Rcs::TaskEuler3D::computeDX(double* dx,
                                 const double* x_des,
                                 const double* x_curr) const
{
  double A_curr[3][3], A_des[3][3];

  Mat3d_fromEulerAngles(A_curr, x_curr);
  Mat3d_fromEulerAngles(A_des, x_des);

  double angle = Mat3d_diffAngle(A_des, A_curr);

  if (angle<1.0*(M_PI/180.0))   // Less than 1 deg error: Use Euler error
  {
    Mat3d_getEulerError(dx, A_curr, A_des);
  }
  else   // Larger error: Use axis angle error
  {
    Mat3d_getAxisAngle(dx, A_des, A_curr);
    Vec3d_constMulSelf(dx, angle);
  }
}

/*******************************************************************************
 * Computes the Euler velocity error.
 * See F. Pfeiffer: Mechanical System Dynamics, p. 23
 * Note: The inverse Euler velocity matrix does not have any numerical problems
 *       due to singularities. It is always well-defined.
 ******************************************************************************/
void Rcs::TaskEuler3D::computeDXp(double* delta_om,
                                  const double* eap_des) const
{
  double om_curr[3], ea[3], invH[3][3], om_des[3];

  computeOmega(om_curr);
  computeX(ea);
  Mat3d_getInverseEulerVelocityMatrix(invH, ea);
  Vec3d_rotate(om_des, invH, eap_des);
  Vec3d_sub(delta_om, om_des, om_curr);
}

/*******************************************************************************
 * Computes the feedback acceleration according to Spong, Hutchison &
 * Vidyasagar. Robot Modeling and Control. Chapter 4.8, the transformation of
 * the Euler Jacobian to the omega Jacobian is
 *
 * \f$ J_{Euler}(q) = H(\alpha) J_{\omega}(q) \f$
 *
 * we have \f$ \tau = J^T(q)F \f$ (Chapter 4.10)
 *
 * so with a transformed Jacobian we get
 *
 * \f$ \tau = J_{Euler}^T F_{Euler} = (H J_{\omega})^T F_{Euler}
 *          = J_{\omega}^T (H^T F_{Euler})\f$
 *
 * so we can pre-multiply the forces here to transform them
 ******************************************************************************/
void Rcs::TaskEuler3D::computeAF(double* ft_res,
                                 double* ft_int,
                                 const double* ft_des,
                                 const double* selection,
                                 const double* ft_task,
                                 const double a_des,
                                 const double kp,
                                 const double ki) const
{
  Rcs::Task::computeAF(ft_res, ft_int, ft_des, selection, ft_task,
                       a_des, kp, ki);

  double ea[3], H[3][3];
  computeX(ea);
  Mat3d_getEulerVelocityMatrix(H, ea);
  Vec3d_transRotateSelf(ft_res, H);
}

/*******************************************************************************
 * Transforms force from Jacobi coordinates to task coordinates \e ft_task here
 * from omegas to Euler angles:
 *         \f$ f_{Euler} = (H^{-1})^T f_{\omega} \f$
 ******************************************************************************/
void Rcs::TaskEuler3D::forceTrafo(double* ft_task) const
{
  double ea[3], invH[3][3];
  computeX(ea);
  Mat3d_getInverseEulerVelocityMatrix(invH, ea);

  // transform the values obtained by the controller from the complete Jacobian
  // from omegas to Euler
  Vec3d_transRotateSelf(ft_task, invH);
}

/*******************************************************************************
 * Transforms the selection into the Jacobian coordinates
 ******************************************************************************/
void Rcs::TaskEuler3D::selectionTrafo(double* S_des_trafo,
                                      const double* S_des) const
{
  double sqrLengthS = Vec3d_sqrLength(S_des);

  if ((sqrLengthS==3.0) || (sqrLengthS==0.0)) // fully selected or de-selected
  {
    VecNd_copy(S_des_trafo, S_des, getDim());
  }
  else
  {
    double ea[3], invH[3][3], S_full[3];
    computeX(ea);
    Mat3d_getInverseEulerVelocityMatrix(invH, ea);

    Vec3d_rotate(S_des_trafo, invH, S_des);

    Vec3d_setElementsTo(S_full, 1.);
    Vec3d_rotateSelf(S_full, invH);

    for (size_t i=0; i<3; i++)
    {
      S_des_trafo[i] /= S_full[i];
    }
  }
}

/*******************************************************************************
 * dg/dq = dx^T W J
 ******************************************************************************/
void Rcs::TaskEuler3D::computeTaskGradient(MatNd* dgDq,
                                           const double* x_des,
                                           const double* diagW) const
{
  Rcs::Task::computeTaskGradient(dgDq, x_des, diagW);
  MatNd_constMulSelf(dgDq, 0.05);
}

/*******************************************************************************
 * Don't scale a_res here! It might lead to very undesired behavior, since the
 * gains for the velocity and position don't depend linearly.
 * NOTE: Currently, the integral term and the acceleration feed forward term
 *       are not being used (see NULL arguments).
 ******************************************************************************/
void Rcs::TaskEuler3D::computeAX(double* a_res,
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
  // Rcs::TaskGenericIK::computeAX(a_res, integral_x, x_des, x_dot_des, x_ddot_des,
  //                               S_des, a_des, kp, kd, ki);
  Rcs::Task::computeAX(a_res, integral_x, x_des, x_dot_des, x_ddot_des,
                       S_des, a_des, kp, kd, ki);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::TaskEuler3D::isValid(xmlNode* node, const RcsGraph* graph)
{
  // Check if xml body names exists in the graph
  bool success = Rcs::Task::isValid(node, graph, "ABC");

  return success;
}
