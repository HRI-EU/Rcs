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

#ifndef RCS_TASKGENERICEULER3D_H
#define RCS_TASKGENERICEULER3D_H

#include "TaskGenericIK.h"

namespace Rcs
{

/*! \ingroup RcsTask
 * \brief This tasks allows to set a 3D orientation (Euler angles)
 *         of an effector
 *
 *  The orientation can also be relative to another body and reference frame.
 */
class TaskGenericEuler3D : public Task
{
public:

  /*! Constructor based on xml parsing
   */
  TaskGenericEuler3D(const std::string& className, xmlNode* node,
                     RcsGraph* graph, int dim=3);

  /*! Constructor based on graph, euler order and effectors.
   */
  TaskGenericEuler3D(RcsGraph* graph, const char* eulerOrder,
                     const RcsBody* effector, const RcsBody* refBdy,
                     const RcsBody* refFrame);

  /*! Destructor
   */
  virtual ~TaskGenericEuler3D();

  /*!
   * \brief Virtual copy constructor with optional new graph
   */
  virtual TaskGenericEuler3D* clone(RcsGraph* newGraph=NULL) const;

  /*! \brief Computes the current value of the task variable
   *
   *  The result is written to parameter \e x_res.
   */
  virtual void computeX(double* x_res) const;

  /*! \brief Computes the angular velocity Jacobian in the coordinates of
   *         the task's refBody:
   *         \f$
   *         \mathbf{J = A_{ref-I} ( {_I}J_{R,ef} - {_I}J_{R,ref} ) }
   *         \f$.
   *         If the refBody is NULL, the Jacobian is calculated in the
   *         world frame. Only the unconstrained dofs are computed.
   *
   *  \param[out] jacobian Angular velocity Jacobian
   */
  virtual void computeJ(MatNd* jacobian) const;

  /*! \brief Computes current task Hessian to parameter \e hessian. It is
   *         formulated for the angular velocities of the body with respect
   *         to its reference. This is due to the ambiguities in the Euler
   *         angles for certain configurations. Internally, the function
   *         \ref RcsGraph_3dOmegaHessian() is called.
   *
   *  \param[out] hessian Angular velocity Hessian with dimension nJ x (3*nJ)
   *                      where nJ is the number of unconstrained degrees of
   *                      freedom (see \ref RcsGraph).
   */
  virtual void computeH(MatNd* hessian) const;

  static void computeEulerAngles(double* ea, const RcsBody* effector,
                                 const RcsBody* referenceBody,
                                 const int eulerOrder);

  /*! \brief Computes the current velocity in task space
   *
   *  The result is written to parameter \e eap.
   */
  virtual void computeXp(double* eap) const;

  /*! \brief Computes the current acceleration in task space
   *
   *  The result is written to parameter \e eapp.
   */
  virtual void computeXpp(double* eapp, const MatNd* qpp) const;

  /*! \brief Computes the feedback error dx based on Chiaverini's formula.
   *
   *         The algorithm has been introduced by Chiaverini: <br> <br>
   *         Stefano Chiaverini: <br>
   *         <b> Singularity-robust task priority redundancy resolution for
   *         real-time kinematic control of robot manipulators, <br>
   *         <i> IEEE Transactions on Robotics and Automation, Vol. 13,
   *         No. 3, June 1997. </i></b>
   */
  virtual void computeDX(double* dx_res, const double* desired,
                         const double* curr) const;

  /*! \brief Computes the velocity error
   *
   *  \param[out]  delta_omega   Delta velocity
   *  \param[out]  desired_eap   Desired Euler angular velocity
   */
  virtual void computeDXp(double* delta_omega, // Desired delta omega
                          const double* desired_eap) const;

  /*! \brief Computes the feed-forward acceleration
   *
   * see Rcs/1.4/RcsController/1.4/doc/latex Sect. 1.7
   */
  virtual void computeFfXpp(double* omegapp_des,
                            const double* eapp_des) const;

  /*! \brief Computes the feedback acceleration
   *
   * according to Spong, Hutchison & Vidyasagar. Robot Modeling and
   * Control. Chapter 4.8,
   * the transformation of the Euler Jacobian to the omega Jacobian is
   * \f$ J_{Euler}(q) = H(\alpha) J_{\omega}(q) \f$
   *
   * we have \f$ \tau = J^T(q)F \f$ (Chapter 4.10) so with a transformed
   * Jacobian we get
   * \f$ \tau = J_{Euler}^T F_{Euler} = (H J_{\omega})^T F_{Euler} = J_{\omega}^T (H^T F_{Euler})\f$ so we
   * can pre-multiply the forces here to transform them
   */
  virtual void computeAF(double* ft_res,
                         double* ft_int,
                         const double* ft_des,
                         const double* selection,
                         const double* ft_task,
                         const double a_des,
                         const double kp,
                         const double ki) const;

  /*! \brief Adds the rotation increment to the Euler angles resulting from
   *         the angular velocity omega. The method computes the rotation
   *         matrix from the Euler angles, and then rotates it about the
   *         angular velocity displacement. The resulting rotation matrix is
   *         converted back to Euler angles.
   *
   *  \param[out] x_res  Integrated Euler angles
   *  \param[in] x       Euler angles before integration
   *  \param[in] omega   Angular velocity vector (3-dimensional)
   *  \param[in] dt      Time step
   */
  virtual void integrateXp_ik(double* x_res, const double* x,
                              const double* omega, double dt) const;

  /*! \brief Transforms force from Jacobi coordinates to task
   *         coordinates \e ft_task
   *
   *         here from omegas to Euler angles:
   *         \f$ f_{Euler} = (H^{-1})^T f_{\omega} \f$
   */
  virtual void forceTrafo(double* ft_task) const;

  virtual void selectionTrafo(double* S_des_trafo,
                              const double* S_des) const;

  /*! \brief Computes matrix that converts Euler rates to omegas
   *
   * Equivalent to \ref Mat3d_getInverseEulerVelocityMatrix
   *
   * for this implementation see
   * James Diebel. Representing Attitude:
   * Euler Angles, Unit Quaternions, and Rotation Vectors. 2006
   * Sect. 5.2
   *
   * and Rcs/1.4/RcsController/1.4/doc/latex Sect. 1.7
   */
  static void EulerRateToOmegasMat(double B[3][3], const double ea[3],
                                   const int eulerOrderVect[4]);

  /*! \brief Computes matrix that converts omegas to Euler rates
   *
   * Equivalent to \ref Mat3d_getEulerVelocityMatrix
   */
  static void OmegasToEulerRateMat(double Binv[3][3], const double ea[3],
                                   const int eulerOrderVect[4]);

  /*! \brief Calculates the time derivative of the matrix that converts
   *         Euler rates to omegas multiplied by Euler velocities
   *
   * Equivalent to \ref Mat3d_getInverseEulerVelocityMatrixDerivative * eap
   */
  static void EulerRateToOmegasDerivativeTimesEulerVel(double omegap[3],
                                                       const double ea[3],
                                                       const double eap[3],
                                                       const int eulerOrderVect[4]);

  /*! \brief Returns true if the task is specified correctly, false
   *         otherwise. The task is invalid if
   *         - XML tag "controlVariable" must be "genericEuler"
   *         - The effector body given in the xmlNode does not exist in the
   *           given graph
   */
  static bool isValid(xmlNode* xml_node, const RcsGraph* graph);

protected:
  int eulerOrder;
  int eulerOrderVect[4];
};

}

#endif // RCS_TASKGENERICEULER3D_H
