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

#ifndef RCS_TASKEULER3D_H
#define RCS_TASKEULER3D_H

#include "TaskGenericIK.h"


namespace Rcs
{
/*! \ingroup RcsTask
 * \brief    3-dimensional Euler Angles task variable
 *
 *           This tasks allows to set a 3D orientation (Euler XYZ angles)
 *           of an effector. The orientation can also be relative to another
 *           body and reference frame.
 */
class TaskEuler3D: public Task
{
public:

  /*! \brief Constructor based on xml parsing
   */
  TaskEuler3D(const std::string& className, xmlNode* node, RcsGraph* graph,
              int dim=3);

  /*! \brief Copy constructor doing deep copying with optional new graph
   *         pointer
   */
  TaskEuler3D(const TaskEuler3D& copyFromMe, RcsGraph* newGraph=NULL);

  /*! Constructor based on graph and effectors.
   */
  TaskEuler3D(RcsGraph* graph, const RcsBody* effector,
              const RcsBody* refBdy, const RcsBody* refFrame);

  /*! \brief Destructor.
   */
  virtual ~TaskEuler3D();

  /*! \brief Virtual copy constructor with optional new graph
   */
  virtual TaskEuler3D* clone(RcsGraph* newGraph=NULL) const;

  /*! \brief Computes the current XYZ Euler angles.
   *
   *  \param[out] ea Euler angles
   */
  virtual void computeX(double* ea) const;

  /*! \brief Computes the current XYZ Euler angle velocity vector according
   *         to
   *         \f$
   *         \mbox{$\left( \begin{array}{c}
   *                       \dot{\alpha} \\ \dot{\beta} \\  \dot{\gamma}
   *                       \end{array} \right)$}
   *         =
   *         \mbox{$\left( \begin{array}{ccc}
   *                       {1} &
   *                       {\frac{sin(\alpha) sin(\beta)}{cos(\beta)}} &
   *                       {-\frac{cos(\alpha) sin(\beta)}{cos(\beta)}} \\
   *                       [0.2em]
   *                       {0} & {cos(\alpha)} & {sin(\alpha)} \\ [0.2em]
   *                       {0} &
   *                       { -\frac{sin(\alpha)}{cos(\beta)}} &
   *                       { \frac{cos(\alpha)} {cos(\beta)}}
   *                       \end{array} \right)$}
   *         \mbox{$\left( \begin{array}{c}
   *                       _I \dot{\omega}_y \\
   *                       _I \dot{\omega}_y \\
   *                       _I \dot{\omega}_z
   *                       \end{array} \right)$}
   *         \f$
   *
   *  The angular velocities are computed based on the rigid body angular
   *  velocities from the graph.
   *  If the Euler angles are in a close-to-singular configuration, the
   *  Euler velocities can become very large.
   *
   *  \param[out] eap Euler velocity vector.
   */
  virtual void computeXp(double* eap) const;

  /*! \brief Computes the task's angular velocity vector in the frame of the
   *         reference body (or world, if refBody is NULL). It calls
   *         TaskGenericIK::computeOmega() internally.
   *
   *  \param[out] omega Angular velocity vector (3-dimensional)
   */
  virtual void computeXp_ik(double* omega) const;

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

  /*! \brief Computes the current XYZ Euler angle acceleration vector
   * app = H (wp - d(H^-1)/dt ap)
   *
   *  \param[out] eapp Euler acceleration vector.
   *  \param[in] qpp joint acceleration vector.
   */
  virtual void computeXpp(double* eapp, const MatNd* qpp) const;

  /*! \brief Computes the feed-forward acceleration:
   *
   * Computes the desired angular accelerations from given angular Euler
   * angle accelerations:
   *  wp = d(H^-1)/dt ap + (H^-1) app
   *     = d(H^-1)/dt H om +  (H^-1) app
   *
   *  \param[out] omegap_des Desired angular accelerations
   *  \param[in]  eapp_des    Desired Euler angle accelerations
   */
  virtual void computeFfXpp(double* omegap_des,
                            const double* eapp_des) const;

  /*! \brief Computes the angular velocity Jacobian in the coordinates of
   *         the task's refBody:
   *         \f$
   *         \mathbf{J = A_{ref-I} ( {_I}J_{R,ef} - {_I}J_{R,ref} ) }
   *         \f$.
   *         If the refBody is NULL, the Jacobian is calculated in the
   *         world frame. Only the unconstrained dofs are computed.
   *
   *  \param[out] jacobian Angular velocity Jacobian with dimension 3 x nJ
   *                       where nJ is the number of unconstrained degrees of
   *                       freedom (see \ref RcsGraph).
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

  /*! \brief Computes the angular velocity error. It is represented in the
   *         coordinate frame of the reference (the one x_curr is
   *         represented in). The angular velocity error is the shortest
   *         distance between the current and desired rotation on the great
   *         arc (SLERP).
   *
   *  \param[out] dx     3 x 1 angular displacement to align rotation defined
   *                     by Euler angles ea_curr with Euler angles ea_des.
   *  \param[in] ea_des  Desired Euler angles
   *  \param[in] ea_curr Current Euler angles
   */
  virtual void computeDX(double* dx,
                         const double* ea_des,
                         const double* ea_curr) const;

  /*! \brief Computes the angular velocity error based on a desired Euler
   *         velocity and the graph's current valocity state. This function
   *         does not suffer from numerical problems due to singularities.
   *         The mapping from Euler velocities to angular velocities is
   *         always well-defined:
   *         \f$
   *         \mathbf{ \Delta \omega}
   *         =
   *          \mathbf{ \omega_{des} }
   *          - \mathbf{ \omega_{curr} }
   *         =
   *         \mbox{$\left( \begin{array}{ccc}
   *                       {1} & {0} & {sin(\beta)} \\ [0.2em]
   *                       {0} & {cos(\alpha)} &
   *                       {-sin(\alpha) cos(\beta)} \\ [0.2em]
   *                       {0} & sin(\alpha) & {cos(\alpha) cos(\beta)}
   *                       \end{array} \right)$}
   *         \mbox{$\left( \begin{array}{c}
   *                       \dot{\alpha} \\ \dot{\beta} \\ \dot{\gamma}
   *                       \end{array} \right)$}
   *          - \mathbf{ \omega_{curr} }
   *          \f$
   *
   *  See also F. Pfeiffer: Mechanical System Dynamics, p. 23
   *  \param[out]  delta_omega_des  Delta velocity
   *  \param[out]  desired_eap      Desired Euler angular velocity
   */
  virtual void computeDXp(double* delta_omega_des,
                          const double* desired_eap) const;

  /*! \brief Computes the feedback acceleration
   *
   *         according to Spong, Hutchison & Vidyasagar. Robot Modeling and
   *         Control. Chapter 4.8, the transformation of the Euler Jacobian
   *         to the omega Jacobian is
   *         \f$ J_{Euler}(q) = H(\alpha) J_{\omega}(q) \f$
   *
   *        we have \f$ \tau = J^T(q)F \f$ (Chapter 4.10) so with a
   *        transformed Jacobian we get
   *        \f$ \tau = J_{Euler}^T F_{Euler} = (H J_{\omega})^T F_{Euler}
   *                 = J_{\omega}^T (H^T F_{Euler})\f$
   *        so we can pre-multiply the forces here to transform them
   *
   *  \param[out]    ft_res       3 x 1 task force in Jacobian coordinates
   *  \param[in,out] ft_int       3 x 1 integral force
   *  \param[in]     ft_des       3 x 1 desired force in Euler angle coordinates
   *  \param[in]     selection    3 x 1 selection mask for hybrid control:
   *                              1 for tracking, 0 for force
   *  \param[in]     ft_task      3 x 1 task force in WHATEVER coordinates TODO
   *  \param[in]     a_des        Activation value for the task
   *  \param[in]     kp           Position gain
   *  \param[in]     ki           Integral gain
   */
  virtual void computeAF(double* ft_res,
                         double* ft_int,
                         const double* ft_des,
                         const double* selection,
                         const double* ft_task,
                         const double a_des,
                         const double kp,
                         const double ki) const;

  /*! \brief Computes the task gradient according to
   *         \ref Task::computeTaskGradient and scales the result down with
   *         a constant factor.
   */
  virtual void computeTaskGradient(MatNd* dgDq,
                                   const double* x_des,
                                   const double* diagW=NULL) const;

  /*! \brief See \ref TaskGenericIK::computeAX
   */
  virtual void computeAX(double* a_res,
                         double* integral_x,
                         const double* x_des,
                         const double* xp_des,
                         const double* xpp_des,
                         const double* S_des,
                         const double a_des,
                         const double kp,
                         const double kd,
                         const double ki) const;

  /*! \brief Transforms force from Jacobi coordinates to task coordinates
   *         \e ft_task
   *
   *         here from omegas to Euler angles:
   *         \f$ f_{Euler} = (H^{-1})^T f_{\omega} \f$
   */
  virtual void forceTrafo(double* ft_task) const;

  /*! \brief See \ref TaskGenericIK::selectionTrafo
   */
  virtual void selectionTrafo(double* S_des_trafo,
                              const double* S_des) const;

  /*! \brief Returns true if the task is specified correctly, false
   *         otherwise. The task is invalid if
   *         - XML tag "controlVariable" must be "ABC"
   *         - The effector body given in the xmlNode does not exist in the
   *           given graph
   */
  static bool isValid(xmlNode* node, const RcsGraph* graph);
};

}

#endif // RCS_TASKEULER3D_H
