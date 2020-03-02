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

#ifndef RCS_IKSOLVERRMR_H
#define RCS_IKSOLVERRMR_H



#include "ControllerBase.h"



namespace Rcs
{

/*! \ingroup RcsController
 * \brief Inverse kinematics solver (Resolved motion rate control).
 */
class IkSolverRMR
{
public:

  /*! \brief Constructs a Resolved Motion Rate IK class instance based on the
   *         given controller. All tasks from the controller's task list can
   *         be used. The state of the system will be taken from the
   *         controller's graph. This class will not change any internals of
   *         the controller.
   */
  IkSolverRMR(ControllerBase* controller);

  /*! \brief Deletes the IK class instance and frees all memory. The pased
   *         controller is not considered to be owned, so it will not be
   *         deleted.
   */
  virtual ~IkSolverRMR();

  /*! \brief Inverse kinematics with left-hand pseudo inverse, with both
   *         task-space and joint-space weighting:<br><br>
   *         The weighted Jacobian is:
   *         \f$
   *         \mathbf{J_{w} = J A W_q^{-1} }
   *         \f$
   *         <br><br>The weighted Jacobian pseudo-inverse is:
   *         \f$
   *         \mathbf{J_w^{\#} = (J_w^T W_x J_w+diag(\lambda))^{-1} J_w^T W_x}
   *         \f$
   *         <br><br>The overall inverse kinematics is:
   *         \f$
   *         \mathbf{\dot{q} = A W_q^{-1} (J_w^{\#} \dot{x} +
   *         (I-J_w^{\#} J_w) W_q^{-1} A^{-1}
   *         (\frac{\partial H}{\partial q})^T})
   *         \f$
   *         <br><br>
   *         Only the task elements with non-zero activations a_des are
   *         considered.
   *
   *  \param[out] dq_ts       Desired task space joint displacements. The array
   *                          is reshaped to contain all dofs (RcsGraph::dof)
   *  \param[out] dq_ns       Desired null space joint displacements. The array
   *                          is reshaped to contain all dofs (RcsGraph::dof)
   *  \param[in]  dx          Delta in task space. Vector dx is a column
   *                          vector of either the overall task dimension, or
   *                          of the dimension of the active tasks only. If
   *                          this argument is NULL, the task space related
   *                          joint velocity is zero.
   *  \param[in]  activation  Desired task activations. If this argument is
   *                          NULL, all tasks are considered active.
   *  \param[in]  dH          Null space gradient. It must be of size nJ x 1,
   *                          or of 1 x nJ, where nJ is the number of
   *                          unconstrained dofs.
   *  \param[in]  lambda      Regularization factor for the pseudo-inverse.
   *  \param[in]  recomputeKinematics If this is true, the function
   *                                  \ref computeKinematics is called before
   *                                  calculating the above equations.
   */
  virtual void solveLeftInverse(MatNd* dq_ts, MatNd* dq_ns, const MatNd* dx,
                                const MatNd* dH, const MatNd* activation,
                                double lambda, bool recomputeKinematics=true);

  /*! \brief Same as
   *         \ref IkSolverRMR::solveLeftInverse(MatNd* dq_ts, MatNd* dq_ns,
   *              const MatNd* dx, const MatNd* dH, const MatNd* activation,
   *              double lambda, bool recomputeKinematics=true)
   *
   *         Argument dq comprises bot task and null space terms
   *         (dq = dq_ts + dq_ns)
   */
  virtual void solveLeftInverse(MatNd* dq, const MatNd* dx, const MatNd* dH,
                                const MatNd* activation, double lambda,
                                bool recomputeKinematics=true);

  /*! \brief Computes the joint velocities based on the task space inverse
     *         kinematics according to Liegeois:<br>
     *         \f$
     *         \mathbf{\dot{q} =  J^{\#} \dot{x} -
     *         \alpha (I - J^{\#} J) W_q^{-1}
     *         (\frac{\partial H}{\partial q})^T }
     *         \f$
     *         <br>
     *         Only the task elements with non-zero activations a_des are
     *         considered. The pseudo-inverse is computed as
     *         <br>
     *         \f$
     *         \mathbf{J^{\#} =  W_q^{-1} J^T (J W_q^{-1} J^T}
     *         + \mathsf{diag} (\mbox{\boldmath$\lambda$}))^{-1}
     *         \f$
     *         <br>
     *         where \f$\lambda\f$ is self->lambda0 and \f$\mathbf{W_q}\f$ is
     *         computed according to the joint ranges.
     *
     *  \param[out] dq_des      Desired joint velocities. The array is only
     *                          composed of unconstrained dofs (RcsGraph::nJ)
     *  \param[in]  dx          Delta in task space
     *  \param[in]  dH          Null space gradient. It must be of size nJ x 1,
     *                          or of 1 x nJ, where nJ is the number of
     *                          unconstrained dofs.
     *  \param[in]  activation  Desired task activations.
     *  \param[in]  lambda      Regularization factors for the pseudo-inverse.
     *                          Must be 1 x 1 or nx x 1, where nx is the
     *                          number of the active task dimensions.
     */
  virtual void solveRightInverse(MatNd* dq_des,
                                 const MatNd* dx,
                                 const MatNd* dH,
                                 const MatNd* activation,
                                 const MatNd* lambda);

  /*! \brief Same as
   *         \ref IkSolverRMR::solveRightInverse(MatNd* dq_des, const MatNd* dx,
   *              const MatNd* dH, const MatNd* activation, const MatNd* lambda)
   *
   *         Argument lambda is a scalar and will be applied to all task
   *         elements.
   */
  virtual void solveRightInverse(MatNd* dq_des, const MatNd* dx,
                                 const MatNd* dH, const MatNd* activation,
                                 double lambda);

  /*! \brief Computes the task blending matrix Wx. We compute it as a
   *         vector, since the matrix is diagonal. The vector only
   *         comprises the rows that have a non-zero activation. Wx will
   *         be reshaped by this function.
   *
   *  \param[in]   controller Controller with current graph state
   *  \param[out]  Wx       Weight vector of dimension nx x 1
   *  \param[in]   a_des    Full activation vector (including inactive
   *                        dimensions)
   *  \param[in]   J        Task Jacobian
   *  \param[in]   lambda0  Small regularization value
   *  \param[in]   useInnerProduct Flag to use normalization based on the
   *                               Jacobian
   */
  static void computeBlendingMatrix(const ControllerBase& controller,
                                    MatNd* Wx,
                                    const MatNd* a_des,
                                    const MatNd* J,
                                    const double lambda0,
                                    const bool useInnerProduct=true);

  /*! \brief Returns the determinant of the Jacobian from the last solver
   *         iteration.
   */
  virtual double getDeterminant() const;

  /*! \brief Returns the pointer to the internal controller.
   */
  virtual ControllerBase* getController() const;

  /*! \brief Returns the internal number of degrees of freedom. These are the
   *         ones relevant for the IK (RcsGraph::nJ) substracted by the
   *         number of coupled degrees of freedom / joints.
   *
   *  \return Number of internal dof of the IK algorithm.
   */
  virtual unsigned int getInternalDof() const;



protected:

  void reshape();
  void computeKinematics(const MatNd* activation, double lambda0);

  ControllerBase* controller;
  unsigned int nx, nTasks, nq, nqr;
  double det;
  MatNd* A, *invA, *invWq, *J, *pinvJ, *N, *dHA, *dq, *dqr, *dxr, *NinvW,
         *Wx, *dHr, *dH, *dH_jl, *dH_ca, *dx, *ax_curr;
};


}   // namespace Rcs


#endif // RCS_IKSOLVERRMR_H
