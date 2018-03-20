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

#ifndef RCS_IKSOLVERPRIORMR_H
#define RCS_IKSOLVERPRIORMR_H


#include "ControllerBase.h"


namespace Rcs
{

class IkSolverPrioRMR
{
public:
  IkSolverPrioRMR(const Rcs::ControllerBase* controller);

  virtual ~IkSolverPrioRMR();

  /*! \brief Computes the joint velocities based on the prioritized task
   *         space inverse kinematics according to:<br>
   *
   *         Task - Priority Formulations for the Kinematic Control of
   *         Highly Redundant Articulated Structures
   *         P. Baerlocher, R. Boulic
   *
   *         dq = J1# dx1 +                          1st priority
   *              [J2 N1]# (dx2 - J2 J1# dx1) +      2nd priority
   *              N1 (I - (J2 N1)# (J2 N1)) dH       3rd priority
   *
   *  \param[out] dq1        Joint space displacement of the first
   *                         priority task space. The dimension is
   *                         dof x 1, where dof is the full configuration
   *                         space dimension (including constrained dofs).
   *  \param[out] dq2        Joint space displacement of the second
   *                         priority task space. The dimension is
   *                         dof x 1, where dof is the full configuration
   *                         space dimension (including constrained dofs).
   *  \param[out] dq_ns      Joint space displacement of the null space.
   *                         The  dimension is dof x 1, where dof is
   *                         the full configuration space dimension
   *                         (including constrained dofs).
   *  \param[in]  dx         Task-level policy vector. The dimension is
   *                         of the sum of all task dimension. The vector
   *                         will be split up into the different
   *                         priorities internally.
   *  \param[in]  dH         Null space gradient. It must be of size
   *                         nJ x 1, or of 1 x nJ, where nJ is the number
   *                         of unconstrained dofs.
   *  \param[in]  activation Activation vector. Tasks with an activation
   *                         equal zero will be ignored. If activation is
   *                         NULL, all tasks will be considered as active.
   *                         The vector must be of dimension numTasks x 1.
   *  \param[in]  lambda     Regularization factor for the pseudo-inverse.
   *  \return true for success, false if one of the IK projections
   *          became singular (determinant <=0). In this case, all
   *          return arrays are set to 0.
   */
  bool solve(MatNd* dq1, MatNd* dq2, MatNd* dq_ns, const MatNd* dx,
             const MatNd* dH, const MatNd* activation,
             double lambda);


  /*! \brief Same as above, but considering all tasks, and having zero
   *         regularization.
   */
  bool solve(MatNd* dq1, MatNd* dq2, MatNd* dq_ns, const MatNd* dx,
             const MatNd* dH);


  /*! \brief Same as above, but computing the overall dq instead of the
   *         priority components.
   */
  bool solve(MatNd* dq, const MatNd* dx, const MatNd* dH,
             const MatNd* activation, double lambda0);


  /*! \brief Same as above, but considering all tasks, and having zero
   *         regularization.
   */
  bool solve(MatNd* dq, const MatNd* dx, const MatNd* dH);


  size_t getTaskDimForPriority(int prioLevel) const;
  size_t getTaskDimForPriority(int prioLevel,
                               const MatNd* activation) const;
  void setTaskPriority(size_t taskIdx, int prioLevel);

  const Rcs::ControllerBase* controller;
  unsigned int nx, nx1, nx2, nTasks, nTasks1, nTasks2, nq, nqr;
  MatNd* dx1, *dx2, *activation1, *activation2, *A, *invA, *invWq, *J1,
         *J2, *pinvJ1, *N1, *J2N1, *pinvJ2N1, *J2pinvJ1, *J2pinvJ1dx,
         *dx2_mod, *N2, *dHA, *dq_2c;
  std::vector<int> priorityLevel;
};


}   // namespace Rcs


#endif // RCS_IKSOLVERPRIORMR_H
