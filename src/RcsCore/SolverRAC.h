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

#ifndef RCS_SOLVERRAC_H
#define RCS_SOLVERRAC_H


#include "ControllerBase.h"


namespace Rcs
{

/*! \ingroup RcsController
 * \brief Resolved acceleration controller
 */
class SolverRAC
{
public:
  SolverRAC(const Rcs::ControllerBase* controller);

  virtual ~SolverRAC();

  /*! \brief This function computes a resolved acceleration control:
   *         q_ddot = J# (ax_des - Jdot q_dot) - N dH
   *         The state q and q_dot is taken from the RcsGraph. The right pseudo-
   *         inverse is used, which means that in case of non-conflicting tasks
   *         no regularization is needed (lambda may be 0). The function can
   *         deal with coupled joints. The Jacobian pseudo-inverse is weighted
   *         with the joint ranges. Null space damping is applied in this
   *         method. That's not so clean and should probably get out to the
   *         caller.
   *
   *  \param[out] q_ddot   Joint space acceleration vector, its dimension is
   *                       RcsGraph::dof x 1
   *  \param[in] a_des     Task activation vector, its dimension is numTasks x 1
   *  \param[in] ax_des    Task space acceleration, its dimension is
   *                       nx x 1, where nx is number of active task vector
   *                       elements
   *  \param[in] dH        Null space gradient of dimension 1 x RcsGrph::nJ
   *  \param[in] lambda    Regularization for the pseudo-inverse
   *  \return Mean squared error of back-projected task acceleration
   */
  void solve(MatNd* q_ddot, const MatNd* a_des, const MatNd* ax_des,
             const MatNd* dH, double lambda);


  /*! \brief Makes a copy of the solver (including the controller and graph),
   *         sets the position and velocity state to random values, and performs
   *         an inverse test in the form qpp = solve(a_des, ax_rnd, dH_rnd) with
   *         a_des being the function argument and ax_rnd and dH_rnd being
   *         random vectors. The resulting qpp is projected back to the task
   *         accelerations: ax_test = J*qpp + Jp*qp. The mean squared error
   *         between them is returned:
   *         err = sqrt[(ax_test-ax_rnd)^T * (ax_test-ax_rnd)]
   *         The function can deal with coupled joints as long as they are not
   *         within the kinematic chain of the tested tasks.
   *
   *  \param[in] a_des Task activation vector, dimension numTasks x 1
   *  \return Mean squared error of back-projected task acceleration
   */
  double test(const MatNd* a_des) const;

  const Rcs::ControllerBase* controller;
};

}   // namespace Rcs


#endif // RCS_SOLVERRAC_H
