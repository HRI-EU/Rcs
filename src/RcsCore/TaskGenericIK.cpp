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

#include "TaskGenericIK.h"
#include "TaskRegion.h"
#include "Rcs_typedef.h"
#include "Rcs_macros.h"
#include "Rcs_math.h"

#include <cfloat>


/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::TaskGenericIK::TaskGenericIK(): Task()
{
}

/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::TaskGenericIK::TaskGenericIK(const std::string& className,
                                  xmlNode* node,
                                  const RcsGraph* _graph,
                                  int _dim):
  Task(className, node, _graph, _dim)
{
}

/*******************************************************************************
 * Computes the delta in task space for the differential kinematics
 ******************************************************************************/
void Rcs::TaskGenericIK::computeDX(double* dx,
                                   const double* x_des,
                                   const double* x_curr) const
{
  if (this->tsr)
  {
    tsr->computeDX(this, dx, x_des, x_curr);
  }
  else
  {
  VecNd_sub(dx, x_des, x_curr, getDim());
  }

}

/*******************************************************************************
 * Computes the current velocity x_dot = J q_dot. For tasks with different
 * representations on position and velocitiy levels, there is a mismatch.
 ******************************************************************************/
void Rcs::TaskGenericIK::computeXp(double* x_dot_res) const
{
  computeXp_ik(x_dot_res);
}

/*******************************************************************************
 * Computes the velocity error
 ******************************************************************************/
void Rcs::TaskGenericIK::computeDXp(double* dx_dot_res,
                                    const double* desired_vel) const
{
  const unsigned int dim = getDim();
  computeXp(dx_dot_res);
  VecNd_constMulSelf(dx_dot_res, -1.0, dim);    // -x_dot_curr
  VecNd_addSelf(dx_dot_res, desired_vel, dim);  // x_dot_des - x_dot_curr
}

/*******************************************************************************
 * Computes the current acceleration in task space
 ******************************************************************************/
void Rcs::TaskGenericIK::computeXpp(double* x_ddot_res,
                                    const MatNd* q_ddot) const
{
  Task::computeXpp_ik(x_ddot_res, q_ddot);
}

/*******************************************************************************
 * Computes the feed forward acceleration given a desired acceleration. The
 * argument x_ddot_res is expected to be in Jacobi-coordinate. In this
 * implementation, it's just a copy operation. But in some derieved classes,
 * it needs to be overwritten to account for the different coordinates of the
 * position and velocity levels.
 ******************************************************************************/
void Rcs::TaskGenericIK::computeFfXpp(double* x_ddot_res,
                                      const double* desired_acc) const
{
  VecNd_copy(x_ddot_res, desired_acc, getDim());
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskGenericIK::integrateXp_ik(double* x_res, const double* x,
                                        const double* x_dot, double dt) const
{
  VecNd_constMulAndAdd(x_res, x, x_dot, dt, getDim());
}

/*******************************************************************************
 * Projects measured FT values into the task space forces \e ft_task
 ******************************************************************************/
void Rcs::TaskGenericIK::computeTaskForce(double* ft_task,
                                          const MatNd* ft_torque,
                                          const MatNd* J_full,
                                          int row) const
{
  // task Jacobian
  MatNd J_task = MatNd_fromPtr(getDim(), J_full->n,
                               MatNd_getRowPtr(J_full, row)); // Task Jacobian
  MatNd* PInv_J = NULL;
  MatNd_create2(PInv_J, getDim(), this->graph->nJ); // Pseudo Inverse
  MatNd ft_t = MatNd_fromPtr(getDim(), 1, ft_task); // task space forces

  double lambda0   = 10.e-6;
  MatNd lam0 = MatNd_fromPtr(1, 1, &lambda0);

  MatNd_rwPinv2(PInv_J, &J_task, NULL, &lam0);
  MatNd_transposeSelf(PInv_J);
  MatNd_mul(&ft_t, PInv_J, ft_torque);
  MatNd_destroy(PInv_J);

  forceTrafo(ft_task);
}

/*******************************************************************************
 * Transforms force from Jacobi coordinates to task coordinates.
 ******************************************************************************/
void Rcs::TaskGenericIK::forceTrafo(double* ft_task) const
{
}

/*******************************************************************************
 * Transforms the selection into the Jacobian coordinates
*******************************************************************************/
void Rcs::TaskGenericIK::selectionTrafo(double* S_des_trafo,
                                        const double* S_des) const
{
  VecNd_copy(S_des_trafo, S_des, getDim());
}
