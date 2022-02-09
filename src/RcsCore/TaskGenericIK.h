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

#ifndef RCS_TASKGENERICIK_H
#define RCS_TASKGENERICIK_H

#include "Task.h"




namespace Rcs
{
/*! \ingroup RcsTask
 * \brief    Generic task variable base class
 *
 *           This class implements generic functions for task variables. It
 *           adds methods and members common to all inverse kinematics tasks
 *           to the Rcs::Task base class, assuming that the task-level
 *           velocities, accelerations and forces are represented in the same
 *           space as the task variable itself.
 *
 *           What to implement
 *
 *           Case 1: Positions, velocities and accelerations are represented
 *                   in the same coordinate frame (e.g. TaskPosition3D)
 *           - computeX()
 *           - computeJ()
 *           - computeH()
 *           - computeXp() not required, but the base class version is not
 *                         very efficient (computes xp = J qp)
 *           - integrateXp_ik()
 *
 *           Case 2: Positions are represented in a different coordinate frame
 *                   than velocities and accelerations (e.g. TaskEuler3D)
 *           - computeX()
 *           - computeJ()
 *           - computeH()
 *           - computeDX()
 *           - computeXp()
 *           - computeDXp()
 *
 *           automatically computed (in Task.cpp):
 *           - computeJdot()           uses computeH()
 *           - computeJdotQdot()       uses computeJdot()
 *           - computeTaskCost()       uses computeDX()
 *           - computeTaskGradient()   uses computeDX(), computeJ()
 *           - computeAX()             uses computeDX(), computeDXp(),
 *                                     computeXp(), computeXpp()
 */
class TaskGenericIK: public Task
{
public:

  using Task::computeDX;

  /*! \brief Default constructor
   */
  TaskGenericIK();

  /*! \brief Constructor based on xml parsing
   */
  TaskGenericIK(const std::string& className, xmlNode* node, RcsGraph* graph,
                int dim=0);

  /*! \brief Destructor
   */
  virtual ~TaskGenericIK();

  /*! \brief Computes the displacement in task space
   *         \f$
   *         \mathbf{dx = x_{des} - x_{curr} }
   *         \f$
   *         to reach the desired value  \e x_des.
   *
   *  \param[out] dx Task space displacement
   *  \param[in] x_des desired task space coordinates
   *  \param[in] x_curr current task space coordinates
   */
  virtual void computeDX(double* dx, const double* x_des,
                         const double* x_curr) const;

  /*! \brief Computes the current velocity in task space. In this class's
   *         implementation, it just calls \ref computeXp_ik().
   *
   *  \param[out] xp Task space velocity
   */
  virtual void computeXp(double* xp) const;

  /*! \brief Computes the velocity error:
   *         \f$
   *         \mathbf{\Delta \dot{x} = \dot{x}_{des} - \dot{x}_{curr} }
   *         \f$
   */
  virtual void computeDXp(double* delta_xp,
                          const double* desired_vel) const;

  /*! \brief Computes the current acceleration in task space. In this class's
   *         implementation, it is calculated as
   *         \f$
   *         \mathbf{\ddot{x} = J \ddot{q} + \dot{J} \dot{q} }
   *         \f$
   *         It needs to be overwritten for task variables that don't
   *         have velocities and positions represented in the same frame
   *         (e.g. for Euler angles, J might be representing the angular
   *         velocities instead of the Euler velocities).
   *
   *  \param[out] xpp Task space acceleration
   *  \param[in]  qpp Joint space acceleration
   */
  virtual void computeXpp(double* xpp, const MatNd* qpp) const;

  /*! \brief Computes the feed-forward acceleration. The argument xpp_res is
   *         expected to be in Jacobi-coordinate. In this implementation,
   *         it's just a copy operation. But in some derieved classes, it
   *         needs to be overwritten to account for the different coordinates
   *         of the position and velocity levels.
   *
   *  \param[out] xpp_res    Resulting acceleration. It is represented
   *                         in the IK-relevant coordinates.
   *  \param[in]  xpp_des    Vector of desired task acceleration. It is
   *                         represented in the task-relevant coordinates.
   */
  virtual void computeFfXpp(double* xpp_res, const double* xpp_des) const;

  /*! \addtogroup TaskLinearDerivative
   *  \brief Adds a velocity to the given state. Here it is simply
   *         x_res = x + x_dot*dt
   *
   *  \param[out] x_res    State after adding velocity increment
   *  \param[in]  x        State before adding velocity increment
   *  \param[in]  x_dot    Velocity in task velocity space
   *  \param[in]  dt       Time interval
   */
  virtual void integrateXp_ik(double* x_res, const double* x,
                              const double* x_dot, double dt) const;

  /*! \brief Projects measured FT values into the task space forces \e ft_task
   *         \f[
   *         f_{task} = (J_{task}^\#)^T J_{sensor}^{T} f_{sensor}
   *         \f]
   */
  virtual void computeTaskForce(double* ft_task,
                                const MatNd* ft_torque,
                                const MatNd* J_full,
                                int row) const;

  /*! \brief Transforms force from Jacobi coordinates to task coordinates.
   */
  virtual void forceTrafo(double* ft_task) const;

  /*! \brief Transforms the selection into the Jacobian coordinates
   */
  virtual void selectionTrafo(double* S_des_trafo,
                              const double* S_des) const;

};

}

#endif // RCS_TASKGENERICIK_H
