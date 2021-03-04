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

#ifndef RCS_TASKREGION_H
#define RCS_TASKREGION_H

#include "Task.h"

/*!
 *  \defgroup RcsTaskRegion Task space region classes
 *
 *  A library for generalizing task space commands to the Task Interval (or
 *  Task Space Region) concept. Here is the corresponding paper:
 *
 *    Gienger, Michael, Herbert Janben, and Christian Goerick:
 *    Exploiting task intervals for whole body robot control.
 *    2006 IEEE/RSJ International Conference on Intelligent Robots and Systems.
 *    IEEE, 2006.
 *
 *  The general idea is to relax the concept of a desired task-space set point
 *  to a permissible manifold that the task variable is allowed to be inside.
 *  Strict satisfaction to stay within the interval is enforced. If inside the
 *  permissable interval, the null space criteria function is re-projected
 *  into the interval, so that the motion converges to the null-space optimal
 *  point inside the interval.
 */

namespace Rcs
{

/*! \ingroup RcsTaskRegion
 *  \brief Base class for various task regions implementations.
 *
 *  It is kept as lean as possible, the base class has no member variable
 *  storing the corresponding task. There is also an xml factory that allows
 *  to easily instantiate TaskRegions within a task's xml description. This
 *  class can be created through xml by the name "SetPoint".
 *
 *  Example:
 *  \code
 *    <Task name="Position XYZ" controlVariable="XYZ" effector="EndEffector" >
 *      <TaskRegion type="SetPoint" />
 *    </Task>
 *  \endcode
 */
class TaskRegion
{
public:

  /*! \brief Constructor used in xml factory. This needs to be re-implemented
   *         for each class if parameters need to be parsed.
   */
  TaskRegion(const Task* task, xmlNode* node);

  /*! \brief Copy constructor. This needs to be implemented in each derieved
   *         class to support copying.
   */
  TaskRegion(const TaskRegion& src);

  /*! \brief Virtual destructor to support polymorphism. This one is empty.
   */
  virtual ~TaskRegion();

  /*! \brief Returns a deep copy of the instance. This needs to be implemented
   *         in each derieved class to support copying.
   */
  virtual TaskRegion* clone() const;

  /*! \brief Computes the task-space delta dx according to the region's
   *         concept. In this implementation, it is simply x_des-x_curr.
   *
   *  \param[in]  task    Task related to the task region
   *  \param[out] dx      Resulting task space displacement
   *  \param[in]  x_des   Desired coordinate of the task region, for instance
   *                      reference point of a box interval.
   *  \param[in]  x_curr  Current task coordinates.
   */
  virtual void computeDX(const Task* task, double* dx, const double* x_des,
                         const double* x_curr) const;

  /*! \brief Method to store the inside-interval motion gradient. It is called
   *         from some IK-solvers, the re-projection follows the concepts
   *         described in the above mentioned paper. It might not apply to all
   *         derieved region classes.
   *
   *  \param[in]  dx_proj   Gradient to be applied to the inside-interval
   *                        motion. The size must match the task's dimenion.
   */
  virtual void setTaskReprojection(const std::vector<double>& dx_proj);

protected:

  std::vector<double> dx_proj;
};

}

#endif // RCS_TASKREGION_H
