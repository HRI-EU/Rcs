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

#ifndef RCS_TASKREGIONBOXINTERVAL_H
#define RCS_TASKREGIONBOXINTERVAL_H

#include "TaskRegion.h"


namespace Rcs
{

/*! \ingroup RcsTaskRegion
 *  \brief Box interval region.
 *
 *  This class enforces the task-space coordinates to be on or within the given
 *  bounding box. The motion inside the bounding box is computed to follow the
 *  gradient set by the \ref setTaskReprojection() method. For parsing, the min
 *  and max attributes can be specified with 1 value or as many values as the
 *  dimension of the task. The "slowDownRatio" is the ratio of the component's
 *  range in which the projected gradient is scaled down. At the border of the
 *  interval, it is scaled to zero, and at the beggining of the slowDownRatio,
 *  the scaling is 1. The default for the slowDownRatio is 0, which leads to an
 *  abrupt velocity stop when getting to the interval's limit. Everything else
 *  is gradually reducing the velocity in the interval when approaching the
 *  limit, leading to a smoother motion. The velocity in the opposite direction
 *  of the interval boundary is not scaled. If the value is specified outside
 *  of the range [0...1] the constructor exits fatally.
 *  Here is an illustration for a slowDownRatio of 0.5:
 *
 *    scaling
 *    ^
 *  1 |
 *    |            __________
 *    |           /          \
 *    |          /            \
 *    |         /              \
 *    |        /                \
 *    |       /                  \
 *  0 o-------|----|----|----|----|-----> range
 *           min       cntr    max
 *
 *  Example:
 *  \code
 *    <Task name="Position XYZ" controlVariable="XYZ" effector="EndEffector" >
 *      <TaskRegion type="BoxInterval" min="-0.1 -0.2 -0.05" max="0.1"
 *                  slowDownRatio="0.5" />
 *    </Task>
 *  \endcode
 */
class TaskRegionBoxInterval : public TaskRegion
{
public:

  /*! \brief Constructor used in xml factory. Parses "min" and "max" as the
   *         parameters for an axis-aligned bounding box. The constructor
   *         exits fatally if the number of parameters in min and max do not
   *         correspond to the task's dimension.
   */
  TaskRegionBoxInterval(const Task* task, xmlNode* node);

  TaskRegionBoxInterval(const TaskRegionBoxInterval& src);

  virtual ~TaskRegionBoxInterval();

  virtual TaskRegionBoxInterval* clone() const;

  virtual void computeDX(const Task* task, double* dx, const double* x_des,
                         const double* x_curr) const;

private:

  std::vector<double> bbMin, bbMax;
  double slowDownRatio;
};

}

#endif // RCS_TASKREGIONBOXINTERVAL_H
