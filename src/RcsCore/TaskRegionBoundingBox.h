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

#ifndef RCS_TASKREGIONBOUNDINGBOX_H
#define RCS_TASKREGIONBOUNDINGBOX_H

#include "TaskRegion.h"

namespace Rcs
{

/*! \ingroup RcsTaskRegion
 *  \brief Simple bounding box region.
 *
 *  This class just crops the desired task space values to be within the
 *  given box extents. There is no in-interval motion based on the
 *  gradient re-projection.
 *
 *  Example:
 *  \code
 *    <Task name="Position XYZ" controlVariable="XYZ" effector="EndEffector" >
 *      <TaskRegion type="BoundingBox" min="0.6 0 0.8" max="1 0.5 1.2" />
 *    </Task>
 *  \endcode
 */
class TaskRegionBoundingBox : public TaskRegion
{
public:

  TaskRegionBoundingBox(const Task* task, xmlNode* node);

  virtual ~TaskRegionBoundingBox();

  virtual TaskRegionBoundingBox* clone() const;

  /*! \brief Computes a cropped desired task space vector based on the given
   *         bounding box, and computes dx = x_des_cropped - x_curr.
   */
  virtual void computeDX(const Task* task, double* dx, const double* x_des,
                         const double* x_curr) const;

private:

  std::vector<double> bbMin, bbMax;
};

}

#endif // RCS_TASKREGIONBOUNDINGBOX_H
