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

#include "TaskRegionBoxInterval.h"
#include "TaskRegionFactory.h"
#include "Rcs_parser.h"
#include "Rcs_stlParser.h"
#include "Rcs_macros.h"
#include "Rcs_basicMath.h"
#include "Rcs_VecNd.h"
#include "Rcs_typedef.h"
#include "Rcs_kinematics.h"

#include <cfloat>


namespace Rcs
{
REGISTER_TASKREGION(TaskRegionBoxInterval, "BoxInterval");

TaskRegionBoxInterval::TaskRegionBoxInterval(const Task* task, xmlNode* node) :
  TaskRegion(task, node), slowDownRatio(0.0)
{
  getXMLNodePropertyDouble(node, "slowDownRatio", &slowDownRatio);
  RCHECK_MSG((slowDownRatio >= 0) && (slowDownRatio <= 1.0),
             "slowDownRatio is %f but must be [0...1]", slowDownRatio);

  bbMin = std::vector<double>(task->getDim(), -DBL_MAX);
  bbMax = std::vector<double>(task->getDim(), DBL_MAX);

  std::vector<double> tmp = getXMLNodePropertyVecSTLDouble(node, "min");

  // For parsing min and max interval bounds, we allow one value, or as many
  // values as there are task dimensions.
  if (tmp.size() == 1)
  {
    bbMin = std::vector<double>(task->getDim(), tmp[0]);
  }
  else if (tmp.size() == task->getDim())
  {
    bbMin = tmp;
  }
  else if (!tmp.empty())
  {
    RFATAL("Dimension mismatch: attribute \"min\" has %zu values, but task"
           " dimension is %u", tmp.size(), task->getDim());
  }

  tmp = getXMLNodePropertyVecSTLDouble(node, "max");

  if (tmp.size() == 1)
  {
    bbMax = std::vector<double>(task->getDim(), tmp[0]);
  }
  else if (tmp.size() == task->getDim())
  {
    bbMax = tmp;
  }
  else if (!tmp.empty())
  {
    RFATAL("Dimension mismatch: attribute \"min\" has %zu values, but task"
           " dimension is %u", tmp.size(), task->getDim());
  }

}

TaskRegionBoxInterval::~TaskRegionBoxInterval()
{
}

Rcs::TaskRegionBoxInterval* Rcs::TaskRegionBoxInterval::clone() const
{
  return new TaskRegionBoxInterval(*this);
}

void TaskRegionBoxInterval::computeDX(const Task* task, double* dx,
                                      const double* x_des,
                                      const double* x_curr) const
{

  for (size_t i = 0; i < bbMin.size(); ++i)
  {
    const double x_lb = x_des[i] + bbMin[i]; // plus, since sign comes from xml
    const double x_ub = x_des[i] + bbMax[i];

    dx[i] = dx_proj[i];

    const double slowRange = 0.5*slowDownRatio*(bbMax[i]-bbMin[i]);

    if (slowRange>0.0)
    {
      if ((x_curr[i] < x_lb + slowRange) && (dx[i] < 0.0))
      {
        dx[i] *= Math_clip((x_curr[i] - x_lb) / slowRange, 0.0, 1.0);
      }
      else if ((x_curr[i] > x_ub - slowRange) && (dx[i] > 0.0))
      {
        dx[i] *= Math_clip((x_ub - x_curr[i]) / slowRange, 0.0, 1.0);
      }
    }

    // Limit so that dx does not bring us out
    double x_next = x_curr[i] + dx[i];

    if (x_next < x_lb)
    {
      dx[i] += x_lb - x_next;
    }
    else if (x_next > x_ub)
    {
      dx[i] -=  x_next - x_ub;
    }

  }

}

}   // namespace Rcs
