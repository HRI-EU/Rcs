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

#include "TaskRegionBoundingBox.h"
#include "TaskRegionFactory.h"
#include "Rcs_parser.h"
#include "Rcs_stlParser.h"
#include "Rcs_macros.h"
#include "Rcs_basicMath.h"
#include "Rcs_VecNd.h"

#include <cfloat>


namespace Rcs
{
REGISTER_TASKREGION(TaskRegionBoundingBox, "BoundingBox");

TaskRegionBoundingBox::TaskRegionBoundingBox(const Task* task, xmlNode* node) :
  TaskRegion(task, node)
{
  bbMin = std::vector<double>(task->getDim(), -DBL_MAX);
  bbMax = std::vector<double>(task->getDim(), DBL_MAX);

  size_t nVals = getXMLNodePropertyVecSTLDouble(node, "min", bbMin);
  RCHECK(nVals==task->getDim());
  nVals = getXMLNodePropertyVecSTLDouble(node, "max", bbMax);
  RCHECK(nVals==task->getDim());
}

TaskRegionBoundingBox::~TaskRegionBoundingBox()
{
}

Rcs::TaskRegionBoundingBox* Rcs::TaskRegionBoundingBox::clone() const
{
  return new TaskRegionBoundingBox(*this);
}

void TaskRegionBoundingBox::computeDX(const Task* task, double* dx,
                                      const double* x_des,
                                      const double* x_curr) const
{
  for (size_t i = 0; i < bbMin.size(); ++i)
  {
    double x_bb = Math_clip(x_des[i], bbMin[i], bbMax[i]);
    dx[i] = x_bb - x_curr[i];
  }

}

}   // namespace Rcs
