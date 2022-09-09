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

#include "TaskRegion.h"
#include "TaskRegionFactory.h"
#include "Rcs_VecNd.h"

// Specializations of computeDX() needed for:
// - TaskCylindrical3D
// - TaskCylindrical1D
// - TaskCylindricalPos3D
// - TaskDistance
// - TaskEuler3D
// - TaskGenericEuler3D
// - TaskGenericIK
// - TaskInclination
// - TaskOmega1D
// - TaskOmega3D
// - TaskPolar2D
// - TaskPolarSurfaceNormal
// - TaskPolarTarget2D
// - TaskPositionTarget3D
// - TaskSpherical1D
// - TaskSpherical3D
// - TaskVelocity1D
// - TaskVelocity3D
// - TaskVelocityJoint



namespace Rcs
{
REGISTER_TASKREGION(TaskRegion, "SetPoint");

TaskRegion::TaskRegion(const Task* task, xmlNode* node)
{
  dx_proj = std::vector<double>(task->getDim(), 0.0);
}

TaskRegion::TaskRegion(const TaskRegion& src) : dx_proj(src.dx_proj)
{
}

TaskRegion::~TaskRegion()
{
}

Rcs::TaskRegion* Rcs::TaskRegion::clone() const
{
  return new TaskRegion(*this);
}

void Rcs::TaskRegion::computeDX(const Task* task, double* dx,
                                const double* x_des, const double* x_curr) const
{
  VecNd_sub(dx, x_des, x_curr, task->getDim());
}

void Rcs::TaskRegion::setTaskReprojection(const std::vector<double>& dx)
{
  dx_proj = dx;
}

void Rcs::TaskRegion::toXML(FILE* out, bool activation) const
{
}

}
