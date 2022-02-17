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

#ifndef RCS_TASKVELOCITYJOINT_H
#define RCS_TASKVELOCITYJOINT_H

#include "TaskJoint.h"

namespace Rcs
{

/*! \ingroup RcsTask
 * \brief This tasks allows to set a joint velocity.
 */
class TaskVelocityJoint: public TaskJoint
{
public:

  using Task::computeDX;

  TaskVelocityJoint(const std::string& className, xmlNode* node,
                    const RcsGraph* graph, int dim=1);

  virtual TaskVelocityJoint* clone(const RcsGraph* newGraph=NULL) const;

  virtual void computeX(double* x_res) const;

  virtual void computeDX(double* dx, const double* x_des) const;

  virtual void computeDXp(double* dxp_res, const double* desired_vel) const;

  /*! \brief Returns true for success, false otherwise:
   *         - Xml tag "controlVariable" is not "Jointd".
   *         - Joint with the tag "jnt" must exist in the graph
   */
  static bool isValid(xmlNode* node, const RcsGraph* graph);
};

}

#endif // RCS_TASKVELOCITYJOINT_H
