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

#ifndef TASKVELOCITY3D_H
#define TASKVELOCITY3D_H

#include "TaskPosition3D.h"

namespace Rcs
{

/*! \ingroup RcsTask
 * \brief This tasks allows to set a 3D velocity (X, Y, or Z) of an effector
 *
 *  The position can also be relative to another body and reference frame.
 */
class TaskVelocity3D: public Rcs::TaskPosition3D
{
public:

  using Task::computeDX;

  /*! Constructor based on graph and effectors.
   */
  TaskVelocity3D(RcsGraph* graph, const RcsBody* effector,
                 const RcsBody* refBdy, const RcsBody* refFrame);

  /*! Constructor based on xml parsing
   */
  TaskVelocity3D(const std::string& className, xmlNode* node,
                 RcsGraph* graph, int dim=1);

  /*! \brief Copy constructor doing deep copying with optional new graph
   *         pointer
   */
  TaskVelocity3D(const TaskVelocity3D& copyFromMe, RcsGraph* newGraph=NULL);

  /*! Destructor
   */
  virtual ~TaskVelocity3D();

  /*!
   * \brief Virtual copy constructor with optional new graph
   */
  virtual TaskVelocity3D* clone(RcsGraph* newGraph=NULL) const;

  virtual void computeX(double* x_res) const;

  virtual void computeDX(double* dx, const double* x_des) const;

  virtual void computeDXp(double* dxp_res, const double* desiredVel) const;

  /*! \brief Returns true for success, false otherwise:
   *         - Xml tag "controlVariable" is not "Xd", "Yd" or "Zd".
   */
  static bool isValid(xmlNode* node, const RcsGraph* graph);

protected:
  double velocity_des_temp[3];
};

}

#endif // TASKVELOCITY3D_H
