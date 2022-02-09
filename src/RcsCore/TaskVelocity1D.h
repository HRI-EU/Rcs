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

#ifndef TASKVELOCITY1D_H
#define TASKVELOCITY1D_H

#include "TaskPosition1D.h"

namespace Rcs
{

/*! \ingroup RcsTask
 * \brief This tasks allows to set a 1D position (X, Y, or Z) of an effector
 *
 *  The position can also be relative to another body and reference frame.
 */
class TaskVelocity1D: public Rcs::TaskPosition1D
{
public:

  using Task::computeDX;

  /*! Constructor based on xml parsing
   */
  TaskVelocity1D(const std::string& className, xmlNode* node,
                 RcsGraph* graph, int dim=1);

  /*! Constructor based on graph and effectors.
   */
  TaskVelocity1D(const std::string& className, RcsGraph* graph,
                 const RcsBody* effector, const RcsBody* refBdy=NULL,
                 const RcsBody* refFrame=NULL);

  /*! Virtual destructor to allow correct polymorphism.
   */
  virtual ~TaskVelocity1D();

  /*! \brief Returns a deep copy of a task.
   */
  virtual TaskVelocity1D* clone(RcsGraph* newGraph=NULL) const;

  virtual void computeX(double* x_res) const;

  /*! \brief In this particular task, the x_des command corresponds to the
   *         desired velocity. Therefore it is just copied into dx.
   */
  virtual void computeDX(double* dx, const double* x_des) const;

  /*! \brief Returns true, since this class has no good finite difference
   *         Jacobian test.
   */
  virtual bool testJacobian(double errorLimit=1.0e-4, double delta=1.0e-6,
                            bool relativeError=false, bool verbose=false);

  /*! \brief Returns true for success, false otherwise:
   *         - Xml tag "controlVariable" is not "Xd", "Yd" or "Zd".
   */
  static bool isValid(xmlNode* node, const RcsGraph* graph);
};

}

#endif // TASKVELOCITY1D_H
