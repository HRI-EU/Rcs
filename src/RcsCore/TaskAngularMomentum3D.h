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

#ifndef RCS_TASKANGULARMOMENTUM3D_H
#define RCS_TASKANGULARMOMENTUM3D_H

#include "TaskGenericIK.h"


namespace Rcs
{

/*! \ingroup RcsTask
 *  \brief
 */
class TaskAngularMomentum3D: public Rcs::TaskGenericIK
{
public:

  /*! \brief
   */
  TaskAngularMomentum3D(const std::string& className, xmlNode* node,
                        RcsGraph* graph, int dim=3);

  /*! \brief Copy constructor doing deep copying with optional new graph
   *         pointer
   */
  TaskAngularMomentum3D(const TaskAngularMomentum3D& copyFromMe,
                        RcsGraph* newGraph=NULL);

  /*! \brief
   */
  virtual ~TaskAngularMomentum3D();

  /*!
   * \brief Virtual copy constructor with optional new graph
   */
  virtual TaskAngularMomentum3D* clone(RcsGraph* newGraph=NULL) const;

  /*! \brief
   */
  virtual void computeX(double* x_res) const;

  /*! \brief
   */
  virtual void computeJ(MatNd* jacobian) const;

  /*! \brief Computes the current task Hessian to parameter \e hessian
   */
  virtual void computeH(MatNd* hessian) const;

  /*! \brief Returns true if the task is specified correctly, false
   *         otherwise. The following checks are performed:
   *         - tag "controlVariable" contains "AngularMomentum"
   */
  static bool isValid(xmlNode* node, const RcsGraph* graph);
};

}

#endif // RCS_TASKANGULARMOMENTUM3D_H
