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

#ifndef RCS_TASKJOINTLIMIT_H
#define RCS_TASKJOINTLIMIT_H

#include "TaskGenericIK.h"


namespace Rcs
{

/*! \ingroup RcsTask
 * \brief Joint limit avoidance task.
 */
class TaskJointLimit: public Rcs::TaskGenericIK
{
public:

  /*! Constructor based on xml parsing
   */
  TaskJointLimit(const std::string& className, xmlNode* node,
                 RcsGraph* graph, int dim=1);

  /*! \brief Copy constructor doing deep copying with optional new graph
   *         pointer
   */
  TaskJointLimit(const TaskJointLimit& copyFromMe, RcsGraph* newGraph=NULL);

  /*! Destructor
   */
  virtual ~TaskJointLimit();

  /*!
   * \brief Virtual copy constructor with optional new graph
   */
  virtual TaskJointLimit* clone(RcsGraph* newGraph=NULL) const;

  /*! \brief Computes the current value of the task variable
   *
   *  \param[out] x_res Joint limit cost according to function
   *                    \ref RcsGraph_jointLimitBorderCost
   */
  virtual void computeX(double* x_res) const;

  /*! \brief Computes current task Jacobian to parameter \param jacobian
   *
   *  \param[out] jacobian Joint limit gradient of dimension
   *              1 x RcsGraph::nJ according to function
   *              \ref RcsGraph_jointLimitBorderGradient
   */
  virtual void computeJ(MatNd* jacobian) const;

  /*! \brief Computes current task Hessian to parameter \param hessian
   *
   *  \param[out] hessian Joint limit Hessian of dimension
   *              RcsGraph::nJ x RcsGraph::nJ according to function
   *              \ref RcsGraph_jointLimitBorderHessian
   */
  virtual void computeH(MatNd* hessian) const;

  /*! \brief Returns true if the task is specified correctly, false
   *         otherwise. The task is invalid if
   *         - the value in tag "borderRatio" is not inside [0...1]
   */
  static bool isValid(xmlNode* node, const RcsGraph* graph);

protected:

  double borderRatio;
};

}

#endif // RCS_TASKJOINTLIMIT_H
