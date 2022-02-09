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

#ifndef RCS_TASKDISTANCE1D_H
#define RCS_TASKDISTANCE1D_H

#include "TaskDistance3D.h"


namespace Rcs
{
/*! \ingroup RcsTask
 * \brief One component of the TaskDistance3D task.
 */
class TaskDistance1D: public TaskDistance3D
{
public:

  /*! Constructor based on xml parsing.
   */
  TaskDistance1D(const std::string& className, xmlNode* node,
                 RcsGraph* graph, int dim=1);

  /*! Constructor based on graph and effectors.
  */
  TaskDistance1D(RcsGraph* graph, const RcsBody* effector,
                 const RcsBody* refBdy, int index);

  /*! Destructor.
   */
  virtual ~TaskDistance1D();

  /*!
   * \brief Virtual copy constructor with optional new graph.
   */
  virtual TaskDistance1D* clone(RcsGraph* newGraph=NULL) const;

  /*! \brief Computes the current value of the task variable
   *
   *  The result is written to parameter \e x_res.
   */
  virtual void computeX(double* x_res) const;

  /*! \brief Computes current task Jacobian to parameter \e jacobian.
   *
   *  \param[out] jacobian Task Jacobian with dimension 3 x nJ
   *                       where nJ is the number of unconstrained degrees of
   *                       freedom (see \ref RcsGraph)
   */
  virtual void computeJ(MatNd* jacobian) const;

  /*! \brief Computes current task Hessian to parameter \e hessian.
   *
   *  \param[out] hessian Task Hessian with dimension nJ x (3*nJ)
   *                      where nJ is the number of unconstrained degrees of
   *                      freedom (see \ref RcsGraph).
   */
  virtual void computeH(MatNd* hessian) const;

  /*! \brief Returns true if the task is specified correctly, false
   *         otherwise. The following checks are performed:
   *         - XML tag "effector" exists and corresponds to body in graph
   *         - XML tag "refBdy" or "refBody"  exists and corresponds to body
   *           in graph
   *         - XML tag "controlVariable" is "Distance3D"
   */
  static bool isValid(xmlNode* xml_node, const RcsGraph* graph);

protected:

  int index;
};
}

#endif // RCS_TASKDISTANCE1D_H
