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

#ifndef RCS_TASKPOSITION2D_H
#define RCS_TASKPOSITION2D_H

#include "TaskPosition3D.h"

namespace Rcs
{

/*! \ingroup RcsTask
 *  \brief This tasks allows to set a 2D position (XY, XZ, or YZ) of an
 *         effector.
 *
 *  The task calls the methods of TaskPosition3D, and extracts the components
 *  of the arrays that correspond to the given indices.
 *
 *  Example:
 *  \code
 *    <Task name="Hand XY" controlVariable="XY" effector="Hand" active="true" />
 *  \endcode
 */
class TaskPosition2D: public Rcs::TaskPosition3D
{
public:

  /*! Constructor based on xml parsing
   */
  TaskPosition2D(const std::string& className, xmlNode* node,
                 const RcsGraph* graph, int dim=2);

  /*! Constructor based on graph and effectors.
   */
  TaskPosition2D(const std::string& className, const RcsGraph* graph,
                 const RcsBody* effector=NULL, const RcsBody* refBdy=NULL,
                 const RcsBody* refFrame=NULL);

  /*! \brief Virtual copy constructor with optional new graph
   */
  virtual TaskPosition2D* clone(const RcsGraph* newGraph=NULL) const;

  /*! \brief Computes the current value of the task variable
   *
   *  Reuse implementation of TaskPosition3D, but select only relevant
   *  component
   */
  virtual void computeX(double* x_res) const;

  /*! \brief Computes the current velocity component in task space:
   *         \f$
   *         \mathbf{\dot{x} = A_{ref-I} (_I \dot{x}_{ef} - _I \dot{x}_{ref}) }
   *         \f$
   */
  virtual void computeXp_ik(double* xp) const;

  /*! \brief Computes current task Jacobian to parameter \e jacobian
   *
   *  Reuse implementation of TaskPosition3D, but select only relevant
   *  component
   */
  virtual void computeJ(MatNd* jacobian) const;

  /*! \brief Computes current task Hessian to parameter \e hessian
   */
  virtual void computeH(MatNd* hessian) const;

  /*! \brief Returns true if the task is specified correctly, false
   *         otherwise. The following checks are performed:
   *         - XML tag "effector" corresponds to body in graph
   *         - XML tag "controlVariable" is "XY", "XZ" or "YZ"
   */
  static bool isValid(xmlNode* node, const RcsGraph* graph);

protected:

  int index1, index2;
};
}

#endif // RCS_TASKPOSITION2D_H