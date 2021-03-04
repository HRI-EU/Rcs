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

#ifndef TASKCYLINDRICAL1D_H
#define TASKCYLINDRICAL1D_H

#include "TaskCylindrical3D.h"

namespace Rcs
{
/*! \ingroup RcsTask
 * \brief This tasks allows to set a 1D position (in cylindrical coordinates)
 *        of an effector. The position can also be relative to another body
 *        and reference frame.
 */
class TaskCylindrical1D : public Rcs::TaskCylindrical3D
{
public:

  /*! \brief Constructor based on xml parsing
   */
  TaskCylindrical1D(const std::string& className, xmlNode* node,
                    RcsGraph* graph, int dim=1);

  /*! \brief Copy constructor doing deep copying with optional new graph
   *         pointer
   */
  TaskCylindrical1D(const TaskCylindrical1D& copyFromFe,
                    RcsGraph* newGraph=NULL);

  /*! \brief Destructor
   */
  virtual ~TaskCylindrical1D();

  /*! \brief Virtual copy constructor with optional new graph
   */
  virtual TaskCylindrical1D* clone(RcsGraph* newGraph=NULL) const;

  /*! \brief Computes the current value of the task variable
   *
   *  Reuse implementation of TaskCylindrical3D, but select only relevant
   *  component
   */
  virtual void computeX(double* x_res) const;

  /*! \brief Computes the current velocity component in task space
   *
   *  Reuse implementation of TaskCylindrical3D, but select only relevant
   *  component
   */
  virtual void computeXp(double* xp) const;

  /*! \brief Computes current task Jacobian to parameter \e jacobian
   *
   *  Reuse implementation of TaskCylindrical3D, but select only relevant
   *  component
   */
  virtual void computeJ(MatNd* jacobian) const;

  /*! \brief Computes the displacement in task space
   *
   *   TaskCylindrical3D::computeDX cannot be reused as it calls
   *  TaskCylindrical1D::computeX (virtual function)
   */
  virtual void computeDX(double* dx, const double* x_des) const;

  /*! \brief Computes current task Jacobian derivative to parameter \e Jdot
   *
   *  Reuse implementation of TaskCylindrical3D, but select only relevant
   *  component
   */
  virtual void computeJdot(MatNd* Jdot) const;

  /*! \brief Computes current task Hessian to parameter \e hessian
   *
   *  Reuse implementation of TaskCylindrical3D, but select only relevant
   *  component
   */
  virtual void computeH(MatNd* hessian) const;

  /*! \brief Returns true if the task is specified correctly, false
   *         otherwise. The following checks are performed:
   *         - XML tag "effector" corresponds to body in graph
   *         - XML tag "controlVariable" is "CylR" or "CylP"
   */
  static bool isValid(xmlNode* node, const RcsGraph* graph);

protected:
  int index;

};
}

#endif // TASKCYLINDRICAL1D_H
