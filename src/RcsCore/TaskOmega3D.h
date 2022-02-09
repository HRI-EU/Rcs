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

#ifndef RCS_TASKOMEGA3D_H
#define RCS_TASKOMEGA3D_H

#include "TaskGenericIK.h"


namespace Rcs
{

/*! \ingroup RcsTask
 * \brief This tasks allows to set a 3D angular velocity.
 *
 *        The angular velocity is represented in the coordinates of the
 *        reference body (refBdy). If it is not specified, it is represented
 *        in world coordinates.
 *        In order to get the angular velocity in the bodie's frame, the
 *        effector and refFrame bodies must be the same, and no refBdy
 *        must be given.
 *
 */
class TaskOmega3D: public TaskGenericIK
{
public:

  using Task::computeDX;

  /*! Constructor based on graph and effectors.
   */
  TaskOmega3D(RcsGraph* graph, const RcsBody* effector,
              const RcsBody* refBdy, const RcsBody* refFrame);

  /*! Constructor based on xml parsing
   */
  TaskOmega3D(const std::string& className, xmlNode* node, RcsGraph* graph);

  /*! \brief Virtual destructor so that classes can properly inherit from this.
   */
  virtual ~TaskOmega3D();

  /*! \brief Virtual copy constructor with optional new graph
   */
  virtual TaskOmega3D* clone(RcsGraph* newGraph=NULL) const;

  /*!  \brief Computes current task Jacobian to parameter \e jacobian
   */
  virtual void computeJ(MatNd* jacobian) const;

  /*!  \brief Computes current task Hessian to parameter \e hessian
   */
  virtual void computeH(MatNd* hessian) const;

  /*! \brief Computes the current value of the task variable
   *
   *  The result is written to parameter \e x_res.
   */
  virtual void computeX(double* x_res) const;

  /*!  \brief Copies x_des into dx. Both pointers must point to 3d-arrays (or
   *          more)
   */
  virtual void computeDX(double* dx, const double* x_des) const;

  /*! \brief Returns true if the task is specified correctly, false otherwise.
   *         The following chechs are carried out:
   *         - tag "controlVariable" contains "ABCd"
   */
  static bool isValid(xmlNode* node, const RcsGraph* graph);
};

}

#endif // RCS_TASKOMEGA3D_H
