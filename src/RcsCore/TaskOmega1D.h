/*******************************************************************************

  Copyright (c) 2017, Honda Research Institute Europe GmbH.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.

  3. All advertising materials mentioning features or use of this software
     must display the following acknowledgement: This product includes
     software developed by the Honda Research Institute Europe GmbH.

  4. Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER "AS IS" AND ANY EXPRESS OR
  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#ifndef RCS_TASKOMEGA1D_H
#define RCS_TASKOMEGA1D_H

#include "TaskGenericIK.h"


namespace Rcs
{

/*! \ingroup RcsTask
 * \brief This tasks allows to set a 1D angular velocity.
 *
 *        The angular velocity is represented in the coordinates of the
 *        reference body (refBdy). If it is not specified, it is represented
 *        in world coordinates.
 *        In order to get the agular velocity in the bodie's frame, the
 *        effector and refFrame bodies must be the same, and no refBdy
 *        must be given.
 *
 */
class TaskOmega1D: public TaskGenericIK
{
public:

  using Task::computeDX;

  /*! Constructor based on xml parsing
   */
  TaskOmega1D(const std::string& className, xmlNode* node, RcsGraph* _graph,
              int dim=1);

  /*! Constructor based on graph and effectors for programmatic construction.
   */
  TaskOmega1D(const std::string& className, RcsGraph* graph,
              const RcsBody* effector, const RcsBody* refBdy=NULL,
              const RcsBody* refFrame=NULL);

  /*! \brief Copy constructor doing deep copying with optional new graph pointer
   */
  TaskOmega1D(const TaskOmega1D& copyFromMe, RcsGraph* newGraph=NULL);

  /*! Destructor
   */
  virtual ~TaskOmega1D();

  /*!
   * \brief Virtual copy constructor with optional new graph
   */
  virtual TaskOmega1D* clone(RcsGraph* newGraph=NULL) const;

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

  virtual void computeDX(double* dx, const double* x_des) const;

  /*! \brief Returns true if the task is specified correctly, false otherwise.
   *         The following chechs are carried out:
   *         - tag "controlVariable" contains "Ad", "Bd" or "Cd"
   */
  static bool isValid(xmlNode* node, const RcsGraph* graph);

protected:

  int index;
};

}

#endif // RCS_TASKOMEGA1D_H
