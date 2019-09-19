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

#ifndef RCS_TASKEULER1D_H
#define RCS_TASKEULER1D_H

#include "TaskGenericIK.h"


namespace Rcs
{

/*! \ingroup RcsTask
 *  \brief This tasks allows to set a 1D orientation (One Euler XYZ angle
 *         component) of an effector. It gets singular for cos(beta) = 0 which
 *         is periodically happening for beta = pi/2 + k*pi
 *
 *         The orientation can also be relative to another body and reference
 *         frame.
 */
class TaskEuler1D: public TaskGenericIK
{
public:

  /*! Constructor based on xml parsing
   */
  TaskEuler1D(const std::string& className, xmlNode* node, RcsGraph* graph,
              int dim=1);

  /*! Constructor based on graph and effectors for programmatic construction.
   */
  TaskEuler1D(const std::string& className, RcsGraph* graph,
              const RcsBody* effector, const RcsBody* refBdy=NULL,
              const RcsBody* refFrame=NULL);

  /*! \brief Copy constructor doing deep copying with optional new graph
   *         pointer
   */
  TaskEuler1D(const TaskEuler1D& copyFromMe, RcsGraph* newGraph=NULL);

  /*! Virtual destructor to allow proper inheriting from this class.
   */
  virtual ~TaskEuler1D();

  /*!
   * \brief Virtual copy constructor with optional new graph
   */
  virtual TaskEuler1D* clone(RcsGraph* newGraph=NULL) const;

  /*! \brief Computes the current value of the task variable
   *
   *  The result is written to parameter \e x_res.
   */
  virtual void computeX(double* x_res) const;

  /*! \addtogroup TaskMethodsThroughInheritance
   *  \brief Computes the Euler velocity of the task space component. In this
   *         class's implementation, it is calculated as
   *         \f$
   *         \mathbf{\dot{x} = H \dot{\omega} }
   *         \f$
   *
   *  \param[out] x_dot Task space velocity
   */
  virtual void computeXp_ik(double* x_dot) const;

  void computeXpp(double* eapp, const MatNd* qpp) const;

  /*! \brief Computes current task Jacobian to parameter \e jacobian
   */
  virtual void computeJ(MatNd* jacobian) const;

  /*! \brief Computes the current task Hessian to parameter \e hessian
   */
  virtual void computeH(MatNd* hessian) const;

  /*! \brief Returns true if the task is specified correctly, false otherwise:
   *         - XML tag "controlVariable" must be "A", "B" or "C"
   *         - Body with name in XML tag "effector" must exist in graph
   */
  static bool isValid(xmlNode* xml_node, const RcsGraph* graph);

protected:
  int index;   ///< Rotation axis index: 0 for x, 1 for y, 2 for z
};

}

#endif // RCS_TASKEULER1D_H
