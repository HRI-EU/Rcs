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

#ifndef RCS_TASKPOSITION3D_H
#define RCS_TASKPOSITION3D_H

#include "TaskGenericIK.h"


namespace Rcs
{
/*! \ingroup RcsTask
 * \brief This tasks allows to set a 3D position (XYZ) of an effector
 *
 *  This tasks allows to set a 3D position (XYZ) of an effector. The position
 *  can also be relative to another body and reference frame.
 */
class TaskPosition3D: public TaskGenericIK
{
public:

  /*! Constructor based on xml parsing.
   */
  TaskPosition3D(const std::string& className, xmlNode* node,
                 RcsGraph* graph, int dim=3);

  /*! \brief Copy constructor doing deep copying with optional new graph
   *         pointer.
   */
  TaskPosition3D(const TaskPosition3D& src, RcsGraph* newGraph=NULL);

  /*! Constructor based on graph and effectors.
   */
  TaskPosition3D(RcsGraph* graph, const RcsBody* effector,
                 const RcsBody* refBdy, const RcsBody* refFrame);

  /*! Destructor.
   */
  virtual ~TaskPosition3D();

  /*!
   * \brief Virtual copy constructor with optional new graph.
   */
  virtual TaskPosition3D* clone(RcsGraph* newGraph=NULL) const;

  /*! \brief Computes the current value of the task variable
   *
   *  The result is written to parameter \e x_res.
   */
  virtual void computeX(double* x_res) const;

  /*! \brief Computes the current velocity in task space:
   *         \f$
   *         \mathbf{\dot{x} = A_{ref-I}
   *                           (_I \dot{x}_{ef} - _I \dot{x}_{ref}) }
   *         \f$
   */
  virtual void computeXp_ik(double* xp) const;

  /*! \brief Computes current task Jacobian to parameter \e jacobian.
   *         Internally, the function \ref RcsGraph_3dPosJacobian() is called.
   *
   *  \param[out] jacobian Task linear Jacobian with dimension 3 x nJ
   *                       where nJ is the number of unconstrained degrees of
   *                       freedom (see \ref RcsGraph)
   */
  virtual void computeJ(MatNd* jacobian) const;

  /*! \brief Computes current task Hessian to parameter \e hessian. Internally,
   *         the function \ref RcsGraph_3dPosHessian() is called.
   *
   *  \param[out] hessian Task linear Hessian with dimension nJ x (3*nJ)
   *                      where nJ is the number of unconstrained degrees of
   *                      freedom (see \ref RcsGraph).
   */
  virtual void computeH(MatNd* hessian) const;

  /*! \brief Returns true if the task is specified correctly, false
   *         otherwise. The following checks are performed:
   *         - XML tag "effector" corresponds to body in graph
   *         - XML tag "controlVariable" is "XYZ"
   */
  static bool isValid(xmlNode* xml_node, const RcsGraph* graph);
};

}

#endif // RCS_TASKPOSITION3D_H
