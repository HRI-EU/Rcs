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

#ifndef RCS_TASKPOSITION3D_H
#define RCS_TASKPOSITION3D_H

#include "TaskGenericIK.h"


namespace Rcs
{
/*! \ingroup RcsTask
 *  \brief This tasks allows to set a 3D position (XYZ) of an effector. The
 *         position can also be relative to another body and reference frame.
 *
 *  The task vector is computed as:
 *
 *  1. If a reference body does not exist:
 *
 *     A) No reference frame exist:
 *       I_r = I_r_effector
 *
 *     B) A reference frame and no reference body exist:
 *       refFrame_r = A_refFrame-I * I_r_effector
 *
 *  2. If a reference body exists:
 *
 *     A) No reference frame exists, or reference frame is equal to reference
 *        body: subtract its origin and rotate the result into the reference
 *        bodies's basis:
 *        refBdy_r = A_refBdy-I * (I_r - I_r_refBdy)
 *
 *     B) A reference frame that is different to the reference body exists:
 *        subtract its origin and rotate the result into the reference frame's
 *        basis:
 *        refFrame_r = A_refFrame-I * (I_r - I_r_refBdy)
 *
 *  The following xml tags are parsed in the xml constructor:
 *  - controlVariable: must be "XYZ"
 *  - effector="Name of effector RcsBody"
 *  - refBdy="Name of reference RcsBody"
 *  - refFrame="Name of reference frame RcsBody"
 *  - guiMin: three values for lower bounds slider values (default: -2.5)
 *  - guiMax: three values for lower bounds slider values (default: +2.5)
 *  - name: The task name will be displayed in Gui (unless it is the default
 *          name "Unnamed task")
 *
 *  Example:
 *  \code
 *    <Task name="Hand XYZ" controlVariable="XYZ" effector="HandTip" active="true" />
 *  \endcode
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

  /*! \brief Computes the relative position of the effector with respect to the
   *         reference body, projected into the coordinate system of the
   *         refFrame body. Any of these max or may not exist (in the sense that
   *         the body member is NULL). If it does not exist, it is considered
   *         the world reference.
   *
   *  The result is written to parameter \e x_res.
   */
  virtual void computeX(double* x_res) const;

  /*! \brief Computes the current velocity in task space:
   *         \f$
   *         \mathbf{\dot{x} = A_{refFrm-I}
   *                           (_I \dot{x}_{ef} - _I \dot{x}_{ref} +
   *                           r_{ref-ef} \times \omega_{ref} } )
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
