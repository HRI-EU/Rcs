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

#ifndef RCS_TASKDISTANCE3D_H
#define RCS_TASKDISTANCE3D_H

#include "TaskGenericIK.h"


namespace Rcs
{
/*! \ingroup RcsTask
 * \brief This tasks allows to set the cartesian position (XYZ) of an effector
 *        with respect to a reference body. The distance components are
 *        represented in the frame of the reference body.
 *
 *        It is assumed that the closest points between effector and reference
 *        body don't change under infinitesimal changes of the state. This is
 *        however not precisely true for surfaces with curvatures. The effect
 *        can be observed in the finite difference gradient tests, where there
 *        might be differences between analytic and numeric estimates of the
 *        Jacobians or Hessians.
 */
class TaskDistance3D: public TaskGenericIK
{
public:

  /*! Constructor based on xml parsing.
   */
  TaskDistance3D(const std::string& className, xmlNode* node,
                 RcsGraph* graph, int dim=3, float scaleFactor=1.0);

  /*! \brief Copy constructor doing deep copying with optional new graph
   *         pointer.
   */
  TaskDistance3D(const TaskDistance3D& src, RcsGraph* newGraph=NULL);

  /*! Constructor based on graph and effectors.
  */
  TaskDistance3D(RcsGraph* graph, const RcsBody* effector,
                 const RcsBody* refBdy, float scaleFactor=1.0);

  /*! Destructor.
   */
  virtual ~TaskDistance3D();

  /*! \brief Virtual clone method with optional new graph.
   */
  virtual TaskDistance3D* clone(RcsGraph* newGraph=NULL) const;

  /*! \brief Computes the relative distance between effector and reference
   *         body. It is represented in the reference bodie's frame.
   */
  virtual void computeX(double* x_res) const;

  /*! \brief Computes the relative distance Jacobian between effector and
   *         reference body according to \ref computeX().
   *
   *  \param[out] jacobian Task Jacobian with dimension 3 x nJ where nJ is
   *                       the number of unconstrained degrees of freedom (see
   *                       \ref RcsGraph)
   */
  virtual void computeJ(MatNd* jacobian) const;

  /*! \brief Computes the relative distance Hessian between effector and
   *         reference body according to \ref computeX().
   *
   *  \param[out] hessian Task Hessian with dimension nJ x (3*nJ) where nJ is
   *                      the number of unconstrained degrees of freedom (see
   *                      \ref RcsGraph).
   */
  virtual void computeH(MatNd* hessian) const;

  /*! \brief Returns true if the task is specified correctly, false
   *         otherwise. The following checks are performed:
   *         - XML tag "effector" exists and corresponds to body in graph
   *         - XML tag "refBdy" or "refBody"  exists and corresponds to body
   *           in graph
   *         - XML tag "controlVariable" is "Distance3D"
   */
  static bool isValid(xmlNode* node, const RcsGraph* graph);
};

}

#endif // RCS_TASKDISTANCE3D_H
