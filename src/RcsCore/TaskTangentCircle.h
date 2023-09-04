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

#ifndef RCS_TASKTANGENTCIRCLE_H
#define RCS_TASKTANGENTCIRCLE_H

#include "Task.h"


namespace Rcs
{

/*! \ingroup RcsTask
 * \brief This task constrains an effector to a circle around the refBdy's
 *        z-axis, while maintaining the y-axis tangential to the circle.
 */
class TaskTangentCircle: public Task
{
public:

  /*! Constructor based on xml parsing
   */
  TaskTangentCircle(const std::string& className, xmlNode* node,
                    const RcsGraph* graph, int dim=1);

  /*! \brief Polymorphic clone function.
   */
  virtual TaskTangentCircle* clone(const RcsGraph* newGraph=NULL) const;

  /*! \brief Computes the Polar Angles between reference body and effector.
   */
  virtual void computeX(double* polarAngles) const;

  virtual void computeXp(double* polarVelocity) const;

  virtual void computeXpp(double* polarAcceleration, const MatNd* qpp) const;

  virtual void computeDXp(double* dOmega, const double* phip_des) const;

  virtual void computeJ(MatNd* jacobian) const;

  virtual void computeH(MatNd* hessian) const;

  virtual void computeDX(double* ref_omega, const double* a_des) const;

  virtual void computeDX(double* ref_omega, const double* a_des,
                         const double* a_curr) const;

  virtual void integrateXp_ik(double* x_res, const double* x,
                              const double* x_dot, double dt) const;

  virtual void forceTrafo(double* ft_task) const;

  virtual void selectionTrafo(double* S_des_trafo, const double* S_des) const;

  /*! \brief Computes the feed-forward acceleration. The argument xpp_res is
   *         expected to be in Jacobi-coordinate.
   *
   *  \param[out] xpp_res    Resulting acceleration. It is represented
   *                         in the IK-relevant coordinates.
   *  \param[in]  xpp_des    Vector of desired task acceleration. It is
   *                         represented in the task-relevant coordinates.
   */
  virtual void computeFfXpp(double* xpp_res, const double* xpp_des) const;

  /*! \brief Returns true if the task is specified correctly, false
   *         otherwise.
   */
  static bool isValid(xmlNode* node, const RcsGraph* graph);

  virtual bool test(bool verbose=false) const;
};

}

#endif // RCS_TASKTANGENTCIRCLE_H
