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

#ifndef RCS_TASKRADIAL_H
#define RCS_TASKRADIAL_H

#include "TaskPosition3D.h"


namespace Rcs
{
/*! \ingroup RcsTask
 * \brief This tasks allows to set a 3D position (cylindrical coordinates) of
 *        an effector. The position can also be relative to another body and
 *        reference frame. This class calculates purely on cylinder coordinates.
 *        This allows to decompose the motion into cylinder coordinate elements,
 *        for instance when performing hybrid position-force control: A radial
 *        force is independent of an angular velocity. This is very general,
 *        has however some drawbacks, for instance singularities for a radius
 *        of zero. If you don't need this component-wise orthogonal
 *        decomposition, the task  TaskCylindricalPos3D might be the better
 *        and more robust choice.
 */
class TaskRadial : public Rcs::TaskPosition3D
{
public:

  using Task::computeDX;

  /*! \brief Constructor based on xml parsing
   */
  TaskRadial(const std::string& className, xmlNode* node,
             const RcsGraph* graph, int dim=1);

  /*! \brief Virtual copy constructor with optional new graph
   */
  virtual TaskRadial* clone(const RcsGraph* newGraph=NULL) const;

  /*! \brief Computes the current value of the task variable. The result is
   *         written to parameter \e x_res. Reuses TaskPosition3D::computeX
   *         and then converts to cylindrical coordinates
   */
  virtual void computeX(double* x_res) const;

  /*! \brief Computes the current velocity in task space. Reuses
   *         TaskPosition3D::computeXp and then converts to cylindrical
   *         coordinates
   */
  virtual void computeXp(double* xp) const;

  /*! \brief Computes current task Jacobian to parameter \e jacobian.
   *         Reuses TaskPosition3D::computeJ and then converts to
   *         cylindrical coordinates
   */
  virtual void computeJ(MatNd* jacobian) const;

  /*! \brief Computes the delta in task space for the differential
   *        kinematics. Ensures that always the shortest path in phi is
   *        followed
   */
  virtual void computeDX(double* dx, const double* x_des) const;

  /*! \brief Computes current task Hessian to parameter \e hessian
   *
   * Not implemented!
   */
  virtual void computeH(MatNd* hessian) const;

  /*! \brief Computes the current derivative Jacobian. Reuses
   *         TaskPosition3D::computeJdot and then converts to cylindrical
   *         coordinates
   */
  virtual void computeJdot(MatNd* Jdot) const;

  virtual bool testHessian(bool verbose=false) const;

  /*! \brief Returns true if the task is specified correctly, false
   *         otherwise:
   *         - XML tag "effector" corresponds to body in graph
   *         - XML tag "controlVariable" is "CylRPZ"
   */
  static bool isValid(xmlNode* xml_node, const RcsGraph* graph);
};

}

#endif // RCS_TASKRADIAL_H
