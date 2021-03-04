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

#ifndef RCS_TASKPOLARSURFACENORMAL_H
#define RCS_TASKPOLARSURFACENORMAL_H

#include "Task.h"


namespace Rcs
{

/*! \ingroup RcsTask
 * \brief This tasks allows to set a 2D orientation (Polar angles) of an
 *        effector. The orientation can also be relative to another body and
 *        reference frame. The polar axis is taken from the effector body. By
 *        default, it is the z-axis. It can be specified with the xml tag
 *        - axisDirection="X"
 *        - axisDirection="Y"
 *        - axisDirection="Z"
 *        to select which axis direction from the effector body is used.
 *
 *        To avoid jumps in the feedback, the velocities and accelerations are
 *        calculated in the effector's angular velocity (with respect to the
 *        reference body, which is the world rfame if none is given).
 */
class TaskPolarSurfaceNormal: public Task
{
public:

  /*! Constructor based on xml parsing
   */
  TaskPolarSurfaceNormal(const std::string& className, xmlNode* node,
                         RcsGraph* graph, int dim=2);

  /*! \brief Copy constructor doing deep copying with optional new graph
   *         pointer
   */
  TaskPolarSurfaceNormal(const TaskPolarSurfaceNormal& copyFromMe,
                         RcsGraph* newGraph=NULL);

  /*! Destructor
   */
  virtual ~TaskPolarSurfaceNormal();

  /*!
   * \brief Virtual copy constructor with optional new graph.
   */
  virtual TaskPolarSurfaceNormal* clone(RcsGraph* newGraph=NULL) const;

  /*! \brief The first component of the 2-dimensional polarAngles is the angle
  *          between current Polar axis and the distance normal between
  *          refBody and effector. It will always be within the angular
  *          range [0 ... PI]. The second component is zero.
   */
  virtual void computeX(double* polarAngles) const;

  /*! \brief Angular velocity Jacobian, projected into the directions as
   *         described in \ref TaskPolarSurfaceNormal::computeX
   */
  virtual void computeJ(MatNd* jacobian) const;

  /*! \brief Calculates the angular error as the angle between the current and
   *         desired Polar axes (omega[0]). The first component of polar_des
   *         is the desired angle between current and desired polar axes. The
   *         desired Polar axis is the distance normal between refBody and
   *         effector. It is be clipped to the range [0 ... PI]. The second
   *         component is ignored. The corresponding component (omega[1]) is
   *         always set to zero.
   *
   *  \param[out] omega Two-dimensional angular error as described above
   *  \param[in] polar_des First element: Angle between desired and current
   *                       Polar axes. Second element: ignored.
   *  \param[in] polar_curr current polar angles (represented in world
   *                        coordinates)
   */
  virtual void computeDX(double* omega, const double* polar_des,
                         const double* polar_curr) const;

  /*! \brief Returns true if the task is specified correctly, false
   *         otherwise. The task is invalid if
   *         - The direction index in tag "axisDirection" exists, but
   *           is not "x", "y", "z", "X", "Y or "Z"
   *         - The check hasDistanceFunction() of TaskDistance fails
   */
  static bool isValid(xmlNode* node, const RcsGraph* graph);

  void computeXp(double* polarVelocity) const {}
  void computeDXp(double* dOmega, const double* phip_des) const {}
  void computeXp_ik(double* omega) const {}
  void computeXpp(double* polarAcceleration, const MatNd* qpp) const {}
  void computeFfXpp(double* xpp_res, const double* xpp_des) const {}
  void computeH(MatNd* hessian) const {}
  void forceTrafo(double* ft_task) const {}
  void selectionTrafo(double* S_trans, const double* S) const {}
  void integrateXp_ik(double* x_res, const double* x,
                      const double* x_dot, double dt) const {}

protected:

  void computePolarNormal(double polarAngs[2]) const;
  int direction;
  double gainDX;
};

}

#endif // RCS_TASKPOLARSURFACENORMAL_H
