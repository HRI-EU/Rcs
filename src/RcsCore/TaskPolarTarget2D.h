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

#ifndef RCS_TASKPOLARTARGET2D_H
#define RCS_TASKPOLARTARGET2D_H

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
class TaskPolarTarget2D: public Task
{
public:

  /*! Constructor based on xml parsing
   */
  TaskPolarTarget2D(const std::string& className, xmlNode* node,
                    const RcsGraph* graph, int dim=2);

  /*! \brief Virtual copy constructor with optional new graph.
   */
  virtual TaskPolarTarget2D* clone(const RcsGraph* newGraph=NULL) const;

  /*! \brief Computes the Polar Angles between reference body and effector.
   */
  virtual void computeX(double* polarAngles) const;

  /*! \brief TODO
   */
  virtual void computeXp(double* polarVelocity) const;

  /*! \brief TODO
   */
  virtual void computeXp_ik(double* omega) const;

  virtual void forceTrafo(double* ft_task) const;

  virtual void selectionTrafo(double* S_des_trafo, const double* S_des) const;

  /*! \brief TODO
   */
  virtual void integrateXp_ik(double* x_res, const double* x,
                              const double* x_dot, double dt) const;

  /*! \brief TODO
   */
  virtual void computeXpp(double* polarAcceleration, const MatNd* qpp) const;

  /*! \brief TODO
   */
  virtual void computeJ(MatNd* jacobian) const;

  /*! \brief TODO
   */
  virtual void computeH(MatNd* hessian) const;

  /*! \brief TODO
   */
  virtual void computeFfXpp(double* xpp_res, const double* xpp_des) const;

  /*! \brief TODO
   */
  virtual void computeDXp(double* dOmega,
                          const double* phip_des) const;

  /*! \brief Calculates the angular error as the angle between the
   *         current and desired Polar axes (omega[0]). The second
   *         component (omega[1]) is set to zero.
   *
   *  \param[out] omega 2-dimensional angular error as described above
   *  \param[in] polar_des desired polar angles (represented in the
   *                       coordinates of the task's refBdy)
   *  \param[in] polar_curr current polar angles (represented in the
   *                        coordinates of the task's refBdy)
   */
  virtual void computeDX(double* omega, const double* polar_des,
                         const double* polar_curr) const;

  /*! \brief Returns true if the task is specified correctly, false
   *         otherwise. The task is invalid if
   *         - The direction index in tag "axisDirection" exists, but
   *           is not "x", "y", "z", "X", "Y or "Z"
   */
  static bool isValid(xmlNode* node, const RcsGraph* graph);

  /*! \brief Assigns the given Polar angles to the class member polarDes.
   */
  void setTarget(const double polarTarget[2]);

  /*! \brief Assigns the given Polar angles to the class member polarDes.
   */
  void setTarget(double phi, double theta);


protected:

  /*! \brief Constructs a rotation matrix A_SR with the z-axis being the
   *         current Polar axis. It is determined based on the tasks's bodies
   *         and the state of the graph. The y-axis is the rotation axis that
   *         is perpendicular to the plane spanned by current and desired
   *         Polar axis. Rotating about it will move the current to the
   *         axis on the shortest possible path. The x-axis completes a
   *         right hand frame. All axes are normalized. In case the angle
   *         between current and desired axis is zero, there are infinity
   *         solutions to determine the y-axis. We then just pick any of
   *         these.
   */
  void computeSlerpFrame(double A_SR[3][3],
                         const double polarDes[2]) const;

  /*! \brief Calls \ref computeSlerpFrame(double[3][3], const double[2]) const
  *          with the polarDes member variable.
   */
  void computeSlerpFrame(double A_SR[3][3]) const;

  int direction;
  double polarDes[2];
};

}

#endif // RCS_TASKPOLARTARGET2D_H
