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

#ifndef RCS_TASKPOLAR2D_H
#define RCS_TASKPOLAR2D_H

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
 *        reference body, which is the world frame if none is given).
 */
class TaskPolar2D: public Task
{
public:

  /*! Constructor based on xml parsing
   */
  TaskPolar2D(const std::string& className, xmlNode* node,
              RcsGraph* graph, int dim=2);

  /*! \brief Copy constructor doing deep copying with optional new graph
   *         pointer
   */
  TaskPolar2D(const TaskPolar2D& copyFromMe, RcsGraph* newGraph=NULL);

  /*! Destructor
   */
  virtual ~TaskPolar2D();

  /*!
   * \brief Virtual copy constructor with optional new graph.
   */
  virtual TaskPolar2D* clone(RcsGraph* newGraph=NULL) const;

  /*! \brief Computes the Polar Angles between reference body and effector.
   */
  virtual void computeX(double* polarAngles) const;

  /*! \brief Computes the current Polar angular velocity. In this classes
   *         implementation, it is calculated as
   *         \f$
   *         \mathbf{\dot{x} = H \omega }
   *         \f$
   *         where H is computed with Vec3d_getPolarVelocityMatrix().
   *
   *  \param[out] polarVelocity Polar angle velocity
   */
  virtual void computeXp(double* polarVelocity) const;

  virtual void computeXpp(double* polarAcceleration, const MatNd* qpp) const;

  virtual void computeDXp(double* dOmega,
                          const double* phip_des) const;

  /*! \brief This Jacobian relates the x- and y-components of
   *         the angular velocity (with respect to a reference body) to
   *         the state velocity vector. Index 2 denotes the body to be
   *         controlled wrt. index 1. Index 1 is having identity values if
   *         no reference body is given. Index 0 denotes an inertial
   *         reference frame. The 2-fixed angular velocity wrt. body 1 is
   *         \f[
   *         {_2} \omega_{12} = A_{2I} ( {_I}\omega_{02} - {_I}\omega_{01} )
   *         \f]
   *         The differential kinematics give
   *         \f[
   *         {_2} J_{R,12} = A_{2I} ( {_I}J_{R,02} - {_I}J_{R,01} )
   *         \f]
   *         with \f$ J_R \f$ being the rotation Jacobians of the
   *         respective body. To control the axis direction of the 2-fixed
   *         z-axis, we only need the x and y component, so that the
   *         resulting Jacobian contains only the first 2 rows.
   */
  virtual void computeJ(MatNd* jacobian) const;

  /*! \brief This Hessian is the partial derivative of the classes Jacobian
   *         function and is computed as
   *         \f[
   *         {_2} H_{R,12} = \frac{\partial A_{2I}}{\partial q}
   *                         ( {_I}J_{R,02} - {_I}J_{R,01} )
   *                         + A_{2I} ( {_I}H_{R,02} - {_I}H_{R,01} )
   *         \f]
   */
  virtual void computeH(MatNd* hessian) const;

  /*! \brief Computes the angular velocity around the reference bodie's
   *         (or world frames if refBdy is NULL) x- and y-axis so that the
   *         z-axis will be aligned with the desired z-axis a_des.
   */
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

  /*! \brief Performs a set of tests:
   *         - Jacobian finite difference test
   *         - Hessian finite difference test
   *
   *  \param[in] verbose If true, debug information is printed to the console.
   *  \return True for success, false otherwise.
   */
  virtual bool test(bool verbose=false);

  /*! \brief Returns true if the task is specified correctly, false
   *         otherwise. The task is invalid if
   *         - The direction index in tag "axisDirection" exists, but
   *           is not "x", "y", "z", "X", "Y or "Z"
   */
  static bool isValid(xmlNode* node, const RcsGraph* graph);

protected:

  /*! \brief Additionally writes the axis direction into the output stream.
   */
  void toXMLBody(FILE* out) const;

  int direction;
};

}

#endif // RCS_TASKPOLAR2D_H
