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

#ifndef RCS_TASKSPHERICAL3D_H
#define RCS_TASKSPHERICAL3D_H

#include "TaskPosition3D.h"


namespace Rcs
{
/*! \ingroup RcsTask
 * \brief This tasks allows to set a 3D position (spherical coordinates) of
 *        an effector. The position can also be relative to another body and
 *        reference frame.
 */
class TaskSpherical3D : public Rcs::TaskPosition3D
{
public:

  /*! \brief Constructor based on xml parsing
   */
  TaskSpherical3D(const std::string& className, xmlNode* node,
                  RcsGraph* graph, int dim=3);

  /*! \brief Copy constructor doing deep copying with optional new graph
   *         pointer
   */
  TaskSpherical3D(const TaskSpherical3D& src, RcsGraph* newGraph=NULL);

  /*! \brief Destructor
   */
  virtual ~TaskSpherical3D();

  /*!
   * \brief Virtual copy constructor with optional new graph
   */
  virtual TaskSpherical3D* clone(RcsGraph* newGraph=NULL) const;

  /*! \brief Computes the current value of the task variable. The result
   *         is written to parameter \e x_res. Reuses
   *         TaskPosition3D::computeX and then converts to spherical
   *         coordinates
   */
  virtual void computeX(double* x_res) const;

  /*! \brief Computes the current velocity in task space.
   *
   * Reuses TaskPosition3D::computeXp and then converts to spherical
   * coordinates
   */
  virtual void computeXp(double* xp) const;

  /*! \brief Computes current task Jacobian to parameter \e jacobian.
   *         Reuses TaskPosition3D::computeJ and then converts to spherical
   *         coordinates
   */
  virtual void computeJ(MatNd* jacobian) const;

  /*! \brief Computes the delta in task space for the differential
   *         kinematics.
   *
   * Ensures that always the shortest path in phi is followed
   */
  virtual void computeDX(double* dx, const double* x_des, const double* x_curr) const;

  /*! \brief Computes current task Hessian to parameter \e hessian
   *
   * Not implemented!
   */
  virtual void computeH(MatNd* hessian) const;

  /*! \brief Computes the current derivative Jacobian. Reuses
   *         TaskPosition3D::computeJdot and then converts to spherical
   *         coordinates
   */
  virtual void computeJdot(MatNd* Jdot) const;

  /*! \brief Performs a Jacobian finite difference test. The Hessian is not yet
   *         implemented, after that's done, this function can be removed (TODO).
   *
   *  \param[in] verbose If true, debug information is printed to the console.
   *  \return True for success, false otherwise.
   */
  virtual bool test(bool verbose=false);

  /*! \brief Returns true if the task is specified correctly, false
   *         otherwise. The following checks are performed:
   *         - XML tag "effector" corresponds to body in graph
   *         - XML tag "controlVariable" is "SphRTP"
   */
  static bool isValid(xmlNode* node, const RcsGraph* graph);
};

}

#endif // RCS_TASKSPHERICAL3D_H
