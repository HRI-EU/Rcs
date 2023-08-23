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

#ifndef RCS_TASKINCLINATION_H
#define RCS_TASKINCLINATION_H

#include "TaskGenericIK.h"



namespace Rcs
{

/*! \ingroup RcsTask
 * \brief This tasks allows to set a 1D inclination of an effector. The
 *        inclination can also be relative to another body (refBody). The
 *        inclination axis is taken from the effector body. By default, it is
 *        the z-axis. It can be specified with the xml tag
 *        - axisDirection="X"
 *        - axisDirection="Y"
 *        - axisDirection="Z"
 *        to select which axis direction from the effector body is used. The
 *        reference axis can be specified by the xml tag
 *        - refAxisDirection="X"
 *        - refAxisDirection="Y"
 *        - refAxisDirection="Z"
 *        to select which axis direction from the reference body is used.
 *
 *        It is possible to specify several effectors in form of a space-
 *        separated list in the xml-tag "effector". In this case, the resulting
 *        inclination is averaged between them.
 *
 *        This representation is not particularly ill-defined, but a bit
 *        critical at the poles of the inclination (0 and 180 degrees): At
 *        these values, the rotation axis to align current and desired might
 *        change quite rapidly if there are only minor changes in the kinematic
 *        configuration. Therefore it is required to somehow avoid the pole
 *        configurations. This is particularly the case if a null space
 *        movement is added. There are no means implemented in this class.
 */
class TaskInclination: public TaskGenericIK
{
public:

  /*! Constructor based on xml parsing
   */
  TaskInclination(const std::string& className, xmlNode* node,
                  const RcsGraph* graph, int dim=1);

  /*!
   * \brief Virtual copy constructor with optional new graph.
   */
  virtual TaskInclination* clone(const RcsGraph* newGraph=NULL) const;

  /*! \brief Computes the inclination angle between reference body and effector.
   */
  virtual void computeX(double* inclination) const;

  /*! \brief Computes the displacement in task space. The value of x_des is
   *         limited to [0 ... 180] degrees.
   */
  virtual void computeDX(double* dx, const double* x_des,
                         const double* x_curr) const;

  /*! \brief Rotation Jacobian about inclination rotation axis.
   */
  virtual void computeJ(MatNd* jacobian) const;

  /*! \brief Rotation Hessian about inclination rotation axis.
   */
  virtual void computeH(MatNd* hessian) const;

  /*! \brief Rotation axis between refBodie's z-axis and effector's
   *         task axis (see member direction).
   *
   *  \param[out] a_rot Normalized rotation axis
   *  \param[in]  num   Number of effector
   *  \return Length of the rotation axis (before normalization)
   */
  virtual double computeRotationAxis(double a_rot[3], size_t num) const;

  /*! \brief Sets all effectorVec ids to bodies with the given suffix appended.
   */
  virtual bool setIdsToSuffix(const std::string& suffix);

  /*! \brief Returns true if the task is specified correctly, false
   *         otherwise. The task is invalid if
   *         - The direction index in tag "axisDirection" exists, but
   *           is not "x", "y", "z", "X", "Y or "Z"
   *         - The direction index in tag "refAxisDirection" exists, but
   *           is not "x", "y", "z", "X", "Y or "Z"
   */
  static bool isValid(xmlNode* node, const RcsGraph* graph);


private:
  void toXMLBody(FILE* out) const;
  const double* aRef() const;
  const double* aEf(size_t num) const;
  const RcsBody* getEffectorVec(size_t num) const;

  int direction;
  int refDirection;
  std::vector<int> effectorVec;
};

}

#endif // RCS_TASKINCLINATION_H
