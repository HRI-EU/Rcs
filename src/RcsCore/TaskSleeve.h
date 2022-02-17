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

#ifndef RCS_TASKSLEEVE_H
#define RCS_TASKSLEEVE_H

#include "TaskGenericIK.h"


namespace Rcs
{

class TaskSleeve: public TaskGenericIK
{
public:

  /*! Constructor based on xml parsing
   */
  TaskSleeve(const std::string& className, xmlNode* node,
             const RcsGraph* graph, int dim=3);

  /*! \brief Virtual copy constructor with optional new graph.
   */
  virtual TaskSleeve* clone(const RcsGraph* newGraph=NULL) const;

  /*! \brief Computes the Polar Angles between reference body and effector.
   */
  virtual void computeX(double* polarAngles) const;

  virtual void computeDX(double* dx, const double* x_des,
                         const double* x_curr) const;

  /*! \brief This Jacobian
   */
  virtual void computeJ(MatNd* jacobian) const;

  /*! \brief This Hessian
   */
  virtual void computeH(MatNd* hessian) const;

  /*! \brief Returns true if the task is specified correctly
   */
  static bool isValid(xmlNode* node, const RcsGraph* graph);

protected:

  /*! \brief Additionally writes the axis direction into the output stream.
   */
  void toXMLBody(FILE* out) const;

  void computeFrame(double A[3][3]) const;

  void computeContactNormal(double n[3]) const;

  const RcsBody* getShoulder() const;
  const RcsBody* getElbow() const;
  const RcsBody* getWrist() const;
  const RcsBody* getSlider() const;

  int slideBdyId;
};

}

#endif // RCS_TASKSLEEVE_H
