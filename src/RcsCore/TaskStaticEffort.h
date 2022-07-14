/*******************************************************************************

  Copyright (c) Honda Research Institute Europe GmbH

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

#ifndef RCS_TASKSTATICEFFORT_H
#define RCS_TASKSTATICEFFORT_H

#include "TaskGenericIK.h"


namespace Rcs
{

/*! \ingroup RcsTask
 * \brief Static effort minimization task.
 */
class TaskStaticEffort: public TaskGenericIK
{
public:

  /*! Constructor based on xml parsing
   */
  TaskStaticEffort(const std::string& className, xmlNode* node,
                   const RcsGraph* graph, int dim=1);

  /*! Copy constructor
   */
  TaskStaticEffort(const TaskStaticEffort& copyFromMe);

  /*! \brief Virtual copy constructor with optional new graph
   */
  virtual TaskStaticEffort* clone(const RcsGraph* newGraph=NULL) const;

  /*! \brief Computes the current value of the task variable
   *
   *  \param[out] x_res The result of the calculation
   */
  virtual void computeX(double* x_res) const;

  /*! \brief Computes current task Jacobian to parameter \param jacobian
   */
  virtual void computeJ(MatNd* jacobian) const;

  /*! \brief Computes current task Hessian to parameter \param hessian
   */
  virtual void computeH(MatNd* hessian) const;

  /*! \brief Computes the sensor force in world coordinates.
   */
  void getForceInWorldCoords(double f[3]) const;

  void toXMLBody(FILE* out) const;

  /*! \brief Returns true for success, false for failure:
     *         - Xml tag "effector" doesn't exist
     *         - Body with name in tag "effector" not in graph
     *         - Xml tag "sensor" doesn't exist
     *         - Sensor with name in tag "sensor" not in graph
     *         - More or less than 1 sensor are found
     *         - Sensor is not of type "RCSSENSOR_LOAD_CELL"
     */
  static bool isValid(xmlNode* xml_node, const RcsGraph* graph);

protected:

  const RcsSensor* getSensor() const;
  MatNd* getJointWeights() const;

  std::vector<double> jointWeights;
  int sensorId;
};

}

#endif // RCS_TASKSTATICEFFORT_H
