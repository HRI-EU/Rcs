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

#include "TaskStaticEffort.h"
#include "TaskFactory.h"
#include "Rcs_typedef.h"
#include "Rcs_sensor.h"
#include "Rcs_macros.h"
#include "Rcs_parser.h"
#include "Rcs_utils.h"
#include "Rcs_Vec3d.h"
#include "Rcs_HTr.h"
#include "Rcs_kinematics.h"
#include "Rcs_stlParser.h"



namespace Rcs
{

static TaskFactoryRegistrar<TaskStaticEffort> registrar("StaticEffort");


/*******************************************************************************
 * \todo: Joint weights parsing
 ******************************************************************************/
TaskStaticEffort::TaskStaticEffort(const std::string& className,
                                   xmlNode* node,
                                   const RcsGraph* _graph,
                                   int dim):
  TaskGenericIK(className, node, _graph, dim), sensorId(-1)
{
  std::string sensorName = getXMLNodePropertySTLString(node, "sensor");

  for (unsigned int i=0; i<getGraph()->nSensors; ++i)
  {
    if (std::string(graph->sensors[i].name) == sensorName)
    {
      sensorId = i;
      break;
    }
  }

  jointWeights = std::vector<double>(getGraph()->dof, 1.0);

  resetParameter(Parameters(0.0, 1.0, 1.0, "Effort"));
}

/*******************************************************************************
 *
 ******************************************************************************/
TaskStaticEffort::TaskStaticEffort(const TaskStaticEffort& copyFromMe):
  jointWeights(copyFromMe.jointWeights), sensorId(copyFromMe.sensorId)
{
}

/*******************************************************************************
 * Clone function
 ******************************************************************************/
TaskStaticEffort* TaskStaticEffort::clone(const RcsGraph* newGraph) const
{
  TaskStaticEffort* task = new TaskStaticEffort(*this);
  task->setGraph(newGraph);
  return task;
}

/*******************************************************************************
 *
 ******************************************************************************/
void TaskStaticEffort::getForceInWorldCoords(double f[3]) const
{
  Vec3d_copy(f, getSensor()->rawData->ele);
  HTr A_SI;
  HTr_copy(&A_SI, &getEffector()->A_BI);
  HTr_transformSelf(&A_SI, &getSensor()->A_SB);
  Vec3d_transRotateSelf(f, A_SI.rot);
}

/*******************************************************************************
 *
 ******************************************************************************/
void TaskStaticEffort::computeX(double* x_res) const
{
  double fbuf[3];
  MatNd f = MatNd_fromPtr(3, 1, fbuf);
  getForceInWorldCoords(f.ele);
  MatNd* wj = getJointWeights();

  *x_res = RcsGraph_staticEffort(this->graph, getEffector(), &f, wj,
                                 getSensor()->A_SB.org);

  MatNd_destroy(wj);
}

/*******************************************************************************
 *
 ******************************************************************************/
MatNd* TaskStaticEffort::getJointWeights() const
{
  MatNd* wj = NULL;

  if (!jointWeights.empty())
  {
    RCHECK(graph->nJ == jointWeights.size());
    wj = MatNd_create(graph->nJ, 1);
    for (size_t i = 0; i < jointWeights.size(); ++i)
    {
      wj->ele[i] = jointWeights[i];
    }
  }

  return wj;
}

/*******************************************************************************
 *
 ******************************************************************************/
void TaskStaticEffort::computeJ(MatNd* dH) const
{
  MatNd_reshape(dH, this->graph->nJ, 1);
  double fbuf[3];
  MatNd f = MatNd_fromPtr(3, 1, fbuf);
  getForceInWorldCoords(f.ele);
  MatNd* wj = getJointWeights();

  RcsGraph_staticEffortGradient(this->graph, getEffector(), &f, wj,
                                getSensor()->A_SB.org, dH);
  MatNd_reshape(dH, 1, this->graph->nJ);
  MatNd_destroy(wj);
}

/*******************************************************************************
 *  COM Hessian for one direction element. The 3d functions of the parent class
 *  are called, and just the relevant elements are copied. Their index is given
 *  in this->index (see constructor).
 ******************************************************************************/
void TaskStaticEffort::computeH(MatNd* hessian) const
{
  RFATAL("Not yet implemented");
}

/*******************************************************************************
 *
 ******************************************************************************/
const RcsSensor* TaskStaticEffort::getSensor() const
{
  RCHECK((sensorId>=0) && (sensorId<(int)graph->nSensors));
  return &graph->sensors[sensorId];
}

/*******************************************************************************
 *
 ******************************************************************************/
void TaskStaticEffort::toXMLBody(FILE* out) const
{
  fprintf(out, " sensor=\"%d\"", sensorId);
}

/*******************************************************************************
 *
 ******************************************************************************/
bool TaskStaticEffort::setIdsToSuffix(const std::string& suffix)
{
  std::string newName = std::string(getSensor()->name) + suffix;
  int sid = RcsGraph_getSensorIdByName(graph, newName.c_str());

  if (sid == -1)
  {
    RLOG(1, "Sensor \"%s\" not found - setIdsToSuffix() failed",
         newName.c_str());
    return false;
  }

  sensorId = sid;

  return true;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool TaskStaticEffort::isValid(xmlNode* node, const RcsGraph* graph)
{
  bool success = Task::isValid(node, graph, "StaticEffort");

  if (success == false)
  {
    return false;
  }

  char taskName[256];
  strcpy(taskName, "unnamed task");
  getXMLNodePropertyStringN(node, "name", taskName, 256);

  char msg[256] = "";
  int len = getXMLNodePropertyStringN(node, "effector", msg, 256);

  if (len == 0)
  {
    RLOG(3, "Task \"%s\": \"effector\" missing", taskName);
    return false;
  }

  RcsBody* ef = RcsGraph_getBodyByName(graph, msg);
  if (ef == NULL)
  {
    RLOG(3, "Task \"%s\": body \"%s\" doesn't exist!", taskName, msg);
    return false;
  }

  // sensor
  len = getXMLNodePropertyStringN(node, "sensor", msg, 256);

  if (len == 0)
  {
    RLOG(3, "Task \"%s\": tag \"sensor\" missing", taskName);
    return false;
  }

  int nSensors = 0;
  RcsSensor* foundSensor = NULL;

  for (unsigned int i=0; i<graph->nSensors; ++i)
  {
    if (STREQ(graph->sensors[i].name, msg))
    {
      nSensors++;
      foundSensor = &graph->sensors[i];
      break;
    }
  }

  if (nSensors != 1)
  {
    RLOG(3, "Task \"%s\": %d sensors found", taskName, nSensors);
    return false;
  }

  if (foundSensor->type != RCSSENSOR_LOAD_CELL)
  {
    RLOG(3, "Task \"%s\": Sensors with name \"%s\" is not of type "
         "RCSSENSOR_LOAD_CELL, but of %d",
         taskName, msg, foundSensor->type);
    return false;
  }



  return true;
}



}   // namespace Rcs
