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



static Rcs::TaskFactoryRegistrar<Rcs::TaskStaticEffort> registrar("StaticEffort");



/*******************************************************************************
 *
 ******************************************************************************/
Rcs::TaskStaticEffort::TaskStaticEffort(const std::string& className,
                                        xmlNode* node,
                                        RcsGraph* _graph,
                                        int dim):
  TaskGenericIK(className, node, _graph, dim),
  W(NULL),
  sensor(NULL)
{
  RFATAL("Parsing sensor and W missing, also toXML needs update");
  resetParameter(Parameters(0.0, 1.0, 1.0, "Effort"));
}

/*******************************************************************************
 * Copy constructor doing deep copying
 ******************************************************************************/
Rcs::TaskStaticEffort::TaskStaticEffort(const TaskStaticEffort& copyFromMe,
                                        RcsGraph* newGraph):
  TaskGenericIK(copyFromMe, newGraph),
  sensor(NULL)
{
  if (newGraph != NULL)
  {
    if (copyFromMe.sensor)
    {
      this->sensor = RcsGraph_getSensorByName(newGraph,
                                              copyFromMe.sensor->name);
    }
  }
  else
  {
    this->sensor = copyFromMe.sensor;
  }

  this->W = MatNd_clone(copyFromMe.W);
}

/*******************************************************************************
 * Destructor
 ******************************************************************************/
Rcs::TaskStaticEffort::~TaskStaticEffort()
{
}

/*******************************************************************************
 * Clone function
 ******************************************************************************/
Rcs::TaskStaticEffort* Rcs::TaskStaticEffort::clone(RcsGraph* newGraph) const
{
  return new TaskStaticEffort(*this, newGraph);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskStaticEffort::getForceInWorldCoords(double f[3]) const
{
  Vec3d_copy(f, this->sensor->rawData->ele);
  HTr A_SI;
  HTr_copy(&A_SI, this->ef->A_BI);
  HTr_transformSelf(&A_SI, this->sensor->offset);
  Vec3d_transRotateSelf(f, A_SI.rot);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskStaticEffort::computeX(double* x_res) const
{
  double fbuf[3];
  MatNd f = MatNd_fromPtr(3, 1, fbuf);
  getForceInWorldCoords(f.ele);
  *x_res = RcsGraph_staticEffort(this->graph, this->ef, &f, this->W,
                                 this->sensor->offset->org);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskStaticEffort::computeJ(MatNd* dH) const
{
  MatNd_reshape(dH, this->graph->nJ, 1);
  double fbuf[3];
  MatNd f = MatNd_fromPtr(3, 1, fbuf);
  getForceInWorldCoords(f.ele);
  RcsGraph_staticEffortGradient(this->graph, this->ef, &f, this->W,
                                this->sensor->offset->org, dH);
  MatNd_reshape(dH, 1, this->graph->nJ);
}

/*******************************************************************************
 *  COM Hessian for one direction element. The 3d functions of the parent class
 *  are called, and just the relevant elements are copied. Their index is given
 *  in this->index (see constructor).
 ******************************************************************************/
void Rcs::TaskStaticEffort::computeH(MatNd* hessian) const
{
  RFATAL("Not yetimplemented");
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::TaskStaticEffort::isValid(xmlNode* node, const RcsGraph* graph)
{
  bool success = Rcs::Task::isValid(node, graph, "StaticEffort");

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

  RCSGRAPH_TRAVERSE_SENSORS(graph)
  {
    if (STREQ(SENSOR->name, msg))
    {
      nSensors++;
      foundSensor = SENSOR;
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
