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

#include "TaskPositionForce1D.h"
#include "TaskFactory.h"
#include "Rcs_typedef.h"
#include "Rcs_macros.h"
#include "Rcs_parser.h"
#include "Rcs_utils.h"
#include "Rcs_VecNd.h"
#include "Rcs_kinematics.h"
#include "Rcs_sensor.h"


static Rcs::TaskFactoryRegistrar<Rcs::TaskPositionForce1D> registrar1("ForceX");
static Rcs::TaskFactoryRegistrar<Rcs::TaskPositionForce1D> registrar2("ForceY");
static Rcs::TaskFactoryRegistrar<Rcs::TaskPositionForce1D> registrar3("ForceZ");
static Rcs::TaskFactoryRegistrar<Rcs::TaskPositionForce1D> registrar4("ForceCylZ");



/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::TaskPositionForce1D::TaskPositionForce1D(const std::string& className,
                                              xmlNode* node,
                                              RcsGraph* _graph,
                                              int _dim):
  TaskPosition1D(className, node, _graph, _dim),
  ft_curr_temp(0.0), ft_des_temp(0.0), force_feedback(true), fts(NULL)
{
  if (className=="ForceX")
  {
    this->index = 0;
    resetParameter(Parameters(-10.0, 10.0, 1.0, "X Force [N]"));
  }
  else if (className=="ForceY")
  {
    this->index = 1;
    resetParameter(Parameters(-10.0, 10.0, 1.0, "Y Force [N]"));
  }
  else if ((className=="ForceZ") || (className=="ForceCylZ"))
  {
    this->index = 2;
    resetParameter(Parameters(-10.0, 10.0, 1.0, "Z Force [N]"));
  }

  getXMLNodePropertyBoolString(node, "forceFeedback", &this->force_feedback);

  char tmp[64] = "";
  getXMLNodePropertyStringN(node, "sensor", tmp, 64);
  this->fts = RcsGraph_getSensorByName(getGraph(), tmp);
}

/*******************************************************************************
 * For programmatic creation
 ******************************************************************************/
Rcs::TaskPositionForce1D::TaskPositionForce1D(const std::string& className,
                                              RcsGraph* graph,
                                              const RcsBody* effector,
                                              const RcsBody* refBdy,
                                              const RcsBody* refFrame,
                                              const std::string& sensorName,
                                              bool forceFeedback):
  TaskPosition1D(className, graph, effector, refBdy, refFrame),
  ft_curr_temp(0.0), ft_des_temp(0.0), force_feedback(true), fts(NULL)
{

  if (className=="ForceX")
  {
    this->index = 0;
    resetParameter(Parameters(-10.0, 10.0, 1.0, "X Force [N]"));
  }
  else if (className=="ForceY")
  {
    this->index = 1;
    resetParameter(Parameters(-10.0, 10.0, 1.0, "Y Force [N]"));
  }
  else if ((className=="ForceZ") || (className=="ForceCylZ"))
  {
    this->index = 2;
    resetParameter(Parameters(-10.0, 10.0, 1.0, "Z Force [N]"));
  }

  this->force_feedback = forceFeedback;

  this->fts = RcsGraph_getSensorByName(graph, sensorName.c_str());
}

/*******************************************************************************
 * Copy constructor doing deep copying
 ******************************************************************************/
Rcs::TaskPositionForce1D::TaskPositionForce1D(const TaskPositionForce1D& copyFromMe, RcsGraph* newGraph):
  TaskPosition1D(copyFromMe, newGraph),
  ft_curr_temp(copyFromMe.ft_curr_temp),
  ft_des_temp(copyFromMe.ft_des_temp),
  force_feedback(copyFromMe.force_feedback),
  fts(NULL)
{
  if (copyFromMe.fts != NULL)
  {
    this->fts = RcsGraph_getSensorByName(getGraph(), copyFromMe.fts->name);
  }
}

/*******************************************************************************
 * Destructor
 ******************************************************************************/
Rcs::TaskPositionForce1D::~TaskPositionForce1D()
{
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::TaskPositionForce1D* Rcs::TaskPositionForce1D::clone(RcsGraph* newGraph) const
{
  return new Rcs::TaskPositionForce1D(*this, newGraph);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskPositionForce1D::computeX(double* x_res) const
{
  if (this->fts == NULL)
  {
    x_res[0] = this->ft_curr_temp;
  }
  else
  {
    MatNd res = MatNd_fromPtr(getDim(), 1, x_res);
    projectTaskForce(&res, this->fts);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskPositionForce1D::computeAX(double* a_res,
                                         double* integral_x,
                                         const double* x_des,
                                         const double* x_dot_des,
                                         const double* x_ddot_des,
                                         const double* S_des,
                                         const double a_des,
                                         const double kp,
                                         const double kd,
                                         const double ki) const
{
  a_res[0] = 0.0;
  const_cast<TaskPositionForce1D*>(this)->ft_des_temp = x_des[0];
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskPositionForce1D::computeAF(double* ft_res,
                                         double* ft_int,
                                         const double* ft_des,
                                         const double* selection,
                                         const double* ft_task,
                                         const double a_des,
                                         const double kp,
                                         const double ki) const
{
  const_cast<double*>(ft_des)[0] = this->ft_des_temp;
  const_cast<double*>(ft_task)[0] = this->ft_curr_temp;
  const_cast<double*>(selection)[0] = 0.0;

  if (this->force_feedback == true)
  {
    Rcs::TaskPosition1D::computeAF(ft_res, ft_int, ft_des, selection,
                                   ft_task, a_des, kp, ki);
  }
  else
  {
    Rcs::TaskPosition1D::computeAF(ft_res, ft_int, ft_des, selection,
                                   NULL, a_des, kp, ki);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskPositionForce1D::forceTrafo(double* ft_task) const
{
  Rcs::TaskPosition1D::forceTrafo(ft_task);
  const_cast<TaskPositionForce1D*>(this)->ft_curr_temp = ft_task[0];
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::TaskPositionForce1D::isValid(xmlNode* node, const RcsGraph* graph)
{
  std::vector<std::string> classNameVec;
  classNameVec.push_back(std::string("ForceX"));
  classNameVec.push_back(std::string("ForceY"));
  classNameVec.push_back(std::string("ForceZ"));
  classNameVec.push_back(std::string("ForceCylZ"));

  bool success = Rcs::Task::isValid(node, graph, classNameVec);

  // Check is the task has a sensor
  if (getXMLNodeProperty(node, "sensor")==true)
  {
    char tmp[64] = "";
    getXMLNodePropertyStringN(node, "sensor", tmp, 64);
    RcsSensor* fts = RcsGraph_getSensorByName(graph, tmp);

    if (fts == NULL)
    {
      success = false;
      char taskName[RCS_MAX_NAMELEN] = "";
      getXMLNodePropertyStringN(node, "name", taskName, RCS_MAX_NAMELEN);
      RLOG(3, "Task \"%s\" has no sensor \"%s\"", taskName, tmp);
    }
    else if (fts->type != RCSSENSOR_LOAD_CELL)
    {
      success = false;
      char taskName[RCS_MAX_NAMELEN] = "";
      getXMLNodePropertyStringN(node, "name", taskName, RCS_MAX_NAMELEN);
      RLOG(3, "Task \"%s\": Sensor \"%s\" is not of type RCSSENSOR_LOAD_CELL",
           taskName, fts->name);
    }
  }

  return success;
}
