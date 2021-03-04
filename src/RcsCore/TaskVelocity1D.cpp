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

#include "TaskVelocity1D.h"
#include "TaskFactory.h"
#include "Rcs_typedef.h"
#include "Rcs_macros.h"
#include "Rcs_parser.h"
#include "Rcs_utils.h"
#include "Rcs_VecNd.h"
#include "Rcs_kinematics.h"



static Rcs::TaskFactoryRegistrar<Rcs::TaskVelocity1D> registrar1("Xd");
static Rcs::TaskFactoryRegistrar<Rcs::TaskVelocity1D> registrar2("Yd");
static Rcs::TaskFactoryRegistrar<Rcs::TaskVelocity1D> registrar3("Zd");



/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::TaskVelocity1D::TaskVelocity1D(const std::string& taskType,
                                    xmlNode* node,
                                    RcsGraph* _graph,
                                    int dim):
  TaskPosition1D(taskType, node, _graph, dim)
{
  double guiMax = 1.0, guiMin = -1.0;

  if (getDim() == 1)
  {
    getXMLNodePropertyDouble(node, "guiMax", &guiMax);
    getXMLNodePropertyDouble(node, "guiMin", &guiMin);
  }

  if (taskType=="Xd")
  {
    resetParameter(Parameters(guiMin, guiMax, 1.0, "X Velocity [m/s]"));
    this->index = 0;
  }
  else if (taskType=="Yd")
  {
    resetParameter(Parameters(guiMin, guiMax, 1.0, "Y Velocity [m/s]"));
    this->index = 1;
  }
  else if (taskType=="Zd")
  {
    resetParameter(Parameters(guiMin, guiMax, 1.0, "Z Velocity [m/s]"));
    this->index = 2;
  }
}

/*******************************************************************************
 * For programmatic creation
 ******************************************************************************/
Rcs::TaskVelocity1D::TaskVelocity1D(const std::string& className,
                                    RcsGraph* graph,
                                    const RcsBody* effector,
                                    const RcsBody* refBdy,
                                    const RcsBody* refFrame):
  TaskPosition1D(className, graph, effector, refBdy, refFrame)
{
  if (getClassName()=="Xd")
  {
    this->index = 0;
    resetParameter(Task::Parameters(-1.0, 1.0, 1.0, "X Velocity [m/s]"));
  }
  else if (getClassName()=="Yd")
  {
    this->index = 1;
    resetParameter(Task::Parameters(-1.0, 1.0, 1.0, "Y Velocity [m/s]"));
  }
  else if (getClassName()=="Zd")
  {
    this->index = 2;
    resetParameter(Task::Parameters(-1.0, 1.0, 1.0, "Z Velocity [m/s]"));
  }
}

/*******************************************************************************
 * Copy constructor doing deep copying
 ******************************************************************************/
Rcs::TaskVelocity1D::TaskVelocity1D(const TaskVelocity1D& copyFromMe,
                                    RcsGraph* newGraph):
  TaskPosition1D(copyFromMe, newGraph)
{
}

/*******************************************************************************
 * Destructor
 ******************************************************************************/
Rcs::TaskVelocity1D::~TaskVelocity1D()
{
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::TaskVelocity1D* Rcs::TaskVelocity1D::clone(RcsGraph* newGraph) const
{
  return new Rcs::TaskVelocity1D(*this, newGraph);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskVelocity1D::computeX(double* x_res) const
{
  computeXp(x_res);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskVelocity1D::computeDX(double* dx, const double* x_des) const
{
  *dx = *x_des;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::TaskVelocity1D::isValid(xmlNode* node, const RcsGraph* graph)
{
  std::vector<std::string> classNameVec;
  classNameVec.push_back(std::string("Xd"));
  classNameVec.push_back(std::string("Yd"));
  classNameVec.push_back(std::string("Zd"));

  bool success = Rcs::Task::isValid(node, graph, classNameVec);

  return success;
}

/*******************************************************************************
 * No good test yet.
 ******************************************************************************/
bool Rcs::TaskVelocity1D::testJacobian(double errorLimit,
                                       double delta,
                                       bool relativeError,
                                       bool verbose)
{
  return true;
}
