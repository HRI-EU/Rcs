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

#include "TaskPose6D.h"
#include "TaskPosition3D.h"
#include "TaskEuler3D.h"
#include "TaskFactory.h"
#include "Rcs_typedef.h"
#include "Rcs_parser.h"
#include "Rcs_VecNd.h"

#include <cmath>


static Rcs::TaskFactoryRegistrar<Rcs::TaskPose6D> registrar("XYZABC");


/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::TaskPose6D::TaskPose6D(const std::string& className_,
                            xmlNode* node,
                            const RcsGraph* _graph):
  CompositeTask(className_, node, _graph)
{
  addTask(new TaskPosition3D("XYZ", node, _graph, 3));

  Rcs::Task* abc = new TaskEuler3D("ABC", node, _graph, 3);

  abc->setRefFrameId(abc->getRefBodyId());

  addTask(abc);

  if (getClassName() == "XYZABC")
  {
    double guiMax[6], guiMin[6];
    VecNd_set6(guiMin, -2.5, -2.5, -2.5, -M_PI, -M_PI, -M_PI);
    VecNd_set6(guiMax, 2.5, 2.5, 2.5, M_PI, M_PI, M_PI);
    getXMLNodePropertyVecN(node, "guiMax", guiMax, 6);
    getXMLNodePropertyVecN(node, "guiMin", guiMin, 6);

    bool hide = false;
    getXMLNodePropertyBoolString(node, "hide", &hide);
    if (hide)
    {
      VecNd_setZero(guiMin, 6);
      VecNd_setZero(guiMax, 6);
    }
    resetParameter(Parameters(guiMin[0], guiMax[0], 1.0, "X [m]"));
    addParameter(Parameters(guiMin[1], guiMax[1], 1.0, "Y [m]"));
    addParameter(Parameters(guiMin[2], guiMax[2], 1.0, "Z [m]"));
    addParameter(Parameters(guiMin[3], guiMax[3], (180.0 / M_PI), "A [deg]"));
    addParameter(Parameters(guiMin[4], guiMax[4], (180.0 / M_PI), "B [deg]"));
    addParameter(Parameters(guiMin[5], guiMax[5], (180.0 / M_PI), "C [deg]"));
  }

}

/*******************************************************************************
 * Constructor based on body pointers
 ******************************************************************************/
Rcs::TaskPose6D::TaskPose6D(const RcsGraph* graph_,
                            const RcsBody* effector,
                            const RcsBody* refBdy,
                            const RcsBody* refFrame): CompositeTask()
{
  this->graph = graph_;
  setClassName("XYZABC");

  std::string taskName = std::string("Pose6D");

  if (effector != NULL)
  {
    taskName += std::string(" ");
    taskName += std::string(effector->name);
  }

  if (refBdy != NULL)
  {
    taskName += std::string("-");
    taskName += std::string(refBdy->name);
  }

  setName(taskName);
  addTask(new TaskPosition3D(graph_, effector, refBdy, refFrame));
  addTask(new TaskEuler3D(graph_, effector, refBdy, refFrame));
  setEffectorId(effector ? effector->id : -1);
  setRefBodyId(refBdy ? refBdy->id : -1);
  setRefFrameId(refFrame ? refFrame->id : getRefBodyId());
}

/*******************************************************************************
 * Clone function
 ******************************************************************************/
Rcs::TaskPose6D* Rcs::TaskPose6D::clone(const RcsGraph* newGraph) const
{
  TaskPose6D* task = new Rcs::TaskPose6D(*this);
  task->setGraph(newGraph);
  return task;
}

/*******************************************************************************
 *
 ******************************************************************************/
const RcsBody* Rcs::TaskPose6D::getEffector() const
{
  return getSubTask(0)->getEffector();
}

/*******************************************************************************
 *
 ******************************************************************************/
const RcsBody* Rcs::TaskPose6D::getRefBody() const
{
  return getSubTask(0)->getRefBody();
}

/*******************************************************************************
 *
 ******************************************************************************/
const RcsBody* Rcs::TaskPose6D::getRefFrame() const
{
  return getSubTask(0)->getRefFrame();
}

/*******************************************************************************
 * See header.  \todo Check isValid() of subtasks
 ******************************************************************************/
bool Rcs::TaskPose6D::isValid(xmlNode* node, const RcsGraph* graph)
{
  bool success = Rcs::Task::isValid(node, graph, "XYZABC");

  return success;
}
