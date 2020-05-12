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
                            RcsGraph* _graph):
  CompositeTask(className_, node, _graph)
{
  addTask(new TaskPosition3D("XYZ", node, _graph, 3));

  Rcs::Task* abc = new TaskEuler3D("ABC", node, _graph, 3);

  abc->setRefFrame(abc->getRefBody());

  addTask(abc);

  if (getClassName() == "XYZABC")
  {
    double guiMax[6], guiMin[6];
    VecNd_set6(guiMin, -2.5, -2.5, -2.5, -M_PI, -M_PI, -M_PI);
    VecNd_set6(guiMax, 2.5, 2.5, 2.5, M_PI, M_PI, M_PI);
    getXMLNodePropertyVecN(node, "guiMax", guiMax, 6);
    getXMLNodePropertyVecN(node, "guiMin", guiMin, 6);

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
Rcs::TaskPose6D::TaskPose6D(RcsGraph* graph_,
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
  setEffector(effector);
  setRefBody(refBdy);
  setRefFrame(refFrame ? refFrame : refBdy);
}

/*******************************************************************************
 * Copy constructor doing deep copying
 ******************************************************************************/
Rcs::TaskPose6D::TaskPose6D(const Rcs::TaskPose6D& copyFromMe,
                            RcsGraph* newGraph):
  CompositeTask(copyFromMe, newGraph)
{
}

/*******************************************************************************
 * Clone function
 ******************************************************************************/
Rcs::TaskPose6D* Rcs::TaskPose6D::clone(RcsGraph* newGraph) const
{
  return new Rcs::TaskPose6D(*this, newGraph);
}

/*******************************************************************************
 * See header.  \todo Check isValid() of subtasks
 ******************************************************************************/
bool Rcs::TaskPose6D::isValid(xmlNode* node, const RcsGraph* graph)
{
  bool success = Rcs::Task::isValid(node, graph, "XYZABC");

  return success;
}
