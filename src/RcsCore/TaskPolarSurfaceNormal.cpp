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

#include "TaskPolarSurfaceNormal.h"
#include "TaskFactory.h"
#include "Rcs_typedef.h"
#include "Rcs_macros.h"
#include "Rcs_parser.h"
#include "Rcs_stlParser.h"
#include "Rcs_math.h"
#include "Rcs_kinematics.h"
#include "Rcs_body.h"


namespace Rcs
{

// Register tasks at the task factory. Here we don't use our macro, since it
// would create the same name for each instance.
static TaskFactoryRegistrar<TaskPolarSurfaceNormal> regX("POLAR_SURFACE_X");
static TaskFactoryRegistrar<TaskPolarSurfaceNormal> regY("POLAR_SURFACE_Y");
static TaskFactoryRegistrar<TaskPolarSurfaceNormal> regZ("POLAR_SURFACE_Z");



/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
TaskPolarSurfaceNormal::TaskPolarSurfaceNormal(const std::string& className,
                                                    xmlNode* node,
                                               const RcsGraph* _graph,
                                                    int dim) :
  Task(className, node, _graph, dim), direction(2), gainDX(1.0)
{
  if (getClassName()=="POLAR_SURFACE_X" ||
      getClassName()=="POLAR_SURFACE_Y" ||
      getClassName()=="POLAR_SURFACE_Z")
  {
    resetParameter(Parameters(-M_PI, M_PI, (180.0/M_PI), "Phi [deg]"));
    addParameter(Parameters(-M_PI, M_PI, (180.0/M_PI), "Theta [deg]"));

    if (getClassName()=="POLAR_SURFACE_X")
    {
      this->direction = 0;
    }
    else if (getClassName()=="POLAR_SURFACE_Y")
    {
      this->direction = 1;
    }
    else if (getClassName()=="POLAR_SURFACE_Z")
    {
      this->direction = 2;
    }
  }

  // The gainDX scales the position error coming out of computeDX. It is
  // sometimes needed to scale it down to avoid jitter due to the contact
  // normal updates.
  getXMLNodePropertyDouble(node, "gainDX", &this->gainDX);

  std::vector<std::string> surfBodies = getXMLNodePropertyVecSTLString(node, "surfaceBodies");

  for (size_t i=0; i<surfBodies.size(); ++i)
  {
    const RcsBody* bi = RcsGraph_getBodyByName(getGraph(), surfBodies[i].c_str());
    RCHECK(bi);
    this->surfaceBodies.push_back(bi->id);
  }
}

/*******************************************************************************
 * Clone function required for Polymorphism
 ******************************************************************************/
TaskPolarSurfaceNormal* TaskPolarSurfaceNormal::clone(const RcsGraph* newGraph) const
{
  TaskPolarSurfaceNormal* task = new TaskPolarSurfaceNormal(*this);
  task->setGraph(newGraph);
  return task;
}

/*******************************************************************************
 * First element: angle between current and desired polar axis
 * Second element: 0
 ******************************************************************************/
void TaskPolarSurfaceNormal::computeX(double* x_curr) const
{
  // Compute the normal pointing from the closest surface body to the effector
  double a_des[3];
  RcsBody_distance(closestSurfaceBody(), getEffector(), NULL, NULL, a_des);

  // Get the current polar axis
  const double* a_curr = getEffector()->A_BI.rot[direction];

  x_curr[0] = Vec3d_diffAngle(a_curr, a_des);
  x_curr[1] = 0.0;
}

/*******************************************************************************
 *
 ******************************************************************************/
void TaskPolarSurfaceNormal::computeDX(double* dx_ik,
                                            const double* x_des,
                                            const double* x_curr) const
{
  double phi_des = Math_clip(x_des[0], 0.0, M_PI);
  dx_ik[0] = this->gainDX*(x_curr[0] - phi_des);
  dx_ik[1] = 0.0;
}

/*******************************************************************************
 *
 ******************************************************************************/
void TaskPolarSurfaceNormal::computeJ(MatNd* jacobian) const
{
  // Reshape to correct dimensions
  MatNd_reshape(jacobian, 2, this->graph->nJ);

  // Get the current Polar axis in world coordinates
  const double* a_curr = getEffector()->A_BI.rot[this->direction];

  // Compute the normal pointing from the closest surface body to the effector.
  // This is the desired Polar axis in world coordinates.
  double a_des[3];
  RcsBody_distance(closestSurfaceBody(), getEffector(), NULL, NULL, a_des);

  double rotAxis[3];
  Vec3d_crossProduct(rotAxis, a_curr, a_des);
  double lengthRot = Vec3d_getLength(rotAxis);

  if (lengthRot > 0.0)
  {
    // Normalize rotation axis if not ill-defined
    Vec3d_constMulSelf(rotAxis, 1.0 / lengthRot);
  }
  else
  {
    Vec3d_orthonormalVec(rotAxis, a_curr);
  }

  double rhAxis[3];
  Vec3d_crossProduct(rhAxis, a_curr, rotAxis);

  MatNd* proj = NULL;
  MatNd_create2(proj, 2, 3);
  Vec3d_copy(&proj->ele[0], rotAxis);
  Vec3d_copy(&proj->ele[3], rhAxis);

  // Compute the angular velocity Jacobian
  MatNd* JR2 = NULL;
  MatNd_create2(JR2, 3, this->graph->nJ);
  RcsGraph_rotationJacobian(this->graph, getEffector(), NULL, JR2);
  MatNd_mul(jacobian, proj, JR2);

  MatNd_destroy(proj);
  MatNd_destroy(JR2);
}

/*******************************************************************************
 *
 *******************************************************************************/
void TaskPolarSurfaceNormal::computePolarNormal(double polarAngs[2]) const
{
  double n12[3];
  const RcsBody* b1 = getRefBody();
  const RcsBody* b2 = getEffector();
  RcsBody_distance(b1, b2, NULL, NULL, n12);
  Vec3d_getPolarAngles(polarAngs, n12);
}

/*******************************************************************************
 *
 ******************************************************************************/
const RcsBody* TaskPolarSurfaceNormal::closestSurfaceBody() const
{
  if (surfaceBodies.empty())
  {
    RCHECK(getRefBody());
    return getRefBody();
  }

  const RcsBody* surfBdy = RCSBODY_BY_ID(getGraph(), surfaceBodies[0]);
  const RcsBody* closestBdy = surfBdy;
  double d = RcsBody_distance(surfBdy, getEffector(), NULL, NULL, NULL);

  for (size_t i=1; i<surfaceBodies.size(); ++i)
  {
    surfBdy = RCSBODY_BY_ID(getGraph(), surfaceBodies[i]);
    double d_i = RcsBody_distance(surfBdy, getEffector(), NULL, NULL, NULL);
    if (d_i < d)
    {
      RCHECK(surfBdy);
      closestBdy = surfBdy;
    }
  }

  return closestBdy;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool TaskPolarSurfaceNormal::isValid(xmlNode* node, const RcsGraph* graph)
{
  std::vector<std::string> cVars;
  cVars.push_back("POLAR_SURFACE_X");
  cVars.push_back("POLAR_SURFACE_Y");
  cVars.push_back("POLAR_SURFACE_Z");
  bool success = Task::isValid(node, graph, cVars);

  char taskName[RCS_MAX_NAMELEN] = "Unnamed task";
  getXMLNodePropertyStringN(node, "name", taskName, RCS_MAX_NAMELEN);

  // Check if there is a distance function called between effector and refBdy
  char name1[RCS_MAX_NAMELEN] = "";
  int len = getXMLNodePropertyStringN(node, "effector", name1, RCS_MAX_NAMELEN);

  if (len == 0)
  {
    RLOG(4, "Attribute \"effector\" missing in task \"%s\"", taskName);
    return false;
  }

  const RcsBody* effector = RcsGraph_getBodyByName(graph, name1);

  if (!effector)
  {
    RLOG(4, "Effector \"%s\" not found in graph for task \"%s\"",
         name1, taskName);
    success = false;
  }

  std::vector<std::string> surfBodies = getXMLNodePropertyVecSTLString(node, "surfaceBodies");

  for (size_t i = 0; i < surfBodies.size(); ++i)
  {
    const RcsBody* bi = RcsGraph_getBodyByName(graph, surfBodies[i].c_str());

    if (!bi)
    {
      RLOG(4, "Surface body \"%s\" not found in graph for task \"%s\"",
           surfBodies[i].c_str(), taskName);
    success = false;
  }

    if (RcsBody_getNumDistanceQueries(effector, bi) == 0)
  {
    RLOG(1, "Task \"%s\": No distance function between \"%s\" and \"%s\"!",
           taskName, effector->name, bi->name);
    success = false;
    }

  }

  if (surfBodies.empty())
  {
    char name2[RCS_MAX_NAMELEN] = "";
    getXMLNodePropertyStringN(node, "refBdy", name2, RCS_MAX_NAMELEN);
    getXMLNodePropertyStringN(node, "refBody", name2, RCS_MAX_NAMELEN);

    const RcsBody* refBdy = RcsGraph_getBodyByName(graph, name2);

    if (!refBdy)
    {
      RLOG(4, "Reference body \"%s\" not found in graph for task \"%s\"",
           name2, taskName);
      success = false;
    }

    if (RcsBody_getNumDistanceQueries(effector, refBdy) == 0)
    {
      RLOG(1, "Task \"%s\": No distance function between \"%s\" and \"%s\"!",
           taskName, effector->name, name2);
      success = false;
    }

  }

  return success;
}

}   // namespace Rcs
