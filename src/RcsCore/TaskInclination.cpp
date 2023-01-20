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

#include "TaskInclination.h"
#include "TaskFactory.h"

#include <Rcs_typedef.h>
#include <Rcs_macros.h>
#include <Rcs_parser.h>
#include <Rcs_stlParser.h>
#include <Rcs_Vec3d.h>
#include <Rcs_basicMath.h>
#include <Rcs_kinematics.h>


// register task at the task factory
static Rcs::TaskFactoryRegistrar<Rcs::TaskInclination> registrar("Inclination");



/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::TaskInclination::TaskInclination(const std::string& className,
                                      xmlNode* node,
                                      const RcsGraph* _graph,
                                      int dim) :
  TaskGenericIK(className, node, _graph, dim),
  direction(2),
  refDirection(2)
{
  if (getClassName()=="Inclination")
  {
    resetParameter(Parameters(0.0, M_PI, 180.0/M_PI, "Inclination [deg]"));
  }

  // Parse axis direction (should be X, Y or Z)
  char text[16] = "Z";
  getXMLNodePropertyStringN(node, "axisDirection", text, 16);

  if (STRCASEEQ(text, "X"))
  {
    this->direction = 0;
  }
  else if (STRCASEEQ(text, "Y"))
  {
    this->direction = 1;
  }
  else if (STRCASEEQ(text, "Z"))
  {
    this->direction = 2;
  }

  // Parse reference axis direction (should be X, Y or Z)
  char textRef[16] = "Z";
  getXMLNodePropertyStringN(node, "refAxisDirection", textRef, 16);

  if (STRCASEEQ(textRef, "X"))
  {
    this->refDirection = 0;
  }
  else if (STRCASEEQ(textRef, "Y"))
  {
    this->refDirection = 1;
  }
  else if (STRCASEEQ(textRef, "Z"))
  {
    this->refDirection = 2;
  }

  // Generate vector of effector links
  std::vector<std::string> vec =
    getXMLNodePropertyVecSTLString(node, "effector");

  for (size_t i=0; i<vec.size(); ++i)
  {
    if (STRNEQ(vec[i].c_str(), "GenericBody", 11))
    {
      this->effectorVec.push_back(getGenericBodyId(vec[i].c_str()));
    }
    else
    {
      const RcsBody* ef_i = RcsGraph_getBodyByName(graph, vec[i].c_str());
      RCHECK(ef_i);
      this->effectorVec.push_back(ef_i->id);
    }
  }

  REXEC(5)
  {
    RMSG("Found %zu effectors", effectorVec.size());

    for (size_t i=0; i<effectorVec.size(); ++i)
    {
      RMSG("Effector %zu is \"%s\"", i, getEffectorVec(i)->name);
    }
  }

}

/*******************************************************************************
 * Clone function
 ******************************************************************************/
Rcs::TaskInclination* Rcs::TaskInclination::clone(const RcsGraph* newGraph) const
{
  TaskInclination* task = new Rcs::TaskInclination(*this);
  task->setGraph(newGraph);
  return task;
}

/*******************************************************************************
 * Computes the inclination bewteen effector axis and refBodie's z-axis:
 * inclination = acos((v1 o v2)/(|v1|*|v2|)) with "o" being the inner product
 ******************************************************************************/
void Rcs::TaskInclination::computeX(double* inclination) const
{
  double tmp = 0.0;

  for (size_t i=0; i<effectorVec.size(); ++i)
  {
    tmp += Vec3d_diffAngle(aRef(), aEf(i));
  }


  *inclination = tmp/effectorVec.size();
}

/*******************************************************************************
 * Computes the delta in task space for the differential kinematics
 ******************************************************************************/
void Rcs::TaskInclination::computeDX(double* dx,
                                     const double* x_des,
                                     const double* x_curr) const
{
  double x_des_clip = Math_clip(*x_des, 0.0, M_PI);
  *dx = x_des_clip - (*x_curr);
}

/*******************************************************************************
 * Jacobian around rotation axis: J = I_a_rot^T(q) * I_JR(q)
 ******************************************************************************/
void Rcs::TaskInclination::computeJ(MatNd* jacobian) const
{
  MatNd* JR = NULL, *a_rot = NULL;
  MatNd_create2(JR, 3, this->graph->nJ);
  MatNd_fromStack(a_rot, 1, 3);

  MatNd_reshapeAndSetZero(jacobian, 1, this->graph->nJ);

  for (size_t i=0; i<effectorVec.size(); ++i)
  {
    RcsGraph_3dOmegaJacobian(this->graph, getEffectorVec(i),
                             getRefBody(), NULL, JR);

    computeRotationAxis(a_rot->ele, i);
    MatNd_mulAndAddSelf(jacobian, a_rot, JR);
  }

  MatNd_constMulSelf(jacobian, 1.0/effectorVec.size());

  MatNd_destroy(JR);
}

/*******************************************************************************
 * Hessian around rotation axis. Doesn't yet work.
 *
 * The Jacobian is           J = a_rot^T(q) * JR(q)
 * The Hessian is therefore  H = d/dq(a_rot^T(q)) * JR(q) + a_rot^T(q) * HR(q)
 *
 * where a_rot(q) = normalize(a_ref(q) x a_ef(q))
 * for no refFrame, a_rot = (0 0 1)^T x a_ef) = (-y x 0)^T
 * and therefore d/dq(a_rot^T(q)) = (-JR_y JR_x 0)
 *
 ******************************************************************************/
void Rcs::TaskInclination::computeH(MatNd* hessian) const
{
  MatNd* HR = NULL, *JR_a = NULL, *JR = NULL, *a_rot = NULL;
  MatNd_create2(JR_a, 3, this->graph->nJ);
  MatNd_create2(JR, 3, this->graph->nJ);
  MatNd_create2(HR, 3, this->graph->nJ*this->graph->nJ);
  MatNd_fromStack(a_rot, 1, 3);

  // Reset result
  MatNd_reshapeAndSetZero(hessian, this->graph->nJ, this->graph->nJ);

  // Go through all effectors
  for (size_t i=0; i<effectorVec.size(); ++i)
  {
    // I_a_rot^T(q) * I_HR
    double lengthRot = computeRotationAxis(a_rot->ele, i);

    RcsGraph_3dOmegaHessian(this->graph, getEffectorVec(i),
                            getRefBody(), NULL, HR);
    MatNd_reshape(HR, 3, this->graph->nJ*this->graph->nJ);
    MatNd_reshape(hessian, 1, this->graph->nJ*this->graph->nJ);
    MatNd_mulAndAddSelf(hessian, a_rot, HR);

    // d/dq(a_rot^T(q)) * JR
    RcsGraph_3dOmegaJacobian(this->graph, getEffectorVec(i),
                             getRefBody(), NULL, JR);

    MatNd_copyRow(JR_a, 0, JR, 1);
    MatNd_constMulSelf(JR_a, -1.0);
    MatNd_copyRow(JR_a, 1, JR, 0);
    MatNd_transposeSelf(JR_a);
    MatNd_constMulSelf(JR_a, 1.0/lengthRot);

    MatNd_reshape(HR, this->graph->nJ, this->graph->nJ);
    MatNd_mul(HR, JR_a, JR);
    MatNd_reshape(hessian, this->graph->nJ, this->graph->nJ);
    MatNd_addSelf(hessian, HR);
  }


  MatNd_constMulSelf(hessian, 1.0/effectorVec.size());

  // Clean up
  MatNd_destroy(JR_a);
  MatNd_destroy(JR);
  MatNd_destroy(HR);
}

/*******************************************************************************
 * a_rot = a_ref x a_ef
 ******************************************************************************/
double Rcs::TaskInclination::computeRotationAxis(double a_rot[3], size_t num) const
{
  Vec3d_crossProduct(a_rot, aRef(), aEf(num));

  // We need to make sure that the current and desired axis are not
  // parallel. This can be determined by checking the length of the
  // rotation axis A_SR[1].
  double lengthRot = Vec3d_getLength(a_rot);

  // If the length is 0, there are infinity rotation axes. We take just one
  // that is perpendicular to the current inclination axis according to the
  // function Vec3d_orthonormalVec().
  if (lengthRot==0.0)
  {
    Vec3d_orthonormalVec(a_rot, aEf(num));
  }
  else
  {
    // Normalize rotation axis
    Vec3d_constMulSelf(a_rot, 1.0/lengthRot);
  }

  return lengthRot;
}

/*******************************************************************************
 * Returns a pointer to the desired inclination axis.
 ******************************************************************************/
const double* Rcs::TaskInclination::aRef() const
{
  const RcsBody* refBody = getRefBody();

  if (refBody)
  {
    return refBody->A_BI.rot[this->refDirection];
  }
  else
  {
    return Vec3d_unitVector(this->refDirection);
  }
}

/*******************************************************************************
 * Returns a pointer to the current inclination axis of effector num.
 ******************************************************************************/
const double* Rcs::TaskInclination::aEf(size_t num) const
{
  RCHECK(num < effectorVec.size());

  return getEffectorVec(num)->A_BI.rot[this->direction];
}

/*******************************************************************************
 * Returns a pointer to the effector num.
 ******************************************************************************/
const RcsBody* Rcs::TaskInclination::getEffectorVec(size_t num) const
{
  int id = this->effectorVec[num];

  if (id>=0)
  {
    return RCSBODY_BY_ID(this->graph, id);
  }

  // Generic bodies
  if (id<=-10)
  {
    int gBdyId = -id-10;
    RCHECK(gBdyId<RCS_NUM_GENERIC_BODIES);
    return RCSBODY_BY_ID(this->graph, graph->gBody[gBdyId]);
  }

  return NULL;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::TaskInclination::setIdsToSuffix(const std::string& suffix)
{
  bool success = true;
  std::vector<int> newEffectorVec = effectorVec;

  for (size_t i=0; i<effectorVec.size(); ++i)
  {
    const RcsBody* bdy = RCSBODY_BY_ID(getGraph(), effectorVec[i]);

    if (!bdy)
    {
      RLOG(1, "Task \"%s\": Found NULL body in effectorVec[%zu]",
           getName().c_str(), i);
      success = false;
      continue;
    }

    std::string newName = std::string(bdy->name) + suffix;
    const RcsBody* bdy_suffixed = RcsGraph_getBodyByName(graph, newName.c_str());

    if ((!bdy_suffixed) || (bdy_suffixed->id == -1))
    {
      RLOG(1, "Body \"%s\" not found or invalid - setIdsToSuffix() failed",
           newName.c_str());
      success = false;
    }
    else
    {
      newEffectorVec[i] = bdy_suffixed->id;
    }
  }

  if (success)
  {
    effectorVec = newEffectorVec;
  }

  return success;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::TaskInclination::isValid(xmlNode* node, const RcsGraph* graph)
{
  char taskName[256] = "unnamed task";
  getXMLNodePropertyStringN(node, "name", taskName, 256);

  // Check if reference body and frame exists in the graph, if it is specified
  // in the xml node. We don't check the effector here, because this task may
  // have a string list of effector names. This is checked below
  bool success = checkBody(node, "refBody", graph, taskName);
  success = checkBody(node, "refBdy", graph, taskName) && success;
  success = checkBody(node, "refFrame", graph, taskName) && success;

  // We don't check the effector
  std::vector<std::string> vec =
    getXMLNodePropertyVecSTLString(node, "effector");

  for (size_t i=0; i<vec.size(); ++i)
  {
    if (RcsGraph_getBodyByName(graph, vec[i].c_str()) == NULL)
    {
      RLOG(3, "Task \"%s\": Effector number %zu \"%s\" not found",
           taskName, i, vec[i].c_str());
      success = false;
    }
  }


  // Check proper class name
  success = Rcs::Task::isValid(node, graph, "Inclination") && success;


  // Check if axis direction is X, Y or Z
  char text[256] = "Z";
  getXMLNodePropertyStringN(node, "axisDirection", text, 256);

  if ((!STRCASEEQ(text, "X")) &&
      (!STRCASEEQ(text, "Y")) &&
      (!STRCASEEQ(text, "Z")))
  {
    success = false;

    REXEC(3)
    {
      char taskName[256] = "unnamed task";
      getXMLNodePropertyStringN(node, "name", taskName, 256);
      RMSG("Task \"%s\": Axis direction not [0...2]: %s", taskName, text);
    }
  }

  // Check if reference axis direction is X, Y or Z
  char textRef[256] = "Z";
  getXMLNodePropertyStringN(node, "refAxisDirection", textRef, 256);

  if ((!STRCASEEQ(textRef, "X")) &&
      (!STRCASEEQ(textRef, "Y")) &&
      (!STRCASEEQ(textRef, "Z")))
  {
    success = false;

    REXEC(3)
    {
      char taskName[256] = "unnamed task";
      getXMLNodePropertyStringN(node, "name", taskName, 256);
      RMSG("Task \"%s\": Reference axis direction not [0...2]: %s",
           taskName, textRef);
    }
  }

  return success;
}
