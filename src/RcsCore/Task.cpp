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

#include "Task.h"
#include "TaskFactory.h"
#include "TaskRegionFactory.h"
#include "Rcs_macros.h"
#include "Rcs_parser.h"
#include "Rcs_stlParser.h"
#include "Rcs_typedef.h"
#include "Rcs_math.h"
#include "Rcs_utils.h"
#include "Rcs_joint.h"
#include "Rcs_kinematics.h"
#include "StackVec.h"

#include <cfloat>

typedef Rcs::StackVec<double, 8> TaskVec;



/*******************************************************************************
 * Constructor of nested class Parameters
 ******************************************************************************/
Rcs::Task::Parameters::Parameters(const double minVal_,
                                  const double maxVal_,
                                  const double scaleFactor_,
                                  const std::string& name_):
  minVal(minVal_),
  maxVal(maxVal_),
  scaleFactor(scaleFactor_),
  name(name_)
{
}

/*******************************************************************************
 * Setter method of nested class Parameters
 ******************************************************************************/
void Rcs::Task::Parameters::setParameters(const double minVal_,
                                          const double maxVal_,
                                          const double scaleFactor_,
                                          const std::string& name_)
{
  this->minVal = minVal_;
  this->maxVal = maxVal_;
  this->scaleFactor = scaleFactor_;
  this->name.assign(name_);
}

/*******************************************************************************
 * Default constructor
 ******************************************************************************/
Rcs::Task::Task():
  graph(NULL),
  tsr(NULL),
  effectorId(-1),
  refBodyId(-1),
  refFrameId(-1),
  taskDim(0)
{
}

/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::Task::Task(const std::string& className_,
                xmlNode* node,
                RcsGraph* graph_,
                int dim):
  graph(graph_),
  tsr(NULL),
  effectorId(-1),
  refBodyId(-1),
  refFrameId(-1),
  taskDim(dim),
  className(className_)
{
  RCHECK(node);
  RCHECK(graph);

  // parse the xml node
  // Task name
  this->name = getXMLNodePropertySTLString(node, "name");

  // initialize parameter list with default values
  for (unsigned int i = 0; i < getDim(); i++)
  {
    addParameter(Parameters(-1.0, 1.0, 1.0, "unnamed"));
  }

  // Parse end effectors
  char msg[RCS_MAX_NAMELEN] = "";

  // Effector
  if (getXMLNodePropertyStringN(node, "effector", msg, RCS_MAX_NAMELEN))
  {
    if (STRNEQ(msg, "GenericBody", 11))
    {
      this->effectorId = getGenericBodyId(msg);
    }
    else
    {
      const RcsBody* ef = RcsGraph_getBodyByName(graph, msg);

      if (ef == NULL)
      {
        RLOG(1, "Effector \"%s\" doesn't exist!", msg);
        this->effectorId = -1;
      }
      else
      {
        this->effectorId = ef->id;
      }
    }
  }

  // Reference body
  if (getXMLNodePropertyStringN(node, "refBdy", msg, RCS_MAX_NAMELEN) ||
      getXMLNodePropertyStringN(node, "refBody", msg, RCS_MAX_NAMELEN))
  {
    if (STRNEQ(msg, "GenericBody", 11))
    {
      this->refBodyId = getGenericBodyId(msg);
    }
    else
    {
      const RcsBody* refBody = RcsGraph_getBodyByName(graph, msg);

      if (refBody == NULL)
      {
        RLOG(1, "Reference body \"%s\" doesn't exist!", msg);
        this->refBodyId = -1;
      }
      else
      {
        this->refBodyId = refBody->id;
      }
    }
  }

  // Reference frame body
  if (getXMLNodePropertyStringN(node, "refFrame", msg, RCS_MAX_NAMELEN))
  {
    if (STRNEQ(msg, "GenericBody", 11))
    {
      this->refFrameId = getGenericBodyId(msg);
    }
    else
    {
      const RcsBody* refFrame = RcsGraph_getBodyByName(graph, msg);

      if (refFrame == NULL)
      {
        RLOG(1, "Reference frame body \"%s\" doesn't exist!", msg);
        this->refFrameId = -1;
      }
      else
      {
        this->refFrameId = refFrame->id;
      }
    }
  }

  // in case no reference frame was given, set it to the reference body
  if (this->refFrameId == -1)
  {
    this->refFrameId = this->refBodyId;
  }

  // Go through all children and look for TaskRegion
  node = node->children;
  while (node)
  {
    if (isXMLNodeName(node, "TaskRegion"))
    {
      RLOG(5, "Found TaskRegion");
      this->tsr = TaskRegionFactory::create(this, node);

      // Create the new task, and add it to the task list
      if (tsr==NULL)
      {
        RLOG(1, "Failed to instantiate TaskRegion");
      }
      else
      {
        RLOG(5, "Succeeded to instantiate TaskRegion");
      }

    }

    node = node->next;
  }

  RLOG(5, "constructed task \"%s\" of type %s: dim: %d, dimIK: %d",
       getName().c_str(), className.c_str(), getDim(), getDim());
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::Task::Task(const Task& copyFromMe) :
  graph(copyFromMe.graph),
  tsr(copyFromMe.tsr ? copyFromMe.tsr->clone() : NULL),
  effectorId(copyFromMe.effectorId),
  refBodyId(copyFromMe.refBodyId),
  refFrameId(copyFromMe.refFrameId),
  taskDim(copyFromMe.taskDim),
  name(copyFromMe.name),
  className(copyFromMe.className),
  params(copyFromMe.params)
{
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::Task::setGraph(RcsGraph* newGraph)
{
  if (newGraph == this->graph)
  {
    return;
  }

  // If the task is created with a new graph, we don't assume that the body
  // topology is the same as in the task we copy from. Therefore we retrieve
  // the task's bodies by name lookup.

    // The following section is the same for effector, refBdy and refFrame. We
    // check if the id >= 0. If this is not the the case, the body
    // is either not set, or refers to a GenericBody (with id range [-10 ...].
    // In such cases, we just keep the id, and do not resolve the id by name.
  int otherId = getEffectorId();
    if (otherId >= 0)
    {
    const RcsBody* otherBdy = RCSBODY_BY_ID(getGraph(), otherId);
      if (otherBdy)
      {
        const RcsBody* myBdy = RcsGraph_getBodyByName(newGraph, otherBdy->name);
        setEffectorId(myBdy ? myBdy->id : -1);
      }
    }

  otherId = getRefBodyId();
    if (otherId >= 0)
    {
    const RcsBody* otherBdy = RCSBODY_BY_ID(getGraph(), otherId);
      if (otherBdy)
      {
        const RcsBody* myBdy = RcsGraph_getBodyByName(newGraph, otherBdy->name);
        setRefBodyId(myBdy ? myBdy->id : -1);
      }
    }

  otherId = getRefFrameId();
    if (otherId >= 0)
    {
    const RcsBody* otherBdy = RCSBODY_BY_ID(getGraph(), otherId);
      if (otherBdy)
      {
        const RcsBody* myBdy = RcsGraph_getBodyByName(newGraph, otherBdy->name);
        setRefFrameId(myBdy ? myBdy->id : -1);
    }
  }

  this->graph = newGraph;
}

/*******************************************************************************
 * Destructor. Calling delete on NULL is safe.
 ******************************************************************************/
Rcs::Task::~Task()
{
  delete this->tsr;
}

/*******************************************************************************
 * name is "GenericBodyX" with X = [0...RCS_NUM_GENERIC_BODIES-1]
 ******************************************************************************/
int Rcs::Task::getGenericBodyId(const char* name) const
{
  int id, num = atoi(&name[11]);

  if ((num < 0) || (num >= RCS_NUM_GENERIC_BODIES))
  {
    RLOG(1, "GenericBody \"%s\": suffix must be [0...%d]",
         name, RCS_NUM_GENERIC_BODIES-1);
    id = -1;
  }
  else
  {
    id = -10-num;
  }

  return id;
}

/*******************************************************************************
 * Returns the dimension of the task
 ******************************************************************************/
unsigned int Rcs::Task::getDim() const
{
  return this->taskDim;
}

/*******************************************************************************
 * Sets the dimension of the task. You shouldn't need to use this unless you
 * create your own CompositeTask.
 ******************************************************************************/
void Rcs::Task::setDim(unsigned int dim)
{
  this->taskDim = dim;
}

/*******************************************************************************
 * Sets the name of the task.
 ******************************************************************************/
void Rcs::Task::setName(const std::string& newName)
{
  this->name = newName;
}

/*******************************************************************************
 * Returns reference to the name of the task
 ******************************************************************************/
const std::string& Rcs::Task::getName() const
{
  return this->name;
}

/*******************************************************************************
 * Returns pointer to internal graph
 ******************************************************************************/
RcsGraph* Rcs::Task::getGraph() const
{
  return this->graph;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::Task::print() const
{
  printf("Task \"%s\": type \"%s\" with %d dimensions\n",
         getName().c_str(), getClassName().c_str(), getDim());

  if (getEffector())
  {
    printf("Effector: \"%s\"\n", getEffector()->name);
  }

  if (getRefBody())
  {
    printf("Reference body: \"%s\"\n", getRefBody()->name);
  }

  if (getRefFrame())
  {
    printf("Reference frame: \"%s\"\n", getRefFrame()->name);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::TaskRegion* Rcs::Task::getTaskRegion()
{
  return this->tsr;
}

/*******************************************************************************
 * Returns the parameters for task dimension index
 ******************************************************************************/
Rcs::Task::Parameters& Rcs::Task::getParameter(size_t index)
{
  RCHECK_MSG(index<this->params.size(), "%zu %zu", index, this->params.size());
  return this->params.at(index);
}

/*******************************************************************************
 * Returns the parameters for task dimension index
 ******************************************************************************/
const Rcs::Task::Parameters& Rcs::Task::getParameter(size_t index) const
{
  RCHECK_MSG(index<this->params.size(), "%zu %zu", index, this->params.size());
  return this->params.at(index);
}

/*******************************************************************************
 * Returns the whole parameter list
 ******************************************************************************/
const std::vector<Rcs::Task::Parameters>& Rcs::Task::getParameters() const
{
  return this->params;
}

/*******************************************************************************
 * Returns the whole parameter list
 ******************************************************************************/
std::vector<Rcs::Task::Parameters>& Rcs::Task::getParameters()
{
  return this->params;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::Task::clearParameters()
{
  params.clear();
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::Task::addParameter(const Task::Parameters& newParam)
{
  params.push_back(newParam);
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::Task::removeParameter(size_t index)
{
  if (index > params.size() - 1)
  {
    RLOG_CPP(1, "Failed to erase task with index " << index
             << " - should be less than " << params.size());
    return false;
  }

  params.erase(params.begin() + index);

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::Task::resetParameter(const Task::Parameters& newParam)
{
  clearParameters();
  params.push_back(newParam);
}

/*******************************************************************************
 * See header
 ******************************************************************************/
std::string Rcs::Task::getClassName() const
{
  return this->className;
}

/*******************************************************************************
 * See header
 ******************************************************************************/
void Rcs::Task::setClassName(const std::string& className_)
{
  this->className = className_;
}

/*******************************************************************************
 * Computes the current velocity x_dot = J q_dot. For tasks with different
 * representations on position and velocitiy levels, there is a mismatch.
 ******************************************************************************/
void Rcs::Task::computeXp_ik(double* x_dot_res) const
{
  const unsigned int dim = getDim();

  // Get the current joint velocities
  MatNd* q_dot = NULL;
  MatNd_create2(q_dot, this->graph->nJ, 1);
  RcsGraph_stateVectorToIK(this->graph, this->graph->q_dot, q_dot);

  // Calculate x_dot = J q_dot
  MatNd x_dot = MatNd_fromPtr(dim, 1, x_dot_res);
  MatNd* J = NULL;
  MatNd_create2(J, dim, this->graph->nJ);
  computeJ(J);
  MatNd_mul(&x_dot, J, q_dot);

  // Clean up
  MatNd_destroy(J);
  MatNd_destroy(q_dot);
}

/*******************************************************************************
 * Computes the current acceleration in task space
 ******************************************************************************/
void Rcs::Task::computeXpp_ik(double* x_ddot, const MatNd* q_ddot) const
{
  // x_ddot = Jdot q_dot + J q_ddot
  MatNd x_ddot_ = MatNd_fromPtr(getDim(), 1, x_ddot);
  computeJdotQdot(&x_ddot_); //Jdot q_dot

  MatNd* J = NULL;
  MatNd_create2(J, getDim(), this->graph->nJ);
  computeJ(J);

  // Get the current joint accelerations
  MatNd* q_ddot_IK = NULL;
  MatNd_create2(q_ddot_IK, this->graph->nJ, 1);
  RcsGraph_stateVectorToIK(this->graph, q_ddot, q_ddot_IK);

  MatNd_mulAndAddSelf(&x_ddot_, J, q_ddot_IK); // Jdot q_dot + J q_ddot

  // Clean up
  MatNd_destroy(q_ddot_IK);
  MatNd_destroy(J);
}

/*******************************************************************************
 * Computes the delta in task space for the differential kinematics
 ******************************************************************************/
void Rcs::Task::computeDX(double* dx, const double* x_des) const
{
  TaskVec x_curr(getDim());
  computeX(x_curr);
  computeDX(dx, x_des, x_curr);
}

/*******************************************************************************
 * Text book: q_dot^T H
 * in our memory layout: H q_dot
 ******************************************************************************/
void Rcs::Task::computeJdot(MatNd* Jdot) const
{
  size_t nq = this->graph->nJ;
  size_t nx = getDim();
  MatNd* hessian = NULL;
  MatNd_create2(hessian, nx*nq, nq);

  computeH(hessian);
  MatNd_reshape(Jdot, nx*nq, 1);

  // Get the current joint velocities
  MatNd* q_dot = NULL;
  MatNd_create2(q_dot, this->graph->nJ, 1);
  RcsGraph_stateVectorToIK(this->graph, this->graph->q_dot, q_dot);

  MatNd_mul(Jdot, hessian, q_dot);
  MatNd_reshape(Jdot, nx, nq);

  MatNd_destroy(hessian);
  MatNd_destroy(q_dot);
}

/*******************************************************************************
 * q_dot^T H q_dot
 ******************************************************************************/
void Rcs::Task::computeJdotQdot(MatNd* JdotQdot) const
{
  size_t nq = this->graph->nJ;
  size_t nx = getDim();
  MatNd* Jdot = NULL;
  MatNd_create2(Jdot, nx, nq);

  computeJdot(Jdot);
  MatNd_reshape(JdotQdot, nx, 1);

  // Get the current joint velocities
  MatNd* q_dot = NULL;
  MatNd_create2(q_dot, this->graph->nJ, 1);
  RcsGraph_stateVectorToIK(this->graph, this->graph->q_dot, q_dot);

  MatNd_mul(JdotQdot, Jdot, q_dot);

  MatNd_destroy(Jdot);
  MatNd_destroy(q_dot);
}

/*******************************************************************************
 * g = 0.5*dx^T W dx
 ******************************************************************************/
double Rcs::Task::computeTaskCost(const double* x_des,
                                  const double* diagW) const
{
  unsigned int i, nx = getDim();
  double goalCost = 0.0;
  TaskVec dx(nx);

  computeDX(dx, x_des);

  if (diagW!=NULL)
  {
    for (i = 0; i < nx; i++)
    {
      goalCost += dx[i] * dx[i] * diagW[i];
    }
  }
  else
  {
    for (i = 0; i < nx; i++)
    {
      goalCost += dx[i] * dx[i];
    }
  }

  return 0.5 * goalCost;
}

/*******************************************************************************
 * dg/dq = dx^T W J
 ******************************************************************************/
void Rcs::Task::computeTaskGradient(MatNd* dgDq,
                                    const double* x_des,
                                    const double* diagW) const
{
  unsigned int nq = this->graph->nJ, nx = getDim();
  MatNd* dx=NULL, *J=NULL;

  MatNd_create2(dx, 1, nx);
  MatNd_create2(J, nx, nq);

  // dx^T W J
  computeDX(dx->ele, x_des);

  if (diagW!=NULL)
  {
    VecNd_eleMulSelf(dx->ele, diagW, nx);
  }

  computeJ(J);

  MatNd_reshape(dgDq, 1, nq);
  MatNd_mul(dgDq, dx, J);

  // Clean up
  MatNd_destroy(dx);
  MatNd_destroy(J);
}

/*******************************************************************************
 * Computes the feed-back acceleration:
 *
 * ax = x_ddot_des + kp*dx + kd*dx_dot + ki Int(dx)
 *
 *         Textbook: kp=omega^2, kd=2*omega
 ******************************************************************************/
void Rcs::Task::computeAX(double* a_res,
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
  const unsigned int dim = getDim();

  // calculate position error dx using the task's specialized function
  TaskVec dx(dim);
  this->computeDX(dx, x_des);

  // a_res = kp * dx
  VecNd_constMul(a_res, dx, kp, dim);

  // ax += kd * dx_dot
  // calculate velocity error dx_dot using the task's specialized function
  TaskVec dx_dot(dim);
  computeDXp(dx_dot, x_dot_des);
  VecNd_constMulAndAddSelf(a_res, dx_dot, kd, dim);

  // ax += ff_x_ddot
  if (x_ddot_des != NULL)
  {
    TaskVec ff_x_ddot(dim);
    this->computeFfXpp(ff_x_ddot, x_ddot_des);
    VecNd_addSelf(a_res, ff_x_ddot, dim);
  }

  // Integral term. This must be done in the space of the inverse kinematics
  // projection, otherwise the integral term might get screwed up by jumps in
  // Euler or Polar angles.
  if (integral_x != NULL)
  {
    // x_error = -dx*selection
    TaskVec x_error(dim);

    if (S_des != NULL)
    {
      VecNd_eleMul(x_error, dx, S_des, dim);
    }
    else
    {
      VecNd_copy(x_error, dx, dim);
    }

    // x_int += gain*x_error
    // the gain is scaled by the activation
    VecNd_constMulAndAddSelf(integral_x, x_error, a_des*ki, dim);

    // saturate integral term to avoid windup
    // the clipping is scaled by the activation
    const double iLim = a_des*2.0;
    VecNd_clipSelf(integral_x, -iLim, iLim, dim);
    VecNd_addSelf(a_res, integral_x, dim);
  }

}

/*******************************************************************************
 * Computes the feed-back force
 ******************************************************************************/
void Rcs::Task::computeAF(double* ft_res,
                          double* ft_int,
                          const double* ft_des,
                          const double* selection,
                          const double* ft_task,
                          const double a_des,
                          const double kp,
                          const double ki) const
{
  if (ft_des == NULL)
  {
    VecNd_setZero(ft_res, getDim());
    return;
  }

  const unsigned int dim = getDim();

  // feed-forward part of force
  VecNd_copy(ft_res, ft_des, dim);

  if (ft_task != NULL)
  {
    // ft_error = (ft_des - ft_current)
    TaskVec ft_error(dim);
    VecNd_sub(ft_error, ft_des, ft_task, dim);

    // proportional part
    VecNd_constMulAndAddSelf(ft_res, ft_error, kp, dim);

    if (ft_int != NULL)
    {
      // integrate proportional to the selection
      for (unsigned int i = 0; i < dim; i++)
      {
        // force selection = complement of selection
        ft_error[i] *= (1.0-selection[i]);
      }
      // the gain is scaled by the task activation
      VecNd_constMulAndAddSelf(ft_int, ft_error, a_des*ki, dim);

      // saturate integral term to avoid windup
      // the clipping is scaled by the task activation
      const double iLim = a_des*10.0;
      VecNd_clipSelf(ft_int, -iLim, iLim, dim);

      // add integral part
      VecNd_addSelf(ft_res, ft_int, dim);
    }

  }
}

/*******************************************************************************
 * Finite difference test for the Hessian
 ******************************************************************************/
static void testJ(double* f, const double* x, void* params)
{
  Rcs::Task* self = (Rcs::Task*) params;
  unsigned int nx = self->getDim();
  unsigned int nq = self->getGraph()->nJ;
  MatNd q = MatNd_fromPtr(nq, 1, (double*) x);
  MatNd J = MatNd_fromPtr(nx, nq, f);
  RcsGraph_setState(self->getGraph(), &q, NULL);
  self->computeJ(&J);
}

static void testH(double* f, const double* x, void* params)
{
  Rcs::Task* self = (Rcs::Task*) params;
  unsigned int nx = self->getDim();
  unsigned int nq = self->getGraph()->nJ;
  MatNd q = MatNd_fromPtr(nq, 1, (double*) x);
  MatNd H = MatNd_fromPtr(nx, nq*nq, f);

  RcsGraph_setState(self->getGraph(), &q, NULL);
  self->computeH(&H);
}

bool Rcs::Task::testHessian(bool verbose)
{
  double tolerance = 0.01;   // We accept 1% error
  MatNd* q_ik = MatNd_create(getGraph()->nJ, 1);
  RcsGraph_stateVectorToIK(getGraph(), getGraph()->q, q_ik);

  bool success = Rcs_testGradient(testJ, testH, this, q_ik->ele,
                                  getGraph()->nJ, getDim()*getGraph()->nJ,
                                  tolerance, verbose);

  MatNd_destroy(q_ik);

  if (verbose == true)
  {
    RLOG(1, "Task %s: Hessian test %s",
         getName().c_str(), success ? "SUCCESS" : "FAILURE");
  }

  return success;
}

/*******************************************************************************
 * Finite difference test for the Jacobian
 ******************************************************************************/
bool Rcs::Task::testJacobian(double errorLimit,
                             double delta,
                             bool relativeError,
                             bool verbose)
{
  const unsigned int nx = getDim();

  bool success = true;
  MatNd* x0, *x1, *dx;
  MatNd_create2(x0, nx, 1);
  MatNd_create2(x1, nx, 1);
  MatNd_create2(dx, nx, 1);

  // Set the state.
  RcsGraph_computeForwardKinematics(this->graph, NULL, NULL);

  // Memorize the graph's state for the finite difference computation
  MatNd* q0;
  MatNd_clone2(q0, this->graph->q);

  // Compute the task vector for the current state
  computeX(x0->ele);

  // Compute the Jacobian analytically for the given state
  MatNd* J = NULL;
  MatNd_create2(J, nx, this->graph->nJ);
  computeJ(J);

  // Numerical Jacobian approximation with finite differences
  MatNd* J_fd = NULL;
  MatNd_create2(J_fd, J->m, J->n);

  // Error
  MatNd* J_err = NULL;
  MatNd_create2(J_err, J->m, J->n);

  RCSGRAPH_TRAVERSE_JOINTS(this->graph)
  {
    // Ignore dofs that are constrained
    if (JNT->jacobiIndex == -1)
    {
      continue;
    }

    // Determine the finite difference step according to the joint type:
    // rotational joint: 0.01 degree, translational joint: 0.01mm
    double delta = RcsJoint_isRotation(JNT) ? 0.01*(M_PI/180.0) : 0.01*0.001;

    // Calculate task variable x1 after applying finite difference step
    MatNd_copy(this->graph->q, q0);
    MatNd_addToEle(this->graph->q, JNT->jointIndex, 0, delta);

    // Here we don't use the RcsGraph_setState() function, since it also
    // updates potentially kinematically coupled joints. Here we don't
    // consider any kinematic coupling, and therefore don't want to see
    // them in the pertubations of the finite differences.
    RcsGraph_computeForwardKinematics(this->graph, NULL, NULL);
    computeX(x1->ele);

    // Reset state and calculate dx
    RcsGraph_setState(this->graph, q0, NULL);
    computeDX(dx->ele, x1->ele);
    VecNd_constMulSelf(dx->ele, 1.0 / delta, nx);
    MatNd_setColumn(J_fd, JNT->jacobiIndex, dx->ele, nx);
  }

  // Compute error: percentual difference of solutions
  double err = 0.0;
  const double epsDenom = 1.0e-2;
  unsigned int idx_errRow = 0, idx_errCol = 0;

  // Find the max. absolut element of the analytic derivative. It is
  // used to compute the relative error of each finite difference
  // approximation. We add a tiny value to avoid divisions by zero
  // later.
  double maxAbsEleJ = fabs(J_fd->ele[0]);

  for (unsigned int i = 0; i < J->m*J->n; i++)
  {
    if (fabs(J_fd->ele[i]) > maxAbsEleJ)
    {
      maxAbsEleJ = fabs(J_fd->ele[i]);
    }
  }

  //double scaleMag = maxAbsEleJ > 1.0 ? maxAbsEleJ : 1.0;

  for (unsigned int rows = 0; rows < J->m; rows++)
  {
    for (unsigned int cols = 0; cols < J->n; cols++)
    {
      int idx_ele = rows*J->n+cols;
      double denom = 1.0;

      if (relativeError==true)
      {
        // Calculate the approximation error. We refer to the max. of absolute
        // values of the analytic and numeric gradients. In order to avoid
        // numerical artefacts, we impose a minimum norm for the denominator.
        // This also avoids divisions by zero if the gradients are 0. The
        // denominator is always larger than epsDenom. If the largest element
        // of the gradient is larger than 1, the denominator is scaled with
        // it. This is an implicit normalization of the gradient and avoids
        // numerical effects if the gradient magnitude is very large.
        // denom = fabs(J->ele[idx_ele]) > fabs(J_fd->ele[idx_ele]) ?
        //         fabs(J->ele[idx_ele]) : fabs(J_fd->ele[idx_ele]);

        // if (denom < scaleMag*epsDenom)
        // {
        //   denom = scaleMag*epsDenom;
        // }

        // We normalize with the largest absolute value of the finite difference
        // Jacobian and add a small regularizer to avoid division by zero.
        denom = maxAbsEleJ + epsDenom;

      }   // if (relativeError==true)

      double err_i = fabs(J->ele[idx_ele] - J_fd->ele[idx_ele]) / denom;

      J_err->ele[idx_ele] = err_i;

      if (err_i > err)
      {
        err = err_i;
        idx_errRow = rows;
        idx_errCol = cols;
      }
    }
  }




  if (err > errorLimit)
  {
    success = false;

    if (verbose == true)
    {
      if (relativeError==true)
      {
        RMSG("Task %s: Failure: err = %.1f %%, index = %d %d",
             getName().c_str(), 100.0*err, idx_errRow, idx_errCol);
      }
      else
      {
        RMSG("Task %s: Failure: err = %g, index = %d %d",
             getName().c_str(), err, idx_errRow, idx_errCol);
      }

      RMSG("J");
      MatNd_printDigits(J, 4);
      RMSG("J_fd");
      MatNd_printDigits(J_fd, 4);
      if (relativeError==true)
      {
        MatNd_constMulSelf(J_err, 100.0);
        RMSG("diff [%%]");
      }
      else
      {
        RMSG("diff [abs]");
      }

      MatNd_printDigits(J_err, 4);
    }
  }
  else
  {
    if (verbose == true)
    {
      RLOG(1, "Task %s: Jacobian test success: err = %g",
           getName().c_str(), err);
    }
  }

  REXEC(5)
  {
    RMSG("J");
    MatNd_printDigits(J, 4);
    RMSG("J_fd");
    MatNd_printDigits(J_fd, 4);
    RMSG("diff [%%]");
    MatNd_printDigits(J_err, 4);
  }

  // Reset graph to original state
  RcsGraph_setState(this->graph, q0, NULL);

  MatNd_destroy(q0);
  MatNd_destroy(J_fd);
  MatNd_destroy(J_err);
  MatNd_destroy(J);

  MatNd_destroy(x0);
  MatNd_destroy(x1);
  MatNd_destroy(dx);

  return success;
}

/*******************************************************************************
 * Tests computeXp_ik = J q_dot
 ******************************************************************************/
bool Rcs::Task::testVelocity(double maxErr) const
{
  // Compute the Jacobian projection
  MatNd* x1 = NULL;
  MatNd_fromStack(x1, getDim(), 1);
  Task::computeXp_ik(x1->ele);

  // Compute with the specialized function
  MatNd* x2 = NULL;
  MatNd_fromStack(x2, getDim(), 1);
  computeXp_ik(x2->ele);

  double err = MatNd_rmsqError(x1, x2);
  bool success = err < maxErr;

  RLOG(2, "Task %s: velocity test %s: rms error = %g",
       getName().c_str(), success ? "SUCCESS" : "FAILURE", err);

  REXEC(3)
  {
    RLOG(3, "\nJ*q_dot   computeXp_ik()   diff");
    MatNd_printTwoArraysDiff(x1, x2, 6);
  }

  return success;
}

/*******************************************************************************
 * See header
 ******************************************************************************/
bool Rcs::Task::test(bool verbose)
{
  const double delta = 1.0e-6;
  bool success = true;
  bool success_i;
  bool relativeError = true;
  double errorLimit;

  if (relativeError == true)
  {
    errorLimit = 5.0e-2; // 5.0% error limit
  }
  else
  {
    errorLimit = 1.0e-4;
  }

  success_i = testJacobian(errorLimit, delta, relativeError, verbose);
  success = success_i && success;

  success_i = testHessian(verbose);
  success = success_i && success;

  success_i = testVelocity();
  success = success_i && success;

  return success;
}

/*******************************************************************************
 * See header
 ******************************************************************************/
bool Rcs::Task::isValid(xmlNode* node, const RcsGraph* graph,
                        const char* className)
{
  std::vector<std::string> classNameVec;
  classNameVec.push_back(std::string(className));
  return isValid(node, graph, classNameVec);
}

/*******************************************************************************
 * See header
 ******************************************************************************/
bool Rcs::Task::isValid(xmlNode* node, const RcsGraph* graph,
                        const std::vector<std::string>& className)
{
  bool success = true;

  // Get the name
  std::string taskName = getXMLNodePropertySTLString(node, "name");


  // Check is the task has a control variable
  if (getXMLNodeProperty(node, "controlVariable")==false)
  {
    success = false;

    REXEC(3)
    {
      RMSG("Task \"%s\" has no control variable tag", taskName.c_str());
    }
  }


  // Check if control variable is within className vector
  char tag[256] = "";
  bool cVarInVec = false;
  getXMLNodePropertyStringN(node, "controlVariable", tag, 256);

  for (size_t i=0; i<className.size(); ++i)
  {
    if (STRCASEEQ(className[i].c_str(), tag))
    {
      cVarInVec = true;
    }
  }


  if (cVarInVec == false)
  {
    success = false;

    REXEC(3)
    {
      RMSG("Task \"%s\": Control variable \"%s\" is not found in",
           taskName.c_str(), tag);

      for (size_t i=0; i<className.size(); ++i)
      {
        RMSG("%s", className[i].c_str());
      }
    }
  }

  // Check if effector exists in the graph, if it is specified in the xml node
  success = checkBody(node, "effector", graph, taskName.c_str()) && success;
  success = checkBody(node, "refBody", graph, taskName.c_str()) && success;
  success = checkBody(node, "refBdy", graph, taskName.c_str()) && success;
  success = checkBody(node, "refFrame", graph, taskName.c_str()) && success;

  return success;
}

/*******************************************************************************
 * See header
 ******************************************************************************/
Rcs::Task* Rcs::Task::createRandom(std::string className, RcsGraph* graph)
{
  int rndId = Math_getRandomInteger(-1, graph->nBodies-1);
  const RcsBody* ef = RCSBODY_BY_ID(graph, rndId);
  rndId = Math_getRandomInteger(-1, graph->nBodies-1);
  const RcsBody* refBdy = RCSBODY_BY_ID(graph, rndId);
  rndId = Math_getRandomInteger(-1, graph->nBodies-1);
  int rnd10 = Math_getRandomInteger(0, 10);
  const RcsBody* refFrame = rnd10==0 ? RCSBODY_BY_ID(graph, rndId) : refBdy;

  std::string tStr = "<Task name=\"Random\" ";
  tStr += "controlVariable=\"" + className + "\" ";
  if (ef)
  {
    tStr += "effector=\"" + std::string(ef->name) + "\" ";
  }
  if (refBdy)
  {
    tStr += "refBdy=\"" + std::string(refBdy->name) + "\" ";
  }
  if (refFrame && refFrame!=refBdy)
  {
    tStr += "refFrame=\"" + std::string(refFrame->name) + "\" ";
  }

  tStr += " />";

  Task* task = TaskFactory::createTask(tStr, graph);

  if (!task)
  {
    RLOG(1, "createRandom() not working for task of type \"%s\":\n[%s]",
         className.c_str(), tStr.c_str());
    return NULL;
  }

  for (unsigned int i = 0; i < task->getDim(); i++)
  {
    task->addParameter(Parameters(-1.0, 1.0, 1.0, "unnamed"));
  }

  return task;
}

/*******************************************************************************
 * Computes the current task forces. Here's what we do:
 *    f_task = J_task J_sensor# f_sensor
 ******************************************************************************/
void Rcs::Task::projectTaskForce(MatNd* ft_task,
                                 const RcsSensor* loadCell) const
{
  if (loadCell == NULL)
  {
    MatNd_reshapeAndSetZero(ft_task, getDim(), 1);
    return;
  }

  const double* S_ft = loadCell->rawData->ele;

  RcsBody* loadCellBdy = &graph->bodies[loadCell->bodyId];

  // Transform sensor values into world coordinates
  double I_ft[6];
  MatNd I_ft_ = MatNd_fromPtr(6, 1, I_ft);
  VecNd_copy(I_ft, S_ft, 6);
  Vec3d_transRotateSelf(&I_ft[0], (double (*)[3])loadCell->A_SB.rot);
  Vec3d_transRotateSelf(&I_ft[0], loadCellBdy->A_BI.rot);
  Vec3d_transRotateSelf(&I_ft[3], (double (*)[3])loadCell->A_SB.rot);
  Vec3d_transRotateSelf(&I_ft[3], loadCellBdy->A_BI.rot);

  // Compute the sensor Jacobian
  MatNd* J_sensor = NULL;
  MatNd_create2(J_sensor, 6, this->graph->nJ);
  RcsGraph_bodyPointJacobian(getGraph(), loadCellBdy, loadCell->A_SB.org,
                             NULL, J_sensor);
  MatNd_reshape(J_sensor, 6, this->graph->nJ);
  MatNd J_rot = MatNd_fromPtr(3, this->graph->nJ, MatNd_getRowPtr(J_sensor, 3));
  RcsGraph_rotationJacobian(this->graph, loadCellBdy, NULL, &J_rot);

  // Sensor wrench Pseudo-inverse
  MatNd* pinvJ_sensor = NULL;
  MatNd_create2(pinvJ_sensor, J_sensor->n, J_sensor->m);
  MatNd_rwPinv(pinvJ_sensor, J_sensor, NULL, NULL);

  // Task Jacobian
  MatNd* J_task = NULL;
  MatNd_create2(J_task, getDim(), this->graph->nJ);
  computeJ(J_task);

  // J_task J_sensor#
  MatNd* JJt;
  MatNd_create2(JJt, J_task->m, pinvJ_sensor->n);
  MatNd_mul(JJt, J_task, pinvJ_sensor);

  MatNd_reshape(ft_task, J_task->m, 1);
  MatNd_mul(ft_task, JJt, &I_ft_);

  MatNd_destroy(J_sensor);
  MatNd_destroy(pinvJ_sensor);
  MatNd_destroy(J_task);
  MatNd_destroy(JJt);
}

/*******************************************************************************
 * Returns the effector body of the task
 ******************************************************************************/
int Rcs::Task::getEffectorId() const
{
  return this->effectorId;
}

/*******************************************************************************
 * Returns the refBody of the task
 ******************************************************************************/
int Rcs::Task::getRefBodyId() const
{
  return this->refBodyId;
}

/*******************************************************************************
 * Returns the refBody of the task
 ******************************************************************************/
int Rcs::Task::getRefFrameId() const
{
  return this->refFrameId;
}

/*******************************************************************************
 * Returns the effector body of the task
 ******************************************************************************/
const RcsBody* Rcs::Task::getEffector() const
{
  if (this->effectorId>=0)
  {
    return RCSBODY_BY_ID(this->graph, this->effectorId);
  }

  // Generic bodies
  if (this->effectorId<=-10)
  {
    int gBdyId = -this->effectorId-10;
    RCHECK(gBdyId<RCS_NUM_GENERIC_BODIES);
    return RCSBODY_BY_ID(this->graph, graph->gBody[gBdyId]);
  }

  return NULL;
}

/*******************************************************************************
 * Returns the refBody of the task
 ******************************************************************************/
const RcsBody* Rcs::Task::getRefBody() const
{
  if (this->refBodyId>=0)
  {
    return RCSBODY_BY_ID(this->graph, this->refBodyId);
  }

  // Generic bodies
  if (this->refBodyId<=-10)
  {
    int gBdyId = -this->refBodyId-10;
    RCHECK(gBdyId<RCS_NUM_GENERIC_BODIES);
    return RCSBODY_BY_ID(this->graph, graph->gBody[gBdyId]);
  }

  return NULL;
}

/*******************************************************************************
 * Returns the refBody of the task
 ******************************************************************************/
const RcsBody* Rcs::Task::getRefFrame() const
{
  if (this->refFrameId>=0)
  {
    return RCSBODY_BY_ID(this->graph, this->refFrameId);
  }

  // Generic bodies
  if (this->refFrameId<=-10)
  {
    int gBdyId = -this->refFrameId-10;
    RCHECK(gBdyId<RCS_NUM_GENERIC_BODIES);
    return RCSBODY_BY_ID(this->graph, graph->gBody[gBdyId]);
  }

  return NULL;
}

/*******************************************************************************
 * Overwrites the refFrame  body of the task
 ******************************************************************************/
void Rcs::Task::setRefFrameId(int referenceFrameId)
{
  this->refFrameId = referenceFrameId;
}

/*******************************************************************************
 * Overwrites the effector body of the task
 ******************************************************************************/
void Rcs::Task::setEffectorId(int effectorId)
{
  this->effectorId = effectorId;
}

/*******************************************************************************
 * Overwrites the refBody of the task
 ******************************************************************************/
void Rcs::Task::setRefBodyId(int referenceBodyId)
{
  this->refBodyId = referenceBodyId;
}

/*******************************************************************************
 * Computes the relative rotation matrix between refBdy and effector
 ******************************************************************************/
void Rcs::Task::computeRelativeRotationMatrix(double A_ER[3][3],
                                              const RcsBody* bdyEff,
                                              const RcsBody* bdyRef)
{
  if (bdyRef == NULL)
  {
    if (bdyEff != NULL)
    {
      // No refBody, but effector
      Mat3d_copy(A_ER, (double(*)[3]) bdyEff->A_BI.rot);
    }
    else
    {
      // Neither refBody, nor effector
      Mat3d_setIdentity(A_ER);
    }
  }
  else
  {
    if (bdyEff != NULL)
    {
      // refBody and effector
      Mat3d_mulTranspose(A_ER, (double(*)[3]) bdyEff->A_BI.rot,
                         (double(*)[3]) bdyRef->A_BI.rot);
    }
    else
    {
      // refBody, but no effector
      Mat3d_transpose(A_ER, (double(*)[3]) bdyRef->A_BI.rot);
    }

  }

}

/*******************************************************************************
 * Computes the current value of the task variable: XYZ Euler angles
 ******************************************************************************/
void Rcs::Task::computeEulerAngles(double* ea,
                                   const RcsBody* effector,
                                   const RcsBody* referenceBody)
{
  double A_ER[3][3];
  computeRelativeRotationMatrix(A_ER, effector, referenceBody);
  Mat3d_toEulerAngles(ea, A_ER);
}

/*******************************************************************************
 * Computes the current angular velocity
 ******************************************************************************/
void Rcs::Task::computeOmega(double* omega_curr) const
{
  const RcsBody* ef = getEffector();
  const RcsBody* refBody = getRefBody();
  const RcsBody* refFrame = getRefFrame();

  Vec3d_copy(omega_curr, ef ? ef->omega : Vec3d_zeroVec());

  if (refBody != NULL)
  {
    Vec3d_subSelf(omega_curr, refBody->omega);
    Vec3d_rotateSelf(omega_curr, (double (*)[3])refFrame->A_BI.rot);
  }

}

/*******************************************************************************
 * Check if effector exists in the graph, if it is specified in the xml node
 ******************************************************************************/
bool Rcs::Task::checkBody(xmlNode* node, const char* tag,
                          const RcsGraph* graph, const char* taskName)
{
  // If no such tag exists in the xml node, it's ok and we return success
  if (getXMLNodeProperty(node, tag)==false)
  {
    return true;
  }

  // If the tag exists, we read it out and check if the body exists in the
  // graph.
  char name[256] = "";
  getXMLNodePropertyStringN(node, tag, name, 256);
  RcsBody* bdy = RcsGraph_getBodyByName(graph, name);

  if (bdy == NULL)
  {
    RLOG(3, "Task \"%s\": %s \"%s\" not found", taskName, tag, name);
    return false;
  }

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::Task::toXML(FILE* out, bool activation) const
{
  toXMLStart(out);
  toXMLBody(out);
  toXMLEnd(out, activation);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::Task::toXMLStart(FILE* out) const
{
  fprintf(out, "<Task ");

  if (!getName().empty())
  {
    fprintf(out, "name=\"%s\" ", getName().c_str());
  }

  fprintf(out, "controlVariable=\"%s\"", getClassName().c_str());
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::Task::toXMLBody(FILE* out) const
{
  // Here, names of Generic Bodies remain preserved
  if (getEffectorId()<=-10)
  {
    fprintf(out, " effector=\"GenericBody%d\"", -10-getEffectorId());
  }
  else if (getEffectorId()>=0)
  {
    fprintf(out, " effector=\"%s\"", getEffector()->name);
  }

  if (getRefBodyId()<=-10)
  {
    fprintf(out, " refBdy=\"GenericBody%d\"", -10-getRefBodyId());
  }
  else if (getRefBodyId()>=0)
  {
    fprintf(out, " refBdy=\"%s\"", getRefBody()->name);
  }

  if (getRefFrameId()<=-10)
  {
    if (getRefFrameId()!= getRefBodyId())
    {
      fprintf(out, " refFrame=\"GenericBody%d\"", -10-getRefFrameId());
    }
  }
  else if (getRefFrameId()>=0)
  {
    if (getRefFrame() && (getRefFrame()!= getRefBody()))
    {
      fprintf(out, " refFrame=\"%s\"", getRefFrame()->name);
    }
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::Task::toXMLEnd(FILE* out, bool activation) const
{
  fprintf(out, " active=\"%s\" />\n", activation ? "true" : "false");
}
