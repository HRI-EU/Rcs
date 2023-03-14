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

#include "IkSolverProjectedQP.h"
#include "TaskDistance.h"
#include "TaskJoint.h"
#include "TaskJoints.h"
#include "Rcs_typedef.h"
#include "Rcs_macros.h"
#include "Rcs_math.h"



/*******************************************************************************
 *
 ******************************************************************************/
Rcs::IkSolverProjectedQP::IkSolverProjectedQP(Rcs::ControllerBase* ctrl) :
  IkSolverRMR(ctrl), originalTasks(controller->getTasks())
{
  // Create active set constraints for collisions
  if (controller->getCollisionMdl())
  {
    for (unsigned int i=0; i<controller->getCollisionMdl()->nPairs; ++i)
    {
      const RcsPair* PAIR = &controller->getCollisionMdl()->pair[i];
      const RcsBody* b1 = RCSBODY_BY_ID(controller->getGraph(), PAIR->b1);
      const RcsBody* b2 = RCSBODY_BY_ID(controller->getGraph(), PAIR->b2);
      RCHECK(b1 && b2);
      RLOG(1, "Adding distance constraint between \"%s\" and \"%s\"",
           b1->name, b2->name);
      Task* ti = new TaskDistance(controller->getGraph(), b1, b2);
      activeSet.push_back(ti);
    }
  }

  // Append active set constraints for joint limits
  RCSGRAPH_TRAVERSE_JOINTS(controller->getGraph())
  {
    if (JNT->constrained || (JNT->coupledToId!=-1))
    {
      continue;
    }

    bool constraintAlreadyExists = false;
    for (size_t i = 0; i < activeSet.size(); ++i)
    {
      const TaskJoint* tj = dynamic_cast<TaskJoint*>(activeSet[i]);
      if (tj && STREQ(tj->getJoint()->name, JNT->name))
      {
        constraintAlreadyExists = true;
      }

      TaskJoints* tjn = dynamic_cast<TaskJoints*>(activeSet[i]);
      if (tjn)
      {
        for (size_t j = 0; j < tjn->getDim(); ++j)
        {
          tj = dynamic_cast<const TaskJoint*>(tjn->getSubTask(j));
          if (tj && STREQ(tj->getJoint()->name, JNT->name))
          {
            constraintAlreadyExists = true;
          }
        }
      }
    }

    if (constraintAlreadyExists == false)
    {
      RLOG(2, "Adding joint constraint for joint \"%s\"", JNT->name);
      //activeSet.push_back(new TaskJoint(controller->getGraph(), JNT));
    }
  }
  RLOG_CPP(1, "Added " << activeSet.size() << " constraints");
}

/*******************************************************************************
 * Only delete the tasks that have been created in this class. The first ones
 * are pointers belonging to the ControllerBase.
 ******************************************************************************/
Rcs::IkSolverProjectedQP::~IkSolverProjectedQP()
{
  for (size_t i=0; i< activeSet.size(); ++i)
  {
    delete activeSet[i];
  }
}

/*******************************************************************************
 * Active set constraint solver. This is a rapid version of it that neglects
 * the interplay between the constraints. Formally, when adding a new
 * constraint equation, this might break other constraints. This function
 * does not consider this and only adds constraints for instantaneously
 * violated constraints. Why do we do this? Because it's a lot more efficient.
 *
 * For active constraints, we apply a pseudo Baumgarte stabilization. In
 * order to avoid jitter, violated limits get a constraint velocity of zero.
 * But if the violation went deep into the dark zone, we might get a "sticky"
 * constraint that does not easily escape. We therefore define a boundary
 * region (see variable "baumgrateLimit" a tiny bit below the constraint
 * violation to which we will slowly move for all active constraints. This
 * value has been chosen so that an "escape" from the active constraint can
 * easily happen with the first IK call.
 ******************************************************************************/
double Rcs::IkSolverProjectedQP::solveRightInverse(MatNd* dq_des,
                                                   const MatNd* dx,
                                                   const MatNd* dH,
                                                   const MatNd* activation,
                                                   double lambda0)
{
  double det = IkSolverRMR::solveRightInverse(dq_des, dx, dH, activation, lambda0);

  if ((controller->getNumberOfTasks()==0) || det==0.0)
  {
    return det;
  }

  MatNd* q_org = MatNd_clone(controller->getGraph()->q);

  unsigned int nViolations = 0;
  MatNd_addSelf(controller->getGraph()->q, dq_des);
  RcsGraph_setState(controller->getGraph(), NULL, NULL);
  controller->swapTaskVec(activeSet, true);

  RLOG(1, "Now: %zu tasks with dim=%zu", controller->getNumberOfTasks(), controller->getTaskDim());

  MatNd* a_c = MatNd_create(controller->getNumberOfTasks(), 1);
  MatNd* dx_c = MatNd_create(controller->getTaskDim(), 1);

  for (size_t i = 0; i < controller->getNumberOfTasks(); ++i)
  {

    if (TaskJoint* jntTask = dynamic_cast<TaskJoint*>(controller->getTask(i)))
    {
      RLOG(1, "Checking joint task %s", jntTask->getName().c_str());
      const int jIdx = jntTask->getJointIndex();
      const double x = controller->getGraph()->q->ele[jIdx] + dq_des->ele[jIdx];
      const double q_min = jntTask->getJoint()->q_min + RCS_DEG2RAD(2.0);
      const double q_max = jntTask->getJoint()->q_max - RCS_DEG2RAD(2.0);

      if (x < q_min)
      {
        const int xIdx = controller->getTaskArrayIndex(i);
        const double baumgarteLimit = RCS_DEG2RAD(0.1);
        a_c->ele[i] = 1.0;
        dx_c->ele[xIdx] = (q_min-x > baumgarteLimit) ? 0.1*baumgarteLimit : 0.0;
        nViolations++;
        RLOG(5, "Applying upper limit constraint for joint %s at violation "
             "depth %f deg", jntTask->getJoint()->name, RCS_RAD2DEG(x-q_min));
      }
      else if (x > q_max)
      {
        const int xIdx = controller->getTaskArrayIndex(i);
        const double baumgarteLimit = RCS_DEG2RAD(0.1);
        a_c->ele[i] = 1.0;
        dx_c->ele[xIdx] = (x-q_max > baumgarteLimit) ? -0.1*baumgarteLimit : 0.0;
        nViolations++;
        RLOG(5, "Applying lower limit constraint for joint %s at violation "
             "depth %f deg", jntTask->getJoint()->name, RCS_RAD2DEG(q_max-x));
      }
    }
    else if (TaskDistance* distTsk = dynamic_cast<TaskDistance*>(controller->getTask(i)))
    {
      double x, eps = 0.01;
      distTsk->computeX(&x);
      double penetration = eps-x;
      RLOG(1, "Checking distance task %s: penetration is %f", distTsk->getName().c_str(), penetration);

      if (penetration > 0.0)
      {
        const int xIdx = controller->getTaskArrayIndex(i);
        const double baumgarteLimit = 0.0001;
        a_c->ele[i] = 1.0;
        //dx->ele[xIdx] = (penetration > baumgarteLimit) ? 0.1*baumgarteLimit : 0.0;
        dx_c->ele[xIdx] = (penetration > baumgarteLimit) ? 0.9*(penetration-baumgarteLimit)+0.1*baumgarteLimit : 0.0;
        nViolations++;
        RLOG(1, "Applying distance constraint %s - %s at violation depth %f mm: dx=%f",
             distTsk->getEffector()->name, distTsk->getRefBody()->name,
             1000.0*(x-eps), dx_c->ele[xIdx]);
      }
    }   // "Distance"
  }   // for (... activeSet.size())


  RLOG(1, "Found %d violations", nViolations);
  if (nViolations>0)
  {
    RLOG(0, "Found %d violations", nViolations);
  }

  if (nViolations>0)
  {
    controller->printX(dx_c, a_c);
    // Project unconstrained solution into null space
    MatNd* dH_c = MatNd_clone(dq_des);
    RcsGraph_stateVectorToIKSelf(controller->getGraph(), dH_c);
    MatNd_transposeSelf(dH_c);
    MatNd_scaleSelfToScalar(dH_c, 0.001);

    IkSolverRMR::solveRightInverse(dq_des, dx_c, dH_c, a_c, lambda0);

    REXEC(1)   // Debug output only
    {
      int nConstraints = 0;
      for (size_t i = 0; i < a_c->m; ++i)
      {
        if (a_c->ele[i] > 0.0)
        {
          nConstraints++;
        }
      }

      RLOG(1, "Active set: %d violations, %d constraints",
           nViolations, nConstraints);

    }   // REXEC(5)

    MatNd_destroy(dH_c);
  }

  MatNd_destroy(dx_c);
  MatNd_destroy(a_c);
  RcsGraph_setState(controller->getGraph(), q_org, NULL);

  controller->swapTaskVec(activeSet, true);

  return det;
}
