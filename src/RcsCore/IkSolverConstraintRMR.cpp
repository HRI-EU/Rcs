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

#include "IkSolverConstraintRMR.h"
#include "TaskDistance.h"
#include "TaskJoint.h"
#include "TaskJoints.h"
#include "Rcs_typedef.h"
#include "Rcs_macros.h"
#include "Rcs_math.h"

#include <cfloat>



/*******************************************************************************
 *
 ******************************************************************************/
Rcs::IkSolverConstraintRMR::IkSolverConstraintRMR(Rcs::ControllerBase* ctrl) :
  IkSolverRMR(ctrl)
{
  this->activeSet = controller->getTasks();

  // Active set constraints for collisions
  if (controller->getCollisionMdl())
  {
    RCSPAIR_TRAVERSE(controller->getCollisionMdl()->pair)
    {
      RLOG(5, "Adding distance constraint between \"%s\" and \"%s\"",
           PAIR->b1->name, PAIR->b2->name);
      Task* ti = new TaskDistance(controller->getGraph(), PAIR->b1, PAIR->b2);
      RCHECK(ti->getDim() == 1);
      controller->add(ti);
    }
  }

  // Active set constraints for joint limits
  RCSGRAPH_TRAVERSE_JOINTS(controller->getGraph())
  {
    if (JNT->constrained || JNT->coupledTo)
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
      Task* ti = new TaskJoint(controller->getGraph(), JNT);
      RCHECK(ti->getDim() == 1);
      controller->add(ti);
    }
  }
  RLOG_CPP(5, "Added " << activeSet.size() << " constraints");

  // Update with added active set constraints
  controller->swapTaskVec(activeSet);

  RLOG(5, "activeSet: %zu   tasks: %zu",
       activeSet.size(), controller->getNumberOfTasks());

  MatNd_realloc(this->J, activeSet.size(), this->nq);
  MatNd_realloc(this->pinvJ, this->nq, activeSet.size());
  MatNd_realloc(this->dxr, activeSet.size(), 1);
  MatNd_realloc(this->Wx, activeSet.size(), 1);
  MatNd_realloc(this->dx, activeSet.size(), 1);
}

/*******************************************************************************
 * Only delete the tasks that have been created in this class. The first ones
 * are pointers belonging to the ControllerBase.
 ******************************************************************************/
Rcs::IkSolverConstraintRMR::~IkSolverConstraintRMR()
{
  for (size_t i=controller->getNumberOfTasks(); i<activeSet.size(); ++i)
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
void Rcs::IkSolverConstraintRMR::solveRightInverse(MatNd* dq_des,
                                                   const MatNd* dx_,
                                                   const MatNd* dH,
                                                   const MatNd* activation_,
                                                   double lambda0)
{
  controller->swapTaskVec(activeSet);
  this->nx = controller->getTaskDim();

  MatNd* dx = MatNd_clone(dx_);
  MatNd* activation = MatNd_clone(activation_);
  MatNd_realloc(dx, controller->getTaskDim(), 1);
  MatNd_realloc(activation, controller->getNumberOfTasks(), 1);

  IkSolverRMR::solveRightInverse(dq_des, dx, dH, activation, lambda0);

  if (activeSet.empty() || getDeterminant()==0.0)
  {
    controller->swapTaskVec(activeSet);
    this->nx = controller->getTaskDim();
    MatNd_destroy(dx);
    MatNd_destroy(activation);
    return;
  }

  unsigned int nViolations = 0;

  for (size_t i = activeSet.size(); i < controller->getNumberOfTasks(); ++i)
  {

    if (TaskJoint* jntTask = dynamic_cast<TaskJoint*>(controller->getTask(i)))
    {
      const int jIdx = jntTask->getJoint()->jointIndex;
      const double x = controller->getGraph()->q->ele[jIdx] + dq_des->ele[jIdx];
      const double q_min = jntTask->getJoint()->q_min + RCS_DEG2RAD(2.0);
      const double q_max = jntTask->getJoint()->q_max - RCS_DEG2RAD(2.0);

      if (x < q_min)
      {
        const int xIdx = controller->getTaskArrayIndex(i);
        const double baumgarteLimit = RCS_DEG2RAD(0.1);
        activation->ele[i] = 1.0;
        dx->ele[xIdx] = (q_min-x > baumgarteLimit) ? 0.1*baumgarteLimit : 0.0;
        nViolations++;
        RLOG(5, "Applying upper limit constraint for joint %s at violation "
             "depth %f deg", jntTask->getJoint()->name, RCS_RAD2DEG(x-q_min));
      }
      else if (x > q_max)
      {
        const int xIdx = controller->getTaskArrayIndex(i);
        const double baumgarteLimit = RCS_DEG2RAD(0.1);
        activation->ele[i] = 1.0;
        dx->ele[xIdx] = (x-q_max > baumgarteLimit) ? -0.1*baumgarteLimit : 0.0;
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

      if (penetration > 0.0)
      {
        const int xIdx = controller->getTaskArrayIndex(i);
        const double baumgarteLimit = 0.0001;
        activation->ele[i] = 1.0;
        //dx->ele[xIdx] = (penetration > baumgarteLimit) ? 0.1*baumgarteLimit : 0.0;
        dx->ele[xIdx] = (penetration > baumgarteLimit) ? 0.9*(penetration-baumgarteLimit)+0.1*baumgarteLimit : 0.0;
        nViolations++;
        RLOG(5, "Applying distance constraint %s - %s at violation depth %f mm",
             distTsk->getEffector()->name, distTsk->getRefBody()->name,
             1000.0*(x-eps));
      }
    }   // "Distance"
  }   // for (... activeSet.size())




  if (nViolations>0)
  {
    IkSolverRMR::solveRightInverse(dq_des, dx, dH, activation, lambda0);

    REXEC(5)   // Debug output only
    {
      int nConstraints = 0;
      for (size_t i = activeSet.size(); i < activation->m; ++i)
      {
        if (activation->ele[i] > 0.0)
        {
          nConstraints++;
        }
      }

      RLOG(1, "Active set: %d violations, %d constraints",
           nViolations, nConstraints);
      VecNd_setZero(activation->ele, activeSet.size());
      std::vector<Rcs::Task*> as = controller->getTasks(activation);
      for (size_t i=0; i<as.size(); ++i)
      {
        RLOG(2, "%zu: %s", i, as[i]->getName().c_str());
      }
    }   // REXEC(5)

  }

  MatNd_destroy(dx);
  MatNd_destroy(activation);

  controller->swapTaskVec(activeSet);
  this->nx = controller->getTaskDim();
}
