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

#include "IkSolverPrioRMR.h"
#include "TaskFactory.h"
#include "Rcs_typedef.h"
#include "Rcs_macros.h"
#include "Rcs_math.h"
#include "Rcs_parser.h"

#include <cfloat>



/*******************************************************************************
 * \brief Constructor based on xml parsing.
 ******************************************************************************/
Rcs::IkSolverPrioRMR::IkSolverPrioRMR(const Rcs::ControllerBase* controller_) :
  controller(controller_), nx(controller->getTaskDim()), nx1(0), nx2(0),
  nTasks(controller_->getNumberOfTasks()), nTasks1(0), nTasks2(0),
  nq(controller_->getGraph()->nJ), nqr(0),
  dx1(NULL), dx2(NULL), activation1(NULL), activation2(NULL),
  A(NULL), invA(NULL), invWq(NULL), J1(NULL), J2(NULL), pinvJ1(NULL),
  N1(NULL), J2N1(NULL), pinvJ2N1(NULL), J2pinvJ1(NULL), J2pinvJ1dx(NULL),
  dx2_mod(NULL), N2(NULL), dHA(NULL)
{
  this->dx1 = MatNd_create(this->nx, 1);
  this->dx2 = MatNd_create(this->nx, 1);
  this->activation1 = MatNd_create(this->nTasks, 1);
  this->activation2 = MatNd_create(this->nTasks, 1);
  this->priorityLevel.resize(this->nTasks, 0);

  // Coupled joint matrix
  this->A = MatNd_create(this->nq, this->nq);
  this->invA = MatNd_create(this->nq, this->nq);
  RcsGraph_coupledJointMatrix(controller_->getGraph(), this->A, this->invA);
  this->nqr = this->A->n;

  // Joint space weghting matrix
  this->invWq = MatNd_create(this->nq, 1);
  RcsGraph_getInvWq(controller_->getGraph(), this->invWq, RcsStateIK);
  MatNd_preMulSelf(this->invWq, this->invA);

  // First priority Jacobian
  this->J1 = MatNd_create(this->nx, this->nq);

  // Second priority Jacobian
  this->J2 = MatNd_create(this->nx, this->nq);

  // First priority pseudo-inverse
  this->pinvJ1 = MatNd_create(this->nq, this->nx);

  // First priority null space
  this->N1 = MatNd_create(this->nq, this->nq);

  // J2*N1
  this->J2N1 = MatNd_create(this->nx, this->nq);

  // (J2*N1)#
  this->pinvJ2N1 = MatNd_create(this->nq, this->nx);

  // J2*J1#
  this->J2pinvJ1 = MatNd_create(this->nx, this->nx);

  // (J2*J1#) dx1
  this->J2pinvJ1dx = MatNd_create(this->nx, 1);

  // dx2 - J2 J1# dx1
  this->dx2_mod = MatNd_create(this->nx, 1);

  // N2 = pinvJ2N1*J2N1
  this->N2 = MatNd_create(this->nq, this->nq);

  // dHA = dH*A
  this->dHA = MatNd_create(this->nq, 1);

  // dq_2c = [J2 N1]# J2 J1# dx1
  this->dq_2c = MatNd_create(this->nq, 1);

  // Initialize priority levels from xml file
  xmlDocPtr xmlDoc;
  xmlNodePtr node = parseXMLFile(controller->getXmlFileName().c_str(),
                                 "Controller", &xmlDoc);

  if (node == NULL)
  {
    RLOG(1, "Failed to parse xml file \"%s\"",
         controller->getXmlFileName().c_str());
    return;
  }

  node = node->children;
  unsigned int taskCount = 0;

  while (node)
  {
    if (TaskFactory::isValid(node, controller->getGraph()) == true)
    {
      int prioLevel = 0;

      if (getXMLNodePropertyInt(node, "prio", &prioLevel) ||
          getXMLNodePropertyInt(node, "priority", &prioLevel))
      {
        this->priorityLevel[taskCount] = prioLevel;
        RLOG(5, "Setting task %d to level %d", taskCount, prioLevel);
      }

      taskCount++;
    }

    node = node->next;
  }

  xmlFreeDoc(xmlDoc);
}


/*******************************************************************************
 * \brief Destructor.
 ******************************************************************************/
Rcs::IkSolverPrioRMR::~IkSolverPrioRMR()
{
  MatNd_destroy(this->dx1);
  MatNd_destroy(this->dx2);
  MatNd_destroy(this->activation1);
  MatNd_destroy(this->activation2);
  MatNd_destroy(this->A);
  MatNd_destroy(this->invA);
  MatNd_destroy(this->invWq);
  MatNd_destroy(this->J1);
  MatNd_destroy(this->J2);
  MatNd_destroy(this->pinvJ1);
  MatNd_destroy(this->N1);
  MatNd_destroy(this->J2N1);
  MatNd_destroy(this->pinvJ2N1);
  MatNd_destroy(this->J2pinvJ1);
  MatNd_destroy(this->J2pinvJ1dx);
  MatNd_destroy(this->dx2_mod);
  MatNd_destroy(this->N2);
  MatNd_destroy(this->dHA);
  MatNd_destroy(this->dq_2c);
}


/*******************************************************************************
 * \brief Calculate the inverse kinematics.
 ******************************************************************************/
bool Rcs::IkSolverPrioRMR::solve(MatNd* dq1,
                                 MatNd* dq2,
                                 MatNd* dq_ns,
                                 const MatNd* dx,
                                 const MatNd* dH,
                                 const MatNd* activation,
                                 double lambda0)
{
  bool success = true;
  this->nx = controller->getTaskDim();
  this->nTasks = controller->getNumberOfTasks();

  RCHECK_MSG(dx->m==this->nx, "dx->m=%d nx=%d", dx->m, this->nx);

  if (activation!=NULL)
  {
    RCHECK_MSG(activation->m==this->nTasks, "activation->m=%d nTasks=%d",
               activation->m, this->nTasks);
  }

  // Determine task-space dimensions of first and second priority tasks, and
  // Compute dx1 and dx2 accordingly
  MatNd_reshape(this->dx1, 0, 1);
  MatNd_reshape(this->dx2, 0, 1);
  this->nTasks1 = 0;
  this->nTasks2 = 0;
  this->nx1 = 0;
  this->nx2 = 0;

  MatNd_setZero(this->activation1);
  MatNd_setZero(this->activation2);

  for (unsigned int i=0; i<this->nTasks; i++)
  {
    const Rcs::Task* ti   = controller->getTask(i);
    unsigned int dim_i    = ti->getDim();
    unsigned int arrayIdx = controller->getTaskArrayIndex(i);
    double act_i          = activation ? MatNd_get2(activation, i, 0) : 1.0;

    if ((priorityLevel[i]==0) && (act_i>0.0))
    {
      this->nTasks1++;
      VecNd_copy(&dx1->ele[this->nx1], &dx->ele[arrayIdx], dim_i);
      this->nx1 += dim_i;
      MatNd_set2(this->activation1, i, 0, 1.0);
    }
    else if ((priorityLevel[i]==1) && (act_i>0.0))
    {
      this->nTasks2++;
      VecNd_copy(&dx2->ele[this->nx2], &dx->ele[arrayIdx], dim_i);
      this->nx2 += dim_i;
      MatNd_set2(this->activation2, i, 0, 1.0);
    }
  }

  dx1->m = this->nx1;
  dx2->m = this->nx2;

  // Regularization value
  MatNd lambda = MatNd_fromPtr(1, 1, &lambda0);

  // Compute first priority Jacobian
  MatNd_reshape(this->J1, this->nx1, this->nq);
  controller->computeJ(this->J1, activation1);

  // Compute second priority Jacobian
  MatNd_reshape(this->J2, this->nx2,nq);
  controller->computeJ(this->J2, activation2);

  // Consider joint coupling (if any)
  if (this->nq != this->nqr)
  {
    MatNd_postMulSelf(this->J1, A);
    MatNd_postMulSelf(this->J2, A);
  }

  // Compute first priority pseudo-inverse
  MatNd_reshape(this->pinvJ1, this->J1->n, this->J1->m);
  double det1 = MatNd_rwPinv(pinvJ1, J1, invWq, &lambda);

  if (det1<=0.0)
  {
    success = false;
  }

  // Compute first priority dq1 and convert to inverse kinematics dimension
  MatNd_reshape(dq1, this->nqr, 1);

  if (this->pinvJ1->n>0)
  {
    MatNd_mul(dq1, this->pinvJ1, dx1);
  }
  else
  {
    MatNd_setZero(dq1);
  }

  if (this->nq != this->nqr)
  {
    MatNd_preMulSelf(dq1, this->A);
  }

  // Compute first priority null space
  MatNd_reshape(this->N1, this->nqr, this->nqr);
  MatNd_nullspace(this->N1, this->pinvJ1, this->J1);

  // Compute J2 N1
  MatNd_reshape(this->J2N1, this->J2->m, this->N1->n);
  MatNd_mul(this->J2N1, this->J2, this->N1);

  // Compute (J2 N1)#
  MatNd_reshape(this->pinvJ2N1, this->J2N1->n, this->J2N1->m);


#if 1
  double det2 = MatNd_rwPinv(this->pinvJ2N1, this->J2N1, this->invWq,
                             &lambda);
#else
  MatNd* invWq2 = MatNd_create(this->nq, 1);
  MatNd_setElementsTo(invWq2, 1.0);
  MatNd_preMulSelf(invWq2, this->invA);
  double det2 = MatNd_rwPinv(this->pinvJ2N1, this->J2N1, invWq2, &lambda);
  MatNd_destroy(invWq2);
#endif

  if (det2<=0.0)
  {
    success = false;
  }

  // Compute J2 J1#
  MatNd_reshape(this->J2pinvJ1, this->J2->m, this->pinvJ1->n);
  MatNd_mul(this->J2pinvJ1, this->J2, this->pinvJ1);

  // Compute J2 J1# dx1
  MatNd_reshape(this->J2pinvJ1dx, this->J2->m, this->dx1->n);
  MatNd_mul(this->J2pinvJ1dx, this->J2pinvJ1, this->dx1);

  // Compute dq_2c = [J2 N1]# J2 J1# dx1
  MatNd_reshape(this->dq_2c, this->nqr, 1);
  MatNd_mul(this->dq_2c, this->pinvJ2N1, this->J2pinvJ1dx);

  // Compute dx2 - J2 J1# dx1
  MatNd_reshape(this->dx2_mod, this->dx2->m, this->dx2->n);
  MatNd_sub(this->dx2_mod, this->dx2, this->J2pinvJ1dx);

  // Compute second priority dq
  MatNd_reshape(dq2, this->nqr, 1);
  MatNd_mul(dq2, this->pinvJ2N1, this->dx2_mod);
  if (this->nq != this->nqr)
  {
    MatNd_preMulSelf(dq2, A);
  }

  // Third priority (null space)
  if ((dH != NULL) && (dq_ns != NULL))
  {
    RCHECK((dH->m==nq && dH->n==1) || (dH->m==1 && dH->n==nq));

    MatNd_reshape(this->N2, this->nqr, this->nqr);
    MatNd_nullspace(this->N2, this->pinvJ2N1, this->J2N1);
    MatNd_preMulSelf(this->N2, this->N1);
    MatNd_reshape(this->dHA, this->nqr, 1);
    MatNd_reshape(dq_ns, this->nqr, 1);

    MatNd gradH = MatNd_fromPtr(this->nq, 1, dH->ele);
    if (this->nq == this->nqr)
    {
      MatNd_copy(this->dHA, &gradH);
    }
    else
    {
      MatNd_mul(this->dHA, this->invA, &gradH);
    }

    MatNd_eleMulSelf(this->dHA, this->invWq);
    MatNd_mul(dq_ns, this->N2, this->dHA);
    MatNd_constMulSelf(dq_ns, -1.0);
    if (this->nq != this->nqr)
    {
      MatNd_preMulSelf(dq_ns, this->A);
    }
  }

  // Uncompress to all states
  if (success == true)
  {
    RcsGraph_stateVectorFromIKSelf(controller->getGraph(), dq1);
    RcsGraph_stateVectorFromIKSelf(controller->getGraph(), dq2);
    if (dq_ns != NULL)
    {
      RcsGraph_stateVectorFromIKSelf(controller->getGraph(), dq_ns);
    }
  }
  else
  {
    MatNd_reshapeAndSetZero(dq1, controller->getGraph()->dof, 1);
    MatNd_reshapeAndSetZero(dq2, controller->getGraph()->dof, 1);
    if (dq_ns != NULL)
    {
      MatNd_reshapeAndSetZero(dq_ns, controller->getGraph()->dof, 1);
    }
  }


  return success;
}


/*******************************************************************************
 * \brief Convenience function for solving the inverse kinematics
 ******************************************************************************/
bool Rcs::IkSolverPrioRMR::solve(MatNd* dq1,
                                 MatNd* dq2,
                                 MatNd* dq_ns,
                                 const MatNd* dx,
                                 const MatNd* dH)
{
  return solve(dq1, dq2, dq_ns, dx, dH, NULL, 0.0);
}


/*******************************************************************************
 * \brief Convenience function for solving the inverse kinematics
 ******************************************************************************/
bool Rcs::IkSolverPrioRMR::solve(MatNd* dq,
                                 const MatNd* dx,
                                 const MatNd* dH,
                                 const MatNd* activation,
                                 double lambda0)
{
  MatNd* dq1   = MatNd_create(controller->getGraph()->dof, 1);
  MatNd* dq2   = MatNd_create(controller->getGraph()->dof, 1);
  MatNd* dq_ns = MatNd_create(controller->getGraph()->dof, 1);

  bool success = solve(dq1, dq2, dq_ns, dx, dH, activation, lambda0);

  MatNd_reshape(dq, controller->getGraph()->dof, 1);
  MatNd_add(dq, dq1, dq2);
  MatNd_addSelf(dq, dq_ns);

  MatNd_destroy(dq1);
  MatNd_destroy(dq2);
  MatNd_destroy(dq_ns);

  return success;
}


/*******************************************************************************
 * \brief Convenience function for solving the inverse kinematics
 ******************************************************************************/
bool Rcs::IkSolverPrioRMR::solve(MatNd* dq,
                                 const MatNd* dx,
                                 const MatNd* dH)
{
  MatNd* dq1   = MatNd_create(controller->getGraph()->dof, 1);
  MatNd* dq2   = MatNd_create(controller->getGraph()->dof, 1);
  MatNd* dq_ns = MatNd_create(controller->getGraph()->dof, 1);

  bool success = solve(dq1, dq2, dq_ns, dx, dH, NULL, 0.0);

  MatNd_reshape(dq, controller->getGraph()->dof, 1);
  MatNd_add(dq, dq1, dq2);
  MatNd_addSelf(dq, dq_ns);

  MatNd_destroy(dq1);
  MatNd_destroy(dq2);
  MatNd_destroy(dq_ns);

  return success;
}


/*******************************************************************************
 * \brief Convenience function for getting task dimensions
 ******************************************************************************/
size_t Rcs::IkSolverPrioRMR::getTaskDimForPriority(int prioLevel) const
{
  size_t dim = 0;

  for (size_t i=0; i<priorityLevel.size(); ++i)
  {
    if (priorityLevel[i] == prioLevel)
    {
      dim += controller->getTaskDim(i);
    }
  }


  return dim;
}


/*******************************************************************************
 * \brief Convenience function for getting task dimensions
 ******************************************************************************/
size_t Rcs::IkSolverPrioRMR::getTaskDimForPriority(int prioLevel,
                                                   const MatNd* activation) const
{
  size_t dim = 0;

  for (size_t i=0; i<priorityLevel.size(); ++i)
  {
    if ((priorityLevel[i]==prioLevel) && (MatNd_get(activation, i, 0)>0.0))
    {
      dim += controller->getTaskDim(i);
    }
  }


  return dim;
}


/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::IkSolverPrioRMR::setTaskPriority(size_t taskIdx, int prioLevel)
{
  priorityLevel[taskIdx] = prioLevel;
}


/*******************************************************************************
 *
 ******************************************************************************/
int Rcs::IkSolverPrioRMR::getTaskPriority(size_t taskIdx) const
{
  return priorityLevel[taskIdx];
}


/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::IkSolverPrioRMR::print() const
{
  RMSG("IkSolverPrioRMR::print()");
  for (size_t i=0; i<priorityLevel.size(); ++i)
  {
    std::cout << "Task " << controller->getTaskName(i) << " (" << i
              << "): Priority is " << priorityLevel[i] << std::endl;
  }

  std::cout << "nx: " << nx << std::endl;
  std::cout << "nx1: " << nx1 << std::endl;
  std::cout << "nx2: " << nx2 << std::endl;
}
