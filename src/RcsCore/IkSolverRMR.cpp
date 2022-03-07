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

#include "IkSolverRMR.h"
#include "Rcs_typedef.h"
#include "Rcs_macros.h"
#include "Rcs_math.h"
#include "Rcs_parser.h"
#include "TaskRegion.h"
#include "TaskSpaceBlender.h"

namespace Rcs
{

/*******************************************************************************
 *
 ******************************************************************************/
IkSolverRMR::IkSolverRMR(ControllerBase* controller_) :
  controller(controller_),
  A(NULL), invA(NULL), invWq(NULL), J(NULL), pinvJ(NULL), N(NULL), dHA(NULL),
  dq(NULL), dqr(NULL), dxr(NULL), NinvW(NULL), Wx(NULL), dHr(NULL), dH(NULL),
  dx_proj(NULL), ax_curr(NULL), blendingMode(0), hasCoupledJoints(true)
{
  const unsigned int nq = controller->getGraph()->nJ;
  const unsigned int dof = controller->getGraph()->dof;
  const size_t nx = controller->getTaskDim();

  // Coupled joint matrix
  this->A = MatNd_create(dof, dof);
  this->invA = MatNd_create(dof, dof);
  const int nCpldJnts = RcsGraph_countCoupledJoints(controller->getGraph());
  this->hasCoupledJoints = (nCpldJnts > 0);
  RcsGraph_coupledJointMatrix(controller->getGraph(), this->A, this->invA);


  // Joint space weghting matrix
  this->invWq = MatNd_create(dof, 1);
  RcsGraph_getInvWq(controller->getGraph(), this->invWq, RcsStateIK);
  MatNd_preMulSelf(this->invWq, this->invA);

  // Jacobian
  this->J = MatNd_create(nx, dof);
  MatNd_reshape(this->J, nx, nq);

  // Pseudo-inverse
  this->pinvJ = MatNd_create(dof, nx);
  MatNd_reshape(this->pinvJ, nq, nx);

  // Null space
  this->N = MatNd_create(dof, dof);
  MatNd_reshape(this->N, nq, nq);

  // dHA = dH*A
  this->dHA = MatNd_create(dof, 1);
  MatNd_reshape(this->dHA, nq, 1);

  // Configuration space vector
  this->dq = MatNd_create(dof, 1);

  // Configuration space vector with only non-coupled dof
  this->dqr = MatNd_create(dof, 1);
  MatNd_reshape(this->dqr, nq, 1);

  // Task space vector with only non-coupled dof
  this->dxr = MatNd_create(nx, 1);

  // N*invWq
  this->NinvW = MatNd_create(dof, dof);
  MatNd_reshape(this->NinvW, nq, nq);

  // Task space weighting matrix
  this->Wx = MatNd_create(nx, 1);

  // Null space potential (in reduced coordinates)
  this->dHr = MatNd_create(dof, 1);
  MatNd_reshape(this->dHr, nq, 1);

  // Null space potential (in IK-relevant coordinates)
  this->dH = MatNd_create(1, dof);
  MatNd_reshape(this->dH, 1, nq);

  // Null space -> task space re-projection
  this->dx_proj = MatNd_create(nx, 1);

  // True task activation ax_curr = J*J#
  this->ax_curr = MatNd_create(nx, 1);
}

/*******************************************************************************
 *
 ******************************************************************************/
IkSolverRMR::~IkSolverRMR()
{
  MatNd_destroy(this->A);
  MatNd_destroy(this->invA);
  MatNd_destroy(this->invWq);
  MatNd_destroy(this->J);
  MatNd_destroy(this->pinvJ);
  MatNd_destroy(this->N);
  MatNd_destroy(this->dHA);
  MatNd_destroy(this->dq);
  MatNd_destroy(this->dqr);
  MatNd_destroy(this->dxr);
  MatNd_destroy(this->NinvW);
  MatNd_destroy(this->Wx);
  MatNd_destroy(this->dHr);
  MatNd_destroy(this->dH);
  MatNd_destroy(this->dx_proj);
  MatNd_destroy(this->ax_curr);
}

/*******************************************************************************
 * Returns the pointer to the internal controller
 ******************************************************************************/
ControllerBase* IkSolverRMR::getController() const
{
  return this->controller;
}

/*******************************************************************************
 * Calculate the inverse kinematics, left-hand (nq x nq) inverse.
 ******************************************************************************/
double IkSolverRMR::solveLeftInverse(MatNd* dq_des,
                                     const MatNd* dx,
                                     const MatNd* dH,
                                     const MatNd* activation,
                                     double lambda0,
                                     bool recomputeKinematics)
{
  MatNd_reshape(dq_des, controller->getGraph()->dof, 1);
  MatNd* dq_ts = MatNd_createLike(dq_des);
  MatNd* dq_ns = MatNd_createLike(dq_des);
  double det = solveLeftInverse(dq_ts, dq_ns, dx, dH, activation, lambda0,
                                recomputeKinematics);
  MatNd_add(dq_des, dq_ts, dq_ns);
  MatNd_destroyN(2, dq_ts, dq_ns);

  return det;
}


/*******************************************************************************
 * Calculate the inverse kinematics, left-hand (nq x nq) inverse.
 ******************************************************************************/
double IkSolverRMR::solveLeftInverse(MatNd* dq_ts,
                                     MatNd* dq_ns,
                                     const MatNd* dx,
                                     const MatNd* dH,
                                     const MatNd* activation,
                                     double lambda0,
                                     bool recomputeKinematics)
{
  // Compute the required terms for the inverse kinematics calculation
  if (recomputeKinematics==true)
  {
    // Compute the joint coupling matrices. This is required only for joints
    // that have a non-constant coupling (e.g. polynomial coupling as in some
    // parallel linkage joints).
    if (hasCoupledJoints)
    {
      RcsGraph_coupledJointMatrix(controller->getGraph(), this->A, this->invA);
    }

    // Update of the joint space wegihting matrix. This needs to be done if the
    // joint coupling matrices change, or the joint weights are dynamically
    // modified between successive calls.
    RcsGraph_getInvWq(controller->getGraph(), this->invWq, RcsStateIK);
    if (hasCoupledJoints)
    {
      MatNd_preMulSelf(this->invWq, this->invA);
    }

    // Compute the Jacobian
    controller->computeJ(this->J, activation);
    if (hasCoupledJoints)
    {
      MatNd_postMulSelf(this->J, this->A);  // J_r = J*A
    }
    MatNd_postMulDiagSelf(this->J, this->invWq); // J_r = J*A*invWq_r

    // Compute the blending matrix
    TaskSpaceBlender b(controller);
    b.setBlendingMode((TaskSpaceBlender::BlendingMode)blendingMode);
    b.compute(this->Wx, this->ax_curr, activation, this->J, lambda0);
  }

  // Compute the left weighted regularized pseudo-inverse Jacobian
  MatNd lambda = MatNd_fromPtr(1, 1, &lambda0);
  double det = MatNd_rwPinv2(this->pinvJ, J, Wx, &lambda);

  // Do nothing if the projection is singular
  if (det == 0.0)
  {
    RLOG(1, "Singular Jacobian - setting velocities to zero");
    MatNd_reshapeAndSetZero(dq_ts, controller->getGraph()->dof, 1);
    MatNd_reshapeAndSetZero(dq_ns, controller->getGraph()->dof, 1);
    return det;
  }

  // Reset the result and some internal arrays to match the problem indices
  const unsigned int nq = controller->getGraph()->nJ;
  const unsigned int nqr = this->A->n;
  MatNd_reshapeAndSetZero(this->dqr, nqr, 1);

  // Add task space component to the result
  if (dx != NULL)
  {
    if ((activation!=NULL) && (dx->m== controller->getTaskDim()))
    {
      controller->compressToActive(this->dxr, dx, activation);
      MatNd_mul(this->dqr, pinvJ, dxr);
    }
    else
    {
      MatNd_mul(this->dqr, pinvJ, dx);
    }

    // Project back into non-weighted space
    MatNd_eleMulSelf(this->dqr, invWq);

    // Expand from coupled joint space to constraint joints
    MatNd_reshape(dq_ts, nq, 1);

    if (hasCoupledJoints)
    {
      MatNd_mul(dq_ts, A, dqr);
    }
    else
    {
      MatNd_reshapeCopy(dq_ts, dqr);
    }

    // Uncompress to all states
    RcsGraph_stateVectorFromIKSelf(controller->getGraph(), dq_ts);
  }
  else
  {
    MatNd_reshapeAndSetZero(dq_ts, controller->getGraph()->dof, 1);
  }




  // Compute null space
  if (dH != NULL)
  {
    RCHECK_MSG((dH->m==nq && dH->n==1) || (dH->m==1 && dH->n==nq),
               "dH: %d x %d, should be 1 x %d", dH->m, dH->n, nq);
    MatNd_reshapeCopy(this->dHr, dH);
    MatNd_reshape(dHr, nq, 1);
    MatNd_constMulSelf(this->dHr, -1.0);
    if (hasCoupledJoints)
    {
      MatNd_preMulSelf(this->dHr, invA);
    }
    MatNd_reshape(this->NinvW, nqr, nqr);
    MatNd_nullspace(this->NinvW, pinvJ, J);
    MatNd_postMulDiagSelf(this->NinvW, invWq);   // apply joint metric
    MatNd_mul(this->dqr, NinvW, dHr);

    // Project back into non-weighted space
    MatNd_eleMulSelf(this->dqr, invWq);

    // Expand from coupled joint space to constraint joints
    MatNd_reshape(dq_ns, nq, 1);
    if (hasCoupledJoints)
    {
      MatNd_mul(dq_ns, A, dqr);
    }
    else
    {
      MatNd_reshapeCopy(dq_ns, dqr);
    }

    // Uncompress to all states
    RcsGraph_stateVectorFromIKSelf(controller->getGraph(), dq_ns);
  }
  else
  {
    MatNd_reshapeAndSetZero(dq_ns, controller->getGraph()->dof, 1);
  }

  return det;
}

/*******************************************************************************
 * Calculate the inverse kinematics using the right-hand pseudo-inverse. This
 * follows the derivation of Liegeois.
 ******************************************************************************/
double IkSolverRMR::solveRightInverse(MatNd* dq_des,
                                      const MatNd* dx,
                                      const MatNd* dH,
                                      const MatNd* activation,
                                      double lambda0)
{
  MatNd lambdaArr = MatNd_fromPtr(1, 1, &lambda0);
  return solveRightInverse(dq_des, dx, dH, activation, &lambdaArr);
}

double IkSolverRMR::solveRightInverse(MatNd* dq_ts,
                                      MatNd* dq_ns,
                                      const MatNd* dx,
                                      const MatNd* dH,
                                      const MatNd* activation,
                                      double lambda0)
{
  MatNd lambdaArr = MatNd_fromPtr(1, 1, &lambda0);
  return solveRightInverse(dq_ts, dq_ns, dx, dH, activation, &lambdaArr);
}

/*******************************************************************************
 * Calculate the inverse kinematics using the right-hand pseudo-inverse. This
 * follows the derivation of Liegeois.
 *
 * This version uses a full lambda vector instead of a single value
 ******************************************************************************/
double IkSolverRMR::solveRightInverse(MatNd* dq_des,
                                      const MatNd* dx,
                                      const MatNd* dH,
                                      const MatNd* activation,
                                      const MatNd* lambda)
{
  MatNd* dq_ns = MatNd_createLike(dq_des);
  double det = solveRightInverse(dq_des, dq_ns, dx, dH, activation, lambda);
  MatNd_addSelf(dq_des, dq_ns);
  MatNd_destroy(dq_ns);
  return det;
}

/*******************************************************************************
 * Implementation for all cases. We adjust arrays with task dimensions, but
 * assume that the size of the graph does not grow.
 ******************************************************************************/
double IkSolverRMR::solveRightInverse(MatNd* dq_ts,
                                      MatNd* dq_ns,
                                      const MatNd* dx,
                                      const MatNd* dH,
                                      const MatNd* activation,
                                      const MatNd* lambda)
{
  const RcsGraph* graph = controller->getGraph();

  // Adjust task space array sizes
  const size_t nx = controller->getTaskDim();
  const size_t nxa = controller->getActiveTaskDim(activation);

  MatNd_realloc(this->J, nxa, graph->nJ);
  MatNd_realloc(this->pinvJ, graph->nJ, nxa);
  MatNd_realloc(this->dx_proj, nxa, 1);
  MatNd_realloc(this->dxr, nxa, 1);

  // Create coupled joint matrix
  if (hasCoupledJoints)
  {
    RcsGraph_coupledJointMatrix(graph, this->A, this->invA);
  }

  // Compute the Jacobian and joint metric
  controller->computeJ(this->J, activation);
  RcsGraph_getInvWq(graph, this->invWq, RcsStateIK);

  // Task space
  if (hasCoupledJoints)
  {
    MatNd_postMulSelf(this->J, this->A);
    MatNd_preMulSelf(this->invWq, this->invA);
  }

  MatNd_reshape(this->pinvJ, this->J->n, this->J->m);
  double det = MatNd_rwPinv(this->pinvJ, this->J, this->invWq, lambda);

  // Do nothing if the projection is singular
  if (det == 0.0)
  {
    RLOG(1, "Singular Jacobian - setting velocities to zero");
    MatNd_reshapeAndSetZero(dq_ts, graph->dof, 1);
    MatNd_reshapeAndSetZero(dq_ns, graph->dof, 1);
    return det;
  }

  const unsigned int nq = controller->getGraph()->nJ;

  // Compute null space terms
  if (dH != NULL)
  {
    RCHECK((dH->m==nq && dH->n==1) || (dH->m==1 && dH->n==nq));
    MatNd_reshapeCopy(this->dHr, dH);
    MatNd_constMulSelf(this->dHr, -1.0);
    MatNd_reshape(this->dHr, nq, 1);
    if (hasCoupledJoints)
    {
      MatNd_preMulSelf(this->dHr, this->invA);
    }
    MatNd_reshape(this->NinvW, this->pinvJ->m, this->pinvJ->m);
    MatNd_nullspace(this->NinvW, this->pinvJ, this->J);
    MatNd_postMulDiagSelf(this->NinvW, invWq);   // apply joint metric
  }


  bool hasTaskRegions = false;

  for (size_t i=0; i<controller->getNumberOfTasks(); ++i)
  {
    Task* tsk = controller->getTask(i);
    TaskRegion* tsr = tsk->getTaskRegion();

    if (tsr)
    {
      hasTaskRegions = true;
      break;
    }
  }


  if (hasTaskRegions)
  {
#if 0
    // Compute null space -> task space re-projection: (dH^T J#)^T
    MatNd_reshape(this->dHr, dHr->n, dHr->m);
    MatNd_reshape(this->dx_proj, 1, pinvJ->n);
    MatNd_mul(this->dx_proj, this->dHr, this->pinvJ);
    MatNd_reshape(this->dHr, dHr->n, dHr->m);
    MatNd_reshape(this->dx_proj, pinvJ->n, 1);
#else
    // Compute null space -> task space re-projection: J dH^T
    MatNd_reshape(this->dx_proj, J->m, 1);
    MatNd_mul(this->dx_proj, this->J, this->dHr);
#endif

    // Here comes the sinister HACK
    for (size_t i=0; i< controller->getNumberOfTasks(); ++i)
    {
      Task* tsk = controller->getTask(i);
      TaskRegion* tsr = tsk->getTaskRegion();

      if (tsr)
      {
        size_t idx = controller->getTaskArrayIndex(i);
        const double* xi = &dx_proj->ele[idx];
        std::vector<double> dx_proj(xi, xi+tsk->getDim());
        tsr->setTaskReprojection(dx_proj);
      }
    }

  }   // if (hasTaskRegions)

  // Here ends the sinister HACK





  // Compute task space displacement vector
  if ((dx != NULL) && (J->m>0))
  {
    MatNd_reshape(this->dqr, this->pinvJ->m, 1);

    if ((activation!=NULL) && (dx->m==nx))
    {
      controller->compressToActive(this->dxr, dx, activation);
      MatNd_mul(this->dqr, this->pinvJ, this->dxr);
    }
    else
    {
      MatNd_mul(this->dqr, this->pinvJ, dx);
    }

    // Expand from coupled joint space to constraint joints
    if (hasCoupledJoints)
    {
      MatNd_reshape(dq_ts, nq, 1);
      MatNd_mul(dq_ts, this->A, this->dqr);
    }
    else
    {
      MatNd_reshapeCopy(dq_ts, this->dqr);
    }

    // Uncompress to all states
    RcsGraph_stateVectorFromIKSelf(graph, dq_ts);
  }
  else
  {
    MatNd_reshapeAndSetZero(dq_ts, graph->dof, 1);
  }











  // Compute null space displacement vector
  if (dH != NULL)
  {
    MatNd_reshape(this->dqr, this->pinvJ->m, 1);
    MatNd_mul(this->dqr, this->NinvW, this->dHr);

    // Expand from coupled joint space to constraint joints
    if (hasCoupledJoints)
    {
      MatNd_reshape(dq_ns, nq, 1);
      MatNd_mul(dq_ns, this->A, this->dqr);
    }
    else
    {
      MatNd_reshapeCopy(dq_ns, this->dqr);
    }

    // Uncompress to all states
    RcsGraph_stateVectorFromIKSelf(graph, dq_ns);
  }
  else
  {
    MatNd_reshapeAndSetZero(dq_ns, graph->dof, 1);
  }

  return det;
}

/*******************************************************************************
 *
 ******************************************************************************/
unsigned int IkSolverRMR::getInternalDof() const
{
  return this->A->n;
}

/*******************************************************************************
 *
 ******************************************************************************/
const MatNd* IkSolverRMR::getJacobian() const
{
  return this->J;
}

/*******************************************************************************
 *
 ******************************************************************************/
const MatNd* IkSolverRMR::getPseudoInverse() const
{
  return this->pinvJ;
}

/*******************************************************************************
 *
 ******************************************************************************/
const MatNd* IkSolverRMR::getNullSpace() const
{
  return this->NinvW;
}

/*******************************************************************************
 *
 ******************************************************************************/
const MatNd* IkSolverRMR::getCurrentActivation() const
{
  return this->ax_curr;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool IkSolverRMR::computeRightInverse(MatNd* pinvJ_,
                                      const MatNd* activation,
                                      double lambda) const
{
  const RcsGraph* graph = controller->getGraph();
  const unsigned int nq = controller->getGraph()->nJ;
  const size_t nx = controller->getTaskDim();

  MatNd* A_ = MatNd_create(nq, nq);
  MatNd* invA_ = MatNd_create(nq, nq);
  MatNd* J_ = MatNd_create(nx, nq);
  MatNd* invWq_ = MatNd_create(nq, 1);

  // Look for kinematic joint couplings in each iteration so that they can be
  // changed at run-time
  const int nCpldJnts = RcsGraph_countCoupledJoints(graph);
  const bool hasCouplings = (nCpldJnts>0);

  // Create coupled joint matrix
  if (hasCouplings)
  {
    RcsGraph_coupledJointMatrix(graph, A_, invA_);
  }

  // Compute the Jacobian and joint metric
  controller->computeJ(J_, activation);
  RcsGraph_getInvWq(graph, invWq_, RcsStateIK);

  // Task space
  if (hasCouplings)
  {
    MatNd_postMulSelf(J_, A_);
    MatNd_preMulSelf(invWq_, invA_);
  }

  MatNd lambdaArr = MatNd_fromPtr(1, 1, &lambda);
  double det_ = MatNd_rwPinv(pinvJ_, J_, invWq_, &lambdaArr);

  if (hasCouplings)
  {
    MatNd_preMulSelf(pinvJ_, A_);
  }

  MatNd_destroy(A_);
  MatNd_destroy(invA_);
  MatNd_destroy(J_);
  MatNd_destroy(invWq_);

  return (det_==0.0) ? false : true;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool IkSolverRMR::setActivationBlending(const std::string& mode)
{
  bool success = true;

  if (mode=="Binary")
  {
    blendingMode = TaskSpaceBlender::BlendingMode::Binary;
  }
  else if (mode=="Linear")
  {
    blendingMode = TaskSpaceBlender::BlendingMode::Linear;
  }
  else if (mode=="Approximate")
  {
    blendingMode = TaskSpaceBlender::BlendingMode::Approximate;
  }
  else if (mode=="IndependentTasks")
  {
    blendingMode = TaskSpaceBlender::BlendingMode::IndependentTasks;
  }
  else if (mode=="DependentTasks")
  {
    blendingMode = TaskSpaceBlender::BlendingMode::DependentTasks;
  }
  else
  {
    RLOG_CPP(4, "Activation blending mode " << mode << " not supported");
    success = false;
  }

  return success;
}

}   // namespace Rcs