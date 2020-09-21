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

#include "IkSolverRMR.h"
#include "Rcs_typedef.h"
#include "Rcs_macros.h"
#include "Rcs_math.h"
#include "Rcs_parser.h"

#include <cfloat>



/*******************************************************************************
 *
 ******************************************************************************/
Rcs::IkSolverRMR::IkSolverRMR(Rcs::ControllerBase* controller_) :
  controller(controller_), nx(controller->getTaskDim()),
  nTasks(controller_->getNumberOfTasks()),
  nq(controller_->getGraph()->nJ), nqr(0), det(0.0),
  A(NULL), invA(NULL), invWq(NULL), J(NULL), pinvJ(NULL), N(NULL), dHA(NULL),
  dq(NULL), dqr(NULL), dxr(NULL), NinvW(NULL), Wx(NULL), dHr(NULL), dH(NULL),
  dH_jl(NULL), dH_ca(NULL), dx(NULL)
{
  // Coupled joint matrix
  this->A = MatNd_create(this->nq, this->nq);
  this->invA = MatNd_create(this->nq, this->nq);
  RcsGraph_coupledJointMatrix(controller_->getGraph(), this->A, this->invA);
  this->nqr = this->A->n;

  // Joint space weghting matrix
  this->invWq = MatNd_create(this->nq, 1);
  RcsGraph_getInvWq(controller_->getGraph(), this->invWq, RcsStateIK);
  MatNd_preMulSelf(this->invWq, this->invA);

  // Jacobian
  this->J = MatNd_create(this->nx, this->nq);

  // Pseudo-inverse
  this->pinvJ = MatNd_create(this->nq, this->nx);

  // Null space
  this->N = MatNd_create(this->nq, this->nq);

  // dHA = dH*A
  this->dHA = MatNd_create(this->nq, 1);

  // Configuration space vector
  this->dq = MatNd_create(controller_->getGraph()->dof, 1);

  // Configuration space vector with only non-coupled dof
  this->dqr = MatNd_create(this->nq, 1);

  // Task space vector with only non-coupled dof
  this->dxr = MatNd_create(this->nx, 1);

  // N*invWq
  this->NinvW = MatNd_create(this->nq, this->nq);

  // Task space weighting matrix
  this->Wx = MatNd_create(this->nx, 1);

  // Null space potential (in reduced coordinates)
  this->dHr = MatNd_create(this->nq, 1);

  // Null space potential (in IK-relevant coordinates)
  this->dH = MatNd_create(1, this->nq);

  // Null space potential for joint limits
  this->dH_jl = MatNd_create(1, this->nq);

  // Null space potential for collision avoidance
  this->dH_ca = MatNd_create(1, this->nq);

  // Task space error
  this->dx = MatNd_create(this->nx, 1);

  reshape();
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::IkSolverRMR::~IkSolverRMR()
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
  MatNd_destroy(this->dH_jl);
  MatNd_destroy(this->dH_ca);
  MatNd_destroy(this->dx);
}

/*******************************************************************************
 * Returns the determinant from the last solver call
 ******************************************************************************/
double Rcs::IkSolverRMR::getDeterminant() const
{
  return this->det;
}

/*******************************************************************************
 * Returns the pointer to the internal controller
 ******************************************************************************/
Rcs::ControllerBase* Rcs::IkSolverRMR::getController() const
{
  return this->controller;
}

/*******************************************************************************
 * \brief Change array dimensions of the internal arrays
 ******************************************************************************/
void Rcs::IkSolverRMR::reshape()
{
  this->nx     = controller->getTaskDim();
  this->nTasks = controller->getNumberOfTasks();
  this->nq     = controller->getGraph()->nJ;

  MatNd* tmpA    = MatNd_create(this->nq, this->nq);
  MatNd* tmpInvA = MatNd_create(this->nq, this->nq);
  RcsGraph_coupledJointMatrix(controller->getGraph(), tmpA, tmpInvA);
  this->nqr = tmpA->n;
  MatNd_destroy(tmpA);
  MatNd_destroy(tmpInvA);

  MatNd_reshape(this->dH, 1, this->nq);
  MatNd_reshape(this->dH_jl, 1, this->nq);
  MatNd_reshape(this->dH_ca, 1, this->nq);
}

/*******************************************************************************
 * Calculate Jacobians etc.
 ******************************************************************************/
void Rcs::IkSolverRMR::computeKinematics(const MatNd* activation,
                                         double lambda0)
{
  // Compute the joint coupling matrices. This is required only for joints
  // that have a non-constant coupling (e.g. polynomial coupling as in some
  // parallel linkage joints).
  RcsGraph_coupledJointMatrix(controller->getGraph(), this->A, this->invA);
  this->nqr = this->A->n;

  // Update of the joint space wegihting matrix. This needs to be done if the
  // joint coupling matrices change, or the joint weights are dynamically
  // modified between successive calls.
  RcsGraph_getInvWq(controller->getGraph(), this->invWq, RcsStateIK);
  MatNd_preMulSelf(this->invWq, this->invA);

  // Compute the Jacobian
  controller->computeJ(this->J, activation);
  MatNd_postMulSelf(this->J, this->A);         // J_r = J*A
  MatNd_postMulDiagSelf(this->J, this->invWq); // J_r = J*A*invWq_r

  // Compute the blending matrix
  computeBlendingMatrix(*controller, this->Wx, activation, J, lambda0, true);
}

/********************************************************************************
 * Calculate the inverse kinematics, left-hand (nq x nq) inverse.
 *******************************************************************************/
void Rcs::IkSolverRMR::solveLeftInverse(MatNd* dq_des,
                                        const MatNd* dx,
                                        const MatNd* dH,
                                        const MatNd* activation,
                                        double lambda0,
                                        bool recomputeKinematics)
{
  // Compute the required terms for the inverse kinematics calculation
  if (recomputeKinematics==true)
  {
    computeKinematics(activation, lambda0);
  }

  // Compute the left weighted regularized pseudo-inverse Jacobian
  MatNd lambda = MatNd_fromPtr(1, 1, &lambda0);
  this->det = MatNd_rwPinv2(this->pinvJ, J, Wx, &lambda);

  // Do nothing if the projection is singular
  if (this->det == 0.0)
  {
    RLOG(1, "Singular Jacobian - setting velocities to zero");
    MatNd_reshapeAndSetZero(dq_des, controller->getGraph()->dof, 1);
    return;
  }

  // Reset the result and some internal arrays to match the problem indices
  MatNd_reshapeAndSetZero(this->dqr, nqr, 1);

  // Add task space component to the result
  if (dx != NULL)
  {
    if ((activation!=NULL) && (dx->m==this->nx))
    {
      controller->compressToActive(this->dxr, dx, activation);
      MatNd_mul(this->dqr, pinvJ, dxr);
    }
    else
    {
      MatNd_mul(this->dqr, pinvJ, dx);
    }

  }

  // Compute null space
  if (dH != NULL)
  {
    RCHECK_MSG((dH->m==nq && dH->n==1) || (dH->m==1 && dH->n==nq),
               "dH: %d x %d, should be 1 x %d", dH->m, dH->n, nq);
    MatNd_reshapeCopy(this->dHr, dH);
    MatNd_reshape(dHr, nq, 1);
    MatNd_constMulSelf(this->dHr, -1.0);
    MatNd_preMulSelf(this->dHr, invA);
    MatNd_reshape(this->NinvW, nqr, nqr);
    MatNd_nullspace(this->NinvW, pinvJ, J);
    MatNd_postMulDiagSelf(this->NinvW, invWq);   // apply joint metric
    MatNd_mulAndAddSelf(this->dqr, NinvW, dHr);
  }

  // Project back into non-weighted space
  MatNd_eleMulSelf(dqr, invWq);

  // Expand from coupled joint space to constraint joints
  MatNd_reshape(dq_des, nq, 1);
  MatNd_mul(dq_des, A, dqr);

  // Uncompress to all states
  RcsGraph_stateVectorFromIKSelf(controller->getGraph(), dq_des);
}


/*******************************************************************************
 * Calculate the inverse kinematics, left-hand (nq x nq) inverse.
 ******************************************************************************/
void Rcs::IkSolverRMR::solveLeftInverse(MatNd* dq_ts,
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
    computeKinematics(activation, lambda0);
  }

  // Compute the left weighted regularized pseudo-inverse Jacobian
  MatNd lambda = MatNd_fromPtr(1, 1, &lambda0);
  this->det = MatNd_rwPinv2(this->pinvJ, J, Wx, &lambda);

  // Do nothing if the projection is singular
  if (this->det == 0.0)
  {
    RLOG(1, "Singular Jacobian - setting velocities to zero");
    MatNd_reshapeAndSetZero(dq_ts, controller->getGraph()->dof, 1);
    MatNd_reshapeAndSetZero(dq_ns, controller->getGraph()->dof, 1);
    return;
  }

  // Add task space component to the result
  if (dx != NULL)
  {
    // Reset the result and some internal arrays to match the problem indices
    MatNd_reshapeAndSetZero(this->dqr, nqr, 1);

    if ((activation!=NULL) && (dx->m==this->nx))
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
    MatNd_mul(dq_ts, A, dqr);

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
    MatNd_preMulSelf(this->dHr, invA);
    MatNd_reshape(this->NinvW, nqr, nqr);
    MatNd_nullspace(this->NinvW, pinvJ, J);
    MatNd_postMulDiagSelf(this->NinvW, invWq);   // apply joint metric
    MatNd_reshape(this->dqr, nqr, 1);
    MatNd_mul(this->dqr, NinvW, dHr);

    // Project back into non-weighted space
    MatNd_eleMulSelf(this->dqr, invWq);

    // Expand from coupled joint space to constraint joints
    MatNd_reshape(dq_ns, nq, 1);
    MatNd_mul(dq_ns, A, dqr);

    // Uncompress to all states
    RcsGraph_stateVectorFromIKSelf(controller->getGraph(), dq_ns);
  }
  else
  {
    MatNd_reshapeAndSetZero(dq_ns, controller->getGraph()->dof, 1);
  }
}

/*******************************************************************************
 * Calculate the inverse kinematics using the right-hand pseudo-inverse. This
 * follows the derivation of Liegeois.
 ******************************************************************************/
void Rcs::IkSolverRMR::solveRightInverse(MatNd* dq_des,
                                         const MatNd* dx,
                                         const MatNd* dH,
                                         const MatNd* activation,
                                         double lambda0)
{
  MatNd lambdaArr = MatNd_fromPtr(1, 1, &lambda0);
  solveRightInverse(dq_des, dx, dH, activation, &lambdaArr);
  return;
}

void Rcs::IkSolverRMR::solveRightInverse(MatNd* dq_ts,
                                         MatNd* dq_ns,
                                         const MatNd* dx,
                                         const MatNd* dH,
                                         const MatNd* activation,
                                         double lambda0)
{
  MatNd lambdaArr = MatNd_fromPtr(1, 1, &lambda0);
  solveRightInverse(dq_ts, dq_ns, dx, dH, activation, &lambdaArr);
  return;
}

/*******************************************************************************
 * Calculate the inverse kinematics using the right-hand pseudo-inverse. This
 * follows the derivation of Liegeois.
 *
 * This version uses a full lambda vector instead of a single value
 ******************************************************************************/
void Rcs::IkSolverRMR::solveRightInverse(MatNd* dq_des,
                                         const MatNd* dx,
                                         const MatNd* dH,
                                         const MatNd* activation,
                                         const MatNd* lambda)
{
  MatNd* dq_ns = MatNd_createLike(dq_des);
  solveRightInverse(dq_des, dq_ns, dx, dH, activation, lambda);
  MatNd_addSelf(dq_des, dq_ns);
  MatNd_destroy(dq_ns);
}

void Rcs::IkSolverRMR::solveRightInverse(MatNd* dq_ts,
                                         MatNd* dq_ns,
                                         const MatNd* dx,
                                         const MatNd* dH,
                                         const MatNd* activation,
                                         const MatNd* lambda)
{
  RcsGraph* graph = controller->getGraph();

  // Look for kinematic joint couplings in each iteration so that they can be
  // changed at run-time
  const int nCpldJnts = RcsGraph_countCoupledJoints(controller->getGraph());
  const bool hasCouplings = (nCpldJnts>0);

  // Create coupled joint matrix
  if (hasCouplings)
  {
    RcsGraph_coupledJointMatrix(graph, this->A, this->invA);
  }

  // Compute the Jacobian and joint metric
  controller->computeJ(this->J, activation);
  RcsGraph_getInvWq(graph, this->invWq, RcsStateIK);

  // Task space
  if (hasCouplings)
  {
    MatNd_postMulSelf(this->J, this->A);
    MatNd_preMulSelf(this->invWq, this->invA);
  }

  this->det = MatNd_rwPinv(this->pinvJ, this->J, this->invWq, lambda);

  // Do nothing if the projection is singular
  if (this->det == 0.0)
  {
    RLOG(1, "Singular Jacobian - setting velocities to zero");
    MatNd_reshapeAndSetZero(dq_ts, graph->dof, 1);
    MatNd_reshapeAndSetZero(dq_ns, graph->dof, 1);
    return;
  }

  // Add task space component to the result
  MatNd_reshapeAndSetZero(this->dqr, this->pinvJ->m, 1);

  if ((dx != NULL) && (J->m>0))
  {
    if ((activation!=NULL) && (dx->m==nx))
    {
      controller->compressToActive(this->dxr, dx, activation);
      MatNd_mul(this->dqr, this->pinvJ, this->dxr);
    }
    else
    {
      MatNd_mul(this->dqr, this->pinvJ, dx);
    }

  }

  // Expand from coupled joint space to constraint joints
  if (hasCouplings)
  {
    MatNd_reshape(dq_ts, graph->nJ, 1);
    MatNd_mul(dq_ts, this->A, this->dqr);
  }
  else
  {
    MatNd_reshapeCopy(dq_ts, this->dqr);
  }



  // Compute null space
  MatNd_setZero(this->dqr);

  if (dH != NULL)
  {
    RCHECK((dH->m==nq && dH->n==1) || (dH->m==1 && dH->n==nq));
    MatNd_reshapeCopy(this->dHr, dH);
    MatNd_constMulSelf(this->dHr, -1.0);
    MatNd_reshape(this->dHr, nq, 1);
    if (hasCouplings)
    {
      MatNd_preMulSelf(this->dHr, this->invA);
    }
    MatNd_reshape(this->NinvW, this->pinvJ->m, this->pinvJ->m);
    MatNd_nullspace(this->NinvW, this->pinvJ, this->J);
    MatNd_postMulDiagSelf(this->NinvW, invWq);   // apply joint metric
    MatNd_mulAndAddSelf(this->dqr, this->NinvW, this->dHr);
  }

  // Expand from coupled joint space to constraint joints
  if (hasCouplings)
  {
    MatNd_reshape(dq_ns, nq, 1);
    MatNd_mul(dq_ns, this->A, this->dqr);
  }
  else
  {
    MatNd_reshapeCopy(dq_ns, this->dqr);
  }

  // Uncompress to all states
  RcsGraph_stateVectorFromIKSelf(graph, dq_ts);
  RcsGraph_stateVectorFromIKSelf(graph, dq_ns);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::IkSolverRMR::computeBlendingMatrix(const Rcs::ControllerBase& cntrl,
                                             MatNd* Wx,
                                             const MatNd* a_des,
                                             const MatNd* J,
                                             const double lambda0,
                                             const bool useInnerProduct)
{
  unsigned int i, j, dimTask, nRows = 0;

  for (i=0; i<cntrl.getNumberOfTasks(); i++)
  {

    if (MatNd_get(a_des, i, 0) == 0.0)
    {
      NLOG(4, "Skipping task \"%s\": activation is %f",
           cntrl.getTaskName(i).c_str(), a_des->ele[i]);
      continue;
    }

    const double ci = Math_clip(MatNd_get(a_des, i, 0), 0.0, 1.0);

    dimTask = cntrl.getTaskDim(i);

    RCHECK_MSG(Wx->size >= nRows + dimTask, "While adding task "
               "\"%s\": size of Wx: %d   m: %d   dimTask: %d",
               cntrl.getTaskName(i).c_str(), Wx->size, nRows, dimTask);

    for (j=0; j<dimTask; j++)
    {
      // Task space blending part 1
      double aBuf = 1.0, wi1 = 1.0;

      if (useInnerProduct==true)
      {
        MatNd Ji = MatNd_fromPtr(1, J->n, MatNd_getRowPtr(J, nRows));
        MatNd a = MatNd_fromPtr(1, 1, &aBuf);
        MatNd_sqrMulABAt(&a, &Ji, NULL);
      }

      if (ci < 1.0)
      {
        if (aBuf == 0.0)
        {
          wi1 = 0.0;
        }
        else
        {
          wi1 = lambda0 * ci / (aBuf * (1.0 - ci));
        }
      }

      // Task space blending part 2
      // The minimum lambda_i is one or two orders of magnitude smaller
      // than the joint space metric lambda
      double wi2 = pow(0.01*lambda0, 1.0-ci);

      // Fusion of both parts
      Wx->ele[nRows] = a_des->ele[i]*wi2 + (1.0-a_des->ele[i])*wi1;
      nRows++;

    }   // for(j=0;j<dimTask;j++)


  }   // for(i=0; i<nTasks; i++)

  // Reshape
  Wx->m = nRows;
  Wx->n = 1;
}

/*******************************************************************************
 *
 ******************************************************************************/
unsigned int Rcs::IkSolverRMR::getInternalDof() const
{
  return this->nqr;
}

/*******************************************************************************
 *
 ******************************************************************************/
const MatNd* Rcs::IkSolverRMR::getJacobian() const
{
  return J;
}

/*******************************************************************************
 *
 ******************************************************************************/
const MatNd* Rcs::IkSolverRMR::getPseudoInverse() const
{
  return pinvJ;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::IkSolverRMR::computeRightInverse(MatNd* pinvJ_,
                                           const MatNd* activation,
                                           double lambda) const
{
  const RcsGraph* graph = controller->getGraph();

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
