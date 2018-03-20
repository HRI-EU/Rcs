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

#include "SolverRAC.h"
#include "IkSolverRMR.h"
#include "Rcs_macros.h"
#include "Rcs_typedef.h"


/*******************************************************************************
 * Constructor based on xml parsing.
 ******************************************************************************/
Rcs::SolverRAC::SolverRAC(const Rcs::ControllerBase* controller_) :
  controller(controller_)
{
}


/*******************************************************************************
 * Destructor.
 ******************************************************************************/
Rcs::SolverRAC::~SolverRAC()
{

}

/*******************************************************************************
 * compute torque inverse dynamics task space
 * Tracking accelerations: ax = xpp_des - kp (x - x_des) - kd (xp - xp_des)
 ******************************************************************************/
void Rcs::SolverRAC::solve(MatNd* qpp_des,
                           const MatNd* a_des,
                           const MatNd* ax_des,
                           const MatNd* dH_,
                           double lambda)
{
  RcsGraph* graph = controller->getGraph();
  unsigned int nq = graph->dof;
  unsigned int nJ = graph->nJ;
  unsigned int nx = controller->getTaskDim();

  MatNd* jointMetric     = MatNd_create(nq, 1);
  MatNd* jointMetricInvA = MatNd_create(nq, 1);
  MatNd* J               = MatNd_create(nx,nq);
  MatNd* Wx              = MatNd_create(nx,1);
  MatNd* invWq           = MatNd_create(1,1);
  invWq->ele[0] = 1.0e-8;
  MatNd* wJpinv          = MatNd_create(nq,nx);
  MatNd* C_curr          = MatNd_create(nx, nx);
  MatNd* ax_curr         = MatNd_create(nx, 1);
  MatNd* ax              = MatNd_create(nx, 1);
  MatNd* aq              = MatNd_create(nq,1);
  MatNd* JdotQdot        = MatNd_create(nx,1);
  MatNd* N               = MatNd_create(nq,nq);
  MatNd* dH              = MatNd_create(1, nq);
  MatNd* q_dot_ik        = MatNd_create(1, nJ);
  MatNd* A               = NULL;
  MatNd* invA            = NULL;

  // If there are coupled joints, we create the matrices. This is a bit time
  // comsuming, but most general to do it here, since it depends on the graph's
  // state for polynomial couplings.
  if (RcsGraph_countCoupledJoints(graph)>0)
  {
    A    = MatNd_create(nq, nq);
    invA = MatNd_create(nq, nq);
    RcsGraph_coupledJointMatrix(graph, A, invA);
  }

  // Joint space weighting matrix
  RcsGraph_getInvWq(graph, jointMetric, RcsStateIK);

  // Reduce to coupled joint coordinates
  if (invA != NULL)
  {
    MatNd_reshape(jointMetricInvA, invA->m, 1);
    MatNd_mul(jointMetricInvA, invA, jointMetric);
    //MatNd_preMulSelf(controller->jointMetricInvA, controller->invA);
  }
  else
  {
    MatNd_reshapeCopy(jointMetricInvA, jointMetric);
  }

  // Jacobian, pseudo-inverse and weighting matrix for task blending
  // Here we don't make any difference between torque- and velocity actuators.
  controller->computeJ(J, a_des);

  if (A != NULL)
  {
    MatNd_postMulSelf(J, A);         // J_r = J*A
  }

  MatNd_postMulDiagSelf(J, jointMetricInvA);     // J_r = J*A*invWq_r

  // Compute the task-space weight matrix for blending
  Rcs::IkSolverRMR::computeBlendingMatrix(*controller, Wx, a_des, J, false);

  // MatNd_printDims("wJpinv", wJpinv);
  // MatNd_printDims("J", J);
  // MatNd_printDims("Wx", Wx);
  // MatNd_printDims("invWq", invWq);
  //  double det = MatNd_rwPinv2(wJpinv, J, Wx, invWq);
  MatNd lambdaMat = MatNd_fromPtr(1, 1, &lambda);
  double det = MatNd_rwPinv(wJpinv, J, jointMetricInvA, &lambdaMat);

  if (det==0.0)
  {
    RLOG(1, "Singular!");
    MatNd_reshapeAndSetZero(qpp_des, graph->dof, 1);
    return;
  }

  // Calculate the current activations based on the re-projection of the
  // weighted pseudo-inverse into the task-space
  MatNd_reshape(C_curr, J->m, J->m);
  MatNd_mul(C_curr, J, wJpinv);
  MatNd_reshape(ax_curr, J->m, 1);
  MatNd_getDiag(ax_curr, C_curr);
  controller->decompressFromActiveSelf(ax_curr, a_des);


  // Compute joint-space accelerations, Here we consider also the Jacobian
  // derivative term: qpp = J# (ax - Jdot qp)
  MatNd_reshapeCopy(ax, ax_des);
  controller->computeJdotQdot(JdotQdot, a_des);
  MatNd_subSelf(ax, JdotQdot);
  MatNd_reshape(aq, wJpinv->m, 1);
  MatNd_mul(aq, wJpinv, ax);


  // Compute null space acceleration, again here we don't make any difference
  // between torque- and velocity actuators.
  MatNd_reshape(N, wJpinv->m, wJpinv->m);
  MatNd_nullspace(N, wJpinv, J);

  // Null space gradient on position level. Here we apply the joint metric
  // weighting
  MatNd_reshapeCopy(dH, dH_);
  MatNd_reshape(dH, graph->nJ, 1);
  MatNd_eleMulSelf(dH, jointMetric);

  // Joint speed damping: Kd(qp_des - qp) with qp_des = 0
  RcsGraph_stateVectorToIK(graph, graph->q_dot, q_dot_ik);

  // Remove effect of joint metric for damping
  // for(unsigned int i=0;i<controller->qp_temp->m;i++)
  //   {
  //     controller->qp_temp->ele[i] /= controller->jointMetric->ele[i];
  //   }

  // Add damping term to null space
  double kd_nullspace = 1.0;
  MatNd_constMulAndAddSelf(dH, q_dot_ik, -kd_nullspace);

  // Compress to coupled joint space
  if (invA != NULL)
  {
    MatNd_preMulSelf(dH, invA);
  }

  // Project gradients into the null space of the movement and add them to the
  // joint space accelerations
  MatNd_preMulSelf(dH, N);
  MatNd_addSelf(aq, dH);
  MatNd_eleMulSelf(aq, jointMetricInvA);

  // Here we inflate the joint space accelerations to the IK dimensions
  if (A != NULL)
  {
    MatNd_preMulSelf(aq, A);
  }

  // Reshape the velocity vector to the coupled degrees of freedom space
  // and integrate the accelerations
  RcsGraph_stateVectorFromIK(graph, aq, qpp_des);

  // Clean up
  MatNd_destroy(jointMetric);
  MatNd_destroy(invA);
  MatNd_destroy(jointMetricInvA);
  MatNd_destroy(J);
  MatNd_destroy(A);
  MatNd_destroy(Wx);
  MatNd_destroy(invWq);
  MatNd_destroy(wJpinv);
  MatNd_destroy(C_curr);
  MatNd_destroy(ax_curr);
  MatNd_destroy(ax);
  MatNd_destroy(aq);
  MatNd_destroy(JdotQdot);
  MatNd_destroy(N);
  MatNd_destroy(dH);
}


