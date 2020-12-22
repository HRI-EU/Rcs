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

/*!
 *  Description: Kinematics and differential kinematics functions for forward
 *  finite difference tests.
 */

#include "Rcs_gradientTests.h"
#include "Rcs_typedef.h"
#include "Rcs_kinematics.h"
#include "Rcs_body.h"
#include "Rcs_utils.h"
#include "Rcs_macros.h"
#include "Rcs_timer.h"
#include "Rcs_math.h"


//#define STATIC_FUNC static
#define STATIC_FUNC

static RcsBody* RCS_GRADIENTTESTS_BODY1 = NULL;
static RcsBody* RCS_GRADIENTTESTS_BODY2 = NULL;
static RcsBody* RCS_GRADIENTTESTS_BODY3 = NULL;
double RCS_GRADIENTTESTS_BODYPT1[3];
double RCS_GRADIENTTESTS_BODYPT2[3];
static int RCS_RNDXYZ = 0;



static RcsBody* bdy_target_qprop = NULL;

static void clearQProp(void)
{
  if (!bdy_target_qprop)
  {
    return;
  }

  RFREE(bdy_target_qprop);
  bdy_target_qprop = NULL;
}

static void initQProp(void)
{
  if (bdy_target_qprop)
  {
    return;
  }

  bdy_target_qprop = RcsBody_create();
  atexit(clearQProp);
}



/*******************************************************************************

  \brief Joint limit with zero bassin gradient test, constrained dofs only

*******************************************************************************/

static void f_jlBorderIK(double* y, const double* x, void* params)
{
  RcsGraph* graph = (RcsGraph*) params;
  MatNd q = MatNd_fromPtr(graph->nJ, 1, (double*) x);
  RcsGraph_setState(graph, &q, NULL);
  *y = RcsGraph_jointLimitBorderCost(graph, 0.75, RcsStateIK);
}

static void df_jlBorderIK(double* dy, const double* x, void* params)
{
  RcsGraph* graph = (RcsGraph*) params;
  MatNd q = MatNd_fromPtr(graph->nJ, 1, (double*) x);
  RcsGraph_setState(graph, &q, NULL);

  MatNd grad = MatNd_fromPtr(1, graph->nJ, dy);
  MatNd_setZero(&grad);
  RcsGraph_jointLimitBorderGradient(graph, &grad,0.75, RcsStateIK);
}



/*******************************************************************************

  \brief Joint limit with zero bassin gradient test, constrained dofs only

*******************************************************************************/

static void f_jlBorderFull(double* y, const double* x, void* params)
{
  RcsGraph* graph = (RcsGraph*) params;
  MatNd q = MatNd_fromPtr(graph->dof, 1, (double*) x);
  RcsGraph_setState(graph, &q, NULL);
  *y = RcsGraph_jointLimitBorderCost(graph, 0.75, RcsStateFull);
}

static void df_jlBorderFull(double* dy, const double* x, void* params)
{
  RcsGraph* graph = (RcsGraph*) params;
  MatNd q = MatNd_fromPtr(graph->dof, 1, (double*) x);
  RcsGraph_setState(graph, &q, NULL);

  MatNd grad = MatNd_fromPtr(1, graph->dof, dy);
  MatNd_setZero(&grad);
  RcsGraph_jointLimitBorderGradient(graph, &grad,0.75, RcsStateFull);
}



/******************************************************************************

  Spatial orientation test

******************************************************************************/

static void test_attitude3D(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  double Id[3][3];
  Mat3d_setIdentity(Id);

  Rcs_setIKState(self, x);

  RcsBody* ef       = RCS_GRADIENTTESTS_BODY2;
  //const RcsBody* refBdy   = RCS_GRADIENTTESTS_BODY1;
  const double* target    = RCS_GRADIENTTESTS_BODYPT1;

  double phi, E[3][3], A_TR[3][3], A_ER[3][3], A_TE[3][3];
  Mat3d_setIdentity(E);
  Mat3d_fromEulerAngles(A_TR, target);
  Mat3d_mulTranspose(A_ER, ef ? ef->A_BI.rot : Id, E);
  Mat3d_mulTranspose(A_TE, A_TR, A_ER);
  phi = Mat3d_diffAngleSelf(A_TE);
  *f = phi * phi;
}

static void test_dAttitude3D(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  double Id[3][3];
  Mat3d_setIdentity(Id);

  Rcs_setIKState(self, x);

  RcsBody* ef       = RCS_GRADIENTTESTS_BODY2;
  //const RcsBody* refBdy   = RCS_GRADIENTTESTS_BODY1;
  const double* target    = RCS_GRADIENTTESTS_BODYPT1;

  MatNd* gGradX = MatNd_create(1, 3);
  MatNd gGradQ = MatNd_fromPtr(1, self->nJ, f);

  const double eps = 1.0e-8;
  double phi0, E[3][3], A_TR[3][3], A_ER[3][3], A_TE[3][3];
  Mat3d_setIdentity(E);
  Mat3d_fromEulerAngles(A_TR, target);
  Mat3d_mulTranspose(A_ER, ef ? ef->A_BI.rot : Id, E);
  Mat3d_mulTranspose(A_TE, A_TR, A_ER);   // A_TE = A_TR * A_ER^T
  phi0 = Mat3d_diffAngleSelf(A_TE);

  // Finite difference gradient dphi/d(I_omega) (Pfui!)
  for (int i = 0; i < 3; i++)
  {
    double A_ERi[3][3];
    Mat3d_copy(A_ERi, ef ? ef->A_BI.rot : Id);
    Mat3d_rotateSelfAboutXYZAxis(A_ERi, i, eps);
    Mat3d_mulTranspose(A_TE, A_TR, A_ERi);
    double phi1 = Mat3d_diffAngleSelf(A_TE);
    gGradX->ele[i] = -(phi1 - phi0) / eps;
  }

  MatNd_constMulSelf(gGradX, -2.0 * phi0);
  MatNd* JR = MatNd_create(3, self->nJ);
  RcsGraph_rotationJacobian(self, ef, NULL, JR);
  MatNd_rotateSelf(JR, ef ? ef->A_BI.rot : Id);
  MatNd_mul(&gGradQ, gGradX, JR);
  MatNd_destroy(JR);

  MatNd_destroy(gGradX);
}



/******************************************************************************

  \brief f  = J# e
         df = J# dqe + dqJ# e

         This test has a problem: If e is getting large, the numerical
         gradient tends to deviate from the analytical one. It's an issue
         since when we use large feedback gains, this will be the case, and
         the analytical solutions seems to get incorrect.
         The reason might be the violation of the linearization assumption:
         Having large steps in the feedback error will make the resulting
         posture deviate from the tangential direction.

******************************************************************************/

void test_Jpinv_e(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  int nx = 3, nq = self->nJ;
  const double* pt = NULL;

  // Initialize target body
  if (!bdy_target_qprop)
  {
    initQProp();
  }
  const RcsBody* b     = RcsGraph_getBodyByName(self, "Seg6");
  RCHECK(b);
  //const RcsBody *b     = Rcs_getRndBdy1();
  const RcsBody* b_des = bdy_target_qprop;

  Rcs_setIKState(self, x);

  // Determine e
  MatNd* e = MatNd_create(nx, 1);
  Vec3d_sub(e->ele, b_des->A_BI.org, b->A_BI.org);

  // J# e
  MatNd* J     = MatNd_create(nx, nq);
  MatNd* Jpinv = MatNd_create(nq, nx);
  MatNd J_pinv_e = MatNd_fromPtr(nq, 1, f);
  RcsGraph_bodyPointJacobian(self, b, pt, NULL, J);
  MatNd_rwPinv(Jpinv, J, NULL, NULL);
  MatNd_mul(&J_pinv_e, Jpinv, e);

  MatNd_destroy(e);
  MatNd_destroy(J);
  MatNd_destroy(Jpinv);
}

void test_dJpinv_e(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  int nx = 3, nq = self->nJ;
  const double* pt = NULL;

  // Initialize target body
  if (!bdy_target_qprop)
  {
    initQProp();
  }
  RcsBody* b     = Rcs_getRndBdy1();
  RCHECK(b);
  RcsBody* b_des = bdy_target_qprop;

  Rcs_setIKState(self, x);

  // Determine target body transformation
  const double eps = 1.0e-3;
  Vec3d_constMul(b_des->A_BI.org, b->A_BI.org, 1.0 - eps);
  Mat3d_copy(b_des->A_BI.rot, b->A_BI.rot);
  Mat3d_rotateSelfAboutXYZAxis(b_des->A_BI.rot, 0, eps);
  Mat3d_rotateSelfAboutXYZAxis(b_des->A_BI.rot, 1, eps);
  Mat3d_rotateSelfAboutXYZAxis(b_des->A_BI.rot, 2, eps);

  // J# e
  MatNd* e     = MatNd_create(3, 1);
  MatNd* J     = MatNd_create(nx, nq);
  MatNd* Jpinv = MatNd_create(nq, nx);
  Vec3d_sub(e->ele, b->A_BI.org, b_des->A_BI.org);
  RcsGraph_bodyPointJacobian(self, b, pt, NULL, J);
  MatNd_rwPinv(Jpinv, J, NULL, NULL);

  // Hessian and Pseudo-inverse derivative
  MatNd* dqJ    = MatNd_create(nx * nq, nq);
  MatNd* dJpinv = MatNd_create(nq * nq, nx);
  RcsGraph_bodyPointHessian(self, b, pt, NULL, dqJ);
  MatNd_PinvHessian(dJpinv, J, dqJ, NULL, NULL, Jpinv, false);

  // term1 = dq(J#) e
  MatNd* term1 = MatNd_create(nq * nq, 1);
  MatNd_mul(term1, dJpinv, e);
  MatNd_reshape(term1, nq, nq);

  // term2 = J# dq(e)
  // feedback error derivative dq(e) = -J for linear displacements
  MatNd* dqe   = MatNd_create(nx, nq);
  MatNd* term2 = MatNd_create(nq, nq);
  MatNd_constMul(dqe, J, -1.0);
  MatNd_mul(term2, Jpinv, dqe);

  // dq(J#) e + J# dq(e) = term1 + term2
  MatNd grad = MatNd_fromPtr(nq, nq, f);
  MatNd_add(&grad, term1, term2);

  // Cleanup
  MatNd_destroy(e);
  MatNd_destroy(J);
  MatNd_destroy(Jpinv);
  MatNd_destroy(dqJ);
  MatNd_destroy(dJpinv);
  MatNd_destroy(dqe);
  MatNd_destroy(term1);
  MatNd_destroy(term2);
}



/******************************************************************************

  \brief Null space backpropagation gradient. We compute it for the 6d position
         and orientation task space of the last body that is connected to its
         predecessor with joints. If the Pseudo-inverse is ill-conditioned,
         the gradient is set to zero.

         q[t+1]        = -a*N*invW*dqH
         dq[t+1]/dq[t] =  a { (dq(J#) J + J# dqJ) invW dH - N invW d2H }

******************************************************************************/

static void test_qprop_ns(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  Rcs_setIKState(self, x);

  int nx = 6, nq = self->nJ;
  const RcsBody* b = NULL;
  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    if (BODY->jnt)
    {
      b = BODY;
    }
  }
  const double* pt = NULL;
  const double alpha = 1.0, eps = 1.0e-4;
  MatNd* invW = NULL;

  // J
  MatNd* J = MatNd_create(nx, nq);
  MatNd JT = MatNd_fromPtr(3, nq, J->ele);
  MatNd JR = MatNd_fromPtr(3, nq, &J->ele[3 * nq]);
  RcsGraph_bodyPointJacobian(self, b, pt, NULL, &JT);
  RcsGraph_rotationJacobian(self, b, NULL, &JR);

  // J#
  MatNd* Jpinv = MatNd_create(nq, nx);
  double det    = MatNd_rwPinv(Jpinv, J, invW, NULL);

  // Return if ill-conditioned
  if (det < eps)
  {
    RLOG(1, "Determinant %g < %g - setting gradient of body %s to 0",
         det, eps, b ? b->bdyName : "NULL");
    MatNd fA = MatNd_fromPtr(nq, 1, f);
    MatNd_setZero(&fA);
    MatNd_destroy(J);
    MatNd_destroy(Jpinv);
    return;
  }

  // NW = (E - J# J) invW
  MatNd* NW = MatNd_create(nq, nq);
  MatNd_nullspace(NW, Jpinv, J);
  if (invW)
  {
    MatNd_postMulDiagSelf(NW, invW);
  }

  // dH
  MatNd* dH = MatNd_create(nq, 1);
  RcsGraph_jointLimitGradient(self, dH, RcsStateIK);

  // -a*N*invW*dqH
  MatNd fA = MatNd_fromPtr(nq, 1, f);
  MatNd_mul(&fA, NW, dH);
  MatNd_constMulSelf(&fA, -alpha);

  // Cleanup
  MatNd_destroy(J);
  MatNd_destroy(Jpinv);
  MatNd_destroy(NW);
  MatNd_destroy(dH);
}

static void test_dqprop_ns(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  Rcs_setIKState(self, x);

  double dt = Timer_getTime();

  int nx = 6, nq = self->nJ;
  const RcsBody* b = NULL;
  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    if (BODY->jnt)
    {
      b = BODY;
    }
  }
  const double* pt = NULL;
  const double alpha = 1.0, eps = 1.0e-4;
  MatNd* invW = NULL;

  // Jacobian
  MatNd* J   = MatNd_create(nx, nq);
  MatNd Jtmp = MatNd_fromPtr(3, nq, J->ele);
  RcsGraph_bodyPointJacobian(self, b, pt, NULL, &Jtmp);
  Jtmp = MatNd_fromPtr(3, nq, &J->ele[3 * nq]);
  RcsGraph_rotationJacobian(self, b, NULL, &Jtmp);

  // WJt = W*J^T
  MatNd* WJt = MatNd_create(nq, nx);
  MatNd_transpose(WJt, J);
  if (invW)
  {
    MatNd_preMulDiagSelf(WJt, invW);
  }

  // invJWJt = (J invW J^T)^-1
  MatNd* invJWJt = MatNd_create(nx, nx);
  MatNd_mul(invJWJt, J, WJt);
  double det = MatNd_choleskyInverse(invJWJt, invJWJt);

  // Return if ill-conditioned
  if (det < eps)
  {
    RLOG(1, "Determinant %g < %g - setting gradient of body %s to 0",
         det, eps, b ? b->bdyName : "NULL");
    MatNd fA = MatNd_fromPtr(nq, nq, f);
    MatNd_setZero(&fA);
    MatNd_destroy(J);
    MatNd_destroy(WJt);
    MatNd_destroy(invJWJt);
    return;
  }

  // Pseudoinverse J#
  MatNd* Jpinv = MatNd_create(nq, nx);
  MatNd_mul(Jpinv, WJt, invJWJt);

  // NW = (E - J# J) invW
  MatNd* NW = MatNd_create(nq, nq);
  MatNd_nullspace(NW, Jpinv, J);
  if (invW)
  {
    MatNd_postMulDiagSelf(NW, invW);
  }

  // dH
  MatNd* dH = MatNd_create(nq, 1);
  RcsGraph_jointLimitGradient(self, dH, RcsStateIK);

  // d2H
  MatNd* d2H = MatNd_create(nq, nq);
  RcsGraph_jointLimitHessian(self, d2H, RcsStateIK);

  // Hessian
  MatNd* dqJ = MatNd_create(nx * nq, nq);
  RcsGraph_bodyPointHessian(self, b, pt, NULL, dqJ);
  MatNd dqJ_rot = MatNd_fromPtr(3*nq, nq, &dqJ->ele[3*nq*nq]);
  RcsGraph_rotationHessian(self, b, NULL, &dqJ_rot);

  // Pseudoinverse derivative
  MatNd* dJpinv = MatNd_create(nq * nq, nx);
  MatNd_PinvHessian(dJpinv, J, dqJ, NULL, invJWJt, Jpinv, true);

  // term 1 = dq(J#) J = (nq x nq x nx) * (nx x nq) = (nq x nq x nq)
  MatNd* term1 = MatNd_create(nq * nq, nq);
  MatNd_mul(term1, dJpinv, J);

  // term2 = J# dqJ = (nq x nx) * (nx x nq x nq) = (nq x nq x nq)
  MatNd* term2 = MatNd_create(nq, nq * nq);
  MatNd_reshape(dqJ, nx, nq * nq);
  MatNd_mul(term2, Jpinv, dqJ);

  // Term2 = dq(J#) J + J# dqJ
  MatNd_reshape(term2, nq * nq, nq);
  MatNd_addSelf(term2, term1);

  // Term3 = (dq(J#) J + J# dqJ) invW dH
  MatNd* term3 = MatNd_create(nq * nq, 1);
  MatNd_reshape(term2, nq * nq, nq);
  MatNd_mul(term3, term2, dH);
  MatNd_reshape(term3, nq, nq);

  // Term4 = NW d2H
  MatNd* term4 = MatNd_create(nq, nq);
  MatNd_mul(term4, NW, d2H);

  // dq[t+1]/dq[t] =  a { (dq(J#) J + J# dqJ) invW dH - N invW d2H }
  MatNd fA = MatNd_fromPtr(nq, nq, f);
  MatNd_sub(&fA, term3, term4);
  MatNd_constMulSelf(&fA, alpha);

  // Cleanup
  MatNd_destroy(J);
  MatNd_destroy(dH);
  MatNd_destroy(d2H);
  MatNd_destroy(dqJ);
  MatNd_destroy(dJpinv);
  MatNd_destroy(WJt);
  MatNd_destroy(invJWJt);
  MatNd_destroy(Jpinv);
  MatNd_destroy(NW);
  MatNd_destroy(term1);
  MatNd_destroy(term2);
  MatNd_destroy(term3);
  MatNd_destroy(term4);

  dt = Timer_getTime() - dt;
  RLOG(5, "test_dqprop_ns took %g msec", 1.0e3 * dt);
}



/******************************************************************************

  \brief Task space propagation gradient. The task space is 6-dimensional,
         composed of the linear and angular (Euler angles) elements with
         respect to a non-moving reference body (bdy_target_qprop). The
         feedback error is computed differentially, this makes the mapping
         unique and numerically stable.

         q[t+1]        = q[t] + J# e
         dq[t+1]/dq[t] = E + dq(J#) e + J# dq(e)

******************************************************************************/

static void test_qprop_ts(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  Rcs_setIKState(self, x);

  // Initialize target body
  if (!bdy_target_qprop)
  {
    initQProp();
  }

  int nx = 6, nq = self->nJ;
  RcsBody* b     = Rcs_getRndBdy1();
  RcsBody* b_des = bdy_target_qprop;
  const double* pt     = NULL;

  // Determine feedback error
  MatNd* e = MatNd_create(nx, 1);
  Vec3d_sub(e->ele, b_des->A_BI.org, b->A_BI.org);
  Mat3d_getEulerError(&e->ele[3], b->A_BI.rot, b_des->A_BI.rot);

  // J# e
  MatNd* J = MatNd_create(nx, nq);
  MatNd* Jpinv = MatNd_create(nq, nx);
  MatNd JT = MatNd_fromPtr(3, nq, J->ele);
  RcsGraph_bodyPointJacobian(self, b, pt, NULL, &JT);
  MatNd JR = MatNd_fromPtr(3, nq, &J->ele[3 * nq]);
  RcsGraph_rotationJacobian(self, b, NULL, &JR);
  MatNd_rwPinv(Jpinv, J, NULL, NULL);
  MatNd fA = MatNd_fromPtr(nq, 1, f);
  MatNd_mul(&fA, Jpinv, e);

  // q[t+1] = q[t] + J# e
  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    if (JNT->jacobiIndex != -1)
    {
      fA.ele[JNT->jacobiIndex] += self->q->ele[JNT->jointIndex];
    }
  }

  MatNd_destroy(e);
  MatNd_destroy(J);
  MatNd_destroy(Jpinv);
}

static void test_dqprop_ts(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  Rcs_setIKState(self, x);

  // Initialize target body
  if (!bdy_target_qprop)
  {
    initQProp();
  }

  double dt = Timer_getTime();

  int nx = 6, nq = self->nJ;
  RcsBody* b     = Rcs_getRndBdy1();
  RcsBody* b_des = bdy_target_qprop;
  const double* pt     = NULL;

  // Determine target body transformation
  const double eps = 1.0e-6;
  Vec3d_constMul(b_des->A_BI.org, b->A_BI.org, 1.0 - eps);
  Mat3d_copy(b_des->A_BI.rot, b->A_BI.rot);
  Mat3d_rotateSelfAboutXYZAxis(b_des->A_BI.rot, 0, eps);
  Mat3d_rotateSelfAboutXYZAxis(b_des->A_BI.rot, 1, eps);
  Mat3d_rotateSelfAboutXYZAxis(b_des->A_BI.rot, 2, eps);

  // Determine feedback error e
  MatNd* e = MatNd_create(nx, 1);
  Vec3d_sub(e->ele, b_des->A_BI.org, b->A_BI.org);
  //HTr_getEulerError(b->A_BI, b_des->A_BI, &e->ele[3]);
  Mat3d_getEulerError(&e->ele[3], b->A_BI.rot, b_des->A_BI.rot);

  // Jacobian
  MatNd* J = MatNd_create(nx, nq);
  MatNd JT = MatNd_fromPtr(3, nq, J->ele);
  MatNd JR = MatNd_fromPtr(3, nq, &J->ele[3 * nq]);
  RcsGraph_bodyPointJacobian(self, b, pt, NULL, &JT);
  RcsGraph_rotationJacobian(self, b, NULL, &JR);

  // WJt = W*J^T
  MatNd* invW = NULL;
  MatNd* WJt = MatNd_create(nq, nx);
  MatNd_transpose(WJt, J);
  if (invW)
  {
    MatNd_preMulDiagSelf(WJt, invW);
  }

  // invJWJt = (J invW J^T)^-1
  MatNd* invJWJt = MatNd_create(nx, nx);
  MatNd_mul(invJWJt, J, WJt);
  MatNd_choleskyInverse(invJWJt, invJWJt);

  // Pseudoinverse
  MatNd* Jpinv = MatNd_create(nq, nx);
  MatNd_mul(Jpinv, WJt, invJWJt);

  // Hessian
  MatNd* dqJ = MatNd_create(nx * nq, nq);
  MatNd* dJpinv = MatNd_create(nq * nq, nx);
  RcsGraph_bodyPointHessian(self, b, pt, NULL, dqJ);
  MatNd dqJ_rot = MatNd_fromPtr(3*nq, nq, &dqJ->ele[3*nq*nq]);
  RcsGraph_rotationHessian(self, b, NULL, &dqJ_rot);

  // Pseudo-inverse derivative
  MatNd_PinvHessian(dJpinv, J, dqJ, invW, invJWJt, Jpinv, false);

  // Feedback error derivative dq(e)
  MatNd* dqe = MatNd_create(nx, nq);
  MatNd dqeT = MatNd_fromPtr(3, nq, dqe->ele);
  MatNd_constMul(&dqeT, &JT, -1.0);
  MatNd dqeR = MatNd_fromPtr(3, nq, &dqe->ele[3 * nq]);
  RcsGraph_dEulerErrorDq(self, b, b_des, &dqeR);

  // term1 = dq(J#) e
  MatNd* term1 = MatNd_create(nq * nq, 1);
  MatNd_mul(term1, dJpinv, e);
  MatNd_reshape(term1, nq, nq);

  // term2 = J# dq(e)
  MatNd* term2 = MatNd_create(nq, nq);
  MatNd_mul(term2, Jpinv, dqe);

  // dq[t+1]/dq[t] = E + term1 + term2
  MatNd fA = MatNd_fromPtr(nq, nq, f);
  MatNd_setIdentity(&fA);
  MatNd_addSelf(&fA, term1);
  MatNd_addSelf(&fA, term2);

  // Cleanup
  MatNd_destroy(e);
  MatNd_destroy(J);
  MatNd_destroy(WJt);
  MatNd_destroy(invJWJt);
  MatNd_destroy(Jpinv);
  MatNd_destroy(dqJ);
  MatNd_destroy(dJpinv);
  MatNd_destroy(dqe);
  MatNd_destroy(term1);
  MatNd_destroy(term2);

  dt = Timer_getTime() - dt;
  RLOG(5, "test_dqprop_ts took %g msec", 1.0e3 * dt);
}



/******************************************************************************

  \brief q[t+1] = q[t] + J# e
         dq[t+1]/dq[t] = E + dq(J#) e + J# dq(e)

******************************************************************************/

static double x_target_qprop[3];

STATIC_FUNC void test_qprop_ts3(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  Rcs_setIKState(self, x);

  int nx = 3, nq = self->nJ;
  const RcsBody* b = NULL;
  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    if (BODY->jnt)
    {
      b = BODY;
    }
  }
  const double* pt = NULL, eps = 1.0e-3;

  // Determine e
  MatNd* e = MatNd_create(nx, 1);
  Vec3d_sub(e->ele, b ? b->A_BI.org : Vec3d_zeroVec(), x_target_qprop);

  // J#
  MatNd* J = MatNd_create(nx, nq);
  MatNd* Jpinv = MatNd_create(nq, nx);
  RcsGraph_bodyPointJacobian(self, b, pt, NULL, J);
  double det = MatNd_rwPinv(Jpinv, J, NULL, NULL);
  MatNd fA  = MatNd_fromPtr(nq, 1, f);

  // Return if ill-conditioned
  if (det < eps)
  {
    RLOG(1, "Determinant %g < %g - setting gradient of body %s to 0",
         det, eps, b ? b->bdyName : "NULL");
    MatNd_setZero(&fA);
    MatNd_destroy(e);
    MatNd_destroy(J);
    MatNd_destroy(Jpinv);
    return;
  }

  // J# e
  MatNd_mul(&fA, Jpinv, e);

  // q[t+1] = q[t] + J# e
  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    if (JNT->jacobiIndex != -1)
    {
      fA.ele[JNT->jacobiIndex] += self->q->ele[JNT->jointIndex];
    }
  }

  MatNd_destroy(e);
  MatNd_destroy(J);
  MatNd_destroy(Jpinv);
}

STATIC_FUNC void test_dqprop_ts3(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  Rcs_setIKState(self, x);

  int nx = 3, nq = self->nJ;
  const RcsBody* b = NULL;
  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    if (BODY->jnt)
    {
      b = BODY;
    }
  }
  const double* pt = NULL, eps = 1.0e-3;

  // Determine x_target
  const double eps_target = 1.0e-3;
  Vec3d_constMul(x_target_qprop, b ? b->A_BI.org : Vec3d_zeroVec(),
                 1.0 - eps_target);

  // Determine e
  MatNd* e = MatNd_create(3, 1);
  Vec3d_sub(e->ele, b ? b->A_BI.org : Vec3d_zeroVec(), x_target_qprop);

  // Jacobian
  MatNd* J = MatNd_create(nx, nq);
  RcsGraph_bodyPointJacobian(self, b, pt, NULL, J);

  // WJt = W*J^T
  MatNd* invW = NULL;
  MatNd* WJt = MatNd_create(nq, nx);
  MatNd_transpose(WJt, J);
  if (invW)
  {
    MatNd_preMulDiagSelf(WJt, invW);
  }

  // invJWJt = (J invW J^T)^-1
  MatNd* invJWJt = MatNd_create(nx, nx);
  MatNd_mul(invJWJt, J, WJt);
  double det = MatNd_choleskyInverse(invJWJt, invJWJt);

  // Return if ill-conditioned
  if (det < eps)
  {
    RLOG(1, "Determinant %g < %g - setting gradient of body %s to 0",
         det, eps, b ? b->bdyName : "NULL");
    MatNd fA = MatNd_fromPtr(nq, nq, f);
    MatNd_setZero(&fA);
    MatNd_destroy(e);
    MatNd_destroy(J);
    MatNd_destroy(WJt);
    MatNd_destroy(invJWJt);
    return;
  }

  // Pseudoinverse
  MatNd* Jpinv = MatNd_create(nq, nx);
  MatNd_mul(Jpinv, WJt, invJWJt);

  // Hessian
  MatNd* dqJ = MatNd_create(nx * nq, nq);
  MatNd* dJpinv = MatNd_create(nq * nq, nx);
  RcsGraph_bodyPointHessian(self, b, pt, NULL, dqJ);

  // Pseudo-inverse derivative
  MatNd_PinvHessian(dJpinv, J, dqJ, invW, invJWJt, Jpinv, false);

  // Feedback error derivative dq(e) = -J for linear displacements
  MatNd* dqe = MatNd_create(nx, nq);
  MatNd_constMul(dqe, J, -1.0);

  // term1 = dq(J#) e
  MatNd* term1 = MatNd_create(nq * nq, 1);
  MatNd_mul(term1, dJpinv, e);
  MatNd_reshape(term1, nq, nq);

  // term2 = J# dq(e)
  MatNd* term2 = MatNd_create(nq, nq);
  MatNd_mul(term2, Jpinv, dqe);

  // dq[t+1]/dq[t] = E + term1 + term2
  MatNd fA = MatNd_fromPtr(nq, nq, f);
  MatNd_setIdentity(&fA);
  MatNd_subSelf(&fA, term1);
  MatNd_subSelf(&fA, term2);

  // Cleanup
  MatNd_destroy(e);
  MatNd_destroy(J);
  MatNd_destroy(WJt);
  MatNd_destroy(invJWJt);
  MatNd_destroy(Jpinv);
  MatNd_destroy(dqJ);
  MatNd_destroy(dJpinv);
  MatNd_destroy(dqe);
  MatNd_destroy(term1);
  MatNd_destroy(term2);
}



/******************************************************************************

  \brief q[t+1]        = q[t] + J# e -a*N*invW*dqH
         dq[t+1]/dq[t] =  E + dq(J#) e + J# dq(e) +
                          a { (dq(J#) J + J# dqJ) invW dH - N invW d2H }

******************************************************************************/

STATIC_FUNC void test_qprop(double* f, const double* x, void* params)
{
  unsigned int i;
  RcsGraph* self = (RcsGraph*) params;
  // double f1[self->nJ], f2[self->nJ];
  double* f1 = RNALLOC(self->nJ, double);
  double* f2 = RNALLOC(self->nJ, double);

  test_qprop_ts(f1, x, params);
  test_qprop_ns(f2, x, params);

  for (i = 0; i < self->nJ; i++)
  {
    f[i] = f1[i] + f2[i];
  }

  RFREE(f1);
  RFREE(f2);
}

STATIC_FUNC void test_dqprop(double* f, const double* x, void* params)
{
  unsigned int i;
  RcsGraph* self = (RcsGraph*) params;
  // double f1[self->nJ * self->nJ], f2[self->nJ * self->nJ];
  double* f1 = RNALLOC(self->nJ * self->nJ, double);
  double* f2 = RNALLOC(self->nJ * self->nJ, double);

  test_dqprop_ts(f1, x, params);
  test_dqprop_ns(f2, x, params);

  for (i = 0; i < self->nJ * self->nJ; i++)
  {
    f[i] = f1[i] + f2[i];
  }

  RFREE(f1);
  RFREE(f2);
}



/******************************************************************************

  \brief Computes the pseudo inverse of the linear Jacobian dx/dq for the last
         body that is connected to its predecessor with a joint:

         J# = Wq J^T (J Wq J^T)^-1

******************************************************************************/

STATIC_FUNC void test_jacobianPinv(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  Rcs_setIKState(self, x);
  MatNd* J = MatNd_create(3, self->nJ);
  MatNd J_pinv = MatNd_fromPtr(3, self->nJ, f);
  const double* pt = RCS_GRADIENTTESTS_BODYPT1, eps = 1.0e-3;

  RcsBody* b = NULL;
  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    if (BODY->jnt)
    {
      b = BODY;
    }
  }

  RcsGraph_bodyPointJacobian(self, b, pt, NULL, J);
  double det = MatNd_rwPinv(&J_pinv, J, NULL, NULL);

  // Return if ill-conditioned
  if (det < eps)
  {
    RLOG(4, "Determinant %g < %g - setting gradient of body %s to 0",
         det, eps, b ? b->bdyName : "NULL");
    MatNd_setZero(&J_pinv);
  }

  MatNd_destroy(J);
}



/******************************************************************************

  \brief Computes the partial derivative of the small (m x m) pseudo inverse.

         J#     = Wq J^T (J Wq J^T)^-1
         dq(J#) = -J# dq(J) J# + N invW del_q(J^T) (J invW J^T)^-1

******************************************************************************/

STATIC_FUNC void test_hessianPinv(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  Rcs_setIKState(self, x);

  double dt = Timer_getTime();

  int nx = 3, nq = self->nJ;
  RcsBody* b = NULL;
  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    if (BODY->jnt)
    {
      b = BODY;
    }
  }
  const double* pt = RCS_GRADIENTTESTS_BODYPT1, eps = 1.0e-3;

  // Jacobian
  MatNd* J = MatNd_create(nx, nq);
  RcsGraph_bodyPointJacobian(self, b, pt, NULL, J);

  // WJt = W*J^T
  MatNd* invW = NULL;
  MatNd* WJt = MatNd_create(nq, nx);
  MatNd_transpose(WJt, J);
  if (invW)
  {
    MatNd_preMulDiagSelf(WJt, invW);
  }

  // invJWJt = (J invW J^T)^-1
  MatNd* invJWJt = MatNd_create(nx, nx);
  MatNd_mul(invJWJt, J, WJt);
  double det = MatNd_choleskyInverse(invJWJt, invJWJt);

  // Return if ill-conditioned
  if (det < eps)
  {
    RLOG(4, "Determinant %g < %g - setting gradient of body %s to 0",
         det, eps, b ? b->bdyName : "NULL");
    memset(f, 0, nx * nq * nq * sizeof(double));
    MatNd_destroy(J);
    MatNd_destroy(WJt);
    MatNd_destroy(invJWJt);
    return;
  }

  // Pseudoinverse
  MatNd* J_pinv = MatNd_create(nq, nx);
  MatNd_mul(J_pinv, WJt, invJWJt);

  // Hessian
  MatNd* dqJ = MatNd_create(nx * nq, nq);
  RcsGraph_bodyPointHessian(self, b, pt, NULL, dqJ);

  // Pseudo-inverse derivative
  MatNd dJpinv = MatNd_fromPtr(nq * nq, nx, f);
  MatNd_PinvHessian(&dJpinv, J, dqJ, invW, invJWJt, J_pinv, false);

  MatNd_destroy(J);
  MatNd_destroy(WJt);
  MatNd_destroy(invJWJt);
  MatNd_destroy(J_pinv);
  MatNd_destroy(dqJ);

  dt = Timer_getTime() - dt;
  RLOG(5, "C inverse Hessian test took %g msec", 1.0e3 * dt);
}



/******************************************************************************

  \brief Computes the pseudo inverse of the linear Jacobian dx/dq for the last
         body that is connected to its predecessor with a joint:

         J# = (J^T invWx J + invWq)^-1 * J^T invWx

         Please note that the regularization matrix should have some decent
         positive values, since otherwise the pseudo-inverse gets
         ill-conditioned and its derivative might be numerically not very
         accurate.

******************************************************************************/

STATIC_FUNC void test_jacobianPinv2(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  Rcs_setIKState(self, x);
  MatNd* J     = MatNd_create(3, self->nJ);
  MatNd J_pinv = MatNd_fromPtr(3, self->nJ, f);
  const double* pt = RCS_GRADIENTTESTS_BODYPT1;
  const double lambda0 = 1.0e-3;

  RcsBody* b = NULL;
  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    if (BODY->jnt)
    {
      b = BODY;
    }
  }

  RcsGraph_bodyPointJacobian(self, b, pt, NULL, J);

  MatNd* invWx = NULL;
  MatNd* lambda = MatNd_create(self->nJ, 1);
  MatNd_setElementsTo(lambda, lambda0);
  double det = MatNd_rwPinv2(&J_pinv, J, invWx, lambda);
  MatNd_destroy(lambda);

  // Return if ill-conditioned
  if (det == 0.0)
  {
    RLOG(1, "Determinant is 0 - setting gradient of body %s to 0",
         b ? b->bdyName : "NULL");
    MatNd_setZero(&J_pinv);
  }

  MatNd_destroy(J);
}


/******************************************************************************

  \brief Computes the partial derivative of the large (n x n) pseudo inverse.

         J#     = (J^T invWx J + invWq)^-1 * J^T invWx
         dq(J#) = -J# dq(J) J# + inv(J^T invWx J) dq(J^T) invWx (E - J J#)

         Please note that the regularization matrix should have some decent
         positive values, since otherwise the pseudo-inverse gets
         ill-conditioned and its derivative might be numerically not very
         accurate.

******************************************************************************/

STATIC_FUNC void test_hessianPinv2(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  Rcs_setIKState(self, x);

  double dt = Timer_getTime();

  int nx = 3, nq = self->nJ;
  RcsBody* b = NULL;
  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    if (BODY->jnt)
    {
      b = BODY;
    }
  }
  const double* pt = RCS_GRADIENTTESTS_BODYPT1;
  const double lambda0 = 1.0e-3;

  // Jacobian
  MatNd* J = MatNd_create(nx, nq);
  RcsGraph_bodyPointJacobian(self, b, pt, NULL, J);

  // JtW = J^T*invWx
  MatNd* invWx = NULL;
  MatNd* JtW = MatNd_create(nq, nx);
  MatNd_transpose(JtW, J);
  if (invWx)
  {
    MatNd_postMulDiagSelf(JtW, invWx);
  }

  // ----------- invJWJt = (J invW J^T)^-1
  // invJtWJ = (J^T invWx J + lambda0 E)^-1
  MatNd* invJtWJ = MatNd_create(nq, nq);
  MatNd_mul(invJtWJ, JtW, J);
  MatNd_addConstToDiag(invJtWJ, lambda0);   // lambda
  double det = MatNd_choleskyInverse(invJtWJ, invJtWJ);

  // Return if ill-conditioned
  if (det == 0.0)
  {
    RLOG(1, "Determinant is 0 - setting gradient of body %s to 0",
         b ? b->bdyName : "NULL");
    memset(f, 0, nx * nq * nq * sizeof(double));
    MatNd_destroy(J);
    MatNd_destroy(JtW);
    MatNd_destroy(invJtWJ);
    return;
  }

  // Pseudoinverse J# = (J^T invWx J + lambda0 E)^-1 J^T invWx
  MatNd* J_pinv = MatNd_create(nq, nx);
  MatNd_mul(J_pinv, invJtWJ, JtW);

  // Hessian
  MatNd* dqJ = MatNd_create(nx * nq, nq);
  RcsGraph_bodyPointHessian(self, b, pt, NULL, dqJ);

  // Pseudo-inverse derivative
  MatNd dJpinv = MatNd_fromPtr(nq * nq, nx, f);
  MatNd_PinvHessian2(&dJpinv, J, dqJ, invWx, invJtWJ, J_pinv, false);

  MatNd_destroy(J);
  MatNd_destroy(JtW);
  MatNd_destroy(invJtWJ);
  MatNd_destroy(J_pinv);
  MatNd_destroy(dqJ);

  dt = Timer_getTime() - dt;
  RLOG(5, "C inverse Hessian test took %g msec", 1.0e3 * dt);
}



/******************************************************************************

  \brief Array x is assumed to have nJ elements. The function traverses all
         joints and sets the unconstrained joints to the respective value
         of the x array.

******************************************************************************/

void Rcs_setIKState(RcsGraph* self, const double* x)
{
  int k = 0;
  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    if (JNT->jacobiIndex != -1)
    {
      MatNd_set(self->q, JNT->jointIndex, 0, x[JNT->jacobiIndex]);
      k++;
    }
  }
  RCHECK(k == self->nJ);
  RcsGraph_setState(self, NULL, NULL);
}



/******************************************************************************

  \brief

******************************************************************************/

void Rcs_setGradientTestBdy1(RcsBody* b)
{
  RCS_GRADIENTTESTS_BODY1 = b;
}



/******************************************************************************

  \brief

******************************************************************************/

void Rcs_setGradientTestBdy2(RcsBody* b)
{
  RCS_GRADIENTTESTS_BODY2 = b;
}



/******************************************************************************

  \brief

******************************************************************************/

static void Rcs_setRndBdy1(RcsGraph* self)
{
  unsigned int nBodies = RcsGraph_numBodies(self);
  RCHECK(nBodies>0);

  unsigned int rnd1 = Math_getRandomInteger(0, nBodies-1);

#ifdef OLD_TOPO
  RCS_GRADIENTTESTS_BODY1 = self->root;

  for (unsigned int i = 0; i <= rnd1; i++)
  {
    RCS_GRADIENTTESTS_BODY1 =
      RcsBody_depthFirstTraversalGetNext(RCS_GRADIENTTESTS_BODY1);
  }
#else
  RCS_GRADIENTTESTS_BODY1 = &self->bodies[rnd1];
#endif

  RCHECK(RCS_GRADIENTTESTS_BODY1);
}



/******************************************************************************

  \brief

******************************************************************************/

static void Rcs_setRndBdy2(RcsGraph* self)
{
  unsigned int nBodies = RcsGraph_numBodies(self);
  RCHECK(nBodies>0);

  unsigned int rnd2 = Math_getRandomInteger(0, nBodies-1);

#ifdef OLD_TOPO
  RCS_GRADIENTTESTS_BODY2 = self->root;

  for (unsigned int i = 0; i < rnd2; i++)
  {
    RCS_GRADIENTTESTS_BODY2 =
      RcsBody_depthFirstTraversalGetNext(RCS_GRADIENTTESTS_BODY2);
  }
#else
  RCS_GRADIENTTESTS_BODY2 = &self->bodies[rnd2];
#endif

  RCHECK(RCS_GRADIENTTESTS_BODY2);
}



/******************************************************************************

  \brief

******************************************************************************/

static void Rcs_setRndBdy3(RcsGraph* self)
{
  unsigned int nBodies = RcsGraph_numBodies(self);
  RCHECK(nBodies>0);

  unsigned int rnd3 = Math_getRandomInteger(0, nBodies-1);

#ifdef OLD_TOPO
  RCS_GRADIENTTESTS_BODY3 = self->root;

  for (unsigned int i = 0; i < rnd3; i++)
  {
    RCS_GRADIENTTESTS_BODY3 =
      RcsBody_depthFirstTraversalGetNext(RCS_GRADIENTTESTS_BODY3);
  }
#else
  RCS_GRADIENTTESTS_BODY3 = &self->bodies[rnd3];
#endif

  RCHECK(RCS_GRADIENTTESTS_BODY3);
}



/******************************************************************************

  \brief

******************************************************************************/

RcsBody* Rcs_getRndBdy1(void)
{
  return RCS_GRADIENTTESTS_BODY1;
}



/******************************************************************************

  \brief

******************************************************************************/

RcsBody* Rcs_getRndBdy2(void)
{
  return RCS_GRADIENTTESTS_BODY2;
}



/******************************************************************************

  \brief Test for del_q { A1(q) * A2(q) }

    Here's the derivative function using the more readeable C++ arrays, just
    for reference:

    void test_dA(double *f, const double *x, void *params)
    {
      RcsGraph *self = (RcsGraph*) params;
      Rcs_setIKState(self, x);
      int n = self->nJ;

      arr A1(3,3), A2(3,3);
      VecN_copy(A1.p, &RCS_GRADIENTTESTS_BODY1->A_BI->rot[0][0], 9);
      VecN_copy(A2.p, &RCS_GRADIENTTESTS_BODY2->A_BI->rot[0][0], 9);

      // Term 1: A1 * dA2
      arr dA2(3,n,3);
      RcsGraph_dAdq(self, RCS_GRADIENTTESTS_BODY2, dA2.p);
      arr term1 = A1 * dA2;

      // Term 2: dA1 * A2
      arr dA1(9,n);
      RcsGraph_dAdq(self, RCS_GRADIENTTESTS_BODY1, dA1.p);
      dA1 = ~dA1;
      dA1.reshape(3,n,3);

      arr term2 = dA1*A2;
      term2.reshape(n,9);
      term2 = ~term2;
      term2.reshape(3,n,3);

      // Just add them together
      arr res = term1 + term2;

      VecN_copy(f, res.p, 3*3*n);
    }

******************************************************************************/

static void test_A(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  Rcs_setIKState(self, x);

  const HTr* A1_ = RCS_GRADIENTTESTS_BODY1 ? &RCS_GRADIENTTESTS_BODY1->A_BI : HTr_identity();
  const HTr* A2_ = RCS_GRADIENTTESTS_BODY2 ? &RCS_GRADIENTTESTS_BODY2->A_BI : HTr_identity();

  MatNd A1 = MatNd_fromMat3d((double (*)[3])A1_->rot);
  MatNd A2 = MatNd_fromMat3d((double (*)[3])A2_->rot);
  MatNd A3 = MatNd_fromPtr(3, 3, f);

  MatNd_mul(&A3, &A1, &A2);
}

static void test_dA(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  Rcs_setIKState(self, x);
  int n = self->nJ;

  const HTr* A1_ = RCS_GRADIENTTESTS_BODY1 ? &RCS_GRADIENTTESTS_BODY1->A_BI : HTr_identity();
  const HTr* A2_ = RCS_GRADIENTTESTS_BODY2 ? &RCS_GRADIENTTESTS_BODY2->A_BI : HTr_identity();

  MatNd A1 = MatNd_fromMat3d((double (*)[3])A1_->rot);
  MatNd A2 = MatNd_fromMat3d((double (*)[3])A2_->rot);

  // Term 1: A1 * dA2
  MatNd* dA2 = MatNd_create(3, 3 * n);
  RcsGraph_dAdq(self, RCS_GRADIENTTESTS_BODY2, dA2->ele, false);
  MatNd* term1 = MatNd_create(3, 3 * n);
  MatNd_mul(term1, &A1, dA2);   // term1 = 3 x (3*n)
  MatNd_reshape(term1, 3 * 3, n);

  // Term 2: dA1 * A2
  MatNd* dA1 = MatNd_create(3 * 3, n);
  RcsGraph_dAdq(self, RCS_GRADIENTTESTS_BODY1, dA1->ele, false);
  MatNd_transposeSelf(dA1); // n x 9
  MatNd_reshape(dA1, 3 * n, 3);

  MatNd* term2 = MatNd_create(3 * n, 3);
  MatNd_mul(term2, dA1, &A2);
  MatNd_reshape(term2, n, 3 * 3);
  MatNd_transposeSelf(term2); // 9 x n

  // Just add them together
  MatNd res = MatNd_fromPtr(3 * 3, n, f);
  MatNd_add(&res, term1, term2);

  // Clean up
  MatNd_destroy(dA1);
  MatNd_destroy(dA2);
  MatNd_destroy(term1);
  MatNd_destroy(term2);
}



/******************************************************************************

  \brief Gradient test functions for the partial derivatives of a bodie's
         rotation matrix wrt. the state vector. The body is randomly assigned
         by the global RCS_GRADIENTTESTS_BODY1 pointer.

******************************************************************************/

void test_rotMat(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  double Id[3][3];
  Mat3d_setIdentity(Id);

  Rcs_setIKState(self, x);

  const HTr* A_BI = RCS_GRADIENTTESTS_BODY1 ? &RCS_GRADIENTTESTS_BODY1->A_BI : HTr_identity();
  /* double* A_BI = RCS_GRADIENTTESTS_BODY1 ? */
  /*                (double*) RCS_GRADIENTTESTS_BODY1->A_BI.rot : (double*) Id; */

  memcpy(f, (double*) A_BI->rot, 9 * sizeof(double));
}

void test_rotMatDerivative(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  Rcs_setIKState(self, x);
  RcsGraph_dAdq(self, RCS_GRADIENTTESTS_BODY1, f, false);
}



/******************************************************************************

  \brief Computes the rotation Jacobian dom/dq for body
         RCS_GRADIENTTESTS_BODY1.

******************************************************************************/

void test_jacobianRot(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  Rcs_setIKState(self, x);
  MatNd J = MatNd_fromPtr(3, self->nJ, f);
  RcsGraph_rotationJacobian(self, RCS_GRADIENTTESTS_BODY1, NULL, &J);
}



/******************************************************************************

  \brief Computes the rotation Hessian d2om/dq2 for body
         RCS_GRADIENTTESTS_BODY1.

******************************************************************************/

void test_hessianRot(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  Rcs_setIKState(self, x);
  MatNd f_arr = MatNd_fromPtr(3*self->nJ, self->nJ, f);

  RcsGraph_rotationHessian(self, RCS_GRADIENTTESTS_BODY1, NULL, &f_arr);
}



/******************************************************************************

  \brief

  1_r_12 = A_1I * (I_r2 - I_r1)
  J      = A_1I * (JT_2  - JT_1 + r12 x JR_1
  H      = dq(A_1I)(J2-J1+r12xJR1) + A_1I(H2-H1+dq(r_12x)J_R1+(r_12 x)HR1)

******************************************************************************/

static void test_position3D(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  double v0[3], Id[3][3];

  Vec3d_setZero(v0);
  Mat3d_setIdentity(Id);
  Rcs_setIKState(self, x);

  RcsBody* ef       = RCS_GRADIENTTESTS_BODY2;
  RcsBody* refBdy   = RCS_GRADIENTTESTS_BODY1;
  RcsBody* refFrame = RCS_GRADIENTTESTS_BODY3;

  Vec3d_sub(f, ef ? ef->A_BI.org : v0, refBdy ? refBdy->A_BI.org : v0);

  if ((refFrame!=NULL) && (refFrame!=refBdy))
  {
    Vec3d_rotateSelf(f, refFrame->A_BI.rot);
  }
  else
  {
    Vec3d_rotateSelf(f, refBdy ? refBdy->A_BI.rot : Id);
  }

}

static void test_jacobian3D(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  Rcs_setIKState(self, x);

  const RcsBody* ef       = RCS_GRADIENTTESTS_BODY2;
  const RcsBody* refBdy   = RCS_GRADIENTTESTS_BODY1;
  const RcsBody* refFrame = RCS_GRADIENTTESTS_BODY3;

  MatNd J = MatNd_fromPtr(3, self->nJ, f);
  RcsGraph_3dPosJacobian(self, ef, refBdy, refFrame, &J);
}

static void test_hessian3D(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  Rcs_setIKState(self, x);

  const RcsBody* ef       = RCS_GRADIENTTESTS_BODY2;
  const RcsBody* refBdy   = RCS_GRADIENTTESTS_BODY1;
  const RcsBody* refFrame = RCS_GRADIENTTESTS_BODY3;

  MatNd H = MatNd_fromPtr(3*self->nJ, self->nJ, f);
  RcsGraph_3dPosHessian(self, ef, refBdy, refFrame, &H);
}

STATIC_FUNC void test_omegaJacobian3D(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  Rcs_setIKState(self, x);

  const RcsBody* ef       = RCS_GRADIENTTESTS_BODY2;
  const RcsBody* refBdy   = RCS_GRADIENTTESTS_BODY1;
  const RcsBody* refFrame = RCS_GRADIENTTESTS_BODY3;

  MatNd J = MatNd_fromPtr(3, self->nJ, f);
  RcsGraph_3dOmegaJacobian(self, ef, refBdy, refFrame, &J);
}

STATIC_FUNC void test_omegaHessian3D(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  Rcs_setIKState(self, x);

  const RcsBody* ef       = RCS_GRADIENTTESTS_BODY2;
  const RcsBody* refBdy   = RCS_GRADIENTTESTS_BODY1;
  const RcsBody* refFrame = RCS_GRADIENTTESTS_BODY3;

  MatNd H = MatNd_fromPtr(3 * self->nJ, self->nJ, f);
  RcsGraph_3dOmegaHessian(self, ef, refBdy, refFrame, &H);
}



/******************************************************************************

  \brief 1d version of above tests

******************************************************************************/
//! \todo re-activate tests for refFrame != refBdy
static void test_position1D(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  double v0[3], Id[3][3];

  Vec3d_setZero(v0);
  Mat3d_setIdentity(Id);
  Rcs_setIKState(self, x);

  RcsBody* ef       = RCS_GRADIENTTESTS_BODY2;
  RcsBody* refBdy   = RCS_GRADIENTTESTS_BODY1;
  RcsBody* refFrame = refBdy;//RCS_GRADIENTTESTS_BODY3;

  double r[3];
  Vec3d_sub(r, ef ? ef->A_BI.org : v0, refBdy ? refBdy->A_BI.org : v0);
  Vec3d_rotateSelf(r, refFrame ? refFrame->A_BI.rot : Id);
  *f = r[RCS_RNDXYZ];
}

static void test_jacobian1D(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  Rcs_setIKState(self, x);

  const RcsBody* ef       = RCS_GRADIENTTESTS_BODY2;
  const RcsBody* refBdy   = RCS_GRADIENTTESTS_BODY1;
  const RcsBody* refFrame = refBdy;//RCS_GRADIENTTESTS_BODY3;

  MatNd J = MatNd_fromPtr(1, self->nJ, f);
  RcsGraph_1dPosJacobian(self, ef, refBdy, refFrame, RCS_RNDXYZ, &J);
}

static void test_hessian1D(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  Rcs_setIKState(self, x);

  const RcsBody* ef       = RCS_GRADIENTTESTS_BODY2;
  const RcsBody* refBdy   = RCS_GRADIENTTESTS_BODY1;
  const RcsBody* refFrame = refBdy;//RCS_GRADIENTTESTS_BODY3;

  MatNd H = MatNd_fromPtr(self->nJ, self->nJ, f);
  RcsGraph_1dPosHessian(self, ef, refBdy, refFrame, RCS_RNDXYZ, &H);
}



/******************************************************************************

  \brief COG kinematics test function. Array x is the input state
         vector, array f is the 3d COG position. Argument param
         will be passed through, it points to a RcsGraph structure that is
         required to loop through the kinematics.

******************************************************************************/

static void test_COG(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  Rcs_setIKState(self, x);
  RcsGraph_COG(self, f);
}



/******************************************************************************

  \brief COG Jacobian test function. Array x is the input state
         vector, array f is the d x 3 COG Jacobian. Argument param
         will be passed through, it points to a RcsGraph structure that is
         required to loop through the kinematics.

******************************************************************************/

static void test_COGJ(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  Rcs_setIKState(self, x);
  MatNd J = MatNd_fromPtr(self->nJ, 3, f);
  RcsGraph_COGJacobian(self, &J);
}



/******************************************************************************

  \brief COG Hessian test function. Array x is the input state
         vector, array f is the d x 3 COG Jacobian. Argument param
         will be passed through, it points to a RcsGraph structure that is
         required to loop through the kinematics.

******************************************************************************/

static void test_COGH(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  Rcs_setIKState(self, x);
  RcsGraph_computeCOGHessian(self, f);
}



/******************************************************************************

  \brief Body point kinematics test function. Array x is the input state
         vector, array f is the 3d body point position. Argument param
         will be passed through, it points to a RcsGraph structure that is
         required to loop through the kinematics.

******************************************************************************/

static void test_jointLimit_Full(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  MatNd_fromArray(self->q, x, self->dof);
  RcsGraph_setState(self, NULL, NULL);
  *f = RcsGraph_jointLimitCost(self, RcsStateFull);
}

static void test_jointLimit_IK(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  Rcs_setIKState(self, x);
  *f = RcsGraph_jointLimitCost(self, RcsStateIK);
}



/******************************************************************************

  \brief Joint limit gradient test function. Array x is the input state
         vector, array f is the d x 1 collision gradient. Argument param
         will be passed through, it points to a RcsGraph structure that is
         required to loop through the kinematics.

******************************************************************************/

static void test_jointLimitJ_Full(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  MatNd_fromArray(self->q, x, self->dof);
  RcsGraph_setState(self, NULL, NULL);
  MatNd dH = MatNd_fromPtr(self->dof, 1, f);
  RcsGraph_jointLimitGradient(self, &dH, RcsStateFull);
}

static void test_jointLimitJ_IK(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  Rcs_setIKState(self, x);
  MatNd dH = MatNd_fromPtr(self->nJ, 1, f);
  RcsGraph_jointLimitGradient(self, &dH, RcsStateIK);
}



/******************************************************************************

  \brief Yoshikawa's Manipulability index

******************************************************************************/
static void test_manipulability(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  Rcs_setIKState(self, x);

  MatNd* J = MatNd_create(3, self->nJ);
  RcsGraph_bodyPointJacobian(self, RCS_GRADIENTTESTS_BODY1,
                             RCS_GRADIENTTESTS_BODYPT1, NULL, J);

  MatNd* W = MatNd_create(J->m, 1);

  for (unsigned int i=0; i<W->m; i++)
  {
    W->ele[i] = i+1;
  }
  MatNd_constMulSelf(W, 1.0/W->size);

  *f = MatNd_computeManipulabilityIndex(J, NULL);

  MatNd_destroy(J);
  MatNd_destroy(W);
}

static void test_manipulabilityJ(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  Rcs_setIKState(self, x);

  MatNd* J = MatNd_create(3, self->nJ);
  RcsGraph_bodyPointJacobian(self, RCS_GRADIENTTESTS_BODY1,
                             RCS_GRADIENTTESTS_BODYPT1, NULL, J);

  MatNd* H = MatNd_create(3*self->nJ, self->nJ);
  RcsGraph_bodyPointHessian(self, RCS_GRADIENTTESTS_BODY1,
                            RCS_GRADIENTTESTS_BODYPT1, NULL, H);

  MatNd* W = MatNd_create(J->m, 1);

  for (unsigned int i=0; i<W->m; i++)
  {
    W->ele[i] = i+1;
  }
  MatNd_constMulSelf(W, 1.0/W->size);

  MatNd grad = MatNd_fromPtr(1, self->nJ, f);
  MatNd_computeManipulabilityIndexGradient(&grad, J, H, NULL);

  MatNd_destroy(J);
  MatNd_destroy(H);
  MatNd_destroy(W);
}



/******************************************************************************

  \brief Joint limit Hessian test function. Array x is the input state
         vector, array f is the d x 1 collision gradient. Argument param
         will be passed through, it points to a RcsGraph structure that is
         required to loop through the kinematics.

******************************************************************************/

static void test_jointLimitH_Full(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  MatNd_fromArray(self->q, x, self->dof);
  RcsGraph_setState(self, NULL, NULL);
  MatNd dH = MatNd_fromPtr(self->dof, self->dof, f);
  RcsGraph_jointLimitHessian(self, &dH, RcsStateFull);
}

static void test_jointLimitH_IK(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  Rcs_setIKState(self, x);
  MatNd dH = MatNd_fromPtr(self->nJ, self->nJ, f);
  RcsGraph_jointLimitHessian(self, &dH, RcsStateIK);
}



/******************************************************************************

  \brief Body point kinematics test function. Array x is the input state
         vector, array f is the 3d body point position. Argument param
         will be passed through, it points to a RcsGraph structure that is
         required to loop through the kinematics.

******************************************************************************/

static void test_kinematics(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  Rcs_setIKState(self, x);
  RcsGraph_bodyPoint(RCS_GRADIENTTESTS_BODY1, RCS_GRADIENTTESTS_BODYPT1, f);
}



/******************************************************************************

  \brief Body Jacobian test function. Array x is the input state
         vector, array f is the 3 x dof body point Jacobian. Argument
         param will be passed through, it points to a RcsGraph structure
         that is required to loop through the kinematics.

******************************************************************************/

static void test_kinematicsJ(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  Rcs_setIKState(self, x);
  MatNd J = MatNd_fromPtr(3, self->nJ, f);
  RcsGraph_bodyPointJacobian(self, RCS_GRADIENTTESTS_BODY1,
                             RCS_GRADIENTTESTS_BODYPT1, NULL, &J);
}



/******************************************************************************

  \brief Computes the translational Hessian d2x/dq2 for a body point.

******************************************************************************/

static void test_kinematicsH(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  Rcs_setIKState(self, x);
  MatNd H = MatNd_fromPtr(3*self->nJ, self->nJ, f);
  RcsGraph_bodyPointHessian(self, RCS_GRADIENTTESTS_BODY1,
                            RCS_GRADIENTTESTS_BODYPT1, NULL, &H);
}



/******************************************************************************

  \brief Distance test function. Array x is the input state
         vector, array f is the scalar distance cost. Argument param
         will be passed through, it points to a RcsGraph structure that is
         required to loop through the kinematics.

******************************************************************************/

static void test_distance(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  RCHECK(self);
  Rcs_setIKState(self, x);
  *f = 0.0;

  if (RcsBody_numDistanceShapes(RCS_GRADIENTTESTS_BODY1) == 0 ||
      RcsBody_numDistanceShapes(RCS_GRADIENTTESTS_BODY2) == 0)
  {
    return;
  }

  double I_p1[3], I_p2[3];
  Vec3d_transRotate(I_p1, RCS_GRADIENTTESTS_BODY1->A_BI.rot,
                    RCS_GRADIENTTESTS_BODYPT1);
  Vec3d_addSelf(I_p1, RCS_GRADIENTTESTS_BODY1->A_BI.org);
  Vec3d_transRotate(I_p2, RCS_GRADIENTTESTS_BODY2->A_BI.rot,
                    RCS_GRADIENTTESTS_BODYPT2);
  Vec3d_addSelf(I_p2, RCS_GRADIENTTESTS_BODY2->A_BI.org);

  *f = Vec3d_distance(I_p1, I_p2);
}



/******************************************************************************

  \brief Distance gradient test function. Array x is the input state
         vector, array f is the d x 1 distance gradient. Argument param
         will be passed through, it points to a RcsGraph structure that is
         required to loop through the kinematics.

******************************************************************************/

static void test_distanceJ(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  Rcs_setIKState(self, x);
  MatNd dDdq = MatNd_fromPtr(self->nJ, 1, f);
  MatNd_setZero(&dDdq);

  if (RcsBody_numDistanceShapes(RCS_GRADIENTTESTS_BODY1) == 0 ||
      RcsBody_numDistanceShapes(RCS_GRADIENTTESTS_BODY2) == 0)
  {
    return;
  }

  double I_p1[3], I_p2[3];
  Vec3d_transRotate(I_p1, RCS_GRADIENTTESTS_BODY1->A_BI.rot,
                    RCS_GRADIENTTESTS_BODYPT1);
  Vec3d_addSelf(I_p1, RCS_GRADIENTTESTS_BODY1->A_BI.org);
  Vec3d_transRotate(I_p2, RCS_GRADIENTTESTS_BODY2->A_BI.rot,
                    RCS_GRADIENTTESTS_BODYPT2);
  Vec3d_addSelf(I_p2, RCS_GRADIENTTESTS_BODY2->A_BI.org);

  RcsBody_distanceGradient(self, RCS_GRADIENTTESTS_BODY1,
                           RCS_GRADIENTTESTS_BODY2, true, I_p1, I_p2, NULL, &dDdq);
}



/******************************************************************************

  \brief Distance Hessian test function. Array x is the input state
         vector, array f is the d x 1 distance gradient. Argument param
         will be passed through, it points to a RcsGraph structure that is
         required to loop through the kinematics.

******************************************************************************/

static void test_distanceH(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  Rcs_setIKState(self, x);
  memset(f, 0, self->nJ * self->nJ * sizeof(double));

  if (RcsBody_numDistanceShapes(RCS_GRADIENTTESTS_BODY1) == 0 ||
      RcsBody_numDistanceShapes(RCS_GRADIENTTESTS_BODY2) == 0)
  {
    RLOG(4, "[%s - %s]: Distance skipped, no shapes found",
         RCS_GRADIENTTESTS_BODY1->bdyName,
         RCS_GRADIENTTESTS_BODY2->bdyName);
    return;
  }

  double I_p1[3], I_p2[3];
  Vec3d_transRotate(I_p1, RCS_GRADIENTTESTS_BODY1->A_BI.rot,
                    RCS_GRADIENTTESTS_BODYPT1);
  Vec3d_addSelf(I_p1, RCS_GRADIENTTESTS_BODY1->A_BI.org);
  Vec3d_transRotate(I_p2, RCS_GRADIENTTESTS_BODY2->A_BI.rot,
                    RCS_GRADIENTTESTS_BODYPT2);
  Vec3d_addSelf(I_p2, RCS_GRADIENTTESTS_BODY2->A_BI.org);

  RcsBody_distanceHessian(self, RCS_GRADIENTTESTS_BODY1,
                          RCS_GRADIENTTESTS_BODY2, true, I_p1, I_p2, f);
}



/******************************************************************************

  \brief

******************************************************************************/
#if 0
static void test_CPdistance(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  RCHECK(self);
  Rcs_setIKState(self, x);
  *f = 0.0;

  if (RcsBody_numDistanceShapes(RCS_GRADIENTTESTS_BODY1) == 0 ||
      RcsBody_numDistanceShapes(RCS_GRADIENTTESTS_BODY2) == 0)
  {
    return;
  }

  *f = RcsBody_distance(RCS_GRADIENTTESTS_BODY1, RCS_GRADIENTTESTS_BODY2,
                        NULL, NULL);
}
#endif


/******************************************************************************

  \brief

******************************************************************************/
#if 0
static void test_CPdistanceJ(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  Rcs_setIKState(self, x);

  double d, I_cp1[3], I_cp2[3];
  const RcsBody* b1 = RCS_GRADIENTTESTS_BODY1;
  const RcsBody* b2 = RCS_GRADIENTTESTS_BODY2;
  MatNd dDdq = MatNd_fromPtr(self->nJ, 1, f);
  MatNd_setZero(&dDdq);

  if (RcsBody_numDistanceShapes(RCS_GRADIENTTESTS_BODY1) == 0 ||
      RcsBody_numDistanceShapes(RCS_GRADIENTTESTS_BODY2) == 0)
  {
    return;
  }

  d = RcsBody_distance(b1, b2, I_cp1, I_cp2);

  RcsBody_distanceGradient(self, b1, b2, d >= 0.0 ? true : false,
                           I_cp1, I_cp2, &dDdq);
}
#endif


/******************************************************************************

  \brief Distance Hessian test function. Array x is the input state
         vector, array f is the d x 1 distance gradient. Argument param
         will be passed through, it points to a RcsGraph structure that is
         required to loop through the kinematics.

******************************************************************************/
#if 0
/* static */ void test_CPdistanceH(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  Rcs_setIKState(self, x);
  memset(f, 0, self->nJ * self->nJ * sizeof(double));

  if (RcsBody_numDistanceShapes(RCS_GRADIENTTESTS_BODY1) == 0 ||
      RcsBody_numDistanceShapes(RCS_GRADIENTTESTS_BODY2) == 0)
  {
    return;
  }

  double d, I_cp1[3], I_cp2[3];
  const RcsBody* b1 = RCS_GRADIENTTESTS_BODY1;
  const RcsBody* b2 = RCS_GRADIENTTESTS_BODY2;

  d = RcsBody_distance(b1, b2, I_cp1, I_cp2);

  RcsBody_distanceHessian(self, b1, b2, d >= 0.0 ? true : false, I_cp1, I_cp2, f);
}
#endif


/******************************************************************************

  \brief Collision test function. Array x is the input state
         vector, array f is the scalar distance cost. Argument param
         will be passed through, it points to a RcsGraph structure that is
         required to loop through the kinematics.

******************************************************************************/

STATIC_FUNC void test_bodyCollision(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  RCHECK(self);
  Rcs_setIKState(self, x);
  *f = 0.0;

  if (RcsBody_numDistanceShapes(RCS_GRADIENTTESTS_BODY1) == 0 ||
      RcsBody_numDistanceShapes(RCS_GRADIENTTESTS_BODY2) == 0)
  {
    return;
  }

  *f = RcsBody_collisionCost(RCS_GRADIENTTESTS_BODY1,
                             RCS_GRADIENTTESTS_BODY2, 1.0);
}



/******************************************************************************

  \brief Collision gradient test function. Array x is the input state
         vector, array f is the d x 1 distance gradient. Argument param
         will be passed through, it points to a RcsGraph structure that is
         required to loop through the kinematics.

******************************************************************************/

STATIC_FUNC void test_bodyCollisionJ(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  Rcs_setIKState(self, x);
  MatNd dDdq = MatNd_fromPtr(self->nJ, 1, f);
  MatNd_setZero(&dDdq);
  double status;

  if (RcsBody_numDistanceShapes(RCS_GRADIENTTESTS_BODY1) == 0 ||
      RcsBody_numDistanceShapes(RCS_GRADIENTTESTS_BODY2) == 0)
  {
    return;
  }

  RcsBody_collisionGradient(self, RCS_GRADIENTTESTS_BODY1,
                            RCS_GRADIENTTESTS_BODY2, 1.0, &dDdq, &status);
}



/******************************************************************************

  \brief Collision gradient test function. Array x is the input state
         vector, array f is the d x 1 distance gradient. Argument param
         will be passed through, it points to a RcsGraph structure that is
         required to loop through the kinematics.

******************************************************************************/

/* static */ void test_bodyCollisionH(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  Rcs_setIKState(self, x);
  memset(f, 0, self->nJ * self->nJ * sizeof(double));

  if (RcsBody_numDistanceShapes(RCS_GRADIENTTESTS_BODY1) == 0 ||
      RcsBody_numDistanceShapes(RCS_GRADIENTTESTS_BODY2) == 0)
  {
    return;
  }

  RcsBody_collisionHessian(self, RCS_GRADIENTTESTS_BODY1,
                           RCS_GRADIENTTESTS_BODY2, 1.0, f);
}



/******************************************************************************

  \brief Align frustrum test function. Array x is the input state
         vector, array f is the scalar frustrum cost. Argument param
         will be passed through, it points to a RcsGraph structure that is
         required to loop through the kinematics.

******************************************************************************/

STATIC_FUNC void test_frustrum(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  Rcs_setIKState(self, x);
  double theta = 60.0 * (M_PI / 180.);
  double ratio = 0.5;
  *f = RcsGraph_pointFrustrumCost(RCS_GRADIENTTESTS_BODY1,
                                  RCS_GRADIENTTESTS_BODY2,
                                  theta, theta, ratio, ratio,
                                  RCS_GRADIENTTESTS_BODYPT1);
}



/******************************************************************************

  \brief Align frustrum gradient test function. Array x is the input state
         vector, array f is the d x 1 frustrum gradient. Argument param
         will be passed through, it points to a RcsGraph structure that is
         required to loop through the kinematics.

******************************************************************************/

STATIC_FUNC void test_frustrumJ(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  Rcs_setIKState(self, x);
  MatNd dDdq = MatNd_fromPtr(self->nJ, 1, f);
  double theta = 60.0 * (M_PI / 180.);
  double ratio = 0.5;
  RcsGraph_pointFrustrumGradient(self, RCS_GRADIENTTESTS_BODY1,
                                 RCS_GRADIENTTESTS_BODY2,
                                 theta, theta, ratio, ratio,
                                 RCS_GRADIENTTESTS_BODYPT1, &dDdq);
}



/******************************************************************************

  \brief Effort test function. Array x is the input state
         vector, array f is the scalar effort cost. Argument param
         will be passed through, it points to a RcsGraph structure that is
         required to loop through the kinematics.

******************************************************************************/

STATIC_FUNC void test_effort(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  Rcs_setIKState(self, x);
  MatNd F = MatNd_fromPtr(3, 1, RCS_GRADIENTTESTS_BODYPT2);

  *f = RcsGraph_staticEffort(self, RCS_GRADIENTTESTS_BODY1, &F, NULL,
                             RCS_GRADIENTTESTS_BODYPT1);
}



/******************************************************************************

  \brief Effort gradient test function. Array x is the input state
         vector, array f is the d x 1 effort gradient. Argument param
         will be passed through, it points to a RcsGraph structure that is
         required to loop through the kinematics.

******************************************************************************/

STATIC_FUNC void test_effortJ(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  Rcs_setIKState(self, x);
  MatNd F = MatNd_fromPtr(3, 1, RCS_GRADIENTTESTS_BODYPT2);
  MatNd dDdq = MatNd_fromPtr(self->nJ, 1, f);

  RcsGraph_staticEffortGradient(self, RCS_GRADIENTTESTS_BODY1, &F, NULL,
                                RCS_GRADIENTTESTS_BODYPT1, &dDdq);
}



/******************************************************************************

  \brief Euler error test function. Array x is the input state
         vector, array f is the scalar effort cost. Argument param
         will be passed through, it points to a RcsGraph structure that is
         required to loop through the kinematics.

******************************************************************************/

static void test_EulerError(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  double Id[3][3];
  Mat3d_setIdentity(Id);

  double(*A1_)[3] = RCS_GRADIENTTESTS_BODY1 ?
                    RCS_GRADIENTTESTS_BODY1->A_BI.rot : Id;
  double(*A2_)[3] = RCS_GRADIENTTESTS_BODY2 ?
                    RCS_GRADIENTTESTS_BODY2->A_BI.rot : Id;

  Rcs_setIKState(self, x);
  Mat3d_getEulerError(f, A2_, A1_);
}



/******************************************************************************

  \brief Euler error gradient test function. Array x is the input state
         vector, array f is the d x 1 effort gradient. Argument param
         will be passed through, it points to a RcsGraph structure that is
         required to loop through the kinematics.

******************************************************************************/

static void test_EulerErrorJ(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  double Id[3][3];
  MatNd grad = MatNd_fromPtr(3, self->nJ, f);

  double* A1_ = RCS_GRADIENTTESTS_BODY1 ?
                (double*) RCS_GRADIENTTESTS_BODY1->A_BI.rot : (double*) Id;
  double* A2_ = RCS_GRADIENTTESTS_BODY2 ?
                (double*) RCS_GRADIENTTESTS_BODY2->A_BI.rot : (double*) Id;

  Mat3d_setIdentity(Id);

  Rcs_setIKState(self, x);
  RcsGraph_dEulerErrorDq(self, RCS_GRADIENTTESTS_BODY2,
                         RCS_GRADIENTTESTS_BODY1, &grad);

  double err[3];
  Mat3d_getEulerError(err, (double(*)[3]) A2_, (double(*)[3]) A1_);
  RLOG(5, "Euler error is [ %g   %g   %g ]",
       err[0] * (M_PI / 180.), err[1] * (M_PI / 180.), err[2] * (M_PI / 180.));
}



/******************************************************************************

  \brief Linear position error test function:

         e = -x(bdy1)

         Array x is the input state vector, array f is the linear position.
         Argument param will be passed through, it points to a RcsGraph
         structure that is required to loop through the kinematics.

******************************************************************************/

static void test_positionError(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  double v0[3];

  Vec3d_setZero(v0);
  Rcs_setIKState(self, x);
  Vec3d_copy(f, RCS_GRADIENTTESTS_BODY1 ? RCS_GRADIENTTESTS_BODY1->A_BI.org : v0);
  Vec3d_constMulSelf(f, -1.0);
}



/******************************************************************************

  \brief Linear position gradient test function:

         de/dq = -J(bdy1)

         Array x is the input state vector, array f is the linear position.
         Argument param will be passed through, it points to a RcsGraph
         structure that is required to loop through the kinematics.

******************************************************************************/

static void test_positionErrorJ(double* f, const double* x, void* params)
{
  RcsGraph* self = (RcsGraph*) params;
  Rcs_setIKState(self, x);
  MatNd grad = MatNd_fromPtr(3, self->nJ, f);
  RcsGraph_bodyPointJacobian(self, RCS_GRADIENTTESTS_BODY1, NULL, NULL, &grad);
  MatNd_constMulSelf(&grad, -1.0);
}




/******************************************************************************

  \brief See header.

******************************************************************************/

bool Rcs_gradientTestGraph(RcsGraph* graph, const MatNd* q, bool verbose)
{
  bool success = true;
  double tolerance = 5.0e-2;   // We accept 5% error

  Rcs_setRndBdy1(graph);
  Rcs_setRndBdy2(graph);
  Rcs_setRndBdy3(graph);

  RCS_GRADIENTTESTS_BODYPT1[0] = Math_getRandomNumber(-0.1, 0.1);
  RCS_GRADIENTTESTS_BODYPT1[1] = Math_getRandomNumber(-0.1, 0.1);
  RCS_GRADIENTTESTS_BODYPT1[2] = Math_getRandomNumber(-0.1, 0.1);

  RCS_GRADIENTTESTS_BODYPT2[0] = Math_getRandomNumber(-0.1, 0.1);
  RCS_GRADIENTTESTS_BODYPT2[1] = Math_getRandomNumber(-0.1, 0.1);
  RCS_GRADIENTTESTS_BODYPT2[2] = Math_getRandomNumber(-0.1, 0.1);

  // Pseudo random number between 0 and 2
  RCS_RNDXYZ = (int)(3.0 * (rand() / (RAND_MAX + 1.0)));

  if (verbose==true)
  {
    RMSG("Testing gradients");
    /* RMSG("Random body 1=%s", RCS_GRADIENTTESTS_BODY1->bdyName); */
    /* RMSG("Random body 2=%s", RCS_GRADIENTTESTS_BODY2->bdyName); */
    /* RMSG("Random body 3=%s", RCS_GRADIENTTESTS_BODY3->bdyName); */
  }

  RcsGraph_setState(graph, q, NULL);

  REXEC(5)
  {
    double d = 0.0;

    if (RcsBody_numDistanceShapes(RCS_GRADIENTTESTS_BODY1) > 0 &&
        RcsBody_numDistanceShapes(RCS_GRADIENTTESTS_BODY2) > 0)
    {
      d = RcsBody_distance(RCS_GRADIENTTESTS_BODY1,
                           RCS_GRADIENTTESTS_BODY2, NULL, NULL, NULL);
    }

    RMSG("\n\n\t\tBodies \"%s\" - \"%s\" - \"%s\"\n\t\t%d dofs, distance is"
         " %f\n\t\tbodyPt1 (%g %g %g) \n\t\tbodyPt2 (%g %g %g)\n\t\t"
         "Random index: %d\n",
         RCS_GRADIENTTESTS_BODY1->bdyName,
         RCS_GRADIENTTESTS_BODY2->bdyName,
         RCS_GRADIENTTESTS_BODY3->bdyName,
         graph->nJ, d,
         RCS_GRADIENTTESTS_BODYPT1[0],
         RCS_GRADIENTTESTS_BODYPT1[1],
         RCS_GRADIENTTESTS_BODYPT1[2],
         RCS_GRADIENTTESTS_BODYPT2[0],
         RCS_GRADIENTTESTS_BODYPT2[1],
         RCS_GRADIENTTESTS_BODYPT2[2],
         RCS_RNDXYZ);
  }

  if (verbose==true)
  {
    fprintf(stderr, "Rotmat gradient:         ");
  }
  success = Rcs_testGradient(test_rotMat, test_rotMatDerivative, graph,
                             q->ele, graph->nJ, 9, tolerance, verbose) && success;


  if (verbose==true)
  {
    fprintf(stderr, "Rotmat gradient (rel):   ");
  }
  success = Rcs_testGradient(test_A, test_dA, graph,
                             q->ele, graph->nJ, 9, tolerance, verbose) && success;

  if (verbose==true)
  {
    fprintf(stderr, "3D translation Jacobian: ");
  }
  success = Rcs_testGradient(test_position3D, test_jacobian3D,
                             graph, q->ele, graph->nJ, 3, tolerance, verbose) && success;

  if (verbose==true)
  {
    fprintf(stderr, "3D translation Hessian:  ");
  }
  success = Rcs_testGradient(test_jacobian3D, test_hessian3D,
                             graph, q->ele, graph->nJ, 3 * graph->nJ, tolerance,
                             verbose) && success;

  if (verbose==true)
  {
    fprintf(stderr, "Manipulability grad. (IK):  ");
  }
  success = Rcs_testGradient(test_manipulability, test_manipulabilityJ,
                             graph, q->ele, graph->nJ, 1, tolerance, verbose) && success;


  if (verbose==true)
  {
    fprintf(stderr, "Joint limit gradient:    ");
  }
  success = Rcs_testGradient(test_jointLimit_Full, test_jointLimitJ_Full,
                             graph, q->ele, graph->dof, 1, tolerance, verbose) && success;

  if (verbose==true)
  {
    fprintf(stderr, "Joint limit Hessian:     ");
  }
  success = Rcs_testGradient(test_jointLimitJ_Full, test_jointLimitH_Full,
                             graph, q->ele, graph->dof, graph->dof, tolerance,
                             verbose) && success;

  if (verbose==true)
  {
    fprintf(stderr, "Joint limit grad. (IK):  ");
  }
  success = Rcs_testGradient(test_jointLimit_IK, test_jointLimitJ_IK,
                             graph, q->ele, graph->nJ, 1, tolerance, verbose) && success;

  if (verbose==true)
  {
    fprintf(stderr, "Joint limit Hess. (IK):  ");
  }
  success = Rcs_testGradient(test_jointLimitJ_IK, test_jointLimitH_IK,
                             graph, q->ele, graph->nJ, graph->nJ, tolerance,
                             verbose) && success;

  if (verbose==true)
  {
    fprintf(stderr, "Joint limit border:      ");
  }
  success = Rcs_testGradient(f_jlBorderFull, df_jlBorderFull,
                             graph, q->ele, graph->dof, 1, tolerance, verbose) && success;

  if (verbose==true)
  {
    fprintf(stderr, "Joint limit border (IK): ");
  }
  success = Rcs_testGradient(f_jlBorderIK, df_jlBorderIK,
                             graph, q->ele, graph->nJ, 1, tolerance, verbose) && success;

  if (verbose==true)
  {
    fprintf(stderr, "Position Jacobian:       ");
  }
  success = Rcs_testGradient(test_kinematics, test_kinematicsJ, graph,
                             q->ele, graph->nJ, 3, tolerance, verbose) && success;

  if (verbose==true)
  {
    fprintf(stderr, "Position Hessian:        ");
  }
  success = Rcs_testGradient(test_kinematicsJ, test_kinematicsH,
                             graph, q->ele, graph->nJ, 3 * graph->nJ, tolerance,
                             verbose) && success;

  if (verbose==true)
  {
    fprintf(stderr, "Rotational Hessian:      ");
  }
  success = Rcs_testGradient(test_jacobianRot, test_hessianRot,
                             graph, q->ele, graph->nJ, 3 * graph->nJ, tolerance,
                             verbose) && success;
  ;

  if (verbose==true)
  {
    fprintf(stderr, "COG Jacobian:            ");
  }
  success = Rcs_testGradient(test_COG, test_COGJ,
                             graph, q->ele, graph->nJ, 3, tolerance, verbose) && success;
  ;

  if (verbose==true)
  {
    fprintf(stderr, "GOG Hessian:             ");
  }
  success = Rcs_testGradient(test_COGJ, test_COGH,
                             graph, q->ele, graph->nJ, 3 * graph->nJ, tolerance,
                             verbose) && success;

  if (verbose==true)
  {
    fprintf(stderr, "3D translation Jacobian: ");
  }
  success = Rcs_testGradient(test_position3D, test_jacobian3D,
                             graph, q->ele, graph->nJ, 3, tolerance, verbose) && success;

  if (verbose==true)
  {
    fprintf(stderr, "3D translation Hessian:  ");
  }
  success = Rcs_testGradient(test_jacobian3D, test_hessian3D,
                             graph, q->ele, graph->nJ, 3 * graph->nJ, tolerance,
                             verbose) && success;

  if (verbose==true)
  {
    fprintf(stderr, "1D translation Jacobian: ");
  }
  success = Rcs_testGradient(test_position1D, test_jacobian1D,
                             graph, q->ele, graph->nJ, 1, tolerance, verbose) && success;

  if (verbose==true)
  {
    fprintf(stderr, "1D translation Hessian:  ");
  }
  success = Rcs_testGradient(test_jacobian1D, test_hessian1D,
                             graph, q->ele, graph->nJ, graph->nJ, tolerance,
                             verbose) && success;

  if (verbose==true)
  {
    fprintf(stderr, "3D omega Hessian:        ");
  }
  success = Rcs_testGradient(test_omegaJacobian3D, test_omegaHessian3D,
                             graph, q->ele, graph->nJ, 3 * graph->nJ, tolerance,
                             verbose) && success;

  if (verbose==true)
  {
    fprintf(stderr, "Euler err gradient:      ");
  }
  success = Rcs_testGradient(test_EulerError, test_EulerErrorJ,
                             graph, q->ele, graph->nJ, 3, tolerance, verbose) && success;

  if (verbose==true)
  {
    fprintf(stderr, "Position err gradient:   ");
  }
  success = Rcs_testGradient(test_positionError, test_positionErrorJ,
                             graph, q->ele, graph->nJ, 3, tolerance, verbose) && success;

  if (verbose==true)
  {
    fprintf(stderr, "Distance gradient:       ");
  }
  success = Rcs_testGradient(test_distance, test_distanceJ,
                             graph, q->ele, graph->nJ, 1, tolerance, verbose) && success;

  if (verbose==true)
  {
    fprintf(stderr, "Distance Hessian:        ");
  }
  success = Rcs_testGradient(test_distanceJ, test_distanceH,
                             graph, q->ele, graph->nJ, graph->nJ, tolerance,
                             verbose) && success;

  if (verbose==true)
  {
    fprintf(stderr, "Effort gradient :        ");
  }
  success = Rcs_testGradient(test_effort, test_effortJ,
                             graph, q->ele, graph->nJ, 1, tolerance, verbose) && success;

  if (verbose==true)
  {
    fprintf(stderr, "Goal attitude gradient:  ");
  }
  success = Rcs_testGradient(test_attitude3D, test_dAttitude3D,
                             graph, q->ele, graph->nJ, 1, tolerance, verbose) && success;



  /* fprintf(stderr, "Delta Rot. Hessian:      "); */
  /* success = Rcs_testGradient(test_dDeltaJRot, test_dDeltaHRot,  */
  /*        graph, q->ele, graph->nJ, 3*graph->nJ, tolerance) && success; */

  /* fprintf(stderr, "Delta Trans. Hessian:    "); */
  /* success = Rcs_testGradient(test_dDeltaJTrans, test_dDeltaHTrans,  */
  /*        graph, q->ele, graph->nJ, 3*graph->nJ, tolerance) && success; */

  /* fprintf(stderr, "CP Distance gradient:    "); */
  /* success = Rcs_testGradient(test_CPdistance, test_CPdistanceJ, */
  /*        graph, q->ele, graph->nJ, 1, tolerance) && success; */

  /* fprintf(stderr, "Collision gradient:      "); */
  /* success = Rcs_testGradient(test_bodyCollision, test_bodyCollisionJ, */
  /*        graph, q->ele, graph->nJ, 1, tolerance) && success; */

  /* fprintf(stderr, "Frustrum gradient :      "); */
  /* success = Rcs_testGradient(test_frustrum, test_frustrumJ, */
  /*        graph, q->ele, graph->nJ, 1, tolerance) && success; */

  /* fprintf(stderr, "J# e:                    "); */
  /* success = Rcs_testGradient(test_Jpinv_e, test_dJpinv_e, */
  /*        graph, q->ele, graph->nJ, graph->nJ, tolerance) && success; */

  /* fprintf(stderr, "Linear Pinv Hessian CPP: "); */
  /* success = Rcs_testGradient(test_jacobianPinvCPP, test_hessianPinvCPP, */
  /*        graph, q->ele, graph->nJ, 3*graph->nJ, tolerance) && success; */

  if (verbose==true)
  {
    fprintf(stderr, "Pinv Hessian:            ");
  }
  success = Rcs_testGradient(test_jacobianPinv, test_hessianPinv,
                             graph, q->ele, graph->nJ, 3*graph->nJ, tolerance,
                             verbose) && success;


  if (verbose==true)
  {
    fprintf(stderr, "Pinv Hessian2:           ");
  }
  success = Rcs_testGradient(test_jacobianPinv2, test_hessianPinv2,
                             graph, q->ele, graph->nJ, 3*graph->nJ, tolerance,
                             verbose) && success;

  /* fprintf(stderr, "Task space prop 3:       "); */
  /* success = Rcs_testGradient(test_qprop_ts3, test_dqprop_ts3, */
  /*        graph, q->ele, graph->nJ, graph->nJ, 1.0e-4) && success; */

  /* fprintf(stderr, "Task space prop C 6:     "); */
  /* success = Rcs_testGradient(test_qprop_ts, test_dqprop_ts, */
  /*        graph, q->ele, graph->nJ, graph->nJ, 1.0e-4) && success; */

  /* fprintf(stderr, "Null space prop CPP 6:   "); */
  /* success = Rcs_testGradient(test_qprop_nsCPP, test_dqprop_nsCPP, */
  /*        graph, q->ele, graph->nJ, graph->nJ, 1.0e-4) && success; */

  /* fprintf(stderr, "Null space prop C 6:     "); */
  /* success = Rcs_testGradient(test_qprop_ns, test_dqprop_ns, */
  /*        graph, q->ele, graph->nJ, graph->nJ, 1.0e-4) && success; */

  /* fprintf(stderr, "dq[t+1]/dq[t]:           "); */
  /* success = Rcs_testGradient(test_qprop, test_dqprop, */
  /*        graph, q->ele, graph->nJ, graph->nJ, 1.0e-4) && success; */



  // Here we seem to run into numerical problems due to the closest point
  // computation. Ideally, it is assumed that the closest point doesn't change
  // for very small increments. That's (I guess) not the case, which makes the
  // gradient tests fail. However, they work when using a fixed point. This can
  // be verified by commenting out the TEST_FIXED_CLOSESTPOINTS variable in
  // Rcs_distance.c. I'm now not sure what to do. It basically means that
  // our approximation d2d/dp2 isn't very good, since the finite differences
  // behave differently. Maybe it can be assumed to be zero, and we gain some
  // efficiency here? Or maybe I'm still missing something?
#if 0
  fprintf(stderr, "CP Distance Hessian:   ");
  success = Rcs_testGradient(test_CPdistanceJ, test_CPdistanceH,
                             graph, q->ele, graph->nJ, graph->nJ, tolerance,
                             verbose) && success;
#endif

#if 0
  fprintf(stderr, "Collision Hessian:     ");
  success = Rcs_testGradient(test_bodyCollisionJ, test_bodyCollisionH,
                             graph, q->ele, graph->nJ, graph->nJ, tolerance,
                             verbose) && success;
#endif

  if (verbose==true)
  {
    if (success==false)
    {
      RMSG("FAILURE");
      RPAUSE();
    }
    else
    {
      fprintf(stderr, "All gradient tests succeeded\n\n");
    }
  }

  return success;
}
