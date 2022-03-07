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

#include "Rcs_dynamics.h"
#include "Rcs_typedef.h"
#include "Rcs_kinematics.h"
#include "Rcs_macros.h"
#include "Rcs_math.h"



/*******************************************************************************
 *
 ******************************************************************************/
static inline const double* getDefaultGravity()
{
  static double g[3] = { 0.0, 0.0, -RCS_GRAVITY };
  return g;
}

/*******************************************************************************
 *
 ******************************************************************************/
static void runge_kutta_fehlberg_1_2(int nz,
                                     DirDynFunc FCN,
                                     void* param,
                                     double t,
                                     double dt,
                                     const double* x1,
                                     double* x2,
                                     double* err)
{
  double* k1 = RNALLOC(nz, double);
  double* k2 = RNALLOC(nz, double);
  double* tmp = RNALLOC(nz, double);

  (*FCN)(x1, param, k1, dt);

  // tmp is 1st-order approximation of x2
  for (int i = 0; i < nz; i++)
  {
    tmp[i] = x1[i] + dt*k1[i];
  }

  // Computation of k2 = dx(x2)
  (*FCN)(tmp, param, k2, dt);

  // Error estimate
  for (int i = 0; i < nz; i++)
  {
    const double x_ord1 = x1[i] + dt*k1[i];
    const double x_ord2 = x1[i] + 0.5*dt*(k1[i]+k2[i]);
    x2[i] = x_ord2;

    if (err)
    {
      err[i] = x_ord2-x_ord1;  // = 0.5*dt*(k2[i]-k1[i]);
    }
  }

  RFREE(k1);
  RFREE(k2);
  RFREE(tmp);
}

/*******************************************************************************
 * See IfM_Preprint_M_02_09.pdf page 18
 ******************************************************************************/
static void runge_kutta_fehlberg_2_3(int nz,
                                     DirDynFunc FCN,
                                     void* param,
                                     double t,
                                     double dt,
                                     const double* x1,
                                     double* x2,
                                     double* err)
{
  double* k1 = RNALLOC(nz, double);
  double* k2 = RNALLOC(nz, double);
  double* k3 = RNALLOC(nz, double);
  double* tmp = RNALLOC(nz, double);

  t = 0.0;

  // Computation of k1 = dx(x1)
  (*FCN)(x1, param, k1, dt);

  // tmp is 1st-order approximation of x2
  for (int i = 0; i < nz; i++)
  {
    tmp[i] = x1[i] + dt*k1[i];
  }

  // Computation of k2 = dx(x2)
  (*FCN)(tmp, param, k2, t+dt);

  // Computation of k3
  for (int i = 0; i < nz; i++)
  {
    tmp[i] = x1[i] + 0.25*dt*(k1[i]+k2[i]);
  }

  /* (*FCN)(tmp, param, k3, t+0.5*dt); */
  (*FCN)(tmp, param, k3, dt);

  // Error estimate
  for (int i = 0; i < nz; i++)
  {
    const double x_ord2 = x1[i] + (dt/2.0)*(k1[i]+k2[i]);
    const double x_ord3 = x1[i] + (dt/6.0)*(k1[i]+k2[i]+4.0*k3[i]);
    x2[i] = x_ord3;

    if (err)
    {
      err[i] = (x_ord3-x_ord2);
    }
  }

  RFREE(k1);
  RFREE(k2);
  RFREE(k3);
  RFREE(tmp);
}

/*******************************************************************************
 * Integration of differential equations of first order according to
 * Runge - Kutta - Fehlberg - Order 2 and 3. From:
 * Stoer, Bulirsch, Numerische Mathematik II,
 * Springer- Verlag, fifth edition, 2005, pp.135.
 * Parameters:
 *
 * n_gl    : Number of differential equations
 * *FCN    : Evaluation function for the differential equations
 * t       : Integration time
 * dt      : Integration time step
 * x1      : State at t
 * x2      : State at t+dt
 * err     : Estimated error at x2
 ******************************************************************************/
static void runge_kutta_2_3(int nz,
                            DirDynFunc FCN,
                            void* param,
                            double t,
                            double dt,
                            const double* x1,
                            double* x2,
                            double* err)
{
  // Integration constants
  //const double a1 = 1. / 4.;
  //const double a2 = 27. / 40.;
  const double b10 = 1. / 4.;
  const double b20 = -189. / 800.;
  const double b21 = 729. / 800.;
  const double b30 = 214. / 891.;
  const double b31 = 1. / 33.;
  const double b32 = 650. / 891.;

  const double c0 = 214. / 891.;
  const double c1 = 1. / 33.;
  const double c2 = 650. / 891.;

  const double cd0 = 533. / 2106.;
  const double cd2 = 800. / 1053.;
  const double cd3 = -1. / 78.;

  // Intermediate integrated states
  double* w0 = RNALLOC(nz, double);
  double* w1 = RNALLOC(nz, double);
  double* w2 = RNALLOC(nz, double);
  double* w3 = RNALLOC(nz, double);
  double* w4 = RNALLOC(nz, double);
  double* w5 = RNALLOC(nz, double);

  // 1st call to the equations of motion
  (*FCN)(x1, param, w0, dt);

  // 1st integration of the equations of motion
  for (int i = 0; i < nz; i++)
  {
    w5[i] = x1[i] + dt * (b10 * w0[i]);
  }

  // 2nd call to the equations of motion
  (*FCN)(w5, param, w1, dt);

  // 2nd integration of the equations of motion
  for (int i = 0; i < nz; i++)
  {
    w5[i] = x1[i] + dt * (b20 * w0[i] + b21 * w1[i]);
  }

  // 3rd call to the equations of motion
  (*FCN)(w5, param, w2, dt);

  // 3rd integration of the equations of motion
  for (int i = 0; i < nz; i++)
  {
    w5[i] = x1[i] + dt * (b30 * w0[i] + b31 * w1[i] + b32 * w2[i]);
  }

  // 4th call to the equations of motion
  (*FCN)(w5, param, w3, dt);

  // Computation of X2 with 2nd order strategy
  for (int i = 0; i < nz; i++)
  {
    x2[i] = x1[i] + dt * (c0 * w0[i] + c1 * w1[i] + c2 * w2[i]);
  }

  // Computation of X2 with 3rd order strategy
  for (int i = 0; i < nz; i++)
  {
    w4[i] = x1[i] + dt * (cd0 * w0[i] + cd2 * w2[i] + cd3 * w3[i]);
  }

  // Error estimation: err=w[4]-X2;
  for (int i = 0; i < nz; i++)
  {
    err[i] = w4[i] - x2[i];
  }

  RFREE(w0);
  RFREE(w1);
  RFREE(w2);
  RFREE(w3);
  RFREE(w4);
  RFREE(w5);
}

/*******************************************************************************
 *
 ******************************************************************************/
void integration_rkf23(DirDynFunc FCN,
                       void* param,
                       int nz,
                       double dt,
                       const double* x,
                       double* x2)
{
  runge_kutta_fehlberg_2_3(nz, FCN, param, 0.0, dt, x, x2, NULL);
}

/*******************************************************************************
 * Euler single step integration
 *
 *        FCN   : differential equations function
 *        nz    : number of differential equations
 *        dt    : Integration interval
 *        x     : initial state vector
 *        x2    : state vector after integration
 ******************************************************************************/
void integration_euler(DirDynFunc FCN,
                       void* param,
                       int nz,
                       double dt,
                       const double* x,
                       double* x2)
{
  int i;
  double* xp = RNALLOC(nz, double);

  (*FCN)(x, param, xp, dt);

  for (i = 0; i < nz; i++)
  {
    x2[i] = x[i] + xp[i] * dt;
  }

  RFREE(xp);
}

/*******************************************************************************
 * Integration of equations of motion between t1 and t2. This function performs
 * a step size adaptation. Parameters:
 *
 * FCN   : Evaluation function for the differential equations
 * nz    : Number of differential equations
 * t1    : Integration start time
 * t2    : Integration end time
 * dt_opt: optimal integration step size
 * x     : State at t
 * x2     : State at t t2
 * fehler_zul: Permissable integration error per dimension
 ******************************************************************************/
#define ALMOST_ZERO (1.0e-5)    // was 1.0e-8 and 1.0e-6

int integration_t1_t2(DirDynFunc FCN,
                      void* param,
                      int nz,
                      double t1,
                      double t2,
                      double* dt_opt,
                      const double* x,
                      double* x2,
                      double* fehler_zul)
{
  int j, anzahl_integrationsschritte = 0, anzahl_schaltpunktschritte;
  double max_fehler;
  double t  = t1;
  double dt = *dt_opt;
  double* x1  = RNALLOC(nz, double);
  double* err = RNALLOC(nz, double);

  memmove(x1, x, nz * sizeof(double));

  do
  {
    // dt_opt is assigned before clamping dt to the interval, since the step
    // size adaptation shall be performed based on the error bounds only, and
    // not based on any clamping.
    *dt_opt = dt;

    if (t + dt > t2)
    {
      dt = t2 - t;
    }
    anzahl_schaltpunktschritte = 0;

    do
    {
      anzahl_schaltpunktschritte += 1;
      anzahl_integrationsschritte += 1;

      //runge_kutta_fehlberg_1_2(nz, FCN, param, t, dt, x1, x2, err);
      //runge_kutta_fehlberg_2_3(nz, FCN, param, t, dt, x1, x2, err);
      runge_kutta_2_3(nz, FCN, param, t, dt, x1, x2, err);


      // Computation of max. error
      for (j = 0; j < nz; j++)
      {
        err[j] = fabs(err[j] / fehler_zul[j]);
      }

      max_fehler = pow(VecNd_maxEle(err, nz), (1.0 / 3.0));

      // Step size adaptation
      if (max_fehler < ALMOST_ZERO)
      {
        max_fehler = ALMOST_ZERO;
      }

      if (max_fehler <= 1.0)
      {
        t += dt;
        if (max_fehler < 0.22)
        {
          dt = dt * 4.0;
        }
        else
        {
          dt = 0.9 * dt / max_fehler;
        }
        // war auskommentiert
        // vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
        if ((t < t1 - 1.0e-10) && (t + dt > t1))
        {
          dt = t1 - t;
        }
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
      }
      else
      {
        dt = 0.9 * dt / max_fehler;
      }

      if (anzahl_schaltpunktschritte > 100)
      {
        RFATAL("Anzahl Schaltpunktschritte: %d => Abbruch",
               anzahl_schaltpunktschritte);
      }

    }
    while (max_fehler > 1.0);

    // Neuer Ausgangszustand
    memmove(x1, x2, nz * sizeof(double));
    if (anzahl_integrationsschritte == 10000)
    {
      RFATAL("Needed more than 10000 integration steps");
    }

  }
  while (t < (t2 - 1.0e-10));

  RFREE(x1);
  RFREE(err);

  return anzahl_integrationsschritte;
}

/*******************************************************************************
 * Local h-vector with respect to bodies center of gravity.
 * h_local(0,2) = 0
 * h_local(3,5) = (~om)*I_I*om;
 ******************************************************************************/
static void RcsBody_HVector(const RcsBody* bdy,
                            const double omPtr[3],
                            const double vPtr[3],
                            MatNd* hVec)
{
  MatNd_reshapeAndSetZero(hVec, 6, 1);

  if (bdy->m == 0.0)
  {
    return;
  }

  // Some intermediate terms
  double h1Buf[3];
  MatNd h1 = MatNd_fromPtr(3, 1, h1Buf);
  MatNd om = MatNd_fromPtr(3, 1, (double*) omPtr);
  //MatNd v  = MatNd_fromPtr(3, 1, (double*) vPtr);

  double tildeOmBuf[3][3];
  Mat3d_skew(tildeOmBuf, omPtr);
  MatNd tildeOm = MatNd_fromPtr(3, 3, &tildeOmBuf[0][0]);

  // Transform inertia matrix into inertial frame:
  double I_IBuf[3][3];
  MatNd I_I = MatNd_fromPtr(3, 3, &I_IBuf[0][0]);
  Mat3d_similarityTransform(I_IBuf, (double(*)[3])bdy->A_BI.rot,
                            (double(*)[3])bdy->Inertia.rot);

  // h_local(3,5) = (~om)*I*om;
  MatNd hPtr = MatNd_fromPtr(3, 1, &hVec->ele[3]);
  MatNd_mul(&h1, &I_I, &om);              // I*om
  MatNd_mul(&hPtr, &tildeOm, &h1);        // (~om)*I*om
}

/*******************************************************************************
 * Bodies local mass and inertia matrix with respect to center
 * of gravity. The inertia tensor needs to be projected into the
 * inertial frame, so this computation can not be done beforehand.
 *
 *         M(0,0,2,2) = m*E;
 *         M(0,3,2,5) = 0
 *         M(3,0,5,2) = 0
 *         M(3,3,5,5) = I_I;
 *
 ******************************************************************************/
static void RcsBody_massMatrix(const RcsBody* bdy, MatNd* MassMatrix)
{
  MatNd_reshapeAndSetZero(MassMatrix, 6, 6);

  if (bdy->m == 0.0)
  {
    return;
  }

  // Transform inertia matrix into inertial frame
  double I_I[3][3];
  Mat3d_similarityTransform(I_I, (double (*)[3])bdy->A_BI.rot,
                            (double (*)[3])bdy->Inertia.rot);

  for (int i = 0; i < 3; i++)
  {
    MatNd_set(MassMatrix, i, i, bdy->m);

    for (int j = 0; j < 3; j++)
    {
      MatNd_set(MassMatrix, i + 3, j + 3, I_I[i][j]);
    }
  }

}

/*******************************************************************************
 *
 *  double E = RcsGraph_computeKineticTerms(self, NULL, NULL, NULL);
 *
 *  There's some speed-up potential:
 *  - The body mass matrix is block-symmetric (or even diagonal) so that
 *    J^T*M_local*J can be computed more efficiently. TODO: Check if
 *    M_local is diagonal for inertia tensors wrt. COM. I guess yes.
 *  - The computation of the bodies linear velocity can be skipped, it
 *    is not required for the h-vector wrt. the COM.
 *  - The dot Jacobians are computed in a very poor way. They use the
 *    analytic Hessian, but there's many zero-multiplies. We should make
 *    a separate J-dot function similar to the Hessian and compute it more
 *    efficiently.
 *  - Probably the kinetic energy can be more efficiently computed within
 *    the body traversal than afterwards using q^T M q. This would also
 *    eliminate the construction of M when a null pointer is passed.
 *  - Some smaller arrays can be allocated on the stack (like the 6x1
 *    ones or so).
 ******************************************************************************/
double RcsGraph_computeKineticTerms(const RcsGraph* graph,
                                    const double gravityVec[3],
                                    MatNd* M_,
                                    MatNd* h,
                                    MatNd* F_gravity)
{
  int n = graph->nJ;
  double V_pot = 0.0, T_kin = 0.0, buf[63], I_r_com[3];
  const double* K_r_com = NULL;
  MatNd* M = M_;

  double g[3];
  Vec3d_copy(g, gravityVec ? gravityVec : getDefaultGravity());

  MatNd* q_dot = NULL;
  MatNd_clone2(q_dot, graph->q_dot);
  RcsGraph_stateVectorToIKSelf(graph, q_dot);

  MatNd T_kinArr = MatNd_fromPtr(1, 1, &T_kin);
  MatNd M_local  = MatNd_fromPtr(6, 6, &buf[0]);
  MatNd h_local  = MatNd_fromPtr(6, 1, &buf[36]);
  MatNd om       = MatNd_fromPtr(3, 1, &buf[42]);
  MatNd v        = MatNd_fromPtr(3, 1, &buf[45]);
  MatNd ta1      = MatNd_fromPtr(6, 1, &buf[48]);
  MatNd ta2      = MatNd_fromPtr(6, 1, &buf[54]);
  MatNd I_g      = MatNd_fromPtr(3, 1, &buf[60]);

  MatNd* ta3=NULL, *J=NULL, *Jq=NULL, *Jtpcom= NULL, *Jtp=NULL, *Ftmp=NULL;

  MatNd_create2(ta3, n, 1);
  MatNd_create2(J, 6, n);
  MatNd_create2(Jq, 6, n);
  MatNd_create2(Jtpcom, n, 3);
  MatNd_create2(Jtp, n, 6);
  MatNd_create2(Ftmp, n, 1);

  MatNd JT  = MatNd_fromPtr(3, n, MatNd_getRowPtr(J, 0));
  MatNd JR  = MatNd_fromPtr(3, n, MatNd_getRowPtr(J, 3));
  MatNd JTq = MatNd_fromPtr(3, n, MatNd_getRowPtr(Jq, 0));
  MatNd JRq = MatNd_fromPtr(3, n, MatNd_getRowPtr(Jq, 3));

  if (M_ == NULL)
  {
    MatNd_create2(M, n, n);
  }
  else
  {
    MatNd_reshapeAndSetZero(M, n, n);
  }

  if (h != NULL)
  {
    MatNd_reshapeAndSetZero(h, n, 1);
  }

  if (F_gravity != NULL)
  {
    MatNd_reshape(F_gravity, n, 1);
    MatNd_setZero(F_gravity);
  }


  RCSGRAPH_TRAVERSE_BODIES(graph)
  {
    // We don't consider the body if it has no mass
    if (BODY->m == 0.0)
    {
      continue;
    }

    // Compute linear and angular Jacobian of body COM in inertial frame
    K_r_com = BODY->Inertia.org;
    RcsGraph_bodyPointJacobian(graph, BODY, K_r_com, NULL, &JT);
    RcsGraph_rotationJacobian(graph, BODY, NULL, &JR);
    MatNd_transpose(Jtp, J);

    // Compute bodie's mass matrix M_local (inertial coordinates) and add it
    // to the overall mass and inertia matrix: M += J^T*M_local*J;
    RcsBody_massMatrix(BODY, &M_local);
    MatNd_sqrMulAndAddAtBA(M, J, &M_local);

    // Compute the bodie's h-vector and add it to the overall h-vector:
    if (h != NULL)
    {
      // 1. Compute the translation and rotation dot Jacobian
      RcsGraph_bodyPointDotJacobian(graph, BODY, K_r_com, NULL, q_dot, &JTq);
      RcsGraph_rotationDotJacobian(graph,  BODY, q_dot, &JRq);

      // 2. Compute bodie's angular and linear velocity in bodie's frame. We
      //    don't use the bodie's velocities, since they are not describing
      //    the COM, but the body frame.
      MatNd_mul(&om, &JR, q_dot);
      MatNd_mul(&v, &JT, q_dot);

      // 3. h += J^T*(M_local*J_q*q_dot + h_local);
      RcsBody_HVector(BODY, om.ele, v.ele, &h_local);
      MatNd_mul(&ta1, Jq, q_dot);      // J_q*q_dot
      MatNd_mul(&ta2, &M_local, &ta1); // M_local*J_q*q_dot
      MatNd_addSelf(&ta2, &h_local);   // M_local*J_q*q_dot + h_local
      MatNd_mul(ta3, Jtp, &ta2);       // J^T*(M_local*J_q*q_dot + h_local)
      MatNd_subSelf(h, ta3);
    }

    // Compute gravity forces: F_gravity += m*(J_com^T)*g
    if (F_gravity != NULL)
    {
      MatNd_transpose(Jtpcom, &JT);
      Vec3d_copy(I_g.ele, g);
      Vec3d_constMulSelf(I_g.ele, BODY->m);
      MatNd_mul(Ftmp, Jtpcom, &I_g);
      MatNd_addSelf(F_gravity, Ftmp);
    }

    // Add the bodies potential energy: V_bdy = m*g*r_z
    RcsGraph_bodyPoint(BODY, K_r_com, I_r_com);
    V_pot -= BODY->m * Vec3d_innerProduct(I_r_com, g);
  }

  // Compute kinetic energy: 0.5*(q_dot^T)*M*q_dot
  MatNd_sqrMulAtBA(&T_kinArr, q_dot, M);
  T_kin *= 0.5;

  // Clean up
  MatNd_destroy(J);
  MatNd_destroy(Jq);
  MatNd_destroy(Jtpcom);
  MatNd_destroy(Jtp);
  MatNd_destroy(Ftmp);
  MatNd_destroy(ta3);
  MatNd_destroy(q_dot);

  if (M_==NULL)
  {
    MatNd_destroy(M);
  }

  return V_pot + T_kin;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsGraph_computeGravityTorque(const RcsGraph* graph,
                                   const double gravityVec[3],
                                   MatNd* T_gravity)
{
  double g[3];
  Vec3d_copy(g, gravityVec ? gravityVec : getDefaultGravity());
  MatNd_reshape(T_gravity, graph->nJ, 1);

  MatNd* J_cog_tp = NULL;
  MatNd_create2(J_cog_tp, 3, graph->nJ);

  RcsGraph_COGJacobian(graph, J_cog_tp);
  MatNd_transposeSelf(J_cog_tp);
  MatNd F_g = MatNd_fromPtr(3, 1, g);
  MatNd_constMulSelf(&F_g, RcsGraph_mass(graph));
  MatNd_mul(T_gravity, J_cog_tp, &F_g);

  MatNd_destroy(J_cog_tp);
}

/*******************************************************************************
 *
 *  This code makes use of the block structure of the local mass matrices:
 *
 *  MatNd_copyBlock(M_local_u, M_local, 0, 0, 2, 2);
 *  MatNd_copyBlock(M_local_l, M_local, 3, 3, 5, 5);
 *  MatNd_sqrMulAndAddAtBA(M, &JT, M_local_u);
 *  MatNd_sqrMulAndAddAtBA(M, &JR, M_local_l);
 *
 *  It is only valid if the inertia tensor is represented with respect to
 *  the bodie's center of mass. However, it's a bit slower than the below
 *  code, which is weird, since the above code has a lot more (zero)
 *  multiplies.
 *
 ******************************************************************************/
void RcsGraph_computeMassMatrix(const RcsGraph* graph, MatNd* M)
{
  int n = graph->nJ;
  MatNd* M_local=NULL, *J=NULL;

  MatNd_create2(M_local, 6, 6);
  MatNd_create2(J, 6, n);
  MatNd JT = MatNd_fromPtr(3, n, MatNd_getRowPtr(J, 0));
  MatNd JR = MatNd_fromPtr(3, n, MatNd_getRowPtr(J, 3));
  MatNd_reshape(M, n, n);
  MatNd_setZero(M);

  RCSGRAPH_TRAVERSE_BODIES(graph)
  {
    // We don't consider the body if it has no mass
    if (BODY->m == 0.0)
    {
      continue;
    }

    // Compute linear and angular Jacobian of body COM in inertial frame
    RcsGraph_bodyPointJacobian(graph, BODY, BODY->Inertia.org, NULL, &JT);
    RcsGraph_rotationJacobian(graph, BODY, NULL, &JR);

    // Compute bodie's mass matrix M_local (in inertial coordinates) and add it
    // to the overall mass and inertia matrix: M += J^T*M_local*J;
    RcsBody_massMatrix(BODY, M_local);
    MatNd_sqrMulAndAddAtBA(M, J, M_local);
  }

  // Clean up
  MatNd_destroy(M_local);
  MatNd_destroy(J);
}

/*******************************************************************************
 *
 * Solves the multibody equations of motion for the joint accelerations:
 *
 * M(q) q_ddot + h(q,q_dot) + F_gravity + F_ext' = F_jnt
 *
 * with F_ext = F_ext' - F_jnt we get:
 *
 * M(q) q_ddot + h(q,q_dot) + F_gravity + F_ext = 0
 *
 * The signs follow the equations of the Springer handbook of robotics,
 * but not the variable names.
 *
 * q, q_dot, q_ddot: Generalized coordinates, velocities and accelerations
 *             The dimension is graph->nJ (number of unconstrained dof)
 * M:          Mass and inertia matrix
 * F_gravity:  Gravity force projected on generalized coordinates
 * h:          Coriolis vector
 * F_ext:      External forces projected on generalized coordinates
 *             The joint torque vector needs to be substracted: F_ext-F_jnt
 *
 * Currently F_jnt is projected on all unconstrained degrees of
 * freedom. Arrays F_ext and F_jnt can be pointers to NULL. In this
 * case, they are assumed to be zero.
 *
 * The function returns the overall energy of the system. It will
 * warn on debug level 1 if
 * - the Cholesky decomposition failed
 * - one of the elements of q_ddot is not finite
 *
 ******************************************************************************/
double Rcs_directDynamics(const RcsGraph* graph,
                          const MatNd* F_ext,
                          MatNd* q_ddot)
{
  RCHECK(graph);
  int n = graph->nJ;

  if (F_ext != NULL)
  {
    RCHECK((F_ext->m == n)  && (F_ext->n == 1));

    if (!MatNd_isFinite(F_ext))
    {
      MatNd_printCommentDigits("F_ext", F_ext, 5);
      RPAUSE_MSG("F_ext contains non-finite values");
    }
  }

  MatNd* M         = NULL;
  MatNd_create2(M, n, n);
  MatNd* F_gravity = NULL;
  MatNd_create2(F_gravity, n, 1);
  MatNd* h         = NULL;
  MatNd_create2(h, n, 1);
  MatNd* b         = NULL;
  MatNd_create2(b, n, 1);

  MatNd_reshapeAndSetZero(q_ddot, n, 1);

  // Compute mass matrix, h-vector and gravity forces
  double E = RcsGraph_computeKineticTerms(graph, NULL, M, h, F_gravity);

  // Assemble RHS vector b considering gravity and Coriolis forces
  MatNd_addSelf(b, F_gravity);
  MatNd_addSelf(b, h);

  // Add external forces comming in as arguments
  if (F_ext)
  {
    MatNd_addSelf(b, F_ext);
  }











#if 1
  // Compute kinematic constraint Lagrange Multipliers
  {
    const unsigned int nc = 6;   // Number of constraints

    const RcsBody* ef = RcsGraph_getBodyByName(graph, "LeftHand");
    RCHECK(ef);

    static bool init = false;
    static double x0[6];
    if (!init)
    {
      Vec3d_copy(x0, ef->A_BI.org);
      init = true;
    }

    // Constraint Jacobian
    MatNd* J = MatNd_create(nc, n);
    MatNd Jx = MatNd_fromPtr(3, J->n, MatNd_getRowPtr(J, 0));
    RcsGraph_bodyPointJacobian(graph, ef, NULL, NULL, &Jx);
    if (nc==6)
    {
      MatNd Jw = MatNd_fromPtr(3, J->n, MatNd_getRowPtr(J, 3));
      RcsGraph_rotationJacobian(graph, ef, NULL, &Jw);
    }

    MatNd* qp_ik2 = MatNd_clone(graph->q_dot);
    RcsGraph_stateVectorToIKSelf(graph, qp_ik2);

    // Constraint dot-Jacobian
    MatNd* J_dot = MatNd_createLike(J);
    MatNd J_dot_x = MatNd_fromPtr(3, J_dot->n, MatNd_getRowPtr(J_dot, 0));
    RcsGraph_bodyPointDotJacobian(graph, ef, NULL, NULL, qp_ik2, &J_dot_x);
    if (nc==6)
    {
      MatNd J_dot_w = MatNd_fromPtr(3, J_dot->n, MatNd_getRowPtr(J_dot, 3));
      RcsGraph_rotationDotJacobian(graph, ef, qp_ik2, &J_dot_w);
    }

    // J inv(M)
    MatNd* invM = MatNd_createLike(M);
    double det = MatNd_choleskyInverse(invM, M);
    RCHECK(det != 0.0);
    MatNd* JinvM = MatNd_create(nc, n);
    MatNd_mul(JinvM, J, invM);

    // inv(J inv(M) JT)
    MatNd* invJinvMJT = MatNd_create(nc, nc);
    MatNd_sqrMulABAt(invJinvMJT, J, invM);
    det = MatNd_choleskyInverse(invJinvMJT, invJinvMJT);
    RCHECK(det != 0.0);

    // term1: -J inv(M) b
    MatNd* term1 = MatNd_create(nc, 1);
    MatNd_mul(term1, JinvM, b);
    MatNd_constMulSelf(term1, -1.0);

    // term1 += J_dot q_dot
    MatNd_constMulSelf(qp_ik2, -1.0);// Hack: Where does the minus come from? But so it looks correct
    MatNd_mulAndAddSelf(term1, J_dot, qp_ik2);

    // term 3: stabilization, either PD control law or other like Baumgarte
    // Stabilization: ax = kp*dx + kd*dx_dot
    const double kp = 0.0;
    const double kd = 0.5*sqrt(4.0*kp);
    MatNd* ax, * dx_c, *dxd_c;
    MatNd_fromStack(ax, nc, 1);
    MatNd_fromStack(dx_c, nc, 1);
    MatNd_fromStack(dxd_c, nc, 1);
    Vec3d_sub(dx_c->ele, x0, ef->A_BI.org);
    Vec3d_sub(dxd_c->ele, Vec3d_zeroVec(), ef->x_dot);

    // Orientation feedback missing
    if (nc==6)
    {
      Vec3d_sub(&dxd_c->ele[3], Vec3d_zeroVec(), ef->omega);
    }

    for (size_t i=0; i<nc; ++i)
    {
      ax->ele[i] = kp*dx_c->ele[i] + kd*dxd_c->ele[i];
    }

    RLOG(0, "x0: %f %f %f", x0[0], x0[1], x0[2]);
    RLOG(0, "dx: %f %f %f", dx_c->ele[0], dx_c->ele[1], dx_c->ele[2]);
    RLOG(0, "FootL: %f %f %f", ef->A_BI.org[0], ef->A_BI.org[1], ef->A_BI.org[2]);

    MatNd_addSelf(term1, ax);

    // Lagrange Multipliers in constraint space:
    // lambda = inv(J inv(M) JT) (-J inv(M) b + J_dot q_dot)
    MatNd* lambda = MatNd_create(nc, 1);
    MatNd_mul(lambda, invJinvMJT, term1);

    // Project into joint space and add to RHS vector: b += JT lambda
    MatNd_transposeSelf(J);
    MatNd_mulAndAddSelf(b, J, lambda);

    // Clean up
    MatNd_destroy(lambda);
    MatNd_destroy(term1);
    MatNd_destroy(qp_ik2);
    MatNd_destroy(invJinvMJT);
    MatNd_destroy(JinvM);
    MatNd_destroy(invM);
    MatNd_destroy(J_dot);
    MatNd_destroy(J);
  }
#endif















  // Solve direct dynamics for joint accelerations: q_ddot = inv(M) b
  double det = MatNd_choleskySolve(q_ddot, M, b);

  // Use Singular Value Decomposition if system is ill-conditioned. That's
  // sometimes the case if the rigid body Euler angles are in a configuration
  // singularity, or if the mass and inertia properties are screwed up.
  if (det==0.0)
  {
    RLOG(1, "Solving for q_ddot with Cholesky decomposition failed - trying "
         "again with mass matrix regularization");

    REXEC(4)
    {
      MatNd_printCommentDigits("Mass matrix", M, 12);
      MatNd_printCommentDigits("Sum of torques", b, 3);
    }

    MatNd_addConstToDiag(M, 1.0e-3);
    det = MatNd_choleskySolve(q_ddot, M, b);
  }

  // Warn if something seriously goes wrong.
  if (det == 0.0)
  {

    REXEC(3)
    {
      MatNd_printCommentDigits("Mass matrix", M, 4);
    }

    RLOG(2, "Couldn't solve direct dynamics - determinant is 0");
  }

  if (!MatNd_isFinite(q_ddot))
  {
    REXEC(3)
    {
      MatNd_printCommentDigits("q", graph->q, 4);
      MatNd_printCommentDigits("q_dot", graph->q_dot, 4);
      MatNd_printCommentDigits("q_ddot", q_ddot, 4);
      MatNd_printCommentDigits("h", h, 4);
      MatNd_printCommentDigits("F_gravity", F_gravity, 4);

      if (F_ext != NULL)
      {
        MatNd_printCommentDigits("F_ext", F_ext, 4);
      }

      MatNd_printCommentDigits("b", b, 4);
      MatNd_printCommentDigits("M", M, 2);

      MatNd* invM = MatNd_create(M->m, M->n);
      double det = MatNd_choleskyInverse(invM, M);
      MatNd_printCommentDigits("invM", invM, 2);
      MatNd_destroy(invM);

      if (det==0.0)
      {
        RMSGS("Determinant of mass matrix is 0");
      }
    }

    MatNd_setZero(q_ddot);
    RPAUSE_MSG("q_ddot has infinite values - det was %g", det);
  }

  // Clean up
  MatNd_destroy(M);
  MatNd_destroy(F_gravity);
  MatNd_destroy(h);
  MatNd_destroy(b);

  return E;
}

/******************************************************************************
 * Wrapper function of the direct dynamics to match the function
 * signature of the integrator. Argument param is assumed to point to a
 * RcsGraph structure. It's userData field is assumed to point to an
 * MatNd holding the external forces projected into the configuration space.
 ******************************************************************************/
double Rcs_directDynamicsIntegrationStep(const double* x,
                                         void* param,
                                         double* xp,
                                         double time)
{
  DirDynParams* p = (DirDynParams*) param;
  RcsGraph* graph = p->graph;
  MatNd* F_ext    = p->F_ext;
  int n           = graph->nJ;
  MatNd q         = MatNd_fromPtr(n, 1, (double*) &x[0]);
  MatNd q_dot     = MatNd_fromPtr(n, 1, (double*) &x[n]);
  MatNd q_dot_out = MatNd_fromPtr(n, 1, (double*) &xp[0]);
  MatNd q_ddot    = MatNd_fromPtr(n, 1, &xp[n]);

  RcsGraph_setState(graph, &q, &q_dot);

  double energy = Rcs_directDynamics(graph, F_ext, &q_ddot);

  MatNd_copy(&q_dot_out, &q_dot);

  // Set the speed of the coupled joints to zero. Otherwise, they will get
  // integrated and drift away.
  RCSGRAPH_TRAVERSE_JOINTS(graph)
  {
    if ((JNT->coupledToId!=-1) && (JNT->jacobiIndex!=-1))
    {
      q_dot_out.ele[JNT->jacobiIndex] = 0.0;
      q_ddot.ele[JNT->jacobiIndex] = 0.0;
    }
  }

  return energy;
}
