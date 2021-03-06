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
 * Integration of differential equations of first order according to
 * Runge - Kutta - Fehlberg - Order 2 and 2. From:
 * Stoer, Bulirsch, Numerische Mathematik II,
 * Springer- Verlag, fifth edition, 2005, pp.135.
 * Parameters:
 *
 * n_gl    : Number of differential equations
 * *FCN    : Evaluation function for the differential equations
 * t       : Integration time
 * dt      : Integration time step
 * x1      : State at t
 * x2      : Atate at t+dt
 * err     : Estimated error at x2
 ******************************************************************************/
void runge_kutta_2_3(int nz,
                     void (*FCN)(const double*, void*, double*, double),
                     void* param,
                     double t,
                     double dt,
                     const double* x1,
                     double* x2,
                     double* err,
                     int* init)
{
  static bool initRK    = false;
  static double t_merk  = 0.0;
  static double dt_merk = 0.0;
  static int init_test  = false;

  // Memorize configurationen
  static double* XP_merk1 = NULL;
  static double* XP_merk2 = NULL;

  if (!initRK)
  {
    initRK    = true;
    t_merk    = t;
    dt_merk   = dt;
    init_test = *init;
    XP_merk1  = RNALLOC(nz, double);
    XP_merk2  = RNALLOC(nz, double);
    if (!init_test)
    {
      RFATAL("Integrator not initialized!");
    }
  }

  // Konstanten des Integrationsverfahrens
  //double a1, a2;
  double b10, b20, b21, b30, b31, b32, c0, c1, c2, cd0, cd2, cd3;
  //a1 = 1. / 4.;
  //a2 = 27. / 40.;
  b10 = 1. / 4.;
  b20 = -189. / 800.;
  b21 = 729. / 800.;
  b30 = 214. / 891.;
  b31 = 1. / 33.;
  b32 = 650. / 891.;

  c0 = 214. / 891.;
  c1 = 1. / 33.;
  c2 = 650. / 891.;

  cd0 = 533. / 2106.;
  cd2 = 800. / 1053.;
  cd3 = -1. / 78.;

  // Ergebnisvektor und Hilfsvektoren
  double* w0 = RNALLOC(nz, double);
  double* w1 = RNALLOC(nz, double);
  double* w2 = RNALLOC(nz, double);
  double* w3 = RNALLOC(nz, double);
  double* w4 = RNALLOC(nz, double);
  double* w5 = RNALLOC(nz, double);

  if (init == 0)
  {
    // no complete re-initialization
    if (fabs(t_merk - t) < dt_merk / 10.)
    {
      // Integration failed -> new initial gradient
      memmove(w0, XP_merk1, nz * sizeof(double));
    }
    else
    {
      // Integration succeeded -> previous gradient is initial gradient
      memmove(w0, XP_merk2, nz * sizeof(double));
      memmove(XP_merk1, XP_merk2, nz * sizeof(double));
    }
  }
  else
  {
    // Re-initialization after impact etc.
    // 1st call to the equations of motion
    (*FCN)(x1, param, XP_merk1, dt);
    memmove(w0, XP_merk1, nz * sizeof(double));
    init = 0;
  }

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

  // Memorize last system state
  memmove(XP_merk2, w3, nz * sizeof(double));
  t_merk  = t;
  dt_merk = dt;

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
 * Euler single step integration
 *
 *        FCN   : differential equations function
 *        nz    : number of differential equations
 *        dt    : Integration interval
 *        x     : initial state vector
 *        x2    : state vector after integration
 ******************************************************************************/
void integration_euler(void (*FCN)(const double*, void*, double*, double),
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

int integration_t1_t2(void (*FCN)(const double*, void*, double*, double),
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
  static int init = 1;
  double t  = t1;
  double dt = *dt_opt;
  double* x1  = RNALLOC(nz, double);
  double* err = RNALLOC(nz, double);

  memmove(x1, x, nz * sizeof(double));

  do
  {
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

      runge_kutta_2_3(nz, FCN, param, t, dt, x1, x2, err, &init);

      // Computation of max. error
      for (j = 0; j < nz; j++)
      {
        err[j] = err[j] / fehler_zul[j];
      }
      max_fehler = pow(VecNd_maxAbsEle(err, nz), (1.0 / 3.0));

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
  MatNd_reshape(hVec, 6, 1);
  MatNd_setZero(hVec);

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
  Mat3d_similarityTransform(I_IBuf, bdy->A_BI->rot, bdy->Inertia->rot);

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
  MatNd_reshape(MassMatrix, 6, 6);
  MatNd_setZero(MassMatrix);

  if (bdy->m == 0.0)
  {
    return;
  }

  // Transform inertia matrix into inertial frame
  double I_I[3][3];
  Mat3d_similarityTransform(I_I, bdy->A_BI->rot, bdy->Inertia->rot);

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
                                    MatNd* M_,
                                    MatNd* h,
                                    MatNd* F_gravity)
{
  int n = graph->nJ;
  double V_pot = 0.0, T_kin = 0.0, buf[63], I_r_com[3];
  const double* K_r_com = NULL;
  MatNd* M = M_;

  MatNd* q_dot = NULL;
  MatNd_clone2(q_dot, graph->q_dot);
  RcsGraph_stateVectorToIKSelf(graph, q_dot);

  MatNd T_kinArr = MatNd_fromPtr(1, 1, &T_kin);
  MatNd M_local  = MatNd_fromPtr(6, 6, &buf[0]);
  MatNd h_local  = MatNd_fromPtr(6, 1, &buf[36]);
  //MatNd om       = MatNd_fromPtr(3, 1, &buf[42]);
  //MatNd v        = MatNd_fromPtr(3, 1, &buf[45]);
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
    MatNd_reshape(M, n, n);
    MatNd_setZero(M);
  }

  if (h != NULL)
  {
    MatNd_reshape(h, n, 1);
    MatNd_setZero(h);
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
    K_r_com = BODY->Inertia->org;
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

      // 2. Compute bodie's angular and linear velocity in bodie's frame
      //MatNd_mul(&om, &JR, q_dot);
      //MatNd_mul(&v, &JT, q_dot);

      // 3. h += J^T*(M_local*J_q*q_dot + h_local);
      RcsBody_HVector(BODY, BODY->omega, BODY->x_dot, &h_local);
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
      Vec3d_set(I_g.ele, 0.0, 0.0, -RCS_GRAVITY*BODY->m);
      MatNd_mul(Ftmp, Jtpcom, &I_g);
      MatNd_addSelf(F_gravity, Ftmp);
    }

    // Add the bodies potential energy: V_bdy = m*g*r_z
    RcsGraph_bodyPoint(BODY, K_r_com, I_r_com);
    V_pot += BODY->m * RCS_GRAVITY * I_r_com[2];
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
                                   MatNd* T_gravity)
{
  MatNd_reshape(T_gravity, graph->nJ, 1);

  MatNd* J_cog_tp = NULL;
  MatNd_create2(J_cog_tp, 3, graph->nJ);

  RcsGraph_COGJacobian(graph, J_cog_tp);
  MatNd_transposeSelf(J_cog_tp);
  MatNd* F_g = NULL;
  MatNd_create2(F_g, 3,1);
  MatNd_set(F_g, 2, 0, -RcsGraph_mass(graph)*RCS_GRAVITY);
  MatNd_mul(T_gravity, J_cog_tp, F_g);

  MatNd_destroy(J_cog_tp);
  MatNd_destroy(F_g);
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
    RcsGraph_bodyPointJacobian(graph, BODY, BODY->Inertia->org, NULL, &JT);
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
 *   Solves the multibody equations of motion for the joint
 *   accelerations:
 *
 *   M(q) q_ddot + h(q,q_dot) + F_gravity + F_ext = F_jnt
 *
 *   The signs follow the equations of the Springer handbook of robotics,
 *   but not the variable names.
 *
 *   q, q_dot, q_ddot: Generalized coordinates, velocities and accelerations
 *               The dimension is graph->nJ (number of unconstrained dof)
 *   M:          Mass and inertia matrix
 *   F_gravity:  Gravity force projected on generalized coordinates
 *   h:          Coriolis vector
 *   F_ext:      External forces projected on generalized coordinates
 *   F_jnt:      Joint torque vector
 *
 *   Currently F_jnt is projected on all unconstrained degrees of
 *   freedom. Arrays F_ext and F_jnt can be pointers to NULL. In this
 *   case, they are assumed to be zero.
 *
 *   The function returns the overall energy of the system. It will
 *   warn on debug level 1 if
 *   - the Cholesky decomposition failed
 *   - one of the elements of q_ddot is not finite
 *
 ******************************************************************************/
double Rcs_directDynamics(const RcsGraph* graph,
                          const MatNd* F_ext,
                          const MatNd* F_jnt,
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

  if (F_jnt != NULL)
  {
    RCHECK((F_jnt->m == n) && (F_jnt->n == 1));

    if (!MatNd_isFinite(F_jnt))
    {
      MatNd_printCommentDigits("F_jnt", F_jnt, 5);
      RPAUSE_MSG("F_jnt contains non-finite values");
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
  double E = RcsGraph_computeKineticTerms(graph, M, h, F_gravity);

  // Solve direct dynamics for joint accelerations
  MatNd_addSelf(b, F_gravity);

  if (F_ext)
  {
    MatNd_addSelf(b, F_ext);
  }

  if (F_jnt)
  {
    MatNd_subSelf(b, F_jnt);
  }

  MatNd_addSelf(b, h);
  double det = MatNd_choleskySolve(q_ddot, M, b);

  // Use Singular Value Decomposition if system is ill-conditioned. That's
  // sometimes the case if the rigid body Euler angles are in a configuration
  // singularity, or if the mass and inertia properties are screwed up.
  if (det==0.0)
  {
    //det = MatNd_SVDSolve(q_ddot, M, b);
    RLOG(1, "Solving for q_ddot with Cholesky decomposition failed - using SVD");
    REXEC(4)
    {
      MatNd_printCommentDigits("Mass matrix", M, 12);
      MatNd_printCommentDigits("Sum of torques", b, 3);
    }
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

  if (MatNd_isINF(q_ddot))
  {
    REXEC(3)
    {
      MatNd_printCommentDigits("q", graph->q, 4);
      MatNd_printCommentDigits("q_dot", graph->q_dot, 4);
      MatNd_printCommentDigits("q_ddot", q_ddot, 4);
      MatNd_printCommentDigits("h", h, 4);
      MatNd_printCommentDigits("F_gravity", F_gravity, 4);

      if (F_jnt != NULL)
      {
        MatNd_printCommentDigits("F_jnt", F_jnt, 4);
      }

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
void Rcs_directDynamicsIntegrationStep(const double* x,
                                       void* param,
                                       double* xp,
                                       double dt)
{
  RcsGraph* graph = (RcsGraph*) param;
  MatNd* F_ext    = (MatNd*) graph->userData;
  int n           = graph->nJ;
  MatNd q         = MatNd_fromPtr(n, 1, (double*) &x[0]);
  MatNd q_dot     = MatNd_fromPtr(n, 1, (double*) &x[n]);
  MatNd q_ddot    = MatNd_fromPtr(n, 1, &xp[n]);

  RcsGraph_setState(graph, &q, &q_dot);

  // Set the speed of the coupled joints to zero. Otherwise, they will get
  // integrated and drift away.
  RCSGRAPH_TRAVERSE_JOINTS(graph)
  {
    if (JNT->coupledTo != NULL)
    {
      q_dot.ele[JNT->jacobiIndex] = 0.0;
    }
  }

  Rcs_directDynamics(graph, F_ext, NULL, &q_ddot);

  memmove(&xp[0], &x[n], n*sizeof(double));
}


