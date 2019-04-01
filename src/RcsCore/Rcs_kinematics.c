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

#include "Rcs_kinematics.h"
#include "Rcs_typedef.h"
#include "Rcs_body.h"
#include "Rcs_joint.h"
#include "Rcs_utils.h"
#include "Rcs_macros.h"
#include "Rcs_math.h"



/*******************************************************************************
 * Computes the world coordinates of the body-fixed point b_p of body b.
 ******************************************************************************/
void RcsGraph_bodyPoint(const RcsBody* b, const double b_p[3], double I_pt[3])
{
  if (b == NULL)
  {
    if (b_p != NULL)
    {
      Vec3d_copy(I_pt, b_p);
    }
    else
    {
      Vec3d_setZero(I_pt);
    }
    return;
  }

  Vec3d_copy(I_pt, b->A_BI->org);

  if (b_p != NULL)
  {
    double I_offset[3];
    Vec3d_transRotate(I_offset, b->A_BI->rot, b_p);
    Vec3d_addSelf(I_pt, I_offset);
  }

}

/*******************************************************************************
 * Calculates the column of the translation Jacobian that belongs to the
 *        corresponding joint. The function returns true if the column could
 *        be computed, false otherwise. Reasons for that are
 *        - the argument jnt is a NULL-pointer
 *        - the joint is constrained
 ******************************************************************************/
static bool computeWorldPointJacobianForJoint(const RcsJoint* jnt,
                                              double A_BI[3][3],
                                              const double I_r_IP[3],
                                              double* colPtr,
                                              unsigned int stride)
{
  // Skip column if joint doesn't exist
  if (jnt == NULL)
  {
    return false;
  }

  // Skip column if joint is constrained
  if (jnt->constrained == true)
  {
    return false;
  }

  double col[3];

  switch (jnt->type)
  {
    // Direction vector of the joint axis
    case RCSJOINT_TRANS_X:
    case RCSJOINT_TRANS_Y:
    case RCSJOINT_TRANS_Z:
    {
      if (A_BI == NULL)
      {
        Vec3d_copy(col, jnt->A_JI.rot[jnt->dirIdx]);
      }
      else
      {
        Vec3d_rotate(col, A_BI, jnt->A_JI.rot[jnt->dirIdx]);
      }
      break;
    }

    // A_BI * (I_a_jnt x (I_r_pt - I_r_jnt))
    case RCSJOINT_ROT_X:
    case RCSJOINT_ROT_Y:
    case RCSJOINT_ROT_Z:
    {
      double I_r_JP[3];
      Vec3d_sub(I_r_JP, I_r_IP, jnt->A_JI.org);
      Vec3d_crossProduct(col, jnt->A_JI.rot[jnt->dirIdx], I_r_JP);
      if (A_BI != NULL)
      {
        Vec3d_rotateSelf(col, A_BI);
      }
    }
    break;

    default:
      RFATAL("Unknown joint type: %d", jnt->type);

  }   // switch(jnt->type)



  colPtr[0]        = col[0];
  colPtr[stride]   = col[1];
  colPtr[2*stride] = col[2];

  return true;
}



/*******************************************************************************
 * Adds a world point Jacobian to J. Since this is local, the reshaping is the
 * job of the calling function.
 ******************************************************************************/
static void RcsGraph_addWorldPointJacobian(const RcsGraph* self,
                                           const RcsBody* body,
                                           const double I_bodyPt[3],
                                           double A_BI[3][3],
                                           MatNd* J)
{
  const double* I_r_IP = I_bodyPt ? I_bodyPt :
                         (body ? body->A_BI->org : Vec3d_zeroVec());

  // Find "driving" joint of the body
  RcsJoint* jnt = RcsBody_lastJointBeforeBody(body);

  // Traverse backwards through the joints
  while (jnt != NULL)
  {
    computeWorldPointJacobianForJoint(jnt, A_BI, I_r_IP,
                                      &J->ele[jnt->jacobiIndex], J->n);
    jnt = jnt->prev;
  }

}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsGraph_worldPointJacobian(const RcsGraph* self,
                                 const RcsBody* body,
                                 const double I_bodyPt[3],
                                 double A_BI[3][3],
                                 MatNd* J)
{
  // Reset the Jacobian
  MatNd_reshapeAndSetZero(J, 3, self->nJ);

  // If body is NULL, return without doing anything
  if (body==NULL)
  {
    return;
  }

  // Add bodie's contribution
  RcsGraph_addWorldPointJacobian(self, body, I_bodyPt, A_BI, J);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsGraph_bodyPointJacobian(const RcsGraph* self,
                                const RcsBody* body,
                                const double k_bdyPt[3],
                                double A_BI[3][3],
                                MatNd* J)
{
  if ((k_bdyPt==NULL) || (body==NULL))
  {
    RcsGraph_worldPointJacobian(self, body, NULL, A_BI, J);
  }
  else
  {
    double I_bdyPt[3];
    Vec3d_copy(I_bdyPt, body->A_BI->org);
    Vec3d_transMulAndAdd(I_bdyPt, body->A_BI->org, body->A_BI->rot,
                         k_bdyPt);
    RcsGraph_worldPointJacobian(self, body, I_bdyPt, A_BI, J);
  }
}

/*******************************************************************************
 * Body point Jacobian with scaling factor.
 ******************************************************************************/
static void RcsGraph_constMulAndAddBodyPointJacobian(const RcsGraph* self,
                                                     const RcsBody* body,
                                                     const double k_bdyPt[3],
                                                     double A_BI[3][3],
                                                     double multiplier,
                                                     MatNd* J)
{
  if (body==NULL)
  {
    return;
  }

  RCHECK_MSG((J->m==3) && (J->n==self->nJ), "Jacobian dimension mismatch: "
             "J is %d x %d, should be 3 x %d", J->m, J->n, self->nJ);

  // Convert body point into world coordinates
  double I_r_IP[3];
  Vec3d_copy(I_r_IP, body->A_BI->org);

  if (k_bdyPt != NULL)
  {
    Vec3d_transMulAndAdd(I_r_IP, body->A_BI->org, body->A_BI->rot, k_bdyPt);
  }

  // Find "driving" joint of the body
  RcsJoint* jnt = RcsBody_lastJointBeforeBody(body);

  // Traverse backwards through the joints
  double col[3];
  while (jnt != NULL)
  {
    if (computeWorldPointJacobianForJoint(jnt, A_BI, I_r_IP, col, 1))
    {
      J->ele[jnt->jacobiIndex]        += multiplier*col[0];
      J->ele[jnt->jacobiIndex+J->n]   += multiplier*col[1];
      J->ele[jnt->jacobiIndex+2*J->n] += multiplier*col[2];
    }

    jnt = jnt->prev;
  }

}

/*******************************************************************************
 * Computes the vector from joint jnt to the local body point k_r
 *         (represented in bdy's frame) in world coordinates.
 ******************************************************************************/
static void RcsGraph_computeJointToBodyPt(const RcsJoint* jnt,
                                          const RcsBody* bdy,
                                          const double k_r[3],
                                          double I_r[3])
{
  Vec3d_sub(I_r, bdy->A_BI->org, jnt->A_JI.org);

  if (k_r != NULL)
  {
    double I_offset[3];
    Vec3d_transRotate(I_offset, bdy->A_BI->rot, k_r);
    Vec3d_addSelf(I_r, I_offset);
  }
}

/*******************************************************************************
 *
 *  Computes the body point Hessian dJp/dq.
 *
 *  Here's how the memory is aligned:
 *
 *  0         1             n-1        n             (m-1)*(n-1)
 *  -------------------------------------------------------------------
 *  dJ00/dq0 dJ00/dq1 ... dJ00/dq(n-1) dJ01/dq0 ... dJ(m-1)(n-1)/dq(n-1)
 *
 *  Here's how to rotate it into another coordinate frame:
 *
 *  for (int j=0;j<n;j++)
 *    for (int k=0;k<n;k++)
 *    {
 *      double sj[3];
 *      sj[0] = H->ele[0*nn+j*n+k];
 *      sj[1] = H->ele[1*nn+j*n+k];
 *      sj[2] = H->ele[2*nn+j*n+k];
 *      Vec3d_rotateSelf(sj, b1->A_BI);
 *      H->ele[0*nn+j*n+k] = sj[0];
 *      H->ele[1*nn+j*n+k] = sj[1];
 *      H->ele[2*nn+j*n+k] = sj[2];
 *    }
 *
 ******************************************************************************/
void RcsGraph_bodyPointHessian(const RcsGraph* self,
                               const RcsBody* b,
                               const double k_bodyPt[3],
                               double A_BI[3][3],
                               MatNd* H)
{
  int n = self->nJ, nn = n * n;

  MatNd_reshapeAndSetZero(H, 3*n, n);

  // Find last joint of previous body (which has joints)
  RcsJoint* jnt_j = RcsBody_lastJointBeforeBody(b);
  if (jnt_j == NULL)
  {
    return;
  }

  // Loop recursively through the joints
  while (jnt_j)
  {
    // Skip constrained joints
    if (jnt_j->constrained)
    {
      jnt_j = jnt_j->prev;
      continue;
    }

    bool jIsRotation = RcsJoint_isRotation(jnt_j);
    RcsJoint* jnt_k  = jnt_j;

    while (jnt_k != NULL)
    {
      if (jnt_k->constrained == true)
      {
        jnt_k = jnt_k->prev;
        continue;
      }

      double sj[3];
      bool kIsRotation = RcsJoint_isRotation(jnt_k);
      const double* jntDir_j = jnt_j->A_JI.rot[jnt_j->dirIdx];
      const double* jntDir_k = jnt_k->A_JI.rot[jnt_k->dirIdx];

      if (jIsRotation && kIsRotation)
      {
        double I_r_jp[3], ajxrjp[3];

        RcsGraph_computeJointToBodyPt(jnt_j, b, k_bodyPt, I_r_jp);
        Vec3d_crossProduct(ajxrjp, jntDir_j, I_r_jp);

        Vec3d_crossProduct(sj, jntDir_k, ajxrjp);
      }
      else if ((!jIsRotation) && kIsRotation)
      {
        Vec3d_crossProduct(sj, jntDir_k, jntDir_j);
      }
      else // rot-trans or trans-trans makes zero entry
      {
        jnt_k = jnt_k->prev;
        continue;
      }

      if (A_BI != NULL)
      {
        Vec3d_rotateSelf(sj, A_BI);
      }

      for (int i = 0; i < 3; i++)
      {
        H->ele[i*nn + jnt_j->jacobiIndex*n + jnt_k->jacobiIndex] = sj[i];
        H->ele[i*nn + jnt_k->jacobiIndex*n + jnt_j->jacobiIndex] = sj[i];
      }

      jnt_k = jnt_k->prev;
    }

    jnt_j = jnt_j->prev;
  }

}

/*******************************************************************************
 * Translation dot Jacobian: q_dot^T H
 ******************************************************************************/
void RcsGraph_bodyPointDotJacobian(const RcsGraph* self,
                                   const RcsBody* b,
                                   const double k_bodyPt[3],
                                   double A_BI[3][3],
                                   const MatNd* q_dot,
                                   MatNd* J)
{
  int n = self->nJ;
  MatNd* H = NULL;

  MatNd_create2(H, 3*n, n);
  RcsGraph_bodyPointHessian(self, b, k_bodyPt, A_BI, H);
  MatNd_reshape(J, 3*n, 1);
  MatNd_mul(J, H, q_dot);
  MatNd_reshape(J, 3, n);
  MatNd_destroy(H);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsGraph_rotationJacobian(const RcsGraph* self,
                               const RcsBody* b,
                               double A_BI[3][3],
                               MatNd* J)
{
  // Reset the Jacobian
  MatNd_reshape(J, 3, self->nJ);
  MatNd_setZero(J);

  // If body is NULL, return a zero-Jacobian
  if (b == NULL)
  {
    return;
  }

  // Find the last joint of the previous body which has joints
  RcsJoint* jnt = RcsBody_lastJointBeforeBody(b);

  // Traverse backwards through the joints
  while (jnt != NULL)
  {
    if ((jnt->constrained == true) || (RcsJoint_isRotation(jnt)==false))
    {
      jnt = jnt->prev;
      continue;
    }

    const double* jntDir = jnt->A_JI.rot[jnt->dirIdx];

    if (A_BI != NULL)
    {
      double axisDir[3];
      Vec3d_rotate(axisDir, A_BI, jntDir);
      MatNd_setColumn(J, jnt->jacobiIndex, axisDir, 3);
    }
    else
    {
      MatNd_setColumn(J, jnt->jacobiIndex, jntDir, 3);
    }

    jnt = jnt->prev;
  }

}

/*******************************************************************************
 *
 *  Computes the partial derivative of the Jacobian with respect to the
 *  state vector. This is the analytic solution.
 *
 *  Here's how the memory is aligned:
 *
 *  0         1             n-1        n             (m-1)*(n-1)
 *  -------------------------------------------------------------------
 *  dJ00/dq0 dJ00/dq1 ... dJ00/dq(n-1) dJ01/dq0 ... dJ(m-1)(n-1)/dq(n-1)
 *
 ******************************************************************************/
void RcsGraph_rotationHessian(const RcsGraph* self,
                              const RcsBody* b,
                              double A_BI[3][3],
                              MatNd* H)
{
  int n = self->nJ;
  MatNd_reshapeAndSetZero(H, n*3, n);

  // Find last joint of previous body (which has joints)
  RcsJoint* jnt_j = RcsBody_lastJointBeforeBody(b);

  if (!jnt_j)
  {
    return;
  }

  // Loop recursively through the joints
  while (jnt_j != NULL)
  {
    // Skip non-rotation and constrained joints
    if ((jnt_j->constrained==true) || (RcsJoint_isRotation(jnt_j)==false))
    {
      jnt_j = jnt_j->prev;
      continue;
    }

    RcsJoint* jnt_k = jnt_j;

    while (jnt_k != NULL)
    {
      if (RcsJoint_isRotation(jnt_k) &&       // skip non-rotation joints
          (jnt_k->constrained == false) &&     // skip constrained joints
          (jnt_j->jacobiIndex != jnt_k->jacobiIndex))  // is 0 vector for j=k
      {
        double sj[3];
        const double* jntDir_j = jnt_j->A_JI.rot[jnt_j->dirIdx];
        const double* jntDir_k = jnt_k->A_JI.rot[jnt_k->dirIdx];
        Vec3d_crossProduct(sj, jntDir_k, jntDir_j);

        if (A_BI != NULL)
        {
          Vec3d_rotateSelf(sj, A_BI);
        }

        for (int i = 0; i < 3; i++)
        {
          H->ele[i*n*n + jnt_j->jacobiIndex*n + jnt_k->jacobiIndex] = sj[i];
        }
      }

      jnt_k = jnt_k->prev;
    }

    jnt_j = jnt_j->prev;
  }

}

/*******************************************************************************
 * Rotation dot Jacobian: q_dot^T H
 ******************************************************************************/
void RcsGraph_rotationDotJacobian(const RcsGraph* self,
                                  const RcsBody* b,
                                  const MatNd* q_dot,
                                  MatNd* J)
{
  int n = self->nJ, n3 = 3*self->nJ;
  MatNd* H = NULL;

  MatNd_create2(H, n3, n);
  RcsGraph_rotationHessian(self, b, NULL, H);
  MatNd_reshape(J, n3, 1);
  MatNd_mul(J, H, q_dot);
  MatNd_reshape(J, 3, n);
  MatNd_destroy(H);
}

/*******************************************************************************
 *
 *  Position XYZ Jacobian. Effector 2 relative to reference body 1, both
 *  in fixed and moving references. The relative position is
 *
 *  1_r_12 = A_1I * (I_r2 - I_r1)
 *
 *  The Jacobian is
 *
 *  J = A_1I * (JT_2 - JT_1 + r12 x JR_1)
 *
 *  For the case of an articulated refFrame 3 that is not the refBdy, the
 *  Jacobian gets
 *
 *  J = A_3I * (JT_2 - JT_1 + r12 x JR_3)
 *
 ******************************************************************************/
void RcsGraph_3dPosJacobian(const RcsGraph* self, const RcsBody* effector,
                            const RcsBody* refBdy, const RcsBody* refFrame,
                            MatNd* J)
{
  // Jacobian wrt. world coordinate frame
  if (refBdy == NULL)
  {
    RcsGraph_bodyPointJacobian(self, effector, NULL,
                               refFrame ? refFrame->A_BI->rot : NULL, J);
    return;
  }
  // From here on, both refBdy and refFrame are valid bodies. We compute the
  // Jacobian wrt. articulated reference
  else
  {
    // Workspace
    MatNd* bufJ = NULL;
    MatNd_create2(bufJ, 3, self->nJ);

    // I_JT_2 - I_JT_1
    RcsGraph_bodyPointJacobian(self, effector, NULL, NULL, J);
    RcsGraph_bodyPointJacobian(self, refBdy, NULL, NULL, bufJ);
    MatNd_subSelf(J, bufJ);

    // I_r12 = I_r2 - I_r1
    double r12[3];
    Vec3d_sub(r12, effector ? effector->A_BI->org : Vec3d_zeroVec(),
              refBdy->A_BI->org);

    // I_r12 x I_JR1
    RcsGraph_rotationJacobian(self, refFrame, NULL, bufJ);
    MatNd_columnCrossProductSelf(bufJ, r12);  // J_rel = (I_r2-I_r1) x J_rel
    MatNd_addSelf(J, bufJ);

    // A_3I (I_JT2 - I_JT1 - I_r12 x I_JR2)
    if ((refFrame!=NULL) && (refFrame!=refBdy))
    {
      MatNd_rotateSelf(J, refFrame->A_BI->rot);
    }
    // A_2I (I_JT2 - I_JT1 - I_r12 x I_JR2)
    else
    {
      MatNd_rotateSelf(J, refBdy->A_BI->rot);
    }

    // Cleanup
    MatNd_destroy(bufJ);
  }

}

/*******************************************************************************
 *
 *  Position XYZ Hessian for the articulated reference frame case:
 *
 *  H = dq(A_1I) (J2 - J1 + r12 x JR1) +            (Term 1)
 *      A_1I (dq(r_12 x) J_R1) +                    (Term 2)
 *      A_1I (H2 - H1) +                            (Term 3)
 *      A_1I ((r_12 x) HR1)                         (Term 4)
 *
 *  For the case of an articulated refFrame 3 that is not the refBdy,
 *  the Hessian gets
 *
 *  H = dq(A_3I) (J2 - J1 + r12 x JR3) +            (Term 1)
 *      A_3I (dq(r_12 x) J_R3) +                    (Term 2)
 *      A_3I (H2 - H1) +                            (Term 3)
 *      A_3I ((r_12 x) HR3)                         (Term 4)
 *
 ******************************************************************************/
void RcsGraph_3dPosHessian(const RcsGraph* self, const RcsBody* effector,
                           const RcsBody* refBdy, const RcsBody* refFrame,
                           MatNd* H)
{
  // Hessian wrt. fixed or refFrame coordinate frame
  if (refBdy == NULL)
  {
    if (refFrame!=NULL)
    {
      RcsGraph_bodyPointHessian(self, effector, NULL, refFrame->A_BI->rot, H);
    }
    else
    {
      RcsGraph_bodyPointHessian(self, effector, NULL, NULL, H);
    }
  }
  // From here on, both refBdy and refFrame are valid bodies. We compute
  // the Jacobian wrt. articulated reference
  else
  {
    int n = self->nJ, n3 = n * 3, nn = n * n;
    MatNd_reshape(H, n3, n);

    const RcsBody* b1 = refBdy;
    const RcsBody* b2 = effector;
    const RcsBody* b3 = refFrame;

    // Workspace
    MatNd* bufH1 = NULL;
    MatNd_create2(bufH1, n3, n);
    MatNd* bufJ1 = NULL;
    MatNd_create2(bufJ1, 3, n);

    // del(A_3I)/del(q)
    MatNd* dA3 = NULL;
    MatNd_create2(dA3, n3, 3);
    RcsGraph_dAdq(self, b3, &dA3->ele[0], true);

    // JT2 - JT1
    MatNd* J12   = NULL;
    MatNd_create2(J12, 3, self->nJ);
    RcsGraph_bodyPointJacobian(self, b1, NULL, NULL, bufJ1);
    RcsGraph_bodyPointJacobian(self, b2, NULL, NULL, J12);
    MatNd_subSelf(J12, bufJ1);

    // JR3
    MatNd* JR3 = NULL;
    MatNd_create2(JR3, 3, n);
    RcsGraph_rotationJacobian(self, b3, NULL, JR3);

    // I_r_12
    double r12[3];
    Vec3d_sub(r12, b2->A_BI->org, b1->A_BI->org);

    // Term 1: del(A_3I)/del(q) (J2 - J1 + r12 x JR3)
    MatNd_columnCrossProduct(bufJ1, JR3, r12);
    MatNd_addSelf(bufJ1, J12);
    MatNd_mul(H, dA3, bufJ1);

    // Term 2: A_3I (dq(r_12 x) J_R3) = A_3I (((J2-J1) x) J_R3)
    double col[3];
    MatNd dqr12 = MatNd_fromPtr(3, 1, col);

    for (int i = 0; i < n; i++)
    {
      MatNd Jcross = MatNd_fromPtr(3, n, &bufH1->ele[i * n3]);
      MatNd_getColumn(&dqr12, i, J12);
      MatNd_columnCrossProduct(&Jcross, JR3, dqr12.ele);
      MatNd_rotateSelf(&Jcross, b3->A_BI->rot);
    }
    MatNd_addSelf(H, bufH1);

    // We need to transpose terms 1 and 2 here, since above computation
    // creates the transposed memory layout dq0(J) dq1(J) ... dqn-1(J),
    // but we need dq0(J0,0) dq1(J0,0) ... dqn-1(J3,n-1)
    MatNd_reshape(H, n, n3);
    MatNd_transposeSelf(H);

    // Term 3: A_3I (H2 - H1)
    RcsGraph_bodyPointHessian(self, b2, NULL, b3->A_BI->rot, bufH1);
    MatNd_addSelf(H, bufH1);
    RcsGraph_bodyPointHessian(self, b1, NULL, b3->A_BI->rot, bufH1);
    MatNd_subSelf(H, bufH1);

    // Term 4: A_3I ((r_12 x) HR3)
    MatNd* HR3 = bufH1;
    RcsGraph_rotationHessian(self, b3, NULL, HR3);

    for (int j = 0; j < n; j++)
    {
      for (int k = 0; k < n; k++)
      {
        double col[3], dst[3];
        col[0] = HR3->ele[         j * n + k];
        col[1] = HR3->ele[    nn + j * n + k];
        col[2] = HR3->ele[2 * nn + j * n + k];
        Vec3d_crossProduct(dst, r12, col);
        Vec3d_rotateSelf(dst, b3->A_BI->rot);
        H->ele[         j * n + k] += dst[0];
        H->ele[    nn + j * n + k] += dst[1];
        H->ele[2 * nn + j * n + k] += dst[2];
      }
    }

    // Clean up
    MatNd_destroy(dA3);
    MatNd_destroy(bufJ1);
    MatNd_destroy(J12);
    MatNd_destroy(JR3);
    MatNd_destroy(bufH1);
  }

}

/*******************************************************************************
 * Computes the respective column using the RcsGraph_3dPosJacobian() function.
 ******************************************************************************/
void RcsGraph_1dPosJacobian(const RcsGraph* self, const RcsBody* effector,
                            const RcsBody* refBdy, const RcsBody* refFrame,
                            int index, MatNd* J)
{
  MatNd* J3 = NULL;
  MatNd_create2(J3, 3, self->nJ);
  RcsGraph_3dPosJacobian(self, effector, refBdy, refFrame, J3);
  MatNd_getRow(J, index, J3);
  MatNd_destroy(J3);
}

/*******************************************************************************
 * Computes the respective column using the RcsGraph_3dPosHessian() function.
 ******************************************************************************/
void RcsGraph_1dPosHessian(const RcsGraph* self, const RcsBody* effector,
                           const RcsBody* refBdy, const RcsBody* refFrame,
                           int index, MatNd* H)
{
  int n = self->nJ, nn = n * n;
  MatNd* H3 = NULL;
  MatNd_create2(H3, 3 * n, n);

  RcsGraph_3dPosHessian(self, effector, refBdy, refFrame, H3);

  RCHECK((index>=0) && (index<3));  // Only x, y and z
  MatNd_reshape(H, n, n);
  memcpy(H->ele, &H3->ele[index * nn], nn * sizeof(double));
  MatNd_destroy(H3);
}

/*******************************************************************************
 * Rotation Jacobian J = A_3I * (JR_2 - JR_1)
 *
 *         (Possibly articulated) refFrame: 3
 *         (Possibly articulated) refBdy:   1
 *         Effector:                        2
 ******************************************************************************/
void RcsGraph_3dOmegaJacobian(const RcsGraph* self, const RcsBody* effector,
                              const RcsBody* refBdy, const RcsBody* refFrame,
                              MatNd* J)
{
  // Jacobian wrt. world coordinate frame: J = JR_2
  if ((refBdy==NULL) && (refFrame==NULL))
  {
    RcsGraph_rotationJacobian(self, effector, NULL, J);
  }
  // Jacobian wrt. static reference frame: J = A_2I * JR_2
  else if ((refBdy!=NULL) && (refFrame==refBdy) &&
           (RcsBody_isArticulated(refBdy)==false))
  {
    RcsGraph_rotationJacobian(self, effector, refBdy->A_BI->rot, J);
  }
  // Jacobian wrt. articulated reference: J = A_3I * (JR_2 - JR_1)
  else
  {
    // JR2 - JR1
    MatNd* bufJ = NULL;
    MatNd_create2(bufJ, 3, self->nJ);
    RcsGraph_rotationJacobian(self, effector, NULL, J);
    RcsGraph_rotationJacobian(self, refBdy, NULL, bufJ);
    MatNd_subSelf(J, bufJ);
    MatNd_destroy(bufJ);

    // A_3I (JR2 - JR1)
    if (refFrame != NULL)
    {
      MatNd_rotateSelf(J, refFrame->A_BI->rot);
    }
  }

}

/*******************************************************************************
 * Rotation Hessian H = dq(A_3I) (JR_2 - JR_1) + A_3I (H2 - H1)
 *
 *         (Possibly articulated) refFrame: 3
 *         (Possibly articulated) refBdy:   1
 *         Effector:                        2
 ******************************************************************************/
void RcsGraph_3dOmegaHessian(const RcsGraph* self, const RcsBody* effector,
                             const RcsBody* refBdy, const RcsBody* refFrame,
                             MatNd* H)
{
  int n = self->nJ, n3 = n * 3;

  // Hessian wrt. world coordinate frame
  if ((refBdy==NULL) && (refFrame==NULL))
  {
    MatNd_reshape(H, n, n3);
    RcsGraph_rotationHessian(self, effector, NULL, H);
  }
  // Hessian wrt. static reference frame
  else if ((refBdy!=NULL) && (refFrame==refBdy) &&
           (RcsBody_isArticulated(refBdy)==false))
  {
    MatNd_reshape(H, n, n3);
    RcsGraph_rotationHessian(self, effector, refBdy->A_BI->rot, H);
  }
  // From here on, both refBdy and refFrame are valid bodies. We compute the
  // Jacobian wrt. articulated reference
  else
  {
    MatNd_reshape(H, n3, n);

    // Workspace
    MatNd* bufH1 = NULL;
    MatNd_create2(bufH1, n3, n);
    MatNd* bufJ1 = NULL;
    MatNd_create2(bufJ1, 3, n);

    // del(A_3I)/del(q)
    MatNd* dA1 = NULL;
    MatNd_create2(dA1, n3, 3);
    RcsGraph_dAdq(self, refFrame, &dA1->ele[0], true);

    // JR2 - JR1
    MatNd* JR12   = NULL;
    MatNd_create2(JR12, 3, self->nJ);
    RcsGraph_rotationJacobian(self, refBdy, NULL, bufJ1);
    RcsGraph_rotationJacobian(self, effector, NULL, JR12);
    MatNd_subSelf(JR12, bufJ1);

    // Term 1: del(A_3I)/del(q) (JR2 - JR1)
    MatNd_mul(H, dA1, JR12);

    // We need to transpose terms 1 here, since above computation
    // creates the transposed memory layout dq0(J) dq1(J) ... dqn-1(J),
    // but we need dq0(J0,0) dq1(J0,0) ... dqn-1(J3,n-1)
    MatNd_reshape(H, n, n3);
    MatNd_transposeSelf(H);

    // Term 2: A_3I (HR2 - HR1)
    RcsGraph_rotationHessian(self, effector,
                             refFrame ? refFrame->A_BI->rot : NULL, bufH1);
    MatNd_addSelf(H, bufH1);
    RcsGraph_rotationHessian(self, refBdy,
                             refFrame ? refFrame->A_BI->rot : NULL, bufH1);
    MatNd_subSelf(H, bufH1);

    // Clean up
    MatNd_destroy(dA1);
    MatNd_destroy(bufJ1);
    MatNd_destroy(JR12);
    MatNd_destroy(bufH1);
  }

  MatNd_reshape(H, n3, n);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
double RcsGraph_COG(const RcsGraph* self, double r_cog[3])
{
  double tmp[3], m = 0.0;

  Vec3d_setZero(r_cog);

  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    if (BODY->m>0.0)
    {
      Vec3d_transRotate(tmp, BODY->A_BI->rot,
                        BODY->Inertia->org); // I_r_Ki-COMi
      Vec3d_addSelf(tmp, BODY->A_BI->org);   // I_r_I-COMi
      Vec3d_constMulSelf(tmp, BODY->m);
      Vec3d_addSelf(r_cog, tmp);
      m += BODY->m;
    }
  }

  if (m>0.0)
  {
    Vec3d_constMulSelf(r_cog, 1.0/m);
  }
  else
  {
    RLOG(1, "Computed COM for graph with zero mass - setting to 0");
  }

  return m;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
double RcsGraph_COG_Body(const RcsBody* body, double r_cog[3])
{
  double tmp[3], m = 0.0;

  Vec3d_setZero(r_cog);

  RCSBODY_TRAVERSE_BODIES((RcsBody*) body)
  {
    if (BODY->m>0.0)
    {
      Vec3d_transRotate(tmp, BODY->A_BI->rot,
                        BODY->Inertia->org); // I_r_Ki-COMi
      Vec3d_addSelf(tmp, BODY->A_BI->org);   // I_r_I-COMi
      Vec3d_constMulSelf(tmp, BODY->m);
      Vec3d_addSelf(r_cog, tmp);
      m += BODY->m;
    }
  }

  if (m>0.0)
  {
    Vec3d_constMulSelf(r_cog, 1.0/m);
  }
  else
  {
    RLOG(1, "Computed COM for graph with zero mass - setting to 0");
  }

  return m;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
// \todo: Consider reference body.
void RcsGraph_COGJacobian(const RcsGraph* self, MatNd* J_cog)
{
  double m = 0.0;
  MatNd_reshapeAndSetZero(J_cog, 3, self->nJ);

  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    if (BODY->m > 0.0)
    {
      RcsGraph_constMulAndAddBodyPointJacobian(self, BODY,
                                               BODY->Inertia->org, NULL,
                                               BODY->m, J_cog);
      m += BODY->m;
    }

  }

  if (m>0.0)
  {
    MatNd_constMulSelf(J_cog, 1.0 / m);
  }
  else
  {
    RLOG(1, "Computed COM Jacobian for graph with zero mass: setting to 0");
  }
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
// \todo: Consider reference body.
void RcsGraph_COGJacobian_Body(const RcsGraph* self, const RcsBody* body,
                               MatNd* J_cog)
{
  double m = 0.0;
  MatNd* J_local = NULL;

  MatNd_create2(J_local, 3, self->nJ);
  MatNd_reshape(J_cog, 3, self->nJ);
  MatNd_setZero(J_cog);

  RCSBODY_TRAVERSE_BODIES((RcsBody*) body)
  {
    if (BODY->m > 0.0)
    {
      RcsGraph_bodyPointJacobian(self, BODY, BODY->Inertia->org, NULL,
                                 J_local);
      MatNd_constMulAndAddSelf(J_cog, J_local, BODY->m);
      m += BODY->m;
    }

  }

  MatNd_destroy(J_local);

  if (m>0.0)
  {
    MatNd_constMulSelf(J_cog, 1.0 / m);
  }
  else
  {
    RLOG(1, "Computed COM Jacobian for graph with zero mass - setting to 0");
  }
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
// \todo: Consider reference body.
void RcsGraph_computeCOGHessian(const RcsGraph* self, double* H)
{
  int i, n = self->nJ, hEle = 3 * n * n;
  double mass = 0.0;
  MatNd* Hi = MatNd_create(hEle, 1);

  memset(H, 0, hEle * sizeof(double));

  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    if (BODY->m > 0.0)
    {
      RcsGraph_bodyPointHessian(self, BODY, BODY->Inertia->org, NULL, Hi);
      for (i = 0; i < hEle; i++)
      {
        H[i] += Hi->ele[i] * BODY->m;
      }
      mass += BODY->m;
    }

  }

  MatNd_destroy(Hi);

  if (mass > 0.0)
  {
    for (i = 0; i < hEle; i++)
    {
      H[i] /= mass;
    }
  }

}

/*******************************************************************************
 * See header.
 ******************************************************************************/
// \todo: Consider reference body.
void RcsGraph_computeCOGHessian_Body(const RcsGraph* self, const RcsBody* body,
                                     MatNd* H)
{
  int n = self->nJ, hEle = 3*n*n;
  double mass = 0.0;
  MatNd* Hi = NULL;
  MatNd_create2(Hi, hEle, 1);

  MatNd_reshape(H, 3*n, n);
  MatNd_setZero(H);

  RCSBODY_TRAVERSE_BODIES((RcsBody*) body)
  {
    if (BODY->m > 0.0)
    {
      RcsGraph_bodyPointHessian(self, BODY, BODY->Inertia->org, NULL, Hi);
      MatNd_constMulAndAddSelf(H, Hi, BODY->m);
      mass += BODY->m;
    }

  }

  MatNd_destroy(Hi);

  if (mass > 0.0)
  {
    MatNd_constMulSelf(H, 1.0/mass);
  }

}

/*******************************************************************************
 * See header.
 ******************************************************************************/
// \todo: Consider reference body.
void RcsGraph_computeCOGHessian_Body_(const RcsGraph* self, const RcsBody* body,
                                      MatNd* H, MatNd* Hi)
{
  int n = self->nJ;
  double mass = 0.0;

  RCHECK((int)Hi->size>=3*n*n);

  MatNd_reshape(H, 3*n, n);
  MatNd_setZero(H);

  RCSBODY_TRAVERSE_BODIES((RcsBody*) body)
  {
    if (BODY->m > 0.0)
    {
      RcsGraph_bodyPointHessian(self, BODY, BODY->Inertia->org, NULL, Hi);
      MatNd_constMulAndAddSelf(H, Hi, BODY->m);
      mass += BODY->m;
    }

  }

  if (mass > 0.0)
  {
    MatNd_constMulSelf(H, 1.0/mass);
  }

}

/*******************************************************************************
 * See header.
 ******************************************************************************/
double RcsGraph_jointLimitCost(const RcsGraph* self, RcsStateType type)
{
  double cost = 0.0;

  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    if ((JNT->constrained==true) && (type == RcsStateIK))
    {
      continue;
    }

    double range = JNT->q_max - JNT->q_min;

    if ((range>0.0) && (JNT->constrained==false))
    {
      double qi = MatNd_get(self->q, JNT->jointIndex, 0);
      cost += JNT->weightJL * pow((qi - JNT->q0) / range , 2);
    }
  }

  return 0.5 * cost;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
double RcsGraph_jointLimitCostPlateau(const RcsGraph* self,
                                      const double border_ratio,
                                      const RcsStateType type)
{
  double cost = 0.0;

  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    if ((JNT->constrained==true) && (type==RcsStateIK))
    {
      continue;
    }

    double border_size = (JNT->q_max - JNT->q_min) * border_ratio;

    if ((border_size > 0.0) && (JNT->jacobiIndex != -1))
    {
      double lower_threshold = JNT->q_min + border_size;

      if (JNT->q0 < lower_threshold)
      {
        lower_threshold = JNT->q0;
      }

      double upper_threshold = JNT->q_max - border_size;

      if (JNT->q0 > upper_threshold)
      {
        upper_threshold = JNT->q0;
      }

      double qi = MatNd_get(self->q, JNT->jointIndex, 0);

      if (qi < lower_threshold)
      {
        cost += JNT->weightJL*pow((qi-lower_threshold)/border_size, 2);
      }
      else if (qi > upper_threshold)
      {
        cost += JNT->weightJL*pow((qi-upper_threshold)/border_size, 2);
      }
    }
  }

  return 0.5 * cost;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
double RcsGraph_jointLimitBorderCost(const RcsGraph* self, double freeRatio,
                                     RcsStateType type)
{
  RCHECK((freeRatio>=0.0) & (freeRatio<=1.0));

  if (freeRatio==1.0)
  {
    return 0.0;
  }

  double cost = 0.0;

  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    if ((JNT->constrained==true) && (type==RcsStateIK))
    {
      continue;
    }

    // Coupled joints don't contribute. Limit avoidance is the master joint's
    // job.
    if (JNT->coupledTo != NULL)
    {
      continue;
    }

    double q = MatNd_get(self->q, JNT->jointIndex, 0);

    if (q > JNT->q0)
    {
      // We are in the upper half range
      double range = JNT->q_max - JNT->q0;

      if (range>0.0)
      {
        double freeRange = freeRatio*range;
        double penaltyRange = range - freeRange;
        double q0 = JNT->q0 + freeRange;

        if (q > q0)
        {
          cost += JNT->weightJL * pow((q-q0) / penaltyRange, 2);
        }
      }
    }
    else
    {
      // We are in the lower half range
      double range = JNT->q0 - JNT->q_min;

      if (range>0.0)
      {
        double freeRange = freeRatio*range;
        double penaltyRange = range - freeRange;
        double q0 = JNT->q0 - freeRange;

        if (q < q0)
        {
          cost += JNT->weightJL * pow((q-q0) / penaltyRange, 2);
        }
      }
    }

  }   // RCSGRAPH_TRAVERSE_JOINTS(self)

  return 0.5*cost;
}



/******************************************************************************

  Value freeRatio is the ratio of the joint's half range in which the
  joint limit gradient kicks in. For instance if freeRatio is 1, the whole
  joint range is without gradient. If freeRatio is 0, the gradient follows
  a quadratic cost over the whole range.

  Value freeRange is the range starting from q0, where the gradient is 0. The
  value penaltyRange is the range where the gradient is not 0. Here is an
  illustration:

  q_lowerLimit       q_lowerRange     q_0      q_upperRange     q_upperLimit
        |                  |           |             |                |
        |                  |           |             |                |
        \__________________/\__________/\____________/\_______________/
                 a               b             c               d

  freeRatio:           b/(a+b)                c/(c+d)
  freeRange:           b                      c
  penaltyRange:        a                      d

  cost =
    q in a: 0.5 * wJL * ((q-q_lowerRange)/a)^2   => c(limit) = 0.5*wJL
    q in b: 0
    q in c: 0
    q in d: 0.5 * wJL * ((q-q_upperRange)/d)^2   => c(limit) = 0.5*wJL

  gradient =
    q in a: (q-q_lowerRange)*wJL/a^2   => grad(limit) = 0.5*wJL/a
    q in b: 0
    q in c: 0
    q in d: (q-q_upperRange)*wJL/a^2   => grad(limit) = 0.5*wJL/d

    For the default case wJL=1 and freeRange=0, we get: dH_i = 0.5/halfRange
    and for symmetric joint centers, we get: dH_i = 1.0/range
    If we interpret it as a velocity, it's the inverse of the joint range.

******************************************************************************/

void RcsGraph_jointLimitBorderGradient(const RcsGraph* self,
                                       MatNd* dH,
                                       double freeRatio,
                                       RcsStateType type)
{
  RCHECK((freeRatio>=0.0) && (freeRatio<=1.0));

  int dimension = (type == RcsStateFull ? self->dof : self->nJ);

  MatNd_reshapeAndSetZero(dH, 1, dimension);

  if (freeRatio==1.0)
  {
    return;
  }

  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    if ((JNT->constrained==true) && (type==RcsStateIK))
    {
      continue;
    }

    // Coupled joints don't contribute. Limit avoidance is the master joint's
    // job.
    if (JNT->coupledTo != NULL)
    {
      continue;
    }


    int index = (type == RcsStateIK) ? JNT->jacobiIndex : JNT->jointIndex;
    double q = MatNd_get2(self->q, JNT->jointIndex, 0);

    if (q > JNT->q0)
    {
      // We are in the upper half range
      double range = JNT->q_max - JNT->q0;

      if (range>0.0)
      {
        double freeRange = freeRatio*range;
        double penaltyRange = range - freeRange;
        double q0 = JNT->q0 + freeRange;

        if (q > q0)
        {
          dH->ele[index] = JNT->weightJL * (q-q0) / (penaltyRange*penaltyRange);
        }
      }
    }
    else
    {
      // We are in the lower half range
      double range = JNT->q0 - JNT->q_min;

      if (range>0.0)
      {
        double freeRange = freeRatio*range;
        double penaltyRange = range - freeRange;
        double q0 = JNT->q0 - freeRange;

        if (q < q0)
        {
          dH->ele[index] = JNT->weightJL * (q-q0) / (penaltyRange*penaltyRange);
        }
      }
    }

  }   // RCSGRAPH_TRAVERSE_JOINTS(self)

}



/******************************************************************************

  \brief See header.

******************************************************************************/

void RcsGraph_jointLimitBorderHessian(const RcsGraph* self,
                                      MatNd* dH,
                                      double freeRatio,
                                      RcsStateType type)
{
  RCHECK((freeRatio>=0.0) & (freeRatio<1.0));

  int dimension = (type == RcsStateFull ? self->dof : self->nJ);

  MatNd_reshapeAndSetZero(dH, dimension, dimension);

  if (freeRatio==1.0)
  {
    return;
  }

  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    if ((JNT->constrained==true) && (type==RcsStateIK))
    {
      continue;
    }

    // Coupled joints don't contribute. Limit avoidance is the master joint's
    // job.
    if (JNT->coupledTo != NULL)
    {
      continue;
    }


    int index = (type == RcsStateIK) ? JNT->jacobiIndex : JNT->jointIndex;
    double q = MatNd_get(self->q, JNT->jointIndex, 0);

    if (q > JNT->q0)
    {
      // We are in the upper half range
      double range = JNT->q_max - JNT->q0;

      if (range>0.0)
      {
        double freeRange = freeRatio*range;
        double penaltyRange = range - freeRange;
        double q0 = JNT->q0 + freeRange;

        if (q > q0)
        {
          MatNd_set2(dH, index, index,
                     JNT->weightJL / (penaltyRange*penaltyRange))
        }
      }
    }
    else
    {
      // We are in the lower half range
      double range = JNT->q0 - JNT->q_min;

      if (range>0.0)
      {
        double freeRange = freeRatio*range;
        double penaltyRange = range - freeRange;
        double q0 = JNT->q0 - freeRange;

        if (q < q0)
        {
          MatNd_set2(dH, index, index,
                     JNT->weightJL / (penaltyRange*penaltyRange));
        }
      }
    }

  }   // RCSGRAPH_TRAVERSE_JOINTS(self)

}



/******************************************************************************

  \brief See header.

******************************************************************************/

double RcsGraph_jointLimitGradient(const RcsGraph* self,
                                   MatNd* dH,
                                   RcsStateType type)
{
  double cost = 0.0;
  int dimension = (type == RcsStateFull ? self->dof : self->nJ);

  MatNd_reshape(dH, 1, dimension);
  MatNd_setZero(dH);

  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    if ((JNT->constrained==true) && (type == RcsStateIK))
    {
      continue;
    }

    double qi = MatNd_get(self->q, JNT->jointIndex, 0);
    int index = (type == RcsStateIK) ? JNT->jacobiIndex : JNT->jointIndex;
    double range = JNT->q_max - JNT->q_min;
    double delta = qi - JNT->q0;

    if ((range > 0.0) && (JNT->jacobiIndex != -1))
    {
      dH->ele[index] = JNT->weightJL * delta / (range * range);
      cost += JNT->weightJL * pow(delta / range , 2);
    }
  }

  return 0.5 * cost;
}

/******************************************************************************

  \brief See header.

******************************************************************************/

double RcsGraph_jointLimitGradientPlateau(const RcsGraph* self,
                                          MatNd* dH,
                                          const double border_ratio,
                                          RcsStateType type)
{
  double cost = 0.0;
  int dimension = (type == RcsStateFull ? self->dof : self->nJ);

  MatNd_reshapeAndSetZero(dH, 1, dimension);

  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    if ((JNT->jacobiIndex == -1) && (type == RcsStateIK))
    {
      continue;
    }

    int index = (type == RcsStateIK) ? JNT->jacobiIndex : JNT->jointIndex;
    double qi = MatNd_get(self->q, JNT->jointIndex, 0);

    double border_size = (JNT->q_max - JNT->q_min) * border_ratio;

    if ((border_size > 0.0) && (JNT->jacobiIndex != -1))
    {
      double lower_threshold = JNT->q_min + border_size;

      if (JNT->q0 < lower_threshold)
      {
        dH->ele[index] = 0;
        lower_threshold = JNT->q0;
      }
      double upper_threshold = JNT->q_max - border_size;
      if (JNT->q0 > upper_threshold)
      {
        dH->ele[index] = 0;
        upper_threshold = JNT->q0;
      }
      if (qi < lower_threshold)
      {
        dH->ele[index] = JNT->weightJL * (qi - lower_threshold) / (border_size * border_size);
        cost += JNT->weightJL * pow((qi - lower_threshold) / border_size , 2);
      }
      else if (qi > upper_threshold)
      {
        dH->ele[index] = JNT->weightJL * (qi - upper_threshold) / (border_size * border_size);
        cost += JNT->weightJL * pow((qi - upper_threshold) / border_size , 2);
      }
    }
  }

  return 0.5 * cost;
}



/******************************************************************************

  \brief See header.

******************************************************************************/

void RcsGraph_jointLimitHessian(const RcsGraph* self, MatNd* ddH,
                                RcsStateType type)
{
  int dimension = (type == RcsStateFull ? self->dof : self->nJ);
  MatNd_reshape(ddH, dimension, dimension);
  MatNd_setZero(ddH);

  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    if ((JNT->jacobiIndex == -1) && (type == RcsStateIK))
    {
      continue;
    }

    double range = JNT->q_max - JNT->q_min;
    int index = (type == RcsStateIK) ? JNT->jacobiIndex : JNT->jointIndex;

    if ((range > 0.0) && (JNT->jacobiIndex != -1))
    {
      MatNd_set(ddH, index, index, JNT->weightJL / (range * range));
    }
  }

}



/******************************************************************************

  \brief Computes the partial derivative of the bodies rotation matrix with
         respect to a given joint.

         dA/dqi = A_left * dA_jnt * A_right

         with A_jnt,qi = the elementary rotation matrix according to the
         local joint axis (0 for non-rotations)
         A_left   = A_bdy-jnt
         dA_jnt   = d(A_jnt,qi)/dqi
         A_right  = trans(A_jnt,qi) * A_jnt-I

******************************************************************************/

static void RcsGraph_dAdq_i(const RcsGraph* self,
                            const RcsBody* b,
                            const RcsJoint* jnt,
                            double* dAdq)
{
  // Return zero matrix if joint is constrained or no rotation joint
  if ((jnt->jacobiIndex == -1) || (!RcsJoint_isRotation(jnt)))
  {
    VecNd_setZero(dAdq, 9);
    return;
  }

  // Get local rotation matrix from joint's q
  double qi = MatNd_get(self->q, jnt->jointIndex, 0);
  double A_q[3][3];
  Mat3d_setElementaryRotation(A_q, jnt->dirIdx, qi);

  // Get rotation matrix from root to joint without A_q. This is
  // A_right = trans(A_q) * A_jnt
  double A_right[3][3];
  Mat3d_transposeMul(A_right, A_q, (double (*)[3]) jnt->A_JI.rot);

  // Get rotation matrix from joint to body including A_q. This is
  // A_left = A_BI * A_Ijnt = A_BI * trans(A_jntI)
  double A_left[3][3];
  Mat3d_mulTranspose(A_left, b->A_BI->rot, (double (*)[3]) jnt->A_JI.rot);

  // Partial derivative of joint's local rotation matrix. Argument jnt->dirIdx
  // holds the joints elementary axis (x, y or z).
  double dA_jnt[3][3];
  Mat3d_dAdq(dA_jnt, jnt->dirIdx, qi);

  // Compute joint's rotation matrix partial derivative as
  // dAdq_i = A_left * dA_jnt * A_right
  double tmp[3][3], dAdq_i[3][3];
  Mat3d_mul(tmp, A_left, dA_jnt);
  Mat3d_mul(dAdq_i, tmp, A_right);

  VecNd_copy(dAdq, (const double*) dAdq_i, 9);
}



/******************************************************************************

  \brief This function computes the partial derivative of a bodies rotation
         matrix with respect to the state (q) vector. Argument dAdq is expected
         to have memory for 3*3*dof double values.

         The derivative is computed through all joints i as

         dA/dqi = A_left * dA_jnt * A_right

         with A_jnt,qi = the elementary rotation matrix according to the
         local joint axis (o for non-rotations)
         A_left   = A_bdy-jnt
         dA_jnt   = d(A_jnt,qi)/dqi
         A_right  = trans(A_jnt,qi) * A_jnt-I

         Here's how the memory is aligned:

         0          1               8         9               9*dof-1
         -------------------------------------------------------------------
         dA00/dq0   dA00/dq1   ...  dA00/dq8  dA01/dq0   ...  dA22/dq(dof-1)


         Is mapped to indices

                      / dA20/dq0   dA20/dq1   dA20/dq2 ... dA20/dq8 \
                     |                                               |
                     |  dA21/dq0   dA21/dq1   dA21/dq2 ... dA21/dq8  |
                     |                                               |
       l              \ dA22/dq0   dA22/dq1   dA22/dq2 ... dA22/dq8 /
     /
    /            / dA10/dq0   dA10/dq1   dA10/dq2 ... dA10/dq8 \
   /            |                                               |
  /_________ n  |  dA11/dq0   dA11/dq1   dA11/dq2 ... dA11/dq8  |
 |              |                                               |
 |               \ dA12/dq0   dA12/dq1   dA12/dq2 ... dA12/dq8 /
 |
 |        / dA00/dq0   dA00/dq1   dA00/dq2 ... dA00/dq8 \
m        |                                               |
         |  dA01/dq0   dA01/dq1   dA01/dq2 ... dA01/dq8  |
         |                                               |
          \ dA02/dq0   dA02/dq1   dA02/dq2 ... dA02/dq8 /

         This means: It's a 3rd order tensor with m x n x l = 3 x dof x 3
               For instance: arr dqA(3,dof,3);

         When transposed, the memory is aligned:

            0          1               8         9               9*dof-1
         -------------------------------------------------------------------
         dA00/dq0   dA01/dq0   ...  dA22/dq0  dA00/dq1   ...  dA22/dq(dof-1)

         Here's how the rotation matrix is aligned:

          / A00   A01   A02 \   ...
         |                   |
         |  A10   A11   A12  |  ...
         |                   |
          \ A20   A21   A22 /

******************************************************************************/

void RcsGraph_dAdq(const RcsGraph* self, const RcsBody* b, double* dAdq,
                   bool transposed)
{
  memset(dAdq, 0, 9*self->nJ*sizeof(double));

  if (b == NULL)
  {
    return;
  }

  const RcsJoint* jnt = RcsBody_lastJointBeforeBody(b);

  while (jnt != NULL)
  {

    if ((jnt->constrained==true) || (RcsJoint_isRotation(jnt)==false))
    {
      jnt = jnt->prev;
      continue;
    }

    double dAdq_i[3][3];
    RcsGraph_dAdq_i(self, b, jnt, (double*) dAdq_i);
    double* res = &dAdq_i[0][0];

    if (transposed == true)
    {
      memcpy(&dAdq[jnt->jacobiIndex * 9], res, 9 * sizeof(double));
    }
    else
    {
      for (int i = 0; i < 9; i++)
      {
        dAdq[self->nJ * i + jnt->jacobiIndex] = res[i];
      }
    }

    jnt = jnt->prev;
  }

}



/******************************************************************************

  \brief Cost function that penalizes if a point lies within the boundary
         of a frustrum. The frustrum is aligned with the body "cam". It's
         forward direction is x. Parameters:
         - cam     Body the frustrum is attached to
         - bdy     Body with respect to which the point is defined. If bdy is
                   NULL, the point is assumed to be represented in the world
                   frame
         - theta1  Frustrum half range in the x-z plane
         - theta2  Frustrum half range in the x-y plane
         - ratio1  Ratio of the x-z half range in which the quadratic cost is
                   zero. It's a "zero bassin" so to say.
         - ratio2  Ratio of the x-y half range in which the quadratic cost is
                   zero. It's a "zero bassin" so to say.
         - bdy_p   Point to be penalized, represented in bdy's coordinate
                   frame (world frame if bdy is NULL)

******************************************************************************/

double RcsGraph_pointFrustrumCost(const RcsBody* cam,
                                  const RcsBody* body,
                                  double theta1,
                                  double theta2,
                                  double ratio1,
                                  double ratio2,
                                  double bdy_p[3])
{
  RCHECK(cam);

  double p[3];   // Point in camera's coordinates

  const RcsBody* bdy = NULL;
  // check if body is GenericBody
  if (STRNEQ(body->name, "GenericBody", 11))
  {
    bdy = (RcsBody*) body->extraInfo;
  }
  else
  {
    bdy = body;
  }

  // If bdy exists, array bdy_p is assumed to be given in the bdy's frame. We
  // then transform it from bdy's frame to cam's frame
  if (bdy)
  {
    HTr A_CB;
    HTr_invTransform(&A_CB, bdy->A_BI, cam->A_BI);
    Vec3d_invTransform(p, &A_CB, bdy_p);
  }
  // If bdy is NULL, we transform bdy_p from the world frame to cam's frame
  else
  {
    Vec3d_invTransform(p, cam->A_BI, bdy_p);
  }

  // Angles phi1 in x-z plane and phi2 in x-y plane
  // double phi1 = atan(p[2]/p[0]) - theta1*ratio1;
  double phi2 = atan(p[1] / p[0]) - theta2 * ratio2;

  // Add cost terms
  double cost = 0.0;
  //if(phi1*p[2]>=0.0) cost += phi1*phi1;
  if (phi2 * p[1] >= 0.0)
  {
    cost += phi2 * phi2;
  }

  REXEC(6)
  {
    if (bdy)
    {
      RMSG("Cost for body %s is %f", bdy->name, cost);
    }
  }

  return cost;
}



/******************************************************************************

  \brief See header.

******************************************************************************/

void RcsGraph_pointFrustrumGradient(const RcsGraph* graph,
                                    const RcsBody* cam,
                                    const RcsBody* body,
                                    double theta1,
                                    double theta2,
                                    double ratio1,
                                    double ratio2,
                                    double bdy_p[3],
                                    MatNd* grad)
{
  RCHECK(cam);

  double p[3];   // Point vector in camera's coordinates

  const RcsBody* bdy = NULL;
  // check if body is GenericBody
  if (STRNEQ(body->name, "GenericBody", 11))
  {
    bdy = (RcsBody*) body->extraInfo;
  }
  else
  {
    bdy = body;
  }

  // If bdy exists, array bdy_p is assumed to be given in the bdy's frame. We
  // then transform it from bdy's frame to cam's frame
  if (bdy)
  {
    HTr A_CB;
    HTr_invTransform(&A_CB, bdy->A_BI, cam->A_BI);
    Vec3d_invTransform(p, &A_CB, bdy_p);
  }
  // If bdy is NULL, we transform bdy_p from the world frame to cam's frame
  else
  {
    Vec3d_invTransform(p, cam->A_BI, bdy_p);
  }

  // Angles phi1 in x-z plane and phi2 in x-y plane
  // double phi1 = atan(p[2]/p[0]) - theta1*ratio1;
  double phi2 = atan(p[1] / p[0]) - theta2 * ratio2;

  // Partial derivatives del(c) / del(e)
  // If the points are in the "flat" bassin, the gradient is zero. In
  // this case we leave the respective del(c) / del(phi) = 0
  double dcdphi1 = 0.0; //if(phi1*p[2]>=0.0) dcdphi1 = 2.0*phi1;
  double dcdphi2 = 0.0;
  if (phi2 * p[1] >= 0.0)
  {
    dcdphi2 = 2.0 * phi2;
  }

  // Partial derivatives del(e) / del(p)
  double dphi1dp[3], dphi2dp[3];
  double px2 = p[0] * p[0], py2 = p[1] * p[1], pz2 = p[2] * p[2];

  dphi1dp[0] = -p[2] / (px2 + pz2);
  dphi1dp[1] =  0.0;
  dphi1dp[2] =  p[0] / (px2 + pz2);

  dphi2dp[0] = -p[1] / (px2 + py2);
  dphi2dp[1] =  p[0] / (px2 + py2);
  dphi2dp[2] =  0.0;

  // Partial derivative dp/dq
  MatNd* dpdq  = NULL;
  MatNd_create2(dpdq, 3, graph->nJ);

  if (bdy)
  {
    MatNd* JT_k  = NULL;
    MatNd_create2(JT_k, 3, graph->nJ);
    MatNd* JT_p  = NULL;
    MatNd_create2(JT_p, 3, graph->nJ);
    MatNd* JT_kp = NULL;
    MatNd_create2(JT_kp, 3, graph->nJ);
    MatNd* JR_k  = NULL;
    MatNd_create2(JR_k, 3, graph->nJ);

    // JT_kp = JT_p  - JT_k
    RcsGraph_bodyPointJacobian(graph, cam, NULL, NULL, JT_k);
    RcsGraph_bodyPointJacobian(graph, bdy, bdy_p, NULL, JT_p);
    MatNd_sub(JT_kp, JT_p, JT_k);

    // I_rp: Transform body-fixed point vector to world frame
    double I_rp[3];
    Vec3d_copy(I_rp, bdy_p);
    Vec3d_transformSelf(I_rp, bdy->A_BI);

    // I_rp-I_rk: Delta vector from cam to point in world coords
    double I_dr[3];
    const double* I_rk = cam->A_BI->org;
    Vec3d_sub(I_dr, I_rp, I_rk);

    // JR_k x (I_rp-I_rk)
    RcsGraph_rotationJacobian(graph, cam, NULL, JR_k);
    MatNd_columnCrossProductSelf(JR_k, I_dr);

    // dpdq = A_BI * (JT_kp + JR_k x (I_r2-I_r1))
    MatNd_add(dpdq, JT_kp, JR_k);
    MatNd_rotateSelf(dpdq, cam->A_BI->rot);

    MatNd_destroy(JT_k);
    MatNd_destroy(JT_p);
    MatNd_destroy(JT_kp);
    MatNd_destroy(JR_k);
  }
  else // bdy is NULL, the point is assumed in world coordinates
  {
    RcsGraph_bodyPointJacobian(graph, cam, p, NULL, dpdq);
    MatNd_constMulSelf(dpdq, -1.0);
    MatNd_rotateSelf(dpdq, cam->A_BI->rot);
  }

  // Multiply gradients together
  double dcdp_buf[3];
  for (int i = 0; i < 3; i++)
  {
    dcdp_buf[i] = dcdphi1 * dphi1dp[i] + dcdphi2 * dphi2dp[i];
  }
  MatNd dcdp = MatNd_fromPtr(1, 3, &dcdp_buf[0]);

  MatNd_reshape(grad, 1, graph->nJ);
  MatNd_mul(grad, &dcdp, dpdq);

  // Clean up heap memory
  MatNd_destroy(dpdq);

  REXEC(6)
  {
    if (bdy)
    {
      RMSG("Gradient for body %s (cam is %s)", bdy->name, cam->name);
      MatNd_printDigits(grad, 2);
    }
  }

}



/******************************************************************************

  \brief The effort is the squared joint torque: 0.5 M^T M with M = J^T F

******************************************************************************/

double RcsGraph_staticEffort(const RcsGraph* self,
                             const RcsBody* bdy,
                             const MatNd* F,
                             const MatNd* W,
                             const double* k_pt)
{
  int n = self->nJ;
  MatNd* Jt=NULL, *M=NULL;

  MatNd_create2(Jt, 3, n);
  MatNd_create2(M, n, 1);

  // M = J^T F
  RcsGraph_bodyPointJacobian(self, bdy, k_pt, NULL, Jt);
  MatNd_transposeSelf(Jt);
  MatNd_mul(M, Jt, F);

  // effort = M^T M
  double buf;
  MatNd effort = MatNd_fromPtr(1, 1, &buf);
  MatNd_sqrMulAtBA(&effort, M, NULL);

  MatNd_destroy(Jt);
  MatNd_destroy(M);

  return 0.5 * effort.ele[0];
}



/******************************************************************************

  \brief The effort is the squared joint torque: M^T W M with M = J^T F
         Its derivative is

         d(0.5 M^T W M)/dq = M^T W dM/dq
                           = M^T W (H^T F)
                           = M^T W (F^T H)^T

         with H being dJ/dq.

******************************************************************************/

void RcsGraph_staticEffortGradient(const RcsGraph* self,
                                   const RcsBody* bdy,
                                   const MatNd* F,
                                   const MatNd* W,
                                   const double* k_pt,
                                   MatNd* grad)
{
  int n = self->nJ, nn = n * n;

  MatNd* Jt    = NULL;
  MatNd_create2(Jt, 3, n);
  MatNd* M     = NULL;
  MatNd_create2(M, n, 1);
  MatNd* H     = NULL;
  MatNd_create2(H, 3 * n, n);
  MatNd* dqJTF = NULL;
  MatNd_create2(dqJTF, n, n);

  // M = J^T F
  RcsGraph_bodyPointJacobian(self, bdy, k_pt, NULL, Jt);
  MatNd_transposeSelf(Jt);
  MatNd_mul(M, Jt, F);

  // (F^T H)^T
  int slice0 = 0, slice1 = nn, slice2 = 2 * nn;
  RcsGraph_bodyPointHessian(self, bdy, k_pt, NULL, H);

  for (int i = 0; i < nn; i++)
  {
    dqJTF->ele[i] += F->ele[0] * H->ele[slice0 + i]; // X
    dqJTF->ele[i] += F->ele[1] * H->ele[slice1 + i]; // Y
    dqJTF->ele[i] += F->ele[2] * H->ele[slice2 + i]; // Z
  }

  if (W)
  {
    MatNd_preMulDiagSelf(dqJTF, W);
  }

  MatNd_reshape(grad, dqJTF->m, M->n);
  MatNd_mul(grad, dqJTF, M);
  MatNd_transposeSelf(grad);

  MatNd_destroy(Jt);
  MatNd_destroy(M);
  MatNd_destroy(H);
  MatNd_destroy(dqJTF);
}



/******************************************************************************

  \brief Euler error partial derivative corresponding to HTr_getEulerError()

         de/dq = d(0.5*(n x nd + s x sd + a x ad))/dq

         =   (tilde(nd)^T tilde(sd)^T tilde(ad)^T) dA/dq
           + (tilde(n)    tilde(s)    tilde(a)) dAd/dq

         with n being the 1st row of the A_BI rotation matrix, s the second
         and a the third one. Index d stands for desired. Result grad is
         of dimension 3 x dof.

******************************************************************************/

void RcsGraph_dEulerErrorDq(const RcsGraph* self, const RcsBody* b_act,
                            const RcsBody* b_des, MatNd* grad)
{
  MatNd* dAdq = NULL;
  MatNd_create2(dAdq, 18, self->nJ);
  RcsGraph_dAdq(self, b_act, &dAdq->ele[0], false);
  RcsGraph_dAdq(self, b_des, &dAdq->ele[9 * self->nJ], false);

  // To make use of the contiguous memory regions, we construct the transpose
  // of del(e)/del(A) and transpose it afterwards.
  MatNd* tildeA = NULL;
  MatNd_create2(tildeA, 18, 3);
  double A_ActI[3][3], A_DesI[3][3], Id[3][3];
  Mat3d_setIdentity(Id);
  if (b_act)
  {
    Mat3d_copy(A_ActI, b_act->A_BI->rot);
  }
  else
  {
    Mat3d_setIdentity(Id);
  }
  if (b_des)
  {
    Mat3d_copy(A_DesI, b_des->A_BI->rot);
  }
  else
  {
    Mat3d_setIdentity(Id);
  }

  Mat3d_skew((double (*)[3]) &tildeA->ele[0],  A_DesI[0]);
  Mat3d_skew((double (*)[3]) &tildeA->ele[9],  A_DesI[1]);
  Mat3d_skew((double (*)[3]) &tildeA->ele[18], A_DesI[2]);
  Mat3d_transposeSkew((double (*)[3])&tildeA->ele[27], A_ActI[0]);
  Mat3d_transposeSkew((double (*)[3])&tildeA->ele[36], A_ActI[1]);
  Mat3d_transposeSkew((double (*)[3])&tildeA->ele[45], A_ActI[2]);
  MatNd_transposeSelf(tildeA);

  MatNd_mul(grad, tildeA, dAdq);
  MatNd_constMulSelf(grad, 0.5);

  MatNd_destroy(dAdq);
  MatNd_destroy(tildeA);
}
