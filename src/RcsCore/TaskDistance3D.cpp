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

#include "TaskDistance3D.h"
#include "TaskDistance.h"
#include "TaskFactory.h"
#include "Rcs_typedef.h"
#include "Rcs_macros.h"
#include "Rcs_parser.h"
#include "Rcs_utils.h"
#include "Rcs_math.h"
#include "Rcs_kinematics.h"
#include "Rcs_body.h"
#include "Rcs_shape.h"


static Rcs::TaskFactoryRegistrar<Rcs::TaskDistance3D> registrar("Distance3D");



/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::TaskDistance3D::TaskDistance3D(const std::string& className_,
                                    xmlNode* node,
                                    RcsGraph* _graph,
                                    int dim):
  TaskGenericIK(className_, node, _graph, dim)
{
  if (getClassName()=="Distance3D")
  {
    double guiMax[3], guiMin[3];
    Vec3d_set(guiMax, 2.5, 2.5, 2.5);
    Vec3d_set(guiMin, -2.5, -2.5, -2.5);
    getXMLNodePropertyVec3(node, "guiMax", guiMax);
    getXMLNodePropertyVec3(node, "guiMin", guiMin);

    resetParameter(Task::Parameters(guiMin[0], guiMax[0], 1.0, "X [m]"));
    addParameter(Task::Parameters(guiMin[1], guiMax[1], 1.0, "Y [m]"));
    addParameter(Task::Parameters(guiMin[2], guiMax[2], 1.0, "Z [m]"));
  }

}

/*******************************************************************************
 * Copy constructor doing deep copying
 ******************************************************************************/
Rcs::TaskDistance3D::TaskDistance3D(const TaskDistance3D& src,
                                    RcsGraph* newGraph):
  TaskGenericIK(src, newGraph)
{
}

/*******************************************************************************
 * Constructor based on body pointers
 ******************************************************************************/
Rcs::TaskDistance3D::TaskDistance3D(RcsGraph* graph_,
                                    const RcsBody* effector,
                                    const RcsBody* refBdy) : TaskGenericIK()
{
  this->graph = graph_;
  setClassName("Distance3D");
  setName("Dist3D " + std::string(effector ? effector->name : "NULL") + "-"
          + std::string(refBdy ? refBdy->name : "NULL"));
  setDim(3);
  setEffector(effector);
  setRefBody(refBdy);
  setRefFrame(refBdy);
  resetParameter(Task::Parameters(-1.0, 1.0, 1.0, "X [m]"));
  addParameter(Task::Parameters(-1.0, 1.0, 1.0, "Y [m]"));
  addParameter(Task::Parameters(-1.0, 1.0, 1.0, "Z [m]"));
}

/*******************************************************************************
 * Destructor
 ******************************************************************************/
Rcs::TaskDistance3D::~TaskDistance3D()
{
}

/*******************************************************************************
 * Clone function
 ******************************************************************************/
Rcs::TaskDistance3D* Rcs::TaskDistance3D::clone(RcsGraph* newGraph) const
{
  return new Rcs::TaskDistance3D(*this, newGraph);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskDistance3D::computeX(double* I_r) const
{
  double cpEf[3], cpRef[3];
  RcsBody_distance(this->refBody, this->ef, cpRef, cpEf, NULL);
  Vec3d_sub(I_r, cpEf, cpRef);
  Vec3d_rotateSelf(I_r, this->refBody->A_BI->rot);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskDistance3D::computeJ(MatNd* jacobian) const
{
  double cpEf[3], cpRef[3], nRE[3];
  RcsBody_distance(this->refBody, this->ef, cpRef, cpEf, nRE);

  // Delta Jacobian: Jp_ef-Jp_ef
  MatNd* J_ref = NULL;
  MatNd_create2(J_ref, 3, graph->nJ);
  MatNd* J_ef = NULL;
  MatNd_create2(J_ef, 3, graph->nJ);
  RcsGraph_worldPointJacobian(graph, refBody, cpRef, refBody->A_BI->rot, J_ref);
  RcsGraph_worldPointJacobian(graph, ef, cpEf, refBody->A_BI->rot, J_ef);
  MatNd_sub(jacobian, J_ef, J_ref);

  MatNd_destroy(J_ef);
  MatNd_destroy(J_ref);
}

/*******************************************************************************
 *
 *  Contact point Hessian for the articulated reference frame case:
 *  See RcsGraph_3dPosHessian()
 *
 *  H = dq(A_1I) (J2 - J1 + r12 x JR1) +            (Term 1)
 *      A_1I (dq(r_12 x) J_R1) +                    (Term 2)
 *      A_1I (H2 - H1) +                            (Term 3)
 *      A_1I ((r_12 x) HR1)                         (Term 4)
 *
 ******************************************************************************/
void Rcs::TaskDistance3D::computeH(MatNd* H) const
{
  // Closest points on effector and reference body in world coordinates
  double cpEf[3], cpRef[3], nRE[3];
  RcsBody_distance(this->refBody, this->ef, cpRef, cpEf, nRE);

  // Closest points in body coordinates k_p1 and k_p2 for the
  // closest point Jacobian Jp1 and Jp2
  Vec3d_invTransformSelf(cpEf, ef->A_BI);
  Vec3d_invTransformSelf(cpRef, refBody->A_BI);

  const int n = graph->nJ, n3 = n * 3, nn = n * n;
  const RcsBody* b1 = this->refBody;
  const RcsBody* b2 = this->ef;
  const RcsBody* b3 = this->refBody;
  const double* k_p2 = cpEf;
  const double* k_p1 = cpRef;

  // Workspace
  MatNd* bufH1 = NULL;
  MatNd_create2(bufH1, n3, n);
  MatNd* bufJ1 = NULL;
  MatNd_create2(bufJ1, 3, n);

  // del(A_3I)/del(q)
  MatNd* dA3 = NULL;
  MatNd_create2(dA3, n3, 3);
  RcsGraph_dAdq(graph, b3, &dA3->ele[0], true);

  // JT2 - JT1
  MatNd* J12   = NULL;
  MatNd_create2(J12, 3, graph->nJ);
  RcsGraph_bodyPointJacobian(graph, b1, k_p1, NULL, bufJ1);
  RcsGraph_bodyPointJacobian(graph, b2, k_p2, NULL, J12);
  MatNd_subSelf(J12, bufJ1);

  // JR3
  MatNd* JR3 = NULL;
  MatNd_create2(JR3, 3, n);
  RcsGraph_rotationJacobian(graph, b3, NULL, JR3);

  // I_r_12
  double r12[3];
  Vec3d_sub(r12, b2->A_BI->org, b1->A_BI->org);

  // Term 1: del(A_3I)/del(q) (J2 - J1 + r12 x JR3)
  MatNd_columnCrossProduct(bufJ1, JR3, r12);
  MatNd_addSelf(bufJ1, J12);
  MatNd_reshape(H, n3, n);
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
  RcsGraph_bodyPointHessian(graph, b2, k_p2, b3->A_BI->rot, bufH1);
  MatNd_addSelf(H, bufH1);
  RcsGraph_bodyPointHessian(graph, b1, k_p1, b3->A_BI->rot, bufH1);
  MatNd_subSelf(H, bufH1);

  // Term 4: A_3I ((r_12 x) HR3)
  MatNd* HR3 = bufH1;
  RcsGraph_rotationHessian(graph, b3, NULL, HR3);

  for (int j = 0; j < n; j++)
  {
    for (int k = 0; k < n; k++)
    {
      double col[3], dst[3];
      const int jnk = j * n + k;
      col[0] = HR3->ele[         jnk];
      col[1] = HR3->ele[    nn + jnk];
      col[2] = HR3->ele[2 * nn + jnk];
      Vec3d_crossProduct(dst, r12, col);
      Vec3d_rotateSelf(dst, b3->A_BI->rot);
      H->ele[         jnk] += dst[0];
      H->ele[    nn + jnk] += dst[1];
      H->ele[2 * nn + jnk] += dst[2];
    }
  }

  // Clean up
  MatNd_destroy(dA3);
  MatNd_destroy(bufJ1);
  MatNd_destroy(J12);
  MatNd_destroy(JR3);
  MatNd_destroy(bufH1);
}

/*******************************************************************************
 * This task required an effector and a reference body.
 ******************************************************************************/
bool Rcs::TaskDistance3D::isValid(xmlNode* node, const RcsGraph* graph)
{
  bool success = Task::isValid(node, graph, "Distance3D");
  success = Rcs::TaskDistance::hasDistanceFunction(node, graph) && success;

  return success;
}
