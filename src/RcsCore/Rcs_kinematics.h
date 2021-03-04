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

#ifndef RCS_KINEMATICS_H
#define RCS_KINEMATICS_H

#include "Rcs_graph.h"



#ifdef __cplusplus
extern "C" {
#endif


/*!
 * \defgroup RcsKinematicsFunctions Kinematics
 *
 *           Arrays are set to the proper dimensions by the functions.
 *
 *           Unless not stated differently, all pointers to bodies, joints,
 *           shapes  etc. are assumed to be contained in the RcsGraph if
 *           given as an argument. If a pointer to a body is NULL, the
 *           functions interpret it as the "world" body with origin zero and
 *           default alignment.
 *
 *           All rotation matrices are assumed to be in row-major form. It
 *           means that A_21 denotes a rotation of frame 1 into frame 2. It
 *           also means that the rows of matrix A_12 correspond to the unit
 *           vectors of frame 2, represented in frame 1.
 *
 */



/*! \ingroup RcsKinematicsFunctions
 *  \brief Computes the world coordinates of the body-fixed point
 *         b_p of body b.
 *
 *  \param[in]   body   Pointer to body. If it is NULL, the world reference
 *                      frame is assumed.
 *  \param[in]   B_pt   Point in the bodies coordinates. If it is NULL, a zero
 *                      vector is assumed.
 *  \param[out]  I_pt   Corresponding point in world coordinates. If body is
 *                      NULL and B_pt is not NULL, B_pt is copied to I_pt,
 *                      otherwise I_pt is set to zero.
 */
void RcsGraph_bodyPoint(const RcsBody* body, const double B_pt[3],
                        double I_pt[3]);

/*! \ingroup RcsKinematicsFunctions
 *  \brief Computes the translation Jacobian of a body fixed B_pt of a body
 *         with respect to the world (I) reference frame.
 *
 *  \param[in]   graph  Pointer to RcsGraph that contains the body.
 *  \param[in]   body   Pointer to body. If body is  NULL, the result is a
 *                      zero-Jacobian.
 *  \param[in]   B_pt   Point in the bodies coordinates. If it is NULL, a zero
 *                      vector is assumed.
 *  \param[in]   A_BI   Optional rotation  matrix from world (I) to body (B)
 *                      frame (row-major). If it is not NULL, the  Jacobian
 *                      is rotated into the B-frame.
 *  \param[out]  I_J_pt Translation Jacobian of the point in world
 *                      coordinates. It will be reshaped to 3 x RcsGraph::nJ.
 */
void RcsGraph_bodyPointJacobian(const RcsGraph* graph, const RcsBody* body,
                                const double B_pt[3], double A_BI[3][3],
                                MatNd* I_J_pt);

/*! \ingroup RcsKinematicsFunctions
 *  \brief Same as \ref RcsGraph_bodyPointJacobian(), except that argument
 *         I_pt is represented in world coordinates. This is particularly
 *         efficient for distance Jacobians, where contact points are
 *         determined in world coordinates.
 *
 *  \param[in]   graph  Pointer to RcsGraph that contains the body.
 *  \param[in]   body   Pointer to body. If body is  NULL, the result is a
 *                      zero-Jacobian.
 *  \param[in]   I_pt   Point in the bodies coordinates. If it is NULL, a zero
 *                      vector is assumed.
 *  \param[in]   A_BI   Optional rotation  matrix from world (I) to body (B)
 *                      frame (row-major). If it is not NULL, the  Jacobian
 *                      is rotated into the B-frame.
 *  \param[out]  I_J_pt Translation Jacobian of the point in world
 *                      coordinates. It will be reshaped to 3 x RcsGraph::nJ.
 */
void RcsGraph_worldPointJacobian(const RcsGraph* graph, const RcsBody* body,
                                 const double I_pt[3], double A_BI[3][3],
                                 MatNd* I_J_pt);

/*! \ingroup RcsKinematicsFunctions
 *  \brief Computes the Hessian
 *         \f$
 *         \mathbf{_I H_{pt}} =
 *         \mathbf{\frac{\partial^2 (_I x_{pt})}{\partial q^2}} =
 *         \mathbf{\frac{\partial (_I J_{pt})}{\partial q}}
 *         \f$
 *          of the bodies local reference point B_pt. The tensor is indexed
 *          like this:
 \verbatim
         0         1             n-1        n             (m-1)*(n-1)
         -------------------------------------------------------------------
         dJ00/dq0 dJ00/dq1 ... dJ00/dq(n-1) dJ01/dq0 ... dJ(m-1)(n-1)/dq(n-1)
 \endverbatim
 *
 *  \param[in]   graph  Pointer to RcsGraph that contains the body.
 *  \param[in]   body   Pointer to body. If body is  NULL, the result is a
 *                      3n x n zero-Hessian (with n being \ref RcsGraph::nJ).
 *  \param[in]   B_pt   Point in the bodies coordinates. If it is NULL, a zero
 *                      vector is assumed.
 *  \param[in]   A_BI   Optional rotation  matrix from world (I) to body (B)
 *                      frame (row-major). If it is not NULL, the  Jacobian
 *                      is rotated into the B-frame.
 *  \param[out]  I_H_pt Translation Hessian of the point, represented in world
 *                      coordinates. It will be reshaped to 3n x n (with n
 *                      being \ref RcsGraph::nJ).
 */
void RcsGraph_bodyPointHessian(const RcsGraph* graph, const RcsBody* body,
                               const double B_pt[3], double A_BI[3][3],
                               MatNd* I_H_pt);

/*! \ingroup RcsKinematicsFunctions
 *  \brief Computes the body point dot Jacobian according to
 *         \f$
 *         \mathbf{\dot{_I J_{pt}}} = \mathbf{\dot{q}^T \; _I H_{pt}}
 *         \f$
 *         where the Hessian H is determined by the function
 *         \ref RcsGraph_bodyPointHessian().
 *         If the number of active dofs is larger than
 *         MATND_MAX_STACK_VECTOR_SIZE doubles, heap memory will be
 *         allocated.
 *
 *  \param[in]   graph  Pointer to RcsGraph that contains the body.
 *  \param[in]   body   Pointer to body. If body is  NULL, the result is a
 *                      3 x n zero matrix (with n being
 *                      \ref RcsGraph::nJ).
 *  \param[in]   B_pt   Point in the bodies coordinates. If it is NULL, a zero
 *                      vector is assumed.
 *  \param[in]   A_BI   Optional rotation  matrix from world (I) to body (B)
 *                      frame (row-major). If it is not NULL, the  Jacobian
 *                      is rotated into the B-frame.
 *  \param[in]   qp     Joint velocity vector. It must be of dimension
 *                      RcsGraph::nJ x 1.
 *  \param[out]  J_dot  Dot Jacobian of the point, represented in world
 *                      coordinates. It will be reshaped to 3n x n (with n
 *                      being \ref RcsGraph::nJ).
 */
void RcsGraph_bodyPointDotJacobian(const RcsGraph* graph, const RcsBody* body,
                                   const double B_pt[3], double A_BI[3][3],
                                   const MatNd* qp, MatNd* J_dot);

/*! \ingroup RcsKinematicsFunctions
 *  \brief Computes the rotation Jacobian of a given body with respect to the
 *         world (I) reference frame.
 *
 *  \param[in]   graph   Pointer to RcsGraph that contains the body.
 *  \param[in]   body    Pointer to body. If body is  NULL, the result is a
 *                       zero-Jacobian.
 *  \param[in]   A_BI    Optional rotation matrix from world (I) to body (B)
 *                       frame (row-major). If it is not NULL, the Jacobian
 *                       is rotated into the B-frame.
 *  \param[out]  I_J_rot Rotation Jacobian in world coordinates. It will be
 *                       reshaped to 3 x RcsGraph::nJ.
 */
void RcsGraph_rotationJacobian(const RcsGraph* graph, const RcsBody* body,
                               double A_BI[3][3], MatNd* I_J_rot);

/*! \ingroup RcsKinematicsFunctions
 *  \brief Computes the rotation Jacobian derivative for a given body. See
 *         \ref RcsGraph_bodyPointHessian() for refecence.
 *
 *  \param[in]   graph   Pointer to RcsGraph that contains the body.
 *  \param[in]   body    Pointer to body. If body is  NULL, the result is a
 *                       3n x n zero-Hessian (with n being \ref RcsGraph::nJ).
 *  \param[in]   A_BI    Optional rotation  matrix from world (I) to body (B)
 *                       frame (row-major). If it is not NULL, the  Jacobian
 *                       is rotated into the B-frame.
 *  \param[out]  I_H_rot Rotation Hessian of the point, represented in world
 *                       coordinates. It is reshaped to 3n x n dimensions,
 *                       where n is the number of unconstrained dofs
 *                       (RcsGraph::nJ)
 */
void RcsGraph_rotationHessian(const RcsGraph* graph, const RcsBody* body,
                              double A_BI[3][3], MatNd* I_H_rot);

/*! \ingroup RcsKinematicsFunctions
 *  \brief See \ref RcsGraph_bodyPointDotJacobian() for reference.
 *
 *  \param[in]   graph  Pointer to RcsGraph that contains the body.
 *  \param[in]   body   Pointer to body. If body is  NULL, the result is a
 *                      3 x n zero matrix (with n being \ref RcsGraph::nJ).
 *  \param[in]   qp     Joint velocity vector. It must be of dimension
 *                      RcsGraph::nJ x 1.
 *  \param[out]  J_dot  Dot Jacobian of the point, represented in world
 *                      coordinates. It will be reshaped to 3n x n (with n
 *                      being \ref RcsGraph::nJ).
 */
void RcsGraph_rotationDotJacobian(const RcsGraph* graph, const RcsBody* body,
                                  const MatNd* qp, MatNd* J_dot);

/*! \ingroup RcsKinematicsFunctions
 *  \brief Position XYZ Jacobian. Effector relative to reference body, both
 *         in fixed and moving references. Index 1 denotes the effector body,
 *         index 2 the reference body (refBdy), and index 3 the reference
 *         frame (refFrame). The Jacobian is:
 *         \f[
 *         \mathbf{J} = \mathbf{ A_{3I}
 *                    (J_{T,2}-J_{T,1} + J_{R,1} \times ({_I}r_2-{_I}r_1))}
 *         \f]
 *         with \f$\mathbf{J_T}\f$ being the linear and \f$\mathbf{J_R}\f$
 *         being the rotational Jacobian.
 *         If refBdy is NULL, refFrame must also be NULL. If refBdy is not
 *         NULL and the number of active dofs is larger than
 *         MATND_MAX_STACK_VECTOR_SIZE doubles, heap memory will be
 *         allocated. The function is written quite efficiently and will skip
 *         terms according to NULL arguments.
 */
void RcsGraph_3dPosJacobian(const RcsGraph* self, const RcsBody* effector,
                            const RcsBody* refBdy, const RcsBody* refFrame,
                            MatNd* J);

/*! \ingroup RcsKinematicsFunctions
 *  \brief Partial derivative of RcsGraph_3dPosJacobian(). It will be rehaped
 *         to the dimension [3*nJ] x [nJ], where nJ is the graph's number of
 *         unconstrained degrees of freedom.
 *         \f[
 *         \mathbf{H} = \mathbf{
 *                      \frac{\partial  A_{3I}} {\partial q}
 *                      (J_{T,2}-J_{T,1}+J_{R,1} \times {_I}r_{12} ) +
 *                      A_{3I} \left(H_{T,2}-H_{T,1} -
 *                      \frac{\partial (r_{12} \times)}{\partial q} J_{R,1}
 *                      - r_{12} \times H_{R,1}) \right)
 *                             }
 *         \f]
 *
 *         H=dq(A_3I)(J2-J1+r12xJR1)+A_1I(H2-H1+dq(r_12x)J_R1+(r_12x)HR1)
 */
void RcsGraph_3dPosHessian(const RcsGraph* self, const RcsBody* effector,
                           const RcsBody* refBdy, const RcsBody* refFrame,
                           MatNd* H);

/*! \ingroup RcsKinematicsFunctions
 *  \brief The index-th row of the 3d position Jacobian. Index must be
 *         0 (x), 1 (y) or 2 (z), otherwise the function exits fatally.
 *
 *         If refBdy is not NULL or the number of active dofs is larger than
 *         MATND_MAX_STACK_VECTOR_SIZE doubles, heap memory will be
 *         allocated.
 */
void RcsGraph_1dPosJacobian(const RcsGraph* self, const RcsBody* effector,
                            const RcsBody* refBdy, const RcsBody* refFrame,
                            int index, MatNd* J);

/*! \ingroup RcsKinematicsFunctions
 *  \brief The index-th matrix of the 3d position Hessian. Index must be
 *         0 (x), 1 (y) or 2 (z).
 */
void RcsGraph_1dPosHessian(const RcsGraph* self, const RcsBody* effector,
                           const RcsBody* refBdy, const RcsBody* refFrame,
                           int index, MatNd* H);

/*! \ingroup RcsKinematicsFunctions
 *  \brief Angular velocity Jacobian. Effector relative to reference body,
 *         both in fixed and moving references.
 *
 *         J = A_3I * (JR_2 - JR_1)
 *
 *         If refBdy is not NULL and the number of active dofs is larger than
 *         MATND_MAX_STACK_VECTOR_SIZE doubles, heap memory will be
 *         allocated.
 */
void RcsGraph_3dOmegaJacobian(const RcsGraph* self, const RcsBody* effector,
                              const RcsBody* refBdy, const RcsBody* refFrame,
                              MatNd* J);

/*! \ingroup RcsKinematicsFunctions
 *  \brief Partial derivative of RcsGraph_3dOmegaJacobian(). It will be rehaped
 *         to the dimension [3*nJ] x [nJ], where nJ is the graph's number of
 *         unconstrained degrees of freedom.
 *
 *         H = dq(A_3I) (JR_2 - JR_1) + A_3I (H2 - H1)
 */
void RcsGraph_3dOmegaHessian(const RcsGraph* self, const RcsBody* effector,
                             const RcsBody* refBdy, const RcsBody* refFrame,
                             MatNd* H);

/*! \ingroup RcsKinematicsFunctions
 *  \brief Computes the overall center of gravity of the graph in world
 *         coordinates. The function returns the mass of the tree
 *         starting from the root body.
 */
double RcsGraph_COG(const RcsGraph* self, double r_cog[3]);

/*! \ingroup RcsKinematicsFunctions
 *  \brief Computes the overall center of gravity of the graph with the given
 *         body as root node in world coordinates. The function returns the
 *         mass of the tree starting from body.
 */
double RcsGraph_COG_Body(const RcsGraph* self, const RcsBody* body,
                         double r_cog[3]);

/*! \ingroup RcsKinematicsFunctions
 *  \brief Computes the center of gravity Jacobian of the full graph.
 */
void RcsGraph_COGJacobian(const RcsGraph* self, MatNd* J_cog);

/*! \ingroup RcsKinematicsFunctions
 *  \brief Computes the center of gravity Jacobian of the sub-graph starting
 *         at the given body. The Hessian will be reshaped to 3n x n with n
 *         being the unconstrained dof.
 */
void RcsGraph_COGJacobian_Body(const RcsGraph* self, const RcsBody* body,
                               MatNd* J_cog);

/*! \ingroup RcsKinematicsFunctions
 *  \brief Computes the center of gravity Hessian of the full graph.
 */
void RcsGraph_computeCOGHessian(const RcsGraph* self, double* H);

/*! \ingroup RcsKinematicsFunctions
 *  \brief Computes the center of gravity Hessian of the sub-graph starting
 *         at the given body.
 */
void RcsGraph_computeCOGHessian_Body(const RcsGraph* self,
                                     const RcsBody* body, MatNd* H);

/*! \ingroup RcsKinematicsFunctions
 *  \brief Same as \ref RcsGraph_computeCOGHessian_Body, but buf is used for
 *         intermediate results. There is guaranteed no dynamic memory
 *         allocation. The size of buf must be >= 3*n*n, where n is the
 *         number of unconstrained dofs. The Hessian will be reshaped to
 *         3n x n.
 */
void RcsGraph_computeCOGHessian_Body_(const RcsGraph* self,
                                      const RcsBody* body, MatNd* H,
                                      MatNd* buf);

/*! \ingroup RcsKinematicsFunctions
 *  \brief Returns the joint limit cost. It is the squared sum of the
 *         deviations to the joint centers, normalized with respect to
 *         the joint range: <br>
 *         c = 0.5*sum{ ((q-q0)/(qmax-qmin))^2 }  <br>
 *         only elements that contribute to the IK are considered. If
 *         the joint range is 0, the joint is ignored.
 */
double RcsGraph_jointLimitCost(const RcsGraph* self,
                               RcsStateType type);

/*! \ingroup RcsKinematicsFunctions
 *  \brief Similar to RcsGraph_jointLimitCost, but only the border
 *         of the joint's range are respected. Independent of q0.
 */
double RcsGraph_jointLimitCostPlateau(const RcsGraph* self,
                                      const double border_ratio,
                                      const RcsStateType type);

/*! \ingroup RcsKinematicsFunctions
 *  \brief Returns the joint limit cost. It is the squared sum of the
 *         deviations to the joint centers, normalized with respect to
 *         the joint half range. Scalar freeRatio is a ratio of the
 *         joint's half range where the cost is zero. Beyond this point,
 *         the cost is parabolic. There's a test function in the bin
 *         directory.
 */
double RcsGraph_jointLimitBorderCost(const RcsGraph* self,
                                     const double freeRatio,
                                     const RcsStateType type);

/*! \ingroup RcsKinematicsFunctions
 *  \brief Gradient according to RcsGraph_jointLimitBorderCost(). The
 *         gradient is a row vector.
 */
void RcsGraph_jointLimitBorderGradient(const RcsGraph* self,
                                       MatNd* dH,
                                       double freeRatio,
                                       RcsStateType type);

/*! \ingroup RcsKinematicsFunctions
 *  \brief Hessian according to RcsGraph_jointLimitBorderCost().
 */
void RcsGraph_jointLimitBorderHessian(const RcsGraph* self,
                                      MatNd* dH,
                                      double freeRatio,
                                      RcsStateType type);

/*! \ingroup RcsKinematicsFunctions
 *  \brief Computes the joint limit gradient according to the above
 *         documented cost, which is returned: <br>
 *         dc/dqi = (q-q0)/((qmax-qmin)^2) <br>
 *         It consists of all state vector elements and such needs
 *         to be reduced before being applied to the inverse
 *         kinematics coputation. The elements with active constraints
 *         are set to zero. The array is reshaped to match the proper
 *         dimensions.
 */
double RcsGraph_jointLimitGradient(const RcsGraph* self, MatNd* dH,
                                   RcsStateType type);

/*! \ingroup RcsKinematicsFunctions
 *  \brief Computes the joint limit gradient according to the above
 *         documented cost, which is returned: <br>
 *         dc/dqi = (q-q0)/((qmax-qmin)^2) <br>
 *         It consists of all state vector elements and such needs
 *         to be reduced before being applied to the inverse
 *         kinematics coputation. The elements with active constraints
 *         are set to zero. The array is reshaped to match the proper
 *         dimensions.
 */
double RcsGraph_jointLimitGradientPlateau(const RcsGraph* self, MatNd* dH,
                                          const double border_ratio,
                                          RcsStateType type);

/*! \ingroup RcsKinematicsFunctions
 *  \brief Computes the joint limit Hessian according to the above
 *         documented cost: <br>
 *         d^2c/dqi^2 = 1/((qmax-qmin)^2) <br>
 *         It consists only of the elements that are relevant for the
 *         inverse kinematics computation.
 */
void RcsGraph_jointLimitHessian(const RcsGraph* self, MatNd* ddH,
                                RcsStateType type);

/*! \ingroup RcsKinematicsFunctions
 *  \brief This function computes the partial derivative of a bodies rotation
 *         matrix with respect to the IK-relevant state (q) vector. Argument
 *         dAdq is expected to have memory for 3*3*dof double values. The
 *         tensor is indexed like
 \verbatim
              0          1             8         9               9*dof-1
           -----------------------------------------------------------------
           dA00/dq0   dA00/dq1  ... dA00/dq8  dA01/dq0   ...  dA22/dq(dof-1)
 \endverbatim
 *
 */
void RcsGraph_dAdq(const RcsGraph* self, const RcsBody* b, double* dAdq,
                   bool transposed);

/*! \ingroup RcsKinematicsFunctions
 *  \brief Cost function that penalizes if a point lies within the boundary
 *         of a frustrum. The frustrum is aligned with the body "cam". It's
 *         forward direction is x. The body can be defined in a body's (bdy)
 *         coordinate frame. In this case, the point's velocity according
 *         to the bdy's movement is computed consistently. An example is a
 *         point on an object that is held in the hand of a robot.
 *         Parameters:
 *         - cam     Body the frustrum is attached to
 *         - bdy     Body with respect to which the point is defined. If bdy
 *                   is NULL, the point is assumed to be represented in the
 *                   world frame
 *         - theta1  Frustrum half range in the x-z plane
 *         - theta2  Frustrum half range in the x-y plane
 *         - ratio1  Ratio of the x-z half range in which the quadratic cost
 *                   is zero. It's a "zero bassin" so to say.
 *         - ratio2  Ratio of the x-y half range in which the quadratic cost
 *                   is zero. It's a "zero bassin" so to say.
 *         - bdy_p   Point to be penalized, represented in bdy's coordinate
 *                   frame (world frame if bdy is NULL)
 */
double RcsGraph_pointFrustrumCost(const RcsBody* cam, const RcsBody* body,
                                  double theta1, double theta2,
                                  double ratio1, double ratio2,
                                  double bdy_p[3]);

/*! \ingroup RcsKinematicsFunctions
 *  \brief The gradient of the cost function computed in
 *         RcsGraph_pointFrustrumCost() with respect to the q-vector.
 */
void RcsGraph_pointFrustrumGradient(const RcsGraph* graph,
                                    const RcsBody* cam, const RcsBody* body,
                                    double theta1, double theta2,
                                    double ratio1, double ratio2,
                                    double bdy_p[3], MatNd* grad);

/*! \ingroup RcsKinematicsFunctions
 *  \brief Computes the (static) squared joint torque (effort): <br>
 *         effort = 0.5 M^T W M with M = J^T F. <br>
 *         Array W is a dim(q_ik) x 1 weighting vector which constitues to
 *         a diagonal dim(q_ik) x dim(q_ik) weighting matrix. If W is NULL,
 *         a unit weighting matrix is assumed. Array F is a 3 x 1 force
 *         vector represented in world coordinates, which is applied to bdy
 *         with offset k_pt (given in bdy's coordinates). If k_pt is NULL,
 *         the force is assumed to act to the  origin of bdy.
 */
double RcsGraph_staticEffort(const RcsGraph* self, const RcsBody* bdy,
                             const MatNd* F, const MatNd* W,
                             const double* k_pt);

/*! \ingroup RcsKinematicsFunctions
 *  \brief Computes the partial derivative of the (static) squared joint
 *         torque with respect to the IK-relevant state vector: <br>
 *         d(0.5 M^T W M)/dq = M^T W (F^T dq(J))^T. <br>
 *         Array W is a dim(q_ik) x 1 weighting vector which constitutes to
 *         a diagonal dim(q_ik) x dim(q_ik) weighting matrix. If W is NULL,
 *         a unit weighting matrix is assumed. Array F is a 3 x 1 force
 *         vector represented in world coordinates, which is applied to bdy
 *         with offset k_pt (given in bdy's coordinates). If k_pt is NULL,
 *         the force is assumed to act to the  origin of bdy.
 */
void RcsGraph_staticEffortGradient(const RcsGraph* self, const RcsBody* bdy,
                                   const MatNd* F, const MatNd* W,
                                   const double* k_pt, MatNd* dMtMdq);

/*! \ingroup RcsKinematicsFunctions
 *  \brief Euler error partial derivative corresponding to: <br>
 *         d(0.5*(n x nd + s x sd + a x ad))/dq. <br>
 *         with n being the 1st row of the A_KI rotation matrix, s the second
 *         and a the third one. Index d stands for desired. Result grad is
 *         of dimension 3 x dof.
 */
void RcsGraph_dEulerErrorDq(const RcsGraph* self, const RcsBody* act,
                            const RcsBody* des, MatNd* dedq);


#ifdef __cplusplus
}
#endif

#endif   // RCS_KINEMATICS_H
