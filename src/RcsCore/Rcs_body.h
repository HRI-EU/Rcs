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

#ifndef RCS_BODY_H
#define RCS_BODY_H

#include "Rcs_graph.h"


#ifdef __cplusplus
extern "C" {
#endif


/*!
 * \defgroup RcsBodyFunctions Bodies
 */


/*! \ingroup RcsBodyFunctions
 *  \brief Deletes the body and frees all memory. The function does nothing
 *         if self is NULL.
 */
void RcsBody_destroy(RcsBody* self);

/*! \ingroup RcsBodyFunctions
 *  \brief Creates a shallow copy of the body (Shapes etc. are not copied).
 *         These members are left unchanged and need to be handled separately:
 *         - parent
 *         - child
 *         - last
 *         - next
 *         - prev
 *         - shape
 *         - jnt
 *         - actor
 *         - extraInfo
 *         - extraInfoVortex
 */
void RcsBody_copy(RcsBody* dst, const RcsBody* src);

/*! \ingroup RcsBodyFunctions
*  \brief Returns a clone of the body, and relates it to the graph. The
*         following elements are cloned:
*         - All body joints.
*         - All body shapes.
*         The following elements need to be considered for fully linking the
*         body into a new graph: Joint connection to prev. body and from next
*         body, coupled joints, body depth first pointers.
*
* \param[in] src        Body to be cloned.
*  \return Clone of the body, or NULL on failure.
*/
RcsBody* RcsBody_clone(const RcsBody* src);


/*! \ingroup RcsBodyFunctions
 *  \brief Returns the number of joints by which the body is attached to its
 *         predecessor.
 */
unsigned int RcsBody_numJoints(const RcsBody* b);

/*! \ingroup RcsBodyFunctions
 *  \brief Returns the number of shapes attached to the body. If self is NULL,
 *         the function returns 0.
 */
unsigned int RcsBody_numShapes(const RcsBody* self);

/*! \ingroup RcsBodyFunctions
 *  \brief Returns the number of shapes attached to the body that go into
 *         the distance computation.
 */
unsigned int RcsBody_numDistanceShapes(const RcsBody* self);

/*! \ingroup RcsBodyFunctions
 *  \brief Appends a shape to the bodie's shapes. The shapes array will be
 *         resized if needed (using realloc).
 */
void RcsBody_addShape(RcsBody* self, RcsShape* shape);

/*! \ingroup RcsBodyFunctions
 *  \brief Returns the last body in the graph according to a depth-first
 *         traversal. If the graph is empty, NULL is returned.
 */
RcsBody* RcsBody_getLastInGraph(const RcsGraph* self);

/*! \ingroup RcsBodyFunctions
 *  \brief Returns the graph's root body, by first traversing the bodie's
 *         parents, and then the bodie's previous bodies. If the body is
 *         NULL, the function also returns NULL.
 */
RcsBody* RcsBody_getGraphRoot(const RcsBody* self);

/*! \ingroup RcsBodyFunctions
 *  \brief Attaches body to a target body. If A_KV is NULL, the identity
 *         transform is assumed. It is not supported to attach a body to a
 *         target body that is a generic body. In this case, the function
 *         will exit fatally. If the body is a generic body, the function
 *         attaches the body it is pointing to.
 *
 *  \param[in,out] graph   RcsGraph whose structure is to be changed
 *  \param[in]     body    RcsBody that is going to be attached to the target
 *  \param[in]     target  RcsBody which will have the body as child. If it is
 *                         NULL, body will be attached to the root.
 *  \param[in]     A_BP    Relative transformation from target to body. If it
 *                         is NULL, an identity transformation is used.
 *  \return true for success, false if body is NULL or the target is a
 *          GenericBody.
 */
bool RcsBody_attachToBody(RcsGraph* graph, RcsBody* body, RcsBody* target,
                          const HTr* A_BP);

/*! \ingroup RcsBodyFunctions
 *  \brief Returns true if child is a child of parent, false otherwise.
 */
bool RcsBody_isChild(const RcsBody* possibleChild,
                     const RcsBody* possibleParent);

/*! \ingroup RcsBodyFunctions
*  \brief Returns true if bdy is a leaf node, false otherwise.
*/
bool RcsBody_isLeaf(const RcsBody* bdy);

/*! \ingroup RcsBodyFunctions
 *  \brief Returns true if the body is connected to the root node having at
 *         least one unconstrained joint, false otherwise. If self is NULL,
 *         the function returns false.
 */
bool RcsBody_isArticulated(const RcsBody* self);

/*! \ingroup RcsBodyFunctions
 *  \brief Returns true if the body is part of the graph, false otherwise.
 */
bool RcsBody_isInGraph(const RcsBody* self, const RcsGraph* graph);

/*! \ingroup RcsBodyFunctions
 *  \brief Prints out the body to a file descriptor.
 */
void RcsBody_fprint(FILE* fd, const RcsBody* b);

/*! \ingroup RcsBodyFunctions
 *  \brief Returns the volume of the body including all shapes in cubic
 *         meter. See RcsShape_computeVolume() for details.
 */
double RcsBody_computeVolume(const RcsBody* self);

/*! \ingroup RcsBodyFunctions
 *  \brief Computes the inertia tensor of the body around its COM, rotated
 *         into its local frame of reference, by traversing all attached
 *         shapes and summing up their contributions. The shapes relative
 *         transform A_CB is considered. If the bodie's mass or volume is
 *         zero, the inertia tensor is also zero.
 */
void RcsBody_computeInertiaTensor(const RcsBody* self, HTr* InertiaTensor);

/*! \ingroup RcsBodyFunctions
 *  \brief Computes the distance of the closest points of two objects. It
 *         is determined as the shortest distance between the two most
 *         proximate geometrical shape primitives that are associated with
 *         the object. Vectors cp1 and cp2 hold the closest points of the
 *         respective object in world coordinates. Vector n holds the
 *         normal on body b1. The arguments cp1, cp2 and n may be NULL.
 *         In this case, only the distance is returned. The following
 *         proximity pair computations are supported, also in
 *         reverse order:
 *         - SSL - SSL
 *         - SSL - SSR
 *         - SSR - SSR
 *         - SSL - BOX
 *         - SSR - BOX
 *         - BOX - BOX
 */
double RcsBody_distance(const RcsBody* b1, const RcsBody* b2,
                        double cp1[3], double cp2[3], double n12[3]);

/*! \ingroup RcsBodyFunctions
 *  \brief Computes the distance between a body and a 3d point.
 *
 *  \param[in]  body    RcsBody to which distance is to be computed
 *  \param[in]  I_pt    Point in world coordinates
 *  \param[out] I_cpBdy Closest point on the body in world coordinates
 *  \param[out] I_nBdyPt Unit normal vector from body towards point
 */
double RcsBody_distanceToPoint(const RcsBody* body, const double I_pt[3],
                               double I_cpBdy[3], double I_nBdyPt[3]);

/*! \ingroup RcsBodyFunctions
 *  \brief Computes the distance of the center points of two objects. The
 *         used center points are the bodie's origin, with an offset given
 *         in body coordinates in vectors k_p1 and k_p2. If these vectors
 *         point to NULL, a zero offset is assumed.
 *         The distance is returned, the vectors I_cp1 and I_cp2 hold the
 *         center points of the respective object in world coordinates.
 *         Arguments I_cp1 and cI_p2 may be NULL. In this case, only the
 *         distance is returned.
 */
double RcsBody_centerDistance(const RcsBody* b1, const RcsBody* b2,
                              const double k_p1[3], const double k_p2[3],
                              double I_cp1[3], double I_cp2[3]);

/*! \ingroup RcsBodyFunctions
 *  \brief Computes the distance gradient between two bodies. Vector I_p1 is
 *         a relative point wrt. body b1, I_p2 wrt. body 2. Vectors I_p1 and
 *         I_p2 are represented in world coordinates. If they are NULL, the
 *         origin of the respective body is used. The distance gradient
 *         is copied into dDdq.
 */
void RcsBody_distanceGradient(const RcsGraph* self, const RcsBody* b1,
                              const RcsBody* b2, bool repelling,
                              const double* I_p1, const double* I_p2,
                              const double* I_n12, MatNd* dDdq);

/*! \ingroup RcsBodyFunctions
 *  \brief Computes the distance Hessian between two bodies. Vector I_p1 is
 *         a relative point wrt. body b1, I_p2 wrt. body 2. Vectors I_p1 and
 *         I_p2 are represented in world coordinates. If they are NULL, the
 *         origin of the respective body is used. The distance Hessian
 *         is copied into H.
 */
void RcsBody_distanceHessian(const RcsGraph* self, const RcsBody* b1,
                             const RcsBody* b2, bool repelling,
                             const double* I_p1, const double* I_p2,
                             double* H);

/*! \ingroup RcsBodyFunctions
 *  \brief Computes the collision cost between two bodies. The cost is
 *         computed as
 *         \f{eqnarray*}{
 *         c & = & 0 \hspace{17mm} \mbox{ for } d>d_b \\
 *           & = & \frac{s}{d_b^2}(d-d_b)^2 \mbox{ for } 0<d<d_b \\
 *           & = & s (1 -2 \frac{d}{d_b} ) \hspace{2mm} \mbox{ for } d<0
 *         \f}
 *         where s is the inclination of the cost at the point of penetration,
 *         d is the distance of the bodie's closest points, and \f$ d_b \f$
 *         is a distance threshold.
 */
double RcsBody_collisionCost(const RcsBody* b1,
                             const RcsBody* b2,
                             double dThreshold);

/*! \ingroup RcsBodyFunctions
 *  \brief Computes the collision gradient between two bodies. The
 *         gradient's dimensions are that of the ik-relevant dofs. The
 *         collision cost is returned.
 */
double RcsBody_collisionGradient(const RcsGraph* self,
                                 const RcsBody* b1,
                                 const RcsBody* b2,
                                 double dThreshold,
                                 MatNd* dH,
                                 double* status);

/*! \ingroup RcsBodyFunctions
 *  \brief Returns the collision Hessian of two bodies.
 */
void RcsBody_collisionHessian(const RcsGraph* self, const RcsBody* b1,
                              const RcsBody* b2, double dThreshold,
                              double* H);

/*! \ingroup RcsBodyFunctions
 *  \brief Returns the last "driving" joint before the body. If body is
 *         NULL, the function returns NULL. If there is no driving joint,
 *         the function returns NULL.
 */
RcsJoint* RcsBody_lastJointBeforeBody(const RcsBody* body);

/*! \ingroup RcsBodyFunctions
 *  \brief Creates and initializes the 6 joints associated to a rigid body
 *         joint. The joint ordering is
 *         - Translation into x-direction
 *         - Translation into y-direction
 *         - Translation into xz-direction
 *         - Rotation around x-axis
 *         - Rotation around y-axis
 *         - Rotation around z-axis
 *
 *  \return Pointer to the first of the six rigid body joints.
 */
RcsJoint* RcsBody_createRBJ(RcsGraph* self, RcsBody* b, const double q_rbj[6]);

/*! \ingroup RcsBodyFunctions
 *  \brief Prints the bodie's xml representation to the given file
 *         descriptor. The arguments are assumed to be not NULL. Otherwise,
 *         the function exits with a fatal error. The graph is needed to
 *         find the sensors.
 */
void RcsBody_fprintXML(FILE* out, const RcsBody* self, const RcsGraph* graph);

/*! \ingroup RcsBodyFunctions
 *  \brief Checks if a body has 6 degrees of freedom. This is independent of the
 *         rigid_body_joints flag. It is also ignored if the joints are
 *         constrained or not. If the body is NULL, the function returns false.
 *
 *  \return true if the body has 6 joints in the order xyzabc, false otherwise.
 */
bool RcsBody_isFloatingBase(const RcsBody* self);

/*! \ingroup RcsBodyFunctions
 *  \brief Merges the body with its parent so that the kinematic and dynamic
 *         properties are consistent. The following assumptions must be met:
 *         - body with the given name exists
 *         - body has no joints
 *
 *  \return true for success, false otherwise.
 */
bool RcsBody_mergeWithParent(RcsGraph* graph, const char* bodyName);

/*! \ingroup RcsBodyFunctions
 *  \brief This function computes the axis-aligned bounding box of a shape.
 *
 *  \param[in] body       Body data. If it is NULL, or contains no shapes, the
 *                        AABB is set to zero, and a debug message is issued
 *                        on debul level 4.
 *  \param[in] xyzMin     Minimum point of the box
 *  \param[in] xyzMax     Maximum point of the box
 */
void RcsBody_computeAABB(const RcsBody* body,
                         double xyzMin[3], double xyzMax[3]);

/*! \ingroup RcsBodyFunctions
 *  \brief This convenience function creates a sphere with the given mass and
 *         radius that can be added to a physics engine. The caller is
 *         responsible to free the returned memory.
 *
 *  \param[in] pos      Initial position of the sphere. This rigid body joint's
 *                      q0 members will be initialized with this.
 *  \param[in] mass     Sphere mass in [kg]
 *  \param[in] radius   Sphere radius in [m]
 *  \return Body with the given properties. The function will always succeed,
 *          even if weird values (e.g. negative mass) is given as input.
 */
RcsBody* RcsBody_createBouncingSphere(const double pos[3],
                                      double mass, double radius);

/*! \ingroup RcsBodyFunctions
 *  \brief This function removes all joints from a body. The relative transforms
 *         of the bodie's shapes will be preserved. The joint positions will be
 *         considered as given in the graph's q-vector. All changes will
 *         consistently be reflected in the graph.
 *
 *  \param[in] self   Valid (not NULL) pointer to body.
 *  \param[in] graph  Graph containing the body.
 *  \return true for success, false otherwise:
 *          - self is NULL
 *          - self is not part of graph
 */

bool RcsBody_removeJoints(RcsBody* self, RcsGraph* graph);

/*! \ingroup RcsBodyFunctions
 *  \brief Replaces all body shapes with a minimum enclosing oriented box. On
 *         success, the original shapes will all be deleted. Otherwise, they
 *         remain unchanged.
 *
 *  \param[in,out] self     Body whose shapes are to be replaced by the OBB.
 *  \param[in] computeType  See enum RCSSHAPE_COMPUTE_TYPE
 *  \return True for success, false otherwise.
 */
bool RcsBody_boxify(RcsBody* self, int computeType);


#ifdef __cplusplus
}
#endif

#endif   // RCS_BODY_H
