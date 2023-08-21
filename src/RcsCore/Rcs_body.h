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
 *  \brief Creates and initializes a new RcsBody structure. All connection
 *         indices are set to -1, the rest of the structure is set to 0.
 */
RcsBody* RcsBody_create();

/*! \ingroup RcsBodyFunctions
 *  \brief Initializes the RcsBody structure with default values.
 */
void RcsBody_init(RcsBody* self);

/*! \ingroup RcsBodyFunctions
 *  \brief Deletes the body and frees all memory. The function does nothing
 *         if self is NULL.
 */
void RcsBody_destroy(RcsBody* self);

/*! \ingroup RcsBodyFunctions
 *  \brief Clears and deletes all internal RcsBody members (joints, shapes),
 *         but does not delete the body itself.
 */
void RcsBody_clear(RcsBody* self);

/*! \ingroup RcsBodyFunctions
 *  \brief Creates a shallow copy of the body (Shapes etc. are not copied).
 *         These members are left unchanged and need to be handled separately:
 *         - id
 *         - parentId
 *         - firstChildId
 *         - lastChildId
 *         - nextId
 *         - prevId
 *         - jntId
 *         - nShapes
 *         - shapes
 */
void RcsBody_copy(RcsBody* dst, const RcsBody* src);

/*! \ingroup RcsBodyFunctions
 *  \brief Returns the number of joints by which the body is attached to its
 *         predecessor. If body is NULL, the function returns 0.
 */
unsigned int RcsBody_numJoints(const RcsGraph* graph, const RcsBody* body);

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
 *  \brief Returns the number of shapes attached to the body that are of the
 *         given shape type.
 */
unsigned int RcsBody_numShapesOfType(const RcsBody* self, int type);

/*! \ingroup RcsBodyFunctions
 *  \brief Appends a shape to the bodie's shapes. The shapes array will be
 *         resized if needed (using realloc). The bodie's shape count will be
 *         incremented. The new shape is initialized with the defaults
 *         (see \ref RcsShape_init).
 */
RcsShape* RcsBody_appendShape(RcsBody* self);

/*! \ingroup RcsBodyFunctions
 *  \brief Removes the idx-th shape of the body, and packs the other shapes
 *         so that there is no NULL gap in the array. There will be no memory
 *         re-allocation.
 *
 *  \return True for success, false otherwies (e.g. there is no idx shapes in
 *          the array)
 */
bool RcsBody_removeShape(RcsBody* self, unsigned int idx);

/*! \ingroup RcsBodyFunctions
 *  \brief Attaches body to a target body. It is not supported to attach a body
 *         to a target body that is a generic body. In this case, the function
 *         returns false. If the body is a generic body, the function
 *         attaches the body it is pointing to.
 *
 *  \param[in,out] graph       RcsGraph whose structure is to be changed
 *  \param[in]     bodyId      Id of RcsBody to be attached to the target
 *  \param[in]     targetId    Id of RcsBody which will have the body as child.
 *                             If it isNULL, body will be attached to the root.
 *  \return True for success, false if body is NULL or the target is a
 *          GenericBody.
 */
bool RcsBody_attachToBodyId(RcsGraph* graph, int bodyId, int targetId);

/*! \ingroup RcsBodyFunctions
 *  \brief Returns true if child is a child of parent, false otherwise.
 */
bool RcsBody_isChild(const RcsGraph* graph,
                     const RcsBody* possibleChild,
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
bool RcsBody_isArticulated(const RcsGraph* graph, const RcsBody* self);

/*! \ingroup RcsBodyFunctions
 *  \brief Returns true if the body is part of the graph, false otherwise.
 */
bool RcsBody_isInGraph(const RcsBody* self, const RcsGraph* graph);

/*! \ingroup RcsBodyFunctions
 *  \brief Returns true if the bodies are indentical, false otherwise. The
 *         data of the bodies as well as all of their shapes are checked.
 */
bool RcsBody_isEqual(const RcsBody* b1, const RcsBody* b2);

/*! \ingroup RcsBodyFunctions
 *  \brief Prints out the body to a file descriptor.
 */
void RcsBody_fprint(FILE* fd, const RcsBody* b, const RcsGraph* graph);

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
 *         In this case, only the distance is returned. The proximity pair
 *         computations that are are supported can be seen with the function
 *         \ref RcsShape_fprintDistanceFunctions(FILE*).
 */
double RcsBody_distance(const RcsBody* b1, const RcsBody* b2,
                        double cp1[3], double cp2[3], double n12[3]);

/*! \ingroup RcsBodyFunctions
 *  \brief Computes the distance between a body and a 3d point.
 *
 *  \param[in]  body     RcsBody to which distance is to be computed
 *  \param[in]  I_pt     Point in world coordinates
 *  \param[out] I_cpBdy  Closest point on the body in world coordinates. If it
 *                       is NULL, it will be ignored.
 *  \param[out] I_nBdyPt Unit normal vector from body towards point. If it is
 *                       NULL, it will be ignored.
 */
double RcsBody_distanceToPoint(const RcsBody* body, const double I_pt[3],
                               double I_cpBdy[3], double I_nBdyPt[3]);

/*! \ingroup RcsBodyFunctions
 *  \brief Computes the distance to the closest body in a given direction. This
 *         is equivalent to a ray cast. Only shapes with RcsShape::computeType
 *         RCSSHAPE_DISTANCE are considered. If no intersection was found, all
 *         output arguments remain unchanged.
 *
 *  \param[in]  graph       RcsGraph containing the body
 *  \param[in]  pt          Point in world coordinates from which the distance
 *                          is to be computed
 *  \param[in]  direction   Direction vector in world coordinates. It does not
 *                          need to be of unit length. If it is degenerate, the
 *                          function will return no intersection and warn on
 *                          debug level 4.
 *  \param[out] I_cpBdy     Closest point on the body in world coordinates. If
 *                          it is NULL, it will be ignored.
 *  \param[out] distance    Closest distance along line direction. If it is
 *                          NULL, it will be ignored.
 *  \return Closest body, or NULL if no intersection is found.
 */
const RcsBody* RcsBody_closestInDirection(const RcsGraph* graph,
                                          const double pt[3],
                                          const double direction[3],
                                          double I_cpBdy[3],
                                          double* distance);

/*! \ingroup RcsBodyFunctions
 *  \brief Same as RcsBody_closestInDirection, but considres only bodies that
 *         have rigid body degrees of freedom.
 */
const RcsBody* RcsBody_closestRigidBodyInDirection(const RcsGraph* graph,
                                                   const double pt[3],
                                                   const double direction[3],
                                                   double I_cpBdy[3],
                                                   double* distance);

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
 *         is copied into dDdq. It is reshaped to dimensions 1 x RcsGraph::nJ.
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
RcsJoint* RcsBody_lastJointBeforeBody(const RcsGraph* graph,
                                      const RcsBody* body);

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
 *  \param[in] self    Graph the body is represented in.
 *  \param[in] b       Body that is connected to the rigid body dofs.
 *  \param[in] q_rbj   Values that the dof are initialized with. If this
 *                     argument is NULL, the values are initialized with 0.
 *
 *  \return Pointer to the first of the six rigid body joints.
 */
RcsJoint* RcsBody_createRBJ(RcsGraph* self, RcsBody* b, const double q_rbj[6]);

/*! \ingroup RcsBodyFunctions
 *  \brief Creates and initializes the 6 joints associated to a rigid body
 *         joint. The joint ordering is given in the array indexOrdering.
 *
 *  \param[in] self    Graph the body is represented in.
 *  \param[in] body    Body that is connected to the rigid body dofs.
 *  \param[in] q_rbj   Values that the dof are initialized with. If this
 *                     argument is NULL, the values are initialized with 0.
 *  \param[in] indexOrdering Index order, e.g. for y-x-z-thz-thy-thx it is
 *                           1-0-2-5-4-3.
 *
 *  \return Pointer to the first of the six rigid body joints.
 */
RcsJoint* RcsBody_createOrdered6DofJoints(RcsGraph* self, RcsBody* body,
                                          const double q_rbj[6],
                                          const int indexOrdering[6]);

/*! \ingroup RcsBodyFunctions
 *  \brief Prints the bodie's xml representation to the given file descriptor.
 *         The arguments are assumed to be not NULL.
 *
 *  \param[in] out     File to write to.
 *  \param[in] self    Body to write to the xml file.
 *  \param[in] graph   Graph the body is represented in.
 *
 *  \return Number of errors encountered. Errors are reported on debug level 1.
 */
int RcsBody_fprintXML(FILE* out, const RcsBody* self, const RcsGraph* graph);

/*! \ingroup RcsBodyFunctions
 *  \brief Checks if a body has rigid body degrees of freedom. The function
 *         does not check the rigid_body_joints flag, but goes through the
 *         first six joints of a body and checks if the body has 6 joints
 *         in the order xyzabc. It is ignored if the joints are constrained
 *         or not. If the body is NULL, the function returns false.
 *
 *  \return true if the bodie's first 6 joints are in the order xyzabc,
 *          false otherwise.
 */
bool RcsBody_isFloatingBase(const RcsGraph* graph, const RcsBody* self);

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
 *  \brief This function computes the axis-aligned bounding box of a body.
 *         It is computed over all contained shapes with the given computeType.
 *         Shapes of type RCSSHAPE_REFFRAME are ignored.
 *
 *  \param[in] body        Body data. If it is NULL, or contains no shapes, the
 *                         AABB is set to zero, and a debug message is issued
 *                         on debul level 4.
 *  \param[in] computeType See enum RCSSHAPE_COMPUTE_TYPE. If set to -1, all
 *                         shapes are considered.
 *  \param[in] xyzMin      Minimum point of the box
 *  \param[in] xyzMax      Maximum point of the box
 *  \return True for success, false otherwise. In case of no success, the
 *          bounding box is set to the bodie's origin, or to 0 if body is NULL.
 */
bool RcsBody_computeAABB(const RcsBody* body, int computeType,
                         double xyzMin[3], double xyzMax[3]);

/*! \ingroup RcsBodyFunctions
 *  \brief This convenience function creates a sphere with the given mass and
 *         radius that can be added to a physics engine. The sphere is created
 *         in the graph's body array, six rigid body dofs are created in the
 *         graph's joint array. The sphere is created as the last body on the
 *         root level of the graph. It means that the body and joint indices
 *         of all other bodies and joints remain unchanged. Please note that
 *         after calling this function, the structure of the graph has changed,
 *         and potential q-space arrays also need updating. See the function
 *         \ref RcsGraph_addBodyDofs() for details.
 *
 *  \param[in] graph    Graph in which the sphere is to be created
 *  \param[in] pos      Initial position of the sphere. This rigid body joint's
 *                      q0 members will be initialized with this.
 *  \param[in] mass     Sphere mass in [kg]
 *  \param[in] radius   Sphere radius in [m]
 *  \return Body with the given properties. The function will always succeed,
 *          even if weird values (e.g. negative mass) is given as input.
 */
RcsBody* RcsBody_createBouncingSphere(RcsGraph* graph, const double pos[3],
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
 *         remain unchanged. The funcion uses a method from the GeometricTools
 *         library and will therefore return failure if this is not available.
 *
 *  \param[in,out] self     Body whose shapes are to be replaced by the OBB.
 *  \param[in] computeType  See enum RCSSHAPE_COMPUTE_TYPE
 *  \param[in] replaceShapes If true, all shapes except for the frames will be
 *                           replaced by the enclosing box. If false, the box
 *                           will be added as an additional shape to the body.
 *  \return True for success, false otherwise.
 */
bool RcsBody_boxify(RcsBody* self, int computeType, bool replaceShapes);

/*! \ingroup RcsBodyFunctions
 *  \brief Replaces all body shapes with a minimum enclosing capsule. On
 *         success, the original shapes will all be deleted. Otherwise, they
 *         remain unchanged.
 *
 *  \param[in,out] self      Body whose shapes are to be replaced by the capule
 *  \param[in] computeType   See enum RCSSHAPE_COMPUTE_TYPE
 *  \param[in] replaceShapes If true, all shapes except for the frames will be
 *                           replaced by the enclosing capsule. If false, the
 *                           capsule will be added as an additional shape to
 *                           the body.
 *  \return True for success, false otherwise.
 */
bool RcsBody_capsulify(RcsBody* self, int computeType, bool replaceShapes);

/*! \ingroup RcsGraphFunctions
 *  \brief Creates a mesh of the body by traversing through all shapes.
 */
RcsMeshData* RcsBody_meshify(const RcsBody* self, char computeType);

/*! \ingroup RcsBodyFunctions
 *  \brief Scales the geometry of the body, and of all attached joints and
 *         shapes. Translational joints require a scaling of the q-vector.
 *         Since it is not accessible from the RcsBody, this is done separately
 *         in RcsGraph_scale(). The mass and inertia properties are not scaled.
 *
 *  \param[in] graph        Graph that contains the body
 *  \param[in,out] self     Body to be reshaped.
 *  \param[in] scale        Scaling factor.
 */
void RcsBody_scale(RcsGraph* graph, RcsBody* self, double scale);



/**
 * @name Accessors
 *
 * Functions to access / modify the body
 */

///@{

/*! \ingroup RcsBodyFunctions
 *  \brief Returns the last body in the graph according to a depth-first
 *         traversal. If the graph is empty, NULL is returned.
 */
RcsBody* RcsBody_getLastInGraph(const RcsGraph* self);

/*! \ingroup RcsBodyFunctions
 *  \brief Determines the number of distance calculations carried out between
 *         the bodies when using the function \ref RcsBody_distance. The result
 *         depends on the shapes that are considered for distance calculation
 *         (see enum RCSSHAPE_COMPUTE_TYPE), and the libraries that provide
 *         their distance function implementation.
 *
 *  \param[in] b1     First body
 *  \param[in] b2     Second body
 *  \return Number of distance function calls between the bodies. If either
 *          b1 or b2 is NULL, the function returns 0.
 */
unsigned int RcsBody_getNumDistanceQueries(const RcsBody* b1,
                                           const RcsBody* b2);

/*! \ingroup RcsBodyFunctions
 *  \brief Returns a pointer to the body referred to by RcsBody::prevId.
 */
RcsBody* RcsBody_getPrev(RcsGraph* graph, RcsBody* body);

/*! \ingroup RcsBodyFunctions
 *  \brief Returns a pointer to the body referred to by RcsBody::nextId.
 */
RcsBody* RcsBody_getNext(RcsGraph* graph, RcsBody* body);

/*! \ingroup RcsBodyFunctions
 *  \brief Returns a pointer to the body referred to by RcsBody::parentId.
 */
RcsBody* RcsBody_getParent(RcsGraph* graph, RcsBody* body);

/*! \ingroup RcsBodyFunctions
 *  \brief Returns a pointer to the body referred to by RcsBody::firstChildId.
 */
RcsBody* RcsBody_getFirstChild(RcsGraph* graph, RcsBody* body);

/*! \ingroup RcsBodyFunctions
 *  \brief Returns a pointer to the body referred to by RcsBody::lastChildId.
 */
RcsBody* RcsBody_getLastChild(RcsGraph* graph, RcsBody* body);

/*! \ingroup RcsBodyFunctions
 *  \brief Returns a pointer to the graph's q-vector at the index of the bodie's
 *         joint, or NULL if no joint exists. The dimension of the returned
 *         pointer can be queried with RcsBody_numJoints() if necessary.
 *
 *  \param[in] graph  Pointer to valid graph. Must not be NULL.
 *  \param[in] body   Pointer to body for which result is to be returned. If it
 *                    is NULL, the function returns NULL.
 *  \return Pointer to graph's q-vector at the joint's index, or NULL.
 */
double* RcsBody_getStatePtr(RcsGraph* graph, const RcsBody* body);

///@}


#ifdef __cplusplus
}
#endif

#endif   // RCS_BODY_H
