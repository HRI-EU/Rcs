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

#ifndef RCS_GRAPH_H
#define RCS_GRAPH_H

typedef struct _RcsGraph          RcsGraph;
typedef struct _RcsBody           RcsBody;
typedef struct _RcsJoint          RcsJoint;
typedef struct _RcsShape          RcsShape;
typedef struct _RcsPair           RcsPair;
typedef struct _RcsCollisionMdl   RcsCollisionMdl;
typedef struct _RcsSensor         RcsSensor;
typedef struct _RcsTexel          RcsTexel;


typedef enum
{
  RcsStateFull = 0,   ///< All dofs
  RcsStateIK   = 1    ///< Only IK-relevant dofs

} RcsStateType;

#include <Rcs_MatNd.h>
#include <Rcs_HTr.h>
#include <Rcs_mesh.h>


#ifdef __cplusplus
extern "C" {
#endif



/*!
 *  \page RcsGraph Kinematics and dynamics
 *
 *  <h1> A graph library for kinematics and dynamics </h1>
 *
 *  \ref XMLParsing
 *
 *  \ref RcsGraphFunctions
 *
 *  \ref RcsKinematicsFunctions
 *
 *  \ref RcsKineticsFunctions
 *
 *  \ref RcsGraphTraversalFunctions
 *
 *  \ref RcsBodyFunctions
 *
 *  \ref RcsJointFunctions
 *
 *  \ref RcsShapeFunctions
 *
 *  \ref RcsParserFunctions
 *
 *  \ref RcsCollisionMdlFunctions
 *
 *  \ref RcsSensorFunctions
 *
 */








/*!
 * \defgroup RcsGraphTraversalFunctions Graph traversal
 *
 */

/*! \ingroup RcsSensorFunctions
 *  \brief Returns pointer to the first sensor in the graph's sensor array. It
 *         is the sensor with id=0. If no sensor have been added, the function
 *         returns NULL. Implementation in Rcs_sensor.c
 */
RcsSensor* RcsSensor_first(const RcsGraph* graph);

/*! \ingroup RcsSensorFunctions
 *  \brief Returns pointer to the last sensor in the graph's sensor array. It
 *         is the sensor with id=RcsGraph::nSensors-1. If no sensor have been
 *         added, the function returns NULL. Implementation in Rcs_sensor.c
 */
RcsSensor* RcsSensor_last(const RcsGraph* graph);

/*! \ingroup RcsBodyFunctions
 *  \brief Returns a pointer to the first body in the body's memory array. It
 *         is the body with id=0. If no bodies have been added, the function
 *         returns NULL. Note that the body with id=0 is not necessarily the
 *         root body. Implementation in Rcs_body.c
 */
RcsBody* RcsBody_first(const RcsGraph* graph);

/*! \ingroup RcsBodyFunctions
 *  \brief Returns a pointer to the last body in the body's memory array. It is
 *         the body with id=nBodies-1. This function is needed for connection
 *         related computations. Implementation in Rcs_body.c
 */
RcsBody* RcsBody_last(const RcsGraph* graph);

/*! \ingroup RcsBodyFunctions
 *  \brief Returns a pointer to the joint with the id RcsBody::jntId. If it is
 *         -1, the function returns NULL. Implementation in Rcs_body.c.
 */
RcsJoint* RcsBody_getJoint(const RcsBody* b, const RcsGraph* graph);

/*! \ingroup RcsJointFunctions
 *  \brief Returns a pointer to the first joint in the graph's joint array. It
 *         is the joint with id=0. This function is needed for connection
 *         related computations, and not for q-array indexing. Implementation
 *         in Rcs_joint.c.
 */
RcsJoint* RcsJoint_first(const RcsGraph* graph);

/*! \ingroup RcsJointFunctions
 *  \brief Returns a pointer to the last joint in the graph's joint array. It
 *         is the joint with id=dof-1. This function is needed for connection
 *         related computations, and not for q-array indexing. Implementation
 *         in Rcs_joint.c.
 */
RcsJoint* RcsJoint_last(const RcsGraph* graph);

/*! \ingroup RcsJointFunctions
 *  \brief Returns a pointer to the joint next in the kinematic chain. This
 *         forward chain ends at each RcsBody.
 */
RcsJoint* RcsJoint_next(const RcsJoint* joint, const RcsGraph* graph);

/*! \ingroup RcsJointFunctions
 *  \brief Returns a pointer to the parent joint in the kinematic chain. This
 *         backward chain will end in the root of the graph.
 */
RcsJoint* RcsJoint_prev(const RcsJoint* joint, const RcsGraph* graph);

/*! \ingroup RcsJointFunctions
 *  \brief Returns a pointer to the master joint of the given joint. The master
 *         joint only exists if the joint has been kinematically coupled to it.
 *         Otherwise, the function returns NULL.
 */
RcsJoint* RcsJoint_master(const RcsJoint* joint, const RcsGraph* graph);

/*! \ingroup RcsGraphTraversalFunctions
*  \brief Depth-first traversal through a graph, starting from body. This
*         function steps through the complete graph, even through the
*         parents of body.
*/
RcsBody* RcsBody_depthFirstTraversalGetNextById(const RcsGraph* graph,
                                                const RcsBody* body);

/*! \ingroup RcsGraphTraversalFunctions
 *  \brief Returns the leaf-node that comes last in a depth-first traversal
 *         starting from the body b. If b is the final leaf, b will be
 *         returned. If b is NULL, the function retuens NULL.
 */
RcsBody* RcsBody_getLastLeaf(const RcsGraph* graph, RcsBody* b);

/*! \ingroup RcsGraphFunctions
 *  \brief Returns the root body of the graph. It is the first one in the
 *         depth-first traversal.
 */
RcsBody* RcsGraph_getRootBody(const RcsGraph* self);
/*! \ingroup RcsGraphTraversalFunctions
 *  \brief Depth-first traversal through all bodies of the graph, starting
 *         from the root body. The bodies can be accessed with variable
 *         "BODY".
 */
#define RCSGRAPH_TRAVERSE_BODIES(graph)                                    \
  for (RcsBody* BODY = RcsGraph_getRootBody(graph); BODY;                  \
       BODY = RcsBody_depthFirstTraversalGetNextById(graph, BODY))

/*! \ingroup RcsGraphTraversalFunctions
 *  \brief Loops through the given body and its children, depth-first. The
 *         "next" bodies of body are not considered.
 */
#define RCSBODY_TRAVERSE_BODIES(graph, body)                               \
  for (RcsBody* BODY = (body),                                             \
       *LAST_LEAF = RcsBody_getLastLeaf((graph), BODY),                    \
       *LAST = RcsBody_depthFirstTraversalGetNextById((graph), LAST_LEAF); \
       BODY && BODY != LAST;                                               \
       BODY = RcsBody_depthFirstTraversalGetNextById((graph),BODY))

/*! \ingroup RcsGraphTraversalFunctions
 *  \brief This macro is almost identical to RCSBODY_TRAVERSE_BODIES(),
 *         except that the body argument itself will not be traversed, but
 *         only its children.
 */
#define RCSBODY_TRAVERSE_CHILD_BODIES(graph, body)                             \
  for (RcsBody* BODY = RcsBody_depthFirstTraversalGetNextById((graph), (body)),\
       *LAST_LEAF = RcsBody_getLastLeaf((graph), BODY),                        \
       *LAST = RcsBody_depthFirstTraversalGetNextById((graph), LAST_LEAF);     \
       BODY && BODY != LAST;                                                   \
       BODY = RcsBody_depthFirstTraversalGetNextById((graph), BODY))

/*! \ingroup RcsGraphTraversalFunctions
 *  \brief Traverses the joint and all "next" joints of it.
 *         The joints can be accessed with variable "JNT".
 */
#define RCSJOINT_TRAVERSE_FORWARD(graph, joint) \
  for (RcsJoint* JNT = (joint); JNT; JNT = RcsJoint_next(JNT, (graph)))

/*! \ingroup RcsGraphTraversalFunctions
 *  \brief Traverses all joints of a body, starting from the first one.
 *         The joints can be accessed with variable "JNT".
 */
#define RCSBODY_FOREACH_JOINT(graph, body) \
  RCSJOINT_TRAVERSE_FORWARD((graph), RcsBody_getJoint((body),(graph)))

/*! \ingroup RcsGraphTraversalFunctions
 *  \brief Traverses all joints of the graph, starting from the root body.
 *         The joints can be accessed with variable "JNT".
 */
#define RCSGRAPH_TRAVERSE_JOINTS(graph)\
  RCSGRAPH_TRAVERSE_BODIES((graph))    \
  RCSBODY_FOREACH_JOINT((graph),(BODY))

/*! \ingroup RcsGraphTraversalFunctions
*  \brief Traverses all shapes of a body, starting from the first one.
*         The joints can be accessed with variable "SHAPE".
*/
#define RCSBODY_TRAVERSE_SHAPES(body)                                        \
  for (RcsShape *S0 = (body)->nShapes > 0 ? &(body)->shapes[0] : NULL,       \
       *S1 = (S0!=NULL) ? &(body)->shapes[body->nShapes-1] : NULL,           \
       *SHAPE = S0; SHAPE && SHAPE <= S1; SHAPE++)

#define RCSGRAPH_FOREACH_SENSOR(graph)                                       \
  for (RcsSensor *S0 = RcsSensor_first(graph), *SENSOR = S0,                 \
       *S1 = RcsSensor_last(graph);                                          \
       SENSOR && SENSOR<=S1; SENSOR++)

#define RCSGRAPH_FOREACH_BODY(graph)                                         \
  for (RcsBody *B0 = RcsBody_first(graph), *B1 = RcsBody_last(graph),        \
       *BODY = B0; BODY && BODY<=B1; BODY++)

#define RCSGRAPH_FOREACH_JOINT(graph)                                        \
  for (RcsJoint *J0 = RcsJoint_first(graph), *J1 = RcsJoint_last(graph),     \
       *JNT = J0; JNT && JNT<=J1; JNT++)




/*!
 * \defgroup RcsGraphFunctions Basic graph functions
 *
 */


/*! \ingroup RcsGraphFunctions
 *  \brief Creates an instance of a graph. Here is what this function can parse:
 *         - From an xml-file with Rcs conventions
 *         - From a bvh-file (suffix .bvh)
 *         - From an URDF-file (filename starts with \<robot\> keyword)
 *         - From a string (filename starts with \<Graph\> keyword)
 *         There is also some rudimentary OpenRave parsing that needs more work
 *         for a seamless usage.
 */
RcsGraph* RcsGraph_create(const char* filename);

/*! \ingroup RcsGraphFunctions
 *  \brief Creates a random graph with the given amount of bodies and the
 *         given branching factor. The branching factor determines after how
 *         many added bodies a branch will be created.
 */
RcsGraph* RcsGraph_createRandom(unsigned int nBodies,
                                unsigned int branchingFactor);

/*! \ingroup RcsGraphFunctions
 *  \brief Destroys an instance of a graph. The function does nothing if
 *         self is NULL.
 */
void RcsGraph_destroy(RcsGraph* self);

/*! \ingroup RcsGraphFunctions
 *  \brief Scales all elements of the graph with the given scale factor.
 */
void RcsGraph_scale(RcsGraph* self, double scaleFactor);

/*! \ingroup RcsGraphFunctions
 *  \brief Blows the vector of Jacobian coordinates up to the full state
 *         vector.
 */
void RcsGraph_stateVectorFromIK(const RcsGraph* self, const MatNd* q_ik,
                                MatNd* q_full);

/*! \ingroup RcsGraphFunctions
 *  \brief In-place function of RcsGraph_stateVectorFromIK(). If the memory
 *         of q is not large enough, new memory will be allocated. The
 *         constrained elements of q will be set to 0.
 */
void RcsGraph_stateVectorFromIKSelf(const RcsGraph* self, MatNd* q);

/*! \ingroup RcsGraphFunctions
 *  \brief Compresses the full state vector to the vector of Jacobian
 *         relevant coordinates. Vector q_ik will be reshaped by the function.
 *
 *  \param[in]  self Pointer to a valid RcsGraph
 *  \param[in]  q_full Array of dimension dof x 1
 *  \param[out] q_ik Array with memory for at least nJ elements
 */
void RcsGraph_stateVectorToIK(const RcsGraph* self, const MatNd* q_full,
                              MatNd* q_ik);

/*! \ingroup RcsGraphFunctions
 *  \brief In-place function of \ref RcsGraph_stateVectorToIK.
 */
void RcsGraph_stateVectorToIKSelf(const RcsGraph* self, MatNd* q);

/*! \ingroup RcsGraphFunctions
 *  \brief Sets the state vector to q, computes the forward kinematics
 *         and recomputes the index vector for the accelerated Jacobian
 *         computation. If the graph contains any coupled joints, their
 *         kinematics will also be updated. If qp is NULL, the velocities are
 *         not updated. The function returns true if the internal q vector
 *         was modified due to kinematic joint couplings, and is not the same
 *         as the q vector given as the argument. If the sizes of the input
 *         arrays are neither RcsGraph::dof x 1 nor RcsGraph::nJ x 1, the
 *         function exits with a fatal error.
 *
 *  \param[in]  self Pointer to a valid RcsGraph
 *  \param[in]  q Joint angles array of dimension RcsGraph::dof x 1 or
 *              RcsGraph::nJ x 1 (the function takes care of the conversion).
 *  \param[in]  qp Joint velocity array of dimension RcsGraph::dof x 1 or
 *              RcsGraph::nJ x 1 (the function takes care of the conversion).
 *              If it is NULL, the velocity array RcsGraph::qp remains
 *              unchanged.
 */
bool RcsGraph_setState(RcsGraph* self, const MatNd* q, const MatNd* qp);

/*! \ingroup RcsGraphFunctions
 *  \brief Computes the forward kinematics of the graph. If vectors q and qp
 *         are not NULL, they are assumed to be of full state vector dimension
 *         (self->dof), and to contain the values for the degrees of freedom
 *         and their velocities. The function will also update the Jacobian
 *         indices considering the constraints, and recompute self->nJ. The
 *         kinematically coupled joints will not be made consistent.
 */
void RcsGraph_computeForwardKinematics(RcsGraph* self, const MatNd* q,
                                       const MatNd* q_dot);

/*! \ingroup RcsGraphFunctions
 *  \brief Computes the forward kinematics starting from a given body of the
 *         graph. The body will be initialized with the state of its parent
 *         or with identity if it has no parent. If the argument subTree is
 *         false, just the body and its joints will undergo a forward
 *         kinematics calculation. If subTree is true, the forward kinematics
 *         will be computed starting from the body in a depth-first way. The
 *         traversal will not go through the bodie's neighbors. The state
 *         vectors q and q_dot may be NULL, or must be of dimension
 *         RcsGraph::dof x 1. In the first case, the graph's internal q- and
 *         q_dot vectors are used. In the latter case, the passed arrays are
 *         used. They will not be copied into the graph's vectors. This is
 *         similar to RcsGraph_computeForwardKinematics, and different to
 *         RcsGraph_setState.
 *
 *  \param[in] self    The graph containing the body
 *  \param[in] bdy     Start body for traversal
 *  \param[in] q       Joint vector of dimension RcsGraph::dof x 1, or NULL
 *  \param[in] q_dot   Joint velocity vector of dimension RcsGraph::dof x 1,
 *                     or NULL
 *  \param[in] subTree Traverse subtree if true, compute body only if false
 */
void RcsGraph_computeBodyKinematics(RcsGraph* self, RcsBody* bdy,
                                    const MatNd* q, const MatNd* q_dot,
                                    bool subTree);

/*! \ingroup RcsGraphFunctions
 *  \brief Sets the state default state vector, computes the forward
 *         kinematics and recomputes the index vector for the accelerated
 *         Jacobian computation. The default state vector corresponds to
 *         the center joint angles q_init of each joint in the XML file.
 *         If it is not specified, the center is assumed to be 0. The
 *         velocities are set to zero.
 */
void RcsGraph_setDefaultState(RcsGraph* self);

/*! \ingroup RcsGraphFunctions
 * \brief This function overwrites the joint centers with the given values.
 *
 *  \param[in]  self   The graph containing the joint centers to be overwritten
 *  \param[in]  q0     Vector of new joint centers to be written to the joints
 *                     member q0. The vector must be of dimension
 *                     RcsGraph::dof x 1 or nJ x 1, otherwise the function
 *                     will exit fatally. If it is nJ, the constrained dof
 *                     remain unchanged.
 */
void RcsGraph_changeDefaultState(RcsGraph* self, const MatNd* q0);

/*! \ingroup RcsGraphFunctions
 *  \brief Returns the default state vector of the graph. It corresponds to
 *         an array holding the center angles of all joints.
 */
void RcsGraph_getDefaultState(const RcsGraph* self, MatNd* q0);

/*! \ingroup RcsGraphFunctions
 *  \brief Returns the initial state vector of the graph. It corresponds to
 *         an array holding the center angles of all joints right after
 *         construction, even before parsing any model state.
 */
void RcsGraph_getInitState(const RcsGraph* self, MatNd* q_init);

/*! \ingroup RcsGraphFunctions
 * \brief This function overwrites the initial joint values with q_init.
 *
 *  \param[in]  self   The graph containing the joint centers to be overwritten
 *  \param[in]  q_init Vector of new initial joint values to be written to the
 *                     joints member q_init. The vector must be of dimension
 *                     RcsGraph::dof x 1 or nJ x 1, otherwise the function
 *                     will exit fatally. If it is nJ, the constrained dof
 *                     remain unchanged.
 */
void RcsGraph_changeInitState(RcsGraph* self, const MatNd* q_init);

/*! \ingroup RcsGraphFunctions
 *  \brief Returns a n x 1 array with elements that correspond to the range
 *         of the respective joint. The elements are aligned in the same
 *         order as in the state vector. If type is RcsStateFull, all
 *         joints are considered (dimension is self->dof), for RcsStateIK
 *         only the relevant ones for the inverse kinematics are considered
 *         (dimension is self->nJ). The function reshapes the array W_inv to
 *         the proper size. The entries of the vector are computed as
 *         <br>
 *         invWq_i = jnt_i->weightMetric * (jnt_i->q_max - jnt_i->q_min)
 */
void RcsGraph_getInvWq(const RcsGraph* self, MatNd* invWq,
                       RcsStateType type);

/*! \ingroup RcsGraphFunctions
 *  \brief Returns a pointer to the body with the indicated name. If
 *         several bodies have the same name, the closest one to the root
 *         node (depth first traversal) will be taken. If no matching
 *         body is found, NULL will be returned. If name or self are NULL, also
 *         NULL is returned.
 *
 *  \param[in] self   Pointer to graph. If it is NULL, the function returns
 *                    NULL.
 *  \param[in] name   Character array holding the the joint name. If it is NULL,
 *                    the function returns NULL.
 *  \return Pointer to the body with the name, or NULL if no match has been found.
 */
RcsBody* RcsGraph_getBodyByName(const RcsGraph* self, const char* name);

/*! \ingroup RcsGraphFunctions
 *  \brief Similar to RcsGraph_RcsGraph_getBodyByName() except that only the
 *         number of characters of argument name are compared. This allows to
 *         find bodies with names that have a suffix.
 *
 *  \param[in] self   Pointer to graph. If it is NULL, the function returns
 *                    NULL.
 *  \param[in] name   Character array holding the first characters of the joint
 *                    name. If it is NULL, the function returns NULL.
 *  \return Pointer to the first (in the depth-first traversal sense) body found
 *          with the truncated name, or NULL if no match has been found.
 */
RcsBody* RcsGraph_getBodyByTruncatedName(const RcsGraph* self,
                                         const char* name);

/*! \ingroup RcsGraphFunctions
 *  \brief Returns the summed mass of all bodies of the graph.
 */
double RcsGraph_mass(const RcsGraph* self);

/*! \ingroup RcsGraphFunctions
 *  \brief Returns the summed mass of all bodies of the graph. If root is
 *         not NULL it refers to the body from which the mass is being
 *         computed in a depth-first traversal. In this case, only the
 *         body and its sub-tree are considered, but not its neighbors
 *         (refered to with the nextId).
 */
double RcsGraph_massFromBody(const RcsGraph* self, const char* root);

/*! \ingroup RcsGraphFunctions
 *  \brief Returns the number of bodies in the graph. The function performs a
 *         depth-first traversal and counts all traversed bodies. Therefore it
 *         computes the "true" number of bodies, which might differ from the
 *         RcsGraph::nBodies member once bodies have been deleted from the
 *         graph.
 */
unsigned int RcsGraph_numBodies(const RcsGraph* self);

/*! \ingroup RcsGraphFunctions
 *  \brief Returns the torque for all joints that is caused by the mass
 *         of a specific body. Note that parameter torque is not
 *         overwritten, but the bodyTorque is added to it. So set torque
 *         to zero before calling this method if you only want this
 *         specific torque.
 */
void RcsGraph_bodyTorque(const RcsGraph* self, const RcsBody* body,
                         MatNd* torque);

/*! \ingroup RcsGraphFunctions
 *  \brief Checks the graph for consistency.
 *
 *         Failing the following checks is considered to be an error:
 *         - self is NULL pointer
 *         - Root body not set, has parent or previous body
 *         - bodies with rigid body joints have 6 joints in the order
 *           transX-transY-transZ-rotX-rotY-rotZ (rotations are relative to
 *           previous frame, not static axis representation), and NULL or
 *           identity relative transforms
 *         - duplicate body names
 *         - body names with zero size (empty ones like "")
 *         - body name truncation
 *         - shape mesh file truncation
 *         - joint indices out of range
 *         - Jacobian indices out of range
 *         - joint direction index out of range
 *         - consistency of coupled joints
 *         - correct connectivity of graph structure
 *         - dof and dimension of q-vector match number of joints
 *         - finiteness of q and q_dot
 *         - bodies have a mass >= 0
 *
 *         Failing the following checks is considered to be a warning:
 *         - joint centers out of range
 *         - joint positions out of range
 *         - bodies with finite inertia have a mass > 0
 *         - shape texture file, color or material truncation
 *
 *  \param[in]     self      Pointer to the graph to be checked.
 *  \param[in,out] nErrors   If not NULL, the error count will be copied here.
 *  \param[in,out] nWarnings If not NULL, the warning count will be copied here.
 *  \return True for success, false in case there is one or more warnings
 *          or errors.
 */
bool RcsGraph_check(const RcsGraph* self, int* nErrors, int* nWarnings);

/*! \ingroup RcsGraphFunctions
 *  \brief Creates a deep copy of a graph.
 *
 *  \param[in] src  Pointer to the graph to be copied.
 *  \return Pointer to copy of the graph. If argument src is NULL, or copying
 *                  failed, the function returns NULL. Failure cases are only
 *                  related to memory allocations.
 */
RcsGraph* RcsGraph_clone(const RcsGraph* src);
/*! \ingroup RcsGraphFunctions
 *  \brief Returns a copy of the subgraph starting at the given rootId. The
 *         root node will not have any next neighbor.
 *
 *  \param[in] src  Pointer to the graph to be copied.
 *  \return Pointer to copy of the subgraph. If argument src is NULL, or
 *                  copying failed, the function returns NULL.
 */
RcsGraph* RcsGraph_cloneSubGraph(const RcsGraph* src, const char* rootName);

/*! \ingroup RcsGraphFunctions
 *  \brief Sets the bdyNum-th generic body pointer to the given body.
 *         Index bdyNum must be [0...9]. The body must be a part of
 *         the graph, otherwise it is unlinked. The function returns the
 *         bodie against which the generic body is linked. It is not possible
 *         to link a generic body to itself or another generic body. In this
 *         case, a warning is issued on debug level 1, and NULL is returned.
 */
RcsBody* RcsGraph_linkGenericBody(RcsGraph* self, int bdyNum,
                                  const char* bdyName);

/*! \ingroup RcsGraphFunctions
 *  \brief Returns the pointer to the body that is linked to the generic
 *         body. If no body is linked or value bdyNum is out of range (it is
 *         valid from 0 to 9), NULL is returned.
 */
RcsBody* RcsGraph_getGenericBodyPtr(const RcsGraph* self, int bdyNum);

/*! \ingroup RcsGraphFunctions
 *  \brief Adds memory for a new body to the graph, and connects the new
 *         body to the parent body with the given id. If the parent body
 *         id is -1, the body is inserted as the last sibling on the top
 *         level of the graph. If the graph is empty, the body will straight
 *         be connected to its root. This function is meant for building up
 *         a graph.
 *
 *  \param[in] graph      Pointer to the graph to be extended with the body.
 *  \param[in] parentId   Id of the parent body (may be -1)
 *
 *  \return Pointer to the created body. Please be careful to not store the
 *          pointer address, since in consecutive calls to this function, the
 *          memory might change its location due to realloc calls. To be safe,
 *          always adress a body through its id:
 *          RcsBody* bdy = &graph->bodies[bodyId];
 */
RcsBody* RcsGraph_insertGraphBody(RcsGraph* graph, int parentId);

/*! \ingroup RcsGraphFunctions
 *  \brief Adds a random body to the graph. It will have 1-5 random shapes
 *         attached (See RcsShape_createRandomShape()), and will be connected
 *         to the parent body with either six rigid body joints or with
 *         1-8 randomly created joints.
 *
 *  \param[in] graph      Pointer to the graph to be extended with the body.
 *  \param[in] parentId   Id of the parent body (may be -1)
 *
 *  \return Pointer to the created random body.
 */
RcsBody* RcsGraph_insertRandomBody(RcsGraph* graph, int parentId);

/*! \ingroup RcsGraphFunctions
 *  \brief Adds memory for a new joint to the graph, and connects the new
 *         joint to to last joint of the body with the given id. If the body
 *         with bodyId does not exist, the function exits fatally.
 *         be connected to its root. This function is meant for building up
 *         a graph.
 *
 *  \param[in] graph      Pointer to the graph to be extended with the joint.
 *  \param[in] bodyId     Id of the body the new joint is added to.
 *
 *  \return Pointer to the created joint.
 */
RcsJoint* RcsGraph_insertGraphJoint(RcsGraph* graph, int bodyId);

/*! \ingroup RcsGraphFunctions
 *  \brief Re-order joint indices according to depth-first traversal.
 *         Link pointers of coupled joints and initialize range from master
 *         (if its a complex coupling)
 *
 *  \param[in] self  Pointer to a valid RcsGraph (Must not be NULL)
 */
void RcsGraph_makeJointsConsistent(RcsGraph* self);

/*! \ingroup RcsGraphFunctions
 *  \brief Sets all unconstrained joints of the graph to random values and
 *         computes the forward kinematics
 */
void RcsGraph_setRandomState(RcsGraph* self);

/*! \ingroup RcsGraphFunctions
 *  \brief Appends a deep copy of the RcsGraph other to the body root of the
 *         graph self. The body root must not have any children, and the graph
 *         that will be appended must have an unique root body (without any next
 *         pointer). If the root body is NULL, the RcsGraph other will be
 *         appended on the top level. The merged graph is set to the state of
 *         both original graphs. The sensors of the other graph are appended to
 *         the sensor array of self.
 *
 *  \param[in] self   Graph to be extended. Must not be NULL.
 *  \param[in] root   Body of attachement. It must belong to RcsGraph self. If
 *                    it doesn't, the behavoir of the function is undefined. The
 *                    body must not have any children. If it does, the function
 *                    fails and returns false.
 *  \param[in] other  Graph that will be attached to the body root. A deep copy
 *                    of this graph will be made internally.
 *  \param[in] suffix Character string that will be appended to all bodies,
 *                    joints and sensors of the appended graph. If it is NULL,
 *                    the body names remain unchanged.
 *  \param[in] A_BP   Optional transformation between other graph and root body.
 *  \return           True for success, false for failure. Only these failure
 *                    cases are checked: The body root must not have any
 *                    children, and the other graph must have an unique root
 *                    body (without next body on the same level).
 */
bool RcsGraph_appendCopyOfGraph(RcsGraph* self, RcsBody* root,
                                const RcsGraph* other, const char* suffix,
                                const HTr* A_BP);

/*! \ingroup RcsGraphFunctions
 *  \brief Removes the named body from the graph, and deletes all its memory.
 *         The graph will be restructured so that the removed body will not
 *         lead to a jump of the remaining bodies and their transforms.
 *
 *  \param[in] self    Graph containing body to be removed
 *  \param[in] bdyName Name of body to be removed.
 *  \param[in] qVec    Array of pointers to q-vectors that need to be reshaped
 *                     to match the newly added dof.
 *  \param[in] nVec    Number of vectors in qVec.
 *  \return    True for success, false for failure.
 */
bool RcsGraph_removeBody(RcsGraph* self, const char* bdyName,
                         MatNd* qVec[], unsigned int nVec);

/*! \ingroup RcsGraphFunctions
 *  \brief This function assumes that the RcsBody body has been added to the
 *         graph, and the q-space arrays need updating of sizes and indexing
 *         due to the bodie's joints. For the body joints, the graph's q and
 *         q_dot arrays, as well as the nVec arrays pointed to by qVec, will
 *         be adjusted in length and indexing. This function should be called
 *         after adding a body with joints to a graph. The graph's q vector
 *         elements of the added body will get assigned with their q_init
 *         values.
 *
 *  \param[in] graph  Pointer to the graph that is extended with the body.
 *  \param[in] parent Pointer to the parent body (may be NULL)
 *  \param[in] body   Pointer to the body to be added. After adding, the graph
 *                    takes its ownership.
 *  \param[in] qVec   Array of pointers to q-vectors that need to be reshaped
 *                    to match the newly added dof.
 *  \param[in] nVec   Number of vectors in qVec.
 *  \return True for success, false otherwise (body is NULL). In the failure
 *          case, the graph is not modified.
 */
bool RcsGraph_addBodyDofs(RcsGraph* graph, RcsBody* parent, RcsBody* body,
                          MatNd* qVec[], unsigned int nVec);

/*! \ingroup RcsGraphFunctions
 *  \brief Adds box shapes between parent and child bodies so that an
 *         approximate can be visualized.
 *
 *  \param[in] self    Graph to which random shapes are to be added
 */
void RcsGraph_addRandomGeometry(RcsGraph* self);

/*! \ingroup RcsGraphFunctions
 *  \brief This function computes the axis-aligned bounding box of a shape.
 *
 *  \param[in] self       Graph. If it is NULL, or contains no bodies, the
 *                        AABB is set to zero, and a debug message is issued
 *                        on debul level 4.
 *  \param[in] xyzMin     Minimum point of the box
 *  \param[in] xyzMax     Maximum point of the box
 */
void RcsGraph_computeAABB(const RcsGraph* self,
                          double xyzMin[3], double xyzMax[3]);

/*! \ingroup RcsGraphFunctions
 *  \brief Sets the resizeable property of all shapes of the graph. This has an
 *         effect for the visualization (3d graphics), and for copying the
 *         shapes with \ref RcsGraph_copyResizeableShapes()
 *
 *  \param[in,out] self      Graph whose shapes should be modified.
 *  \param[in] resizeable    True for resizeable, false otherwise.
 */
void RcsGraph_setShapesResizeable(RcsGraph* self, bool resizeable);

/*! \ingroup RcsGraphFunctions
 *  \brief Copies all resizeable shapes from graph src to graph dst. It is
 *         assumed that the graphs have the same structure, otherwise the
 *         result is undefined.
 *
 *  \param[in,out] dst    Graph receiving the updated shapes from src
 *  \param[in]     src    Graph providing the shapes to dst
 *  \param[in]     level  0: extents and computeType, 1: additionally transform,
 *                        2: everything including meshes if any
 */
void RcsGraph_copyResizeableShapes(RcsGraph* dst, const RcsGraph* src,
                                   int level);
void RcsGraph_copy(RcsGraph* dst, const RcsGraph* src);


/**
 * @name Joints
 *
 * Functions to access / modify the graph's joints
 */

///@{

/*! \ingroup RcsGraphFunctions
 *  \brief Limits the joint angles to the range limits. The function
 *         returns false (0) if the joints are within their range,
 *         true if one or more joints had been clipped.
 */
bool RcsGraph_limitJoints(const RcsGraph* self, MatNd* q, RcsStateType type);

/*! \ingroup RcsGraphFunctions
 *  \brief Returns the number of joints whose limits are violated.
 *
 *  \param[in] self   Pointer to graph, must not be NULL.
 *  \param[in] angularMargin  Distance to joint limit that is considered a
 *                            violation, for angular joints.
 *  \param[in] linearMargin   Distance to joint limit that is considered a
 *                            violation, for translational joints.
 *  \param[in] verbose        If true, console information on violations will
 *                            be printed to stderr.
 */
unsigned int RcsGraph_numJointLimitsViolated(const RcsGraph* self,
                                             double angularMargin,
                                             double linearMargin,
                                             bool verbose);

/*! \ingroup RcsGraphFunctions
 *  \brief This function checks all joint and joint speed limits of the
 *         given trajectory q. Array q is assumed to have all
 *         (RcsGraph::dof) or the constrained (RcsGraph::nJ) degrees of
 *         freedom in each row, otherwise false (failure) is returned.
 *         Each row corresponds to the state at one time step.
 *
 *  \param[in] self   Pointer to graph, must not be NULL.
 *  \param[in] q      Trajectory as described above, must not be NULL.
 *  \param[in] dt     Time interval between two consecutive joint vectors. This
 *                    is needed to determine the joint velocities.
 *  \return True if no limit hs been violated, false otherwise.
 */
bool RcsGraph_checkJointTrajectory(const RcsGraph* self, const MatNd* q,
                                   double dt);

/*! \ingroup RcsGraphFunctions
 *  \brief Copies the lower and upper joint limits into q_lower and q_upper.
 *         The arrays hold all dof for RcsStateTypeFull, and the unconstrained
 *         ones for RcsStateTypeIK. The arrays are reshaped to column vectors
 *         of the corresponsing size.
 */
void RcsGraph_getJointLimits(const RcsGraph* self, MatNd* q_lower,
                             MatNd* q_upper, RcsStateType type);

/*! \ingroup RcsGraphFunctions
 *  \brief Copies the joint speed limits into q_dot_limit. The arrays hold all
 *         dof for RcsStateTypeFull, and the unconstrained ones for
 *         RcsStateTypeIK. The array is reshaped to a column vector
 *         of the corresponsing size.
 */
void RcsGraph_getSpeedLimits(const RcsGraph* self, MatNd* q_dot_limit,
                             RcsStateType type);

/*! \ingroup RcsGraphFunctions
 *  \brief Copies the joint torque limits into T_limit. The arrays hold all
 *         dof for RcsStateTypeFull, and the unconstrained ones for
 *         RcsStateTypeIK. The array is reshaped to a column vector
 *         of the corresponsing size.
 */
void RcsGraph_getTorqueLimits(const RcsGraph* self, MatNd* T_limit,
                              RcsStateType type);

/*! \ingroup RcsGraphFunctions
 *  \brief Returns the joint angle of the joint with the given name. If this
 *         joint doesn't exist, the function will terminate the application.
 */
double RcsGraph_getJointValue(const RcsGraph* self, const char* name);

/*! \ingroup RcsGraphFunctions
 *  \brief Returns a pointer to the joint with the indicated name. If
 *         several joints have the same name, the first one in the order of
 *         construction will be taken. If no matching joint is found, NULL
 *         will be returned.
 *
 *  \param[in] self  Pointer to graph. Must be not NULL.
 *  \param[in] name  Character array holding the name of the joint. Must not
 *                   be NULL.
 *  \return Pointer to joint with the given name. If no joint with this name
 *          exist, the function returns NULL.
 */
RcsJoint* RcsGraph_getJointByName(const RcsGraph* self, const char* name);

/*! \ingroup RcsGraphFunctions
 *  \brief Similar to RcsGraph_getJointByName() except that only the number of
 *         characters of argument name are compared. This allows to find joints
 *         with names that have a suffix.
 *  \param[in] self   Pointer to graph, must not be NULL.
 *  \param[in] name   Character array holding the first characters of the joint
 *                    name, must not be NULL.
 *  \return Pointer to joint with the given name. If several joints with the
 *          same name exist, the first one according to a depth-first
 *          traversal is returned. If no joint with this name exist, the
 *          function returns NULL.
 */
RcsJoint* RcsGraph_getJointByTruncatedName(const RcsGraph* self,
                                           const char* name);

/*! \ingroup RcsGraphFunctions
 *  \brief Returns a pointer to the joint with the given index. If no matching
 *         joint is found, NULL will be returned.
 *
 *  \param[in] self   Pointer to graph. Must not be NULL.
 *  \param[in] idx    Index of the joint, see above description.
 *  \param[in] type   If type is RcsStateIK, the searched index is the
 *                    jacobiIndex, otherwise the jointIndex.
 *  \return Pointer to joint with the corresponsing index. If no joint with
 *          this index exist, the function returns NULL.
 */
RcsJoint* RcsGraph_getJointByIndex(const RcsGraph* self, unsigned int idx,
                                   RcsStateType type);

/*! \ingroup RcsGraphFunctions
 *  \brief Sets the value of the indicated degree of freedom to the value.
 *         If the joint doesn't exist, false will be returned, and a debug
 *         message will be outputted on the console on debug level 1. The
 *         joint angle will then remain unchanged.
 */
bool RcsGraph_setJoint(RcsGraph* self, const char* jntName, double val);

/*! \ingroup RcsGraphFunctions
 *  \brief Limits the joint speeds according to the limits given.
 *         The function returns the scaling factor with which dq is scaled.
 *         The array dq is assumed to be of size nq x 1 or 1 x nq, where
 *         nq is RcsGraph::nJ for type being RcsStateIK, and RcsGraph::dof
 *         for type being RcsStateFull.
 *
 *  \param[in] self     Valid pointer to graph.
 *  \param[in,out] dq   Array of joint displacements. The array may be a row
 *                      or a column vector. See explanations about
 *                      dimensions above.
 *  \param[in] dt       Time step to be considered for the displacement vector.
 *                      If dq corresponds to velocities, dt should be 1.
 *  \param[in] type     If type is RcsStateFull, the large dimension of the
 *                      array dq is assumed to be RcsGraph::dof, otherwise
 *                      it must be RcsGraph::nJ.
 *  \return Scaling factor to be applied to dq so that it satisfies all joint
 *          speed limits. If it is 1, no scaling is needed.
 */
double RcsGraph_limitJointSpeeds(const RcsGraph* self, MatNd* dq,
                                 double dt, RcsStateType type);

/*! \ingroup RcsGraphFunctions
 *  \brief Checks the joint speeds according to the limits given. The function
 *         returns the scaling factor with which dq needs to be scaled.
 *         The array dq is assumed to be of size nq x 1 or 1 x nq, where
 *         nq is RcsGraph::nJ for type being RcsStateIK, and RcsGraph::dof
 *         for type being RcsStateFull.
 *
 *  \param[in] self     Valid pointer to graph.
 *  \param[in] dq       Array of joint displacements. The array may be a row
 *                      or a column vector. See explanations about
 *                      dimensions above.
 *  \param[in] dt       Time step to be considered for the displacement vector.
 *                      If dq corresponds to velocities, dt should be 1.
 *  \param[in] type     If type is RcsStateFull, the large dimension of the
 *                      array dq is assumed to be RcsGraph::dof, otherwise
 *                      it must be RcsGraph::nJ.
 *  \return Scaling factor to be applied to dq so that it satisfies all joint
 *          speed limits. If it is 1, no scaling is needed.
 */
double RcsGraph_checkJointSpeeds(const RcsGraph* self, const MatNd* dq,
                                 double dt, RcsStateType type);

/*! \ingroup RcsGraphFunctions
 *  \brief Clips the joint speeds qdot independently according to the limits
 *         given. The array dq is assumed to be of size nq x 1 or 1 x nq, where
 *         nq is RcsGraph::nJ for type being RcsStateIK, and RcsGraph::dof
 *         for type being RcsStateFull.
 *
 *  \param[in] self      Valid pointer to graph.
 *  \param[in] qdot      Array of joint velocities. The array may be a row
 *                       or a column vector. See explanations about
 *                       dimensions above.
 *  \param[in] qdot_prev Array of the joint velocities of the previous time
 *                       step. The array may be a row. See explanations about
 *                       dimensions above.
 *  \param[in] dt        Time between qdot and qdot_prev.
 *  \param[in] ratio     Scale to a ratio of the limts. If ratio is 1, the
 *                       values will be clipped to the limits, for 0.5 to half
 *                       of the limits etc.
 *  \param[in] type      If type is RcsStateFull, the large dimension of the
 *                       array dq is assumed to be RcsGraph::dof, otherwise
 *                       it must be RcsGraph::nJ.
 *  \return Number of joints that have been clipped.
 */
int RcsGraph_clipJointAccelerations(const RcsGraph* self, MatNd* qdot,
                                    const MatNd* qdot_prev, double dt,
                                    double ratio, RcsStateType type);

///@}



/**
 * @name Coupled Joints
 *
 * Functions to handle kinematically coupled joints
 */

///@{

/*! \ingroup RcsGraphFunctions
 *  \brief Returns the number of kinematically coupled joints. Coupled
 *         joints of joints that are constrained are not considered.
 *
 *  \param[in] self  Pointer to a valid RcsGraph (Must not be NULL)
 */
int RcsGraph_countCoupledJoints(const RcsGraph* self);

/*! \ingroup RcsGraphFunctions
 *  \brief Constraint matrix for coupled joints and its inverse. The matrix
 *         \f$ \mathbf{A^{-1}} \f$ reduces the number of degrees of freedom
 *         by the number of coupled joints:
 *
 *         \f[ \mathbf{ \delta q_{red} = A^{-1} \delta q } \f]
 *
 *         The matrix \f$ \mathbf{A} \f$ expands the number of degrees of
 *         freedom by the number of coupled joints:
 *
 *         \f[ \mathbf{ \delta q = A \delta q_{red}}\f]
 *
 *         Since the Jacobians are holding the dof in the columns, it's the
 *         other way around:
 *
 *         \f[ \mathbf{ J_{red} = J A }\f]
 *
 *         We create a reduced Jacobian JA = J * A with matrix A being the
 *         constraint matrix. The inverse kinematics then becomes
 *
 *         \f[ \mathbf{ \delta q = A (J A)^{\#} \delta x}\f]
 *
 *         Matrix A is of dimension nq x nqr, where
 *         nqr = nq - nCoupledJoints.
 *
 *         The function returns the number of coupled joints.
 *
 *         Note: It is currently not possible to couple a joint to another
 *               coupled joint.
 *
 *  \param[in] self  Pointer to a valid RcsGraph (Must not be NULL)
 *  \param[out] A       Coupled joints reduction matrix. It will be reshaped
 *                      to RcsGraph::nJ x nqr, where nqr is the dimension of
 *                      the reduced state space by the coupled joints.
 *  \param[out] invA    nqr x nq coupled joints reduction matrix. It will be
 *                      reshaped to RcsGraph::nJ x nqr, where nqr is the
 *                      dimension of the reduced state space by the coupled
 *                      joints.
 */
int RcsGraph_coupledJointMatrix(const RcsGraph* self, MatNd* A, MatNd* invA);

/*! \ingroup RcsGraphFunctions
 *  \brief Updates the values of the slave joints to be consistent with the
 *         value of the master joint. The function returns true if one or
 *         more joint values are changed.
 *
 *  \param[in] self  Pointer to a valid RcsGraph (Must not be NULL)
 *  \param[in,out] q State vector (Dimension must be RcsGraph::dof x 1). The
 *                   elements corresponding to slave joints will be updated
 *                   according to the joint coupling equation. All other
 *                   elements remain unchanged.
 *  \param[in,out] qp State velocity vector. If this argument is NULL, the
 *                    velocities will not get updated. Otherwise, qp's
 *                    dimension must be RcsGraph::dof x 1). The velocities
 *                    corresponding to slave joints will be updated according
 *                    to the joint coupling equation. All other elements
 *                    remain unchanged.
 */
bool RcsGraph_updateSlaveJoints(const RcsGraph* self, MatNd* q, MatNd* qp);

///@}



/**
 * @name Rigid body degrees of freedom
 *
 * Functions to access / modify a bodie's rigid body degrees of freedom
 */

///@{

/*! \ingroup RcsGraphFunctions
 *  \brief Sets the degrees of freedom of all bodies that have the flag
 *         rigid_body_joints to their initial values (q_init), and their
 *         velocities to zero, and then computes the forward kinematics.
 *         All other internal q and qp vector elements remain unchanged.
 */
void RcsGraph_resetRigidBodyDofs(RcsGraph* self);

/*! \ingroup RcsGraphFunctions
 *  \brief Copies the joint angles of all bodies that have the flag
 *         rigid_body_joints into q. Array q is assumed to have the same
 *         dimensions as the graph's q-vector (RcsGraph::dof).
 *
 *  \param[in,out] q   Array of joint angles. The size of this array must
 *                     match the size of the q-array of the second argument
 *                     (self).
 *  \param[in]  self   The graph against which the rigid body dofs are
 *                     compared and updated.
 *  \param[in]  src    If src is != NULL, then this q array will be used
 *                     as src instead of self->q
 *  \return True if one or more rigid body joint values in q and the graph
 *          differed (and have been updated), false if the values are
 *          identical.
 */
bool RcsGraph_copyRigidBodyDofs(MatNd* q, const RcsGraph* self,
                                const MatNd* src);

/*! \ingroup RcsGraphFunctions
 *  \brief Computes the angles of three translational and three rotational
 *         (euler) joints connecting a rigid body to its parent body (can
 *         be ROOT), if the body is constructed with the "rigid_body_joints"
 *         xml tag. Pass the RcsBody and optional A_KIs for the body and
 *         its parent body. The six joint angles are written to "angles".
 *         The calculation also considers the optional, constant \f$A_{KV}\f$
 *         transformation from after the six joints to the body.
 *
 *  Assuming the given body is body \f$1\f$ and its parent is body
 *  \f$0\f$ with transformations \f$A^1_{KI}\f$ and \f$A^0_{KI}\f$
 *  respectively, and the frame before body \f$1\f$ that is connected
 *  via \f$A_{KV}\f$ be named \f$1'\f$, the computed rotation of the
 *  wanted transformation from frame \f$0\f$ to frame \f$1'\f$ is
 *  given by
 *  \f[
 *    A_{1'0} = A_{1'1} A_{1I} A_{I0}
 *            = A^1_{KV}\mbox{.rot}^T \, A^1_{KI}\mbox{.rot} \,\, A^0_{KI}\mbox{.rot}^T
 *  \f]
 *  and the translation by
 *  \f{eqnarray*}{
 *    _0r_{01'} &=& A_{0I} (_Ir_{I1'} - _Ir_{I0} ) \\
 *              &=& A_{0I} (_Ir_{I1} - _Ir_{1'1} - _Ir_{I0} ) \\
 *              &=& A_{0I} (_Ir_{I1} - A_{I1} A_{11'} \;_{1'}r_{1'1} - _Ir_{I0} ) \\
 *              &=& A^0_{KI}\mbox{.rot} \, (A^1_{KI}\mbox{.org}
 *                  - A^1_{KI}\mbox{.rot} \,\, A^1_{KV}\mbox{.rot} \,\, A^1_{KV}\mbox{.org}
 *                  -  A^0_{KI}\mbox{.org} \,)
 *  \f}
 */
void RcsGraph_relativeRigidBodyDoFs(const RcsGraph* self,
                                    const RcsBody* body,
                                    const HTr* new_A_KI_body,
                                    const HTr* new_A_KI_parent,
                                    double angles[6]);

/*! \ingroup RcsGraphFunctions
 *  \brief Copies the joint angles into the graph's state vector elements that
 *         correspond to the rigid body degrees of freedom.
 *
 *  \param[in,out] self Pointer to a valid RcsGraph
 *  \param[in] body     Pointer to the body that owns the rigid body joints
 *  \param[in] angles   Six values to be assigned to the rigid body joints in
 *                      the order x, y, z, thx, thy, thz
 *  \return True for success, false for failure:
 *          - self is NULL
 *          - body is NULL
 *          - body has no rigid body degrees of freedom
 */
bool RcsGraph_setRigidBodyDoFs(RcsGraph* self, const RcsBody* body,
                               const double angles[6]);

///@}


/**
 * @name Graph printing and writing to file
 *
 * Printing the graph in various formats to various outputs.
 */

///@{

/*! \ingroup RcsGraphFunctions
 *  \brief Prints the graph's xml representation to a file with the given name.
 *         If the file cannot be written, the function returns false and
 *         complains on debug level 1.
 *
 *  \param[in] fileName   Desired file name for xml file
 *  \param[in] self       Pointer to a valid RcsGraph
 */
bool RcsGraph_toXML(const char* fileName, const RcsGraph* self);

/*! \ingroup RcsGraphFunctions
 *  \brief Prints the graph's usage description to stdout.
 *
 *  \param[in] xmlFile   RcsGraph xml file (not required to be full path)
 */
void RcsGraph_printUsage(const char* xmlFile);

/*! \ingroup RcsGraphFunctions
 *  \brief Prints the graph's xml representation to the given file
 *         descriptor. Both arguments are assumed to be not NULL. Otherwise,
 *         the function exits with a fatal error.
 *
 *  \param[in] out  Valid file descriptor for xml file
 *  \param[in] self  Pointer to a valid RcsGraph
 */
void RcsGraph_fprintXML(FILE* out, const RcsGraph* self);

/*! \ingroup RcsGraphFunctions
 *  \brief Prints out the graph to a file descriptor.
 */
void RcsGraph_fprint(FILE* out, const RcsGraph* self);

/*! \ingroup RcsGraphFunctions
 *  \brief Prints all joints of the graph in consecutive (starting
 *         from root) order to a file.
 */
void RcsGraph_fprintJoints(FILE* out, const RcsGraph* self);

/*! \ingroup RcsGraphFunctions
 *  \brief Prints the Jacobian recursion path starting in the indicated body
 *         to a file descriptor.
 */
void RcsGraph_fprintJointRecursion(FILE* out, const RcsGraph* self,
                                   const char* bdyName);

/*! \ingroup RcsGraphFunctions
 *  \brief Prints out the state vector in a human readeable form. The
 *         function automatically determines if the vector is of type
 *         RcsStateFull or RcsStateIK. If neither is the case or the array
 *         has not exactly one column, it will be printed without human-
 *         readable units, and a warning will be issued.
 */
void RcsGraph_printState(const RcsGraph* self, const MatNd* q);

/*! \ingroup RcsGraphFunctions
 *  \brief Prints out the state vector in the format of the xml model state. The
 *         function automatically determines if the vector is of type
 *         RcsStateFull or RcsStateIK. If neither is the case or the array
 *         has not exactly one column, it will be printed without human-
 *         readable units, and a warning will be issued.
 */
void RcsGraph_fprintModelState(FILE* out, const RcsGraph* self, const MatNd* q);

/*! \ingroup RcsGraphFunctions
 *  \brief Writes the graph to a file in the Graphviz dot format. The graph
 *         can then be visualized with for instance "dotty" or any
 *         other graphviz capable visualization tool. The file will be written
 *         in the same directory as the xml graph file.
 *
 *  \return True for success, false if the file could not be written.
  */
bool RcsGraph_writeDotFile(const RcsGraph* self, const char* filename);

/*! \ingroup RcsGraphFunctions
 *  \brief Writes the depth-first-traversal structure of the graph to a file in
 *         the Graphviz dot format. The graph can then be visualized with for
 *         instance "dotty" or any other graphviz capable visualization tool.
 *         The file will be written in the same directory as the xml graph file.
 */
bool RcsGraph_writeDotFileDfsTraversal(const RcsGraph* self,
                                       const char* filename);

/*! \ingroup RcsGraphFunctions
 *  \brief Writes the graph to a xml file in the Rcs standard format. The graph
 *         can then be visualized with for instance Rcs test mode 2.
 *
 *  \return True for success, false if the file could not be written.
 */
bool RcsGraph_writeXmlFile(const RcsGraph* self, const char* filename);

/*! \ingroup RcsGraphFunctions
 *  \brief Creates a mesh of the overall graph by traversing through all bodies
 *         and shapes. The mesh will be scaled with the given scale factor.
 */
RcsMeshData* RcsGraph_meshify(const RcsGraph* self, double scale,
                              char computeType);


///@}


/**
 * @name Sensor-related graph functions
 *
 */

///@{

/*! \ingroup RcsGraphFunctions
 *  \brief Returns a pointer to the sensor with the indicated name. If
 *         several sensors have the same name, the closest one to the root
 *         node will be taken. If no matching sensor is found, NULL will be
 *         returned.
 */
RcsSensor* RcsGraph_getSensorByName(const RcsGraph* self, const char* name);

/*! \ingroup RcsGraphFunctions
 *  \brief Returns the id of the first sensor with the indicated name. If no
 *         matching sensor is found, -1 will be returned.
 */
int RcsGraph_getSensorIdByName(const RcsGraph* graph, const char* name);

///@}


#ifdef __cplusplus
}
#endif

#endif   // RCS_GRAPH_H
