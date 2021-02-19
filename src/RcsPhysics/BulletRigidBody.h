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

#ifndef RCS_BULLETRIGIDBODY_H
#define RCS_BULLETRIGIDBODY_H

#include "PhysicsConfig.h"

#include <Rcs_graph.h>

#include <btBulletDynamicsCommon.h>


namespace Rcs
{

class BulletRigidBody : public btRigidBody
{
  friend class BulletSimulation;
  friend class BulletSoftSimulation;

public:
  void getBodyTransform(HTr* A_BI) const;
  const HTr* getCOMTransformPtr() const;
  void getRelativeBodyVelocity(double x_dot[3], double omega[3],
                               const BulletRigidBody* other) const;
  void getBodyVelocity(double x_dot[3], double omega[3]) const;
  const BulletRigidBody* getParent() const;

private:
  const HTr* getBodyTransformPtr() const;

  /*! \brief Factory method for creating a BulletRigidBody instance. It reads
   *         all Bullet-related parameters from the config file and sets them
   *         to the body. Further, all Bullet shapes are created and attached
   *         to the BulletRigidBody.
   *
   *  \return Pointer to class instance in case of success, NULL otherwise:
   *          - bdy is NULL
   *          - bdy has physicsSim flag disabled
   *          - bdy has no shapes
   */
  static BulletRigidBody* create(const RcsGraph* graph,
                                 const RcsBody* body,
                                 const PhysicsConfig* config);

  BulletRigidBody(const btRigidBody::btRigidBodyConstructionInfo& rbInfo,
                  const RcsGraph* graph, const RcsBody* body);
  virtual ~BulletRigidBody();

  void clearCompoundShapes();
  btFixedConstraint* createFixedJoint(const RcsGraph* graph);
  btTypedConstraint* createJoint(const RcsGraph* graph);
  void updateBodyTransformFromPhysics();
  void setParentBody(BulletRigidBody* parent);


  /*! \brief Determines the transformation of the Bullet rigid body. Its
   *         origin is the bodie's center of mass, and the rotation is
   *         aligned with the inertia tensor. See implementation for
   *         details.
   */
  void getPhysicsTransform(HTr* A_PI) const;
  void reset(const HTr* A_BI=NULL);

  /*! \brief Creates and returns a Bullet collision shape from the RcsShape.
   *         Only shapes with enables physics compute types are considered.
   *         For RcsShapes with a mesh that has more vertices as specified in
   *         argument convexHullVertexLimit, the mesh will be condensed into
   *         a convex hull. That's why the argument sh is not a const pointer.
   */
  static btCollisionShape* createShape(RcsShape* sh,
                                       btTransform& relTrans,
                                       const RcsBody* body,
                                       unsigned int convexHullVertexLimit);

  btCollisionShape* getShape(const RcsShape* shape);

  /*! \brief Returns the RcsBody pointer. It is read from the graph by the
   *         stored body-id, so that there is no dangling pointer in case
   *         new bodies are added and the body array is re-allocated.
   */
  const RcsBody* getBodyPtr() const;

  /*! \brief Returns the RcsBodie's name. It is safely determined through
   *         the body-id.
   */
  const char* getBodyName() const;

  /*! \brief Calculate hinge point and axis in Bullet body coordinates.
   */
  void calcHingeTrans(const RcsJoint* jnt, btVector3& pivot, btVector3& axis);

  /*! \brief Calculate slider transformation in Bullet body coordinates.
   */
  btTransform calcSliderTrans(const RcsJoint* jnt);

  void setBodyTransform(const HTr* A_BI);
  void setBodyTransform(const HTr* A_BI, double dt);

  // Obsolete
  void getLocalBodyTransform(HTr* A_PB) const;

  const RcsGraph* graph;   // Pointer to simulation's graph
  int bodyId;              // Id of corresponding graph's body
  BulletRigidBody* parent; // Depth-first parent body
  HTr A_PB_;               // From body frame to physics frame
  HTr A_BI_;               // From world frame to body frame
  HTr A_PI_;               // From world frame to COM frame
  double x_dot[3];
  double omega[3];
  btJointFeedback jf;
  bool linearJointForceLimit;
  btFixedConstraint* fixedJnt;   // NULL if body not attached to fixed joint
};

}   // namespace Rcs

#endif   // RCS_BULLETRIGIDBODY_H
