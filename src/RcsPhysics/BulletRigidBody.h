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

private:
  const HTr* getBodyTransformPtr() const;
  static BulletRigidBody* create(const RcsBody* body,
                                 const PhysicsConfig* config);
  BulletRigidBody(const btRigidBody::btRigidBodyConstructionInfo& rbInfo,
                  const RcsBody* body);
  virtual ~BulletRigidBody();

  void clearShapes();
  void calcHingeTrans(const RcsJoint* jnt, btVector3& pivot, btVector3& axis);
  btTransform calcSliderTrans(const RcsJoint* jnt);
  btTypedConstraint* createFixedJoint(const RcsGraph* graph);
  btTypedConstraint* createJoint(const RcsGraph* graph);
  void updateBodyTransformFromPhysics();
  void setParentBody(BulletRigidBody* parent);
  void getPhysicsTransform(HTr* A_PI) const;
  void reset(const HTr* A_BI=NULL);
  static btCollisionShape* createShape(RcsShape* sh,
                                       btTransform& relTrans,
                                       const RcsBody* body,
                                       unsigned int convexHullVertexLimit);
  btCollisionShape* getShape(const RcsShape* shape);
  const RcsBody* getBodyPtr() const;
  void setBodyTransform(const HTr* A_BI);
  void setBodyTransform(const HTr* A_BI, double dt);
  const char* getBodyName() const;

  // Obsolete
  void getLocalBodyTransform(HTr* A_PB) const;

  const RcsBody* body;     // Pointer to corresponding graph's body
  BulletRigidBody* parent; // Depth-first parent body
  HTr A_PB_;               // From body frame to physics frame
  HTr A_BI_;               // From world frame to body frame
  HTr A_PI_;               // From world frame to COM frame
  double x_dot[3];
  double omega[3];
  btJointFeedback jf;
};

}   // namespace Rcs

#endif   // RCS_BULLETRIGIDBODY_H
