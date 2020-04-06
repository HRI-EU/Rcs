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

#ifndef RCS_BULLETHINGEJOINT_H
#define RCS_BULLETHINGEJOINT_H

#include "BulletJointBase.h"

#include <Rcs_graph.h>

#include <btBulletDynamicsCommon.h>



namespace Rcs
{
class BulletHingeJoint : public BulletJointBase, public btHingeConstraint
{
public:

  BulletHingeJoint(RcsJoint* jnt, double q0,
                   btRigidBody& rbA, btRigidBody& rbB,
                   const btVector3& pivotInA, const btVector3& pivotInB,
                   const btVector3& axisInA, const btVector3& axisInB,
                   bool useReferenceFrameA);
  virtual ~BulletHingeJoint();

  double getJointPosition() const;
  double getJointVelocity() const;
  double getJointAcceleration() const;
  double getJointTorque() const;
  unsigned int getJointIndex() const;
  void update(double dt);
  void setJointPosition(double angle, double dt);
  void setJointTorque(double torque, double dt);
  void setJointLimit(bool enable, double q_min, double q_max);
  void reset(double hingeAngle);
  bool isHinge() const;
  bool isSlider() const;

protected:
  
  double getConstraintPos();

private:

  RcsJoint* rcsJoint;
  double hingeAngleCurr;
  double hingeAnglePrev;
  double jointAngleCurr;
  double jointAnglePrev;
  double jointVelocity;
  double jointVelocityPrev;
  double jointAcceleration;
  double flipAngle;
  double offset;
  btJointFeedback jf;
};

}   // namespace Rcs

#endif   // RCS_BULLETHINGEJOINT_H
