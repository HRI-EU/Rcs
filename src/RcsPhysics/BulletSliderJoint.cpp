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

#include "BulletSliderJoint.h"
#include "BulletRigidBody.h"
#include "BulletHelpers.h"

#include <Rcs_typedef.h>
#include <Rcs_macros.h>
#include <Rcs_math.h>
#include <Rcs_joint.h>



/*******************************************************************************
 *
 ******************************************************************************/
Rcs::BulletSliderJoint::BulletSliderJoint(const RcsGraph* graph_,
                                          int jntId, double q0,
                                          btRigidBody& rbA, btRigidBody& rbB,
                                          const btTransform& frameInA,
                                          const btTransform& frameInB,
                                          bool useReferenceFrameA,
                                          bool enableForceLimit):
  BulletJointBase(),
  btSliderConstraint(rbA, rbB, frameInA, frameInB, useReferenceFrameA),
  graph(graph_), rcsJointId(jntId), constraintPosCurr(0.0),
  constraintPosPrev(0.0), jointPosCurr(0.0), jointPosPrev(0.0),
  jointVelocity(0.0), jointVelocityPrev(0.0), jointAcceleration(0.0),
  offset(0.0), jf(), withForceLimit(enableForceLimit)
{
  RCHECK(rcsJointId!=-1);
  const RcsJoint* rcsJoint = getJoint();
  RCHECK(RcsJoint_isTranslation(rcsJoint));

  // The below line would be ideal, unfortunately getLinearPos() Is not
  // properly initialized
  // this->offset = btSliderConstraint::getLinearPos() - q0;
  this->offset = - q0;

  // Same here.
  this->constraintPosCurr = 0.0;//getConstraintPos();
  this->constraintPosPrev = constraintPosCurr;

  this->jointPosCurr = constraintPosCurr - this->offset;
  this->jointPosPrev = this->jointPosCurr;

  if ((rcsJoint->ctrlType == RCSJOINT_CTRL_POSITION) ||
      (rcsJoint->ctrlType == RCSJOINT_CTRL_VELOCITY))
  {
    RLOG(5, "Enabling motor for joint %s", rcsJoint->name);
    if (withForceLimit)
    {
      setJointLimit(true, rcsJoint->q_min, rcsJoint->q_max);
    }
    else
    {
      setJointLimit(true, q0, q0);
    }
  }
  else
  {
    setJointLimit(true, rcsJoint->q_min, rcsJoint->q_max);
  }

  // Disallow the rotation around the slider axis
  setLowerAngLimit(0.0);
  setUpperAngLimit(0.0);

  // This allows us to query inter-body reaction forces and moments
  setJointFeedback(&this->jf);
  enableFeedback(true);
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::BulletSliderJoint::~BulletSliderJoint()
{
}

/*******************************************************************************
 *
 ******************************************************************************/
double Rcs::BulletSliderJoint::getJointPosition() const
{
  return this->jointPosCurr;
}

/*******************************************************************************
 *
 ******************************************************************************/
double Rcs::BulletSliderJoint::getJointVelocity() const
{
  return this->jointVelocity;
}

/*******************************************************************************
 *
 ******************************************************************************/
double Rcs::BulletSliderJoint::getJointAcceleration() const
{
  return this->jointAcceleration;
}

/*******************************************************************************
 *
 ******************************************************************************/
double Rcs::BulletSliderJoint::getJointTorque() const
{
  RFATAL("Not implemented yet");
}

/*******************************************************************************
 *
 ******************************************************************************/
unsigned int Rcs::BulletSliderJoint::getJointIndex() const
{
  return getJoint()->jointIndex;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletSliderJoint::update(double dt)
{
  RCHECK(dt>0.0);

  this->constraintPosPrev = this->constraintPosCurr;
  this->constraintPosCurr = getConstraintPos();

  this->jointPosPrev = this->jointPosCurr;
  this->jointPosCurr = this->constraintPosCurr - this->offset;

  this->jointVelocityPrev = this->jointVelocity;
  this->jointVelocity = (this->jointPosCurr-this->jointPosPrev)/dt;

  this->jointAcceleration = (this->jointVelocity-this->jointVelocityPrev)/dt;
}

/*******************************************************************************
 * There's two options for setting the joint position. Using a linear motor,
 * or modifying the joint limits. It turned out to be a bit more stable to do
 * the latter one.
 ******************************************************************************/
void Rcs::BulletSliderJoint::setJointPosition(double angle, double dt)
{
  if (withForceLimit)
  {
    setPoweredLinMotor(true);
    setMaxLinMotorForce(getJoint()->maxTorque);
    double vel = (angle-getJointPosition())/dt;
    vel = Math_clip(vel, -0.5, 0.5);
    setTargetLinMotorVelocity(vel);
  }
  else
  {
    setJointLimit(true, angle, angle);
  }

}

/*******************************************************************************
 * Below from Gazebo
 ******************************************************************************/
void Rcs::BulletSliderJoint::setJointTorque(double torque, double dt)
{
  const RcsJoint* rcsJoint = getJoint();

  setJointLimit(false, 1.0, -1.0);

  if (rcsJoint->coupledToId != -1)
  {
    RLOGS(1, "Joint \"%s\" is coupled to joint %s - not supported",
          rcsJoint->name,
          RCSJOINT_BY_ID(graph, rcsJoint->coupledToId)->name);
    return;
  }

  torque = Math_clip(torque, -rcsJoint->maxTorque, rcsJoint->maxTorque);

  NLOGS(0, "Setting torque for joint \"%s\" to %g Nm",
        rcsJoint ? rcsJoint->name : "NULL", torque);

  // z-axis of constraint frame
  btVector3 hingeAxisLocalA = getFrameOffsetA().getBasis().getColumn(0);
  btVector3 hingeAxisLocalB = getFrameOffsetB().getBasis().getColumn(0);

  btVector3 hingeAxisWorldA =
    getRigidBodyA().getWorldTransform().getBasis()*hingeAxisLocalA;
  btVector3 hingeAxisWorldB =
    getRigidBodyB().getWorldTransform().getBasis()*hingeAxisLocalB;

  btVector3 forceA = torque*hingeAxisWorldA;
  btVector3 forceB = -torque*hingeAxisWorldB;

  // We use the btTypedConstraint namespace since the getRigidBody() methods
  // are only overwritten as const in the btSliderConstraint class
  btTypedConstraint::getRigidBodyA().applyCentralForce(forceA);
  btTypedConstraint::getRigidBodyB().applyCentralForce(forceB);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletSliderJoint::setJointLimit(bool enable,
                                           double q_min, double q_max)
{
  if (enable)
  {
    setLowerLinLimit(btScalar(q_min+this->offset));
    setUpperLinLimit(btScalar(q_max+this->offset));
  }
  else
  {
    // Joint limits are automatically disabled if lower kimit > upper limit
    setLowerLinLimit(1.0);
    setUpperLinLimit(-1.0);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletSliderJoint::reset(double q)
{
  this->constraintPosCurr = q + this->offset;
  this->constraintPosPrev = constraintPosCurr;
  this->jointPosCurr = q;
  this->jointPosPrev = jointPosCurr;
  this->jointVelocity = 0.0;
  this->jointVelocityPrev = 0.0;
  this->jointAcceleration = 0.0;
}

/*******************************************************************************
 *
 ******************************************************************************/
double Rcs::BulletSliderJoint::getConstraintPos()
{
  return btSliderConstraint::getLinearPos();
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::BulletSliderJoint::isHinge() const
{
  return false;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::BulletSliderJoint::isSlider() const
{
  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
const RcsJoint* Rcs::BulletSliderJoint::getJoint() const
{
  return &graph->joints[this->rcsJointId];
}
