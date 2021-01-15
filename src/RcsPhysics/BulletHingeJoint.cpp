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

#include "BulletHingeJoint.h"
#include "BulletRigidBody.h"
#include "BulletHelpers.h"

#include <Rcs_typedef.h>
#include <Rcs_macros.h>
#include <Rcs_math.h>
#include <Rcs_joint.h>



/*******************************************************************************
 *
 ******************************************************************************/
Rcs::BulletHingeJoint::BulletHingeJoint(const RcsGraph* graph_,
                                        int jntId, double q0,
                                        btRigidBody& rbA, btRigidBody& rbB,
                                        const btVector3& pivotInA,
                                        const btVector3& pivotInB,
                                        const btVector3& axisInA,
                                        const btVector3& axisInB,
                                        bool useReferenceFrameA):
  BulletJointBase(),
  btHingeConstraint(rbA, rbB, pivotInA, pivotInB, axisInA, axisInB,
                    useReferenceFrameA),
  graph(graph_), rcsJointId(jntId), hingeAngleCurr(0.0), hingeAnglePrev(0.0),
  jointAngleCurr(0.0), jointAnglePrev(0.0), jointVelocity(0.0),
  jointVelocityPrev(0.0), jointAcceleration(0.0), flipAngle(0.0), offset(0.0),
  jf()
{
  const RcsJoint* jnt = RCSJOINT_BY_ID(graph, jntId);
  RCHECK(RcsJoint_isRotation(jnt));

  this->offset = getConstraintPos() - q0;
  this->hingeAngleCurr = getConstraintPos();
  this->hingeAnglePrev = hingeAngleCurr;

  this->jointAngleCurr = getConstraintPos() - this->offset + this->flipAngle;
  this->jointAnglePrev = this->jointAngleCurr;

  if ((jnt->ctrlType == RCSJOINT_CTRL_POSITION) ||
      (jnt->ctrlType == RCSJOINT_CTRL_VELOCITY))
  {
    enableMotor(true);
  }

  // Increase the constraint error correction, since we set a tiny global
  // cfm value to increase contact stability
  setParam(BT_CONSTRAINT_ERP, 0.8);

  // This allows us to query inter-body reaction forces and moments
  setJointFeedback(&this->jf);
  enableFeedback(true);

  // Initialize with given joint angle q0
  setJointPosition(q0, 1.0);

  // Limit joint movement to RsJoint range
  if (jnt->q_max - jnt->q_min < 2.0*M_PI)
  {
    setJointLimit(true, jnt->q_min, jnt->q_max);
  }
  else
  {
    RLOG(1, "Joint %s has range > 360 deg (%.3f deg), disabling joint limits",
         jnt->name, RCS_RAD2DEG(jnt->q_max-jnt->q_min));
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::BulletHingeJoint::~BulletHingeJoint()
{
}

/*******************************************************************************
 *
 ******************************************************************************/
double Rcs::BulletHingeJoint::getJointPosition() const
{
  return this->jointAngleCurr;
}

/*******************************************************************************
 *
 ******************************************************************************/
const RcsJoint* Rcs::BulletHingeJoint::getJoint() const
{
  return RCSJOINT_BY_ID(graph, this->rcsJointId);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletHingeJoint::setJointPosition(double angle, double dt)
{
  enableMotor(true);
  double maxImpulse = Math_clip(getJoint()->maxTorque*dt, 0.0, 1.0);
  setMaxMotorImpulse(maxImpulse);
  setMotorTarget(angle+this->offset-this->flipAngle, dt);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletHingeJoint::setJointTorque(double torque, double dt)
{
  const RcsJoint* jnt = getJoint();

  if (jnt->coupledToId != -1)
  {
    RLOGS(1, "Joint \"%s\" is coupled to joint %s - not supported",
          jnt->name, RCSJOINT_BY_ID(this->graph, jnt->coupledToId)->name);
    return;
  }

#if 1
  enableMotor(false);

  torque = Math_clip(torque, -jnt->maxTorque, jnt->maxTorque);

  NLOGS(0, "Setting torque for joint \"%s\" to %g Nm",
        jnt ? jnt->name : "NULL", torque);

  // z-axis of constraint frame
  btVector3 hingeAxisLocalA = getFrameOffsetA().getBasis().getColumn(2);
  btVector3 hingeAxisLocalB = getFrameOffsetB().getBasis().getColumn(2);

  btVector3 hingeAxisWorldA =
    getRigidBodyA().getWorldTransform().getBasis()*hingeAxisLocalA;
  btVector3 hingeAxisWorldB =
    getRigidBodyB().getWorldTransform().getBasis()*hingeAxisLocalB;

  getRigidBodyA().applyTorque(torque*hingeAxisWorldA);
  getRigidBodyB().applyTorque(-torque*hingeAxisWorldB);

#else

  //enableMotor(true);
  //setMaxMotorImpulse(torque*dt); // 1.0f / 8.0f is about the minimum
  //setMotorTarget(torque >= 0.0 ? 1.0e6 : -1.0e6, dt);

  enableAngularMotor(true, torque >= 0.0 ? 1.0e1 : -1.0e1, torque*dt);

#endif
}

/*******************************************************************************
 * Bullet does not support joint limits outside a range of [-2*pi ... 2*pi].
 * If limits outside that range are set, the joint doesn't seem to move. We
 * therefore check this and only apply the joint limits when the condition is
 * fine. We also allow cases where the limits are exactly 2*pi, since that's
 * a common case in many configuration files. For these angles we modify the
 * limits a tiny little bit to eb within the range, and hope that nobody will
 * get mad at us.
 ******************************************************************************/
void Rcs::BulletHingeJoint::setJointLimit(bool enable,
                                          double q_min, double q_max)
{
  btScalar ll = btScalar(q_min + this->offset);
  btScalar ul = btScalar(q_max + this->offset);

  if (ll == -2.0*M_PI)
  {
    ll += 1.0e-12;
  }

  if (ul == 2.0*M_PI)
  {
    ul -= 1.0e-12;
  }

  if ((ll > -2.0*M_PI) && (ul < 2.0*M_PI))
  {
    setLimit(ll, ul);
  }
  else
  {
    RLOG(1, "[%s]: Joint limits outside [-2*pi ... 2*pi] not supported",
         getJoint()->name);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
double Rcs::BulletHingeJoint::getJointVelocity() const
{
  return this->jointVelocity;
}

/*******************************************************************************
 *
 ******************************************************************************/
double Rcs::BulletHingeJoint::getJointAcceleration() const
{
  return this->jointAcceleration;
}

/*******************************************************************************
 *
 ******************************************************************************/
unsigned int Rcs::BulletHingeJoint::getJointIndex() const
{
  return rcsJointId;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletHingeJoint::update(double dt)
{
  RCHECK(dt>0.0);

  this->hingeAnglePrev = this->hingeAngleCurr;
  this->hingeAngleCurr = getConstraintPos();

  // positive flip +179 -> -179
  if ((hingeAnglePrev>M_PI_2) && (hingeAngleCurr<-M_PI_2))
  {
    NLOGS(0, "POS FLIP %s", rcsJoint->name);
    this->flipAngle += 2.0*M_PI;
  }
  // negative flip -179 -> +179
  else if ((hingeAnglePrev<-M_PI_2) && (hingeAngleCurr>M_PI_2))
  {
    NLOGS(0, "NEG FLIP %s", rcsJoint->name);
    this->flipAngle -= 2.0*M_PI;
  }

  this->jointAnglePrev = this->jointAngleCurr;
  this->jointAngleCurr = getConstraintPos() - this->offset + this->flipAngle;

  this->jointVelocityPrev = this->jointVelocity;
  this->jointVelocity = (this->jointAngleCurr-this->jointAnglePrev)/dt;

  this->jointAcceleration = (this->jointVelocity-this->jointVelocityPrev)/dt;

  //getJointTorque();
}

/*******************************************************************************
 * M_J = M_A + M_B + r_JA x F_A + r_JB x F_B
 ******************************************************************************/
double Rcs::BulletHingeJoint::getJointTorque() const
{
  RFATAL("Fixme");
  if (!STREQ(getJoint()->name, "lbr_joint_5_L"))
  {
    return 0.0;
  }

  // Update joint torques
  const btJointFeedback* jf = getJointFeedback();

  if (jf==NULL)
  {
    RLOG(1, "No joint feedback found for joint \"%s\"", getJoint()->name);
    return 0.0;
  }

  const RcsJoint* rcsJoint = getJoint();

  // Force and torque in world coordinates to the COM of the attached body
  double F_A[3], F_B[3], M_B[3];//, M_A[3];
  F_A[0] = jf->m_appliedForceBodyA.x();
  F_A[1] = jf->m_appliedForceBodyA.y();
  F_A[2] = jf->m_appliedForceBodyA.z();
  // M_A[0] = jf->m_appliedTorqueBodyA.x();
  // M_A[1] = jf->m_appliedTorqueBodyA.y();
  // M_A[2] = jf->m_appliedTorqueBodyA.z();

  F_B[0] = jf->m_appliedForceBodyB.x();
  F_B[1] = jf->m_appliedForceBodyB.y();
  F_B[2] = jf->m_appliedForceBodyB.z();
  M_B[0] = jf->m_appliedTorqueBodyB.x();
  M_B[1] = jf->m_appliedTorqueBodyB.y();
  M_B[2] = jf->m_appliedTorqueBodyB.z();

  // Rotate forces and torques into joint's frame of reference
  const HTr* A_JI = &rcsJoint->A_JI;
  //Vec3d_rotateSelf (F_A, A_JI->rot);
  //Vec3d_rotateSelf (F_B, A_JI->rot);
  //Vec3d_rotateSelf (M_A, A_JI->rot);
  //Vec3d_rotateSelf (M_B, A_JI->rot);

  // Origins of body A, B and joint in world coordinates
  HTr A_AI, A_BI;
  HTrFromBtTransform(&A_AI, getRigidBodyA().getWorldTransform());
  HTrFromBtTransform(&A_BI, getRigidBodyB().getWorldTransform());

  // Lever arms in world coordinates
  double r_JA[3], r_JB[3];
  Vec3d_sub(r_JA, A_AI.org, A_JI->org);
  Vec3d_sub(r_JB, A_BI.org, A_JI->org);

  // Compute resultant torque in world coordinates:
  // M_J = M_A + M_B + r_JA x F_A + r_JB x F_B
  double M_J[3], tmp[3];
  Vec3d_setZero(M_J);
  //Vec3d_addSelf(M_J, M_A);
  Vec3d_addSelf(M_J, M_B);
  Vec3d_crossProduct(tmp, r_JA, F_A);
  //Vec3d_addSelf(M_J, tmp);
  Vec3d_crossProduct(tmp, r_JB, F_B);
  Vec3d_addSelf(M_J, tmp);

  // Project on constraint axis
  const double* jAxis = rcsJoint->A_JI.rot[rcsJoint->dirIdx];
  double torque = Vec3d_innerProduct(jAxis, M_J);

  // Rotate torque vector into joint frame
  Vec3d_rotateSelf(M_J, (double(*)[3])A_JI->rot);

  RLOG(0, "torque = %f %f %f (%f)", M_J[0], M_J[1], M_J[2], torque);

  return torque;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletHingeJoint::reset(double q)
{
  this->hingeAngleCurr = q + this->offset;
  this->hingeAnglePrev = hingeAngleCurr;
  this->jointAngleCurr = q;
  this->jointAnglePrev = jointAngleCurr;
  this->jointVelocity = 0.0;
  this->jointVelocityPrev = 0.0;
  this->jointAcceleration = 0.0;
  this->flipAngle = 0.0;
}

/*******************************************************************************
 *
 ******************************************************************************/
double Rcs::BulletHingeJoint::getConstraintPos()
{
  return btHingeConstraint::getHingeAngle();
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::BulletHingeJoint::isHinge() const
{
  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::BulletHingeJoint::isSlider() const
{
  return false;
}
