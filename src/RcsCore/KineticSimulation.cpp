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

#include "KineticSimulation.h"
#include "PhysicsFactory.h"
#include "Rcs_typedef.h"
#include "Rcs_macros.h"
#include "Rcs_dynamics.h"
#include "Rcs_kinematics.h"
#include "Rcs_math.h"
#include "Rcs_body.h"
#include "Rcs_shape.h"
#include "Rcs_joint.h"
#include "Rcs_eigen.h"


namespace Rcs
{

static const char className[] = "NewtonEuler";
static PhysicsFactoryRegistrar<KineticSimulation> physics(className);

/*******************************************************************************
 * Constructor.
 ******************************************************************************/
KineticSimulation::KineticSimulation() : PhysicsBase(), draggerTorque(NULL),
  jointTorque(NULL), contactForces(NULL), contactPositions(NULL),
  integrator("Fehlberg"), energy(0.0), dt_opt(0.0), lastDt(0.0)
{
  Vec3d_set(this->gravity, 0.0, 0.0, -RCS_GRAVITY);
}

/*******************************************************************************
 * Constructor.
 ******************************************************************************/
KineticSimulation::KineticSimulation(const RcsGraph* graph_) :
  PhysicsBase(graph_), draggerTorque(NULL), jointTorque(NULL),
  contactForces(NULL), contactPositions(NULL), integrator("Fehlberg"),
  energy(0.0), dt_opt(0.0), lastDt(0.0)
{
  initialize(graph_, NULL);
}

/*******************************************************************************
 * Copy constructor.
 ******************************************************************************/
KineticSimulation::KineticSimulation(const KineticSimulation& copyFromMe) :
  PhysicsBase(copyFromMe), contact(copyFromMe.contact),
  constraint(copyFromMe.constraint), draggerTorque(NULL), jointTorque(NULL),
  contactForces(NULL), contactPositions(NULL),
  integrator(copyFromMe.integrator), energy(copyFromMe.energy),
  dt_opt(copyFromMe.dt_opt), lastDt(copyFromMe.lastDt)
{
  this->draggerTorque = MatNd_create(getGraph()->dof, 1);
  MatNd_reshapeCopy(this->draggerTorque, copyFromMe.draggerTorque);
  this->jointTorque = MatNd_clone(copyFromMe.jointTorque);
  this->contactForces = MatNd_clone(copyFromMe.contactForces);
  this->contactPositions = MatNd_clone(copyFromMe.contactPositions);
  Vec3d_copy(gravity, copyFromMe.gravity);
}

/*******************************************************************************
 * Copy constructor.
 ******************************************************************************/
KineticSimulation::KineticSimulation(const KineticSimulation& copyFromMe,
                                     const RcsGraph* newGraph) :
  PhysicsBase(copyFromMe, newGraph), contact(copyFromMe.contact),
  constraint(copyFromMe.constraint), draggerTorque(NULL),jointTorque(NULL),
  contactForces(NULL), contactPositions(NULL),
  integrator(copyFromMe.integrator), energy(copyFromMe.energy),
  dt_opt(copyFromMe.dt_opt), lastDt(copyFromMe.lastDt)
{
  this->draggerTorque = MatNd_create(getGraph()->dof, 1);
  MatNd_reshapeCopy(this->draggerTorque, copyFromMe.draggerTorque);
  this->jointTorque = MatNd_clone(copyFromMe.jointTorque);
  this->contactForces = MatNd_clone(copyFromMe.contactForces);
  this->contactPositions = MatNd_clone(copyFromMe.contactPositions);
  Vec3d_copy(gravity, copyFromMe.gravity);
}

/*******************************************************************************
 * Assignment operator.
 ******************************************************************************/
KineticSimulation& KineticSimulation::operator= (const KineticSimulation& other)
{
  // check for self-assignment by comparing the address of the
  // implicit object and the parameter
  if (this == &other)
  {
    return *this;
  }

  PhysicsBase::operator =(other);
  MatNd_resizeCopy(&this->draggerTorque, other.draggerTorque);
  MatNd_resizeCopy(&this->jointTorque, other.jointTorque);
  MatNd_resizeCopy(&this->contactForces, other.contactForces);
  MatNd_resizeCopy(&this->contactPositions, other.contactPositions);
  this->integrator = other.integrator;
  this->energy = other.energy;
  this->dt_opt = other.dt_opt;
  Vec3d_copy(gravity, other.gravity);
  return *this;
}

/*******************************************************************************
 * Destructor.
 ******************************************************************************/
KineticSimulation::~KineticSimulation()
{
  MatNd_destroy(this->draggerTorque);
  MatNd_destroy(this->jointTorque);
  MatNd_destroy(this->contactForces);
  MatNd_destroy(this->contactPositions);
}

/*******************************************************************************
 *
 ******************************************************************************/
bool KineticSimulation::initialize(const RcsGraph* g, const PhysicsConfig* cfg)
{
  Vec3d_set(this->gravity, 0.0, 0.0, -RCS_GRAVITY);
  initGraph(g);
  if (this->draggerTorque==NULL)
  {
    this->draggerTorque = MatNd_createLike(g->q);
    MatNd_reshape(this->draggerTorque, getGraph()->nJ, 1);
  }

  if (this->jointTorque==NULL)
  {
    this->jointTorque = MatNd_createLike(g->q);
  }

  // Initialize contact points
  RCSGRAPH_TRAVERSE_BODIES(getGraph())
  {
    int shapeIdx = 0;

    RCSBODY_TRAVERSE_SHAPES(BODY)
    {
      if (RcsShape_isOfComputeType(SHAPE, RCSSHAPE_COMPUTE_CONTACT))
      {
        const double mu = SHAPE->scale3d[0];
        const double stiffness = SHAPE->scale3d[1];
        const double z0 = SHAPE->scale3d[2];

        contact.push_back(FrictionContactPoint(BODY->id, shapeIdx,
                                               mu, stiffness, z0));
      }

      // Initialize kinematic constraints
      const double kp = SHAPE->scale3d[0];
      if (RcsShape_isOfComputeType(SHAPE, RCSSHAPE_COMPUTE_WELDPOS) &&
          RcsShape_isOfComputeType(SHAPE, RCSSHAPE_COMPUTE_WELDORI))
      {
        double x0[6];
        Vec3d_copy(x0, BODY->A_BI.org);
        Mat3d_toEulerAngles(&x0[3], BODY->A_BI.rot);
        std::vector<double> x_des(x0, x0+6);
        constraint.push_back(KinematicConstraint(KinematicConstraint::PosAndOri,
                                                 BODY->id, x_des, kp));
      }
      else if (RcsShape_isOfComputeType(SHAPE, RCSSHAPE_COMPUTE_WELDPOS))
      {
        std::vector<double> x_des(BODY->A_BI.org, BODY->A_BI.org+3);
        constraint.push_back(KinematicConstraint(KinematicConstraint::Pos,
                                                 BODY->id, x_des, kp));
      }
      else if (RcsShape_isOfComputeType(SHAPE, RCSSHAPE_COMPUTE_WELDORI))
      {
        double ea[3];
        Mat3d_toEulerAngles(ea, BODY->A_BI.rot);
        std::vector<double> x_des(ea, ea+3);
        constraint.push_back(KinematicConstraint(KinematicConstraint::Ori,
                                                 BODY->id, x_des, kp));
      }

      shapeIdx++;
    }
  }

  // Create arrays for contact forces and positions
  this->contactForces = MatNd_create(contact.size(), 3);
  this->contactPositions = MatNd_create(contact.size(), 3);

  // Initialize contact point with attachement point coordinates
  for (size_t i=0; i<contact.size(); ++i)
  {
    HTr A_CI;
    const RcsBody* bdy = &g->bodies[contact[i].bdyId];
    HTr_transform(&A_CI, &bdy->A_BI, &bdy->shapes[contact[i].shapeIdx].A_CB);
    MatNd_setRow(this->contactPositions, i, A_CI.org, 3);
  }

  // Remove kinematic constraints from all bodies with rigid body dofs
  RCSGRAPH_TRAVERSE_BODIES(getGraph())
  {
    if (RcsBody_isFloatingBase(getGraph(), BODY))
    {
      RCSBODY_FOREACH_JOINT(getGraph(), BODY)
      {
        JNT->constrained = false;
        JNT->weightMetric = 1.0;
        JNT->ctrlType = RCSJOINT_CTRL_TORQUE;
      }
    }
  }
  RcsGraph_setState(getGraph(), NULL, NULL);

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
void KineticSimulation::step(double dt)
{
  simulate(dt);
}

/*******************************************************************************
 *
 ******************************************************************************/
void KineticSimulation::simulate(double dt, MatNd* q, MatNd* q_dot,
                                 MatNd* q_ddot, MatNd* T, bool control)
{
  if (dt <= 0.0)
  {
    return;
  }

  // We need this to compute the correct accelerations inside variable step
  // size integration.
  this->lastDt = dt;

  incrementTime(dt);

  // This has two roles. Firstly, upon class instantiation and resetting,
  // dt_opt is resetted to 0. Here, we then intialize it to the incoming dt.
  // Secondly, the step size adaptation has no notion of an upper limit.
  // Therefore her we make sure that the adapted step size does not exceed
  // our desired dt.
  if ((dt_opt==0.0) || (dt_opt>dt))
  {
    dt_opt = dt;
  }

  const int n = getGraph()->nJ, nc = contact.size(), nz = 2*n+3*nc;
  MatNd* z = MatNd_create(nz, 1);
  MatNd zq = MatNd_fromPtr(n, 1, &z->ele[0]);
  MatNd zqd = MatNd_fromPtr(n, 1, &z->ele[n]);
  MatNd zc = MatNd_fromPtr(nc, 3, &z->ele[2*n]);

  // Copy joint positions and velocities into overal state vector z
  RcsGraph_stateVectorToIK(getGraph(), getGraph()->q, &zq);
  RcsGraph_stateVectorToIK(getGraph(), getGraph()->q_dot, &zqd);

  // Copy contact point positions into overal state vector z
  MatNd_copy(&zc, contactPositions);

  if (integrator == "Fehlberg")
  {
    MatNd* err = MatNd_create(nz, 1);
    MatNd err_q  = MatNd_fromPtr(n, 1, &err->ele[0]);
    MatNd err_qp = MatNd_fromPtr(n, 1, &err->ele[n]);
    MatNd err_xc = MatNd_fromPtr(3*nc, 1, &err->ele[2*n]);

    MatNd_setElementsTo(&err_xc, DBL_MAX);
    MatNd_setElementsTo(&err_q,  1.0e-3);
    MatNd_setElementsTo(&err_qp, 1.0e-4);

    integration_t1_t2(integrationStep, (void*)this, nz, time(), time()+dt,
                      &dt_opt, z->ele, z->ele, err->ele);
    MatNd_destroy(err);
  }
  else if (integrator == "Euler")
  {
    integration_euler(integrationStep, (void*)this, nz, dt, z->ele, z->ele);
  }
  else
  {
    RFATAL("Unknonw integrator: \"%s\"", integrator.c_str());
  }

  // Copy integrated state back into graph ...
  RcsGraph_setState(getGraph(), &zq, &zqd);

  // ... and into contact point coordinates
  MatNd_copy(contactPositions, &zc);

  // Clean up
  MatNd_destroy(z);

  if (q)
  {
    MatNd_reshapeCopy(q, getGraph()->q);
  }

  if (q_dot)
  {
    MatNd_reshapeCopy(q_dot, getGraph()->q_dot);
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
void KineticSimulation::reset()
{
  MatNd_copy(this->q_des, getGraph()->q);
  MatNd_setZero(getGraph()->q_dot);
  MatNd_setZero(this->q_dot_des);
  MatNd_setZero(this->T_des);

  // Update also the internal desired graph for the rigid body transforms.
  setControlInput(this->q_des, this->q_dot_des, this->T_des);

  // This leads to assigning dt_opt=dt in the next simulation step
  this->dt_opt = 0.0;
}

/*******************************************************************************
 *
 ******************************************************************************/
const char* KineticSimulation::getClassName() const
{
  return className;
}

/*******************************************************************************
 *
 ******************************************************************************/
void KineticSimulation::setGravity(const double newGravity[3])
{
  Vec3d_copy(this->gravity, newGravity);
}

/*******************************************************************************
 *
 ******************************************************************************/
void KineticSimulation::setForce(const RcsBody* body, const double F[3],
                                 const double p[3])
{
  RFATAL("FIXME");
}

/*******************************************************************************
 *
 ******************************************************************************/
void KineticSimulation::applyImpulse(const RcsBody* body, const double F[3],
                                     const double r[3])
{
  RFATAL("FIXME");
}

/*******************************************************************************
 *
 ******************************************************************************/
void KineticSimulation::applyForce(const RcsBody* body, const double f[3],
                                   const double r[3])
{
  MatNd_reshape(this->draggerTorque, getGraph()->nJ, 1);

  if (body == NULL)
  {
    MatNd_setZero(this->draggerTorque);
    return;
  }

  // Compute transpose Jacobian of force contact point
  MatNd* JT = MatNd_create(3, getGraph()->nJ);
  RcsGraph_bodyPointJacobian(getGraph(), body, r, NULL, JT);
  MatNd_transposeSelf(JT);

  // Project drag force on joints
  MatNd* dH = MatNd_create(getGraph()->nJ, 1);
  MatNd Fx = MatNd_fromPtr(3, 1, (double*)f);
  MatNd_mul(dH, JT, &Fx);
  MatNd_subSelf(this->draggerTorque, dH);

  MatNd_destroy(JT);
  MatNd_destroy(dH);
}

/*******************************************************************************
 *
 ******************************************************************************/
void KineticSimulation::applyTransform(const RcsBody* body, const HTr* A_BI)
{
  RFATAL("FIXME");
}

/*******************************************************************************
 *
 ******************************************************************************/
void KineticSimulation::applyLinearVelocity(const RcsBody* body,
                                            const double v[3])
{
  if (RcsBody_isFloatingBase(getGraph(), body))
  {
    const RcsJoint* jnt = &getGraph()->joints[body->jntId];
    Vec3d_copy(&getGraph()->q_dot->ele[jnt->jointIndex], v);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void KineticSimulation::applyAngularVelocity(const RcsBody* body,
                                             const double omega[3])
{
  if (RcsBody_isFloatingBase(getGraph(), body))
  {
    const RcsJoint* jnt = &getGraph()->joints[body->jntId];
    Vec3d_copy(&getGraph()->q_dot->ele[jnt->jointIndex+3], omega);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void KineticSimulation::getLinearVelocity(const RcsBody* body,
                                          double v[3]) const
{
  Vec3d_copy(v, getGraph()->bodies[body->id].x_dot);
}

/*******************************************************************************
 *
 ******************************************************************************/
void KineticSimulation::getAngularVelocity(const RcsBody* body,
                                           double omega[3]) const
{
  Vec3d_copy(omega, getGraph()->bodies[body->id].omega);
}

/*******************************************************************************
 *
 ******************************************************************************/
void KineticSimulation::setJointTorque(const MatNd* T)
{
  MatNd_copy(this->T_des, T);
}

/*******************************************************************************
 *
 ******************************************************************************/
void KineticSimulation::getJointTorque(MatNd* T_curr, RcsStateType type) const
{
  if (type==RcsStateFull)
  {
    MatNd_reshapeCopy(T_curr, this->jointTorque);
  }
  else
  {
    RcsGraph_stateVectorToIK(getGraph(), this->jointTorque, T_curr);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void KineticSimulation::getJointAngles(MatNd* q, RcsStateType type) const
{
  if (type == RcsStateFull)
  {
    MatNd_reshapeCopy(q, getGraph()->q);
  }
  else
  {
    RcsGraph_stateVectorToIK(getGraph(), getGraph()->q, q);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void KineticSimulation::getJointVelocities(MatNd* q_dot,
                                           RcsStateType type) const
{
  if (type == RcsStateFull)
  {
    MatNd_reshapeCopy(q_dot, getGraph()->q_dot);
  }
  else
  {
    RcsGraph_stateVectorToIK(getGraph(), getGraph()->q_dot, q_dot);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void KineticSimulation::setMassAndInertiaFromPhysics(RcsGraph* graph)
{
  // Nothing to do here
}

/*******************************************************************************
 *
 ******************************************************************************/
void KineticSimulation::getPhysicsTransform(HTr* A_BI,
                                            const RcsBody* body) const
{
  const RcsBody* pBdy = RCSBODY_BY_ID(getGraph(), body->id);
  RCHECK(pBdy);
  HTr_copy(A_BI, &pBdy->A_BI);
}

/*******************************************************************************
 *
 ******************************************************************************/
const HTr* KineticSimulation::getPhysicsTransformPtr(const RcsBody* body) const
{
  const RcsBody* pBdy = RCSBODY_BY_ID(getGraph(), body->id);
  return pBdy ? &pBdy->A_BI : NULL;
}

/*******************************************************************************
 *
 ******************************************************************************/
void KineticSimulation::disableCollision(const RcsBody* b0, const RcsBody* b1)
{
  RFATAL("FIXME");
}

/*******************************************************************************
 *
 ******************************************************************************/
PhysicsBase::Contacts KineticSimulation::getContacts()
{
  Contacts contacts;

  for (size_t i = 0; i < contact.size(); ++i)
  {
    Contact c;
    const double* xi = MatNd_getRowPtr(contactPositions, i);
    Vec3d_copy(c.pos, xi);
    const double* fi = MatNd_getRowPtr(contactForces, i);
    Vec3d_copy(c.force, fi);
    contacts.push_back(c);
  }

  return contacts;
}

/*******************************************************************************
 *
 ******************************************************************************/
void KineticSimulation::setJointLimits(bool enable)
{
  RFATAL("FIXME");
}

/*******************************************************************************
 *
 ******************************************************************************/
KineticSimulation* KineticSimulation::clone(RcsGraph* newGraph) const
{
  return new KineticSimulation(*this, newGraph);
}

/*******************************************************************************
 *
 ******************************************************************************/
void KineticSimulation::setJointCompliance(const MatNd* stiffness,
                                           const MatNd* damping)
{
  RFATAL("FIXME");
}

/*******************************************************************************
 *
 ******************************************************************************/
void KineticSimulation::getJointCompliance(MatNd* stiffness,
                                           MatNd* damping) const
{
  RFATAL("FIXME");
}

/*******************************************************************************
 *
 ******************************************************************************/
bool KineticSimulation::addBody(const RcsGraph* graph, const RcsBody* body_)
{
  RLOG(5, "Creating body for \"%s\"", body_->name);
  RcsBody* body = RcsGraph_insertGraphBody(getGraph(), body_->parentId);
  RcsBody_copy(body, body_);

  // Make a copy of all body shapes and attach them to the body
  body->nShapes = RcsBody_numShapes(body_);
  body->shapes = RREALLOC(body->shapes, body->nShapes+1, RcsShape);
  RCHECK(body->shapes);
  for (unsigned int i = 0; i < body->nShapes; i++)
  {
    RcsShape_copy(&body->shapes[i], &body_->shapes[i]);
  }

  // Create the joints into the simulation's body.
  RCSBODY_FOREACH_JOINT(graph, body_)
  {
    RcsJoint* newJnt = RcsGraph_insertGraphJoint(getGraph(), body->id);
    RcsJoint_copy(newJnt, JNT);
  }

  MatNd* arrBuf[4];
  arrBuf[0] = this->q_des;
  arrBuf[1] = this->q_dot_des;
  arrBuf[2] = this->T_des;
  arrBuf[3] = this->draggerTorque;

  unsigned int nJoints = RcsBody_numJoints(getGraph(), body);

  if (nJoints > 0)
  {
    MatNd_realloc(this->T_des, getGraph()->dof, 1);
    MatNd_realloc(this->q_des, getGraph()->dof, 1);
    MatNd_realloc(this->q_dot_des, getGraph()->dof, 1);
    MatNd_realloc(this->draggerTorque, getGraph()->dof, 1);
  }

  // If a floating base body is added, we remove its constraints so that the
  // dof go into the EoM.
  if (body->rigid_body_joints)
  {
    RCSBODY_FOREACH_JOINT(getGraph(), body)
    {
      JNT->constrained = false;
    }
  }


  RcsGraph_addBodyDofs(getGraph(), NULL, body, arrBuf, 4);
  this->draggerTorque->m = getGraph()->nJ;

  RCSBODY_TRAVERSE_SHAPES(body)
  {
    int shapeIdx = 0;
    if (RcsShape_isOfComputeType(SHAPE, RCSSHAPE_COMPUTE_CONTACT))
    {
      // Here we have to initialize the contact point in the contactPositions
      // array. That's a bit hard with the indexing. If somebody needs it, it
      // can be done.
      RFATAL("Currently can't handle adding new bodies with contact points");
      contact.push_back(FrictionContactPoint(body->id, shapeIdx));
    }
    shapeIdx++;
  }

  applyLinearVelocity(body, body->x_dot);
  applyAngularVelocity(body, body->omega);

  RLOG(5, "SUCCESS adding \"%s\" to simulation", body->name);

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool KineticSimulation::setParameter(ParameterCategory category,
                                     const char* name, const char* type,
                                     double value)
{
  switch (category)
  {
    case Simulation:
      if (STREQ(type, "Integrator"))
      {
        this->integrator = std::string(name);
        RLOG_CPP(1, "Changed integrator to " << this->integrator);
      }
      break;

    default:
      break;
  }

  return true;
}

/*******************************************************************************
 * This is almost the same as in PhysicsBase except that we don't compute the
 * forward kinematics based on the desired joint values. This simulator is a
 * bit different in the sense that there is no underlying physics engine and we
 * operate on the internal graph directly.
 ******************************************************************************/
void KineticSimulation::setControlInput(const MatNd* q_des_,
                                        const MatNd* q_dot_des_,
                                        const MatNd* T_des_)
{
  // Desired joint angles
  if (q_des_ != NULL)
  {
    RCHECK_EQ(q_des_->n, 1);

    if (q_des_->m == getGraph()->dof)
    {
      MatNd_copy(this->q_des, q_des_);
    }
    else if (q_des_->m == getGraph()->nJ)
    {
      RcsGraph_stateVectorFromIK(getGraph(), q_des_, this->q_des);
    }
    else
    {
      RFATAL("Dimension mismatch: q_des_->m=%d, but dof=%d and nJ=%d",
             q_des_->m, getGraph()->dof, getGraph()->nJ);
    }

  }

  // Desired joint velocities
  if (q_dot_des_ != NULL)
  {
    RCHECK_EQ(q_dot_des_->n, 1);

    if (q_dot_des_->m == getGraph()->dof)
    {
      MatNd_copy(this->q_dot_des, q_dot_des_);
    }
    else if (q_dot_des_->m == getGraph()->nJ)
    {
      RcsGraph_stateVectorFromIK(getGraph(), q_dot_des_, this->q_dot_des);
    }
    else
    {
      RFATAL("Dimension mismatch: q_dot_des_->m=%d, but dof=%d and nJ=%d",
             q_dot_des_->m, getGraph()->dof, getGraph()->nJ);
    }
  }

  // Desired joint torques
  if (T_des_ != NULL)
  {
    RCHECK_EQ(T_des_->n, 1);

    if (T_des_->m == getGraph()->dof)
    {
      MatNd_copy(this->T_des, T_des_);
    }
    else if (T_des_->m == getGraph()->nJ)
    {
      RcsGraph_stateVectorFromIK(getGraph(), T_des_, this->T_des);
    }
    else
    {
      RFATAL("Dimension mismatch: T_des_->m=%d, but dof=%d and nJ=%d",
             T_des_->m, getGraph()->dof, getGraph()->nJ);
    }
  }
}

/*******************************************************************************
 *
 * Wrapper function of the direct dynamics to match the function
 * signature of the integrator. Argument param is assumed to point to a
 * KineticSimulation instance. We set up the state vector as
 *
 * x = (q^T qp^T x_c^T)^T
 *
 * The contact dynamics part x_c is ignored. This function only models a
 * minimum, it ignores contact and spring forces, joint speed damping and
 * PID controls for position / velocity controlled joints.
 *
 ******************************************************************************/
double KineticSimulation::integrationStepSimple(const double* x_, void* param,
                                                double* xp, double dt)
{
  KineticSimulation* kSim = (KineticSimulation*)param;
  MatNd* F_external = MatNd_create(kSim->getGraph()->nJ, 1);
  DirDynParams simpleParams;
  simpleParams.graph = kSim->getGraph();
  simpleParams.F_ext = F_external;

  // Add joint torque
  MatNd* T_des_ik = MatNd_create(kSim->getGraph()->nJ, 1);
  RcsGraph_stateVectorToIK(kSim->getGraph(), kSim->T_des, T_des_ik);
  MatNd_addSelf(F_external, T_des_ik);
  MatNd_destroy(T_des_ik);

  // Subtract torque from mouse dragger.
  MatNd_subSelf(F_external, kSim->draggerTorque);

  kSim->energy = Rcs_directDynamicsIntegrationStep(x_, &simpleParams, xp, dt);
  MatNd_destroy(F_external);

  return kSim->energy;
}

/*******************************************************************************
 *
 * Wrapper function of the direct dynamics to match the function
 * signature of the integrator. Argument param is assumed to point to a
 * KineticSimulation instance.
 *
 * We augment this here a bit to account for the frictional forces of
 * the ground contacts. When exceeding the friction cone limit, the
 * contact points have a velocity that needs to be integrated consistently
 * with the system. Therefore, we set up the state vector as
 *
 * x = (q^T qp^T x_c^T)^T
 *
 * In this function, we compute the state vector derivative, which is
 *
 * xp = (qp^T qpp^T xp_c^T)^T
 *
 * We do the following updates:
 *
 * qp is the qp from the last time step
 * qpp comes out of the direct dynamics
 * xp_c comes out of the contact point velocity computations
 *
 ******************************************************************************/
double KineticSimulation::integrationStep(const double* x_, void* param,
                                          double* xp, double dt)
{
  KineticSimulation* kSim = (KineticSimulation*)param;
  RcsGraph* graph = kSim->getGraph();
  const int nq = graph->nJ, nc = kSim->contact.size(), nz = 2*nq+3*nc;

  // Allow calling the function with x and xp pointing to the same memory
  double* x = VecNd_clone(x_, nz);

  // Update kinematics
  MatNd q = MatNd_fromPtr(nq, 1, (double*)&x[0]);
  MatNd qp = MatNd_fromPtr(nq, 1, (double*)&x[nq]);
  RcsGraph_setState(graph, &q, &qp);

  // Compute the contact and spring forces and friction contact velocities. We
  // need dt here to consistently compute the contact point velocities.
  MatNd* F_external = MatNd_create(nq, 1);
  MatNd xp_c = MatNd_fromPtr(nc, 3, &xp[2*nq]);
  kSim->addContactForces(F_external, &xp_c, kSim->contactForces, x);
  MatNd_constMulSelf(&xp_c, 1.0/dt);   // xp^T = [ ... ...  xp_c^T ]^T
  kSim->addSpringForces(F_external, &qp);

  // Add joint torque
  MatNd* T_des_ik = MatNd_create(nq, 1);
  RcsGraph_stateVectorToIK(graph, kSim->T_des, T_des_ik);
  MatNd_addSelf(F_external, T_des_ik);
  MatNd_destroy(T_des_ik);

  // Subtract torque from mouse dragger. The dragger torque comes from the
  // applyForce() function, called from the ForceDragger of the PhysicsNode.
  MatNd_subSelf(F_external, kSim->draggerTorque);

  // Compute accelerations using direct dynamics. Here is how we assemble the
  VecNd_copy(&xp[0], qp.ele, nq);   // xp^T = [ qp^T ... ... ]^T
  MatNd qpp = MatNd_fromPtr(nq, 1, &xp[nq]);
  MatNd* b = MatNd_create(nq, 1);
  kSim->energy = kSim->dirdyn(graph, F_external, &qpp, b);  // xp^T = [ ... qpp^T ... ]^T
  RcsGraph_stateVectorFromIK(graph, b, kSim->jointTorque); // Memorize torques.
  MatNd_destroy(b);

  // Check direct dynamics outputs
  MatNd xpArr = MatNd_fromPtr(nz, 1, xp);
  if (!MatNd_isFinite(&xpArr))
  {
    RLOG(1, "Found non-finite elements in xp - setting to zero");

    REXEC(2)
    {
      MatNd_printCommentDigits("qp", &qp, 3);
      MatNd_printCommentDigits("qpp", &qpp, 3);
      MatNd_printCommentDigits("xp_c", &xp_c, 3);
    }

    MatNd_setZero(&xpArr);
  }

  // Cleanup
  MatNd_destroy(F_external);
  RFREE(x);

  return kSim->energy;
}

/*******************************************************************************
 *
 *   Solves the multibody equations of motion for the joint
 *   accelerations:
 *
 *   M(q) q_ddot + h(q,q_dot) + F_gravity + F_ext = F_jnt
 *
 *   The signs follow the equations of the Springer handbook of robotics,
 *   but not the variable names.
 *
 *   q, q_dot, q_ddot: Generalized coordinates, velocities and accelerations
 *               The dimension is graph->nJ (number of unconstrained dof)
 *   M:          Mass and inertia matrix
 *   F_gravity:  Gravity force projected on generalized coordinates
 *   h:          Coriolis vector
 *   F_ext:      External forces projected on generalized coordinates
 *   F_jnt:      Joint torque vector
 *
 *   Currently F_jnt is projected on all unconstrained degrees of
 *   freedom. Arrays F_ext and F_jnt can be pointers to NULL. In this
 *   case, they are assumed to be zero.
 *
 *   The function returns the overall energy of the system. It will
 *   warn on debug level 1 if
 *   - the Cholesky decomposition failed
 *   - one of the elements of q_ddot is not finite
 *
 ******************************************************************************/
static void getSubVec(MatNd* dst, const MatNd* src, const std::vector<int>& idx)
{
  MatNd_reshape(dst, idx.size(), src->n);

  // Fast track for column vector
  if (src->n == 1)
  {
  for (size_t i = 0; i < idx.size(); ++i)
  {
    RCHECK(idx[i]<(int)src->m);
    dst->ele[i] = src->ele[idx[i]];
  }
}
  else
  {
    // Handle matrix interpreted as side-by-side column vectors
    for (size_t i = 0; i < idx.size(); ++i)
    {
      RCHECK(idx[i] < (int)src->m);
      double* rowDst = MatNd_getRowPtr(dst, i);
      const double* rowSrc = MatNd_getRowPtr(src, idx[i]);
      VecNd_copy(rowDst, rowSrc, src->n);
    }

  }
}

static void getSubVec(MatNd* dst, const std::vector<int>& idx)
{
  MatNd* src;
  MatNd_clone2(src, dst);
  getSubVec(dst, src, idx);
  MatNd_destroy(src);
}

static void getSubMat(MatNd* dst, const MatNd* src,
                      const std::vector<int>& rowIdx,
                      const std::vector<int>& colIdx)
{
  MatNd_reshape(dst, rowIdx.size(), colIdx.size());

  for (size_t i = 0; i < rowIdx.size(); ++i)
  {
    MatNd srcRow = MatNd_getRowViewTranspose(src, rowIdx[i]);
    MatNd dstRow = MatNd_getRowViewTranspose(dst, i);
    getSubVec(&dstRow, &srcRow, colIdx);
  }
}

static void getSubMat(MatNd* dst, const MatNd* src, const std::vector<int>& idx)
{
  RCHECK(src->m == src->n);
  getSubMat(dst, src, idx, idx);
}

double KineticSimulation::dirdyn(const RcsGraph* graph,
                                 const MatNd* F_ext,
                                 MatNd* q_ddot,
                                 MatNd* b) const
{
  int n = graph->nJ;
  MatNd* M = MatNd_create(n, n);
  MatNd* F_gravity = MatNd_create(n, 1);
  MatNd* h = MatNd_create(n, 1);

  // Compute mass matrix, h-vector and gravity forces
  double E = RcsGraph_computeKineticTerms(graph, gravity, M, h, F_gravity);

  // Joint speed damping: M Kv(qp_des - qp) with qp_des = 0
  // We consider all bodies with six joints as floating and do not apply any
  // damping. \todo: Provide interface on per-joint basis
  const double damping = 2.0;
  MatNd* Fi = MatNd_create(n, 1);
  MatNd* qp_ik = MatNd_clone(graph->q_dot);

  RCSGRAPH_TRAVERSE_BODIES(graph)
  {
    if (RcsBody_numJoints(graph, BODY)==6)
    {
      const RcsJoint* ji = RCSJOINT_BY_ID(graph, BODY->jntId);
      VecNd_setZero(&qp_ik->ele[ji->jointIndex], 6);
    }
  }

  RcsGraph_stateVectorToIKSelf(graph, qp_ik);
  MatNd_mul(Fi, M, qp_ik);
  MatNd_constMulAndAddSelf(b, Fi, -damping);
  MatNd_destroyN(2, qp_ik, Fi);

  // Assemble RHS generalized forces
  MatNd_addSelf(b, F_gravity);
  MatNd_addSelf(b, F_ext);
  MatNd_addSelf(b, h);

  // Enforce kinematic constraints. This must be called after vector b has been
  // completely assembled with all external force components.
  addConstraintForces(b, M, b);

  /*
    Matrix decomposition into torque and position controlled partitions. We
    decompose the standard EoM

    M qdd - h = F

    into

    / M00   M01 \  / qpp_f \     / h_f \      / F_f \
    |           |  |       |  =  |     |  +   |     |
    \ M10   M11 /  \ qpp_c /     \ h_c /      \ F_c /

    We then solve for the free accelerations under consideration of the
    constrained ones:

    qdd_f = inv(M00) (h_f + F_f - M01 qpp_c)

    We can obtain the joint torques like that:

    F_c = M10 qpp_f + M11 qpp_c - hc
  */

  // pIdx are all indices of position and velocity controlled dofs, tIdx are the
  // dofs that are controlled by forces / torques.
  std::vector<int> pIdx, tIdx;
  RCSGRAPH_TRAVERSE_JOINTS(graph)
  {
    if (JNT->constrained)
    {
      continue;
    }

    switch (JNT->ctrlType)
    {
      case RCSJOINT_CTRL_POSITION:
      case RCSJOINT_CTRL_VELOCITY:
        pIdx.push_back(JNT->jacobiIndex);
        break;

      case RCSJOINT_CTRL_TORQUE:
        tIdx.push_back(JNT->jacobiIndex);
        break;

      default:
        RFATAL("Unknown joint type %d", JNT->ctrlType);
    }

  }

  MatNd* M00 = MatNd_create(tIdx.size(), tIdx.size());
  MatNd* M01 = MatNd_create(tIdx.size(), pIdx.size());

  getSubMat(M00, M, tIdx);
  getSubMat(M01, M, tIdx, pIdx);

  MatNd* b_f = MatNd_clone(b);
  MatNd* qpp_f = MatNd_clone(q_ddot);
  MatNd* qpp_c = MatNd_clone(q_ddot);
  getSubVec(qpp_f, tIdx);

  // Here we need to compute the desired accelerations from the current state
  // and the desired position, and project it onto the constrained elements.
  RCSGRAPH_TRAVERSE_JOINTS(graph)
  {
    if (JNT->jacobiIndex == -1)
    {
      continue;
    }

    // \todo: Here we could feed-forward the accelerations similar as for the
    // velocity-controlled joints, and do the PD on top of it.
    if (JNT->ctrlType == RCSJOINT_CTRL_POSITION)
    {
      double q_des_i = this->q_des->ele[JNT->jointIndex];
      double q_curr_i = graph->q->ele[JNT->jointIndex];
      double qp_curr_i = graph->q_dot->ele[JNT->jointIndex];
      // double qpp_i = 2.0 * (q_des_i - q_curr_i - qp_curr_i*dt) / (dt*dt);
      // tmp->ele[JNT->jacobiIndex] = 0.01*qpp_i;

      // aq = -kp*(q-q_des)  -kd*qp_curr
      const double kp = 100.0;
      const double kd = 0.5*sqrt(4.0*kp);
      qpp_c->ele[JNT->jacobiIndex] = -kp*(q_curr_i- q_des_i) - kd*qp_curr_i;
    }
    // Forward integration of accelerations. This is subject to drift, however
    // it might be visible only after longer simulation periods.
    else if (JNT->ctrlType == RCSJOINT_CTRL_VELOCITY)
    {
      double qp_curr_i = graph->q_dot->ele[JNT->jointIndex];
      double qp_des_i = this->q_dot_des->ele[JNT->jointIndex];
      qpp_c->ele[JNT->jacobiIndex] = (qp_des_i - qp_curr_i)/lastDt;
    }
  }


  getSubVec(qpp_c, pIdx);
  getSubVec(b_f, tIdx);
  MatNd* Mqpp_c = MatNd_create(tIdx.size(), 1);
  MatNd_mul(Mqpp_c, M01, qpp_c);
  MatNd_subSelf(b_f, Mqpp_c);
  MatNd* q_ddot_f = MatNd_createLike(b_f);

  // Fixed kinematic constraints in the reduced space
#if 0
  {
    MatNd* J = MatNd_create(0, graph->nJ);
    MatNd* J_dot = MatNd_create(0, graph->nJ);
    MatNd* ax = MatNd_create(0, 1);
    MatNd* qp_ik = MatNd_clone(graph->q_dot);
    RcsGraph_stateVectorToIKSelf(graph, qp_ik);

    // Compute an augmented overall Jacobian, dot Jacobian and error compensation
    for (size_t i = 0; i < constraint.size(); ++i)
    {
      constraint[i].appendJacobian(J, graph);
      constraint[i].appendDotJacobian(J_dot, graph, qp_ik);
      constraint[i].appendStabilization(ax, graph);
    }

    // Project terms into free space
    RLOG(1, "A");
    MatNd_transposeSelf(J);
    getSubVec(J, tIdx);
    MatNd_transposeSelf(J);
    MatNd_transposeSelf(J_dot);
    getSubVec(J_dot, tIdx);
    MatNd_transposeSelf(J_dot);
    getSubVec(ax, tIdx);
    getSubVec(qp_ik, tIdx);
    MatNd_printDims("J", J);

    // J inv(M)
    RLOG(1, "B");
    MatNd* invM00 = MatNd_createLike(M00);
    double det = MatNd_choleskyInverse(invM00, M00);
    RCHECK(det);
    MatNd* JinvM = MatNd_create(J->m, invM00->n);
    MatNd_mul(JinvM, J, invM00);
    MatNd_printDims("JinvM", JinvM);

    // inv(J inv(M) JT)
    RLOG(1, "C");
    MatNd* invJinvMJT = MatNd_create(J->m, J->m);
    MatNd_sqrMulABAt(invJinvMJT, J, invM00);
    int rank = MatNd_SVDInverse(invJinvMJT, invJinvMJT);
    RLOG(1, "Rank is %d", rank);

    // lambda = -J inv(M00) b_f - J_dot q_dot_f
    RLOG(1, "D");
    MatNd* lambda = MatNd_create(JinvM->m, 1);
    MatNd_mul(lambda, JinvM, b_f);
    MatNd_mulAndAddSelf(lambda, J_dot, qp_ik);
    MatNd_constMulSelf(lambda, -1.0);

    // Add stabilization: ax = kp*dx + kd*dx_dot
    RLOG(1, "E");
    MatNd_addSelf(lambda, ax);

    // Lagrange Multipliers: lambda = inv(J inv(M) JT) (-J inv(M) b - J_dot q_dot)
    RLOG(1, "F");
    MatNd_preMulSelf(lambda, invJinvMJT);

    // Project into joint space: lambdaQ = JT lambda : n x 1 = [n x nc] * [nx x 1]
    // or in transpose space: lambdaQ^T = lambda^T J : 1 x n = [1 x nc] * [nc x n]
    RLOG(1, "G");
    MatNd* lambda_f = MatNd_create(1, graph->nJ);
    MatNd_transposeSelf(lambda);   // now 1 x nc
    MatNd_mulAndAddSelf(lambda_f, lambda, J);
    MatNd_transposeSelf(lambda_f);   // now n x 1


    // Clean up
    RLOG(1, "H");
    MatNd_destroyN(7, J, J_dot, ax, qp_ik, invM00, invJinvMJT, lambda_f);
  }
#endif
  double det = MatNd_choleskySolve(q_ddot_f, M00, b_f);

  MatNd_reshape(q_ddot, n, 1);
  for (size_t i = 0; i < tIdx.size(); ++i)
  {
    q_ddot->ele[tIdx[i]] = q_ddot_f->ele[i];
  }
  for (size_t i = 0; i < pIdx.size(); ++i)
  {
    q_ddot->ele[pIdx[i]] = qpp_c->ele[i];
  }

#if 0
  // Get joint torques: F_c = M10 qpp_f + M11 qpp_c - hc
  {
    MatNd_transposeSelf(M01);   // now M10
    MatNd* jointTorque = MatNd_createLike(qpp_c);
    MatNd* M11 = MatNd_create(pIdx.size(), pIdx.size());
    MatNd* b_c = MatNd_createLike(qpp_c);


    getSubMat(M11, M, pIdx);
    getSubVec(b_c, b, pIdx);

    MatNd_mul(jointTorque, M01, qpp_f);
    MatNd_mulAndAddSelf(jointTorque, M11, qpp_c);
    MatNd_subSelf(jointTorque, b_c);
    MatNd_printCommentDigits("torque", jointTorque, 4);

    MatNd_destroy(M11);
    MatNd_destroy(b_c);
    MatNd_destroy(jointTorque);
  }
#endif

  // Clean up
  MatNd_destroyN(10, M00, M01, qpp_f, qpp_c, Mqpp_c, q_ddot_f, b_f, M,
                 F_gravity, h);

  // Warn if something seriously goes wrong.
  if (det == 0.0)
  {
    RLOG(1, "Couldn't solve direct dynamics - determinant is 0");
    E = -1.0;
  }

  return E;
}

/*******************************************************************************
 *
 ******************************************************************************/
double KineticSimulation::getEnergy() const
{
  return this->energy;
}

/*******************************************************************************
 *
 ******************************************************************************/
std::string KineticSimulation::getIntegrator() const
{
  return this->integrator;
}

/*******************************************************************************
 *
 ******************************************************************************/
double KineticSimulation::getAdaptedDt() const
{
  return this->dt_opt;
}

/*******************************************************************************
 *
 ******************************************************************************/
const double* KineticSimulation::getContactPositionPtr(size_t i) const
{
  RCHECK(i<contactPositions->m);
  return MatNd_getRowPtr(this->contactPositions, i);
}

/*******************************************************************************
 *
 ******************************************************************************/
size_t KineticSimulation::getNumContacts() const
{
  return this->contactPositions->m;
}

/*******************************************************************************
 *
 ******************************************************************************/
void KineticSimulation::addContactForces(MatNd* M_contact,      // nq x 1
                                         MatNd* dx_contact,     // nc x 3
                                         MatNd* f_contact,      // nc x 3
                                         const double* x) const // 2*nq+3*nc
{
  const int nq = getGraph()->nJ;
  MatNd qp = MatNd_fromPtr(nq, 1, (double*)&x[nq]);
  MatNd x_c = MatNd_fromPtr(contact.size(), 3, (double*)&x[2*nq]);
  MatNd* J = MatNd_create(3, nq);

  // Compute the velocities of the attachement points
  for (size_t i = 0; i < contact.size(); i++)
  {
    const RcsBody* cBdy = &getGraph()->bodies[contact[i].bdyId];
    const RcsShape* cSh = &cBdy->shapes[contact[i].shapeIdx];
    RcsGraph_bodyPointJacobian(getGraph(), cBdy, cSh->A_CB.org, NULL, J);

    // Compute the attachement point velocities: I_xp_c = I_xp_b + I_om x I_r_c
    double xp_attach_i[3], omxr[3], I_r_cp[3];
    Vec3d_transRotate(I_r_cp, (double (*)[3])cBdy->A_BI.rot, cSh->A_CB.org);
    Vec3d_crossProduct(omxr, cBdy->omega, I_r_cp);
    Vec3d_add(xp_attach_i, cBdy->x_dot, omxr);

    // Compute the attachement point velocities: I_xp_c = J_cp qp
    // double xp_attach_i[3];
    // MatNd xp_a_i = MatNd_fromPtr(3, 1, xp_attach_i);
    // MatNd_mul(&xp_a_i, J, &qp);

    // Test: Compute the attachement point velocities using the Jacobian.
    REXEC(1)
    {
      double xp_test[3];
      MatNd xp_a_i = MatNd_fromPtr(3, 1, xp_test);
      MatNd_mul(&xp_a_i, J, &qp);
      double err = Vec3d_distance(xp_attach_i, xp_a_i.ele);

      if (err>1.0e-8)
      {
        RLOG(0, "Attachement error[%s]: %f (%f = %f)", cBdy->name, err,
             Vec3d_getLength(xp_attach_i), Vec3d_getLength(xp_a_i.ele));
      }
    }

    // Compute friction contact speeds
    double* dx_contact_i = &dx_contact->ele[3*i];
    const double* x_contact_i = &x_c.ele[3*i];
    HTr A_CI;
    HTr_transform(&A_CI, &cBdy->A_BI, &cSh->A_CB);

    double* fi = MatNd_getRowPtr(f_contact, i);
    const double* x_attach_i = A_CI.org;
    contact[i].computeContacts(dx_contact_i, fi,
                               x_contact_i, x_attach_i, xp_attach_i);

    // Project contact forces into joint space: M_c = J^T * F_i
    // We save the transpose and do: M_c^T = F_i^T*J
    MatNd F_i = MatNd_fromPtr(1, 3, fi);
    MatNd_reshape(M_contact, 1, nq);
    MatNd_mulAndAddSelf(M_contact, &F_i, J);
    MatNd_reshape(M_contact, nq, 1);
  }

  MatNd_destroy(J);
}

/*******************************************************************************
 *
 ******************************************************************************/
void KineticSimulation::addSpringForces(MatNd* M_spring, const MatNd* q_dot) const
{
  MatNd* J = MatNd_create(3, getGraph()->nJ);

  RCSGRAPH_FOREACH_BODY(getGraph())
  {
    RCSBODY_TRAVERSE_SHAPES(BODY)
    {
      if (!RcsShape_isOfComputeType(SHAPE, RCSSHAPE_COMPUTE_ATTACHMENT))
      {
        continue;
      }

      const RcsBody* anchor = RcsGraph_getBodyByName(getGraph(), SHAPE->meshFile);
      RCHECK_MSG(anchor, "Not found: \"%s\"", SHAPE->meshFile);
      const double* x_anchor = anchor->A_BI.org;
      const double* xp_anchor = anchor->x_dot;

      // Compute the velocities of the spring point
      HTr A_CI;
      HTr_transform(&A_CI, &BODY->A_BI, &SHAPE->A_CB);
      const double* x_spring = A_CI.org;
      double xp_spring_[3];
      MatNd xp_spring = MatNd_fromPtr(3, 1, xp_spring_);
      RcsGraph_bodyPointJacobian(getGraph(), BODY, SHAPE->A_CB.org, NULL, J);
      MatNd_mul(&xp_spring, J, q_dot);

      // Compute spring force
      double f[3];
      const double stiffness = SHAPE->scale3d[0];
      const double damping = SHAPE->scale3d[1];
      for (int i = 0; i < 3; ++i)
      {
        f[i] = stiffness*(x_anchor[i]-x_spring[i]) +
               damping*(xp_anchor[i]-xp_spring.ele[i]);
      }

      // Project contact forces into joint space
      MatNd F_i = MatNd_fromPtr(3, 1, f);
      MatNd_transposeSelf(J);
      MatNd_mulAndAddSelf(M_spring, J, &F_i);
    }
  }

  MatNd_destroy(J);
}

/*******************************************************************************
 * M_constraint += J^T inv(J inv(M) J^T) (-J inv(M) b - J_dot q_dot)
 ******************************************************************************/
void KineticSimulation::addConstraintForces(MatNd* M_constraint,
                                            const MatNd* M,
                                            const MatNd* b) const
{
  const RcsGraph* graph = getGraph();

  MatNd* J = MatNd_create(0, graph->nJ);
  MatNd* J_dot = MatNd_create(0, graph->nJ);
  MatNd* ax = MatNd_create(0, 1);
  MatNd* qp_ik = MatNd_clone(graph->q_dot);
  RcsGraph_stateVectorToIKSelf(graph, qp_ik);

  // Compute an augmented overall Jacobian, dot Jacobian and error compensation
  for (size_t i=0; i<constraint.size(); ++i)
  {
    constraint[i].appendJacobian(J, graph);
    constraint[i].appendDotJacobian(J_dot, graph, qp_ik);
    constraint[i].appendStabilization(ax, graph);
  }

  // J inv(M)
  MatNd* invM = MatNd_createLike(M);
  double det = MatNd_choleskyInverse(invM, M);
  if (det == 0.0)
  {
    RLOG(1, "Failed to invert mass matrix");
  }
  MatNd* JinvM = MatNd_create(J->m, graph->nJ);
  MatNd_mul(JinvM, J, invM);

  // inv(J inv(M) JT)
  MatNd* invJinvMJT = MatNd_create(J->m, J->m);
  MatNd_sqrMulABAt(invJinvMJT, J, invM);
  det = MatNd_choleskyInverse(invJinvMJT, invJinvMJT);
  if (det == 0.0)
  {
    MatNd_sqrMulABAt(invJinvMJT, J, invM);
    int rank = MatNd_SVDInverseSelf(invJinvMJT);
    RLOG(1, "Using SVD to invert J inv(M) transpose(J): rank is %d", rank);
  }

  // lambda = -J inv(M) b - J_dot q_dot
  MatNd* lambda = MatNd_create(J->m, 1);
  MatNd_mul(lambda, JinvM, b);
  MatNd_mulAndAddSelf(lambda, J_dot, qp_ik);
  MatNd_constMulSelf(lambda, -1.0);

  // Add stabilization: ax = kp*dx + kd*dx_dot
  MatNd_addSelf(lambda, ax);

  // Lagrange Multipliers: lambda = inv(J inv(M) JT) (-J inv(M) b - J_dot q_dot)
  MatNd_preMulSelf(lambda, invJinvMJT);

  // Project into joint space: lambdaQ = JT lambda : n x 1 = [n x nc] * [nx x 1]
  // or in transpose space: lambdaQ^T = lambda^T J : 1 x n = [1 x nc] * [nc x n]
  MatNd_reshape(M_constraint, 1, graph->nJ);
  MatNd_transposeSelf(lambda);   // now 1 x nc
  MatNd_mulAndAddSelf(M_constraint, lambda, J);
  MatNd_transposeSelf(M_constraint);   // now n x 1

  // Clean up
  MatNd_destroyN(8, J, J_dot, ax, qp_ik, invM, JinvM, invJinvMJT, lambda);
}





/*******************************************************************************

  We assume a number of things:

  1. The contact plane normal is pointing up (0 0 1), and its height is (0 0 z0)
  2. Since z is pointing up (0 0 1), the vertical component of the contact
   force can only be >=0.0
  3. The contact point is attached to the body point with an asymptotically
   damped second order system and is mass-less
  4. Gains are positive
  5. Everything is represented in world coordinates

*******************************************************************************/
KineticSimulation::FrictionContactPoint::FrictionContactPoint(int bdyId_,
                                                              int shapeIdx_,
                                                              double mu_,
                                                              double k_p_,
                                                              double z0_) :
  bdyId(bdyId_), shapeIdx(shapeIdx_), mu(mu_), k_p(k_p_), k_v(sqrt(4.0*k_p_)),
  z0(z0_)
{
}

void KineticSimulation::FrictionContactPoint::computeContactForce(double f[3],
                                                                  const double x_contact[3],
                                                                  const double x_attach[3],
                                                                  const double xp_attach[3]) const
{
  f[0] = 0.0;
  f[1] = 0.0;
  f[2] = 0.0;

  // No force if contact is above ground
  if (x_attach[2] > this->z0)
  {
    return;
  }

  // Vertical component
  f[2] = this->k_p*(this->z0 - x_attach[2]) - this->k_v*xp_attach[2];

  if (f[2] < 0.0)
  {
    f[2] = 0.0;
    return;
  }

  // Horizontal components
  f[0] = this->k_p*(x_contact[0] - x_attach[0]) - this->k_v*xp_attach[0];
  f[1] = this->k_p*(x_contact[1] - x_attach[1]) - this->k_v*xp_attach[1];

  double ft = sqrt(f[0] * f[0] + f[1] * f[1]);
  double ft_limit = this->mu*f[2];

  if (ft > ft_limit)
  {
    f[0] *= ft_limit / ft;
    f[1] *= ft_limit / ft;
  }
}

void KineticSimulation::FrictionContactPoint::computeContacts(double dx_contact[3],
                                                              double f_contact[3],
                                                              const double x_contact[3],
                                                              const double x_attach[3],
                                                              const double xp_attach[3]) const
{
  // If the body point is not penetrating, the velocity is computed so
  // that x_contact coincides with x_attach. Contact forces are zero.
  if (x_attach[2] > this->z0)
  {
    for (int i = 0; i < 3; ++i)
    {
      dx_contact[i] = x_attach[i] - x_contact[i];
      f_contact[i] = 0.0;
    }
    return;
  }


  // If the body point is penetrating, the velocity is computed so that
  // x_contact[2] lies at the surface.
  dx_contact[2] = this->z0 - x_contact[2];

  // Horizontal component. There are 2 cases:
  // 1. The tangential force is within the friction cone. Then, the tangential
  //    contact point velocities are zero.
  // 2. The tangential force exceeds the friction force. Then, the tangential
  //    contact point velocity is computed so that the contact point ends up
  //    on the friction cone's boundary, in the direction towards the
  //    attachement point
  double ft, ft_limit;

  computeContactForce(f_contact, x_contact, x_attach, xp_attach);
  ft = sqrt(f_contact[0] * f_contact[0] + f_contact[1] * f_contact[1]);
  ft_limit = this->mu*f_contact[2];

  // Tangential forces are inside friction cone - contact points don't move
  if (ft <= ft_limit)
  {
    dx_contact[0] = 0.0;
    dx_contact[1] = 0.0;
  }
  else
  {
    double dx = sqrt(pow(x_contact[0] - x_attach[0], 2) +
                     pow(x_contact[1] - x_attach[1], 2));

    dx -= ft_limit / this->k_p;

    for (int i = 0; i < 2; i++)
    {
      dx_contact[i] = -dx*f_contact[i]/ft;
    }
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
KineticSimulation::KinematicConstraint::KinematicConstraint(ConstraintType type_,
                                                            int bdyId_,
                                                            std::vector<double> x_des_,
                                                            double kp_) :
  type(type_), bdyId(bdyId_), x_des(x_des_), kp(kp_)
{
}

size_t KineticSimulation::KinematicConstraint::dim() const
{
  return x_des.size();
}

void KineticSimulation::KinematicConstraint::print(const RcsGraph* graph) const
{
  std::cout << "Body " << RCSBODY_NAME_BY_ID(graph, bdyId) << ": " << std::endl;

  std::cout << "ConstraintType: ";
  switch (type)
  {
    case Pos:
      std::cout << "Pos";
      break;

    case Ori:
      std::cout << "Ori";
      break;

    case PosAndOri:
      std::cout << "PosAndOri";
      break;

    default:
      std::cout << "Unknown";
  }

  std::cout << std::endl << "dim: " << dim() << std::endl;
  std::cout << "kp: " << kp << std::endl;

  for (size_t i=0; i<x_des.size(); ++i)
  {
    std::cout << "x_des[" << i << "] = " << x_des[i] << std::endl;
  }
}

void KineticSimulation::KinematicConstraint::appendJacobian(MatNd* J_full,
                                                            const RcsGraph* graph) const
{
  const unsigned int nc = dim();   // Number of constraints
  const unsigned int n = graph->nJ;

  const RcsBody* ef = RCSBODY_BY_ID(graph, bdyId);
  RCHECK(ef);

  // Constraint Jacobian
  MatNd* J = MatNd_create(nc, n);
  switch (this->type)
  {
    case Pos:
    {
      MatNd Jx = MatNd_fromPtr(3, n, MatNd_getRowPtr(J, 0));
      RcsGraph_bodyPointJacobian(graph, ef, NULL, NULL, &Jx);
      break;
    }
    case Ori:
    {
      MatNd Jw = MatNd_fromPtr(3, n, MatNd_getRowPtr(J, 0));
      RcsGraph_rotationJacobian(graph, ef, NULL, &Jw);
      break;
    }
    case PosAndOri:
    {
      MatNd Jx = MatNd_fromPtr(3, n, MatNd_getRowPtr(J, 0));
      RcsGraph_bodyPointJacobian(graph, ef, NULL, NULL, &Jx);
      MatNd Jw = MatNd_fromPtr(3, n, MatNd_getRowPtr(J, 3));
      RcsGraph_rotationJacobian(graph, ef, NULL, &Jw);
      break;
    }

    default:
      RFATAL("Unknown constraint type: %d", this->type);
  };

  MatNd_appendRows(J_full, J);
  MatNd_destroy(J);
}

void KineticSimulation::KinematicConstraint::appendDotJacobian(MatNd* J_dot_full,
                                                               const RcsGraph* graph,
                                                               const MatNd* qp_ik) const
{
  const unsigned int nc = dim();   // Number of constraints
  const unsigned int n = graph->nJ;

  const RcsBody* ef = RCSBODY_BY_ID(graph, bdyId);
  RCHECK(ef);

  // Constraint dot-Jacobian
  MatNd* J_dot = MatNd_create(nc, n);
  switch (this->type)
  {
    case Pos:
    {
      MatNd J_dot_x = MatNd_fromPtr(3, n, MatNd_getRowPtr(J_dot, 0));
      RcsGraph_bodyPointDotJacobian(graph, ef, NULL, NULL, qp_ik, &J_dot_x);
      break;
    }
    case Ori:
    {
      MatNd J_dot_w = MatNd_fromPtr(3, n, MatNd_getRowPtr(J_dot, 0));
      RcsGraph_rotationDotJacobian(graph, ef, qp_ik, &J_dot_w);
      break;
    }
    case PosAndOri:
    {
      MatNd J_dot_x = MatNd_fromPtr(3, n, MatNd_getRowPtr(J_dot, 0));
      RcsGraph_bodyPointDotJacobian(graph, ef, NULL, NULL, qp_ik, &J_dot_x);
      MatNd J_dot_w = MatNd_fromPtr(3, n, MatNd_getRowPtr(J_dot, 3));
      RcsGraph_rotationDotJacobian(graph, ef, qp_ik, &J_dot_w);
      break;
    }

    default:
      RFATAL("Unknown constraint type: %d", this->type);
  };

  MatNd_appendRows(J_dot_full, J_dot);
  MatNd_destroy(J_dot);
}

void KineticSimulation::KinematicConstraint::appendStabilization(MatNd* ax_full,
                                                                 const RcsGraph* graph) const
{
  const size_t nc = dim();
  MatNd* ax;
  MatNd_fromStack(ax, nc, 1);

  if (kp == 0.0)
  {
    MatNd_appendRows(ax_full, ax);
    return;
  }

  const RcsBody* ef = RCSBODY_BY_ID(graph, bdyId);
  RCHECK(ef);

  const double kd = 0.5*sqrt(4.0*kp);
  double dxPos[3], dxOri[3], dvPos[3], dvOri[3];

  // Compute linear position and velocity error
  if (type==Pos || type==PosAndOri)
  {
    Vec3d_sub(dxPos, x_des.data(), ef->A_BI.org);
    Vec3d_constMul(dvPos, ef->x_dot, -1.0);
  }

  // Compute angular position and velocity error
  if (type==Ori || type==PosAndOri)
  {
    double A_des[3][3];
    size_t offset = (type == Ori) ? 0 : 3;
    Mat3d_fromEulerAngles(A_des, x_des.data()+ offset);
    Mat3d_getEulerError(dxOri, (double(*)[3])ef->A_BI.rot, A_des);

    Vec3d_constMul(dvOri, ef->omega, -1.0);
  }


  MatNd* dx_c, *dxd_c;
  MatNd_fromStack(dx_c, nc, 1);
  MatNd_fromStack(dxd_c, nc, 1);

  switch (type)
  {
    case Pos:
      Vec3d_copy(dx_c->ele, dxPos);
      Vec3d_copy(dxd_c->ele, dvPos);
      break;

    case Ori:
      Vec3d_copy(dx_c->ele, dxOri);
      Vec3d_copy(dxd_c->ele, dvOri);
      break;

    case PosAndOri:
      Vec3d_copy(dx_c->ele, dxPos);
      Vec3d_copy(dxd_c->ele, dvPos);
      Vec3d_copy(dx_c->ele+3, dxOri);
      Vec3d_copy(dxd_c->ele+3, dvOri);
      break;

    default:
      RFATAL("Unknown constraint type: %d", type);
  }

  for (size_t i=0; i<nc; ++i)
  {
    ax->ele[i] = kp*dx_c->ele[i] + kd*dxd_c->ele[i];
  }

  MatNd_appendRows(ax_full, ax);

}

}   // namespace Rcs
