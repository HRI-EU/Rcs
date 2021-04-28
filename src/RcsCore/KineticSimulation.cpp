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


namespace Rcs
{

static const char className[] = "NewtonEuler";
static PhysicsFactoryRegistrar<KineticSimulation> physics(className);

/*******************************************************************************
 * Constructor.
 ******************************************************************************/
KineticSimulation::KineticSimulation() : PhysicsBase(), draggerTorque(NULL),
  jointTorque(NULL), integrator("Fehlberg"), energy(0.0), dt_opt(0.0)
{
}

/*******************************************************************************
 * Constructor.
 ******************************************************************************/
KineticSimulation::KineticSimulation(const RcsGraph* graph_) :
  PhysicsBase(graph_), draggerTorque(NULL), jointTorque(NULL),
  integrator("Fehlberg"), energy(0.0), dt_opt(0.0)
{
  initialize(graph_, NULL);
}

/*******************************************************************************
 * Copy constructor.
 ******************************************************************************/
KineticSimulation::KineticSimulation(const KineticSimulation& copyFromMe) :
  PhysicsBase(copyFromMe), draggerTorque(NULL),
  integrator(copyFromMe.integrator), energy(0.0), dt_opt(0.0)
{
  this->draggerTorque = MatNd_create(getGraph()->nJ, 1);
  MatNd_reshapeCopy(this->draggerTorque, copyFromMe.draggerTorque);
  this->jointTorque = MatNd_clone(copyFromMe.jointTorque);
}

/*******************************************************************************
 * Copy constructor.
 ******************************************************************************/
KineticSimulation::KineticSimulation(const KineticSimulation& copyFromMe,
                                     const RcsGraph* newGraph) :
  PhysicsBase(copyFromMe, newGraph), draggerTorque(NULL),
  integrator(copyFromMe.integrator), energy(copyFromMe.energy),
  dt_opt(copyFromMe.dt_opt)
{
  this->draggerTorque = MatNd_create(getGraph()->nJ, 1);
  MatNd_reshapeCopy(this->draggerTorque, copyFromMe.draggerTorque);
  this->jointTorque = MatNd_clone(copyFromMe.jointTorque);
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
  this->integrator = other.integrator;
  this->energy = other.energy;
  this->dt_opt = other.dt_opt;
  return *this;
}

/*******************************************************************************
 * Destructor.
 ******************************************************************************/
KineticSimulation::~KineticSimulation()
{
  MatNd_destroy(this->draggerTorque);
  MatNd_destroy(this->jointTorque);
}

/*******************************************************************************
 *
 ******************************************************************************/
bool KineticSimulation::initialize(const RcsGraph* g, const PhysicsConfig* cfg)
{
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
        double mu = SHAPE->scale3d[0];
        double stiffness = SHAPE->scale3d[1];
        double z0 = SHAPE->scale3d[2];
        contact.push_back(FrictionContactPoint(BODY, shapeIdx,
                                               mu, stiffness, z0));
      }

      shapeIdx++;
    }
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
  for (int i=0; i<nc; ++i)
  {
    double* row = MatNd_getRowPtr(&zc, i);
    Vec3d_copy(row, contact[i].x_contact);
  }

  MatNd* F_ext = MatNd_create(n, 1);
  RcsGraph_stateVectorToIK(getGraph(), this->T_des, F_ext);
  MatNd_subSelf(F_ext, this->draggerTorque);

  if (integrator == "Fehlberg")
  {
    MatNd* err = MatNd_create(nz, 1);
    MatNd err_q  = MatNd_fromPtr(n, 1, &err->ele[0]);
    MatNd err_qp = MatNd_fromPtr(n, 1, &err->ele[n]);
    MatNd err_xc = MatNd_fromPtr(3*nc, 1, &err->ele[2*n]);

    MatNd_setElementsTo(&err_xc, DBL_MAX);
    MatNd_setElementsTo(&err_q,  1.0e-1);
    MatNd_setElementsTo(&err_qp, 1.0e-2);
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
  for (int i = 0; i < nc; ++i)
  {
    double* row = MatNd_getRowPtr(&zc, i);
    Vec3d_copy(contact[i].x_contact, row);
  }

  // Clean up
  MatNd_destroy(z);
  MatNd_destroy(F_ext);

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
void KineticSimulation::setGravity(const double gravity[3])
{
  RFATAL("FIXME");
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
    Vec3d_copy(c.pos, contact[i].x_contact);
    Vec3d_copy(c.force, contact[i].f_contact);
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
  int nShapes = RcsBody_numShapes(body_);
  body->shape = RNALLOC(nShapes + 1, RcsShape*);
  for (int i = 0; i < nShapes; i++)
  {
    body->shape[i] = RcsShape_clone(body_->shape[i]);
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
      contact.push_back(FrictionContactPoint(body, shapeIdx));
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

  \brief Wrapper function of the direct dynamics to match the function
     signature of the integrator. Argument param is assumed to point to a
     RcsGraph structure. It's userData field is assumed to point to an
     MatNd holding the external forces projected into the configuration
     space.

     We augment this here a bit to account for the frictional forces of
     the ground contacts. When exceeding the friction cone limit, the
     contact points have a velocity that needs to be integrated consistently
     with the system. Therefore, we set up the state vector as

     x = (q^T qp^T x_c^T)^T

     In this function, we compute the state vector derivative, which is

     xp = (qp^T qpp^T xp_c^T)^T

     We do the following updates:

     qp is the qp from the last time step
     qpp comes out of the direct dynamics
     xp_c comes out of the contact point velocity computations

*******************************************************************************/
void KineticSimulation::integrationStep(const double* x, void* param,
                                        double* xp, double dt)
{
  KineticSimulation* kSim = (KineticSimulation*)param;
  RcsGraph* graph = kSim->getGraph();
  const int nq = graph->nJ;
  const int nc = kSim->contact.size();
  MatNd q = MatNd_fromPtr(nq, 1, (double*)&x[0]);
  MatNd qp = MatNd_fromPtr(nq, 1, (double*)&x[nq]);
  MatNd x_c = MatNd_fromPtr(nc, 3, (double*)&x[2*nq]);
  MatNd qpp = MatNd_fromPtr(nq, 1, &xp[nq]);
  MatNd xp_c = MatNd_fromPtr(nc, 3, &xp[2*nq]);

  // Update kinematics
  RcsGraph_setState(graph, &q, &qp);

  // Compute the contact forces and friction contact velocities
  MatNd* J = MatNd_create(3, nq);
  MatNd* M_contact = MatNd_create(nq, 1);

  // Compute the velocities of the attachement points
  for (int i = 0; i < nc; i++)
  {
    const RcsBody* cBdy = &graph->bodies[kSim->contact[i].bdyId];
    const RcsShape* cSh = cBdy->shape[kSim->contact[i].shapeIdx];
    RcsGraph_bodyPointJacobian(graph, cBdy, cSh->A_CB.org, NULL, J);

    // Compute the velocities of the attachement points
    double xp_attach_i[3];
    MatNd xp_a_i = MatNd_fromPtr(3, 1, xp_attach_i);
    MatNd_mul(&xp_a_i, J, &qp);

    // Compute friction contact speeds
    double* xp_contact_i = &xp_c.ele[3 * i];
    const double* x_contact_i = &x_c.ele[3*i];
    HTr A_CI;
    HTr_transform(&A_CI, &cBdy->A_BI, &cSh->A_CB);
    const double* x_attach_i = A_CI.org;

    kSim->contact[i].computeContactSpeed(xp_contact_i, dt, x_contact_i,
                                         x_attach_i, xp_attach_i);
    MatNd F_i = MatNd_fromPtr(3, 1, kSim->contact[i].f_contact);
    kSim->contact[i].computeContactForce(F_i.ele, x_contact_i,
                                         x_attach_i, xp_attach_i);

    // Project contact forces into joint space
    MatNd_transposeSelf(J);
    MatNd_mulAndAddSelf(M_contact, J, &F_i);
  }
  // Spring forces
#if 1
  RCSGRAPH_FOREACH_BODY(graph)
  {
    RCSBODY_TRAVERSE_SHAPES(BODY)
    {
      if (!RcsShape_isOfComputeType(SHAPE, RCSSHAPE_COMPUTE_ATTACHMENT))
      {
        continue;
      }

      const RcsBody* anchor = RcsGraph_getBodyByName(graph, SHAPE->meshFile);
      RCHECK_MSG(anchor, "Not found: \"%s\"", SHAPE->meshFile);
      const double* x_anchor = anchor->A_BI.org;
      const double* xp_anchor = anchor->x_dot;

      // Compute the velocities of the spring point
      HTr A_CI;
      HTr_transform(&A_CI, &BODY->A_BI, &SHAPE->A_CB);
      const double* x_spring = A_CI.org;
      double xp_spring_[3];
      MatNd xp_spring = MatNd_fromPtr(3, 1, xp_spring_);
      RcsGraph_bodyPointJacobian(graph, BODY, SHAPE->A_CB.org, NULL, J);
      MatNd_mul(&xp_spring, J, &qp);

      // Compute spring force
      double f[3];
      const double stiffness = SHAPE->scale3d[0];
      const double damping = SHAPE->scale3d[1];
      for (int i = 0; i < 3; ++i)
      {
        f[i] = stiffness*(x_anchor[i]-x_spring[i]) +
               damping*(xp_anchor[i]-xp_spring_[i]);
      }

      // Project contact forces into joint space
      MatNd F_i = MatNd_fromPtr(3, 1, f);
      MatNd_transposeSelf(J);
      MatNd_mulAndAddSelf(M_contact, J, &F_i);
    }
  }
#endif

  // Transfer joint velocities
  memmove(&xp[0], qp.ele, nq * sizeof(double));

  // Add joint torque
  MatNd* T_des_ik = MatNd_create(nq, 1);
  RcsGraph_stateVectorToIK(graph, kSim->T_des, T_des_ik);
  MatNd_addSelf(M_contact, T_des_ik);
  MatNd_destroy(T_des_ik);

  // Compute accelerations
  //kSim->energy = Rcs_directDynamics(graph, M_contact, kSim->draggerTorque, &qpp);
  kSim->energy = kSim->dirdyn(graph, M_contact, kSim->draggerTorque, &qpp, dt);

  if (MatNd_isNAN(&qpp) == true)
  {
    RLOG(1, "Found NaN elements in qpp - setting to zero");

    REXEC(2)
    {
      MatNd_printCommentDigits("qp", &qp, 3);
      MatNd_printCommentDigits("qpp", &qpp, 3);
      MatNd_printCommentDigits("xp_c", &xp_c, 3);
    }

    memset(xp, 0, 2 * nq + 3 * nc * sizeof(double));
    RPAUSE();
  }

  // Cleanup
  MatNd_destroy(J);
  MatNd_destroy(M_contact);
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
  RCHECK(src->n==1);
  MatNd_reshape(dst, idx.size(), 1);

  for (size_t i = 0; i < idx.size(); ++i)
  {
    RCHECK(idx[i]<(int)src->m);
    dst->ele[i] = src->ele[idx[i]];
  }
}

static void getSubVec(MatNd* dst, const std::vector<int>& idx)
{
  MatNd* src = MatNd_clone(dst);
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

#define EOM_DECOMPOSE

double KineticSimulation::dirdyn(const RcsGraph* graph,
                                 const MatNd* F_ext,
                                 const MatNd* F_jnt,
                                 MatNd* q_ddot,
                                 double dt)
{
  int n = graph->nJ;
  MatNd* M = MatNd_create(n, n);
  MatNd* F_gravity = MatNd_create(n, 1);
  MatNd* h = MatNd_create(n, 1);
  MatNd* b = MatNd_create(n, 1);

  // Compute mass matrix, h-vector and gravity forces
  double E = RcsGraph_computeKineticTerms(graph, M, h, F_gravity);

  // Joint speed damping: M Kv(qp_des - qp) with qp_des = 0
  // We consider all bodies with six joints as floating and do not apply any
  // damping.
  // \todo: Provide interface on per-joint basis
  const double damping = 5.0;
  MatNd* Fi = MatNd_create(n, 1);
  MatNd* qp_ik = MatNd_clone(graph->q_dot);

  RCSGRAPH_TRAVERSE_BODIES(graph)
  {
    if (RcsBody_numJoints(graph, BODY)==6)
    {
      const RcsJoint* ji = RCSJOINT_BY_ID(graph, BODY->jntId);
      RCHECK(ji);
      VecNd_setZero(&qp_ik->ele[ji->jointIndex], 6);
    }
  }

  RcsGraph_stateVectorToIKSelf(graph, qp_ik);
  MatNd_mul(Fi, M, qp_ik);
  MatNd_constMulSelf(Fi, -damping);
  MatNd_addSelf(b, Fi);
  MatNd_destroy(qp_ik);
  MatNd_destroy(Fi);

  // Solve direct dynamics for joint accelerations
  MatNd_addSelf(b, F_gravity);
  MatNd_addSelf(b, F_ext);
  MatNd_subSelf(b, F_jnt);
  MatNd_addSelf(b, h);



  // Matrix decomposition into torque and position controlled partitions
#if defined (EOM_DECOMPOSE)
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
  getSubVec(qpp_c, pIdx);
  MatNd_setZero(qpp_c);

  MatNd* tmp = MatNd_create(graph->nJ, 1);
  RCSGRAPH_TRAVERSE_JOINTS(graph)
  {
    if (JNT->jacobiIndex == -1)
    {
      continue;
    }

    if (JNT->ctrlType == RCSJOINT_CTRL_POSITION)
    {
      double q_des_i = this->q_des->ele[JNT->jointIndex];
      double q_curr_i = graph->q->ele[JNT->jointIndex];
      double qp_curr_i = graph->q_dot->ele[JNT->jointIndex];
      double qpp_i = 2.0 * (q_des_i - q_curr_i - qp_curr_i*dt) / (dt*dt);
      tmp->ele[JNT->jacobiIndex] = 0.01*qpp_i;

      // aq = -kp*(q-q_des)  -kd*qp_curr
      double kp = 100.0;
      double kd = 0.5*sqrt(4.0*kp);
      tmp->ele[JNT->jacobiIndex] = -kp*(q_curr_i- q_des_i) - kd*qp_curr_i;
    }
    else if (JNT->ctrlType == RCSJOINT_CTRL_VELOCITY)
    {
      double qp_curr_i = graph->q_dot->ele[JNT->jointIndex];
      double qp_des_i = this->q_dot_des->ele[JNT->jointIndex];
      tmp->ele[JNT->jacobiIndex] = (qp_des_i-qp_curr_i)/dt;
    }
  }
  getSubVec(tmp, pIdx);
  MatNd_copy(qpp_c, tmp);
  MatNd_destroy(tmp);
  getSubVec(b_f, tIdx);
  MatNd* Mqpp_c = MatNd_create(tIdx.size(), 1);
  MatNd_mul(Mqpp_c, M01, qpp_c);
  MatNd_subSelf(b_f, Mqpp_c);
  MatNd* q_ddot_f = MatNd_createLike(b_f);
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

  MatNd_destroyN(7, M00, M01, qpp_f, qpp_c, Mqpp_c, q_ddot_f, b_f);
#else
  MatNd_reshape(q_ddot, n, 1);
  double det = MatNd_choleskySolve(q_ddot, M, b);
#endif

  // Warn if something seriously goes wrong.
  if (det == 0.0)
  {
    RLOG(2, "Couldn't solve direct dynamics - determinant is 0");
  }

  if (MatNd_isINF(q_ddot))
  {
    MatNd_setZero(q_ddot);
    RLOG(1, "q_ddot has infinite values - det was %g", det);
  }

  // Memorize joint torques.
  RcsGraph_stateVectorFromIK(graph, b, this->jointTorque);

  // Clean up
  MatNd_destroy(M);
  MatNd_destroy(F_gravity);
  MatNd_destroy(h);
  MatNd_destroy(b);

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

  We assume a number of things:

  1. The contact plane normal is pointing up (0 0 1), and its height is (0 0 z0)
  2. Since z is pointing up (0 0 1), the vertical component of the contact
   force can only be >=0.0
  3. The contact point is attached to the body point with an asymptotically
   damped second order system and is mass-less
  4. Gains are positive
  5. Everything is represented in world coordinates

*******************************************************************************/
KineticSimulation::FrictionContactPoint::FrictionContactPoint(const RcsBody* bdy_,
                                                              int shapeIdx_,
                                                              double mu_,
                                                              double k_p_,
                                                              double z0_) :
  bdyId(bdy_->id), shapeIdx(shapeIdx_), mu(mu_), k_p(k_p_), k_v(sqrt(4.0*k_p_)),
  z0(z0_)
{
  // Initialize contact point with attachement point coordinates
  HTr A_CI;
  HTr_transform(&A_CI, &bdy_->A_BI, &bdy_->shape[shapeIdx]->A_CB);
  Vec3d_copy(x_contact, A_CI.org);
  Vec3d_setZero(f_contact);
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

void KineticSimulation::FrictionContactPoint::computeContactSpeed(double xp_contact[3],
                                                                  const double dt,
                                                                  const double x_contact[3],
                                                                  const double x_attach[3],
                                                                  const double xp_attach[3]) const
{
  // If the body point is not penetrating, the velocity is computed so
  // that x_contact coincides with x_attach
  if (x_attach[2] > this->z0)
  {
    xp_contact[0] = (x_attach[0] - x_contact[0]) / dt;
    xp_contact[1] = (x_attach[1] - x_contact[1]) / dt;
    xp_contact[2] = (x_attach[2] - x_contact[2]) / dt;
    return;
  }


  // If the body point is penetrating, the velocity is computed so that
  // x_contact[2] lies at the surface.
  xp_contact[2] = (this->z0 - x_contact[2]) / dt;

  // Horizontal component. There are 2 cases:
  // 1. The tangential force is within the friction cone. Then, the tangential
  //    contact point velocities are zero.
  // 2. The tangential force exceeds the friction force. Then, the tangential
  //    contact point velocity is computed so that the contact point ends up
  //    on the friction cone's boundary, in the direction towards the
  //    attachement point
  double f[3], ft, ft_limit;

  computeContactForce(f, x_contact, x_attach, xp_attach);
  ft = sqrt(f[0] * f[0] + f[1] * f[1]);
  ft_limit = this->mu*f[2];

  // Tangential forces are inside friction cone - contact points don't move
  if (ft <= ft_limit)
  {
    xp_contact[0] = 0.0;
    xp_contact[1] = 0.0;
  }
  else
  {
    double dx = sqrt(pow(x_contact[0] - x_attach[0], 2) +
                     pow(x_contact[1] - x_attach[1], 2));

    dx -= ft_limit / this->k_p;

    for (int i = 0; i < 2; i++)
    {
      xp_contact[i] = -(dx*f[i] / ft) / dt;
    }
  }

}


}   // namespace Rcs
