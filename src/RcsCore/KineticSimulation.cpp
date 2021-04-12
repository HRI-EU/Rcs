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
#include "Rcs_shape.h"


namespace Rcs
{

static const char className[] = "NewtonEuler";
static PhysicsFactoryRegistrar<KineticSimulation> physics(className);

/*******************************************************************************
 * Constructor.
 ******************************************************************************/
KineticSimulation::KineticSimulation() : PhysicsBase(), draggerTorque(NULL)
{
}

/*******************************************************************************
 * Constructor.
 ******************************************************************************/
KineticSimulation::KineticSimulation(const RcsGraph* graph_) :
  PhysicsBase(graph_), draggerTorque(NULL)
{
  this->draggerTorque = MatNd_createLike(graph_->q);
  MatNd_reshape(this->draggerTorque, getGraph()->nJ, 1);
}

/*******************************************************************************
 * Copy constructor.
 ******************************************************************************/
KineticSimulation::KineticSimulation(const KineticSimulation& copyFromMe) :
  PhysicsBase(copyFromMe), draggerTorque(NULL)
{
  this->draggerTorque = MatNd_clone(copyFromMe.draggerTorque);
  MatNd_reshape(this->draggerTorque, getGraph()->nJ, 1);
}

/*******************************************************************************
 * Copy constructor.
 ******************************************************************************/
KineticSimulation::KineticSimulation(const KineticSimulation& copyFromMe,
                                     const RcsGraph* newGraph) :
  PhysicsBase(copyFromMe, newGraph), draggerTorque(NULL)
{
  this->draggerTorque = MatNd_clone(copyFromMe.draggerTorque);
  MatNd_reshape(this->draggerTorque, getGraph()->nJ, 1);
}

/*******************************************************************************
 * Assignment operator.
 ******************************************************************************/
KineticSimulation& KineticSimulation::operator= (const KineticSimulation& copyFromMe)
{
  RFATAL("FIXME");
  return *this;
}

/*******************************************************************************
 * Destructor.
 ******************************************************************************/
KineticSimulation::~KineticSimulation()
{
  MatNd_destroy(this->draggerTorque);
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

  // Initialize contact points
  RCSGRAPH_TRAVERSE_BODIES(getGraph())
  {
    int shapeIdx = 0;

    RCSBODY_TRAVERSE_SHAPES(BODY)
    {
      if (RcsShape_isOfComputeType(SHAPE, RCSSHAPE_COMPUTE_CONTACT))
      {
        contact.push_back(FrictionContactPoint(BODY, shapeIdx));
      }

      shapeIdx++;
    }
  }

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

  const int n = getGraph()->nJ;
  const int nc = contact.size();
  double dt_opt = dt;// \todo: Fix this
  MatNd* z = MatNd_create(2*n+3*nc, 1);
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

  // Joint speed damping: M Kv(qp_des - qp)
  const double damping = 1.0;
  MatNd* MM = MatNd_create(n, n);
  RcsGraph_computeMassMatrix(getGraph(), MM);
  MatNd* Fi = MatNd_create(n, 1);
  MatNd_mul(Fi, MM, &zqd);
  MatNd_constMulSelf(Fi, -damping);
  MatNd_addSelf(F_ext, Fi);
  MatNd_destroy(Fi);
  MatNd_destroy(MM);

  DirDynParams params;
  params.graph = getGraph();
  params.F_ext = F_ext;
  const char* integrator = "Euler";

  if (STREQ(integrator, "Fehlberg"))
  {
    MatNd* err = MatNd_create(2 * n, 1);
    for (int i = 0; i < n; i++)
    {
      err->ele[i] = 1.0e-2;
      err->ele[i + n] = 1.0e-3;
    }

    int nSteps = integration_t1_t2(Rcs_directDynamicsIntegrationStep,
                                   (void*)&params, 2*n, time(), time()+dt,
                                   &dt_opt, z->ele, z->ele, err->ele);
    MatNd_destroy(err);

    RLOG(1, "Adaptive integration took %d steps", nSteps);
  }
  else if (STREQ(integrator, "Euler"))
  {
    //integration_euler(Rcs_directDynamicsIntegrationStep,
    // (void*)&params, 2 * n, dt, z->ele, z->ele);
    integration_euler(integrationStep, (void*)this, 2*n+3*nc, dt, z->ele, z->ele);
  }
  else
  {
    RFATAL("Unknonw integrator: \"%s\"", integrator);
  }

  // Copy integrated state back into graph ...
  RcsGraph_setState(params.graph, &zq, &zqd);

  // ... and into contact point coordinates
  for (int i = 0; i < nc; ++i)
  {
    double* row = MatNd_getRowPtr(&zc, i);
    Vec3d_copy(contact[i].x_contact, row);
  }

  MatNd_destroy(z);
  MatNd_destroy(F_ext);

  if (q)
  {
    MatNd_reshapeCopy(q, params.graph->q);
  }

  if (q_dot)
  {
    MatNd_reshapeCopy(q_dot, params.graph->q_dot);
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
  RFATAL("FIXME");
}

/*******************************************************************************
 *
 ******************************************************************************/
void KineticSimulation::applyAngularVelocity(const RcsBody* body,
                                             const double omega[3])
{
  RFATAL("FIXME");
}

/*******************************************************************************
 *
 ******************************************************************************/
void KineticSimulation::getLinearVelocity(const RcsBody* body,
                                          double v[3]) const
{
  RFATAL("FIXME");
}

/*******************************************************************************
 *
 ******************************************************************************/
void KineticSimulation::getAngularVelocity(const RcsBody* body,
                                           double omega[3]) const
{
  RFATAL("FIXME");
}

/*******************************************************************************
 *
 ******************************************************************************/
void KineticSimulation::setJointTorque(const MatNd* T_des)
{
  RFATAL("FIXME");
}

/*******************************************************************************
 *
 ******************************************************************************/
void KineticSimulation::getJointTorque(MatNd* T_curr, RcsStateType type) const
{
  RFATAL("FIXME");
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
  RFATAL("FIXME");
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
bool KineticSimulation::addBody(const RcsGraph* graph, const RcsBody* body)
{
  RFATAL("FIXME");
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
    const RcsBody* cBdy = kSim->contact[i].bdy;
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

    RLOG(2, "%s: Contact point: %f %f %f", kSim->contact[i].bdy->name,
         x_contact_i[0], x_contact_i[1], x_contact_i[2]);
    RLOG(2, "Attachement point: %f %f %f",
         x_attach_i[0], x_attach_i[1], x_attach_i[2]);
    RLOG(1, "F: %f %f %f (%f)", F_i.ele[0], F_i.ele[1], F_i.ele[2], x_contact_i[2]);
    RLOG(1, "xp: %f %f %f", xp_contact_i[0], xp_contact_i[1], xp_contact_i[2]);

    // Project contact forces into joint space
    MatNd_transposeSelf(J);
    MatNd_mulAndAddSelf(M_contact, J, &F_i);
  }

  // Transfer joint velocities
  memmove(&xp[0], qp.ele, nq * sizeof(double));

  // Add joint torque
  MatNd* T_des_ik = MatNd_create(nq, 1);
  RcsGraph_stateVectorToIK(graph, kSim->T_des, T_des_ik);
  MatNd_addSelf(M_contact, T_des_ik);
  MatNd_destroy(T_des_ik);

  // Compute accelerations
  Rcs_directDynamics(graph, M_contact, kSim->draggerTorque, &qpp);

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

  We assume a number of things:

  1. The contact plane normal is pointing up (0 0 1), and its height is (0 0 z0)
  2. Since z is pointing up (0 0 1), the vertical component of the contact
   force can only be >=0.0
  3. The contact point is attached to the body point with an asymptotically
   damped second order system and is mass-less
  4. Gains are positive
  5. Everything is represented in world coordinates

*******************************************************************************/
KineticSimulation::FrictionContactPoint::FrictionContactPoint(const RcsBody* bdy_, int shapeIdx_, double mu_, double k_p_, double z0_) :
  bdy(bdy_), shapeIdx(shapeIdx_), mu(mu_), k_p(k_p_), k_v(sqrt(4.0*k_p_)), z0(z0_)
{
  // Initialize contact point with attachement point coordinates
  HTr A_CI;
  HTr_transform(&A_CI, &bdy->A_BI, &bdy->shape[shapeIdx]->A_CB);
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
