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

#include "PhysicsFactory.h"
#include "BulletSimulation.h"
#include "BulletRigidBody.h"
#include "BulletHelpers.h"
#include "BulletDebugDrawer.h"

#include <Rcs_typedef.h>
#include <Rcs_macros.h>
#include <Rcs_math.h>
#include <Rcs_body.h>
#include <Rcs_joint.h>
#include <Rcs_sensor.h>

#include <BulletDynamics/MLCPSolvers/btDantzigSolver.h>
#include <BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h>
#include <BulletDynamics/MLCPSolvers/btMLCPSolver.h>

#include <iostream>
#include <climits>



static const char className[] = "Bullet";
static Rcs::PhysicsFactoryRegistrar<Rcs::BulletSimulation> physics(className);


typedef std::map<const RcsJoint*, Rcs::BulletHingeJoint*>::iterator hinge_it;
typedef std::map<const RcsBody*, Rcs::BulletRigidBody*>::iterator body_it;


/*******************************************************************************
 * Callback for collision filtering. Do your collision logic here
 ******************************************************************************/
static inline void MyNearCallbackDisabled(btBroadphasePair& collisionPair,
                                          btCollisionDispatcher& dispatcher,
                                          const btDispatcherInfo& dispatchInfo)
{
  return;
}

/*******************************************************************************
 * Callback for collision filtering. Do your collision logic here
 ******************************************************************************/
static inline void MyNearCallbackEnabled(btBroadphasePair& collisionPair,
                                         btCollisionDispatcher& dispatcher,
                                         const btDispatcherInfo& dispatchInfo)
{
  // Do your collision logic here
  btBroadphaseProxy* p0 = collisionPair.m_pProxy0;
  btBroadphaseProxy* p1 = collisionPair.m_pProxy1;
  btCollisionObject* co0 = static_cast<btCollisionObject*>(p0->m_clientObject);
  btCollisionObject* co1 = static_cast<btCollisionObject*>(p1->m_clientObject);

  Rcs::BulletRigidBody* rb0 = dynamic_cast<Rcs::BulletRigidBody*>(co0);
  Rcs::BulletRigidBody* rb1 = dynamic_cast<Rcs::BulletRigidBody*>(co1);

  if ((rb0!=NULL) && (rb1!=NULL))
  {
    NLOG(0, "Broadphase collision between %s and %s",
         rb0->getBodyName(), rb1->getBodyName());

    if (rb0->getBodyPtr()->rigid_body_joints==false ||
        rb1->getBodyPtr()->rigid_body_joints==false)
    {

      if ((RcsBody_isChild(rb0->getBodyPtr(), rb1->getBodyPtr())) ||
          (RcsBody_isChild(rb1->getBodyPtr(), rb0->getBodyPtr())))
      {
        //return;
      }

    }

    // if (STREQ(rb0->getBodyPtr()->suffix, rb0->getBodyPtr()->suffix) &&
    //     (strlen(rb0->getBodyPtr()->suffix)==0))
    // {
    //   return;
    // }

    //RLOG(0, "Checking %s - %s", rb0->getBodyName(), rb1->getBodyName());
  }

  // Only dispatch the Bullet collision information if you want the physics to
  // continue
  dispatcher.defaultNearCallback(collisionPair, dispatcher, dispatchInfo);
}






/*******************************************************************************
 *
 ******************************************************************************/
Rcs::BulletSimulation::BulletSimulation(const RcsGraph* graph_,
                                        const char* cfgFile) :
  Rcs::PhysicsBase(graph_), lastDt(0.001), debugDrawer(NULL)
{
  pthread_mutex_init(&this->mtx, NULL);
  initPhysics();

  REXEC(5)
  {
    print();
    RLOG(5, "Done BulletSimulation constructor");
  }
}

/*******************************************************************************
 * Copy constructor
 ******************************************************************************/
Rcs::BulletSimulation::BulletSimulation(const BulletSimulation& copyFromMe):
  PhysicsBase(copyFromMe, copyFromMe.graph),
  lastDt(copyFromMe.lastDt)
{
  pthread_mutex_init(&this->mtx, NULL);
  initPhysics();
}

/*******************************************************************************
 * Copy constructor
 ******************************************************************************/
Rcs::BulletSimulation::BulletSimulation(const BulletSimulation& copyFromMe,
                                        const RcsGraph* newGraph):
  PhysicsBase(copyFromMe, newGraph),
  lastDt(copyFromMe.lastDt)
{
  pthread_mutex_init(&this->mtx, NULL);
  initPhysics();
}

/*******************************************************************************
 * Cleanup in the reverse order of creation/initialization.
 ******************************************************************************/
//! \todo Clean up ground shape
Rcs::BulletSimulation::~BulletSimulation()
{
  // Remove the constraints from the dynamics world and delete them
  for (int i=dynamicsWorld->getNumConstraints()-1; i>=0 ; i--)
  {
    btTypedConstraint* constraint_i = dynamicsWorld->getConstraint(i);
    dynamicsWorld->removeConstraint(constraint_i);
    delete constraint_i;
  }

  // Remove the rigidbodies from the dynamics world and delete them
  for (int i=dynamicsWorld->getNumCollisionObjects()-1; i>=0 ; i--)
  {
    btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
    btRigidBody* body = btRigidBody::upcast(obj);
    if (body && body->getMotionState())
    {
      delete body->getMotionState();
    }
    dynamicsWorld->removeCollisionObject(obj);
#if !defined (_MSC_VER)
    delete obj;
#endif
  }

  delete this->dynamicsWorld;
  delete this->m_solver;
  delete this->m_broadphase;
  delete this->m_dispatcher;
  delete this->m_collisionConfiguration;

  if (this->mlcpSolver != NULL)
  {
    delete this->mlcpSolver;
  }

  pthread_mutex_destroy(&this->mtx);
}

/*******************************************************************************
 * Cone function
 ******************************************************************************/
Rcs::BulletSimulation* Rcs::BulletSimulation::clone(RcsGraph* newGraph) const
{
  return new Rcs::BulletSimulation(*this, newGraph);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletSimulation::lock()
{
  pthread_mutex_lock(&this->mtx);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletSimulation::unlock()
{
  pthread_mutex_unlock(&this->mtx);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletSimulation::initPhysics()
{
  m_collisionConfiguration = new btDefaultCollisionConfiguration();

  m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
  m_dispatcher->setNearCallback(MyNearCallbackEnabled);

  btVector3 worldAabbMin(-10.0, -10.0, -10.0);
  btVector3 worldAabbMax(10.0, 10.0, 10.0);
  m_broadphase = new btAxisSweep3(worldAabbMin, worldAabbMax);

  bool useMCLPSolver = false;

#if BT_BULLET_VERSION > 281
  useMCLPSolver = true;
  if (useMCLPSolver)
  {
    this->mlcpSolver = new btDantzigSolver();
    //this->mlcpSolver = new btSolveProjectedGaussSeidel;
    m_solver = new btMLCPSolver(this->mlcpSolver);
  }
  else
#endif
  {
    m_solver = new btSequentialImpulseConstraintSolver;
  }



  this->dynamicsWorld =
    new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver,
                                m_collisionConfiguration);

  if (useMCLPSolver)
  {
    this->dynamicsWorld ->getSolverInfo().m_minimumSolverBatchSize = 1;
  }
  else
  {
    this->dynamicsWorld ->getSolverInfo().m_minimumSolverBatchSize = 128;
  }

  dynamicsWorld->getDispatchInfo().m_useConvexConservativeDistanceUtil = true;
  dynamicsWorld->getDispatchInfo().m_convexConservativeDistanceThreshold = 0.01f;
  dynamicsWorld->setGravity(btVector3(0.0, 0.0, -9.81));

  btContactSolverInfo& si = dynamicsWorld->getSolverInfo();
  si.m_numIterations = 20;
  si.m_erp = 0.2;   // 0: no joint error correction (Recommended: 0.1-0.8, default 0.2)
  si.m_globalCfm = 1.0e-4; // 0: hard constraint (Default)
  si.m_restingContactRestitutionThreshold = INT_MAX;
  si.m_splitImpulse = 1;
  si.m_solverMode = SOLVER_RANDMIZE_ORDER |
                    SOLVER_FRICTION_SEPARATE |
                    SOLVER_USE_2_FRICTION_DIRECTIONS |
                    SOLVER_USE_WARMSTARTING;

  // Create physics for RcsGraph
  RCSGRAPH_TRAVERSE_BODIES(graph)
  {
    RLOGS(5, "Creating bullet body for \"%s\"", BODY->name);

    BulletRigidBody* btBody = BulletRigidBody::create(BODY);

    if (btBody!=NULL)
    {
      bdyMap[BODY] = btBody;

      body_it it = bdyMap.find(BODY->parent);

      if (it!=bdyMap.end())
      {
        btBody->setParentBody(it->second);
      }



      dynamicsWorld->addRigidBody(btBody);
      //dynamicsWorld->addCollisionObject(btBdy);

      btTypedConstraint* jnt = btBody->createJoint(graph);

      if (jnt!=NULL)
      {
        // Fixed joints don't have a RcsJoint pointer, this must be checked
        // before applying the offset
        if (BODY->jnt!=NULL)
        {
          BulletHingeJoint* hinge = dynamic_cast<BulletHingeJoint*>(jnt);

          if (hinge != NULL)
          {
            hingeMap[BODY->jnt] = hinge;
            RLOGS(5, "Joint %s has value %f (%f)",
                  BODY->jnt->name, hinge->getHingeAngle(),
                  graph->q->ele[BODY->jnt->jointIndex]);
          }
        }

        dynamicsWorld->addConstraint(jnt, true);
      }

    }

    RLOGS(5, "%s adding \"%s\" to Bullet universe",
          btBody ? "SUCCESS" : "FAILURE", BODY->name);
  }

  // Create ground plane
  btCollisionShape* ground = new btStaticPlaneShape(btVector3(0, 0, 1), 0);
  btDefaultMotionState* gms = new btDefaultMotionState();
  btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0, gms, ground);
  btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
  dynamicsWorld->addRigidBody(groundRigidBody);

  // Force dragging
  this->dragBody = NULL;
  Vec3d_setZero(this->dragForce);
  Vec3d_setZero(this->dragAnchor);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletSimulation::setGravity(const double gravity[3])
{
  dynamicsWorld->setGravity(btVector3(gravity[0], gravity[1], gravity[2]));
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletSimulation::step(double dt)
{
  simulate(dt);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletSimulation::simulate(double dt, MatNd* q, MatNd* q_dot,
                                     MatNd* q_ddot, MatNd* T, bool control)
{
  this->simTime += dt;
  this->lastDt = dt;
  applyControl(dt);


  if (this->dragBody)
  {
    btVector3 f(dragForce[0], dragForce[1], dragForce[2]);

    // COM in world frame
    double I_com[3];
    Vec3d_copy(I_com, dragBody->getCOMTransformPtr()->org);

    // Force anchor point in world frame
    double I_dragAnchor[3];
    HTr A_BI;
    dragBody->getBodyTransform(&A_BI);
    Vec3d_transform(I_dragAnchor, &A_BI, this->dragAnchor);

    // Vector from COM to drag point in world frame
    double relPos[3];
    Vec3d_sub(relPos, I_dragAnchor, I_com);

    btVector3 a(relPos[0], relPos[1], relPos[2]);
    this->dragBody->applyForce(f, a);
  }

  lock();
  if (this->debugDrawer != NULL)
  {
    debugDrawer->clear();
  }

  dynamicsWorld->stepSimulation(dt, 0);

  if (this->debugDrawer != NULL)
  {
    dynamicsWorld->debugDrawWorld();
    debugDrawer->apply();
  }
  unlock();

  updateSensors();



  for (body_it it=bdyMap.begin(); it!=bdyMap.end(); ++it)
  {
    Rcs::BulletRigidBody* btBdy = it->second;
    btBdy->updateBodyTransformFromPhysics();
  }



  for (hinge_it it = hingeMap.begin(); it != hingeMap.end(); ++it)
  {
    Rcs::BulletHingeJoint* hinge = it->second;
    hinge->update(dt);
  }


  // Read joint angles
  if (q != NULL)
  {
    if (q->m == graph->dof)
    {
      getJointAngles(q, RcsStateFull);
    }
    else if (q->m == graph->nJ)
    {
      getJointAngles(q, RcsStateIK);
    }
    else
    {
      RFATAL("Wrong dimensions of q-vector: [%d x %d], dof=%d nJ=%d",
             q->m, q->n, graph->dof, graph->nJ);
    }
  }


  // Read joint velocities
  if (q_dot != NULL)
  {
    if (q_dot->m == graph->dof)
    {
      getJointVelocities(q_dot, RcsStateFull);
    }
    else if (q_dot->m == graph->nJ)
    {
      getJointVelocities(q_dot, RcsStateIK);
    }
    else
    {
      RFATAL("Wrong dimensions of q_dot-vector: [%d x %d], dof=%d nJ=%d",
             q_dot->m, q_dot->n, graph->dof, graph->nJ);
    }
  }


}

/*******************************************************************************
 * Adapted from bullet/Demos/OpenGL/DemoApplication.cpp
 ******************************************************************************/
void Rcs::BulletSimulation::reset()
{
  MatNd_copy(this->q_des, graph->q);
  MatNd_setZero(this->q_dot_des);
  MatNd_setZero(this->T_des);

  btBroadphaseInterface* bi = dynamicsWorld->getBroadphase();
  btOverlappingPairCache* opc = bi->getOverlappingPairCache();

  for (hinge_it it = hingeMap.begin(); it != hingeMap.end(); ++it)
  {
    Rcs::BulletHingeJoint* hinge = it->second;
    hinge->reset(this->q_des->ele[hinge->getJointIndex()]);
  }

  for (body_it it=bdyMap.begin(); it!=bdyMap.end(); ++it)
  {
    Rcs::BulletRigidBody* btBdy = it->second;

    if (btBdy==NULL)
    {
      const RcsBody* rb = it->first;
      RCHECK_MSG(btBdy, "%s", rb ? rb->name : "NULL");
    }

    btBdy->reset();

    // Remove cached contact points
    if (opc)
    {
      opc->cleanProxyFromPairs(btBdy->getBroadphaseHandle(),
                               dynamicsWorld->getDispatcher());
    }
  }

  // Reset some internal cached data in the broadphase
  bi->resetPool(dynamicsWorld->getDispatcher());
  dynamicsWorld->getConstraintSolver()->reset();
}

/******************************************************************************
 *
 *****************************************************************************/
const HTr* Rcs::BulletSimulation::getPhysicsTransformPtr(const RcsBody* body) const
{
  Rcs::BulletRigidBody* bb = getRigidBody(body);

  if (bb == NULL)
  {
    return body->A_BI;
  }

  return bb->getBodyTransformPtr();
}

/*******************************************************************************
 * setDamping() internally clips values between [0...1]
 ******************************************************************************/
//! \todo dragBody != oldBody can also happen (theoretically)
void Rcs::BulletSimulation::applyForce(const RcsBody* body, const double F[3],
                                       const double B_r[3])
{
  BulletRigidBody* oldDragBody = this->dragBody;
  this->dragBody = getRigidBody(body);

  if (this->dragBody == NULL)
  {
    Vec3d_setZero(this->dragForce);
    Vec3d_setZero(this->dragAnchor);

    if (oldDragBody!=NULL)
    {
      oldDragBody->applyCentralForce(btVector3(0.0, 0.0, 0.0));
      oldDragBody->setDamping(0.1, 0.9);
    }

    return;
  }

  dragBody->setDamping(0.9, 0.999999999999);

  // Drag anchor is in body frame
  Vec3d_copy(this->dragAnchor, B_r);
  Vec3d_constMul(this->dragForce, F, 10.0);
}

/*******************************************************************************
 * Sets the kinematic transform to the body.
 ******************************************************************************/
void Rcs::BulletSimulation::applyTransform(const RcsBody* body, const HTr* A_BI)
{
  Rcs::BulletRigidBody* bdy = getRigidBody(body);

  if (bdy != NULL)
  {
    bdy->setBodyTransform(A_BI);
  }
}

/*******************************************************************************
 * Set the linear velocity of a body.
 ******************************************************************************/
void Rcs::BulletSimulation::applyLinearVelocity(const RcsBody* body,
                                                const double v[3])
{
  Rcs::BulletRigidBody* bdy = getRigidBody(body);

  if (bdy == NULL)
  {
    RLOG(1, "Body \"%s\": Couldn't set velocity", body ? body->name : "NULL");
    return;
  }

  bdy->setLinearVelocity(btVector3(v[0], v[1], v[2]));
}

/*******************************************************************************
 * Set the linear velocity of a body.
 ******************************************************************************/
void Rcs::BulletSimulation::applyAngularVelocity(const RcsBody* body,
                                                 const double v[3])
{
  Rcs::BulletRigidBody* bdy = getRigidBody(body);

  if (bdy == NULL)
  {
    RLOG(1, "Body \"%s\": Couldn't set velocity", body ? body->name : "NULL");
    return;
  }

  bdy->setAngularVelocity(btVector3(v[0], v[1], v[2]));
}

/*******************************************************************************
 * Get the linear velocity of a body.
 ******************************************************************************/
void Rcs::BulletSimulation::getLinearVelocity(const RcsBody* body,
                                              double v[3]) const
{
  Rcs::BulletRigidBody* bdy = getRigidBody(body);

  if (bdy == NULL)
  {
    RLOG(1, "Body \"%s\": Couldn't set velocity", body ? body->name : "NULL");
    return;
  }

  btVector3 vel = bdy->getLinearVelocity();
  v[0] = vel[0];
  v[1] = vel[1];
  v[2] = vel[2];
}

/*******************************************************************************
 * Get the angular velocity of a body.
 ******************************************************************************/
void Rcs::BulletSimulation::getAngularVelocity(const RcsBody* body,
                                               double v[3]) const
{
  Rcs::BulletRigidBody* bdy = getRigidBody(body);

  if (bdy == NULL)
  {
    RLOG(1, "Body \"%s\": Couldn't set velocity", body ? body->name : "NULL");
    return;
  }

  btVector3 vel = bdy->getAngularVelocity();
  v[0] = vel[0];
  v[1] = vel[1];
  v[2] = vel[2];
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletSimulation::getJointAngles(MatNd* q, RcsStateType type) const
{
  MatNd_reshape(q, (type == RcsStateFull) ? graph->dof : graph->nJ, 1);

#if 0
  // Update all hinge joints
  std::map<const RcsJoint*, Rcs::BulletHingeJoint*>::const_iterator jit;

  for (jit = hingeMap.begin(); jit != hingeMap.end(); ++jit)
  {
    const RcsJoint* rj = jit->first;
    Rcs::BulletHingeJoint* hinge = jit->second;

    if (hinge != NULL)
    {
      int idx = (type==RcsStateFull) ? rj->jointIndex : rj->jacobiIndex;
      RCHECK_MSG(idx>=0 && idx<(int)graph->dof, "Joint \"%s\": idx = %d",
                 rj ? rj->name : "NULL", idx);
      q->ele[idx] = hinge->getJointAngle();
    }
    else
    {
      RLOG(5, "Skipping Updating %s", rj ? rj->name : "NULL");
    }

  }
#else
  RCSGRAPH_TRAVERSE_JOINTS(this->graph)
  {
    int idx = (type==RcsStateFull) ? JNT->jointIndex : JNT->jacobiIndex;
    RCHECK_MSG(idx>=0 && idx<(int)graph->dof, "Joint \"%s\": idx = %d",
               JNT->name, idx);

    Rcs::BulletHingeJoint* hinge = getHinge(JNT);

    if (hinge != NULL)
    {
      q->ele[idx] = hinge->getJointAngle();
    }
    else
    {
      NLOG(5, "Updating kinematic joint %s", JNT->name);
      q->ele[idx] = this->q_des->ele[JNT->jointIndex];
    }

  }
#endif

  // Then update all rigid body dofs
  std::map<const RcsBody*, Rcs::BulletRigidBody*>::const_iterator it;

  for (it=bdyMap.begin(); it!=bdyMap.end(); ++it)
  {
    const RcsBody* rb = it->first;
    RCHECK(rb);

    if (rb->rigid_body_joints == true)
    {
      Rcs::BulletRigidBody* btBdy = it->second;
      RCHECK_MSG(btBdy, "%s", rb->name);
      HTr A_BI;
      btBdy->getBodyTransform(&A_BI);
      int idx = (type==RcsStateFull) ? rb->jnt->jointIndex : rb->jnt->jacobiIndex;
      RCHECK_MSG(idx>=0 && idx<(int)graph->dof, "Joint \"%s\": idx = %d",
                 rb->jnt ? rb->jnt->name : "NULL", idx);

      // Transformation update: Here we compute the rigid body degrees of
      // freedom so that the state vector matches the results from physics.
      // We need to distinguish between the case of no parent body, and if a
      // parent body exists. In the latter case, the transformation must be
      // relative between two bodies.
      HTr A_ParentI;
      RcsBody* rcsParent = rb->parent;

      if (rcsParent != NULL)
      {
        Rcs::BulletRigidBody* btParent = getRigidBody(rcsParent);

        // If the parent body is simulated in Bullet, we take it's sumulated
        // transformation.
        if (btParent != NULL)
        {
          btParent->getBodyTransform(&A_ParentI);
        }
        // Otherwise we use the kinematic transformation.
        else
        {
          HTr_copy(&A_ParentI, rcsParent->A_BI);
        }
      }
      else
      {
        HTr_setIdentity(&A_ParentI);
      }

      RcsGraph_relativeRigidBodyDoFs(rb, &A_BI, &A_ParentI, &q->ele[idx]);
    }
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletSimulation::getJointVelocities(MatNd* q_dot,
                                               RcsStateType type) const
{
  MatNd_reshape(q_dot, (type==RcsStateFull) ? graph->dof : graph->nJ, 1);

  // First update all hinge joints
  std::map<const RcsJoint*, Rcs::BulletHingeJoint*>::const_iterator it;

  for (it = hingeMap.begin(); it != hingeMap.end(); ++it)
  {
    const RcsJoint* rj = it->first;
    Rcs::BulletHingeJoint* hinge = it->second;

    if (hinge != NULL)
    {
      int idx = (type==RcsStateFull) ? rj->jointIndex : rj->jacobiIndex;
      RCHECK_MSG(idx>=0 && idx<(int)graph->dof, "Joint \"%s\": idx = %d",
                 rj ? rj->name : "NULL", idx);
      q_dot->ele[idx] = hinge->getJointVelocity();
    }
    else
    {
      RLOG(5, "Skipping Updating %s", rj ? rj->name : "NULL");
    }

  }

  // Update the rigid body dofs. Since they are constrained, it is only done
  // for the full state vector.
  std::map<const RcsBody*, Rcs::BulletRigidBody*>::const_iterator it2;

  for (it2=bdyMap.begin(); it2!=bdyMap.end(); ++it2)
  {
    const RcsBody* rb = it2->first;
    RCHECK(rb);

    if (rb->rigid_body_joints == true)
    {
      Rcs::BulletRigidBody* btBdy = it2->second;
      RCHECK_MSG(btBdy, "%s", rb->name);

      // btVector3 linearVelocity = btBdy->getLinearVelocity();
      // btVector3 angularVelocity = btBdy->getAngularVelocity();

      // NLOG(0, "Updating body %s: %f %f %f %f %f %f", rb->name,
      //      linearVelocity[0], linearVelocity[1], linearVelocity[2],
      //      angularVelocity[0], angularVelocity[1], angularVelocity[2]);

      RcsJoint* jnt = rb->jnt;
      RCHECK(jnt);
      MatNd_set(q_dot, jnt->jointIndex, 0, btBdy->x_dot[0]);
      jnt = jnt->next;
      RCHECK(jnt);
      MatNd_set(q_dot, jnt->jointIndex, 0, btBdy->x_dot[1]);
      jnt = jnt->next;
      RCHECK(jnt);
      MatNd_set(q_dot, jnt->jointIndex, 0, btBdy->x_dot[2]);
      jnt = jnt->next;
      RCHECK(jnt);
      MatNd_set(q_dot, jnt->jointIndex, 0, btBdy->omega[0]);
      jnt = jnt->next;
      RCHECK(jnt);
      MatNd_set(q_dot, jnt->jointIndex, 0, btBdy->omega[1]);
      jnt = jnt->next;
      RCHECK(jnt);
      MatNd_set(q_dot, jnt->jointIndex, 0, btBdy->omega[2]);
    }
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletSimulation::setJointTorque(const RcsJoint* jnt, double torque)
{
  Rcs::BulletHingeJoint* hinge = getHinge(jnt);

  if (hinge == NULL)
  {
    RLOGS(1, "Calling function setJointTorque() with NULL hinge (RcsJoint "
          "\"%s\") - skipping", jnt ? jnt->name : "NULL");
    return;
  }

  hinge->setJointTorque(torque, this->lastDt);
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::BulletSimulation::setJointAngle(const RcsJoint* jnt, double angle,
                                          double dt)
{
  Rcs::BulletHingeJoint* hinge = getHinge(jnt);

  if (hinge == NULL)
  {
    NLOG(5, "Couldn't find hinge joint for joint \"%s\"",
         jnt ? jnt->name : "NULL");
    return false;
  }

  hinge->setJointAngle(angle, dt);

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletSimulation::setJointTorque(const MatNd* T_des)
{
  RCSGRAPH_TRAVERSE_JOINTS(this->graph)
  {
    int idx = (T_des->m==graph->dof) ? JNT->jointIndex : JNT->jacobiIndex;
    setJointTorque(JNT, MatNd_get(T_des, idx, 0));
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletSimulation::getJointTorque(MatNd* T_curr,
                                           RcsStateType type) const
{
  RLOG(4, "BulletSimulation::getJointTorque: Implement me");
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletSimulation::setMassAndInertiaFromPhysics(RcsGraph* graph_)
{
  RLOG(4, "BulletSimulation::setMassAndInertiaFromPhysics: Implement me");
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletSimulation::getPhysicsTransform(HTr* A_BI,
                                                const RcsBody* body) const
{
  Rcs::BulletRigidBody* bdy = getRigidBody(body);
  if (bdy != NULL)
  {
    bdy->getBodyTransform(A_BI);
  }
  else
  {
    RLOG(1, "Couldn't get physics transformation of body \"%s\"",
         body ? body->name : "NULL");
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletSimulation::disableCollisions()
{
  m_dispatcher->setNearCallback(MyNearCallbackDisabled);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletSimulation::disableCollision(const RcsBody* b0,
                                             const RcsBody* b1)
{
  RFATAL("Implement me");
}

/*******************************************************************************
 * Lock axis: lower limit = upper limit
 * Disable limits: lower limit > upper limit
 ******************************************************************************/
//! \todo Prismatic joints are missing.
void Rcs::BulletSimulation::setJointLimits(bool enable)
{
  for (hinge_it it = hingeMap.begin(); it != hingeMap.end(); ++it)
  {
    const RcsJoint* jnt = it->first;
    Rcs::BulletHingeJoint* hinge = it->second;
    RCHECK(jnt);
    RCHECK(hinge);
    if (enable==true)
    {
      hinge->setJointLimit(enable, jnt->q_min, jnt->q_max);
    }
    else
    {
      hinge->setJointLimit(enable, btScalar(1.0), btScalar(-1.0));
    }
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::PhysicsBase::Contacts Rcs::BulletSimulation::getContacts()
{
  lock();
  Contacts contacts;
  int numManifolds = this->dynamicsWorld->getDispatcher()->getNumManifolds();

  for (int i=0; i<numManifolds; i++)
  {
    btPersistentManifold* contactManifold =
      this->dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);

#if 0
    const btCollisionObject* obA =
      static_cast<const btCollisionObject*>(contactManifold->getBody0());
    const btCollisionObject* obB =
      static_cast<const btCollisionObject*>(contactManifold->getBody1());

    const Rcs::BulletRigidBody* rbA = dynamic_cast<const Rcs::BulletRigidBody*>(obA);
    const Rcs::BulletRigidBody* rbB = dynamic_cast<const Rcs::BulletRigidBody*>(obB);
#endif

    int numContacts = contactManifold->getNumContacts();

#if 0
    if ((numContacts>0) && rbA && rbB)
    {
      RLOG(5, "%s - %s:", rbA->getBodyName(), rbB->getBodyName());
    }
#endif

    for (int j=0; j<numContacts; j++)
    {
      btManifoldPoint& pt = contactManifold->getContactPoint(j);

      if (pt.getDistance()<0.f)
      {
        Contact c;

        for (int i=0; i<3; ++i)
        {
          c.pos[i] = pt.m_positionWorldOnB[i];
          c.force[i] = (pt.m_normalWorldOnB[i]*pt.m_appliedImpulse +
                        pt.m_lateralFrictionDir1[i]*pt.m_appliedImpulseLateral1 +
                        pt.m_lateralFrictionDir2[i]*pt.m_appliedImpulseLateral2)/this->lastDt;
        }
        contacts.push_back(c);
        NLOGS(0, "Contact %d: %.4f %.4f %.4f   %.3f %.3f %.3f", j,
              c.pos[0], c.pos[1], c.pos[2], c.force[0], c.force[1], c.force[2]);

        // if (rbA && rbB)
        // {
        //   double contactForce[3];
        //   Vec3d_set(contactForce, c.force[0], c.force[1], c.force[2]);
        //   RLOG(0, "Force[%d] = %f", j, Vec3d_getLength(contactForce));
        // }
      }
    }
  }
  unlock();

  return contacts;
}

/*******************************************************************************
 *
 ******************************************************************************/
size_t Rcs::BulletSimulation::getNumberOfContacts() const
{
  size_t numContacts = 0;

  int numManifolds = this->dynamicsWorld->getDispatcher()->getNumManifolds();

  for (int i=0; i<numManifolds; i++)
  {
    btPersistentManifold* contactManifold = this->dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
    numContacts += contactManifold->getNumContacts();
  }

  return numContacts;
}

/*******************************************************************************
 *
 ******************************************************************************/
//! \todo Traverse jointmap instead of graph
void Rcs::BulletSimulation::applyControl(double dt)
{
  // Apply transformations to all kinematic bodies
  if (!bdyMap.empty())
  {
    RcsGraph* graphCpy = RcsGraph_clone(this->graph);
    RcsGraph_setState(graphCpy, this->q_des, NULL);

    std::map<const RcsBody*, Rcs::BulletRigidBody*>::const_iterator it;

    for (it = bdyMap.begin(); it!=bdyMap.end(); ++it)
    {
      const RcsBody* rb_ = it->first;
      const RcsBody* rb = RcsGraph_getBodyByName(graphCpy, rb_->name);
      RCHECK(rb);
      BulletRigidBody* btBdy = it->second;
      if (btBdy && btBdy->isStaticOrKinematicObject())
      {
        btBdy->setBodyTransform(rb->A_BI, dt);
      }
    }

    RcsGraph_destroy(graphCpy);
  }



  // Set desired joint controls
  if (T_des->m==graph->dof)
  {
    RCSGRAPH_TRAVERSE_JOINTS(this->graph)
    {
      if (JNT->ctrlType==RCSJOINT_CTRL_TORQUE)
      {
        setJointTorque(JNT, MatNd_get(T_des, JNT->jointIndex, 0));
      }
      else if (JNT->ctrlType==RCSJOINT_CTRL_POSITION)
      {
        int index = JNT->jointIndex;
        double q_cmd;
        RcsJoint* j_master = JNT->coupledTo;

        if (j_master != NULL)
        {
          // If the joint is coupled, set the position command according
          // to its coupling factor.
          if (JNT->couplingFactors->size == 0)
          {
            q_cmd = MatNd_get(this->q_des, index, 0);
          }
          else
          {
            double q_master = MatNd_get(this->q_des, j_master->jointIndex, 0);
            q_cmd = RcsJoint_computeSlaveJointAngle(JNT, q_master);
          }
        }
        else
        {
          q_cmd = MatNd_get(this->q_des, JNT->jointIndex, 0);
        }

        setJointAngle(JNT, q_cmd, dt);
      }
      else   // RCSJOINT_CTRL_VELOCITY
      {
        BulletHingeJoint* hinge = getHinge(JNT);

        if (hinge != NULL)
        {
          double q_curr_i = hinge->getJointAngle();
          double q_dot_des_i = MatNd_get(this->q_dot_des, JNT->jointIndex, 0);
          double q_des_i  = q_curr_i + q_dot_des_i*dt;
          setJointAngle(JNT, q_des_i, dt);
        }
        else
        {
          RLOG(5, "No hinge joint found for joint \"%s\"", JNT->name);
        }
      }
    }
  }
  else if (T_des->m==graph->nJ)
  {
    RFATAL("CHECKME");
    RCSGRAPH_TRAVERSE_JOINTS(this->graph)
    {
      if (JNT->jacobiIndex == -1)
      {
        continue;
      }

      if (JNT->ctrlType==RCSJOINT_CTRL_TORQUE)
      {
        //Rcs::BulletHingeJoint* hinge = getHinge(JNT);

        //if (hinge == NULL)
        //{
        //  RLOGS(1, "Calling function setJointTorque() with NULL hinge (RcsJoint "
        //    "\"%s\") - skipping", JNT ? JNT->name : "NULL");
        //  return;
        //}

        //hinge->setJointTorque(MatNd_get(T_des, JNT->jacobiIndex, 0), dt);
        setJointTorque(JNT, MatNd_get(T_des, JNT->jacobiIndex, 0));
      }
      else if (JNT->ctrlType==RCSJOINT_CTRL_POSITION)
      {
        setJointAngle(JNT, MatNd_get(q_des, JNT->jacobiIndex, 0), dt);
      }
      else
      {
        RFATAL("NIY");
      }
    }
  }
  else
  {
    RFATAL("Wrong row size of T_des: %d, should be %d (dof) or %d (nJ)",
           T_des->m, graph->dof, graph->nJ);
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::BulletSimulation::updateLoadcell(const RcsSensor* fts)
{
  Rcs::BulletRigidBody* rb = getRigidBody(fts->body);

  if (rb == NULL)
  {
    RLOG(5, "No Rcs::BulletRigidBody found for RcsBody \"%s\"", fts->body->name);
    return false;
  }

  btFixedConstraint* jnt =
    static_cast<btFixedConstraint*>(rb->getUserPointer());

  const btJointFeedback* jf = jnt->getJointFeedback();

  if (jf==NULL)
  {
    RLOG(1, "No joint feedback found for RcsBody \"%s\"", fts->body->name);
    return false;
  }

  if (fts->rawData->size<6)
  {
    RLOG(1, "Data size mismatch for loadcell in  RcsBody \"%s\": size is %d",
         fts->body->name, fts->rawData->size);
    return false;
  }

  // Force and torque in world coordinates to the COM of the attached body
  double I_force[3], I_torque[3];
  I_force[0]  = jf->m_appliedForceBodyB.x();
  I_force[1]  = jf->m_appliedForceBodyB.y();
  I_force[2]  = jf->m_appliedForceBodyB.z();
  I_torque[0] = jf->m_appliedTorqueBodyB.x();
  I_torque[1] = jf->m_appliedTorqueBodyB.y();
  I_torque[2] = jf->m_appliedTorqueBodyB.z();

  // Project into sensor frame
  const BulletRigidBody& rbA =
    static_cast<const BulletRigidBody&>(jnt->getRigidBodyA());
  const BulletRigidBody& rbB =
    static_cast<const BulletRigidBody&>(jnt->getRigidBodyB());

  // Bullet force frame
  HTr A_FI;
  btTransform A_FI_ = rbB.getWorldTransform();
  Rcs::HTrFromBtTransform(&A_FI, A_FI_);

  // Sensor frame: A_SI = A_SB * A_BI
  const HTr* A_SB = fts->offset; // Body -> Sensor
  HTr A_SI, A_BI;
  rbA.getBodyTransform(&A_BI);
  HTr_transform(&A_SI, &A_BI, A_SB);

  // Torque in sensor's frame: S_torque = A_SI*(I_t + I_r_SF x I_f)
  double ftWrench[6];
  double* S_torque = &ftWrench[3];
  double I_r_SF[3];
  Vec3d_sub(I_r_SF, A_FI.org, A_SI.org);
  Vec3d_crossProduct(S_torque, I_r_SF, I_force);
  Vec3d_addSelf(S_torque, I_torque);
  Vec3d_rotateSelf(S_torque, A_SI.rot);

  // Force in sensor's frame: S_f = A_SI*I_f
  double* S_force = &ftWrench[0];
  Vec3d_rotate(S_force, A_SI.rot, I_force);

  // Sensor's mass compensation (static forces only)
  double S_f_gravity[6];
  RcsSensor_computeStaticForceCompensation(fts, S_f_gravity);
  Vec3d_subSelf(ftWrench, S_f_gravity);

  // Simple filter
  for (int i=0; i<6; ++i)
  {
    fts->rawData->ele[i] = 0.9*fts->rawData->ele[i] + 0.1*ftWrench[i];
  }

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletSimulation::updateSensors()
{
  RCSGRAPH_TRAVERSE_SENSORS(this->graph)
  {
    switch (SENSOR->type)
    {
      case RCSSENSOR_LOAD_CELL:
        updateLoadcell(SENSOR);
        break;

      case RCSSENSOR_PPS:
        //RcsSensor_computePPS(SENSOR, SENSOR->rawData);
        break;

      default:
        NLOG(5, "No update function for sensor type %d", SENSOR->type);
    }
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::BulletRigidBody* Rcs::BulletSimulation::getRigidBody(const RcsBody* bdy) const
{
  if (bdy == NULL)
  {
    return NULL;
  }

  std::map<const RcsBody*, Rcs::BulletRigidBody*>::const_iterator it;

  it = bdyMap.find(bdy);

  if (it==bdyMap.end())
  {
    return NULL;
  }
  else
  {
    return it->second;
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::BulletHingeJoint* Rcs::BulletSimulation::getHinge(const RcsJoint* jnt) const
{
  if (jnt == NULL)
  {
    return NULL;
  }

  std::map<const RcsJoint*, Rcs::BulletHingeJoint*>::const_iterator it;

  it = hingeMap.find(jnt);

  if (it!=hingeMap.end())
  {
    return it->second;
  }

  return NULL;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletSimulation::print() const
{
  RMSGS("Bullet version %d", btGetVersion());

  std::cout << "Body map has " << bdyMap.size() << " entries" << std::endl;
  unsigned int bdyCount = 0;

  if (bdyMap.empty()==false)
  {
    std::map<const RcsBody*, Rcs::BulletRigidBody*>::const_iterator it;

    for (it = bdyMap.begin(); it!=bdyMap.end(); ++it)
    {
      const RcsBody* rb = it->first;
      BulletRigidBody* btBdy = it->second;
      printf("%d: %s - %s   friction: %f\n", bdyCount++,
             rb ? rb->name : "NULL", btBdy ? btBdy->getBodyName() : "NULL",
             btBdy ? btBdy->getFriction(): 0.0);
    }
  }

  btContactSolverInfo& info = dynamicsWorld->getSolverInfo();
  printf("m_erp %f\n", info.m_erp);
  printf("m_erp2 %f\n", info.m_erp2);
  printf("m_linearSlop %f\n", info.m_linearSlop);
  printf("m_globalCfm %f\n", info.m_globalCfm);
  printf("m_maxErrorReduction %f\n", info.m_maxErrorReduction);
}

/*******************************************************************************
 *
 ******************************************************************************/
const char* Rcs::BulletSimulation::getClassName() const
{
  return className;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletSimulation::setDebugDrawer(BulletDebugDrawer* drawer)
{
  lock();

  this->debugDrawer = drawer;
  dynamicsWorld->setDebugDrawer(this->debugDrawer);

#if 0

  if ((drawer!=NULL) && (this->debugDrawer==NULL))
  {
    this->debugDrawer = drawer;
    this->debugDrawer->setEnabled(false);

    if (dynamicsWorld->getDebugDrawer() == NULL)
    {
      dynamicsWorld->setDebugDrawer(this->debugDrawer);
    }
    return;
  }

  if ((drawer!=NULL) && (this->debugDrawer==NULL))
  {
    this->debugDrawer = drawer;
    this->debugDrawer->setEnabled(false);

    if (dynamicsWorld->getDebugDrawer() == NULL)
    {
      dynamicsWorld->setDebugDrawer(this->debugDrawer);
    }
  }

#elif 0

  if ((drawer==NULL) && (this->debugDrawer!=NULL))
  {
    this->debugDrawer->setEnabled(false);
  }
  else if ((drawer!=NULL) && (this->debugDrawer==NULL))
  {
    drawer->setEnabled(false);
  }

  this->debugDrawer = drawer;
  dynamicsWorld->setDebugDrawer(this->debugDrawer);

#endif

  unlock();
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::BulletDebugDrawer* Rcs::BulletSimulation::getDebugDrawer() const
{
  return this->debugDrawer;
}

/*******************************************************************************
 * Sets compliance properties for all hinge joints
 ******************************************************************************/
void Rcs::BulletSimulation::setJointCompliance(const MatNd* stiffness,
                                               const MatNd* damping)
{
  RFATAL("Implement me");
}

/*******************************************************************************
 * Sets compliance properties for all hinge joints
 ******************************************************************************/
void Rcs::BulletSimulation::getJointCompliance(MatNd* stiffness,
                                               MatNd* damping) const
{
  RFATAL("Implement me");
}
