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

#include "PhysicsFactory.h"
#include "BulletSimulation.h"
#include "BulletRigidBody.h"
#include "BulletHingeJoint.h"
#include "BulletHelpers.h"

#ifndef HEADLESS_BUILD
#include "BulletDebugDrawer.h"
#endif

#include <Rcs_typedef.h>
#include <Rcs_macros.h>
#include <Rcs_math.h>
#include <Rcs_body.h>
#include <Rcs_shape.h>
#include <Rcs_joint.h>
#include <Rcs_sensor.h>
#include <Rcs_utils.h>
#include <Rcs_parser.h>
#include <Rcs_kinematics.h>

#include <BulletDynamics/MLCPSolvers/btDantzigSolver.h>
#include <BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h>
#include <BulletSoftBody/btSoftBody.h>

#include <iostream>
#include <climits>



static const char className[] = "Bullet";
static Rcs::PhysicsFactoryRegistrar<Rcs::BulletSimulation> physics(className);


typedef std::map<int, Rcs::BulletJointBase*>::iterator hinge_it;
typedef std::map<int, Rcs::BulletJointBase*>::const_iterator hinge_cit;
typedef std::map<int, Rcs::BulletRigidBody*>::iterator body_it;
typedef std::map<int, Rcs::BulletRigidBody*>::const_iterator body_cit;

struct MyCollisionDispatcher : public btCollisionDispatcher
{
  MyCollisionDispatcher(btDefaultCollisionConfiguration* cc, RcsGraph* graph) :
    btCollisionDispatcher(cc), graphPtr(graph)
  {
  }

  RcsGraph* graphPtr;
};

/*******************************************************************************
 * Callback for collision filtering. Do your collision logic here
 ******************************************************************************/
void Rcs::BulletSimulation::NearCallbackAllToAll(btBroadphasePair& collisionPair,
                                                 btCollisionDispatcher& dispatcher,
                                                 const btDispatcherInfo& dispatchInfo)
{
  dispatcher.defaultNearCallback(collisionPair, dispatcher, dispatchInfo);
}

/*******************************************************************************
 * Callback for collision filtering. Do your collision logic here
 ******************************************************************************/
void Rcs::BulletSimulation::MyNearCallbackDisabled(btBroadphasePair& collisionPair,
                                                   btCollisionDispatcher& dispatcher,
                                                   const btDispatcherInfo& dispatchInfo)
{
  return;
}

/*******************************************************************************
 * Callback for collision filtering. Do your collision logic here
 ******************************************************************************/
void Rcs::BulletSimulation::MyNearCallbackEnabled(btBroadphasePair& collisionPair,
                                                  btCollisionDispatcher& dispatcher,
                                                  const btDispatcherInfo& dispatchInfo)
{
  // Do your collision logic here
  btBroadphaseProxy* p0 = collisionPair.m_pProxy0;
  btBroadphaseProxy* p1 = collisionPair.m_pProxy1;

  if (p0->isSoftBody(SOFTBODY_SHAPE_PROXYTYPE) ||
      p1->isSoftBody(SOFTBODY_SHAPE_PROXYTYPE))
  {
    dispatcher.defaultNearCallback(collisionPair, dispatcher, dispatchInfo);
    return;
  }

  btCollisionObject* co0 = static_cast<btCollisionObject*>(p0->m_clientObject);
  btCollisionObject* co1 = static_cast<btCollisionObject*>(p1->m_clientObject);

  Rcs::BulletRigidBody* rb0 = dynamic_cast<Rcs::BulletRigidBody*>(co0);
  Rcs::BulletRigidBody* rb1 = dynamic_cast<Rcs::BulletRigidBody*>(co1);

  if ((rb0!=NULL) && (rb1!=NULL))
  {
    NLOG(0, "Broadphase collision between %s and %s",
         rb0->getBodyName(), rb1->getBodyName());

    MyCollisionDispatcher& myCD = dynamic_cast<MyCollisionDispatcher&>(dispatcher);

    const RcsBody* parent0 = RcsBody_getParent(myCD.graphPtr, (RcsBody*)rb0->getBodyPtr());
    const RcsBody* parent1 = RcsBody_getParent(myCD.graphPtr, (RcsBody*)rb1->getBodyPtr());

    if ((rb0->getBodyPtr()->rigid_body_joints==false && parent0) ||
        (rb1->getBodyPtr()->rigid_body_joints==false && parent1))
    {

      if ((RcsBody_isChild(myCD.graphPtr, rb0->getBodyPtr(), rb1->getBodyPtr())) ||
          (RcsBody_isChild(myCD.graphPtr, rb1->getBodyPtr(), rb0->getBodyPtr())))
      {
        NLOG(1, "Skipping %s - %s", rb0->getBodyName(), rb1->getBodyName());
        return;
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
Rcs::BulletSimulation::BulletSimulation() :
  Rcs::PhysicsBase(),
  dynamicsWorld(NULL),
  broadPhase(NULL),
  dispatcher(NULL),
  solver(NULL),
  mlcpSolver(NULL),
  collisionConfiguration(NULL),
  lastDt(0.001),
  dragBody(NULL),
  debugDrawer(NULL),
  rigidBodyLinearDamping(0.1),
  rigidBodyAngularDamping(0.9),
  jointedBodyLinearDamping(0.0),
  jointedBodyAngularDamping(0.0)
{
  pthread_mutex_init(&this->mtx, NULL);
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::BulletSimulation::BulletSimulation(const RcsGraph* graph_,
                                        const char* cfgFile) :
  Rcs::PhysicsBase(graph_),
  dynamicsWorld(NULL),
  broadPhase(NULL),
  dispatcher(NULL),
  solver(NULL),
  mlcpSolver(NULL),
  collisionConfiguration(NULL),
  lastDt(0.001),
  dragBody(NULL),
  debugDrawer(NULL),
  rigidBodyLinearDamping(0.1),
  rigidBodyAngularDamping(0.9),
  jointedBodyLinearDamping(0.0),
  jointedBodyAngularDamping(0.0)
{
  pthread_mutex_init(&this->mtx, NULL);

  PhysicsConfig config(cfgFile);
  initPhysics(&config);
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::BulletSimulation::BulletSimulation(const RcsGraph* graph_,
                                        const PhysicsConfig* config) :
  Rcs::PhysicsBase(graph_),
  dynamicsWorld(NULL),
  broadPhase(NULL),
  dispatcher(NULL),
  solver(NULL),
  mlcpSolver(NULL),
  collisionConfiguration(NULL),
  lastDt(0.001),
  dragBody(NULL),
  debugDrawer(NULL),
  rigidBodyLinearDamping(0.1),
  rigidBodyAngularDamping(0.9),
  jointedBodyLinearDamping(0.0),
  jointedBodyAngularDamping(0.0)
{
  pthread_mutex_init(&this->mtx, NULL);
  initPhysics(config);
}

/*******************************************************************************
 * Copy constructor
 ******************************************************************************/
Rcs::BulletSimulation::BulletSimulation(const BulletSimulation& copyFromMe):
  PhysicsBase(copyFromMe, copyFromMe.getGraph()),
  dynamicsWorld(NULL),
  broadPhase(NULL),
  dispatcher(NULL),
  solver(NULL),
  mlcpSolver(NULL),
  collisionConfiguration(NULL),
  lastDt(copyFromMe.lastDt),
  dragBody(NULL),
  debugDrawer(NULL),
  physicsConfigFile(copyFromMe.physicsConfigFile),
  rigidBodyLinearDamping(copyFromMe.rigidBodyLinearDamping),
  rigidBodyAngularDamping(copyFromMe.rigidBodyAngularDamping),
  jointedBodyLinearDamping(copyFromMe.jointedBodyLinearDamping),
  jointedBodyAngularDamping(copyFromMe.jointedBodyAngularDamping)
{
  pthread_mutex_init(&this->mtx, NULL);
  PhysicsConfig config(copyFromMe.physicsConfigFile.c_str());
  initPhysics(&config);
}

/*******************************************************************************
 * Copy constructor
 ******************************************************************************/
Rcs::BulletSimulation::BulletSimulation(const BulletSimulation& copyFromMe,
                                        const RcsGraph* newGraph):
  PhysicsBase(copyFromMe, newGraph),
  dynamicsWorld(NULL),
  broadPhase(NULL),
  dispatcher(NULL),
  solver(NULL),
  mlcpSolver(NULL),
  collisionConfiguration(NULL),
  lastDt(copyFromMe.lastDt),
  dragBody(NULL),
  debugDrawer(NULL),
  physicsConfigFile(copyFromMe.physicsConfigFile),
  rigidBodyLinearDamping(copyFromMe.rigidBodyLinearDamping),
  rigidBodyAngularDamping(copyFromMe.rigidBodyAngularDamping),
  jointedBodyLinearDamping(copyFromMe.jointedBodyLinearDamping),
  jointedBodyAngularDamping(copyFromMe.jointedBodyAngularDamping)
{
  pthread_mutex_init(&this->mtx, NULL);
  PhysicsConfig config(copyFromMe.physicsConfigFile.c_str());
  initPhysics(&config);
}

/*******************************************************************************
 * Cleanup in the reverse order of creation/initialization.
 ******************************************************************************/
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

    // The soft bodies need to be cleaned up separately. It is not so great
    // that we need to check for this here, since it somehow breaks the
    // whole idea of the object oriented design.
    if (dynamic_cast<btSoftBody*>(obj))
    {
      continue;
    }

    btRigidBody* body = btRigidBody::upcast(obj);

    // BulletRigidBody takes care of recursively deleting all shapes
    // Other shapes such as ground plane need explicit destruction of shapes
    if (!dynamic_cast<BulletRigidBody*>(body))
    {
      delete obj->getCollisionShape();
    }

    if (body)
    {
      if (body->getMotionState())
      {
        delete body->getMotionState();
      }
      dynamicsWorld->removeRigidBody(body);
      delete body;
    }
    else
    {
      dynamicsWorld->removeCollisionObject(obj);
      delete obj;
    }

  }

  delete this->dynamicsWorld;
  delete this->solver;
  delete this->broadPhase;
  delete this->dispatcher;
  delete this->collisionConfiguration;

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
 * Physics initialization
 ******************************************************************************/
bool Rcs::BulletSimulation::initialize(const RcsGraph* g,
                                       const PhysicsConfig* config)
{
  RCHECK(getGraph()==NULL);
  initGraph(g);
  initPhysics(config);
  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletSimulation::lock() const
{
  pthread_mutex_lock(&this->mtx);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletSimulation::unlock() const
{
  pthread_mutex_unlock(&this->mtx);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletSimulation::setNearCallback(btNearCallback nearCallback)
{
  dispatcher->setNearCallback(nearCallback);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletSimulation::initPhysics(const PhysicsConfig* config)
{
  RCHECK_MSG(getGraph(), "Graph not yet created. Did you call init()?");
  RCHECK(config->getConfigFileName());
  this->physicsConfigFile = std::string(config->getConfigFileName());

  // lookup bullet config node
  xmlNodePtr bulletParams = getXMLChildByName(config->getXMLRootNode(),
                                              "bullet_parameters");
  if (bulletParams == NULL)
  {
    RLOG(1, "Physics configuration file %s did not contain a "
         "\"bullet parameters\" node!", this->physicsConfigFile.c_str());
  }

  // Create discrete dynamics world etc. This function is overwritten in the
  // BulletSoftBody class.
  createWorld(bulletParams);

  if (bulletParams)
  {
    // load body params from xml
    getXMLNodePropertyDouble(bulletParams, "body_linear_damping",
                             &this->rigidBodyAngularDamping);
    getXMLNodePropertyDouble(bulletParams, "body_angular_damping",
                             &this->rigidBodyAngularDamping);
    getXMLNodePropertyDouble(bulletParams, "jointed_body_linear_damping",
                             &this->jointedBodyLinearDamping);
    getXMLNodePropertyDouble(bulletParams, "jointed_body_angular_damping",
                             &this->jointedBodyAngularDamping);
  }

  // Create physics for RcsGraph
  RCSGRAPH_TRAVERSE_BODIES(getGraph())
  {
    RLOGS(5, "Creating bullet body for \"%s\"", BODY->name);

    BulletRigidBody* btBody = BulletRigidBody::create(getGraph(), BODY, config);

    if (btBody!=NULL)
    {
      // apply configured damping
      if (BODY->rigid_body_joints)
      {
        btBody->setDamping(rigidBodyLinearDamping, rigidBodyAngularDamping);
      }
      else
      {
        btBody->setDamping(jointedBodyLinearDamping, jointedBodyAngularDamping);
      }

      bdyMap[BODY->id] = btBody;

      body_it it = bdyMap.find(BODY->parentId);

      if (it!=bdyMap.end())
      {
        btBody->setParentBody(it->second);
      }

      dynamicsWorld->addRigidBody(btBody);
      //dynamicsWorld->addCollisionObject(btBdy);

      btTypedConstraint* jnt = btBody->createJoint(getGraph());

      if (jnt!=NULL)
      {
        // Fixed joints don't have a RcsJoint pointer, this must be checked
        // before applying the offset
        if (BODY->jntId!=-1)
        {
          BulletJointBase* jBase = dynamic_cast<BulletJointBase*>(jnt);

          if (jBase != NULL)
          {
            jntMap[BODY->jntId] = jBase;
          }
        }

        dynamicsWorld->addConstraint(jnt, true);
      }

    }

    RLOGS(5, "%s adding \"%s\" to Bullet universe",
          btBody ? "SUCCESS" : "FAILURE", BODY->name);
  }

  // Create ground plane
  if (bulletParams)
  {
    // Check if ground plane is to be skipped
    bool useGroundPlane = true;
    getXMLNodePropertyBoolString(bulletParams, "use_ground_plane",
                                 &useGroundPlane);

    if (useGroundPlane)
    {
      btCollisionShape* gnd = new btStaticPlaneShape(btVector3(0.0, 0.0, 1.0),
                                                     0.0);
      btDefaultMotionState* gms = new btDefaultMotionState();
      btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0, gms, gnd);
      btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
      dynamicsWorld->addRigidBody(groundRigidBody);
    }
  }

  // Force dragging
  this->dragBody = NULL;
  Vec3d_setZero(this->dragForce);
  Vec3d_setZero(this->dragAnchor);

  // Update all transforms of the BulletRigidBody instance so that they show
  // the correct transformations before the first simulate() call.
  for (body_it it=bdyMap.begin(); it!=bdyMap.end(); ++it)
  {
    Rcs::BulletRigidBody* btBdy = it->second;
    btBdy->updateBodyTransformFromPhysics();
  }

  RCHECK(check());
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
  if (dt<=0.0)
  {
    return;
  }

  incrementTime(dt);
  this->lastDt = dt;

  // Get joint velocities before step to compute accelerations:
  // q_ddot = (q_dot-q_dot_prev)/dt
  if (q_ddot != NULL)
  {
    getJointVelocities(q_ddot);
    MatNd_constMulSelf(q_ddot, -1.0);
  }

  if (control==true)
  {
    applyControl(dt);
  }


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



#ifndef HEADLESS_BUILD
  if (this->debugDrawer != NULL)
  {
    debugDrawer->clear();
  }
#endif

  dynamicsWorld->stepSimulation(dt, 0);

#ifndef HEADLESS_BUILD
  if (this->debugDrawer != NULL)
  {
    dynamicsWorld->debugDrawWorld();
    debugDrawer->apply();
  }
#endif

  // This call only exists for compatibility with the BulletSoftSimulation
  // class, where mesh vertices are updated according to the soft physics
  // simulation. In this class, the function does nothing. It needs to be
  // called with a locked mutex, since otherwise, the viewer update will
  // be concurrent with the physics updates.
  // \todo: Rethink this with efficiency in mind.
  updateSoftMeshes();

  unlock();

  // \todo: Check if we ave some concurrency issue here for the FTSensorNode
  updateSensors();


  // Copy bullet's transformations into the BulletRigidBody instances.
  for (body_it it=bdyMap.begin(); it!=bdyMap.end(); ++it)
  {
    Rcs::BulletRigidBody* btBdy = it->second;
    btBdy->updateBodyTransformFromPhysics();
  }


  // Memorize joint reated values in joint classes
  for (hinge_it it = jntMap.begin(); it != jntMap.end(); ++it)
  {
    Rcs::BulletJointBase* hinge = it->second;
    hinge->update(dt);
  }


  // Read joint angles
  if (q != NULL)
  {
    if (q->m == getGraph()->dof)
    {
      getJointAngles(q, RcsStateFull);
    }
    else if (q->m == getGraph()->nJ)
    {
      getJointAngles(q, RcsStateIK);
    }
    else
    {
      RFATAL("Wrong dimensions of q-vector: [%d x %d], dof=%d nJ=%d",
             q->m, q->n, getGraph()->dof, getGraph()->nJ);
    }
  }


  // Read joint velocities
  if (q_dot != NULL)
  {
    if (q_dot->m == getGraph()->dof)
    {
      getJointVelocities(q_dot, RcsStateFull);
    }
    else if (q_dot->m == getGraph()->nJ)
    {
      getJointVelocities(q_dot, RcsStateIK);
    }
    else
    {
      RFATAL("Wrong dimensions of q_dot-vector: [%d x %d], dof=%d nJ=%d",
             q_dot->m, q_dot->n, getGraph()->dof, getGraph()->nJ);
    }
  }


  if (q_ddot != NULL)
  {
    bool q_dotOnHeap = false;

    if (q_dot == NULL)
    {
      q_dot = MatNd_create(q_ddot->m, q_ddot->n);
      q_dotOnHeap = true;
      getJointVelocities(q_dot);
    }
    MatNd_addSelf(q_ddot, q_dot);
    MatNd_constMulSelf(q_ddot, 1.0 / dt);
    if (q_dotOnHeap == true)
    {
      MatNd_destroy(q_dot);
    }
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletSimulation::reset()
{
  MatNd_copy(this->q_des, getGraph()->q);
  MatNd_setZero(getGraph()->q_dot);
  MatNd_setZero(this->q_dot_des);
  MatNd_setZero(this->T_des);

  // Update also the internal desired graph for the rigid body transforms.
  setControlInput(this->q_des, this->q_dot_des, this->T_des);

  btBroadphaseInterface* bi = dynamicsWorld->getBroadphase();
  btOverlappingPairCache* opc = bi->getOverlappingPairCache();

  for (hinge_it it = jntMap.begin(); it != jntMap.end(); ++it)
  {
    Rcs::BulletJointBase* hinge = it->second;
    hinge->reset(this->q_des->ele[hinge->getJointIndex()]);
  }

  for (body_it it=bdyMap.begin(); it!=bdyMap.end(); ++it)
  {
    Rcs::BulletRigidBody* btBdy = it->second;

    if (btBdy==NULL)
    {
      const RcsBody* rb = RCSBODY_BY_ID(getGraph(), it->first);
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

/*******************************************************************************
 *
 ******************************************************************************/
const HTr* Rcs::BulletSimulation::getPhysicsTransformPtr(const RcsBody* body) const
{
  Rcs::BulletRigidBody* bb = getRigidBody(body);

  if (bb == NULL)
  {
    return &body->A_BI;
  }

  return bb->getBodyTransformPtr();
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletSimulation::setForce(const RcsBody* body, const double F[3],
                                     const double p[3])
{
  RCHECK_MSG(body != NULL, "Cannot set force on a NULL body!");
  BulletRigidBody* rigidBody = getRigidBody(body);

  if (rigidBody != NULL)
  {
    if (p != NULL)
    {
      double relPos[3];
      Vec3d_sub(relPos, p, body->A_BI.org);

      rigidBody->applyForce(btVector3(F[0], F[1], F[2]),
                            btVector3(relPos[0], relPos[1], relPos[2]));
    }
    else
    {
      rigidBody->applyCentralForce(btVector3(F[0], F[1], F[2]));
    }
  }
  else
  {
    RLOG(1, "Could not find a physical body for RcsBody: \"%s\"", body->name);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletSimulation::applyImpulse(const RcsBody* body, const double F[3],
                                         const double p[3])
{
  RCHECK_MSG(body != NULL, "Cannot apply impulse to a NULL body!");
  BulletRigidBody* rigidBody = getRigidBody(body);

  if (rigidBody != NULL)
  {
    if (p != NULL)
    {
      double relPos[3];
      Vec3d_sub(relPos, p, body->A_BI.org);

      rigidBody->applyImpulse(btVector3(F[0], F[1], F[2]),
                              btVector3(relPos[0], relPos[1], relPos[2]));
    }
    else
    {
      rigidBody->applyCentralImpulse(btVector3(F[0], F[1], F[2]));
    }
  }
  else
  {
    RLOG(1, "Could not find a physical body for RcsBody: '%s'", body->name);
  }
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

    if (oldDragBody != NULL)
    {
      oldDragBody->applyCentralForce(btVector3(0.0, 0.0, 0.0));
      oldDragBody->setDamping(rigidBodyLinearDamping, rigidBodyAngularDamping);
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
  MatNd_reshape(q, (type==RcsStateFull) ? getGraph()->dof : getGraph()->nJ, 1);

#if 0
  // Update all hinge joints
  std::map<const RcsJoint*, Rcs::BulletHingeJoint*>::const_iterator jit;

  for (jit = jntMap.begin(); jit != jntMap.end(); ++jit)
  {
    const RcsJoint* rj = jit->first;
    Rcs::BulletHingeJoint* hinge = jit->second;

    if (hinge != NULL)
    {
      int idx = (type==RcsStateFull) ? rj->jointIndex : rj->jacobiIndex;
      RCHECK_MSG(idx>=0 && idx<(int)getGraph()->dof, "Joint \"%s\": idx = %d",
                 rj ? rj->name : "NULL", idx);
      q->ele[idx] = hinge->getJointPosition();
    }
    else
    {
      RLOG(5, "Skipping Updating %s", rj ? rj->name : "NULL");
    }

  }
#else
  RCSGRAPH_TRAVERSE_JOINTS(getGraph())
  {
    int idx = (type==RcsStateFull) ? JNT->jointIndex : JNT->jacobiIndex;
    RCHECK_MSG(idx>=0 && idx<(int)getGraph()->dof, "Joint \"%s\": idx = %d",
               JNT->name, idx);

    Rcs::BulletJointBase* hinge = getHinge(JNT);

    if (hinge != NULL)
    {
      q->ele[idx] = hinge->getJointPosition();
    }
    else
    {
      NLOG(5, "Updating kinematic joint %s", JNT->name);
      q->ele[idx] = this->q_des->ele[JNT->jointIndex];
    }

  }
#endif

  // Then update all rigid body dofs
  for (body_cit it=bdyMap.begin(); it!=bdyMap.end(); ++it)
  {
    const RcsBody* rb = RCSBODY_BY_ID(getGraph(), it->first);

    if (rb && rb->rigid_body_joints == true)
    {
      Rcs::BulletRigidBody* btBdy = it->second;
      RCHECK_MSG(btBdy, "%s", rb->name);
      HTr A_BI;
      btBdy->getBodyTransform(&A_BI);
      const RcsJoint* jnt = &getGraph()->joints[rb->jntId];
      int idx = (type==RcsStateFull) ? jnt->jointIndex : jnt->jacobiIndex;
      RCHECK_MSG(idx>=0 && idx<(int)getGraph()->dof, "Joint \"%s\": idx = %d",
                 jnt ? jnt->name : "NULL", idx);

      // Transformation update: Here we compute the rigid body degrees of
      // freedom so that the state vector matches the results from physics.
      // We need to distinguish between the case of no parent body, and if a
      // parent body exists. In the latter case, the transformation must be
      // relative between two bodies.
      HTr A_ParentI;
      //RcsBody* rcsParent = rb->parent;
      const RcsBody* rcsParent = RCSBODY_BY_ID(getGraph(), rb->parentId);

      if (rcsParent != NULL)
      {
        Rcs::BulletRigidBody* btParent = getRigidBody(rcsParent);

        // If the parent body is simulated in Bullet, we take it's simulated
        // transformation.
        if (btParent != NULL)
        {
          btParent->getBodyTransform(&A_ParentI);
        }
        // Otherwise we use the kinematic transformation.
        else
        {
          HTr_copy(&A_ParentI, &rcsParent->A_BI);
        }
      }
      else
      {
        HTr_setIdentity(&A_ParentI);
      }

      // Simply project relative transform on 6d vector. This leads to jumps
      // in the angles when leaving the Euler Angles representation ranges.
      // Alternatively, we could determine the angular velocity and integrate
      // it.
      RcsGraph_relativeRigidBodyDoFs(getGraph(), rb, &A_BI, &A_ParentI, &q->ele[idx]);

    }   // if (rb->rigid_body_joints == true)
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletSimulation::getJointVelocities(MatNd* q_dot,
                                               RcsStateType type) const
{
  int nq = (type==RcsStateFull) ? getGraph()->dof : getGraph()->nJ;
  MatNd_reshape(q_dot, nq, 1);

  // First update all hinge joints
  for (hinge_cit it = jntMap.begin(); it != jntMap.end(); ++it)
  {
    const RcsJoint* rj = &getGraph()->joints[it->first];
    Rcs::BulletJointBase* hinge = it->second;

    if (hinge != NULL)
    {
      int idx = (type==RcsStateFull) ? rj->jointIndex : rj->jacobiIndex;

      if (idx!=-1)
      {
        q_dot->ele[idx] = hinge->getJointVelocity();
      }

    }

  }

  // Update the rigid body dofs. Since they are constrained, it is only done
  // for the full state vector.
  for (body_cit it2=bdyMap.begin(); it2!=bdyMap.end(); ++it2)
  {
    const RcsBody* rb = RCSBODY_BY_ID(getGraph(), it2->first);

    if (rb && rb->rigid_body_joints == true)
    {
      const Rcs::BulletRigidBody* btBdy = it2->second;
      const RcsJoint* jnt = &getGraph()->joints[rb->jntId];
      int idx = (type==RcsStateFull) ? jnt->jointIndex : jnt->jacobiIndex;

      if (idx == -1)
      {
        continue;
      }

      MatNd_set(q_dot, idx, 0, btBdy->x_dot[0]);
      MatNd_set(q_dot, idx+1, 0, btBdy->x_dot[1]);
      MatNd_set(q_dot, idx+2, 0, btBdy->x_dot[2]);
      MatNd_set(q_dot, idx+3, 0, btBdy->omega[0]);
      MatNd_set(q_dot, idx+4, 0, btBdy->omega[1]);
      MatNd_set(q_dot, idx+5, 0, btBdy->omega[2]);
    }
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletSimulation::setJointTorque(const RcsJoint* jnt, double torque)
{
  Rcs::BulletJointBase* hinge = getHinge(jnt);

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
  Rcs::BulletJointBase* hinge = getHinge(jnt);

  if (hinge == NULL)
  {
    NLOG(5, "Couldn't find hinge joint for joint \"%s\"",
         jnt ? jnt->name : "NULL");
    return false;
  }

  hinge->setJointPosition(angle, dt);

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletSimulation::setJointTorque(const MatNd* T_des)
{
  RCSGRAPH_TRAVERSE_JOINTS(getGraph())
  {
    int idx = (T_des->m==getGraph()->dof) ? JNT->jointIndex : JNT->jacobiIndex;
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
  dispatcher->setNearCallback(MyNearCallbackDisabled);
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
void Rcs::BulletSimulation::setJointLimits(bool enable)
{
  for (hinge_it it = jntMap.begin(); it != jntMap.end(); ++it)
  {
    Rcs::BulletJointBase* hinge = it->second;

    if (enable==true)
    {
      const RcsJoint* jnt = &getGraph()->joints[it->first];
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

    int numContacts = contactManifold->getNumContacts();

#if 0
    const btCollisionObject* obA =
      static_cast<const btCollisionObject*>(contactManifold->getBody0());
    const btCollisionObject* obB =
      static_cast<const btCollisionObject*>(contactManifold->getBody1());

    const Rcs::BulletRigidBody* rbA = dynamic_cast<const Rcs::BulletRigidBody*>(obA);
    const Rcs::BulletRigidBody* rbB = dynamic_cast<const Rcs::BulletRigidBody*>(obB);

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
    btPersistentManifold* contactManifold =
      this->dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
    numContacts += contactManifold->getNumContacts();
  }

  return numContacts;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletSimulation::applyControl(double dt)
{
  // Apply transformations to all kinematic bodies
  for (body_cit it = bdyMap.begin(); it!=bdyMap.end(); ++it)
  {
    BulletRigidBody* btBdy = it->second;
    if (btBdy->isStaticOrKinematicObject())
    {
      const RcsBody* rb = RCSBODY_BY_ID(getGraph(), it->first);
      btBdy->setBodyTransform(&rb->A_BI, dt);
    }
  }



  // Set desired joint controls
  for (hinge_cit it = jntMap.begin(); it != jntMap.end(); ++it)
  {
    const RcsJoint* JNT = &getGraph()->joints[it->first];
    Rcs::BulletJointBase* hinge = it->second;

    if (JNT->ctrlType==RCSJOINT_CTRL_TORQUE)
    {
      double torque = MatNd_get(T_des, JNT->jointIndex, 0);
      hinge->setJointTorque(torque, dt);
    }
    else if (JNT->ctrlType==RCSJOINT_CTRL_POSITION)
    {
      double q_cmd = MatNd_get(this->q_des, JNT->jointIndex, 0);
      hinge->setJointPosition(q_cmd, dt);
    }
    else   // RCSJOINT_CTRL_VELOCITY
    {
      double q_curr_i = hinge->getJointPosition();
      double q_dot_des_i = MatNd_get(this->q_dot_des, JNT->jointIndex, 0);
      double q_des_i  = q_curr_i + q_dot_des_i*dt;
      hinge->setJointPosition(q_des_i, dt);
    }
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::BulletSimulation::updateLoadcell(const RcsSensor* fts)
{
  RcsBody* ftsBdy = &getGraph()->bodies[fts->bodyId];
  Rcs::BulletRigidBody* rb = getRigidBody(ftsBdy);

  if (rb == NULL)
  {
    RLOG(5, "No BulletRigidBody found for RcsBody \"%s\"", ftsBdy->name);
    return false;
  }

  btFixedConstraint* jnt =rb->fixedJnt;

  if (jnt == NULL)
  {
    RLOG(1, "Load cell of body \"%s\" is not attached to joint",
         ftsBdy->name);
    return false;
  }

  const btJointFeedback* jf = jnt->getJointFeedback();

  if (jf==NULL)
  {
    RLOG(1, "No joint feedback found for RcsBody \"%s\"", ftsBdy->name);
    return false;
  }

  if (fts->rawData->size<6)
  {
    RLOG(1, "Data size mismatch for loadcell in  RcsBody \"%s\": size is %d",
         ftsBdy->name, fts->rawData->size);
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
  const HTr* A_SB = &fts->A_SB; // Body -> Sensor
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
  RcsSensor_computeStaticForceCompensation(getGraph(), fts, S_f_gravity);
  VecNd_subSelf(ftWrench, S_f_gravity, 6);

  // Copy into sensor's rawData array
  MatNd_fromArray(fts->rawData, ftWrench, 6);

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletSimulation::updateSensors()
{
  for (unsigned int i=0; i<getGraph()->nSensors; ++i)
  {
    RcsSensor* si = &getGraph()->sensors[i];

    switch (si->type)
    {
      case RCSSENSOR_LOAD_CELL:
        updateLoadcell(si);
        break;

      case RCSSENSOR_PPS:
        if (getEnablePPS()==true)
        {
          updatePPSSensor(si);
        }
        break;

      default:
        NLOG(5, "No update function for sensor type %d", si->type);
    }
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::BulletRigidBody* Rcs::BulletSimulation::getRigidBody(const RcsBody* bdy) const
{
  if (bdy==NULL)
  {
    return NULL;
  }

  body_cit it = bdyMap.find(bdy->id);

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
Rcs::BulletJointBase* Rcs::BulletSimulation::getHinge(const RcsJoint* jnt) const
{
  if (jnt == NULL)
  {
    return NULL;
  }

  hinge_cit it = jntMap.find(jnt->id);

  if (it!=jntMap.end())
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
    for (body_cit it = bdyMap.begin(); it!=bdyMap.end(); ++it)
    {
      const RcsBody* rb =  RCSBODY_BY_ID(getGraph(), it->first);
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
#ifndef HEADLESS_BUILD
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

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::BulletSimulation::setParameter(ParameterCategory category,
                                         const char* name,
                                         const char* type,
                                         double value)
{
  bool success = false;

  if (name==NULL || type==NULL)
  {
    RLOG(1, "Parameter name or type is NULL");
    return false;
  }

  lock();

  switch (category)
  {

    case Simulation:
    {
      if (STRCASEEQ(name, "gravity"))
      {
        btVector3 gravity = dynamicsWorld->getGravity();

        if (STRCASEEQ(type, "x"))
        {
          gravity[0] = value;
          dynamicsWorld->setGravity(gravity);
          success = true;
        }
        else if (STRCASEEQ(type, "y"))
        {
          gravity[1] = value;
          dynamicsWorld->setGravity(gravity);
          success = true;
        }
        else if (STRCASEEQ(type, "z"))
        {
          gravity[2] = value;
          dynamicsWorld->setGravity(gravity);
          success = true;
        }
      }
      break;
    }

    case Body:
    {
      const RcsBody* bdy = RcsGraph_getBodyByName(getGraph(), name);
      BulletRigidBody* btBdy = getRigidBody(bdy);

      if ((bdy!=NULL) && (btBdy!=NULL))
      {
        // Rigid bodies need to be removed from the simulation if their
        // parameters are to be changed (at least for some parameters).
        dynamicsWorld->removeRigidBody(btBdy);

        if (STRCASEEQ(type, "mass"))
        {
          btScalar oldMass = 1.0/btBdy->getInvMass();
          btScalar scaling = value/oldMass;

          const btVector3& invI = btBdy->getInvInertiaDiagLocal();
          btVector3 newInertia;
          newInertia[0] = scaling/invI[0];
          newInertia[1] = scaling/invI[1];
          newInertia[2] = scaling/invI[2];
          btBdy->setMassProps(value, newInertia);
          btBdy->updateInertiaTensor();   // Updates inertia in world frame
          success = true;
        }
        else if (STRCASEEQ(type, "friction"))
        {
          btBdy->setFriction(value);
          success = true;
        }
        else if (STRCASEEQ(type, "rolling_friction"))
        {
          btBdy->setRollingFriction(value);
          success = true;
        }
        else if (STRCASEEQ(type, "restitution"))
        {
          btBdy->setRestitution(value);
          success = true;
        }
        else if (STRCASEEQ(type, "linear_damping"))
        {
          btBdy->setDamping(value, btBdy->getAngularDamping());
          success = true;
        }
        else if (STRCASEEQ(type, "radius"))
        {
          RcsShape* ball = NULL;
          RCSBODY_TRAVERSE_SHAPES(bdy)
          {
            if (SHAPE->type==RCSSHAPE_SPHERE)
            {
              ball = SHAPE;
            }
          }

          if (ball==NULL)
          {
            RLOG(4, "No RCSSHAPE_SPHERE found in body \"%s\"", name);
            success = false;
          }
          else
          {
            btSphereShape* sphere =
              dynamic_cast<btSphereShape*>(btBdy->getShape(ball));

            if (sphere==NULL)
            {
              RLOG(4, "No btSphereShape found in body \"%s\"", name);
              success = false;
            }
            else
            {
              sphere->setUnscaledRadius(value);
              ball->extents[0] = value;
              // \todo: Don't we need the below lies?
              RFATAL("FIXME");
              //              btBdy = BulletRigidBody::create(bdy);
              //              bdyMap[bdy] = btBdy;

              success = true;
            }

          }   // ball != NULL

        }
        else if (STRCASEEQ(type, "angular_damping"))
        {
          btBdy->setDamping(btBdy->getLinearDamping(), value);
          success = true;
        }

        // Add body again
        dynamicsWorld->addRigidBody(btBdy);
      }
      else
      {
        RLOG(1, "Unknown body \"%s\"", name);
      }
      break;
    }

    case Joint:
    {
      break;
    }

    default:
    {
      RLOG(1, "Unknown parameter category: %d", category);
      break;
    }

  }   // switch

  unlock();

  if (success==false)
  {
    RLOG(1, "Failed to set parameter with name \"%s\" and type \"%s\" (value"
         " %g, category %d)", name, type, value, category);
  }

  return success;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletSimulation::getWorldBoundingBox(btVector3& aabbMin,
                                                btVector3& aabbMax) const
{
  dynamicsWorld->getBroadphase()->getBroadphaseAabb(aabbMin, aabbMax);
  NLOG(5, "Broadphase is %.3f %.3f %.3f - %.3f %.3f %.3f",
       aabbMin[0], aabbMin[1], aabbMin[2],
       aabbMax[0], aabbMax[1], aabbMax[2]);
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::BulletSimulation::updatePPSSensor(RcsSensor* sensor)
{
  double contactForce[3];
  Vec3d_setZero(contactForce);

  int numManifolds = this->dynamicsWorld->getDispatcher()->getNumManifolds();

  for (int i=0; i<numManifolds; i++)
  {
    btPersistentManifold* contactManifold =
      this->dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);

    const Rcs::BulletRigidBody* rbA = dynamic_cast<const Rcs::BulletRigidBody*>(contactManifold->getBody0());
    const Rcs::BulletRigidBody* rbB = dynamic_cast<const Rcs::BulletRigidBody*>(contactManifold->getBody1());

    double signOfForce;

    if (rbA && rbA->getBodyPtr()->id==sensor->bodyId)
    {
      signOfForce = 1.0;
    }
    else if (rbB && rbB->getBodyPtr()->id==sensor->bodyId)
    {
      signOfForce = -1.0;
    }
    else
    {
      continue;
    }

    for (int j=0; j<contactManifold->getNumContacts(); j++)
    {
      btManifoldPoint& pt = contactManifold->getContactPoint(j);

      if (pt.getDistance()<0.0)
      {
        double force[3];

        for (int i=0; i<3; ++i)
        {
          force[i] = (pt.m_normalWorldOnB[i]*pt.m_appliedImpulse +
                      pt.m_lateralFrictionDir1[i]*pt.m_appliedImpulseLateral1 +
                      pt.m_lateralFrictionDir2[i]*pt.m_appliedImpulseLateral2)/this->lastDt;
        }

        NLOG(1, "Adding f[%s,%d]=%f %f %f", sensor->body->name, j,
             force[0], force[1], force[2]);

        Vec3d_constMulAndAddSelf(contactForce, force, signOfForce);
      }

    }   // for (int j=0; j<contactManifold->getNumContacts(); j++)

  }   // for (int i=0; i<numManifolds; i++)

  return RcsSensor_computePPS(getGraph(), sensor, sensor->rawData, contactForce);
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::BulletSimulation::removeBody(const char* name)
{
  RcsBody* bdy = RcsGraph_getBodyByName(getGraph(), name);
  if (bdy==NULL)
  {
    RLOG(1, "Couldn't find body \"%s\" in graph - skipping removal",
         name ? name : "NULL");
    return false;
  }

  body_it it = bdyMap.find(bdy->id);

  if (it==bdyMap.end())
  {
    RLOG(1, "Couldn't find body \"%s\" in bdyMap - skipping removal",
         name ? name : "NULL");
    return false;
  }

  BulletRigidBody* btBdy = it->second;
  lock();
  btBdy->clearCompoundShapes();
  dynamicsWorld->removeRigidBody(btBdy);
  bdyMap.erase(it);
  delete btBdy;


  MatNd* arrBuf[3];
  arrBuf[0] = this->q_des;
  arrBuf[1] = this->q_dot_des;
  arrBuf[2] = this->T_des;
  RcsGraph_removeBody(getGraph(), name, arrBuf, 3);

  unlock();

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::BulletSimulation::addBody(const RcsGraph* graph, const RcsBody* body_)
{
  RLOG(5, "Creating bullet body for \"%s\"", body_->name);

  PhysicsConfig config(this->physicsConfigFile.c_str());

  lock();

  RLOG(5, "Copying graph body");
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


  RLOG(5, "Creating BulletRigidBody");
  BulletRigidBody* btBody = BulletRigidBody::create(getGraph(), body, &config);
  RCHECK(btBody);

  // Continuous collision detection
  // btBody->setCcdSweptSphereRadius(0.05);
  // btBody->setCcdMotionThreshold(0.0);

  RLOG(5, "Applying damping");
  if (btBody != NULL)
  {
    // apply configured damping
    if (body->rigid_body_joints)
    {
      btBody->setDamping(rigidBodyLinearDamping, rigidBodyAngularDamping);
    }
    else
    {
      btBody->setDamping(jointedBodyLinearDamping, jointedBodyAngularDamping);
    }

    bdyMap[body->id] = btBody;

    body_it it = bdyMap.find(body->parentId);

    if (it != bdyMap.end())
    {
      btBody->setParentBody(it->second);
    }

    RLOG(5, "Adding to dynamicsworld");
    dynamicsWorld->addRigidBody(btBody);

    btTypedConstraint* jnt = btBody->createJoint(getGraph());
    RCHECK(jnt==NULL);

    if (jnt != NULL)
    {
      // Fixed joints don't have a RcsJoint pointer, this must be checked
      // before applying the offset
      if (body->jntId != -1)
      {
        BulletJointBase* jBase = dynamic_cast<BulletJointBase*>(jnt);

        if (jBase != NULL)
        {
          jntMap[body->jntId] = jBase;
          const RcsJoint* bodyJnt = RCSJOINT_BY_ID(getGraph(), body->jntId);
          RLOGS(5, "Joint %s has value %f (%f)",
                bodyJnt->name, jBase->getJointPosition(),
                MatNd_get(getGraph()->q, bodyJnt->jointIndex, 0));
        }
      }

      dynamicsWorld->addConstraint(jnt, true);
    }

  }


  MatNd* arrBuf[3];
  arrBuf[0] = this->q_des;
  arrBuf[1] = this->q_dot_des;
  arrBuf[2] = this->T_des;

  unsigned int nJoints = RcsBody_numJoints(getGraph(), body);

  if (nJoints > 0)
  {
    this->T_des = MatNd_realloc(this->T_des, getGraph()->dof, 1);
    this->q_des = MatNd_realloc(this->q_des, getGraph()->dof, 1);
    this->q_dot_des = MatNd_realloc(this->q_dot_des, getGraph()->dof, 1);
  }

  RcsGraph_addBodyDofs(getGraph(), NULL, body, arrBuf, 3);

  unlock();

  RLOG(5, "%s adding \"%s\" to Bullet universe",
       btBody ? "SUCCESS" : "FAILURE", body->name);

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::BulletSimulation::deactivateBody(const char* name)
{
  RcsBody* bdy = RcsGraph_getBodyByName(getGraph(), name);
  if (bdy == NULL)
  {
    RLOG(1, "Couldn't find body \"%s\" in graph - skipping deactivation",
         name ? name : "NULL");
    return false;
  }

  if (!RcsBody_isLeaf(bdy))
  {
    RLOG(1, "Can't deactivate non-leaf body \"%s\" - skipping", bdy->name);
    return false;
  }

  body_it it = bdyMap.find(bdy->id);
  BulletRigidBody* part = NULL;

  if (it != bdyMap.end())
  {
    part = it->second;
  }
  else
  {
    RLOG(1, "Can't find BulletRigidBody for body \"%s\" - skipping "
         "deactivation", bdy->name);
    return false;
  }

  dynamicsWorld->removeRigidBody(part);
  bdyMap.erase(it);

  this->deactivatedBodies[name] = part;

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::BulletSimulation::activateBody(const char* name, const HTr* A_BI)
{
  if (name == NULL)
  {
    RLOG(1, "Can't retrieve body with name pointing to NULL");
    return false;
  }

  std::map<std::string, BulletRigidBody*>::iterator it;
  it = deactivatedBodies.find(std::string(name));

  if (it == deactivatedBodies.end())
  {
    RLOG(1, "Body \"%s\" is not deactivated - skipping activation", name);
    return false;
  }

  BulletRigidBody* part = it->second;
  const RcsBody* bdy = RcsGraph_getBodyByName(getGraph(), it->first.c_str());
  part->reset(A_BI);

  dynamicsWorld->addRigidBody(part);

  bdyMap[bdy->id] = part;
  deactivatedBodies.erase(it);

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::BulletSimulation::check() const
{
  bool success = PhysicsBase::check();

  // Check for valid pairs in joint map
  if (!jntMap.empty())
  {
    for (hinge_cit it = jntMap.begin(); it != jntMap.end(); ++it)
    {
      if ((it->first<0) || (it->first>=(int)getGraph()->dof))
      {
        RLOG_CPP(1, "Joint id out of range: " << it->first);
        success = false;
      }

      Rcs::BulletJointBase* hinge = it->second;

      if (hinge == NULL)
      {
        RLOG_CPP(1, "Bullet joint for id  " << it->first << " is NULL "
                 << getGraph()->joints[it->first].name);
        success = false;
      }

    }

  }


  if (!bdyMap.empty())
  {
    for (body_cit it = bdyMap.begin(); it != bdyMap.end(); ++it)
    {
      if (it->first==-1)
      {
        RLOG(1, "Found bdyMap key with -1");
        success = false;
      }

      BulletRigidBody* btBdy = it->second;

      if (btBdy==NULL)
      {
        RLOG(1, "Found bdyMap value with NULL at id %d", it->first);
        success = false;
        continue;
      }

      // Here we check that objects that are labelled static or dynamic in
      // Bullet correspond to an RcsBody with the RCSBODY_PHYSICS_KINEMATIC
      // property. That's the only ones that should be static or kinematic.
      if (btBdy && btBdy->isStaticOrKinematicObject())
      {
        if (btBdy->getBodyPtr()->physicsSim != RCSBODY_PHYSICS_KINEMATIC)
        {
          RLOG(1, "Body \"%s\" is not kinematic in the RcsBody properties but "
               "somehow got kinematic in Bullet - did you forget to assign "
               "a mass?", btBdy->getBodyName());
          success = false;
        }
      }

      // Here we check that all bodies within the body map are connected through
      // 6 consecutive joints (not necessarily having the RcsBody::rigid_body_jnt
      // property)
      //if (RcsBody_isFloatingBase(rb) == false)
      //{
      //  RLOG(1, "Body \"%s\" does not have six consecutive joints but somehow got"
      //       " into the Bullet bdyMap", btBdy->getBodyName());
      //  success = false;
      //}

      // Check velocity vector forward kinematics against body velocities from
      // physics.
      const RcsBody* bdy = btBdy->getBodyPtr();

      RcsGraph* tmp = RcsGraph_clone(getGraph());
      RCHECK(tmp);
      MatNd* q_dot = MatNd_create(tmp->nJ, 1);
      MatNd* J_lin = MatNd_create(3, tmp->nJ);
      MatNd* J_rot = MatNd_create(3, tmp->nJ);
      MatNd* x_dot = MatNd_create(3, 1);
      MatNd* omega = MatNd_create(3, 1);

      getJointAngles(tmp->q);
      RcsGraph_setState(tmp, NULL, NULL);
      getJointVelocities(q_dot, RcsStateIK);
      RcsGraph_rotationJacobian(tmp, bdy, NULL, J_rot);
      RcsGraph_worldPointJacobian(tmp, bdy, NULL, NULL, J_lin);
      MatNd_mul(x_dot, J_lin, q_dot);
      MatNd_mul(omega, J_rot, q_dot);

      REXEC(1)
      {
        RLOG(0, "Body %s:", btBdy->getBodyName());
        RLOG(0, "x_dot: [%f %f %f] vs [%f %f %f] err=%.2f %%",
             x_dot->ele[0], x_dot->ele[1], x_dot->ele[2],
             btBdy->x_dot[0], btBdy->x_dot[1], btBdy->x_dot[2],
             100.0*Vec3d_distance(x_dot->ele, btBdy->x_dot)/(Vec3d_getLength(btBdy->x_dot)+0.001));

        RLOG(0, "omega: [%f %f %f] vs [%f %f %f] err=%.2f %%",
             omega->ele[0], omega->ele[1], omega->ele[2],
             btBdy->omega[0], btBdy->omega[1], btBdy->omega[2],
             100.0*Vec3d_distance(omega->ele, btBdy->omega)/(Vec3d_getLength(btBdy->omega)+0.001));
      }

      MatNd_destroy(q_dot);
      MatNd_destroy(J_lin);
      MatNd_destroy(J_rot);
      MatNd_destroy(x_dot);
      MatNd_destroy(omega);

      RcsGraph_destroy(tmp);
    }

  }

  success = PhysicsBase::check() && success;

  return success;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletSimulation::createWorld(xmlNodePtr bulletParams)
{
  bool useMCLPSolver = false;

  if (bulletParams)
  {
    // load solver type from xml
    getXMLNodePropertyBoolString(bulletParams, "use_mclp_solver",
                                 &useMCLPSolver);
  }

  this->collisionConfiguration = new btDefaultCollisionConfiguration();
  //this->dispatcher = new btCollisionDispatcher(collisionConfiguration);
  this->dispatcher = new MyCollisionDispatcher(collisionConfiguration, getGraph());
  dispatcher->setNearCallback(MyNearCallbackEnabled);
  broadPhase = new btDbvtBroadphase();

  if (useMCLPSolver)
  {
    this->mlcpSolver = new btDantzigSolver();
    //this->mlcpSolver = new btSolveProjectedGaussSeidel;
    solver = new btMLCPSolver(this->mlcpSolver);
    RLOG(5, "Using MCLP solver");
  }
  else
  {
    solver = new btSequentialImpulseConstraintSolver;
    RLOG(5, "Using sequential impulse solver");
  }

  this->dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadPhase, solver,
                                                    collisionConfiguration);
  dynamicsWorld->setGravity(btVector3(0.0, 0.0, -RCS_GRAVITY));

  btDispatcherInfo& di = dynamicsWorld->getDispatchInfo();
  di.m_useConvexConservativeDistanceUtil = true;
  di.m_convexConservativeDistanceThreshold = 0.01f;

  btContactSolverInfo& si = dynamicsWorld->getSolverInfo();
  si.m_numIterations = 200;//20;
  // ERP: 0: no joint error correction (Recommended: 0.1-0.8, default 0.2)
  si.m_erp = 0.2;
  si.m_globalCfm = 1.0e-4; // 0: hard constraint (Default)
  si.m_restingContactRestitutionThreshold = INT_MAX;
  si.m_splitImpulse = 1;
  si.m_solverMode = SOLVER_RANDMIZE_ORDER |
                    SOLVER_FRICTION_SEPARATE |
                    SOLVER_USE_2_FRICTION_DIRECTIONS |
                    SOLVER_USE_WARMSTARTING;
  si.m_minimumSolverBatchSize = useMCLPSolver ? 1 : 128;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletSimulation::updateSoftMeshes()
{
}
