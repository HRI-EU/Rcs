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

#ifndef RCS_BULLETSIMULATION_H
#define RCS_BULLETSIMULATION_H

#include "PhysicsConfig.h"

#include <PhysicsBase.h>

#include <btBulletDynamicsCommon.h>
#include <BulletDynamics/MLCPSolvers/btMLCPSolver.h>

#include <pthread.h>
#include <map>
#include <string>


namespace Rcs
{
class BulletRigidBody;
class BulletJointBase;
class BulletDebugDrawer;

/*! \ingroup RcsPhysics
 *  \brief Bullet physics simulation class.
 *
 *  To consider:
 *  - currently joint limits do not work for joints with ranges larger than
 *    360 degrees. This is checked during construction, and a warning is
 *    issued on debug level 1.
 */
class BulletSimulation : public PhysicsBase
{
public:

  BulletSimulation();
  BulletSimulation(const RcsGraph* graph, const char* configFile=NULL);
  BulletSimulation(const RcsGraph* graph, const PhysicsConfig* config);
  BulletSimulation(const BulletSimulation& copyFromMe);
  BulletSimulation(const BulletSimulation& copyFromMe,
                   const RcsGraph* newGraph);
  virtual ~BulletSimulation();

  /**
  * @name InheritanceInterface
  *
  * Inheritance interface
  */

  ///@{


  /*! \copydoc PhysicsBase::step(double)
   */
  virtual void step(double dt);
  virtual void simulate(double dt, MatNd* q =NULL, MatNd* q_dot=NULL,
                        MatNd* q_ddot=NULL, MatNd* T=NULL, bool control=false);
  virtual void setGravity(const double gravity[3]);
  virtual void reset();
  virtual const char* getClassName() const;
  virtual void setForce(const RcsBody* body, const double F[3],
                        const double p[3]);
  virtual void applyImpulse(const RcsBody* body, const double F[3],
                            const double p[3]);
  virtual void applyForce(const RcsBody* body, const double F[3],
                          const double r[3]);
  virtual void applyTransform(const RcsBody* body, const HTr* A_BI);
  virtual void applyLinearVelocity(const RcsBody* body, const double v[3]);
  virtual void applyAngularVelocity(const RcsBody* body, const double v[3]);
  virtual void getLinearVelocity(const RcsBody* body, double v[3]) const;
  virtual void getAngularVelocity(const RcsBody* body, double omega[3]) const;
  void setJointTorque(const MatNd* T_des);
  virtual void getJointTorque(MatNd* T_curr,
                              RcsStateType type=RcsStateFull) const;
  virtual void getJointAngles(MatNd* q,
                              RcsStateType type=RcsStateFull) const;
  virtual void getJointVelocities(MatNd* q_dot,
                                  RcsStateType type=RcsStateFull) const;
  virtual void setMassAndInertiaFromPhysics(RcsGraph* graph);
  virtual void getPhysicsTransform(HTr* A_BI, const RcsBody* body) const;
  virtual const HTr* getPhysicsTransformPtr(const RcsBody* body) const;
  virtual void disableCollision(const RcsBody* b0, const RcsBody* b1);
  virtual void disableCollisions();
  virtual Contacts getContacts();
  virtual void setJointLimits(bool enable);
  virtual BulletSimulation* clone(RcsGraph* newGraph=NULL) const;
  virtual void setJointCompliance(const MatNd* stiffness,
                                  const MatNd* damping=NULL);
  virtual void getJointCompliance(MatNd* stiffness, MatNd* damping=NULL) const;
  virtual bool initialize(const RcsGraph* g, const PhysicsConfig* config);

  ///@}








  /*! \brief Function for setting physics parameter. The function is thread-
   *         safe. All strings are interpreted case insensitive. The following
   *         parmeter types are supported:
   *
   *         - category "Simulation":
   *           - name: gravity
   *           - type:
   *             - x
   *             - y
   *             - z
   *         - category "Body":
   *           - name: Name of the body
   *           - type:
   *             - mass
   *             - restitution
   *             - friction
   *             - rolling_friction
   *             - linear_damping
   *             - angular_damping
   *             - radius
   *
   *         For the category "Simulation", the argument "name" is not needed
   *         and is ignored (you may use NULL).
   *
   *  \param[in] category   See enum ParameterCategory
   *  \param[in] name   Parameter name, such as "SoftMaterial".
   *  \param[in] type   Parameter type, such as "Restitution".
   *  \param[in] value  Value the parameter should be assigned with
   *  \return true for success, false otherwise
   */
  virtual bool setParameter(ParameterCategory category,
                            const char* name, const char* type, double value);

  virtual size_t getNumberOfContacts() const;
  virtual void print() const;
  virtual void lock() const;
  virtual void unlock() const;
  virtual void setDebugDrawer(BulletDebugDrawer* drawer);
  virtual BulletDebugDrawer* getDebugDrawer() const;
  virtual BulletRigidBody* getRigidBody(const RcsBody* bdy) const;
  virtual void getWorldBoundingBox(btVector3& aabbMin,
                                   btVector3& aabbMax) const;
  virtual bool addBody(const RcsGraph* graph, const RcsBody* body);
  virtual bool removeBody(const char* name);
  virtual bool deactivateBody(const char* name);
  virtual bool activateBody(const char* name, const HTr* A_BI = NULL);

  virtual bool check() const;

  void setNearCallback(btNearCallback nearCallback);

protected:

  static void NearCallbackAllToAll(btBroadphasePair& collisionPair,
                                   btCollisionDispatcher& dispatcher,
                                   const btDispatcherInfo& dispatchInfo);

  static void MyNearCallbackEnabled(btBroadphasePair& collisionPair,
                                    btCollisionDispatcher& dispatcher,
                                    const btDispatcherInfo& dispatchInfo);

  static void MyNearCallbackDisabled(btBroadphasePair& collisionPair,
                                     btCollisionDispatcher& dispatcher,
                                     const btDispatcherInfo& dispatchInfo);


  virtual void initPhysics(const PhysicsConfig* config);
  void applyControl(double dt);
  void updateSensors();
  bool updatePPSSensor(RcsSensor* sensor);
  bool updateLoadcell(const RcsSensor* loadCell);
  void setJointTorque(const RcsJoint* jnt, double torque);
  bool setJointAngle(const RcsJoint* jnt, double angle, double dt);
  BulletJointBase* getHinge(const RcsJoint* jnt) const;
  virtual void createWorld(xmlNodePtr bulletParams);
  virtual void updateSoftMeshes();

  btDynamicsWorld* dynamicsWorld;
  btBroadphaseInterface* broadPhase;
  btCollisionDispatcher* dispatcher;
  btConstraintSolver* solver;
  btMLCPSolverInterface* mlcpSolver;
  btDefaultCollisionConfiguration* collisionConfiguration;
  mutable pthread_mutex_t mtx;
  double lastDt;
  std::map<int, BulletRigidBody*> bdyMap;
  std::map<int, BulletJointBase*> jntMap;
  std::map<std::string, BulletRigidBody*> deactivatedBodies;


  BulletRigidBody* dragBody;
  double dragForce[3];
  double dragAnchor[3];

  BulletDebugDrawer* debugDrawer;
  std::string physicsConfigFile;

  double rigidBodyLinearDamping;
  double rigidBodyAngularDamping;
  double jointedBodyLinearDamping;
  double jointedBodyAngularDamping;

private:

  /*! \brief Private assignment operator to avoid it from being used
   */
  BulletSimulation& operator = (const BulletSimulation&);
};

}   // namespace Rcs

#endif   // RCS_BULLETSIMULATION_H
