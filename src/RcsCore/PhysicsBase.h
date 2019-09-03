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

#ifndef RCS_PHYSICSBASE_H
#define RCS_PHYSICSBASE_H

#include "Rcs_graph.h"

#include <vector>



/*!
 *  \defgroup RcsPhysics Physics simulation
 *
 *  A library for rigid body physics simulation.
 *
 */

namespace Rcs
{

/*! \ingroup RcsPhysics
 *  \brief Base class for rigid body physics simulation. Contains basic
 *         interface and a set of generic methods.
 */
class PhysicsBase
{
public:
  struct Contact
  {
    double pos[3];
    double force[3];
  };
  typedef std::vector<Contact> Contacts;



public:

  typedef enum
  {
    Simulation = 0,
    Material,
    Body,
    Joint

  } ParameterCategory;

  /*! \brief Computes a single simulation step with the given time interval
   *         in [secs]. The previously set control command is applied.
   *
   *  \param[in] dt     Simulation time interval in seconds.
   */
  virtual void step(double dt) = 0;

  /*! \brief Computes a single simulation step with the given time interval
   *         in [secs].
   *
   *  \param[in] dt      Simulation time interval in seconds. If it is smaller
   *                     or equal to zero, the function returns without doing
   *                     anything.
   *  \param[out] q      Vector holding the value for each degree of freedom.
   *                     If the vector is on input of dimension
   *                     RcsGraph::dof x 1, all degrees of freedom are copied.
   *                     If it is of dimension RcsGraph::nJ x 1, only the
   *                     unconstrained dof are considered. The array is not
   *                     reshaped. If its row size is neither RcsGraph::dof
   *                     nor RcsGraph::nJ, the function exits with a fatal
   *                     error.
   *  \param[out] q_dot  Vector holding the value for each degree of freedom
   *                     velocity. It has the same behavior as input
   *                     parameter q.
   *  \param[out] q_ddot Vector holding the value for each degree of freedom
   *                     acceleration. It has the same behavior as input
   *                     parameter q.
   *  \param[out] T      Vector holding the value for each degree of freedom
   *                     torque. It has the same behavior as input
   *                     parameter q.
   *  \param[in] control If true, the function applies the previously set
   *                     control command (see \ref setControlInput()).
   */
  virtual void simulate(double dt, MatNd* q=NULL, MatNd* q_dot=NULL,
                        MatNd* q_ddot=NULL, MatNd* T=NULL,
                        bool control=false) = 0;

  virtual void simulate(double dt, RcsGraph* graph, MatNd* q_ddot = NULL,
                        MatNd* T=NULL, bool control=true);

  /*! \brief Resets the simulation to the current state, so that all
   *         momentum is zero.
   */
  virtual void reset() = 0;

  /*! \brief Returns the name of the instance, such as used in the
   *         PhysicsFactory for instantiation.
   */
  virtual const char* getClassName() const = 0;

  /*! \brief Sets the gravity force in world coordinates.
   */
  virtual void setGravity(const double gravity[3]) = 0;

  /*! \brief Applies a force to the body. Both force and point are in world
   *         coordinates. The force will only persist for the next simulation
   *         timestep.
   *
   *  \param[in] body   RcsBody that the given force is applied to.
   *  \param[in] F      Force vector in world coordinates.
   *  \param[in] p      Force point in world coordinates. If NULL, force is
   *                    applied to object's COM.
   */
    virtual void setForce(const RcsBody* body, const double F[3],
                            const double p[3]) = 0;

  /*! \brief Applies a force impulse to the body. Both force and point are
   *         in world coordinates.
   *
   *  \param[in] body   RcsBody that the given force is applied to.
   *  \param[in] F      Force vector in world coordinates.
   *  \param[in] r      Force point in world coordinates. If NULL, force is
   *                    applied to object's COM.
   */
    virtual void applyImpulse(const RcsBody* body, const double F[3],
                              const double r[3]) = 0;

  /*! \brief Applies a force to the body. Both force and point are in world
   *         coordinates. The force will not be resetted after stepping the
   *         simulation.
   *
   *  \param[in] body   RcsBody that the given force is applied to. If it is
   *                    NULL, the previously applied force is set to zero.
   *  \param[in] F      Force vector in world coordinates.
   *  \param[in] r      Force point in world coordinates.
   */
  virtual void applyForce(const RcsBody* body, const double F[3],
                          const double r[3]) = 0;

  /*! \brief Move the body to the given transformation. If the physics entity
   *         corresponding to parameter body doesn't exist, the function does
   *         nothing except complaining on debug level 1.
   *
   *  \param[in] body   RcsBody that the given force is applied to. If it is
   *                    NULL, the previously applied force is set to zero.
   *  \param[in] A_BI   Transformation from the world (I) frame to the body
   *                    (B) frame.
   */
  virtual void applyTransform(const RcsBody* body, const HTr* A_BI) = 0;

  /*! \brief Set the linear velocity of a body in the world reference frame.
   *         If the physics entity corresponding to body doesn't exist, the
   *         function does nothing except complaining on debug level 1.
   *
   *  \param[in] body   RcsBody that the given force is applied to. If it is
   *                    NULL, the previously applied force is set to zero.
   *  \param[in] v      Desired linear velocity of the physics entity
   *                    associated with body.
   */
  virtual void applyLinearVelocity(const RcsBody* body, const double v[3]) = 0;

  /*! \brief Set the angular velocity of a body in the world reference frame.
   *         If the physics entity corresponding to body doesn't exist, the
   *         function does nothing except complaining on debug level 1.
   *
   *  \param[in] body   RcsBody that the given force is applied to. If it is
   *                    NULL, the previously applied force is set to zero.
   *  \param[in] omega  Desired angular velocity of the physics entity
   *                    associated with body.
   */
  virtual void applyAngularVelocity(const RcsBody* body,
                                    const double omega[3]) = 0;

  /*! \brief Get the linear velocity of a body in world coordinates. If the
   *         body has no physics entity, v remains unchanged, and a warning
   *         message is issued on debug level 1.
   *
   *  \param[in] body   RcsBody whose velocity is requested. If it is NULL,
   *                    argument v remains unchanged.
   *  \param[in] v      Linear velocity of the physics entity associated with
   *                    body, in world coordinates.
   */
  virtual void getLinearVelocity(const RcsBody* body, double v[3]) const = 0;

  /*! \brief Get the angular velocity of a body in world coordinates. If the
   *         body has no physics entity, omega remains unchanged, and a warning
   *         message is issued on debug level 1.
   *
   *  \param[in] body   RcsBody whose velocity is requested. If it is NULL,
   *                    argument omega remains unchanged.
   *  \param[in] omega  Angular velocity of the physics entity associated with
   *                    body, in world coordinates.
   */
  virtual void getAngularVelocity(const RcsBody* body,
                                  double omega[3]) const = 0;

  /*! \brief Set the joint torque to the joints of the RcsGraph.
   *
   *  \param[in] T_des  Joint torque vector. The function can deal with
   *                    different dimensions, [RcsGraph::dof x 1] and
   *                    [RcsGraph::nJ x 1]. For all other dimensions, the
   *                    function exits with a fatal error. Torque will only be
   *                    applied to unconstrained degrees of freedom. This
   *                    function ignores the RcsJoint's setting (e.g. being
   *                    a kinematic joint).
   */
  virtual void setJointTorque(const MatNd* T_des) = 0;

  /*! \brief Gets the joint torque of the joints of the RcsGraph.
   *
   *  \param[out] T_curr  Joint torque vector. If argument type is RcsStateFull,
   *                      the vector is reshaped to [RcsGraph::dof x 1],
   *                      if type is RcsStateIK, it is reshaped to dimension
   *                      [RcsGraph::nJ x 1].
   *  \param[in] type     Enum that specifies if the vector T_curr should get
   *                      a torque for each dof (RcsStateFull), or only for the
   *                      unconstrained ones (RcsStateIK). Anything else leads
   *                      to a fatal error.
   */
  virtual void getJointTorque(MatNd* T_curr,
                              RcsStateType type=RcsStateFull) const = 0;

  /*! \brief Copies the joint angles from the physics simulation into the array
   *         q. The function reshapes the array q to either the full degrees of
   *         freedom or the unconstrained degrees of freedom. If a joint
   *         doesn't have a physics representation, the kinematic value of the
   *         graph is copied. For bodies with rigid body dof, the corresponding
   *         values from the physics entity are copied (if it exists).
   *
   *  \param[in,out] q   Joint angle vector, it will be reshaped to dimension
   *                     [RcsGraph::dof x 1] for type RcsStateFull, and to
   *                     dimension [RcsGraph::nJ x 1] for type RcsStateIK.
   *  \param[in] type    Enum that specifies if the vector T_curr should get
   *                     a torque for each dof (RcsStateFull), or only for the
   *                     unconstrained ones (RcsStateIK). Anything else leads
   *                     to a fatal error.
   */
  virtual void getJointAngles(MatNd* q,
                              RcsStateType type=RcsStateFull) const = 0;

  /*! \brief Copies the joint velocities from the physics simulation into q_dot.
   *         The function accounts for the size of q_dot, which is either the
   *         full degrees of freedom or the unconstrained degrees of freedom.
   *         If a joint doesn't have a physics representation, the kinematic
   *         value of the graph is copied. For bodies with rigid body dof, the
   *         corresponding values from the physics entity are copied (if it
   *         exists).
   *
   *  \param[in,out] q_dot Joint velocity vector, it will be reshaped to
   *                       dimension [RcsGraph::dof x 1] for type RcsStateFull,
   *                       and to dimension [RcsGraph::nJ x 1] for type
   *                       RcsStateIK.
   *  \param[in] type      Enum that specifies if the vector T_curr should get
   *                       a torque for each dof (RcsStateFull), or only for the
   *                       unconstrained ones (RcsStateIK). Anything else leads
   */
  virtual void getJointVelocities(MatNd* q_dot,
                                  RcsStateType type=RcsStateFull) const = 0;

  /*! \brief Copies all physics entities masses, center of mass vectors and
   *         inertia tensors into all bodies if the graph that have a physics
   *         equivalent.
   *
   *         \param[in,out] graph   RcsGraph structure to which physics
   *                                properties are assigned.
   */
  virtual void setMassAndInertiaFromPhysics(RcsGraph* graph) = 0;

  /*! \brief Copies the transformation of the physics entity associated with
   *         argument body to argument A_BI. This is the transformation from the
   *         world (I) frame to the body (B) frame (row major form).
   *
   *  \param[in] A_BI   Transformation from the world (I) frame to the body (B)
   *                    frame (row major form).
   *
   *  \param[in] body   Body of interest. If it is NULL or doesn't exist in
   *                    the graph, the argument A_BI is set to identity, and a
   *                    warning is issues on debug level 1.
   */
  virtual void getPhysicsTransform(HTr* A_BI, const RcsBody* body) const = 0;

  /*! \brief Returns a pointer to the transformation of the body inside
   *         the physics.
   */
  virtual const HTr* getPhysicsTransformPtr(const RcsBody* body) const = 0;

  /*! \brief Disables the collision between the bodies b0 and b1.
   *
   *  \param[in] b0   First body to consider. If it does not exist, or it
   *                  has no associated physics entity, the function does
   *                  nothing than complain on debug level 3.
   *  \param[in] b1   Second body to consider. If it does not exist, or it
   *                  has no associated physics entity, the function does
   *                  nothing than complain on debug level 3.
   */
  virtual void disableCollision(const RcsBody* b0, const RcsBody* b1) = 0;

  /*! \brief Queries all contacts of the most recent simulation step.
   *
   *  \return All contacts of the most recent simulation step.
   */
  virtual Contacts getContacts() = 0;

  /*! \brief Enables or disables the joint limits of all joints.
   *
   *  \param[in] enable   If true, all joint limits are enabled. If false, all
   *                      joint limits are disabled. Currently, this is only
   *                      implemented for torque hinge joints.
   */
  virtual void setJointLimits(bool enable) = 0;

  /*! \brief Cloning function with optional graph.
   *
   *  \param[in] newGraph     RcsGraph to refer to. If it is NULL, the graph
   *                          of the copy points to the one it is copied from.
   */
  virtual PhysicsBase* clone(RcsGraph* newGraph=NULL) const = 0;

  virtual void setJointCompliance(const MatNd* stiffness,
                                  const MatNd* damping=NULL) = 0;

  virtual void getJointCompliance(MatNd* stiffness,
                                  MatNd* damping=NULL) const = 0;










  /*! \brief Function for setting physics parameter. The concrete implementation
   *         depends on the derieved physics class. This classes implementation
   *         is empty.
   *
   *  \param[in] category   See enum ParameterCategory
   *  \param[in] name       Parameter name, such as "SoftMaterial".
   *  \param[in] type       Parameter type, such as "Restitution".
   *  \param[in] value      Value the parameter should be assigned with
   *  \return true for success, false otherwise
   */
  virtual bool setParameter(ParameterCategory category,
                            const char* name, const char* type, double value);

  /*! \brief Base class constructor.
   *
   *  \param[in] graph   Graph that the simulation should be build on.
   */
  PhysicsBase(const RcsGraph* graph);

  /*! \brief Copy constructor
   *
   *  \param[in] copyFromMe   Simulation instance to be copied from.
   */
  PhysicsBase(const PhysicsBase& copyFromMe);

  /*! \brief Copy constructor. The configuration state vector will not be set
   *         from the graph of copyFromMe, but will be the one of newGraph.
   *         If they must be equal, this needs to be ensured by the caller. The
   *         graph will never be modified within the physics simulation.
   *
   *  \param[in] copyFromMe   Simulation instance to be copied from.
   *  \param[in] newGraph     RcsGraph to refer to.
   */
  PhysicsBase(const PhysicsBase& copyFromMe, const RcsGraph* newGraph);

  /*! \brief Assignment operator for deep copying everything.
   */
  PhysicsBase& operator = (const PhysicsBase&);

  /*! \brief Virtual destructor to allow overloading.
   */
  virtual ~PhysicsBase();

  /*! \brief Copies the given control inputs to the class's internal control
   *         input arrays. Each of the arguments that is NULL will be ignored.
   *         Each input array may be of size [RcsGraph::dof x 1] or of size
   *         [RcsGraph::nJ x 1].
   *
   *  \param[in] q_des      Vector of desired degrees of freedom.
   *  \param[in] q_dot_des  Vector of desired degrees of freedom velocities.
   *  \param[in] T_des      Vector of desired degrees of freedom torques.
   */
  virtual void setControlInput(const MatNd* q_des, const MatNd* q_dot_des,
                               const MatNd* T_des);


  /*! \brief Copies the command vector that has been set with the last call to
   *         \ref setControlInput() into q_des.
   *
   *  \param[out] q_des  Last commanded joint position vector. It is of the
   *                     graph's dimension (RcsGraph::dof x 1) and will be
   *                     reshaped by the function.
   */
  virtual void getLastPositionCommand(MatNd* q_des) const;

  /*! \brief Disables collisions between bodies within one group
   *
   *  \param[in] suffix  Groups's suffix. If it is NULL,  then collisions
   *                     within all groups are disabled
   */
  virtual void disableCollisionsWithinGroup(const char* suffix);

  /*! \brief Disables collisions between all meshes.
   */
  virtual void disableMeshCollisions();

  /*! \brief Disables collisions between all bodies.
   */
  virtual void disableCollisions();

  /*! \brief Disables all joint limits.
  */
  virtual void disableJointLimits();

  /*! \brief Enables all joint limits.
  */
  virtual void enableJointLimits();

  /*! \brief Returns a pointer to the underlying graph.
   *
   *  \return Pointer to the simulation's RcsGraph structure.
   */
  virtual const RcsGraph* getGraph() const;

  /*! \brief Returns a pointer to the underlying graph.
   *
   *  \return Pointer to the simulation's RcsGraph structure.
   */
  virtual RcsGraph* getGraph();

  /*! \brief Returns the simulation time. The simulation time is 0 on
   *         construction, and will be increased by dt for each call of
   *         \ref simulate.
   *
   *  \return Time in [sec].
   */
  virtual double time() const;

  /*! \brief Resets the internal simulation time. The simulation time is 0 on
   *         construction, and will be increased by dt for each call of
   *         \ref simulate.
   */
  virtual void resetTime();

  /*! \brief Prints out simulation-class specific information to the console.
   *         This classes implementation does nothing.
   */
  virtual void print() const;

  /*! \brief Enables or disables the tactile sensor computation.
   */
  virtual void setEnablePPS(bool enable);

  /*! \brief Returns true if tactile sensors are computed, false otherwise.
   */
  virtual bool getEnablePPS() const;

  /*! \brief Adds a body to the simulation. The function returns true on
   *         success, false otherwise.
   */
  virtual bool addBody(const RcsBody* body);

  /*! \brief Removes a body from the simulation. The function returns true on
   *         success, false otherwise.
   */
  virtual bool removeBody(const char* name);

  virtual bool deactivateBody(const char* name);
  virtual bool activateBody(const char* name, const HTr* A_BI=NULL);

  virtual bool check() const;



protected:

  /*! \brief Sets the internal simulation time.
   */
  virtual void setTime(double t);

  /*! \brief Adds dt to the internal simulation time.
   */
  virtual void incrementTime(double dt);

  MatNd* T_des;      ///< Desired joint torque [RcsGraph::dof x 1]
  MatNd* q_des;      ///< Desired joint angles [RcsGraph::dof x 1]
  MatNd* q_dot_des;  ///< Desired joint velocities [RcsGraph::dof x 1]



private:

  double simTime;    ///< Simulation time in [sec]
  bool enablePPS;    ///< Tactile sensor arrays flag
  RcsGraph* internalDesiredGraph;   // For kinematic transforms based on q_des
};

}   // namespace Rcs

#endif // RCS_PHYSICSBASE_H

