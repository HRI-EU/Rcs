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

#ifndef RCS_VORTEXSIMULATION_H
#define RCS_VORTEXSIMULATION_H

#include "PhysicsConfig.h"

#include <PhysicsBase.h>

#include <unordered_map>
#include <pthread.h>
#include <fstream>


namespace Vx
{
class VxPart;
class VxUniverse;
class VxMaterial;
class VxMaterialTable;
}

namespace Rcs
{

/*! \ingroup RcsPhysics
 *  \brief Vortex physics simulation class.
 *
 *         Each instance of this class owns its own universe.
 */
class VortexBody;

class VortexSimulation : public PhysicsBase
{
public:

  /*! \brief Constructs an instance of a physics simulation.
   *
   *  \param[in] graph         Pointer to the RcsGraph structure that the
   *                           simulation should correspond to.
   *  \param[in] cfgFile       Config file with the default parameters of the
   *                           simulation. For details, see \ref initPhysics
   *                           and \ref initMaterial.
   *  \param[in] groundPlane   If true, a ground plane in the x-y plane on
   *                           height z=0 is created.
   */
  VortexSimulation(const RcsGraph* graph,
                   const char* cfgFile="config/physics/vortex.xml",
                   bool groundPlane=true);

  /*! \brief Constructs an instance of a physics simulation.
   *
   *  \param[in] graph         Pointer to the RcsGraph structure that the
   *                           simulation should correspond to.
   *  \param[in] config        Config with the default parameters of the
   *                           simulation. For details, see \ref initPhysics
   *                           and \ref initMaterial.
   *  \param[in] groundPlane   If true, a ground plane in the x-y plane on
   *                           height z=0 is created.
   */
  VortexSimulation(const RcsGraph* graph,
                   const PhysicsConfig* config,
                   bool groundPlane=true);
  VortexSimulation(const VortexSimulation& copyFromMe,
                   const RcsGraph* newGraph);



  /*! \brief Destroys the class and frees all internal memory.
   */
  virtual ~VortexSimulation();

  virtual VortexSimulation* clone(RcsGraph* newGraph=NULL) const;

  /*! \brief Sets the gravity force in world coordinates.
   */
  virtual void setGravity(const double gravity[3]);


  /*! \brief Computes a single simulation step with the given time interval
   *         in [secs]. The previously set control command is applied.
   *
   *  \param[in] dt     Simulation time interval in seconds. It is internally
   *                    subdivided according to the interval given in \ref
   *                    integratorDt.
   */
  void step(double dt);



  /*! \brief Computes a single simulation step with the given time interval
   *         in [secs].
   *
   *  \param[in] dt      Simulation time interval in seconds. It is internally
   *                     subdivided according to the interval given in \ref
   *                     integratorDt.
   *  \param[out] q      Vector holding the value for each degree of freedom.
   *                     If the vector is on input of dimension
   *                     RcsGraph::dof x 1, all degrees of freedom are copied.
   *                     If it is of dimension RcsGraph::nJ x 1, only the
   *                     unconstrained dof are considered. The array is not
   *                     reshaped. If its row size is neither RcsGraph::dof
   *                     nor RcsGraph::nJ, the function exits with a fatal
   *                     error.
   *  \param[out] qp     Vector holding the value for each degree of freedom
   *                     velocity. It has the same behavior as input
   *                     parameter q.
   *  \param[out] qpp    Vector holding the value for each degree of freedom
   *                     acceleration. It has the same behavior as input
   *                     parameter q.
   *  \param[out] T      Vector holding the value for each degree of freedom
   *                     torque. It has the same behavior as input
   *                     parameter q.
   *  \param[in] control If true, the function applies the previously set
   *                     control command.
   */
  void simulate(double dt, MatNd* q = NULL, MatNd* qp = NULL,
                MatNd* qpp = NULL, MatNd* T = NULL, bool control = false);

  void reset();
  void applyForce(const RcsBody* body, const double F[3], const double r[3]);
  void applyTransform(const RcsBody* body, const HTr* A_BI);
  void applyLinearVelocity(const RcsBody* body, const double v[3]);
  void applyAngularVelocity(const RcsBody* body, const double omega[3]);
  void getLinearVelocity(const RcsBody* body, double v[3]) const;
  void getAngularVelocity(const RcsBody* body, double omega[3]) const;
  void setJointTorque(const MatNd* T_des);
  void getJointTorque(MatNd* T_curr, RcsStateType type=RcsStateFull) const;
  void getJointAngles(MatNd* q, RcsStateType type=RcsStateFull) const;
  void getJointVelocities(MatNd* qp, RcsStateType type=RcsStateFull) const;
  void setMassAndInertiaFromPhysics(RcsGraph* graph);
  void getPhysicsTransform(HTr* A_BI, const RcsBody* body) const;
  void disableCollision(const RcsBody* b0, const RcsBody* b1);
  void disableCollisions();
  void setJointLimits(bool enable);
  Contacts getContacts();
  void setJointCompliance(const MatNd* stiffness, const MatNd* damping=NULL);
  void getJointCompliance(MatNd* stiffness, MatNd* damping=NULL) const;
  void printMaterialTable(std::ostream& out) const;
  void printMaterialTable() const;

  /*! \brief Prints out the material table.
   */
  virtual void print() const;



  /*! \brief Returns the name of the instance, such as used in the
   *         PhysicsFactory for instantiation.
   */
  virtual const char* getClassName() const;



  /*! \brief After calling this function, the \ref updateTransformations()
   *         methods calls a mutex lock and unlock before doing any
   *         computation on the transformation data (If mutex is not NULL).
   *
   *  \param[in] mutex   Mutex to be set in the \ref updateTransformations
   *                     method.
   */
  virtual void setTransformUpdateMutex(pthread_mutex_t* mutex);



  /*! \brief Returns a pointer to the transformation of the VxPart.
   */
  virtual const HTr* getPhysicsTransformPtr(const RcsBody* body) const;



  /*! \brief Calls the update methods of all registered sensors
   */
  void updateSensors();



  /*! \brief Returns the internal integration time value. If the argument dt
   *         in the \ref simulate or \ref step function exceede this value,
   *         the simulation is sub-stepped with the time interval returned by
   *         this function.
   */
  double getIntegratorDt() const;




  /*! \brief Removes all Vortex constraints of a body in preparation for a
   *         graph restructuring
   */
  bool removeBodyConstraints(RcsBody* body);

  /*! \brief Removes a body from the simulation. The function returns true on
  *         success, false otherwise.
  */
  bool removeBody(const char* name);

  bool addBody(const RcsBody* body);

  bool deactivateBody(const char* name);
  bool activateBody(const char* name, const HTr* A_BI=NULL);



  /*! \brief Adds Vortex constraints according to the body joints etc. after a
   *         graph restructuring
   */
  bool addBodyConstraints(RcsBody* body);

  Vx::VxPart* getPart(const char* name);

  /*! \brief Function for setting physics parameter. The function is thread-
   *         safe. All strings are interpreted case insensitive. The following
   *         parmeter types are supported:
   *
   *         - category "Material":
   *           - name: Material name within the material table
   *           - type:
   *             - restitution
   *             - compliance
   *             - damping
   *             - adhesiveforce
   *             - linearfriction
   *             - linearfrictionprimary
   *             - linearfrictionsecondary
   *             - angularfriction
   *             - angularfrictionprimary
   *             - angularfrictionsecondary
   *             - angularfrictionnormal
   *         - category "Simulation":
   *           - name: gravity
   *           - type:
   *             - x
   *             - y
   *             - z
   *         - category "Body":
   *           - name: Name of the body
   *           - type: mass
   *           - type: com_x
   *           - type: com_y
   *           - type: com_z
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

  Vx::VxMaterialTable* getMaterialTable() const;
  Vx::VxMaterial* getMaterial(const char* name) const;
  Vx::VxUniverse* getUniverse();
  const Vx::VxUniverse* getUniverse() const;

private:

  VortexBody* getPartPtr(const RcsBody* body) const;

  bool initSettings(const PhysicsConfig* config);

  void initMaterial(const PhysicsConfig* config);

  /*! \brief Creates a physical body and adds it to the simulation. All
   *         collision shapes are subsumed into one composite shape, which
   *         is more stable than adding them individually. That's due to
   *         the internal contact point generation, which works differently
   *         for composite shapes.
   */
  bool createCompositeBody(const RcsBody* body);

  /*! \brief Creates a physical joint and adds it to the simulation.
   */
  bool createJoint(const RcsBody* body);

  /*! \brief Removes a physical joint from the simulation and deletes it. The
   *         function returns true if the joint removed succeeded, and false
   *         otherwise.
   */
  bool removeJoint(RcsBody* body);

  void updateTransformations();
  void applyControl(double dt);
  void initPhysics(const PhysicsConfig* physicsConfig, bool groundPlane);
  bool updateFTS(RcsSensor* sensor);
  bool updateJointTorqueSensor(RcsSensor* sensor);
  bool updateContactForceSensor(RcsSensor* sensor);
  bool updatePPSSensor(RcsSensor* sensor);

  pthread_mutex_t* trafoUpdateLock;

  double integratorDt;
  double bodyLinearDamping;
  double bodyAngularDamping;
  double jointLockStiffness;
  double jointLockDamping;
  double jointMotorLoss;
  bool jointLimitsActive;

  double F_ext[3];
  double r_ext[3];
  Vx::VxPart* b_ext;
  Vx::VxPart* groundPlane;
  Vx::VxUniverse* universe;
  char* materialFileName;

  mutable pthread_mutex_t extForceLock;

  std::unordered_map<const RcsBody*, VortexBody*> bdyMap;
  std::map<std::string,Vx::VxPart*> deactivatedBodies;


  /*! \brief Assignment operator. Ideally, this should be implemented, rather
   *         than being made private.
   */
  VortexSimulation& operator=(const VortexSimulation&);



  /*! \brief Copy constructor. Ideally, this should be implemented, rather than
   *         being made private.
   */
  VortexSimulation(const VortexSimulation&);
};

}   // namespace Rcs

#endif // RCS_VORTEXSIMULATION_H

