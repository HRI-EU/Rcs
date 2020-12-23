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

#include "VortexSimulation.h"
#include "PhysicsFactory.h"
#include "VortexHelpers.h"

#include <Rcs_math.h>
#include <Rcs_typedef.h>
#include <Rcs_utils.h>
#include <Rcs_macros.h>
#include <Rcs_parser.h>
#include <Rcs_resourcePath.h>
#include <Rcs_body.h>
#include <Rcs_joint.h>
#include <Rcs_shape.h>
#include <Rcs_sensor.h>
#include <Rcs_timer.h>

#include <Vx/VxFrame.h>
#include <Vx/VxFrameStepperOption.h>
#include <Vx/VxSolverParameters.h>
#include <Vx/VxRPRO.h>
#include <Vx/VxHinge.h>
#include <Vx/VxPrismatic.h>
#include <Vx/VxMassProperties.h>
#include <Vx/VxPlane.h>
#include <Vx/VxCollisionGeometry.h>
#include <Vx/VxSolverParameters.h>
#include <Vx/VxCompositeCollisionGeometry.h>
#include <Vx/VxRigidBodyResponseModel.h>
#include <Vx/VxMaterialTable.h>
#include <Vx/VxUniverse.h>

#include <fstream>
#include <iostream>
#include <utility>

#define RCSVORTEX_DEFAULT_INTEGRATOR_DT                 (0.005)
//#define RCSVORTEX_DEFAULT_LINEAR_DAMPING                (0.0001)
//#define RCSVORTEX_DEFAULT_ANGULAR_DAMPING               (0.00002)
//#define RCSVORTEX_DEFAULT_CONTACT_TOLERANCE             (0.0001)
//#define RCSVORTEX_DEFAULT_CONSTRAINT_LINEAR_COMPLIANCE  (1.0e-10)
//#define RCSVORTEX_DEFAULT_CONSTRAINT_ANGULAR_COMPLIANCE (1.0e-10)
#define RCSVORTEX_DEFAULT_JOINT_MOTOR_LOSS              (0.002)
#define RCSVORTEX_DEFAULT_JOINT_LOCK_STIFFNESS          (100000.0)
#define RCSVORTEX_DEFAULT_JOINT_LOCK_DAMPING            (300.0)


static const char className[] = "Vortex";
static Rcs::PhysicsFactoryRegistrar<Rcs::VortexSimulation> physics(className);



/*******************************************************************************
 * Constructor from Graph & Config file
 ******************************************************************************/
Rcs::VortexSimulation::VortexSimulation() :
  trafoUpdateLock(NULL),
  integratorDt(RCSVORTEX_DEFAULT_INTEGRATOR_DT),
  bodyLinearDamping(-1.0),
  bodyAngularDamping(-1.0),
  jointLockStiffness(RCSVORTEX_DEFAULT_JOINT_LOCK_STIFFNESS),
  jointLockDamping(RCSVORTEX_DEFAULT_JOINT_LOCK_DAMPING),
  jointMotorLoss(RCSVORTEX_DEFAULT_JOINT_MOTOR_LOSS),
  jointLimitsActive(true),
  b_ext(NULL),
  groundPlane(NULL),
  universe(NULL),
  materialFileName(NULL)
{
}

/*******************************************************************************
 * Constructor from Graph & Config file
 ******************************************************************************/
Rcs::VortexSimulation::VortexSimulation(const RcsGraph* g,
                                        const char* physicsConfigFile) :
  PhysicsBase(g),
  trafoUpdateLock(NULL),
  integratorDt(RCSVORTEX_DEFAULT_INTEGRATOR_DT),
  bodyLinearDamping(-1.0),
  bodyAngularDamping(-1.0),
  jointLockStiffness(RCSVORTEX_DEFAULT_JOINT_LOCK_STIFFNESS),
  jointLockDamping(RCSVORTEX_DEFAULT_JOINT_LOCK_DAMPING),
  jointMotorLoss(RCSVORTEX_DEFAULT_JOINT_MOTOR_LOSS),
  jointLimitsActive(true),
  b_ext(NULL),
  groundPlane(NULL),
  universe(NULL),
  materialFileName(NULL)
{
  this->materialFileName = String_clone(physicsConfigFile);

  PhysicsConfig config(materialFileName);
  initPhysics(&config);
}

/*******************************************************************************
 * Constructor from Graph & Config object
 ******************************************************************************/
Rcs::VortexSimulation::VortexSimulation(const RcsGraph* g,
                                        const Rcs::PhysicsConfig* config):
  PhysicsBase(g),
  trafoUpdateLock(NULL),
  integratorDt(RCSVORTEX_DEFAULT_INTEGRATOR_DT),
  bodyLinearDamping(-1.0),
  bodyAngularDamping(-1.0),
  jointLockStiffness(RCSVORTEX_DEFAULT_JOINT_LOCK_STIFFNESS),
  jointLockDamping(RCSVORTEX_DEFAULT_JOINT_LOCK_DAMPING),
  jointMotorLoss(RCSVORTEX_DEFAULT_JOINT_MOTOR_LOSS),
  jointLimitsActive(true),
  b_ext(NULL),
  groundPlane(NULL),
  universe(NULL),
  materialFileName(NULL)
{
  this->materialFileName = String_clone(config->getConfigFileName());
  initPhysics(config);
}

/*******************************************************************************
 * Copy constructor
 ******************************************************************************/
Rcs::VortexSimulation::VortexSimulation(const VortexSimulation& copyFromMe,
                                        const RcsGraph* newGraph):
  PhysicsBase(copyFromMe, newGraph),
  trafoUpdateLock(NULL),
  integratorDt(RCSVORTEX_DEFAULT_INTEGRATOR_DT),
  bodyLinearDamping(-1.0),
  bodyAngularDamping(-1.0),
  jointLockStiffness(RCSVORTEX_DEFAULT_JOINT_LOCK_STIFFNESS),
  jointLockDamping(RCSVORTEX_DEFAULT_JOINT_LOCK_DAMPING),
  jointMotorLoss(RCSVORTEX_DEFAULT_JOINT_MOTOR_LOSS),
  jointLimitsActive(true),
  b_ext(NULL),
  groundPlane(NULL),
  universe(NULL),
  materialFileName(NULL)
{
  this->materialFileName = String_clone(copyFromMe.materialFileName);
  PhysicsConfig config(materialFileName);
  initPhysics(&config);
}

/*******************************************************************************
 * Physics initialization
 ******************************************************************************/
bool Rcs::VortexSimulation::initialize(const RcsGraph* g,
                                       const PhysicsConfig* config)
{
  RCHECK(getGraph()==NULL);
  initGraph(g);
  this->materialFileName = String_clone(config->getConfigFileName());
  initPhysics(config);
  return true;
}

/*******************************************************************************
 * Physics initialization
 ******************************************************************************/
void Rcs::VortexSimulation::initPhysics(const PhysicsConfig* physicsConfig)
{
  Vec3d_setZero(this->F_ext);
  Vec3d_setZero(this->r_ext);

  pthread_mutex_init(&this->extForceLock, NULL);

  // Create the frame
  RLOG(5, "VxFrame::instance()");
  Vx::VxFrame* frame = Vx::VxFrame::instance();
  frame->setMaxThreadCount(4);

  // Create the universe
  RLOG(5, "Creating universe");
  this->universe = new Vx::VxUniverse(100, 10000);
  universe->setCollisionMultithreaded(true);
  universe->setGravity(0.0, 0.0, -RCS_GRAVITY);
  universe->setAutoSleep(false);

  // Add the universe to the VxFrame
  RLOG(5, "frame->addUniverse");
  frame->addUniverse(universe);

  Vx::VxFrameStepperOption& fo = frame->getStepperOption();
  fo.setStepperMode(Vx::VxFrameStepperOption::kModeSafe);
  fo.setRecoverySubstepCount(10);
  fo.setCheckAcceleration(true);
  fo.setCheckAccelerationAngularThreshold(300.0);
  fo.setCheckAccelerationLinearThreshold(20.0);
  fo.setVerbose(true);


  // Now that the frame and universe are created, check for parameters in xml
  // file. Set default configuration parameters.
  // The damping values could also be set to the critical damping with
  // universe->getCriticalDamping(stiffness)
  this->integratorDt = RCSVORTEX_DEFAULT_INTEGRATOR_DT;
  this->jointLockStiffness = RCSVORTEX_DEFAULT_JOINT_LOCK_STIFFNESS;
  this->jointLockDamping = RCSVORTEX_DEFAULT_JOINT_LOCK_DAMPING;
  this->jointMotorLoss = RCSVORTEX_DEFAULT_JOINT_MOTOR_LOSS;

  // load settings and materials from config and create optional ground plane
  initSettings(physicsConfig);
  RLOG(5, "Physics initialized");

  initMaterial(physicsConfig);
  RLOG(5, "Materials initialized");

  // Create all Vortex VxParts from the RcsGraph's bodies
  RLOG(5, "Creating bodies");
  RCSGRAPH_TRAVERSE_BODIES(getGraph())
  {
    createCompositeBody(BODY);
  }

  // Create all joints between the RcsGraph's bodies
  RLOG(5, "Creating joints");
  RCSGRAPH_TRAVERSE_BODIES(getGraph())
  {
    createJoint(BODY);
  }

  // Disable all mesh-mesh collisions
  RLOG(5, "Disabling mesh collisions");
  disableMeshCollisions();

  // Disable collisions within all groups
  disableCollisionsWithinGroup(NULL);

  // Copy transforms
  RLOG(5, "Updating transforms");
  updateTransformations();

  // checking for possible problems of universe
  universe->verifyUniverseContent();

  RLOG(5, "finished Vortex constructor");
}

/*******************************************************************************
 * Stops application and deletes all memory
 ******************************************************************************/
Rcs::VortexSimulation::~VortexSimulation()
{
  Vx::VxFrame* frame = Vx::VxFrame::instance();

  frame->removeUniverse(this->universe);
  // removeUniverse doesn't delete it!
  delete this->universe;

  if (frame->getUniverseCount()==0)
  {
    frame->release();
  }

  pthread_mutex_destroy(&this->extForceLock);

  if (this->materialFileName != NULL)
  {
    RFREE(this->materialFileName);
  }

  RLOG(5, "VortexSimulation destroyed");
}

/*******************************************************************************
 * Clone function with additional graph
 ******************************************************************************/
Rcs::VortexSimulation* Rcs::VortexSimulation::clone(RcsGraph* newGraph) const
{
  return new Rcs::VortexSimulation(*this, newGraph);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::VortexSimulation::setTransformUpdateMutex(pthread_mutex_t* mutex)
{
  trafoUpdateLock = mutex;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::VortexSimulation::setGravity(const double gravity[3])
{
  universe->setGravity(gravity[0], gravity[1], gravity[2]);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::VortexSimulation::updateSensors()
{
  const Vx::VxConstraintSet& constraints = universe->getConstraints();

  for (auto it = constraints.begin(); it != constraints.end(); ++it)
  {
    Vx::VxRPRO* rpro = dynamic_cast<Vx::VxRPRO*>(*it);

    if (rpro != NULL)
    {
      RcsSensor* fts = (RcsSensor*) rpro->getUserDataPtr();

      if (fts != NULL)
      {
        // Update force and torque values
        double ftWrench[9];
        ftWrench[0] = rpro->getConstraintEquationForce(Vx::VxRPRO::kConstraintP0);
        ftWrench[1] = rpro->getConstraintEquationForce(Vx::VxRPRO::kConstraintP1);
        ftWrench[2] = rpro->getConstraintEquationForce(Vx::VxRPRO::kConstraintP2);
        ftWrench[3] = rpro->getConstraintEquationForce(Vx::VxRPRO::kConstraintA0);
        ftWrench[4] = rpro->getConstraintEquationForce(Vx::VxRPRO::kConstraintA1);
        ftWrench[5] = rpro->getConstraintEquationForce(Vx::VxRPRO::kConstraintA2);
        ftWrench[6] = 0.0;
        ftWrench[7] = 0.0;
        ftWrench[8] = 0.0;

        // Sensor's mass compensation (static forces only)
        double S_f_gravity[6];
        RcsSensor_computeStaticForceCompensation(getGraph(), fts, S_f_gravity);
        VecNd_subSelf(ftWrench, S_f_gravity, 6);

        // Update accelerations and rotate into body frame
        // \todo: Consider sensor offset transformation
        RCHECK_MSG(rpro->getMaxPartCount()==2, "Sensor has %d parts - must be 2",
                   rpro->getMaxPartCount());
        VortexBody* vxBdy = dynamic_cast<VortexBody*>(rpro->getPart(1));
        RCHECK(vxBdy);
        Vx::VxVector3 accel = vxBdy->getLinearAcceleration();
        ftWrench[6] = accel[0];
        ftWrench[7] = accel[1];
        ftWrench[8] = accel[2];
        Vec3d_rotateSelf(&ftWrench[6], vxBdy->A_PI.rot);

        // Copy into sensor's rawData array
        MatNd_fromArray(fts->rawData, ftWrench, 9);
      }
    }
  }



  for (unsigned int i=0; i<getGraph()->nSensors; ++i)
  {
    RcsSensor* si = &getGraph()->sensors[i];

    switch (si->type)
    {
      case RCSSENSOR_LOAD_CELL:
        //updateFTS(si);
        break;
      case RCSSENSOR_JOINT_TORQUE:
        updateJointTorqueSensor(si);
        break;
      case RCSSENSOR_CONTACT_FORCE:
        updateContactForceSensor(si);
        break;
      case RCSSENSOR_PPS:
        if (getEnablePPS()==true)
        {
          updatePPSSensor(si);
        }
        break;
      default:
        RLOG(4, "No update function for sensor type %d", si->type);
    }
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::VortexSimulation::initSettings(const PhysicsConfig* config)
{
  int grpIdx = 0, halflife = 5;
  Vx::VxSolverParameters* sParam = universe->getSolverParameters(grpIdx);

  // Read XML file
  xmlNodePtr node = config->getXMLRootNode()->children;
  bool found_param_node = false;

  while (node)
  {
    if (isXMLNodeName(node, "vortex_parameters"))
    {
      found_param_node = true;

      // first check for mandatory parameters
      if (!getXMLNodePropertyDouble(node, "integrator_dt", &this->integratorDt))
      {
        RFATAL("Physics configuration file did not contain the "
               "\"integrator_dt\" tag!");
      }
      Vx::VxFrame* frame = Vx::VxFrame::instance();
      frame->setTimeStep(this->integratorDt);

      // override default values that are used later (during joint creation
      // etc.)
      getXMLNodePropertyDouble(node, "body_linear_damping",
                               &this->bodyLinearDamping);
      getXMLNodePropertyDouble(node, "body_angular_damping",
                               &this->bodyAngularDamping);
      getXMLNodePropertyDouble(node, "joint_motor_loss",
                               &this->jointMotorLoss);
      getXMLNodePropertyDouble(node, "joint_lock_stiffness",
                               &this->jointLockStiffness);
      getXMLNodePropertyDouble(node, "joint_lock_damping",
                               &this->jointLockDamping);

      double value;
      if (getXMLNodePropertyDouble(node, "constraint_linear_compliance",
                                   &value))
      {
        sParam->setConstraintLinearCompliance(value);
        universe->setCriticalConstraintParameters(grpIdx, halflife);
      }

      if (getXMLNodePropertyDouble(node, "constraint_angular_compliance",
                                   &value))
      {
        sParam->setConstraintAngularCompliance(value);
        universe->setCriticalConstraintParameters(grpIdx, halflife);
      }

      bool useGroundPlane = true;
      getXMLNodePropertyBoolString(node, "use_ground_plane",
                                   &useGroundPlane);

      if (useGroundPlane)
      {
        Vx::VxPart* groundPlane = new Vx::VxPart();
        groundPlane->setName("GroundPlane");
        Vx::VxCollisionGeometry* g;
        g = new Vx::VxCollisionGeometry(new Vx::VxPlane());
        groundPlane->addCollisionGeometry(g);
        universe->addPart(groundPlane);
        RLOG(5, "Created ground plane");
      }


    }

    node = node->next;
  }

  if (found_param_node==false)
  {
    RLOG(1, "Physics configuration file did not contain a \"vortex parameters\""
         " node!");
    return false;
  }

  RLOG(5, "initPhysics finished");
  return true;
}

/*******************************************************************************
 * Define materials using XML file
 ******************************************************************************/
void Rcs::VortexSimulation::initMaterial(const PhysicsConfig* config)
{
  // copy PhysicsMaterial definitions into vortex material table
  PhysicsMaterial matDef = config->getFirstMaterial();

  char name[256];
  while (matDef)
  {
    // create corresponding vortex material
    matDef.getMaterialName(name);
    Vx::VxMaterial* material = getMaterialTable()->registerMaterial(name);

    // set common properties
    material->setFrictionCoefficient(Vx::VxMaterialBase::kFrictionAxisLinear,
                                     matDef.getFrictionCoefficient());

    material->setFrictionCoefficient(Vx::VxMaterialBase::kFrictionAxisAngularPrimary,
                                     matDef.getRollingFrictionCoefficient());
    material->setFrictionCoefficient(Vx::VxMaterialBase::kFrictionAxisAngularSecondary,
                                     matDef.getRollingFrictionCoefficient());

    material->setRestitution(matDef.getRestitution());

    // set vortex-specific properties from xml
    double value;
    char option[64];

    if (matDef.getString("friction_model", option, 64))
    {
      if (STRCASEEQ(option, "Box"))
      {
        material->setFrictionModel(Vx::VxMaterialBase::kFrictionAxisLinear,
                                   Vx::VxMaterialBase::kFrictionModelBox);
        material->setFrictionModel(Vx::VxMaterialBase::kFrictionAxisAngularPrimary,
                                   Vx::VxMaterialBase::kFrictionModelBox);
        material->setFrictionModel(Vx::VxMaterialBase::kFrictionAxisAngularSecondary,
                                   Vx::VxMaterialBase::kFrictionModelBox);
      }
      else if (STRCASEEQ(option, "ScaledBox"))
      {
        material->setFrictionModel(Vx::VxMaterialBase::kFrictionAxisLinear,
                                   Vx::VxMaterialBase::kFrictionModelScaledBox);
        material->setFrictionModel(Vx::VxMaterialBase::kFrictionAxisAngularPrimary,
                                   Vx::VxMaterialBase::kFrictionModelScaledBox);
        material->setFrictionModel(Vx::VxMaterialBase::kFrictionAxisAngularSecondary,
                                   Vx::VxMaterialBase::kFrictionModelScaledBox);
      }
      else if (STRCASEEQ(option, "ScaledBoxFast"))
      {
        material->setFrictionModel(Vx::VxMaterialBase::kFrictionAxisLinear,
                                   Vx::VxMaterialBase::kFrictionModelScaledBoxFast);
        material->setFrictionModel(Vx::VxMaterialBase::kFrictionAxisAngularPrimary,
                                   Vx::VxMaterialBase::kFrictionModelScaledBoxFast);
        material->setFrictionModel(Vx::VxMaterialBase::kFrictionAxisAngularSecondary,
                                   Vx::VxMaterialBase::kFrictionModelScaledBoxFast);
      }
      else
      {
        RLOG(1, "unknown friction model \"%s\" in material definition", option);
      }
    }

    if (matDef.getDouble("static_friction_scale", value))
    {
      material->setStaticFrictionScale(Vx::VxMaterialBase::kFrictionAxisLinear, value);
    }
    if (matDef.getDouble("slip", value))
    {
      material->setSlip(Vx::VxMaterialBase::kFrictionAxisLinear, value);
    }
    bool isd = false;
    if (matDef.getBoolean("integrated_slip_displacement", isd))
    {
      if (isd)
      {
        material->setIntegratedSlipDisplacement(Vx::VxMaterial::kIntegratedSlipDisplacementActivated);
      }
      else
      {
        material->setIntegratedSlipDisplacement(Vx::VxMaterial::kIntegratedSlipDisplacementDeactivated);
      }
    }
    if (matDef.getDouble("slide", value))
    {
      material->setSlide(Vx::VxMaterialBase::kFrictionAxisLinear, value);
    }
    if (matDef.getDouble("compliance", value))
    {
      material->setCompliance(value);
      // as a default, set critical damping
      // material->setDamping(universe->getCriticalDamping(value));
      // from the Vortex documentation:
      // Nearly optimal value of damping is given by:
      // damping = 5*time_step/compliance
      material->setDamping((5.0 * this->integratorDt) / value);
    }
    if (matDef.getDouble("damping", value))
    {
      material->setDamping(value);
    }
    if (matDef.getDouble("restitution_threshold", value))
    {
      material->setRestitutionThreshold(value);
    }
    if (matDef.getDouble("adhesive_force", value))
    {
      material->setAdhesiveForce(value);
    }

    matDef = matDef.next();
  }

  RLOG(5, "initMaterial finished");
}

/*******************************************************************************
 * Disables collision between body b0 and b1.
 ******************************************************************************/
void Rcs::VortexSimulation::disableCollision(const RcsBody* b0,
                                             const RcsBody* b1)
{
  Vx::VxPart* part0 = getPartPtr(b0);
  Vx::VxPart* part1 = getPartPtr(b1);

  if (part0==NULL)
  {
    RLOG(5, "No VxPart found in body \"%s\"", b0->bdyName);
    return;
  }

  if (part1==NULL)
  {
    RLOG(5, "No VxPart found in body \"%s\"", b1->bdyName);
    return;
  }

  universe->disablePairIntersect(part0, part1);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Rcs::VortexSimulation::disableCollisions()
{
  universe->disableAllPairs();
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::VortexSimulation::Contacts Rcs::VortexSimulation::getContacts()
{
  Contacts contacts;

  // get iterators for first and last contact
  Vx::VxUniverse::DynamicsContactIterator it, ie;
  it = universe->dynamicsContactBegin();
  ie = universe->dynamicsContactEnd();

  for (; it != ie; ++it)
  {
    Contact contact1;
    (*it)->getPosition(contact1.pos);
    (*it)->getForce(0, contact1.force);

    // \todo: strange, we have to draw both contacts to see them all. But there
    // are no complementing forces
    Contact contact2;
    (*it)->getPosition(contact2.pos);
    (*it)->getForce(1, contact2.force);

    contacts.push_back(contact1);
    contacts.push_back(contact2);
  }

  return contacts;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::VortexSimulation::setJointLimits(bool enable)
{
  this->jointLimitsActive = enable;

  pthread_mutex_lock(&this->extForceLock);
  const Vx::VxConstraintSet& constraints = universe->getConstraints();

  for (auto it = constraints.begin(); it != constraints.end(); ++it)
  {
    if ((*it)->isOfClassType(Vx::VxHinge::getStaticClassType()))
    {
      Vx::VxHinge* hinge = dynamic_cast<Vx::VxHinge*>(*it);
      hinge->setLimitsActive(Vx::VxHinge::kAngularCoordinate, enable);
    }
  }
  pthread_mutex_unlock(&this->extForceLock);
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::VortexSimulation::createCompositeBody(const RcsBody* body)
{
  if (body == NULL)
  {
    RLOG(1, "Body is NULL - not creating physics part");
    return false;
  }

  if (body->physicsSim == false)
  {
    NLOG(1, "Body \"%s\": physicsSim is false - skipping VxPart creation",
         body->bdyName);
    return false;
  }

  if (body->shape == NULL)
  {
    RLOG(1, "Body \"%s\" has no shape attached - skipping", body->bdyName);
    return false;
  }

#ifdef OLD_TOPO
  Vx::VxPart* parentPrt = body->parent ? getPartPtr(body->parent) : NULL;
#else
  Vx::VxPart* parentPrt = getPartPtrById(body->parentId);
#endif

  if ((body->physicsSim == RCSBODY_PHYSICS_DYNAMIC) &&
      (body->rigid_body_joints==false) &&
      (parentPrt==NULL))
  {
    RLOG(1, "Body \"%s\" needs rigid body joints! It is simulated dynamically "
         "and has no parent part", body->bdyName);
    return false;
  }

  RLOG(5, "Creating body \"%s\"", body->bdyName);

  // Traverse through shapes
  RcsShape** sPtr = &body->shape[0];


  Vx::VxCompositeCollisionGeometry* composite =
    new Vx::VxCompositeCollisionGeometry();


  while (*sPtr)
  {

    if (((*sPtr)->computeType & RCSSHAPE_COMPUTE_PHYSICS) == 0)
    {
      RLOG(5, "Skipping shape %s", RcsShape_name((*sPtr)->type));
      sPtr++;
      continue;
    }

    RLOG(5, "Creating shape %s", RcsShape_name((*sPtr)->type));

    // Set proper material
    char* materialName = (*sPtr)->material;
    Vx::VxMaterial* material = getMaterial(materialName);

    if (material == NULL)
    {
      RLOG(1, "%s shape of body \"%s\" has unknown material \"%s\", setting "
           "it to default material",
           RcsShape_name((*sPtr)->type), body->bdyName, materialName);
    }

    // create shape objects with the given material id
    switch ((*sPtr)->type)
    {
      case RCSSHAPE_SSL:
        RLOG(5, "Creating SSL for object \"%s\"", body->bdyName);
        composite->addCollisionGeometry(createCapsule(*sPtr, &body->A_BI, material));
        break;

      case RCSSHAPE_MESH:
        RLOG(5, "Creating mesh for object \"%s\"", body->bdyName);
        composite->addCollisionGeometry(createMesh(*sPtr, &body->A_BI, material));
        break;

      case RCSSHAPE_BOX:
        RLOG(5, "Creating box for object \"%s\"", body->bdyName);
        composite->addCollisionGeometry(createBox(*sPtr, &body->A_BI, material));
        break;

      case RCSSHAPE_SPHERE:
        RLOG(5, "Creating sphere for object \"%s\"", body->bdyName);
        composite->addCollisionGeometry(createSphere(*sPtr, &body->A_BI, material));
        break;

      case RCSSHAPE_CYLINDER:
        RLOG(5, "Creating cylinder for object \"%s\"", body->bdyName);
        composite->addCollisionGeometry(createCylinder(*sPtr, &body->A_BI, material));
        break;

      case RCSSHAPE_SSR:
        RLOG(5, "Creating SSR for object \"%s\"", body->bdyName);
        composite->addCollisionGeometry(createSSR(*sPtr, &body->A_BI, material));
        break;

      case RCSSHAPE_CONE:
        RLOG(5, "Creating cone for object \"%s\"", body->bdyName);
        composite->addCollisionGeometry(createCone(*sPtr, &body->A_BI, material));
        break;

      case RCSSHAPE_TORUS:
        RLOG(5, "Creating torus for object \"%s\"", body->bdyName);
        composite->addCollisionGeometry(createTorus(*sPtr, &body->A_BI, material));
        break;

      default:
        RLOG(3, "Shape %d (\"%s\") not yet implemented", (*sPtr)->type,
             RcsShape_name((*sPtr)->type));
    }

    sPtr++;

  } // while(*sPtr)

  // Check whether success, otherwise return
  if (composite->getCollisionGeometries().size() == 0)
  {
    RLOG(1, "Failed to create Vortex representation of body \"%s\": No shapes",
         body->bdyName);
    delete composite;
    return false;
  }

  VortexBody* p = new VortexBody(body);

  p->addCollisionGeometry(composite);

  // Assign dynamic properties
  if (body->m > 0.0)
  {
    p->setMassAndInertia(body->m, body->Inertia.rot);

    // if the inertia tensor is zero, Vortex calculates it based on the collision model
    // and sets the mass to 1 kg
    // if this fails for some reason (like for tri-meshes) vortexInertiaTensor is identity
    // (stl and obj seem to work fine)
    if (Mat3d_isZero((double(*)[3])body->Inertia.rot))
    {
      Vx::VxReal33 vortexInertiaTensor;
      p->getMassProperties().getInertiaTensorAbsolute(vortexInertiaTensor);

      if (Mat3d_isIdentity(vortexInertiaTensor))
      {
        // approximate as a tiny sphere
        Mat3d_setZero(vortexInertiaTensor);
        Mat3d_addConstToDiag(vortexInertiaTensor, 1e-8 * body->m);
        p->setMassAndInertia(body->m, vortexInertiaTensor);
      }
      else
      {
        // set to the correct mass, the inertia tensor is automatically rescaled
        p->setMass(body->m);
      }
    } //if(Mat3d_isZero(body->Inertia->rot))
  } //if(body->m > 0.0)

  // \todo: if body->Inertia->org is a zero vector, we cannot figure out whether
  // this is the default value or whether it is specified as a zero vector in the xml
  //
  // the COM is also calculated from the mesh by setMassAndInertia if the inertia tensor is zero
  // which we could/should use to replace the default
  p->setCOMOffset(body->Inertia.org[0], body->Inertia.org[1], body->Inertia.org[2]);
  p->setName(body->bdyName);
  p->setTransform(VxTransform_fromHTr(&body->A_BI));

  if (this->bodyLinearDamping >= 0.0)
  {
    p->setLinearVelocityDamping(this->bodyLinearDamping);
  }

  if (this->bodyAngularDamping >= 0.0)
  {
    p->setAngularVelocityDamping(this->bodyAngularDamping);
  }

  p->setLinearVelocity(Vx::VxVector3(body->x_dot[0], body->x_dot[1], body->x_dot[2]));
  p->setAngularVelocity(Vx::VxVector3(body->omega[0], body->omega[1], body->omega[2]));


  // Update body map
  std::pair<const RcsBody*, VortexBody*> bPair(body,p);
  this->bdyMap.insert(bPair);
  p->setUserDataPtr((void*)body);

  // Add to universe
  universe->addPart(p);

  RLOG(5, "Created body \"%s\"", body->bdyName);

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::VortexSimulation::updateTransformations()
{
  if (trafoUpdateLock != NULL)
  {
    pthread_mutex_lock(trafoUpdateLock);
  }

  const Vx::VxPartSet& parts = universe->getParts();

  for (auto it = parts.begin(); it != parts.end(); ++it)
  {
    VortexBody* vp = dynamic_cast<VortexBody*>(*it);
    if (vp != NULL)
    {
      HTr_fromVxTransform(&vp->A_PI, vp->getTransform());
    }

  }

  if (trafoUpdateLock != NULL)
  {
    pthread_mutex_unlock(trafoUpdateLock);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
const HTr* Rcs::VortexSimulation::getPhysicsTransformPtr(const RcsBody* body) const
{
  Rcs::VortexBody* vb = getPartPtr(body);

  if (vb == NULL)
  {
    return &body->A_BI;
  }

  return &vb->A_PI;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::VortexSimulation::step(double dt)
{
  simulate(dt, NULL, NULL, NULL, NULL, true);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::VortexSimulation::simulate(double dt,
                                     MatNd* q,
                                     MatNd* q_dot,
                                     MatNd* q_ddot,
                                     MatNd* T,
                                     bool control)
{
  if (dt<=0.0)
  {
    return;
  }

  if (control==true)
  {
    applyControl(dt);
  }

  // Get joint velocities before step to compute accelerations:
  // q_ddot = (q_dot-q_dot_prev)/dt
  if (q_ddot != NULL)
  {
    getJointVelocities(q_ddot);
    MatNd_constMulSelf(q_ddot, -1.0);
  }


  pthread_mutex_lock(&this->extForceLock);
  if (this->b_ext != NULL)
  {
    this->b_ext->addForceAtRelativePosition(Vx::VxVector3(this->F_ext),
                                            Vx::VxVector3(this->r_ext));
  }

  Vx::VxFrame* frame = Vx::VxFrame::instance();
  frame->setTimeStep(dt);
  frame->step();
  pthread_mutex_unlock(&this->extForceLock);
  updateSensors();
  updateTransformations();


  incrementTime(dt);


  // Update arrays
  if (q != NULL)
  {
    getJointAngles(q);
  }
  if (T != NULL)
  {
    getJointTorque(T);
  }
  if (q_dot != NULL)
  {
    getJointVelocities(q_dot);
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
  REXEC(9)
  {
    universe->verifyDynamics("Rcs::VortexSimulation", 3);
  }
}

/*******************************************************************************
 * Reset physics.
 ******************************************************************************/
void Rcs::VortexSimulation::reset()
{
  MatNd_copy(this->q_des, getGraph()->q);
  MatNd_setZero(this->q_dot_des);
  MatNd_setZero(this->T_des);

  // Update also the internal desired graph for the rigid body transforms.
  setControlInput(this->q_des, this->q_dot_des, this->T_des);




  const Vx::VxPartSet& parts = universe->getParts();

  for (auto it = parts.begin(); it != parts.end(); ++it)
  {
    VortexBody* vxBdy = dynamic_cast<VortexBody*>(*it);

    if (vxBdy != NULL)
    {
      if (vxBdy->body->physicsSim == RCSBODY_PHYSICS_KINEMATIC)
      {
        vxBdy->setTransformKinematic(VxTransform_fromHTr(&vxBdy->body->A_BI),
                                     getIntegratorDt());
      }
      else
      {
        vxBdy->setTransform(VxTransform_fromHTr(&vxBdy->body->A_BI));
      }

      vxBdy->setAngularVelocity(Vx::VxVector3(0.0, 0.0, 0.0));
      vxBdy->setLinearVelocity(Vx::VxVector3(0.0, 0.0, 0.0));
      vxBdy->resetDynamics();
    }

  }

#if 0
  RCSGRAPH_TRAVERSE_JOINTS(getGraph())
  {
    Vx::VxConstraint* c = (Vx::VxConstraint*) JNT->extraInfo;
    if (c != NULL)
    {
      c->resetDynamics();
      c->setLockPosition(0, MatNd_get(getGraph()->q, JNT->jointIndex, 0));
    }
  }
#else
  const Vx::VxConstraintSet& constraints = universe->getConstraints();

  for (auto it = constraints.begin(); it != constraints.end(); ++it)
  {
    Vx::VxConstraint* c = *it;
    c->resetDynamics();

    if (c->isOfClassType(Vx::VxHinge::getStaticClassType()) ||
        c->isOfClassType(Vx::VxPrismatic::getStaticClassType()))
    {
      RcsJoint* jnt = (RcsJoint*)c->getUserDataPtr();
      c->setLockPosition(0, MatNd_get(getGraph()->q, jnt->jointIndex, 0));
    }

  }

#endif

  universe->resetDynamics();
  universe->resetContacts();
  // step(1.0e-8);
}

/*******************************************************************************
 * Applies force F to the given body at location p (in world coordinates).
 ******************************************************************************/
void Rcs::VortexSimulation::setForce(const RcsBody* body,
                                     const double F[3],
                                     const double p[3])
{
  RCHECK_MSG(body != NULL, "Cannot set force on a NULL body!");

  Vx::VxPart* vxBdy = getPartPtr(body);
  if (vxBdy != NULL)
  {
    pthread_mutex_lock(&this->extForceLock);
    if (p != nullptr)
    {
      vxBdy->addForceAtAbsolutePosition(Vx::VxVector3(F[0], F[1], F[2]), Vx::VxVector3(p[0], p[1], p[2]));
    }
    else
    {
      vxBdy->addForce(Vx::VxVector3(F[0], F[1], F[2]));
    }
    pthread_mutex_unlock(&this->extForceLock);
  }
  else
  {
    NLOG(1, "Could not find a physical body for RcsBody '%s'", body->bdyName);
  }
}

/*******************************************************************************
 * Applies impulse F to the given body at location p (in world coordinates).
 ******************************************************************************/
void Rcs::VortexSimulation::applyImpulse(const RcsBody* body,
                                         const double F[3],
                                         const double p[3])
{
  RCHECK_MSG(body != NULL, "Cannot set force on a NULL body!");

  Vx::VxPart* vxBdy = getPartPtr(body);
  if (vxBdy != NULL)
  {
    pthread_mutex_lock(&this->extForceLock);
    if (p != nullptr)
    {
      vxBdy->addImpulseAtAbsolutePosition(Vx::VxVector3(F[0], F[1], F[2]), Vx::VxVector3(p[0], p[1], p[2]));
    }
    else
    {
      vxBdy->addImpulse(Vx::VxVector3(F[0], F[1], F[2]));
    }
    pthread_mutex_unlock(&this->extForceLock);
  }
  else
  {
    NLOG(1, "Could not find a physical body for RcsBody '%s'", body->bdyName);
  }
}

/*******************************************************************************
 * Applies force F to the body at vector r (in world coordinates).
 ******************************************************************************/
void Rcs::VortexSimulation::applyForce(const RcsBody* body,
                                       const double F[3],
                                       const double r[3])
{
  Vx::VxPart* vxBdy = getPartPtr(body);

  // Reset the force if no body is specified. Required for the force dragging.
  if (vxBdy == NULL)
  {
    pthread_mutex_lock(&this->extForceLock);
    Vec3d_setZero(this->F_ext);
    if (this->b_ext != NULL)
    {
      this->b_ext->addForceAtAbsolutePosition(Vx::VxVector3(0.0, 0.0, 0.0),
                                              Vx::VxVector3(0.0, 0.0, 0.0));
      // Reset body damping
      RcsBody* dragBody = (RcsBody*) this->b_ext->getUserDataPtr();
      RCHECK(dragBody);
      NLOG(0, "%s Damping reset to linear=%g angular=%g", dragBody->bdyName,
           this->bodyLinearDamping, this->bodyAngularDamping);
      if (dragBody->rigid_body_joints==true)
      {
        if (this->bodyLinearDamping >= 0.0)
        {
          this->b_ext->setLinearVelocityDamping(this->bodyLinearDamping);
        }
        if (this->bodyAngularDamping >= 0.0)
        {
          this->b_ext->setAngularVelocityDamping(this->bodyAngularDamping);
        }
      }   // Reset body damping

    }
    this->b_ext = NULL;
    pthread_mutex_unlock(&this->extForceLock);
    return;
  }

  // This is protected since this function might get called during
  // the step call.
  pthread_mutex_lock(&this->extForceLock);

  // Apply larger body damping for rigid bodyies when being dragged. That's
  // required for stability reasons.
  if ((body->rigid_body_joints==true) && (this->b_ext==NULL))
  {
    NLOG(0, "Setting body %s damping to linear=%g angular=%g",  body->bdyName,
         10.0*this->bodyLinearDamping, 100.0*this->bodyAngularDamping);
    vxBdy->setLinearVelocityDamping(10000.0*this->bodyLinearDamping);
    vxBdy->setAngularVelocityDamping(1000.0*this->bodyAngularDamping);
  }

  Vec3d_constMul(this->F_ext, F, 10.0);
  Vec3d_copy(this->r_ext, r);
  this->b_ext = vxBdy;
  pthread_mutex_unlock(&this->extForceLock);

  NLOG(0, "Applying F = %5.3f %5.3f %5.3f at %5.3f %5.3f %5.3f "
       "to body \"%s\"", this->F_ext[0], this->F_ext[1], this->F_ext[2],
       this->r_ext[0], this->r_ext[1], this->r_ext[2], body->bdyName);
}

/*******************************************************************************
 * Sets the kinematic transform to the body.
 ******************************************************************************/
void Rcs::VortexSimulation::applyTransform(const RcsBody* body, const HTr* A_BI)
{
  Vx::VxPart* vxBdy = getPartPtr(body);

  if (vxBdy == NULL)
  {
    RLOG(1, "No VxPart found in body \"%s\"", body ? body->bdyName : "NULL");
    return;
  }

  pthread_mutex_lock(&this->extForceLock);
  vxBdy->setTransform(VxTransform_fromHTr(A_BI));
  pthread_mutex_unlock(&this->extForceLock);
}

/*******************************************************************************
 * Set the linear velocity of a body
 ******************************************************************************/
void Rcs::VortexSimulation::applyLinearVelocity(const RcsBody* body,
                                                const double v[3])
{
  Vx::VxPart* vxBdy = getPartPtr(body);

  if (vxBdy == NULL)
  {
    RLOG(1, "Body \"%s\": Couldn't set velocity", body ? body->bdyName : "NULL");
    return;
  }

  NLOG(0, "Setting linear velocity of body \"%s\" to (%.3f, %.3f, %.3f)",
       body->bdyName, v[0], v[1], v[2]);

  pthread_mutex_lock(&this->extForceLock);
  vxBdy->setLinearVelocity(Vx::VxVector3(v));
  pthread_mutex_unlock(&this->extForceLock);
}

/*******************************************************************************
 * Get the linear velocity of a body
 ******************************************************************************/
void Rcs::VortexSimulation::getLinearVelocity(const RcsBody* body,
                                              double v[3]) const
{
  Vx::VxPart* vxBdy = getPartPtr(body);

  if (vxBdy == NULL)
  {
    RLOG(1, "Body \"%s\": Couldn't get velocity", body ? body->bdyName : "NULL");
    return;
  }

  Vx::VxVector3 v3 = vxBdy->getLinearVelocity();
  v[0] = v3[0];
  v[1] = v3[1];
  v[2] = v3[2];
}

/*******************************************************************************
 * Set the angular velocity of a body
 ******************************************************************************/
void Rcs::VortexSimulation::applyAngularVelocity(const RcsBody* body,
                                                 const double v[3])
{
  Vx::VxPart* vxBdy = getPartPtr(body);

  if (vxBdy == NULL)
  {
    return;
  }

  NLOG(0, "Setting angular velocity of body \"%s\" to (%.3f, %.3f, %.3f)",
       body->bdyName, v[0], v[1], v[2]);

  pthread_mutex_lock(&this->extForceLock);
  vxBdy->setAngularVelocity(Vx::VxVector3(v));
  pthread_mutex_unlock(&this->extForceLock);
}

/*******************************************************************************
 * Set the linear velocity of a body.
 ******************************************************************************/
void Rcs::VortexSimulation::getAngularVelocity(const RcsBody* body,
                                               double v[3]) const
{
  Vx::VxPart* vxBdy = getPartPtr(body);

  if (vxBdy == NULL)
  {
    RLOG(1, "Body \"%s\": Couldn't get velocity", body ? body->bdyName : "NULL");
    return;
  }

  Vx::VxVector3 v3 = vxBdy->getAngularVelocity();
  v[0] = v3[0];
  v[1] = v3[1];
  v[2] = v3[2];
}

/*******************************************************************************

 \brief Create the joint that connects the body to the previous body. This
 is currently very simple.

 - If the body is tagged as "kinematic", the corresponding VxPart's
 gravity is disabled, and it will be moved according to the bodies'
 kinematic (A_BI) transformation (So there is no "joint").

 - If the body is tagged as "dynamic" and the bodies are only
 connected with 1 rotational joint, a rotational joint is generated
 ("Springy joint"). Currently, only one joint is allowed. If there
 are more than one joint between the bodies, no joint will be
 created and the corresponding VxPart will have its 6 rigid body dofs.

 - If the body is tagged as "fixed", it is connected to its previous
 body with a fixed joint. There will be no relative movement
 between those bodies. If there are joints between the bodies, we
 will quit with an error message.

*******************************************************************************/
bool Rcs::VortexSimulation::createJoint(const RcsBody* body)
{
  if (body == NULL)
  {
    RLOG(1, "Body is NULL - skipping joint creation");
    return false;
  }

#ifdef OLD_TOPO
  Vx::VxPart* part0 = body->parent ? getPartPtr(body->parent) : NULL;
#else
  Vx::VxPart* part0 = getPartPtrById(body->parentId);
#endif
  Vx::VxPart* part1 = getPartPtr(body);

  switch (body->physicsSim)
  {

    case RCSBODY_PHYSICS_KINEMATIC:
    {
      RCHECK_MSG(part1, "No VxPart for body \"%s\"", body->bdyName);
      part1->setControl(Vx::VxPart::kControlAnimated);
    }
    break;

    case RCSBODY_PHYSICS_DYNAMIC:
    {
      if (part0 == NULL)
      {
        if (body->rigid_body_joints == false)
        {
          RLOG(1, "%s has no previous VxPart, therefore needs rigid body "
               "joints. Did you forget to add the rigid_body_joints flag?",
               body->bdyName);
        }
        break;
      }

      if (RcsBody_numJoints(body) != 1)
      {
        if (body->rigid_body_joints && RcsBody_numJoints(body) == 6)
        {
          break;
        }
        else
        {
          RFATAL("%s has %d joints - only one is currently supported",
                 body->bdyName, RcsBody_numJoints(body));
        }
      }

      double qInit = MatNd_get(getGraph()->q, body->jnt->jointIndex, 0);
      Vx::VxConstraint* vJnt = createJoint1D(part0, part1,
                                             this->jointLockStiffness,
                                             this->jointLockDamping,
                                             this->jointMotorLoss,
                                             qInit);
      if (vJnt != NULL)
      {
        universe->disablePairIntersect(part0, part1);
        universe->addConstraint(vJnt);
        vJnt->setUserDataPtr(body->jnt);
      }

      NLOG(0, "Created revolute joint for body \"%s\"", body->bdyName);
    }
    break;

    case RCSBODY_PHYSICS_FIXED:
    {
#ifdef OLD_TOPO
      if (body->parent == NULL)
#else
      if (body->parentId == -1)
#endif
      {
        RLOG(1, "Can't create fixed joint for body \"%s\": Parent is NULL",
             body->bdyName);
        break;
      }

      if (RcsBody_numJoints(body) != 0)
      {
        RLOG(1, "Can't create fixed joint for body \"%s\": Has %d joints "
             "(should be 0)", body->bdyName, RcsBody_numJoints(body));
        break;
      }

      // The constraints user data pointer will be set to the corresponding
      // RcsSensor
      Vx::VxConstraint* vJnt = createFixedJoint(part0, part1, getGraph());

      if (vJnt != NULL)
      {
        universe->disablePairIntersect(part0, part1);
        universe->addConstraint(vJnt);
        NLOG(0, "Created fixed joint (VxRPRO) between %s - %s",
             body->parent->bdyName, body->bdyName);
      }
      else
      {
#ifdef OLD_TOPO
        RLOG(4, "Creating fixed joint between \"%s\" and \"%s\" failed",
             body->paren ? body->parent->bdyName : "NULL", body->bdyName);
#else
        RLOG(4, "Creating fixed joint between \"%s\" and \"%s\" failed",
             body->parentId!=-1 ? getGraph()->bodies[body->parentId].bdyName : "NULL", body->bdyName);
#endif
      }
    }
    break;

    default:
      return false;
  }

  return true;
}

/*******************************************************************************
 * This function is not called by anyone, anyway
 ******************************************************************************/
void Rcs::VortexSimulation::setJointTorque(const MatNd* T_des)
{
  RcsStateType type;

  if (T_des->m == getGraph()->dof)
  {
    type = RcsStateFull;
  }
  else if (T_des->m == getGraph()->nJ)
  {
    type = RcsStateIK;
  }
  else
  {
    RFATAL("Dimension mismatch in T_des: has %d rows, but RcsGraph::dof=%d and"
           " RcsGraph::nJ=%d", T_des->m, getGraph()->dof, getGraph()->nJ);
  }


  const Vx::VxConstraintSet& constraints = universe->getConstraints();

  for (auto it = constraints.begin(); it != constraints.end(); ++it)
  {
    Vx::VxConstraint* c = *it;

    if (!c->isEnabled())
    {
      continue;
    }

    bool isHinge = c->isOfClassType(Vx::VxHinge::getStaticClassType());
    bool isPrism = c->isOfClassType(Vx::VxPrismatic::getStaticClassType());
    if ((!isHinge) && (!isPrism))
    {
      continue;
    }

    RcsJoint* jnt = (RcsJoint*) c->getUserDataPtr();
    int index = (type == RcsStateIK) ? jnt->jacobiIndex : jnt->jointIndex;
    Vx::VxReal Ti = MatNd_get(T_des, index, 0);
    Ti = Math_clip(Ti, -jnt->maxTorque, jnt->maxTorque);

    c->setLimitsActive(0, false);
    c->setMotorMaximumForce(0, fabs(Ti) + 1.0e-8);
    c->setMotorLoss(0, 0.0);
    c->setControl(0, Vx::VxConstraint::kControlMotorized);
    c->setMotorDesiredVelocity(0, 1.0e6 * Math_dsign(Ti));
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::VortexSimulation::getJointTorque(MatNd* T, RcsStateType sType) const
{
  const Vx::VxConstraintSet& constraints = universe->getConstraints();

  for (auto it = constraints.begin(); it != constraints.end(); ++it)
  {
    Vx::VxConstraint* c = *it;

    if (!c->isEnabled())
    {
      continue;
    }

    bool isHinge = c->isOfClassType(Vx::VxHinge::getStaticClassType());
    bool isPrism = c->isOfClassType(Vx::VxPrismatic::getStaticClassType());

    if ((!isHinge) && (!isPrism))
    {
      continue;
    }

    const RcsJoint* jnt = (const RcsJoint*) c->getUserDataPtr();
    Vx::VxReal Ti = c->getCoordinateForce(0);
    int index = (sType == RcsStateIK) ? jnt->jacobiIndex : jnt->jointIndex;
    MatNd_set(T, index, 0, Ti);

  }

  switch (sType)
  {
    case RcsStateFull:
      MatNd_reshapeAndSetZero(T, getGraph()->dof, 1);
      break;
    case  RcsStateIK:
      MatNd_reshapeAndSetZero(T, getGraph()->nJ, 1);
      break;
    default:
      RFATAL("Wrong state type: %d", sType);
  }
}

/*******************************************************************************
 * Copies the joint angles into q. The function automatically determines if
 * q is of size RcsStateIK or RcsstateFull.
 ******************************************************************************/
void Rcs::VortexSimulation::getJointAngles(MatNd* q, RcsStateType sType) const
{
  // First copy the joint angles from the kinematics to also get those
  // joints that have no physical representation
  switch (sType)
  {
    case RcsStateFull:
      MatNd_reshapeCopy(q, this->q_des);
      break;
    case  RcsStateIK:
      MatNd_reshape(q, getGraph()->nJ, 1);
      RcsGraph_stateVectorToIK(getGraph(), this->q_des, q);
      break;
    default:
      RFATAL("Wrong state type: %d", sType);
  }

  const Vx::VxConstraintSet& constraints = universe->getConstraints();

  for (auto it = constraints.begin(); it != constraints.end(); ++it)
  {
    Vx::VxConstraint* c = *it;

    if (!c->isEnabled())
    {
      continue;
    }

    bool isHinge = c->isOfClassType(Vx::VxHinge::getStaticClassType());
    bool isPrism = c->isOfClassType(Vx::VxPrismatic::getStaticClassType());
    if ((isHinge == false) && (isPrism == false))
    {
      continue;
    }

    const RcsJoint* jnt = (const RcsJoint*) c->getUserDataPtr();
    if ((sType == RcsStateIK) && (jnt->constrained == true))
    {
      continue;
    }
    int index = (sType == RcsStateIK) ? jnt->jacobiIndex : jnt->jointIndex;
    RCHECK_MSG(index >= 0, "joint: %s", jnt->name);
    Vx::VxReal qi = c->getCoordinateCurrentPosition(0);
    MatNd_set(q, index, 0, qi);
  }


  // then also override 6 joint angles for each rigid body with joints. This is
  // only required for the full state vector, since the rigid body joints are
  // constrained degrees of freedom.
  if (sType == RcsStateFull)
  {
    const Vx::VxPartSet& parts = universe->getParts();

    for (auto it = parts.begin(); it != parts.end(); ++it)
    {
      VortexBody* vxBdy = dynamic_cast<VortexBody*>(*it);

      if ((vxBdy!=NULL) && (vxBdy->body->rigid_body_joints==true))
      {
        HTr pose;
        HTr_fromVxTransform(&pose, vxBdy->getTransform());
        double* q_i = &q->ele[vxBdy->body->jnt->jointIndex];
#ifdef OLD_TOPO
        HTr* A_ParentI = vxBdy->body->parent ? vxBdy->body->parent->A_BI : NULL;
        RcsGraph_relativeRigidBodyDoFs(vxBdy->body, &pose, A_ParentI, q_i);
#else
        HTr* A_ParentI = vxBdy->body->parentId!=-1 ? &getGraph()->bodies[vxBdy->body->parentId].A_BI : NULL;
        RcsGraph_relativeRigidBodyDoFs(getGraph(), vxBdy->body, &pose, A_ParentI, q_i);
#endif
      }

    }

  }

}

/*******************************************************************************
 * If no physical joint is attached, so we set the velocity to 0.
 ******************************************************************************/
void Rcs::VortexSimulation::getJointVelocities(MatNd* q_dot,
                                               RcsStateType sType) const
{

  switch (sType)
  {
    case RcsStateFull:
      MatNd_reshapeAndSetZero(q_dot, getGraph()->dof, 1);
      break;
    case  RcsStateIK:
      MatNd_reshapeAndSetZero(q_dot, getGraph()->nJ, 1);
      break;
    default:
      RFATAL("Wrong state type: %d", sType);
  }
  const Vx::VxConstraintSet& constraints = universe->getConstraints();

  for (auto it = constraints.begin(); it != constraints.end(); ++it)
  {
    Vx::VxConstraint* c = *it;

    if (!c->isEnabled())
    {
      continue;
    }

    bool isHinge = c->isOfClassType(Vx::VxHinge::getStaticClassType());
    bool isPrism = c->isOfClassType(Vx::VxPrismatic::getStaticClassType());

    if ((isHinge == false) && (isPrism == false))
    {
      continue;
    }

    const RcsJoint* jnt = (const RcsJoint*) c->getUserDataPtr();

    if ((sType == RcsStateIK) && (jnt->constrained == true))
    {
      continue;
    }

    int index = (sType == RcsStateIK) ? jnt->jacobiIndex : jnt->jointIndex;
    RCHECK_MSG(index >= 0, "joint: %s", jnt->name);
    Vx::VxReal qi = c->getCoordinateVelocity(0);
    MatNd_set(q_dot, index, 0, qi);

  }


  // Update the rigid body dofs. Since they are constrained, it is only done
  // for the full state vector.
  if (sType == RcsStateFull)
  {
    const Vx::VxPartSet& parts = universe->getParts();

    for (auto it = parts.begin(); it != parts.end(); ++it)
    {
      VortexBody* vxBdy = dynamic_cast<VortexBody*>(*it);

      if ((vxBdy!=NULL) && (vxBdy->body->rigid_body_joints==true))
      {
        Vx::VxVector3 linearVelocity = vxBdy->getLinearVelocity();
        Vx::VxVector3 angularVelocity = vxBdy->getAngularVelocity();

        double* q6 = &q_dot->ele[vxBdy->body->jnt->jointIndex];
        linearVelocity.get<double>(q6, q6+1, q6+2);
        angularVelocity.get<double>(q6+3, q6+4, q6+5);
      }

    }

  }   // sType==RcsStateFull

}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::VortexSimulation::setMassAndInertiaFromPhysics(RcsGraph* graph_)
{
  bool isGraphFromPhysics = (graph_ == getGraph());

  const Vx::VxPartSet& parts = universe->getParts();

  for (auto it = parts.begin(); it != parts.end(); ++it)
  {
    Vx::VxPart* p = *it;
    RcsBody* body = NULL;

    if (isGraphFromPhysics == true)
    {
      body = (RcsBody*) p->getUserDataPtr();
    }
    else
    {
      body = RcsGraph_getBodyByName(graph_, p->getName());
    }

    if (body == NULL)
    {
      NLOG(1, "setMassAndInertiaFromPhysics(): VxPart \"%s\" has no body "
           "attached", p->getName());
      continue;
    }

    Vx::VxReal33 I;
    Vx::VxVector3 r_com;
    p->getMassProperties().getInertiaTensorLocal(I);
    body->m = p->getMassProperties().getMass();
    r_com = p->getMassProperties().getCOMPositionLocal();
    bool hasMassProperties = p->getControl() != Vx::VxPart::kControlStatic;

    // otherwise uninitialized values are written
    if (hasMassProperties == true)
    {
      Vec3d_copy(body->Inertia.org, r_com);
      Mat3d_copy(body->Inertia.rot, I);
    }
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
double Rcs::VortexSimulation::getIntegratorDt() const
{
  return this->integratorDt;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::VortexSimulation::getPhysicsTransform(HTr* A_BI,
                                                const RcsBody* body) const
{
  if (body == NULL)
  {
    RLOG(4, "Can't get transformation of NULL body");
    return;
  }

  Vx::VxPart* vxBdy = getPartPtr(body);

  if (vxBdy != NULL)
  {
    pthread_mutex_lock(&this->extForceLock);
    HTr_fromVxTransform(A_BI, vxBdy->getTransform());
    pthread_mutex_unlock(&this->extForceLock);
  }
  // Body has no physics attached - we check one level up if it is attached to
  // a physics body and in that case compute the relative transformation.
  else
  {
#ifdef OLD_TOPO
    Vx::VxPart* vxParent = getPartPtr(body->parent);
#else
    Vx::VxPart* vxParent = getPartPtrById(body->parentId);
#endif

    // If there is no parent in the physics or if there are joints between
    // body and parent, we give up.
#ifdef OLD_TOPO
    const RcsJoint* parentJnt = body->parent->jnt;
#else
    const RcsBody* parent = RcsBody_getConstParent(getGraph(), body);
    const RcsJoint* parentJnt = parent ? parent->jnt : NULL;
#endif

    if ((vxParent==NULL) || (parentJnt!=NULL))
    {
      HTr_setIdentity(A_BI);
      RLOG(1, "Couldn't get physics transformation of body \"%s\"",
           body ? body->bdyName : "NULL");
    }
    else
    {
      HTr A_PI;
      pthread_mutex_lock(&this->extForceLock);
      HTr_fromVxTransform(&A_PI, vxParent->getTransform());
      pthread_mutex_unlock(&this->extForceLock);
      HTr_transform(A_BI, &A_PI, &body->A_BP);
    }
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::VortexSimulation::removeJoint(RcsBody* body)
{
  if (body == NULL)
  {
    RLOG(1, "Body is NULL - skipping joint removal");
    return false;
  }


  if (body->physicsSim == RCSBODY_PHYSICS_KINEMATIC)
  {
    Vx::VxPart* part1 = getPartPtr(body);

    if (part1 != NULL)
    {
      part1->setControl(Vx::VxPart::kControlDynamic);
      return true;
    }

    return false;
  }

#ifdef OLD_TOPO
  Vx::VxPart* part0 = body->parent ? getPartPtr(body->parent) : NULL;
#else
  Vx::VxPart* part0 = getPartPtrById(body->parentId);
#endif

  if (part0 == NULL)
  {
    RLOG(1, "%s has no prev. VxPart", body->bdyName);
    return false;
  }

  Vx::VxConstraint* vJnt = NULL;

  const Vx::VxConstraintSet& constraints = universe->getConstraints();

  for (auto it = constraints.begin(); it != constraints.end(); ++it)
  {
    if ((*it)->getUserDataPtr() == body->jnt)
    {
      vJnt = *it;
      break;
    }
  }

  if (vJnt != NULL)
  {
    Vx::VxPart* part1 = getPartPtr(body);
    universe->enablePairIntersect(part0, part1);
    universe->removeConstraint(vJnt);
    delete vJnt;
    RLOG(0, "Removed joint for body \"%s\" (Joint \"%s\")", body->bdyName, body->jnt ? body->jnt->name : "NULL");

    return true;
  }

  return false;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::VortexSimulation::removeBodyConstraints(RcsBody* body)
{
  bool success = removeJoint(body);
  Vx::VxPart* part = getPartPtr(body);

  if (part != NULL)
  {
    part->resetDynamics();
    part->setLinearVelocity(Vx::VxVector3(0.0, 0.0, 0.0));
    part->setAngularVelocity(Vx::VxVector3(0.0, 0.0, 0.0));
  }
  else
  {
    RLOG(1, "Couldn't reset dynamics of part \"%s\"", body->bdyName);
  }

  return success;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::VortexSimulation::addBodyConstraints(RcsBody* body)
{
  this->T_des = MatNd_realloc(this->T_des, getGraph()->dof, 1);
  this->q_des = MatNd_realloc(this->q_des, getGraph()->dof, 1);
  MatNd_copy(this->q_des, getGraph()->q);
  this->q_dot_des = MatNd_realloc(this->q_dot_des, getGraph()->dof, 1);

  if (body == NULL)
  {
    return false;
  }
  applyTransform(body, &body->A_BI);
  return createJoint(body);
}

/*******************************************************************************
 *
 ******************************************************************************/
const char* Rcs::VortexSimulation::getClassName() const
{
  return className;
}

/*******************************************************************************
 * Updates a force-torque sensor
 ******************************************************************************/
bool Rcs::VortexSimulation::updateFTS(RcsSensor* fts)
{
  RcsBody* ftsBdy = &getGraph()->bodies[fts->bodyId];

  // For a load cell sensor, the body must be attached by a fixed joint
  if (ftsBdy->physicsSim != RCSBODY_PHYSICS_FIXED)
  {
    RLOG(4, "Load cell sensors must be attached to a body of type "
         "\"fixed\" -> not updating sensor %s", fts->name);
    return false;
  }

  // Get the Vortex part that the sensor is attached to
  VortexBody* part = getPartPtr(ftsBdy);
  if (part == NULL)
  {
    RLOG(4, "Load cell sensors %s not attached to VxPart", fts->name);
    return false;
  }

  // Search for the VxRPRO joint
  Vx::VxRPRO* joint = NULL;
  for (size_t i = 0; i < part->getConstraintCount() && joint == NULL; i++)
  {
    Vx::VxConstraint* c = part->getConstraint(i);

    if (c->isOfClassType(Vx::VxRPRO::getStaticClassType()))
    {
      joint = dynamic_cast<Vx::VxRPRO*>(c);
    }
  }

  if (joint == NULL)
  {
    RLOG(4, "Load cell sensors %s not attached through VxRPRO", fts->name);
    return false;
  }

  // Update force and torque values
  double ftWrench[9];
  ftWrench[0] = joint->getConstraintEquationForce(Vx::VxRPRO::kConstraintP0);
  ftWrench[1] = joint->getConstraintEquationForce(Vx::VxRPRO::kConstraintP1);
  ftWrench[2] = joint->getConstraintEquationForce(Vx::VxRPRO::kConstraintP2);
  ftWrench[3] = joint->getConstraintEquationForce(Vx::VxRPRO::kConstraintA0);
  ftWrench[4] = joint->getConstraintEquationForce(Vx::VxRPRO::kConstraintA1);
  ftWrench[5] = joint->getConstraintEquationForce(Vx::VxRPRO::kConstraintA2);

  // Sensor's mass compensation (static forces only)
  double S_f_gravity[6];
  RcsSensor_computeStaticForceCompensation(getGraph(), fts, S_f_gravity);
  VecNd_subSelf(ftWrench, S_f_gravity, 6);

  // Update accelerations and rotate into body frame
  Vx::VxVector3 accel = part->getLinearAcceleration();
  ftWrench[6] = accel[0];
  ftWrench[7] = accel[1];
  ftWrench[8] = accel[2];

  Vec3d_rotateSelf(&ftWrench[6], part->A_PI.rot);

  // Copy into sensor's rawData array
  MatNd_fromArray(fts->rawData, ftWrench, 6);

  return true;
}

/*******************************************************************************
 * Updates a force-torque sensor
 ******************************************************************************/
bool Rcs::VortexSimulation::updateJointTorqueSensor(RcsSensor* sensor)
{
  RcsBody* sensorBdy = &getGraph()->bodies[sensor->bodyId];

  // For a joint torque sensor, the body must be attached by a revolute joint
  if (sensorBdy->physicsSim != RCSBODY_PHYSICS_DYNAMIC)
  {
    RLOG(1, "Joint torque sensors must be attached to a body of type "
         "\"dynamic\" -> not creating sensor %s", sensor->name);
    return false;
  }

  // Get the Vortex part that the sensor is attached to
  Vx::VxPart* part = getPartPtr(sensorBdy);
  if (part == NULL)
  {
    RLOG(4, "Torque sensors %s not attached to VxPart", sensor->name);
    return false;
  }

  // Search through the joints of the part
  Vx::VxHinge* joint = NULL;
  for (size_t i = 0; i < part->getConstraintCount() && joint == NULL; i++)
  {
    Vx::VxConstraint* c = part->getConstraint(i);

    if (c->isOfClassType(Vx::VxHinge::getStaticClassType()))
    {
      joint = dynamic_cast<Vx::VxHinge*>(c);
    }
  }

  if (joint == NULL)
  {
    RLOG(4, "Torque sensor %s not attached through VxHinge", sensor->name);
    return false;
  }

  sensor->rawData->ele[0] =
    joint->getCoordinateForce(Vx::VxHinge::kAngularCoordinate);

  return true;
}

/*******************************************************************************
 * Updates a contact force sensor
 ******************************************************************************/
bool Rcs::VortexSimulation::updateContactForceSensor(RcsSensor* sensor)
{
  RcsBody* sensorBdy = &getGraph()->bodies[sensor->bodyId];

  // Get the Vortex part that the sensor is attached to
  VortexBody* part = getPartPtr(sensorBdy);
  if (part == NULL)
  {
    RLOG(4, "Contact force sensor %s not attached to VxPart", sensor->name);
    return false;
  }

  HTr A_SI;
  const HTr* A_SB = &sensor->offset;
  HTr_transform(&A_SI, &part->A_PI, A_SB);

  double forceVec[3], sensorNormal[3] = {1.0, 0.0, 0.0};
  Vec3d_transRotateSelf(sensorNormal, A_SI.rot);
  Vec3d_setZero(forceVec);

  // Search current contacts for mount body
  Vx::VxPart::DynamicsContactIterator it = part->dynamicsContactBegin();
  Vx::VxPart::DynamicsContactIterator ie = part->dynamicsContactEnd();

  while (it != ie)
  {
    Vx::VxPart* pair[2];
    (*it)->getPartPair(pair, pair + 1);
    if (((pair[0] == part) || (pair[1] == part)))
    {
      double f[3];
      (*it)->getForce((pair[0] == part ? 0 : 1), f);

      // check if contact is relevant, if yes, add it to the force vector
      // only push, no pull
      if (Vec3d_diffAngle(sensorNormal, f) < M_PI_2 * 0.8)
      {
        Vec3d_addSelf(forceVec, f);
      }
    }
    it++;
  }

  sensor->rawData->ele[0] = Vec3d_getLength(forceVec);

  return true;
}

/*******************************************************************************
 * Updates a tactile array sensor
 ******************************************************************************/
bool Rcs::VortexSimulation::updatePPSSensor(RcsSensor* sensor)
{
  RcsBody* sensorBdy = &getGraph()->bodies[sensor->bodyId];

  // Get the Vortex part that the sensor is attached to
  VortexBody* vxPart = getPartPtr(sensorBdy);

  if (vxPart == NULL)
  {
    RLOG(4, "Contact force sensor %s not attached to VxPart", sensor->name);
    return false;
  }

  // Search current contacts for mount body
  Vx::VxPart::DynamicsContactIterator it = vxPart->dynamicsContactBegin();
  Vx::VxPart::DynamicsContactIterator ie = vxPart->dynamicsContactEnd();
  double contactForce[3];
  Vec3d_setZero(contactForce);

  while (it != ie)
  {
    Vx::VxPart* part[2];
    double f[3];
    (*it)->getPartPair(part, part + 1);
    (*it)->getForce((part[0] == vxPart ? 0 : 1), f);
    Vec3d_constMulAndAddSelf(contactForce, f, 1.0);
    it++;
  }

  return RcsSensor_computePPS(getGraph(), sensor, sensor->rawData, contactForce);
}

/*******************************************************************************
 * Sets compliance properties for all hinge joints
 ******************************************************************************/
void Rcs::VortexSimulation::setJointCompliance(const MatNd* stiffness,
                                               const MatNd* damping)
{
  const Vx::VxConstraintSet& constraints = universe->getConstraints();

  for (auto it = constraints.begin(); it != constraints.end(); ++it)
  {
    if ((*it)->isOfClassType(Vx::VxHinge::getStaticClassType()))
    {
      Vx::VxHinge* hinge = dynamic_cast<Vx::VxHinge*>(*it);
      RcsJoint* jnt = (RcsJoint*) hinge->getUserDataPtr();
      RCHECK(jnt);

      if (jnt->ctrlType == RCSJOINT_CTRL_POSITION)
      {
        // Setting the stiffness to zero results in weird movements and joint
        // constraints not satisfied any more.
        const double eps = 1.0e-8;
        int idx = jnt->jointIndex;
        double stiffness_i = MatNd_get(stiffness, idx, 0);

        if ((stiffness_i>-eps) && (stiffness_i<0.0))
        {
          stiffness_i = -eps;
        }
        else if ((stiffness_i<eps) && (stiffness_i>=0.0))
        {
          stiffness_i = eps;
        }

        hinge->setLockStiffness(Vx::VxHinge::kAngularCoordinate, stiffness_i);

        if (damping!=NULL)
        {
          hinge->setLockDamping(Vx::VxHinge::kAngularCoordinate,
                                MatNd_get(damping, idx, 0));
        }
        hinge->setLimitsActive(Vx::VxHinge::kAngularCoordinate, true);
      }

    }
  }
}

/*******************************************************************************
 * Sets compliance properties for all hinge joints
 ******************************************************************************/
void Rcs::VortexSimulation::getJointCompliance(MatNd* stiffness,
                                               MatNd* damping) const
{
  MatNd_reshapeAndSetZero(stiffness, getGraph()->dof, 1);

  if (damping!=NULL)
  {
    MatNd_reshapeAndSetZero(damping, getGraph()->dof, 1);
  }

  const Vx::VxConstraintSet& constraints = universe->getConstraints();

  for (auto it = constraints.begin(); it != constraints.end(); ++it)
  {
    if ((*it)->isOfClassType(Vx::VxHinge::getStaticClassType()))
    {
      Vx::VxHinge* hinge = dynamic_cast<Vx::VxHinge*>(*it);
      const RcsJoint* jnt = (const RcsJoint*) hinge->getUserDataPtr();
      RCHECK(jnt);

      if (jnt->ctrlType == RCSJOINT_CTRL_POSITION)
      {
        unsigned int idx = jnt->jointIndex;

        stiffness->ele[idx] =
          hinge->getLockStiffness(Vx::VxHinge::kAngularCoordinate);

        if (damping!=NULL)
        {
          damping->ele[idx] =
            hinge->getLockDamping(Vx::VxHinge::kAngularCoordinate);
        }
      }

    }
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::VortexSimulation::applyControl(double dt)
{
  // Set transformation of all kinematic parts. We are looking through all
  // VortexRigidBodies that are kinematically moved in the physics. For each
  // of these, we retrieve the corresponding body in the internalDesiredGraph
  // (which is set to the state of the desired joint angles) and set its
  // transformation accordingly.
  const Vx::VxPartSet& parts = universe->getParts();

  for (auto it = parts.begin(); it != parts.end(); ++it)
  {
    VortexBody* vxBdy = dynamic_cast<VortexBody*>(*it);

    if ((vxBdy!=NULL) && (vxBdy->body->physicsSim==RCSBODY_PHYSICS_KINEMATIC))
    {
      RcsBody* transformBdy = RcsGraph_getBodyByName(getGraph(),
                                                     vxBdy->body->bdyName);

      RCHECK_MSG(transformBdy, "This must never happen: body \"%s\" is not"
                 " contained in internalDesiredGraph", vxBdy->body->bdyName);

      vxBdy->setTransformKinematic(VxTransform_fromHTr(&transformBdy->A_BI), dt);
    }

  }







  const Vx::VxConstraintSet& constraints = universe->getConstraints();

  for (auto it = constraints.begin(); it != constraints.end(); ++it)
  {
    Vx::VxConstraint* c = *it;

    if (c->getCoordinateCount() != 1)
    {

      if (c->getCoordinateCount() > 1)   // 0 for fixed joints
      {
        RLOG(4, "Skipping constraint %s with %d dofs (only 1 is supported)",
             c->getName(), c->getCoordinateCount());
      }
      continue;
    }

    RcsJoint* jnt = (RcsJoint*) c->getUserDataPtr();

    if (jnt == NULL)
    {
      RLOG(4, "Skipping constraint %s - no userData found", c->getName());
      continue;
    }

    // Velocity-controlled joints
    if (jnt->ctrlType == RCSJOINT_CTRL_VELOCITY)
    {
      Vx::VxReal q_dot_cmd = MatNd_get(this->q_dot_des, jnt->jointIndex, 0);
      c->setMotorDesiredVelocity(0, q_dot_cmd);
      // c->setControl(coordinate, Vx::VxConstraint::kControlMotorized);
      // c->setMotorMaximumForce(coordinate, jnt->maxTorque);
    }
    // Torque-controlled joints
    else if (jnt->ctrlType == RCSJOINT_CTRL_TORQUE)
    {
      double Ti = MatNd_get(this->T_des, jnt->jointIndex, 0);
      Ti = Math_clip(Ti, -jnt->maxTorque, jnt->maxTorque);

      // If the max. force is set to 0, the motor gets deactivated. We
      // therefore always set it to  tiny value.
      c->setMotorMaximumForce(0, fabs(Ti) + 1.0e-8);
      c->setControl(0, Vx::VxConstraint::kControlMotorized);
      c->setMotorLoss(0, 0.0);

      if (Ti==0.0)
      {
        c->setMotorDesiredVelocity(0, 0.0);
      }
      else
      {
        c->setMotorDesiredVelocity(0, Ti > 0.0 ? 1.0e6 : -1.0e6);
      }
    }
    // Position-controlled joints
    else if (jnt->ctrlType == RCSJOINT_CTRL_POSITION)
    {
      Vx::VxReal q_cmd = MatNd_get(this->q_des, jnt->jointIndex, 0);

      double lockCurr = c->getLockPosition(0);
      double deltaPos = q_cmd-lockCurr;

      if (RcsJoint_isRotation(jnt))
      {
        while (deltaPos > M_PI)
        {
          deltaPos -= M_PI;
        }

        while (deltaPos < -M_PI)
        {
          deltaPos += M_PI;
        }
      }

      double lockVel = deltaPos/dt;
      c->setLockMaximumForce(0, jnt->maxTorque);
      c->setLockDamping(0, this->jointLockDamping);
      c->setLockStiffness(0, this->jointLockStiffness);
      c->setLockVelocity(0, lockVel);
      // c->setLockPosition(0, q_cmd);
    }
    // Should never happen
    else
    {
      RFATAL("Unknown cntrlType: %d", jnt->ctrlType);
    }

  }

}

/*******************************************************************************
 *
 ******************************************************************************/
Vx::VxPart* Rcs::VortexSimulation::getPart(const char* name)
{
  if (name==NULL)
  {
    return NULL;
  }

  const Vx::VxPartSet& parts = universe->getParts();

  for (auto it = parts.begin(); it != parts.end(); ++it)
  {
    if (STREQ((*it)->getName(), name))
    {
      return *it;
    }
  }

  return NULL;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::VortexSimulation::printMaterialTable(std::ostream& out) const
{
  Vx::VxMaterialTable* mt = getMaterialTable();

  for (size_t i=0; i<mt->getMaterialCount(); ++i)
  {
    printMaterial(mt->getMaterial(i), out);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::VortexSimulation::printMaterialTable() const
{
  printMaterialTable(std::cout);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Rcs::VortexSimulation::print() const
{
  universe->verifyDynamics("Rcs::VortexSimulation", 3);

  printMaterialTable();

  auto it = bdyMap.begin();

  // Iterate over the map using iterator
  printf("bdyMap:\n");
  while (it != bdyMap.end())
  {
    const RcsBody* bdy_i = it->first;
    VortexBody* vbdy_i = it->second;
    printf("Body %s - %s\n", bdy_i->bdyName, vbdy_i->body->bdyName);
    it++;
  }
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::VortexSimulation::setParameter(ParameterCategory category,
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

  pthread_mutex_lock(&this->extForceLock);

  switch (category)
  {
    case Material:
    {
      Vx::VxMaterial* material = getMaterial(name);

      if (material==NULL)
      {
        RLOG(1, "Material \"%s\" not found", name);
        break;
      }

      if (STRCASEEQ(type, "restitution"))
      {
        material->setRestitution(value);
        success = true;
      }
      else if (STRCASEEQ(type, "compliance"))
      {
        material->setCompliance(value);
        success = true;
      }
      else if (STRCASEEQ(type, "damping"))
      {
        material->setDamping(value);
        success = true;
      }
      else if (STRCASEEQ(type, "adhesiveforce"))
      {
        material->setAdhesiveForce(value);
        success = true;
      }
      else if (STRCASEEQ(type, "linearfriction"))
      {
        material->setFrictionCoefficient(Vx::VxMaterialBase::kFrictionAxisLinearPrimary, value);
        material->setFrictionCoefficient(Vx::VxMaterialBase::kFrictionAxisLinearSecondary, value);
        success = true;
      }
      else if (STRCASEEQ(type, "linearfrictionprimary"))
      {
        material->setFrictionCoefficient(Vx::VxMaterialBase::kFrictionAxisLinearPrimary, value);
        success = true;
      }
      else if (STRCASEEQ(type, "linearfrictionsecondary"))
      {
        material->setFrictionCoefficient(Vx::VxMaterialBase::kFrictionAxisLinearSecondary, value);
        success = true;
      }
      else if (STRCASEEQ(type, "angularfriction"))
      {
        material->setFrictionCoefficient(Vx::VxMaterialBase::kFrictionAxisAngularPrimary, value);
        material->setFrictionCoefficient(Vx::VxMaterialBase::kFrictionAxisAngularSecondary, value);
        material->setFrictionCoefficient(Vx::VxMaterialBase::kFrictionAxisAngularNormal, value);
        success = true;
      }
      else if (STRCASEEQ(type, "angularfrictionprimary"))
      {
        material->setFrictionCoefficient(Vx::VxMaterialBase::kFrictionAxisAngularPrimary, value);
        success = true;
      }
      else if (STRCASEEQ(type, "angularfrictionsecondary"))
      {
        material->setFrictionCoefficient(Vx::VxMaterialBase::kFrictionAxisAngularSecondary, value);
        success = true;
      }
      else if (STRCASEEQ(type, "angularfrictionnormal"))
      {
        material->setFrictionCoefficient(Vx::VxMaterialBase::kFrictionAxisAngularNormal, value);
        success = true;
      }

      break;
    }

    case Simulation:
    {
      if (STRCASEEQ(name, "gravity"))
      {
        Vx::VxReal3 gravity;
        universe->getGravity(gravity);

        if (STRCASEEQ(type, "x"))
        {
          gravity[0] = value;
          universe->setGravity(gravity);
          success = true;
        }
        else if (STRCASEEQ(type, "y"))
        {
          gravity[1] = value;
          universe->setGravity(gravity);
          success = true;
        }
        else if (STRCASEEQ(type, "z"))
        {
          gravity[2] = value;
          universe->setGravity(gravity);
          success = true;
        }
      }
      break;
    }

    case Body:
    {
      Vx::VxPart* vxBdy = getPart(name);

      if (vxBdy != NULL)
      {
        if (STRCASEEQ(type, "mass"))
        {
          // Inertia tensor is automatically rescaled by Vortex
          vxBdy->setMass(value);
          success = true;
        }
        else if (STRCASEEQ(type, "com_x"))
        {
          const Vx::VxMassProperties& mp = vxBdy->getMassProperties();
          Vx::VxReal3 com;
          mp.getCOMPositionLocal(com);
          com[0] = value;
          vxBdy->setCOMOffset(com);
          success = true;
        }
        else if (STRCASEEQ(type, "com_y"))
        {
          const Vx::VxMassProperties& mp = vxBdy->getMassProperties();
          Vx::VxReal3 com;
          mp.getCOMPositionLocal(com);
          com[1] = value;
          vxBdy->setCOMOffset(com);
          success = true;
        }
        else if (STRCASEEQ(type, "com_z"))
        {
          const Vx::VxMassProperties& mp = vxBdy->getMassProperties();
          Vx::VxReal3 com;
          mp.getCOMPositionLocal(com);
          com[2] = value;
          vxBdy->setCOMOffset(com);
          success = true;
        }
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

  pthread_mutex_unlock(&this->extForceLock);

  if (success==false)
  {
    RLOG(1, "Failed to set parameter with name \"%s\" and type \"%s\" (value %g, "
         "category %d)", name, type, value, category);
  }

  return success;
}

/*******************************************************************************
 *
 ******************************************************************************/
Vx::VxUniverse* Rcs::VortexSimulation::getUniverse()
{
  return this->universe;
}

/*******************************************************************************
 *
 ******************************************************************************/
const Vx::VxUniverse* Rcs::VortexSimulation::getUniverse() const
{
  return this->universe;
}

/*******************************************************************************
 *
 ******************************************************************************/
Vx::VxMaterial* Rcs::VortexSimulation::getMaterial(const char* name) const
{
  if (name==NULL)
  {
    return NULL;
  }

  return getMaterialTable()->getMaterial(name);
}

/*******************************************************************************
 *
 ******************************************************************************/
Vx::VxMaterialTable* Rcs::VortexSimulation::getMaterialTable() const
{
  return universe->getRigidBodyResponseModel()->getMaterialTable();
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::VortexBody* Rcs::VortexSimulation::getPartPtr(const RcsBody* body) const
{
  if (body == NULL)
  {
    return NULL;
  }

  auto it = bdyMap.find(body);

  return (it == bdyMap.end()) ? NULL : it->second;
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::VortexBody* Rcs::VortexSimulation::getPartPtrById(int bodyId) const
{
  if (bodyId == -1)
  {
    return NULL;
  }

  const RcsBody* body = &getGraph()->bodies[bodyId];

  return getPartPtr(body);
}

/*******************************************************************************
*
******************************************************************************/
bool Rcs::VortexSimulation::removeBody(const char* name)
{
  RcsBody* bdy = RcsGraph_getBodyByName(getGraph(), name);
  if (bdy == NULL)
  {
    RLOG(1, "Couldn't find body \"%s\" in graph - skipping removal",
         name ? name : "NULL");
    return false;
  }

  if (!RcsBody_isLeaf(bdy))
  {
    RLOG(1, "Can't delete non-leaf body \"%s\" - skipping removal", bdy->bdyName);
    return false;
  }

  VortexBody* part = getPartPtr(bdy);
  if (part == NULL)
  {
    RLOG(1, "Can't find VxPart for body \"%s\" - skipping removal", bdy->bdyName);
    return false;
  }

  // Remove the constraints that link the part to its predecessor. We do this
  // in two steps since it's not clear if the universe's removeConstraint()
  // function is modifying the constraint count in the VxPart.
  size_t nc = part->getConstraintCount();
  std::vector<Vx::VxConstraint*> bdyConstraints;

  for (size_t i = 0; i<nc; ++i)
  {
    bdyConstraints.push_back(part->getConstraint(i));
  }

  for (size_t i = 0; i<bdyConstraints.size(); ++i)
  {
    universe->removeConstraint(bdyConstraints[i]);
  }

  MatNd* arrBuf[3];
  arrBuf[0] = this->q_des;
  arrBuf[1] = this->q_dot_des;
  arrBuf[2] = this->T_des;
  RcsGraph_removeBody(getGraph(), name, arrBuf, 3);

  Vx::VxPart* nowMyPart = universe->removePart(part);
  delete nowMyPart;

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::VortexSimulation::addBody(const RcsBody* body)
{
  RLOG(5, "Creating Vortex part for \"%s\"", body->bdyName);

  pthread_mutex_lock(&this->extForceLock);

  RcsBody* cpyOfBody = RcsBody_clone(body);
  RCHECK(cpyOfBody);

#ifdef OLD_TOPO
  cpyOfBody->parent = RcsGraph_getBodyByName(getGraph(), body->bdyName);
#else
  RFATAL("Fix me");
#endif

  bool success = createCompositeBody(cpyOfBody);

  createJoint(cpyOfBody);

  MatNd* arrBuf[3];
  arrBuf[0] = this->q_des;
  arrBuf[1] = this->q_dot_des;
  arrBuf[2] = this->T_des;

  unsigned int nJoints = RcsBody_numJoints(cpyOfBody);

  if (nJoints > 0)
  {
    this->T_des = MatNd_realloc(this->T_des, getGraph()->dof, 1);
    this->q_des = MatNd_realloc(this->q_des, getGraph()->dof, 1);
    this->q_dot_des = MatNd_realloc(this->q_dot_des, getGraph()->dof, 1);
  }

#ifdef OLD_TOPO
  RcsGraph_addBody(getGraph(), cpyOfBody->parent, cpyOfBody, arrBuf, 3);
#else
  RFATAL("Fix me");
#endif


  pthread_mutex_unlock(&this->extForceLock);

  RLOG(5, "%s adding \"%s\" to Vortex universe",
       success ? "SUCCESS" : "FAILURE", body->bdyName);

  return success;
}

/*******************************************************************************
*
******************************************************************************/
bool Rcs::VortexSimulation::deactivateBody(const char* name)
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
    RLOG(1, "Can't deactivate non-leaf body \"%s\" - skipping", bdy->bdyName);
    return false;
  }

  VortexBody* part = getPartPtr(bdy);
  if (part == NULL)
  {
    RLOG(1, "Can't find VxPart for body \"%s\" - skipping deactivation",
         bdy->bdyName);
    return false;
  }

  Vx::VxPart* nowMyPart = universe->removePart(part);

  if (nowMyPart == NULL)
  {
    RLOG(1, "Can't find VxPart \"%s\" in universe", bdy->bdyName);
    return false;

  }

  this->deactivatedBodies[name] = nowMyPart;

  return true;
}

/*******************************************************************************
*
******************************************************************************/
bool Rcs::VortexSimulation::activateBody(const char* name, const HTr* A_BI)
{
  if (name == NULL)
  {
    RLOG(1, "Can't retrieve body with name pointing to NULL");
    return false;
  }

  auto it = deactivatedBodies.find(std::string(name));

  if (it == deactivatedBodies.end())
  {
    RLOG(1, "Body \"%s\" is not deactivated - skipping activation", name);
    return false;
  }



  Vx::VxPart* part = it->second;
  part->resetDynamics();
  part->setLinearVelocity(Vx::VxVector3(0.0, 0.0, 0.0));
  part->setAngularVelocity(Vx::VxVector3(0.0, 0.0, 0.0));

  if (A_BI != NULL)
  {
    part->setTransform(VxTransform_fromHTr(A_BI));
  }

  universe->addPart(part);

  deactivatedBodies.erase(it);

  return true;
}
