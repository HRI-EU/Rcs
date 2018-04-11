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
#include <Vx/VxUniverse.h>

#include <fstream>


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
 * Constructor from Graph
 ******************************************************************************/
Rcs::VortexSimulation::VortexSimulation(const RcsGraph* g,
                                        const char* physicsConfigFile,
                                        bool withGroundPlane) :
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
  groundPlane(NULL)
{
  initPhysics(physicsConfigFile, withGroundPlane);
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
  groundPlane(NULL)
{
  initPhysics();
}

/*******************************************************************************
 * Physics initialization
 ******************************************************************************/
void Rcs::VortexSimulation::initPhysics(const char* physicsConfigFile,
                                        bool withGroundPlane)
{
  Vec3d_setZero(this->F_ext);
  Vec3d_setZero(this->r_ext);

  pthread_mutex_init(&this->extForceLock, NULL);

  // create the frame
  RLOG(5, "VxFrame::instance()");
  Vx::VxFrame* frame = Vx::VxFrame::instance();
  frame->setMaxThreadCount(4);

  // create the universe
  Vx::VxUniverse* universe = new Vx::VxUniverse(100, 10000);
  RLOG(5, "Creating universe");
  universe->setCollisionMultithreaded(true);

  // Creating stepper for particle systems
  // this->vxPsStepper = new VxPs::VxPsStepper;

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

  universe->setGravity(0.0, 0.0, -RCS_GRAVITY);
  Vx::VxReal lin_speed, lin_accel, ang_speed, ang_accel;
  int step;
  Vx::VxPart::getDefaultAutoSleepParameters(lin_speed, lin_accel, ang_speed,
                                            ang_accel, step);
  lin_speed *= 0.001;
  lin_accel *= 0.001;
  Vx::VxPart::setDefaultAutoSleepParameters(lin_speed, lin_accel, ang_speed,
                                            ang_accel, step);
  universe->setAutoSleep(false);

  // Now that the frame and universe are created, check for parameters in xml
  // file. Set default configuration parameters.
  this->integratorDt = RCSVORTEX_DEFAULT_INTEGRATOR_DT;
  this->jointLockStiffness = RCSVORTEX_DEFAULT_JOINT_LOCK_STIFFNESS;
  this->jointLockDamping = RCSVORTEX_DEFAULT_JOINT_LOCK_DAMPING;
  this->jointMotorLoss = RCSVORTEX_DEFAULT_JOINT_MOTOR_LOSS;

  // Determine absolute file name of config file and copy the XML file name
  char absFileName[256] = "";
  bool file_exists = Rcs_getAbsoluteFileName(physicsConfigFile, absFileName);

  if (file_exists == false)
  {
    RLOG(1, "Rcs physics configuration file \"%s\" not found",
         physicsConfigFile);
  }
  else
  {
    initSettings(absFileName);
    RLOG(5, "Physics initialized");

    initMaterial(absFileName);
    RLOG(5, "Materials initialized");
  }

  // Ground plane
  if (withGroundPlane)
  {
    createGroundPlane();
    RLOG(5, "Created ground plane");
  }

  // Create all Vortex VxParts from the RcsGraph's bodies
  RLOG(5, "Creating bodies");
  RCSGRAPH_TRAVERSE_BODIES(this->graph)
  {
    createCompositeBody(BODY);
  }

  // Create all joints between the RcsGraph's bodies
  RLOG(5, "Creating joints");
  RCSGRAPH_TRAVERSE_BODIES(this->graph)
  {
    createJoint(BODY);
  }

  // Disable all mesh-mesh collisions
  RLOG(5, "Disabling mesh collisions");
  disableMeshCollisions();

  // by MM disable collisions within all groups
  disableCollisionsWithinGroup(NULL);

  // Copy transforms
  RLOG(5, "Updating transforms");
  updateTransformations();

  // checking for possible problems of universe
  getUniverse()->verifyUniverseContent();

  RLOG(5, "finished Vortex constructor");
}

/*******************************************************************************
 * Stops application and deletes all memory
 ******************************************************************************/
Rcs::VortexSimulation::~VortexSimulation()
{
  const Vx::VxConstraintSet& constraints = getUniverse()->getConstraints();

  for (auto it2 = constraints.begin(); it2 != constraints.end(); ++it2)
  {
    Vx::VxConstraint* c = *it2;

    VortexJointInfo* jnt_info = ((VortexJointInfo*) c->getUserDataPtr());
    // in case of fixed joints, no VortexJointInfo is constructed, but the
    // user date is set to NULL
    if (jnt_info)
    {
      delete jnt_info;
    }
  }


  Vx::VxFrame* frame = Vx::VxFrame::instance();
  frame->release();

  pthread_mutex_destroy(&this->extForceLock);

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
  RCHECK(mutex);
  trafoUpdateLock = mutex;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::VortexSimulation::setGravity(const double gravity[3])
{
  getUniverse()->setGravity(gravity[0], gravity[1], gravity[2]);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::VortexSimulation::updateSensors()
{
  RCSGRAPH_TRAVERSE_SENSORS(this->graph)
  {
    switch (SENSOR->type)
    {
      case RCSSENSOR_LOAD_CELL:
        updateFTS(SENSOR);
        break;
      case RCSSENSOR_JOINT_TORQUE:
        updateJointTorqueSensor(SENSOR);
        break;
      case RCSSENSOR_CONTACT_FORCE:
        updateContactForceSensor(SENSOR);
        break;
      case RCSSENSOR_PPS:
        updatePPSSensor(SENSOR);
        break;
      default:
        RLOG(4, "No update function for sensor type %d", SENSOR->type);
    }
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
Vx::VxUniverse* Rcs::VortexSimulation::getUniverse()
{
  return &Vx::VxFrame::instance()->getUniverse();
}

/*******************************************************************************
 *
 ******************************************************************************/
const Vx::VxUniverse* Rcs::VortexSimulation::getUniverse() const
{
  const Vx::VxUniverse* universe = &Vx::VxFrame::instance()->getUniverse();
  return universe;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::VortexSimulation::initSettings(const char* configFile)
{
  xmlDocPtr xmlDoc;
  xmlNodePtr xmlRootNode = parseXMLFile(configFile, "content", &xmlDoc);

  if (xmlRootNode==NULL)
  {
    RLOG(1, "Node \"content\" not found in file \"%s\"", configFile);
    return false;
  }

  int grpIdx = 0, halflife = 5;
  Vx::VxSolverParameters* sParam = getUniverse()->getSolverParameters(grpIdx);

  // Read XML file
  xmlNodePtr node = xmlRootNode->children;
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
        getUniverse()->setCriticalConstraintParameters(grpIdx, halflife);
      }

      if (getXMLNodePropertyDouble(node, "constraint_angular_compliance",
                                   &value))
      {
        sParam->setConstraintAngularCompliance(value);
        getUniverse()->setCriticalConstraintParameters(grpIdx, halflife);
      }
    }

    node = node->next;
  }
  // Free the xml memory
  xmlFreeDoc(xmlDoc);

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
void Rcs::VortexSimulation::initMaterial(const char* material_file)
{
  Vx::VxMaterialTable* materialTbl = getMaterialTable();

  // Read XML file
  char msg[256];
  xmlDocPtr xmlDoc;
  xmlNodePtr xmlRootNode = parseXMLFile(material_file, "content", &xmlDoc);
  RCHECK_MSG(xmlRootNode, "Node \"content\" not found in file \"%s\"", material_file);

  xmlNodePtr node = xmlRootNode->children;
  while (node)
  {
    if (isXMLNodeName(node, "material"))
    {
      if (getXMLNodePropertyStringN(node, "name", msg, 256))
      {
        Vx::VxMaterial* material = materialTbl->registerMaterial(msg);

        double value;
        char option[64];

        if (getXMLNodePropertyStringN(node, "friction_model", option, 64))
        {
          if (STRCASEEQ(option, "Box"))
          {
            material->setFrictionModel(Vx::VxMaterialBase::kFrictionAxisLinear,
                                       Vx::VxMaterialBase::kFrictionModelBox);
          }
          else if (STRCASEEQ(option, "ScaledBox"))
          {
            material->setFrictionModel(Vx::VxMaterialBase::kFrictionAxisLinear,
                                       Vx::VxMaterialBase::kFrictionModelScaledBox);
          }
          else if (STRCASEEQ(option, "ScaledBoxFast"))
          {
            material->setFrictionModel(Vx::VxMaterialBase::kFrictionAxisLinear,
                                       Vx::VxMaterialBase::kFrictionModelScaledBoxFast);
          }
          else
          {
            RLOG(0, "unknown friction model \"%s\" in material definition "
                 "for %s", option, msg);
          }
        }

        if (getXMLNodePropertyDouble(node, "friction_coefficient", &value))
        {
          material->setFrictionCoefficient(Vx::VxMaterialBase::kFrictionAxisLinear, value);
        }
        if (getXMLNodePropertyDouble(node, "static_friction_scale", &value))
        {
          material->setStaticFrictionScale(Vx::VxMaterialBase::kFrictionAxisLinear, value);
        }
        if (getXMLNodePropertyDouble(node, "slip", &value))
        {
          material->setSlip(Vx::VxMaterialBase::kFrictionAxisLinear, value);
        }
        bool isd = false;
        if (getXMLNodePropertyBoolString(node, "integrated_slip_displacement", &isd))
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
        if (getXMLNodePropertyDouble(node, "slide", &value))
        {
          material->setSlide(Vx::VxMaterialBase::kFrictionAxisLinear, value);
        }
        if (getXMLNodePropertyDouble(node, "compliance", &value))
        {
          material->setCompliance(value);
          // as a default, set critical damping
          //          material->setDamping(getUniverse()->getCriticalDamping(value));
          // from the Vortex documentation:
          // Nearly optimal value of damping is given by:
          // damping = 5*time_step/compliance
          material->setDamping((5.0 * this->integratorDt) / value);
        }
        if (getXMLNodePropertyDouble(node, "damping", &value))
        {
          material->setDamping(value);
        }
        if (getXMLNodePropertyDouble(node, "restitution", &value))
        {
          material->setRestitution(value);
        }
        if (getXMLNodePropertyDouble(node, "restitution_threshold", &value))
        {
          material->setRestitutionThreshold(value);
        }
        if (getXMLNodePropertyDouble(node, "adhesive_force", &value))
        {
          material->setAdhesiveForce(value);
        }
      }
      else
      {
        RLOG(0, "file \"%s\" has a material entry without a name property, so it is ignored", material_file);
      }
    }


    node = node->next;
  }

  // Free the xml memory
  xmlFreeDoc(xmlDoc);

  REXEC(5)
  {
    getMaterialTable()->dump(0);
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
    RLOG(5, "No VxPart found in body \"%s\"", b0->name);
    return;
  }

  if (part1==NULL)
  {
    RLOG(5, "No VxPart found in body \"%s\"", b1->name);
    return;
  }

  getUniverse()->disablePairIntersect(part0, part1);
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::VortexSimulation::Contacts Rcs::VortexSimulation::getContacts()
{
  Contacts contacts;

  // get iterators for first and last contact
  Vx::VxUniverse::DynamicsContactIterator it, ie;
  it = getUniverse()->dynamicsContactBegin();
  ie = getUniverse()->dynamicsContactEnd();

  for (; it != ie; ++it)
  {
    Contact contact1;
    (*it)->getPosition(contact1.pos.data());
    (*it)->getForce(0, contact1.force.data());

    // \todo: strange, we have to draw both contacts to see them all. But there
    // are no complementing forces
    Contact contact2;
    (*it)->getPosition(contact2.pos.data());
    (*it)->getForce(1, contact2.force.data());

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
  const Vx::VxConstraintSet& constraints = getUniverse()->getConstraints();

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
 * Creates a ground plane at height 0, up vector is (0 0 1).
 ******************************************************************************/
void Rcs::VortexSimulation::createGroundPlane()
{
  if (this->groundPlane)
  {
    RLOG(3, "Ground plane already exists - skipping");
    return;
  }

  this->groundPlane = new Vx::VxPart();
  this->groundPlane->setName("GroundPlane");

  Vx::VxMaterial* mtl = getMaterialTable()->getDefaultMaterial();

  Vx::VxCollisionGeometry* cg = new Vx::VxCollisionGeometry(new Vx::VxPlane(),
                                                            mtl);
  this->groundPlane->addCollisionGeometry(cg);
  this->groundPlane->setPosition(0.0, 0.0, 0.0);
  getUniverse()->addPart(this->groundPlane);
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::VortexSimulation::createCompositeBody(RcsBody* body)
{
  if (body == NULL)
  {
    RLOG(1, "Body is NULL - not creating physics part");
    return false;
  }

  if (body->physicsSim == false)
  {
    NLOG(1, "Body \"%s\": physicsSim is false - skipping VxPart creation",
         body->name);
    return false;
  }

  if (body->shape == NULL)
  {
    RLOG(1, "Body \"%s\" has no shape attached - skipping", body->name);
    return false;
  }


  Vx::VxPart* parentPrt = body->parent ? getPartPtr(body->parent) : NULL;

  if ((body->physicsSim == RCSBODY_PHYSICS_DYNAMIC) &&
      (body->rigid_body_joints==false) &&
      (parentPrt==NULL))
  {
    RLOG(1, "Body \"%s\" needs rigid body joints! It is simulated dynamically "
         "and has no parent part", body->name);
    return false;
  }

  RLOG(5, "Creating body \"%s\"", body->name);

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
    Vx::VxMaterialTable* materialTbl = getMaterialTable();
    char* material_name = (*sPtr)->material;
    Vx::VxMaterial* material = materialTbl->registerMaterial(material_name);

    if (material == NULL)
    {
      RLOG(1, "%s shape of body \"%s\" has unknown material \"%s\", setting "
           "it to default material",
           RcsShape_name((*sPtr)->type), body->name, material_name);
    }

    // create shape objects with the given material id
    switch ((*sPtr)->type)
    {
      case RCSSHAPE_SSL:
        RLOG(5, "Creating SSL for object \"%s\"", body->name);
        composite->addCollisionGeometry(createCapsule(*sPtr, body->A_BI, material));
        break;

      case RCSSHAPE_MESH:
        RLOG(5, "Creating mesh for object \"%s\"", body->name);
        composite->addCollisionGeometry(createMesh(*sPtr, body->A_BI, material));
        break;

      case RCSSHAPE_BOX:
        RLOG(5, "Creating box for object \"%s\"", body->name);
        composite->addCollisionGeometry(createBox(*sPtr, body->A_BI, material));
        break;

      case RCSSHAPE_SPHERE:
        RLOG(5, "Creating sphere for object \"%s\"", body->name);
        composite->addCollisionGeometry(createSphere(*sPtr, body->A_BI, material));
        break;

      case RCSSHAPE_CYLINDER:
        RLOG(5, "Creating cylinder for object \"%s\"", body->name);
        composite->addCollisionGeometry(createCylinder(*sPtr, body->A_BI, material));
        break;

      case RCSSHAPE_SSR:
        RLOG(5, "Creating SSR for object \"%s\"", body->name);
        composite->addCollisionGeometry(createSSR(*sPtr, body->A_BI, material));
        break;

      case RCSSHAPE_CONE:
        RLOG(5, "Creating cone for object \"%s\"", body->name);
        composite->addCollisionGeometry(createCone(*sPtr, body->A_BI, material));
        break;

      case RCSSHAPE_TORUS:
        RLOG(5, "Creating torus for object \"%s\"", body->name);
        composite->addCollisionGeometry(createTorus(*sPtr, body->A_BI, material));
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
         body->name);
    delete composite;
    return false;
  }

  Vx::VxPart* p = new VortexBody(body);

  p->addCollisionGeometry(composite);

  // Assign dynamic properties
  if (body->m > 0.0)
  {
#if 1
    p->setMassAndInertia(body->m, body->Inertia->rot);
#else // old&wrong code works better for force control
    // The inertia tensor used by Vortex must factor in the mass (see
    // documentation)
    Vx::VxReal33 I;
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
      {
        I[i][j] = body->Inertia->rot[i][j] * body->m;
      }

    p->setMassAndInertia(body->m, I);
#endif

    // Vx::VxMassProperties::setMass
    // Vx::VxMassProperties::setInertiaTensorLocal
    // Vx::VxMassProperties::setCOMPositionLocal

    // if the inertia tensor is zero, Vortex calculates it based on the collision model
    // and sets the mass to 1 kg
    // if this fails for some reason (like for tri-meshes) vortexInertiaTensor is identity
    // (stl and obj seem to work fine)
    if (Mat3d_isZero(body->Inertia->rot))
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
  p->setCOMOffset(body->Inertia->org);

  p->setName(body->name);
  p->setTransform(VxTransform_fromHTr(body->A_BI));

  if (this->bodyLinearDamping >= 0.0)
  {
    p->setLinearVelocityDamping(this->bodyLinearDamping);
  }

  if (this->bodyAngularDamping >= 0.0)
  {
    p->setAngularVelocityDamping(this->bodyAngularDamping);
  }

  // Add extra info to RcsBody
  body->actor = p;
  p->setUserDataPtr(body);

  // Add to universe
  getUniverse()->addPart(p);

  RLOG(5, "Created body \"%s\"", body->name);

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

  const Vx::VxPartSet& parts = getUniverse()->getParts();

  for (auto it = parts.begin(); it != parts.end(); ++it)
  {
    Vx::VxPart* part = *it;

    VortexBody* vp = dynamic_cast<VortexBody*>(*it);
    if (vp != NULL)
    {
      HTr_fromVxTransform(&vp->A_PI, part->getTransform());
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
    return body->A_BI;
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
  RCHECK(dt>=0.0);

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


  this->simTime += dt;


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
  REXEC(4)
  {
    getUniverse()->verifyDynamics("Rcs::VortexSimulation", 3);
  }
}

/*******************************************************************************
 * Reset physics.
 ******************************************************************************/
void Rcs::VortexSimulation::reset()
{
  MatNd_copy(this->q_des, graph->q);
  MatNd_setZero(this->q_dot_des);
  MatNd_setZero(this->T_des);

  RCSGRAPH_TRAVERSE_BODIES(this->graph)
  {
    Vx::VxPart* vxBdy = getPartPtr(BODY);

    if (vxBdy != NULL)
    {
      if (BODY->physicsSim == RCSBODY_PHYSICS_KINEMATIC)
      {
        vxBdy->setTransformKinematic(VxTransform_fromHTr(BODY->A_BI),
                                     getIntegratorDt());
      }
      else
      {
        vxBdy->setTransform(VxTransform_fromHTr(BODY->A_BI));
      }

      vxBdy->setAngularVelocity(Vx::VxVector3(0.0, 0.0, 0.0));
      vxBdy->setLinearVelocity(Vx::VxVector3(0.0, 0.0, 0.0));
      // vxBdy->resetDynamics();
    }

  }

  // RCSGRAPH_TRAVERSE_JOINTS(this->graph)
  //   {
  //     Vx::VxConstraint* c = (Vx::VxConstraint*) JNT->extraInfo;
  //     if (c != NULL)
  //       {
  //         c->resetDynamics();
  //       }
  //   }

  getUniverse()->resetDynamics();
  getUniverse()->resetContacts();

  step(1.0e-8);
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
      RLOG(5, "%s Damping reset to linear=%g angular=%g", dragBody->name,
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
    RLOG(5, "Setting body %s damping to linear=%g angular=%g",  body->name,
         10.0*this->bodyLinearDamping, 100.0*this->bodyAngularDamping);
    vxBdy->setLinearVelocityDamping(10.0*this->bodyLinearDamping);
    vxBdy->setAngularVelocityDamping(100.0*this->bodyAngularDamping);
  }

  Vec3d_constMul(this->F_ext, F, 10.0);
  Vec3d_copy(this->r_ext, r);
  this->b_ext = vxBdy;
  pthread_mutex_unlock(&this->extForceLock);

  NLOG(0, "Applying F = %5.3f %5.3f %5.3f at %5.3f %5.3f %5.3f "
       "to body \"%s\"", this->F_ext[0], this->F_ext[1], this->F_ext[2],
       this->r_ext[0], this->r_ext[1], this->r_ext[2], body->name);
}

/*******************************************************************************
 * Sets the kinematic transform to the body.
 ******************************************************************************/
void Rcs::VortexSimulation::applyTransform(const RcsBody* body, const HTr* A_BI)
{
  Vx::VxPart* vxBdy = getPartPtr(body);

  if (vxBdy == NULL)
  {
    RLOG(1, "No VxPart found in body \"%s\"", body ? body->name : "NULL");
    return;
  }

  pthread_mutex_lock(&this->extForceLock);
  vxBdy->setTransform(VxTransform_fromHTr(A_BI));
  pthread_mutex_unlock(&this->extForceLock);
}

/*******************************************************************************
 * Set the linear velocity of a body
 ******************************************************************************/
void Rcs::VortexSimulation::applyLinearVelocity(const RcsBody* body, const double v[3])
{
  Vx::VxPart* vxBdy = getPartPtr(body);

  if (vxBdy == NULL)
  {
    RLOG(1, "Body \"%s\": Couldn't set velocity", body ? body->name : "NULL");
    return;
  }

  NLOG(0, "Setting linear velocity of body \"%s\" to (%.3f, %.3f, %.3f)",
       body->name, v[0], v[1], v[2]);

  pthread_mutex_lock(&this->extForceLock);
  vxBdy->setLinearVelocity(Vx::VxVector3(v));
  pthread_mutex_unlock(&this->extForceLock);
}

/*******************************************************************************
 * Get the linear velocity of a body
 ******************************************************************************/
void Rcs::VortexSimulation::getLinearVelocity(const RcsBody* body, double v[3]) const
{
  Vx::VxPart* vxBdy = getPartPtr(body);

  if (vxBdy == NULL)
  {
    RLOG(1, "Body \"%s\": Couldn't get velocity", body ? body->name : "NULL");
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
void Rcs::VortexSimulation::applyAngularVelocity(const RcsBody* body, const double v[3])
{
  Vx::VxPart* vxBdy = getPartPtr(body);

  if (vxBdy == NULL)
  {
    return;
  }

  NLOG(0, "Setting angular velocity of body \"%s\" to (%.3f, %.3f, %.3f)",
       body->name, v[0], v[1], v[2]);

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
    RLOG(1, "Body \"%s\": Couldn't get velocity", body ? body->name : "NULL");
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
bool Rcs::VortexSimulation::createJoint(RcsBody* body)
{
  if (body == NULL)
  {
    RLOG(1, "Body is NULL - skipping joint creation");
    return false;
  }

  Vx::VxPart* part0 = body->parent ? getPartPtr(body->parent) : NULL;
  Vx::VxPart* part1 = getPartPtr(body);

  switch (body->physicsSim)
  {

    case RCSBODY_PHYSICS_KINEMATIC:
    {
      RCHECK_MSG(part1, "No VxPart for body \"%s\"", body->name);
      part1->setControl(Vx::VxPart::kControlAnimated);
    }
    break;

    case RCSBODY_PHYSICS_DYNAMIC:
    {
      if (part0 == NULL)
      {
        if (body->rigid_body_joints == false)
        {
          RLOG(1, "%s has no previous VxPart, therefore needs rigid body joints. Did you forget to add the rigid_body_joints flag?", body->name);
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
          RFATAL("%s has %d joints - only one is currently supported", body->name, RcsBody_numJoints(body));
        }
      }

      Vx::VxConstraint* vJnt = NULL;
      switch (body->jnt->type)
      {
        case RCSJOINT_ROT_X:
        case RCSJOINT_ROT_Y:
        case RCSJOINT_ROT_Z:
        {
          double qInit = MatNd_get(graph->q, body->jnt->jointIndex, 0);
          vJnt = createRevoluteJoint(part0, part1, this->jointLockStiffness,
                                     this->jointLockDamping,
                                     //getUniverse()->getCriticalDamping(jointLockStiffness),
                                     this->jointMotorLoss,
                                     qInit);
          if (vJnt != NULL)
          {
            getUniverse()->disablePairIntersect(part0, part1);
            getUniverse()->addConstraint(vJnt);
          }

          NLOG(0, "Created revolute joint for body \"%s\"", body->name);
          break;
        }
        case RCSJOINT_TRANS_X:
        case RCSJOINT_TRANS_Y:
        case RCSJOINT_TRANS_Z:
        {
          // double qInit = *body->jnt->q;
          // RCHECK(qInit==MatNd_get(graph->q, body->jnt->jointIndex, 0));
          double qInit = MatNd_get(graph->q, body->jnt->jointIndex, 0);
          vJnt = createPrismaticJoint(part0, part1, this->jointLockStiffness, this->jointLockDamping, qInit);
          if (vJnt != NULL)
          {
            getUniverse()->disablePairIntersect(part0, part1);
            getUniverse()->addConstraint(vJnt);
          }
          NLOG(0, "Created prismatic joint for body \"%s\"", body->name);
          break;
        }
        default:
          RLOG(1, "Only revolute joints are supported for connecting "
               "dynamically simulated physical joints!");
      }
      if (vJnt != NULL)
      {
        body->jnt->extraInfo = (void*) vJnt;
        VortexJointInfo* jData = new VortexJointInfo;
        jData->jnt = body->jnt;
        vJnt->setUserDataPtr(jData);
      }
    }
    break;

    case RCSBODY_PHYSICS_FIXED:
    {
      if (body->parent == NULL)
      {
        RLOG(1, "Can't create fixed joint for body \"%s\": Parent is NULL",
             body->name);
        break;
      }

      if (RcsBody_numJoints(body) != 0)
      {
        RLOG(1, "Can't create fixed joint for body \"%s\": Has %d joints "
             "(should be 0)", body->name, RcsBody_numJoints(body));
        break;
      }

      Vx::VxConstraint* vJnt = createFixedJoint(part0, part1, this->graph);

      if (vJnt != NULL)
      {
        //        vJnt->setUserData(body);
        vJnt->setUserDataPtr(NULL); // TODO: Is this correct? MG 2012-11-12

        getUniverse()->disablePairIntersect(part0, part1);
        getUniverse()->addConstraint(vJnt);
        RLOG(5, "Created fixed joint (VxRPRO) between %s - %s",
             body->parent->name, body->name);
        body->extraInfoVortex = (void*) vJnt;
      }
    }
    break;

    default:
      return false;
  }

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::VortexSimulation::applyControl(double dt)
{
  RcsGraph* graphCpy = RcsGraph_clone(this->graph);
  RcsGraph_setState(graphCpy, this->q_des, NULL);

  RCSGRAPH_TRAVERSE_BODIES(this->graph)
  {
    switch (BODY->physicsSim)
    {
      case RCSBODY_PHYSICS_KINEMATIC:
      {
        RcsBody* BODY2 = RcsGraph_getBodyByName(graphCpy, BODY->name);

        if (BODY2 == NULL)
        {
          RLOG(3, "Body \"%s\" does not exist", BODY->name);
          break;
        }

        Vx::VxPart* p = getPartPtr(BODY);

        if (p == NULL)
        {
          RLOG(3, "Body \"%s\" has no Vx::Part attached", BODY->name);
          break;
        }

        RLOG(5, "Setting transform of body \"%s\" to %f %f %f", BODY2->name,
             BODY2->A_BI->org[0], BODY2->A_BI->org[1], BODY2->A_BI->org[2]);
        p->setTransformKinematic(VxTransform_fromHTr(BODY2->A_BI), dt);
      }
      break;

      case RCSBODY_PHYSICS_DYNAMIC:
      {
        // Vx::VxPart* p = getPartPtr(BODY);
        // if (p != NULL && BODY->rigid_body_joints)
        // {
        //   Vx::VxVector3 negGrav(0.0, 0.0, -9.81*BODY->m);
        //   p->addForce(negGrav);
        // }


        // No control input if number of joints != 1
        if (RcsBody_numJoints(BODY) != 1)
        {
          if ((RcsBody_numJoints(BODY) != 0) && (!BODY->rigid_body_joints))
          {
            RLOG(4, "Body \"%s\" has %d joints - only one supported yet",
                 BODY->name, RcsBody_numJoints(BODY));
          }
          break;
        }

        // No control input if maxTorque is zero
        RcsJoint* jnt = BODY->jnt;

        // TODO: Set hinge to free?
        if (jnt->maxTorque == 0.0)
        {
          break;
        }

        // Pedantic: Only unconstrained joints should be driven by physics
        // RCHECK_MSG(jnt->jacobiIndex != -1, "body name: %s, jnt name: %s",
        //            BODY->name, jnt->name);

        // Only apply control if this is a supported joint type
        if (RcsJoint_isRotation(jnt) || RcsJoint_isTranslation(jnt))
        {
          Vx::VxConstraint::CoordinateID coordinate;
          if (RcsJoint_isRotation(jnt))
          {
            coordinate = Vx::VxHinge::kAngularCoordinate;
          }
          else
          {
            coordinate = Vx::VxPrismatic::kLinearCoordinate;
          }

          Vx::VxConstraint* c = (Vx::VxConstraint*) jnt->extraInfo;
          if (c == NULL)
          {
            RLOG(1, "constraint is NULL - body name: %s, jnt name: %s",
                 BODY->name, jnt->name);
            break;
          }

          // this is used in case of lock control (position control)
          if (jnt->ctrlType == RCSJOINT_CTRL_POSITION)
          {
            int index = jnt->jointIndex;

            Vx::VxReal q_cmd;

            RcsJoint* j_master = jnt->coupledTo;

            if (j_master != NULL)
            {
              // If the joint is coupled, set the position command according
              // to its coupling factor.
              if (jnt->couplingFactors->size == 0)
              {
                q_cmd = MatNd_get(this->q_des, index, 0);
              }
              else if (jnt->couplingFactors->size == 1)
              {
                Vx::VxReal q_cmd_master = MatNd_get(this->q_des,
                                                    j_master->jointIndex, 0);
                q_cmd = jnt->q0 + jnt->couplingFactors->ele[0] *
                        (q_cmd_master - j_master->q0);
              }
              else
              {
                RFATAL("Incorrect number of coupling factors of joint %s: %d",
                       jnt->name, jnt->couplingFactors->size);
              }
            }
            else
            {
              q_cmd = MatNd_get(this->q_des, index, 0);
            }

            // TODO: Should be better a lock velocity
#if 0
            c->setLockMaximumForce(coordinate, jnt->maxTorque);
            c->setLockPosition(coordinate, q_cmd);
            RLOG(0, "[%s]: q_cmd = %f deg", jnt->name, q_cmd*180.0/M_PI);
#else
            double lockCurr = c->getLockPosition(coordinate);
            double deltaPos = q_cmd-lockCurr;

            if (coordinate==Vx::VxHinge::kAngularCoordinate)
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

            //double lockVel = Math_clip(deltaPos/dt, -90.0*M_PI/180.0, 90.0*M_PI/180.0);

            //RLOG(0, "%s: dt=%g   deltaPos=%g   q_cmd=%g   q_curr=%g", jnt->name, dt, deltaPos, q_cmd, lockCurr);
            c->setLockMaximumForce(coordinate, jnt->maxTorque);
            //c->setLimitVelocity(coordinate,  Vx::VxConstraint::kLimitLower, -90.0*M_PI/180.0);
            //c->setLimitVelocity(coordinate,  Vx::VxConstraint::kLimitUpper, +90.0*M_PI/180.0);
            c->setLockDamping(coordinate, 1.0);
            c->setLockVelocity(coordinate, lockVel);
#endif
          }
          // this is used in case of motor control (velocity control)
          else if (jnt->ctrlType == RCSJOINT_CTRL_VELOCITY)
          {
            Vx::VxReal q_dot_cmd = MatNd_get(this->q_dot_des, jnt->jointIndex, 0);
            c->setMotorDesiredVelocity(coordinate, q_dot_cmd);
            c->setControl(coordinate, Vx::VxConstraint::kControlMotorized);
            c->setMotorMaximumForce(coordinate, jnt->maxTorque);
          }
          // Torque-controlled joints
          else if (jnt->ctrlType == RCSJOINT_CTRL_TORQUE)
          {
            double Ti = MatNd_get(this->T_des, jnt->jointIndex, 0);
            Ti = Math_clip(Ti, -jnt->maxTorque, jnt->maxTorque);

            c->setControl(coordinate, Vx::VxConstraint::kControlMotorized);
            c->setMotorLoss(coordinate, 0.0);
            // c->setLimitsActive(coordinate, false);

            // If the max. force is set to 0, the motor gets deactivated
            double pos = c->getCoordinateCurrentPosition(coordinate);

            // Here we consider joint limits for torque-controlled joints
            // \todo: What does factor 2.0 do?
            if ((this->jointLimitsActive==true) && (pos>=jnt->q_max))
            {
              c->setMotorMaximumForce(coordinate, jnt->maxTorque * 2.0);
              c->setMotorDesiredVelocity(coordinate, -0.01);
            }
            else if ((this->jointLimitsActive==true) && (pos<=jnt->q_min))
            {
              c->setMotorMaximumForce(coordinate, jnt->maxTorque * 2.0);
              c->setMotorDesiredVelocity(coordinate, +0.01);
            }
            else
            {
              // double vel = c->getCoordinateVelocity(coordinate);
              // Ti -= Math_clip(1.5*vel, -0.5, 0.5);
              c->setMotorMaximumForce(coordinate, fabs(Ti) + 1.0e-8);
              c->setMotorDesiredVelocity(coordinate, Ti >= 0.0 ? 1.0e6 : -1.0e6);
            }
          }
          else
          {
            RFATAL("Unknown cntrlType: %d", jnt->ctrlType);
          }

        }
      }
      break;

      default:
      {
        // other options are no physics or fixed, so no control has to be
        // applied
      }

    } // switch

  } // RCSGRAPH_TRAVERSE

  RcsGraph_destroy(graphCpy);
}

/*******************************************************************************
 * This function is not called by anyone, anyway
 ******************************************************************************/
void Rcs::VortexSimulation::setJointTorque(const MatNd* T_des)
{
  RcsStateType type;

  if (T_des->m == this->graph->dof)
  {
    type = RcsStateFull;
  }
  else if (T_des->m == this->graph->nJ)
  {
    type = RcsStateIK;
  }
  else
  {
    RFATAL("Dimension mismatch in T_des: has %d rows, but RcsGraph::dof=%d and"
           " RcsGraph::nJ=%d", T_des->m, this->graph->dof, this->graph->nJ);
  }


  const Vx::VxConstraintSet& constraints = getUniverse()->getConstraints();

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

    VortexJointInfo* ji = (VortexJointInfo*) c->getUserDataPtr();
    int index = (type == RcsStateIK) ? ji->jnt->jacobiIndex : ji->jnt->jointIndex;
    Vx::VxReal Ti = MatNd_get(T_des, index, 0);
    Ti = Math_clip(Ti, -ji->jnt->maxTorque, ji->jnt->maxTorque);

    RLOG(0, "Setting joint torque of \"%s\" to %f", ji->jnt->name, Ti);

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
  const Vx::VxConstraintSet& constraints = getUniverse()->getConstraints();

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

    VortexJointInfo* ji = (VortexJointInfo*) c->getUserDataPtr();
    Vx::VxReal Ti = c->getCoordinateForce(0);
    int index = (sType == RcsStateIK) ? ji->jnt->jacobiIndex : ji->jnt->jointIndex;
    MatNd_set(T, index, 0, Ti);

  }

  switch (sType)
  {
    case RcsStateFull:
      MatNd_reshapeAndSetZero(T, this->graph->dof, 1);
      break;
    case  RcsStateIK:
      MatNd_reshapeAndSetZero(T, this->graph->nJ, 1);
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
      //MatNd_reshapeCopy(q, this->graph->q);
      MatNd_reshapeCopy(q, this->q_des);
      break;
    case  RcsStateIK:
      MatNd_reshape(q, this->graph->nJ, 1);
      //RcsGraph_stateVectorToIK(this->graph, this->graph->q, q);
      RcsGraph_stateVectorToIK(this->graph, this->q_des, q);
      break;
    default:
      RFATAL("Wrong state type: %d", sType);
  }

  const Vx::VxConstraintSet& constraints = getUniverse()->getConstraints();

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

    VortexJointInfo* ji = (VortexJointInfo*) c->getUserDataPtr();
    if ((sType == RcsStateIK) && (ji->jnt->constrained == true))
    {
      continue;
    }
    int index = (sType == RcsStateIK) ? ji->jnt->jacobiIndex : ji->jnt->jointIndex;
    RCHECK_MSG(index >= 0, "joint: %s", ji->jnt->name);
    Vx::VxReal qi = c->getCoordinateCurrentPosition(0);
    MatNd_set(q, index, 0, qi);
  }


  // then also override 6 joint angles for each rigid body with joints. This is
  // only required for the full state vector, since the rigid body joints are
  // constrained degrees of freedom.
  if (sType == RcsStateFull)
  {
    RCSGRAPH_TRAVERSE_BODIES(this->graph)
    {

      if (BODY->rigid_body_joints)
      {
        Vx::VxPart* vx_body = getPartPtr(BODY);

        if (vx_body != NULL)
        {
          HTr pose;
          HTr_fromVxTransform(&pose, vx_body->getTransform());
          RcsJoint* jnt = BODY->jnt;
          RCHECK(jnt);
          RcsGraph_relativeRigidBodyDoFs(BODY, &pose, BODY->parent ? BODY->parent->A_BI : NULL, &q->ele[jnt->jointIndex]);
        }
      }
    }
  }   // sType==RcsStateFull

}

/*******************************************************************************
 * If no physical joint is attached, so we set the velocity to 0.
 ******************************************************************************/
void Rcs::VortexSimulation::getJointVelocities(MatNd* q_dot, RcsStateType sType) const
{

  switch (sType)
  {
    case RcsStateFull:
      MatNd_reshapeAndSetZero(q_dot, this->graph->dof, 1);
      break;
    case  RcsStateIK:
      MatNd_reshapeAndSetZero(q_dot, this->graph->nJ, 1);
      break;
    default:
      RFATAL("Wrong state type: %d", sType);
  }
  const Vx::VxConstraintSet& constraints = getUniverse()->getConstraints();
  Vx::VxConstraintSet::const_iterator it;

  for (it = constraints.begin(); it != constraints.end(); ++it)
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

    VortexJointInfo* ji = (VortexJointInfo*) c->getUserDataPtr();
    if ((sType == RcsStateIK) && (ji->jnt->constrained == true))
    {
      continue;
    }
    int index = (sType == RcsStateIK) ? ji->jnt->jacobiIndex : ji->jnt->jointIndex;
    RCHECK_MSG(index >= 0, "joint: %s", ji->jnt->name);
    Vx::VxReal qi = c->getCoordinateVelocity(0);
    MatNd_set(q_dot, index, 0, qi);

  }


  // Update the rigid body dofs. Since they are constrained, it is only done
  // for the full state vector.
  if (sType == RcsStateFull)
  {
    RCSGRAPH_TRAVERSE_BODIES(this->graph)
    {
      if (BODY->rigid_body_joints==true)
      {
        Vx::VxPart* vx_body = getPartPtr(BODY);

        // refFrame
        if (vx_body != NULL)
        {
          double linearVelocity[3];
          getLinearVelocity(BODY, linearVelocity);
          double angularVelocity[3];
          getAngularVelocity(BODY, angularVelocity);

          RcsJoint* jnt = BODY->jnt;
          RCHECK(jnt);
          MatNd_set(q_dot, jnt->jointIndex, 0, linearVelocity[0]);
          jnt = jnt->next;
          RCHECK(jnt);
          MatNd_set(q_dot, jnt->jointIndex, 0, linearVelocity[1]);
          jnt = jnt->next;
          RCHECK(jnt);
          MatNd_set(q_dot, jnt->jointIndex, 0, linearVelocity[2]);
          jnt = jnt->next;
          RCHECK(jnt);
          MatNd_set(q_dot, jnt->jointIndex, 0, angularVelocity[0]);
          jnt = jnt->next;
          RCHECK(jnt);
          MatNd_set(q_dot, jnt->jointIndex, 0, angularVelocity[1]);
          jnt = jnt->next;
          RCHECK(jnt);
          MatNd_set(q_dot, jnt->jointIndex, 0, angularVelocity[2]);
        }
      }
    }
  }   // sType==RcsStateFull
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::VortexSimulation::setMassAndInertiaFromPhysics(RcsGraph* graph_)
{
  bool isGraphFromPhysics = graph_ == this->graph;

  const Vx::VxPartSet& parts = getUniverse()->getParts();

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
      Vec3d_copy(body->Inertia->org, r_com);
      Mat3d_copy(body->Inertia->rot, I);
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
    Vx::VxPart* vxParent = getPartPtr(body->parent);

    // If there is no parent in the physics or if there are joints between
    // body and parent, we give up.
    if ((vxParent==NULL) || (body->parent->jnt!=NULL))
    {
      HTr_setIdentity(A_BI);
      RLOG(1, "Couldn't get physics transformation of body \"%s\"",
           body ? body->name : "NULL");
    }
    else
    {
      HTr A_PI;
      pthread_mutex_lock(&this->extForceLock);
      HTr_fromVxTransform(&A_PI, vxParent->getTransform());
      pthread_mutex_unlock(&this->extForceLock);
      HTr_transform(A_BI, &A_PI, body->A_BP ? body->A_BP : HTr_identity());
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

  Vx::VxPart* part0 = body->parent ? getPartPtr(body->parent) : NULL;
  Vx::VxPart* part1 = getPartPtr(body);

  if (body->physicsSim == RCSBODY_PHYSICS_KINEMATIC)
  {
    part1->setControl(Vx::VxPart::kControlDynamic);
    return true;
  }

  if (part0 == NULL)
  {
    RLOG(1, "%s has no prev. VxPart", body->name);
    return false;
  }

  Vx::VxConstraint* vJnt = NULL;

  if (body->jnt == NULL && body->physicsSim == RCSBODY_PHYSICS_FIXED)
  {
    vJnt = (Vx::VxConstraint*) body->extraInfoVortex;
    if (vJnt != NULL)
    {
      body->extraInfoVortex = NULL;
    }
  }

  if (body->jnt != NULL && body->physicsSim == RCSBODY_PHYSICS_DYNAMIC)
  {
    RcsJoint* jnt = body->jnt;
    vJnt = (Vx::VxConstraint*) jnt->extraInfo;
    if (vJnt != NULL)
    {
      jnt->extraInfo = NULL;
    }
  }

  if (vJnt != NULL)
  {
    getUniverse()->enablePairIntersect(part0, part1);
    getUniverse()->removeConstraint(vJnt);
    delete vJnt;
    NLOG(0, "Removed joint for body \"%s\"", body->name);
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
    RLOG(1, "Couldn't reset dynamics of part \"%s\"", body->name);
  }

  return success;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::VortexSimulation::addBodyConstraints(RcsBody* body)
{
  MatNd_realloc(this->T_des, graph->dof, 1);
  MatNd_realloc(this->q_des, graph->dof, 1);
  MatNd_copy(this->q_des, graph->q);
  MatNd_realloc(this->q_dot_des, graph->dof, 1);

  if (body == NULL)
  {
    return false;
  }
  applyTransform(body, body->A_BI);
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
  // For a load cell sensor, the body must be attached by a fixed joint
  if (fts->body->physicsSim != RCSBODY_PHYSICS_FIXED)
  {
    RLOG(4, "Load cell sensors must be attached to a body of type "
         "\"fixed\" -> not updating sensor %s", fts->name);
    return false;
  }

  // Get the Vortex part that the sensor is attached to
  VortexBody* part = getPartPtr(fts->body);
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
  RcsSensor_computeStaticForceCompensation(fts, S_f_gravity);
  Vec3d_subSelf(ftWrench, S_f_gravity);

  // Update accelerations and rotate into body frame
  Vx::VxVector3 accel = part->getLinearAcceleration();
  ftWrench[6] = accel[0];
  ftWrench[7] = accel[1];
  ftWrench[8] = accel[2];

  Vec3d_rotateSelf(&ftWrench[6], part->A_PI.rot);

  // Simple filter
  for (int i=0; i<6; ++i)
  {
    //fts->rawData->ele[i] = 0.9*fts->rawData->ele[i] + 0.1*ftWrench[i];
    fts->rawData->ele[i] = ftWrench[i];
  }

  return true;
}

/*******************************************************************************
 * Updates a force-torque sensor
 ******************************************************************************/
bool Rcs::VortexSimulation::updateJointTorqueSensor(RcsSensor* sensor)
{
  // For a joint torque sensor, the body must be attached by a revolute joint
  if (sensor->body->physicsSim != RCSBODY_PHYSICS_DYNAMIC)
  {
    RLOG(1, "Joint torque sensors must be attached to a body of type "
         "\"dynamic\" -> not creating sensor %s", sensor->name);
    return false;
  }

  // Get the Vortex part that the sensor is attached to
  Vx::VxPart* part = getPartPtr(sensor->body);
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
  // Get the Vortex part that the sensor is attached to
  VortexBody* part = getPartPtr(sensor->body);
  if (part == NULL)
  {
    RLOG(4, "Contact force sensor %s not attached to VxPart", sensor->name);
    return false;
  }

  HTr A_SI;
  const HTr* A_SB = sensor->offset;
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
  // Get the Vortex part that the sensor is attached to
  VortexBody* vxPart = getPartPtr(sensor->body);
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
    Vec3d_constMulAndAddSelf(contactForce, f, -1.0);
    it++;
  }

  return false;
  return RcsSensor_computePPS(sensor, sensor->rawData, contactForce);
}

/*******************************************************************************
 * Sets compliance properties for all hinge joints
 ******************************************************************************/
void Rcs::VortexSimulation::setJointCompliance(const MatNd* stiffness,
                                               const MatNd* damping)
{
  const Vx::VxConstraintSet& constraints = getUniverse()->getConstraints();

  for (auto it = constraints.begin(); it != constraints.end(); ++it)
  {
    if ((*it)->isOfClassType(Vx::VxHinge::getStaticClassType()))
    {
      Vx::VxHinge* hinge = dynamic_cast<Vx::VxHinge*>(*it);
      VortexJointInfo* jntInfo = ((VortexJointInfo*) hinge->getUserDataPtr());
      RCHECK(jntInfo);
      RCHECK(jntInfo->jnt);

      if (jntInfo->jnt->ctrlType == RCSJOINT_CTRL_POSITION)
      {
        // Setting the stiffness to zero results in weird movements and joint
        // constraints not satisfied any more.
        const double eps = 1.0e-8;
        int idx = jntInfo->jnt->jointIndex;
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
  MatNd_reshapeAndSetZero(stiffness, graph->dof, 1);

  if (damping!=NULL)
  {
    MatNd_reshapeAndSetZero(damping, graph->dof, 1);
  }

  const Vx::VxConstraintSet& constraints = getUniverse()->getConstraints();

  for (auto it = constraints.begin(); it != constraints.end(); ++it)
  {
    if ((*it)->isOfClassType(Vx::VxHinge::getStaticClassType()))
    {
      Vx::VxHinge* hinge = dynamic_cast<Vx::VxHinge*>(*it);
      VortexJointInfo* jntInfo = ((VortexJointInfo*) hinge->getUserDataPtr());
      RCHECK(jntInfo);
      RCHECK(jntInfo->jnt);

      if (jntInfo->jnt->ctrlType == RCSJOINT_CTRL_POSITION)
      {
        unsigned int idx = jntInfo->jnt->jointIndex;

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
