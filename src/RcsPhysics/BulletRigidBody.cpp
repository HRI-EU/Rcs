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

#include "BulletRigidBody.h"
#include "BulletHingeJoint.h"
#include "BulletSliderJoint.h"
#include "BulletHelpers.h"

#include <Rcs_typedef.h>
#include <Rcs_macros.h>
#include <Rcs_math.h>
#include <Rcs_joint.h>
#include <Rcs_body.h>
#include <Rcs_shape.h>
#include <Rcs_utils.h>
#include <Rcs_parser.h>

#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include <LinearMath/btGeometryUtil.h>

#include <iostream>
#include <fstream>

#define DEFAULT_COLLISION_MARGIN (0.001)   // 1mm
#define MAX_VERTICES_WITHOUT_COMPRESSION (100)


/*******************************************************************************
 * Sets the collision margin of a shape.
 ******************************************************************************/
static void setMargin(btCollisionShape* shape)
{
  //shape->setMargin(0*DEFAULT_COLLISION_MARGIN);
}

/*******************************************************************************
 * Create a shape
 ******************************************************************************/
//! \todo Check margins
btCollisionShape* Rcs::BulletRigidBody::createShape(RcsShape* sh,
                                                    btTransform& relTrans,
                                                    const RcsBody* body,
                                                    unsigned int convexHullVertexLimit)
{
  btCollisionShape* bShape = NULL;

  // Compute the transformation from COM (Index P) frame to body frame
  // The principal axes of inertia correspond to the Eigenvectors
  double lambda[3], A_BP[3][3];
  Mat3d_getEigenVectors(A_BP, lambda, body->Inertia->rot);
  HTr A_SP;   // Rotation from physics to shape: A_SP = A_SB*A_BP
  Mat3d_mul(A_SP.rot, (double (*)[3])sh->A_CB.rot, A_BP);

  // Vector from COM to shape, in COM-frame: p_r_ps = A_PB*(-b_r_com + b_r_s)
  Vec3d_sub(A_SP.org, sh->A_CB.org, body->Inertia->org);   // -b_r_com + b_r_s
  Vec3d_transRotateSelf(A_SP.org, A_BP);

  relTrans = btTransformFromHTr(&A_SP);

  switch (sh->type)
  {

    case RCSSHAPE_SSL:
    {
      RLOG(5, "Creating SSL for object \"%s\"", body->name);

      // Local transformation from body frame into collision geometry frame:
      // We have to consider the offset to make the ball end to the
      // reference point
      double C_r[3], B_r[3];
      Vec3d_set(C_r, 0.0, 0.0, sh->extents[2]/2.0);
      Vec3d_transRotate(B_r, A_SP.rot, C_r);
      Vec3d_addSelf(A_SP.org, B_r);
      bShape = new btCapsuleShapeZ(btScalar(sh->extents[0]),
                                   btScalar(sh->extents[2]));
      setMargin(bShape);
      relTrans = btTransformFromHTr(&A_SP);
      break;
    }

    case RCSSHAPE_CONE:
    {
      RLOG(5, "Creating cone for object \"%s\"", body->name);
      bShape = new btConeShapeZ(btScalar(sh->extents[0]),
                                btScalar(sh->extents[2]));
      bShape->setMargin(0.0);

      // Local transformation from body frame into collision geometry frame:
      // We have to consider the offset to make the base plane center to the
      // reference point
      double C_r[3], B_r[3];
      Vec3d_set(C_r, 0.0, 0.0, 0.5*sh->extents[2]);
      Vec3d_transRotate(B_r, A_SP.rot, C_r);
      Vec3d_addSelf(A_SP.org, B_r);
      relTrans = btTransformFromHTr(&A_SP);
      break;
    }

    case RCSSHAPE_SPHERE:
    {
      RLOG(5, "Creating sphere for object \"%s\"", body->name);
      bShape = new btSphereShape(btScalar(sh->extents[0]));
      setMargin(bShape);
      break;
    }

    case RCSSHAPE_CYLINDER:
    {
      RLOG(5, "Creating CYLINDER for object \"%s\"", body->name);
      btVector3 halfExt(btScalar(sh->extents[0]), btScalar(sh->extents[0]),
                        btScalar(0.5*sh->extents[2]));
      bShape = new btCylinderShapeZ(halfExt);
      setMargin(bShape);
      break;
    }

    case RCSSHAPE_BOX:
    {
      RLOG(5, "Creating BOX for object \"%s\"", body->name);
      btVector3 halfExt(0.5*sh->extents[0], 0.5*sh->extents[1],
                        btScalar(0.5*sh->extents[2]));
      bShape = new btBoxShape(halfExt);
      setMargin(bShape);
      break;
    }

    case RCSSHAPE_SSR:
    {
      RLOG(5, "Creating SSR for object \"%s\"", body->name);
      btVector3 positions[4];
      btScalar radi[4];
      btScalar hx = sh->extents[0];
      btScalar hy = sh->extents[1];
      btScalar r = 0.5*sh->extents[2];

      for (int i=0; i<4; ++i)
      {
        radi[i] = r;
      }

      positions[0] = btVector3(-hx/2.0, +hy/2.0, 0.0);
      positions[1] = btVector3(+hx/2.0, +hy/2.0, 0.0);
      positions[2] = btVector3(+hx/2.0, -hy/2.0, 0.0);
      positions[3] = btVector3(-hx/2.0, -hy/2.0, 0.0);

      bShape = new btMultiSphereShape(positions, radi, 4);
      bShape->setMargin(0.0);
      break;
    }

    case RCSSHAPE_TORUS:
    {
      RLOG(5, "Creating TORUS for object \"%s\"", body->name);
      btCompoundShape* compound = new btCompoundShape();
      bShape = compound;
      const int nSlices = 16;
      MatNd* points = MatNd_create(nSlices, 3);

      for (int i=0; i<nSlices; ++i)
      {
        double angle = 2.0*M_PI*i/nSlices;
        double* point_i = MatNd_getRowPtr(points, i);
        point_i[0] = sh->extents[0]*sin(angle);
        point_i[1] = sh->extents[0]*cos(angle);
        Vec3d_transformSelf(point_i, &A_SP);
      }

      for (int i=1; i<=nSlices; ++i)
      {
        HTr A;
        double* pPrev = MatNd_getRowPtr(points, i-1);
        double* pCurr = MatNd_getRowPtr(points, i<nSlices?i:0);
        double h = Vec3d_distance(pCurr, pPrev);
        HTr_from2Points(&A, pPrev, pCurr);

        // The btCapsuleShapeZ's origin is the center point, therefore we need
        // to shift it along the radial direction.
        double diff[3];
        Vec3d_sub(diff, pCurr, pPrev);
        Vec3d_constMulAndAddSelf(A.org, diff, 0.5);

        // h is the distance between the ball ends.
        btCapsuleShapeZ* ssl = new btCapsuleShapeZ(btScalar(0.5*sh->extents[2]),
                                                   btScalar(h));
        setMargin(ssl);

        btTransform sslTrans = btTransformFromHTr(&A);
        compound->addChildShape(sslTrans, ssl);
      }

      MatNd_destroy(points);
      relTrans.setIdentity();
      setMargin(bShape);
      break;
    }

    case RCSSHAPE_MESH:
    {
      RLOG(5, "Creating MESH for object \"%s\"", body->name);
      RcsMeshData* mesh = (RcsMeshData*) sh->userData;

      if (mesh==NULL)
      {
        RLOG(4, "[%s]: Failed to find or create mesh", body->name);
        break;
      }

      // If the mesh is large, we compress it.
      if (mesh->nVertices>convexHullVertexLimit)
      {
        RLOG(5, "[%s]: Compressing mesh with %u vertices (limit: %u)",
             body->name, mesh->nVertices, convexHullVertexLimit);
        btConvexHullShape* hull = meshToCompressedHull(mesh);
        sh->userData = hullToMesh(hull);   // and link the compressed one

        if (sh->userData != NULL)
        {
          RcsMesh_destroy(mesh);             // Delete original mesh
        }
        else
        {
          sh->userData = mesh;
        }
        bShape = hull;
      }
      else // If convexHullVertexLimit vertices or less, use it like it is
      {
        bShape = meshToHull(mesh);
      }
      bShape->setMargin(0.0);

      break;
    }

    default:
      RLOGS(1, "Shape type \"%s\" not yet supported!", RcsShape_name(sh->type));
  }


  RLOG(5, "Successfully created shape");

  return bShape;
}

/*******************************************************************************
 *
 ******************************************************************************/
//! \todo Compound shape principal axis transform
Rcs::BulletRigidBody* Rcs::BulletRigidBody::create(const RcsBody* bdy,
                                                   const PhysicsConfig* config)
{
  if (bdy == NULL)
  {
    RLOG(1, "Body is NULL - not creating physics rigid body");
    return NULL;
  }

  if (bdy->physicsSim == RCSBODY_PHYSICS_NONE)
  {
    RLOG(5, "Body \"%s\": physicsSim is not set - skipping part", bdy->name);
    return NULL;
  }

  if (RcsBody_numShapes(bdy) == 0)
  {
    RLOG(1, "Body \"%s\" has no shape attached - skipping", bdy->name);
    return NULL;
  }

  // Read number of vertices limit to be compressed into convex hull
  RCHECK(config);
  unsigned int convexHullVertexLimit = MAX_VERTICES_WITHOUT_COMPRESSION;
  xmlNodePtr bulletParams = getXMLChildByName(config->getXMLRootNode(),
                                              "bullet_parameters");
  if (bulletParams == NULL)
  {
    RLOG(1, "Physics configuration file %s did not contain a "
         "\"bullet parameters\" node!", config->getConfigFileName());
  }
  else
  {
    getXMLNodePropertyUnsignedInt(bulletParams, "convex_hull_vertex_limit",
                                  &convexHullVertexLimit);
  }





  // Traverse through shapes
  RcsShape** sPtr = &bdy->shape[0];
  btCompoundShape* cSh = new btCompoundShape();
  const char* materialName = NULL;
  while (*sPtr)
  {

    if (((*sPtr)->computeType & RCSSHAPE_COMPUTE_PHYSICS) == 0)
    {
      RLOG(0, "Skipping shape %s", RcsShape_name((*sPtr)->type));
      sPtr++;
      continue;
    }

    if (((*sPtr)->computeType & RCSSHAPE_COMPUTE_SOFTPHYSICS) != 0)
    {
      RLOG(0, "Skipping soft shape %s", RcsShape_name((*sPtr)->type));
      sPtr++;
      continue;
    }

    

    RLOG(0, "Creating shape %s", RcsShape_name((*sPtr)->type));

    btTransform relTrans;
    btCollisionShape* shape = createShape(*sPtr, relTrans, bdy,
                                          convexHullVertexLimit);

    if (shape != NULL)
    {
      shape->setUserPointer(*sPtr);
      cSh->addChildShape(relTrans, shape);

      if (materialName == NULL)
      {
        // Bullet cannot set material properties per shape, so we only use the
        // material of the first shape.
        materialName = (*sPtr)->material;
      }
    }
    else
    {
      RLOG(4, "Skipping shape for body %s", bdy->name);
    }

    sPtr++;

  } // while(*sPtr)


  if (cSh->getNumChildShapes() == 0)
  {
    RLOG(1, "Body %s: Compound shape has 0 bodies", bdy->name);
    delete cSh;
    return NULL;
  }

  RLOG(5, "All shapes created");


  // Compute the transformation from COM (Index P) frame to body frame
  // The principal axes of inertia correspond to the Eigenvectors
  double lambda[3], A_PB[3][3];
  Mat3d_getEigenVectors(A_PB, lambda, bdy->Inertia->rot);
  Mat3d_transposeSelf(A_PB);
  RCHECK(Mat3d_isValid(A_PB));

  HTr A_PI;   // Rotation from COM to world: A_PI = A_PB*A_BI
  Mat3d_mul(A_PI.rot, A_PB, bdy->A_BI->rot);

  double I_r_com[3];
  Vec3d_transRotate(I_r_com, bdy->A_BI->rot, bdy->Inertia->org);
  Vec3d_add(A_PI.org, bdy->A_BI->org, I_r_com);

  btTransform bdyTrans = btTransformFromHTr(&A_PI);
  btDefaultMotionState* ms = new btDefaultMotionState(bdyTrans);

  btVector3 bdyInertia(0,0,0);

  if (bdy->m != 0.0)
  {
    // The principal axes of inertia correspond to the Eigenvectors
    bdyInertia[0] = lambda[0];
    bdyInertia[1] = lambda[1];
    bdyInertia[2] = lambda[2];
  }

  BulletRigidBody* btBody = NULL;

  if (bdy->physicsSim == RCSBODY_PHYSICS_KINEMATIC)
  {
    // We set the collision flags to kinematic so that the body will
    btRigidBody::btRigidBodyConstructionInfo rbInfo(0.0, ms, cSh);
    btBody = new BulletRigidBody(rbInfo, bdy);
    btBody->setCollisionFlags(btBody->getCollisionFlags() |
                              btCollisionObject::CF_KINEMATIC_OBJECT);
  }
  else
  {
    btRigidBody::btRigidBodyConstructionInfo rbc(bdy->m, ms, cSh, bdyInertia);
    btBody = new BulletRigidBody(rbc, bdy);

    REXEC(5)
    {
      RLOG(0, "%s inertia before: %f %f %f", bdy->name,
           bdyInertia[0], bdyInertia[1], bdyInertia[2]);

      btVector3 bulletInertia = btBody->getLocalInertia();
      RLOG(0, "%s inertia from Bullet: %f %f %f", bdy->name,
           bulletInertia[0], bulletInertia[1], bulletInertia[2]);
      cSh->calculateLocalInertia(bdy->m, bulletInertia);
      RLOG(0, "%s shape inertia from Bullet: %f %f %f", bdy->name,
           bulletInertia[0], bulletInertia[1], bulletInertia[2]);
    }
  }

  // Deactivation time in an environment sets the amount of time (in seconds)
  // after objects with a velocity less than a certain threshold will
  // deactivate. Deactivation means that it has become stationary, and
  // physics updates for it are not required until it is activated, or
  // moved, or something collides with it.
  btBody->setDeactivationTime(1.0e8);

  // Never sleep
  btBody->setSleepingThresholds(0.0, 0.0);

  // Assign a bit higher damping for bodies that are not connected through
  // joints
  if (bdy->rigid_body_joints)
  {
    //btBody->setDamping(0.05, 0.85);
    btBody->setDamping(0.1, 0.9);

    btBody->setLinearVelocity(btVector3(bdy->x_dot[0], bdy->x_dot[1], bdy->x_dot[2]));
    btBody->setAngularVelocity(btVector3(bdy->omega[0], bdy->omega[1], bdy->omega[2]));
  }

  // apply material properties
  RCHECK(materialName);
  const PhysicsMaterial material = config->getMaterial(materialName);

  btBody->setFriction(material.getFrictionCoefficient());
  btBody->setRollingFriction(material.getRollingFrictionCoefficient());
  //btBody->setSpinningFriction(0.1);
  btBody->setRestitution(material.getRestitution());

  Vec3d_copy(btBody->A_PB_.org, bdy->Inertia->org);
  Mat3d_copy(btBody->A_PB_.rot, A_PB);

  // For rigid body joints, the joint's weightMetric propoerty is interpreted
  // as flag if the corresponding dof os locked or free. This allows to
  // constrain the rigid body movement in any of the 6 directions.
  if (bdy->rigid_body_joints)
  {
    btVector3 linFac, angFac;
    const RcsJoint* rbj = bdy->jnt;

    linFac[0] = rbj->weightMetric;
    rbj = rbj->next;
    linFac[1] = rbj->weightMetric;
    rbj = rbj->next;
    linFac[2] = rbj->weightMetric;
    rbj = rbj->next;

    angFac[0] = rbj->weightMetric;
    rbj = rbj->next;
    angFac[1] = rbj->weightMetric;
    rbj = rbj->next;
    angFac[2] = rbj->weightMetric;
    rbj = rbj->next;

    btBody->setLinearFactor(linFac);
    btBody->setAngularFactor(angFac);
  }

  RLOG(0, "Successfully created BulletRigidBody for \"%s\"",
       btBody->getBodyName());

  return btBody;
}

/*******************************************************************************
 *
 ******************************************************************************/
const RcsBody* Rcs::BulletRigidBody::getBodyPtr() const
{
  return this->body;
}

/*******************************************************************************
 *
 ******************************************************************************/
const char* Rcs::BulletRigidBody::getBodyName() const
{
  return this->body ? this->body->name : "NULL";
}

/*******************************************************************************
 * Calculate hinge point and axis in Bullet body coordinates. We calculate it
 * using the absoulte transforms of the bodies. They must be consistent, which
 * means that the forward kinematics must have been calculated at least once
 * before calling this function.
 ******************************************************************************/
void Rcs::BulletRigidBody::calcHingeTrans(const RcsJoint* jnt,
                                          btVector3& pivot, btVector3& axis)
{
  HTr A_PI;   // Rotation from world to COM: A_PI = A_PB*A_BI
  Mat3d_mul(A_PI.rot, this->A_PB_.rot, body->A_BI->rot);

  double I_r_com[3];
  Vec3d_transRotate(I_r_com, body->A_BI->rot, body->Inertia->org);
  Vec3d_add(A_PI.org, body->A_BI->org, I_r_com);

  // Transformation from physics body to joint in physics coordinates
  HTr A_JP;   // A_JP = A_JI*A_IP
  RcsJoint* seppl = (RcsJoint*) jnt;
  Mat3d_mulTranspose(A_JP.rot, seppl->A_JI.rot, A_PI.rot);

  Vec3d_sub(A_JP.org, jnt->A_JI.org, A_PI.org);
  Vec3d_rotateSelf(A_JP.org, A_PI.rot);

  // Joint hinge point and axis in physics frame
  for (int i=0; i<3; i++)
  {
    pivot[i] = A_JP.org[i];
    axis[i] = A_JP.rot[jnt->dirIdx][i];
  }

}

/*******************************************************************************
 * Calculate slider transformation in Bullet body coordinates. We calculate it
 * using the absoulte transforms of the bodies. They must be consistent, which
 * means that the forward kinematics must have been calculated at least once
 * before calling this function.
 ******************************************************************************/
btTransform Rcs::BulletRigidBody::calcSliderTrans(const RcsJoint* jnt)
{
  HTr A_PI;   // Rotation from world to COM: A_PI = A_PB*A_BI
  Mat3d_mul(A_PI.rot, this->A_PB_.rot, body->A_BI->rot);

  double I_r_com[3];
  Vec3d_transRotate(I_r_com, body->A_BI->rot, body->Inertia->org);
  Vec3d_add(A_PI.org, body->A_BI->org, I_r_com);

  // Transformation from physics body to joint in physics coordinates
  HTr A_JP;   // A_JP = A_JI*A_IP
  RcsJoint* seppl = (RcsJoint*) jnt;
  Mat3d_mulTranspose(A_JP.rot, seppl->A_JI.rot, A_PI.rot);

  Vec3d_sub(A_JP.org, jnt->A_JI.org, A_PI.org);
  Vec3d_rotateSelf(A_JP.org, A_PI.rot);


  int dirIdx = RcsJoint_getDirectionIndex(jnt);

  if (dirIdx==1)
  {
    Mat3d_rotateSelfAboutXYZAxis(A_JP.rot, 2, M_PI_2);
  }
  else if (dirIdx==2)
  {
    Mat3d_rotateSelfAboutXYZAxis(A_JP.rot, 1, -M_PI_2);
  }

  return btTransformFromHTr(&A_JP);
}

/*******************************************************************************
 * Here we use the sensor's transform to align the fixed joint axes. This
 * allows to read the FT values directly.
 ******************************************************************************/
btTypedConstraint* Rcs::BulletRigidBody::createFixedJoint(const RcsGraph* graph)
{
  btTransform frameInW, frameInA, frameInB;

  // Compute the fixed joint frame in world coordinates
  HTr A_JI;
  HTr_copy(&A_JI, body->A_BI);

  // Lets search for load cells connected to the body. If we found one,
  // its transform is used for the fixed joint.
  RcsSensor* loadCell = NULL;
  RCSGRAPH_TRAVERSE_SENSORS(graph)
  {
    if ((SENSOR->type==RCSSENSOR_LOAD_CELL) && (SENSOR->body==this->body))
    {
      RCHECK_MSG(loadCell==NULL, "Body \"%s\" has more than 1 load cells "
                 "attached", body->name);

      loadCell = SENSOR;

      // If a load cell exists, we consider it's offset
      // transformation for the fixed joint position and orientation.
      HTr_transform(&A_JI, body->A_BI, SENSOR->offset);
      RLOGS(5, "Using load cell transform");
    }

  }  // RCSGRAPH_TRAVERSE_SENSORS(graph)

  // Compute frames in local coordinate systems
  frameInW = btTransformFromHTr(&A_JI);
  frameInA = getWorldTransform().inverse()*frameInW;
  frameInB = parent->getWorldTransform().inverse()*frameInW;

  btFixedConstraint* jnt =
    new btFixedConstraint(*this, *parent, frameInA, frameInB);

  // This allows us to query inter-body reaction forces and moments
  jnt->setJointFeedback(&this->jf);
  jnt->enableFeedback(true);

  // Increase the constraint error correction, since we set a tiny global
  // cfm value to increase contact stability
  jnt->setParam(BT_CONSTRAINT_ERP, 0.8, 0);
  jnt->setParam(BT_CONSTRAINT_ERP, 0.8, 1);
  jnt->setParam(BT_CONSTRAINT_ERP, 0.8, 2);
  jnt->setParam(BT_CONSTRAINT_ERP, 0.8, 3);
  jnt->setParam(BT_CONSTRAINT_ERP, 0.8, 4);
  jnt->setParam(BT_CONSTRAINT_ERP, 0.8, 5);

  setUserPointer((void*) jnt);

  return jnt;
}

/*******************************************************************************
 *
 ******************************************************************************/
btTypedConstraint* Rcs::BulletRigidBody::createJoint(const RcsGraph* graph)
{
  if (parent == NULL)
  {
    RLOG(5, "Skipping joint creation: body \"%s\" has no parent body",
         getBodyName());
    return NULL;
  }

  if ((body->physicsSim != RCSBODY_PHYSICS_DYNAMIC) &&
      (body->physicsSim != RCSBODY_PHYSICS_FIXED))
  {
    RLOG(5, "Skipping joint creation: body \"%s\" is neither dynamic "
         "nor fixed", getBodyName());
    return NULL;
  }

  // Fixed constraint
  if (RcsBody_numJoints(body)==0)
  {
    RLOG(5, "Creating fixed joint between \"%s\" and \"%s\"",
         getBodyName(), parent->getBodyName());

    return createFixedJoint(graph);
  }

  if (RcsBody_numJoints(body) != 1)
  {
    RLOG(1, "Skipping joint: found %d joints in body %s (should be 1)",
         RcsBody_numJoints(body), getBodyName());
    return NULL;
  }

  btTypedConstraint* joint = NULL;

  // Create hinge joint
  if (RcsJoint_isRotation(body->jnt))
  {
    btVector3 pivotInA, pivotInB, axisInA, axisInB;
    bool useReferenceFrameA = false;

    calcHingeTrans(body->jnt, pivotInA, axisInA);
    parent->calcHingeTrans(body->jnt, pivotInB, axisInB);

    joint = new BulletHingeJoint(body->jnt,
                                 graph->q->ele[body->jnt->jointIndex],
                                 *this, *parent, pivotInA, pivotInB,
                                 axisInA, axisInB, useReferenceFrameA);
  }
  // Create slider joint
  else
  {
    bool useReferenceFrameA = false;
    btTransform frameInA = calcSliderTrans(body->jnt);
    btTransform frameInB = parent->calcSliderTrans(body->jnt);

    joint = new BulletSliderJoint(body->jnt,
                                  graph->q->ele[body->jnt->jointIndex],
                                  *this, *parent,
                                  frameInA, frameInB,
                                  useReferenceFrameA);
  }

  return joint;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletRigidBody::getBodyTransform(HTr* A_BI) const
{
  HTr A_PI;
  getPhysicsTransform(&A_PI);

  // Rotation matrix: A_BI = transpose(A_PB)*A_PI
  Mat3d_transposeMul(A_BI->rot, (double (*)[3])A_PB_.rot, A_PI.rot);

  // Origin: I_r_IB = I_r_IP - A_IB*B_r_BP
  //                = A_PI.org - transpose(A_BI)*body->Inertia->org
  const double* B_r_BP = body->Inertia->org;
  Vec3d_transRotate(A_BI->org, A_BI->rot, B_r_BP);
  Vec3d_constMulSelf(A_BI->org, -1.0);
  Vec3d_addSelf(A_BI->org, A_PI.org);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletRigidBody::getLocalBodyTransform(HTr* A_PB) const
{
  HTr_copy(A_PB, &this->A_PB_);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletRigidBody::getPhysicsTransform(HTr* A_PI) const
{
  HTrFromBtTransform(A_PI, getWorldTransform());
  Mat3d_transposeSelf(A_PI->rot);   // worldTransform is column major
}

/*******************************************************************************
 *
 ******************************************************************************/
const HTr* Rcs::BulletRigidBody::getBodyTransformPtr() const
{
  return &this->A_BI_;
}

/*******************************************************************************
 *
 ******************************************************************************/
const HTr* Rcs::BulletRigidBody::getCOMTransformPtr() const
{
  return &this->A_PI_;
}

/*******************************************************************************
 * See information and links on top of BulletSimulation.cpp
 ******************************************************************************/
void Rcs::BulletRigidBody::setBodyTransform(const HTr* A_BI_des)
{
  HTr A_PI;
  HTr_transform(&A_PI, A_BI_des, &this->A_PB_);
  setWorldTransform(btTransformFromHTr(&A_PI));
  setInterpolationWorldTransform(btTransformFromHTr(&A_PI));
  setInterpolationLinearVelocity(btVector3(0.0, 0.0, 0.0));
  setInterpolationAngularVelocity(btVector3(0.0, 0.0, 0.0));
}

/*******************************************************************************
 * See information and links on top of BulletSimulation.cpp
 ******************************************************************************/
void Rcs::BulletRigidBody::setBodyTransform(const HTr* A_BI_des, double dt)
{
  if (isStaticOrKinematicObject())
  {
    HTr A_BI_curr;
    getBodyTransform(&A_BI_curr);
    Vec3d_sub(this->x_dot, A_BI_des->org, A_BI_curr.org);
    Vec3d_constMulSelf(this->x_dot, 1.0/dt);
    Mat3d_getOmega((double (*)[3]) A_BI_des->rot, A_BI_curr.rot, this->omega);
    Vec3d_constMulSelf(this->omega, 1.0/dt);
  }

  HTr A_PI;
  HTr_transform(&A_PI, A_BI_des, &this->A_PB_);
  setWorldTransform(btTransformFromHTr(&A_PI));
  setInterpolationWorldTransform(btTransformFromHTr(&A_PI));
  setInterpolationLinearVelocity(btVector3(0.0, 0.0, 0.0));
  setInterpolationAngularVelocity(btVector3(0.0, 0.0, 0.0));
}

/*******************************************************************************
 * Called from BulletSimulation::simulate()
 ******************************************************************************/
void Rcs::BulletRigidBody::updateBodyTransformFromPhysics()
{
  getBodyTransform(&this->A_BI_);
  getPhysicsTransform(&this->A_PI_);

  if (!isStaticOrKinematicObject())
  {
    btVector3 linearVelocity = getLinearVelocity();
    btVector3 angularVelocity = getAngularVelocity();

    for (int i=0; i<3; ++i)
    {
      this->x_dot[i] = linearVelocity[i];
      this->omega[i] = angularVelocity[i];
    }

  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletRigidBody::reset(const HTr* A_BI_)
{
  Vec3d_setZero(this->x_dot);
  Vec3d_setZero(this->omega);

  setBodyTransform(A_BI_ ? A_BI_ : body->A_BI);
  clearForces();
  setLinearVelocity(btVector3(0.0, 0.0, 0.0));
  setAngularVelocity(btVector3(0.0, 0.0, 0.0));
  //forceActivationState(ACTIVE_TAG);
  activate();
  setDeactivationTime(1.0e8);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletRigidBody::setParentBody(BulletRigidBody* p)
{
  this->parent = p;
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::BulletRigidBody::BulletRigidBody(const btRigidBody::btRigidBodyConstructionInfo& rbInfo,
                                      const RcsBody* body_) :
  btRigidBody(rbInfo), body(body_), parent(NULL), jf()
{
  HTr_setIdentity(&this->A_PB_);
  HTr_setIdentity(&this->A_BI_);
  HTr_setIdentity(&this->A_PI_);
  Vec3d_setZero(this->x_dot);
  Vec3d_setZero(this->omega);
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::BulletRigidBody::~BulletRigidBody()
{
  clearShapes();
  delete getCollisionShape();  // It's safe to call delete on a NULL pointer
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletRigidBody::clearShapes()
{
  btCollisionShape* sh = getCollisionShape();
  btCompoundShape* compound = NULL;

  if (sh->isCompound())
  {
    compound = static_cast<btCompoundShape*>(sh);
  }

  // A body can have a compound shape if several shapes have been added to it.
  if (compound != NULL)
  {
    for (int i= compound->getNumChildShapes()-1; i>=0 ; i--)
    {
      // Some shapes are compounds themselves (e.g. torus or SSR). We need to
      // explicitely delete their children as well.
      if (compound->getChildShape(i)->isCompound())
      {
        btCompoundShape* subCompound = static_cast<btCompoundShape*>(compound->getChildShape(i));
        for (int j=subCompound->getNumChildShapes()-1; j>=0 ; j--)
        {
          btCollisionShape* subChild = subCompound->getChildShape(j);
          subCompound->removeChildShape(subChild);
          delete subChild;
        }
      }

      btCollisionShape* child = compound->getChildShape(i);
      compound->removeChildShape(child);
      delete child;
    }

  }

  //delete sh;
  //sh = NULL;
}

/*******************************************************************************
 *
 ******************************************************************************/
btCollisionShape* Rcs::BulletRigidBody::getShape(const RcsShape* shape)
{
  btCollisionShape* sh = getCollisionShape();

  if (sh==NULL)
  {
    return NULL;
  }

  RcsShape* ptr = (RcsShape*) sh->getUserPointer();

  if (ptr == shape)
  {
    return sh;
  }

  btCompoundShape* cSh = dynamic_cast<btCompoundShape*>(sh);

  if (cSh != NULL)
  {
    for (int i=0; i<cSh->getNumChildShapes(); ++i)
    {
      ptr = (RcsShape*) cSh->getChildShape(i)->getUserPointer();

      if (ptr == shape)
      {
        return cSh->getChildShape(i);
      }
    }
  }

  return NULL;
}
