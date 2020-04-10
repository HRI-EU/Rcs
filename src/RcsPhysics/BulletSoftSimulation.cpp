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

#include "BulletSoftSimulation.h"
#include "PhysicsFactory.h"

#include <Rcs_typedef.h>
#include <Rcs_math.h>
#include <Rcs_macros.h>
#include <Rcs_parser.h>
#include <Rcs_shape.h>
#include <Rcs_utils.h>

#include <BulletDynamics/MLCPSolvers/btDantzigSolver.h>
#include <BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h>
#include <BulletDynamics/MLCPSolvers/btMLCPSolver.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>



static const char className[] = "SoftBullet";
static Rcs::PhysicsFactoryRegistrar<Rcs::BulletSoftSimulation> physics(className);

static inline btSoftRigidDynamicsWorld* softCast(btDynamicsWorld* world)
{
  btSoftRigidDynamicsWorld* sw = dynamic_cast<btSoftRigidDynamicsWorld*>(world);
  RCHECK(sw);
  return sw;
}

namespace Rcs
{

BulletSoftSimulation::BulletSoftSimulation() :
  BulletSimulation(), softBodyWorldInfo(NULL)
{
}

BulletSoftSimulation::BulletSoftSimulation(const BulletSoftSimulation& copyFromMe):
  BulletSimulation(copyFromMe), softBodyWorldInfo(NULL)
{
  RFATAL("Can't copy soft bodies yet");
}

BulletSoftSimulation::BulletSoftSimulation(const BulletSoftSimulation& copyFromMe,
                                           const RcsGraph* newGraph):

  BulletSimulation(copyFromMe, newGraph), softBodyWorldInfo(NULL)
{
  RFATAL("Can't copy soft bodies yet");
}

BulletSoftSimulation::~BulletSoftSimulation()
{
  delete this->softBodyWorldInfo;
}

const char* BulletSoftSimulation::getClassName() const
{
  return className;
}

bool BulletSoftSimulation::initialize(const RcsGraph* g,
                                      const PhysicsConfig* config)
{
  initGraph(g);
  initPhysics(config);
  createSoftBodies();
  return true;
}

void BulletSoftSimulation::updateSoftMeshes()
{
  btSoftRigidDynamicsWorld* softWorld = softCast(this->dynamicsWorld);

  btSoftBodyArray& arr = softWorld->getSoftBodyArray();

  for (int i=0; i<arr.size(); ++i)
  {
    btSoftBody* sbi = arr[i];
    RcsBody* rcsSoftBdy = (RcsBody*) sbi->getUserPointer();
    RcsShape* softShape = rcsSoftBdy->shape[0];
    RCHECK(softShape->type==RCSSHAPE_MESH);
    RcsMeshData* dstMesh = (RcsMeshData*) softShape->userData;

    const size_t nValues = 3*sbi->m_faces.size();
    dstMesh->vertices = (double*) realloc(dstMesh->vertices,
                                          3*nValues*sizeof(double));
    dstMesh->nVertices = nValues;
    dstMesh->faces = (unsigned int*) realloc(dstMesh->faces,
                                             nValues*sizeof(unsigned int));
    dstMesh->nFaces = sbi->m_faces.size();

    RCHECK(RcsMesh_check(dstMesh));
    

    for (i=0; i<sbi->m_faces.size(); ++i)
    {
      const btSoftBody::Face& f = sbi->m_faces[i];
      const btScalar  scl=(btScalar)0.9;
      const btVector3 x[]= {f.m_n[0]->m_x,f.m_n[1]->m_x,f.m_n[2]->m_x};
      const btVector3 c=(x[0]+x[1]+x[2])/3.0;

      const int i3 = 3*i;
      dstMesh->faces[i3+0] = i3+0;
      dstMesh->faces[i3+1] = i3+1;
      dstMesh->faces[i3+2] = i3+2;

      btVector3 v0 = (x[0]-c)*scl+c;
      btVector3 v1 = (x[1]-c)*scl+c;
      btVector3 v2 = (x[2]-c)*scl+c;

      double* vtx0 = &dstMesh->vertices[3*(i3)];
      double* vtx1 = &dstMesh->vertices[3*(i3+1)];
      double* vtx2 = &dstMesh->vertices[3*(i3+2)];

      Vec3d_set(vtx0, v0[0], v0[1], v0[2]);
      Vec3d_set(vtx1, v1[0], v1[1], v1[2]);
      Vec3d_set(vtx2, v2[0], v2[1], v2[2]);

      Vec3d_invTransformSelf(vtx0, rcsSoftBdy->A_BI);
      Vec3d_invTransformSelf(vtx1, rcsSoftBdy->A_BI);
      Vec3d_invTransformSelf(vtx2, rcsSoftBdy->A_BI);
    }

    // if (fabs(time() - 1.0)<1.0e-5)

    REXEC(1)
      {
    // RcsMesh_toFile(dstMesh, "deformed.stl");
    RLOG(0, "nFaces=%d nVertices=%d", dstMesh->nFaces, dstMesh->nVertices);
    RcsMesh_print(dstMesh);
      }
  }


}

// void BulletSoftSimulation::updateSoftMeshes()
// {
//   btSoftRigidDynamicsWorld* softWorld = softCast(this->dynamicsWorld);

//   btSoftBodyArray& arr = softWorld->getSoftBodyArray();

//   for (int i=0; i<arr.size(); ++i)
//   {
//     btSoftBody* sbi = arr[i];
//     RcsBody* rcsSoftBdy = (RcsBody*) sbi->getUserPointer();
//     RcsShape* softShape = rcsSoftBdy->shape[0];
//     RCHECK(softShape->type==RCSSHAPE_MESH);
//     RcsMeshData* dstMesh = (RcsMeshData*) softShape->userData;

//     const size_t nValues = 3*sbi->m_faces.size();
//     dstMesh->vertices = (double*) realloc(dstMesh->vertices,
//                                           3*nValues*sizeof(double));
//     dstMesh->faces = (unsigned int*) realloc(dstMesh->faces,
//                                              nValues*sizeof(unsigned int));

//     for (i=0; i<sbi->m_faces.size(); ++i)
//     {
//       const btSoftBody::Face& f = sbi->m_faces[i];
//       const btScalar  scl=(btScalar)0.9;
//       const btVector3 x[]= {f.m_n[0]->m_x,f.m_n[1]->m_x,f.m_n[2]->m_x};
//       const btVector3 c=(x[0]+x[1]+x[2])/3.0;

//       const int i3 = 3*i;
//       dstMesh->faces[i3+0] = i3+0;
//       dstMesh->faces[i3+1] = i3+1;
//       dstMesh->faces[i3+2] = i3+2;

//       btVector3 v0 = (x[0]-c)*scl+c;
//       btVector3 v1 = (x[1]-c)*scl+c;
//       btVector3 v2 = (x[2]-c)*scl+c;

//       double* vtx0 = &dstMesh->vertices[3*(i3)];
//       double* vtx1 = &dstMesh->vertices[3*(i3+1)];
//       double* vtx2 = &dstMesh->vertices[3*(i3+2)];

//       Vec3d_set(vtx0, v0[0], v0[1], v0[2]);
//       Vec3d_set(vtx1, v1[0], v1[1], v1[2]);
//       Vec3d_set(vtx2, v2[0], v2[1], v2[2]);

//       Vec3d_invTransformSelf(vtx0, rcsSoftBdy->A_BI);
//       Vec3d_invTransformSelf(vtx1, rcsSoftBdy->A_BI);
//       Vec3d_invTransformSelf(vtx2, rcsSoftBdy->A_BI);
//     }

//     // if (fabs(time() - 1.0)<1.0e-5)
//     // RcsMesh_toFile(dstMesh, "deformed.stl");
//     // RcsMesh_print(dstMesh);
//   }


// }

void BulletSoftSimulation::createWorld(xmlNodePtr bulletParams)
{
  bool useMCLPSolver = false;

  if (bulletParams)
  {
    // load solver type from xml
    getXMLNodePropertyBoolString(bulletParams, "use_mclp_solver",
                                 &useMCLPSolver);
  }

  this->collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
  this->dispatcher = new btCollisionDispatcher(collisionConfiguration);
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

  this->dynamicsWorld = new btSoftRigidDynamicsWorld(dispatcher, broadPhase,
                                                     solver,
                                                     collisionConfiguration);
  dynamicsWorld->setGravity(btVector3(0.0, 0.0, -RCS_GRAVITY));
  this->softBodyWorldInfo = new btSoftBodyWorldInfo();
  this->softBodyWorldInfo->m_broadphase = broadPhase;
  this->softBodyWorldInfo->m_dispatcher = dispatcher;
  this->softBodyWorldInfo->m_gravity = dynamicsWorld->getGravity();
  this->softBodyWorldInfo->m_sparsesdf.Initialize();

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

void BulletSoftSimulation::createSoftBodies()
{
  convertShapesToMesh();

  btSoftRigidDynamicsWorld* softWorld = softCast(this->dynamicsWorld);

  RCSGRAPH_TRAVERSE_BODIES(getGraph())
  {
    RCSBODY_TRAVERSE_SHAPES(BODY)
    {
      if ((SHAPE->computeType & RCSSHAPE_COMPUTE_SOFTPHYSICS) == 0)
      {
        continue;
      }

      RLOG(0, "Creating soft body for %s", BODY->name);
      RcsMeshData* softMesh = (RcsMeshData*)SHAPE->userData;
      RCHECK_MSG(softMesh, "Could not create mesh from file %s",
                 SHAPE->meshFile ? SHAPE->meshFile : NULL);

      // Here we need to transform all vertices with the body transform.
      RcsMesh_transform(softMesh, BODY->A_BI->org, BODY->A_BI->rot);

      // We create an array of btScalar, since this can be of type float.
      // That's incompatible with our double representation.
      btScalar* btVerts = RNALLOC(3*softMesh->nVertices, btScalar);
      for (unsigned int i=0; i<3*softMesh->nVertices; ++i)
      {
        btVerts[i] = softMesh->vertices[i];
      }

      btSoftBody* softBdy;
      softBdy = btSoftBodyHelpers::CreateFromTriMesh(*softBodyWorldInfo,
                                                     btVerts,
                                                     (int*)softMesh->faces,
                                                     softMesh->nFaces);

      // For all parameters, see btSoftBody.h (struct Config)
      softBdy->m_cfg.kDF = 0.5;
      softBdy->m_cfg.kMT = 0.05;// was 0.05
      softBdy->m_cfg.piterations = 25;

      // softBdy->m_cfg.kSRHR_CL = 1.0;
      // softBdy->m_cfg.kCHR = 1.0;
      // softBdy->m_cfg.kKHR = 1.0;
      // softBdy->m_cfg.kSHR = 1.0;

      softBdy->randomizeConstraints();
      softBdy->setTotalMass(BODY->m, true);
      softBdy->setPose(false, true);
      softBdy->getCollisionShape()->setMargin(0.01);
      softBdy->setUserPointer((void*) BODY);

      softWorld->addSoftBody(softBdy);
    }
  }

}

void BulletSoftSimulation::convertShapesToMesh()
{
  RCSGRAPH_TRAVERSE_BODIES(getGraph())
  {
    RCSBODY_TRAVERSE_SHAPES(BODY)
    {
      if ((SHAPE->computeType & RCSSHAPE_COMPUTE_SOFTPHYSICS) == 0)
      {
        continue;
      }

      if (SHAPE->type == RCSSHAPE_MESH)
      {
        continue;
      }

      RcsMeshData* shapeMesh = RcsShape_createMesh(SHAPE);

      if (shapeMesh)
        {
          SHAPE->userData = (void*) shapeMesh;
          SHAPE->type = RCSSHAPE_MESH;
          SHAPE->computeType = RCSSHAPE_COMPUTE_SOFTPHYSICS;
          SHAPE->meshFile = String_clone(RcsShape_name(SHAPE->type));
          RLOG(0, "Successfully converted shape %s of body %s",
               RcsShape_name(SHAPE->type), BODY->name);
          Math_printBinaryVector(SHAPE->computeType);
          // RcsMesh_print(shapeMesh);
        }
      else
        {
          RLOG(0, "Failed to convert shape %s of body %s",
               RcsShape_name(SHAPE->type), BODY->name);
        }

      
    }   // RCSBODY_TRAVERSE_SHAPES

  }   // RCSGRAPH_TRAVERSE_BODIES
}

}  // namespace Rcs 
