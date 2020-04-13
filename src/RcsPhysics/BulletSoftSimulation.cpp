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
#include "BulletRigidBody.h"
#include "PhysicsFactory.h"

#include <Rcs_typedef.h>
#include <Rcs_math.h>
#include <Rcs_macros.h>
#include <Rcs_parser.h>
#include <Rcs_shape.h>
#include <Rcs_utils.h>
#include <Rcs_body.h>

#include <BulletDynamics/MLCPSolvers/btDantzigSolver.h>
#include <BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h>
#include <BulletDynamics/MLCPSolvers/btMLCPSolver.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>



static const char className[] = "SoftBullet";
static Rcs::PhysicsFactoryRegistrar<Rcs::BulletSoftSimulation> physics(className);



namespace Rcs
{

BulletSoftSimulation::BulletSoftSimulation() :
  BulletSimulation(), softBodyWorldInfo(NULL), softWorld(NULL)
{
}

BulletSoftSimulation::BulletSoftSimulation(const BulletSoftSimulation& copyFromMe):
  BulletSimulation(copyFromMe), softBodyWorldInfo(NULL), softWorld(NULL)
{
  RFATAL("Can't copy soft bodies yet");
}

BulletSoftSimulation::BulletSoftSimulation(const BulletSoftSimulation& copyFromMe,
                                           const RcsGraph* newGraph):

  BulletSimulation(copyFromMe, newGraph), softBodyWorldInfo(NULL), softWorld(NULL)
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


    // Transformation from world into shape's frame
    HTr A_CI;
    HTr_transform(&A_CI, rcsSoftBdy->A_BI, &softShape->A_CB);

    for (int j=0; j<sbi->m_faces.size(); ++j)
    {
      const btSoftBody::Face& f = sbi->m_faces[j];
      const btScalar  scl=(btScalar)0.99;
      const btVector3 x[]= {f.m_n[0]->m_x, f.m_n[1]->m_x, f.m_n[2]->m_x};
      const btVector3 c=(x[0]+x[1]+x[2])/3.0;
      const int j3 = 3*j;

      dstMesh->faces[j3+0] = j3+0;
      dstMesh->faces[j3+1] = j3+1;
      dstMesh->faces[j3+2] = j3+2;

      btVector3 v0 = (x[0]-c)*scl+c;
      btVector3 v1 = (x[1]-c)*scl+c;
      btVector3 v2 = (x[2]-c)*scl+c;

      double* vtx0 = &dstMesh->vertices[3*(j3)];
      double* vtx1 = &dstMesh->vertices[3*(j3+1)];
      double* vtx2 = &dstMesh->vertices[3*(j3+2)];

      Vec3d_set(vtx0, v0[0], v0[1], v0[2]);
      Vec3d_set(vtx1, v1[0], v1[1], v1[2]);
      Vec3d_set(vtx2, v2[0], v2[1], v2[2]);

      RCHECK_MSG(Vec3d_isFinite(vtx0), "%s", rcsSoftBdy->name);
      RCHECK_MSG(Vec3d_isFinite(vtx1), "%s", rcsSoftBdy->name);
      RCHECK_MSG(Vec3d_isFinite(vtx2), "%s", rcsSoftBdy->name);

      Vec3d_invTransformSelf(vtx0, &A_CI);
      Vec3d_invTransformSelf(vtx1, &A_CI);
      Vec3d_invTransformSelf(vtx2, &A_CI);
    }

  }   // arr.size()

}

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

  this->softWorld = new btSoftRigidDynamicsWorld(dispatcher, broadPhase,
                                                 solver,
                                                 collisionConfiguration, NULL);
  this->dynamicsWorld = softWorld;

  softWorld->setGravity(btVector3(0.0, 0.0, -RCS_GRAVITY));
  this->softBodyWorldInfo = new btSoftBodyWorldInfo();
  this->softBodyWorldInfo->m_broadphase = broadPhase;
  this->softBodyWorldInfo->m_dispatcher = dispatcher;
  this->softBodyWorldInfo->m_gravity = softWorld->getGravity();
  this->softBodyWorldInfo->m_sparsesdf.Initialize();

  btDispatcherInfo& di = softWorld->getDispatchInfo();
  di.m_useConvexConservativeDistanceUtil = true;
  di.m_convexConservativeDistanceThreshold = 0.01f;

  btContactSolverInfo& si = softWorld->getSolverInfo();
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

  RCSGRAPH_TRAVERSE_BODIES(getGraph())
  {
    RCSBODY_TRAVERSE_SHAPES(BODY)
    {
      if ((SHAPE->computeType & RCSSHAPE_COMPUTE_SOFTPHYSICS) == 0)
      {
        continue;
      }

      RLOG(5, "Creating soft body for %s", BODY->name);
      RcsMeshData* softMesh = (RcsMeshData*)SHAPE->userData;
      RCHECK_MSG(softMesh, "Could not create mesh from file %s",
                 SHAPE->meshFile ? SHAPE->meshFile : NULL);

      // Here we need to transform all vertices with the shape's transform.
      HTr A_CI;
      HTr_transform(&A_CI, BODY->A_BI, &SHAPE->A_CB);
      RcsMesh_transform(softMesh, A_CI.org, A_CI.rot);
      btSoftBody* softBdy = NULL;

      if (STREQ(SHAPE->meshFile, "RCSSHAPE_SSL") ||
          STREQ(SHAPE->meshFile, "RCSSHAPE_SSR") ||
          STREQ(SHAPE->meshFile, "RCSSHAPE_BOX") ||
          STREQ(SHAPE->meshFile, "RCSSHAPE_CYLINDER") ||
          STREQ(SHAPE->meshFile, "RCSSHAPE_CONE"))
      {
        btVector3* hull = new btVector3[softMesh->nVertices];
        for (unsigned int i=0; i<softMesh->nVertices; ++i)
        {
          const double* v = &softMesh->vertices[3*i];
          hull[i] = btVector3(v[0], v[1], v[2]);
        }

        softBdy = btSoftBodyHelpers::CreateFromConvexHull(*softBodyWorldInfo,
                                                          hull,
                                                          softMesh->nVertices);
        delete [] hull;
      }
      else if (STREQ(SHAPE->meshFile, "RCSSHAPE_SPHERE"))
      {
        const int res = 256;
        const double r = SHAPE->extents[0];
        btVector3 center(A_CI.org[0], A_CI.org[1], A_CI.org[2]);
        btVector3 radius(r, r, r);
        softBdy = btSoftBodyHelpers::CreateEllipsoid(*softBodyWorldInfo,
                                                     center, radius, res);
      }
      else
      {
        // We create an array of btScalar, since this can be of type float.
        // That's incompatible with our double representation.
        btScalar* btVerts = RNALLOC(3*softMesh->nVertices, btScalar);
        for (unsigned int i=0; i<3*softMesh->nVertices; ++i)
        {
          btVerts[i] = softMesh->vertices[i];
        }

        softBdy = btSoftBodyHelpers::CreateFromTriMesh(*softBodyWorldInfo,
                                                       btVerts,
                                                       (int*)softMesh->faces,
                                                       softMesh->nFaces);
        RFREE(btVerts);
      }

      // For all parameters, see btSoftBody.h (struct Config)
      RCHECK_MSG(softBdy, "Failed to create soft body for %s", BODY->name);
      RCHECK_MSG(BODY->m>0.0, "Soft body %s has zero mass", BODY->name);
#if 1
      softBdy->m_cfg.kDF = 0.9;
      softBdy->m_cfg.kDP = 0.01;// damping
      softBdy->m_cfg.kMT = 0.5;// was 0.05
      softBdy->m_cfg.piterations = 25;
      //softBdy->m_cfg.viterations = 25;
      //softBdy->m_cfg.diterations = 25;
      softBdy->m_cfg.kVCF = 0.1;
#endif

      // softBdy->m_cfg.kSRHR_CL = 1.0;
      // softBdy->m_cfg.kCHR = 1.0;
      // softBdy->m_cfg.kKHR = 1.0;
      // softBdy->m_cfg.kSHR = 1.0;

      softBdy->randomizeConstraints();
      softBdy->setTotalMass(BODY->m, true);
      //softBdy->setPose(false, true);
      softBdy->setPose(true, true);
      softBdy->getCollisionShape()->setMargin(0.0);
      softBdy->setUserPointer((void*) BODY);

      // softBdy->m_cfg.collisions = btSoftBody::fCollision::CL_SS +
      // btSoftBody::fCollision::CL_RS;
      // softBdy->generateClusters(8);





      // Link soft body to parent if it exists and the body physics type is
      // fixed
      if (BODY->parent)
      {
        RLOG(0, "Body %s has %zu nodes (%zu faces %zu vertices) ",
             BODY->name, softBdy->m_nodes.size(),
             softMesh->nFaces, softMesh->nVertices);

        RCHECK(BODY->physicsSim==RCSBODY_PHYSICS_FIXED);
        std::map<const RcsBody*, Rcs::BulletRigidBody*>::iterator it;
        it = bdyMap.find(BODY->parent);
        RCHECK(it!=bdyMap.end());
        BulletRigidBody* bParent = it->second;

        int anchoredVertices = connectSoftToRigidBody(softBdy, bParent);
        RLOG(0, "Anchored %d vertices to parent", anchoredVertices);
      }

      // End link soft body to parent


      // Link rigid child bodies to soft parent
      RcsBody* child = BODY->firstChild;

      while (child)
      {
        RCHECK(child->physicsSim==RCSBODY_PHYSICS_FIXED);
        std::map<const RcsBody*, Rcs::BulletRigidBody*>::iterator it;
        it = bdyMap.find(child);
        RCHECK(it!=bdyMap.end());
        BulletRigidBody* bChild = it->second;

        int anchoredVertices = connectSoftToRigidBody(softBdy, bChild);
        RLOG(0, "Anchored %d vertices to child %s",
             anchoredVertices, child->name);

        child = child->next;
      }








      softWorld->addSoftBody(softBdy);
    }
  }

  RLOG(5, "Done creating soft bodies");
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
        SHAPE->meshFile = String_clone(RcsShape_name(SHAPE->type));
        SHAPE->type = RCSSHAPE_MESH;
        SHAPE->computeType = RCSSHAPE_COMPUTE_SOFTPHYSICS;
      }
      else
      {
        RLOG(0, "Failed to convert shape %s of body %s",
             RcsShape_name(SHAPE->type), BODY->name);
      }

    }   // RCSBODY_TRAVERSE_SHAPES

  }   // RCSGRAPH_TRAVERSE_BODIES
}

int BulletSoftSimulation::connectSoftToRigidBody(btSoftBody* softBdy,
                                                 BulletRigidBody* rigidBdy)
{
  int anchoredVertices = 0;

  for (int j=0; j<softBdy->m_nodes.size(); ++j)
  {
    double vtx[3];
    vtx[0] = softBdy->m_nodes[j].m_x.x();
    vtx[1] = softBdy->m_nodes[j].m_x.y();
    vtx[2] = softBdy->m_nodes[j].m_x.z();

    double dParent = RcsBody_distanceToPoint(rigidBdy->getBodyPtr(), vtx,
                                             NULL, NULL);

    if (dParent < 0.0)
    {
      anchoredVertices++;
      softBdy->appendAnchor(j, rigidBdy);
    }

  }
  RLOG(0, "Anchored %d vertices to parent", anchoredVertices);

  return anchoredVertices;
}



}  // namespace Rcs
