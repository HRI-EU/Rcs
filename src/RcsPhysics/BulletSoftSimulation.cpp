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



static void setSoftMaterial(btSoftBody* softBdy, int materialId)
{
  switch (materialId)
  {
    case 0:   // "Spongy" solid objects
    {
      softBdy->m_cfg.kDF = (btScalar)0.9;
      softBdy->m_cfg.kDP = (btScalar)0.01;// damping
      softBdy->m_cfg.kMT = (btScalar)0.5;// Pose matching coefficient [0,1]  was 0.05
      softBdy->m_cfg.piterations = 25;
      //softBdy->m_cfg.viterations = 25;
      //softBdy->m_cfg.diterations = 25;
      softBdy->m_cfg.kVCF = (btScalar)0.1;
      //softBdy->m_cfg.kVC = 0.01;
      // softBdy->m_cfg.kSRHR_CL = 1.0;
      // softBdy->m_cfg.kCHR = 1.0;
      // softBdy->m_cfg.kKHR = 1.0;
      // softBdy->m_cfg.kSHR = 1.0;

      // softBdy->generateBendingConstraints(2, softBdy->appendMaterial());
      softBdy->setPose(true, true);
    }
    break;

    case 1:// "cloth"
    {
      softBdy->m_cfg.kDF = (btScalar)0.9; // dynamic friction
      softBdy->m_cfg.kDP = (btScalar)0.01;// damping
      softBdy->m_cfg.kMT = (btScalar)0.00;// Pose matching coefficient [0,1]  was 0.05
      softBdy->m_cfg.kVCF = (btScalar)0.1;// Velocities correction factor (Baumgarte)
      softBdy->m_cfg.kVC = (btScalar)0.0;// Volume conversation coefficient [0,+inf]
      softBdy->m_cfg.piterations = 250;
      softBdy->m_cfg.citerations = 10;
      softBdy->m_cfg.diterations = 10;

      softBdy->m_cfg.kCHR = (btScalar)1.0;   // Rigid contacts hardness [0,1]
      softBdy->m_cfg.kKHR = (btScalar)1.0;   // Kinetic contacts hardness [0,1]
      softBdy->m_cfg.kSHR = (btScalar)1.0;   // Soft contacts hardness [0,1]

      //softBdy->m_cfg.collisions = btSoftBody::fCollision::SDF_RS;// + btSoftBody::fCollision::SDF_RDF;
      // btSoftBody::fCollision::CL_SS +
      // btSoftBody::fCollision::CL_RS;

      softBdy->m_cfg.collisions = btSoftBody::fCollision::CL_SS +
                                  btSoftBody::fCollision::CL_RS;
      softBdy->generateClusters(1200);
    }
    break;

    case 2:// "cloth2"
    {
      softBdy->m_cfg.kMT = (btScalar)0.0;// Pose matching coefficient [0,1]
      softBdy->m_cfg.kVCF = (btScalar)1.0;// Velocities correction factor (Baumgarte)
      softBdy->m_cfg.kVC = (btScalar)0.0;// Volume conversation coefficient [0,+inf]
      softBdy->m_cfg.piterations = 250;
      softBdy->m_cfg.citerations = 10;
      softBdy->m_cfg.diterations = 10;

      softBdy->m_cfg.kCHR = (btScalar)1.0;   // Rigid contacts hardness [0,1]
      softBdy->m_cfg.kKHR = (btScalar)1.0;   // Kinetic contacts hardness [0,1]
      softBdy->m_cfg.kSHR = (btScalar)1.0;   // Soft contacts hardness [0,1]
    }
    break;

    case 3:
    {
      btSoftBody::Material* material = softBdy->appendMaterial();
      material->m_kLST = (btScalar)1.0;  // Linear stiffness coefficient [0,1]
      material->m_kAST = (btScalar)1.0;  // Area/Angular stiffness coefficient [0,1]

      softBdy->m_cfg.kDF = (btScalar)0.0; // dynamic friction
      softBdy->m_cfg.kDP = (btScalar)0.1;// damping
      softBdy->m_cfg.kMT = (btScalar)0.0;// Pose matching coefficient [0,1]  was 0.05
      softBdy->m_cfg.kVCF = (btScalar)0.1;// Velocities correction factor (Baumgarte)
      softBdy->m_cfg.kVC = (btScalar)0.0;// Volume conversation coefficient [0,+inf]
      softBdy->m_cfg.piterations = 250;
      softBdy->m_cfg.citerations = 100;
      softBdy->m_cfg.diterations = 100;

      // softBdy->m_cfg.kAHR = 1.0;   // Anchor hardness

      // softBdy->m_cfg.kCHR = 1.0;   // Rigid contacts hardness [0,1]
      // softBdy->m_cfg.kKHR = 1.0;   // Kinetic contacts hardness [0,1]
      // softBdy->m_cfg.kSHR = 1.0;   // Soft contacts hardness [0,1]
    }
    break;

    default:
      break;
  }

}

namespace Rcs
{

BulletSoftSimulation::BulletSoftSimulation() :
  BulletSimulation(), softBodyWorldInfo(NULL), softWorld(NULL),
  transformVerticesToShapeFrame(true)
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
  for (int i=softWorld->getNumCollisionObjects()-1; i>=0 ; i--)
  {
    btCollisionObject* obj = softWorld->getCollisionObjectArray()[i];
    btSoftBody* body = btSoftBody::upcast(obj);

    if (body)
    {
      softWorld->removeSoftBody(body);
      delete body;
    }

  }

  delete this->softBodyWorldInfo;
}

const char* BulletSoftSimulation::getClassName() const
{
  return className;
}

bool BulletSoftSimulation::initialize(const RcsGraph* graph,
                                      const char* physicsCfg)
{
  return PhysicsBase::initialize(graph, physicsCfg);
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
    int bodyId = sbi->getUserIndex();
    RcsBody* rcsSoftBdy = RCSBODY_BY_ID(getGraph(), bodyId);
    RCHECK(rcsSoftBdy);
    RcsShape* softShape = &rcsSoftBdy->shapes[0];
    RCHECK(softShape->type==RCSSHAPE_MESH);
    RcsMeshData* dstMesh = softShape->mesh;

    const size_t nf = sbi->m_faces.size();
    const size_t nv = 3*nf;

    if (dstMesh->nVertices != nv)
    {
      dstMesh->vertices = (double*) realloc(dstMesh->vertices,
                                            3*nv*sizeof(double));
      RCHECK_MSG(dstMesh->vertices, "Failed to reallocate %zd bytes for "
                 " %zd vertices", 3*nv*sizeof(double), nv);
      dstMesh->nVertices = nv;
    }

    if (dstMesh->nFaces != nf)
    {
      dstMesh->faces = (unsigned int*) realloc(dstMesh->faces,
                                               3*nf*sizeof(unsigned int));
      RCHECK_MSG(dstMesh->faces, "Failed to reallocate %zd bytes for %zd faces",
                 3*nf*sizeof(unsigned int), nf);
      dstMesh->nFaces = nf;
    }

    // The vertices of all soft objects are represented in the world frame.
    // There is no equivalent to a rigid body transform. If there is a good
    // answer why and how to do this, it should probably be done here. For
    // instance: Compute the initial vertex centroid and store it in a
    // relative body transform. During simulation, propagate A_BI through the
    // centroid. Probably we need to get a notion of the rotations as well,
    // for instance through the Eigenvectors of the vertices. But these might
    // make the body flip around and not keep consistent rotations if the
    // Eigenvalues change their order.
    HTr A_CI;
    HTr_transform(&A_CI, &rcsSoftBdy->A_BI, &softShape->A_CB);

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

      // Transformation from world into shape's frame so that parent-child
      // relations in the graphics scene graph are preserved. The vertices
      // become a child of the shape, same as any other shape. Note that the
      // shape transform in this case is not moving.
      if (transformVerticesToShapeFrame == true)
      {
        Vec3d_invTransformSelf(vtx0, &A_CI);
        Vec3d_invTransformSelf(vtx1, &A_CI);
        Vec3d_invTransformSelf(vtx2, &A_CI);
      }
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

  softWorld->setGravity(btVector3((btScalar)0.0, (btScalar)0.0, (btScalar)-RCS_GRAVITY));
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
  si.m_erp = (btScalar)0.2;
  si.m_globalCfm = (btScalar)1.0e-4; // 0: hard constraint (Default)
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
      if (!RcsShape_isOfComputeType(SHAPE, RCSSHAPE_COMPUTE_SOFTPHYSICS))
      {
        continue;
      }

      RLOG(5, "Creating soft body for %s", BODY->name);
      RcsMeshData* softMesh = SHAPE->mesh;

      if (softMesh == NULL)
      {
        RLOG(1, "Could not create mesh from file %s - skipping soft body",
             SHAPE->meshFile);
        continue;
      }

      // Here we need to transform all vertices with the shape's transform.
      HTr A_CI;
      HTr_transform(&A_CI, &BODY->A_BI, &SHAPE->A_CB);
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
          hull[i] = btVector3((btScalar)v[0], (btScalar)v[1], (btScalar)v[2]);
        }

        softBdy = btSoftBodyHelpers::CreateFromConvexHull(*softBodyWorldInfo,
                                                          hull,
                                                          softMesh->nVertices);
        delete [] hull;
      }
      else if (STREQ(SHAPE->meshFile, "RCSSHAPE_SPHERE"))
      {
        const int res = 256;
        const btScalar r = (btScalar)SHAPE->extents[0];
        btVector3 center((btScalar)A_CI.org[0], (btScalar)A_CI.org[1], (btScalar)A_CI.org[2]);
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
          btVerts[i] = (btScalar)softMesh->vertices[i];
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

      int materialId = 0;
      if (STRCASEEQ(SHAPE->material, "cloth"))
      {
        materialId = 1;
      }
      else if (STRCASEEQ(SHAPE->material, "cloth2"))
      {
        materialId = 2;
      }
      else if (STRCASEEQ(SHAPE->material, "cloth3"))
      {
        materialId = 3;
      }

      setSoftMaterial(softBdy, materialId);

      softBdy->randomizeConstraints();
      softBdy->setTotalMass((btScalar)BODY->m, true);
      softBdy->getCollisionShape()->setMargin(0.0);
      softBdy->setUserIndex(BODY->id);
      btSoftBodyHelpers::ReoptimizeLinkOrder(softBdy);

      // Link soft body to parent if it exists and the body physics type is
      // fixed
      //const RcsBody* parent = RcsBody_getConstParent(getGraph(), BODY);
      const RcsBody* parent = RCSBODY_BY_ID(getGraph(), BODY->parentId);

      if (parent)
      {
        RLOG_CPP(5, "Body " << BODY->name << " has "
                 << softBdy->m_nodes.size() << " nodes (" << softMesh->nFaces
                 << " faces " << softMesh->nVertices << " vertices) ");

        // RCHECK_MSG(BODY->physicsSim==RCSBODY_PHYSICS_FIXED,
        //            "Body: %s   parent: %s", BODY->name, parent->name);
        std::map<int, Rcs::BulletRigidBody*>::iterator it;
        it = bdyMap.find(parent->id);
        RCHECK(it!=bdyMap.end());
        BulletRigidBody* bParent = it->second;

        int anchoredVertices = connectSoftToRigidBody(softBdy, bParent);
        RLOG(5, "Anchored %d vertices to parent %s",
             anchoredVertices, parent->name);
      }

      // Link rigid child bodies to soft parent
      RcsBody* child = RcsBody_getFirstChild(getGraph(), BODY);

      while (child)
      {
        RCHECK(child->physicsSim==RCSBODY_PHYSICS_FIXED);
        std::map<int, Rcs::BulletRigidBody*>::iterator it;
        it = bdyMap.find(child->id);
        RCHECK_MSG(it!=bdyMap.end(), "Failed to find body \"%s\" (id %d)",
                   child->name, child->id);
        BulletRigidBody* bChild = it->second;

        int anchoredVertices = connectSoftToRigidBody(softBdy, bChild);
        RLOG(5, "Anchored %d vertices to child %s",
             anchoredVertices, child->name);

        child = RcsBody_getNext(getGraph(), child);
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
      if (!RcsShape_isOfComputeType(SHAPE, RCSSHAPE_COMPUTE_SOFTPHYSICS))
      {
        continue;
      }

      // If the shape is already a mesh, we only remove the duplicate vertices
      // and continue.
      if (SHAPE->type == RCSSHAPE_MESH)
      {
        RcsMeshData* shapeMesh = SHAPE->mesh;

        if (shapeMesh)
        {
          RLOG(5, "Mesh %s has %d vertices and %d facecs",
               BODY->name, shapeMesh->nVertices, shapeMesh->nFaces);
          int nDuplicates = RcsMesh_compressVertices(shapeMesh, 1.0e-8);
          RLOG(5, "Reduced mesh by %d duplicates - now %d vertices and %d faces",
               nDuplicates, shapeMesh->nVertices, shapeMesh->nFaces);
        }
        continue;
      }

      // Otherwise, we convert the shape into a mesh using the shape's mesh
      // creation functionality
      RcsMeshData* shapeMesh = RcsShape_createMesh(SHAPE);

      if (shapeMesh)
      {
        RLOG(5, "Mesh %s has %d vertices and %d facecs",
             BODY->name, shapeMesh->nVertices, shapeMesh->nFaces);
        int nDuplicates = RcsMesh_compressVertices(shapeMesh, 1.0e-8);
        RLOG(5, "Reduced mesh by %d duplicates - now %d vertices and %d faces",
             nDuplicates, shapeMesh->nVertices, shapeMesh->nFaces);

        SHAPE->mesh = shapeMesh;
        snprintf(SHAPE->meshFile, RCS_MAX_FILENAMELEN, "%s",
                 RcsShape_name(SHAPE->type));
        SHAPE->type = RCSSHAPE_MESH;
        SHAPE->computeType |= RCSSHAPE_COMPUTE_SOFTPHYSICS;
        SHAPE->computeType |= RCSSHAPE_COMPUTE_RESIZEABLE;
      }
      else
      {
        RLOG(1, "Failed to convert shape %s of body %s",
             RcsShape_name(SHAPE->type), BODY->name);
      }

    }   // RCSBODY_TRAVERSE_SHAPES

  }   // RCSGRAPH_TRAVERSE_BODIES
}

// This function walks through all vertices of a softbody mesh and checks if
// a vertex lies inside the rigid body. If it is the case, a rigid link between
// soft and rigid body is created. This will "meld" the bodies togehter. The
// checking of the overlap relies on the distance function between point and
// rigid body shapes. Currently there is no good function for non-convex meshes,
// therefore this might not work perfectly (it will assume the mesh being
// convex).
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
  RLOG(5, "Anchored %d vertices to parent", anchoredVertices);

  return anchoredVertices;
}

void BulletSoftSimulation::transformVerticesToWorld()
{
  transformVerticesToShapeFrame = false;
}

void BulletSoftSimulation::transformVerticesToShape()
{
  transformVerticesToShapeFrame = true;
}

}  // namespace Rcs
