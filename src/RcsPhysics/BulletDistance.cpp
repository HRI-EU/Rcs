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

#include "BulletDistance.h"
#include "BulletHelpers.h"

#include <Rcs_typedef.h>
#include <Rcs_shape.h>
#include <Rcs_math.h>
#include <Rcs_macros.h>

#include <BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h>
#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <BulletCollision/CollisionShapes/btCylinderShape.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>

#include <BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h>
#include <BulletCollision/NarrowPhaseCollision/btPointCollector.h>
#include <BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btConvexPenetrationDepthSolver.h>

#include <limits>



/*******************************************************************************
 * Create a shape
 ******************************************************************************/
static btConvexShape* createShape(const RcsShape* sh, HTr* A_SB)
{
  btConvexShape* bShape = NULL;
  HTr_setIdentity(A_SB);

  switch (sh->type)
  {

    case RCSSHAPE_POINT:
    {
      bShape = new btSphereShape(0.0);
      RCHECK(bShape);
      break;
    }

    case RCSSHAPE_SSL:
    {
      // Local transformation from body frame into collision geometry frame:
      // Consider the offset to make the ball end to the reference point
      double C_r[3], B_r[3];
      Vec3d_set(C_r, 0.0, 0.0, sh->extents[2]/2.0);
      Vec3d_transRotate(B_r, A_SB->rot, C_r);
      Vec3d_addSelf(A_SB->org, B_r);
      bShape = new btCapsuleShapeZ(btScalar(sh->extents[0]),
                                   btScalar(sh->extents[2]));
      break;
    }

    case RCSSHAPE_CONE:
    {
      bShape = new btConeShapeZ(btScalar(sh->extents[0]),
                                btScalar(sh->extents[2]));

      // Local transformation from body frame into collision geometry frame:
      // We have to consider the offset to make the base plane center to the
      // reference point.
      double C_r[3], B_r[3];
      Vec3d_set(C_r, 0.0, 0.0, 0.5*sh->extents[2]);
      Vec3d_transRotate(B_r, A_SB->rot, C_r);
      Vec3d_addSelf(A_SB->org, B_r);
      break;
    }

    case RCSSHAPE_SPHERE:
    {
      bShape = new btSphereShape(btScalar(sh->extents[0]));
      break;
    }

    case RCSSHAPE_CYLINDER:
    {
      btVector3 halfExt(btScalar(sh->extents[0]),
                        btScalar(sh->extents[0]),
                        btScalar(0.5*sh->extents[2]));
      bShape = new btCylinderShapeZ(halfExt);
      break;
    }

    case RCSSHAPE_BOX:
    {
      btVector3 halfExt(0.5*sh->extents[0], 0.5*sh->extents[1],
                        btScalar(0.5*sh->extents[2]));
      bShape = new btBoxShape(halfExt);
      break;
    }

    case RCSSHAPE_SSR:
    {
      btVector3 positions[4];
      btScalar radi[4];
      btScalar hx = sh->extents[0];
      btScalar hy = sh->extents[1];
      double r = sh->extents[2] / 2.0;

      for (int i=0; i<4; ++i)
      {
        radi[i] = r;
      }

      positions[0] = btVector3(-hx/2.0, +hy/2.0, 0.0);
      positions[1] = btVector3(+hx/2.0, +hy/2.0, 0.0);
      positions[2] = btVector3(+hx/2.0, -hy/2.0, 0.0);
      positions[3] = btVector3(-hx/2.0, -hy/2.0, 0.0);

      bShape = new btMultiSphereShape(positions, radi, 4);
      break;
    }

    case RCSSHAPE_MESH:
    {
      RcsMeshData* mesh = (RcsMeshData*) sh->userData;

      if (mesh == NULL)
      {
        RLOG(4, "Shape has no loaded mesh from file \"%s\"",
             sh->meshFile ? sh->meshFile : "NULL");
        break;
      }

      btConvexHullShape* chShape = new btConvexHullShape();
      for (unsigned int i=0; i<mesh->nVertices; ++i)
      {
        const double* vi = &mesh->vertices[i*3];
        chShape->addPoint(sh->scale*btVector3(vi[0], vi[1], vi[2]), false);
      }

      const double collisionMargin = 0.002;   // 2 mm
      chShape->recalcLocalAabb();
      chShape->setMargin(collisionMargin);
      bShape = chShape;
      break;
    }

    default:
      RLOGS(1, "Shape type \"%s\" not yet supported!", RcsShape_name(sh->type));
  }

  return bShape;
}

/*******************************************************************************
 *
 ******************************************************************************/
double RcsShape_distanceBullet(const RcsShape* s1, const RcsShape* s2,
                               const HTr* A_C1I_, const HTr* A_C2I_,
                               double cp1[3], double cp2[3], double n12[3])
{
  double distance = std::numeric_limits<double>::max();
  static btVoronoiSimplexSolver sGjkSimplexSolver;
  btGjkEpaPenetrationDepthSolver epa;

  HTr A_C1I, A_C2I, A_S1B, A_S2B;
  btConvexShape* sh1 = createShape(s1, &A_S1B);
  btConvexShape* sh2 = createShape(s2, &A_S2B);

  if ((sh1==NULL) || (sh2==NULL))
  {
    Vec3d_set(cp1, -distance, -distance, -distance);
    Vec3d_set(cp2,  distance,  distance,  distance);
    Vec3d_setUnitVector(n12, 2);
    return distance;
  }

  HTr_transform(&A_C1I, A_C1I_, &A_S1B);
  HTr_transform(&A_C2I, A_C2I_, &A_S2B);

  if ((s1->type == RCSSHAPE_BOX) ||
      (s1->type == RCSSHAPE_CYLINDER) ||
      (s1->type == RCSSHAPE_CONE))
  {
    sh1->setMargin(0.0);
  }

  if ((s2->type == RCSSHAPE_BOX) ||
      (s2->type == RCSSHAPE_CYLINDER) ||
      (s2->type == RCSSHAPE_CONE))
  {
    sh2->setMargin(0.0);
  }

  btGjkPairDetector::ClosestPointInput input;
  input.m_transformA = Rcs::btTransformFromHTr(&A_C1I);
  input.m_transformB = Rcs::btTransformFromHTr(&A_C2I);

  btGjkPairDetector convexConvex(sh1, sh2, &sGjkSimplexSolver, &epa);
  btPointCollector gjkOutput;
  convexConvex.getClosestPoints(input, gjkOutput, NULL);

  if (gjkOutput.m_hasResult)
  {
    btVector3 endPt = gjkOutput.m_pointInWorld +
                      gjkOutput.m_normalOnBInWorld*gjkOutput.m_distance;

    if (gjkOutput.m_distance < distance)
    {
      distance = gjkOutput.m_distance;

      for (int k=0; k<3; ++k)
      {
        if (cp2)
        {
          cp2[k] = gjkOutput.m_pointInWorld[k];
        }
        if (cp1)
        {
          cp1[k] = endPt[k];
        }
        if (n12)
        {
          n12[k] = -gjkOutput.m_normalOnBInWorld[k];
        }
      }
    }

  }

  delete sh1;
  delete sh2;

  return distance;
}

/*******************************************************************************
 * Not a nice function. It essentially approximates a torus with a set of
 * radially aligned capsules, and computes the distance to the closest of
 * these.
 ******************************************************************************/
static double RcsShape_distanceTorusShape(const RcsShape* torus,
                                          const RcsShape* shape,
                                          const HTr* A_TI, const HTr* A_SI,
                                          double cp1[3], double cp2[3],
                                          double n12[3])
{
  double distance = std::numeric_limits<double>::max();
  double dist_i = distance;
  const int nSlices = 16;
  MatNd* points = MatNd_create(nSlices, 3);

  for (int i=0; i<nSlices; ++i)
  {
    double angle = 2.0*M_PI*i/nSlices;
    double* point_i = MatNd_getRowPtr(points, i);
    point_i[0] = torus->extents[0]*sin(angle);
    point_i[1] = torus->extents[0]*cos(angle);
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

    HTr A_CapI;
    HTr_transform(&A_CapI, A_TI, &A);

    RcsShape capsule;
    memset(&capsule, 0, sizeof(RcsShape));
    HTr_setIdentity(&capsule.A_CB);
    capsule.scale = 1.0;
    capsule.computeType = RCSSHAPE_COMPUTE_DISTANCE;
    capsule.type = RCSSHAPE_SSL;
    capsule.extents[0] = 0.5*torus->extents[2];
    capsule.extents[1] = 0.0;
    capsule.extents[2] = h;

    double tmpCp1[3], tmpCp2[3], tmpN12[3];
    dist_i = RcsShape_distanceBullet(&capsule, shape, &A_CapI, A_SI,
                                     tmpCp1, tmpCp2, tmpN12);

    if (dist_i<distance)
    {
      distance = dist_i;
      Vec3d_copy(cp1, tmpCp1);
      Vec3d_copy(cp2, tmpCp2);
      Vec3d_copy(n12, tmpN12);
    }
  }

  MatNd_destroy(points);

  return distance;
}

/*******************************************************************************
 *
 ******************************************************************************/
static double RcsShape_distanceShapeTorus(const RcsShape* shape,
                                          const RcsShape* torus,
                                          const HTr* A_SI, const HTr* A_TI,
                                          double cpS[3], double cpT[3],
                                          double nST[3])
{
  double d = RcsShape_distanceTorusShape(torus, shape, A_TI, A_SI,
                                         cpT, cpS, nST);
  Vec3d_constMulSelf(nST, -1.0);
  return d;
}

/*******************************************************************************
 * Add Bullet distance functions to the array of distance functions
 ******************************************************************************/
static bool setBulletDistanceFunctions()
{
  bool success = true;

  // SSL
  success = RcsShape_setDistanceFunction(RCSSHAPE_SSL, RCSSHAPE_CONE,
                                         RcsShape_distanceBullet) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_SSL, RCSSHAPE_MESH,
                                         RcsShape_distanceBullet) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_SSL, RCSSHAPE_CYLINDER,
                                         RcsShape_distanceBullet) && success;

  // SSR
  success = RcsShape_setDistanceFunction(RCSSHAPE_SSR, RCSSHAPE_SSL,
                                         RcsShape_distanceBullet) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_SSR, RCSSHAPE_BOX,
                                         RcsShape_distanceBullet) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_SSR, RCSSHAPE_MESH,
                                         RcsShape_distanceBullet) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_SSR, RCSSHAPE_CYLINDER,
                                         RcsShape_distanceBullet) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_SSR, RCSSHAPE_SPHERE,
                                         RcsShape_distanceBullet) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_SSR, RCSSHAPE_CONE,
                                         RcsShape_distanceBullet) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_SSR, RCSSHAPE_TORUS,
                                         RcsShape_distanceShapeTorus) && success;

  // MESH
  success = RcsShape_setDistanceFunction(RCSSHAPE_MESH, RCSSHAPE_SSL,
                                         RcsShape_distanceBullet) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_MESH, RCSSHAPE_SSR,
                                         RcsShape_distanceBullet) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_MESH, RCSSHAPE_MESH,
                                         RcsShape_distanceBullet) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_MESH, RCSSHAPE_BOX,
                                         RcsShape_distanceBullet) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_MESH, RCSSHAPE_CYLINDER,
                                         RcsShape_distanceBullet) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_MESH, RCSSHAPE_SPHERE,
                                         RcsShape_distanceBullet) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_MESH, RCSSHAPE_CONE,
                                         RcsShape_distanceBullet) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_MESH, RCSSHAPE_TORUS,
                                         RcsShape_distanceShapeTorus) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_MESH, RCSSHAPE_POINT,
                                         RcsShape_distanceBullet) && success;

  // BOX
  success = RcsShape_setDistanceFunction(RCSSHAPE_BOX, RCSSHAPE_SSR,
                                         RcsShape_distanceBullet) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_BOX, RCSSHAPE_MESH,
                                         RcsShape_distanceBullet) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_BOX, RCSSHAPE_CYLINDER,
                                         RcsShape_distanceBullet) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_BOX, RCSSHAPE_BOX,
                                         RcsShape_distanceBullet) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_BOX, RCSSHAPE_CONE,
                                         RcsShape_distanceBullet) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_BOX, RCSSHAPE_TORUS,
                                         RcsShape_distanceShapeTorus) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_BOX, RCSSHAPE_POINT,
                                         RcsShape_distanceBullet) && success;

  // CYLINDER
  success = RcsShape_setDistanceFunction(RCSSHAPE_CYLINDER, RCSSHAPE_MESH,
                                         RcsShape_distanceBullet) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_CYLINDER, RCSSHAPE_SSR,
                                         RcsShape_distanceBullet) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_CYLINDER, RCSSHAPE_MESH,
                                         RcsShape_distanceBullet) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_CYLINDER, RCSSHAPE_BOX,
                                         RcsShape_distanceBullet) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_CYLINDER, RCSSHAPE_CYLINDER,
                                         RcsShape_distanceBullet) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_CYLINDER, RCSSHAPE_CONE,
                                         RcsShape_distanceBullet) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_CYLINDER, RCSSHAPE_TORUS,
                                         RcsShape_distanceBullet) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_CYLINDER, RCSSHAPE_SSL,
                                         RcsShape_distanceBullet) && success;

  // SPHERE
  success = RcsShape_setDistanceFunction(RCSSHAPE_SPHERE, RCSSHAPE_SSR,
                                         RcsShape_distanceBullet) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_SPHERE, RCSSHAPE_MESH,
                                         RcsShape_distanceBullet) && success;

  // CONE
  success = RcsShape_setDistanceFunction(RCSSHAPE_CONE, RCSSHAPE_SSL,
                                         RcsShape_distanceBullet) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_CONE, RCSSHAPE_SSR,
                                         RcsShape_distanceBullet) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_CONE, RCSSHAPE_MESH,
                                         RcsShape_distanceBullet) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_CONE, RCSSHAPE_BOX,
                                         RcsShape_distanceBullet) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_CONE, RCSSHAPE_CYLINDER,
                                         RcsShape_distanceBullet) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_CONE, RCSSHAPE_CONE,
                                         RcsShape_distanceBullet) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_CONE, RCSSHAPE_TORUS,
                                         RcsShape_distanceBullet) && success;

  // TORUS
  success = RcsShape_setDistanceFunction(RCSSHAPE_TORUS, RCSSHAPE_SSR,
                                         RcsShape_distanceTorusShape) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_TORUS, RCSSHAPE_MESH,
                                         RcsShape_distanceTorusShape) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_TORUS, RCSSHAPE_BOX,
                                         RcsShape_distanceTorusShape) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_TORUS, RCSSHAPE_CYLINDER,
                                         RcsShape_distanceTorusShape) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_TORUS, RCSSHAPE_CONE,
                                         RcsShape_distanceTorusShape) && success;

  // POINT
  success = RcsShape_setDistanceFunction(RCSSHAPE_POINT, RCSSHAPE_MESH,
                                         RcsShape_distanceBullet) && success;
  success = RcsShape_setDistanceFunction(RCSSHAPE_POINT, RCSSHAPE_BOX,
                                         RcsShape_distanceBullet) && success;


  RLOG(5, "%s Bullet distance functions",
       success ? "SUCCESFULLY added" : "FAILED to add");

  return success;
}



// This is being called before main()
static bool distanceInitialized = setBulletDistanceFunctions();

