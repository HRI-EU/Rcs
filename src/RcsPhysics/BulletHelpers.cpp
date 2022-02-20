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

#include "BulletHelpers.h"
#include <Rcs_macros.h>

#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include <LinearMath/btGeometryUtil.h>


/*******************************************************************************
 * Transformation conversion to bullet transform. Bullet represents rotation
 * matrices in column-major form.
 ******************************************************************************/
btTransform Rcs::btTransformFromHTr(const HTr* A_KI)
{
  btMatrix3x3 rm((btScalar)A_KI->rot[0][0],
                 (btScalar)A_KI->rot[1][0],
                 (btScalar)A_KI->rot[2][0],
                 (btScalar)A_KI->rot[0][1],
                 (btScalar)A_KI->rot[1][1],
                 (btScalar)A_KI->rot[2][1],
                 (btScalar)A_KI->rot[0][2],
                 (btScalar)A_KI->rot[1][2],
                 (btScalar)A_KI->rot[2][2]);

  btTransform transform;
  transform.setOrigin(btVector3((btScalar)A_KI->org[0],
                                (btScalar)A_KI->org[1],
                                (btScalar)A_KI->org[2]));
  transform.setBasis(rm);

  return transform;
}

/*******************************************************************************
 * Transformation conversion from bullet transform. Bullet represents rotation
 * matrices in column-major form.
 ******************************************************************************/
void Rcs::HTrFromBtTransform(HTr* A_KI, const btTransform& trf)
{
  btMatrix3x3 rm = trf.getBasis();
  btVector3 org = trf.getOrigin();

  A_KI->org[0] = org[0];
  A_KI->org[1] = org[1];
  A_KI->org[2] = org[2];

  A_KI->rot[0][0] = rm[0][0];
  A_KI->rot[0][1] = rm[0][1];
  A_KI->rot[0][2] = rm[0][2];
  A_KI->rot[1][0] = rm[1][0];
  A_KI->rot[1][1] = rm[1][1];
  A_KI->rot[1][2] = rm[1][2];
  A_KI->rot[2][0] = rm[2][0];
  A_KI->rot[2][1] = rm[2][1];
  A_KI->rot[2][2] = rm[2][2];
}

/*******************************************************************************
 * Transformation print function.
 ******************************************************************************/
void Rcs::printTransform(const btTransform& trf)
{
  HTr T;
  HTrFromBtTransform(&T, trf);
  HTr_fprint(stderr, &T);
}

/*******************************************************************************
 * Transformation print function.
 ******************************************************************************/
void Rcs::printTransform(const char* comment, const btTransform& trf)
{
  HTr T;
  HTrFromBtTransform(&T, trf);
  printf("%s\n", comment);
  HTr_fprint(stdout, &T);
}

/*******************************************************************************
 * Convert mesh to convex hull.
 ******************************************************************************/
btConvexHullShape* Rcs::meshToHull(const RcsMeshData* mesh,
                                   double collisionMargin)
{
  if (mesh == NULL)
  {
    RLOG(4, "Input mesh is NULL");
    return NULL;
  }

  btConvexHullShape* chShape = new btConvexHullShape();

  for (unsigned int i=0; i<mesh->nVertices; ++i)
  {
    const double* vi = &mesh->vertices[i*3];
    chShape->addPoint(btVector3((btScalar)vi[0],
                                (btScalar)vi[1],
                                (btScalar)vi[2]), false);
  }

  chShape->recalcLocalAabb();
  chShape->setMargin((btScalar)collisionMargin);

  return chShape;
}

/*******************************************************************************
 * Convert convex hull to mesh
 ******************************************************************************/
RcsMeshData* Rcs::hullToMesh(const btConvexHullShape* convHull)
{
  int nPts = convHull->getNumPoints();
  const btVector3* vertices = convHull->getUnscaledPoints();

  double* v = RNALLOC(3*nPts, double);
  for (int i=0; i<nPts; ++i)
  {
    v[3*i+0] = vertices[i].getX();
    v[3*i+1] = vertices[i].getY();
    v[3*i+2] = vertices[i].getZ();
  }

  RcsMeshData* hullMesh = RcsMesh_fromVertices(v, nPts);
  RFREE(v);

  if (hullMesh==NULL)
  {
    RLOG(1, "Failed to compute Delaunay triangulation for mesh with %d points",
         nPts);
  }
  else
  {
    RLOG(5, "Succeeded to compute Delaunay triangulation for mesh: %d vertices",
         hullMesh->nVertices);
    int nDuplicates = RcsMesh_compressVertices(hullMesh, 1.0e-8);
    bool success = nDuplicates > 0 ? true : false;
    RLOG(5, "%s to compress mesh: now %d vertices",
         success ? "Succeeded" : "Failed", hullMesh->nVertices);
  }

  return hullMesh;
}

/*******************************************************************************
 * Mesh compression to Bullet convex hull
 ******************************************************************************/
void Rcs::convertMesh(const char* to, const char* from)
{
  RcsMeshData* src = RcsMesh_createFromFile(from);
  btConvexHullShape* hull = meshToCompressedHull(src);
  RcsMeshData* dst = hullToMesh(hull);
  RcsMesh_toFile(dst, to);
}


/*******************************************************************************
 * Convert mesh to convex hull.
 ******************************************************************************/
btConvexHullShape* Rcs::meshToCompressedHull(const RcsMeshData* mesh,
                                             double collisionMargin)
{
  if (mesh == NULL)
  {
    RLOG(4, "Input mesh is NULL");
    return NULL;
  }

  btConvexHullShape* chShape = new btConvexHullShape();

  for (unsigned int i=0; i<mesh->nVertices; ++i)
  {
    const double* vi = &mesh->vertices[i*3];
    chShape->addPoint(btVector3((btScalar)vi[0],
                                (btScalar)vi[1],
                                (btScalar)vi[2]), false);
  }

  chShape->recalcLocalAabb();
  chShape->setMargin((btScalar)collisionMargin);

  // Simplify hull (without shrinking the margin)
  btShapeHull* hull = new btShapeHull(chShape);
  btScalar margin = chShape->getMargin();
  hull->buildHull(margin);   // margin is ignored in buildMargin()

  btConvexHullShape* simplifiedHull = new btConvexHullShape();
  simplifiedHull->setMargin((btScalar)collisionMargin);

  for (int i=0; i<hull->numVertices(); ++i)
  {
    bool recalcAABB = (i==hull->numVertices()-1) ? true : false;
    simplifiedHull->addPoint(hull->getVertexPointer()[i], recalcAABB);
  }

  delete chShape;
  delete hull;

#if 0       // Shrinking the margin
  btConvexHullShape* shrinkedShape = new btConvexHullShape();
  shrinkedShape->setMargin(collisionMargin);

  for (int i=0; i<simplifiedHull->getNumVertices(); ++i)
  {
    btVector3 vtx, planeNormal, planeSupport;
    simplifiedHull->getVertex(i, vtx);
    simplifiedHull->getPlane(planeNormal, planeSupport, i);

    RLOG(0, "%d planeSupport: %f %f %f   Vertex: %f %f %f", simplifiedHull->getNumPlanes(), planeNormal[0], planeNormal[1], planeNormal[2], vtx[0], vtx[1], vtx[2]);

    bool recalcAABB = (i==simplifiedHull->getNumVertices()-1) ? true : false;
    shrinkedShape->addPoint(vtx - collisionMargin*planeNormal, recalcAABB);
  }
  bShape = shrinkedShape;
#endif

  RLOG(5, "Compressed hull has %u vertices", simplifiedHull->getNumVertices());

  return simplifiedHull;
}
