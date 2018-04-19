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

#include "VortexHelpers.h"

#include <Rcs_math.h>
#include <Rcs_typedef.h>
#include <Rcs_utils.h>
#include <Rcs_macros.h>
#include <Rcs_body.h>
#include <Rcs_joint.h>
#include <Rcs_shape.h>
#include <Rcs_mesh.h>

#include <Vx/VxVersion.h>
#include <Vx/VxRPRO.h>
#include <Vx/VxHinge.h>
#include <Vx/VxPrismatic.h>
#include <Vx/VxPlane.h>
#include <Vx/VxCylinder.h>
#include <Vx/VxCapsule.h>
#include <Vx/VxBox.h>
#include <Vx/VxSphere.h>
#include <Vx/VxConvexMesh.h>
#include <Vx/VxCollisionGeometry.h>
#include <Vx/VxCompositeCollisionGeometry.h>
#include <Vx/VxUniverse.h>
#include <Vx/VxFrame.h>
#include <Vx/VxRigidBodyResponseModel.h>

#include <iostream>
#include <cmath>



/*******************************************************************************
 * Helper class for rigid bodies
 ******************************************************************************/
Rcs::VortexBody::VortexBody(const RcsBody* body_) : Vx::VxPart(1.0), body(body_)
{
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::VortexBody::~VortexBody()
{
}

/*******************************************************************************
 *
 ******************************************************************************/
Vx::VxMaterialTable* Rcs::getMaterialTable()
{
  Vx::VxUniverse* universe = &Vx::VxFrame::instance()->getUniverse();
  return universe->getRigidBodyResponseModel()->getMaterialTable();
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::VortexBody* Rcs::getPartPtr(const RcsBody* body)
{
  return body ? static_cast<Rcs::VortexBody*>(body->actor) : NULL;
}

/******************************************************************************
 * Returns a VxTransform from a HTr data structure.
 ******************************************************************************/
Vx::VxTransform Rcs::VxTransform_fromHTr(const HTr* A_BI)
{
  Vx::VxReal44 tm44;

  // Rotation matrix
  tm44[0][0] = A_BI->rot[0][0];
  tm44[0][1] = A_BI->rot[0][1];
  tm44[0][2] = A_BI->rot[0][2];

  tm44[1][0] = A_BI->rot[1][0];
  tm44[1][1] = A_BI->rot[1][1];
  tm44[1][2] = A_BI->rot[1][2];

  tm44[2][0] = A_BI->rot[2][0];
  tm44[2][1] = A_BI->rot[2][1];
  tm44[2][2] = A_BI->rot[2][2];

  // Origin vector
  tm44[3][0] = A_BI->org[0];
  tm44[3][1] = A_BI->org[1];
  tm44[3][2] = A_BI->org[2];

  // Scaling
  tm44[0][3] = 0.0;
  tm44[1][3] = 0.0;
  tm44[2][3] = 0.0;
  tm44[3][3] = 1.0;

  Vx::VxTransform tm(tm44);

  return tm;
}

/*******************************************************************************
 * Converts a VxTransform into a HTr data structure.
 ******************************************************************************/
void Rcs::HTr_fromVxTransform(HTr* A_BI, const Vx::VxTransform tm)
{
  Vx::VxReal44 tm44;

  tm.get(tm44);

  // Rotation matrix
  A_BI->rot[0][0] = tm44[0][0];
  A_BI->rot[0][1] = tm44[0][1];
  A_BI->rot[0][2] = tm44[0][2];

  A_BI->rot[1][0] = tm44[1][0];
  A_BI->rot[1][1] = tm44[1][1];
  A_BI->rot[1][2] = tm44[1][2];

  A_BI->rot[2][0] = tm44[2][0];
  A_BI->rot[2][1] = tm44[2][1];
  A_BI->rot[2][2] = tm44[2][2];

  // Origin vector
  A_BI->org[0] = tm44[3][0];
  A_BI->org[1] = tm44[3][1];
  A_BI->org[2] = tm44[3][2];
}

/*******************************************************************************
 * \todo: What does A_BI do here?
 ******************************************************************************/
Vx::VxCollisionGeometry* Rcs::createSphere(const RcsShape* sh,
                                           const HTr* A_BI,
                                           Vx::VxMaterial* material)
{
  // Local transformation from body frame into collision geometry frame
  Vx::VxSphere* sphere = new Vx::VxSphere(sh->extents[0]);
  Vx::VxCollisionGeometry* cg =
    new Vx::VxCollisionGeometry(sphere, material,
                                VxTransform_fromHTr(&sh->A_CB));

  return cg;
}

/*******************************************************************************
 * Adds a capsule to a VxPart. If the VxPart object doesn't exist, it will be
 * created.
 * \todo: What does A_BI do here?
 ******************************************************************************/
Vx::VxCollisionGeometry* Rcs::createCapsule(const RcsShape* sh,
                                            const HTr* A_BI,
                                            Vx::VxMaterial* material)
{
  Vx::VxReal h = sh->extents[2];
  Vx::VxReal r = sh->extents[0];

  // Local transformation from body frame into collision geometry frame. We
  // have to consider the offset to make ball end to the reference point
  HTr A_CB;
  HTr_copy(&A_CB, &sh->A_CB);

  double C_r[3], B_r[3];
  Vec3d_set(C_r, 0.0, 0.0, h / 2.0);
  Vec3d_transRotate(B_r, A_CB.rot, C_r);
  Vec3d_addSelf(A_CB.org, B_r);

  Vx::VxCapsule* capsule = new Vx::VxCapsule(r, h);
  Vx::VxCollisionGeometry* cg =
    new Vx::VxCollisionGeometry(capsule, material, VxTransform_fromHTr(&A_CB));

  return cg;
}

/*******************************************************************************
 * \todo: What does A_BI do here?
 ******************************************************************************/
Vx::VxCollisionGeometry* Rcs::createCylinder(const RcsShape* sh,
                                             const HTr* A_BI,
                                             Vx::VxMaterial* material)
{
  Vx::VxReal h = sh->extents[2];
  Vx::VxReal r = sh->extents[0];

  // Local transformation from body frame into collision geometry frame
  Vx::VxCylinder* cylinder = new Vx::VxCylinder(r, h);
  Vx::VxCollisionGeometry* cg =
    new Vx::VxCollisionGeometry(cylinder, material,
                                VxTransform_fromHTr(&sh->A_CB));

  return cg;
}

/*******************************************************************************
 * Adds a sphere swept rectangle to a VxPart. If the VxPart object doesn't
 * exist, it will be created.
 ******************************************************************************/
Vx::VxCollisionGeometry* Rcs::createSSR(const RcsShape* sh,
                                        const HTr* A_KI,
                                        Vx::VxMaterial* material)
{
  Vx::VxCompositeCollisionGeometry* composite =
    new Vx::VxCompositeCollisionGeometry();

  // Local transformation from body frame into collision geometry frame
  HTr A_CB;
  double r = sh->extents[2] / 2.0;
  Vx::VxReal hx = sh->extents[0];
  Vx::VxReal hy = sh->extents[1];
  HTr_copy(&A_CB, &sh->A_CB);

  Vx::VxBox* ssr = new Vx::VxBox(sh->extents);
  Vx::VxCollisionGeometry* cg =
    new Vx::VxCollisionGeometry(ssr, material, VxTransform_fromHTr(&A_CB));
  composite->addCollisionGeometry(cg);

  // Bounding SSL's
  double k_offset[3];
  HTr A_capC, A_capB; // A_capB = A_cap_C * A_CB

  // First edge SSL
  Mat3d_setRotMatY(A_capC.rot, M_PI_2);
  HTr_copy(&A_capB, &A_CB);
  Mat3d_mul(A_capB.rot, A_capC.rot, A_CB.rot);
  Vec3d_set(k_offset, 0.0, hy / 2.0, 0.0);
  Vec3d_transRotateSelf(k_offset, A_CB.rot);
  Vec3d_addSelf(A_capB.org, k_offset);
  Vx::VxCapsule* capsule = new Vx::VxCapsule(r, hx);
  cg = new Vx::VxCollisionGeometry(capsule, material,
                                   VxTransform_fromHTr(&A_capB));
  composite->addCollisionGeometry(cg);

  // Second edge SSL
  Mat3d_setRotMatY(A_capC.rot, M_PI_2);
  HTr_copy(&A_capB, &A_CB);
  Mat3d_mul(A_capB.rot, A_capC.rot, A_CB.rot);
  Vec3d_set(k_offset, 0.0, -hy / 2.0, 0.0);
  Vec3d_transRotateSelf(k_offset, A_CB.rot);
  Vec3d_addSelf(A_capB.org, k_offset);
  capsule = new Vx::VxCapsule(r, hx);
  cg = new Vx::VxCollisionGeometry(capsule, material,
                                   VxTransform_fromHTr(&A_capB));
  composite->addCollisionGeometry(cg);

  // Third edge SSL
  Mat3d_setRotMatX(A_capC.rot, M_PI_2);
  HTr_copy(&A_capB, &A_CB);
  Mat3d_mul(A_capB.rot, A_capC.rot, A_CB.rot);
  Vec3d_set(k_offset, -hx / 2.0, 0.0, 0.0);
  Vec3d_transRotateSelf(k_offset, A_CB.rot);
  Vec3d_addSelf(A_capB.org, k_offset);
  capsule = new Vx::VxCapsule(r, hy);
  cg = new Vx::VxCollisionGeometry(capsule, material,
                                   VxTransform_fromHTr(&A_capB));
  composite->addCollisionGeometry(cg);

  // Fourth edge SSL
  Mat3d_setRotMatX(A_capC.rot, M_PI_2);
  HTr_copy(&A_capB, &A_CB);
  Mat3d_mul(A_capB.rot, A_capC.rot, A_CB.rot);
  Vec3d_set(k_offset, hx / 2.0, 0.0, 0.0);
  Vec3d_transRotateSelf(k_offset, A_CB.rot);
  Vec3d_addSelf(A_capB.org, k_offset);
  capsule = new Vx::VxCapsule(r, hy);
  cg = new Vx::VxCollisionGeometry(capsule, material,
                                   VxTransform_fromHTr(&A_capB));
  composite->addCollisionGeometry(cg);

  return composite;
}

/*******************************************************************************
 * Adds a box to a VxPart. If the VxPart object doesn't exist, it will
 * be created.
 ******************************************************************************/
Vx::VxCollisionGeometry* Rcs::createBox(const RcsShape* sh, const HTr* A_KI,
                                        Vx::VxMaterial* material)
{
  // Local transformation from body frame into collision geometry frame
  Vx::VxBox* box = new Vx::VxBox(sh->extents);
  Vx::VxCollisionGeometry* cg =
    new Vx::VxCollisionGeometry(box, material, VxTransform_fromHTr(&sh->A_CB));

  return cg;
}

/*******************************************************************************
 * Adds a cone to a VxPart. If the VxPart object doesn't exist, it
 * will be created.
 ******************************************************************************/
Vx::VxCollisionGeometry* Rcs::createCone(const RcsShape* sh, const HTr* A_KI,
                                         Vx::VxMaterial* material)
{
  // Local transformation from body frame into collision geometry frame
  HTr A_CB;
  HTr_copy(&A_CB, &sh->A_CB);
  Vx::VxReal h = sh->extents[2];
  Vx::VxReal r = sh->extents[0];

  // approximate cone by vertices, calculate number of segments based on radius
  // COM is at h/4
  int segm = (int)std::max(floor(r*300.+.5),6.);
  Vx::VxReal3* vertices = new Vx::VxReal3[segm+1];
  // base circle
  for (int i = 0; i < segm; i++)
  {
    vertices[i][0] = r*sin(2.*M_PI*(double)i/(double)segm);
    vertices[i][1] = r*cos(2.*M_PI*(double)i/(double)segm);
    vertices[i][2] = 0.0;//-h/4.;
  }
  // tip
  vertices[segm][0] = 0.;
  vertices[segm][1] = 0.;
  vertices[segm][2] = h;//3./4.*h;

  Vx::VxConvexMesh* convex = new Vx::VxConvexMesh(vertices, segm+1);
  Vx::VxCollisionGeometry* cg = new Vx::VxCollisionGeometry(convex, material, VxTransform_fromHTr(&sh->A_CB));

  delete[] vertices;

  return cg;
}

/*******************************************************************************
 * Adds a cone to a VxPart. If the VxPart object doesn't exist, it
 *        will be created.
 ******************************************************************************/
void Rcs::addTorus(const RcsShape* sh, const HTr* A_KI, Vx::VxPart** p,
                   Vx::VxMaterial* material)
{
  // If part doesn't exist, create it and assign the bodies transformation
  if (*p == NULL)
  {
    *p = new Vx::VxPart(1.0);
  }

  if (*p == NULL)
  {
    RLOG(1, "Memory allocation error!");
    return;
  }

  (*p)->addCollisionGeometry(createTorus(sh, A_KI, material));
}

/*******************************************************************************
 *
 ******************************************************************************/
Vx::VxCollisionGeometry* Rcs::createTorus(const RcsShape* sh,
                                          const HTr* A_KI,
                                          Vx::VxMaterial* material)
{
  const int nSlices = 16;
  MatNd* points = MatNd_create(nSlices, 3);
  Vx::VxCompositeCollisionGeometry* composite =
    new Vx::VxCompositeCollisionGeometry();

  for (int i=0; i<nSlices; ++i)
  {
    double angle = i*2.0*M_PI/nSlices;
    double* point_i = MatNd_getRowPtr(points, i);
    point_i[0] = sh->extents[2]*sin(angle);
    point_i[1] = sh->extents[2]*cos(angle);
    Vec3d_transformSelf(point_i, &sh->A_CB);
  }

  for (int i=1; i<=nSlices; ++i)
  {
    HTr A;
    double* pPrev = MatNd_getRowPtr(points, i-1);
    double* pCurr = MatNd_getRowPtr(points, i<nSlices?i:0);
    double h = Vec3d_distance(pCurr, pPrev);
    HTr_from2Points(&A, pPrev, pCurr);

    Vx::VxCapsule* capsule = new Vx::VxCapsule(h, sh->extents[0]);
    Vx::VxCollisionGeometry* cg =
      new Vx::VxCollisionGeometry(capsule, material, VxTransform_fromHTr(&A));
    composite->addCollisionGeometry(cg);
  }

  MatNd_destroy(points);

  return composite;
}

/*******************************************************************************
 * Creates a tri-mesh from a tri-file.
 ******************************************************************************/
static Vx::VxTriangleMeshBVTree* createMeshFromTriFile(const char* meshFile,
                                                       const double scaling)
{
  // Check for NULL pointer argument
  if (meshFile == NULL)
  {
    RLOGS(1, "Mesh file is NULL - skipping triangle mesh");
    return NULL;
  }

  // Read only files with suffix .tri
  if (String_hasEnding(meshFile, ".tri", false) == false)
  {
    RLOGS(1, "Mesh file has not ending \".tri\" - skipping");
    return NULL;
  }

  std::ifstream is;
  is.open(meshFile);

  // Return if file can't be read
  if (!is.good())
  {
    RLOGS(1, "Couldn't open mesh file \"%s\"", meshFile);
    return NULL;
  }


  char triString[32];
  unsigned int i, nFaces = 0, nVertices = 0;
  is >> triString >> nVertices >> nFaces;

  NLOG(0, "Found tag \"%s\" and %d vertices and %d triangles",
       triString, nVertices, nFaces);

  double* vertices = new double[3*nVertices];
  unsigned int* faces = new unsigned int[3*nFaces];

  for (i=0; i<3*nVertices; i++)
  {
    is >> vertices[i];
  }

  for (i=0; i<3*nFaces; i++)
  {
    is >> faces[i];
  }

  is.close();

  // Set mesh from vertices and faces
  Vx::VxTriangleMeshBVTree* triMesh = new Vx::VxTriangleMeshBVTree(nFaces);

  for (i=0; i<nFaces; i++)
  {
    unsigned int vertexIdx0 = faces[3*i];
    unsigned int vertexIdx1 = faces[3*i+1];
    unsigned int vertexIdx2 = faces[3*i+2];

    RCHECK_MSG(vertexIdx0<nVertices, "%d   %d", vertexIdx0, nVertices);
    RCHECK_MSG(vertexIdx1<nVertices, "%d   %d", vertexIdx1, nVertices);
    RCHECK_MSG(vertexIdx2<nVertices, "%d   %d", vertexIdx2, nVertices);

    Vx::VxReal3 v0 = { scaling* vertices[3*vertexIdx0],
                       scaling* vertices[3*vertexIdx0+1],
                       scaling* vertices[3*vertexIdx0+2]
                     };

    Vx::VxReal3 v1 = { scaling* vertices[3*vertexIdx1],
                       scaling* vertices[3*vertexIdx1+1],
                       scaling* vertices[3*vertexIdx1+2]
                     };

    Vx::VxReal3 v2 = { scaling* vertices[3*vertexIdx2],
                       scaling* vertices[3*vertexIdx2+1],
                       scaling* vertices[3*vertexIdx2+2]
                     };

    triMesh->addTriangleByVertexCopy(v0, v1, v2);
  }

  delete[] vertices;
  delete[] faces;

  return triMesh;
}

/*******************************************************************************
 * Creates a tri-mesh from a RcsMeshData structure.
 ******************************************************************************/
static Vx::VxTriangleMeshBVTree* createMeshFromData(const RcsMeshData* mesh)
{
  // Check for NULL pointer argument
  if (mesh == NULL)
  {
    RLOGS(1, "Mesh is NULL - skipping triangle mesh");
    return NULL;
  }

  // Set mesh from vertices and faces
  Vx::VxTriangleMeshBVTree* tri = new Vx::VxTriangleMeshBVTree(mesh->nFaces);

  for (unsigned int i=0; i<mesh->nFaces; i++)
  {
    unsigned int vertexIdx0 = mesh->faces[3*i];
    unsigned int vertexIdx1 = mesh->faces[3*i+1];
    unsigned int vertexIdx2 = mesh->faces[3*i+2];

    RCHECK_MSG(vertexIdx0<mesh->nVertices, "%d   %d",
               vertexIdx0, mesh->nVertices);
    RCHECK_MSG(vertexIdx1<mesh->nVertices, "%d   %d",
               vertexIdx1, mesh->nVertices);
    RCHECK_MSG(vertexIdx2<mesh->nVertices, "%d   %d",
               vertexIdx2, mesh->nVertices);

    Vx::VxReal3 v0 = { mesh->vertices[3*vertexIdx0],
                       mesh->vertices[3*vertexIdx0+1],
                       mesh->vertices[3*vertexIdx0+2]
                     };

    Vx::VxReal3 v1 = { mesh->vertices[3*vertexIdx1],
                       mesh->vertices[3*vertexIdx1+1],
                       mesh->vertices[3*vertexIdx1+2]
                     };

    Vx::VxReal3 v2 = { mesh->vertices[3*vertexIdx2],
                       mesh->vertices[3*vertexIdx2+1],
                       mesh->vertices[3*vertexIdx2+2]
                     };

    tri->addTriangleByVertexCopy(v0, v1, v2);
  }

  return tri;
}

/*******************************************************************************
 *
 ******************************************************************************/
Vx::VxCollisionGeometry* Rcs::createMesh(const RcsShape* sh, const HTr* A_KI,
                                         Vx::VxMaterial* material)
{
  Vx::VxCollisionGeometry* cg = NULL;

  if (sh == NULL)
  {
    RLOG(0, "Failed to create physics mesh - shape is NULL");
    return cg;
  }

  if (sh->userData != NULL)
  {
    const RcsMeshData* mesh = (const RcsMeshData*) sh->userData;
    Vx::VxTriangleMeshBVTree* triMesh = createMeshFromData(mesh);

    if (triMesh==NULL)
    {
      RLOG(0, "Failed to create physics mesh from shape user data");
      return NULL;
    }

    cg = new Vx::VxCollisionGeometry(triMesh, material,
                                     VxTransform_fromHTr(&sh->A_CB));

    return cg;
  }

  if (sh->meshFile == NULL)
  {
    RLOG(0, "Failed to create physics mesh - mesh file is NULL");
    return cg;
  }

  if (String_hasEnding(sh->meshFile, ".tri", false) == false)
  {
    const bool merged = true;
    const double mergedEpsilon = 1e-5;
    const bool flipNormals = false;
    Vx::VxTriangleMeshBVTree* triMesh =
      new Vx::VxTriangleMeshBVTree(sh->meshFile, sh->scale, merged,
                                   mergedEpsilon, flipNormals);
    RCHECK_MSG(triMesh, "Trimesh failed from \"%s\"", sh->meshFile);

    cg = new Vx::VxCollisionGeometry(triMesh, material,
                                     VxTransform_fromHTr(&sh->A_CB));
  }
  else
  {
    Vx::VxTriangleMeshBVTree* triMesh =
      createMeshFromTriFile(sh->meshFile, sh->scale);
    RCHECK(triMesh);
    cg = new Vx::VxCollisionGeometry(triMesh, material,
                                     VxTransform_fromHTr(&sh->A_CB));
  }

  return cg;
}

/*******************************************************************************
 * Creates a fixed joint (e.g. for load cells) between the parent body
 * and its child. If the joint cannot be created, the function returns
 * NULL and gets verbose on debug level 1. The function fails if:
 *
 * - One of the bodies is NULL
 * - One of the bodies has no VxPart attached to the bodies's void pointer
 * - If more than one load cell sensor is attached to the body b1
 ******************************************************************************/
Vx::VxConstraint* Rcs::createFixedJoint(Vx::VxPart* vxParent,
                                        Vx::VxPart* vxChild,
                                        const RcsGraph* graph)
{
  if ((vxParent==NULL) || (vxChild==NULL))
  {
    RLOG(1, "Create fixed joint between \"%s\" and \"%s\" failed",
         vxParent ? vxParent->getName() : "NULL",
         vxChild ? vxChild->getName() : "NULL");
    return NULL;
  }

  const RcsBody* parent = (const RcsBody*) vxParent->getUserDataPtr();
  const RcsBody* child = (const RcsBody*) vxChild->getUserDataPtr();

  if (child == NULL)
  {
    RLOG(1, "Creation of joint between \"%s\" and \"%s\" failed: "
         "no body attached to \"%s\"",
         vxParent->getName(), vxChild->getName() , vxChild->getName());
    return NULL;
  }

  HTr A_childI;
  HTr A_parentI;
  HTr_copy(&A_childI, child->A_BI);
  HTr_copy(&A_parentI, parent->A_BI);

  // We search through the sensors and look for load cells attached to
  // body child. If more than one load cell is found, we exit fatally.
  RcsSensor* loadCell = NULL;

  if (graph != NULL)
  {

    RCSGRAPH_TRAVERSE_SENSORS(graph)
    {

      if ((SENSOR->type==RCSSENSOR_LOAD_CELL) && (SENSOR->body==child))
      {
        RCHECK_MSG(loadCell==NULL, "Body \"%s\" has more than 1 load cells "
                   "attached", child->name);

        loadCell = SENSOR;

        // If a load cell exists, we consider it's offset
        // transformation for the fixed joint position and orientation.
        HTr_transform(&A_childI, child->A_BI, SENSOR->offset);
        HTr_transform(&A_parentI, parent->A_BI, SENSOR->offset);
      }

    }  // RCSGRAPH_TRAVERSE_SENSORS(graph)

  }   // if(graph != NULL)

  Vx::VxRPRO* joint = new Vx::VxRPRO();
  joint->setParts(vxParent, vxChild);
  joint->setPosition(A_childI.org);
  joint->setPartAttachmentAxes(0, A_childI.rot[0], A_childI.rot[1]);
  joint->setPartAttachmentAxes(1, A_childI.rot[0], A_childI.rot[1]);
  // joint->setPartAttachmentAxesRel(0, Vec3d_ex(), Vec3d_ey());
  // joint->setPartAttachmentAxesRel(1, Vec3d_ex(), Vec3d_ey());
  joint->setRelativeQuaternionFromPart();
  joint->setName(loadCell ? loadCell->name : "FixedJoint");

  //joint->setPlasticDeformationEnable(true);
  joint->enableRelaxation(0, true);
  joint->enableRelaxation(1, true);
  joint->enableRelaxation(2, true);
  joint->enableRelaxation(3, true);
  joint->enableRelaxation(4, true);
  joint->enableRelaxation(5, true);

  // Give the RPRO joint some reduced stiffness, so that in case we get into
  // a rigid contact, the joint will absorb impacts. The settings have been
  // tuned for an acceptable deflection for a robot wrist FT sensor.
  Vx::VxReal stiffness = 10000.0;
  Vx::VxReal halfLife = 5.0;
  Vx::VxUniverse* universe = &Vx::VxFrame::instance()->getUniverse();
  Vx::VxReal damping = 1.0*universe->getCriticalDamping(stiffness, halfLife);
  Vx::VxReal loss = .0;
  joint->setRelaxationParameters(0, stiffness, damping, loss, true);
  joint->setRelaxationParameters(1, stiffness, damping, loss, true);
  joint->setRelaxationParameters(2, stiffness, damping, loss, true);
  joint->setRelaxationParameters(3, stiffness, damping, loss, true);
  joint->setRelaxationParameters(4, stiffness, damping, loss, true);
  joint->setRelaxationParameters(5, stiffness, damping, loss, true);

  joint->setUserDataPtr((void*) child);

  return joint;
}

/*******************************************************************************
 * Creates a prismatic joint between parent and child bodies.
 ******************************************************************************/
Vx::VxConstraint* Rcs::createPrismaticJoint(Vx::VxPart* part0,
                                            Vx::VxPart* part1,
                                            const double jointLockStiffness,
                                            const double jointLockDamping,
                                            const double q0)
{
  RLOG(5, "Create prismatic joint!");

  if (part0 == NULL || part1 == NULL)
  {
    RLOG(1, "Create prismatic joint between \"%s\" and \"%s\" failed",
         part0 ? part0->getName() : "NULL", part1 ? part1->getName() : "NULL");
    return NULL;
  }

  const RcsBody* child = (const RcsBody*) part1->getUserDataPtr();

  if (child->jnt == NULL)
  {
    RLOG(1, "Creation of joint between \"%s\" and \"%s\" failed: "
         "Bodies are not connected with joint!",
         part0->getName(), part1->getName());
    return NULL;
  }

  RcsJoint* jnt = child->jnt;
  Vx::VxVector3 anchor(jnt->A_JI.org);
  Vx::VxVector3 axis(jnt->A_JI.rot[jnt->dirIdx]);
  axis *= -1.0;

  Vx::VxPrismatic* joint = new Vx::VxPrismatic(part0, part1, anchor, axis);
  joint->setName(jnt->name);

  Vx::VxReal limitStiff = 1.0e10, limitDamping = 1.0e9;

  joint->setCoordinateCurrentPosition(Vx::VxPrismatic::kLinearCoordinate, q0);

  // Joint limits
  joint->setLimitPositions(Vx::VxPrismatic::kLinearCoordinate,
                           jnt->q_min, jnt->q_max);
  joint->setLimitStiffness(Vx::VxPrismatic::kLinearCoordinate,
                           Vx::VxConstraint::kLimitLower, limitStiff);
  joint->setLimitStiffness(Vx::VxPrismatic::kLinearCoordinate,
                           Vx::VxConstraint::kLimitUpper, limitStiff);
  joint->setLimitDamping(Vx::VxPrismatic::kLinearCoordinate,
                         Vx::VxConstraint::kLimitLower, limitDamping);
  joint->setLimitDamping(Vx::VxPrismatic::kLinearCoordinate,
                         Vx::VxConstraint::kLimitUpper, limitDamping);
  joint->setLimitsActive(Vx::VxPrismatic::kLinearCoordinate, true);

  // Joint lock for kinematic control
  joint->setLockMaximumForce(Vx::VxPrismatic::kLinearCoordinate,
                             jnt->maxTorque);

  joint->setLockStiffnessAndDamping(Vx::VxPrismatic::kLinearCoordinate,
                                    jointLockStiffness, jointLockDamping);

  joint->setControl(Vx::VxPrismatic::kLinearCoordinate,
                    Vx::VxConstraint::kControlLocked);
  joint->setLockPosition(Vx::VxPrismatic::kLinearCoordinate, q0);

  RLOG(5, "Created joint between %s - %s: q = %g %s",
       part0->getName(), part1->getName(),
       RcsJoint_isRotation(jnt) ? q0*(180.0/M_PI) : q0*1000.0,
       RcsJoint_isRotation(jnt) ? "deg" : "mm");

  return joint;
}

/*******************************************************************************
 * Creates a revolute joint between body b0 and b1.
 ******************************************************************************/
Vx::VxConstraint* Rcs::createRevoluteJoint(Vx::VxPart* part0,
                                           Vx::VxPart* part1,
                                           const double jointLockStiffness,
                                           const double jointLockDamping,
                                           const double jointMotorLoss,
                                           const double q0)
{
  if ((part0==NULL) || (part1==NULL))
  {
    RLOG(1, "Create revolute joint between \"%s\" and \"%s\" failed",
         part0 ? part0->getName() : "NULL", part1 ? part1->getName() : "NULL");
    return NULL;
  }

  const RcsBody* child = (const RcsBody*) part1->getUserDataPtr();

  if (child->jnt==NULL)
  {
    RLOG(1, "Creation of joint between \"%s\" and \"%s\" failed: "
         "Bodies are not connected with RcsJoint!",
         part0->getName(), part1->getName());
    return NULL;
  }

  RcsJoint* jnt = child->jnt;
  Vx::VxVector3 anchor(jnt->A_JI.org);
  Vx::VxVector3 axis(jnt->A_JI.rot[jnt->dirIdx]);
  axis *= -1.0;
  Vx::VxHinge* joint = new Vx::VxHinge(part0, part1, anchor, axis);
  joint->setName(jnt->name);

  // transfer the initial joint position
  joint->setCoordinateCurrentPosition(Vx::VxHinge::kAngularCoordinate, q0);

  // limit parameters (mechanical stop)
  // -----------------------------------------
  joint->setLimitPositions(Vx::VxHinge::kAngularCoordinate,
                           jnt->q_min, jnt->q_max);
  joint->setLimitsActive(Vx::VxHinge::kAngularCoordinate, true);

  joint->setLimitRestitution(Vx::VxHinge::kAngularCoordinate,
                             Vx::VxConstraint::kLimitUpper, 0.0);
  joint->setLimitRestitution(Vx::VxHinge::kAngularCoordinate,
                             Vx::VxConstraint::kLimitLower, 0.0);

  // joint->setLimitMaximumForce(Vx::VxHinge::kAngularCoordinate,
  //                             Vx::VxConstraint::kLimitLower, jnt->maxTorque);
  // joint->setLimitMaximumForce(Vx::VxHinge::kAngularCoordinate,
  //                             Vx::VxConstraint::kLimitUpper, jnt->maxTorque);


  if (jnt->ctrlType == RCSJOINT_CTRL_POSITION)
  {
    joint->setLockMaximumForce(Vx::VxHinge::kAngularCoordinate,
                               jnt->maxTorque);
    joint->setLockStiffnessAndDamping(Vx::VxHinge::kAngularCoordinate,
                                      jointLockStiffness, jointLockDamping);
    joint->setLockPosition(Vx::VxHinge::kAngularCoordinate, q0);
    joint->setControl(Vx::VxHinge::kAngularCoordinate,
                      Vx::VxConstraint::kControlLocked);
  }
  else if (jnt->ctrlType == RCSJOINT_CTRL_VELOCITY)
  {
    joint->setMotorMaximumForce(Vx::VxHinge::kAngularCoordinate,
                                jnt->maxTorque);
    joint->setMotorLoss(Vx::VxHinge::kAngularCoordinate, jointMotorLoss);
    joint->setMotorDesiredVelocity(Vx::VxHinge::kAngularCoordinate, 0.0);
    joint->setControl(Vx::VxHinge::kAngularCoordinate,
                      Vx::VxConstraint::kControlMotorized);
  }
  else if (jnt->ctrlType == RCSJOINT_CTRL_TORQUE)
  {
    // If the max. force is set to 0, the motor gets deactivated
    joint->setMotorMaximumForce(Vx::VxHinge::kAngularCoordinate, 1.0e-8);
    joint->setMotorLoss(Vx::VxHinge::kAngularCoordinate, 0.0);
    joint->setMotorDesiredVelocity(Vx::VxHinge::kAngularCoordinate, 0.0);
    joint->setControl(Vx::VxHinge::kAngularCoordinate,
                      Vx::VxConstraint::kControlMotorized);
  }
  else
  {
    RFATAL("joint control type %d is not yet supported by Rcs_Vortex",
           jnt->ctrlType);
  }

  RLOG(5, "Created joint between %s - %s: q = %g %s",
       part0->getName(), part1->getName(),
       RcsJoint_isRotation(jnt) ? q0*(180.0/M_PI) : q0*1000.0,
       RcsJoint_isRotation(jnt) ? "deg" : "mm");

  return joint;
}

/*******************************************************************************
 * Print functions for materials / material table
 ******************************************************************************/
static std::string frictionModelToStr(Vx::VxMaterialBase::FrictionModel frictionModel)
{
  std::string frictionMdlStr;

  switch (frictionModel)
  {
    case Vx::VxMaterial::kFrictionModelBox:
      frictionMdlStr = "kFrictionModelBox";
      break;

    case Vx::VxMaterial::kFrictionModelScaledBox:
      frictionMdlStr = "kFrictionModelScaledBox";
      break;

    case Vx::VxMaterial::kFrictionModelBoxProportionalLow:
      frictionMdlStr = "kFrictionModelBoxProportionalLow";
      break;

    case Vx::VxMaterial::kFrictionModelBoxProportionalHigh:
      frictionMdlStr = "kFrictionModelBoxProportionalHigh";
      break;

    case Vx::VxMaterial::kFrictionModelScaledBoxFast:
      frictionMdlStr = "kFrictionModelScaledBoxFast";
      break;

    case Vx::VxMaterial::kFrictionModelNeutral:
      frictionMdlStr = "kFrictionModelNeutral";
      break;

    case Vx::VxMaterial::kFrictionModelNone:
      frictionMdlStr = "kFrictionModelNone";
      break;

    default:
      frictionMdlStr = "undefined (this should never happen)";
  }

  return frictionMdlStr;
}

void Rcs::printMaterial(const Vx::VxMaterial* material, std::ostream& out)
{
  std::string intSlipDisp;
  switch (material->getIntegratedSlipDisplacement())
  {
    case Vx::VxMaterial::kIntegratedSlipDisplacementNeutral:
      intSlipDisp = "kIntegratedSlipDisplacementNeutral";
      break;

    case Vx::VxMaterial::kIntegratedSlipDisplacementDeactivated:
      intSlipDisp = "kIntegratedSlipDisplacementDeactivated";
      break;

    case Vx::VxMaterial::kIntegratedSlipDisplacementActivated:
      intSlipDisp = "kIntegratedSlipDisplacementActivated";
      break;

    case Vx::VxMaterial::kIntegratedSlipDisplacementNever:
      intSlipDisp = "kIntegratedSlipDisplacementNever";
      break;

    default:
      intSlipDisp = "undefined (this should never happen)";
  }
  out
      <<"*** Material \"" << std::string(material->getName())
      << "\" (Index " << material->getIndex() <<") :"
      <<"\n -- IntegratedSlipDisplacement: " << intSlipDisp
      <<"\n -- Adhesive force: " << material->getAdhesiveForce()
      <<"\n -- Compliance: " << material->getCompliance()
      <<"\n -- Damping: " << material->getDamping()
      <<"\n -- Restitution: " << material->getRestitution()
      <<"\n -- Restitution threshold: " << material->getRestitutionThreshold();

  out
      <<"\n -- Friction model:"
      <<"\n      kFrictionAxisLinearPrimary: "
      << frictionModelToStr(material->getFrictionModel(Vx::VxMaterialBase::kFrictionAxisLinearPrimary))
      <<"\n      kFrictionAxisLinearSecondary: "
      << frictionModelToStr(material->getFrictionModel(Vx::VxMaterialBase::kFrictionAxisLinearSecondary))
      <<"\n      kFrictionAxisAngularNormal: "
      << frictionModelToStr(material->getFrictionModel(Vx::VxMaterialBase::kFrictionAxisAngularNormal))
      <<"\n      kFrictionAxisAngularPrimary: "
      << frictionModelToStr(material->getFrictionModel(Vx::VxMaterialBase::kFrictionAxisAngularPrimary))
      <<"\n      kFrictionAxisAngularSecondary: "
      << frictionModelToStr(material->getFrictionModel(Vx::VxMaterialBase::kFrictionAxisAngularSecondary));

  out
      <<"\n -- Friction coefficient:"
      <<"\n      kFrictionAxisLinearPrimary: "
      << material->getFrictionCoefficient(Vx::VxMaterialBase::kFrictionAxisLinearPrimary)
      <<"\n      kFrictionAxisLinearSecondary: "
      << material->getFrictionCoefficient(Vx::VxMaterialBase::kFrictionAxisLinearSecondary)
      <<"\n      kFrictionAxisAngularNormal: "
      << material->getFrictionCoefficient(Vx::VxMaterialBase::kFrictionAxisAngularNormal)
      <<"\n      kFrictionAxisAngularPrimary: "
      << material->getFrictionCoefficient(Vx::VxMaterialBase::kFrictionAxisAngularPrimary)
      <<"\n      kFrictionAxisAngularSecondary: "
      << material->getFrictionCoefficient(Vx::VxMaterialBase::kFrictionAxisAngularSecondary);

  out
      <<"\n -- Slide:"
      <<"\n      kFrictionAxisLinearPrimary: "
      << material->getSlide(Vx::VxMaterialBase::kFrictionAxisLinearPrimary)
      <<"\n      kFrictionAxisLinearSecondary: "
      << material->getSlide(Vx::VxMaterialBase::kFrictionAxisLinearSecondary)
      <<"\n      kFrictionAxisAngularNormal: "
      << material->getSlide(Vx::VxMaterialBase::kFrictionAxisAngularNormal)
      <<"\n      kFrictionAxisAngularPrimary: "
      << material->getSlide(Vx::VxMaterialBase::kFrictionAxisAngularPrimary)
      <<"\n      kFrictionAxisAngularSecondary: "
      << material->getSlide(Vx::VxMaterialBase::kFrictionAxisAngularSecondary);

  out
      <<"\n -- Slip:"
      <<"\n      kFrictionAxisLinearPrimary: "
      << material->getSlip(Vx::VxMaterialBase::kFrictionAxisLinearPrimary)
      <<"\n      kFrictionAxisLinearSecondary: "
      << material->getSlip(Vx::VxMaterialBase::kFrictionAxisLinearSecondary)
      <<"\n      kFrictionAxisAngularNormal: "
      << material->getSlip(Vx::VxMaterialBase::kFrictionAxisAngularNormal)
      <<"\n      kFrictionAxisAngularPrimary: "
      << material->getSlip(Vx::VxMaterialBase::kFrictionAxisAngularPrimary)
      <<"\n      kFrictionAxisAngularSecondary: "
      << material->getSlip(Vx::VxMaterialBase::kFrictionAxisAngularSecondary);

  out << std::endl;
}

void Rcs::printMaterialTable(std::ostream& out)
{
  Vx::VxMaterialTable* mt = getMaterialTable();

  for (size_t i=0; i<mt->getMaterialCount(); ++i)
  {
    printMaterial(mt->getMaterial(i), out);
  }
}

void Rcs::printMaterialTable()
{
  printMaterialTable(std::cout);
}
