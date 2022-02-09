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

#include "Rcs_mesh.h"
#include "Rcs_utils.h"
#include "Rcs_macros.h"
#include "Rcs_Vec3d.h"
#include "Rcs_Mat3d.h"
#include "Rcs_basicMath.h"

#include <stdint.h>
#include <limits.h>
#include <float.h>



/*******************************************************************************
 * Convenience functions to compute number of faces and indices for different
 * shape types.
 ******************************************************************************/
static inline unsigned int nvSphereSeg(unsigned int heightSegments,
                                       unsigned int widthSegments)
{
  return (heightSegments+1)*(widthSegments+1);
}

static inline unsigned int nfSphereSeg(unsigned int heightSegments,
                                       unsigned int widthSegments)
{
  return 2*heightSegments*widthSegments;
}

static inline unsigned int nvCylHull(unsigned int radialSegments,
                                     unsigned int heightSegments)
{
  return (radialSegments+1)*(heightSegments+1);
}

static inline unsigned int nfCylHull(unsigned int radialSegments,
                                     unsigned int heightSegments)
{
  return 2*radialSegments*heightSegments;
}

static inline unsigned int nvCapsule(unsigned int segments)
{
  return nvCylHull(2*segments, 1) + 2*nvSphereSeg(segments, segments);
}

static inline unsigned int nfCapsule(unsigned int segments)
{
  return nfCylHull(2*segments, 1) + 2*nfSphereSeg(segments, segments);
}

static inline unsigned int nvRectangle()
{
  return 4;
}

static inline unsigned int nfRectangle()
{
  return 2;
}

static inline unsigned int nvSSR(unsigned int seg)
{
  return 2*nvRectangle() + 4*nvSphereSeg(seg, seg/2) + 4*nvSphereSeg(seg, seg);
}

static inline unsigned int nfSSR(unsigned int seg)
{
  return 2*nfRectangle() + 4*nfSphereSeg(seg, seg/2) + 4*nfSphereSeg(seg, seg);
}

/*******************************************************************************
 *
 ******************************************************************************/
static inline void getFace(const RcsMeshData* mesh, unsigned int face,
                           double** v1, double** v2, double** v3)
{
  *v1 = &mesh->vertices[3*mesh->faces[face*3+0]];
  *v2 = &mesh->vertices[3*mesh->faces[face*3+1]];
  *v3 = &mesh->vertices[3*mesh->faces[face*3+2]];
}

/*******************************************************************************
 *
 ******************************************************************************/
static inline unsigned int getLargestFaceIdx(const RcsMeshData* mesh)
{
  unsigned int largest = 0;

  for (unsigned int i=0; i<3*mesh->nFaces; ++i)
  {
    if (mesh->faces[i] > largest)
    {
      largest = mesh->faces[i];
    }
  }

  return largest;
}

/*******************************************************************************
 * This function is based on three.js library licensed under the MIT license:
 * See https://raw.githubusercontent.com/mrdoob/three.js/master/src/
 *             geometries/CylinderGeometry.js
 *
 * Number of vertices: (radialSegments+1)*(heightSegments+1)
 * Number of faces:    2*radialSegments*heightSegments
 *
 ******************************************************************************/
static void RcsMesh_appendCylinderHull(RcsMeshData* mesh,
                                       double radiusBottom,
                                       double radiusTop,
                                       double height,
                                       unsigned int radialSegments,
                                       unsigned int heightSegments,
                                       double angleAround,
                                       const double offset[3],
                                       const double eulerAngs[3])
{
  double A_MI[3][3];

  if (eulerAngs)
  {
    Mat3d_fromEulerAngles(A_MI, eulerAngs);
  }

  unsigned int startVertex = mesh->nVertices;
  unsigned int startFace = mesh->nFaces;
  mesh->nVertices += nvCylHull(radialSegments, heightSegments);
  mesh->nFaces += nfCylHull(radialSegments, heightSegments);

  const double thetaStart = 0.0;
  const double thetaLength = angleAround;
  const double halfHeight = 0.5*height;
  unsigned int index = 0;

  unsigned int** indexArray = RNALLOC(heightSegments+1, unsigned int*);
  for (unsigned int i=0; i<=heightSegments; ++i)
  {
    indexArray[i] = RNALLOC(radialSegments+1, unsigned int);
  }

  // generate vertices
  for (unsigned int y = 0; y <= heightSegments; y ++)
  {
    double v = (double) y / heightSegments;

    // calculate the radius of the current row
    double radius = v*(radiusBottom-radiusTop) + radiusTop;

    for (unsigned int x = 0; x <= radialSegments; x ++)
    {
      double u = (double) x / radialSegments;
      double theta = u * thetaLength + thetaStart;

      // vertex
      unsigned int vIdx = y*(radialSegments+1) + x + startVertex;
      double* vertex = &mesh->vertices[3*vIdx];
      vertex[0] = radius * sin(theta);
      vertex[2] = -v * height + halfHeight;
      vertex[1] = radius*cos(theta);

      if (eulerAngs)
      {
        Vec3d_transRotateSelf(vertex, A_MI);
      }

      if (offset)
      {
        Vec3d_addSelf(vertex, offset);
      }


      // save index of vertex in respective row
      indexArray[y][x] = index++;
    }

  }

  // generate indices
  int count = 0;
  for (unsigned int x = 0; x < radialSegments; x ++)
  {
    for (unsigned int y = 0; y < heightSegments; y ++)
    {
      // we use the index array to access the correct indices
      unsigned int a = indexArray[y][x];
      unsigned int b = indexArray[y+1][x];
      unsigned int c = indexArray[y+1][x+1];
      unsigned int d = indexArray[y][x+1];

      // faces
      int fIdx = y*radialSegments + x;
      unsigned int* index = &mesh->faces[6*fIdx + 3*startFace];
      index[0] = d+startVertex;
      index[1] = b+startVertex;
      index[2] = a+startVertex;
      index[3] = d+startVertex;
      index[4] = c+startVertex;
      index[5] = b+startVertex;
      count += 2;
    }

  }

  RCHECK(index+startVertex==mesh->nVertices);
  RCHECK(count+startFace==mesh->nFaces);

  for (unsigned int i=0; i<=heightSegments; ++i)
  {
    RFREE(indexArray[i]);
  }

  RFREE(indexArray);
}

/*******************************************************************************
 * This function is based on three.js library licensed under the MIT license:
 * See https://raw.githubusercontent.com/mrdoob/three.js/master/src/
 *             geometries/CylinderGeometry.js
 *
 * Number of vertices: (heightSegments+1)*(widthSegments+1)
 * Number of faces:    2*heightSegments*widthSegments
 *
 ******************************************************************************/
static void RcsMesh_appendSphereSegment(RcsMeshData* mesh,
                                        double radius,
                                        unsigned int heightSegments,
                                        unsigned int widthSegments,
                                        double phiStart,
                                        double phiLength,
                                        double thetaStart,
                                        double thetaLength,
                                        const double offset[3],
                                        const double eulerAngs[3])
{
  double A_MI[3][3];

  if (eulerAngs)
  {
    Mat3d_fromEulerAngles(A_MI, eulerAngs);
  }

  unsigned int startVertex = mesh->nVertices;
  unsigned int startFace = mesh->nFaces;

  unsigned int** grid = RNALLOC(heightSegments+1, unsigned int*);
  for (unsigned int i=0; i<=heightSegments; ++i)
  {
    grid[i] = RNALLOC(widthSegments+1, unsigned int);
  }

  int gridIndex = 0;
  double thetaEnd = fmin(thetaStart + thetaLength, M_PI);


  // generate vertices
  for (unsigned int iy = 0; iy <= heightSegments; iy ++)
  {
    double v = (double) iy / heightSegments;

    for (unsigned int ix = 0; ix <= widthSegments; ix ++)
    {
      double u = (double) ix / widthSegments;

      // vertex
      unsigned int vIdx = iy*(widthSegments+1)+ix + startVertex;
      double* vtx = &mesh->vertices[3*vIdx];
      vtx[0] = -radius*cos(phiStart+u*phiLength)*sin(thetaStart+v*thetaLength);
      vtx[1] = radius*cos(thetaStart+v*thetaLength);
      vtx[2] = radius*sin(phiStart+u*phiLength)*sin(thetaStart+v*thetaLength);

      if (eulerAngs)
      {
        Vec3d_transRotateSelf(vtx, A_MI);
      }

      if (offset)
      {
        Vec3d_addSelf(vtx, offset);
      }

      grid[iy][ix] = gridIndex + startVertex;
      gridIndex++;
    }

  }

  // indices
  int ridx = startFace;

  for (unsigned int iy = 0; iy < heightSegments; iy ++)
  {
    for (unsigned int ix = 0; ix < widthSegments; ix ++)
    {
      int a = grid[iy][ix+1];
      int b = grid[iy][ix];
      int c = grid[iy+1][ix];
      int d = grid[iy+1][ix+1];

      unsigned int* index = &mesh->faces[3*ridx];

      if (iy != 0 || thetaStart > 0.0)
      {
        index[0] = a;
        index[1] = b;
        index[2] = d;
        ridx++;
      }

      index = &mesh->faces[3*ridx];

      if (iy != heightSegments - 1 || thetaEnd < M_PI)
      {
        index[0] = b;
        index[1] = c;
        index[2] = d;
        ridx++;
      }

    }

  }

  mesh->nFaces = ridx;
  mesh->nVertices = nvSphereSeg(heightSegments, widthSegments) + startVertex;

  for (unsigned int i=0; i<=heightSegments; ++i)
  {
    RFREE(grid[i]);
  }

  RFREE(grid);
}

/*******************************************************************************
 *
 ******************************************************************************/
static void RcsMesh_appendCapsule(RcsMeshData* mesh,
                                  double radius,
                                  double height,
                                  unsigned int segments)
{
  double offset[3];
  Vec3d_set(offset, 0.0, 0.0, 0.5*height);

  RcsMesh_appendCylinderHull(mesh, radius, radius, height,
                             2*segments, 1, 2.0*M_PI, offset, NULL);

  Vec3d_set(offset, 0.0, 0.0, height);
  RcsMesh_appendSphereSegment(mesh, radius, segments, segments,
                              0.0, M_PI, 0.0, M_PI, offset, NULL);

  RcsMesh_appendSphereSegment(mesh, radius, segments, segments,
                              M_PI, M_PI, 0.0, M_PI, NULL, NULL);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
static void RcsMesh_appendRectangle(RcsMeshData* mesh, double x, double y,
                                    const double offset[3], const double ea[3])
{
  double A_MI[3][3];

  if (ea)
  {
    Mat3d_fromEulerAngles(A_MI, ea);
  }
  else
  {
    Mat3d_setIdentity(A_MI);
  }

  const unsigned int vertexIndex[6] =
  {
    0, 1, 2,
    0, 2, 3
  };

  const double verts[12] =
  {
    -0.5, -0.5, 0.0,
      0.5, -0.5, 0.0,
      0.5,  0.5, 0.0,
      -0.5,  0.5, 0.0,
    };

  const unsigned int nf = nfRectangle();

  for (unsigned int i=0; i<6; ++i)
  {
    mesh->faces[3*mesh->nFaces+i] = vertexIndex[i] + mesh->nVertices;
  }

  //memcpy(&mesh->faces[3*mesh->nFaces], vertexIndex, 3*nf*sizeof(unsigned int));

  for (unsigned int i=0; i<12; i=i+3)
  {
    double* vtx = &mesh->vertices[3*mesh->nVertices+i];
    vtx[0] = verts[i+0]*x;
    vtx[1] = verts[i+1]*y;
    vtx[2] = 0.0;

    if (ea)
    {
      Vec3d_transRotateSelf(vtx, A_MI);
    }

    if (offset)
    {
      Vec3d_addSelf(vtx, offset);
    }
  }

  mesh->nVertices += nvRectangle();
  mesh->nFaces += nf;
}

/*******************************************************************************
 *
 ******************************************************************************/
static void RcsMesh_appendSSR(RcsMeshData* mesh, const double extents[3],
                              unsigned int segments)
{
  const double r = 0.5*extents[2];
  double offset[3], ea[3];
  Vec3d_setZero(offset);
  Vec3d_setZero(ea);

  // Top rectangle
  offset[2] = r;
  RcsMesh_appendRectangle(mesh, extents[0], extents[1], offset, NULL);

  // Bottom rectangle
  offset[2] = -r;
  ea[0] = M_PI;
  RcsMesh_appendRectangle(mesh, extents[0], extents[1], offset, ea);

  // Corner +x / +y
  Vec3d_set(offset, 0.5*extents[0], 0.5*extents[1], 0.0);
  ea[0] = -M_PI_2;
  RcsMesh_appendSphereSegment(mesh, r, segments, segments/2,
                              M_PI_2, M_PI_2, 0.0, M_PI, offset, ea);

  // Corner +x / -y
  Vec3d_set(offset, 0.5*extents[0], -0.5*extents[1], 0.0);
  ea[0] = -M_PI_2;
  RcsMesh_appendSphereSegment(mesh, r, segments, segments/2,
                              -M_PI, M_PI_2, 0.0, M_PI, offset, ea);

  // Corner -x / -y
  Vec3d_set(offset, -0.5*extents[0], -0.5*extents[1], 0.0);
  ea[0] = -M_PI_2;
  RcsMesh_appendSphereSegment(mesh, r, segments, segments/2,
                              -M_PI_2, M_PI_2, 0.0, M_PI, offset, ea);

  // Corner -x / +y
  Vec3d_set(offset, -0.5*extents[0], 0.5*extents[1], 0.0);
  ea[0] = -M_PI_2;
  RcsMesh_appendSphereSegment(mesh, r, segments, segments/2,
                              0.0, M_PI_2, 0.0, M_PI, offset, ea);

  // Edge +x
  Vec3d_set(offset, 0.5*extents[0], 0.0, 0.0);
  ea[0] = M_PI_2;
  RcsMesh_appendCylinderHull(mesh, r, r, extents[1], segments, segments,
                             M_PI, offset, ea);

  // Edge +y
  Vec3d_set(offset, 0.0, 0.5*extents[1], 0.0);
  Vec3d_set(ea, M_PI_2, M_PI_2, 0.0);
  RcsMesh_appendCylinderHull(mesh, r, r, extents[0], segments, segments, M_PI,
                             offset, ea);

  // Edge -x
  Vec3d_set(offset, -0.5*extents[0], 0.0, 0.0);
  Vec3d_set(ea, M_PI_2, 0.0, M_PI);
  RcsMesh_appendCylinderHull(mesh, r, r, extents[1], segments, segments, M_PI,
                             offset, ea);

  // Edge -y
  Vec3d_set(offset, 0.0, -0.5*extents[1], 0.0);
  Vec3d_set(ea, M_PI_2, -M_PI_2, 0.0);
  RcsMesh_appendCylinderHull(mesh, r, r, extents[0], segments, segments, M_PI,
                             offset, ea);
}

/*******************************************************************************
 *
 ******************************************************************************/
void RcsMesh_copyCapsule(RcsMeshData* mesh,
                         double radius,
                         double height,
                         unsigned int segments)
{
  mesh->nVertices = 0;
  mesh->nFaces = 0;
  RcsMesh_appendCapsule(mesh, radius, height, segments);
}

/*******************************************************************************
 *
 ******************************************************************************/
void RcsMesh_copySSR(RcsMeshData* mesh,
                     const double extents[3],
                     unsigned int segments)
{
  mesh->nVertices = 0;
  mesh->nFaces = 0;
  RcsMesh_appendSSR(mesh, extents, segments);
}

/*******************************************************************************
 *
 ******************************************************************************/
int RcsMesh_compressVertices(RcsMeshData* mesh, double eps)
{
  unsigned int vertexCount = 0, indexCount = 0;
  unsigned int* faces = RNALLOC(mesh->nFaces*3, unsigned int);

  if (faces==NULL)
  {
    RLOG(4, "Failed to allocate memory for %d faces", mesh->nFaces);
    return -1;
  }

  double* vertices = RNALLOC(mesh->nVertices*3, double);

  if (faces==NULL)
  {
    RLOG(4, "Failed to allocate memory for %d vertices", mesh->nVertices);
    RFREE(faces);
    return -1;
  }

  for (unsigned int i=0; i<3*mesh->nFaces; ++i)
  {
    if (mesh->faces[i]>=mesh->nVertices)
    {
      RLOG(1, "Index error: faces[%d] = %d is equal or larger than nVertices"
           " = %d - skipping compression",  i, mesh->faces[i], mesh->nVertices);
      RFREE(faces);
      RFREE(vertices);
      return -1;
    }
    const double* v_i = &mesh->vertices[3*mesh->faces[i]];
    bool foundDuplicate = false;

    for (unsigned int j=0; j<vertexCount; ++j)
    {
      const double* v_j = &vertices[3*j];

      // If we found a duplicate vertex, we don't add it to the new vertex
      // array, but just update the corresponding face index to point to its
      // original memory.
      if (Vec3d_distance(v_i, v_j) < eps)   // Vertex already exists
      {
        faces[indexCount] = j;
        foundDuplicate = true;
        break;
      }

    }   // for (unsigned int j=0;j<vertexCount;++j)

    // If there is no duplicate for this vertex, we add it and set the face
    // index to point to its location
    if (foundDuplicate==false)
    {
      Vec3d_copy(&vertices[3*vertexCount], v_i);
      faces[indexCount] = vertexCount;
      vertexCount++;
    }

    indexCount++;

  }   // for (unsigned int i=0;i<mesh->nVertices;++i)

  int nDuplicates = mesh->nVertices - vertexCount;
  RFREE(mesh->vertices);
  RFREE(mesh->faces);
  mesh->nVertices = vertexCount;
  mesh->nFaces = indexCount/3;
  mesh->vertices = vertices;
  mesh->faces = faces;

  return nDuplicates;
}

/*******************************************************************************
 *
 * From https://en.wikipedia.org/wiki/STL_(file_format) :
 *
 * solid name
 *
 *   facet normal ni nj nk
 *       outer loop
 *           vertex v1x v1y v1z
 *           vertex v2x v2y v2z
 *           vertex v3x v3y v3z
 *       endloop
 *   endfacet
 *
 *   ...
 *
 * endsolid
 *
 ******************************************************************************/
static double* RcsMesh_readAsciiSTLFile(FILE* fd, unsigned int* numVertices)
{
  char buf[256] = "";
  int itemsMatched = 0;
  *numVertices = 0;

  // Second string is the solid name
  itemsMatched = fscanf(fd, "%255s", buf);

  if (itemsMatched < 1)
  {
    RLOG(1, "Couldn't read first file item");
    return NULL;
  }

  unsigned int numFacets = 0;
  while (fscanf(fd, "%255s", buf) != EOF)
  {
    if (STRCASEEQ(buf, "facet"))
    {
      numFacets++;
    }
  }

  rewind(fd);

  *numVertices = numFacets*3;
  double* vertices = RNALLOC(numFacets*9, double);

  if (vertices == NULL)
  {
    RLOG(4, "Failed to load binary stl file with %d vertices", *numVertices);
    *numVertices = 0;
    return NULL;
  }

  double* vPtr = &vertices[0];

  while (fscanf(fd,"%255s", buf) != EOF)
  {
    if (STRCASEEQ(buf, "vertex"))
    {
      itemsMatched = fscanf(fd, "%255s", buf);
      *vPtr = String_toDouble_l(buf);

      if (itemsMatched < 1)
      {
        RLOG(1, "Couldn't read vertex x");
        *numVertices = 0;
        RFREE(vertices);
        return NULL;
      }

      vPtr++;
      itemsMatched = fscanf(fd, "%255s", buf);
      *vPtr = String_toDouble_l(buf);

      if (itemsMatched < 1)
      {
        RLOG(1, "Couldn't read vertex y");
        *numVertices = 0;
        RFREE(vertices);
        return NULL;
      }

      vPtr++;

      itemsMatched = fscanf(fd, "%255s", buf);
      *vPtr = String_toDouble_l(buf);

      if (itemsMatched < 1)
      {
        RLOG(1, "Couldn't read vertex z");
        *numVertices = 0;
        RFREE(vertices);
        return NULL;
      }

      vPtr++;
    }
  }

  return vertices;
}

/*******************************************************************************
 *
 ******************************************************************************/
static bool RcsMesh_toTRIFile(const RcsMeshData* mesh,
                              const char* fileName,
                              FILE* fd)
{
  fprintf(fd, "TRI\n%d\n%d\n", mesh->nVertices, mesh->nFaces);

  for (unsigned int i=0; i<mesh->nVertices; ++i)
  {
    const double* v = &mesh->vertices[i*3];
    fprintf(fd, "%.8f %.8f %.8f\n", v[0], v[1], v[2]);
  }

  for (unsigned int i=0; i<mesh->nFaces; ++i)
  {
    const unsigned int* f = &mesh->faces[i*3];
    fprintf(fd, "%u %u %u\n", f[0], f[1], f[2]);
  }

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
static bool RcsMesh_toOBJFile(const RcsMeshData* mesh,
                              const char* fileName,
                              FILE* fd)
{
  fprintf(fd, "o %s\n", fileName);

  for (unsigned int i=0; i<mesh->nVertices; ++i)
  {
    const double* v = &mesh->vertices[i*3];
    fprintf(fd, "v %.8f %.8f %.8f\n", v[0], v[1], v[2]);
  }

  for (unsigned int i=0; i<mesh->nFaces; ++i)
  {
    const unsigned int* f = &mesh->faces[i*3];
    fprintf(fd, "f %u %u %u\n", f[0]+1, f[1]+1, f[2]+1);
  }

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
static bool RcsMesh_toSTLFile(const RcsMeshData* mesh,
                              const char* fileName,
                              FILE* fd)
{
  char header[80];
  memset(header, 0, 80*sizeof(char));
  snprintf(header, 80, "%s", fileName);

  size_t itemsWritten = fwrite(header, sizeof(uint8_t), 80, fd);

  if (itemsWritten != 80)
  {
    RLOG(4, "Error writing header to binary STL file \"%s\"",
         fileName);
    return false;
  }

  itemsWritten = fwrite(&mesh->nFaces, sizeof(uint32_t), 1, fd);

  if (itemsWritten != 1)
  {
    RLOG(4, "Error writing number of triangles to binary STL file \"%s\"",
         fileName);
    return false;
  }

  for (unsigned int i=0; i<mesh->nFaces; ++i)
  {
    // Normal vector
    double* v1, *v2, *v3;
    getFace(mesh, i, &v1, &v2, &v3);

    double v12[3], v23[3], n[3];
    Vec3d_setZero(n);
    Vec3d_sub(v12, v2, v1);
    Vec3d_sub(v23, v3, v2);
    Vec3d_crossProduct(n, v12, v23);
    double lenN = Vec3d_normalizeSelf(n);

    if (lenN==0.0)
    {
      RLOG(4, "Could not compute normal vector for face %d", i);
    }

    float nf[3];
    nf[0] = (float) n[0];
    nf[1] = (float) n[1];
    nf[2] = (float) n[2];
    itemsWritten = fwrite(&nf, sizeof(uint32_t), 3, fd);

    if (itemsWritten != 3)
    {
      RLOG(4, "Error writing normal vector of face %d to binary STL file"
           "\"%s\"", i, fileName);
      return false;
    }

    // Vertex 1, 2, 3
    float vf[9];
    for (unsigned int j=0; j<3; ++j)
    {
      vf[j+0] = (float) v1[j];
      vf[j+3] = (float) v2[j];
      vf[j+6] = (float) v3[j];
    }

    itemsWritten = fwrite(&vf, sizeof(uint32_t), 9, fd);

    if (itemsWritten != 9)
    {
      RLOG(4, "Error writing vertices of face %d to binary STL file \"%s\"",
           i, fileName);
      return false;
    }

    // Attribute byte count
    uint16_t abc = 0;

    itemsWritten = fwrite(&abc, sizeof(uint16_t), 1, fd);

    if (itemsWritten != 1)
    {
      RLOG(4, "Error writing attribute byte count of face %d to binary STL "
           "file \"%s\"", i, fileName);
      return false;
    }

  }

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool RcsMesh_toFile(const RcsMeshData* mesh, const char* fileName)
{
  if (sizeof(uint32_t) != sizeof(float))
  {
    RLOG(1, "sizeof(float) = %d, sizeof(uint32_t) = %d - not writing mesh file",
         (int)sizeof(float), (int)sizeof(uint32_t));
    return false;
  }

  if (mesh == NULL)
  {
    RLOG(1, "Mesh is NULL");
    return false;
  }

  if (fileName == NULL)
  {
    RLOG(1, "Mesh file name is NULL");
    return false;
  }

  FILE* fd = fopen(fileName, "wb");

  if (fd==NULL)
  {
    RLOG(1, "Error opening file \"%s\" for writing mesh file", fileName);
    return false;
  }

  bool success = false;

  if (String_hasEnding(fileName, ".stl", false))
  {
    success = RcsMesh_toSTLFile(mesh, fileName, fd);
  }
  else if (String_hasEnding(fileName, ".obj", false))
  {
    success = RcsMesh_toOBJFile(mesh, fileName, fd);
  }
  else if (String_hasEnding(fileName, ".tri", false))
  {
    success = RcsMesh_toTRIFile(mesh, fileName, fd);
  }
  else
  {
    RLOG(4, "Unknown file suffix for \"%s\" - skipping to write mesh file",
         fileName);
  }



  fclose(fd);

  return success;
}

/*******************************************************************************
 *
 *   From https://en.wikipedia.org/wiki/STL_(file_format) :
 *
 *   UINT8[80]: Header
 *   UINT32: Number of triangles
 *
 *   foreach triangle
 *     REAL32[3]: Normal vector
 *     REAL32[3]: Vertex 1
 *     REAL32[3]: Vertex 2
 *     REAL32[3]: Vertex 3
 *     UINT16: Attribute byte count
 *   end
 *
 ******************************************************************************/
static double* RcsMesh_readBinarySTLFile(FILE* fd, unsigned int* numVertices)
{
  if (sizeof(uint32_t) != sizeof(float))
  {
    RLOG(1, "sizeof(float) = %d, sizeof(uint32_t) = %d - not reading mesh file",
         (int)sizeof(float), (int)sizeof(uint32_t));
    return NULL;
  }

  // Read header
  size_t itemsRead = 0;
  char header[80] = "";
  itemsRead = fread(&header, sizeof(char), 80, fd);

  if (itemsRead < 80)
  {
    RLOG(4, "Error reading binary STL file header");
    return NULL;
  }

  // Read number of triangles
  uint32_t numTriangles = 0;
  itemsRead = fread(&numTriangles, sizeof(uint32_t), 1, fd);

  if (itemsRead == 0)
  {
    RLOG(4, "Error reading binary STL file in number of triangles");
    *numVertices = 0;
    return NULL;
  }

  // Prevent overflow
  if (numTriangles > INT32_MAX/3-1)
  {
    RLOG(4, "Too many triangles: %d (should be < %d)",
         numTriangles, INT32_MAX/3);
    *numVertices = 0;
    return NULL;
  }

  *numVertices = numTriangles*3;

  double* vertices = RNALLOC(numTriangles*9, double);

  if (vertices == NULL)
  {
    RLOG(4, "Failed to allocate memory for %d vertices", *numVertices);
    *numVertices = 0;
    return NULL;
  }

  double* vPtr = &vertices[0];

  // Read all vertices
  uint32_t buf[3];

  for (uint32_t i=0; i<numTriangles; ++i)
  {
    // Normal vector
    itemsRead = fread(&buf, sizeof(uint32_t), 3, fd);

    if (itemsRead < 3)
    {
      RLOG(4, "Error reading normals - read %zu items, expected 3", itemsRead);
      RFREE(vertices);
      return NULL;
    }

    // Vertex 1, 2, 3
    for (unsigned int j=0; j<9; ++j)
    {
      float fTmp = 0.0;
      itemsRead = fread(&fTmp, sizeof(uint32_t), 1, fd);

      if (itemsRead < 1)
      {
        RLOG(4, "Error reading vertices");
        RFREE(vertices);
        return NULL;
      }

      //float fTmp = *((float*) buf);
      *vPtr = fTmp;
      vPtr++;
    }

    // Attribute byte count
    uint16_t abc;
    itemsRead = fread(&abc, sizeof(uint16_t), 1, fd);

    if (itemsRead == 0)
    {
      RLOG(4, "Error reading STL file");
      RFREE(vertices);
      return NULL;
    }
  }

  return vertices;
}

/*******************************************************************************
 * Determines if STL file is binary or ASCII, and calls the respective read
 * function.
 ******************************************************************************/
static double* RcsMesh_readSTLFile(const char* fileName,
                                   unsigned int* numVertices)
{
  *numVertices = 0;

  if (fileName==NULL)
  {
    RLOG(4, "Failed to file \"NULL\"");
    return NULL;
  }

  FILE* fd = fopen(fileName, "rb");

  if (fd==NULL)
  {
    RLOG(4, "Failed to open STL file \"%s\"", fileName);
    return NULL;
  }

  // Check if the STL file is in ASCII format: That's the case if the first
  // string is "solid"
  char firstWord[256];
  int nItemsRead = fscanf(fd, "%255s", firstWord);

  if (nItemsRead < 1)
  {
    RLOG(4, "Error reading STL file: Couldn't read first word");
    fclose(fd);
    return NULL;
  }

  double* verts = NULL;

  // In case we find the solid keyword at te start of te file, it is very
  // likely (but not guaranteed) that it is an ASCII STL file.
  if (STRCASEEQ(firstWord, "solid"))
  {
    RLOG(5, "STL file \"%s\" is probably ASCII", fileName);
    verts = RcsMesh_readAsciiSTLFile(fd, numVertices);

    // There exist STL files that contain an ASCII header section, but a
    // binary vertex representation. We identify these edge cases if the
    // ASCII parsing returns no vertices.
    if (*numVertices==0)
    {
      RLOG(5, "ASCII file has no vertices - try parsing binary STL file");
      rewind(fd);
      verts = RcsMesh_readBinarySTLFile(fd, numVertices);
    }
  }
  else
  {
    RLOG(5, "STL file \"%s\" is binary", fileName);
    rewind(fd);
    verts = RcsMesh_readBinarySTLFile(fd, numVertices);
  }

  fclose(fd);

  if (verts == NULL)
  {
    RLOG(1, "Failed to read STL file \"%s\"", fileName);
  }

  return verts;
}

/*******************************************************************************
 * Reads all vertices from a wavefront obj file.
 ******************************************************************************/
static void String_chopOff(char* dst, const char* src, const char* delim)
{
  // Make a local copy of str, since strtok modifies it during processing
  char* lStr = String_clone(src);
  char* pch = strtok(lStr, delim);
  if (pch)
  {
    strcpy(dst, pch);
  }
  RFREE(lStr);
}



bool RcsMesh_readObjFile(const char* fileName, RcsMeshData* mesh)
{
  mesh->nVertices = 0;
  mesh->nFaces = 0;

  if (fileName==NULL)
  {
    RLOG(4, "Failed to file \"NULL\"");
    return false;
  }

  RLOG(5, "Reading mesh file \"%s\"", fileName);
  FILE* fd = fopen(fileName, "rb");

  if (fd==NULL)
  {
    RLOG(4, "Could not open mesh file \"%s\"", fileName);
    return false;
  }

  char lineStr[512];
  while (fgets(lineStr, sizeof(lineStr), fd))
  {
    if (STRNEQ(lineStr, "v ", 2))
    {
      NLOG(5, "Vertex line: %s", lineStr);
      mesh->nVertices++;
    }

    if (STRNEQ(lineStr, "f ", 2))
    {
      NLOG(5, "Face line: %s", lineStr);
      mesh->nFaces++;
    }
  }

  RLOG(5, "Found %d vertices and %d faces", mesh->nVertices, mesh->nFaces);
  rewind(fd);

  mesh->vertices = RNALLOC(3*mesh->nVertices, double);

  if (mesh->vertices == NULL)
  {
    RLOG(4, "Obj file \"%s\": Failed to alloate memory for %d vertices",
         fileName, mesh->nVertices);
    mesh->nVertices = 0;
    mesh->nFaces = 0;
    fclose(fd);
    return false;
  }

  mesh->faces = RNALLOC(3*mesh->nFaces, unsigned int);

  if (mesh->faces == NULL)
  {
    RLOG(4, "Obj file \"%s\": Failed to alloate memory for %d vertices",
         fileName, mesh->nFaces);
    mesh->nVertices = 0;
    mesh->nFaces = 0;
    RFREE(mesh->vertices);
    fclose(fd);
    return false;
  }

  char buf[4][32];
  unsigned int vCount = 0, fCount = 0;

  while (fgets(lineStr, sizeof(lineStr), fd))
  {
    RLOG(5, "vCount=%d fCount=%d", vCount, fCount);
    RLOG(5, "lineStr=%s", lineStr);

    if (STRNEQ(lineStr, "v ", 2))
    {
      int nItemsRead = sscanf(lineStr, "%31s %31s %31s %31s",
                              buf[0], buf[1], buf[2], buf[3]);

      if (nItemsRead < 4)
      {
        RLOG(4, "[%s, vertex %d]: Failed to read 4 vertices (%d only: \"%s\")",
             fileName, vCount / 3, nItemsRead, lineStr);
        RFREE(mesh->vertices);
        RFREE(mesh->faces);
        mesh->nVertices = 0;
        mesh->nFaces = 0;
        fclose(fd);
        return false;
      }
      else
      {
        mesh->vertices[vCount] = String_toDouble_l(buf[1]);
        mesh->vertices[vCount+1] = String_toDouble_l(buf[2]);
        mesh->vertices[vCount+2] = String_toDouble_l(buf[3]);
      }

      vCount += 3;
    }



    // Parse all faces
    if (STRNEQ(lineStr, "f ", 2))
    {
      int nItemsRead = sscanf(lineStr, "%31s %31s %31s %31s",
                              buf[0], buf[1], buf[2], buf[3]);

      if (nItemsRead < 4)
      {
        RLOG(4, "[%s, face %d]: Failed to read 4 faces (%d only: \"%s\")",
             fileName, fCount / 3, nItemsRead, lineStr);
        RFREE(mesh->vertices);
        RFREE(mesh->faces);
        mesh->nVertices = 0;
        mesh->nFaces = 0;
        fclose(fd);
        return false;
      }
      else
      {
        char tmp[32];
        String_chopOff(tmp, buf[1], "//");
        mesh->faces[fCount] = atoi(tmp) - 1;
        String_chopOff(tmp, buf[2], "//");
        mesh->faces[fCount + 1] = atoi(tmp) - 1;
        String_chopOff(tmp, buf[3], "//");
        mesh->faces[fCount + 2] = atoi(tmp) - 1;
        RLOG(5, "face=%d %d %d",
             mesh->faces[fCount],
             mesh->faces[fCount + 1],
             mesh->faces[fCount + 2]);
      }

      fCount += 3;
    }

  }

  fclose(fd);

  return true;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
RcsMeshData* RcsMesh_createFromFile(const char* meshFile)
{
  RcsMeshData* meshData = RALLOC(RcsMeshData);
  bool success = RcsMesh_readFromFile(meshFile, meshData);

  if (success == false)
  {
    RFREE(meshData);
    return NULL;
  }

  return meshData;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool RcsMesh_readFromFile(const char* meshFile, RcsMeshData* meshData)
{
  if (File_exists(meshFile)==false)
  {
    RLOG(1, "Couldn't open file \"%s\"", meshFile);
    return false;
  }

  if (String_hasEnding(meshFile, ".stl", false) == true)
  {
    meshData->vertices = RcsMesh_readSTLFile(meshFile, &meshData->nVertices);

    if (meshData->vertices == NULL)
    {
      RLOG(4, "Failed to load mesh file \"%s\"", meshFile);
      return false;
    }

    meshData->nFaces = meshData->nVertices/3;
    meshData->faces = RNALLOC(3*meshData->nFaces, unsigned int);

    if (meshData->faces == NULL)
    {
      RLOG(4, "Failed to create face list with %d faces for stl file \"%s\"",
           meshData->nFaces, meshFile);
      RFREE(meshData->vertices);
      meshData->nVertices = 0;
      meshData->nFaces = 0;
      return false;
    }

    for (unsigned int i=0; i<3*meshData->nFaces; i++)
    {
      meshData->faces[i] = i;
    }

    return true;
  }
  else if (String_hasEnding(meshFile, ".tri", false) == true)
  {
    FILE* fd = fopen(meshFile, "r");
    RCHECK(fd);
    char triString[256];
    int nItemsRead = fscanf(fd, "%255s %u %u", triString, &meshData->nVertices,
                            &meshData->nFaces);

    if (nItemsRead < 3)
    {
      RLOG(4, "Failed to read 3 items in .tri file: only %d", nItemsRead);
      meshData->nVertices = 0;
      meshData->nFaces = 0;
      fclose(fd);
      return false;
    }

    if ((meshData->nVertices>=UINT_MAX/3) || (meshData->nFaces>=UINT_MAX/3))
    {
      RLOG(4, "Too many vertices (%d) or faces (%d): %d is maximum (\"%s\")",
           meshData->nVertices, meshData->nFaces, UINT_MAX/3, meshFile);
      meshData->nVertices = 0;
      meshData->nFaces = 0;
      fclose(fd);
      return false;
    }

    NLOG(0, "Found tag \"%s\" and %d vertices and %d triangles",
         triString, meshData->nVertices, meshData->nFaces);

    meshData->vertices = RNALLOC(3*meshData->nVertices, double);

    if (meshData->vertices == NULL)
    {
      RLOG(4, "Failed to create memory for %d vertices for mesh file \"%s\"",
           meshData->nVertices, meshFile);
      meshData->nVertices = 0;
      meshData->nFaces = 0;
      fclose(fd);
      return false;
    }

    meshData->faces = RNALLOC(3*meshData->nFaces, unsigned int);

    if (meshData->faces == NULL)
    {
      RLOG(4, "Failed to create memory for %u faces for mesh file \"%s\"",
           meshData->nFaces, meshFile);
      RFREE(meshData->faces);
      meshData->nVertices = 0;
      meshData->nFaces = 0;
      fclose(fd);
      return false;
    }

    for (unsigned int i=0; i<3*meshData->nVertices; ++i)
    {
      int nItemsRead = fscanf(fd, "%255s", triString);
      meshData->vertices[i] = String_toDouble_l(triString);


      if (nItemsRead < 1)
      {
        RLOG(4, "Failed to read vertex coordinate %d for stl file \"%s\"",
             i, meshFile);
        RFREE(meshData->vertices);
        RFREE(meshData->faces);
        meshData->nVertices = 0;
        meshData->nFaces = 0;
        fclose(fd);
        return false;
      }

    }

    for (unsigned int i=0; i<3*meshData->nFaces; ++i)
    {
      int nItemsRead = fscanf(fd, "%u", &meshData->faces[i]);

      if (nItemsRead < 1)
      {
        RLOG(4, "Failed to read face coordinate %d for stl file \"%s\"",
             i, meshFile);
        RFREE(meshData->vertices);
        RFREE(meshData->faces);
        meshData->nVertices = 0;
        meshData->nFaces = 0;
        fclose(fd);
        return false;
      }
    }

    fclose(fd);
    return true;
  }
  else if (String_hasEnding(meshFile, ".obj", false) == true)
  {
    return RcsMesh_readObjFile(meshFile, meshData);
  }

  return false;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsMesh_fprint(FILE* out, const RcsMeshData* mesh)
{
  if (mesh == NULL)
  {
    RLOG(4, "Mesh is NULL - not printing");
    return;
  }

  fprintf(out, "Mesh has %d (mod: %d) vertices and %d faces (mod: %d)\n",
          mesh->nVertices, mesh->nVertices%3, mesh->nFaces, mesh->nFaces%3);

  for (unsigned int i=0; i<mesh->nVertices; ++i)
  {
    fprintf(out, "Vertex %d: %f   %f   %f\n", i, mesh->vertices[i*3+0],
            mesh->vertices[i*3+1], mesh->vertices[i*3+2]);
  }

  for (unsigned int i=0; i<mesh->nFaces; ++i)
  {
    fprintf(out, "Face %d: %d   %d   %d", i,
            mesh->faces[i*3+0], mesh->faces[i*3+1], mesh->faces[i*3+2]);
    fprintf(out, " (%f   %f   %f)\n",
            mesh->vertices[mesh->faces[i*3+0]],
            mesh->vertices[mesh->faces[i*3+1]],
            mesh->vertices[mesh->faces[i*3+2]]);
  }

}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsMesh_print(const RcsMeshData* mesh)
{
  RcsMesh_fprint(stdout, mesh);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
RcsMeshData* RcsMesh_clone(const RcsMeshData* src)
{
  if (src==NULL)
  {
    RLOG(4, "Can't clone NULL mesh - skipping");
    return NULL;
  }

  RcsMeshData* self = RALLOC(RcsMeshData);

  self->nVertices = src->nVertices;
  self->nFaces = src->nFaces;
  self->vertices = RNALLOC(3*self->nVertices, double);
  self->faces = RNALLOC(3*self->nFaces, unsigned int);

  memcpy(self->vertices, src->vertices, 3*self->nVertices*sizeof(double));
  memcpy(self->faces, src->faces, 3*self->nFaces*sizeof(unsigned int));

  return self;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsMesh_copy(RcsMeshData* dst, const RcsMeshData* src)
{
  if (dst->nVertices < src->nVertices)
  {
    dst->vertices = RREALLOC(dst->vertices, 3*dst->nVertices, double);
    RCHECK(dst->vertices);
  }

  if (dst->nFaces < src->nFaces)
  {
    dst->faces = RREALLOC(dst->faces, 3*dst->nFaces, unsigned int);
    RCHECK(dst->faces);
  }

  dst->nVertices = dst->nVertices;
  dst->nFaces = src->nFaces;
  memcpy(dst->vertices, src->vertices, 3*dst->nVertices*sizeof(double));
  memcpy(dst->faces, src->faces, 3*dst->nFaces*sizeof(unsigned int));
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsMesh_destroy(RcsMeshData* mesh)
{
  if (mesh == NULL)
  {
    return;
  }

  RFREE(mesh->vertices);
  RFREE(mesh->faces);
  RFREE(mesh);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsMesh_flipNormals(RcsMeshData* mesh)
{
  for (unsigned int i=0; i<mesh->nFaces; ++i)
  {
    unsigned int tmp = mesh->faces[i*3+0];
    mesh->faces[i*3+0] = mesh->faces[i*3+2];
    mesh->faces[i*3+2] = tmp;
  }
}

/*******************************************************************************
 * Adapted from http://stackoverflow.com/questions/1406029/
 *                     how-to-calculate-the-volume-of-a-3d-mesh-object-the-
 *                     surface-of-which-is-made-up
 ******************************************************************************/
static inline double signedTriangleVolume(const double* v1,
                                          const double* v2,
                                          const double* v3)
{
  double tmp[3];
  Vec3d_crossProduct(tmp, v2, v3);
  return Vec3d_innerProduct(v1, tmp) / 6.0;
}

double RcsMesh_computeVolume(RcsMeshData* mesh)
{
  if (mesh==NULL)
  {
    return 0.0;
  }

  double volume = 0.0;
  double* v1, *v2, *v3;

  for (unsigned int i=0; i<mesh->nFaces; ++i)
  {
    getFace(mesh, i, &v1, &v2, &v3);
    volume += signedTriangleVolume(v1, v2, v3);
  }

  return fabs(volume);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsMesh_computeAABB(const RcsMeshData* mesh,
                         double xyzMin[3], double xyzMax[3])
{
  if (mesh == NULL)
  {
    RLOG(4, "Mesh is NULL - AABB is set to zero");
    Vec3d_setZero(xyzMin);
    Vec3d_setZero(xyzMax);
    return;
  }

  Vec3d_set(xyzMin, DBL_MAX, DBL_MAX, DBL_MAX);
  Vec3d_set(xyzMax, -DBL_MAX, -DBL_MAX, -DBL_MAX);

  for (unsigned int i = 0; i < 3*mesh->nVertices; i=i+3)
  {
    const double* v = &mesh->vertices[i];

    for (int j = 0; j < 3; ++j)
    {
      xyzMin[j] = fmin(xyzMin[j], v[j]);
      xyzMax[j] = fmax(xyzMax[j], v[j]);
    }
  }
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsMesh_scale(RcsMeshData* mesh, double scale)
{
  for (unsigned int i=0; i<mesh->nVertices*3; ++i)
  {
    mesh->vertices[i] *= scale;
  }
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsMesh_scale3D(RcsMeshData* mesh, const double scale[3])
{
  for (unsigned int i=0; i<3*mesh->nVertices; i=i+3)
  {
    mesh->vertices[i+0] *= scale[0];
    mesh->vertices[i+1] *= scale[1];
    mesh->vertices[i+2] *= scale[2];
  }
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsMesh_shift(RcsMeshData* mesh, double x, double y, double z)
{
  for (unsigned int i=0; i<3*mesh->nVertices; i=i+3)
  {
    mesh->vertices[i+0] += x;
    mesh->vertices[i+1] += y;
    mesh->vertices[i+2] += z;
  }
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsMesh_rotate(RcsMeshData* mesh, double A_MI[3][3])
{
  for (unsigned int i=0; i<3*mesh->nVertices; i=i+3)
  {
    Vec3d_transRotateSelf(&mesh->vertices[i], A_MI);
  }
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsMesh_add(RcsMeshData* mesh, const RcsMeshData* other)
{
  const unsigned int largestIdx = getLargestFaceIdx(mesh);

  size_t vMem = 3*(mesh->nVertices+other->nVertices)*sizeof(double);
  mesh->vertices = (double*) realloc(mesh->vertices, vMem);
  RCHECK(mesh->vertices);

  size_t fMem = 3*(mesh->nFaces+other->nFaces)*sizeof(unsigned int);
  mesh->faces = (unsigned int*) realloc(mesh->faces, fMem);
  RCHECK(mesh->faces);

  memcpy(&mesh->vertices[3*mesh->nVertices], other->vertices,
         3*other->nVertices*sizeof(double));

  for (unsigned int i = 0; i < 3*other->nFaces; ++i)
  {
    mesh->faces[3*mesh->nFaces+i] = other->faces[i] + largestIdx + 1;
  }

  mesh->nVertices += other->nVertices;
  mesh->nFaces += other->nFaces;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool RcsMesh_check(const RcsMeshData* mesh)
{
  if (mesh==NULL)
  {
    RLOG(4, "NULL mesh didn't pass test");
    return false;
  }

  // CHeck that largest face index is less than the number of vertices
  const unsigned int maxFaceIdx = getLargestFaceIdx(mesh);

  if (maxFaceIdx > mesh->nVertices-1)
  {
    RLOG(4, "Largest face index is %d, but %d vertices",
         maxFaceIdx, mesh->nVertices);
    return false;
  }

  // Check that faces have three different indices
  for (unsigned int i=0; i<mesh->nFaces; ++i)
  {
    const unsigned int* fi = &mesh->faces[3*i];

    if ((fi[0]==fi[1]) || (fi[0]==fi[2]) || (fi[1]==fi[2]))
    {
      RLOG(4, "Face index %d does not have 3 different indices: %d %d %d",
           i, fi[0], fi[1], fi[2]);
      return false;
    }
  }

  return true;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsMesh_transform(RcsMeshData* mesh, const double pos[3],
                       double A_MI[3][3])
{
  RcsMesh_rotate(mesh, A_MI);
  RcsMesh_shift(mesh, pos[0], pos[1], pos[2]);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
RcsMeshData* RcsMesh_createBox(const double extents[3])
{
  RcsMeshData* mesh = RALLOC(RcsMeshData);
  mesh->nFaces = 12;
  mesh->nVertices = 8;
  mesh->faces = RNALLOC(mesh->nFaces*3, unsigned int);
  mesh->vertices = RNALLOC(mesh->nVertices*3, double);

  const unsigned int vertexIndex[36] =
  {
    0, 2, 1, // front
    0, 3, 2,
    2, 3, 4, // top
    2, 4, 5,
    1, 2, 5, // right
    1, 5, 6,
    0, 7, 4, // left
    0, 4, 3,
    5, 4, 7, // back
    5, 7, 6,
    0, 6, 7, // bottom
    0, 1, 6
  };

  const double verts[24] =
  {
    -0.5, -0.5, -0.5,
      0.5, -0.5, -0.5,
      0.5,  0.5, -0.5,
      -0.5,  0.5, -0.5,
      -0.5,  0.5,  0.5,
      0.5,  0.5,  0.5,
      0.5, -0.5,  0.5,
      -0.5, -0.5,  0.5,
    };

  memcpy(mesh->faces, vertexIndex, 3*mesh->nFaces*sizeof(unsigned int));

  for (unsigned int i=0; i<24; i=i+3)
  {
    mesh->vertices[i+0] = verts[i+0]*extents[0];
    mesh->vertices[i+1] = verts[i+1]*extents[1];
    mesh->vertices[i+2] = verts[i+2]*extents[2];
  }

  return mesh;
}

/*******************************************************************************
 * This function is based on three.js library licensed under the MIT license:
 * See https://raw.githubusercontent.com/mrdoob/three.js/master/src/
 *             geometries/CylinderGeometry.js
 ******************************************************************************/
static RcsMeshData* RcsMesh_createDisk(double radius,
                                       unsigned int radialSegments,
                                       double angleAround,
                                       bool top,
                                       double halfHeight,
                                       double centerOffset)
{
  const double thetaStart = 0.0;
  const double thetaLength = angleAround;
  const int sign = (top == true) ? 1 : - 1;
  unsigned int index = 0;
  unsigned int centerIndexStart, centerIndexEnd;

  RcsMeshData* mesh = RALLOC(RcsMeshData);
  mesh->nVertices = 2*radialSegments+1;
  mesh->vertices = RNALLOC(3*mesh->nVertices, double);
  mesh->nFaces = radialSegments+1;
  mesh->faces = RNALLOC(3*mesh->nFaces, unsigned int);

  // save the index of the first center vertex
  centerIndexStart = index;

  // Center vertex data of the cap.
  for (unsigned int x = 1; x <= radialSegments; ++x)
  {
    double* vertex = &mesh->vertices[3*(x-1)];
    vertex[0] = 0.0;
    vertex[2] = halfHeight * sign + centerOffset;
    vertex[1] = 0;

    index++;
  }

  // Save the index of the last center vertex
  centerIndexEnd = index;

  // Surrounding vertices
  for (unsigned int x = 0; x <= radialSegments; x ++)
  {
    double u = (double) x / radialSegments;
    double theta = u * thetaLength + thetaStart;

    double* vertex = &mesh->vertices[3*(centerIndexEnd+x)];
    vertex[0] = radius * sin(theta);
    vertex[2] = halfHeight * sign;
    vertex[1] = radius * cos(theta);

    index ++;

  }

  // generate indices
  for (unsigned int x = 0; x < radialSegments; x ++)
  {
    unsigned int c = centerIndexStart + x;
    unsigned int i = centerIndexEnd + x;
    unsigned int* index = &mesh->faces[3*x];

    if (top == true)// face top
    {
      index[0] = i;
      index[2] = i+1;
      index[1] = c;
    }
    else// face bottom
    {
      index[0] = i+1;
      index[2] = i;
      index[1] = c;
    }

  }

  return mesh;
}

/*******************************************************************************
 *
 ******************************************************************************/
RcsMeshData* RcsMesh_createCylinderHull(double radiusBottom,
                                        double radiusTop,
                                        double height,
                                        unsigned int radialSegments,
                                        unsigned int heightSegments,
                                        double angleAround)
{
  RcsMeshData* mesh = RALLOC(RcsMeshData);
  mesh->vertices = RNALLOC(3*nvCylHull(radialSegments, heightSegments), double);
  mesh->faces = RNALLOC(3*nfCylHull(radialSegments, heightSegments), unsigned int);
  RcsMesh_appendCylinderHull(mesh, radiusBottom, radiusTop, height,
                             radialSegments, heightSegments, angleAround,
                             NULL, NULL);
  return mesh;
}

/*******************************************************************************
 * This function is based on three.js library licensed under the MIT license:
 * See https://raw.githubusercontent.com/mrdoob/three.js/master/src/
 *             geometries/CylinderGeometry.js
 ******************************************************************************/
RcsMeshData* RcsMesh_createCylinder(double radius, double height,
                                    unsigned int radialSegments)
{
  const double angleAround = 2.0*M_PI;
  const unsigned int heghtSegments = 1;

  RcsMeshData* mesh = RcsMesh_createCylinderHull(radius, radius, height,
                                                 radialSegments, heghtSegments,
                                                 angleAround);

  RcsMeshData* topDisk = RcsMesh_createDisk(radius, radialSegments,
                                            angleAround, true, 0.5*height, 0.0);
  RcsMesh_add(mesh, topDisk);
  RcsMesh_destroy(topDisk);

  RcsMeshData* bottomDisk = RcsMesh_createDisk(radius, radialSegments,
                                               angleAround, false,
                                               0.5*height, 0.0);
  RcsMesh_add(mesh, bottomDisk);
  RcsMesh_destroy(bottomDisk);

  return mesh;
}

/*******************************************************************************
 *
 ******************************************************************************/
RcsMeshData* RcsMesh_createSphereSegment(double radius,
                                         unsigned int heightSegments,
                                         unsigned int widthSegments,
                                         double phiStart,
                                         double phiLength,
                                         double thetaStart,
                                         double thetaLength)
{
  RcsMeshData* mesh = RALLOC(RcsMeshData);
  const unsigned int nv = nvSphereSeg(heightSegments, widthSegments);
  const unsigned int nf = nfSphereSeg(heightSegments, widthSegments);
  mesh->vertices = RNALLOC(3*nv, double);
  mesh->faces = RNALLOC(3*nf, unsigned int);
  RcsMesh_appendSphereSegment(mesh, radius, heightSegments, widthSegments,
                              phiStart, phiLength, thetaStart, thetaLength,
                              Vec3d_zeroVec(), NULL);

  return mesh;
}

/*******************************************************************************
 *
 ******************************************************************************/
RcsMeshData* RcsMesh_createSphere(double radius, unsigned int segments)
{
  return RcsMesh_createSphereSegment(radius, segments/2, segments,
                                     0.0, 2.0*M_PI, 0.0, M_PI);
}

/*******************************************************************************
 *
 ******************************************************************************/
RcsMeshData* RcsMesh_createCapsule(double radius, double height,
                                   unsigned int segments)
{
  RcsMeshData* mesh = RALLOC(RcsMeshData);
  mesh->vertices = RNALLOC(3*nvCapsule(segments), double);
  mesh->faces = RNALLOC(3*nfCapsule(segments), unsigned int);
  RcsMesh_copyCapsule(mesh, radius, height, segments);

  return mesh;
}

/*******************************************************************************
 * This function is based on three.js library licensed under the MIT license:
 * https://github.com/mrdoob/three.js/blob/master/src/
 *         geometries/TorusGeometry.js
 ******************************************************************************/
RcsMeshData* RcsMesh_createTorus(double radius,
                                 double thickness,
                                 unsigned int radialSegments,
                                 unsigned int tubularSegments)
{
  double tube = 0.5*thickness;
  double arc = 2.0*M_PI;
  RcsMeshData* mesh = RALLOC(RcsMeshData);

  mesh->nVertices = (radialSegments+1)*(tubularSegments+1);
  mesh->nFaces = 2*radialSegments*tubularSegments;
  mesh->vertices = RNALLOC(3 * mesh->nVertices, double);
  mesh->faces = RNALLOC(3*mesh->nFaces, unsigned int);

  unsigned int idx = 0;

  for (unsigned int j = 0; j <= radialSegments; j++)
  {
    for (unsigned int i = 0; i <= tubularSegments; i++)
    {
      double u = (double)i / tubularSegments * arc;
      double v = (double)j / radialSegments * M_PI * 2.0;

      mesh->vertices[idx++] = (radius + tube * cos(v)) * cos(u);
      mesh->vertices[idx++] = (radius + tube * cos(v)) * sin(u);
      mesh->vertices[idx++] = tube * sin(v);
    }
  }

  idx = 0;
  for (unsigned int j = 1; j <= radialSegments; j++)
  {
    for (unsigned int i = 1; i <= tubularSegments; i++)
    {
      int a = (tubularSegments + 1) * j + i - 1;
      int b = (tubularSegments + 1) * (j - 1) + i - 1;
      int c = (tubularSegments + 1) * (j - 1) + i;
      int d = (tubularSegments + 1) * j + i;

      // faces
      mesh->faces[idx++] = a;
      mesh->faces[idx++] = b;
      mesh->faces[idx++] = d;
      mesh->faces[idx++] = b;
      mesh->faces[idx++] = c;
      mesh->faces[idx++] = d;
    }
  }

  return mesh;
}

/*******************************************************************************
 *
 ******************************************************************************/
RcsMeshData* RcsMesh_createCone(double radius, double height,
                                unsigned int segments)
{
  RcsMeshData* cone = RcsMesh_createDisk(radius, segments, 2.0*M_PI, true,
                                         0.0, height);
  RcsMeshData* disk = RcsMesh_createDisk(radius, segments, 2.0*M_PI, false,
                                         0.0, 0.0);
  RcsMesh_add(cone, disk);
  RcsMesh_destroy(disk);

  return cone;
}

/*******************************************************************************
 * 5 vertices, 6 triangles
 ******************************************************************************/
RcsMeshData* RcsMesh_createPyramid(double x, double y, double h)
{
  const unsigned int vertexIndex[] =
  {
    2,  1,  0,
    2,  0,  3,
    4,  0,  1,
    4,  1,  2,
    4,  2,  3,
    4,  3,  0
  };

  x *= 0.5;
  y *= 0.5;

  const double verts[] =
  {
    x,    y,     0,
    -x,   y,     0,
    -x,   -y,    0,
    x,   -y,     0,
    0,    0,     h
  };

  RcsMeshData* mesh = RALLOC(RcsMeshData);
  mesh->nFaces = 6;
  mesh->nVertices = 5;
  mesh->faces = RNALLOC(mesh->nFaces*3, unsigned int);
  mesh->vertices = RNALLOC(mesh->nVertices*3, double);

  memcpy(mesh->faces, vertexIndex, 3*mesh->nFaces*sizeof(unsigned int));
  memcpy(mesh->vertices, verts, 3*mesh->nVertices*sizeof(double));

  return mesh;
}

/*******************************************************************************
 * 5 vertices, 6 triangles
 ******************************************************************************/
RcsMeshData* RcsMesh_createFrustum(double fovX, double fovY, double h)
{
  double x = 2.0*tan(0.5*fovX);
  double y = 2.0*tan(0.5*fovY);
  RcsMeshData* mesh = RcsMesh_createPyramid(x, y, h);
  double A_MI[3][3];
  Mat3d_setRotMatX(A_MI, M_PI);
  RcsMesh_rotate(mesh, A_MI);
  RcsMesh_shift(mesh, 0.0, 0.0, h);

  return mesh;
}

/*******************************************************************************
 *
 ******************************************************************************/
RcsMeshData* RcsMesh_createConvexRandom(double x, double y, double z,
                                        unsigned int nPoints)
{
  double* v = RNALLOC(3*nPoints, double);

  for (unsigned int i=0; i<nPoints; ++i)
  {
    v[3*i+0] = Math_getRandomNumber(-0.5*x, 0.5*x);
    v[3*i+1] = Math_getRandomNumber(-0.5*y, 0.5*y);
    v[3*i+2] = Math_getRandomNumber(-0.5*z, 0.5*z);
  }

  RcsMeshData* mesh = RcsMesh_fromVertices(v, nPoints);

  RFREE(v);

  return mesh;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
#if 0
static RcsMeshData* RcsMesh_createRectangle(double x, double y)
{
  RcsMeshData* mesh = RALLOC(RcsMeshData);
  mesh->faces = RNALLOC(3*nfRectangle(), unsigned int);
  mesh->vertices = RNALLOC(3*nvRectangle(), double);
  RcsMesh_appendRectangle(mesh, x, y, NULL, NULL);
  return mesh;
}
#endif

/*******************************************************************************
 *
 ******************************************************************************/
RcsMeshData* RcsMesh_createSSR(const double extents[3], unsigned int segments)
{
  RcsMeshData* mesh = RALLOC(RcsMeshData);
  mesh->vertices = RNALLOC(3*nvSSR(segments), double);
  mesh->faces = RNALLOC(3*nfSSR(segments), unsigned int);
  RcsMesh_copySSR(mesh, extents, segments);

  return mesh;
}

/*******************************************************************************
 *
 ******************************************************************************/
double* RcsMesh_createNormalArray(const RcsMeshData* mesh)
{
  double* normals = RNALLOC(3*mesh->nVertices, double);

  for (unsigned int i=0; i<mesh->nFaces; ++i)
  {
    // Get the 3 triangle points of vertex i
    double* v1, *v2, *v3;
    getFace(mesh, i, &v1, &v2, &v3);

    // Compute the normal on it
    double v12[3], v13[3], n[3];
    Vec3d_sub(v12, v2, v1);
    Vec3d_sub(v13, v3, v1);
    Vec3d_crossProduct(n, v12, v13);
    Vec3d_normalizeSelf(n);

    unsigned int fidx0 = 3*mesh->faces[i*3+0];
    unsigned int fidx1 = 3*mesh->faces[i*3+1];
    unsigned int fidx2 = 3*mesh->faces[i*3+2];

    RCHECK_MSG(fidx0<3*mesh->nVertices, "%d %d", fidx0, 3*mesh->nVertices);
    RCHECK_MSG(fidx1<3*mesh->nVertices, "%d %d", fidx1, 3*mesh->nVertices);
    RCHECK_MSG(fidx2<3*mesh->nVertices, "%d %d", fidx2, 3*mesh->nVertices);

    // Add the face normal to the 3 vertices normal touching this face
    Vec3d_addSelf(&normals[3*mesh->faces[i*3]], n);
    Vec3d_addSelf(&normals[3*mesh->faces[i*3+1]], n);
    Vec3d_addSelf(&normals[3*mesh->faces[i*3+2]], n);
  }

  // After summing up all normals, normalize them to unit length
  for (unsigned int i=0; i<mesh->nVertices; ++i)
  {
    double* ni = &normals[3*i];
    Vec3d_normalizeSelf(ni);
  }

  return normals;
}
