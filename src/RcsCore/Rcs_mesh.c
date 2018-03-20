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

#include "Rcs_mesh.h"
#include "Rcs_utils.h"
#include "Rcs_macros.h"
#include "Rcs_Vec3d.h"

#include <stdint.h>
#include <limits.h>



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
bool RcsMesh_compressVertices(RcsMeshData* mesh, double eps)
{
  unsigned int vertexCount = 0, indexCount = 0;
  unsigned int* faces = RNALLOC(mesh->nFaces*3, unsigned int);

  if (faces==NULL)
  {
    RLOG(4, "Failed to allocate memory for %d faces", mesh->nFaces);
    return false;
  }

  double* vertices = RNALLOC(mesh->nVertices*3, double);

  if (faces==NULL)
  {
    RLOG(4, "Failed to allocate memory for %d vertices", mesh->nVertices);
    RFREE(faces);
    return false;
  }

  for (unsigned int i=0; i<3*mesh->nFaces; ++i)
  {
    if (mesh->faces[i]>=mesh->nVertices)
    {
      RLOG(1, "Index error: faces[%d] = %d is equal or larger than nVertices"
           " = %d - skipping compression",  i, mesh->faces[i], mesh->nVertices);
      RFREE(faces);
      RFREE(vertices);
      return false;
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

  RFREE(mesh->vertices);
  RFREE(mesh->faces);
  mesh->nVertices = vertexCount;
  mesh->nFaces = indexCount/3;
  mesh->vertices = vertices;
  mesh->faces = faces;

  return true;
}

/*******************************************************************************
 * If the file contains NULL bytes, it is a binary file.
 ******************************************************************************/
/* static */ bool Rcs_isBinaryFile(FILE* data)
{
  int c;
  bool isBinary = true;

  while ((c = getc(data)) != EOF && c <= 127)
    ;
  if (c == EOF)
  {
    /* file is all ASCII */
    isBinary = false;
  }
  rewind(data);
  return isBinary;
  //
  //fseek(data, 0, SEEK_END);
  //size_t len = ftell(data);
  //rewind(data);
  //bool isBinary = memchr((const void*) data, '\0', len) != NULL;
  //
  //return isBinary;
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

  while (fscanf(fd ,"%255s", buf) != EOF)
  {
    if (STRCASEEQ(buf, "vertex"))
    {
      itemsMatched = fscanf(fd, "%lf", vPtr);

      if (itemsMatched < 1)
      {
        RLOG(1, "Couldn't read vertex x");
        *numVertices = 0;
        RFREE(vertices);
        return NULL;
      }

      vPtr++;
      itemsMatched = fscanf(fd, "%lf", vPtr);

      if (itemsMatched < 1)
      {
        RLOG(1, "Couldn't read vertex y");
        *numVertices = 0;
        RFREE(vertices);
        return NULL;
      }

      vPtr++;

      if (itemsMatched < 1)
      {
        RLOG(1, "Couldn't read vertex z");
        *numVertices = 0;
        RFREE(vertices);
        return NULL;
      }

      itemsMatched = fscanf(fd, "%lf", vPtr);
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
    fprintf(fd, "f %u %u %u\n", f[0], f[1], f[2]);
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
  strcpy(header, fileName);

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
      RLOG(4, "Error writing normal vector of face %d to binary STL file \"%s\"",
           i, fileName);
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
      RLOG(4, "Error writing attribute byte count of face %d to binary STL file \"%s\"",
           i, fileName);
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
    RLOG(4, "Failed to load binary stl file with %d vertices", *numVertices);
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
      RLOG(4, "Error reading STL file");
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
        RLOG(4, "Error reading STL file");
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
  FILE* fd = fopen(fileName, "rb");
  *numVertices = 0;

  if (fd==NULL)
  {
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

  //bool isBinary = Rcs_isBinaryFile(fd);
  //RLOG(0, "File %s is %s", fileName, isBinary ? "BINARY" : "ASCII");

  if (STRCASEEQ(firstWord, "solid"))
    //if (!isBinary)
  {
    NLOG(5, "STL file \"%s\" is not binary", fileName);
    verts = RcsMesh_readAsciiSTLFile(fd, numVertices);
  }
  else
  {
    NLOG(5, "STL file \"%s\" is binary", fileName);
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
static double* RcsMesh_readObjFile(const char* fileName,
                                   unsigned int* numVertices)
{
  RLOG(5, "Reading mesh file \"%s\"", fileName);
  FILE* fd = fopen(fileName, "rb");
  *numVertices = 0;

  if (fd==NULL)
  {
    return NULL;
  }

  char lineStr[512];
  while (fgets(lineStr, sizeof(lineStr), fd))
  {
    if (STRNEQ(lineStr, "v ", 2))
    {
      RLOG(5, "Vertex line: %s", lineStr);
      (*numVertices) ++;
    }
  }

  RLOG(5, "Found %d vertices", *numVertices);
  rewind(fd);

  double* verts = RNALLOC(3*(*numVertices), double);

  if (verts == NULL)
  {
    RLOG(4, "Failed to load obj file with %d vertices", *numVertices);
    *numVertices = 0;
    fclose(fd);
    return NULL;
  }

  char buf[32];
  unsigned int count = 0;

  while (fgets(lineStr, sizeof(lineStr), fd))
  {
    if (STRNEQ(lineStr, "v ", 2))
    {
      int nItemsRead = fscanf(fd, "%31s %lf %lf %lf", buf,
                              &verts[count], &verts[count+1], &verts[count+2]);
      if (nItemsRead < 4)
      {
        RLOG(4, "Failed to read 4 vertices (%d only)", nItemsRead);
        RFREE(verts);
        *numVertices = 0;
        fclose(fd);
        return NULL;
      }

      count += 3;
    }
  }

  fclose(fd);

  if (verts == NULL)
  {
    RLOG(1, "Failed to read Obj file \"%s\"", fileName);
  }

  return verts;
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
      int nItemsRead = fscanf(fd, "%lf", &meshData->vertices[i]);

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
    meshData->vertices = RcsMesh_readObjFile(meshFile, &meshData->nVertices);
    return (meshData->vertices != NULL) ? true : false;
  }

  return false;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsMesh_print(const RcsMeshData* mesh)
{
  if (mesh == NULL)
  {
    RLOG(4, "Mesh is NULL - not printing");
    return;
  }

  fprintf(stderr, "Mesh has %d (mod: %d) vertices and %d faces (mod: %d)\n",
          mesh->nVertices, mesh->nVertices%3, mesh->nFaces, mesh->nFaces%3);

  for (unsigned int i=0; i<mesh->nVertices/3; ++i)
  {
    fprintf(stderr, "Vertex %d: %f   %f   %f\n", i, mesh->vertices[i*3+0],
            mesh->vertices[i*3+1], mesh->vertices[i*3+2]);
  }

  for (unsigned int i=0; i<mesh->nFaces; ++i)
  {
    fprintf(stderr, "Face %d: %d   %d   %d\n", i,
            mesh->faces[i*3+0], mesh->faces[i*3+1], mesh->faces[i*3+2]);
  }
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
 *
 ******************************************************************************/
#if !defined (USE_WM5)
RcsMeshData* RcsMesh_fromVertices(const double* vCoords,
                                  unsigned int numVertices)
{
  RLOG(4, "Delaunay triangulation requires GeometricTools library - "
       "not available");
  return NULL;
}

#endif
