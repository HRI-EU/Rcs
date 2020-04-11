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
#include "Rcs_Mat3d.h"

#include <stdint.h>
#include <limits.h>
#include <float.h>



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
 *
 ******************************************************************************/
int RcsMesh_compressVertices(RcsMeshData* mesh, double eps)
{
  unsigned int vertexCount = 0, indexCount = 0;
  int nDuplicates = 0;
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
        nDuplicates++;
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

  return nDuplicates;
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
      NLOG(5, "Vertex line: %s", lineStr);
      (*numVertices) ++;
    }
  }

  RLOG(5, "Found %d vertices", *numVertices);
  rewind(fd);

  double* verts = RNALLOC(3*(*numVertices), double);

  if (verts == NULL)
  {
    RLOG(4, "Failed to load obj file \"%s\" with %d vertices",
         fileName, *numVertices);
    *numVertices = 0;
    fclose(fd);
    return NULL;
  }

  char buf[4][32];
  unsigned int count = 0;

  while (fgets(lineStr, sizeof(lineStr), fd))
  {
    if (STRNEQ(lineStr, "v ", 2))
    {
      int nItemsRead = sscanf(lineStr, "%31s %31s %31s %31s",
                              buf[0], buf[1], buf[2], buf[3]);

      if (nItemsRead < 4)
      {
        RLOG(4, "[%s, vertex %d]: Failed to read 4 vertices (%d only: \"%s\")",
             fileName, count/3, nItemsRead, lineStr);
        RFREE(verts);
        *numVertices = 0;
        fclose(fd);
        return NULL;
      }
      else
      {
        verts[count]   = String_toDouble_l(buf[1]);
        verts[count+1] = String_toDouble_l(buf[2]);
        verts[count+2] = String_toDouble_l(buf[3]);
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

  for (unsigned int i=0; i<mesh->nVertices; ++i)
  {
    fprintf(stderr, "Vertex %d: %f   %f   %f\n", i, mesh->vertices[i*3+0],
            mesh->vertices[i*3+1], mesh->vertices[i*3+2]);
  }

  for (unsigned int i=0; i<mesh->nFaces; ++i)
  {
    fprintf(stderr, "Face %d: %d   %d   %d (%f   %f   %f)\n", i,
            mesh->faces[i*3+0], mesh->faces[i*3+1], mesh->faces[i*3+2],
            mesh->vertices[mesh->faces[i*3+0]],
            mesh->vertices[mesh->faces[i*3+1]],
            mesh->vertices[mesh->faces[i*3+2]]);
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

  for (unsigned int i = 0; i < mesh->nVertices; i=i+3)
  {
    const double* v = &mesh->vertices[i];

    for (int j = 0; j < 3; ++j)
    {
      if (v[j] < xyzMin[j])
      {
        xyzMin[j] = v[j];
      }

      if (v[j] > xyzMax[j])
      {
        xyzMax[j] = v[j];
      }
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
  size_t vMem = 3*(mesh->nVertices+other->nVertices)*sizeof(double);
  mesh->vertices = (double*) realloc(mesh->vertices, vMem);

  size_t fMem = 3*(mesh->nFaces+other->nFaces)*sizeof(unsigned int);
  mesh->faces = (unsigned int*) realloc(mesh->faces, fMem);
  RCHECK(mesh->faces);

  memcpy(&mesh->vertices[3*mesh->nVertices], other->vertices,
         3*other->nVertices*sizeof(double));

  unsigned int largestIdx = getLargestFaceIdx(mesh);

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

  const unsigned int maxFaceIdx = getLargestFaceIdx(mesh);

  if (maxFaceIdx > mesh->nVertices-1)
  {
    RLOG(4, "Largest face index is %d, but %d vertices",
         maxFaceIdx, mesh->nVertices);
    return false;
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
 * This function is based on three.js library licensed under the MIT license:
 * See https://raw.githubusercontent.com/mrdoob/three.js/master/src/
 *             geometries/CylinderGeometry.js
 ******************************************************************************/
RcsMeshData* RcsMesh_createCylinderHull(double radiusBottom,
                                        double radiusTop,
                                        double height,
                                        unsigned int radialSegments,
                                        unsigned int heightSegments,
                                        double angleAround)
{
  const double thetaStart = 0.0;
  const double thetaLength = angleAround;
  const double halfHeight = 0.5*height;
  unsigned int index = 0;
  RcsMeshData* mesh = RALLOC(RcsMeshData);
  mesh->nVertices = (radialSegments+1)*(heightSegments+1);
  mesh->vertices = RNALLOC(3*mesh->nVertices, double);
  mesh->nFaces = 2*radialSegments*heightSegments;
  mesh->faces = RNALLOC(3*mesh->nFaces, unsigned int);

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
      unsigned int vIdx = y*(radialSegments+1)+x;
      double* vertex = &mesh->vertices[3*vIdx];
      vertex[0] = radius * sin(theta);
      vertex[2] = -v * height + halfHeight;
      vertex[1] = radius*cos(theta);

      // save index of vertex in respective row
      indexArray[y][x] = index++;
    }

  }

  // generate indices
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
      int fIdx = y*radialSegments+x;
      unsigned int* index = &mesh->faces[6*fIdx];
      index[0] = d;
      index[1] = b;
      index[2] = a;
      index[3] = d;
      index[4] = c;
      index[5] = b;
    }

  }

  for (unsigned int i=0; i<=heightSegments; ++i)
  {
    RFREE(indexArray[i]);
  }

  RFREE(indexArray);

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
 * This function is based on three.js library licensed under the MIT license:
 * See https://raw.githubusercontent.com/mrdoob/three.js/master/src/
 *             geometries/CylinderGeometry.js
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
  mesh->nVertices = (heightSegments+1)*(widthSegments+1);
  mesh->vertices = RNALLOC(3*mesh->nVertices, double);
  mesh->nFaces = 2*heightSegments*widthSegments;
  mesh->faces = RNALLOC(3*mesh->nFaces, unsigned int);

  unsigned int** grid = RNALLOC(heightSegments+1, unsigned int*);
  for (unsigned int i=0; i<=heightSegments; ++i)
  {
    grid[i] = RNALLOC(widthSegments+1, unsigned int);
  }

  int index = 0;
  double thetaEnd = fmin(thetaStart + thetaLength, M_PI);


  // generate vertices, normals and uvs
  for (unsigned int iy = 0; iy <= heightSegments; iy ++)
  {
    double v = (double) iy / heightSegments;

    for (unsigned int ix = 0; ix <= widthSegments; ix ++)
    {
      double u = (double) ix / widthSegments;

      // vertex
      unsigned int vIdx = iy*(widthSegments+1)+ix;
      double* vtx = &mesh->vertices[3*vIdx];
      vtx[0] = -radius*cos(phiStart+u*phiLength)*sin(thetaStart+v*thetaLength);
      vtx[1] = radius*cos(thetaStart+v*thetaLength);
      vtx[2] = radius*sin(phiStart+u*phiLength)*sin(thetaStart+v*thetaLength);

      grid[iy][ix] = index++;
    }

  }

  // indices
  for (unsigned int iy = 0; iy < heightSegments; iy ++)
  {
    for (unsigned int ix = 0; ix < widthSegments; ix ++)
    {
      int a = grid[iy][ix+1];
      int b = grid[iy][ix];
      int c = grid[iy+1][ix];
      int d = grid[iy+1][ix+1];

      int fIdx = iy*widthSegments+ix;
      unsigned int* index = &mesh->faces[6*fIdx];

      if (iy != 0 || thetaStart > 0.0)
      {
        index[0] = a;
        index[1] = b;
        index[2] = d;
      }

      if (iy != heightSegments - 1 || thetaEnd < M_PI)
      {
        index[3] = b;
        index[4] = c;
        index[5] = d;
      }

    }

  }

  for (unsigned int i=0; i<=heightSegments; ++i)
  {
    RFREE(grid[i]);
  }

  RFREE(grid);

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
  RcsMeshData* mesh = RcsMesh_createCylinderHull(radius, radius, height,
                                                 segments, segments, 2.0*M_PI);
  RcsMesh_shift(mesh, 0.0, 0.0, 0.5*height);

  RcsMeshData* topCap = RcsMesh_createSphereSegment(radius, segments/2, segments/2,
                                                    0.0, M_PI, 0.0, M_PI);
  RcsMesh_shift(topCap, 0.0, 0.0, height);
  RcsMesh_add(mesh, topCap);
  RcsMesh_destroy(topCap);

  RcsMeshData* bottomCap = RcsMesh_createSphereSegment(radius, segments/2, segments/2,
                                                       M_PI, M_PI, 0.0, M_PI);
  RcsMesh_add(mesh, bottomCap);
  RcsMesh_destroy(bottomCap);

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
 * See header.
 ******************************************************************************/
static RcsMeshData* RcsMesh_createRectangle(double x, double y)
{
  RcsMeshData* mesh = RALLOC(RcsMeshData);
  mesh->nFaces = 2;
  mesh->nVertices = 4;
  mesh->faces = RNALLOC(mesh->nFaces*3, unsigned int);
  mesh->vertices = RNALLOC(mesh->nVertices*3, double);

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

  memcpy(mesh->faces, vertexIndex, 3*mesh->nFaces*sizeof(unsigned int));

  for (unsigned int i=0; i<12; i=i+3)
  {
    mesh->vertices[i+0] = verts[i+0]*x;
    mesh->vertices[i+1] = verts[i+1]*y;
    mesh->vertices[i+2] = 0.0;
  }

  return mesh;
}

/*******************************************************************************
 *
 ******************************************************************************/
RcsMeshData* RcsMesh_createSSR(const double extents[3], unsigned int segments)
{
  const double r = 0.5*extents[2];
  double offset[3], rm[3][3];
  RcsMeshData* mesh = RcsMesh_createRectangle(extents[0], extents[1]);
  RcsMesh_shift(mesh, 0.0, 0.0, r);

  RcsMeshData* quad = RcsMesh_createRectangle(extents[0], extents[1]);
  Vec3d_set(offset, 0.0, 0.0, -r);
  Mat3d_setRotMatX(rm, M_PI);
  RcsMesh_transform(quad, offset, rm);
  RcsMesh_add(mesh, quad);
  RcsMesh_destroy(quad);

  RcsMeshData* cap = RcsMesh_createSphereSegment(r, segments, segments/2,
                                                 M_PI_2, M_PI_2, 0.0, M_PI);
  Vec3d_set(offset, 0.5*extents[0], 0.5*extents[1], 0.0);
  Mat3d_setRotMatX(rm, -M_PI_2);
  RcsMesh_transform(cap, offset, rm);
  RcsMesh_add(mesh, cap);
  RcsMesh_destroy(cap);

  cap = RcsMesh_createSphereSegment(r, segments, segments/2,
                                    -M_PI, M_PI_2, 0.0, M_PI);
  Vec3d_set(offset, 0.5*extents[0], -0.5*extents[1], 0.0);
  Mat3d_setRotMatX(rm, -M_PI_2);
  RcsMesh_transform(cap, offset, rm);
  RcsMesh_add(mesh, cap);
  RcsMesh_destroy(cap);

  cap = RcsMesh_createSphereSegment(r, segments, segments/2,
                                    -M_PI_2, M_PI_2, 0.0, M_PI);
  Vec3d_set(offset, -0.5*extents[0], -0.5*extents[1], 0.0);
  Mat3d_setRotMatX(rm, -M_PI_2);
  RcsMesh_transform(cap, offset, rm);
  RcsMesh_add(mesh, cap);
  RcsMesh_destroy(cap);

  cap = RcsMesh_createSphereSegment(r, segments, segments/2,
                                    0.0, M_PI_2, 0.0, M_PI);
  Vec3d_set(offset, -0.5*extents[0], 0.5*extents[1], 0.0);
  Mat3d_setRotMatX(rm, -M_PI_2);
  RcsMesh_transform(cap, offset, rm);
  RcsMesh_add(mesh, cap);
  RcsMesh_destroy(cap);

  RcsMeshData* cyl = RcsMesh_createCylinderHull(r, r, extents[1],
                                                segments, segments, M_PI);
  Vec3d_set(offset, 0.5*extents[0], 0.0, 0.0);
  Mat3d_setRotMatX(rm, M_PI_2);
  RcsMesh_transform(cyl, offset, rm);
  RcsMesh_add(mesh, cyl);
  RcsMesh_destroy(cyl);

  cyl = RcsMesh_createCylinderHull(r, r, extents[1], segments, segments, M_PI);
  Vec3d_set(offset, -0.5*extents[0], 0.0, 0.0);
  Mat3d_fromEulerAngles2(rm, M_PI_2, 0.0, M_PI);
  RcsMesh_transform(cyl, offset, rm);
  RcsMesh_add(mesh, cyl);
  RcsMesh_destroy(cyl);

  cyl = RcsMesh_createCylinderHull(r, r, extents[0], segments, segments, M_PI);
  Vec3d_set(offset, 0.0, 0.5*extents[1], 0.0);
  Mat3d_fromEulerAngles2(rm, M_PI_2, M_PI_2, 0.0);
  RcsMesh_transform(cyl, offset, rm);
  RcsMesh_add(mesh, cyl);
  RcsMesh_destroy(cyl);

  cyl = RcsMesh_createCylinderHull(r, r, extents[0], segments, segments, M_PI);
  Vec3d_set(offset, 0.0, -0.5*extents[1], 0.0);
  Mat3d_fromEulerAngles2(rm, M_PI_2, -M_PI_2, 0.0);
  RcsMesh_transform(cyl, offset, rm);
  RcsMesh_add(mesh, cyl);
  RcsMesh_destroy(cyl);

  return mesh;
}
