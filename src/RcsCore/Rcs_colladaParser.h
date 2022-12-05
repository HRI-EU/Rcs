/**************************************************************************
**  Copyright (c) 2017, Honda Research Institute Europe GmbH
**
**  Redistribution and use in source and binary forms, with or without
**  modification, are permitted provided that the following conditions are
**  met:
**
**  1. Redistributions of source code must retain the above copyright notice,
**  this list of conditions and the following disclaimer.
**
**  2. Redistributions in binary form must reproduce the above copyright
**  notice, this list of conditions and the following disclaimer in the
**  documentation and/or other materials provided with the distribution.
**
**  3. Neither the name of the copyright holder nor the names of its
**  contributors may be used to endorse or promote products derived from
**  this software without specific prior written permission.
**
**  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
**  IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
**  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
**  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
**  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
**  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
**  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
**  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
**  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
**  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
**  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
**  Author: weima
**  Date: 8 Dec 2022
**
**************************************************************************/


#ifndef RCS_COLLADAPARSER_H
#define RCS_COLLADAPARSER_H

#include <stdlib.h>
#include <string.h>
#include <Rcs_macros.h>
#include <Rcs_utils.h>
#include <Rcs_parser.h>
#include <Rcs_mesh.h>
#include <Rcs_resourcePath.h>
#include <Rcs_HTr.h>



#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    char name[16];
    char type[16];
} ColladaParameter;

typedef struct {
    char accessorSource[128];
    int count;
    int stride;
    ColladaParameter* param;
    unsigned int paramLen;
} ColladaTechniqueCommon;

typedef struct
{
    char id[128];
    double* array;
    int arrLen;
    ColladaTechniqueCommon* techniqueCommon;
} ColladaSource;

typedef struct
{
    char semantic[16];
    char sourceId[128];
    double* array;  // just a link to source. No need to free.
    int arrayLen;
    int offset;
} ColladaInput;

typedef struct {
    char id[128];
    ColladaInput* input;
} ColladaVertices;

typedef struct
{
    char materialId[128];
    ColladaInput* inputs;
    int inputLen;
    int trianglesNum;
    int* indexArray;
    int indexArrLen;
} ColladaTriangles;

typedef struct
{
    ColladaSource* sources;
    int sourcesLen;
    ColladaVertices* vertices;
    ColladaTriangles* triangles;
} ColladaMesh;

typedef struct
{
    char id[128];
    char name[128];
    ColladaMesh* mesh;
} ColladaGeometry;

typedef struct
{
    ColladaGeometry* geometries;
    int geometriesLen;
} ColladaLibraryGeometries;

typedef struct
{
    char sid[16];
    HTr matrix;
} ColladaMatrix;

typedef struct
{
    char url[128];
    char name[16];
} ColladaInstanceGeometry;

typedef struct
{
    char id[16];
    char name[16];
    char type[16];
    ColladaMatrix* matrix;
    ColladaInstanceGeometry* instanceGeometry;
} ColladaVisualNode;

typedef struct
{
    char id[16];
    char name[16];
    ColladaVisualNode* nodes;
    int nodesLen;
} ColladaVisualScene;

typedef struct
{
    ColladaVisualScene* visualScenes;
    int visualScenesLen;
} ColladaLibraryVisualScenes;

typedef struct
{
    ColladaLibraryGeometries* libraryGeometries;
    ColladaLibraryVisualScenes* libraryVisualScenes;
} Collada;


void Rcs_printCollada(Collada* collada);

void Rcs_freeCollada(Collada* collada);

bool Rcs_parseDAEFile(const char* filename, Collada** collada);

int Rcs_getColladaTotalVerticesNum(Collada* collada);

int Rcs_getColladaTotalTrianglesNum(Collada* collada);

bool Rcs_parseMeshDataFromCollada(RcsMeshData* meshData, Collada* collada);

#ifdef __cplusplus
}
#endif

#endif // RCS_COLLADAPARSER_H
