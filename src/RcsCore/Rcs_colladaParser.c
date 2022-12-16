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

#include "Rcs_parser.h"
#include "Rcs_macros.h"
#include "Rcs_utils.h"
#include "Rcs_colladaParser.h"
#include "Rcs_Vec3d.h"
#include "Rcs_Mat3d.h"

void Rcs_freeSource(ColladaSource* source)
{
    if (!source)
        return;

    if (source->techniqueCommon)
    {
        RFREE(source->techniqueCommon->param);
        RFREE(source->techniqueCommon);
        source->techniqueCommon = NULL;
    }

    if (source->array)
    {
        RFREE(source->array);
        source->array = NULL;
    }
}

void Rcs_freeInput(ColladaInput* input)
{
    if (!input)
        return;

    input->array = NULL;
}

void Rcs_freeVertices(ColladaVertices* vertices)
{
    if (vertices->input)
    {
        Rcs_freeInput(vertices->input);
        RFREE(vertices->input);
        vertices->input = NULL;
    }
}

void Rcs_freeTriangles(ColladaTriangles* triangles)
{
    if (triangles->inputs)
    {
        for (int i = 0; i < triangles->inputLen; ++i)
        {
            Rcs_freeInput(&triangles->inputs[i]);
        }
        RFREE(triangles->inputs);
        triangles->inputs = NULL;
    }

    if (triangles->indexArray)
    {
        RFREE(triangles->indexArray);
        triangles->indexArray = NULL;
    }
}

void Rcs_freeMesh(ColladaMesh* mesh)
{
    if (mesh->sources)
    {
        for (int i = 0; i < mesh->sourcesLen; ++i)
        {
            Rcs_freeSource(&mesh->sources[i]);
        }
        RFREE(mesh->sources);
        mesh->sources = NULL;
    }

    if (mesh->vertices)
    {
        Rcs_freeVertices(mesh->vertices);
        RFREE(mesh->vertices);
        mesh->vertices = NULL;
    }

    if (mesh->triangles)
    {
        Rcs_freeTriangles(mesh->triangles);
        RFREE(mesh->triangles);
        mesh->triangles = NULL;
    }
}

void Rcs_freeGeometry(ColladaGeometry* geometry)
{
    if (geometry->mesh)
    {
        Rcs_freeMesh(geometry->mesh);
        RFREE(geometry->mesh);
        geometry->mesh = NULL;
    }
}

void Rcs_freeLibraryGeometries(ColladaLibraryGeometries* libraryGeometries)
{
    if (libraryGeometries->geometries)
    {
        for (int i = 0; i < libraryGeometries->geometriesLen; ++i)
        {
            Rcs_freeGeometry(&libraryGeometries->geometries[i]);
        }
        RFREE(libraryGeometries->geometries);
        libraryGeometries->geometries = NULL;
    }
}

void Rcs_freeColladaVisualNode(ColladaVisualNode* visualNode)
{
    if (visualNode->matrix)
    {
        RFREE(visualNode->matrix);
        visualNode->matrix = NULL;
    }

    if (visualNode->instanceGeometry)
    {
        RFREE(visualNode->instanceGeometry);
        visualNode->instanceGeometry = NULL;
    }
}

void Rcs_freeColladaVisualScene(ColladaVisualScene* visualScene)
{
    if (visualScene->nodes)
    {
        for (int i = 0; i < visualScene->nodesLen; ++i)
        {
            Rcs_freeColladaVisualNode(&visualScene->nodes[i]);
        }
        RFREE(visualScene->nodes);
        visualScene->nodes = NULL;
    }
}

void Rcs_freeColladaLibraryVisualScenes(ColladaLibraryVisualScenes* libraryVisualScenes)
{
    if (libraryVisualScenes->visualScenes)
    {
        for (int i = 0; i < libraryVisualScenes->visualScenesLen; ++i)
        {
            Rcs_freeColladaVisualScene(&libraryVisualScenes->visualScenes[i]);
        }
        RFREE(libraryVisualScenes->visualScenes);
        libraryVisualScenes->visualScenes = NULL;
    }
}

void Rcs_freeCollada(Collada* collada)
{
    if (collada->libraryGeometries)
    {
        Rcs_freeLibraryGeometries(collada->libraryGeometries);
        RFREE(collada->libraryGeometries);
        collada->libraryGeometries = NULL;
    }

    if (collada->libraryVisualScenes)
    {
        Rcs_freeColladaLibraryVisualScenes(collada->libraryVisualScenes);
        RFREE(collada->libraryVisualScenes);
        collada->libraryVisualScenes = NULL;
    }

    RFREE(collada);
    collada = NULL;
}

void Rcs_printParameter(ColladaParameter* parameter)
{
    printf("\nParameter name=%s, type=%s", parameter->name, parameter->type);
}

void printTechniqueCommon(ColladaTechniqueCommon* techniqueCommon)
{
    printf("\nTechniqueCommon accessorSource=%s, count=%d, stride=%d", techniqueCommon->accessorSource,
           techniqueCommon->count, techniqueCommon->stride);

    if (techniqueCommon->param)
    {
        for (int i = 0; i < (int) techniqueCommon->paramLen; ++i)
        {
            Rcs_printParameter(&techniqueCommon->param[i]);
        }
    }
}

void Rcs_printSource(ColladaSource* source)
{
    printf("\nSource id=%s, array=%ld, arrLen=%d", source->id, (long)source->array, source->arrLen);
    if (source->techniqueCommon)
    {
        printTechniqueCommon(source->techniqueCommon);
    }
}

void Rcs_printInput(ColladaInput* input)
{
    printf("\ninput semantic=%s, sourceId=%s, array=%ld, arrayLen=%d, offset=%d", input->semantic,
           input->sourceId, (long) input->array, input->arrayLen, input->offset);
}

void Rcs_printVertices(ColladaVertices* vertices)
{
    printf("\nVertices id=%s", vertices->id);
    if (vertices->input)
    {
        Rcs_printInput(vertices->input);
    }
}

void Rcs_printTriangles(ColladaTriangles* triangles)
{
    printf("\ntriangles material=%s, trianglesNum=%d, indexArray=%ld, indexArrLen=%d", triangles->materialId, triangles->trianglesNum,
           (long) triangles->indexArray, triangles->indexArrLen);
    if (triangles->inputs)
    {
        for (int i = 0; i < triangles->inputLen; ++i)
        {
            Rcs_printInput(&triangles->inputs[i]);
        }
    }
}

void Rcs_printMesh(ColladaMesh* mesh)
{
    printf("\nMesh: ");
    if (mesh->sources)
    {
        for (int i = 0; i < mesh->sourcesLen; ++i)
        {
            Rcs_printSource(&mesh->sources[i]);
        }
    }

    if (mesh->vertices)
    {
        Rcs_printVertices(mesh->vertices);
    }

    if (mesh->triangles)
    {
        Rcs_printTriangles(mesh->triangles);
    }
}

void Rcs_printGeometry(ColladaGeometry* geometry)
{
    printf("\nGeometry id=%s, name=%s", geometry->id, geometry->name);
    if (geometry->mesh)
    {
        Rcs_printMesh(geometry->mesh);
    }
}

void Rcs_printLibraryGeometries(ColladaLibraryGeometries* libraryGeometries)
{
    printf("\nLibraryGeometries: ");
    if (libraryGeometries->geometries) {
        for (int i = 0; i < libraryGeometries->geometriesLen; ++i)
        {
            Rcs_printGeometry(&libraryGeometries->geometries[i]);
        }
    }
}

void Rcs_printColladaMatrix(ColladaMatrix* matrix)
{
    char tmp[256];
    HTr_toString(tmp, &matrix->matrix);
    printf("\nMatrix sid=%s, matrix=%s", matrix->sid, tmp);
}

void Rcs_printInstanceGeometry(ColladaInstanceGeometry* instanceGeometry)
{
    printf("\nInstanceGeometry url=%s, name=%s", instanceGeometry->url, instanceGeometry->name);
}

void Rcs_printVisualNode(ColladaVisualNode* visualNode)
{
    printf("\nVisualNode id=%s, name=%s, type=%s", visualNode->id, visualNode->name, visualNode->type);
    if (visualNode->matrix)
    {
        Rcs_printColladaMatrix(visualNode->matrix);
    }
    if (visualNode->instanceGeometry)
    {
        Rcs_printInstanceGeometry(visualNode->instanceGeometry);
    }
}

void Rcs_printVisualScene(ColladaVisualScene* visualScene)
{
    printf("\nVisualScene id=%s, name=%s", visualScene->id, visualScene->name);
    if (visualScene->nodes)
    {
        for (int i = 0; i < visualScene->nodesLen; ++i)
        {
            Rcs_printVisualNode(&visualScene->nodes[i]);
        }
    }
}

void Rcs_printLibraryVisualScenes(ColladaLibraryVisualScenes* libraryVisualScenes)
{
    printf("\nLibraryVisualScenes: ");
    if (libraryVisualScenes->visualScenes) {
        for (int i = 0; i < libraryVisualScenes->visualScenesLen; ++i)
        {
            Rcs_printVisualScene(&libraryVisualScenes->visualScenes[i]);
        }
    }
}

void Rcs_printCollada(Collada* collada)
{
    printf("\nCollada: ");
    if (collada->libraryGeometries)
    {
        Rcs_printLibraryGeometries(collada->libraryGeometries);
    }

    if (collada->libraryVisualScenes)
    {
        Rcs_printLibraryVisualScenes(collada->libraryVisualScenes);
    }
    printf("\n");
}

bool Rcs_parseParameterNode(xmlNodePtr parameterNode, ColladaParameter* parameter)
{
    bool result = true;
    result &= (getXMLNodePropertyStringN(parameterNode, "name", parameter->name, 16) > 0);
    result &= (getXMLNodePropertyStringN(parameterNode, "type", parameter->type, 16) > 0);
    return result;
}

bool Rcs_parseTechniqueCommonNode(xmlNodePtr techiniqueCommonNode, ColladaTechniqueCommon* techniqueCommon)
{
    if (!techiniqueCommonNode)
    {
        RLOG(0, "technique_common node ptr invalid.");
        return false;
    }

    bool result = true;
    xmlNodePtr accessorNode = getXMLChildByName(techiniqueCommonNode, "accessor");
    if (!accessorNode)
    {
        RLOG(0, "can not parse accessor");
        return false;
    }
    result &= (getXMLNodePropertyStringN(accessorNode, "source", techniqueCommon->accessorSource, 128) > 0);
    result &= getXMLNodePropertyInt(accessorNode, "count", &techniqueCommon->count);
    result &= getXMLNodePropertyInt(accessorNode, "stride", &techniqueCommon->stride);
    if (!result)
    {
        RLOG(0, "parse accessor node faile");
        return false;
    }

    xmlNodePtr parameterNode = accessorNode->children;
    if (!parameterNode)
    {
        RLOG(0, "no parameter node");
        return result;
    }

    unsigned int parameterLen = getNumXMLNodes(parameterNode, "param");
    techniqueCommon->paramLen = (int) parameterLen;
    techniqueCommon->param = RNALLOC(parameterLen, ColladaParameter);
    unsigned int i = 0;

    while (parameterNode)
    {
        if (isXMLNodeNameNoCase(parameterNode, "param"))
        {
            result &= Rcs_parseParameterNode(parameterNode, &(techniqueCommon->param[i]));
            ++i;
        }
        parameterNode = parameterNode->next;
    }

    if ((i != parameterLen) || !result)
    {
        RLOG(0, "parse param node error.");
        return false;
    }

    return result;
}

bool Rcs_parseXMLNodeContentToDoubleArray(xmlNodePtr node, double* array, int arrLen)
{
    if (arrLen <= 0 || !array)
    {
        return false;
    }

    xmlChar* pNodeArrayStr = xmlNodeGetContent(node);
    bool tmpRes = String_toDoubleArray_l((char*) pNodeArrayStr, array, arrLen);
    xmlFree(pNodeArrayStr);
    if (!tmpRes)
    {
        RLOG(0, "convert int array failed.");
        return false;
    }

    return true;
}

bool Rcs_parseXMLNodeContentToIntArray(xmlNodePtr node, int* array, int arrLen)
{
    if (arrLen <= 0 || !array)
    {
        return false;
    }

    xmlChar* pNodeArrayStr = xmlNodeGetContent(node);
    bool tmpRes = String_toIntArray_l((char*) pNodeArrayStr, array, arrLen);
    xmlFree(pNodeArrayStr);
    if (!tmpRes)
    {
        RLOG(0, "convert int array failed.");
        return false;
    }

    return true;
}

bool Rcs_parseSourceNode(xmlNodePtr sourceNode, ColladaSource* source)
{
    bool result = true;
    result &= (getXMLNodePropertyStringN(sourceNode, "id", source->id, 128) > 0);
    if (!result)
    {
        RLOG(0, "parse source node id error.");
        return false;
    }
    xmlNodePtr techiniqueCommonNode = getXMLChildByName(sourceNode, "technique_common");

    source->techniqueCommon = NULL;
    source->techniqueCommon = RNALLOC(1, ColladaTechniqueCommon);
    if (!source->techniqueCommon)
    {
        RLOG(0, "allocate memory for technique_common failed.");
        return false;
    }

    result &= Rcs_parseTechniqueCommonNode(techiniqueCommonNode, source->techniqueCommon);
    if (!result)
    {
        RLOG(0, "parse techinique common node error.");
        return result;
    }

    xmlNodePtr floatArrNode = NULL;
    floatArrNode = getXMLChildByName(sourceNode, "float_array");
    if (!floatArrNode)
    {
        RLOG(0, "source node has no float array node.");
        return false;
    }

    source->arrLen = 0;
    result &= getXMLNodePropertyInt(floatArrNode, "count", &source->arrLen);
    source->array = NULL;
    source->array = RNALLOC(source->arrLen, double);
    if (!source->array)
    {
        RLOG(0, "allocate memory for source array failed.");
        return false;
    }

    result &= Rcs_parseXMLNodeContentToDoubleArray(floatArrNode, source->array, source->arrLen);
    if (!result)
    {
        RLOG(0, "parse source node float array error");
        return result;
    }

    return result;
}

bool Rcs_parseInputNode(xmlNodePtr inputNode, ColladaInput* input, ColladaSource* sources, int sourcesLen)
{
    bool result = true;
    char tmp[128];
    result &= (getXMLNodePropertyStringN(inputNode, "source", tmp, 128) > 0);
    result &= (getXMLNodePropertyStringN(inputNode, "semantic", input->semantic, 16) > 0);
    if (!result)
    {
        RLOG(0, "parse input node error");
        return result;
    }

    if (tmp[0] == '#')
    {
        strcpy(input->sourceId, &tmp[1]);
    }

    for (int i = 0; i < sourcesLen; ++i)
    {
        if (STREQ(input->sourceId, sources[i].id))
        {
            input->array = sources[i].array;  // link the input array to the source array.
            input->arrayLen = sources[i].arrLen;
        }
    }

    // offset is optional.
    getXMLNodePropertyInt(inputNode, "offset", &input->offset);

    return result;
}

bool Rcs_parseVerticesNode(xmlNodePtr verticesNode, ColladaVertices* vertices, ColladaSource* sources, int sourcesLen)
{
    bool result = true;
    result &= (getXMLNodePropertyStringN(verticesNode, "id", vertices->id, 128) > 0);
    if (!result)
    {
        RLOG(0, "vertices node has no id");
        return result;
    }

    xmlNodePtr inputNode = NULL;
    inputNode = getXMLChildByName(verticesNode, "input");
    if (!inputNode)
    {
        RLOG(0, "vertices node has no input");
        return false;
    }

    vertices->input = NULL;
    vertices->input = RNALLOC(1, ColladaInput);
    if (!vertices->input)
    {
        RLOG(0, "allocate memory for vertices input failed.");
        return false;
    }

    result &= Rcs_parseInputNode(inputNode, vertices->input, sources, sourcesLen);
    if (!result)
    {
        RLOG(0, "parse vertices node error, input node parsing error");
        return result;
    }

    return result;
}

bool Rcs_parseTrianglesNode(xmlNodePtr trianglesNode, ColladaTriangles* triangles, ColladaSource* sources, int sourcesLen, ColladaVertices* vertices)
{
    bool result = true;
    result &= (getXMLNodePropertyStringN(trianglesNode, "material", triangles->materialId, 128) > 0);
    result &= getXMLNodePropertyInt(trianglesNode, "count", &triangles->trianglesNum);
    if (!result)
    {
        RLOG(0, "parse triangles node property failed.");
        return result;
    }

    triangles->inputLen = getNumXMLNodes(trianglesNode->children, "input");
    triangles->inputs = RNALLOC(triangles->inputLen, ColladaInput);
    if (triangles->inputs)
    {
        int i = 0;
        xmlNodePtr inputNode = trianglesNode->children;
        while (inputNode)
        {
            if (isXMLNodeNameNoCase(inputNode, "input"))
            {
                result &= Rcs_parseInputNode(inputNode, &(triangles->inputs[i]), sources, sourcesLen);
                if (STREQ((triangles->inputs[i]).semantic, "VERTEX") && STREQ((triangles->inputs[i]).sourceId, vertices->id))
                {
                    if (vertices->input)
                    {
                        (triangles->inputs[i]).array = vertices->input->array;
                        (triangles->inputs[i]).arrayLen = vertices->input->arrayLen;
                    }
                    else
                    {
                        RLOG(0, "vertices input has no array");
                    }

                }
                ++i;
            }
            inputNode = inputNode->next;
        }
    }

    if (!result)
    {
        RLOG(0, "parse inputs error, in triangles node");
        RFREE(triangles->inputs);  // input does not hold the data, can directly be freed.
        triangles->inputs = NULL;
        return result;
    }


    triangles->indexArrLen = 0;
    triangles->indexArray = NULL;
    triangles->indexArrLen = triangles->inputLen * triangles->trianglesNum * 3;
    triangles->indexArray = RNALLOC(triangles->indexArrLen, int);
    if (!triangles->indexArray)
    {
        RLOG(0, "can not allocate memory for index array, length=%d", triangles->indexArrLen);
        return false;
    }

    // parse content in p node
    xmlNodePtr pNode = getXMLChildByName(trianglesNode, "p");
    result &= Rcs_parseXMLNodeContentToIntArray(pNode, triangles->indexArray, triangles->indexArrLen);
    if (!result)
    {
        RLOG(0, "can not read p node content");
        return result;
    }

    return result;
}

bool Rcs_parseMeshNode(xmlNodePtr meshNode, ColladaMesh* mesh)
{
    bool result = true;
    mesh->sourcesLen = 0;
    mesh->sourcesLen = getNumXMLNodes(meshNode->children, "source");
    mesh->sources = NULL;
    mesh->sources = RNALLOC(mesh->sourcesLen, ColladaSource);
    if (!mesh->sources)
    {
        RLOG(0, "can not allocate memory for Source");
        return false;
    }

    xmlNodePtr tmpNode = meshNode->children;
    int i = 0;
    // first handle sources
    while (tmpNode)
    {
        if (isXMLNodeNameNoCase(tmpNode, "source"))
        {
            result &= Rcs_parseSourceNode(tmpNode, &(mesh->sources[i]));
            ++i;
        }
        tmpNode = tmpNode->next;
    }
    if (!result)
    {
        RLOG(0, "parse souces failed.");
        return result;
    }

    // then parse vertices
    xmlNodePtr verticesNode = getXMLChildByName(meshNode, "vertices");
    mesh->vertices = NULL;
    mesh->vertices = RNALLOC(1, ColladaVertices);
    if (!mesh->vertices)
    {
        RLOG(0, "can not allocate memory for Vertices.");
        return false;
    }
    result &= Rcs_parseVerticesNode(verticesNode, mesh->vertices, mesh->sources, mesh->sourcesLen);
    if (!result)
    {
        RLOG(0, "parse vertices node failed.");
        return result;
    }

    // last parse triangles
    xmlNodePtr trianglesNode = getXMLChildByName(meshNode, "triangles");
    mesh->triangles = NULL;
    mesh->triangles = RNALLOC(1, ColladaTriangles);
    if (!mesh->triangles)
    {
        RLOG(0, "can not allocate memory for Triangles.");
        return false;
    }

    result &= Rcs_parseTrianglesNode(trianglesNode, mesh->triangles, mesh->sources, mesh->sourcesLen, mesh->vertices);
    if (!result)
    {
        RLOG(0, "parse triangles node failed.");
        return result;
    }


    return result;
}

bool Rcs_parseGeometryNode(xmlNodePtr geometryNode, ColladaGeometry* geometry)
{
    bool result = true;
    result &= (getXMLNodePropertyStringN(geometryNode, "id", geometry->id, 128) > 0);
    result &= (getXMLNodePropertyStringN(geometryNode, "name", geometry->name, 128) > 0);
    if (!result)
    {
        RLOG(0, "can not parse geometry node property.");
        return result;
    }

    xmlNodePtr meshNode = getXMLChildByName(geometryNode, "mesh");
    geometry->mesh = NULL;
    geometry->mesh = RNALLOC(1, ColladaMesh);
    if (!geometry->mesh)
    {
        RLOG(0, "can not allocate memory for Mesh.");
        return false;
    }
    result &= Rcs_parseMeshNode(meshNode, geometry->mesh);
    if (!result)
    {
        RLOG(0, "parse mesh node failed.");
        return result;
    }

    return result;
}

bool Rcs_parseLibraryGeometriesNode(xmlNodePtr libraryGeometriesNode, ColladaLibraryGeometries* libraryGeometries)
{
    bool result = true;
    libraryGeometries->geometriesLen = 0;
    libraryGeometries->geometries = NULL;
    libraryGeometries->geometriesLen = getNumXMLNodes(libraryGeometriesNode->children, "geometry");
    libraryGeometries->geometries = RNALLOC(libraryGeometries->geometriesLen, ColladaGeometry);
    if (!libraryGeometries->geometries)
    {
        RLOG(0, "can not allocate memory for Geometry");
        return false;
    }

    xmlNodePtr tmpNode = libraryGeometriesNode->children;
    int i = 0;
    while (tmpNode)
    {
        if (isXMLNodeNameNoCase(tmpNode, "geometry") && i < libraryGeometries->geometriesLen)
        {
            result &= Rcs_parseGeometryNode(tmpNode, &(libraryGeometries->geometries[i]));
            ++i;
        }
        tmpNode = tmpNode->next;
    }

    if (!result)
    {
        RLOG(0, "parse geometry node failed.");
        return result;
    }


    return result;
}

bool Rcs_parseMatrixNode(xmlNodePtr matrixNode, ColladaMatrix* matrix)
{
    bool result = true;
    result &= (getXMLNodePropertyStringN(matrixNode, "sid", matrix->sid, 16) > 0);
    if (!result)
    {
        RLOG(0, "can not parse matrix node property.");
        return result;
    }

    int arrLen = 16;
    double* array = RNALLOC(arrLen, double);
    if (!array)
    {
        RLOG(0, "allocate memory for matrix array failed.");
        return false;
    }

    result &= Rcs_parseXMLNodeContentToDoubleArray(matrixNode, array, arrLen);
    if (!result)
    {
        RLOG(0, "parse source node float array error");
        RFREE(array);
        return result;
    }

    // copy xyz transform
    for (int i = 0; i < 3; ++i)
    {
        matrix->matrix.org[i] = array[3 + 4 * i];
    }

    // copy 3x3 rotation matrix
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            matrix->matrix.rot[i][j] = array[i * 4 + j];
        }
    }

    RFREE(array);

    return result;
}

bool Rcs_parseInstanceGeometryNode(xmlNodePtr instanceGeometryNode, ColladaInstanceGeometry* instanceGeometry)
{
    bool result = true;
    char tmp[128];
    result &= (getXMLNodePropertyStringN(instanceGeometryNode, "url", tmp, 128) > 0);
    result &= (getXMLNodePropertyStringN(instanceGeometryNode, "name", instanceGeometry->name, 16) > 0);
    if (tmp[0] == '#')
    {
        strcpy(instanceGeometry->url, &tmp[1]);
    }
    return result;
}

bool Rcs_parseVisualNode(xmlNodePtr node, ColladaVisualNode* visualNode)
{
    bool result = true;
    result &= (getXMLNodePropertyStringN(node, "id", visualNode->id, 16) > 0);
    result &= (getXMLNodePropertyStringN(node, "name", visualNode->name, 16) > 0);
    result &= (getXMLNodePropertyStringN(node, "type", visualNode->type, 16) > 0);
    if (!result)
    {
        RLOG(0, "can not parse visual node property.");
        return result;
    }

    xmlNodePtr matrixNode = getXMLChildByName(node, "matrix");
    visualNode->matrix = NULL;
    visualNode->matrix = RNALLOC(1, ColladaMatrix);
    if (!visualNode->matrix)
    {
        RLOG(0, "can not allocate memory for visual node matrix.");
        return false;
    }
    result &= Rcs_parseMatrixNode(matrixNode, visualNode->matrix);
    if (!result)
    {
        RLOG(0, "parse matrix node failed.");
        return result;
    }

    xmlNodePtr instanceGeometryNode = getXMLChildByName(node, "instance_geometry");
    visualNode->instanceGeometry = NULL;
    visualNode->instanceGeometry = RNALLOC(1, ColladaInstanceGeometry);
    if (!visualNode->instanceGeometry)
    {
        RLOG(0, "can not allocate memory for instance geometry.");
        return false;
    }
    result &= Rcs_parseInstanceGeometryNode(instanceGeometryNode, visualNode->instanceGeometry);
    if (!result)
    {
        RLOG(0, "parse instance geometry node failed.");
        return result;
    }

    return result;
}

bool Rcs_parseVisualSceneNode(xmlNodePtr visualSceneNode, ColladaVisualScene* visualScene)
{
    bool result = true;
    result &= (getXMLNodePropertyStringN(visualSceneNode, "id", visualScene->id, 16) > 0);
    result &= (getXMLNodePropertyStringN(visualSceneNode, "name", visualScene->name, 16) > 0);
    if (!result)
    {
        RLOG(0, "can not parse visual scene node property.");
        return result;
    }

    visualScene->nodesLen = getNumXMLNodes(visualSceneNode->children, "node");
    visualScene->nodes = RNALLOC(visualScene->nodesLen, ColladaVisualNode);
    if (!visualScene->nodes)
    {
        RLOG(0, "can not allocate memory for visual node");
        return false;
    }

    xmlNodePtr tmpNode = visualSceneNode->children;
    int i = 0;
    while (tmpNode)
    {
        if (isXMLNodeNameNoCase(tmpNode, "node") && i < visualScene->nodesLen)
        {
            result &= Rcs_parseVisualNode(tmpNode, &(visualScene->nodes[i]));
            ++i;
        }
        tmpNode = tmpNode->next;
    }

    if (!result)
    {
        RLOG(0, "parse geometry node failed.");
        return result;
    }

    return result;
}

bool Rcs_parseLibraryVisualScenesNode(xmlNodePtr libraryVisualScenesNode, ColladaLibraryVisualScenes* libraryVisualScenes)
{
    bool result = true;
    libraryVisualScenes->visualScenesLen = 0;
    libraryVisualScenes->visualScenes = NULL;
    libraryVisualScenes->visualScenesLen = getNumXMLNodes(libraryVisualScenesNode->children, "visual_scene");
    libraryVisualScenes->visualScenes = RNALLOC(libraryVisualScenes->visualScenesLen, ColladaVisualScene);
    if (!libraryVisualScenes->visualScenes)
    {
        RLOG(0, "can not allocate memory for Visual Scenes");
        return false;
    }

    xmlNodePtr tmpNode = libraryVisualScenesNode->children;
    int i = 0;
    while (tmpNode)
    {
        if (isXMLNodeNameNoCase(tmpNode, "visual_scene") && i < libraryVisualScenes->visualScenesLen)
        {
            result &= Rcs_parseVisualSceneNode(tmpNode, &(libraryVisualScenes->visualScenes[i]));
            ++i;
        }
        tmpNode = tmpNode->next;
    }

    if (!result)
    {
        RLOG(0, "parse geometry node failed.");
        return result;
    }

    return result;
}

void Rcs_colladaVerticesTransform(ColladaGeometry* geometry, ColladaVisualNode* visualNode)
{
    if (!geometry || !visualNode)
        return;

    int verticesLen = geometry->mesh->vertices->input->arrayLen;
    double* vertices = geometry->mesh->vertices->input->array;
    HTr* tmpHTr = &visualNode->matrix->matrix;

    double tmpVec3d[4];
    double tmpRes[3] = {0};
    // transform every vertex. using homogenous matrix for intuition
    for (int i = 0; i < verticesLen / 3; ++i)
    {
        tmpVec3d[0] = vertices[3 * i];
        tmpVec3d[1] = vertices[3 * i + 1];
        tmpVec3d[2] = vertices[3 * i + 2];
        tmpVec3d[3] = 1.0;

        tmpRes[0] = tmpHTr->rot[0][0] * tmpVec3d[0]+ tmpHTr->rot[0][1] * tmpVec3d[1] + tmpHTr->rot[0][2] * tmpVec3d[2] + tmpHTr->org[0] * tmpVec3d[3];
        tmpRes[1] = tmpHTr->rot[1][0] * tmpVec3d[0]+ tmpHTr->rot[1][1] * tmpVec3d[1] + tmpHTr->rot[1][2] * tmpVec3d[2] + tmpHTr->org[1] * tmpVec3d[3];
        tmpRes[2] = tmpHTr->rot[2][0] * tmpVec3d[0]+ tmpHTr->rot[2][1] * tmpVec3d[1] + tmpHTr->rot[2][2] * tmpVec3d[2] + tmpHTr->org[2] * tmpVec3d[3];

        vertices[3 * i] = tmpRes[0];
        vertices[3 * i + 1] = tmpRes[1];
        vertices[3 * i + 2] = tmpRes[2];
    }
}

bool Rcs_applyColladaInternalTransformation(Collada* collada)
{
    if (collada->libraryGeometries->geometries && collada->libraryVisualScenes->visualScenes->nodes)
    {
        ColladaGeometry* tmpGeometry = NULL;
        ColladaVisualNode* tmpNode = NULL;
        for (int i = 0; i < collada->libraryGeometries->geometriesLen; ++i)
        {
            tmpGeometry = &collada->libraryGeometries->geometries[i];
            for (int j = 0; j < collada->libraryVisualScenes->visualScenes[0].nodesLen; ++j)  // hard coded to first visual scene
            {
                tmpNode = &(collada->libraryVisualScenes->visualScenes[0].nodes[j]);
                if (STREQ(tmpGeometry->id, tmpNode->instanceGeometry->url))
                {
                    RLOG(4, "apply vertices transformation for geometry id=%s", tmpGeometry->id);
                    Rcs_colladaVerticesTransform(tmpGeometry, tmpNode);
                }
            }
        }
    }
    else
    {
        RLOG(0, "failed to apply transformation");
        return false;
    }

    return true;
}

bool Rcs_parseColladaNode(xmlNodePtr colladaNode, Collada* collada)
{
    bool result = true;
    xmlNodePtr libraryGeometriesNode = getXMLChildByName(colladaNode, "library_geometries");
    collada->libraryGeometries = NULL;
    collada->libraryGeometries = RNALLOC(1, ColladaLibraryGeometries);
    if (!collada->libraryGeometries)
    {
        RLOG(0, "can not allocate memory for LibraryGeometries");
        return false;
    }
    result &= Rcs_parseLibraryGeometriesNode(libraryGeometriesNode, collada->libraryGeometries);
    if (!result)
    {
        RLOG(0, "parse collada node failed.");
        return false;
    }

    xmlNodePtr libraryVisualScenesNode = getXMLChildByName(colladaNode, "library_visual_scenes");
    collada->libraryVisualScenes = NULL;
    collada->libraryVisualScenes = RNALLOC(1, ColladaLibraryVisualScenes);
    if (!collada->libraryVisualScenes)
    {
        RLOG(0, "can not allocate memory for LibraryVisualScenes");
        return false;
    }
    result &= Rcs_parseLibraryVisualScenesNode(libraryVisualScenesNode, collada->libraryVisualScenes);
    if (!result)
    {
        RLOG(0, "parse collada node failed.");
        return false;
    }

    result &= Rcs_applyColladaInternalTransformation(collada);
    if (!result)
    {
        RLOG(0, "can not applay collada internal transformation.");
        return false;
    }

    return result;
}


bool Rcs_parseDAEFile(const char* filename, Collada** collada)
{
    bool result = true;
    xmlDocPtr doc;
    xmlNodePtr colladaNode = NULL;
    colladaNode = parseXMLFile(filename, "COLLADA", &doc);
    if (!colladaNode)
    {
        RLOG(0, "can not parse the collada file:%s", filename);
        return false;
    }

    *collada = RNALLOC(1, Collada);
    if (!(*collada))
    {
        RLOG(0, "can not allocate memory for Collada object");
        xmlFreeDoc(doc);
        return false;
    }

    result &= Rcs_parseColladaNode(colladaNode, *collada);
    if (!result)
    {
        RLOG(0, "parse collada node error.");
        Rcs_freeCollada(*collada);
        RFREE(*collada);
        *collada = NULL;
        xmlFreeDoc(doc);
        return result;
    }

    xmlFreeDoc(doc);
    return result;
}

int Rcs_getColladaTotalVerticesNum(Collada* collada)
{
    int count = 0;
    ColladaInput* tmpInputs = NULL;
    ColladaGeometry* tmpGeometry = NULL;
    if (collada->libraryGeometries->geometries)
    {
        for (int i = 0; i < collada->libraryGeometries->geometriesLen; ++i)
        {
            tmpGeometry = &collada->libraryGeometries->geometries[i];
            if (tmpGeometry && tmpGeometry->mesh && tmpGeometry->mesh->triangles->inputs)
            {
                tmpInputs = tmpGeometry->mesh->triangles->inputs;
                for (int j = 0; j < tmpGeometry->mesh->triangles->inputLen; ++j)
                {
                    if (STREQ(tmpInputs[j].semantic, "VERTEX"))
                    {
                        count += tmpInputs[j].arrayLen / 3;
                    }
                }
            }
        }

    }
    return count;
}

int Rcs_getColladaTotalTrianglesNum(Collada* collada)
{
    int count = 0;
    if (collada->libraryGeometries->geometries)
    {
        ColladaGeometry* tmpGeometries = collada->libraryGeometries->geometries;
        for (int i = 0; i < collada->libraryGeometries->geometriesLen; ++i)
        {
            count += tmpGeometries[i].mesh->triangles->trianglesNum;
        }
    }
    return count;
}

bool Rcs_parseMeshDataFromCollada(RcsMeshData* meshData, Collada* collada)
{
    bool result = true;
    if (!meshData || !collada)
    {
        return false;
    }

    meshData->nVertices = Rcs_getColladaTotalVerticesNum(collada);
    meshData->nFaces = Rcs_getColladaTotalTrianglesNum(collada);

    meshData->vertices = RNALLOC(meshData->nVertices * 3, double);
    meshData->faces = RNALLOC(meshData->nFaces * 3, unsigned int);

    RLOG(0, "vertices=%u, totalTriangles=%u", meshData->nVertices, meshData->nFaces);

    if (!meshData->vertices || !meshData->faces)
    {
        RLOG(0, "can not allocate memory for RcsMeshData");
        RFREE(meshData->vertices);
        RFREE(meshData->faces);
        meshData->vertices = NULL;
        meshData->faces = NULL;
        return false;
    }

    // copy vertices
    unsigned int count = 0;
    ColladaInput* tmpInputs = NULL;
    ColladaGeometry* tmpGeometry = NULL;
    if (collada->libraryGeometries->geometries)
    {
        for (int i = 0; i < collada->libraryGeometries->geometriesLen; ++i)
        {
            tmpGeometry = &collada->libraryGeometries->geometries[i];
            if (tmpGeometry && tmpGeometry->mesh && tmpGeometry->mesh->triangles->inputs)
            {
                tmpInputs = tmpGeometry->mesh->triangles->inputs;
                for (int j = 0; j < tmpGeometry->mesh->triangles->inputLen; ++j)
                {
                    if (STREQ(tmpInputs[j].semantic, "VERTEX"))
                    {
                        for (int id = 0; id < tmpInputs[j].arrayLen; ++id)
                        {
                            meshData->vertices[id + count] = tmpInputs[j].array[id];
                        }
                        count += tmpInputs[j].arrayLen;
                    }
                }
            }
        }
    }

    if (count != meshData->nVertices * 3)
    {
        RLOG(0, "copy vertices error");
        RFREE(meshData->vertices);
        RFREE(meshData->faces);
        meshData->vertices = NULL;
        meshData->faces = NULL;
        return false;
    }

    // copy faces.
    count = 0;
    int offset = 0;
    ColladaTriangles* tmpTriangles = NULL;
    tmpGeometry = NULL;
    if (collada->libraryGeometries->geometries)
    {
        for (int i = 0; i < collada->libraryGeometries->geometriesLen; ++i)
        {
            tmpGeometry = &collada->libraryGeometries->geometries[i];
            if (tmpGeometry && tmpGeometry->mesh && tmpGeometry->mesh->triangles)
            {
                tmpTriangles = tmpGeometry->mesh->triangles;
                unsigned int* tmpTargetArray = &meshData->faces[count];
                for (unsigned int id = 0; id < (unsigned int) tmpTriangles->indexArrLen / tmpTriangles->inputLen; ++id)
                {
                    tmpTargetArray[id] = tmpTriangles->indexArray[id * tmpTriangles->inputLen] + offset;
                }
                count += (unsigned int) tmpTriangles->indexArrLen / tmpTriangles->inputLen;

                tmpInputs = tmpGeometry->mesh->triangles->inputs;
                for (int j = 0; j < tmpGeometry->mesh->triangles->inputLen; ++j)
                {
                    if (STREQ(tmpInputs[j].semantic, "VERTEX"))
                    {
                        offset += tmpInputs[j].arrayLen / 3;
                    }
                }
            }
        }
    }

    if (count != meshData->nFaces * 3)
    {
        RLOG(0, "copy faces index error");
        RFREE(meshData->vertices);
        RFREE(meshData->faces);
        meshData->vertices = NULL;
        meshData->faces = NULL;
        return false;
    }

    return result;
}
