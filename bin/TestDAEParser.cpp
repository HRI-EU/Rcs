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
**  Date: 5 Dec 2022
**
**************************************************************************/

#include <Rcs_macros.h>
#include <Rcs_utils.h>
#include <Rcs_utilsCPP.h>
#include <Rcs_resourcePath.h>
#include <Rcs_cmdLine.h>
#include <Rcs_colladaParser.h>
#include <Rcs_mesh.h>

#include <string>
#include <iostream>
#include <stdlib.h>
#include <string.h>


int main(int argc, char** argv)
{
    const char* directory = "config/xml/DAEExample/visual";
    const char* daeXmlFile = "finger.dae";
    Rcs_addResourcePath("config");
    Rcs_addResourcePath(directory);

    char filename[256] = "";
    bool fileExists = Rcs_getAbsoluteFileName(daeXmlFile, filename);
    if (!fileExists) {
        RLOG(0, "file: '%s' not exists", daeXmlFile);
        exit(1);
    }

    Collada* colladaObj = NULL;
    bool result = false;
    result = Rcs_parseDAEFile(filename, &colladaObj);
    if (result && colladaObj)
    {
        if (colladaObj)
        {
            RLOG(0, "parse dae file succeed");
        }
    }
    else
    {
        RLOG(0, "parse dae file error.");
    }

    // printCollada(colladaObj);
    printf("\n");

    RcsMeshData meshData;
    meshData.faces = NULL;
    meshData.vertices = NULL;
    meshData.nFaces = 0;
    meshData.nVertices = 0;

    result = Rcs_parseMeshDataFromCollada(&meshData, colladaObj);
    if (!result)
    {
        RLOG(0, "parseColladaIndexArrToVerticesArr Error");
    }
    RLOG(0,"totalVertices=%d, totalTriangles=%d", meshData.nVertices, meshData.nFaces);
    RLOG(0, "vertices=0x%X, totalTriangles=0x%X", meshData.vertices, meshData.faces);

    Rcs_freeCollada(colladaObj);
    RFREE(meshData.vertices);
    RFREE(meshData.faces);
    colladaObj = NULL;

    return 0;
}
