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
**  Date: 20 Dec 2022
**
**************************************************************************/


#include <Rcs_macros.h>
#include <Rcs_utils.h>
#include <Rcs_utilsCPP.h>
#include <Rcs_timer.h>
#include <Rcs_resourcePath.h>
#include <Rcs_cmdLine.h>

#include <Rcs_typedef.h>
#include <Rcs_graph.h>
#include <Rcs_body.h>
#include <Rcs_joint.h>
#include <Rcs_shape.h>


int main(int argc, char** argv)
{
    // Parse command line arguments
    char handName[32];
    char handBaseName[128];
    char directory[256];
    char fileName[128];
    char dotFile[256] = "RcsGraph.dot";

    Rcs::CmdLineParser argP(argc, argv);
    argP.getArgument("-dl", &RcsLogLevel, "Debug level (default is 0)");
    argP.getArgument("-hand", handName, "Hand Name.");  // hand name will be used as suffix.
    argP.getArgument("-base", handBaseName, "Hand base link name.");
    argP.getArgument("-dir", directory, "files directory.");
    argP.getArgument("-f", fileName, "file Name.");

    Rcs_addResourcePath("config");
    Rcs_addResourcePath(directory);

    // create graph from controller XML file.
    RcsGraph* graph = RcsGraph_create(fileName);

    // append dummy hand base to the graph. This is the mounting point for the extracted part.
    RcsBody* dummyBody = RALLOC(RcsBody);
    dummyBody->name = String_clone("dummy_right_hand_base");
    dummyBody->A_BI = HTr_create();
    //dummyBody->A_BP = HTr_create();
    dummyBody->Inertia = HTr_create();
    HTr_setZero(dummyBody->Inertia);

    // attach the dummy hand base to the graph.
    RcsGraph_insertBody(graph, NULL, dummyBody);


    // extract subgraph.
    RcsBody* handBaseBody = RcsGraph_getBodyByName(graph, handBaseName);

    // TODO: relative transformation between robotiq base link and root link is unclear. How to define it?
    // see    <!-- Rotate hand relative to link_frame -->
    //        <joint name="hand_base_target_joint" type="fixed">
    //        <parent link="target_robotiq_root_link"/>
    //        <child link="target_robotiq_arg2f_base_link"/>
    //        <origin rpy="-1.5707963267948966 0 -1.5707963267948966" xyz="0.0 0.0 0.0"/>
    //        <!-- if you want to change the offset, change it in negative x-direction -->
    //        </joint>
    RcsGraph* subGraph = RcsGraph_cloneSubGraph(graph, handBaseBody);

    // check for illegal coupling.
    RCSGRAPH_TRAVERSE_JOINTS(subGraph)
    {
        if (JNT->coupledJointName)
        {
            RcsJoint* tmp = RcsGraph_getJointByName(subGraph, JNT->coupledJointName);
            if (!tmp)
            {
                RMSG("ERROR, no coupled joint name=%s", JNT->coupledJointName);
            }
            else if (tmp && tmp != JNT->coupledTo)
            {
                RMSG("ERROR, wrong coupling: %s(should)->%s(is)", JNT->coupledJointName, JNT->coupledTo->name);
            }
        }
    }

    RMSG("Graph dof=%u, nJ=%u, before appending", graph->dof, graph->nJ);

    // create RBJ for subgraph. Before attaching the subgraph to other graph, user should create the joint he needs.
    double tmpQVec[6] = {0};
    subGraph->root->rigid_body_joints = true;
    RcsBody_createRBJ(subGraph, subGraph->root, tmpQVec);

    // attach the subgraph to the graph.
    RcsBody* target = RcsGraph_getBodyByName(graph, "dummy_right_hand_base");
    const size_t suffixLen = strlen(handName) + 2;
    char* suffix = RNALLOC(suffixLen, char);
    sprintf(suffix, "_%s", handName);
    bool result = RcsGraph_appendCopyOfGraph(graph, target, subGraph, suffix, NULL);
    if (!result)
    {
        RMSG("append subgraph failed.");
    }
    RcsGraph_makeJointsConsistent(graph);

    RMSG("Graph dof=%u, nJ=%u", graph->dof, graph->nJ);
    RMSG("Subgraph dof=%u, nJ=%u", subGraph->dof, subGraph->nJ);

    RMSG("Here's the forward tree:");
    RcsGraph_fprint(stderr, graph);

    RMSG("Writing graph to dot file \"RcsGraph.dot\"");
    RcsGraph_writeDotFile(graph, "RcsGraph.dot");

    RMSG("Writing graph to xml file \"graph.xml\"");
    FILE* out = fopen("graph.xml", "w+");
    RcsGraph_fprintXML(out, graph);
    fclose(out);

    RcsGraph_destroy(subGraph);
    RcsGraph_destroy(graph);

    std::string dottyCommand = "dotty " + std::string(dotFile) + "&";
    int err = system(dottyCommand.c_str());

    if (err == -1)
    {
        RMSG("Couldn't start dot file viewer!");
    }

    return 0;
}
