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

#include <Rcs_macros.h>
#include <Rcs_utils.h>
#include <Rcs_utilsCPP.h>
#include <Rcs_timer.h>
#include <Rcs_resourcePath.h>
#include <Rcs_cmdLine.h>

#include <fstream>
#include "URDFGenerator/URDFGenerator.h"

const char* urdfExtension = ".urdf";

static bool integrated_test(const char* xmlFileName, const char* directory, const char* outputFile, int withFloatingJoints)
{
    if (!xmlFileName && ! directory) {
        return false;
    }

    Rcs_addResourcePath(directory);

    RcsGraph* graph = RcsGraph_create(xmlFileName);
    if (!graph) {
        RFATAL("graph is empty.");
        return false;
    }

    Rcs::URDFGenerator urdf(graph);
    if (withFloatingJoints > 0) {
        urdf.setFloatingJoints(true);
    }

    if (!urdf.toString().empty()) {
        std::string outputFileName = std::string(outputFile) + urdfExtension;
        std::ofstream output(outputFileName);
        if (output.is_open()) {
            output << urdf.toString() << std::endl;
            return true;
        }
        else {
            RFATAL("can not write urdf to file=%s", outputFile);
        }
    }

    return false;
}

static bool testMode(int mode, int argc, char** argv)
{
    bool success = true;

    switch (mode)
    {
        case 0:
        {
            fprintf(stderr, "\n\tHere's some useful testing modes:\n\n");
            fprintf(stderr, "\t-dl\t<0...>   Sets the debug level (default 0)\n");
            fprintf(stderr, "\t-dir\t<directory>   Working directory \n");
            fprintf(stderr, "\t-f\t<filename>   XML configuration file \n");
            fprintf(stderr, "\t-out\t<filename>   generated URDF file \n");
            fprintf(stderr, "\t-fj\t<0 or 1>   with floating joints or not \n");
            fprintf(stderr, "\t-m");
            fprintf(stderr, "\t0   Prints this message (default)\n");
            fprintf(stderr, "\t\t1   URDF generator class test\n");
            fprintf(stderr, "\t\t2   URDF Element and Attributes test\n");
            fprintf(stderr, "\t\t3   integrated test\n");
            fprintf(stderr, "\n\nResource path:\n");
            Rcs_printResourcePath();
            break;
        }

        case 1:
        {
            // test nullptr
            RcsGraph* graph = nullptr;
            Rcs::URDFGenerator urdf(graph);
            if (urdf.toString() == "NULL") {
                success &= true;
            }
            break;
        }

        case 2:
        {
            // 1 level
            Rcs::URDFElement ele1("test");
            std::string targetStr1 = "<test/>";
            success &= (targetStr1 == ele1.toString());
            RLOG(1, "\ntestStr=%s, \ntargetStr=%s, \nresult=%i", ele1.toString().c_str(), targetStr1.c_str(), (targetStr1 == ele1.toString()));

            // 1 level with multiple Attributes
            Rcs::URDFElement ele2("test");
            ele2.addAttribute("attr1", "value");
            ele2.addAttribute("attr2", "value");
            std::string targetStr2 = "<test attr1=\"value\" attr2=\"value\"/>";
            success &= (targetStr2 == ele2.toString());
            RLOG(1, "\ntestStr=%s, \ntargetStr=%s, \nresult=%i", ele2.toString().c_str(), targetStr2.c_str(), (targetStr2 == ele2.toString()));

            // 1 level with 1 Attribute
            Rcs::URDFElement ele3("test");
            ele3.addAttribute("tag", "value");
            std::string targetStr3 = "<test tag=\"value\"/>";
            success &= (targetStr3 == ele3.toString());
            RLOG(1, "\ntestStr=%s, \ntargetStr=%s, \nresult=%i", ele3.toString().c_str(), targetStr3.c_str(), (targetStr3 == ele3.toString()));

            // 2 level
            auto outer = std::unique_ptr<Rcs::URDFElement>(new Rcs::URDFElement("outer"));
            auto inner = std::unique_ptr<Rcs::URDFElement>(new Rcs::URDFElement("inner"));
            outer->addSubElement(std::move(inner));
            std::string targetStr4="<outer>\n  <inner/>\n</outer>";
            success &= (targetStr4 == outer->toString());
            RLOG(1, "\ntestStr=%s, \ntargetStr=%s, \nresult=%i", outer->toString().c_str(), targetStr4.c_str(), (targetStr4 == outer->toString()));

            // 2 level with Attributes
            auto outer1 = std::unique_ptr<Rcs::URDFElement>(new Rcs::URDFElement("outer1"));
            outer1->addAttribute("testAttr", "value");
            auto inner1 = std::unique_ptr<Rcs::URDFElement>(new Rcs::URDFElement("inner1"));
            inner1->addAttribute("testAttr", "value");
            outer1->addSubElement(std::move(inner1));
            std::string targetStr5="<outer1 testAttr=\"value\">\n  <inner1 testAttr=\"value\"/>\n</outer1>";
            success &= (targetStr5 == outer1->toString());
            RLOG(1, "\ntestStr=%s, \ntargetStr=%s, \nresult=%i", outer1->toString().c_str(), targetStr5.c_str(), (targetStr5 == outer1->toString()));

            // 3 level with Attributes
            auto level_1 = std::unique_ptr<Rcs::URDFElement>(new Rcs::URDFElement("level_1"));
            auto level_2 = std::unique_ptr<Rcs::URDFElement>(new Rcs::URDFElement("level_2"));
            auto level_3 = std::unique_ptr<Rcs::URDFElement>(new Rcs::URDFElement("level_3"));
            level_1->addAttribute("attr", "value");
            level_2->addAttribute("attr", "value");
            level_3->addAttribute("attr", "value");

            level_2->addSubElement(std::move(level_3));
            level_1->addSubElement(std::move(level_2));
            std::string targetStr6 = "<level_1 attr=\"value\">\n  <level_2 attr=\"value\">\n    <level_3 attr=\"value\"/>\n  </level_2>\n</level_1>";
            success &= (targetStr6 == level_1->toString());
            RLOG(1, "\ntestStr=%s, \ntargetStr=%s, \nresult=%i", level_1->toString().c_str(), targetStr6.c_str(), (targetStr6 == level_1->toString()));

            break;
        }

        case 3:
        {
            Rcs::CmdLineParser argP(argc, argv);
            char xmlFileName[128] = "", directory[128] = "", outputFile[128]="";
            int withFloatingJoints = 0;
            argP.getArgument("-f", xmlFileName, "RcsGraph's configuration file name");
            argP.getArgument("-dir", directory, "Configuration file directory");
            argP.getArgument("-out", outputFile, "output file name of generated URDF file, without file extension");
            argP.getArgument("-fj", &withFloatingJoints, "Rcs rigid body joints will be exported as URDF floating joints");
            success = integrated_test(xmlFileName, directory, outputFile, withFloatingJoints);
            break;
        }

        default:
        {
            RMSG("there is no mode %d", mode);
        }

    }   // switch(mode)

    return success;
}

int main(int argc, char** argv)
{
    // Parse command line arguments
    int mode = 0, result = 0;
    bool success = true;

    Rcs::CmdLineParser argP(argc, argv);
    argP.getArgument("-dl", &RcsLogLevel, "Debug level (default is 0)");
    argP.getArgument("-m", &mode, "Test mode");

    Rcs_addResourcePath("config");

    if (mode == -1)
    {
        for (int i = 1; i <= 3; ++i)
        {
            bool success_i = testMode(i, argc, argv);
            if (!success_i)
            {
                result++;
            }

            success = success_i && success;
        }
    }
    else
    {
        success = testMode(mode, argc, argv);
    }

    if (argP.hasArgument("-h"))
    {
        argP.print();
    }
    else
    {
        RLOGS(1, "TestURDF test %s", success ? "SUCCEEDED" : "FAILED");
    }

    fprintf(stderr, "Thanks for using the TestURDF program\n");

    return result > 0 ? result : 0;
}
