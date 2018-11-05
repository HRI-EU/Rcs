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

#include <Rcs_macros.h>
#include <Rcs_utils.h>
#include <Rcs_utilsCPP.h>
#include <Rcs_timer.h>
#include <Rcs_parser.h>
#include <Rcs_resourcePath.h>
#include <Rcs_cmdLine.h>
#include <Rcs_mesh.h>

#include <SegFaultHandler.h>

#include <sstream>
#include <string>
#include <stdio.h>
#include <locale.h>

RCS_INSTALL_SEGFAULTHANDLER

bool runLoop = true;



/******************************************************************************
 * Ctrl-C destructor. Tries to quit gracefully with the first Ctrl-C
 * press, then just exits.
 *****************************************************************************/
static void quit(int /*sig*/)
{
  static int kHit = 0;
  runLoop = false;
  fprintf(stderr, "Trying to exit gracefully - %dst attempt\n", kHit + 1);
  kHit++;

  if (kHit == 2)
  {
    fprintf(stderr, "Exiting without cleanup\n");
    exit(0);
  }
}

/******************************************************************************
 *
 *****************************************************************************/
static bool test_uniqueFileName()
{
  char fileName[256];
  char pattern[64] = "/tmp/myTempFile";
  char suffix[64] = "dat";
  char* namePtr = NULL;
  bool success = false;

  Rcs::CmdLineParser argP;
  bool hasP = argP.getArgument("-template", pattern,
                               "File template for unique file name");
  bool hasS = argP.getArgument("-suffix", suffix, "File suffix");

  if (argP.hasArgument("-h"))
  {
    argP.print();
    return false;
  }

  namePtr = File_createUniqueName(fileName, hasP ? pattern : NULL,
                                  hasS ? suffix : NULL);

  RLOGS(1, "%s: unique file name is %s (Template: \"%s\", suffix: \"%s\")",
        namePtr ? "SUCCESS" : "FAILURE", namePtr ? namePtr : "NULL",
        hasP ? pattern : "NULL", hasS ? suffix : "NULL");

  if (namePtr != NULL)
  {
    FILE* testFile = fopen(namePtr, "w+");

    if (testFile != NULL)
    {
      RLOGS(1, "Successfully opened file \"%s\"", namePtr);
      fclose(testFile);
      success = true;
    }
    else
    {
      RLOGS(1, "Couldn't open file \"%s\"", namePtr);
      success = false;
    }
  }

  return success;
}

/******************************************************************************
 *
 *****************************************************************************/
static bool test_compareEnding()
{
  char str[256] = "";
  char ending[256] = "";
  Rcs::CmdLineParser argP;
  argP.getArgument("-str", str, "String to compare against ending");
  argP.getArgument("-ending", ending, "Ending");
  bool caseSensitive = argP.hasArgument("-case", "Check case sensitive");

  RLOG(5, "Comparing \"%s\" against ending \"%s\"", str, ending);

  return String_hasEnding(str, ending, caseSensitive);
}

/******************************************************************************
 *
 *****************************************************************************/
static bool test_mesh()
{
  std::string meshFile;

  const char* hgr = getenv("SIT");

  if (hgr != NULL)
  {
    meshFile = std::string(hgr) + std::string("/Data/RobotMeshes/1.0/data/") +
               std::string("iiwa_description/meshes/iiwa14/visual/link_1.stl");
  }

  Rcs::CmdLineParser argP;
  argP.getArgument("-f", &meshFile, "Name of mesh file (default is %s)",
                   meshFile.c_str());

  double t = Timer_getTime();
  RcsMeshData* mesh1 = RcsMesh_createFromFile(meshFile.c_str());
  t = Timer_getTime() - t;
  RCHECK(mesh1);
  RLOG(0, "[%.3f msec]: Mesh has %u vertices and %u faces",
       1000.0*t, mesh1->nVertices, mesh1->nFaces);

  t = Timer_getTime();
  RcsMesh_compressVertices(mesh1, 1.0e-8);
  t = Timer_getTime() - t;
  RLOG(0, "[%.3f msec]: Compressed mesh has %u vertices and %u faces",
       t, mesh1->nVertices, mesh1->nFaces);
  RcsMesh_toFile(mesh1, "mesh1.tri");

  t = Timer_getTime();
  RcsMeshData* mesh2 = RcsMesh_createFromFile("mesh1.tri");
  RCHECK(mesh2);
  t = Timer_getTime() - t;
  RLOG(0, "[%.3f msec]: Reloaded mesh has %u vertices and %u faces",
       t, mesh2->nVertices, mesh2->nFaces);

  RcsMesh_destroy(mesh1);
  RcsMesh_destroy(mesh2);

  return true;
}

/******************************************************************************
 *
 *****************************************************************************/
static bool test_envString()
{
  const char* testStr = "${HOME}/test1/${USER}/bin";

  char* expanded = String_expandEnvironmentVariables(testStr);
  RMSG("Expanded \"%s\"", expanded);
  RFREE(expanded);
  return true;
}

/******************************************************************************
 * 
 *****************************************************************************/
static bool test_localeFreeParsing()
{
  char str[256] = "3.1415926535";
  int nIter = 1;
  Rcs::CmdLineParser argP;
  argP.getArgument("-str", str, "String to convert (default is %s)", str);
  argP.getArgument("-iter", &nIter, "Iterations (default is %d)", nIter);

  if (argP.hasArgument("-comma", "Set locale (de_DE.utf8)"))
  {
    char* res = setlocale(LC_ALL, "de_DE.utf8");
    RLOG(1, "setlocale() returned \"%s\"", res ? res : "NULL");
  }

  struct lconv* loc = localeconv();

  RLOG(1, "Decimal character is %c", *(loc->decimal_point));
  RLOG(1, "String to convert to double: \"%s\"", str);


  // Locale conversion with sscanf
  {
    double b, dt = Timer_getSystemTime();
    for (int i=0; i<nIter; ++i)
    {
      sscanf(str, "%lf", &b);
    }
    dt = Timer_getSystemTime() - dt;
    RLOG(1, "sscanf: double value is %f ... took %.4f usec", 
         b, (1.0/nIter)*1.0e6*dt);
  }


  // Locale-free conversion with strtod_l
  {
    double res, dt = Timer_getSystemTime();
    for (int i=0; i<nIter; ++i)
    {
      res = String_toDouble_l(str);
    }
    dt = Timer_getSystemTime() - dt;
    RLOG(1, "strtod_l: double value is %f ... took %.4f usec", 
         res, (1.0/nIter)*1.0e6*dt);
  }


  // Locale-free conversion with stringstream
  {
    double a, dt = Timer_getSystemTime();
    for (int i=0; i<nIter; ++i)
    {
      std::stringstream ss2;
      ss2.imbue(std::locale::classic());
      ss2 << str;
      ss2 >> a;
    }
    dt = Timer_getSystemTime() - dt;
    RLOG(1, "stringstream with imbue: double value is %f ... took %.4f usec", 
         a, (1.0/nIter)*1.0e6*dt);
  }


  // Locale-free double to string conversion
  {
    double num = -12345.1239456789;
    int ndigits = 3;
    argP.getArgument("-num", &num);
    argP.getArgument("-digits", &ndigits);
    char sir[64];
    String_fromDouble(sir, num, ndigits);
    RLOG(1, "str=%s" , sir);
  }

  return true;
}


/******************************************************************************
 *
 *****************************************************************************/
int main(int argc, char** argv)
{
  int mode = 0;
  bool success = false;

  // Ctrl-C callback handler
  signal(SIGINT, quit);

  // This initialize the xml library and check potential mismatches between
  // the version it was compiled for and the actual shared library used.
  LIBXML_TEST_VERSION;

  // Parse command line arguments
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-dl", &RcsLogLevel, "Debug level (default is 0)");
  argP.getArgument("-m", &mode, "Test mode");


  switch (mode)
  {
    case 0:
    {
      fprintf(stderr, "\n\tHere's some useful testing modes:\n\n");
      fprintf(stderr, "\t-dl\t<0...>   Sets the debug level (default 0)\n");
      fprintf(stderr, "\t-m");
      fprintf(stderr, "\t0   Prints this message (default)\n");
      fprintf(stderr, "\t\t1   Unique file name test\n");
      fprintf(stderr, "\t\t2   String ending comparison test\n");
      fprintf(stderr, "\t\t3   Mesh test\n");
      fprintf(stderr, "\t\t4   String environment expansion test\n");
      fprintf(stderr, "\t\t5   Locale-independent parsing test\n");
      fprintf(stderr, "\n\nResource path:\n");
      Rcs_printResourcePath();
      break;
    }

    case 1:
      test_uniqueFileName();
      break;

    case 2:
    {
      success = test_compareEnding();
      break;
    }

    case 3:
    {
      success = test_mesh();
      break;
    }

    case 4:
    {
      success = test_envString();
      break;
    }

    case 5:
    {
      success = test_localeFreeParsing();
      break;
    }

    default:
    {
      RMSG("there is no mode %d", mode);
    }

  }   // switch(mode)

  RMSGS("Test %s", success ? "SUCCEEDED" : "FAILED");

  if (argP.hasArgument("-h"))
    {
      argP.print();
    }

  // Clean up global stuff. From the libxml2 documentation:
  // WARNING: if your application is multithreaded or has plugin support
  // calling this may crash the application if another thread or a plugin is
  // still using libxml2. It's sometimes very hard to guess if libxml2 is in
  // use in the application, some libraries or plugins may use it without
  // notice. In case of doubt abstain from calling this function or do it just
  // before calling exit() to avoid leak reports from valgrind !
  xmlCleanupParser();

  fprintf(stderr, "Thanks for using the RcsCore test\n");

  return 0;
}
