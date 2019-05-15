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
static bool test_stringSplit(std::string src, std::string delim)
{
  RMSG("Original string: \"%s\"", src.c_str());
  std::vector<std::string> split = String_split(src, delim);

  for (size_t i=0; i<split.size(); ++i)
  {
    RMSG("substring[%d] = \"%s\"", (int) i, split[i].c_str());
  }

  RMSG("Remaining string: \"%s\"", src.c_str());

  return true;
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
  char outFile[256] = "mesh1.tri";

  const char* hgr = getenv("SIT");

  if (hgr != NULL)
  {
    meshFile = std::string(hgr) + std::string("/Data/RobotMeshes/1.0/data/") +
               std::string("iiwa_description/meshes/iiwa14/visual/link_1.stl");
  }

  Rcs::CmdLineParser argP;
  argP.getArgument("-outFile", outFile, "Name of mesh file to be written "
                   "(default is %s)", meshFile.c_str());
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
  RcsMesh_toFile(mesh1, outFile);

  t = Timer_getTime();
  RcsMeshData* mesh2 = RcsMesh_createFromFile(outFile);
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
    double b = 0.0, dt = Timer_getSystemTime();
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
    double res = 0.0, dt = Timer_getSystemTime();
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
    double a = 0.0, dt = Timer_getSystemTime();
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
static bool test_countSubStrings()
{
  const char* testStr1 = "1.0 2.0 3.0 4.0 5.0 6.0";
  const char* testStr2 = "1.0 2.0 3.0 4.0 5.0 6.0 ";
  const char* testStr3 = " 1.0 2.0 3.0 4.0 5.0 6.0";
  const char* testStr4 = " 1.0 2.0 3.0 4.0 5.0 6.0 ";
  const char* testStr5 = "        1.0   2.0    3.0   4.0   5.0    6.0     ";

  unsigned int n1 = String_countSubStrings(testStr1, " ");
  unsigned int n2 = String_countSubStrings(testStr2, " ");
  unsigned int n3 = String_countSubStrings(testStr3, " ");
  unsigned int n4 = String_countSubStrings(testStr4, " ");
  unsigned int n5 = String_countSubStrings(testStr5, " ");

  RLOG(0, "n1=%d n2=%d n3=%d n4=%d n5=%d", n1, n2, n3, n4, n5);

  const char* testStr6 = "2.237060 -0.013035 3.141593 0.986085 0.003336 -0.000065 -0.014339 0.050996 0.007063 -0.014165 0.040262 0.013000 -0.013770 0.025398 0.018120 -0.013447 0.003783 0.023044 0.006052 0.040289 0.068821 0.013788 -0.146118 -0.084346 0.009802 0.261406 0.101507 -0.406857 3.048911 -0.915538 -4.403101 6.391742 -0.362587 -0.030473 -0.224328 -0.125127 0.242712 -0.142775 -0.670336 -1.071533 -0.086619 0.515012 0.001037 -0.136142 0.001102 0.267114 -0.002522 0.160046 -0.001497 0.001004 -0.264057 0.000630 0.001037 -0.136153 0.001102 0.267151 -0.002521 0.159979 -0.001497 0.001004 -0.264017 0.000630 ";
  unsigned int n6 = String_countSubStrings(testStr6, " ");

  RLOG(0, "n6=%d", n6);

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
      fprintf(stderr, "\t\t6   Sub-strings in string counting test\n");
      fprintf(stderr, "\t\t7   Test string-splitting\n");
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

    case 6:
    {
      success = test_countSubStrings();
      break;
    }

    case 7:
    {
      success = test_stringSplit("Honda|Hallo1#Hallo2#Hallo 3#abc def ", "#");
      success = test_stringSplit("Honda|Honda|Honda|Honda|Honda|", "#");
      success = test_stringSplit("##", "#");
      success = test_stringSplit("SLAMDeviceInput&1&Tap", "&");
      success = test_stringSplit("ARDeviceTransform&SLAMDeviceId&posx\"posy\"posz$qx\"qy\"qz\"quatw", "&");
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
