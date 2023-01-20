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

#include "template.hpp"

#include <Rcs_macros.h>
#include <Rcs_utils.h>
#include <Rcs_utilsCPP.h>
#include <Rcs_timer.h>
#include <Rcs_resourcePath.h>
#include <Rcs_cmdLine.h>
#include <Rcs_mesh.h>
#include <Rcs_basicMath.h>

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
#if 0
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_ENV_VAR_LENGTH 64
#define MAX_LINE_LENGTH 512

int testEnvVar(int argc, char* argv[])
{
  char line[MAX_LINE_LENGTH];
  char envVar[MAX_ENV_VAR_LENGTH];

  printf("Enter a string with an environment variable: ");
  fgets(line, MAX_LINE_LENGTH, stdin);

  // Find the first occurrence of "${" in the string
  char* varStart = strstr(line, "${");
  if (varStart == NULL)
  {
    printf("Error: No environment variable found in the string\n");
    return 1;
  }

  // Find the end of the environment variable by searching for the "}" character
  char* varEnd = strstr(varStart, "}");
  if (varEnd == NULL || varEnd < varStart+2)
  {
    printf("Error: Malformed environment variable in the string\n");
    return 1;
  }

  // Extract the name of the environment variable
  size_t varLength = varEnd - varStart - 2;
  strncpy(envVar, varStart + 2, varLength);
  envVar[varLength] = '\0';
  printf("envVar='%s'\n", envVar);

  // Get the value of the environment variable
  char* value = getenv(envVar);
  if (value == NULL)
  {
    printf("Error: Environment variable %s not found\n", envVar);
    return 1;
  }

  printf("value='%s'\n", value);
  printf("start index = %zu\n", varStart - line);
  printf("end index = %zu\n", varEnd - line);

  char resolved[256];
  int firstLen = varStart - line;
  char* end = line + strlen(line);
  char* cursor = resolved;
  memcpy(cursor, line, firstLen);   // That's everything up to "${"
  // cursor[firstLen] = '\0';
  // printf("first = '%s'\n", cursor);
  cursor += firstLen;
  memcpy(cursor, value, strlen(value));   // That's the environment variable
  // cursor[strlen(value)] = '\0';
  // printf("second = '%s'\n", cursor);
  cursor += strlen(value);
  memcpy(cursor, varEnd+1, end-varEnd-2); // That's everything after "}" including the '\0'
  // cursor[end-varEnd-2] = '\0';
  // printf("third = '%s' len=%d\n", cursor, end-varEnd-2);
  // cursor += end-varEnd-2;
  //*cursor = '\0';


  // strncpy(resolved, line, start - line);
  // strncat(resolved, value, strlen(value));
  // strncat(resolved, end+1, strlen(line)-(end-line)-2);
  printf("resolved = '%s'\n", resolved);

#if 0
  // Replace the environment variable in the string with its value
  int valueLength = strlen(value);
  int lineLength = strlen(line);
  int diff = valueLength - envVarLength - 2;
  if (diff >= 0)
  {
    memmove(end + diff, end, lineLength - (end - line));
  }
  else
  {
    memmove(start + valueLength, end, lineLength - (end - line));
  }
  memcpy(start, value, valueLength);

  printf("String with environment variable replaced: %s\n", line);
#endif
  return 0;
}
#endif

/******************************************************************************
 *
 *****************************************************************************/
static bool testFuzzyStringMatching()
{
  Rcs::CmdLineParser argP;
  std::string w1, w2;
  argP.getArgument("-w1", &w1, "String 1 (default is %s)", w1.c_str());
  argP.getArgument("-w2", &w2, "String 2 (default is %s)", w2.c_str());

  int dist = String_LevenshteinDistance(w1.c_str(), w2.c_str());

  RLOG(1, "Distance between \"%s\" and \"%s\" is %d",
       w1.c_str(), w2.c_str(), dist);

  return true;
}

/******************************************************************************
 *
 *****************************************************************************/
static bool test_stringSplit(std::string src, std::string delim,
                             size_t expectedSize)
{
  RLOG(4, "Delimiter:       \"%s\"", delim.c_str());
  RLOG(4, "Original string: \"%s\"", src.c_str());
  std::vector<std::string> split = Rcs::String_split(src, delim);

  for (size_t i=0; i<split.size(); ++i)
  {
    RLOG(4, "\tsubstring[%d] = \"%s\" split=%d",
         (int) i, split[i].c_str(), (int) split.size());
  }

  if (split.size() != expectedSize)
  {
    RLOG_CPP(3, "split-size=" << split.size() << " expected=" << expectedSize);
  }

  return split.size() == expectedSize;
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

  RLOGS(3, "%s: unique file name is %s (Template: \"%s\", suffix: \"%s\")",
        namePtr ? "SUCCESS" : "FAILURE", namePtr ? namePtr : "NULL",
        hasP ? pattern : "NULL", hasS ? suffix : "NULL");

  if (namePtr != NULL)
  {
    FILE* testFile = fopen(namePtr, "w+");

    if (testFile != NULL)
    {
      RLOGS(3, "Successfully opened file \"%s\"", namePtr);
      fclose(testFile);
      success = true;
    }
    else
    {
      RLOGS(3, "Couldn't open file \"%s\"", namePtr);
      success = false;
    }
  }

  RLOGS(1, "%s testing File_createUniqueName()",
        success ? "SUCCESS" : "FAILURE");

  return success;
}

/******************************************************************************
 *
 *****************************************************************************/
static bool test_compareEnding()
{
  char str[256] = "file.txt";
  char ending[256] = ".txt";
  Rcs::CmdLineParser argP;
  argP.getArgument("-str", str, "String to compare against ending");
  argP.getArgument("-ending", ending, "Ending");
  bool caseSensitive = argP.hasArgument("-case", "Check case sensitive");

  RLOG(5, "Comparing \"%s\" against ending \"%s\"", str, ending);

  bool success = String_hasEnding(str, ending, caseSensitive);

  RLOGS(1, "%s testing String_hasEnding()", success ? "SUCCESS" : "FAILURE");

  return success;
}

/******************************************************************************
 *
 *****************************************************************************/
static bool test_mesh()
{
  bool success = true;
  char outFile[256] = "mesh1.tri";

  Rcs::CmdLineParser argP;
  argP.getArgument("-outFile", outFile, "Name of mesh file to be written "
                   "(default is %s)", outFile);

  double t = Timer_getTime();

  RcsMeshData* mesh1 = RcsMesh_createTorus(0.5, 0.1, 8, 32);

  if (!mesh1)
  {
    RLOG(3, "Failed to create a torus mesh");
    return false;
  }

  t = Timer_getTime() - t;
  RLOG(3, "[%.3f msec]: Mesh has %u vertices and %u faces",
       1000.0*t, mesh1->nVertices, mesh1->nFaces);

  t = Timer_getTime();
  int res = RcsMesh_compressVertices(mesh1, 1.0e-8);

  if (res==-1)
  {
    RLOG(3, "Failed to compress vertices");
    return false;
  }

  t = Timer_getTime() - t;
  RLOG(3, "[%.3f msec]: Compressed mesh has %u vertices and %u faces",
       t, mesh1->nVertices, mesh1->nFaces);
  success = RcsMesh_toFile(mesh1, outFile);

  if (!success)
  {
    RLOG(3, "Failed to write mesh to file");
    RcsMesh_destroy(mesh1);
    return false;
  }

  t = Timer_getTime();
  RcsMeshData* mesh2 = RcsMesh_createCylinder(0.3, 1.5, 16);

  if (!mesh2)
  {
    RLOG(3, "Failed to create a cylinder mesh");
    RcsMesh_destroy(mesh1);
    return false;
  }

  t = Timer_getTime() - t;
  RLOG(3, "[%.3f msec]: Reloaded mesh has %u vertices and %u faces",
       t, mesh2->nVertices, mesh2->nFaces);

  // Test shifting of mesh
  RcsMesh_shift(mesh2, 0.5, 0.5, 0.5);

  // Test merging of meshes
  RLOG(3, "Merging meshes");
  RcsMesh_add(mesh1, mesh2);
  success = RcsMesh_toFile(mesh1, "MergedMesh.stl");

  if (!success)
  {
    RLOG(3, "Failed to write merged mesh to file");
    RcsMesh_destroy(mesh1);
    RcsMesh_destroy(mesh2);
    return false;
  }

  RLOG(3, "Merging mesh %s - see file MergedMesh.stl",
       success ? "succeeded" : "failed");
  success = RcsMesh_toFile(mesh2, "ShiftedMesh.stl");

  RcsMesh_destroy(mesh1);
  RcsMesh_destroy(mesh2);

  RLOGS(1, "%s testing mesh functions", success ? "SUCCESS" : "FAILURE");

  return success;
}

/******************************************************************************
 *
 *****************************************************************************/
static bool test_envString(const char* testStr)
{
  RLOG(3, "Testing string \"%s\"", testStr);

  Rcs::CmdLineParser argP;
  std::string parsedStr;
  argP.getArgument("-str", &parsedStr, "String to test (default is none)");
  if (!parsedStr.empty())
  {
    testStr = parsedStr.c_str();
  }

  char* expanded = String_expandEnvironmentVariables(testStr);

  std::string home, user;

  const char* envPtr = String_getEnv("HOME");
  if (envPtr)
  {
    home = std::string(envPtr);
  }
  else
  {
    RLOG(2, "Cannot find HOME environment variable");
  }

  envPtr = String_getEnv("USER");
  if (envPtr)
  {
    user = std::string(envPtr);
  }
  else
  {
    RLOG(2, "Cannot find USER environment variable");
  }

  std::string res = home + "/test1/" + user + "/bin";
  RLOG(3, "Expanded \"%s\", truth is \"%s\"", expanded, res.c_str());

  bool success = (res == std::string(expanded));

  RFREE(expanded);

  RLOGS(1, "%s testing String_expandEnvironmentVariables()",
        success ? "SUCCESS" : "FAILURE");

  return success;
}

/******************************************************************************
 *
 *****************************************************************************/
static bool test_localeFreeParsing()
{
  bool success = true;
  char str[256] = "3.1415926535";
  double groundTruth = 3.1415926535;
  int nIter = 1;
  Rcs::CmdLineParser argP;
  argP.getArgument("-str", str, "String to convert (default is %s)", str);
  argP.getArgument("-iter", &nIter, "Iterations (default is %d)", nIter);

  char* res = setlocale(LC_ALL, "de_DE.utf8");
  RLOG(4, "setlocale() returned \"%s\"", res ? res : "NULL");

  if ((res==NULL) || (!STREQ(res, "de_DE.utf8")))
  {
    RLOG(3, "Failed to set locale to \"de_DE.utf8\"");
    return false;
  }

  struct lconv* loc = localeconv();

  RLOG(4, "Decimal character is %c", *(loc->decimal_point));
  RLOG(4, "String to convert to double: \"%s\"", str);


  // Locale conversion with sscanf - should fail (Result should be 3.0)
  {
    double b = 0.0, dt = Timer_getSystemTime();
    for (int i=0; i<nIter; ++i)
    {
      int nItems = sscanf(str, "%lf", &b);
      RCHECK(nItems == 1);
    }
    dt = Timer_getSystemTime() - dt;
    RLOG(4, "sscanf: double value is %f ... took %.4f usec",
         b, (1.0/nIter)*1.0e6*dt);
    if (b != 3.0)
    {
      success = false;
    }
  }


  // Locale-free conversion with strtod_l
  {
    double res = 0.0, dt = Timer_getSystemTime();
    for (int i=0; i<nIter; ++i)
    {
      res = String_toDouble_l(str);
    }
    dt = Timer_getSystemTime() - dt;
    RLOG(4, "strtod_l: double value is %f ... took %.4f usec",
         res, (1.0/nIter)*1.0e6*dt);

    if (res != groundTruth)
    {
      success = false;
    }
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
    RLOG(4, "stringstream with imbue: double value is %f ... took %.4f usec",
         a, (1.0/nIter)*1.0e6*dt);

    if (a != groundTruth)
    {
      success = false;
    }
  }


  // Locale-free double to string conversion
  {
    double num = -12345.1239456789;
    int ndigits = 3;
    argP.getArgument("-num", &num);
    argP.getArgument("-digits", &ndigits);
    char sir[64];
    String_fromDouble(sir, num, ndigits);
    RLOG(4, "str=%s", sir);

    if (!STREQ(sir, "-12345.124"))
    {
      RLOG(3, "Failed to convert number - -12345.124 != %f", num);
      return false;
    }
  }

  // Reset the locale to the system's default.
  // See https://en.cppreference.com/w/c/locale/setlocale for details
  res = setlocale(LC_ALL, "C");
  RLOG(4, "setlocale() returned \"%s\"", res ? res : "NULL");

  RLOGS(1, "%s testing locale-free parsing", success ? "SUCCESS" : "FAILURE");


  return success;
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
  const char* testStr6 = "2.237060 -0.013035 3.141593 0.986085 0.003336 -0.000065 -0.014339 0.050996 0.007063 -0.014165 0.040262 0.013000 -0.013770 0.025398 0.018120 -0.013447 0.003783 0.023044 0.006052 0.040289 0.068821 0.013788 -0.146118 -0.084346 0.009802 0.261406 0.101507 -0.406857 3.048911 -0.915538 -4.403101 6.391742 -0.362587 -0.030473 -0.224328 -0.125127 0.242712 -0.142775 -0.670336 -1.071533 -0.086619 0.515012 0.001037 -0.136142 0.001102 0.267114 -0.002522 0.160046 -0.001497 0.001004 -0.264057 0.000630 0.001037 -0.136153 0.001102 0.267151 -0.002521 0.159979 -0.001497 0.001004 -0.264017 0.000630 ";

  unsigned int n1 = String_countSubStrings(testStr1, " "); // must be 6
  unsigned int n2 = String_countSubStrings(testStr2, " "); // must be 6
  unsigned int n3 = String_countSubStrings(testStr3, " "); // must be 6
  unsigned int n4 = String_countSubStrings(testStr4, " "); // must be 6
  unsigned int n5 = String_countSubStrings(testStr5, " "); // must be 6
  unsigned int n6 = String_countSubStrings(testStr6, " "); // must be 62

  bool success = true;

  if ((n1!=6) || (n2!=6) || (n3!=6) || (n4!=6) || (n5 != 6) || (n6 != 62))
  {
    success = false;
  }

  RLOG(3, "n1=%d n2=%d n3=%d n4=%d n5=%d (should be 6)", n1, n2, n3, n4, n5);
  RLOG(3, "n6=%d (should be 62)", n6);

  RLOGS(1, "%s testing String_countSubStrings()",
        success ? "SUCCESS" : "FAILURE");

  return success;
}

/******************************************************************************
 *
 *****************************************************************************/
static bool test_DoubleToString()
{
  int digits = 8;
  double value = M_PI;
  bool success = true;
  Rcs::CmdLineParser argP;
  argP.getArgument("-digits", &digits, "Digits (default is %d)", digits);
  argP.getArgument("-value", &value, "Value (default is %g)", value);

  char buf[512] = "";
  String_fromDouble(buf, value, digits);
  RLOG(3, "%.8f = \"%s\"", value, buf);
  RLOG_CPP(3, "len: " << strlen(buf));

  if (std::string(buf) != "3.14159265")
  {
    success = false;
  }

  RLOGS(1, "%s testing String_fromDouble()", success ? "SUCCESS" : "FAILURE");

  return success;
}

/******************************************************************************
 *
 *****************************************************************************/
static bool testRemoveSuffix()
{
  bool success = true;
  char tmp[256];

  String_removeSuffix(tmp, "Hallo 1 2 3", ' ');
  RLOG(3, "\"Hallo 1 2 3\" becomes \"%s\"", tmp);

  if (!STREQ(tmp, "Hallo 1 2"))
  {
    RLOG(2, "Failed: \"Hallo 1 2\" expected, but got \"%s\"", tmp);
    success = false;
  }

  RLOGS(1, "%s testing String_removeSuffix()", success ? "SUCCESS" : "FAILURE");

  return success;
}

/******************************************************************************
 *
 *****************************************************************************/
static bool testMeshConversion()
{
  bool success = true;
  std::string inFile, outFile;
  double eps = 1.0e-4;

  Rcs::CmdLineParser argP;
  argP.getArgument("-inFile", &inFile, "Name of mesh file to be read.");
  argP.getArgument("-outFile", &outFile, "Name of mesh file to be written.");
  argP.getArgument("-eps", &eps, "Distance below which vertices are considered"
                   " duplicate (default is %f)", eps);
  bool compress = argP.hasArgument("-compress", "Remove duplicate vertices");

  RcsMeshData* inMesh = RcsMesh_createFromFile(inFile.c_str());

  if (!inMesh)
  {
    RLOG(1, "Failed to read file \"%s\"", inFile.c_str());
    return false;
  }

  if (compress)
  {
    int nDuplicates = RcsMesh_compressVertices(inMesh, eps);
    RLOG(1, "Removed %d duplicates", nDuplicates);
  }

  REXEC(4)
  {
    RcsMesh_print(inMesh);
  }

  success = RcsMesh_toFile(inMesh, outFile.c_str());

  if (!success)
  {
    RLOG(1, "Failed to write file \"%s\"", outFile.c_str());
  }
  return success;
}

/******************************************************************************
 *
 *****************************************************************************/
static bool testResourcePath()
{
  bool success = true;
  Rcs_clearResourcePath();

  Rcs_addResourcePath("MyFirstPath");
  Rcs_addResourcePath("MySecondPathPath");
  Rcs_addResourcePath("MyThirdPathPath");

  unsigned int n = Rcs_numResourcePaths();
  if (n!=3)
  {
    RLOG(1, "Found %d resource paths, should be 3", n);
    REXEC(2)
    {
      Rcs_printResourcePath();
    }
    success = false;
  }

  // Add same paths again: Should not create duplicates
  Rcs_addResourcePath("MyFirstPath");
  Rcs_addResourcePath("MySecondPathPath");
  Rcs_addResourcePath("MyThirdPathPath");

  n = Rcs_numResourcePaths();

  if (n != 3)
  {
    RLOG(1, "Found %d resource paths, should be 3", n);
    REXEC(2)
    {
      Rcs_printResourcePath();
    }
    success = false;
  }

  // Remove paths
  Rcs_removeResourcePath("MyFirstPath");

  n = Rcs_numResourcePaths();

  if (n != 2)
  {
    RLOG(1, "Found %d resource paths, should be 2", n);
    REXEC(2)
    {
      Rcs_printResourcePath();
    }
    success = false;
  }

  // Remove paths
  Rcs_removeResourcePath("MyFirstPath");

  n = Rcs_numResourcePaths();

  if (n != 2)
  {
    RLOG(1, "Found %d resource paths, should be 2", n);
    REXEC(2)
    {
      Rcs_printResourcePath();
    }
    success = false;
  }

  RLOGS(1, "%s testing resource path functions", success ? "SUCCESS" : "FAILURE");

  return success;
}

/******************************************************************************
 *
 *****************************************************************************/
static bool testMode(int mode, int argc, char** argv)
{
  bool success = true;

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
      fprintf(stderr, "\t\t8   Test double to string conversion\n");
      fprintf(stderr, "\t\t9   Test line number extraction for logging\n");
      fprintf(stderr, "\t\t10  Test String_removeSuffix()\n");
      fprintf(stderr, "\t\t11  Test resource path functions\n");
      fprintf(stderr, "\t\t12  Test mesh conversion\n");
      fprintf(stderr, "\t\t13  Test fuzzy string matching\n");
      fprintf(stderr, "\n\nResource path:\n");
      Rcs_printResourcePath();
      break;
    }

    case 1:
      success = test_uniqueFileName();
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
      success = test_envString("${HOME}/test1/${USER}/bin");
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
      bool si;
      si = test_stringSplit("Honda|Hallo1#Hallo2#Hallo 3#abc def ", "#", 4);
      success = success && si;
      RLOG(3, "%s", (si ? "Pass" : "Fail"));
      si = test_stringSplit("Honda|Honda|Honda|Honda|Honda|", "#", 1);
      success = success && si;
      RLOG(3, "%s", (si ? "Pass" : "Fail"));
      // si = test_stringSplit("##", "#", 3);
      // success = success && si;
      // RLOG(3, "%s", (si ? "Pass" : "Fail"));
      si = test_stringSplit("SLAMDeviceInput&1&Tap", "&", 3);
      success = success && si;
      RLOG(3, "%s", (si ? "Pass" : "Fail"));
      si = test_stringSplit("ARDeviceTransform&SLAMDeviceId&posx\"posy\"posz$qx\"qy\"qz\"quatw", "&", 3);
      success = success && si;
      RLOG(3, "%s", (si ? "Pass" : "Fail"));
      si = test_stringSplit("", "", 1);
      success = success && si;
      RLOG(3, "%s", (si ? "Pass" : "Fail"));
      si = test_stringSplit("Honda|Honda|Honda", "", 1);
      success = success && si;
      RLOG(3, "%s", (si ? "Pass" : "Fail"));
      si = test_stringSplit("", "#", 1);
      success = success && si;
      RLOG(3, "%s", (si ? "Pass" : "Fail"));
      si = test_stringSplit("Honda|Honda|Honda|Honda", "Honda", 3);
      success = success && si;
      RLOG(3, "%s", (si ? "Pass" : "Fail"));
      si = test_stringSplit("Honda|Honda|Honda|Honda", "|", 4);
      success = success && si;
      RLOG(3, "%s", (si ? "Pass" : "Fail"));
      si = test_stringSplit("Honda|Honda|Honda|Honda|", "|", 4);
      success = success && si;
      RLOG(3, "%s", (si ? "Pass" : "Fail"));
      si = test_stringSplit("|Honda|Honda|Honda|Honda", "|", 4);
      success = success && si;
      RLOG(3, "%s", (si ? "Pass" : "Fail"));
      si = test_stringSplit("|Honda|Honda|Honda|Honda|", "|", 4);
      success = success && si;
      RLOG(3, "%s", (si ? "Pass" : "Fail"));
      si = test_stringSplit("$$$$$$$$$$$", "$", 0);
      success = success && si;
      RLOG(3, "%s", (si ? "Pass" : "Fail"));
      si = test_stringSplit("$$$$$$$$$$$", "$$", 1);
      success = success && si;
      RLOG(3, "%s", (si ? "Pass" : "Fail"));
      RLOGS(1, "%s testing String_split()", success ? "SUCCESS" : "FAILURE");
      break;
    }

    case 8:
    {
      success = test_DoubleToString();
      break;
    }

    case 9:
    {
      success = logSomething<int>();
      break;
    }

    case 10:
    {
      success = testRemoveSuffix();
      break;
    }

    case 11:
    {
      success = testResourcePath();
      break;
    }

    case 12:
    {
      success = testMeshConversion();
      break;
    }

    case 13:
    {
      success = testFuzzyStringMatching();
      break;
    }

    // case 14:
    // {
    //   testEnvVar(argc, argv);
    //   break;
    // }

    default:
    {
      RMSG("there is no mode %d", mode);
    }

  }   // switch(mode)

  return success;
}

/******************************************************************************
 *
 *****************************************************************************/
int main(int argc, char** argv)
{
  int mode = 0, result = 0;
  bool success = true;

  // Ctrl-C callback handler
  signal(SIGINT, quit);

  // Parse command line arguments
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-dl", &RcsLogLevel, "Debug level (default is 0)");
  argP.getArgument("-m", &mode, "Test mode");

  if (mode == -1)
  {
    for (int i = 1; i <= 11; ++i)
    {
      bool success_i = testMode(i, argc, argv);
      if (!success_i)
      {
        result++;
      }

      success = success_i && success;
      RLOG(0, "%d: %s", i, success ? "SUCCESS" : "FAIL");
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
    RLOGS(1, "TestCore test %s", success ? "SUCCEEDED" : "FAILED");
  }

  fprintf(stderr, "Thanks for using the TestCore program\n");

  return Math_iClip(result, 0, 255);
}
