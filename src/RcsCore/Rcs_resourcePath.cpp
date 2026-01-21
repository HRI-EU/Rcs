/*******************************************************************************

  Copyright (c) Honda Research Institute Europe GmbH

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

#include "Rcs_resourcePath.h"
#include "Rcs_utils.h"
#include "Rcs_utilsCPP.h"
#include "Rcs_macros.h"

#include <vector>
#include <string>
#include <algorithm>


static std::vector<std::string> RCSRESOURCEPATH;

// If str is not ended by the delimiter, it is added. Otherwise, nothing
// is done. We distinguish between Windows and Linux delimiters.
static void makeValidPath(std::string& str)
{
  // Remove white spaces etc. at beginning and end of the string
  str = Rcs::String_trim(str);

  // Replace all '\\' with '/'
  for (size_t i = 0; i < str.length(); ++i)
  {
    if (str[i] == '\\')
    {
      str[i] = '/';
    }
  }

  // Make sure that the string ends with a slash
  Rcs::String_rtrim(str, "/");
  str += '/';
}


namespace Rcs
{
std::vector<std::string> getResourcePath()
{
  return RCSRESOURCEPATH;
}

std::string getAbsoluteFileName(const std::string& filename)
{
  char configFile[512] = "";
  bool fileExists = Rcs_getAbsoluteFileName(filename.c_str(), configFile);
  if (!fileExists)
  {
    return std::string();
  }

  return std::string(configFile);
}

std::string getAbsoluteFileName(const std::vector<std::string>& filenames)
{
  for (size_t i=0; i<filenames.size(); ++i)
  {
    std::string absFileName = getAbsoluteFileName(filenames[i]);
    RMSG_CPP("Searching through " << filenames[i]);
    if (!absFileName.empty())
    {
      RMSG_CPP("Found: " << absFileName);
      return absFileName;
    }
  }

  return std::string();
}


}

extern "C" {



  /*****************************************************************************
   *
   ****************************************************************************/
  void Rcs_clearResourcePath(void)
  {
    RCSRESOURCEPATH.clear();
  }

  /*****************************************************************************
   *
   ****************************************************************************/
  static bool Rcs_appendOrPrependResourcePath(const char* path, bool append)
  {
    if ((path==NULL) || (strlen(path)==0))
    {
      RLOG(4, "Path is NULL or empty - skipping");
      return false;
    }

    std::vector<std::string> paths = Rcs::String_split(path, ";");

    for (size_t i = 0; i < paths.size(); ++i)
    {
      makeValidPath(paths[i]);

      if (std::find(RCSRESOURCEPATH.begin(), RCSRESOURCEPATH.end(), paths[i]) == RCSRESOURCEPATH.end())
      {
        if (append)
        {
          RCSRESOURCEPATH.push_back(paths[i]);
          RLOG(5, "Added path \"%s\" to resource paths", paths[i].c_str());
        }
        else
        {
          RCSRESOURCEPATH.insert(RCSRESOURCEPATH.begin(), paths[i]);
          RLOG(5, "Inserted path \"%s\" to resource paths", paths[i].c_str());
        }
      }
    }

    return true;
  }

  /*****************************************************************************
   *
   ****************************************************************************/
  bool Rcs_addResourcePath(const char* path)
  {
    return Rcs_appendOrPrependResourcePath(path, true);
  }

  /*****************************************************************************
   *
   ****************************************************************************/
  bool Rcs_insertResourcePath(const char* path)
  {
    return Rcs_appendOrPrependResourcePath(path, false);
  }

  /*****************************************************************************
   *
   ****************************************************************************/
  const char* Rcs_getResourcePath(unsigned int index)
  {
    if (index >= RCSRESOURCEPATH.size())
    {
      return NULL;
    }

    return RCSRESOURCEPATH.at(index).c_str();
  }

  /*****************************************************************************
   *
   ****************************************************************************/
  bool Rcs_removeResourcePath(const char* pathStr)
  {
    std::string path = std::string(pathStr);
    makeValidPath(path);
    std::vector<std::string>::iterator it;
    it = std::find(RCSRESOURCEPATH.begin(), RCSRESOURCEPATH.end(), path);

    if (it != RCSRESOURCEPATH.end())
    {
      RCSRESOURCEPATH.erase(it);
      return true;
    }

    return false;
  }

  /*****************************************************************************
   *
   ****************************************************************************/
  bool Rcs_getAbsoluteFileName(const char* fileName, char* absFileName)
  {
    if (fileName==NULL)
    {
      return false;
    }

    for (size_t i = 0; i < RCSRESOURCEPATH.size(); ++i)
    {
      std::string fullName = RCSRESOURCEPATH[i] + std::string(fileName);

      if (File_exists(fullName.c_str()))
      {
        if (absFileName)
        {
          strcpy(absFileName, fullName.c_str());
        }
        return true;
      }
    }

    // Then check current directory.
    if (File_exists(fileName))
    {
      if (absFileName)
      {
        strcpy(absFileName, fileName);
      }
      return true;
    }

    return false;
  }

  /*****************************************************************************
   *
   ****************************************************************************/
  bool Rcs_fileInResourcePath(const char* fileName)
  {
    return Rcs_getAbsoluteFileName(fileName, NULL);
  }

  /*****************************************************************************
   *
   ****************************************************************************/
  void Rcs_printResourcePath(void)
  {
    fprintf(stderr, "[%s]:\n", __FUNCTION__);
    for (size_t i = 0; i < RCSRESOURCEPATH.size(); ++i)
    {
      fprintf(stderr, "Path[%zu] = \"%s\"\n", i, RCSRESOURCEPATH[i].c_str());
    }

  }

  /*****************************************************************************
   *
   ****************************************************************************/
  unsigned int Rcs_numResourcePaths(void)
  {
    return RCSRESOURCEPATH.size();
  }


}  // extern "C"
