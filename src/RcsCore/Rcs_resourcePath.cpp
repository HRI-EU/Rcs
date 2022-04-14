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

#include "Rcs_resourcePath.h"
#include "Rcs_utils.h"
#include "Rcs_macros.h"

#include <vector>
#include <string>
#include <algorithm>


typedef std::vector<std::string> StringList;
typedef StringList::iterator StringListIt;

static StringList RCSRESOURCEPATH;

// If str is not ended by the delimiter, it is added. Otherwise, nothing
// is done. We distinguish between Windows and Linux delimiters.
static void addDelim(std::string& str)
{
  const char* cStr = str.c_str();

#if defined(_MSC_VER)
  if (cStr[strlen(cStr)-1] != '\\')
  {
    str += '\\';
  }
#else
  if (cStr[strlen(cStr)-1] != '/')
  {
    str += '/';
  }
#endif
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
  bool Rcs_addResourcePath(const char* path)
  {
    if (path==NULL)
    {
      RLOG(4, "Path is NULL - skipping");
      return false;
    }

    // Check if the path is already contained in the vector
    StringListIt it;
    std::string pathStr = std::string(path);
    addDelim(pathStr);

    for (it = RCSRESOURCEPATH.begin(); it != RCSRESOURCEPATH.end(); ++it)
    {
      if (*it == pathStr)
      {
        RLOG(6, "Path \"%s\" already in resource path - skipping", path);
        return false;
      }
    }

    // Add resource path
    if (strlen(path) > 0)
    {
      RCSRESOURCEPATH.push_back(pathStr);
      RLOG(6, "Added path \"%s\" to resource paths", path);
    }

    return true;
  }

  /*****************************************************************************
   *
   ****************************************************************************/
  bool Rcs_insertResourcePath(const char* path)
  {
    if (path==NULL)
    {
      RLOG(4, "Path is NULL - skipping");
      return false;
    }

    // Check if the path is already contained in the vector
    StringListIt it;
    std::string pathStr = std::string(path);
    addDelim(pathStr);
    for (it = RCSRESOURCEPATH.begin(); it != RCSRESOURCEPATH.end(); ++it)
    {
      if (*it == pathStr)
      {
        RLOG(6, "Path \"%s\" already in resource path - skipping", path);
        return false;
      }
    }

    // Add resource path
    RCSRESOURCEPATH.insert(RCSRESOURCEPATH.begin(), pathStr);
    RLOG(6, "Added path \"%s\" to resource path list", path);

    return true;
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
    addDelim(path);
    StringListIt it = std::find(RCSRESOURCEPATH.begin(),
                                RCSRESOURCEPATH.end(),
                                path);

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

    StringListIt it;

    // First check all resource paths.
    for (it = RCSRESOURCEPATH.begin(); it != RCSRESOURCEPATH.end(); ++it)
    {
      std::string fullName = *it + std::string(fileName);

      if (File_exists(fullName.c_str()))
      {
        if (absFileName != NULL)
        {
          strcpy(absFileName, fullName.c_str());
        }
        return true;
      }
    }

    // Then check current directory.
    if (File_exists(fileName))
    {
      if (absFileName != NULL)
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
    StringListIt it;
    int k = 0;

    fprintf(stderr, "[%s]:\n", __FUNCTION__);

    for (it = RCSRESOURCEPATH.begin(); it != RCSRESOURCEPATH.end(); ++it)
    {
      std::string str = *it;
      fprintf(stderr, "Path[%d] = \"%s\"\n", k, str.c_str());
      k++;
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
