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

#include "Rcs_resourcePath.h"
#include "Rcs_utils.h"
#include "Rcs_macros.h"

#include <vector>
#include <string>


typedef std::vector<std::string> StringList;
typedef StringList::iterator StringListIt;

static StringList* RCSRESOURCEPATH = NULL;

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
    if (!RCSRESOURCEPATH)
    {
      return;
    }
    RCSRESOURCEPATH->clear();
    delete(RCSRESOURCEPATH);
    RCSRESOURCEPATH = NULL;
  }

  /*****************************************************************************
   *
   ****************************************************************************/
  void Rcs_initResourcePath(void)
  {
    if (!RCSRESOURCEPATH)
    {
      RCSRESOURCEPATH = new StringList;
      atexit(Rcs_clearResourcePath);
    }
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

    // Create data on initial call
    Rcs_initResourcePath();

    // Check if the path is already contained in the vector
    StringListIt it;
    std::string pathStr = std::string(path);
    addDelim(pathStr);

    for (it = RCSRESOURCEPATH->begin(); it != RCSRESOURCEPATH->end(); ++it)
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
      RCSRESOURCEPATH->push_back(pathStr);
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

    // Create data on initial call
    Rcs_initResourcePath();

    // Check if the path is already contained in the vector
    StringListIt it;
    std::string pathStr = std::string(path);
    addDelim(pathStr);
    for (it = RCSRESOURCEPATH->begin(); it != RCSRESOURCEPATH->end(); ++it)
    {
      if (*it == pathStr)
      {
        RLOG(6, "Path \"%s\" already in resource path - skipping", path);
        return false;
      }
    }

    // Add resource path
    RCSRESOURCEPATH->insert(RCSRESOURCEPATH->begin(), pathStr);
    RLOG(6, "Added path \"%s\" to resource path list", path);

    return true;
  }

  /*****************************************************************************
   *
   ****************************************************************************/
  const char* Rcs_getResourcePath(unsigned int index)
  {
    if (!RCSRESOURCEPATH)
    {
      return NULL;
    }
    if (index >= RCSRESOURCEPATH->size())
    {
      return NULL;
    }

    return RCSRESOURCEPATH->at(index).c_str();
  }

  /*****************************************************************************
   *
   ****************************************************************************/
  bool Rcs_getAbsoluteFileName(const char* fileName, char* absFileName)
  {
    if (File_exists(fileName))
    {
      NLOG(6, "Found file \"%s\" in relative path", fileName);
      strcpy(absFileName, fileName);
      return true;
    }
    else
    {
      NLOG(6, "Didn't find file \"%s\" in relative path", fileName);
    }

    if (!RCSRESOURCEPATH)
    {
      NLOG(6, "Resource path doesn't exist yet - returning false");
      return false;
    }

    StringListIt it;

    for (it = RCSRESOURCEPATH->begin(); it != RCSRESOURCEPATH->end(); ++it)
    {
      std::string fullName = *it + std::string(fileName);

      NLOG(6, "Searching file in \"%s\"", fullName.c_str());

      if (File_exists(fullName.c_str()))
      {
        NLOG(6, "Found file \"%s\" in path \"%s\"", fileName, fullName.c_str());
        strcpy(absFileName, fullName.c_str());
        return true;
      }
    }

    NLOG(6, "Didn't find file \"%s\" in resource paths", fileName);

    return false;
  }

  /*****************************************************************************
   *
   ****************************************************************************/
  bool Rcs_getAbsolutePath(const char* fileName, char* absPath)
  {
    if (fileName == NULL)
    {
      RLOG(4, "fileName is NULL - returning false");
      return false;
    }

    if (absPath == NULL)
    {
      RLOG(4, "absPath is NULL - returning false");
      return false;
    }

    // Check if file is in relative path
    RLOG(6, "Searching file \"%s\" in relative path", fileName);
    if (File_exists(fileName))
    {
      RLOG(6, "Found file \"%s\" in relative path", fileName);
      strcpy(absPath, "");
      return true;
    }
    else
    {
      RLOG(6, "Didn't find file \"%s\" in relative path", fileName);
    }

    if (!RCSRESOURCEPATH)
    {
      RLOG(6, "Resource path doesn't exist yet - returning false");
      return false;
    }


    StringListIt it;

    for (it = RCSRESOURCEPATH->begin(); it != RCSRESOURCEPATH->end(); ++it)
    {
      std::string fullName = *it + std::string(fileName);

      RLOG(6, "Searching file in \"%s\"", fullName.c_str());

      if (File_exists(fullName.c_str()))
      {
        RLOG(6, "Found file \"%s\" in path \"%s\"",
             fileName, fullName.c_str());
        strcpy(absPath, (*it).c_str());
        return true;
      }
    }

    RLOG(6, "Didn't find file \"%s\" in resource paths", fileName);

    return false;
  }

  /*****************************************************************************
   *
   ****************************************************************************/
  void Rcs_printResourcePath(void)
  {
    if (!RCSRESOURCEPATH)
    {
      return;
    }
    StringListIt it;
    int k = 0;

    fprintf(stderr, "[%s]:\n", __FUNCTION__);

    for (it = RCSRESOURCEPATH->begin(); it != RCSRESOURCEPATH->end(); ++it)
    {
      std::string str = *it;
      fprintf(stderr, "Path[%d] = \"%s\"\n", k, str.c_str());
      k++;
    }
  }



}  // extern "C"
