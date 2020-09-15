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

#include "Rcs_utilsCPP.h"
#include "Rcs_macros.h"

#include <fstream>
#include <algorithm>

#if !defined(_MSC_VER)
#include <dirent.h>
#else
#include <Windows.h>
#include <stdio.h>
#endif


bool String_hasEnding(const std::string& fullString, const std::string& ending)
{
  if (fullString.length() >= ending.length())
  {
    return (0 == fullString.compare(fullString.length() - ending.length(), ending.length(), ending));
  }
  else
  {
    return false;
  }
}

bool String_startsWith(const std::string& fullString, const std::string& beginning)
{
  if (fullString.length() >= beginning.length())
  {
    return (0 == fullString.compare(0, beginning.length(), beginning));
  }
  else
  {
    return false;
  }
}


// Returns a list of files in a directory (except the ones that begin with a dot)
std::list<std::string> getFilenamesInDirectory(const std::string& dirname, bool returnFullPath, const std::string& extension)
{
#if defined(_MSC_VER)

  std::list<std::string> out;
  RFATAL("No implementation under windows");

  return out;

#else

  DIR* pDIR;
  struct dirent* entry;
  std::list<std::string> files;
  if ((pDIR = opendir(dirname.c_str())))
  {
    while ((entry = readdir(pDIR)))
    {
      std::string file = entry->d_name;
      if (file.compare(".") != 0 &&
          file.compare("..") != 0 &&
          String_hasEnding(file, extension))
      {
        if (returnFullPath)
        {
          if (!String_hasEnding(dirname, "/"))
          {
            file.insert(0, "/");
          }

          // insert dirname at the beginning
          file.insert(0, dirname);
        }
        files.push_back(file);
      }
    }
    closedir(pDIR);
  }
  return files;

#endif
}

std::pair<std::string, std::string> Rcs_getExecutablePathAndFilename(char* argv[])
{
  // Get the last position of '/'
  std::string str(argv[0]);

  // get '/' or '\\' depending on Linux or Windows.
#if defined(_MSC_VER)
  int pos = str.rfind('\\');
#else
  int pos = str.rfind('/');
#endif

  // Get the path and the name
  std::string path = str.substr(0,pos+1);
  std::string name = str.substr(pos+1);

  return std::make_pair(path, name);
}

std::string formatStdString(const char* fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  std::string label_string = formatStdString(fmt, ap);
  va_end(ap);

  return label_string;
}

std::string formatStdString(const char* fmt, va_list ap)
{
  // first check how large our buffer has to be
#if defined(_MSC_VER)
  int size = _vscprintf(fmt, ap);
#else
  va_list ap_copy;
  va_copy(ap_copy, ap);
  int size = vsnprintf(NULL, 0, fmt, ap_copy);
  va_end(ap_copy);
#endif

  // generate formatted string
  std::string ret_val;
  ret_val.resize(size);
  vsprintf(&ret_val[0], fmt, ap);

  return ret_val;
}

bool File_isEqualCpp(const char* file1, const char* file2)
{
  std::ifstream in1(file1, std::ios::binary);
  std::ifstream in2(file2, std::ios::binary);

  std::ifstream::pos_type size1, size2;

  size1 = in1.seekg(0, std::ifstream::end).tellg();
  in1.seekg(0, std::ifstream::beg);

  size2 = in2.seekg(0, std::ifstream::end).tellg();
  in2.seekg(0, std::ifstream::beg);

  if (size1 != size2)
  {
    return false;
  }

  static const size_t BLOCKSIZE = 4096;
  size_t remaining = static_cast<size_t>(size1);

  while (remaining)
  {
    char buffer1[BLOCKSIZE], buffer2[BLOCKSIZE];
    size_t size = (std::min)(BLOCKSIZE, remaining); // Brackets needed for MSVC

    in1.read(buffer1, size);
    in2.read(buffer2, size);

    if (0 != memcmp(buffer1, buffer2, size))
    {
      return false;
    }

    remaining -= size;
  }

  return true;
}



std::vector<std::string> String_split(const std::string& toBeSplitted,
                                      const std::string& delim)
{
  std::vector<std::string> splittedString;
  size_t startIdx = 0, endIdx = 0;

  // Border case: If toBeSplitted is empty, we return an empty string.
  if (toBeSplitted.empty())
  {
    splittedString.push_back(toBeSplitted);
    return splittedString;
  }

  if (!delim.empty())
  {
    while ((endIdx = toBeSplitted.find(delim, startIdx)) != std::string::npos)
    {
      if (startIdx < endIdx) // ignore zero-length substrings
      {
        std::string val = toBeSplitted.substr(startIdx, endIdx - startIdx);
        splittedString.push_back(val);
      }
      startIdx = endIdx + delim.size();
    }
  }
  else
  {
    splittedString.push_back(toBeSplitted);
  }

  if ((endIdx==std::string::npos) && (startIdx<toBeSplitted.size()))
  {
    splittedString.push_back(toBeSplitted.substr(startIdx));
  }

  return splittedString;
}
