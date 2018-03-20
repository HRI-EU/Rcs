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

#include "Rcs_cmdLine.h"
#include "Rcs_macros.h"

#include <algorithm>
#include <iostream>
#include <cstring>
#include <climits>
#include <cfloat>



std::map<std::string, std::string> Rcs::CmdLineParser::parsedArguments;
int Rcs::CmdLineParser::argc = 0;
char** Rcs::CmdLineParser::argv = NULL;



/*******************************************************************************
 * See header.
 ******************************************************************************/
Rcs::CmdLineParser::CmdLineParser(int argc_, char** argv_)
{
  argc = argc_;
  argv = argv_;

  RCHECK(argc >= 0);
  RCHECK(argv);

  for (int i = 0; i < argc; i++)
  {
    RCHECK(argv[i]);
  }

}

/*******************************************************************************
 * See header.
 ******************************************************************************/
Rcs::CmdLineParser::CmdLineParser()
{
  if (argv == NULL)
  {
    RFATAL("You created a CmdLineParser with no arguments before "
           "instantiating it with argc and argv!");
  }
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::CmdLineParser::getArgument(const char* tag, char* str,
                                     const char* description, ...) const
{
  if (tag==NULL)
  {
    RLOG(4, "Tag is NULL - skipping");
    return false;
  }

  if (description != NULL)
  {
    va_list args;
    va_start(args, description);
    appendDescription(tag, description, args);
    va_end(args);
  }

  int idx = getTagIndex(tag);

  if (idx == -1)
  {
    return false;
  }

  if (str != NULL)
  {
    strcpy(str, argv[idx + 1]);
  }

  return true;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::CmdLineParser::getArgument(const char* tag, std::string* result,
                                     const char* description, ...) const
{
  if (tag==NULL)
  {
    RLOG(4, "Tag is NULL - skipping");
    return false;
  }

  if (description != NULL)
  {
    va_list args;
    va_start(args, description);
    appendDescription(tag, description, args);
    va_end(args);
  }

  int idx = getTagIndex(tag);

  if (idx == -1)
  {
    return false;
  }

  if (result != NULL)
  {
    *result = std::string(argv[idx + 1]);
  }

  return true;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::CmdLineParser::getArgument(const char* tag, int* res,
                                     const char* description, ...) const
{
  if (tag==NULL)
  {
    RLOG(4, "Tag is NULL - skipping");
    return false;
  }

  if (description != NULL)
  {
    va_list args;
    va_start(args, description);
    appendDescription(tag, description, args);
    va_end(args);
  }

  int idx = getTagIndex(tag);

  if (idx == -1)
  {
    return false;
  }

  if (res != NULL)
  {
    *res = atoi(argv[idx + 1]);
    RCHECK((*res>INT_MIN) && (*res<INT_MAX));
  }
  return true;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::CmdLineParser::getArgument(const char* tag, unsigned int* res,
                                     const char* description, ...) const
{
  if (tag==NULL)
  {
    RLOG(4, "Tag is NULL - skipping");
    return false;
  }

  if (description != NULL)
  {
    va_list args;
    va_start(args, description);
    appendDescription(tag, description, args);
    va_end(args);
  }

  int idx = getTagIndex(tag);

  if (idx == -1)
  {
    return false;
  }

  if (res != NULL)
  {
    int number = atoi(argv[idx + 1]);
    RCHECK((number>INT_MIN) && (number<INT_MAX));

    // This can lead to very hard to find bugs so that we exit here.
    RCHECK_MSG(number >= 0, "You are trying to read a negative number (%d) "
               "into an unsigned int variable", number);
    *res = (unsigned int) number;
  }
  return true;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
#ifdef __64BIT__
bool Rcs::CmdLineParser::getArgument(const char* tag, size_t* res,
                                     const char* description, ...) const
{
  if (tag==NULL)
  {
    RLOG(4, "Tag is NULL - skipping");
    return false;
  }

  if (description != NULL)
  {
    va_list args;
    va_start(args, description);
    appendDescription(tag, description, args);
    va_end(args);
  }

  int idx = getTagIndex(tag);

  if (idx == -1)
  {
    return false;
  }

  if (res != NULL)
  {
    int number = atoi(argv[idx + 1]);
    RCHECK((number>INT_MIN) && (number<INT_MAX));

    // This can lead to very hard to find bugs so that we exit here.
    RCHECK_MSG(number >= 0, "You are trying to read a negative number (%d) "
               "into an unsigned int variable", number);
    *res = (size_t) number;
  }
  return true;
}
#endif

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::CmdLineParser::getArgument(const char* tag, double* res,
                                     const char* description, ...) const
{
  if (tag==NULL)
  {
    RLOG(4, "Tag is NULL - skipping");
    return false;
  }

  if (description != NULL)
  {
    va_list args;
    va_start(args, description);
    appendDescription(tag, description, args);
    va_end(args);
  }

  int idx = getTagIndex(tag);

  if (idx == -1)
  {
    return false;
  }

  if (res != NULL)
  {
    *res = atof(argv[idx + 1]);
    RCHECK((*res>-DBL_MAX) && (*res<DBL_MAX));
  }

  return true;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::CmdLineParser::hasArgument(const char* tag,
                                     const char* description, ...) const
{
  if (tag==NULL)
  {
    RLOGS(4, "Tag is NULL - skipping");
    return false;
  }

  if (description != NULL)
  {
    va_list args;
    va_start(args, description);
    appendDescription(tag, description, args);
    va_end(args);
  }

  // Here we don't use the getTagIndex() function since the tag does not have
  // any content here and theoretically could be at the last index.
  for (int i = 0; i < argc; i++)
  {
    if (std::string(argv[i]) == std::string(tag))
    {
      return true;
    }
  }

  return false;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Rcs::CmdLineParser::printArguments() const
{
  printf("CmdLineParser has %d arguments\n", argc);

  for (int i = 0; i < argc; i++)
  {
    printf("Argument %d is \"%s\"\n", i, argv[i]);
  }

}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Rcs::CmdLineParser::print()
{
  printf("Parsed arguments:\n\n");
  std::map<std::string, std::string>::iterator it;

  size_t tagWidth = 0;

  for (it = parsedArguments.begin(); it != parsedArguments.end(); ++it)
  {
    tagWidth = std::max(tagWidth, strlen(it->first.c_str()));
  }

  tagWidth += 5;

  for (it = parsedArguments.begin(); it != parsedArguments.end(); ++it)
  {
    printf("\t%s", it->first.c_str());

    int whiteSpaces = tagWidth-strlen(it->first.c_str());

    for (int i=0; i<whiteSpaces; i++)
    {
      printf(" ");
    }

    printf("%s\n", it->second.c_str());
  }

  printf("\n");
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Rcs::CmdLineParser::appendDescription(const char* tag,
                                           const char* description,
                                           va_list args) const
{
  RCHECK(description != NULL);

  char buffer[512] = "";
  int writtenBytes = vsprintf(buffer, description, args);
  RCHECK(writtenBytes<512);

  if (parsedArguments.find(std::string(tag)) == parsedArguments.end())
  {
    parsedArguments[std::string(tag)] = std::string(buffer);
  }
  else
  {
    parsedArguments[std::string(tag)] += " / ";
    parsedArguments[std::string(tag)] += std::string(buffer);
  }

}

/*******************************************************************************
 * Here we go only through argc-2 tags, since we assume that the content of
 * the tag is at worst at index argc-1.
 ******************************************************************************/
int Rcs::CmdLineParser::getTagIndex(const char* tag) const
{
  for (int i = 0; i < argc-1; i++)
  {
    if (std::string(argv[i]) == std::string(tag))
    {
      return i;
    }
  }

  return -1;
}
