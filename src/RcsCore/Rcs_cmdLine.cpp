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
  if (!argv)
  {
    RLOG(1, "You created a CmdLineParser with no arguments before "
         "instantiating it with argc and argv!");
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::CmdLineParser::~CmdLineParser()
{
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::CmdLineParser::getArgument(const char* tag, char* str,
                                     const char* description, ...)
{
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

  strcpy(str, argv[idx + 1]);

  return true;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::CmdLineParser::getArgument(const char* tag, std::string* result,
                                     const char* description, ...)
{
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

  *result = std::string(argv[idx + 1]);

  return true;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::CmdLineParser::getArgument(const char* tag, int* res,
                                     const char* description, ...)
{
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

  *res = atoi(argv[idx + 1]);
  RCHECK((*res>INT_MIN) && (*res<INT_MAX));

  return true;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::CmdLineParser::getArgument(const char* tag, unsigned int* res,
                                     const char* description, ...)
{
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

  int number = atoi(argv[idx + 1]);
  RCHECK((number>INT_MIN) && (number<INT_MAX));

  // This can lead to very hard to find bugs so that we exit here.
  RCHECK_MSG(number >= 0, "You are trying to read a negative number (%d) "
             "into an unsigned int variable", number);
  *res = (unsigned int) number;

  return true;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::CmdLineParser::getArgument(const char* tag, unsigned long* res,
                                     const char* description, ...)
{
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

  int number = atoi(argv[idx + 1]);
  RCHECK((number>INT_MIN) && (number<INT_MAX));

  // This can lead to very hard to find bugs so that we exit here.
  RCHECK_MSG(number >= 0, "You are trying to read a negative number (%d) "
             "into an unsigned int variable", number);
  *res = (size_t) number;

  return true;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::CmdLineParser::getArgument(const char* tag, unsigned long long* res,
                                     const char* description, ...)
{
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

  int number = atoi(argv[idx + 1]);
  RCHECK((number > INT_MIN) && (number < INT_MAX));

  // This can lead to very hard to find bugs so that we exit here.
  RCHECK_MSG(number >= 0, "You are trying to read a negative number (%d) "
             "into an unsigned int variable", number);
  *res = (size_t)number;

  return true;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::CmdLineParser::getArgument(const char* tag, double* res,
                                     const char* description, ...)
{
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

  *res = strtof(argv[idx + 1], NULL);
  RCHECK((*res>-DBL_MAX) && (*res<DBL_MAX));

  return true;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::CmdLineParser::getArgument(const char* tag, float* res,
                                     const char* description, ...)
{
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

  *res = strtof(argv[idx + 1], NULL);
  RCHECK((*res>-FLT_MAX) && (*res<FLT_MAX));

  return true;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::CmdLineParser::getArgument(const char* tag, bool* res,
                                     const char* description, ...)
{
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
    // Here we check if the last string in the arguments vector corresponds to
    // the tag. In this case, we return true.
    if (STREQ(argv[argc-1], tag))
    {
      *res = true;
      return true;
    }

    return false;
  }

  const char* arg = argv[idx + 1];

  // This catches also the case that there is no "true" or "false" given, but
  // another argument follows this one directly. We therefore check for the
  // next argument starting with the '-' character, indicating another command
  // line argument.
  if (STRCASEEQ(arg, "true") || (arg[0]=='-'))
  {
    *res = true;
  }

  return true;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::CmdLineParser::hasArgument(const char* tag,
                                     const char* description, ...)
{
  if (description != NULL)
  {
    va_list args;
    va_start(args, description);
    appendDescription(tag, description, args);
    va_end(args);
  }

  if (!tag)
  {
    RLOGS(4, "Tag is NULL - skipping");
    return false;
  }

  // Here we don't use the getTagIndex() function since the tag does not have
  // any content here and theoretically could be at the last index.
  for (int i = 0; i < argc; i++)
  {
    if (STREQ(argv[i], tag))
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
 * See header. This method is static, therefore not const.
 ******************************************************************************/
void Rcs::CmdLineParser::print()
{
  std::string res = printToString();
  printf("%s", res.c_str());
}

/*******************************************************************************
 * See header. This method is static, therefore not const.
 ******************************************************************************/
std::string Rcs::CmdLineParser::printToString()
{
  std::string res = "Parsed arguments:\n\n";
  std::map<std::string, std::string>::iterator it;

  size_t tagWidth = 0;

  for (it = parsedArguments.begin(); it != parsedArguments.end(); ++it)
  {
    tagWidth = std::max(tagWidth, strlen(it->first.c_str()));
  }

  tagWidth += 5;

  for (it = parsedArguments.begin(); it != parsedArguments.end(); ++it)
  {
    res += '\t';
    res += it->first;

    int whiteSpaces = tagWidth-strlen(it->first.c_str());

    for (int i=0; i<whiteSpaces; i++)
    {
      res += " ";
    }

    res += it->second;
    res += '\n';
  }

  res += '\n';

  return res;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Rcs::CmdLineParser::addDescription(const char* tag,
                                        const char* description,
                                        ...)
{
  if (!tag)
  {
    RLOG(4, "Tag is NULL - skipping");
    return;
  }

  if (description != NULL)
  {
    va_list args;
    va_start(args, description);
    appendDescription(tag, description, args);
    va_end(args);
  }
}

/*******************************************************************************
 * \todo: Multiple descriptions with the same content gat appended too.
 ******************************************************************************/
void Rcs::CmdLineParser::appendDescription(const char* tag,
                                           const char* description,
                                           va_list args)
{
  if (!description)
  {
    RLOG(4, "Failed to add description to command line argument %s",
         tag ? tag : "NULL");
    return;
  }

  // Only when this returned value is non-negative and less than sizeof(buffer),
  // the string has been completely written.
  char buffer[512] = "";
  std::string stag = std::string(tag);
  int res = vsnprintf(buffer, sizeof(buffer), description, args);
  if (res<0)
  {
    RLOG(4, "Description for argument \"%s\" not written correctly due to "
         "encoding error", tag);
  }
  else if (res>=(int)sizeof(buffer))
  {
    RLOG(4, "Description for argument \"%s\" not complete (%d of %zd bytes)",
         tag, res, sizeof(buffer));
  }

  if (parsedArguments.find(stag) == parsedArguments.end())
  {
    parsedArguments[stag] = std::string(buffer);
  }
  else
  {
    parsedArguments[stag] += " / ";
    parsedArguments[stag] += std::string(buffer);
  }

}

/*******************************************************************************
 * Here we go only through argc-2 tags, since we assume that the content of
 * the tag is at worst at index argc-1.
 ******************************************************************************/
int Rcs::CmdLineParser::getTagIndex(const char* tag) const
{
  if (!tag)
  {
    return -1;
  }

  for (int i = 0; i < argc-1; i++)
  {
    if (STREQ(argv[i], tag))
    {
      return i;
    }
  }

  return -1;
}

/*******************************************************************************
 *
 ******************************************************************************/
int Rcs::CmdLineParser::getArgs(char** * argv_) const
{
  *argv_ = argv;

  return argc;
}
