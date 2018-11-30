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

#ifndef RCS_CMDLINE_H
#define RCS_CMDLINE_H

#include <map>
#include <string>
#include <cstdio>
#include <cstdarg>

namespace Rcs
{

/*!
 *           To each getArgument function, a descriptive text can be added.
 *           In this case, it will be displayed when calling the \ref print()
 *           function. If a tag has been parsed several times, the descriptive
 *           texts will be appended. In addition, a warning on debul level 4
 *           will be emitted.
 *
 *           Note: The copy pointers "result" must exist, and have sufficient
 *           memory. In case they are NULL, the compiler will dislike it since
 *           it can't figure out which function to call.
 *
 */
class CmdLineParser
{
public:

  /*!
   *  \brief Constructs a parser instance.
   */
  CmdLineParser(int argc, char** argv);

  /*!
   *  \brief Call this constructor only if the class has already been
   *         instantiated before. Otherwise, the constructor will exit
   *         fatally.
   */
  CmdLineParser();

  /*!
   *  \brief Returns the argument after string "tag" as a character array.
   *         The argument is copied into str, assuming str has sufficiently
   *         memory. <br>
   *         Example: Executable is called with <br>
   *         Rcs.exe -m 5 -dir xml/Humanoids08 -f cAction.xml <br>
   *         getArgumentTag("-m", (char*) myArg) <br>
   *         will copy 5 into character array myArg.
   */
  bool getArgument(const char* tag, char* result,
                   const char* description=NULL, ...) const;

  /*!
   *  \brief Returns the argument after string "tag" as a std::string.
   *         The argument is copied into result if is provided.
   *         Compared to getArgumentTag with char* it is memory save<br>
   *         Example: Executable is called with <br>
   *         Rcs.exe -m 5 -dir xml/Humanoids08 -f cAction.xml <br>
   *         getArgumentTag("-m", (std::string*) myArg) <br>
   *         will copy 5 into the string myArg.
   */
  bool getArgument(const char* tag, std::string* result = NULL,
                   const char* description=NULL, ...) const;

  /*!
   *  \brief Returns the argument after string "tag" as an integer. The
   *         argument is copied into res. <br>
   *         Example: Executable is called with <br>
   *         Rcs.exe -m 5 -dir xml/Humanoids08 -f cAction.xml <br>
   *         getArgumentTag("-m", (int*) myArg) <br>
   *         will copy 5 into integer myArg.
   */
  bool getArgument(const char* tag, int* result,
                   const char* description=NULL, ...) const;

  /*!
   *  \brief Returns the argument after string "tag" as an unsigned integer.
   *         If the tag contains a negative number, a warning on debug
   *         level 1 is issued.
   */
  bool getArgument(const char* tag, unsigned int* result,
                   const char* description=NULL, ...) const;

  /*!
   *  \brief Returns the argument after string "tag" as a size_t type.
   *         If the tag contains a negative number, a warning on debug
   *         level 1 is issued.
   */
#ifdef __64BIT__
  bool getArgument(const char* tag, size_t* result,
                   const char* description=NULL, ...) const;
#endif

  /*!
   *  \brief Returns the argument after string "tag" as a double.
   */
  bool getArgument(const char* tag, double* result,
                   const char* description=NULL, ...) const;

  /*!
   *  \brief Returns true if an argument with name tag exists, false
   *         otherwise.
   */
  bool hasArgument(const char* tag, const char* description=NULL, ...) const;

  /*!
   *  \brief Prints out all arguments contained in argv to stderr.
   */
  void printArguments() const;

  /*!
   *  \brief Prints out all parsed arguments to stderr.
   */
  static void print();

  /*!
   *  \brief Adds a command line description that will be printed to the
   *         console.
   */
  void addDescription(const char* tag, const char* description, ...) const;



protected:

  void appendDescription(const char* tag, const char* description,
                         va_list args) const;
  int getTagIndex(const char* tag) const;
  static int argc;
  static char** argv;
  static std::map<std::string, std::string> parsedArguments;
};

}

#endif   // RCS_CMDLINE_H
