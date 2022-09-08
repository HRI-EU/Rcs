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
 *           Note: The copy pointers "result" must not be NULL, and point to
 *           sufficient memory. In case they are NULL, the compiler cannot
 *           deduce the type and therefore not find the correct function to
 *           call.
 *
 */
class CmdLineParser
{
public:

  /*! \brief Constructs a parser instance.
   */
  CmdLineParser(int argc, char** argv);

  /*! \brief Call this constructor only if the class has already been
   *         instantiated before. Otherwise, the constructor will exit
   *         fatally.
   */
  CmdLineParser();

  /*! \brief Enable polymorphic destruction.
   */
  virtual ~CmdLineParser();

  /*! \brief Returns the argument after string "tag" as a character array.
   *         The argument is copied into str, assuming str has sufficiently
   *         memory. <br>
   *         Example: Executable is called with <br>
   *         Rcs.exe -m 5 -dir xml/Examples -f cAction.xml <br>
   *         char dir[256]; <br>
   *         getArgumentTag("-dir", dir) <br>
   *         will copy "xml/Examples" into character array dir.
   */
  virtual bool getArgument(const char* tag, char* result,
                           const char* description=NULL, ...);

  /*! \brief Returns the argument after string "tag" as a std::string.
   *         The argument is copied into result if is provided.
   *         Compared to getArgumentTag with char* it is memory save<br>
   *         Example: Executable is called with <br>
   *         Rcs.exe -m 5 -dir xml/Humanoids08 -f cAction.xml <br>
   *         std::string myArg; <br>
   *         getArgumentTag("-f", &myArg) <br>
   *         will copy "cAction.xml" into the string myArg.
   */
  virtual bool getArgument(const char* tag, std::string* result,
                           const char* description=NULL, ...);

  /*! \brief Returns the argument after string "tag" as an integer. The
   *         argument is copied into res. <br>
   *         Example: Executable is called with <br>
   *         Rcs.exe -m 5 -dir xml/Humanoids08 -f cAction.xml <br>
   *         int myArg; <br>
   *         getArgumentTag("-m", &myArg) <br>
   *         will copy 5 into integer myArg.
   */
  virtual bool getArgument(const char* tag, int* result,
                           const char* description=NULL, ...);

  /*! \brief Returns the argument after string "tag" as an unsigned integer.
   *         If the tag contains a negative number, a warning on debug
   *         level 1 is issued.
   */
  virtual bool getArgument(const char* tag, unsigned int* result,
                           const char* description=NULL, ...);

  /*! \brief Returns the argument after string "tag" as a unsigned long type.
   *         If the tag contains a negative number, a warning on debug
   *         level 1 is issued.
   */
  virtual bool getArgument(const char* tag, unsigned long* result,
                           const char* description = NULL, ...);

  /*! \brief Returns the argument after string "tag" as a unsigned long long
   *         type. On 64 bit operating systems, the size_t is of the same type.
   *         If the tag contains a negative number, a warning on debug
   *         level 1 is issued.
   */
  virtual bool getArgument(const char* tag, unsigned long long* result,
                           const char* description=NULL, ...);

  /*! \brief Returns the argument after string "tag" as a boolean. Tag must
   *         be "true" (case not considered) in order to become true. All
   *         other attributes will leave result unchanged. The function also
   *         considers some cases without an explicit argument specifier. In
   *         the case where tag is the last entry in the command line args,
   *         and in the case when the next tag starts with a '-', the function
   *         sets result to "true".
   */
  virtual bool getArgument(const char* tag, bool* result,
                           const char* description=NULL, ...);

  /*! \brief Returns the argument after string "tag" as a double.
   */
  virtual bool getArgument(const char* tag, double* result,
                           const char* description=NULL, ...);

  /*! \brief Returns the argument after string "tag" as a float.
   */
  virtual bool getArgument(const char* tag, float* result,
                           const char* description=NULL, ...);

  /*! \brief Returns true if an argument with name tag exists, false
   *         otherwise.
   */
  virtual bool hasArgument(const char* tag, const char* description=NULL, ...);

  /*! \brief Prints out all arguments contained in argv to stderr.
   */
  virtual void printArguments() const;

  /*! \brief Prints out all parsed arguments to stderr.
   */
  static void print();

  /*! \brief Prints out all parsed arguments to a returned string.
   */
  static std::string printToString();

  /*! \brief Adds a command line description that will be printed to the
   *         console.
   */
  virtual void addDescription(const char* tag, const char* description, ...);

  /*! \brief Query the arguments vector and number of arguments. If they have
   *         not been parsed, argc is 0 and argv points to NULL.
   */
  virtual int getArgs(char** * argv) const;


protected:

  virtual void appendDescription(const char* tag, const char* description,
                                 va_list args) ;
  int getTagIndex(const char* tag) const;
  static int argc;
  static char** argv;
  static std::map<std::string, std::string> parsedArguments;
};

}

#endif   // RCS_CMDLINE_H
