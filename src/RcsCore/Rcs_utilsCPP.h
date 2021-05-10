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

#ifndef RCS_UTILSCPP_H
#define RCS_UTILSCPP_H

#include "Rcs_graph.h"

#include <libxml/tree.h>

#include <string>
#include <vector>
#include <list>
#include <utility>
#include <cstdarg>



/*! \ingroup RcsUtilsFunctions
 *  \brief Returns a list of string containing all file names of given directory
 *         that end with a given extension. If returnFullPath is true (default)
 *         then the full path including the provided directory name is returned
 */
std::list<std::string> getFilenamesInDirectory(const std::string& dirname,
                                               bool returnFullPath=true,
                                               const std::string& extension="");

/*! \ingroup RcsUtilsFunctions
 * \brief Returns the path and the filename of the currently executed binary
 * \param argv The argument list coming from the main()
 * \return A pair holding the path and the filename of the currently executed
 *         binary
 */
std::pair<std::string, std::string> Rcs_getExecutablePathAndFilename(char* argv[]);

/*! \ingroup RcsUtilsFunctions
 *  \brief Method for formatting a std::string in the fprintf style
 *  \param fmt Format string + variable arguments
 *  \return String generated from format + arguments
 *
 *  This function uses formatStdString(const char *fmt, va_list ap)
 */
std::string formatStdString(const char* fmt, ...);

/*! \ingroup RcsUtilsFunctions
 *  \brief Method for formatting a std::string in the fprintf style
 *  \param fmt Format string
 *  \param ap Variable argument list already started with va_start
 *  \return String generated from format + arguments
 *
 *  Note that call va_end is the responsibility of the user.
 */
std::string formatStdString(const char* fmt, va_list ap);

/*! \ingroup RcsUtilsFunctions
 *  \brief Checks if two files are equal. Thir binary content is compared.
 *  \param file1 Filename 1
 *  \param file2 Filename 2
 *  \return True if files are equal
 */
bool File_isEqualCpp(const char* file1, const char* file2);

/*! \ingroup RcsUtilsFunctions
 *  \brief Splits the given string into pieces that are separated by the given
 *         delimiter.
 *
 *  \param stringToBeSplitted The string that is to be splitted.
 *  \param delimiter Pattern that spearates the sub-strings
 *  \return Vector of sub-strings without delimiter.
 */
std::vector<std::string> String_split(const std::string& stringToBeSplitted,
                                      const std::string& delimiter);

/*! \ingroup RcsUtilsFunctions
 *  \brief Checks if a given string ends with another string. Basically for
 *         checking if a file has a given extension
 */
bool String_hasEnding(const std::string& fullString,
                      const std::string& ending);

/*! \ingroup RcsUtilsFunctions
 * \brief Checks if a given string starts with another string.
 */
bool String_startsWith(const std::string& fullString,
                       const std::string& beginning);

std::vector<std::pair<double,double>> Math_snapToGridPolygon2D(double polygon[][2],
                                                               unsigned int nVertices,
                                                               double gridSize);

std::vector<std::pair<double,double>> Math_quadsFromPolygon2D(double polygon[][2],
                                                              unsigned int nVertices,
                                                              double gridSize);

std::vector<std::pair<int,double>> RcsGraph_readModelState(xmlNodePtr node,
                                                           const RcsGraph* self,
                                                           const std::string& mdlName);

std::vector<std::string> RcsGraph_getModelStateNames(const RcsGraph* graph);

#endif   // RCS_UTILSCPP_H
