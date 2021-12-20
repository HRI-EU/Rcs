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

#ifndef RCS_STLPARSER_H
#define RCS_STLPARSER_H

#include <libxml/tree.h>

#include <string>
#include <vector>


namespace Rcs
{

/*! \ingroup RcsParserFunctions
 *  \brief Read node property into a STL string
 *
 *  \param[in] node The xml node to read from
 *  \param[in] tag The name of node property to read
 *  \param[out] str Reference to std::string object that gets the result
 *
 *  \return number of read bytes (including trailing '\\0')
 */
size_t getXMLNodePropertySTLString(xmlNodePtr node, const char* tag,
                                   std::string& str);


/*! \ingroup RcsParserFunctions
 *  \brief Read node property into a STL string
 *
 *  \param[in] node The xml node to read from
 *  \param[in] tag The name of node property to read
 *
 *  \return STL string with content of tag, or empty vector otherwise.
 */
std::string getXMLNodePropertySTLString(xmlNodePtr node, const char* tag);


/*! \ingroup RcsParserFunctions
 *  \brief Reads multiple words from xml node into a vector of STL strings
 *
 *  \param[in] node The xml node to read from
 *  \param[in] tag  The name of node property to read
 *
 *  \return Vector of std::strings containing the contents of what's in tag.
 */
std::vector<std::string> getXMLNodePropertyVecSTLString(xmlNodePtr node,
                                                        const char* tag);


/*! \ingroup RcsParserFunctions
 *  \brief Reads multiple double values from xml node into a STL vector
 *
 *  \param[in] node The xml node to read from
 *  \param[in] tag  The name of node property to read
 *
 *  \return Vector of double values found in the xml tag
 */
std::vector<double> getXMLNodePropertyVecSTLDouble(xmlNodePtr node,
                                                   const char* tag);


/*! \ingroup RcsParserFunctions
 *  \brief Reads multiple double values from xml node into a STL vector
 *
 *  \param[in]  node The xml node to read from
 *  \param[in]  tag  The name of node property to read
 *  \param[out] vec  Reference to a vector of doubles containing the values
 *                   in tag.
 *
 *  \return Number of values in vec.
 */
size_t getXMLNodePropertyVecSTLDouble(xmlNodePtr node, const char* tag,
                                      std::vector<double>& vec);


}  // namespace Rcs

#endif   // RCS_STLPARSER_H
