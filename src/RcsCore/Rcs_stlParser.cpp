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

#include "Rcs_stlParser.h"
#include "Rcs_macros.h"
#include "Rcs_parser.h"
#include "Rcs_utils.h"

#include <sstream>
#include <cstdlib>


namespace Rcs
{

/*******************************************************************************
 *
 ******************************************************************************/
size_t getXMLNodePropertySTLString(xmlNodePtr node, const char* tag,
                                   std::string& str)
{
  size_t len = 0;
  xmlChar* txt = xmlGetProp(node, (const xmlChar*) tag);

  if (txt)
  {
    len = strlen((const char*) txt) + 1;    // +1 to account for trailing '\0'
    str = (const char*) txt;
  }

  xmlFree(txt);

  return len;
}

/*******************************************************************************
 *
 ******************************************************************************/
std::string getXMLNodePropertySTLString(xmlNodePtr node, const char* tag)
{
  std::string str;
  getXMLNodePropertySTLString(node, tag, str);
  return str;
}

/*******************************************************************************
 *
 ******************************************************************************/
static size_t getXMLNodePropertyVecSTLString(xmlNodePtr node, const char* tag,
                                             std::vector<std::string>& vec)
{
  std::string tmp;
  size_t len = getXMLNodePropertySTLString(node, tag, tmp);

  if (len > 0)
  {
    std::istringstream iss(tmp);
    std::string token;
    iss >> std::ws;
    while (std::getline(iss, token, ' '))
    {
      vec.push_back(token);
      iss >> std::ws;
    }
  }

  return len;
}

/*******************************************************************************
 *
 ******************************************************************************/
std::vector<std::string> getXMLNodePropertyVecSTLString(xmlNodePtr node,
                                                        const char* tag)
{
  std::vector<std::string> str;
  getXMLNodePropertyVecSTLString(node, tag, str);
  return str;
}

/*******************************************************************************
 *
 ******************************************************************************/
size_t getXMLNodePropertyVecSTLDouble(xmlNodePtr node, const char* tag,
                                      std::vector<double>& vec)
{
  std::vector<double> dVec = getXMLNodePropertyVecSTLDouble(node, tag);

  if (dVec.empty())
  {
    return 0;
  }

  vec = dVec;

  return dVec.size();
}

/*******************************************************************************
 *
 ******************************************************************************/
std::vector<double> getXMLNodePropertyVecSTLDouble(xmlNodePtr node,
                                                   const char* tag)
{
  std::vector<std::string> str = getXMLNodePropertyVecSTLString(node, tag);

  if (str.empty())
  {
    return std::vector<double>();
  }

  std::vector<double> vec(str.size());

  for (size_t i=0; i<vec.size(); ++i)
  {
    vec[i] = atof(str[i].c_str());
  }

  return vec;
}

}  // namespace Rcs
