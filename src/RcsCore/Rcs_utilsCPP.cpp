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

#include "Rcs_utilsCPP.h"
#include "Rcs_macros.h"
#include "Rcs_resourcePath.h"

#include <fstream>
#include <algorithm>
#include <cstdarg>

#if !defined(_MSC_VER)
#include <dirent.h>
#include <unistd.h>
#else
#include <Windows.h>
#include <cstdio>
#include <direct.h>
#define getcwd _getcwd
#endif


namespace Rcs
{


// Returns a list of files in a directory (except these that begin with a dot)
std::vector<std::string> File_getFilenamesInDirectory(const std::string& dirname,
                                                      bool returnFullPath,
                                                      const std::string& extension)
{
  std::vector<std::string> files;

#if defined(_MSC_VER)

  HANDLE hFind;
  WIN32_FIND_DATA FindFileData;

  std::string pattern = dirname + "/*" + extension;
  if ((hFind = FindFirstFile(pattern.c_str(), &FindFileData)) != INVALID_HANDLE_VALUE)
  {
    do
    {
      std::string fileName;

      if (returnFullPath)
      {
        fileName = dirname + "/";
      }

      fileName += FindFileData.cFileName;
      files.push_back(fileName);
    }
    while (FindNextFile(hFind, &FindFileData));
    FindClose(hFind);
  }

  return files;

#else

  DIR* pDIR;
  struct dirent* entry;
  if ((pDIR = opendir(dirname.c_str())))
  {
    while ((entry = readdir(pDIR)))
    {
      std::string file = entry->d_name;
      if (file.compare(".") != 0 &&
          file.compare("..") != 0 &&
          String_endsWith(file, extension))
      {
        if (returnFullPath)
        {
          if (!String_endsWith(dirname, "/"))
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

std::pair<std::string, std::string> File_getExecutablePathAndFilename(char* argv[])
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

std::string File_getCurrentWorkingDir()
{
  char buff[FILENAME_MAX];
  char* res = getcwd(buff, FILENAME_MAX);

  if (!res)
  {
    return std::string();
  }

  std::string directoryName(buff);
  return directoryName;
}

bool String_endsWith(const std::string& fullString,
                     const std::string& ending)
{
  if (fullString.length() >= ending.length())
  {
    return (0 == fullString.compare(fullString.length() - ending.length(),
                                    ending.length(), ending));
  }
  else
  {
    return false;
  }
}

bool String_startsWith(const std::string& fullString,
                       const std::string& beginning)
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

static std::string String_formatStdString(const char* fmt, va_list ap)
{
  // Copy 'ap' so we can safely compute needed size
  va_list ap_copy;
  va_copy(ap_copy, ap);

#if defined(_MSC_VER)
  // _vscprintf: returns the number of characters needed, *excluding* the null terminator
  int size = _vscprintf(fmt, ap_copy);
#else
  // vsnprintf with NULL buffer and 0 size: returns the number of characters needed, *excluding* the null terminator
  int size = vsnprintf(nullptr, 0, fmt, ap_copy);
#endif

  // Done with the copy
  va_end(ap_copy);

  // If an error occurs, size may be negative
  if (size < 0)
  {
    return std::string{};
  }

  // Allocate enough space for the formatted string plus the null terminator
  std::string result(static_cast<size_t>(size) + 1, '\0');

  // Perform the actual formatting with 'ap'
  int written = vsnprintf(&result[0], result.size(), fmt, ap);

  // Check for vsnprintf errors at runtime
  if (written < 0)
  {
    return std::string{};
  }

  // 'written' is the number of characters written (excluding the null terminator).
  // Resize so the string's length matches the number of characters we actually wrote.
  // This also discards the trailing '\0' from the .size() perspective
  // (though the memory is still there if you call result.data()).
  result.resize(static_cast<size_t>(written));

  return result;
}

std::string String_formatStdString(const char* fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  std::string label_string = String_formatStdString(fmt, ap);
  va_end(ap);

  return label_string;
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

std::string& String_ltrim(std::string& str, const std::string& chars)
{
  str.erase(0, str.find_first_not_of(chars));
  return str;
}

std::string& String_rtrim(std::string& str, const std::string& chars)
{
  str.erase(str.find_last_not_of(chars) + 1);
  return str;
}

std::string& String_trim(std::string& str, const std::string& chars)
{
  return String_ltrim(String_rtrim(str, chars), chars);
}

std::string String_concatenate(const std::vector<std::string>& words, std::string sep)
{
  std::string res;

  for (size_t i = 0; i < words.size(); ++i)
  {
    res += words[i];

    if (i < words.size() - 1)
    {
      res += sep;
    }
  }

  return res;
}


}   // namespace Rcs



#include "Rcs_geometry.h"
#include <utility>
#include <set>

namespace Rcs
{

static std::pair<int,int> getGridCell(const double xy[2], double gridSize)
{
  double sgnX = xy[0] >= 0.0 ? 1.0 : -1.0;
  double sgnY = xy[1] >= 0.0 ? 1.0 : -1.0;

  double gridX = int((0.5*gridSize*sgnX + xy[0])/gridSize);
  double gridY = int((0.5*gridSize*sgnY + xy[1])/gridSize);

  std::pair<int,int> cell(lround(gridX),lround(gridY));

  return cell;
}

static std::vector<std::pair<double,double>> getCellVertices(int x, int y, double gridSize)
{
  double cntr[2];
  cntr[0] = gridSize*x;
  cntr[1] = gridSize*y;

  std::vector<std::pair<double,double>> verts;

  const double halfEdge = 0.5*gridSize;
  verts.push_back(std::pair<double,double>(cntr[0]+halfEdge, cntr[1]+halfEdge));
  verts.push_back(std::pair<double,double>(cntr[0]-halfEdge, cntr[1]+halfEdge));
  verts.push_back(std::pair<double,double>(cntr[0]-halfEdge, cntr[1]-halfEdge));
  verts.push_back(std::pair<double,double>(cntr[0]+halfEdge, cntr[1]-halfEdge));

  return verts;
}

struct polyVertCompare
{
  bool operator()(std::pair<double,double> a, std::pair<double,double> b) const
  {
    double phiA = atan2(a.first, a.second);
    double phiB = atan2(b.first, b.second);

    if (phiA==phiB)
    {
      double lenA = a.first*a.first + a.second*a.second;
      double lenB = b.first*b.first + b.second*b.second;

      if (lenA!=lenB)
      {
        // phiA += 1.0e-8*lenA;
        // phiB += 1.0e-8*lenB;

        if (lenA<lenB)
        {
          phiA -= 1.0e-8;
        }
        else
        {
          phiB -= 1.0e-8;
        }
      }


    }

    return phiA<phiB;
  }
};


void test()
{
  double xy[2];
  xy[0] = 0.0;
  xy[1] = 0.0;
  std::pair<int,int> ci = getGridCell(xy, 1.0);
  RLOG(0, "cell[%f, %f] = %d %d", xy[0], xy[1], ci.first, ci.second);

  xy[0] = -1.0e-8;
  xy[1] = 0.0;
  ci = getGridCell(xy, 1.0);
  RLOG(0, "cell[%f, %f] = %d %d", xy[0], xy[1], ci.first, ci.second);

  xy[0] = 0.0;
  xy[1] = -1.0e-8;
  ci = getGridCell(xy, 1.0);
  RLOG(0, "cell[%f, %f] = %d %d", xy[0], xy[1], ci.first, ci.second);

  xy[0] = 0.49;
  xy[1] = 0.49;
  ci = getGridCell(xy, 1.0);
  RLOG(0, "cell[%f, %f] = %d %d", xy[0], xy[1], ci.first, ci.second);

  xy[0] = -0.49;
  xy[1] = -0.49;
  ci = getGridCell(xy, 1.0);
  RLOG(0, "cell[%f, %f] = %d %d", xy[0], xy[1], ci.first, ci.second);

  xy[0] = 0.49;
  xy[1] = -0.01;
  ci = getGridCell(xy, 1.0);
  RLOG(0, "cell[%f, %f] = %d %d", xy[0], xy[1], ci.first, ci.second);

  xy[0] = 0.0;
  xy[1] = 2.49;
  ci = getGridCell(xy, 1.0);
  RLOG(0, "cell[%f, %f] = %d %d", xy[0], xy[1], ci.first, ci.second);

  xy[0] = 0.0;
  xy[1] = 2.51;
  ci = getGridCell(xy, 1.0);
  RLOG(0, "cell[%f, %f] = %d %d", xy[0], xy[1], ci.first, ci.second);

  xy[0] = 0.0;
  xy[1] = -2.49;
  ci = getGridCell(xy, 1.0);
  RLOG(0, "cell[%f, %f] = %d %d", xy[0], xy[1], ci.first, ci.second);

  xy[0] = 0.0;
  xy[1] = -2.51;
  ci = getGridCell(xy, 1.0);
  RLOG(0, "cell[%f, %f] = %d %d", xy[0], xy[1], ci.first, ci.second);



  xy[1] = 0.0;
  xy[0] = 2.49;
  ci = getGridCell(xy, 1.0);
  RLOG(0, "cell[%f, %f] = %d %d", xy[0], xy[1], ci.first, ci.second);

  xy[1] = 0.0;
  xy[0] = 2.51;
  ci = getGridCell(xy, 1.0);
  RLOG(0, "cell[%f, %f] = %d %d", xy[0], xy[1], ci.first, ci.second);

  xy[1] = 0.0;
  xy[0] = -2.49;
  ci = getGridCell(xy, 1.0);
  RLOG(0, "cell[%f, %f] = %d %d", xy[0], xy[1], ci.first, ci.second);

  xy[1] = 0.0;
  xy[0] = -2.51;
  ci = getGridCell(xy, 1.0);
  RLOG(0, "cell[%f, %f] = %d %d", xy[0], xy[1], ci.first, ci.second);
}

std::vector<std::pair<double,double>> Math_snapToGridPolygon2D(double polygon[][2],
                                                               unsigned int nVertices,
                                                               double gridSize)
{
  std::set<std::pair<double,double>, polyVertCompare> s;
  //std::set<std::pair<double,double>> s;

  for (unsigned int i=0; i<nVertices; ++i)
  {
    std::pair<int,int> cell = getGridCell(polygon[i], gridSize);
    std::vector<std::pair<double,double>> vi = getCellVertices(cell.first, cell.second, gridSize);

    for (size_t j=0; j<vi.size(); ++j)
    {
      // RLOG(0, "vi = %f %f", vi[j].first, vi[j].second);
      s.insert(vi[j]);
    }
  }

  std::set<std::pair<double,double>, polyVertCompare>::iterator it;
  std::vector<std::pair<double,double>> result;

  for (it = s.begin(); it != s.end(); ++it)
  {
    std::pair<double,double> pi = *it;

    double pt[2];
    pt[0] = pi.first;
    pt[1] = pi.second;

    int ioa = Math_pointInsideOrOnPolygon2D(pt, polygon, nVertices);

    if (ioa%2==0 || ioa==-1)
    {
      result.push_back(pi);
    }

  }

  return result;
}

std::vector<std::pair<double,double>> Math_quadsFromPolygon2D(double polygon[][2],
                                                              unsigned int nVertices,
                                                              double gridSize)
{
  std::vector<std::pair<double,double>> result;

  for (unsigned int i=0; i<nVertices; ++i)
  {
    std::pair<int,int> cell = getGridCell(polygon[i], gridSize);
    std::vector<std::pair<double,double>> vi = getCellVertices(cell.first, cell.second, gridSize);

    for (size_t j=0; j<vi.size(); ++j)
    {
      result.push_back(vi[j]);
    }
  }


  return result;
}

} // namespace Rcs

/*******************************************************************************
 *
 ******************************************************************************/
#include "Rcs_parser.h"
#include "Rcs_stlParser.h"
#include "Rcs_typedef.h"
#include "Rcs_joint.h"


namespace Rcs
{

/*******************************************************************************
 * This is the return result: The integer index is the joint id (and not the
 * jointIndex since this might change when bodies get deleted), followed by
 * the corresponding joint position value, e.g.:
 * result["default"] = {[2, 0.0], [4, -M_PI_2], ...}
 ******************************************************************************/
static std::map<std::string, std::vector<std::pair<int, double>>>
RcsGraph_getModelStatesFromNode(const RcsGraph* graph,
                                xmlNodePtr node,
                                std::string modelStateName)
{
  std::map<std::string, std::vector<std::pair<int, double>>> result;

  while (node)
  {

    if (isXMLNodeName(node, "Group") || isXMLNodeName(node, "Graph"))
    {
      auto grpRes = RcsGraph_getModelStatesFromNode(graph, node->children,
                                                    modelStateName);
      // Insert does not overwrite previous entries.
      // result.insert(grpRes.begin(), grpRes.end());

      // This overwrites previous entries
      for (const auto& grpEle : grpRes)
      {
        result[grpEle.first] = grpEle.second;
      }
    }

    if ((node->type != XML_ELEMENT_NODE) ||
        (!isXMLNodeNameNoCase(node, "model_state")))
    {
      node = node->next;
      continue;
    }

    std::string stateName = Rcs::getXMLNodePropertySTLString(node, "model");

    if (stateName.empty())
    {
      RLOG(1, "Tag \"mode_state\" lacks \"model\" attribute - skipping");
      node = node->next;
      continue;
    }

    // Only get the one with modelStateName if that has been passed
    if (!modelStateName.empty() && modelStateName!=stateName)
    {
      node = node->next;
      continue;
    }

    std::vector<std::pair<int, double>> jointStateItem;

    xmlNodePtr jsNode = node->children;

    while (jsNode)
    {
      if (!isXMLNodeNameNoCase(jsNode, "joint_state"))
      {
        jsNode = jsNode->next;
        continue;
      }

      std::string jntName = Rcs::getXMLNodePropertySTLString(jsNode, "joint");
      const RcsJoint* jnt = RcsGraph_getJointByName(graph, jntName.c_str());
      if (!jnt)
      {
        RLOG(4, "Joint \"%s\" not found", jntName.c_str());
        jsNode = jsNode->next;
        continue;
      }

      double q;
      bool hasPos = getXMLNodePropertyDouble(jsNode, "position", &q);

      if (hasPos)
      {
        if (jnt->coupledToId != -1)
        {
          RLOG(4, "You are setting the state of a kinematically coupled"
               " joint (\"%s\") - this has no effect", jnt->name);
        }

        // We don't overwrite the q_init value here, since it has
        // influence on the coupled joints.
        q *= RcsJoint_isRotation(jnt) ? (M_PI / 180.0) : 1.0;
        std::pair<int, double> si(jnt->id, q);
        jointStateItem.push_back(si);

      }   // if (hasPos==true)

      double qd;
      bool hasVel = getXMLNodePropertyDouble(jsNode, "velocity", &qd);

      if (hasVel)
      {
        RLOG(1, "Parsing velocities in model_state not yet supported");
      }


      jsNode = jsNode->next;
    }

    result[stateName] = jointStateItem;

    node = node->next;
  }

  return result;
}

/*******************************************************************************
 * This is the return result: The integer index is the joint id (and not the
 * jointIndex since this might change when bodies get deleted), followed by
 * the corresponding joint position value, e.g.:
 * result["default"] = {[2, 0.0], [4, -M_PI_2], ...}
 ******************************************************************************/
static std::map<std::string, std::vector<std::pair<int, double>>>
RcsGraph_getModelStatesFromFile(const RcsGraph* graph, std::string modelStateName)
{
  std::map<std::string, std::vector<std::pair<int, double>>> result;

  if (!graph)
  {
    return result;
  }

  // Read XML file
  xmlDocPtr doc;
  xmlNodePtr node = parseXMLFile(graph->cfgFile, "Graph", &doc);

  if ((node == NULL) || (node->children == NULL))
  {
    xmlFreeDoc(doc);
    return result;
  }

  result = RcsGraph_getModelStatesFromNode(graph, node->children, modelStateName);


  xmlFreeDoc(doc);

  return result;
}

/*******************************************************************************
 * This is the return result: The integer index is the joint id (and not the
 * jointIndex since this might change when bodies get deleted), followed by
 * the corresponding joint position value, e.g.:
 * result["default"] = {[2, 0.0], [4, -M_PI_2], ...}
 ******************************************************************************/
std::map<std::string, std::vector<std::pair<int, double>>>
RcsGraph_getModelStates(const RcsGraph* graph)
{
  return RcsGraph_getModelStatesFromFile(graph, std::string());
}

/*******************************************************************************
 *
 ******************************************************************************/
std::vector<std::pair<int, double>> RcsGraph_getModelState(const RcsGraph* graph,
                                                           std::string modelStateName)
{
  std::map<std::string, std::vector<std::pair<int, double>>> result;
  result = RcsGraph_getModelStatesFromFile(graph, modelStateName);
  return result[modelStateName];
}

/*******************************************************************************
 *
 ******************************************************************************/
std::vector<std::pair<int,double>> RcsGraph_readModelState(xmlNodePtr node,
                                                           const RcsGraph* self,
                                                           const std::string& mdlName)
{
  std::vector<std::pair<int,double>> result;
  node = node->children;

  // Traverse all nodes
  while (node != NULL)
  {
    // Only consider nodes with attribute "model_state"
    if (!isXMLNodeNameNoCase(node, "model_state"))
    {
      node = node->next;
      continue;
    }

    std::string name = Rcs::getXMLNodePropertySTLString(node, "model");

    // Only consider nodes with the given mdlName
    if (name != mdlName)
    {
      node = node->next;
      continue;
    }

    xmlNodePtr jsNode = node->children;

    while (jsNode)
    {
      if (!isXMLNodeNameNoCase(jsNode, "joint_state"))
      {
        jsNode = jsNode->next;
        continue;
      }

      name = Rcs::getXMLNodePropertySTLString(jsNode, "joint");
      RcsJoint* jnt = RcsGraph_getJointByName(self, name.c_str());
      if (jnt == NULL)
      {
        RLOG(4, "Joint \"%s\" not found", name.c_str());
        jsNode = jsNode->next;
        continue;
      }

      double q;
      bool hasPos = getXMLNodePropertyDouble(jsNode, "position", &q);

      if (hasPos)
      {
        if (jnt->coupledToId != -1)
        {
          RLOG(4, "You are setting the state of a kinematically coupled"
               " joint (\"%s\") - this has no effect", jnt->name);
        }

        // We don't overwrite the q_init value here, since it has
        // influence on the coupled joints.
        q *= RcsJoint_isRotation(jnt) ? (M_PI/180.0) : 1.0;
        std::pair<int,double> si(jnt->jointIndex, q);
        result.push_back(si);

      }   // if (hasPos==true)

      double qd;
      bool hasVel = getXMLNodePropertyDouble(jsNode, "velocity", &qd);

      if (hasVel)
      {
        RLOG(1, "Parsing velocities in model_state not yet supported");
      }


      jsNode = jsNode->next;
    }

    node = node->next;
  }



  return result;
}

/*******************************************************************************
 *
 ******************************************************************************/
std::vector<std::string> RcsGraph_getModelStateNames(const RcsGraph* graph)
{
  std::vector<std::string> result;

  if (graph==NULL)
  {
    return result;
  }

  // Read XML file
  xmlDocPtr doc;
  xmlNodePtr node = parseXMLFile(graph->cfgFile, "Graph", &doc);

  if ((node==NULL) || (node->children==NULL))
  {
    xmlFreeDoc(doc);
    return result;
  }

  node = node->children;


  while (node != NULL)
  {
    if (!isXMLNodeNameNoCase(node, "model_state"))
    {
      node = node->next;
      continue;
    }

    std::string stateName;
    int len = Rcs::getXMLNodePropertySTLString(node, "model", stateName);
    RCHECK_MSG(len>0, "Tag \"mode_state\" lacks \"model\" attribute");
    result.push_back(stateName);
    node = node->next;
  }

  xmlFreeDoc(doc);

  return result;
}

/*******************************************************************************
 *
 ******************************************************************************/
std::vector<int> RcsGraph_getModelStateTimeStamps(const RcsGraph* graph,
                                                  const std::string& mdlName)
{
  std::vector<int>  result;

  if (graph==NULL)
  {
    return result;
  }

  // Read XML file
  xmlDocPtr doc;
  xmlNodePtr node = parseXMLFile(graph->cfgFile, "Graph", &doc);

  if ((node==NULL) || (node->children==NULL))
  {
    xmlFreeDoc(doc);
    return result;
  }

  node = node->children;


  while (node != NULL)
  {
    if (!isXMLNodeNameNoCase(node, "model_state"))
    {
      node = node->next;
      continue;
    }

    std::string stateName = Rcs::getXMLNodePropertySTLString(node, "model");

    if (stateName != mdlName)
    {
      node = node->next;
      continue;
    }

    int timeStamp = -1;
    getXMLNodePropertyInt(node, "time_stamp", &timeStamp);

    // Here we catch the case that the xml string is empty.
    if (getXMLNodeBytes(node, "time_stamp")==1)
    {
      timeStamp = -1;
    }

    if (timeStamp!=-1)
    {
      result.push_back(timeStamp);
    }

    node = node->next;
  }

  xmlFreeDoc(doc);

  return result;
}

std::string getResourcePaths()
{
  std::string res = "Resource paths:\n";
  std::vector<std::string> paths = getResourcePath();

  for (size_t i = 0; i < paths.size(); ++i)
  {
    res += '\t';
    res += paths[i];
    res += '\n';
  }

  res += '\n';

  return res;
}

std::string RcsGraph_printUsageToString(std::string xmlFile)
{
  std::string res;
  char cfgFile[RCS_MAX_FILENAMELEN];
  bool fileExists = Rcs_getAbsoluteFileName(xmlFile.c_str(), cfgFile);

  if (!fileExists)
  {
    RLOG_CPP(1, "XML file \"" << xmlFile << "\" not found in resource paths");
    return res;
  }

  xmlDocPtr docPtr;
  xmlNodePtr node = parseXMLFile(cfgFile, "Graph", &docPtr);

  if (node == NULL)
  {
    RLOG(1, "Node \"Graph\" not found in \"%s\"", cfgFile);
    return res;
  }

  xmlChar* usage = xmlGetProp(node, (const xmlChar*)"usage");

  if (usage)
  {
    res = "Graph usage description: \n";
    res += (char*)usage;
    res += '\n';
    xmlFree(usage);
  }
  else
  {
    res = "No graph usage description\n";
  }

  xmlFreeDoc(docPtr);

  return res;
}

}   // namespace Rcs


/*******************************************************************************
 * Prints the distance function array to a file
 ******************************************************************************/
#include <Rcs_shape.h>

namespace Rcs
{
std::string RcsShape_distanceFunctionsToString()
{
  std::string msg;
  char tmp[32];
  size_t nBytes = 0;

  for (size_t i = 0; i < RCSSHAPE_SHAPE_MAX; ++i)
  {
    const char* name_i = RcsShape_name(i);
    size_t nameLen = strlen(name_i);
    if (nameLen > nBytes)
    {
      nBytes = nameLen;
    }
  }
  nBytes += 4;

  for (size_t i = 0; i < nBytes + 2; ++i)
  {
    msg += " ";
  }

  for (int i = 0; i < RCSSHAPE_SHAPE_MAX; ++i)
  {
    if ((i == RCSSHAPE_NONE) || (i == RCSSHAPE_REFFRAME))
    {
      continue;
    }
    snprintf(tmp, 32, "%3d", i);
    msg += tmp;
  }
  msg += '\n';


  for (int i = 0; i < RCSSHAPE_SHAPE_MAX; ++i)
  {
    if ((i == RCSSHAPE_NONE) || (i == RCSSHAPE_REFFRAME))
    {
      continue;
    }
    const char* name_i = RcsShape_name(i);
    snprintf(tmp, 32, "%2d %s", i, name_i);
    msg += tmp;
    for (size_t k = 0; k < nBytes - strlen(name_i); ++k)
    {
      msg += " ";
    }

    for (int j = 0; j < RCSSHAPE_SHAPE_MAX; ++j)
    {
      if ((j == RCSSHAPE_NONE) || (j == RCSSHAPE_REFFRAME))
      {
        continue;
      }

      msg += (RcsShape_hasDistanceFunction(i, j) ? " - " : " o ");
    }
    msg += '\n';
  }

  return msg;
}


/*******************************************************************************
 * Fast joint lookup based on indices map
 ******************************************************************************/
JointNameIndexPair::JointNameIndexPair() : jointId(-1)
{
}

JointNameIndexPair::JointNameIndexPair(const std::string& name, int id) : jointName(name), jointId(id)
{
}

RcsJoint* JointNameIndexPair::getJoint(const RcsGraph* graph)
{
  RcsJoint* jnt = NULL;

  if ((jointId == -1) || (!STREQ(graph->joints[jointId].name, jointName.c_str())))
  {
    jnt = RcsGraph_getJointByName(graph, jointName.c_str());
    if (!jnt)
    {
      return NULL;
    }
    else
    {
      jointId = jnt->jointIndex;
    }

  }
  else
  {
    jnt = &graph->joints[jointId];
  }

  return jnt;
}



} // namespace Rcs
