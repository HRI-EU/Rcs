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

#include "Rcs_parser.h"
#include "Rcs_macros.h"
#include "Rcs_utils.h"
#include "Rcs_math.h"
#include "Rcs_quaternion.h"
#include "Rcs_resourcePath.h"

#include <libxml/xmlmemory.h>
#include <libxml/parser.h>
#include <libxml/xinclude.h>
#include <libxml/xlink.h>
#include <libxml/xmlstring.h>

#include <errno.h>
#include <float.h>
#include <limits.h>



/*******************************************************************************
*
******************************************************************************/
bool getXMLFileTag(const char* filename, char* tag)
{
  bool fileExists = false;
  char fullPath[512];
  xmlNodePtr node;
  xmlDocPtr doc;

#if defined(_MSC_VER)
  xmlParserOption options = static_cast<xmlParserOption>(XML_PARSE_XINCLUDE | XML_PARSE_NOERROR | XML_PARSE_NOWARNING);
#else
  xmlParserOption options = XML_PARSE_XINCLUDE | XML_PARSE_NOERROR | XML_PARSE_NOWARNING;
#endif


  if (filename==NULL)
  {
    RLOG(5, "filename is NULL - returning false");
    return false;
  }

  if (tag==NULL)
  {
    RLOG(5, "tag is NULL when getting xml tag for file \"%s\"- "
         "returning false", filename);
    return false;
  }

  // Copy the XML file name
  fileExists = Rcs_getAbsoluteFileName(filename, fullPath);

  if (fileExists==false)
  {
    RLOG(5, "Configuration file \"%s\" not found in "
         "ressource path - returning false", filename);
    return false;
  }



  if (!(doc = xmlReadFile(fullPath, NULL, options)))
  {
    RLOG(5, "Document \"%s\" (expected tag is \"%s\") not parsed "
         "successfully.", fullPath, tag);
    return false;
  }

  if (!(node = xmlDocGetRootElement(doc)))
  {
    RLOG(5, "Document \"%s\" (expected tag is \"%s\") has no root element.",
         fullPath, tag);
    return false;
  }

  // Copy result
  strcpy(tag, (char*) node->name);

  // Free the xml memory
  xmlFreeDoc(doc);

  return true;
}

/*******************************************************************************
*
******************************************************************************/
xmlNodePtr parseXMLMemory(const char* buffer, size_t size,
                          xmlDocPtr* doc)
{
  *doc = xmlReadMemory(buffer, size, "noname.xml", NULL, 0);

  if (*doc == NULL)
  {
    RLOG(1, "Failed to parse document: XML memory is\n%s", buffer);
    return NULL;
  }

  xmlXIncludeProcess(*doc) ;

  return xmlDocGetRootElement(*doc);   // xml root node
}

/*******************************************************************************
*
******************************************************************************/
xmlNodePtr parseXMLFile(const char* filename, const char* tag, xmlDocPtr* doc)
{
  xmlNodePtr node;

  xmlParserOption options = XML_PARSE_XINCLUDE ;

  RCHECK_MSG(File_exists(filename), "XML-file \"%s\" not found (Tag \"%s\")!",
             filename, tag);

  if (!(*doc = xmlReadFile(filename, NULL, options)))
  {
    /*   if(!(*doc = xmlParseFile(filename)))  */
    RLOG(1, "Document \"%s\" not parsed successfully.", filename);
    return NULL;
  }

  // int subs =
  xmlXIncludeProcess(*doc) ;
  NLOG(0, "%d xml inclusions done for file %s", subs, filename);

  if (!(node = xmlDocGetRootElement(*doc)))
  {
    RLOG(1, "Document  \"%s\" has no root element.", filename);
    return NULL;
  }

  if (tag && xmlStrcmp(node->name, (const xmlChar*) tag))
  {
    NLOG(1, "Wrong file type (\"%s\"), root node is \"%s\" and not \"%s\"",
         filename, node->name, tag);
    return NULL;
  }


  /*   FILE *out = fopen("xmlDump.txt", "w+"); */
  /*   RCHECK(out); */
  /*   RCHECK(doc); */
  /*   xmlDocFormatDump(out,*doc,1); */

  return node;
}

/*******************************************************************************
*
******************************************************************************/
void dumpXMLFileToMemory(char** buffer, unsigned int* size, const char* xmlFile)
{
  xmlDocPtr doc;

  // Determine absolute file name of config file and copy the XML file name
  char filename[256] = "";
  bool fileExists = Rcs_getAbsoluteFileName(xmlFile, filename);

  if (fileExists==false)
  {
    REXEC(1)
    {
      Rcs_printResourcePath();
    }
    RFATAL("RcsGraph configuration file \"%s\" not found in "
           "ressource path - exiting", xmlFile ? xmlFile : "NULL");
  }

  // Read XML file
  parseXMLFile(filename, "Graph", &doc);

  xmlDocDumpMemory(doc, (xmlChar**)buffer, (int*)size);

  // Free the xml memory
  xmlFreeDoc(doc);
}

/*******************************************************************************
*
******************************************************************************/
bool isXMLNodeName(xmlNodePtr node, const char* name)
{
  if (node == NULL)
  {
    return false;
  }

  if (name == NULL)
  {
    return false;
  }

  if (xmlStrcmp(node->name, (const xmlChar*) name))
  {
    return false;
  }

  return true;
}

/*******************************************************************************
*
******************************************************************************/
bool isXMLNodeNameNoCase(xmlNodePtr node, const char* name)
{
  if (node == NULL)
  {
    return false;
  }

  if (name == NULL)
  {
    return false;
  }

  if (xmlStrcasecmp(node->name, (const xmlChar*) name))
  {
    return false;
  }

  return true;
}

/*******************************************************************************
*
******************************************************************************/
unsigned int getNumXMLNodes(xmlNodePtr node, const char* tag)
{
  unsigned int count = 0;
  xmlNodePtr tmpNode = node;

  while (tmpNode)
  {
    if (isXMLNodeName(tmpNode, tag))
    {
      count++;
    }
    tmpNode = tmpNode->next;
  }

  return count;
}

/*******************************************************************************
*
******************************************************************************/
xmlNodePtr getXMLChildByName(xmlNodePtr node, const char* name)
{
  xmlNodePtr child = node->children;

  while (child)
  {
    if (STRCASEEQ((const char*) BAD_CAST child->name, name))
    {
      return child;
    }

    child = child->next;
  }

  return NULL;
}

/*******************************************************************************
*
******************************************************************************/
bool getXMLNodeName(xmlNodePtr node, char* name)
{
  if (node == NULL)
  {
    return false;
  }

  if (name != NULL)
  {
    strcpy(name, (char*) node->name);
  }

  return true;
}

/*******************************************************************************
*
******************************************************************************/
bool getXMLNodeProperty(xmlNodePtr node, const char* tag)
{
  bool exists = false;
  xmlChar* txt = xmlGetProp(node, (const xmlChar*) tag);

  if (txt != NULL)
  {
    exists = true;
    xmlFree(txt);
  }

  return exists;
}

/*******************************************************************************
*
******************************************************************************/
unsigned int getXMLNodeBytes(xmlNodePtr node, const char* tag)
{
  if (tag == NULL)
  {
    return 0;
  }

  xmlChar* txt = xmlGetProp(node, (const xmlChar*) tag);

  if (txt == NULL)
  {
    return 0;
  }

  // +1 to account for trailing '\0'
  unsigned int len = strlen((const char*) txt) + 1;

  xmlFree(txt);

  return len;
}

/*******************************************************************************
*
******************************************************************************/
unsigned int getXMLNodePropertyStringN(xmlNodePtr node, const char* tag,
                                       char* str, unsigned int n)
{
  xmlChar* txt = xmlGetProp(node, (const xmlChar*) tag);

  if (txt == NULL)
  {
    return 0;
  }

  unsigned int len = strlen((const char*) txt) + 1;  // +1 for trailing '\0'

  if (str != NULL)
  {
    if (len>n)
    {
      RLOG(1, "Parsing tag \"%s\" requires %d bytes, you provide %d",
           tag, len, n);
    }
    strncpy(str, (const char*) txt, n);
    str[n-1] = '\0';
  }

  xmlFree(txt);

  return len;
}

/*******************************************************************************
*
******************************************************************************/
bool getXMLNodePropertyDouble(xmlNodePtr node, const char* tag, double* x)
{
  return getXMLNodePropertyVecN(node, tag, x, 1);
}

/*******************************************************************************
*
******************************************************************************/
bool getXMLNodePropertyVec2(xmlNodePtr node, const char* tag, double* x)
{
  return getXMLNodePropertyVecN(node, tag, x, 2);
}

/*******************************************************************************
*
******************************************************************************/
bool getXMLNodePropertyVec3(xmlNodePtr node, const char* tag, double* x)
{
  return getXMLNodePropertyVecN(node, tag, x, 3);
}

/*******************************************************************************
*
******************************************************************************/
bool getXMLNodePropertyVecN(xmlNodePtr nd, const char* tag, double* x,
                            unsigned int n)
{
  unsigned int nBytes = getXMLNodeBytes(nd, tag);

  if (nBytes == 0)
  {
    return false;
  }

  char* tmp = RNALLOC(nBytes, char);
  getXMLNodePropertyStringN(nd, tag, tmp, nBytes);

  bool success = String_toDoubleArray_l(tmp, x, n);

  RFREE(tmp);
  return success;
}

/*******************************************************************************
*
******************************************************************************/
bool getXMLNodePropertyBoolN(xmlNodePtr nd, const char* tag, bool* x,
                             unsigned int n)
{
  double* x_dbl = RNALLOC(n, double);
  bool success = getXMLNodePropertyVecN(nd, tag, x_dbl, n);

  if (success==true)
  {
    for (unsigned int i = 0; i<n; ++i)
    {
      x[i] = (x_dbl[i] != 0.0 ? true : false);
    }
  }

  RFREE(x_dbl);

  return true;
}

/*******************************************************************************
*
******************************************************************************/
bool getXMLNodePropertyInt(xmlNodePtr node, const char* tag, int* x)
{
  bool exists = false;
  xmlChar* txt = xmlGetProp(node, (const xmlChar*) tag);

  if (txt != NULL)
  {
    if (x != NULL)
    {
      *x = atoi((const char*) txt);
      //RCHECK(isfinite(*x));
    }

    exists = true;
    xmlFree(txt);
  }

  return exists;
}

/*******************************************************************************
*
******************************************************************************/
bool getXMLNodePropertyUnsignedInt(xmlNodePtr node, const char* tag,
                                   unsigned int* x)
{
  bool exists = false;
  xmlChar* txt = xmlGetProp(node, (const xmlChar*) tag);

  if (txt)
  {
    errno = 0;
    if (x)
    {
      *x = strtoul((const char*) txt, 0, 0);
      RCHECK(errno == 0);
      //RCHECK(isfinite(*x));
      RCHECK(*x<UINT_MAX);
    }

    exists = true;
  }

  xmlFree(txt);

  return exists;
}

/*******************************************************************************
*
******************************************************************************/
bool getXMLNodePropertyIntN(xmlNodePtr nd, const char* tag, int* x,
                            unsigned int n)
{
  if (x==NULL)
  {
    return false;
  }

  if (n==0)
  {
    return false;
  }

  unsigned int nBytes = getXMLNodeBytes(nd, tag);

  if (nBytes == 0)
  {
    return 0;
  }

  char* tmp = RNALLOC(nBytes, char);

  RCHECK_MSG(tmp, "Couldn't allocate memory for %d bytes", nBytes);

  int len = getXMLNodePropertyStringN(nd, tag, tmp, nBytes);

  if (len == 0)
  {
    RFREE(tmp);
    return false;
  }


  char* pch = strtok(tmp, " ");
  int value;
  unsigned int matchedStrings = 0;
  while (pch != NULL)
  {
    if (sscanf(pch, "%d", &value))
    {
      matchedStrings++;
      if (matchedStrings <= n)
      {
        x[matchedStrings - 1] = value;
      }
    }
    pch = strtok(NULL, " ");
  }
  RCHECK_MSG(matchedStrings == n, "during parsing for tag \"%s\", not all "
             "(or more) values could be found (found %d, should be %d)",
             tag, matchedStrings, n);

  RFREE(tmp);

  return true;
}

/*******************************************************************************
*
******************************************************************************/
bool getXMLNodePropertyUnsignedIntN(xmlNodePtr nd, const char* tag,
                                    unsigned int* x, unsigned int n)
{
  if (x==NULL)
  {
    return false;
  }

  if (n==0)
  {
    return false;
  }

  unsigned int nBytes = getXMLNodeBytes(nd, tag);

  if (nBytes == 0)
  {
    return 0;
  }


  char* tmp = RNALLOC(nBytes, char);

  RCHECK_MSG(tmp, "Couldn't allocate memory for %d bytes", nBytes);

  int len = getXMLNodePropertyStringN(nd, tag, tmp, nBytes);

  if (len == 0)
  {
    RFREE(tmp);
    return false;
  }

  char* pch = strtok(tmp, " ");
  unsigned int value;
  unsigned int matchedStrings = 0;

  while (pch != NULL)
  {
    if (sscanf(pch, "%u", &value))
    {
      matchedStrings++;
      if (matchedStrings <= n)
      {
        x[matchedStrings - 1] = value;
      }
    }
    pch = strtok(NULL, " ");
  }
  RCHECK_MSG(matchedStrings == n, "during parsing for tag \"%s\", not all "
             "(or more) values could be found (found %d, should be %d)",
             tag, matchedStrings, n);

  RFREE(tmp);

  return true;
}

/*******************************************************************************
*
******************************************************************************/
bool getXMLNodePropertyBoolString(xmlNodePtr node, const char* tag, bool* x)
{
  bool exists = false;
  bool res    = false;
  xmlChar* txt = xmlGetProp(node, (const xmlChar*) tag);

  if (txt != NULL)
  {
    NLOG(0, "Tag \"%s\" contains \"%s\"", tag, (const char*) txt);

    res = String_toBool((const char*) txt);

    if (x != NULL)
    {
      *x = res;
    }
    exists = true;
    xmlFree(txt);
  }
  else   // if (txt != NULL)
  {
    NLOG(0, "No contents found for tag \"%s\"", tag);
  }

  return exists;
}

/*******************************************************************************
*
******************************************************************************/
unsigned int getXMLNodeNumStrings(xmlNodePtr nd, const char* tag)
{
  unsigned int nBytes = getXMLNodeBytes(nd, tag);

  if (nBytes == 0)
  {
    return 0;
  }

  char* str = RNALLOC(nBytes, char);
  unsigned int len = getXMLNodePropertyStringN(nd, tag, str, nBytes);
  unsigned int nStrings = 0;

  if (len > 0)
  {
    nStrings = String_countSubStrings(str, " ");
  }

  RFREE(str);

  return nStrings;
}

/*******************************************************************************
 * Matrix with row-major form
 ******************************************************************************/
bool getXMLNodePropertyQuat(xmlNodePtr node, const char* tag, double A_BI[3][3])
{
  double x[4];

  if (!getXMLNodePropertyVecN(node, tag, x, 4))
  {
    return false;
  }

  Quat_toRotationMatrix(A_BI, x);

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool getXMLNodePropertyHTr(xmlNodePtr node, const char* tag, HTr* A)
{
  double x[12];
  unsigned int nItems = getXMLNodeNumStrings(node, tag);
  bool success = getXMLNodePropertyVecN(node, tag, x, nItems);

  if (!success)
  {
    return false;
  }

  switch (nItems)
  {
    case 6:
      // x-y-z-a-b-c:
      // Convert angular magnitudes into radians (xml expects degrees)
      Vec3d_copy(A->org, &x[0]);
      Vec3d_constMulSelf(&x[3], M_PI/180.0);
      Mat3d_fromEulerAngles(A->rot, &x[3]);
      break;

    case 12:
      // x-y-z, followed by row-wise rotation matrix:
      // Convert angular magnitudes into radians (xml expects degrees)
      Vec3d_copy(A->org, &x[0]);
      Mat3d_fromArray(A->rot, &x[3]);
      break;

    default:
      success = false;
      break;
  }

  return success;
}
