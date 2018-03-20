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

  \brief See header.

*******************************************************************************/

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

  \brief See header.

*******************************************************************************/

xmlNodePtr parseXMLMemory(const char* buffer, unsigned int size,
                          xmlDocPtr* doc)
{
  xmlNodePtr node;   // xml root node

  // The document being in memory, it has no base per RFC 2396,
  // and the "noname.xml" argument will serve as its base.
  *doc = xmlReadMemory(buffer, size, "noname.xml", NULL, 0);

  if (*doc == NULL)
  {
    RLOG(1, "Failed to parse document: XML memory is\n%s", buffer);
    return NULL;
  }

  // int subs =
  xmlXIncludeProcess(*doc) ;
  NLOG(0, "%d xml inclusions done for file %s", subs, filename);

  if (!(node = xmlDocGetRootElement(*doc)))
  {
    RLOG(1, "XML memory has no root element - ignoring");
    return NULL;
  }

  // if(xmlStrcmp(node->name, (const xmlChar *) tag))
  // {
  //   RLOG(1, "Wrong file type (\"%s\"), root node is \"%s\" and not \"%s\"",
  //        filename, node->name, tag);
  //   return NULL;
  // }

  return node;
}


/*******************************************************************************

  \brief See header.

*******************************************************************************/

xmlNodePtr parseXMLFile(const char* filename, const char* tag, xmlDocPtr* doc)
{
  xmlNodePtr node;

  xmlParserOption options = XML_PARSE_XINCLUDE ;

  RCHECK(tag);
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

  if (xmlStrcmp(node->name, (const xmlChar*) tag))
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

  \brief See header.

*******************************************************************************/

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

  \brief See header.

*******************************************************************************/

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

  \brief See header.

*******************************************************************************/

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

  \brief See header.

*******************************************************************************/

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

  \brief See header.

*******************************************************************************/

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

  \brief See header.

*******************************************************************************/

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

  \brief See header.

*******************************************************************************/

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

  \brief See header.

*******************************************************************************/

void setXMLNodePropertyString(xmlNodePtr node, const char* tag, const char* str)
{
  xmlNewProp(node, BAD_CAST tag, BAD_CAST str);
}



/*******************************************************************************

  \brief See header.

*******************************************************************************/

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

  \brief See header.

*******************************************************************************/

unsigned int getXMLNodePropertyStringN(xmlNodePtr node, const char* tag,
                                       char* str, unsigned int n)
{
  RCHECK(tag);

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

  \brief See header.

*******************************************************************************/

bool getXMLNodePropertyDouble(xmlNodePtr node, const char* tag, double* x)
{
  return getXMLNodePropertyVecN(node, tag, x, 1);
}

/*******************************************************************************

  \brief See header.

*******************************************************************************/

void setXMLNodePropertyDouble(xmlNodePtr node, const char* tag, const double* x)
{
  setXMLNodePropertyVecN(node, tag, x, 1);
}


/*******************************************************************************

  \brief See header.

*******************************************************************************/

bool getXMLNodePropertyVec2(xmlNodePtr node, const char* tag, double* x)
{
  return getXMLNodePropertyVecN(node, tag, x, 2);
}



/*******************************************************************************

  \brief See header.

*******************************************************************************/

bool getXMLNodePropertyVec3(xmlNodePtr node, const char* tag, double* x)
{
  return getXMLNodePropertyVecN(node, tag, x, 3);
}



/*******************************************************************************

  \brief See header.

*******************************************************************************/

bool getXMLNodePropertyVecN(xmlNodePtr nd, const char* tag, double* x,
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
  double value;
  unsigned int matchedStrings = 0;

  while (pch != NULL)
  {
    if (sscanf(pch, "%lf", &value))
    {
      RCHECK(isfinite(value));
      matchedStrings++;
      if (matchedStrings <= n)
      {
        x[matchedStrings - 1] = value;
      }
    }
    pch = strtok(NULL, " ");
  }

  RCHECK_MSG(matchedStrings == n, "during parsing for tag \"%s\", not all "
             "(or more) values could be found (found %d, should be %d): "
             "\"%s\"", tag, matchedStrings, n, tmp);

  RFREE(tmp);

  return true;
}



/*******************************************************************************

  \brief See header.

*******************************************************************************/

void setXMLNodePropertyVecN(xmlNodePtr node, const char* tag, const double* x,
                            unsigned int n)
{
  // buffer
  unsigned int i, bufferSize = 15 * n;
  char* buff = RNALLOC(bufferSize, char);

  char* ptr = buff;
  for (i = 0; i < n; i++)
  {
    ptr += sprintf(ptr, "%f ", x[i]);
  }
  *(--ptr) = '\0';

  xmlNewProp(node, BAD_CAST tag, BAD_CAST buff);

  RFREE(buff);
}


/*******************************************************************************

  \brief See header.

*******************************************************************************/

bool getXMLNodePropertyBoolN(xmlNodePtr nd, const char* tag, bool* x,
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
  double value;
  unsigned int matchedStrings = 0;

  while (pch != NULL)
  {
    if (sscanf(pch, "%lf", &value))
    {
      RCHECK(isfinite(value));
      matchedStrings++;
      if (matchedStrings <= n)
      {
        x[matchedStrings - 1] = (value != 0.0 ? true : false);
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

  \brief See header.

*******************************************************************************/

void setXMLNodePropertyBoolN(xmlNodePtr node, const char* tag, const bool* x,
                             unsigned int n)
{
  // buffer
  unsigned int i, bufferSize = 15 * n;
  char* buff = RNALLOC(bufferSize, char);

  char* ptr = buff;
  for (i = 0; i < n; i++)
  {
    ptr += sprintf(ptr, "%d ", x[i]);
  }
  *(--ptr) = '\0';

  xmlNewProp(node, BAD_CAST tag, BAD_CAST buff);

  RFREE(buff);
}



/*******************************************************************************

  \brief See header.

*******************************************************************************/

bool getXMLNodePropertyInt(xmlNodePtr node, const char* tag, int* x)
{
  bool exists = false;
  xmlChar* txt = xmlGetProp(node, (const xmlChar*) tag);

  if (txt != NULL)
  {
    if (x != NULL)
    {
      *x = atoi((const char*) txt);
      RCHECK(isfinite(*x));
    }

    exists = true;
    xmlFree(txt);
  }

  return exists;
}


/*******************************************************************************

  \brief See header.

*******************************************************************************/

void setXMLNodePropertyInt(xmlNodePtr node, const char* tag,
                           const int* x)
{
  char buff[32];
  sprintf(buff, "%d", *x);
  xmlNewProp(node, BAD_CAST tag, BAD_CAST buff);
}


/*******************************************************************************

  \brief See header.

*******************************************************************************/

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
      RCHECK(isfinite(*x));
      RCHECK(*x<UINT_MAX);
    }

    exists = true;
  }

  xmlFree(txt);

  return exists;
}


/*******************************************************************************

  \brief See header.

*******************************************************************************/

void setXMLNodePropertyUnsignedInt(xmlNodePtr node, const char* tag,
                                   const unsigned int* x)
{
  char buff[32];
  sprintf(buff, "%u", *x);
  xmlNewProp(node, BAD_CAST tag, BAD_CAST buff);
}


/*******************************************************************************

  \brief See header.

*******************************************************************************/

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

  return true;
}


/*******************************************************************************

  \brief See header.

*******************************************************************************/

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

  return true;
}



/*******************************************************************************

  \brief See header.

*******************************************************************************/

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


void setXMLNodePropertyBoolString(xmlNodePtr node, const char* tag, const bool* x)
{
  xmlNewProp(node, BAD_CAST tag, *x ? BAD_CAST "true" : BAD_CAST "false");
}



/*******************************************************************************

  \brief See header.

*******************************************************************************/

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

  \brief See header.

*******************************************************************************/

bool getXMLNodePropertyQuat(xmlNodePtr node, const char* tag, double A_BI[3][3])
{
  double x[4], qw, qx, qy, qz;

  if (!getXMLNodePropertyVecN(node, tag, x, 4))
  {
    return false;
  }

  qw = x[0];
  qx = x[1];
  qy = x[2];
  qz = x[3];

  // Set matrix with row-major form
  A_BI[0][0] = 1.0f - 2.0f*qy*qy - 2.0f*qz*qz;
  A_BI[1][0] = 2.0f*qx*qy - 2.0f*qz*qw;
  A_BI[2][0] = 2.0f*qx*qz + 2.0f*qy*qw;

  A_BI[0][1] = 2.0f*qx*qy + 2.0f*qz*qw;
  A_BI[1][1] = 1.0f - 2.0f*qx*qx - 2.0f*qz*qz;
  A_BI[2][1] = 2.0f*qy*qz - 2.0f*qx*qw;

  A_BI[0][2] = 2.0f*qx*qz - 2.0f*qy*qw;
  A_BI[1][2] = 2.0f*qy*qz + 2.0f*qx*qw;
  A_BI[2][2] = 1.0f - 2.0f*qx*qx - 2.0f*qy*qy;

  return true;
}

unsigned int getXMLNodeString(xmlNodePtr node, char* str)
{
  unsigned int len = 0;
  xmlChar* txt = xmlNodeGetContent(node);

  if (txt == NULL)
  {
    return 0;
  }

  if (str != NULL)
  {
    strcpy(str, (const char*) txt);
  }

  len = strlen((const char*) txt) + 1;  // +1 to account for trailing '\0'

  xmlFree(txt);

  return len;
}

bool getXMLNodeVec3(xmlNodePtr node, double* x)
{
  return getXMLNodeVecN(node, x, 3);
}

/*******************************************************************************

  \brief See header. \todo: Looks weird. Make better documentation.

*******************************************************************************/

bool getXMLNodeVecN(xmlNodePtr nd, double* x, unsigned int n)
{
  if ((x!=NULL) && (n>0))
  {
    char tmp[512];
    int len = getXMLNodeString(nd, tmp);

    RCHECK_MSG(len<512, "Max. string length is limited to 512 bytes, yours "
               "is %d long", len);

    if (len > 0)
    {
      char* pch = strtok(tmp, " ");
      double value;
      unsigned int matchedStrings = 0;

      while (pch != NULL)
      {
        if (sscanf(pch, "%lf", &value))
        {
          RCHECK(isfinite(value));
          matchedStrings++;
          if (matchedStrings <= n)
          {
            x[matchedStrings - 1] = value;
          }
        }
        pch = strtok(NULL, " ");
      }
      RCHECK_MSG(matchedStrings == n, "during parsing, not all "
                 "(or more) values could be found (found %d, should be %d)",
                 matchedStrings, n);

    }   // if(len>0)
    else
    {
      return false;
    }

  }   // if ((x) && (n > 0))

  return true;
}

bool getXMLNodeQuat(xmlNodePtr node, double A_BI[3][3])
{
  double x[4], qw, qx, qy, qz;

  if (!getXMLNodeVecN(node, x, 4))
  {
    return false;
  }

  qw = x[0];
  qx = x[1];
  qy = x[2];
  qz = x[3];

  // Set matrix with transposed style to fit HRI rotation axis
  A_BI[0][0] = 1.0f - 2.0f*qy*qy - 2.0f*qz*qz;
  A_BI[1][0] = 2.0f*qx*qy - 2.0f*qz*qw;
  A_BI[2][0] = 2.0f*qx*qz + 2.0f*qy*qw;

  A_BI[0][1] = 2.0f*qx*qy + 2.0f*qz*qw;
  A_BI[1][1] = 1.0f - 2.0f*qx*qx - 2.0f*qz*qz;
  A_BI[2][1] = 2.0f*qy*qz - 2.0f*qx*qw;

  A_BI[0][2] = 2.0f*qx*qz - 2.0f*qy*qw;
  A_BI[1][2] = 2.0f*qy*qz + 2.0f*qx*qw;
  A_BI[2][2] = 1.0f - 2.0f*qx*qx - 2.0f*qy*qy;

  return true;
}

bool getXMLNodeIntN(xmlNodePtr node, int* x, unsigned int n)
{
  if ((x) && (n > 0))
  {
    char tmp[512];
    int len = getXMLNodeString(node, tmp);

    if (len > 0)
    {
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
      RCHECK_MSG(matchedStrings == n, "during parsing, not all "
                 "(or more) values could be found (found %d, should be %d)",
                 matchedStrings, n);
    }
    else
    {
      return false;
    }
  }
  return true;
}

void MatNd_toXML(const MatNd* A, xmlNodePtr parentNode, const char* nodeName)
{
  RCHECK(A);

  // buffer
  unsigned int bufferSize = 15 * A->m * A->n;
  char* buff = RNALLOC(bufferSize, char);

  MatNd_toString(A, buff);
  xmlNodePtr node = xmlNewTextChild(parentNode, NULL, BAD_CAST nodeName, BAD_CAST buff);

  setXMLNodePropertyUnsignedInt(node, "M", &A->m);
  setXMLNodePropertyUnsignedInt(node, "N", &A->n);

  RFREE(buff);
}

void MatNd_fromXML(MatNd* A, xmlNodePtr node, bool realloc)
{
  unsigned int m = 0;
  unsigned int n = 0;

  // read dimensions
  getXMLNodePropertyUnsignedInt(node, "M", &m);
  getXMLNodePropertyUnsignedInt(node, "N", &n);

  if (realloc)
  {
    MatNd_realloc(A, m, n);
  }
  else
  {
    RCHECK_MSG(m* n <= A->size,
               "Matrix to be parsed is larger than the memory provided and"
               " realloc has been disabled");
  }

  // read data
  char* buff = (char*)xmlNodeGetContent(node);
  RCHECK(buff);

  MatNd_fromString(A, buff);
  xmlFree(buff);
}

MatNd* MatNd_createFromXML(xmlNodePtr node)
{
  MatNd* mat = MatNd_create(0,0);
  MatNd_fromXML(mat, node, true);
  return mat;
}

void VecNd_toXML(const double* vec, xmlNodePtr parentNode, const char* nodeName, unsigned int dim)
{
  RCHECK(vec);

  xmlNodePtr node = xmlNewChild(parentNode, NULL, BAD_CAST nodeName, NULL);

  setXMLNodePropertyUnsignedInt(node, "dim", &dim);
  setXMLNodePropertyVecN(node, "ele", vec, dim);
}

void VecNd_fromXML(double* vec, xmlNodePtr node, unsigned int dim)
{
  unsigned int d = 0;

  // read dimension
  getXMLNodePropertyUnsignedInt(node, "dim", &d);
  RCHECK(dim == d);

  // read data
  getXMLNodePropertyVecN(node, "ele", vec, dim);
}

void double_fromXML(double* value, xmlNodePtr node)
{
  char* buff = (char*)xmlNodeGetContent(node);
  RCHECK(buff);

  *value = atof(buff);
  xmlFree(buff);
}

void double_toXML(double value, xmlNodePtr parentNode, const char* nodeName)
{
  char buffer[256];
  sprintf(buffer, "%g", value);
  xmlNewTextChild(parentNode, NULL, BAD_CAST nodeName, BAD_CAST buffer);
}



/******************************************************************************

  \brief See header.

******************************************************************************/

bool getXMLNodePropertyHTr(xmlNodePtr node, const char* tag, HTr* A)
{
  double x[6];

  if (!getXMLNodePropertyVecN(node, tag, x, 6))
  {
    return false;
  }

  // Convert angular magnitudes into radians (xml expects degrees)
  Vec3d_copy(A->org, &x[0]);
  Vec3d_constMulSelf(&x[3], M_PI/180.0);
  Mat3d_fromEulerAngles(A->rot, &x[3]);

  return true;
}
