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

#ifndef RCS_PARSER_H
#define RCS_PARSER_H


#include "Rcs_MatNd.h"
#include "Rcs_HTr.h"

#include <libxml/tree.h>


#ifdef __cplusplus
extern "C" {
#endif




/*!
 * \defgroup RcsParserFunctions XML-parser functions
 *
 *           XML parsing utility functions. They are based on the libxml2
 *           library.
 */


/*! \ingroup RcsParserFunctions
 *  \brief Copies the file tag into "tag". If something goes wrong, the
 *         function returns false and leaves argument tag unchanged. In
 *         this case, a warning is issued on debug levels 5 or hogher.
 */
bool getXMLFileTag(const char* filename, char* tag);

/*! \ingroup RcsParserFunctions
 *  \brief Same as parseXMLFile(), but result is read from the memory pointed
 *         to by buffer.
 */
xmlNodePtr parseXMLMemory(const char* buffer, size_t size,
                          xmlDocPtr* doc);

/*! \ingroup RcsParserFunctions
 *  \brief Parses the xml file given in filename with the given tag, and
 *         stores it in the doc pointer. The caller has to take care to
 *         destroy the memory pointed to by doc. If the return value is NULL,
 *         parsing failed and doc is unchanged.
 */
xmlNodePtr parseXMLFile(const char* filename, const char* tag,
                        xmlDocPtr* doc);

/*! \ingroup RcsParserFunctions
 *  \brief Creates a buffer containing the xml data of the given file. The
 *         xmlFile is searched in the Rcs resource path. If it doesn't exist,
 *         the function will exit with a fatal error. If the file is found,
 *         its contents are copied into a buffer. The buffer pointer is set
 *         to this buffer, and the size variable is set to the number of
 *         bytes of the buffer. The caller is responsible to free the memory.
 */
void dumpXMLFileToMemory(char** buffer, unsigned int* size,
                         const char* xmlFile);

/*! \ingroup RcsParserFunctions
 *  \brief Returns "true" if "name" is the name of the XML node "node",
 *         "false" otherwise. If the node points to NULL, false is
 *         returned.
 */
bool isXMLNodeName(xmlNodePtr node, const char* name);

/*! \ingroup RcsParserFunctions
 *  \brief Returns "true" if "name" is the name of the XML node "node",
 *         "false" otherwise. If the node points to NULL, false is
 *         returned. This function is case insensitive
 */
bool isXMLNodeNameNoCase(xmlNodePtr node, const char* name);

/*! \ingroup RcsParserFunctions
 * \brief Returns "true" if a XML node with name "tag" exists in the
 *        XML node "node".
 */
unsigned int getNumXMLNodes(xmlNodePtr node, const char* tag);

/*! \ingroup RcsParserFunctions
 * \brief Iterates over all children of node until it finds a node with the
 *        given name. The name is case insensitive.
 *
 * \return The child node with the given name or NULL if no child with the
 *         name exists
 */
xmlNodePtr getXMLChildByName(xmlNodePtr node, const char* name);

/*! \ingroup RcsParserFunctions
 * \brief Copies the name of XML node "node" to argument "name" and
 *        returns "true". If the node does not exist, "name" will not be
 *        changed and false will be returned.
 */
bool getXMLNodeName(xmlNodePtr node, char* name);

/*! \ingroup RcsParserFunctions
 * \brief Returns true if the XML node contains a property with a
 *        name given in tag.
 */
bool getXMLNodeProperty(xmlNodePtr node, const char* tag);

/*! \ingroup RcsParserFunctions
 * \brief Returns the number of bytes in the property string of the given tag.
 *        It includes the trailing '\0'. If the tag doesn't exist, the function
 *        returns 0.
 */
unsigned int getXMLNodeBytes(xmlNodePtr node, const char* tag);

/*! \ingroup RcsParserFunctions
 * \brief Creates a new property with name tag and content str
 */
void setXMLNodePropertyString(xmlNodePtr node, const char* tag,
                              const char* str);

/*! \ingroup RcsParserFunctions
 * \brief The contents of argument tag are copied into str (interpreted as
 *        char[]), but maximum n bytes including the trailing '\\0'. If the
 *        tag is a NULL pointer, argument str will be unchanged and 0 is
 *        returned. The number of bytes (including the trailing '\\0') is
 *        returned.
 */
unsigned int getXMLNodePropertyStringN(xmlNodePtr node, const char* tag,
                                       char* str, unsigned int n);

/*! \ingroup RcsParserFunctions
 * \brief If the tag is a NULL pointer, argument x will be unchanged and
 *        "false" is returned. Otherwise, the contents of argument tag
 *        are copied into x (interpreted as double). The function will exit
 *        fatally for the same reasons as \ref getXMLNodePropertyVecN.
 */
bool getXMLNodePropertyDouble(xmlNodePtr node, const char* tag, double* x);

/*!\ingroup RcsParserFunctions
 * \brief Creates a new property with name tag and content x
 */
void setXMLNodePropertyDouble(xmlNodePtr node, const char* tag,
                              const double* x);

/*! \ingroup RcsParserFunctions
 * \brief If the tag is a NULL pointer, argument x will be unchanged and
 *        "false" is returned. Otherwise, the contents of argument tag
 *        are copied into x (interpreted as array of 2 double values). The
 *        function will exit fatally for the same reasons as
 *        \ref getXMLNodePropertyVecN.
 */
bool getXMLNodePropertyVec2(xmlNodePtr node, const char* tag, double* x);

/*! \ingroup RcsParserFunctions
 * \brief If the tag is a NULL pointer, argument x will be unchanged and
 *        "false" is returned. Otherwise, the contents of argument tag
 *        are copied into x (interpreted as array of 3 double values). The
 *        function will exit fatally for the same reasons as
 *        \ref getXMLNodePropertyVecN.
 */
bool getXMLNodePropertyVec3(xmlNodePtr node, const char* tag, double* x);

/*! \ingroup RcsParserFunctions
 * \brief If the tag is a NULL pointer, argument x will be unchanged and
 *        "false" is returned. Otherwise, n scanned elements of argument
 *        tag are copied into x (interpreted as double values). The function
 *        will exit with a fatal error if
 *        - the number of values in the xml node does not match n
 *        - one or more of the values are not finite
 */
bool getXMLNodePropertyVecN(xmlNodePtr node, const char* tag, double* x,
                            unsigned int n);

/*! \ingroup RcsParserFunctions
 * \brief creates a new property with values of x and name tag
 */
void setXMLNodePropertyVecN(xmlNodePtr node, const char* tag,
                            const double* x, unsigned int n);

/*! \ingroup RcsParserFunctions
 * \brief If the tag is a NULL pointer, argument x will be unchanged and
 *        "false" is returned. Otherwise, n scanned elements of argument
 *        tag are copied into x (interpreted as boolean values). The function
 *        will exit with a fatal error if
 *        - the number of values in the xml node does not match n
 *        - one or more of the values are not finite
 */
bool getXMLNodePropertyBoolN(xmlNodePtr node, const char* tag, bool* x,
                             unsigned int n);

/*! \ingroup RcsParserFunctions
 * \brief creates a new property with values of x and name tag
 */
void setXMLNodePropertyBoolN(xmlNodePtr node, const char* tag,
                             const bool* x, unsigned int n);

/*! \ingroup RcsParserFunctions
 *  \brief If the tag is a NULL pointer, argument x will be unchanged and
 *         "false" is returned. Otherwise, the contents of argument tag
 *         are copied into x (interpreted as integer). If the value in the
 *         xml node is not finite, the function will exit with a fatal error.
 */
bool getXMLNodePropertyInt(xmlNodePtr node, const char* tag, int* x);


/*! \ingroup RcsParserFunctions
 *  \brief Creates a new property with value x and name tag
 */
void setXMLNodePropertyInt(xmlNodePtr node, const char* tag,
                           const int* x);

/*! \ingroup RcsParserFunctions
 *  \brief If the tag is a NULL pointer, argument x will be unchanged and
 *         "false" is returned. Otherwise, the contents of argument tag
 *         are copied into x (interpreted as unsigned integer).
 */
bool getXMLNodePropertyUnsignedInt(xmlNodePtr node, const char* tag,
                                   unsigned int* x);

/*! \ingroup RcsParserFunctions
 *  \brief Creates a new property with value x and name tag
 */
void setXMLNodePropertyUnsignedInt(xmlNodePtr node, const char* tag,
                                   const unsigned int* x);

/*! \ingroup RcsParserFunctions
 *  \brief If the tag is a NULL pointer, argument x will be unchanged and
 *         "false" is returned. Otherwise, n scanned elements of argument
 *         tag are copied into x (interpreted as integer).
 */
bool getXMLNodePropertyIntN(xmlNodePtr node, const char* tag, int* x,
                            unsigned int n);

/*! \ingroup RcsParserFunctions
 *  \brief If the tag is a NULL pointer, argument x will be unchanged and
 *         "false" is returned. Otherwise, n scanned elements of argument
 *         tag are copied into x (interpreted as integer).
 */
bool getXMLNodePropertyUnsignedIntN(xmlNodePtr node, const char* tag,
                                    unsigned int* x, unsigned int n);

/*! \ingroup RcsParserFunctions
 * \brief If the tag contains a string "false" or "FALSE", argument x
 *        will be set to false (=0). If the tag is "true" or "TRUE",
 *        argument x will be set to true (=1). If the tag contains none
 *        of the above, it will be false, a warning is issued on
 *        debug level 10, and "false" will be returned ("true" will be
 *        returned if the tag is valid).
 */
bool getXMLNodePropertyBoolString(xmlNodePtr node, const char* tag, bool* x);

/*! \ingroup RcsParserFunctions
 *  \brief Creates a new property with value x and name tag
 */
void setXMLNodePropertyBoolString(xmlNodePtr node, const char* tag,
                                  const bool* x);

/*! \ingroup RcsParserFunctions
 * \brief Returns the number of strings found in the tag.
 */
unsigned int getXMLNodeNumStrings(xmlNodePtr nd, const char* tag);

/*! \ingroup RcsParserFunctions
 * \brief If the tag contains 4 values, the function returns true and converts
 *        it from a quaternion (order is w x y z) to a rotation matrix.
 *        Otherwise, A_BI remains unchanged and false is returned.
 */
bool getXMLNodePropertyQuat(xmlNodePtr node, const char* tag,
                            double A_BI[3][3]);

/*! \ingroup RcsParserFunctions
 * \brief Otherwise, the contents of the node are
 *        copied into str (interpreted as char[]), the number of bytes
 *        (including the trailing '\\0') is returned.
 */
unsigned int getXMLNodeString(xmlNodePtr node, char* str);

/*! \ingroup RcsParserFunctions
 * \brief The contents of the node
 *        are copied into x (interpreted as array of 3 double values).
 */
bool getXMLNodeVec3(xmlNodePtr node, double* x);

/*! \ingroup RcsParserFunctions
 * \brief n scanned elements of node are copied into x (interpreted as
 *        double values). The function will exit with a fatal error if
 *        - the number of values in the xml node does not match n
 *        - Any of the values is not finite
 *        - The length of the XML string must not exceed 512 bytes.
 */
bool getXMLNodeVecN(xmlNodePtr node, double* x, unsigned int n);

/*! \ingroup RcsParserFunctions
 * \brief If the node contains 4 values, the function returns true and
 *        converts it from a quaternion (order is w x y z) to a rotation
 *        matrix. Otherwise, A_BI remains unchanged and false is returned.
 */
bool getXMLNodeQuat(xmlNodePtr node, double A_BI[3][3]);

/*! \ingroup RcsParserFunctions
   *  \brief n scanned elements of the node are copied into x (interpreted
   *         as integer).
   */
bool getXMLNodeIntN(xmlNodePtr node, int* x, unsigned int n);

void MatNd_toXML(const MatNd* A, xmlNodePtr parentNode, const char* nodeName);

void MatNd_fromXML(MatNd* A, xmlNodePtr node, bool realloc);

MatNd* MatNd_createFromXML(xmlNodePtr node);

void VecNd_toXML(const double* vec, xmlNodePtr parentNode,
                 const char* nodeName, unsigned int dim);

void VecNd_fromXML(double* vec, xmlNodePtr node, unsigned int dim);

void double_fromXML(double* value, xmlNodePtr node);

void double_toXML(double value, xmlNodePtr parentNode, const char* nodeName);

/*! \ingroup RcsParserFunctions
 *  \brief If the tag is a NULL pointer, argument A will be unchanged and
 *         "false" is returned. Otherwise, the contents of argument tag
 *         must contain 6 values. If not, "false" is returned and A remains
 *         unchanged. The first 3 values are copied to the translation vector
 *         of the HTr, the last 3 values are interpreted as Euler angles
 *         (x-y-z order), converted, and copied into the rotation matrix
 *         part of A.
 */
bool getXMLNodePropertyHTr(xmlNodePtr node, const char* tag, HTr* A);



#ifdef __cplusplus
}
#endif

#endif   // RCS_PARSER_H
