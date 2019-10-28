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

#include "PhysicsConfig.h"

#include <Rcs_resourcePath.h>
#include <Rcs_macros.h>
#include <Rcs_parser.h>
#include <Rcs_utils.h>

namespace Rcs
{

PhysicsMaterial::PhysicsMaterial() : materialNode(NULL), defaultMaterialNode(NULL) {}

PhysicsMaterial::PhysicsMaterial(xmlNodePtr node, xmlNodePtr defaultMaterialNode) : materialNode(node), defaultMaterialNode(defaultMaterialNode) {}

PhysicsMaterial PhysicsMaterial::next() const
{
  // locate next "material" node
  xmlNodePtr node = materialNode->next;
  while (node)
  {
    if (isXMLNodeName(node, "material"))
    {
      // found it
      return {node, defaultMaterialNode};
    }
    node = node->next;
  }
  // we were the last
  return {NULL, NULL};
}

bool PhysicsMaterial::getDouble(const char* attr, double& out) const
{
  if (materialNode == NULL)
  {
    return false;
  }

  // try to load from this
  if (getXMLNodePropertyDouble(materialNode, attr, &out))
  {
    return true;
  }

  // try to fallback to default material if we aren't the default
  if (materialNode == defaultMaterialNode)
  {
    return false;
  }

  // load from default
  return getXMLNodePropertyDouble(defaultMaterialNode, attr, &out);
}

void PhysicsMaterial::setDouble(const char* attr, double value)
{
  if (materialNode == NULL)
  {
    return;
  }
  char cvt[64];
  sprintf(cvt, "%f", value);
  xmlSetProp(materialNode, BAD_CAST attr, BAD_CAST cvt);
}

bool PhysicsMaterial::getBoolean(const char* attr, bool& out) const
{
  if (materialNode == NULL)
  {
    return false;
  }

  // try to load from this
  if (getXMLNodePropertyBoolString(materialNode, attr, &out))
  {
    return true;
  }

  // try to fallback to default material if we aren't the default
  if (materialNode == defaultMaterialNode)
  {
    return false;
  }

  // load from default
  return getXMLNodePropertyBoolString(defaultMaterialNode, attr, &out);
}

void PhysicsMaterial::setBoolean(const char* attr, bool value)
{
  if (materialNode == NULL)
  {
    return;
  }
  if (value)
  {
    xmlSetProp(materialNode, BAD_CAST attr, BAD_CAST "true");
  }
  else
  {
    xmlSetProp(materialNode, BAD_CAST attr, BAD_CAST "false");
  }
}

bool PhysicsMaterial::getString(const char* attr, char* out, unsigned int limit) const
{
  if (materialNode == NULL)
  {
    return false;
  }

  // try to load from this
  if (getXMLNodePropertyStringN(materialNode, attr, out, limit))
  {
    return true;
  }

  // try to fallback to default material if we aren't the default
  if (materialNode == defaultMaterialNode)
  {
    return false;
  }

  // load from default
  return getXMLNodePropertyStringN(defaultMaterialNode, attr, out, limit) ? true : false;
}

void PhysicsMaterial::setString(const char* attr, const char* value)
{
  if (materialNode == NULL)
  {
    return;
  }
  xmlSetProp(materialNode, BAD_CAST attr, BAD_CAST value);
}


// utility to check the material name
static bool isMaterialName(xmlNodePtr materialNode, const char* nameToCheck)
{
  // load value
  xmlChar* actName = xmlGetProp(materialNode, BAD_CAST "name");
  // check
  bool result = STREQ((const char*) actName, nameToCheck);
  // free loaded value
  xmlFree(actName);

  return result;
}



PhysicsConfig::PhysicsConfig(const char* xmlFile)
{
  init(xmlFile);
}

PhysicsConfig::PhysicsConfig(const PhysicsConfig& copyFromMe)
{
  initFromCopy(copyFromMe);
}

PhysicsConfig& PhysicsConfig::operator= (const PhysicsConfig& copyFromMe)
{
  // check for self-assignment by comparing the address of the
  // implicit object and the parameter
  if (this == &copyFromMe)
  {
    return *this;
  }

  RFREE(xmlFile);
  xmlFreeDoc(doc);
  initFromCopy(copyFromMe);

  // return the existing object
  return *this;
}

void PhysicsConfig::init(const char* configFile)
{
  // store passed file name
  if (configFile != NULL)
  {
    this->xmlFile = String_clone(configFile);
  }

  // Determine absolute file name of config file and copy the XML file name
  char filename[256] = "";
  bool fileExists = Rcs_getAbsoluteFileName(configFile, filename);

  if (!fileExists)
  {
    RLOG(1, "Rcs physics configuration file \"%s\" not found",
        configFile ? configFile : "NULL");
    // Build backing doc manually

    // create empty configuration node
    this->doc = xmlNewDoc(BAD_CAST "1.0");
    this->root = xmlNewDocNode(doc, NULL, BAD_CAST "content", NULL);
    xmlDocSetRootElement(doc, root);

    // create material node for default material
    defaultMaterial = xmlNewDocNode(doc, NULL, BAD_CAST "material", NULL);
    xmlSetProp(defaultMaterial, BAD_CAST "name", BAD_CAST DEFAULT_MATERIAL_NAME);
    xmlAddChild(root, defaultMaterial);
  }
  else
  {
    // load xml tree
    this->root = parseXMLFile(filename, "content", &this->doc);
    RCHECK(this->root);

    // find default material
    findDefaultMaterial();
  }
}

void PhysicsConfig::initFromCopy(const PhysicsConfig& copyFromMe)
{
  xmlFile = String_clone(copyFromMe.xmlFile);
  // copy xml document
  doc = xmlCopyDoc(copyFromMe.doc, 1);
  RCHECK(doc);
  root = xmlDocGetRootElement(doc);

  // setup default material
  findDefaultMaterial();
}

void PhysicsConfig::findDefaultMaterial()
{
  xmlNodePtr node = root->children;
  while (node)
  {
    if (isXMLNodeName(node, "material"))
    {
      if (isMaterialName(node, "default"))
      {
        defaultMaterial = node;
      }
    }
    node = node->next;
  }
  if (!defaultMaterial)
  {
    // create material node for default material
    defaultMaterial = xmlNewDocNode(doc, NULL, BAD_CAST "material", NULL);
    xmlSetProp(defaultMaterial, BAD_CAST "name", BAD_CAST DEFAULT_MATERIAL_NAME);
    xmlAddChild(root, defaultMaterial);
  }
}

PhysicsConfig::~PhysicsConfig()
{
  // free filename
  RFREE(xmlFile);
  // free xml document
  xmlFreeDoc(doc);
}

PhysicsMaterial PhysicsConfig::getFirstMaterial() const
{
  xmlNodePtr node = root->children;
  while (node)
  {
    if (isXMLNodeName(node, "material"))
    {
      return {node, defaultMaterial};
    }
    node = node->next;
  }
  return { NULL, NULL };
}

PhysicsMaterial PhysicsConfig::getDefaultMaterial() const
{
  return { defaultMaterial, defaultMaterial };
}

PhysicsMaterial PhysicsConfig::getOrCreateMaterial(const char* materialName)
{
  if (STREQ(materialName, DEFAULT_MATERIAL_NAME))
  {
    // the default material can be found quickly
    return { defaultMaterial, defaultMaterial };
  }

  // search list
  PhysicsMaterial mat = getFirstMaterial();
  while (mat)
  {
    if (isMaterialName(mat.materialNode, materialName))
    {
      return mat;
    }
    mat = mat.next();
  }
  // create new material. xml is empty at first.
  xmlNodePtr newMat = xmlNewDocNode(doc, NULL, BAD_CAST "material", NULL);
  xmlSetProp(newMat, BAD_CAST "name", BAD_CAST materialName);
  xmlAddChild(root, newMat);
  return { newMat, defaultMaterial };
}

PhysicsMaterial PhysicsConfig::getMaterial(const char* materialName) const
{
  if (STREQ(materialName, DEFAULT_MATERIAL_NAME))
  {
    // the default material can be found quickly
    return { defaultMaterial, defaultMaterial };
  }
  // search list
  PhysicsMaterial mat = getFirstMaterial();
  while (mat)
  {
    if (isMaterialName(mat.materialNode, materialName))
    {
      return mat;
    }
    mat = mat.next();
  }
  // return default material since it wasn't found
  return { defaultMaterial, defaultMaterial };
}

const char* PhysicsConfig::getConfigFileName() const
{
  return xmlFile;
}

xmlNodePtr PhysicsConfig::getXMLRootNode() const
{
  return root;
}

} /* namespace Rcs */
