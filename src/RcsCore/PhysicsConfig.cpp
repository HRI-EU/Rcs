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

PhysicsMaterial::PhysicsMaterial()
{
  frictionCoefficient = 0.8;
  rollingFrictionCoefficient = 0.0;
  restitution = 0.0;

  materialNode = NULL;
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
    defaultMaterial.materialNode = xmlNewDocNode(doc, NULL, BAD_CAST "material", NULL);
    xmlSetProp(defaultMaterial.materialNode, BAD_CAST "name", BAD_CAST DEFAULT_MATERIAL_NAME);
    xmlAddChild(root, defaultMaterial.materialNode);
  }
  else
  {
    // load xml tree
    this->root = parseXMLFile(filename, "content", &this->doc);
    RCHECK(this->root);

    // load material definitions
    loadMaterials();
  }
}

void PhysicsConfig::initFromCopy(const PhysicsConfig& copyFromMe)
{
  xmlFile = String_clone(copyFromMe.xmlFile);
  // copy xml document
  doc = xmlCopyDoc(copyFromMe.doc, 1);
  RCHECK(doc);
  root = xmlDocGetRootElement(doc);

  // create material node for default material
  defaultMaterial.materialNode = xmlNewDocNode(doc, NULL, BAD_CAST "material", NULL);
  xmlSetProp(defaultMaterial.materialNode, BAD_CAST "name", BAD_CAST DEFAULT_MATERIAL_NAME);
  xmlAddChild(root, defaultMaterial.materialNode);

  // copy modified material data
  defaultMaterial.frictionCoefficient = copyFromMe.defaultMaterial.frictionCoefficient;
  defaultMaterial.rollingFrictionCoefficient = copyFromMe.defaultMaterial.rollingFrictionCoefficient;
  defaultMaterial.restitution = copyFromMe.defaultMaterial.restitution;

  for (MaterialMap::const_iterator it = copyFromMe.materials.begin(); it != copyFromMe.materials.end(); it++)
  {
    PhysicsMaterial* mat = getMaterial(it->first);

    mat->frictionCoefficient = it->second.frictionCoefficient;
    mat->rollingFrictionCoefficient = it->second.rollingFrictionCoefficient;
    mat->restitution = it->second.restitution;
  }
}

PhysicsConfig::~PhysicsConfig()
{
  // free filename
  RFREE(xmlFile);
  // free xml document
  xmlFreeDoc(doc);

  // the material map is freed automatically
}

PhysicsMaterial* PhysicsConfig::getMaterial(const std::string& materialName)
{
  if (materialName == DEFAULT_MATERIAL_NAME)
  {
    // the default material is not stored in the map
    return &defaultMaterial;
  }

  // obtain reference from map. using [] automatically creates a new entry for missing objects
  PhysicsMaterial* mat = &materials[materialName];

  if (mat->materialNode == NULL)
  {
    // a newly created material. copy values from default material.
    *mat = defaultMaterial;

    // create a deep copy of the xml node. We use xmlDocCopy so that the new node is also owned by the doc
    mat->materialNode = xmlDocCopyNode(mat->materialNode, doc, 1);
    xmlAddChild(root, mat->materialNode);
  }

  return mat;
}

const PhysicsMaterial* PhysicsConfig::getMaterial(const std::string& materialName) const
{
  if (materialName == DEFAULT_MATERIAL_NAME)
  {
    // the default material is not stored in the map
    return &defaultMaterial;
  }
  // try to find in map
  MaterialMap::const_iterator it = materials.find(materialName);

  if (it != materials.end())
  {
    // found
    return &it->second;
  }
  else
  {
    // not found. since the result is const, just return the default material
    return &defaultMaterial;
  }
}

const char* PhysicsConfig::getConfigFileName() const
{
  return xmlFile;
}

xmlNodePtr PhysicsConfig::getXMLRootNode() const
{
  return root;
}

PhysicsConfig::MaterialNameList PhysicsConfig::getMaterialNames() const
{
  MaterialNameList result;
  result.reserve(materials.size() + 1);

  // add default material name
  result.push_back(DEFAULT_MATERIAL_NAME);
  // add other material names
  for (MaterialMap::const_iterator it = materials.begin(); it != materials.end(); it++)
  {
    result.push_back(it->first);
  }

  return result;
}

PhysicsMaterial* PhysicsConfig::getDefaultMaterial()
{
  return &defaultMaterial;
}

const PhysicsMaterial* PhysicsConfig::getDefaultMaterial() const
{
  return &defaultMaterial;
}

void PhysicsConfig::loadMaterials()
{
  char msg[256];

  xmlNodePtr node = root->children;
  while (node)
  {
    if (isXMLNodeName(node, "material"))
    {
      if (getXMLNodePropertyStringN(node, "name", msg, 256))
      {
        PhysicsMaterial* material;
        if (STREQ(msg, DEFAULT_MATERIAL_NAME))
        {
          // default material definition
          if (!materials.empty())
          {
            RLOG_CPP(1, materials.size() << " materials have been defined "
                 "before the default material. They will not be able to use"
         " the correct values.");
          }
          material = &defaultMaterial;
        }
        else
        {
          // create new material from data
          material = &materials[msg];
          // copy values from default material.
          *material = defaultMaterial;
        }
        material->materialNode = node;

        double value;
//  char option[64];

        if (getXMLNodePropertyDouble(node, "friction_coefficient", &value))
        {
          material->frictionCoefficient = value;
        }
        if (getXMLNodePropertyDouble(node, "rolling_friction_coefficient", &value))
        {
          material->rollingFrictionCoefficient = value;
        }
//  if (getXMLNodePropertyDouble (node, "static_friction_scale", &value))
//  {
//    material->setStaticFrictionScale (
//        Vx::VxMaterialBase::kFrictionAxisLinear, value);
//  }
//  if (getXMLNodePropertyDouble (node, "slip", &value))
//  {
//    material->setSlip (Vx::VxMaterialBase::kFrictionAxisLinear, value);
//  }
//  bool isd = false;
//  if (getXMLNodePropertyBoolString (node, "integrated_slip_displacement",
//            &isd))
//  {
//    if (isd)
//    {
//      material->setIntegratedSlipDisplacement (
//    Vx::VxMaterial::kIntegratedSlipDisplacementActivated);
//    }
//    else
//    {
//      material->setIntegratedSlipDisplacement (
//    Vx::VxMaterial::kIntegratedSlipDisplacementDeactivated);
//    }
//  }
//  if (getXMLNodePropertyDouble (node, "slide", &value))
//  {
//    material->setSlide (Vx::VxMaterialBase::kFrictionAxisLinear, value);
//  }
//  if (getXMLNodePropertyDouble (node, "compliance", &value))
//  {
//    material->setCompliance (value);
//    // as a default, set critical damping
//    // material->setDamping(universe->getCriticalDamping(value));
//    // from the Vortex documentation:
//    // Nearly optimal value of damping is given by:
//    // damping = 5*time_step/compliance
//    material->setDamping ((5.0 * this->integratorDt) / value);
//  }
//  if (getXMLNodePropertyDouble (node, "damping", &value))
//  {
//    material->setDamping (value);
//  }
        if (getXMLNodePropertyDouble(node, "restitution", &value))
        {
          material->restitution = value;
        }
//  if (getXMLNodePropertyDouble (node, "restitution_threshold", &value))
//  {
//    material->setRestitutionThreshold (value);
//  }
//  if (getXMLNodePropertyDouble (node, "adhesive_force", &value))
//  {
//    material->setAdhesiveForce (value);
//  }
      }
      else
      {
        RLOG(1, "Found a material entry without a name property, so"
                " it is ignored");
      }
    }

    node = node->next;
  }

}

} /* namespace Rcs */
