/*
 * PhysicsConfig.cpp
 *
 *  Created on: Aug 24, 2018
 *      Author: ftreede
 */

#include "PhysicsConfig.h"

#include <Rcs_resourcePath.h>
#include <Rcs_macros.h>
#include <Rcs_parser.h>

namespace Rcs
{

PhysicsMaterial::PhysicsMaterial()
{
  frictionCoefficient = 0.8;
  rollingFrictionCoefficient = 0.0;
  restitution = 0.0;

  materialNode = NULL;
}

PhysicsConfig::PhysicsConfig (const char* xmlFile)
{
  // Determine absolute file name of config file and copy the XML file name
  char filename[256] = "";
  bool fileExists = Rcs_getAbsoluteFileName (xmlFile, filename);

  if (!fileExists)
  {
    RMSG("Resource path is:");
    Rcs_printResourcePath ();
    RFATAL("Experiment configuration file \"%s\" not found in "
	   "ressource path - exiting",
	   xmlFile ? xmlFile : "NULL");
  }

  // load xml tree
  root = parseXMLFile (filename, "Experiment", &doc);

  // load material definitions
  loadMaterials ();
}

PhysicsConfig::~PhysicsConfig ()
{
  // free xml document
  xmlFreeDoc (doc);

  // the material map is freed automatically
}

PhysicsMaterial*
PhysicsConfig::getMaterial (const std::string& materialName)
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
    mat->materialNode = xmlDocCopyNode (mat->materialNode, doc, 1);
  }

  return mat;
}

const PhysicsMaterial*
PhysicsConfig::getMaterial (const std::string& materialName) const
{
  if (materialName == DEFAULT_MATERIAL_NAME)
  {
    // the default material is not stored in the map
    return &defaultMaterial;
  }
  // try to find in map
  MaterialMap::const_iterator it = materials.find (materialName);

  if (it != materials.end ())
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

xmlNodePtr
PhysicsConfig::getXMLRootNode () const
{
  return root;
}

std::vector<std::string>
PhysicsConfig::getMaterialNames () const
{
  std::vector<std::string> result;
  result.reserve (materials.size () + 1);

  // add default material name
  result.push_back (DEFAULT_MATERIAL_NAME);
  // add other material names
  for (MaterialMap::const_iterator it = materials.begin(); it != materials.end(); it++)
  {
    result.push_back (it->first);
  }

  return result;
}

PhysicsMaterial*
PhysicsConfig::getDefaultMaterial ()
{
  return &defaultMaterial;
}

const PhysicsMaterial*
PhysicsConfig::getDefaultMaterial () const
{
  return &defaultMaterial;
}

void
PhysicsConfig::loadMaterials ()
{
  char msg[256];

  xmlNodePtr node = root->children;
  while (node)
  {
    if (isXMLNodeName (node, "material"))
    {
      if (getXMLNodePropertyStringN (node, "name", msg, 256))
      {
	PhysicsMaterial* material;
	if (STREQ(msg, "default")) {
	  // default material definition
	  if (!materials.empty()) {
	    RLOG(1, "%d materials have been defined before the default material. "
		"They will not be able to use the correct values.", materials.size());
	  }
	  material = &defaultMaterial;
	} else {
	  // create new material from data
	  PhysicsMaterial* material = &materials[msg];
	  // copy values from default material.
	  *material = defaultMaterial;
	}
	material->materialNode = node;

	double value;
//	char option[64];

	if (getXMLNodePropertyDouble (node, "friction_coefficient", &value))
	{
	  material->frictionCoefficient = value;
	}
	if (getXMLNodePropertyDouble (node, "rolling_friction_coefficient", &value))
	{
	  material->rollingFrictionCoefficient = value;
	}
//	if (getXMLNodePropertyDouble (node, "static_friction_scale", &value))
//	{
//	  material->setStaticFrictionScale (
//	      Vx::VxMaterialBase::kFrictionAxisLinear, value);
//	}
//	if (getXMLNodePropertyDouble (node, "slip", &value))
//	{
//	  material->setSlip (Vx::VxMaterialBase::kFrictionAxisLinear, value);
//	}
//	bool isd = false;
//	if (getXMLNodePropertyBoolString (node, "integrated_slip_displacement",
//					  &isd))
//	{
//	  if (isd)
//	  {
//	    material->setIntegratedSlipDisplacement (
//		Vx::VxMaterial::kIntegratedSlipDisplacementActivated);
//	  }
//	  else
//	  {
//	    material->setIntegratedSlipDisplacement (
//		Vx::VxMaterial::kIntegratedSlipDisplacementDeactivated);
//	  }
//	}
//	if (getXMLNodePropertyDouble (node, "slide", &value))
//	{
//	  material->setSlide (Vx::VxMaterialBase::kFrictionAxisLinear, value);
//	}
//	if (getXMLNodePropertyDouble (node, "compliance", &value))
//	{
//	  material->setCompliance (value);
//	  // as a default, set critical damping
//	  // material->setDamping(universe->getCriticalDamping(value));
//	  // from the Vortex documentation:
//	  // Nearly optimal value of damping is given by:
//	  // damping = 5*time_step/compliance
//	  material->setDamping ((5.0 * this->integratorDt) / value);
//	}
//	if (getXMLNodePropertyDouble (node, "damping", &value))
//	{
//	  material->setDamping (value);
//	}
	if (getXMLNodePropertyDouble (node, "restitution", &value))
	{
	  material->restitution = value;
	}
//	if (getXMLNodePropertyDouble (node, "restitution_threshold", &value))
//	{
//	  material->setRestitutionThreshold (value);
//	}
//	if (getXMLNodePropertyDouble (node, "adhesive_force", &value))
//	{
//	  material->setAdhesiveForce (value);
//	}
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
