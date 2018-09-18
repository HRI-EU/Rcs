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

#ifndef SRC_RCSPHYSICS_PHYSICSCONFIG_H_
#define SRC_RCSPHYSICS_PHYSICSCONFIG_H_

#include <libxml/tree.h>

#include <map>
#include <vector>
#include <string>

namespace Rcs
{

/**
 * Name of the default material.
 *
 * When a PhysicsMaterial is created, it's values are initialized to those of the default material.
 *
 * Note that later modifications of the parameter values will not be used.
 */
#define DEFAULT_MATERIAL_NAME "default"

/**
 * Definition of the material properties used by the physics simulator.
 */
struct PhysicsMaterial
{
  // coefficient of linear friction
  double frictionCoefficient;
  // coefficient of angular (rolling) friction.
  double rollingFrictionCoefficient;
  // bouncyness
  double restitution;
  // TODO add others?

  // xml node whose properties defined this material.
  // allows physics engines to load engine-specific attributes
  xmlNodePtr materialNode;

  // default constructor applies default values
  PhysicsMaterial();
};

/**
 * Physics engine configuration parameters.
 *
 * This is the C++-side model of the physics configuration XML file.
 *
 * The first part are the PhysicsMaterial definitions. Every material definition
 * holds material properties such as friction coefficients. When creating a shape,
 * the physics engine will use the shape's material property to select a named
 * material definition.
 * Some physics engines cannot use different material parameters for shapes on the same body.
 * In that case, they should use the values from the material of the first physics shape.
 *
 * To support physics engine specific parameters, the object also holds the parsed xml tree.
 */
class PhysicsConfig
{
public:
  typedef std::map<std::string, PhysicsMaterial> MaterialMap;
  typedef std::vector<std::string> MaterialNameList;

  /**
   * Load the physics configuration from the given xml file.
   */
  explicit PhysicsConfig(const char* xmlFile);
  virtual ~PhysicsConfig();

  /**
   * Obtain the material properties of the named material.
   *
   * If a material of the given name doesn't exist, a new one is inserted,
   * copying it's data from the default material.
   *
   * The returned material object can be modified.
   *
   * The returned object is owned by the PhysicsConfig object.
   *
   * @param materialName material name to look up
   * @return named material
   */
  PhysicsMaterial* getMaterial(const std::string& materialName);

  /**
   * Obtain the material properties of the named material.
   *
   * If a material of the given name doesn't exist, the default material is returned.
   *
   * The returned object is owned by the PhysicsConfig object.
   *
   * @param materialName material name to look up
   * @return named material
   */
  const PhysicsMaterial* getMaterial(const std::string& materialName) const;

  /**
   * Get a reference to the default material data.
   */
  PhysicsMaterial* getDefaultMaterial();

  /**
   * Get a reference to the default material data.
   */
  const PhysicsMaterial* getDefaultMaterial() const;

  /**
   * Get the names of all registered materials.
   */
  MaterialNameList getMaterialNames() const;

  /**
   * Return the name of the xml file the config was loaded from.
   */
  const char* getConfigFileName() const;

  /**
   * Return the root xml node of the config file.
   */
  xmlNodePtr getXMLRootNode() const;


private:

  // load the material data from xml
  void loadMaterials();

  // full path of xml file
  char* xmlFile;

  // xml document, owns all xml objects
  xmlDocPtr doc;
  // document root node
  xmlNodePtr root;

  // default material data
  PhysicsMaterial defaultMaterial;
  // map from material name to material data
  MaterialMap materials;
};

} /* namespace Rcs */

#endif /* SRC_RCSPHYSICS_PHYSICSCONFIG_H_ */
