/*
 * PhysicsConfig.h
 *
 *  Created on: Aug 24, 2018
 *      Author: ftreede
 */

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
  std::vector<std::string> getMaterialNames() const;

  /**
   * Return the root xml node of the config file.
   */
  xmlNodePtr getXMLRootNode() const;


private:

  // load the material data from xml
  void loadMaterials();

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
