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

#ifndef RCS_PHYSICSCONFIG_H
#define RCS_PHYSICSCONFIG_H

#include <libxml/tree.h>

namespace Rcs
{

/*!
 * Name of the default material.
 *
 * When a PhysicsMaterial is created, it's values are initialized to those of
 * the default material.
 *
 * Note that later modifications of the parameter values will not be used.
 */
#define DEFAULT_MATERIAL_NAME "default"

/*!
 * Definition of the material properties used by the physics simulator.
 *
 * This is just a thin wrapper around an xml node that stores the actual
 * properties.
 */
struct PhysicsMaterial
{
public:
  /*!
   * Xml node whose properties defined this material. Accessible to allow
   * physics engines to load engine-specificattributes.
   */
  xmlNodePtr materialNode;

private:

  // default material, used as fallback by getters
  xmlNodePtr defaultMaterialNode;

  // ctor is private, for use by this an PhysicsConfig only.
  PhysicsMaterial(xmlNodePtr node, xmlNodePtr defaultMaterialNode);

  friend class PhysicsConfig;

public:
  /*!
   * Create an empty material reference.
   */
  PhysicsMaterial();

  // treat as xml node pointer for iteration

  //! check if this is a non-empty reference
  operator bool() const;

  /*!
   * Obtain next material. Returns an empty material if done.
   */
  PhysicsMaterial next() const;

  //! check if this is the default material
  bool isDefault() const;

  // generic accessor methods

  /*!
   * Read a double attribute.
   *
   * Will query the default material if not found.
   *
   * \param attr attribute name
   * \param out value storage
   * \return true if the attribute was found
   */
  bool getDouble(const char* attr, double& out) const;

  /*!
   * Write a double attribute.
   *
   * \param attr attribute name
   * \param value new value
   */
  void setDouble(const char* attr, double value);

  /*!
   * Read a boolean attribute.
   *
   * Will query the default material if not found.
   *
   * \param attr attribute name
   * \param out value storage
   * \return true if the attribute was found
   */
  bool getBoolean(const char* attr, bool& out) const;

  /*!
   * Write a boolean attribute.
   *
   * \param attr attribute name
   * \param value new value
   */
  void setBoolean(const char* attr, bool value);

  /*!
   * Read a string attribute.
   *
   * Will query the default material if not found.
   *
   * \param attr attribute name
   * \param out value storage
   * \param limit maximum string length
   * \return true if the attribute was found
   */
  bool getString(const char* attr, char* out, unsigned int limit) const;

  /*!
   * Write a string attribute.
   *
   * \param attr attribute name
   * \param value new value
   */
  void setString(const char* attr, const char* value);


  // these are shortcuts for commonly used properties.

  //! Material name
  void getMaterialName(char name[256]) const;

  double getFrictionCoefficient() const;
  void setFrictionCoefficient(double value);
  double getRollingFrictionCoefficient() const;
  void setRollingFrictionCoefficient(double value);
  double getRestitution() const;
  void setRestitution(double value);
  double getSlip() const;
  void setSlip(double value);
};

/*!
 * Physics engine configuration parameters.
 *
 * This is the C++-side model of the physics configuration XML file. All
 * properties are stored in the xml structure, we merely provide more
 * convenient access.
 *
 * The first part are the PhysicsMaterial definitions. Every material
 * definition holds material properties such as friction coefficients. When
 * creating a shape, the physics engine will use the shape's material property
 * to select a named material definition. Some physics engines cannot use
 * different material parameters for shapes on the same body. In that case,
 * they should use the values from the material of the first physics shape.
 *
 * To support physics engine specific parameters, the object also holds the
 * parsed xml tree.
 */
class PhysicsConfig
{
public:
  /*!
   * Load the physics configuration from the given xml file.
   */
  PhysicsConfig(const char* xmlFile);

  PhysicsConfig(const PhysicsConfig& copyFromMe);

  PhysicsConfig& operator = (const PhysicsConfig&);

  virtual ~PhysicsConfig();

  /*!
   * Obtain the material properties of the named material.
   *
   * If a material of the given name doesn't exist, a new one is inserted,
   * copying it's data from the default material.
   *
   * The returned material object can be modified.
   *
   * The returned object is owned by the PhysicsConfig object.
   *
   * \param materialName material name to look up
   * \return named material
   */
  PhysicsMaterial getOrCreateMaterial(const char* materialName);

  /*!
   * Obtain the material properties of the named material.
   *
   * If a material of the given name doesn't exist, the default material is
   * returned.
   *
   * The returned object is owned by the PhysicsConfig object.
   *
   * \param materialName material name to look up
   * \return named material
   */
  PhysicsMaterial getMaterial(const char* materialName) const;

  /*!
   * Get a reference to the default material data.
   */
  PhysicsMaterial getDefaultMaterial() const;

  /*!
   * Get a reference to the first material data in the material list.
   */
  PhysicsMaterial getFirstMaterial() const;

  /*!
   * Return the name of the xml file the config was loaded from.
   */
  const char* getConfigFileName() const;

  /*!
   * Return the root xml node of the config file.
   */
  xmlNodePtr getXMLRootNode() const;


private:

  // load xml file and initialize default material
  void init(const char* configFile);

  // copy from different config. Used by the copy ctor&assignment
  // assumes fresh state, so copy assignment must clean up first.
  void initFromCopy(const PhysicsConfig&);

  // init once xml doc has been set up
  void findDefaultMaterial();

  char* xmlFile;                     ///< full path of xml file
  xmlDocPtr doc;                     ///< xml document, owns all xml objects
  xmlNodePtr root;                   ///< document root node
  xmlNodePtr defaultMaterial;        ///< default material node
};

} // namespace Rcs

#endif // RCS_PHYSICSCONFIG_H
