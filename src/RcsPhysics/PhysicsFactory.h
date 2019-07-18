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

#ifndef RCS_PHYSICSFACTORY_H
#define RCS_PHYSICSFACTORY_H

#include "PhysicsConfig.h"

#include <PhysicsBase.h>



namespace Rcs
{

/*! \ingroup RcsPhysics
 *  \brief Factory class for physics simulation classes.
 */
class PhysicsFactory
{
  template <class T> friend class PhysicsFactoryRegistrar;

public:

  /*! \brief Creates a new Physics simulation instance by name using the
   *         appropriate registered construction function.
   *
   * \param className The name with which the physics is registered at the
   *                  factory
   * \param graph     The underlying graph for the kinematics
   * \param cfgFile   Name of xml configuration file
   * \return          New PhysicsBase instance or NULL if failure happened
   */
  static PhysicsBase* create(const std::string& className, RcsGraph* graph,
                             const char* cfgFile);

  /*! \brief Creates a new Physics simulation instance by name using the
   *        appropriate registered construction function.
   *
   * \param className The name with which the physics is registered at the
   *        factory
   * \param graph The underlying graph for the kinematics
   * \param config Loaded physics configuration
   * \return New PhysicsBase instance
   */
  static PhysicsBase* create(const std::string& className, RcsGraph* graph,
                             const PhysicsConfig* config);

  /*! \brief Checks if a physics engine with the given name has been
   *         registered in the PhysicsFactory
   *
   * \param className The name with which the physics is registered at the
   *        factory
   * \return True if engine exists, false otherwise
   */
  static bool hasEngine(const std::string& className);

  /*! \brief Prints the list of all registered physics simulations to stdout.
   */
  static void print();

private:

  typedef PhysicsBase* (*PhysicsCreateFunction)(std::string className,
                                                RcsGraph* graph,
                                                const PhysicsConfig* config);

  /*! \brief Private constructor because PhysicsFactory is a singleton
   */
  PhysicsFactory();

  /*! \brief Get the single instance of the factory
   *  \return singleton instance
   */
  static PhysicsFactory* instance();

  /*! \brief Registers a new function for creating physics. You should not
   *        need to call this function directly. Instead us the
   *        PhysicsFactoryRegistrar by adding the following line to your
   *        implementation:
   *        "static Rcs::PhysicsFactoryRegistrar<Rcs::PhysicsBase>
   *                physics("name");"
   */
  void registerPhysics(const std::string& name,
                       PhysicsCreateFunction createFunction);

  std::map<std::string, PhysicsCreateFunction> constructorMap;
};





/*! \ingroup RcsPhysics
 * \brief Registrar class for physics simulation classes. Here is how to use
 *        it:
 *        - Implement a physics simulation derieved from PhysicsBase
 *        - In the implementation of this class on the global scope, add:<br>
 *          static PhysicsFactoryRegistrar<MySimulation> physics("MySim");
 *        - This registers a simulator that can be instantiated by the name
 *          MySim: <br>
 *          PhysicsBase* sim = PhysicsFactory::create("MySim", graph, cfgFile);
 */
template<class T>
class PhysicsFactoryRegistrar
{
public:

  /*! \brief Registers a new physics simulation with a given name
   *
   *  \param className The name that is used for instanciating a new
   *                   physics simulation by name
   */
  PhysicsFactoryRegistrar(std::string className)
  {
    // Register the function to create and check the physics simulation
    PhysicsFactory* tf = PhysicsFactory::instance();
    tf->registerPhysics(className, &PhysicsFactoryRegistrar::create);
  }

  /*! \brief This function creates a new physics simulation instance of type T
   *        passing the given variables to the respective constructor
   *
   * \param className String identifier for task
   * \param graph     Pointer to tasks's RcsGraph structure
   * \param config    Name of xml configuration file
   * \return          New physics simulation instance of type T
   */
  static PhysicsBase* create(std::string className, RcsGraph* graph,
                             const PhysicsConfig* config)
  {
    return new T(graph, config);
  }
};

}

#endif // RCS_PHYSICSFACTORY_H
