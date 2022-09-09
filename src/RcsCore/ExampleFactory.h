/*******************************************************************************

  Copyright (c) Honda Research Institute Europe GmbH.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its
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

#ifndef RCS_EXAMPLEFACTORY_H
#define RCS_EXAMPLEFACTORY_H

#include <ExampleBase.h>

#include <string>
#include <map>
#include <set>
#include <utility>



/*! \brief Convenience macro to register examples in the factory. Here is an
 * example: For the examples ABC, the macro expands to
 * static Rcs::ExampleFactoryRegistrar<ABC> ABC_("ABC")
 */
#define REGISTER_EXAMPLE(T) static Rcs::ExampleFactoryRegistrar<T> T ## _(#T)


namespace Rcs
{

/*! \brief Factory class for ExampleBase classes and its
 *         derieved classes. The factory implements methods to construct
 *         classes of type ExampleBase (and derieved from them). It is based
 *         on a registrar class. In order to enable a
 *         class derived from ExampleBase to be used with this
 *         factory class, the following needs to be provided:
 *
 *         - A constructor that constructs an instance from an xml node: e.g.
 *           MyNewExample::MyNewExample(int argc, char** argv);
 *         - Inserting a macro to register the new set in the implementation
 *           file: REGISTER_EXAMPLE(MyNewExample);
 *
 *         Most of the classes in this library have been implemented like this.
 *         See for instance ExampleFK.
 */
class ExampleFactory
{
  template <class T> friend class ExampleFactoryRegistrar;

public:

  /*! \brief Creates a new example by name using the registered construction
   *         function.
   *
   * \param[in] category  Name of the example category
   * \param[in] example   Name of the example
   * \param[in] argc      Number of command line arguments
   * \param[in] argv      String array of command line arguments
   * \return              New ExampleBase instance or NULL incase of failure
   */
  static ExampleBase* create(std::string category, std::string example,
                             int argc, char** argv);

  /*! \brief Prints out all registered examples to the console
   */
  static void print();

  /*! \brief Checks if an example with the given name has been registered
   */
  static bool hasExample(std::string category, std::string example);

  static std::set<std::string> getCategories();

  /*! \brief Signature of example creation function.
     */
  typedef ExampleBase* (*ExampleMaker)(int argc, char** argv);
  typedef std::map<std::pair<std::string, std::string>, ExampleMaker> ExampleMap;

  static ExampleMap& constructorMap();

private:

  /*! \brief Private constructor because ExampleFactory is a singleton class
   */
  ExampleFactory();

  /*! \brief Registers a new function for creating examples. You can not
   *        call this function directly. Instead us the above macro.
   */
  static void registerExample(std::string category, std::string example,
                              ExampleMaker createFunction);
};





/*! \brief Registrar class for example classes. Here is how to use
 *        it:
 *        - Implement an example derieved from ExampleBase
 *        - In the implementation of this class on the global scope, add:<br>
 *          REGISTER_EXAMPLE(MyCoolNewExample);
 *        - This registers an example of type MyCoolNewExample that can be
 *          instantiated : <br>
 *          ExampleBase* ex = ExampleFactory::create(argc, argv);
 */
template<class T>
class ExampleFactoryRegistrar
{
public:

  /*! \brief Registers a new example with a given name. This line
   *         needs to be put into the cpp file:
   *         REGISTER_Example(MyCoolNewExample);
   *
   *         Then, you can create a MyCoolNewExample such as
   *         ExampleBase* ex = ExampleFactory::create(argc, argv);
   *
   *  \param[in] category  Name of the example category
   *  \param[in] example   Name of the example
   */
  ExampleFactoryRegistrar(std::string category, std::string example)
  {
    // Register the function to create the example
    ExampleFactory::registerExample(category, example,
                                    &ExampleFactoryRegistrar::create);
  }

private:

  /*! \brief This function creates a new example instance of type T
   *         passing the given variables to the respective constructor.
   *
   * \param argc      Number of command line arguments
   * \param argv      String array of command line arguments
   * \return          New example of type T
   */
  static ExampleBase* create(int argc, char** argv)
  {
    return new T(argc, argv);
  }
};

}

#endif // RCS_EXAMPLEFACTORY_H
