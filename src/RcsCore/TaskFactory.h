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

#ifndef RCS_TASKFACTORY_H
#define RCS_TASKFACTORY_H

#include "Task.h"

#include <map>



namespace Rcs
{

/*! \brief This class is a factory to create task instances. It works with a
 *         registrar system: Each task that is intended to be constructible
 *         with this factory needs to implement a registrar instance (examples
 *         can be found in many of this libraries tasks). This templated
 *         registrar class registers a task by name with this factory. It also
 *         registers the task's constructor, and a check function that allows
 *         verifying if the task's xml description is specified correctly. The
 *         factory provides methods to then construct tasks by name / xml. In
 *         addition, it provides a static convenience method to construct any
 *         registered task with a string that holds a valid xml description.
 *         For instance:
 *
 *         const char* descr =
 *           "<Task controlVariable=\"XYZ\" effector=\"PowerGrasp_L\" />";
 *
 *         Task* task = TaskFactory::createTask(descr, graph);
 *
 *         This will construct a Position3d task with the given end effector.
 *         How cool isn't it?
 */
class TaskFactory
{
public:
  typedef Task* (*TaskCreateFunction)(std::string className,
                                      xmlNode* node,
                                      RcsGraph* graph);

  typedef bool (*TaskCheckFunction)(xmlNode* node,
                                    const RcsGraph* graph);

  /*! \brief Get the single instance of the factory
   *  \return singleton instance
   */
  static TaskFactory* instance();

  /*! \brief Creates a new Task by name using the appropriate registered
   *        construction function.
   *
   * \param className The name with which the task is registered at the
   *        factory
   *  \param node The xml node used for parsing the task
   * \param graph The underlying graph for the kinematics
   * \return New task instance
   */
  Task* createTask(std::string className, xmlNode* node, RcsGraph* graph);


  /*! \brief Convenience method to create a task from a string.
   *
   *  \param xmlStr String with a valid task xml description
   *  \param graph The underlying graph for the kinematics
   *  \return New task instance, or NULL in case it can't be constructed.
   */
  static Task* createTask(const char* xmlStr, RcsGraph* graph);

  /*! \brief Registers a new function for creating tasks. You should not
   *        need to call this function directly. Instead us the
   *        TaskFactoryRegistrar by adding the following line to your
   *        implementation:
   *        "static Rcs::TaskFactoryRegistrar<Rcs::Task> task("name");"
   */
  void registerTaskFunctions(std::string name,
                             TaskCreateFunction createFunction,
                             TaskCheckFunction checkFunction);

  /*! \brief Prints the list of all registered tasks to stdout.
   */
  void printRegisteredTasks() const;

  /*! \brief Checks if the task described by the xml node and the
   *        underlying graph is valid.
   * \return true for valid, false otherwise.
   */
  static bool isValid(xmlNode* node, const RcsGraph* graph);

private:
  /*! \brief Private constructor because TaskFactory is a singleton
   */
  TaskFactory();

  /*! \brief Checks if the task described by the xml node and the
   *        underlying graph, along with the className, is valid.
   *
   * \return true for valid, false otherwise.
   */
  bool checkTask(const std::string& className,
                 xmlNode* node,
                 const RcsGraph* graph);

  std::map<std::string, TaskCheckFunction>  checkFunctionMap;
  std::map<std::string, TaskCreateFunction> createFunctionMap;
};





/*! \brief This class is inspired by the MPFactoryRegistrar and follows the
 *        same concepts.
 */
template<class T>
class TaskFactoryRegistrar
{
public:

  /*! \brief Registers a new task with a given name
   * \param className The name that is used for instantiating a new task
   *        by name
   */
  TaskFactoryRegistrar(std::string className)
  {
    // Register the function to create and check the task
    TaskFactory* tf = TaskFactory::instance();
    tf->registerTaskFunctions(className, &TaskFactoryRegistrar::create,
                              &T::isValid);
  }

  /*! \brief This function creates a new task of type T passing the given
   *        variables to the respective constructor
   *
   * \param className String identifier for task
   * \param node XML node for the task
   * \param graph Pointer to tasks's RcsGraph structure
   * \return New task instance of type T
   */
  static Task* create(std::string className, xmlNode* node,
                      RcsGraph* graph)
  {
    return new T(className, node, graph);
  }

};

}

#endif // RCS_TASKFACTORY_H
