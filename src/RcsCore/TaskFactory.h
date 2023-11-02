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

#ifndef RCS_TASKFACTORY_H
#define RCS_TASKFACTORY_H

#include "Task.h"

#include <map>


// Convenience macro to register Tasks in the factory. Here is an example:
// REGISTER_TASK(MyTask, "ABC") expands to
// static Rcs::TaskFactoryRegistrar<MyTask> MyTask_("ABC")
// It has the drawback that it can only be used once per compile unit, since
// the name is automatically created from the type. An alternative would be
// to use the template argument N (like N ## _(N)). This would only lead to
// name clashes of task identifier strings match. However, this solution
// would disallow chosing task identifiers with white spaces.
#define REGISTER_TASK(T,N) static Rcs::TaskFactoryRegistrar<T> T ## _(N)


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
 *         How cool, isn't it?
 */
class TaskFactory
{
  template<typename T>
  friend class TaskFactoryRegistrar;

public:

  /*! \brief Creates a new Task by name using the appropriate registered
   *         construction function.
   *
   *  \param node The xml node used for parsing the task. The class name is
   *              extracted from the xml node.
   *  \param graph The underlying graph for the kinematics
   *  \return New task instance
   */
  static Task* createTask(xmlNode* node, const RcsGraph* graph);

  /*! \brief Convenience method to create a task from a string.
   *
   *  \param xmlStr String with a valid task xml description
   *  \param graph The underlying graph for the kinematics
   *  \return New task instance, or NULL in case it can't be constructed.
   */
  static Task* createTask(std::string xmlStr, const RcsGraph* graph);

  /*! \brief Convenience method to create a vector of tasks from a vector
   *         of strings.
   *
   *  \param xmlStrs Strings with a valid task xml description
   *  \param graph The underlying graph for the kinematics
   *  \return Vector of tasks, or empty vector in case any of the tasks can't
   *          be constructed.
   */
  static std::vector<Task*> createTasks(const std::vector<std::string>& xmlStrs,
                                        const RcsGraph* graph);

  /*! \brief Convenience method to create a random task from a string.
   *
   *  \param xmlStr String with a valid task xml description
   *  \param graph The underlying graph for the kinematics
   *  \return New task instance, or NULL in case it can't be constructed.
   */
  static Task* createRandomTask(std::string className, const RcsGraph* graph);

  /*! \brief Prints the list of all registered tasks to stdout.
   */
  static void printRegisteredTasks();

  /*! \brief Checks if the task described by the xml node and the
   *        underlying graph is valid.
   * \return true for valid, false otherwise.
   */
  static bool isValid(xmlNode* node, const RcsGraph* graph);


private:

  typedef Task* (*TaskBuilder)(std::string className, xmlNode* node,
                               const RcsGraph* graph);

  typedef Task* (*RandomTaskBuilder)(std::string className,
                                     const RcsGraph* graph);

  typedef bool (*TaskChecker)(xmlNode* node, const RcsGraph* graph);

  std::map<std::string, TaskChecker> checkFuncMap;
  std::map<std::string, TaskBuilder> createFuncMap;
  std::map<std::string, RandomTaskBuilder> rndBuilderMap;

  /*! \brief Private constructor because TaskFactory is a singleton
   */
  TaskFactory();

  /*! \brief Get the single instance of the factory
   *  \return singleton instance
   */
  static TaskFactory* instance();

  /*! \brief Registers a new function for creating tasks. You should not
   *        need to call this function directly. Instead us the
   *        TaskFactoryRegistrar by adding the following line to your
   *        implementation:
   *        "static Rcs::TaskFactoryRegistrar<Rcs::Task> task("name");"
   */
  void registerTaskFunctions(std::string name, TaskBuilder builder,
                             TaskChecker checker, RandomTaskBuilder rndBuilder);
};





/*! \brief Registrar class that registers tasks in the TaskFactory. The
 *         registrar constructors are typically called as static instantiations
 *         on the file scope of a task. For each task, a map from the task's
 *         class name to a construction and checking function is added. These
 *         are used by the TaskFactory to decide which instance to construct
 *         based on the class name.
 */
template<class T>
class TaskFactoryRegistrar
{
public:

  /*! \brief Registers a new task with a given name
   *  \param className The name that is used for instantiating a new task
   *         by name
   */
  TaskFactoryRegistrar(std::string className)
  {
    TaskFactory::instance()->registerTaskFunctions(className, &create, &check,
                                                   &createRandom);
  }


private:



  /*! \brief This function creates a new task of type T passing the given
  *         variables to the respective constructor
  *
  *  \param className String identifier for task
  *  \param node XML node for the task
  *  \param graph Pointer to tasks's RcsGraph structure
  *  \return New task instance of type T
  */
  static Task* create(std::string className, xmlNode* node,
                      const RcsGraph* graph)
  {
    return new T(className, node, graph);
  }

  static Task* createRandom(std::string className, const RcsGraph* graph)
  {
    return T::createRandom(className, graph);
  }

  /*! \brief This function returns the result of the isValid() function of a
   *         task of type T.
   *
   *  \param node XML node for the task
   *  \param graph Pointer to tasks's RcsGraph structure
   *  \return True for the task being valid, false otherwise.
   */
  static bool check(xmlNode* node, const RcsGraph* graph)
  {
    return T::isValid(node, graph);
  }

};

}   // namespace Rcs

#endif // RCS_TASKFACTORY_H
