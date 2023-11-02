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

#include "TaskFactory.h"
#include "Rcs_macros.h"
#include "Rcs_parser.h"
#include "Rcs_stlParser.h"


namespace Rcs
{

/*******************************************************************************
 * Private destructor
 ******************************************************************************/
TaskFactory::TaskFactory()
{
}

/*******************************************************************************
 * Returns singleton instance pointer
 ******************************************************************************/
TaskFactory* TaskFactory::instance()
{
  static TaskFactory factory;
  return &factory;
}

/*******************************************************************************
 * Creates the task for className and the given graph and xml content
 ******************************************************************************/
Task* TaskFactory::createTask(std::string str, const RcsGraph* graph)
{
  const char* xmlStr = str.c_str();
  xmlDocPtr doc;
  xmlNodePtr node = parseXMLMemory(xmlStr, strlen(xmlStr), &doc);
  Task* task = createTask(node, graph);
  xmlFreeDoc(doc);

  return task;
}

/*******************************************************************************
 * Creates the task for className and the given graph and xml content
 ******************************************************************************/
std::vector<Task*> TaskFactory::createTasks(const std::vector<std::string>& strs,
                                            const RcsGraph* graph)
{
  std::vector<Task*> tasks;

  // We go through all tasks and do not return on the first failure in order
  // to determine not only the first failed task creation, but log out all
  // of them at once.
  bool valid = true;
  for (size_t i=0; i<strs.size(); ++i)
  {
    Task* ti = TaskFactory::createTask(strs[i], graph);

    if (!ti)
    {
      valid = false;
      RLOG_CPP(1, "This task could not be created:\n\n" << strs[i]);
    }
    else
    {
      tasks.push_back(ti);
    }
  }

  // If we found one or more NULL task, we delete all tasks and return an
  // empty vector
  if (!valid)
  {
    for (size_t i=0; i<tasks.size(); ++i)
    {
      delete tasks[i];   // deleting NULL is safe
    }

    tasks.clear();
  }

  return tasks;
}

/*******************************************************************************
 * Creates the task for className and the given graph and xml content
 ******************************************************************************/
Task* TaskFactory::createTask(xmlNode* node, const RcsGraph* graph)
{
  TaskFactory* tf = TaskFactory::instance();
  std::string cVar = getXMLNodePropertySTLString(node, "controlVariable");

  // Find name in the registry and call factory method.
  std::map<std::string, TaskBuilder>::iterator itCreate;
  itCreate = tf->createFuncMap.find(cVar);

  if (itCreate == tf->createFuncMap.end())
  {
    RLOG_CPP(1, "Unknown task type \"" << cVar << "\"");
    REXEC(4)
    {
      printRegisteredTasks();
    }
    return NULL;
  }


  // First we check if the task is valid or not.
  std::map<std::string, TaskChecker>::iterator it1;
  it1 = tf->checkFuncMap.find(cVar);

  if (it1 == tf->checkFuncMap.end())
  {
    RLOG(1, "Task type \"%s\" has no check function - skipping", cVar.c_str());
    return NULL;
  }
  else
  {
    RLOG(5, "Checking task of type \"%s\"", cVar.c_str());
    bool success = it1->second(node, graph);

    if (success == false)
    {
      RLOG(1, "Task of type \"%s\" could not be created", cVar.c_str());
      return NULL;
    }
  }

  RLOG_CPP(5, "Creating task of type \"" << cVar << "\"");

  return itCreate->second(cVar, node, graph);
}

/*******************************************************************************
 * Creates the task for className and the given graph and xml content
 ******************************************************************************/
Task* TaskFactory::createRandomTask(std::string cVar,
                                    const RcsGraph* graph)
{
  TaskFactory* tf = TaskFactory::instance();

  // Find name in the registry and call factory method.
  std::map<std::string, RandomTaskBuilder>::iterator itCreate;
  itCreate = tf->rndBuilderMap.find(cVar);

  if (itCreate == tf->rndBuilderMap.end())
  {
    RLOG_CPP(1, "Unknown task type \"" << cVar << "\"");
    REXEC(4)
    {
      printRegisteredTasks();
    }
    return NULL;
  }



  return itCreate->second(cVar, graph);
}

/*******************************************************************************
 * This function is called through the registrat class. This happens before
 * main() is entered. Therefore, logging with debug levels doesn't make sense,
 * since the debug level has at that point not yet been parsed.
 ******************************************************************************/
void TaskFactory::registerTaskFunctions(std::string name,
                                        TaskBuilder createFunction,
                                        TaskChecker checkFunction,
                                        RandomTaskBuilder rndBuilder)
{
  // Register check function for task of the type given in "name"
  std::map<std::string, TaskChecker>::const_iterator it2;

  for (it2 = checkFuncMap.begin(); it2 != checkFuncMap.end(); ++it2)
  {
    // If the check function has already been registered for a task with a
    // different  create-function, this means that the class with the given
    // name does not have  its own isValid() function. I haven't found a way
    // to catch it at compile time, therefore we'll exit in the registration
    // procedure (That's before main() is called)
    if ((checkFunction == it2->second) &&
        (createFunction != createFuncMap[it2->first]))
    {
      RFATAL("Task of class \"%s\" has same test function adress as task "
             "class \"%s\" - did you implement a \"isValid()\" function "
             "for this task?", name.c_str(), it2->first.c_str());
    }
  }

  // Check if there is already an entry with the passed class name. If yes, we
  // emit a warning on debug level 1.
  std::map<std::string, TaskBuilder>::iterator itCreate;
  itCreate = createFuncMap.find(name);

  if (itCreate != createFuncMap.end())
  {
    RLOG(1, "Task type \"%s\" will be overwritten!", name.c_str());
  }

  // Register create and check functions for task of the type given in "name"
  checkFuncMap[name] = checkFunction;
  createFuncMap[name] = createFunction;
  rndBuilderMap[name] = rndBuilder;
}


/*******************************************************************************
 * Prints all registered tasks to the console
 ******************************************************************************/
void TaskFactory::printRegisteredTasks()
{
  TaskFactory* tf = TaskFactory::instance();

  RMSG("TaskFactory: registered tasks are");
  std::map<std::string, TaskBuilder>::const_iterator it;
  for (it = tf->createFuncMap.begin(); it != tf->createFuncMap.end(); ++it)
  {
    printf("%s\n", it->first.c_str());
  }

  RLOGS(5, "=== Registered check functions:");
  std::map<std::string, TaskChecker>::const_iterator it2;
  for (it2 = tf->checkFuncMap.begin(); it2 != tf->checkFuncMap.end(); ++it2)
  {
    RLOG_CPP(5, "Function: " << it2->first << " address: " << it2->second);
  }

  std::map<std::string, RandomTaskBuilder>::const_iterator it3;
  for (it3 = tf->rndBuilderMap.begin(); it3 != tf->rndBuilderMap.end(); ++it3)
  {
    RLOG_CPP(0, "Function: " << it3->first << " address: " << it3->second);
  }


}

/*******************************************************************************
 * Factory method for checking of validity
 ******************************************************************************/
bool TaskFactory::isValid(xmlNode* node, const RcsGraph* graph)
{
  // Check if tag is of type "Task"
  if (isXMLNodeName(node, "Task") == false)
  {
    RLOG(5, "XML node \"%s\" is not a \"Task\"", (char*) node->name);
    return false;
  }

  // Get the task name
  std::string taskName = getXMLNodePropertySTLString(node, "name");

  // Check if a control variable is defined
  if (getXMLNodeProperty(node, "controlVariable") == false)
  {
    RLOG(4, "Task \"%s\" has no control variable", taskName.c_str());
    return false;
  }

  // Call class's check function
  std::string className = getXMLNodePropertySTLString(node, "controlVariable");
  std::map<std::string, TaskChecker>::iterator it;
  it = TaskFactory::instance()->checkFuncMap.find(className);

  if (it == TaskFactory::instance()->checkFuncMap.end())
  {
    RLOG(1, "Task type \"%s\" has no check function", className.c_str());
    return false;
  }

  RLOG(5, "Checking task of type \"%s\"", className.c_str());

  return it->second(node, graph);
}

}   // namespace Rcs
