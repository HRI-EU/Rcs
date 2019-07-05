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


/*******************************************************************************

  \todo:
  - Check if new registered task overwrites className of previous one
  - createTask() doesn't need className argument, it is already in xml node

*******************************************************************************/

#include "TaskFactory.h"
#include "Rcs_macros.h"
#include "Rcs_parser.h"



/*******************************************************************************
 * Returns singleton instance pointer
 ******************************************************************************/
Rcs::TaskFactory* Rcs::TaskFactory::instance()
{
  static Rcs::TaskFactory factory;
  return &factory;
}

/*******************************************************************************
 * Private destructor
 ******************************************************************************/
Rcs::TaskFactory::TaskFactory()
{
}

/*******************************************************************************
 * Creates the task for className and the given graph and xml content
 ******************************************************************************/
Rcs::Task* Rcs::TaskFactory::createTask(std::string className,
                                        xmlNode* node,
                                        RcsGraph* graph)
{
  Rcs::Task* task = NULL;

  // First we check if the task is valid or not. If it is invalid, we
  // return NULL.
  std::map<std::string, TaskCheckFunction>::iterator it1;
  it1 = checkFunctionMap.find(className);

  if (it1 == checkFunctionMap.end())
  {
    RLOG(1, "Task type \"%s\" has no check function - skipping",
         className.c_str());
    return NULL;
  }
  else
  {
    RLOG(5, "Checking task of type \"%s\"", className.c_str());
    bool success = it1->second(node, graph);

    if (success == false)
    {
      RLOG(1, "Task of type \"%s\" is invalid - skipping",
           className.c_str());
      return NULL;
    }
  }


  // find name in the registry and call factory method.
  std::map<std::string, TaskCreateFunction>::iterator it;
  it = createFunctionMap.find(className);

  if (it == createFunctionMap.end())
  {
    RLOG(1, "Unknown task type \"%s\" in TaskFactory::createTask",
         className.c_str());
  }
  else
  {
    RLOG(5, "Creating task of type \"%s\" in TaskFactory::createTask",
         className.c_str());
    task = it->second(className, node, graph);
  }

  return task;
}


/*******************************************************************************
 * This function is called through the registrat class. This happens before
 * main() is entered. Therefore, logging with debug levels doesn't make sense,
 * since the debug level has at that point not yet been parsed.
 ******************************************************************************/
void Rcs::TaskFactory::registerTaskFunctions(std::string name,
                                             TaskCreateFunction createFunction,
                                             TaskCheckFunction checkFunction)
{
  // Register check function for task of the type given in "name"
  std::map<std::string, TaskCheckFunction>::const_iterator it2;

  for (it2 = checkFunctionMap.begin(); it2 != checkFunctionMap.end(); ++it2)
  {
    // If the check function has already been registered for a task with a
    // different  create-function, this means that the class with the given
    // name does not have  its own isValid() function. I haven't found a way
    // to catch it at compile time, therefore we'll exit in the registration
    // procedure (That's before main() is called)
    if ((checkFunction == it2->second) &&
        (createFunction != createFunctionMap[it2->first]))
    {
      RFATAL("Task of class \"%s\" has same test function adress as task "
             "class \"%s\" - did you implement a \"isValid()\" function "
             "for this task?", name.c_str(), it2->first.c_str());
    }
  }

  checkFunctionMap[name] = checkFunction;



  // Register create function for task of the type given in "name"
  createFunctionMap[name] = createFunction;
}


/*******************************************************************************
 * Prints all registered tasks to the console
 ******************************************************************************/
void Rcs::TaskFactory::printRegisteredTasks() const
{
  RMSG("TaskFactory: registered tasks are");
  std::map<std::string, TaskCreateFunction>::const_iterator it;
  for (it = createFunctionMap.begin(); it != createFunctionMap.end(); ++it)
  {
    printf("%s\n", it->first.c_str());
  }

  RLOGS(10, "=== Registered checks:");
  std::map<std::string, TaskCheckFunction>::const_iterator it2;
  for (it2 = checkFunctionMap.begin(); it2 != checkFunctionMap.end(); ++it2)
  {
    RLOG_CPP(10, "Function: " << it2->first << " address: " << it2->second);
  }

}


/*******************************************************************************
 * Check function for task types - goes through checkFunctionMap. This must not
 * be a static function, since we can't access private members through the
 * instance() function.
 ******************************************************************************/
bool Rcs::TaskFactory::checkTask(const std::string& className,
                                 xmlNode* node,
                                 const RcsGraph* graph)
{
  bool success = false;

  // Check if tag is of type "Task"
  if (isXMLNodeName(node, "Task") == false)
  {
    RLOG(5, "XML node \"%s\" is not a \"Task\"", (char*) node->name);
    return false;
  }

  // Get the task name
  char taskName[256];
  strcpy(taskName, "unnamed task");
  getXMLNodePropertyStringN(node, "name", taskName, 256);

  // Check if a control variable is defined
  if (getXMLNodeProperty(node, "controlVariable") == false)
  {
    RLOG(4, "Task \"%s\" has no control variable", taskName);
    return false;
  }

  // Check if the control variable matches the className
  // \todo: That doesn't make sense, since this function is called with
  // the className being the controlVariable.
  char cVar[64] = "";
  getXMLNodePropertyStringN(node, "controlVariable", cVar, 64);
  if (std::string(cVar) != className)
  {
    RLOG(4, "Task \"%s\": className (\"%s\") does not match XML tag "
         "controlVariable (\"%s\")", taskName, className.c_str(), cVar);
    return false;
  }

  // Call class's check function
  std::map<std::string, TaskCheckFunction>::iterator it;
  it = checkFunctionMap.find(className);

  if (it == checkFunctionMap.end())
  {
    RLOG(1, "Task type \"%s\" has no check function", className.c_str());
  }
  else
  {
    RLOG(5, "Checking task of type \"%s\"", className.c_str());
    success = it->second(node, graph);
  }

  return success;
}

/*****************************************************************************
 * Factory method for checking of validity
 ****************************************************************************/
bool Rcs::TaskFactory::isValid(xmlNode* node, const RcsGraph* graph)
{
  char cVar[256] = "";
  getXMLNodePropertyStringN(node, "controlVariable", cVar, 256);

  return Rcs::TaskFactory::instance()->checkTask(cVar, node, graph);
}
