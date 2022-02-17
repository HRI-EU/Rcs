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

#ifndef RCS_TASKREGIONFACTORY_H
#define RCS_TASKREGIONFACTORY_H

#include "TaskRegion.h"

#include <map>


// Convenience macro to register TaskRegions in the factory. Here is an
// example: REGISTER_TASKREGION(TaskRegion, "ABC") expands to
// static Rcs::TaskRegionFactoryRegistrar<TaskRegion> TaskRegion_("ABC")
#define REGISTER_TASKREGION(T,N) static Rcs::TaskRegionFactoryRegistrar<T> T ## _(N)


namespace Rcs
{

class TaskRegionFactory
{
public:

  typedef TaskRegion* (*TaskRegionCreateFunction)(const Task* task, xmlNode* node);
  static TaskRegionFactory* instance();

  static TaskRegion* create(const Task* task, xmlNode* node);
  void registerTaskRegionFunction(std::string name, TaskRegionCreateFunction createFunction);
  void printRegisteredTaskRegions() const;

private:

  TaskRegionFactory();

  std::map<std::string, TaskRegionCreateFunction> createFunctionMap;
};





template<class T>
class TaskRegionFactoryRegistrar
{
public:

  TaskRegionFactoryRegistrar(std::string className)
  {
    TaskRegionFactory* tf = TaskRegionFactory::instance();
    tf->registerTaskRegionFunction(className, &TaskRegionFactoryRegistrar::create);
  }

  static TaskRegion* create(const Task* task, xmlNode* node)
  {
    return new T(task, node);
  }

};

}

#endif // RCS_TASKREGIONFACTORY_H
