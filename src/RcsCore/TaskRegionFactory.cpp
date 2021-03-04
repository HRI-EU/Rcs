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

#include "TaskRegionFactory.h"
#include "Rcs_macros.h"
#include "Rcs_parser.h"
#include "Rcs_stlParser.h"



/*******************************************************************************
 * Returns singleton instance pointer
 ******************************************************************************/
Rcs::TaskRegionFactory* Rcs::TaskRegionFactory::instance()
{
  static Rcs::TaskRegionFactory factory;
  return &factory;
}

/*******************************************************************************
 * Private destructor
 ******************************************************************************/
Rcs::TaskRegionFactory::TaskRegionFactory()
{
}

/*******************************************************************************
 * Creates the task for className and the given graph and xml content
 ******************************************************************************/
Rcs::TaskRegion* Rcs::TaskRegionFactory::create(const Task* task, xmlNode* node)
{
  if (isXMLNodeName(node, "TaskRegion") == false)
  {
    RLOG(5, "XML node \"%s\" is not a \"TaskRegion\" - giving up",
         (char*)node->name);
    return NULL;
  }

  std::string className;
  unsigned int len = getXMLNodePropertySTLString(node, "type", className);

  if (len == 0)
  {
    RLOG(1, "TaskRegion has no type - giving up");
    return NULL;
  }

  std::map<std::string, TaskRegionCreateFunction>::iterator itCreate;
  itCreate = instance()->createFunctionMap.find(className);

  if (itCreate == instance()->createFunctionMap.end())
  {
    RLOG(1, "Unknown tsr type \"%s\" in TsrFactory::createTask",
         className.c_str());
    REXEC(4)
    {
      instance()->printRegisteredTaskRegions();
    }
    return NULL;
  }

  Rcs::TaskRegion* tsr = itCreate->second(task, node);

  return tsr;
}


/*******************************************************************************
 * This function is called through the registrat class. This happens before
 * main() is entered. Therefore, logging with debug levels doesn't make sense,
 * since the debug level has at that point not yet been parsed.
 ******************************************************************************/
void Rcs::TaskRegionFactory::registerTaskRegionFunction(std::string name,
                                                        TaskRegionCreateFunction createFunction)
{
  createFunctionMap[name] = createFunction;
}


/*******************************************************************************
 * Prints all registered task space regions to the console
 ******************************************************************************/
void Rcs::TaskRegionFactory::printRegisteredTaskRegions() const
{
  RMSG("TaskRegionFactory: registered regions are");
  std::map<std::string, TaskRegionCreateFunction>::const_iterator it;
  for (it = createFunctionMap.begin(); it != createFunctionMap.end(); ++it)
  {
    printf("%s\n", it->first.c_str());
  }

}
