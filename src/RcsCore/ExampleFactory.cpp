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

#include "ExampleFactory.h"

#include <Rcs_macros.h>
#include <Rcs_parser.h>
#include <Rcs_stlParser.h>
#include <Rcs_utilsCPP.h>



namespace Rcs
{

/*******************************************************************************
 *
 ******************************************************************************/
ExampleFactory::ExampleFactory()
{
}

/*******************************************************************************
 * Print all registered examples to the console
 ******************************************************************************/
void ExampleFactory::print()
{
  auto it = constructorMap().begin();

  std::cout << constructorMap().size() << " registered examples:\n";

  while (it != constructorMap().end())
  {
    std::vector<std::string> strings = String_split(it->first, "_");
    RCHECK_MSG(strings.size()==2, "Splitted into %zu strings: %s",
               strings.size(), it->first.c_str());

    std::cout << "  " << strings[1] << std::endl;
    it++;
  }

  std::set<std::string> categories = getCategories();
  std::cout << categories.size() << " registered categories:\n";


  std::set<std::string>::iterator cit = categories.begin();
  // Iterate till the end of set
  while (cit != categories.end())
  {
    std::cout << "  " << (*cit) << std::endl;
    cit++;
  }

}

/*******************************************************************************
 * Return a vector with all registered categories
 ******************************************************************************/
std::set<std::string> ExampleFactory::getCategories()
{
  std::set<std::string> categories;

  auto it = constructorMap().begin();
  while (it != constructorMap().end())
  {
    std::vector<std::string> strings = String_split(it->first, "_");
    RCHECK_MSG(strings.size()==2, "Splitted into %zu strings: %s",
               strings.size(), it->first.c_str());

    categories.insert(strings[0]);
    it++;
  }

  return categories;
}

/*******************************************************************************
 * Creates the example for the given className
 ******************************************************************************/
ExampleBase* ExampleFactory::create(std::string category,
                                    std::string example,
                                    int argc,
                                    char** argv)
{
  std::string name = category + "_" + example;
  ExampleBase* newExample = NULL;
  std::map<std::string, ExampleMaker>::iterator it;
  it = constructorMap().find(name);

  if (it != constructorMap().end())
  {
    newExample = it->second(argc, argv);
  }
  else
  {
    RLOG_CPP(1, "Couldn't find constructor for class " << name);
  }

  return newExample;
}

/*******************************************************************************
 * Checks if an example with the given name has been registered
 ******************************************************************************/
bool ExampleFactory::hasExample(std::string category, std::string example)
{
  std::string name = category + "_" + example;
  return (constructorMap().find(name) != constructorMap().end());
}

/*******************************************************************************
 * This function is called through the registrar class. This happens before
 * main() is entered. Therefore, logging with debug levels doesn't make sense,
 * since the debug level has at that point not yet been parsed.
 ******************************************************************************/
void ExampleFactory::registerExample(std::string category,
                                     std::string example,
                                     ExampleMaker createFunction)
{
  std::string name = category + "_" + example;
  std::map<std::string, ExampleMaker>::iterator it;
  it = constructorMap().find(name);
  if (it != constructorMap().end())
  {
    // No log level, this happens before main()
    RMSG("Overwriting a constraint creation function: \"%s\"", name.c_str());
  }

  constructorMap()[name] = createFunction;
}

/*******************************************************************************
 *
 ******************************************************************************/
std::map<std::string, ExampleFactory::ExampleMaker>&
ExampleFactory::constructorMap()
{
  static std::map<std::string, ExampleMaker> em;
  return em;
}



}   // namespace Rcs
