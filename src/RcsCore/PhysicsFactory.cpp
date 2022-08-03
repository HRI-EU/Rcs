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

#include "PhysicsFactory.h"

#include <Rcs_macros.h>

#include <map>




static std::map<std::string, Rcs::PhysicsFactory::PhysicsCreateFunction>& constructorMap()
{
  static std::map<std::string, Rcs::PhysicsFactory::PhysicsCreateFunction> cm;
  return cm;
}

/*******************************************************************************
 * Singleton class has private constructor
 ******************************************************************************/
Rcs::PhysicsFactory::PhysicsFactory()
{
}

/*******************************************************************************
 * Creates the physics engine for className and the given graph and xml content
 ******************************************************************************/
Rcs::PhysicsBase* Rcs::PhysicsFactory::create(const char* className,
                                              const RcsGraph* graph,
                                              const char* cfgFile)
{
  Rcs::PhysicsBase* sim = NULL;
  std::map<std::string, PhysicsCreateFunction>::iterator it;

  it = constructorMap().find(className);

  if (it != constructorMap().end())
  {
    PhysicsConfig config(cfgFile);
    sim = it->second(className, graph, &config);
  }
  else
  {
    REXEC(1)
    {
      RMSG("Couldn't instantiate physics engine \"%s\"", className);
      RMSG("Options are:");
      print();
    }
  }

  return sim;
}

/*******************************************************************************
 * Creates the physics engine for className and the given graph and config obj
 ******************************************************************************/
Rcs::PhysicsBase* Rcs::PhysicsFactory::create(const char* className,
                                              const RcsGraph* graph,
                                              const PhysicsConfig* config)
{
  Rcs::PhysicsBase* sim = NULL;
  std::map<std::string, PhysicsCreateFunction>::iterator it;

  it = constructorMap().find(className);

  if (it != constructorMap().end())
  {
    sim = it->second(className, graph, config);
  }
  else
  {
    REXEC(1)
    {
      RMSG("Couldn't instantiate physics engine \"%s\"", className);
      RMSG("Options are:");
      print();
    }
  }

  return sim;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::PhysicsFactory::hasEngine(const char* className)
{
  std::map<std::string, PhysicsCreateFunction>::iterator it;

  it = constructorMap().find(className);

  return (it==constructorMap().end()) ? false : true;
}

/*******************************************************************************
 * This function is called through the registrar class. This happens before
 * main() is entered. Therefore, logging with debug levels doesn't make sense,
 * since the debug level has at that point not yet been parsed.
 ******************************************************************************/
void Rcs::PhysicsFactory::registerPhysics(const char* name,
                                          PhysicsCreateFunction createFunction)
{
  constructorMap()[name] = createFunction;
}

/*******************************************************************************
 * Prints all registered physics engines to the console
 ******************************************************************************/
void Rcs::PhysicsFactory::print()
{
  std::cout << printToString();
}

/*******************************************************************************
 * Prints all registered physics engines to the console
 ******************************************************************************/
std::string Rcs::PhysicsFactory::printToString()
{
  if (constructorMap().empty())
  {
    return std::string("No physics engines found");
  }

  std::string res = "Registered physics engines are:\n";

  std::map<std::string, PhysicsCreateFunction>::const_iterator it;

  for (it = constructorMap().begin(); it != constructorMap().end(); ++it)
  {
    res += '\t';
    res += it->first;
    res += '\n';
  }

  res += '\n';

  return res;
}
