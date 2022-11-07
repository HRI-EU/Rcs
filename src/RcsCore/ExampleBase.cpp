/*******************************************************************************

  Copyright (c) 2022, Honda Research Institute Europe GmbH

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

#include "ExampleFactory.h"
#include "Rcs_cmdLine.h"
#include "Rcs_macros.h"


namespace Rcs
{

//static ExampleFactoryRegistrar<ExampleBase> ExampleBase_("RcsCore", "ExampleBase");

ExampleBase::ExampleBase(int argc, char** argv) : runLoop(false)
{
}

ExampleBase::~ExampleBase()
{
}

void ExampleBase::init(int argc, char** argv)
{
  initParameters();
  CmdLineParser argP(argc, argv);
  parseArgs(&argP);
  bool success = initAlgo();
  initGraphics();
  initGuis();
}

void ExampleBase::clear()
{
}
void ExampleBase::initParameters()
{
}

void ExampleBase::parseArgs(CmdLineParser* parser)
{
}

bool ExampleBase::initAlgo()
{
  return true;
}

void ExampleBase::initGraphics()
{
}

void ExampleBase::initGuis()
{
}

void ExampleBase::start()
{
  runLoop = true;
  run();
}

void ExampleBase::stop()
{
  runLoop = false;
}

void ExampleBase::run()
{
  while (runLoop)
  {
    handleKeys();
    step();
  }
}

void ExampleBase::step()
{
}

void ExampleBase::handleKeys()
{
}

std::string ExampleBase::help()
{
  return "No help message implemented";
}

bool ExampleBase::isRunning() const
{
  return runLoop;
}

}   // namespace Rcs
