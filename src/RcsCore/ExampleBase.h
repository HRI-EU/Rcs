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

#ifndef RCS_EXAMPLEBASE_H
#define RCS_EXAMPLEBASE_H

#include <Rcs_cmdLine.h>


namespace Rcs
{

class ExampleBase
{
public:

  ExampleBase(int argc, char** argv);

  virtual ~ExampleBase();

  /*  \brief Calls these functions:
   *         - initParameters()
   *         - parseArgs()
   *         - initAlgo()
   *         - initGraphics()
   *         - initGuis()
   *
   *         It is adviseable to set the resource path in the initAlgo()
   *         function, so that the init functions can be overwritten by
   *         derieved classes without any side effects.
   */
  virtual bool init(int argc, char** argv);

  /*  \brief Assign defaults to all member variables, allocate memory if needed.
   */
  virtual bool initParameters();

  /*  \brief Initializations related to algorithmic steps, no gui or graphics.
   */
  virtual bool initAlgo();

  /*  \brief Initializations for graphics windows, nodes etc.
   */
  virtual bool initGraphics();

  /*  \brief Initializations for Guis and widgets.
   */
  virtual bool initGuis();

  /*  \brief Deletes all memory and brings the global variables into a
   *         state like before the class was instantiated. In order to
   *         avoid side-effects on other classes, also the resource path
   *         should be resetted. This method should be implemented so
   *         that it can be called several times.
   */
  virtual void clear();

  /*  \brief Assign member variables according to supported command line
   *         options.
   */
  virtual bool parseArgs(CmdLineParser* parser);
  virtual void start();
  virtual void stop();
  virtual void run();
  virtual void step();
  virtual void handleKeys();
  virtual std::string help();
  virtual bool isRunning() const;

protected:
  bool runLoop;
};

}

#endif // RCS_EXAMPLEBASE_H