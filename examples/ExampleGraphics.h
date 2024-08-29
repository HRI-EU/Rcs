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

#ifndef RCS_EXAMPLEGRAPHICS_H
#define RCS_EXAMPLEGRAPHICS_H

#include <ExampleBase.h>
#include <Rcs_graph.h>

#include <RcsViewer.h>
#include <HUD.h>
#include <KeyCatcher.h>
#include <DepthRenderer.h>
#include <PPSGui.h>


namespace Rcs
{

class ExampleGraphics : public ExampleBase
{
public:
  ExampleGraphics(int argc, char** argv);
  virtual ~ExampleGraphics();
  virtual bool parseArgs(CmdLineParser* parser);
  virtual bool initParameters();
  virtual bool initGraphics();
  virtual void run();
  virtual void handleKeys();

protected:
  RcsGraph* graph;
  Rcs::Viewer* viewer;
  Rcs::PixelGui* pixelGui;
  osg::ref_ptr<Rcs::DepthRenderer> zRenderer;
  osg::ref_ptr<HUD> hud;
  osg::ref_ptr<KeyCatcher> kc;

  std::string xmlFileName, directory;
  int width, height;


private:

  // Disallow copying and assigning to avoid double free errors.
  ExampleGraphics& operator=(const ExampleGraphics&);
  ExampleGraphics(const ExampleGraphics&);
};


}   // namespace

#endif   // RCS_EXAMPLEGRAPHICS_H
