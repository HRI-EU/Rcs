/*******************************************************************************

  Copyright (c) Honda Research Institute Europe GmbH

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

#include "ExampleGraphics.h"

#include <ExampleFactory.h>
#include <Rcs_resourcePath.h>
#include <Rcs_cmdLine.h>
#include <Rcs_macros.h>
#include <Rcs_typedef.h>
#include <Rcs_body.h>
#include <Rcs_shape.h>
#include <Rcs_math.h>
#include <Rcs_timer.h>
#include <Rcs_utils.h>
#include <Rcs_utilsCPP.h>

#include <RcsViewer.h>
#include <COSNode.h>
#include <ArrowNode.h>
#include <SphereNode.h>
#include <VertexArrayNode.h>
#include <GraphNode.h>
#include <KeyCatcher.h>
#include <MeshNode.h>
#include <BoxNode.h>
#include <TextNode3D.h>
#include <Rcs_graphicsUtils.h>
#include <MatNdWidget.h>



/*******************************************************************************
 * Depth rendering test
 ******************************************************************************/
// The below comment is for astyle, since otherwise it screws up the formatting
 // *INDENT-OFF*
#define MULTI_LINE_STRING(a) #a
const char* depthExampleGraph =
MULTI_LINE_STRING(
    <Graph>
    <Body name = "Box" rigid_body_joints = "3 0 0 0 0 0" color = "GREEN" >
    <Shape type = "BOX" extents = "1 1 1" transform = "0.5 0 0 0 0 0"
    graphics = "true" />
    </Body>
    </Graph>
);
// *INDENT-ON*




namespace Rcs
{

RCS_REGISTER_EXAMPLE(ExampleGraphics, "Graphics", "Depth rendering");


ExampleGraphics::ExampleGraphics(int argc, char** argv) : ExampleBase(argc, argv)
{
  graph = NULL;
  viewer = NULL;
  pixelGui = NULL;
  width = 0;
  height = 0;
}

ExampleGraphics::~ExampleGraphics()
{
  delete pixelGui;
  delete viewer;
  RcsGraph_destroy(graph);
}

bool ExampleGraphics::initParameters()
{
  width = 640;
  height = 480;
  directory = "config/xml/Examples";

  return true;
}

bool ExampleGraphics::parseArgs(CmdLineParser* argP)
{
  argP->getArgument("-f", &xmlFileName, "Configuration file name "
                    "(default is %s)", xmlFileName.c_str());
  argP->getArgument("-dir", &directory, "Configuration file directory "
                    "(default is %s)", directory.c_str());
  argP->getArgument("-width", &width, "Image width (default is %d)", width);
  argP->getArgument("-height", &height, "Image height (default is %d)", height);

  if (xmlFileName.empty())
  {
    xmlFileName = depthExampleGraph;
  }

  if (argP->hasArgument("-h"))
  {
    return false;
  }

  return true;
}

bool ExampleGraphics::initGraphics()
{
  this->graph = RcsGraph_create(xmlFileName.c_str());
  if (!graph)
  {
    RLOG(1, "Graph %s could not be created", xmlFileName.c_str());
    return false;
  }

  this->viewer = new Rcs::Viewer();
  if (!viewer)
  {
    RLOG(1, "Viewer could not be created");
    return false;
  }
  viewer->setCameraTransform(HTr_identity());

  osg::ref_ptr<Rcs::GraphNode> gn = new Rcs::GraphNode(graph);
  viewer->add(gn.get());

  kc = new Rcs::KeyCatcher();
  viewer->add(kc);

  hud = new Rcs::HUD();
  viewer->add(hud);

  zRenderer = new Rcs::DepthRenderer(width, height);
  zRenderer->setCameraTransform(HTr_identity());
  //zRenderer->addNode(gn.get());
  zRenderer->addNode(viewer->getNode("rootnode"));

  return true;
}

void ExampleGraphics::run()
{

  double* zData = new double[width * height];
  double* cData = new double[width * height * 3];
  std::vector<Rcs::PPSGui::Entry> pps;
  pps.push_back(Rcs::PPSGui::Entry("Depth image", width, height, zData, 1, 0.1));
  pps.push_back(Rcs::PPSGui::Entry("RGB image", width, height, cData, 3, 1.0));
  pixelGui = new Rcs::PixelGui(pps);
  //const std::vector<std::vector<float>>& zImage = zRenderer->getDepthImageRef();
  //const std::vector<std::vector<std::vector<float>>>& rgbImage = zRenderer->getRGBImageRef();
  //const uint8_t* rgbImage = zRenderer->getColorImagePtr();

  //new Rcs::MatNdGui(graph->q, -10.0, 10.0, "q");

  // These come from a Kinect v2 calbration
  //const int windowId = 0;
  double fx = 6.5746697810243404e+002;
  double cx = 3.1950000000000000e+002;
  double fy = 6.5746697810243404e+002;
  double cy = 2.3950000000000000e+002;
  double near = 0.3;
  double far = 10.0;
  zRenderer->setProjectionFromFocalParams(fx, fy, cx, cy, near, far);
  //Rcs::HighGui::showSlider("fx", windowId, 0.0, 1000.0, 10.0, &fx);
  //Rcs::HighGui::showSlider("fy", windowId, 0.0, 1000.0, 10.0, &fy);
  //Rcs::HighGui::showSlider("cx", windowId, 0.0, 1000.0, 10.0, &cx);
  //Rcs::HighGui::showSlider("cy", windowId, 0.0, 1000.0, 10.0, &cy);
  //Rcs::HighGui::showSlider("near", windowId, 0.01, 20.0, 0.01, &near);
  //Rcs::HighGui::showSlider("far", windowId, 0.01, 100.0, 0.01, &far);



  while (runLoop)
  {
    RcsGraph_setState(graph, NULL, NULL);
    viewer->frame();
    HTr camTrf;
    viewer->getCameraTransform(&camTrf);
    zRenderer->setCameraTransform(&camTrf);
    zRenderer->setProjectionFromFocalParams(fx, fy, cx, cy, near, far);
    double t_render = Timer_getSystemTime();
    zRenderer->frame();
    t_render = Timer_getSystemTime() - t_render;

    char hudText[256];
    snprintf(hudText, 255, "Rendering took %.1f msec", 1000.0 * t_render);

    hud->setText(hudText);

    zRenderer->getColorImage(cData, width * height * 3);
    zRenderer->getDepthImage(zData, width * height);

    Timer_waitDT(0.1);
  }

}

void ExampleGraphics::handleKeys()
{
  if (!kc)
  {
    return;
  }

  //////////////////////////////////////////////////////////////
  // Keycatcher
  /////////////////////////////////////////////////////////////////
  if (kc->getAndResetKey('q'))
  {
    RMSGS("Quitting run loop");
    runLoop = false;
  }

}


}   // namespace Rcs
