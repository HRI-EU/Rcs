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

#include "DepthRenderer.h"
#include "Rcs_graphicsUtils.h"

#include <Rcs_math.h>
#include <Rcs_macros.h>

#include <osgDB/WriteFile>
#include <osgGA/TrackballManipulator>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>







// Change viewer to camera perspective
// der FoV der Kinect2 ist circa:
// RGB: 84 x 54 deg (1920 x 1080)
// IR/D: 71 x 60deg  (512 x 424)


namespace Rcs
{

DepthRenderer::DepthRenderer(unsigned int width_, unsigned int height_)
  : osgViewer::Viewer(),
    width(width_), height(height_), zNear(0.001), zFar(10.0),
    fieldOfView(71.0), aspectRatio(71.0/60.0)
{
  init(width, height, zNear, zFar);
}

bool DepthRenderer::init(unsigned int width, unsigned int height,
                         double zNear, double zFar)
{
  // check if executed remotely, rendering to pixel buffer not working for
  // ssh connections
  bool remote_connection = (getenv("SSH_CLIENT") != NULL) ||
                           (getenv("SSH_TTY") != NULL);
  if (remote_connection)
  {
    RFATAL("DepthRenderer does not work via SSH");
  }

  // Initialize viewer
  setCameraManipulator(new osgGA::TrackballManipulator());
  this->rootNode = new osg::Group;
  setSceneData(rootNode.get());

  // Create image for holding the depth values
  this->zImage = new osg::Image;
  zImage->allocateImage(width, height, 1, GL_DEPTH_COMPONENT, GL_FLOAT);

  // Create graphics context with pixel settings
  osg::ref_ptr<osg::GraphicsContext::Traits> traits;
  traits = new osg::GraphicsContext::Traits;
  traits->x = 0;
  traits->y = 0;
  traits->width = width;
  traits->height = height;
  traits->windowDecoration = false;
  traits->doubleBuffer = false;
  traits->sharedContext = 0;
  traits->pbuffer = true;

  osg::ref_ptr<osg::GraphicsContext> gc;
  gc = osg::GraphicsContext::createGraphicsContext(traits.get());

  // Create depth camera and add as slave to the viewer
  this->depthCam = new osg::Camera;
  depthCam->setGraphicsContext(gc.get());
  depthCam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
  depthCam->setViewport(new osg::Viewport(0, 0, width, height));
  depthCam->attach(osg::Camera::DEPTH_BUFFER, zImage.get());
  depthCam->setProjectionMatrixAsPerspective(fieldOfView, aspectRatio,
                                             zNear, zFar);
  addSlave(depthCam.get());

  // Apply viewer settings
  setDataVariance(osg::Object::DYNAMIC);
  setThreadingModel(osgViewer::Viewer::SingleThreaded);
  realize();

  return true;
}

DepthRenderer::~DepthRenderer()
{
}

void DepthRenderer::addNode(osg::Node* node)
{
  rootNode->addChild(node);
}

void DepthRenderer::setCameraTransform(const HTr* A_CI)
{
  osg::Matrix vm = Rcs::viewMatrixFromHTr(A_CI);
  getCameraManipulator()->setByInverseMatrix(vm);
}

void DepthRenderer::setFieldOfView(double fovWidth, double fovHeight)
{
  double fovyOld, aspectRatio, zNear, zFar;
  fovWidth = RCS_RAD2DEG(fovWidth);
  fovHeight = RCS_RAD2DEG(fovHeight);
  getCamera()->getProjectionMatrixAsPerspective(fovyOld, aspectRatio,
                                                zNear, zFar);
  getCamera()->setProjectionMatrixAsPerspective(fovWidth,
                                                fovWidth/fovHeight,
                                                zNear, zFar);
}

void DepthRenderer::frame(double simulationTime)
{
  osgViewer::Viewer::frame(simulationTime);

  // get transformation matrix from world to screen and invert it
  osg::Matrixd pw = depthCam->getProjectionMatrix() *
                    depthCam->getViewport()->computeWindowMatrix();

  osg::Matrixd inverse_pw;
  inverse_pw.invert(pw);

  //MatNd* mat = MatNd_create(height, width);

  depthImage.resize(height);
  for (unsigned int i = 0; i < height; ++i)
  {
    depthImage[i].resize(width);
  }

  const unsigned int n = width * height;
  const float* zData = ((float*) zImage->data());

  for (unsigned int i=0; i<n; ++i)
  {
    const float data = zData[i];

    // screen to world coordinate (but we respect that the point cloud
    // y-direction is downward while in OpenGL y points upward)
    // also the correct point index is calculated this way
    const unsigned int screen_x = i % width;
    const unsigned int screen_y = height - 1 - (i / width);

    osg::Vec3d screen_coord(screen_x, screen_y, data);
    osg::Vec3d world_coord = screen_coord*inverse_pw;
    if (data>=1.0)
    {
      world_coord[2] = -zFar;
    }

    //MatNd_set(mat, screen_y, screen_x, -world_coord[2]);
    depthImage[screen_y][screen_x] = -world_coord[2];
  }

  //MatNd_destroy(mat);
}

const std::vector<std::vector<float>>& DepthRenderer::getDepthImageRef() const
{
  return this->depthImage;
}


}
