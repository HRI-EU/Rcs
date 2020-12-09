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

#include <algorithm>





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
  if (getenv("SSH_CLIENT") || getenv("SSH_TTY"))
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



  // msg_info->K = { 6.5746697810243404e+002, 0., 3.1950000000000000e+002,
  //                 0., 6.5746697810243404e+002, 2.3950000000000000e+002,
  //                 0., 0., 1.
  //               };
  double fx = 1.0;//6.5746697810243404e+002;
  double cx = 0.0;//3.1950000000000000e+002;
  double fy = 1.0;//6.5746697810243404e+002;
  double cy = 0.0;//2.3950000000000000e+002;
  fx = 6.5746697810243404e+002;
  cx = 3.1950000000000000e+002;
  fy = 6.5746697810243404e+002;
  cy = 2.3950000000000000e+002;

  float left0 = 0.0;
  float right0 = 640.0;
  float top0 = 480.0;
  float bottom0 = 0.0;


  float left = left0 * zNear/fx -cx;
  float right = right0 * zNear/fx -cx;
  float top = top0 * zNear/fy - cy;
  float bottom = bottom0 * zNear/fy - cy;

  // float left = zNear * -cx / fx;
  // float right = zNear * (width - cx) / fx;
  // float top = zNear * cy / fy;
  // float bottom = zNear * (cy - height) / fy;

  // glMatrixMode(GL_PROJECTION);
  // glLoadIdentity();
  // glFrustum(left, right, bottom, top, near_, far_);

  // glMatrixMode(GL_MODELVIEW);
  // glLoadIdentity();
  // gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);

  // setProjectionMatrixAsFrustum(left, right, bottom, top, zNear, zFar);







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

  // Create depth camera and add as slave to the viewer. It shares the main
  // camera's view and propjection matrices
  this->depthCam = new osg::Camera;
  depthCam->setGraphicsContext(gc.get());
  depthCam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
  depthCam->setViewport(new osg::Viewport(0, 0, width, height));
  depthCam->attach(osg::Camera::DEPTH_BUFFER, zImage.get());
  depthCam->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
  addSlave(depthCam.get());

  getCamera()->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);


  double left_0, right_0, bottom_0, top_0, zNear_0, zFar_0;
  getCamera()->getProjectionMatrixAsFrustum(left_0, right_0, bottom_0, top_0, zNear_0, zFar_0);
  RLOG(0, "left=%f right=%f top=%f bottom=%f near=%f far=%f",
       left_0, right_0, bottom_0, top_0, zNear_0, zFar_0);



  //getCamera()->setProjectionMatrixAsFrustum(left_0, right_0, bottom_0, top_0, zNear_0, zFar_0);

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

void DepthRenderer::setFrustumProjection(double left, double right, double bottom, double top, double zNear, double zFar)
{
  getCamera()->setProjectionMatrixAsFrustum(left, right, bottom, top, zNear, zFar);
}

// See http://www.songho.ca/opengl/gl_projectionmatrix.html for details
// https://stackoverflow.com/questions/22064084/how-to-create-perspective-projection-matrix-given-focal-points-and-camera-princ
// Here is the code to obtain the OpenGL projection matrix equivalent to a computer vision camera with camera matrix K=[fx, s, cx; 0, fy, cy; 0, 0, 1] and image size [W, H]:
void DepthRenderer::setProjectionFromFocalParams(double fx, double fy, double cx, double cy, double zmin, double zmax)
{
  double W = width, H = height, s = 0.0;

  osg::Matrixf pMat(2.0*fx/W, 0.0, 0.0, 0.0,
                    2.0*s/W, 2.0*fy/H, 0.0, 0.0,
                    2.0*(cx/W)-1.0, 2.0*(cy/H)-1.0, (zmax+zmin)/(zmax-zmin), 1.0,
                    0.0, 0.0, 2.0*zmax* zmin/(zmin-zmax), 0.0);

  osg::Matrixf pMat2(2.0*fx/W, 0.0, 0.0, 0.0,
                     0.0, 2.0*fy/H, 0.0, 0.0,
                     2.0*(cx/W)-1.0, 2.0*(cy/H)-1.0, -(zmax+zmin)/(zmax-zmin), -1.0,
                     0.0, 0.0,-2.0*zmax*zmin/(zmax-zmin), 0.0);




  getCamera()->setProjectionMatrix(pMat2);
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

void DepthRenderer::print() const
{
  if (depthImage.empty() || depthImage[0].empty())
  {
    printf("Empty depth image\n");
    return;
  }

  FILE* fd = fopen("depth.dat", "w+");
  RCHECK(fd);

  float minDepth = depthImage[0][0];
  float maxDepth = minDepth;

  for (size_t i=0; i<depthImage.size(); ++i)
  {
    for (size_t j=0; j<depthImage[i].size(); ++j)
    {
      //printf("%.1f ", depthImage[i][j]);
      fprintf(fd, "%f ", depthImage[i][j]);
      minDepth = std::min(minDepth, depthImage[i][j]);
      maxDepth = std::max(minDepth, depthImage[i][j]);
    }
    fprintf(fd, "\n");
    //printf("\n");
  }

  fclose(fd);
  printf("\nMin. depth: %f   max. depth: %f\n", minDepth, maxDepth);
}


}
