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

#include "DepthRenderer.h"
#include "Rcs_graphicsUtils.h"

#include <Rcs_math.h>
#include <Rcs_macros.h>

#include <osgDB/WriteFile>
#include <osgGA/TrackballManipulator>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <algorithm>



namespace Rcs
{

/*******************************************************************************
 *
 ******************************************************************************/
DepthRenderer::DepthRenderer(unsigned int width_, unsigned int height_,
                             double near, double far)
  : osgViewer::Viewer(), width(width_), height(height_)
{
  init(width, height, near, far);
}

/*******************************************************************************
 *
 ******************************************************************************/
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

  setDataVariance(osg::Object::DYNAMIC);
  setThreadingModel(osgViewer::Viewer::SingleThreaded);
  realize();

  // These are the settings from the Kinect v2
  setFrustumProjection(-0.146243, 0.145787, -0.109739, 0.109283, zNear, zFar);

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
DepthRenderer::~DepthRenderer()
{
}

/*******************************************************************************
 *
 ******************************************************************************/
void DepthRenderer::addNode(osg::Node* node)
{
  rootNode->addChild(node);
}

/*******************************************************************************
 * Removes a node from the scene graph.
 ******************************************************************************/
// size_t DepthRenderer::removeNodeInternal(std::string nodeName)
// {
//   int nnd = 0;
//   osg::Node* ndi;

//   do
//   {
//     ndi = findNamedNodeRecursive(rootNode, nodeName);
//     if (ndi)
//     {
//       //nnd += removeNodeInternal(ndi);

//       osg::Node::ParentList parents = ndi->getParents();

//       for (size_t i=0; i<parents.size(); ++i)
//       {
//         nnd++;
//         parents[i]->removeChild(ndi);
//       }


//     }

//   }
//   while (ndi);

//   RLOG(5, "Removed %d nodes with name %s from the DepthRenderer",
//        nnd, nodeName.c_str());

//   return nnd;
// }

/*******************************************************************************
 * Removes a node from the scene graph.
 ******************************************************************************/
size_t DepthRenderer::removeNode(osg::Node* node)
{
  if (node == NULL)
  {
    RLOG(1, "Node is NULL - can't be deleted");
    return 0;
  }

  osg::Node::ParentList parents = node->getParents();
  size_t nDeleted = 0;

  for (size_t i=0; i<parents.size(); ++i)
  {
    nDeleted++;
    parents[i]->removeChild(node);
  }

  return nDeleted;
}

/*******************************************************************************
 *
 ******************************************************************************/
int DepthRenderer::removeNode(const std::string& nodeName)
{
  int nnd = 0;
  osg::Node* ndi;

  do
  {
    ndi = findNamedNodeRecursive(rootNode, nodeName);

    if (ndi)
    {
      nnd += removeNode(ndi);
    }

  }
  while (ndi);

  return nnd;
}

/*******************************************************************************
 *
 ******************************************************************************/
void DepthRenderer::setCameraTransform(const HTr* A_CI)
{
  osg::Matrix vm = Rcs::viewMatrixFromHTr(A_CI);
  getCameraManipulator()->setByInverseMatrix(vm);
}

/*******************************************************************************
 *
 ******************************************************************************/
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

/*******************************************************************************
 *
 ******************************************************************************/
void DepthRenderer::getFrustumProjection(double& left, double& right,
                                         double& bottom, double& top,
                                         double& zNear, double& zFar) const
{
  getCamera()->getProjectionMatrixAsFrustum(left, right, bottom, top, zNear, zFar);
}

/*******************************************************************************
 *
 ******************************************************************************/
void DepthRenderer::getNearFar(double& zNear, double& zFar) const
{
  double left, right, bottom, top;
  getFrustumProjection(left, right, bottom, top, zNear, zFar);
}

/*******************************************************************************
 *
 ******************************************************************************/
void DepthRenderer::setFrustumProjection(double left, double right,
                                         double bottom, double top,
                                         double zNear, double zFar)
{
  getCamera()->setProjectionMatrixAsFrustum(left, right, bottom, top,
                                            zNear, zFar);
}

/*******************************************************************************
 * See for details:
 *  http://www.songho.ca/opengl/gl_projectionmatrix.html for details
 *  https://stackoverflow.com/questions/22064084/how-to-create-perspective-
 *          projection-matrix-given-focal-points-and-camera-princ
 ******************************************************************************/
void DepthRenderer::setProjectionFromFocalParams(double fx, double fy,
                                                 double cx, double cy,
                                                 double zmin, double zmax)
{
  double w = width, h = height;

  osg::Matrixf pMat(2.0*fx/w, 0.0, 0.0, 0.0,
                    0.0, 2.0*fy/h, 0.0, 0.0,
                    2.0*(cx/w)-1.0, 2.0*(cy/h)-1.0, -(zmax+zmin)/(zmax-zmin), -1.0,
                    0.0, 0.0,-2.0*zmax*zmin/(zmax-zmin), 0.0);

  getCamera()->setProjectionMatrix(pMat);
}

/*******************************************************************************
 *
 ******************************************************************************/
void DepthRenderer::frame(double simulationTime)
{
  // Perform the actual rendering
  osgViewer::Viewer::frame(simulationTime);

  // Get transformation matrix from world to screen and invert it
  osg::Matrixd pw = depthCam->getProjectionMatrix() *
                    depthCam->getViewport()->computeWindowMatrix();

  osg::Matrixd inverse_pw;
  inverse_pw.invert(pw);

  depthImage.resize(height);
  for (unsigned int i = 0; i < height; ++i)
  {
    depthImage[i].resize(width);
  }

  const unsigned int n = width * height;
  const float* zData = ((float*) zImage->data());

  double zNear, zFar;
  getNearFar(zNear, zFar);

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

    depthImage[screen_y][screen_x] = -world_coord[2];
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
const std::vector<std::vector<float>>& DepthRenderer::getDepthImageRef() const
{
  return this->depthImage;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool DepthRenderer::print(const std::string& fileName) const
{
  FILE* fd = fopen(fileName.c_str(), "w+");

  if (!fd)
  {
    RLOG_CPP(1, "Failed to open file " << fileName << " for writing");
    return false;
  }

  bool success = print(fd);
  fclose(fd);

  return success;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool DepthRenderer::print(FILE* fd) const
{
  if (depthImage.empty() || depthImage[0].empty())
  {
    RLOG(1, "Did not write empty depth image");
    return false;
  }

  for (size_t i=0; i<depthImage.size(); ++i)
  {
    for (size_t j=0; j<depthImage[i].size(); ++j)
    {
      fprintf(fd, "%f ", depthImage[i][j]);
    }

    fprintf(fd, "\n");
  }

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
void DepthRenderer::getMinMaxDepth(double& minDepth_, double& maxDepth_) const
{
  if (depthImage.empty() || depthImage[0].empty())
  {
    RLOG_CPP(1, "Empty depth image, setting min and max depth to 0");
    minDepth_ = 0.0;
    maxDepth_ = 0.0;
    return;
  }

  float minDepth = depthImage[0][0];
  float maxDepth = minDepth;

  for (size_t i=0; i<depthImage.size(); ++i)
  {
    for (size_t j=0; j<depthImage[i].size(); ++j)
    {
      minDepth = std::min(minDepth, depthImage[i][j]);
      maxDepth = std::max(minDepth, depthImage[i][j]);
    }
  }

  minDepth_ = minDepth;
  maxDepth_ = maxDepth;
}


}
