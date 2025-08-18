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

#ifndef RCS_DEPTHRENDERER_H
#define RCS_DEPTHRENDERER_H

#include <Rcs_HTr.h>

#include <osgViewer/Viewer>

#include <vector>
#include <mutex>



/**
 * \ingroup RcsGraphics
 */
namespace Rcs
{

/*!
 * \brief Class for rendering a depth image from an OpenSceneGraph scene. The
 *        class is rendering an OpenSceneGraph scene into a depth image. The
 *        methods are not thread safe, the user must make sure that there are
 *        no concurrent accesses, such as for instance adding a node while the
 *        frame() call is running. There are several convenience functions for
 *        setting the camera projection matrices. The default is set to the
 *        Kinect v2 settings.
 */
class DepthRenderer : public osgViewer::Viewer
{
public:

  /*! \brief Constructor
   *
   * \param[in] width Width of the depth image in pixels
   * \param[in] height Height of the depth image in pixels
   * \param[in] near Camera distance to near plane in meters
   * \param[in] far Camera distance to far plane in meters
   */
  DepthRenderer(unsigned int width=640, unsigned int height=480,
                double near=0.1, double far=10.0);

  /*!
   * \brief Destructor to support polymorphism. Does nothing.
   */
  virtual ~DepthRenderer();

  /*! \brief Adds a OpenSceneGraph node to the scene. It will be rendered into
   *         the depth images. This function is thread-safe.
   *
   *  \param node Node to add
   */
  void addNode(osg::Node* node);
  int removeNode(const std::string& nodeName);
  size_t removeNode(osg::Node* node);

  void setCameraTransform(const HTr* A_CI);

  void setFieldOfView(double fovWidth, double fovHeight);
  void getFrustumProjection(double& left, double& right, double& bottom,
                            double& top, double& zNear, double& zFar) const;
  void getNearFar(double& zNear, double& zFar) const;
  void setFrustumProjection(double left, double right, double bottom,
                            double top, double zNear, double zFar);
  void setProjectionFromFocalParams(double fx, double fy, double cx, double cy,
                                    double zmin, double zmax);
  void getFocalParams(double& fx, double& fy, double& cx, double& cy) const;

  /*! \brief Renders the scene into a depth image on each frame call.
   */
  virtual void frame(double simulationTime=USE_REFERENCE_TIME);

  bool getColorImage(uint8_t* data, size_t size) const;
  bool getColorImage(double* data, size_t size) const;
  //const uint8_t* getColorImagePtr() const;

  bool getDepthImage(float* data, size_t size) const;
  bool getDepthImage(double* data, size_t size) const;
  //const float* getDepthImagePtr() const;

  size_t getWidth() const;
  size_t getHeight() const;

  void getMinMaxDepth(double& minDepth, double& maxDepth) const;

private:

  template<typename T>
  bool getDepthImage(T* data, size_t size) const;

  osg::ref_ptr<osg::Group> rootNode;
  osg::ref_ptr<osg::Image> zImage;
  osg::ref_ptr<osg::Image> rgbImage;
  osg::ref_ptr<osg::Camera> depthCam;
  osg::ref_ptr<osg::Camera> rgbCam;
  mutable std::mutex captureMtx;
  unsigned int width;
  unsigned int height;
};

}

#endif   // RCS_DEPTHRENDERER_H
