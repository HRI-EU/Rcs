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

#ifndef RCS_DEPTHRENDERER_H
#define RCS_DEPTHRENDERER_H

#include <Rcs_HTr.h>

#include <osgViewer/Viewer>

#include <vector>



/**
 * \ingroup RcsGraphics
 */
namespace Rcs
{

/*!
 * \brief Class for rendering a depth image from an OpenSceneGraph scene.
 */
class DepthRenderer : public osgViewer::Viewer
{
public:

  /*! \brief Constructor
   *
   * \param[in] width Width of the depth image in pixels
   * \param[in] height Height of the depth image in pixels
   */
  DepthRenderer(unsigned int width, unsigned int height);

  /*!
   * \brief Destructor to support polymorphism. Does nothing.
   */
  virtual ~DepthRenderer();

  /*! \brief Adds a OpenSceneGraph node to the scene
   *
   *  \param node Node to add
   */
  void addNode(osg::Node* node);

  void setCameraTransform(const HTr* A_CI);

  void setFieldOfView(double fovWidth, double fovHeight);
  void setFrustumProjection(double left, double right, double bottom, double top, double zNear, double zFar);
  void setProjectionFromFocalParams(double fx, double fy, double cx, double cy, double zmin, double zmax);

  /*! \brief Renders the scene into a depth image on each frame call.
   */
  virtual void frame(double simulationTime=USE_REFERENCE_TIME);

  const std::vector<std::vector<float>>& getDepthImageRef() const;

  void print() const;

private:

  bool init(unsigned int width, unsigned int height, double zNear, double zFar);

  osg::ref_ptr<osg::Group> rootNode;
  osg::ref_ptr<osg::Image> zImage;
  osg::ref_ptr<osg::Camera> depthCam;
  unsigned int width;
  unsigned int height;
  double zNear;
  double zFar;
  double fieldOfView;
  double aspectRatio;
  std::vector<std::vector<float>> depthImage;
};

}

#endif   // RCS_DEPTHRENDERER_H
