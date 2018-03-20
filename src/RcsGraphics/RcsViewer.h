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

#ifndef RCSVIEWER_H
#define RCSVIEWER_H

/*!
 *  \defgroup RcsGraphics RcsGraphics
 *
 * This group contains classes and methods related to the Rcs 3D visualization.
 */

#include <Rcs_graphicsUtils.h>

#include <pthread.h>

// forward declarations
typedef unsigned long Window;


namespace Rcs
{

/*!
 * \ingroup RcsGraphics
 * @{
 */
static const int ReceivesShadowTraversalMask = 0x1;
static const int CastsShadowTraversalMask = 0x2;
/*!
* @}
*/

/*!
 * \ingroup RcsGraphics
 * \brief The Viewer main class
 */
class Viewer
{
  friend class KeyHandler;
public:

  Viewer();                       // Decide shadows etc. based on X11 forwarding
  Viewer(bool fancy,              // Shadows and anti-aliasing on
         bool startupWithShadow); // Shadows on
  virtual ~Viewer();

  virtual void optimize();
  virtual bool lock() const;
  virtual bool unlock() const;

  void setWindowSize(unsigned int llx,     // lower left x
                     unsigned int lly,     // lower left y
                     unsigned int sizeX,   // size in x-direction
                     unsigned int sizeY);  // size in y-direction
  void add(osg::Node* node);
  void add(osgGA::GUIEventHandler* eventHandler);
  void removeNode(osg::Node* node);
  void runInThread(pthread_mutex_t* mutex = NULL);
  void setUpdateFrequency(double Hz);   // Default is 25Hz
  void displayWireframe(bool wf = true);
  void toggleWireframe();
  void setShadowEnabled(bool enable);
  void setCartoonEnabled(bool enabled);
  void setBackgroundColor(const char* color);
  double updateFrequency() const;
  virtual void frame();

  /*! \brief Copies the camera transformation to A_CI.
   *  \param[out] A_CI Transformation from world to camera frame
   */
  void getCameraTransform(HTr* A_CI) const;

  /*! \brief Sets the camera transformation to A_CI.
   *  \param[in] A_CI Transformation from world to camera frame
   */
  void setCameraTransform(const HTr* A_CI);

  void setCameraHomePosition(const osg::Vec3d& eye,
                             const osg::Vec3d& center,
                             const osg::Vec3d& up=osg::Vec3d(0.0, 0.0, 1.0));
  void setCameraHomePosition(const HTr* transformation);


  /*! \brief Returns the node and the 3D world position of the
   *         mouse pointer (closest intersection of picking ray and node)
   */
  osg::Node* getNodeUnderMouse(double I_mouseCoords[3]=NULL);

  /*! \brief Convenience template function for any type of node: Call it with
   *         MyNode* nd = viewer->getNodeUnderMouse<MyNode*>();
   */
  template <typename T>
  T getBodyNodeUnderMouse(double I_mouseCoords[3]=NULL)
  {
    this->lock();
    T node = Rcs::getNodeUnderMouse<T>(*this->viewer.get(), this->mouseX,
                                       this->mouseY, I_mouseCoords);
    this->unlock();
    return node;
  }


  double getFieldOfView() const;
  void setFieldOfView(double fov);

  float mouseX;
  float mouseY;
  double fps;

protected:

  static void* ViewerThread(void* arg);
  void create(bool fancy, bool startupWithShadow);
  void init();
  void stopThread();
  void setSceneData(osg::Node* node);
  bool isInitialized() const;
  bool isThreadRunning() const;
  bool isRealized() const;

  mutable pthread_mutex_t* mtxFrameUpdate;
  bool threadRunning;
  double updateFreq;
  bool initialized;
  bool wireFrame;
  bool shadowsEnabled;

  unsigned int llx, lly, sizeX, sizeY;
  pthread_t frameThread;

  osg::ref_ptr<osgViewer::Viewer> viewer;
  osg::ref_ptr<osgShadow::ShadowedScene> shadowScene;
  osg::ref_ptr<osg::LightSource> toplight;
  osg::ref_ptr<osg::Group> rootnode;
  osg::ref_ptr<osg::ClearNode> clearNode;
  osg::ref_ptr<osg::Image> _colorImage;
  osg::ref_ptr<osg::Image> _zImage;
  std::vector<osg::ref_ptr<osg::Camera> > hud;

private:
  mutable pthread_mutex_t mtxInternal;
};


}   // namespace Rcs


#endif // RCSVIEWER_H

