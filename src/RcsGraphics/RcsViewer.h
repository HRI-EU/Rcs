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
 *  \defgroup RcsGraphics Graphics and visualization classes
 *
 *  Classes and methods related to the Rcs 3D visualization.
 */

#include <Rcs_graphicsUtils.h>

#include <pthread.h>

// forward declarations
typedef unsigned long Window;


namespace Rcs
{

class KeyHandler;

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
 * \brief The Viewer main class. It is based on the osg::Viewer.
 *
 *        A number of keys are associatde with a function if pressed in the
 *        viewer window:
 *        - Pressing keys 0 - 9 in the viewer window will set the Rcs log
 *          level to the corresponding level
 *        - Pressing F12 will print out to the console all keys that have been
 *          registered with a function.
 *        - Pressing key w will toggle between solid and wireframe display
 *        - Pressing key s will toggle shadows
 *        - Key F10 will toggle full screen mode
 *        - Key F8 will take a screenshot
 *        - Key F9 will start / stop taking screenshort in each frame
 *        - Key R will toggle the cartoon mode
 */
class Viewer
{
  friend class KeyHandler;
public:

  /*! \brief Default constructor with default window dimensions. Shadows and
   *         anti-aliasing settings are selected by checking if the viewer
   *         runs through X11 forwarding, or on a local machine. The constructor
   *         does not launch a window. This needs to be done separately either
   *         through calling \ref frame() or \ref runInThread().
   */
  Viewer();

  Viewer(bool fancy,              // Shadows and anti-aliasing on
         bool startupWithShadow); // Shadows on

  /*! \brief Virtual destructor to allow polymorphism.
   */
  virtual ~Viewer();

  /*! \brief Applies OpenSceneGraph's optimization function to the viewer's
   *         root node.
   */
  virtual void optimize();

  /*! \brief Locks all mutexes around the frame() call.
   */
  virtual bool lock() const;

  /*! \brief Unlocks all mutexes around the frame() call.
   */
  virtual bool unlock() const;

  /*! \brief Changes the window size. This function only takes effect if there
   *         was no frame() call before.
   *
   *  \param[in] llx     Lower left x screen coordinate
   *  \param[in] lly     Lower left y screen coordinate
   *  \param[in] sizeX   Screen size in x-direction
   *  \param[in] sizeY   Screen size in y-direction
   *  \return True for success, false otherwise: View has already been set
   *          up (frame() has been called). In case of failure, there will be a
   *          log message on debug level 1.
   */
  bool setWindowSize(unsigned int llx,     // lower left x
                     unsigned int lly,     // lower left y
                     unsigned int sizeX,   // size in x-direction
                     unsigned int sizeY);  // size in y-direction

  /*! \brief Adds the osg::Node to the root node of the viewer's scene graph.
   */
  bool add(osg::Node* node);

  /*! \brief Adds the event handler to the root node of the viewer's scene
   *         graph.
   */
  void add(osgGA::GUIEventHandler* eventHandler);

  /*! \brief Removes the given node from the viewer's scene graph.
   *
   * \return True for success, false otherwise: node is NULL, or not found in
   *         the viewer's scenegraph. The scenegraph is searched through all
   *         levels.
   */
  bool removeNode(osg::Node* node);

  /*! \brief Removes all osg::Node from the viewer's root node. The frame mutex
   *         is internally set around the scenegraph modification so that no
   *         threading issues will occur.
   *
   * \return Number of nodes removed.
   */
  unsigned int removeNodes();

  /*! \brief Starts a thread that periodically calls the frame() call. The
   *         thread will try to achieve the given updateFrequency. Changing the
   *         update frequency will take effect also when the thread is running.
   *
   * \param[in] mutex   Optional mutex that will be locked whenever the viewer
   *                    traverses or does changes to the scene graph.
   */
  void runInThread(pthread_mutex_t* mutex = NULL);

  /*! \brief Sets the viewer's update frequency. This only has an effect if the
   *         viewer runs its own thread (see runInThread() method). The default
   *         is 25Hz.
   *
   *  \param[in] Hz  Update frequency in [Hz]
   */
  void setUpdateFrequency(double Hz);

  /*! \brief Set wire frame mode.
   *
   *  \param[in] wf  True for wire frame display, false for solid.
   */
  void displayWireframe(bool wf = true);

  /*! \brief Toggles the wire frame mode.
   */
  void toggleWireframe();

  /*! \brief Enable or disable shadow casting.
   *
   *  \param[in] enable  True for shadow casting, false for no shadows.
   */
  void setShadowEnabled(bool enable);

  /*! \brief Enable or disable cartoon mode.
   *
   *  \param[in] enabled  True for cartoon mode, false otherwise.
   */
  void setCartoonEnabled(bool enabled);

  /*! \brief Sets the viewer's background color. See colorFromString() for
   *         colors.
   *
   *  \param[in] color   Color to be set as background color. if color is NULL,
   *                     it is set to white.
   */
  void setBackgroundColor(const char* color);

  /*! \brief Returns the viewer thread's update frequency.
  */
  double updateFrequency() const;

  /*! \brief Does all rendering. If the viewer's thread has been started
   *         (see runInThread() method), the frame() call is called
   *         periodically from there.
   */
  virtual void frame();

  /*! \brief Copies the camera transformation to A_CI.
   *  \param[out] A_CI Transformation from world to camera frame
   */
  void getCameraTransform(HTr* A_CI) const;

  /*! \brief Sets the camera transformation to A_CI.
   *  \param[in] A_CI Transformation from world to camera frame
   */
  void setCameraTransform(const HTr* A_CI);

  /*! \brief Sets the camera transformation to the given position and Euler
   *         angles. Euler angles are in x-y-z order (rotated frame)
   *
   *  \param[in] x      Camera x-position in world frame
   *  \param[in] y      Camera y-position in world frame
   *  \param[in] z      Camera z-position in world frame
   *  \param[in] thx    Euler angle about x-axis
   *  \param[in] thy    Euler angle about rotated y-axis
   *  \param[in] thz    Euler angle about rotated z-axis
   */
  void setCameraTransform(double x, double y, double z,
                          double thx, double thy, double thz);

  /*! \brief Returns the node and the 3D world position of the
   *         mouse pointer (closest intersection of picking ray and node)
   */
  osg::Node* getNodeUnderMouse(double I_mouseCoords[3]=NULL);

  /*! \brief Returns the node with the given name, or NULL if it is not
   *         part of the scene graph.
   */
  osg::Node* getNode(std::string nodeName);

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

  /*! \brief Sets the rotation center of the Trackball manipulator to the given
   *         3d coordinates.
   *
   *  \param[in]   x Position x in [m]
   *  \param[in]   y Position y in [m]
   *  \param[in]   z Position z in [m]
   *  \return True for success, false otherwise (no mouse manipulator found).
   */
  bool setTrackballCenter(double x, double y, double z);

  /*! \brief Gets the rotation center of the Trackball manipulator in 3d
   *         scene coordinates.
   *
   *  \param[out]   pos   3d position coordinates
   *  \return True for success, false otherwise (no mouse manipulator found).
   *          In case of false, argument pos remains unchanged.
   */
  bool getTrackballCenter(double pos[3]) const;

  /*! \brief Toggles the screen capture mode. For screen recording, a process
   *         is forked that records the screen with avsync or ffmpeg. This
   *         function is also accessible through key "M".
   */
  bool toggleVideoRecording();

  /*! \brief Joins the viewer's thread in case it has been started. If not, the
   *         function does nothing.
   */
  void stopUpdateThread();

  /*! \brief Returns a reference to the internal osgViewer instance.
   */
  osg::ref_ptr<osgViewer::Viewer> getOsgViewer() const;
  double getFieldOfView() const;
  void setFieldOfView(double fov);
  void getMouseTip(double I_tip[3]) const;

  void setCameraHomePosition(const osg::Vec3d& eye,
                             const osg::Vec3d& center,
                             const osg::Vec3d& up=osg::Vec3d(0.0, 0.0, 1.0));
  void setCameraHomePosition(const HTr* transformation);

protected:

  double fps;

  float mouseX;
  float mouseY;
  float normalizedMouseX;
  float normalizedMouseY;

  bool handle(const osgGA::GUIEventAdapter& ea,
              osgGA::GUIActionAdapter& aa);

  static void* ViewerThread(void* arg);
  void create(bool fancy, bool startupWithShadow);
  void init();
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
  bool cartoonEnabled;
  pthread_t frameThread;
  osg::ref_ptr<osgViewer::Viewer> viewer;
  osg::ref_ptr<osgShadow::ShadowedScene> shadowScene;
  osg::ref_ptr<osg::LightSource> cameraLight;
  osg::ref_ptr<osg::Group> rootnode;
  osg::ref_ptr<osg::ClearNode> clearNode;
  std::vector<osg::ref_ptr<osg::Camera> > hud;

private:
  mutable pthread_mutex_t mtxInternal;
  osg::ref_ptr<KeyHandler> keyHandler;
};


}   // namespace Rcs


#endif // RCSVIEWER_H
