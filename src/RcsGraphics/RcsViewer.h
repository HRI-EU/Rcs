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
 *        The public API to add and remove nodes etc. is implemented using the
 *        osg event queue. It means that the function call does not directly
 *        take effect, but defers the command to the next event update. This is
 *        for instance important when a removed node has some internal data. You
 *        should not delete this internal data right after removing the node,
 *        since it might still be accessed before the next event traversal is
 *        happening.
 *
 *        A number of keys are associatde with a function if pressed in the
 *        viewer window. These are documented in the KeyCatcherBase and can
 *        be displayed with KeyCatcherBase::printRegisteredKeys():
 *
 *        - Pressing keys 0 - 9 in the viewer window will set the Rcs log
 *          level to the corresponding level
 *        - Pressing F12 will print out to the console all keys that have been
 *          registered with a function.
 *        - Pressing key w will toggle between solid and wireframe display
 *        - Pressing key s will toggle shadows
 *        - Key F10 will toggle full screen mode
 *        - Key F8 will take a screenshot
 *        - Key F9 will start / stop taking screenshort in each frame
 *        - Key M will toggle movie recording (Linux only)
 *        - Key R will toggle the cartoon mode
 *        - LBM while right mouse button is pressed will select the mouse point
 *          as the new rotation center of the mouse manipulator
 *        - Key z will toggle the OpenScenegraph StatsHandler information
 *        - Key Z will print the OpenScenegraph StatsHandler information to
 *          the console
 *
 *        With some older grapics cards, some features such as anti aliasing are
 *        not available. To disable these, there is a constructor with the
 *        options fancy and startupWithShadow that can be set explicitely.
 *        Alternatively, the create function looks for an environment variable
 *        RCSVIEWER_SIMPLEGRAPHICS. If this is set, the viewer starts without
 *        anti aliasing and shadows.
 */
class Viewer
{
  friend class KeyHandler;
public:

  /*!
     * @name Construction and running
     *
     * Functions to create and run viewer instances
     */

  ///@{


  /*! \brief Default constructor with default window dimensions. Shadows and
   *         anti-aliasing settings are selected by checking if the viewer
   *         runs through X11 forwarding, or on a local machine. The constructor
   *         does not launch a window. This needs to be done separately either
   *         through calling \ref frame() or \ref runInThread().
   */
  Viewer();

  /*! \brief Creates an empty viewer window.
   *
   *  \brief fancy               Enable anti-aliasing and some other features.
   *                             In case your graphics card has issues, this
   *                             should be set to false.
   *  \brief startupWithShadow   Enable shadow casting. Shadow casting can be
   *                             toggled with the s key
   */
  Viewer(bool fancy, bool startupWithShadow);

  /*! \brief Virtual destructor to allow polymorphism.
   */
  virtual ~Viewer();

  /*! \brief Applies OpenSceneGraph's optimization function to the viewer's
   *         root node.
   */
  virtual void optimize();

  /*! \brief Starts a thread that periodically calls the frame() call. The
   *         thread will try to achieve the given updateFrequency. Changing the
   *         update frequency will take effect also when the thread is running.
   *
   * \param[in] mutex   Optional mutex that will be locked whenever the viewer
   *                    traverses or does changes to the scene graph.
   */
  void runInThread(pthread_mutex_t* mutex = NULL);

  /*! \brief Does all rendering. If the viewer's thread has been started
   *         (see runInThread() method), the frame() call is called
   *         periodically from there.
   */
  virtual void frame();

  /*! \brief Locks all mutexes around the frame() call. Depending on the
   *         complexity of the scene graph, the lock() function can block for
   *         a while (worst case: rendering framerate)
   */
  virtual bool lock() const;

  /*! \brief Unlocks all mutexes around the frame() call.
   */
  virtual bool unlock() const;

  /*! \brief Joins the viewer's thread in case it has been started. If not, the
   *         function does nothing.
   */
  void stopUpdateThread();

  /*! \brief Sets the viewer's update frequency. This only has an effect if the
   *         viewer runs its own thread (see runInThread() method). The default
   *         is 25Hz.
   *
   *  \param[in] Hz  Update frequency in [Hz]
   */
  void setUpdateFrequency(double Hz);

  /*! \brief Toggles the screen capture mode. For screen recording, a process
   *         is forked that records the screen with avsync or ffmpeg. This
   *         function is also accessible through key "M".
   */
  bool toggleVideoRecording();

  /*! \brief Sets the withh of the field of view to the given angle. The aspect
   *         ratio is kept constant.
   *  \param[in] fov   Horizontal field of view in degrees
   */
  void setFieldOfView(double fov);


  /*! \brief Sets the field of view to the given angles.
   *
   *  \param[in] fovWidth   Horizontal field of view in degrees
   *  \param[in] fovHeight  Vertical field of view in degrees
   */
  void setFieldOfView(double fovWidth, double fovHeight);

  ///@}



  /*!
     * @name Scene graph manipulation
     *
     * Functions are deferred to the osg event loop
     */

  ///@{

  /*! \brief Adds the osg::Node to the root node of the viewer's scene graph.
   *         This function does not directly add the node, but defers it to
   *         the next update traversal.
   */
  void add(osg::Node* node);

  /*! \brief Adds the osg::Node to the given parent node. This function does
   *         not directly add the node, but defers it to the next update
   *         traversal.
   */
  void add(osg::Group* parent, osg::Node* child);

  /*! \brief Adds the event handler to the root node of the viewer's scene
   *         graph. This function does not directly add the node, but defers
   *         it to the next update traversal.
   */
  void add(osgGA::GUIEventHandler* eventHandler);

  /*! \brief Removes the given node from the viewer's scene graph.
   */
  void removeNode(osg::Node* node);

  /*! \brief Removes all nodes with the given name from the viewer's
   *         scene graph. This function does not directly add the node, but
   *         defers it to the next update traversal.
   */
  void removeNode(std::string nodeName);

  /*! \brief Removes all child nodes of parent with the given nodeName from
   *         the parent node. This function does not directly add the node,
   *         but defers it to the next update traversal.
   */
  void removeNode(osg::Group* parent, std::string nodeName);

  /*! \brief Removes all osg::Node from the viewer's root node, but leaves the
   *         background clear node. This function does not directly add the
   *         node, but defers it to the next update traversal.
   */
  void removeNodes();

  /*! \brief Resets the camera to the parameters (field of view, near and far
   *         planes) that the viewer has been initialized with. Deferred to
   *         the event loop.
   */
  void resetView();

  /*! \brief Sets the given title to the window rectangle. Deferred to the
   *         event loop.
   */
  void setTitle(const std::string& title);

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

  /*! \brief Sets the camera transformation to A_CI.
   *  \param[in] A_CI Transformation from world to camera frame
   */
  void setCameraTransform(const HTr* A_CI);

  /*! \brief Sets the viewer's background color. See colorFromString() for
   *         colors.
   *
   *  \param[in] color   Color to be set as background color. if color is NULL,
   *                     it is set to white.
   */
  void setBackgroundColor(const std::string& color);

  /*! \brief Enable or disable shadow casting.
   *
   *  \param[in] enable  True for shadow casting, false for no shadows.
   */
  void setShadowEnabled(bool enable);

  /*! \brief Set wire frame mode.
   *
   *  \param[in] wf  True for wire frame display, false for solid.
   */
  void displayWireframe(bool wf = true);

  /*! \brief Toggles the wire frame mode.
   */
  void toggleWireframe();

  /*! \brief Sets the default camera transform.
   */
  void setCameraHomePosition(const HTr* transformation);

  /*! \brief Enable or disable cartoon mode.
   *
   *  \param[in] enable  True for cartoon mode, false otherwise.
   */
  void setCartoonEnabled(bool enable);

  /*! \brief Sets the rotation center of the Trackball manipulator to the given
   *         3d coordinates.
   *
   *  \param[in]   x Position x in [m]
   *  \param[in]   y Position y in [m]
   *  \param[in]   z Position z in [m]
   */
  void setTrackballCenter(double x, double y, double z);

  ///@}



  /*!
   * @name Accessors
   *
   * Functions to query information from the viewer
   */

  ///@{

  /*! \brief Copies the camera transformation to A_CI.
   *  \param[out] A_CI Transformation from world to camera frame
   */
  void getCameraTransform(HTr* A_CI) const;

  /*! \brief Returns the viewer thread's update frequency.
  */
  double updateFrequency() const;

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

  /*! \brief Gets the rotation center of the Trackball manipulator in 3d
   *         scene coordinates.
   *
   *  \param[out]   pos   3d position coordinates
   *  \return True for success, false otherwise (no mouse manipulator found).
   *          In case of false, argument pos remains unchanged.
   */
  bool getTrackballCenter(double pos[3]) const;

  /*! \brief Returns true after the viewer thread has been joined, false
   *         otherwise.
   */
  bool isThreadStopped() const;

  /*! \brief Returns a reference to the internal osgViewer instance.
   */
  osg::ref_ptr<osgViewer::Viewer> getOsgViewer() const;

  double getFieldOfView() const;

  void getMouseTip(double I_tip[3]) const;

  ///@}






protected:

  /*! \brief Called from the KeyHandler's update function.
   */
  bool handle(const osgGA::GUIEventAdapter& ea,
              osgGA::GUIActionAdapter& aa);

  void handleUserEvents(const osg::Referenced* userEvent);

  static void* ViewerThread(void* arg);
  void create(bool fancy, bool startupWithShadow);
  void init();
  void setSceneData(osg::Node* node);
  bool isInitialized() const;
  bool isThreadRunning() const;
  bool isRealized() const;

  /*! \brief Changes the window size. This function only takes effect if there
   *         was no frame() call before.
   *
   *  \param[in] llx     Lower left x screen coordinate
   *  \param[in] lly     Lower left y screen coordinate
   *  \param[in] sizeX   Screen size in x-direction
   *  \param[in] sizeY   Screen size in y-direction
   *
   *  \return True for success, false otherwise: View has already been set
   *          up (frame() has been called). In case of failure, there will be a
   *          log message on debug level 1.
   */
  bool setWindowSize(unsigned int llx,     // lower left x
                     unsigned int lly,     // lower left y
                     unsigned int sizeX,   // size in x-direction
                     unsigned int sizeY);  // size in y-direction

  double fps;
  float mouseX;
  float mouseY;
  float normalizedMouseX;
  float normalizedMouseY;

  mutable pthread_mutex_t* mtxFrameUpdate;
  bool threadRunning;
  double updateFreq;
  bool initialized;
  bool wireFrame;
  bool shadowsEnabled;

  unsigned int llx, lly, sizeX, sizeY;
  bool cartoonEnabled;
  bool threadStopped;
  bool leftMouseButtonPressed;
  bool rightMouseButtonPressed;
  pthread_t frameThread;

  // osg node members
  osg::ref_ptr<osgViewer::Viewer> viewer;
  osg::ref_ptr<osgShadow::ShadowedScene> shadowScene;
  osg::ref_ptr<osg::LightSource> cameraLight;
  osg::ref_ptr<osg::Group> rootnode;
  osg::ref_ptr<osg::ClearNode> clearNode;
  osg::ref_ptr<KeyHandler> keyHandler;
  osg::Matrix startView;

  // Event handling of user events. We buffer them in a separate vector so that
  // they can be published before the viewer is realized.
  std::vector<osg::ref_ptr<osg::Referenced>> userEventStack;
  OpenThreads::Mutex userEventMtx;



private:

  /*! \brief Adds a node to the rootNode. This function must not be called
   *         concurrently with the viewer's frame update.
   *
   * \return node   Node to be added. Can also be of certain derived types
   *                such as camera etc.
   */
  bool addInternal(osg::Node* node);

  /*! \brief Adds a node to a parent. This function must not be called
   *         concurrently with the viewer's frame update.
   *
   * \return parent Group node the node is to be attached to.
   * \return node   Node to be added. Can also be of certain derived types
   *                such as camera etc.
   */
  bool addInternal(osg::Group* parent, osg::Node* child);

  /*! \brief Removes the node from the viewer's scene graph. This is called
   *         from inside the locked frame() call so that there is no
   *         concurrency issue.
   *
   * \return True for success, false otherwise: node is NULL, or not found in
   *         the viewer's scenegraph. The scenegraph is searched through all
   *         levels. The frame mutex is internally set around the scenegraph
   *         modification so that threading issues can be avoided.
   */
  bool removeInternal(osg::Node* node);

  /*! \brief Removes all nodes with the given name from the viewer's scene
   *         graph. This is called from inside the locked frame() call so that
   *         there is no concurrency issue.
   *
   * \return Number of nodes removed.
   */
  int removeInternal(std::string nodeName);

  /*! \brief Removes all nodes with the given name from the parent node.
   *
   * \return Number of nodes removed.
   */
  int removeInternal(osg::Node* parent, std::string nodeName);

  /*! \brief Removes all nodes from the rootNode.
   *
   * \return Number of nodes removed.
   */
  int removeAllNodesInternal();

  /*! \brief Adds a custom event to the internal user event queue. This is
   *         protected by the userEventMtx in order to avoid concurrent
   *         access with the frame() function.
   */
  void addUserEvent(osg::Referenced* userEvent);
};


}   // namespace Rcs


#endif // RCSVIEWER_H
