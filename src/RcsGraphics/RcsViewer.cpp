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

#include "RcsViewer.h"
#include "Rcs_graphicsUtils.h"
#include "BodyNode.h"

#include <Rcs_macros.h>
#include <Rcs_timer.h>
#include <KeyCatcherBase.h>
#include <Rcs_utils.h>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgViewer/ViewerEventHandlers>
#include <osgViewer/GraphicsWindow>
#include <osgUtil/Optimizer>
#include <osg/StateSet>
#include <osg/PolygonMode>
#include <osgShadow/ShadowMap>
#include <osgFX/Cartoon>
#include <osgGA/TrackballManipulator>

#if !defined (_MSC_VER)
#include <osgViewer/api/X11/GraphicsWindowX11>

#include <X11/Xlib.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

#include <iostream>



#if !defined (_MSC_VER)
static pid_t forkProcess(const char* command)
{
  pid_t pid = fork();

  if (pid == 0)
  {
    execl("/bin/bash", "bash", "-c", command, NULL);
    perror("execl");
    exit(1);
  }

  return pid;
}
#endif






using namespace Rcs;



class RcsManipulator : public osgGA::TrackballManipulator
{
public:
  RcsManipulator() : osgGA::TrackballManipulator(), leftShiftPressed(false)
  {
  }

  ~RcsManipulator()
  {
  }

  virtual bool handle(const osgGA::GUIEventAdapter& ea,
                      osgGA::GUIActionAdapter& aa)
  {

    switch (ea.getEventType())
    {
      case (osgGA::GUIEventAdapter::KEYDOWN):
      {
        if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Shift_L)
        {
          this->leftShiftPressed = true;
        }
        break;
      }

      case (osgGA::GUIEventAdapter::KEYUP):
      {
        if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Shift_L)
        {
          this->leftShiftPressed = false;
        }
        break;
      }

      default:
      {
      }
    }   // switch(...)

    if (this->leftShiftPressed == true)
    {
      return false;
    }

    return osgGA::TrackballManipulator::handle(ea, aa);
  }



  bool leftShiftPressed;
};

/*******************************************************************************
 * Keyboard handler for default keys.
 ******************************************************************************/
namespace Rcs
{
class KeyHandler : public osgGA::GUIEventHandler
{
public:

  KeyHandler(Rcs::Viewer* viewer)
  {
    RCHECK(viewer);
    _viewer = viewer;
    _cartoonEnabled = false;
    _video_capture_process = -1;

    KeyCatcherBase::registerKey("0-9", "Set Rcs debug level", "Viewer");
    KeyCatcherBase::registerKey("w", "Toggle wireframe mode", "Viewer");
    KeyCatcherBase::registerKey("s", "Cycle between shadow modes", "Viewer");
    KeyCatcherBase::registerKey("R", "Toggle cartoon mode", "Viewer");
#if !defined(_MSC_VER)
    KeyCatcherBase::registerKey("M", "Toggle video capture", "Viewer");
#endif
    KeyCatcherBase::registerKey("F11", "Print camera transform", "Viewer");
  }

  ~KeyHandler()
  {
    if (_video_capture_process >= 0)
    {
      toggleVideoCapture();
    }
  }

  virtual bool handle(const osgGA::GUIEventAdapter& ea,
                      osgGA::GUIActionAdapter& aa)
  {
    switch (ea.getEventType())
    {

      case (osgGA::GUIEventAdapter::FRAME):
      {
        _viewer->mouseX = ea.getX();
        _viewer->mouseY = ea.getY();

        if (_viewer->cameraLight.valid())
        {
          HTr A_CI;
          _viewer->getCameraTransform(&A_CI);
          osg::Vec4 lightpos;
          lightpos.set(A_CI.org[0], A_CI.org[1], A_CI.org[2]+0*2.0, 1.0f);
          _viewer->cameraLight->getLight()->setPosition(lightpos);
        }
        break;
      }

      case (osgGA::GUIEventAdapter::KEYDOWN):
      {
        // key '0' is ASCII code 48, then running up to 57 for '9'
        if ((ea.getKey() >= 48) && (ea.getKey() <= 57))
        {
          unsigned int dLev =  ea.getKey() - 48;
          RcsLogLevel = dLev;
          RMSG("Setting debug level to %u", dLev);
          return false;
        }

        else if (ea.getKey() == osgGA::GUIEventAdapter::KEY_F11)
        {
          HTr A_CI;
          double x[6];
          _viewer->getCameraTransform(&A_CI);
          HTr_to6DVector(x, &A_CI);
          RMSGS("Camera pose is %f %f %f   %f %f %f",
                x[0], x[1], x[2], x[3], x[4], x[5]);
        }

        //
        // Toggle wireframe
        //
        else if (ea.getKey() == 'w')
        {
          _viewer->toggleWireframe();
          return false;
        }

        //
        // Toggle shadows
        //
        else if (ea.getKey() == 's')
        {
          _viewer->setShadowEnabled(!_viewer->shadowsEnabled);
          return false;
        }

        //
        // Toggle cartoon mode
        //
        else if (ea.getKey() == 'R')
        {
          // Once cartoon mode is enabled, other keys than R don't work
          // anymore. This is a OSG bug, because the effect node seems to
          // not call the children's eventhandlers
          _cartoonEnabled = !_cartoonEnabled;
          _viewer->setCartoonEnabled(_cartoonEnabled);
          return false;
        }
        else if (ea.getKey() == 'M')
        {
          toggleVideoCapture();
          return false;
        }

        break;
      }   // case(osgGA::GUIEventAdapter::KEYDOWN):

      default:
        break;

    }   // switch(ea.getEventType())

    return false;
  }

  bool toggleVideoCapture()
  {
    bool captureRunning = false;

#if !defined(_MSC_VER)
    if (_video_capture_process >= 0)
    {
      // Stop video taking
      kill(_video_capture_process, SIGINT);
      waitpid(_video_capture_process, NULL, 0);
      _video_capture_process = -1;
    }
    else
    {
      // movie taken using avconv x11grab

      // get the first window
      osgViewer::ViewerBase::Windows windows;
      _viewer->viewer->getWindows(windows, true);

      if (!windows.empty())
      {
        int x = 0;
        int y = 0;
        int w = 0;
        int h = 0;

        windows[0]->getWindowRectangle(x, y, w, h);

        static unsigned int movie_number = 1;

        RMSG("Start capturing: (%d, %d) %dx%d", x, y, w, h);
        std::stringstream cmd;
        cmd << "avconv -y -f x11grab -r 25 -s "
            << w << "x" << h
            << " -i " << getenv("DISPLAY") << "+"
            << x << "," << y
            << " -crf 20 -r 25 -c:v libx264 -c:a n"
            << " /tmp/movie_" << movie_number++ << ".mp4";

        _video_capture_process = forkProcess(cmd.str().c_str());
        captureRunning = true;
      }
    }
#endif

    return captureRunning;
  }

private:

  Rcs::Viewer* _viewer;
  bool _cartoonEnabled;
  pid_t _video_capture_process;
};
}

/*******************************************************************************
 * Viewer class.
 ******************************************************************************/
Viewer::Viewer() :
  mouseX(0.0), mouseY(0.0), fps(0.0), mtxFrameUpdate(NULL),
  threadRunning(false), updateFreq(25.0), initialized(false),
  wireFrame(false), shadowsEnabled(false)
{
  // Check if logged in remotely
  const char* sshClient = getenv("SSH_CLIENT");
  bool fancy = true;

  if (sshClient != NULL)
  {
    if (strlen(sshClient) > 0)
    {
      RLOGS(4, "Remote login detected - simple viewer settings");
      fancy = false;
    }
  }

  create(fancy, fancy);

  RLOG(5, "Done constructor of viewer");
}

/*******************************************************************************
 * Viewer class.
 ******************************************************************************/
Viewer::Viewer(bool fancy, bool startupWithShadow) :
  mouseX(0.0), mouseY(0.0), fps(0.0), mtxFrameUpdate(NULL),
  threadRunning(false), updateFreq(25.0), initialized(false),
  wireFrame(false), shadowsEnabled(false)
{
  create(fancy, startupWithShadow);

  RLOG(5, "Done constructor of viewer");
}

/*******************************************************************************
 * Destructor.
 ******************************************************************************/
Viewer::~Viewer()
{
  stopThread();
  pthread_mutex_destroy(&this->mtxInternal);
}

/*******************************************************************************
 * Initlalization method.
 ******************************************************************************/
void Viewer::create(bool fancy, bool startupWithShadow)
{
#if defined(_MSC_VER)
  setWindowSize(12, 31, 640, 480);
#else
  setWindowSize(0, 0, 640, 480);
#endif

  pthread_mutex_init(&this->mtxInternal, NULL);

  this->shadowsEnabled = startupWithShadow;

  // Rotate loaded file nodes to standard coordinate conventions
  // (z: up, x: forward)
  osgDB::ReaderWriter::Options* options = new osgDB::ReaderWriter::Options;
  options->setOptionString("noRotation");
  osgDB::Registry::instance()->setOptions(options);

  this->viewer = new osgViewer::Viewer();

  // Mouse manipulator (needs to go before event handler)
  osg::ref_ptr<RcsManipulator> trackball = new RcsManipulator();
  viewer->setCameraManipulator(trackball.get());

  // Handle some default keys (see handler above)
  this->keyHandler = new KeyHandler(this);
  viewer->addEventHandler(this->keyHandler);

  // Root node (instead of a Group we create an Cartoon node for optional
  // cell shading)
  this->rootnode = new osgFX::Cartoon;
  dynamic_cast<osgFX::Effect*>(rootnode.get())->setEnabled(false);

  // Light grayish green universe
  this->clearNode = new osg::ClearNode;
  this->clearNode->setClearColor(colorFromString("LIGHT_GRAYISH_GREEN"));
  this->rootnode->addChild(this->clearNode.get());

  // Light model: We switch off hte default viewer light, and configure two
  // light sources. The sunlight shines down from 10m. Another light source
  // moves with the camera, so that there are no dark spots whereever
  // the mouse manipulator moves to.

  // Disable default light
  rootnode->getOrCreateStateSet()->setMode(GL_LIGHT0, osg::StateAttribute::OFF);

  // Light source that moves with the camera
  this->cameraLight = new osg::LightSource;
  cameraLight->getLight()->setLightNum(1);
  cameraLight->getLight()->setPosition(osg::Vec4(0.0, 0.0, 10.0, 1.0));
  cameraLight->getLight()->setSpecular(osg::Vec4(1.0, 1.0, 1.0, 1.0));
  rootnode->addChild(cameraLight.get());
  rootnode->getOrCreateStateSet()->setMode(GL_LIGHT1, osg::StateAttribute::ON);

  // Light source that shines down
  osg::ref_ptr<osg::LightSource> sunlight = new osg::LightSource;
  sunlight->getLight()->setLightNum(2);
  sunlight->getLight()->setPosition(osg::Vec4(0.0, 0.0, 10.0, 1.0));
  rootnode->addChild(sunlight.get());
  rootnode->getOrCreateStateSet()->setMode(GL_LIGHT2, osg::StateAttribute::ON);

  // Shadow map scene. We use the sunlight to case shadows.
  this->shadowScene = new osgShadow::ShadowedScene;
  osg::ref_ptr<osgShadow::ShadowMap> sm = new osgShadow::ShadowMap;
  sm->setTextureSize(osg::Vec2s(2048, 2048));
  sm->setLight(sunlight->getLight());
  sm->setPolygonOffset(osg::Vec2(-0.7, 0.0));
  sm->setAmbientBias(osg::Vec2(0.7, 0.3));   // values need to sum up to 1.0

  shadowScene->setShadowTechnique(sm.get());
  shadowScene->addChild(rootnode.get());
  shadowScene->setReceivesShadowTraversalMask(ReceivesShadowTraversalMask);
  shadowScene->setCastsShadowTraversalMask(CastsShadowTraversalMask);


  // Change the threading model. The default threading model is
  // osgViewer::Viewer::CullThreadPerCameraDrawThreadPerContext.
  // This leads to problems with multi-threaded updates (HUD).
  viewer->setThreadingModel(osgViewer::Viewer::CullDrawThreadPerContext);

  // Create viewer in a window
  if (fancy == false)
  {
    viewer->setSceneData(rootnode.get());
  }
  else
  {
    // Set anti-aliasing
    osg::ref_ptr<osg::DisplaySettings> ds = new osg::DisplaySettings;
    ds->setNumMultiSamples(4);
    viewer->setDisplaySettings(ds.get());
    viewer->setSceneData(startupWithShadow ? shadowScene.get() : rootnode.get());
  }

  // Disable small feature culling to avoid problems with drawing single points
  // as they have zero bounding box size
  viewer->getCamera()->setCullingMode(viewer->getCamera()->getCullingMode() &
                                      ~osg::CullSettings::SMALL_FEATURE_CULLING);

  setCameraHomePosition(osg::Vec3d(4.0,  3.5, 3.0),
                        osg::Vec3d(0.0, -0.2, 0.8),
                        osg::Vec3d(0.0, 0.05, 1.0));

  KeyCatcherBase::registerKey("F10", "Toggle full screen", "Viewer");
  osg::ref_ptr<osgViewer::WindowSizeHandler> wsh = new osgViewer::WindowSizeHandler;
  wsh->setKeyEventToggleFullscreen(osgGA::GUIEventAdapter::KEY_F10);
  viewer->addEventHandler(wsh.get());

  KeyCatcherBase::registerKey("F9", "Toggle continuous screenshots", "Viewer");
  KeyCatcherBase::registerKey("F8", "Take screenshot(s)", "Viewer");
  osg::ref_ptr<osgViewer::ScreenCaptureHandler> captureHandler = new osgViewer::ScreenCaptureHandler(
    new osgViewer::ScreenCaptureHandler::WriteToFile("screenshot", "jpg", osgViewer::ScreenCaptureHandler::WriteToFile::SEQUENTIAL_NUMBER), -1);
  captureHandler->setKeyEventToggleContinuousCapture(osgGA::GUIEventAdapter::KEY_F9);
  captureHandler->setKeyEventTakeScreenShot(osgGA::GUIEventAdapter::KEY_F8);
  viewer->addEventHandler(captureHandler.get());
}

/*******************************************************************************
 * Add a node to the root node.
 ******************************************************************************/
void Viewer::setWindowSize(unsigned int llx_,     // lower left x
                           unsigned int lly_,     // lower left y
                           unsigned int sizeX_,   // size in x-direction
                           unsigned int sizeY_)
{
  if (isInitialized() == true)
  {
    RLOG(1, "The window size can't be changed after launching the viewer "
         "window");
    return;
  }

  this->llx = llx_;
  this->lly = lly_;
  this->sizeX = sizeX_;
  this->sizeY = sizeY_;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Viewer::add(osgGA::GUIEventHandler* eventHandler)
{
  lock();
  viewer->addEventHandler(eventHandler);
  unlock();
}

/*******************************************************************************
 * Add a node to the root node.
 ******************************************************************************/
void Viewer::add(osg::Node* node)
{
  osg::Camera* hud = dynamic_cast<osg::Camera*>(node);

  if (hud != NULL)
  {
    this->hud.push_back(hud);
    return;
  }

  if (node != NULL)
  {
    lock();
    this->rootnode->addChild(node);
    unlock();
  }
  else
  {
    RLOG(1, "Failed to add osg::Node - node is NULL!");
  }
}

/*******************************************************************************
 * Removes a node from the scene graph.
 ******************************************************************************/
void Viewer::removeNode(osg::Node* node)
{
  if (node == NULL)
  {
    RLOG(1, "Node is NULL - can't be deleted");
    return;
  }

  osg::Camera* hud = dynamic_cast<osg::Camera*>(node);

  if (hud != NULL)
  {
    osg::View::Slave* slave = viewer->findSlaveForCamera(hud);

    if (slave != NULL)
    {
      RLOG(4, "Hud can't be deleted - is not part of the scene graph");
      return;
    }

    // We are a bit pedantic and check that the camera is not the
    // viewer's camera.
    unsigned int si = viewer->findSlaveIndexForCamera(hud);
    unsigned int ci = viewer->findSlaveIndexForCamera(viewer->getCamera());

    if (ci != si)
    {
      viewer->removeSlave(si);
      RLOG(5, "Hud successully deleted");
    }
    else
    {
      RLOG(1, "Cannot remove the viewer's camera");
    }

    return;
  }

  if (rootnode->containsNode(node))
  {
    rootnode->removeChild(node);
  }
  else
  {
    RLOG(4, "Node can't be deleted - is not part of the scene graph");
  }

}

/*******************************************************************************
 * Sets the update frequency in [Hz].
 ******************************************************************************/
void Viewer::setUpdateFrequency(double Hz)
{
  this->updateFreq = Hz;
}

/*******************************************************************************
 * Returns the update frequency in [Hz].
 ******************************************************************************/
double Viewer::updateFrequency() const
{
  return this->updateFreq;
}

/*******************************************************************************
 * Sets the camera position and viewing direction
 * First vector is where camera is, Second vector is where the
 * camera points to, the up vector is set internally to always stay upright
 ******************************************************************************/
void Viewer::setCameraHomePosition(const osg::Vec3d& eye,
                                   const osg::Vec3d& center,
                                   const osg::Vec3d& up)
{
  viewer->getCameraManipulator()->setHomePosition(eye, center, up);
  viewer->home();
}

/*******************************************************************************
 *
 ******************************************************************************/
void Viewer::setCameraHomePosition(const HTr* A_CI)
{
  osg::Vec3d eye(A_CI->org[0], A_CI->org[1], A_CI->org[2]);

  osg::Vec3d center(A_CI->org[0] + A_CI->rot[2][0],
                    A_CI->org[1] + A_CI->rot[2][1],
                    A_CI->org[2] + A_CI->rot[2][2]);

  osg::Vec3d up(-A_CI->rot[1][0], -A_CI->rot[1][1], -A_CI->rot[1][2]);

  setCameraHomePosition(eye, center, up);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Viewer::getCameraTransform(HTr* A_CI) const
{
  osg::Matrix matrix = viewer->getCamera()->getViewMatrix();
  HTr_fromViewMatrix(matrix, A_CI);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Viewer::setCameraTransform(const HTr* A_CI)
{
  osg::Matrix vm = viewMatrixFromHTr(A_CI);
  viewer->getCameraManipulator()->setByInverseMatrix(vm);
}

/*******************************************************************************
 *
 ******************************************************************************/
osg::Node* Viewer::getNodeUnderMouse(double I_mouseCoords[3])
{
  return Rcs::getNodeUnderMouse<osg::Node*>(*this->viewer.get(),
                                            mouseX, mouseY, I_mouseCoords);
}

/*******************************************************************************
 *
 ******************************************************************************/
double Viewer::getFieldOfView() const
{
  double fovy, aspectRatio, zNear, zFar;
  viewer->getCamera()->getProjectionMatrixAsPerspective(fovy, aspectRatio,
                                                        zNear, zFar);
  return fovy;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Viewer::setFieldOfView(double fovy)
{
  double fovy_old, aspectRatio, zNear, zFar;
  viewer->getCamera()->getProjectionMatrixAsPerspective(fovy_old, aspectRatio, zNear, zFar);
  viewer->getCamera()->setProjectionMatrixAsPerspective(fovy, aspectRatio, zNear, zFar);
}

/*******************************************************************************
 * Runs the viewer in its own thread.
 ******************************************************************************/
void* Viewer::ViewerThread(void* arg)
{
  Rcs::Viewer* viewer = static_cast<Rcs::Viewer*>(arg);

  if (viewer->isThreadRunning() == true)
  {
    RLOG(1, "Viewer thread is already running");
    return NULL;
  }

  viewer->lock();
  viewer->init();
  viewer->threadRunning = true;
  viewer->unlock();

  while (viewer->isThreadRunning() == true)
  {
    viewer->frame();
    unsigned long dt = (unsigned long)(1.0e6/viewer->updateFrequency());
    Timer_usleep(dt);
  }

  return NULL;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Viewer::runInThread(pthread_mutex_t* mutex)
{
  this->mtxFrameUpdate = mutex;
  pthread_create(&frameThread, NULL, ViewerThread, (void*) this);

  // Wait until the class has been initialized
  while (!isInitialized())
  {
    Timer_usleep(10000);
  }

  // ... and realized
  while (!isRealized())
  {
    Timer_usleep(10000);
  }

}

/*******************************************************************************
 * For true, displays all nodes in wireframe, otherwise in solid
 ******************************************************************************/
void Viewer::displayWireframe(bool wf)
{
  this->wireFrame = wf;
  osg::ref_ptr<osg::StateSet> pStateSet = rootnode->getOrCreateStateSet();

  if (wf == true)
  {
    pStateSet->setAttribute(new osg::PolygonMode
                            (osg::PolygonMode::FRONT_AND_BACK,
                             osg::PolygonMode::LINE));
  }
  else
  {
    pStateSet->setAttribute(new osg::PolygonMode
                            (osg::PolygonMode::FRONT_AND_BACK,
                             osg::PolygonMode::FILL));
  }

}

/*******************************************************************************
 * Toggles between solid and wireframe display
 ******************************************************************************/
void Viewer::toggleWireframe()
{
  displayWireframe(!this->wireFrame);
}

/*******************************************************************************
 * Switches between shadow casting modes.
 ******************************************************************************/
void Viewer::setShadowEnabled(bool enable)
{
  osg::Matrix lastViewMatrix = viewer->getCameraManipulator()->getMatrix();

  if (enable==false)
  {
    RLOG(3, "Shadows off");
    if (this->shadowsEnabled == true)
    {
      viewer->setSceneData(rootnode.get());
    }
    this->shadowsEnabled = false;
  }
  else
  {
    RLOG(3, "Shadows on");
    if (this->shadowsEnabled == false)
    {
      viewer->setSceneData(shadowScene.get());
    }
    this->shadowsEnabled = true;
  }

  viewer->getCameraManipulator()->setByMatrix(lastViewMatrix);
}

/*******************************************************************************
 * Renders the scene in cartoon mode.
 ******************************************************************************/
void Viewer::setCartoonEnabled(bool enabled)
{
  if (enabled == true)
  {
    setShadowEnabled(false);
  }

  dynamic_cast<osgFX::Effect*>(rootnode.get())->setEnabled(enabled);
}

/*******************************************************************************
 * Renders the scene in cartoon mode.
 ******************************************************************************/
void Viewer::setBackgroundColor(const char* color)
{
  this->clearNode->setClearColor(colorFromString(color));
}

/*******************************************************************************
 *
 ******************************************************************************/
void Viewer::frame()
{
  if (isInitialized() == false)
  {
    init();
  }

  double dtFrame = Timer_getSystemTime();

  lock();
  viewer->frame();
  unlock();

  dtFrame = Timer_getSystemTime() - dtFrame;
  this->fps = 0.9*this->fps + 0.1*(1.0/dtFrame);
}

/*******************************************************************************
  This initialization function needs to be called from the thread that also
  calls the osg update traversals. That's why it is separated from the
  constructor. Otherwise, it leads to problems under msvc.
*******************************************************************************/
void Viewer::init()
{
  if (isInitialized() == true)
  {
    return;
  }

  viewer->setUpViewInWindow(llx, lly, sizeX, sizeY);

  // Stop listening to ESC key, cause it doesn't end RCS properly
  viewer->setKeyEventSetsDone(0);
  viewer->realize();

  // Memorize the view matrix
  //this->lastViewMatrix = viewer->getCamera()->getViewMatrix();

  // Add all HUD's after creation of the window
  osgViewer::Viewer::Windows windows;
  viewer->getWindows(windows);

  if (windows.empty())
  {
    RLOG(1, "Failed to add HUD - no viewer window");
  }
  else
  {
    for (size_t i=0; i<hud.size(); i++)
    {
      hud[i]->setGraphicsContext(windows[0]);
      hud[i]->setViewport(0, 0, windows[0]->getTraits()->width,
                          windows[0]->getTraits()->height);
      viewer->addSlave(hud[i].get(), false);
    }

  }

  this->initialized = true;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Viewer::optimize()
{
  osgUtil::Optimizer optimizer;
  optimizer.optimize(rootnode);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Viewer::setSceneData(osg::Node* node)
{
  viewer->setSceneData(node);
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Viewer::isRealized() const
{
  return viewer->isRealized();
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Viewer::isInitialized() const
{
  return this->initialized;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Viewer::isThreadRunning() const
{
  return this->threadRunning;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Viewer::stopThread()
{
  if (threadRunning == false)
  {
    return;
  }

  this->threadRunning = false;
  pthread_join(frameThread, NULL);
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Viewer::lock() const
{
  pthread_mutex_lock(&this->mtxInternal);

  if (this->mtxFrameUpdate!=NULL)
  {
    pthread_mutex_lock(this->mtxFrameUpdate);
    return true;
  }

  return false;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Viewer::unlock() const
{
  pthread_mutex_unlock(&this->mtxInternal);

  if (this->mtxFrameUpdate!=NULL)
  {
    pthread_mutex_unlock(this->mtxFrameUpdate);
    return true;
  }

  return false;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Viewer::toggleVideoRecording()
{
  return keyHandler->toggleVideoCapture();
}
