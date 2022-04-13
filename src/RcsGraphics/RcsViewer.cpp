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

#include "RcsViewer.h"
#include "Rcs_graphicsUtils.h"

#include <Rcs_macros.h>
#include <Rcs_timer.h>
#include <KeyCatcherBase.h>
#include <Rcs_Vec3d.h>
#include <Rcs_VecNd.h>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgUtil/Optimizer>
#include <osg/StateSet>
#include <osg/PolygonMode>
#include <osgShadow/ShadowMap>
#include <osgFX/Cartoon>
#include <osgGA/TrackballManipulator>
#include <osgViewer/ViewerEventHandlers>
#include <osg/GraphicsContext>

#include <iostream>
#include <cstring>

#if !defined (_MSC_VER)

#include <sys/wait.h>
#include <unistd.h>

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

static const std::string defaultBgColor = "LIGHT_GRAYISH_GREEN";

namespace Rcs
{

/*******************************************************************************
 * Keyboard handler for default keys. The default manipulator is extended so
 * that the default space behavior (default camera pose) is disabled, and that
 * the mouse does not move the tracker once Shift-L is pressed. This allows to
 * implement a mouse spring.
 ******************************************************************************/
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
    bool spacePressed = false;

    switch (ea.getEventType())
    {
      case (osgGA::GUIEventAdapter::KEYDOWN):
      {
        if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Shift_L)
        {
          this->leftShiftPressed = true;
        }
        else if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Space)
        {
          spacePressed = true;
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

    if ((this->leftShiftPressed==true) || (spacePressed==true))
    {
      return false;
    }

    return osgGA::TrackballManipulator::handle(ea, aa);
  }



  bool leftShiftPressed;
};

/*******************************************************************************
 * User event data structure for custom events.
 ******************************************************************************/
struct ViewerEventData : public osg::Referenced
{
  enum EventType
  {
    AddNode = 0,
    AddChildNode,
    AddEventHandler,
    RemoveNode,
    RemoveNamedNode,
    RemoveChildNode,
    RemoveAllNodes,
    SetCameraTransform,
    SetCameraHomePose,
    SetCameraHomePoseEyeCenterUp,
    SetTitle,
    ResetView,
    SetBackgroundColor,
    SetShadowEnabled,
    SetWireframeEnabled,
    SetCartoonEnabled,
    SetTrackballCenter,
    SetFrameUpdatesEnabled,
    None
  };

  ViewerEventData(EventType type) : eType(type), flag(false)
  {
    init(type, "No arguments");
  }

  ViewerEventData(osg::ref_ptr<osg::Node> node_, EventType type) :
    node(node_), eType(type), flag(false)
  {
    init(type, node->getName());
  }

  ViewerEventData(const HTr* transform, EventType type) :
    eType(type), flag(false)
  {
    HTr_copy(&trf, transform);
    init(type, "Transform");
  }

  ViewerEventData(std::string nodeName, EventType type) :
    childName(nodeName), eType(type), flag(false)
  {
    init(type, nodeName);
  }

  ViewerEventData(osg::ref_ptr<osg::Group> parent_,
                  osg::ref_ptr<osg::Node> node_, EventType type) :
    parent(parent_), node(node_), eType(type), flag(false)
  {
    init(type, node->getName());
  }

  ViewerEventData(osg::ref_ptr<osgGA::GUIEventHandler> eHandler,
                  EventType type) :
    eventHandler(eHandler), eType(type), flag(false)
  {
    init(type, "osgGA::GUIEventHandler");
  }

  ViewerEventData(osg::Group* parent_, std::string childName_, EventType type) :
    parent(parent_), childName(childName_), eType(type), flag(false)
  {
    init(type, "osgGA::GUIEventHandler");
  }

  ViewerEventData(EventType type, bool enable) : eType(type), flag(enable)
  {
    init(type, "No arguments");
  }

  void init(EventType type, std::string comment)
  {
    this->eType = type;
    RLOG(5, "Creating ViewerEventData %d: %s", userEventCount, comment.c_str());
    userEventCount++;
  }

  ~ViewerEventData()
  {
    userEventCount--;
    RLOG(5, "Deleting ViewerEventData - now %d events", userEventCount);
  }


  osg::ref_ptr<osg::Group> parent;
  osg::ref_ptr<osg::Node> node;
  std::string childName;
  osg::ref_ptr<osgGA::GUIEventHandler> eventHandler;
  EventType eType;
  HTr trf;
  bool flag;
  static int userEventCount;
};

int Rcs::ViewerEventData::userEventCount = 0;

/*******************************************************************************
 * Keyboard handler for default keys.
 ******************************************************************************/
class KeyHandler : public osgGA::GUIEventHandler
{
public:

  KeyHandler(Rcs::Viewer* viewer) : _viewer(viewer), _video_capture_process(-1)
  {
    RCHECK(_viewer);

    KeyCatcherBase::registerKey("0-9", "Set Rcs debug level", "Viewer");
    KeyCatcherBase::registerKey("w", "Toggle wireframe mode", "Viewer");
    KeyCatcherBase::registerKey("s", "Cycle between shadow modes", "Viewer");
    KeyCatcherBase::registerKey("R", "Toggle cartoon mode", "Viewer");
#if !defined(_MSC_VER)
    KeyCatcherBase::registerKey("M", "Toggle video capture", "Viewer");
#endif
    KeyCatcherBase::registerKey("F11", "Print camera transform", "Viewer");
    KeyCatcherBase::registerKey("j", "Print 3d coordinates under mouse", "Viewer");
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
    return _viewer->handle(ea, aa);
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
        cmd << "ffmpeg -y -f x11grab -r 25 -s "
            << w << "x" << h
            << " -i " << getenv("DISPLAY") << "+"
            << x << "," << y
            << " -crf 20 -r 25 -c:v libx264 -c:a n"
            << " /tmp/movie_" << movie_number++ << ".mp4";

        // cmd << "avconv -y -f x11grab -r 25 -s "
        //     << w << "x" << h
        //     << " -i " << getenv("DISPLAY") << "+"
        //     << x << "," << y
        //     << " -crf 20 -r 25 -c:v libx264 -c:a n"
        //     << " /tmp/movie_" << movie_number++ << ".mp4";

        _video_capture_process = forkProcess(cmd.str().c_str());
        captureRunning = true;
      }
    }
#endif

    return captureRunning;
  }

private:

  Rcs::Viewer* _viewer;
  pid_t _video_capture_process;
};

/*******************************************************************************
 * Viewer class.
 ******************************************************************************/
Viewer::Viewer() :
  fps(0.0), mouseX(0.0), mouseY(0.0), normalizedMouseX(0.0),
  normalizedMouseY(0.0), mtxFrameUpdate(NULL), threadRunning(false),
  updateFreq(25.0), initialized(false), wireFrame(false), shadowsEnabled(false),
  llx(0), lly(0), sizeX(640), sizeY(480), cartoonEnabled(false),
  threadStopped(true), leftMouseButtonPressed(false),
  rightMouseButtonPressed(false), pauseFrameUpdates(false)
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
  fps(0.0), mouseX(0.0), mouseY(0.0), normalizedMouseX(0.0),
  normalizedMouseY(0.0), mtxFrameUpdate(NULL), threadRunning(false),
  updateFreq(25.0), initialized(false), wireFrame(false), shadowsEnabled(false),
  llx(0), lly(0), sizeX(640), sizeY(480), cartoonEnabled(false),
  threadStopped(true), leftMouseButtonPressed(false),
  rightMouseButtonPressed(false), pauseFrameUpdates(false)
{
  create(fancy, startupWithShadow);

  RLOG(5, "Done constructor of viewer");
}

/*******************************************************************************
 * Destructor. \todo: Explain why we are explicitely releasing the viewer here.
 * Has something to do with the errors we get from failure of releasing graphics
 * contexts.
 ******************************************************************************/
Viewer::~Viewer()
{
  stopUpdateThread();
#if defined (_MSC_VER)
  viewer.release();
#endif
}

/*******************************************************************************
 * Initlalization method.
 ******************************************************************************/
void Viewer::create(bool fancy, bool startupWithShadow)
{
#if defined(_MSC_VER)
  llx = 12;
  lly = 31;
#endif

  const char* forceSimple = getenv("RCSVIEWER_SIMPLEGRAPHICS");

  if (forceSimple)
  {
    fancy = false;
    startupWithShadow = false;
  }

  this->shadowsEnabled = startupWithShadow;

  // Rotate loaded file nodes to standard coordinate conventions
  // (z: up, x: forward)
  osg::ref_ptr<osgDB::ReaderWriter::Options> options;
  options = new osgDB::ReaderWriter::Options;
  options->setOptionString("noRotation");
  osgDB::Registry::instance()->setOptions(options.get());

  // osg::GraphicsContext::WindowingSystemInterface* wsi =
  //   osg::GraphicsContext::getWindowingSystemInterface();
  // wsi->setScreenResolution(osg::GraphicsContext::ScreenIdentifier(0), 800, 600);

  this->viewer = new osgViewer::Viewer();

  // Mouse manipulator (needs to go before event handler)
  osg::ref_ptr<osgGA::TrackballManipulator> trackball = new RcsManipulator();
  viewer->setCameraManipulator(trackball.get());

  // Handle some default keys (see handler above)
  this->keyHandler = new KeyHandler(this);
  viewer->addEventHandler(this->keyHandler.get());

  // Root node (instead of a Group we create an Cartoon node for optional
  // cell shading)
  if (fancy)
  {
    this->rootnode = new osgFX::Cartoon;
    dynamic_cast<osgFX::Effect*>(rootnode.get())->setEnabled(false);
  }
  else
  {
    this->rootnode = new osg::Group;
  }

  rootnode->setName("rootnode");

  // Light grayish green universe
  this->clearNode = new osg::ClearNode;
  this->clearNode->setClearColor(colorFromString("LIGHT_GRAYISH_GREEN"));
  this->rootnode->addChild(this->clearNode.get());

  // Light model: We switch off the default viewer light, and configure two
  // light sources. The sunlight shines down from 10m. Another light source
  // moves with the camera, so that there are no dark spots whereever
  // the mouse manipulator moves to.

  if (fancy)
  {
    // Disable default light
    rootnode->getOrCreateStateSet()->setMode(GL_LIGHT0, osg::StateAttribute::OFF);

    // Light source that moves with the camera
    this->cameraLight = new osg::LightSource;
    cameraLight->getLight()->setLightNum(1);
    cameraLight->getLight()->setPosition(osg::Vec4(0.0, 0.0, 10.0, 1.0));
    cameraLight->getLight()->setSpecular(osg::Vec4(1.0, 1.0, 1.0, 1.0));
    rootnode->addChild(cameraLight.get());
    rootnode->getOrCreateStateSet()->setMode(GL_LIGHT1, osg::StateAttribute::ON);
  }

  // Shadow map scene. We use the sunlight to case shadows.
  this->shadowScene = new osgShadow::ShadowedScene;
  osg::ref_ptr<osgShadow::ShadowMap> sm = new osgShadow::ShadowMap;
  sm->setTextureSize(osg::Vec2s(2048, 2048));
  if (fancy)
  {
    // Light source that shines down
    osg::ref_ptr<osg::LightSource> sunlight = new osg::LightSource;
    sunlight->getLight()->setLightNum(2);
    sunlight->getLight()->setPosition(osg::Vec4(0.0, 0.0, 10.0, 1.0));
    rootnode->addChild(sunlight.get());
    rootnode->getOrCreateStateSet()->setMode(GL_LIGHT2, osg::StateAttribute::ON);
    sm->setLight(sunlight->getLight());
  }
  sm->setPolygonOffset(osg::Vec2(-0.7, 0.0));
  sm->setAmbientBias(osg::Vec2(0.7, 0.3));   // values need to sum up to 1.0

  shadowScene->setShadowTechnique(sm.get());
  shadowScene->addChild(rootnode.get());
  shadowScene->setReceivesShadowTraversalMask(ReceivesShadowTraversalMask);
  shadowScene->setCastsShadowTraversalMask(CastsShadowTraversalMask);


  // Change the threading model. The default threading model is
  // osgViewer::Viewer::CullThreadPerCameraDrawThreadPerContext.
  if (forceSimple)
  {
    viewer->setThreadingModel(osgViewer::Viewer::SingleThreaded);
  }
  else
  {
    viewer->setThreadingModel(osgViewer::Viewer::CullDrawThreadPerContext);
  }

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
    viewer->setSceneData(startupWithShadow?shadowScene.get():rootnode.get());
  }

  // Disable small feature culling to avoid problems with drawing single points
  // as they have zero bounding box size
  viewer->getCamera()->setCullingMode(viewer->getCamera()->getCullingMode() &
                                      ~osg::CullSettings::SMALL_FEATURE_CULLING);

  viewer->getCameraManipulator()->setHomePosition(osg::Vec3d(4.0,  3.5, 3.0),
                                                  osg::Vec3d(0.0, -0.2, 0.8),
                                                  osg::Vec3d(0.0, 0.05, 1.0));
  viewer->home();

  KeyCatcherBase::registerKey("F10", "Toggle full screen", "Viewer");
  KeyCatcherBase::registerKey("F5", "Full screen resolution down", "Viewer");
  KeyCatcherBase::registerKey("F6", "Full screen resolution up", "Viewer");
  osg::ref_ptr<osgViewer::WindowSizeHandler> wsh = new osgViewer::WindowSizeHandler;
  wsh->setKeyEventToggleFullscreen(osgGA::GUIEventAdapter::KEY_F10);
  wsh->setKeyEventWindowedResolutionDown(osgGA::GUIEventAdapter::KEY_F5);
  wsh->setKeyEventWindowedResolutionUp(osgGA::GUIEventAdapter::KEY_F6);
  viewer->addEventHandler(wsh.get());

  KeyCatcherBase::registerKey("F9", "Toggle continuous screenshots", "Viewer");
  KeyCatcherBase::registerKey("F8", "Take screenshot(s)", "Viewer");

  osg::ref_ptr<osgViewer::ScreenCaptureHandler::WriteToFile> scrw;
  scrw = new osgViewer::ScreenCaptureHandler::WriteToFile("screenshot", "png");

  osg::ref_ptr<osgViewer::ScreenCaptureHandler> capture;
  capture = new osgViewer::ScreenCaptureHandler(scrw.get());
  capture->setKeyEventToggleContinuousCapture(osgGA::GUIEventAdapter::KEY_F9);
  capture->setKeyEventTakeScreenShot(osgGA::GUIEventAdapter::KEY_F8);
  viewer->addEventHandler(capture.get());

  KeyCatcherBase::registerKey("z", "Toggle on-screen stats", "Viewer");
  KeyCatcherBase::registerKey("Z", "Print viewer stats to console", "Viewer");
  osg::ref_ptr<osgViewer::StatsHandler> stats = new osgViewer::StatsHandler;
  stats->setKeyEventTogglesOnScreenStats('z');
  stats->setKeyEventPrintsOutStats('Z');
  viewer->addEventHandler(stats.get());


  setTitle("RcsViewer");
}

/*******************************************************************************
 * Add a node to the root node.
 ******************************************************************************/
bool Viewer::setWindowSize(unsigned int llx_,     // lower left x
                           unsigned int lly_,     // lower left y
                           unsigned int sizeX_,   // size in x-direction
                           unsigned int sizeY_)
{
  if (isInitialized() == true)
  {
    RLOG(1, "The window size can't be changed after launching the viewer "
         "window");
    return false;
  }

  this->llx = llx_;
  this->lly = lly_;
  this->sizeX = sizeX_;
  this->sizeY = sizeY_;

  return true;
}

/*******************************************************************************
 * \ţodo: In case the viewer is about to be realized, we might get into the
 *        realized==false branch. If it then gets realized, we get a
 *        concurrency problem. Can this ever happen? Does it make sense to
 *        handle this?
 ******************************************************************************/
void Viewer::add(osgGA::GUIEventHandler* eventHandler)
{
  RLOG(5, "Adding event handler");
  addUserEvent(new ViewerEventData(eventHandler, ViewerEventData::AddEventHandler));
}

/*******************************************************************************
 * Add a node to the root node.
 ******************************************************************************/
void Viewer::add(osg::Node* node)
{
  RLOG(5, "Adding node %s to eventqueue", node->getName().c_str());
  osg::ref_ptr<osg::Node> refNode(node);
  addUserEvent(new ViewerEventData(refNode, ViewerEventData::AddNode));
}

/*******************************************************************************
 * Add a node to the root node.
 ******************************************************************************/
void Viewer::setEnableFrameUpdates(bool enable)
{
  RLOG(5, "%s frame updates", enable ? "Enabling" : "Disabling");
  addUserEvent(new ViewerEventData(ViewerEventData::SetFrameUpdatesEnabled, enable));
}

/*******************************************************************************
 * Add a node to the root node.
 ******************************************************************************/
bool Viewer::addInternal(osg::Node* node)
{
  osg::Camera* newHud = dynamic_cast<osg::Camera*>(node);

  // If it's a camera, it needs a graphics context. This doesn't exist right
  // after construction, therefore in that case we ignore it. This shouldn't
  // happen here, since this function gets called from within the viewer's
  // frame traversals.
  if (newHud)
  {
    osgViewer::Viewer::Windows windows;
    viewer->getWindows(windows);

    if (windows.empty())
    {
      RLOG(1, "Failed to add HUD - window not created");
    }
    else
    {
      newHud->setGraphicsContext(windows[0]);
      newHud->setViewport(0, 0, windows[0]->getTraits()->width,
                          windows[0]->getTraits()->height);
      viewer->addSlave(newHud, false);
    }

    return true;
  }

  bool success = false;

  if (node != NULL)
  {
    success = this->rootnode->addChild(node);
  }
  else
  {
    RLOG(1, "Failed to add osg::Node - node is NULL!");
  }

  return success;
}

/*******************************************************************************
 * Add a node to the parent node.
 ******************************************************************************/
void Viewer::add(osg::Group* parent, osg::Node* child)
{
  addUserEvent(new ViewerEventData(parent, child, ViewerEventData::AddChildNode));
}

/*******************************************************************************
 * Add a node to the parent node.
 ******************************************************************************/
bool Viewer::addInternal(osg::Group* parent, osg::Node* child)
{
  parent->addChild(child);
  return true;
}

/*******************************************************************************
 * Removes a node from the scene graph.
 ******************************************************************************/
void Viewer::removeNode(osg::Node* node)
{
  addUserEvent(new ViewerEventData(node, ViewerEventData::RemoveNode));
}

/*******************************************************************************
 * Removes a node from the scene graph.
 ******************************************************************************/
void Viewer::removeNode(std::string nodeName)
{
  RLOG_CPP(5, "Removing node " << nodeName);
  addUserEvent(new ViewerEventData(nodeName, ViewerEventData::RemoveNamedNode));
}

/*******************************************************************************
 * Removes all nodes with a given name from a parent node.
 ******************************************************************************/
void Viewer::removeNode(osg::Group* parent, std::string child)
{
  RLOG_CPP(5, "Removing node " << child << " of parent " << parent->getName());
  addUserEvent(new ViewerEventData(parent, child, ViewerEventData::RemoveChildNode));
}

/*******************************************************************************
 *
 ******************************************************************************/
void Viewer::removeNodes()
{
  addUserEvent(new ViewerEventData(ViewerEventData::RemoveAllNodes));
}

/*******************************************************************************
 *
 ******************************************************************************/
void Viewer::setCameraTransform(const HTr* A_CI)
{
  addUserEvent(new ViewerEventData(A_CI, ViewerEventData::SetCameraTransform));
}

/*******************************************************************************
 *
 ******************************************************************************/
void Viewer::setCameraTransform(double x, double y, double z,
                                double thx, double thy, double thz)
{
  HTr A_CI;
  double x6[6];
  VecNd_set6(x6, x, y, z, thx, thy, thz);
  HTr_from6DVector(&A_CI, x6);
  setCameraTransform(&A_CI);
}

/*******************************************************************************
 * Removes all nodes with the given name from the rootNode
 ******************************************************************************/
int Viewer::removeInternal(std::string nodeName)
{
  int nnd = 0;
  osg::Node* ndi;

  do
  {
    ndi = getNode(nodeName);
    if (ndi)
    {
      bool success = removeInternal(ndi);

      if (success)
      {
        nnd++;
      }
      else
      {
        RLOG(4, "Failed to remove node %s at iteration %d",
             nodeName.c_str(), nnd);
      }
    }

  }
  while (ndi);

  RLOG(5, "Removed %d nodes with name %s from the viewer",
       nnd, nodeName.c_str());

  return nnd;
}

/*******************************************************************************
 * Removes a node from the scene graph.
 ******************************************************************************/
bool Viewer::removeInternal(osg::Node* node)
{
  if (node == NULL)
  {
    RLOG(1, "Node is NULL - can't be deleted");
    return false;
  }

  osg::Camera* hud = dynamic_cast<osg::Camera*>(node);

  if (hud != NULL)
  {
    osg::View::Slave* slave = viewer->findSlaveForCamera(hud);

    if (slave != NULL)
    {
      RLOG(4, "Hud can't be deleted - is not part of the scene graph");
      return false;
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
      return false;
    }

    return true;
  }

  osg::Node::ParentList parents = node->getParents();
  size_t nDeleted = 0;

  for (size_t i=0; i<parents.size(); ++i)
  {
    nDeleted++;
    parents[i]->removeChild(node);
  }

  if (nDeleted == 0)
  {
    RLOG(1, "Node can't be deleted - is not part of the scene graph");
    return false;
  }

  return true;
}

/*******************************************************************************
 * Search through the parent node. We do this in a while loop to remove all
 * nodes with the same name
 ******************************************************************************/
int Viewer::removeInternal(osg::Node* parent, std::string nodeName)
{
  osg::Node* toRemove = findNamedNodeRecursive(parent, nodeName);
  int nnd = 0;

  while (toRemove)
  {
    removeInternal(toRemove);
    toRemove = findNamedNodeRecursive(parent, nodeName);
    nnd++;
  }

  return nnd;
}

/*******************************************************************************
 *
 ******************************************************************************/
int Viewer::removeAllNodesInternal()
{
  int nDeleted = rootnode->getNumChildren();
  rootnode->removeChildren(0, nDeleted);
  this->rootnode->addChild(this->clearNode.get());
  RLOG_CPP(5, "Removing all " << nDeleted << " nodes");

  return nDeleted;
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
 *
 ******************************************************************************/
void Viewer::resetView()
{
  addUserEvent(new ViewerEventData(ViewerEventData::ResetView));
}

/*******************************************************************************
 * Sets the camera position and viewing direction
 * First vector is where camera is, Second vector is where the
 * camera points to, the up vector is set internally to always stay upright
 ******************************************************************************/
void Viewer::setCameraHomePosition(const HTr* A_CI)
{
  addUserEvent(new ViewerEventData(A_CI, ViewerEventData::SetCameraHomePose));
}

/*******************************************************************************
 * Sets the camera position and viewing direction
 * First vector is where camera is, Second vector is where the
 * camera points to, the up vector is set internally to always stay upright.
 * The eye, center and up vectors are stored in the rows of the transform's
 * rotation matrix.
 ******************************************************************************/
void Viewer::setCameraHomePosition(const osg::Vec3d& eye,
                                   const osg::Vec3d& center,
                                   const osg::Vec3d& up)
{
  HTr ecu;
  Vec3d_setZero(ecu.org);
  Vec3d_set(ecu.rot[0], eye[0], eye[1], eye[2]);
  Vec3d_set(ecu.rot[1], center[0], center[1], center[2]);
  Vec3d_set(ecu.rot[2], up[0], up[1], up[2]);
  addUserEvent(new ViewerEventData(&ecu, ViewerEventData::SetCameraHomePoseEyeCenterUp));
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
osg::Node* Viewer::getNodeUnderMouse(double I_mouseCoords[3])
{
  return Rcs::getNodeUnderMouse<osg::Node*>(*this->viewer.get(),
                                            mouseX, mouseY,
                                            I_mouseCoords);
}

/*******************************************************************************
 *
 ******************************************************************************/
osg::Node* Viewer::getNode(std::string nodeName)
{
  return findNamedNodeRecursive(rootnode, nodeName);
}

/*******************************************************************************
 * aspectRatio = width/height
 * OSG returns field of view in [degrees]. We convert it to SI units
 ******************************************************************************/
void Viewer::getFieldOfView(double& width, double& height) const
{
  double aspectRatio, zNear, zFar;
  viewer->getCamera()->getProjectionMatrixAsPerspective(width, aspectRatio,
                                                        zNear, zFar);
  width = RCS_DEG2RAD(width);
  height = width/aspectRatio;
}

/*******************************************************************************
 * OSG returns field of view in [degrees]. We convert it to SI units
 ******************************************************************************/
double Viewer::getFieldOfView() const
{
  double fovy, aspectRatio, zNear, zFar;
  viewer->getCamera()->getProjectionMatrixAsPerspective(fovy, aspectRatio,
                                                        zNear, zFar);
  return RCS_DEG2RAD(fovy);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Viewer::setFieldOfView(double fovy)
{
  double fovyOld, aspectRatio, zNear, zFar;
  fovy = RCS_RAD2DEG(fovy);
  viewer->getCamera()->getProjectionMatrixAsPerspective(fovyOld, aspectRatio,
                                                        zNear, zFar);
  viewer->getCamera()->setProjectionMatrixAsPerspective(fovy, aspectRatio,
                                                        zNear, zFar);
}

/*******************************************************************************
 * Defaults are:
 * fov_org = 29.148431   aspectRatio_org = 1.333333
 * znear=1.869018   zfar=10.042613
 ******************************************************************************/
void Viewer::setFieldOfView(double fovWidth, double fovHeight)
{
  double fovyOld, aspectRatio, zNear, zFar;
  fovWidth = RCS_RAD2DEG(fovWidth);
  fovHeight = RCS_RAD2DEG(fovHeight);
  viewer->getCamera()->getProjectionMatrixAsPerspective(fovyOld, aspectRatio,
                                                        zNear, zFar);
  viewer->getCamera()->setProjectionMatrixAsPerspective(fovWidth,
                                                        fovWidth/fovHeight,
                                                        zNear, zFar);
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

  RLOG(5, "Exiting frame thread");

  return NULL;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Viewer::runInThread(pthread_mutex_t* mutex)
{
  this->mtxFrameUpdate = mutex;
  threadStopped = false;
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
  addUserEvent(new ViewerEventData(ViewerEventData::SetWireframeEnabled, wf));
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
  addUserEvent(new ViewerEventData(ViewerEventData::SetShadowEnabled, enable));
}

/*******************************************************************************
 * Renders the scene in cartoon mode.
 ******************************************************************************/
void Viewer::setCartoonEnabled(bool enable)
{
  addUserEvent(new ViewerEventData(ViewerEventData::SetCartoonEnabled, enable));
}

/*******************************************************************************
 *
 ******************************************************************************/
void Viewer::setBackgroundColor(const std::string& color)
{
  addUserEvent(new ViewerEventData(color, ViewerEventData::SetBackgroundColor));
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

  // Publish all queued events before the frame() call
  userEventMtx.lock();
  for (size_t i=0; i<userEventStack.size(); ++i)
  {
    getOsgViewer()->getEventQueue()->userEvent(userEventStack[i].get());
  }
  userEventStack.clear();
  userEventMtx.unlock();

  if (!pauseFrameUpdates)
  {
    lock();
    viewer->frame();
    unlock();
  }
  else
  {
    // Frame calls advance, eventTraversal, updateTraversal and
    // renderingTraversals. We skip the rendering.
    lock();
    viewer->advance();
    viewer->eventTraversal();
    viewer->updateTraversal();
    unlock();
  }

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

  this->startView = viewer->getCamera()->getProjectionMatrix();
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
void Viewer::stopUpdateThread()
{
  if (threadRunning == false)
  {
    return;
  }

  RLOG(5, "Joining thread");
  this->threadRunning = false;

  int res = pthread_join(frameThread, NULL);

  if (res!=0)
  {
    RLOG(1, "Error joining thread: %s (%d)", strerror(res), res);
  }
  else
  {
    RLOG(5, "... done joining thread");
  }

  threadStopped = true;
  this->initialized = false;
}

/*******************************************************************************
 *
 ******************************************************************************/
osg::ref_ptr<osgViewer::Viewer> Viewer::getOsgViewer() const
{
  return this->viewer;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Viewer::lock() const
{

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

/*******************************************************************************
 *
 ******************************************************************************/
void Viewer::getMouseTip(double tip[3]) const
{
  osg::Matrix vm = viewer->getCamera()->getViewMatrix();
  osg::Matrix pm = viewer->getCamera()->getProjectionMatrix();

  HTr A_CamI;
  getCameraTransform(&A_CamI);

  double planePt[3];
  Vec3d_add(planePt, A_CamI.org, A_CamI.rot[0]);

  Rcs::getMouseTip(vm, pm, normalizedMouseX, normalizedMouseY, planePt, tip);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Viewer::handleUserEvents(const osg::Referenced* userEvent)
{
  NLOG(5, "Received user event");

  const ViewerEventData* ev = dynamic_cast<const ViewerEventData*>(userEvent);
  if (!ev)
  {
    RLOG(5, "User event not of type ViewerEventData - skipping");
    return;
  }

  switch (ev->eType)
  {
    case ViewerEventData::AddNode:
      if (ev->node.valid())
      {
        RLOG(5, "Adding node \"%s\"", ev->node->getName().c_str());
        addInternal(ev->node.get());
      }
      else
      {
        RLOG(5, "ViewerEventData::AddNode: Found invalid node");
      }
      break;

    case ViewerEventData::AddChildNode:
      RCHECK(ev->parent.valid());
      RCHECK(ev->node.valid());
      RLOG(5, "Adding node \"%s\"", ev->node->getName().c_str());
      addInternal(ev->parent.get(), ev->node.get());
      break;

    case ViewerEventData::RemoveNode:
      RCHECK(ev->node.valid());
      RLOG(5, "Removing node \"%s\"", ev->node->getName().c_str());
      removeInternal(ev->node.get());
      break;

    case ViewerEventData::RemoveNamedNode:
      RLOG(5, "Removing all nodes with name \"%s\"", ev->childName.c_str());
      removeInternal(ev->childName);
      break;

    case ViewerEventData::RemoveChildNode:
      RCHECK(ev->parent.valid());
      RLOG(5, "Removing child node \"%s\" from parent %s",
           ev->childName.c_str(), ev->parent->getName().c_str());
      removeInternal(ev->parent.get(), ev->childName);
      break;

    case ViewerEventData::RemoveAllNodes:
      removeAllNodesInternal();
      break;

    case ViewerEventData::AddEventHandler:
      RCHECK(ev->eventHandler.valid());
      RLOG(5, "Adding handler \"%s\"", ev->eventHandler->getName().c_str());
      viewer->addEventHandler(ev->eventHandler.get());
      break;

    case ViewerEventData::SetCameraTransform:
      RLOG(5, "Setting camera transform");
      viewer->getCameraManipulator()->setByInverseMatrix(viewMatrixFromHTr(&ev->trf));
      break;

    case ViewerEventData::SetCameraHomePose:
    {
      RLOG(5, "Setting camera home pose");
      const HTr* A_CI = &ev->trf;
      osg::Vec3d eye(A_CI->org[0], A_CI->org[1], A_CI->org[2]);

      osg::Vec3d center(A_CI->org[0] + A_CI->rot[2][0],
                        A_CI->org[1] + A_CI->rot[2][1],
                        A_CI->org[2] + A_CI->rot[2][2]);

      osg::Vec3d up(-A_CI->rot[1][0], -A_CI->rot[1][1], -A_CI->rot[1][2]);

      viewer->getCameraManipulator()->setHomePosition(eye, center, up);
      viewer->home();
    }
    break;

    // The eye, center and up vectors are stored in the rows of the transform's
    // rotation matrix.
    case ViewerEventData::SetCameraHomePoseEyeCenterUp:
    {
      RLOG(5, "Setting camera home pose from eye, center and up");
      const HTr* A_CI = &ev->trf;

      osg::Vec3d eye(A_CI->rot[0][0], A_CI->rot[0][1], A_CI->rot[0][2]);
      osg::Vec3d center(A_CI->rot[1][0], A_CI->rot[1][1], A_CI->rot[1][2]);
      osg::Vec3d up(A_CI->rot[2][0], A_CI->rot[2][1], A_CI->rot[2][2]);

      viewer->getCameraManipulator()->setHomePosition(eye, center, up);
      viewer->home();
    }
    break;

    // Sets the title of the viewer windows. We set the title for all windows,
    // but only one window should be created anyways
    case ViewerEventData::SetTitle:
    {
      RLOG_CPP(5, "Setting window title to" << ev->childName);
      osgViewer::ViewerBase::Windows windows;
      this->viewer->getWindows(windows);
      osgViewer::ViewerBase::Windows::iterator window;

      for (window = windows.begin(); window != windows.end(); window++)
      {
        (*window)->setWindowName(ev->childName);
      }
    }
    break;

    case ViewerEventData::ResetView:
      RLOG(5, "Resetting view");
      viewer->getCamera()->setProjectionMatrix(this->startView);
      break;

    case ViewerEventData::SetBackgroundColor:
      RLOG_CPP(5, "Setting background color to" << ev->childName);
      this->clearNode->setClearColor(colorFromString(ev->childName.c_str()));
      break;

    case ViewerEventData::SetShadowEnabled:
    {
      RLOG(5, "Setting shadows to %s", ev->flag ? "TRUE" : "FALSE");
      osg::Matrix lastViewMatrix = viewer->getCameraManipulator()->getMatrix();

      if (ev->flag==false)
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
    break;

    case ViewerEventData::SetWireframeEnabled:
    {
      RLOG(5, "Setting wireframe to %s", ev->flag ? "TRUE" : "FALSE");
      this->wireFrame = ev->flag;
      osg::ref_ptr<osg::StateSet> sSet = rootnode->getOrCreateStateSet();

      if (ev->flag == true)
      {
        sSet->setAttribute(new osg::PolygonMode
                           (osg::PolygonMode::FRONT_AND_BACK,
                            osg::PolygonMode::LINE));
      }
      else
      {
        sSet->setAttribute(new osg::PolygonMode
                           (osg::PolygonMode::FRONT_AND_BACK,
                            osg::PolygonMode::FILL));
      }

    }
    break;

    case ViewerEventData::SetCartoonEnabled:
    {
      RLOG(5, "Setting cartoon mode to %s", ev->flag ? "TRUE" : "FALSE");
      osgFX::Effect* cartoon = dynamic_cast<osgFX::Effect*>(rootnode.get());

      if (!cartoon)
      {
        return;
      }

      // Disable shadows when switching to cartoon mode
      if ((ev->flag==true) && (this->shadowsEnabled==true))
      {
        viewer->setSceneData(rootnode.get());
        this->shadowsEnabled = false;
      }

      cartoon->setEnabled(ev->flag);
    }
    break;

    case ViewerEventData::SetTrackballCenter:
    {
      RLOG(5, "Setting trackball center to %f %f %f",
           ev->trf.org[0], ev->trf.org[1], ev->trf.org[2]);
      osgGA::TrackballManipulator* trackball =
        dynamic_cast<osgGA::TrackballManipulator*>(viewer->getCameraManipulator());

      if (trackball)
      {
        const double* cntr = ev->trf.org;
        trackball->setCenter(osg::Vec3(cntr[0], cntr[1], cntr[2]));
      }
    }
    break;

    case ViewerEventData::SetFrameUpdatesEnabled:
    {
      this->pauseFrameUpdates = !ev->flag;
      RLOG(5, "pauseFrameUpdates is %s", this->pauseFrameUpdates ? "TRUE" : "FALSE");
    }
    break;

    default:
      RLOG(1, "Unknown event type %d", (int) ev->eType);
      break;
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
bool Viewer::handle(const osgGA::GUIEventAdapter& ea,
                    osgGA::GUIActionAdapter& aa)
{
  switch (ea.getEventType())
  {

    /////////////////////////////////////////////////////////////////
    // User events triggered through classes API
    /////////////////////////////////////////////////////////////////
    case osgGA::GUIEventAdapter::USER:
    {
      handleUserEvents(ea.getUserData());
      break;
    }

    /////////////////////////////////////////////////////////////////
    // Gets called once viewer window is closed. We then leave the
    // viewer's thread so that no more rendering is performed.
    /////////////////////////////////////////////////////////////////
    case (osgGA::GUIEventAdapter::CLOSE_WINDOW):
    {
      // \todo: Check if this is a problem die to dead-locking when
      //        running threaded
      stopUpdateThread();
      break;
    }

    /////////////////////////////////////////////////////////////////
    // Frame update event
    /////////////////////////////////////////////////////////////////
    case (osgGA::GUIEventAdapter::FRAME):
    {
      this->mouseX = ea.getX();
      this->mouseY = ea.getY();
      this->normalizedMouseX = ea.getXnormalized();
      this->normalizedMouseY = ea.getYnormalized();

      if (cameraLight.valid())
      {
        HTr A_CI;
        getCameraTransform(&A_CI);
        osg::Vec4 lightpos;
        lightpos.set(A_CI.org[0], A_CI.org[1], A_CI.org[2] + 0 * 2.0, 1.0f);
        cameraLight->getLight()->setPosition(lightpos);
      }
      break;
    }

    /////////////////////////////////////////////////////////////////
    // Mouse button pressed events.
    /////////////////////////////////////////////////////////////////
    case (osgGA::GUIEventAdapter::PUSH):
    {
      // Left mouse button pressed
      if (ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
      {
        this->leftMouseButtonPressed = true;

        if (this->rightMouseButtonPressed)
        {
          double center[3] = {0.0, 0.0, 0.0};;
          osg::Node* click = getNodeUnderMouse(center);
          if (click)
          {
            setTrackballCenter(center[0], center[1], center[2]);
          }
        }
      }
      // Right mouse button pressed
      else if (ea.getButton() == osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON)
      {
        this->rightMouseButtonPressed = true;
      }

      break;
    }

    /////////////////////////////////////////////////////////////////
    // Mouse button released events.
    /////////////////////////////////////////////////////////////////
    case (osgGA::GUIEventAdapter::RELEASE):
    {
      // Left mouse button released.
      if (ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
      {
        this->leftMouseButtonPressed = false;
      }

      // Right mouse button released.
      if (ea.getButton() == osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON)
      {
        this->rightMouseButtonPressed = false;
      }

      break;
    }

    /////////////////////////////////////////////////////////////////
    // Key pressed events
    /////////////////////////////////////////////////////////////////
    case (osgGA::GUIEventAdapter::KEYDOWN):
    {
      // key '0' is ASCII code 48, then running up to 57 for '9'
      if ((ea.getKey() >= 48) && (ea.getKey() <= 57))
      {
        unsigned int dLev = ea.getKey() - 48;
        if (dLev<9)
        {
          RcsLogLevel = dLev;
        }
        else
        {
          RcsLogLevel = -1;
        }
        RMSG("Setting debug level to %u", dLev);
        return false;
      }

      else if (ea.getKey() == osgGA::GUIEventAdapter::KEY_F11)
      {
        HTr A_CI;
        double x[6];
        this->getCameraTransform(&A_CI);
        HTr_to6DVector(x, &A_CI);
        RMSGS("Camera pose is %f %f %f   %f %f %f   (degrees: %.3f %.3f %.3f)",
              x[0], x[1], x[2], x[3], x[4], x[5],
              RCS_RAD2DEG(x[3]), RCS_RAD2DEG(x[4]), RCS_RAD2DEG(x[5]));
      }

      //
      // Toggle wireframe
      //
      else if (ea.getKey() == 'w')
      {
        toggleWireframe();
        return false;
      }

      //
      // Toggle shadows
      //
      else if (ea.getKey() == 's')
      {
        setShadowEnabled(!this->shadowsEnabled);
        return false;
      }

      //
      // Toggle cartoon mode
      //
      else if (ea.getKey() == 'R')
      {
        this->cartoonEnabled = !this->cartoonEnabled;
        setCartoonEnabled(this->cartoonEnabled);
        return false;
      }

      //
      // Toggle cartoon mode
      //
      else if (ea.getKey() == 'M')
      {
        keyHandler->toggleVideoCapture();
        return false;
      }

      //
      // Print pick coordinates to console
      //
      else if (ea.getKey() == 'j')
      {
        double pt[3];
        osg::Node* nd = getNodeUnderMouse(pt);
        if (nd) RMSG("%s [%.5f   %.5f   %.5f]", nd->getName().c_str(),
                       pt[0], pt[1], pt[2]);
        return false;
      }

      break;
    }   // case(osgGA::GUIEventAdapter::KEYDOWN):

    default:
      break;

  }   // switch(ea.getEventType())

  return false;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Viewer::setTrackballCenter(double x, double y, double z)
{
  HTr cntr;
  HTr_setIdentity(&cntr);
  Vec3d_set(cntr.org, x, y, z);

  addUserEvent(new ViewerEventData(&cntr, ViewerEventData::SetTrackballCenter));
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Viewer::getTrackballCenter(double pos[3]) const
{
  osgGA::TrackballManipulator* trackball =
    dynamic_cast<osgGA::TrackballManipulator*>(viewer->getCameraManipulator());

  if (trackball)
  {
    osg::Vec3d tbCenter = trackball->getCenter();
    Vec3d_set(pos, tbCenter.x(), tbCenter.y(), tbCenter.z());
    return true;
  }

  return false;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Viewer::isThreadStopped() const
{
  return this->threadStopped;
}

/*******************************************************************************
 *
 ******************************************************************************/
double Viewer::getFPS() const
{
  return this->fps;
}

/*******************************************************************************
 *
 ******************************************************************************/
std::string Viewer::getDefaultBackgroundColor() const
{
  return defaultBgColor;
}

/*******************************************************************************
 * Add a node to the root node.
 ******************************************************************************/
void Viewer::setTitle(const std::string& title)
{
  addUserEvent(new ViewerEventData(title, ViewerEventData::SetTitle));
}

/*******************************************************************************
 * This might run concurrent with the frame's appending of events to the event
 * queue.
 ******************************************************************************/
void Viewer::addUserEvent(osg::Referenced* userEvent)
{
  osg::ref_ptr<osg::Referenced> ev(userEvent);
  userEventMtx.lock();
  userEventStack.push_back(ev);
  userEventMtx.unlock();
}

}   // namespace Rcs
