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

#include "TargetSetter.h"
#include "RcsViewer.h"

#include <KeyCatcherBase.h>
#include <Rcs_Mat3d.h>
#include <Rcs_macros.h>
#include <Rcs_resourcePath.h>

#include <osg/ShapeDrawable>
#include <osg/Material>
#include <osg/CoordinateSystemNode>
#include <osg/PositionAttitudeTransform>

#include <iostream>
#include <cstdio>



namespace Rcs
{



/*******************************************************************************
 * Helper class for dragger events.
 ******************************************************************************/
class TargetHandler : public osgGA::GUIEventHandler
{
  friend class TargetSetter;

public:

  unsigned int mode;
  osgManipulator::Dragger* activeDragger;
  osgManipulator::PointerInfo pointer;
  TargetSetter* ts;

  enum Modes
  {
    VIEW = 0,
    PICK = 1
  };

  TargetHandler(TargetSetter* node): osgGA::GUIEventHandler(),
    mode(VIEW), activeDragger(NULL),
    ts(node)
  {
    KeyCatcherBase::registerKey("Tab", "Show target setter node", "Viewer");
  }

  virtual bool handle(const osgGA::GUIEventAdapter& ea,
                      osgGA::GUIActionAdapter& aa)
  {
    osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);

    if (!view)
    {
      RLOG(1, "view is NULL - doing nothing");
      return false;
    }

    if ((ea.getKey() == osgGA::GUIEventAdapter::KEY_Tab)  &&
        (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN) &&
        (this->activeDragger==NULL))
    {
      // if the tab key is pressed and hold, then the target
      // node's dragger is active
      this->mode = PICK;

      // let the target node be visible
      ts->setAllChildrenOn();
    }

    ts->dragger->setMatrix(ts->getTransform());
    ts->updateText();

    // if the mode is VIEW, then update the tracker transformation with
    // joint values
    if (this->mode == VIEW)
    {
      return false;
    }

    switch (ea.getEventType())
    {
      case osgGA::GUIEventAdapter::PUSH:
      {
        osgUtil::LineSegmentIntersector::Intersections intersections;

        this->pointer.reset();

        if (view->computeIntersections(ea.getX(),ea.getY(),intersections))
        {
          this->pointer.setCamera(view->getCamera());
          this->pointer.setMousePosition(ea.getX(), ea.getY());

          for (osgUtil::LineSegmentIntersector::Intersections::iterator hitr = intersections.begin(); hitr != intersections.end(); ++hitr)
          {
            this->pointer.addIntersection(hitr->nodePath, hitr->getLocalIntersectPoint());
          }

          for (osg::NodePath::iterator itr =
                 this->pointer._hitList.front().first.begin();
               itr != this->pointer._hitList.front().first.end();
               ++itr)
          {
            osgManipulator::Dragger* dragger =
              dynamic_cast<osgManipulator::Dragger*>(*itr);

            if (dragger == ts->dragger.get())
            {
              osg::NotifySeverity nl = osg::getNotifyLevel();
              osg::setNotifyLevel(osg::FATAL);
              dragger->handle(this->pointer, ea, aa);
              osg::setNotifyLevel(nl);
              this->activeDragger = dragger;
              break;
            }
          }
        }

        return true;
      }

      case osgGA::GUIEventAdapter::DRAG:
      {

        if ((this->activeDragger==ts->dragger.get()) && (ts->dragger!=NULL))
        {
          this->pointer._hitIter = this->pointer._hitList.begin();
          this->pointer.setCamera(view->getCamera());
          this->pointer.setMousePosition(ea.getX(), ea.getY());

          osg::NotifySeverity nl = osg::getNotifyLevel();
          osg::setNotifyLevel(osg::FATAL);
          this->activeDragger->handle(this->pointer, ea, aa);
          osg::setNotifyLevel(nl);
          ts->updateValues(this->activeDragger->getMatrix());
          ts->updateText();
        }

        return true;
      }

      case osgGA::GUIEventAdapter::RELEASE:
      {
        if ((this->activeDragger==ts->dragger.get()) && (ts->dragger!=NULL))
        {
          this->pointer._hitIter = this->pointer._hitList.begin();
          this->pointer.setCamera(view->getCamera());
          this->pointer.setMousePosition(ea.getX(), ea.getY());

          osg::NotifySeverity nl = osg::getNotifyLevel();
          osg::setNotifyLevel(osg::FATAL);
          this->activeDragger->handle(this->pointer, ea, aa);
          osg::setNotifyLevel(nl);
          ts->updateValues(this->activeDragger->getMatrix());
          ts->updateText();
        }
        break;
      }

      default:
        break;
    }

    if (ea.getEventType() == osgGA::GUIEventAdapter::RELEASE)
    {
      this->activeDragger = NULL;
      this->pointer.reset();
    }

    if ((ea.getKey() ==osgGA::GUIEventAdapter::KEY_Tab)  &&
        (ea.getEventType() == osgGA::GUIEventAdapter::KEYUP))
    {
      this->activeDragger = NULL;
      this->mode = VIEW; // if tab key is released, switch to normal mode
      ts->setAllChildrenOff(); // hide the target node
    }

    return false;
  }

};

/*******************************************************************************
 * Constructor for Euler angle tracking.
 ******************************************************************************/
TargetSetter::TargetSetter(double posPtr_[3], double angPtr_[3],
                           double size, bool withSphericalTracker_) :
  osg::Switch(),
  posPtr(posPtr_),
  angPtr(angPtr_),
  rmPtr(NULL),
  scale(size),
  withSphericalTracker(withSphericalTracker_)
{
  RLOG(5, "Creating Euler angle tracker");
  initGraphics();
}

/*******************************************************************************
 * Constructor for rotation matrix tracking.
 ******************************************************************************/
TargetSetter::TargetSetter(double posPtr_[3], double rmPtr_[3][3],
                           double size, bool withSphericalTracker_) :
  osg::Switch(),
  posPtr(posPtr_),
  angPtr(NULL),
  rmPtr((double*) rmPtr_),
  scale(size),
  withSphericalTracker(withSphericalTracker_)
{
  RLOG(5, "Creating rotation matrix tracker");
  initGraphics();
}

/*******************************************************************************
 * Constructor for rotation matrix tracking.
 ******************************************************************************/
TargetSetter::TargetSetter(double posPtr_[3], double size) :
  osg::Switch(),
  posPtr(posPtr_),
  angPtr(NULL),
  rmPtr(NULL),
  scale(size),
  withSphericalTracker(false)
{
  RLOG(5, "Creating position only tracker");
  initGraphics();
}

/*******************************************************************************
 * Constructor for rotation matrix tracking.
 ******************************************************************************/
TargetSetter::~TargetSetter()
{
}

/*******************************************************************************
 * Returns the GUIEventHandler, which is responsible for dragging the target
 *  node
 ******************************************************************************/
osgGA::GUIEventHandler* TargetSetter::getHandler() const
{
  return this->pickHandler.get();
}

/*******************************************************************************
 * Specifies a reference frame, for example for bodies with rigid_body_joints
 * that have parents
 ******************************************************************************/
void TargetSetter::setReferenceFrame(double pos[3], double rot[3][3])
{
  refFrame->setPosition(osg::Vec3(pos[0], pos[1], pos[2]));

  osg::Quat qA;
  qA.set(osg::Matrix(rot[0][0], rot[0][1], rot[0][2], 0.0,
                     rot[1][0], rot[1][1], rot[1][2], 0.0,
                     rot[2][0], rot[2][1], rot[2][2], 0.0,
                     0.0, 0.0, 0.0, 1.0));

  refFrame->setAttitude(qA);
}

/*******************************************************************************
 * Initialization function
 ******************************************************************************/
void TargetSetter::initGraphics()
{
  this->refFrame = new osg::PositionAttitudeTransform();

  // draw info text
  float radius = 0.01;
  _coord_text = new osgText::Text;

  char fontFile[256];
  bool fontFileFound = Rcs_getAbsoluteFileName("Vera.ttf", fontFile);

  if (fontFileFound==false)
  {
    fontFileFound = Rcs_getAbsoluteFileName("fonts/Vera.ttf", fontFile);
  }

  if (fontFileFound == true)
  {
    _coord_text->setFont(fontFile);
  }
  else
  {
    RLOG(4, "Couldn't find font file \"Vera.ttf\" in resource path");
    _coord_text->setCharacterSize(50);
  }





  _coord_text->setCharacterSize(radius*4);
  _coord_text->setPosition(osg::Vec3(-radius*2.0,radius*10.0, 0.0));
  _coord_text->setAxisAlignment(osgText::Text::SCREEN);
  _coord_text->setBackdropType(osgText::Text::OUTLINE);
  _coord_text->setDrawMode(osgText::Text::TEXT | osgText::Text::BOUNDINGBOX);

  updateText();

  // Geode holds text
  osg::ref_ptr<osg::Geode> geode = new osg::Geode;

  // Settings for always drawing text on top
  osg::ref_ptr<osg::StateSet> stateSet = new osg::StateSet();
  geode->setStateSet(stateSet.get());

  // Disable depth testing so geometry is drawn regardless of depth values
  // of geometry already drawn.
  stateSet->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);
  stateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

  // Need to make sure this geometry is drawn at last. RenderBins are handled
  // in numerical order so set bin number to 11
  stateSet->setRenderBinDetails(11, "RenderBin");

  geode->addDrawable(_coord_text.get());

  // Add dragger
  this->cmdMgr = new osgManipulator::CommandManager;
  osg::ref_ptr<osgManipulator::Selection> selection =
    new osgManipulator::Selection;

  dragger = new Rcs::RigidBodyTracker(withSphericalTracker);
  dragger->setupDefaultGeometry();
  dragger->setMatrix(getTransform());
  dragger->addChild(geode.get());

  // create reference transformation and add childs
  refFrame->addChild(dragger.get());
  refFrame->addChild(selection.get());

  addChild(refFrame.get());

  cmdMgr->connect(*(this->dragger.get()), *(selection.get()));

  // Dragger axes should not cast shadows, and not receive shadows
  // And initially, the dragger is not shown
  setNodeMask(getNodeMask() & ~ReceivesShadowTraversalMask);
  setNodeMask(getNodeMask() & ~CastsShadowTraversalMask);
  setAllChildrenOff();

  this->pickHandler = new TargetHandler(this);
  addEventCallback(pickHandler.get());
}

/*******************************************************************************
 * Copies the dragger's transformation into the desired pointer locations.
 ******************************************************************************/
void TargetSetter::updateValues(const osg::Matrixd& transform)
{
  this->posPtr[0] = transform(3,0);
  this->posPtr[1] = transform(3,1);
  this->posPtr[2] = transform(3,2);

  double A[3][3];
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      A[i][j] = transform(i,j) / this->scale;
    }
  }

  if (this->rmPtr != NULL)
  {
    memcpy(this->rmPtr, A, 9*sizeof(double));
  }

  if (this->angPtr != NULL)
  {
    Mat3d_toEulerAngles(this->angPtr, A);
  }
}

/*******************************************************************************
 * Updates the text label with the current transformation info
 ******************************************************************************/
void TargetSetter::updateText()
{
  char* buf = textBuf;

  unsigned int nBytes =
    snprintf(buf, 128, "[%.3f, %.3f, %.3f]",
             this->posPtr[0], this->posPtr[1], this->posPtr[2]);

  // snprintf returns the number of bytes of the non-truncated string. This
  // can lead to overflows if the values get excessively large (close to inf)
  buf += (nBytes > 128) ? 128 : nBytes;

  if (this->angPtr)
  {
    const double r2d = 180.0/M_PI;
    snprintf(buf, 128, "\n[%.1f, %.1f, %.1f]",
             this->angPtr[0]*r2d, this->angPtr[1]*r2d, this->angPtr[2]*r2d);
  }

  _coord_text->setText(textBuf);
}

/*******************************************************************************
 * Returns the dragger's transform as an osg::Matrixd.
 ******************************************************************************/
osg::Matrixd TargetSetter::getTransform() const
{
  double A_KI[3][3];

  if (this->rmPtr)
  {
    Mat3d_copy(A_KI, (double (*)[3]) this->rmPtr);
  }
  else if (this->angPtr)
  {
    Mat3d_fromEulerAngles(A_KI, this->angPtr);
  }
  else
  {
    Mat3d_setIdentity(A_KI);
  }

  osg::Matrixd draggerMat;

  for (int i=0; i<3; i++)
  {
    draggerMat(3,i) = this->posPtr[i];

    for (int j=0; j<3; j++)
    {
      draggerMat(i,j) = A_KI[i][j];
    }
  }

  draggerMat.preMultScale(osg::Vec3d(this->scale, this->scale, this->scale));

  return draggerMat;
}


}   // namespace Rcs

