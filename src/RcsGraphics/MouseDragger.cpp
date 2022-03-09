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

#include "MouseDragger.h"
#include "BodyNode.h"
#include "Rcs_graphicsUtils.h"

#include <KeyCatcherBase.h>
#include <Rcs_typedef.h>
#include <Rcs_utils.h>
#include <Rcs_math.h>
#include <Rcs_macros.h>

#include <osg/LineWidth>



/******************************************************************************
 * Helper class to handle mouse events
 *****************************************************************************/
namespace Rcs
{
class MouseDraggerHandler: public osgGA::GUIEventHandler
{
public:
  MouseDraggerHandler(Rcs::MouseDragger* dragger_) : dragger(dragger_)
  {
  }

private:
  virtual bool handle(const osgGA::GUIEventAdapter& ea,
                      osgGA::GUIActionAdapter& aa)
  {
    return dragger->callback(ea, aa);
  }

  Rcs::MouseDragger* dragger;
};
}

/******************************************************************************
 * Determines the 3d world coordinates to which the mouse pointer points to.
 * Further, the corresponding body is determined. The function returns a
 * pointer to the body under the mouse pointer and copies the mouse (world)
 * coords, and the mouse coords in the bodies frame of reference. If no body
 * is found under the mouse pointer, the mouse coordinates are left unchanged.
 *****************************************************************************/
static RcsBody* getBodyUnderMouseTip(const osgGA::GUIEventAdapter& ea,
                                     osgGA::GUIActionAdapter& aa,
                                     double I_pt[3], double k_pt[3])
{
  Rcs::BodyNode* nd = Rcs::getNodeUnderMouse<Rcs::BodyNode*>(ea, aa, I_pt);

  if (nd == NULL)
  {
    return NULL;
  }

  RcsBody* bdy = nd->body();

  if (k_pt && I_pt)
  {
    Vec3d_invTransform(k_pt, &bdy->A_BI, I_pt);
  }

  return bdy;
}

/******************************************************************************
 *
 *****************************************************************************/
RcsBody* Rcs::MouseDragger::getBodyUnderMouse(const osgGA::GUIEventAdapter& ea,
                                              osgGA::GUIActionAdapter& aa,
                                              double I_pt[3], double k_pt[3])
{
  return getBodyUnderMouseTip(ea, aa, I_pt, k_pt);
}

/******************************************************************************
 *
 *****************************************************************************/
const RcsBody* Rcs::MouseDragger::getBodyUnderMouse(const osgGA::GUIEventAdapter& ea,
                                                    osgGA::GUIActionAdapter& aa,
                                                    double I_pt[3], double k_pt[3]) const
{
  return getBodyUnderMouseTip(ea, aa, I_pt, k_pt);
}

/******************************************************************************
 *
 *****************************************************************************/
Rcs::MouseDragger::MouseDragger() : osg::Switch(),
  _draggedBody(NULL),
  _leftShiftPressed(false),
  _leftControlPressed(false),
  _LMBPressed(false),
  _RMBPressed(false),
  _enableArrowKeyTranslation(true)
{
  setName("MouseDragger");
  KeyCatcherBase::registerKey("Left Shift", "Enable body dragging", "MouseDragger");
  KeyCatcherBase::registerKey("Left Cntrl", "Amplify force dragging", "MouseDragger");

  Vec3d_setZero(_I_mouseTip);
  Vec3d_setZero(_k_anchor);
  Vec3d_setZero(_I_anchor);
  Vec3d_setZero(_I_anchor0);
  Mat3d_setIdentity(_A_BI0);

  // create the Geode (Geometry Node) to contain all our geometry objects.
  osg::ref_ptr<osg::Geode> geode = new osg::Geode();
  addChild(geode.get());

  // Create lines and pass the created vertex array to the points geometry object.
  // We prealloacte the vertex array, 2 points makes 1 line segment.
  _linesGeom = new osg::Geometry();
  _vertices = new osg::Vec3Array(2);
  (*_vertices)[0].set(0.0, 0.0, 0.0);
  (*_vertices)[1].set(0.0, 0.0, 0.0);
  _linesGeom->setVertexArray(_vertices.get());

  // set the colors as before, plus using the above
  osg::Vec4Array* colors = new osg::Vec4Array;
  colors->push_back(osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f));
  _linesGeom->setColorArray(colors);
  _linesGeom->setColorBinding(osg::Geometry::BIND_OVERALL);

  // set the normal in the same way color.
  osg::Vec3Array* normals = new osg::Vec3Array;
  normals->push_back(osg::Vec3(0.0f, -1.0f, 0.0f));
  _linesGeom->setNormalArray(normals);
  _linesGeom->setNormalBinding(osg::Geometry::BIND_OVERALL);

  // Adjust the line width
  osg::StateSet* linestateset = geode->getOrCreateStateSet();
  osg::LineWidth* linewidth = new osg::LineWidth();
  linewidth->setWidth(3.0f);
  linestateset->setAttribute(linewidth, osg::StateAttribute::ON);
  //  linestateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
  geode->setStateSet(linestateset);

  // This time we simply use primitive, and hardwire the number of coords
  // to use since we know up front,
  _linesGeom->addPrimitiveSet
  (new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 2));

  geode->addDrawable(_linesGeom.get());

  // Add event callback handler
  setEventCallback(new MouseDraggerHandler(this));
}

/******************************************************************************
 * The function returns false so that during dragging, no other events are
 * processed. This includes the mouse manipulator.
 *****************************************************************************/
bool Rcs::MouseDragger::callback(const osgGA::GUIEventAdapter& ea,
                                 osgGA::GUIActionAdapter& aa)
{
  _mtx.lock();

  switch (ea.getEventType())
  {
    /////////////////////////////////////////////////////////////////
    // Mouse button pressed events.
    /////////////////////////////////////////////////////////////////
    case (osgGA::GUIEventAdapter::PUSH):
    {
      // Left mouse button pressed
      if (ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
      {
        _LMBPressed = true;

        if (_leftShiftPressed)
        {
          _draggedBody = getBodyUnderMouse(ea, aa, _I_mouseTip, _k_anchor);

          if (_draggedBody != NULL)
          {
            Vec3d_copy(_I_anchor, _I_mouseTip);
            Vec3d_copy(_I_anchor0, _I_mouseTip);
          }
        }
      }
      // Right mouse button pressed
      else if (ea.getButton() == osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON)
      {
        _RMBPressed = true;

        if (_leftShiftPressed)
        {
          _draggedBody = getBodyUnderMouse(ea, aa, _I_mouseTip, _k_anchor);

          // Memorize rotation matrix
          if (_draggedBody)
          {
            Mat3d_copy(_A_BI0, (double(*)[3])_draggedBody->A_BI.rot);
          }
        }
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
        _LMBPressed  = false;
        resetDragger();
      }

      // Right mouse button released.
      if (ea.getButton() == osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON)
      {
        _draggedBody = NULL;
        _RMBPressed  = false;
      }

      break;
    }

    /////////////////////////////////////////////////////////////////
    // Key pressed events
    /////////////////////////////////////////////////////////////////
    case (osgGA::GUIEventAdapter::KEYDOWN):
    {
      if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Shift_L)
      {
        _leftShiftPressed = true;
        setAllChildrenOn();
      }
      else if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Control_L)
      {
        _leftControlPressed = true;
      }

      if (_enableArrowKeyTranslation)
      {
        RcsBody* bdy = Rcs::MouseDragger::getBodyUnderMouse(ea, aa);
        if (bdy)
        {
          const double disp = 0.01;
          double dx[3];
          Vec3d_setZero(dx);

          if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Left)
          {
            dx[1] -= disp;
          }
          else if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Right)
          {
            dx[1] += disp;
          }
          else if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Up)
          {
            dx[0] += disp;
          }
          else if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Down)
          {
            dx[0] -= disp;
          }
          else if (ea.getKey() == '+')
          {
            dx[2] += disp;
          }
          else if (ea.getKey() == '-')
          {
            dx[2] -= disp;
          }

          Vec3d_rotateSelf(dx, bdy->A_BI.rot);
          Vec3d_addSelf(bdy->A_BP.org, dx);
        }

      }

      break;
    }

    /////////////////////////////////////////////////////////////////
    // Key released events. If the Shift-key is released, we quit
    // dragging and hide the geometry.
    /////////////////////////////////////////////////////////////////
    case (osgGA::GUIEventAdapter::KEYUP):
    {
      if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Shift_L)
      {
        _leftShiftPressed = false;
        resetDragger();
        setAllChildrenOff();
      }
      else if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Control_L)
      {
        _leftControlPressed = false;
      }

      break;
    }

    /////////////////////////////////////////////////////////////////
    // Mouse drag events.
    /////////////////////////////////////////////////////////////////
    case (osgGA::GUIEventAdapter::DRAG):
    {
      // Compute the world coordinates of the drag point.
      if ((_LMBPressed) && (_leftShiftPressed) && (_draggedBody))
      {
        // Anchor point for drag force in world coordinates
        Rcs::getMouseTip(ea, aa, _I_anchor0, _I_mouseTip);
      }
      break;
    }

    /////////////////////////////////////////////////////////////////
    // Frame update: We update the (possibly) moving anchor point.
    // If the left mouse button drags, the vertices are also updated.
    /////////////////////////////////////////////////////////////////
    case (osgGA::GUIEventAdapter::FRAME):
    {
      if (_draggedBody)
      {
        updateWorldAnchor();

        if (_LMBPressed)
        {
          (*_vertices)[0].set(_I_anchor[0], _I_anchor[1], _I_anchor[2]);
          (*_vertices)[1].set(_I_mouseTip[0], _I_mouseTip[1], _I_mouseTip[2]);
          _linesGeom->setVertexArray(_vertices.get());
        }
      }
      update();
      break;
    }

    default:
      break;

  }   // switch(...)

  _mtx.unlock();

  return false;
}

/******************************************************************************
 * Drag function to be overwritten
 *****************************************************************************/
void Rcs::MouseDragger::update()
{
}

/******************************************************************************
 *
 *****************************************************************************/
void Rcs::MouseDragger::setEnableArrowKeyTranslation(bool enable)
{
  _enableArrowKeyTranslation = enable;
}

/******************************************************************************
 * Computes the world anchor point of the mouse tip.
 *****************************************************************************/
void Rcs::MouseDragger::updateWorldAnchor()
{
  Vec3d_transform(_I_anchor, &_draggedBody->A_BI, _k_anchor);
}

/******************************************************************************
 *
 *****************************************************************************/
void Rcs::MouseDragger::resetDragger()
{
  _draggedBody = NULL;
  Vec3d_setZero(_k_anchor);
  Vec3d_setZero(_I_anchor);
  Vec3d_setZero(_I_anchor0);
  Vec3d_setZero(_I_mouseTip);
  Mat3d_setIdentity(_A_BI0);
  (*_vertices)[0].set(0.0, 0.0, 0.0);
  (*_vertices)[1].set(0.0, 0.0, 0.0);
  _linesGeom->setVertexArray(_vertices.get());
}

/******************************************************************************
 *
 *****************************************************************************/
bool Rcs::MouseDragger::leftMouseButtonPressed() const
{
  bool result;

  _mtx.lock();
  result = _LMBPressed;
  _mtx.unlock();

  return result;
}

/******************************************************************************
 *
 *****************************************************************************/
bool Rcs::MouseDragger::rightMouseButtonPressed() const
{
  bool result;

  _mtx.lock();
  result = _RMBPressed;
  _mtx.unlock();

  return result;
}

/******************************************************************************
 *
 *****************************************************************************/
bool Rcs::MouseDragger::leftShiftKeyPressed() const
{
  bool result;

  _mtx.lock();
  result = _leftShiftPressed;
  _mtx.unlock();

  return result;
}

/******************************************************************************
 *
 *****************************************************************************/
bool Rcs::MouseDragger::leftCtrlKeyPressed() const
{
  bool result;

  _mtx.lock();
  result = _leftControlPressed;
  _mtx.unlock();

  return result;
}

/******************************************************************************
 *
 *****************************************************************************/
bool Rcs::MouseDragger::getBodyMove(HTr* A_BI)
{
  bool success = false;

  _mtx.lock();
  if ((_RMBPressed) && (_leftShiftPressed) && (_draggedBody))
  {
    double I_r_offset[3];
    Vec3d_transRotate(I_r_offset, (double(*)[3])_draggedBody->A_BI.rot, _k_anchor);
    Vec3d_copy(A_BI->org, _I_mouseTip);
    Vec3d_subSelf(A_BI->org, I_r_offset);
    Mat3d_copy(A_BI->rot, _A_BI0);
    success = true;
  }
  _mtx.unlock();

  return success;
}

/******************************************************************************
 *
 *****************************************************************************/
bool Rcs::MouseDragger::getBodyAnchor(double I_r[3]) const
{
  bool success = false;

  _mtx.lock();
  if (_draggedBody != NULL)
  {
    Vec3d_copy(I_r, _I_anchor);
    success = true;
  }
  _mtx.unlock();

  return success;
}

/******************************************************************************
 *
 *****************************************************************************/
bool Rcs::MouseDragger::getLocalBodyAnchor(double k_r[3]) const
{
  bool success = false;

  _mtx.lock();
  if (_draggedBody != NULL)
  {
    Vec3d_copy(k_r, _k_anchor);
    success = true;
  }
  _mtx.unlock();

  return success;
}

/******************************************************************************
 * Computes the anchor point and mouse tip in world coordinates
 *****************************************************************************/
bool Rcs::MouseDragger::getMouseTip(double I_tip[3]) const
{
  bool success = false;

  _mtx.lock();
  if (_draggedBody != NULL)
  {
    Vec3d_copy(I_tip, _I_mouseTip);
    success = true;
  }
  _mtx.unlock();

  return success;
}

/******************************************************************************
 *
 *****************************************************************************/
const RcsBody* Rcs::MouseDragger::getDragData(double I_mouseTip[3],
                                              double I_bodyAnchor[3],
                                              double k_bodyAnchor[3],
                                              bool* leftMouseButtonPressed,
                                              bool* rightMouseButtonPressed,
                                              bool* leftShiftPressed,
                                              bool* leftCtrlPressed) const
{
  const RcsBody* dragged = NULL;

  _mtx.lock();
  if (_draggedBody != NULL)
  {
    dragged = _draggedBody;

    if (I_mouseTip!=NULL)
    {
      Vec3d_copy(I_mouseTip, _I_mouseTip);
    }

    if (I_bodyAnchor!=NULL)
    {
      Vec3d_copy(I_bodyAnchor, _I_anchor);
    }

    if (k_bodyAnchor!=NULL)
    {
      Vec3d_copy(k_bodyAnchor, _k_anchor);
    }
  }

  if (leftMouseButtonPressed)
  {
    *leftMouseButtonPressed = _LMBPressed;
  }

  if (rightMouseButtonPressed)
  {
    *rightMouseButtonPressed = _RMBPressed;
  }

  if (leftShiftPressed)
  {
    *leftShiftPressed = _leftShiftPressed;
  }

  if (leftCtrlPressed)
  {
    *leftCtrlPressed = _leftControlPressed;
  }

  _mtx.unlock();

  return dragged;
}

/******************************************************************************
 *
 *****************************************************************************/
const RcsBody* Rcs::MouseDragger::draggedBody() const
{
  const RcsBody* dragged;

  _mtx.lock();
  dragged = _draggedBody;
  _mtx.unlock();

  return dragged;
}
