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

#ifndef RCS_MOUSEDRAGGER_H
#define RCS_MOUSEDRAGGER_H

#include <Rcs_graph.h>

#include <osg/Switch>
#include <osg/Geometry>
#include <osgViewer/Viewer>
#include <osgGA/GUIEventHandler>
#include <OpenThreads/Mutex>



/*! \ingroup RcsGraphics
 * \brief Mouse spring: When the shift-key is pressed, clicking the left
 *        mouse button will make the 3d point of the BodyNode under the mouse
 *        tip to the anchor point. The anchor point as well as the mouse tip
 *        will be updated as long as both left mouse button and the shift-key
 *        are pressed. The 3d points are determined by projecting them into
 *        the viewplane going through the initial anchor point.
 *
 *        All public functions are thread-safe, protected by the mutex _mtx.
 */

namespace Rcs
{

class MouseDragger: public osg::Switch
{
  friend class MouseDraggerHandler;
public:

  /*!
   *  \brief Default constructor.
   */
  MouseDragger();

  /*! \brief Returns a pointer to the currently dragged body. If no body
   *         is dragged, NULL will be returned.
   */
  virtual const RcsBody* draggedBody() const;

  /*! \brief Copies the body anchor coordinates into I_r. The coordinates
   *         are represented in the world frame. If there is no dragged
   *         body, I_r is not modified.
   *
   *  \param[out]   I_r   Anchor point in world coordinates. If no body is
   *                      dragged, I_r is not modified.
   *  \return true if a body is dragged (and I_r is updated), false otherwise
   */
  virtual bool getBodyAnchor(double I_r[3]) const;

  /*! \brief Copies the body anchor coordinates into k_r. The coordinates
   *         are represented in the body frame. If there is no dragged
   *         body, k_r is not modified.
   *
   *  \param[out]   k_r   Anchor point in body coordinates. If no body is
   *                      dragged, k_r is not modified.
   *  \return true if a body is dragged (and k_r is updated), false otherwise
   */
  virtual bool getLocalBodyAnchor(double k_r[3]) const;

  /*! \brief Copies the mouse tip coordinates into I_r. The coordinates
   *         are represented in the world frame. If there is no dragged
   *         body, the mouse tip coordinates are not changed.
   *
   *  \param[out]   I_r   Mouse tip in world coordinates. If no body is
   *                      dragged, I_r is not modified.
   *  \return true if a body is dragged (and I_r is updated), false otherwise
   */
  virtual bool getMouseTip(double I_r[3]) const;

  /*! \brief Copies the mouse tip and anchor point coordinates. They
   *         are represented in the world frame. If there is no dragged
   *         body, the arguments are not changed.
   *
   *  \param[out]   I_mouseTip  Mouse tip in world coordinates. If no body is
   *                            dragged, I_mouseTip is not modified. If the
   *                            argument is NULL, it will be ignored.
   *  \param[out]   I_anchor    Anchor in world coordinates. If no body is
   *                            dragged, I_anchor is not modified. If the
   *                            argument is NULL, it will be ignored.
   *  \param[out]   k_anchor    Anchor in body coordinates. If no body is
   *                            dragged, this argument is not modified. If the
   *                            argument is NULL, it will be ignored.
   *  \param[out]   leftMouseButtonPressed    Will be set to true if the left
   *                                          mouse button is pressed, to false
   *                                          otherwise, and ignored when NULL.
   *  \param[out]   rightMouseButtonPressed   Will be set to true if the left
   *                                          mouse button is pressed, to false
   *                                          otherwise, and ignored when NULL.
   *  \param[out]   leftShiftPressed   Will be set to true if the left shift
   *                                   key is pressed, to false otherwise, and
   *                                   ignored when NULL.
   *  \param[out]   leftCtrlPressed    Will be set to true if the left ctrl
   *                                   key is pressed, to false otherwise, and
   *                                   ignored when NULL.
   *  \return Pointer to dragged body, or NULL if none is dragged.
   */
  virtual const RcsBody* getDragData(double I_mouseTip[3],
                                     double I_anchor[3],
                                     double k_anchor[3],
                                     bool* leftMouseButtonPressed,
                                     bool* rightMouseButtonPressed,
                                     bool* leftShiftPressed,
                                     bool* leftCtrlPressed) const;

  /*! \brief Returns the transformation that has been determined for the
   *         kinematic mouse drag. On Shift - RMB, the transformation of
   *         the body is memorized. This function returns the initial
   *         transformation along with its dragged displacement in the
   *         view plane.
   */
  virtual bool getBodyMove(HTr* A_BI);

  /*! \brief Returns true if the left mouse button is pressed, false
   *         otherwise.
   */
  virtual bool leftMouseButtonPressed() const;

  /*! \brief Returns true if the right mouse button is pressed, false
   *         otherwise.
   */
  virtual bool rightMouseButtonPressed() const;

  /*! \brief Returns true if the left shift key is pressed, false
   *         otherwise.
   */
  virtual bool leftShiftKeyPressed() const;

  /*! \brief Returns true if the left ctrl key is pressed, false
   *         otherwise.
   */
  virtual bool leftCtrlKeyPressed() const;

  /*! \brief Determines the 3d world coordinates to which the mouse pointer
  *          points to. Further, the corresponding body is determined. The
  *          function returns a pointer to the body under the mouse pointer and
  *          copies the mouse (world) coordinates, and the mouse coordinates
  *          in the bodies frame of reference. If no body is found under the
  *          mouse pointer, the mouse coordinates are left unchanged.
  */
  static const RcsBody* getBodyUnderMouse(const osgGA::GUIEventAdapter& ea,
                                          osgGA::GUIActionAdapter& aa,
                                          double I_pt[3]=NULL,
                                          double k_pt[3]=NULL);

protected:

  /*! \brief This function is empty and can be overwritten by derieved
   *         classes. It is called with a locked mutex in each frame.
   */
  virtual void update();

  /*! \brief Handles the Gui events, e.g. memorizes which button has been
   *         pressed, and which body is currently being tracked.
   */
  virtual bool callback(const osgGA::GUIEventAdapter& ea,
                        osgGA::GUIActionAdapter& aa);

  const RcsBody* _draggedBody; ///< Body that is currently being dragged
  double _k_anchor[3];         ///< Local body point that is memorized when a body is initially clicked
  double _I_anchor[3];         ///< Anchor body point in world coordinates, updated with body transform
  double _I_anchor0[3];        ///< Anchor body point in world coordinates when clicked
  double _I_mouseTip[3];       ///< Mouse tip in world coordinates
  double _A_BI0[3][3];         ///< Rotation matrix that is memorized when a body is initially clicked for moving
  bool _leftShiftPressed;      ///< True if left Shift key is pressed
  bool _leftControlPressed;    ///< True if left Control key is pressed
  bool _LMBPressed;            ///< True if left mouse button is pressed
  bool _RMBPressed;            ///< True if right mouse button is pressed

private:

  void resetDragger();

  osg::ref_ptr<osg::Vec3Array> _vertices;   ///< Dragging line points
  osg::ref_ptr<osg::Geometry> _linesGeom;   ///< Geometry for above
  mutable OpenThreads::Mutex _mtx;
};

}   // namespace Rcs

#endif // RCS_MOUSEDRAGGER_H
