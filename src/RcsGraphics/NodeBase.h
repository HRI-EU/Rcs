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

#ifndef RCS_NODEBASE_H
#define RCS_NODEBASE_H

#include <osg/Switch>
#include <osg/PositionAttitudeTransform>
#include <osgGA/GUIEventHandler>

#include <string>


namespace Rcs
{
/*! \ingroup RcsGraphics
 *  \brief Convenience base class that implements a number of methods that are
 *         frequently used in Rcs visualizations. This class inherits from
 *         osg::Switch and allows to add a set of child nodes, whose visibility
 *         can be toggled. A osg::PositionAttitudeTransform child node allows
 *         to set transforma of any added child geometry. Further, it is
 *         possible to link the node to a pointer with transform coordinates
 *         originating from somewhere else. The makeDynamic calls add an
 *         event callback that is called during the viewer's step method.
 */
class NodeBase : public osg::Switch
{
public:

  /*! \brief Initialization of members with defaults. The node is empty, and
   *         no event handler is instantiated.
   */
  NodeBase();

  /*! \brief Makes all added geometry visible.
   */
  virtual void show();

  /*! \brief Makes all added geometry invisible.
   */
  virtual void hide();

  /*! \brief Toggles visibility of all added geometry.
   */
  virtual bool toggle();

  /*! \brief Adds a child node as a child of the pat. This way, the pat's
   *         transform is applied to each node added through this method. This
   *         changes the behavior of the default (osg::Switch) method.
   */
  virtual bool addChild(osg::Node* child);

  /*! \brief Sets the 3d position of the transform pat to the given coordinates.
   *         The orientation remains unchanged.
   */
  virtual void setPosition(double x, double y, double z);

  /*! \brief Sets the 3d position of the transform pat to the given coordinates.
   *         The orientation remains unchanged.
   */
  virtual void setPosition(const double x[3]);

  /*! \brief Sets the 3d position of the transform pat to the given coordinates.
   *         The orientation remains unchanged.
   */
  virtual void setPosition(osg::Vec3 pos);

  /*! \brief Sets the 3d orientation of the transform pat to the given rotation
   *         matrix. It is interpreted as a rotation from the (I)nertial to the
   *         (B)ody frame (rows of the matrix correspond to axes of the B-frame
   *         in I-coordinates.
   */
  virtual void setRotation(double A_BI[3][3]);

  /*! \brief Sets the 3d orientation of the transform pat to the given Euler.
   *         angles. They are interpreted as x-y-z order with a moving frame.
   */
  virtual void setRotation(double eulerABC[3]);

  /*! \brief Sets the 3d orientation of the transform pat to the given
   *         Quaternion.
   */
  virtual void setRotation(osg::Quat att);

  /*! \brief Sets the 3d position and orientation of the transform.
   */
  virtual void setTransformation(const double x[3], double A_BI[3][3]);

  /*! \brief Shows or hides all geometry that has been added to the node in
   *         wireframe display.
   */
  virtual void setWireframe(bool visible=true);

  /*! \brief Toggles wireframe visibility.
   */
  virtual void toggleWireframe();

  /*! \brief Adds an event handler that calls the functions eventCallback()
   *         and frameCallback(). If an event handler has already been
   *         instantiated, the function does nothing and reports this on
   *         debug level 4.
   */
  virtual void makeDynamic();

  /*! \brief Links the node's position coordinates to the passed pointer and
   *         instantiates an event handler through \ref makeDynamic(). The
   *         event handler sets the transform's position on each callback to
   *         the values pointed to by pos.
   */
  virtual void makeDynamic(const double pos[3]);

  /*! \brief Links the node's rotation coordinates to the passed pointer and
   *         instantiates an event handler through \ref makeDynamic(). The
   *         event handler sets the transform's orientation on each callback to
   *         the values pointed to by A_BI.
   */
  virtual void makeDynamic(double A_BI[3][3]);

  /*! \brief Convenience method that calls \ref makeDynamic(const double[3])
   *         and \ref makeDynamic(double[3][3]).
   */
  virtual void makeDynamic(const double pos[3], double A_BI[3][3]);

  /*! \brief Returns the pointer to the external position coordinates, or NULL
   *         if they have not been set.
   */
  virtual const double* getPosPtr() const;

  /*! \brief Returns the pointer to the external rotation matrix coordinates,
   *         or NULL if they have not been set.
   */
  virtual const double* getRotMatPtr() const;

  /*! \brief Sets the pointer to the external position coordinates.
   */
  virtual void setPosPtr(const double* pos);

  /*! \brief Sets the pointer to the external rotation matrix.
   */
  virtual void setRotMatPtr(const double* rot);

  /*! \brief Sets the material properties to the values according to the
   *         passed material name and alpha value.
   */
  virtual void setMaterial(const char* material, double alpha=1.0);

  /*! \brief Sets the material properties to the values according to the
   *         passed material name and alpha value.
   */
  virtual void setMaterial(const std::string& material, double alpha=1.0);

  /*! \brief Returns true if the node is visible, false otherwise.
   */
  virtual bool isVisible() const;

  /*! \brief Callback function that is called on each frame step of the
   *         viewer. It only gets called if an event handler has been started
   *         (see \ref makeDynamic() for details). This function does nothing.
   *         It should be overwritten by derieved classes with meaningful
   *         functionality.
   */
  virtual bool frameCallback();

  /*! \brief Callback function that is called on each frame step of the
   *         viewer. It only gets called if an event handler has been started
   *         (see \ref makeDynamic() for details). This function does nothing.
   *         It should be overwritten by derieved classes with meaningful
   *         functionality.
   */
  virtual bool eventCallback(const osgGA::GUIEventAdapter& ea,
                             osgGA::GUIActionAdapter& aa);

  /*! \brief Returns a reference to the node's transform node for convenience.
   */
  virtual osg::ref_ptr<osg::PositionAttitudeTransform> patPtr() const;

protected:

  osg::ref_ptr<osg::PositionAttitudeTransform> pat;
  bool showWireframe;
  const double* posPtr;
  const double* rmPtr;
  bool isDynamic;
};

}   // namespace Rcs

#endif // RCS_NODEBASE_H
