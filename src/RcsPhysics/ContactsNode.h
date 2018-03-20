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

#ifndef CONTACTSNODE_H
#define CONTACTSNODE_H

#include <PhysicsBase.h>

#include <osg/Geometry>
#include <osg/Switch>
#include <osgGA/GUIEventHandler>

#include <string>


namespace Rcs
{

/*!
 * \ingroup RcsGraphics Contacts node class
 *
 *  This class draws a 3d line for each of the current contacts detected
 *  by the physics engine. The line's origin is located at contact point,
 *  the line points along the force vector. The default line scaling is 1N/m.
 *  The default line color is red.
 */
class ContactsNode : public osg::Switch
{
  friend class ContactUpdateCallback;

public:

  /*! \brief Pass the color of the contact lines and the factor scaling
   *  the force to drawing length, e.g. a factor of 0.01 will result in
   *  one cm per Newton.
   */
  ContactsNode(PhysicsBase* sim, double factor, const std::string& material);

  /*! \brief Scales the line length with the given factor.
   */
  void scaleLineLength(double factor);

  /*! \brief Modifies the force-length convertion factor.
   */
  void setForceFactor(double factor);

  /*! \brief Returns the force-length conversion factor.
   */
  double getForceFactor() const;

  /*! \brief Toggles between drawing and not drawing.
   *  \return True if the contacts are drawn, false otherwise.
   */
  bool toggle();

  /*! \brief Shows the contact lines.
   */
  void show();

  /*! \brief Hides the contact lines.
   */
  void hide();

  /*! \brief Returns the visibility of the node.
   *  \return True if the contacts are drawn, false otherwise.
   */
  bool isVisible() const;

protected:

  /*! \brief Render the contacts.
   */
  bool updateContacts();

  double forceFactor;                     ///< Scaling of displayed lines
  bool drawContacts;                      ///< Toggle flag for rendering
  osg::ref_ptr<osg::Vec3Array> vertices;  ///< Dragging line points
  osg::ref_ptr<osg::Geometry> linesGeom;  ///< Geometry for above
  osg::ref_ptr<osg::DrawArrays> lineSet;  ///< Line array for above

  PhysicsBase* sim; ///> Physics simulation
};

}   // namespace Rcs

#endif // CONTACTSNODE_H
