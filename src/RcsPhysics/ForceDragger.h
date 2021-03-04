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

#ifndef RCS_FORCEDRAGGER_H
#define RCS_FORCEDRAGGER_H

#include <PhysicsBase.h>
#include <BodyPointDragger.h>






namespace Rcs
{
class ForceDragger: public BodyPointDragger
{
public:

  /*! \ingroup ForceDraggerFunctions
   *  \brief Constructs a force dragger that applies the force to the
   *         given physics simulation.
   */
  ForceDragger(PhysicsBase* physics);

protected:

  /*! \brief This function is empty and can be overwritten by derieved
   *         classes. It is called with a locked mutex.
   */
  virtual void update();

  /*! \brief Updates the world mouse point based on the bodie's transformation.
   *         This function uses the physics transformation to account for the
   *         object moving through force interaction etc.
   */
  virtual void updateWorldAnchor();

  const RcsBody* getBodyUnderMouse(const osgGA::GUIEventAdapter& ea,
                                   osgGA::GUIActionAdapter& aa,
                                   double I_pt[3]=NULL,
                                   double k_pt[3]=NULL);

  /*! \brief Handles the Gui events.
   */
  virtual bool callback(const osgGA::GUIEventAdapter& ea,
                        osgGA::GUIActionAdapter& aa);

  PhysicsBase* physics;
};

}   // namespace Rcs

#endif // RCS_FORCEDRAGGER_H
