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

#include "ForceDragger.h"

#include <BodyNode.h>
#include <Rcs_graphicsUtils.h>
#include <KeyCatcherBase.h>
#include <Rcs_macros.h>
#include <Rcs_Vec3d.h>



/******************************************************************************
 *
 *****************************************************************************/
Rcs::ForceDragger::ForceDragger(PhysicsBase* physics_) : BodyPointDragger(),
  physics(physics_)
{
  RCHECK(this->physics);
  setName("ForceDragger");

  KeyCatcherBase::registerKey("Left", "Move body under mouse -Y",
                              "Force dragger");
  KeyCatcherBase::registerKey("Right", "Move body under mouse +Y",
                              "Force dragger");
  KeyCatcherBase::registerKey("Up", "Move body under mouse +X",
                              "Force dragger");
  KeyCatcherBase::registerKey("Down", "Move body under mouse -X",
                              "Force dragger");
}

/******************************************************************************
 * Drag function that interacts with physics
 *****************************************************************************/
void Rcs::ForceDragger::update()
{
  double f[3];
  Vec3d_sub(f, _I_mouseTip, _I_anchor);
  Vec3d_constMulSelf(f, getForceScaling()*(_leftControlPressed ? 10.0 : 1.0));
  physics->applyForce(_draggedBody, f, _k_anchor);
}

/******************************************************************************
 * Computes the world anchor point of the mouse tip. Called by MouseDragger
 * in the event update traversal.
 *****************************************************************************/
void Rcs::ForceDragger::updateWorldAnchor()
{
  HTr physicsTrf;
  physics->getPhysicsTransform(&physicsTrf, _draggedBody);
  Vec3d_transform(_I_anchor, &physicsTrf, _k_anchor);
}

/******************************************************************************
 * The function returns false so that during dragging, no other events are
 * processed. This includes the mouse manipulator.
 *****************************************************************************/
bool Rcs::ForceDragger::callback(const osgGA::GUIEventAdapter& ea,
                                 osgGA::GUIActionAdapter& aa)
{
  MouseDragger::callback(ea, aa);   // calls update (see above)

  const RcsBody* bdy = Rcs::MouseDragger::getBodyUnderMouse(ea, aa);

  switch (ea.getEventType())
  {
    /////////////////////////////////////////////////////////////////
    // Key pressed events
    /////////////////////////////////////////////////////////////////
    case (osgGA::GUIEventAdapter::KEYDOWN):
    {
      if ((ea.getKey()==osgGA::GUIEventAdapter::KEY_Left) && (bdy!=NULL))
      {
        HTr A_new;
        physics->getPhysicsTransform(&A_new, bdy);
        A_new.org[1] -= 0.01;
        physics->applyTransform(bdy, &A_new);
      }
      else if ((ea.getKey()==osgGA::GUIEventAdapter::KEY_Right) && (bdy!=NULL))
      {
        HTr A_new;
        physics->getPhysicsTransform(&A_new, bdy);
        A_new.org[1] += 0.01;
        physics->applyTransform(bdy, &A_new);
      }
      else if ((ea.getKey() == osgGA::GUIEventAdapter::KEY_Up) && (bdy!=NULL))
      {
        HTr A_new;
        physics->getPhysicsTransform(&A_new, bdy);
        A_new.org[0] += 0.01;
        physics->applyTransform(bdy, &A_new);
      }
      else if ((ea.getKey()==osgGA::GUIEventAdapter::KEY_Down) && (bdy!=NULL))
      {
        HTr A_new;
        physics->getPhysicsTransform(&A_new, bdy);
        A_new.org[0] -= 0.01;
        physics->applyTransform(bdy, &A_new);
      }

      break;
    }

    default:
      break;

  }   // switch(ea.getEventType())

  return false;
}

/******************************************************************************
 * Same as in MouseDragger, except that the physics transformation is used.
 *****************************************************************************/
const RcsBody* Rcs::ForceDragger::getBodyUnderMouse(const osgGA::GUIEventAdapter& ea,
                                                    osgGA::GUIActionAdapter& aa,
                                                    double I_pt[3], double k_pt[3])
{
  Rcs::BodyNode* nd = getNodeUnderMouse<Rcs::BodyNode*>(ea, aa, I_pt);

  if (nd == NULL)
  {
    return NULL;
  }

  const RcsBody* bdy = nd->body();

  if (k_pt && I_pt)
  {
    HTr physicsTrf;
    physics->getPhysicsTransform(&physicsTrf, bdy);
    Vec3d_invTransform(k_pt, &physicsTrf, I_pt);
  }

  return bdy;
}
