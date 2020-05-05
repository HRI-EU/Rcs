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

#include "BulletDebugNode.h"
#include "BulletSimulation.h"
#include "BulletRigidBody.h"
#include "BulletDebugDrawer.h"

#include <Rcs_macros.h>
#include <KeyCatcherBase.h>



/*******************************************************************************
 *
 ******************************************************************************/
Rcs::BulletDebugNode::BulletDebugNode(PhysicsBase* sim_, pthread_mutex_t* mtx):
  NodeBase(), sim(sim_), viewerLock(mtx)
{
  KeyCatcherBase::registerKey("d", "Toggle debug viewer", "PhysicsNode");
  BulletDebugDrawer* debugDrawer = new BulletDebugDrawer();
  debugDrawer->setDebugMode(btIDebugDraw::DBG_DrawWireframe |
                            btIDebugDraw::DBG_DrawContactPoints);
  addChild(debugDrawer);
  makeDynamic();
  debugDrawer->show();
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::BulletDebugNode::~BulletDebugNode()
{
}

/*******************************************************************************
*
******************************************************************************/
bool Rcs::BulletDebugNode::eventCallback(const osgGA::GUIEventAdapter& ea,
                                         osgGA::GUIActionAdapter& aa)
{
  switch (ea.getEventType())
  {

    case (osgGA::GUIEventAdapter::KEYDOWN):
    {
      if (ea.getKey() == 'd')
      {
        toggle();
      }
    }   // case (osgGA::GUIEventAdapter::KEYDOWN)

    default:
      break;

  }   // switch (ea.getEventType())

  return false;
}

/*******************************************************************************
*
******************************************************************************/
void Rcs::BulletDebugNode::lockViewer() const
{
  if (viewerLock)
  {
    pthread_mutex_lock(viewerLock);
  }
}

/*******************************************************************************
*
******************************************************************************/
void Rcs::BulletDebugNode::unlockViewer() const
{
  if (viewerLock)
  {
    pthread_mutex_unlock(viewerLock);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletDebugNode::show()
{
  Rcs::BulletSimulation* bSim = dynamic_cast<Rcs::BulletSimulation*>(sim);

  if (bSim == NULL)
  {
    return;
  }

  BulletDebugDrawer* nd_i = NULL;

  // Find the debug drawer from the children of this class
  for (unsigned int i = 0; i < getNumChildren(); ++i)
  {
    nd_i = dynamic_cast<BulletDebugDrawer*>(getChild(i));
  }

  if (nd_i != NULL)
  {
    nd_i->show();
    bSim->setDebugDrawer(nd_i);
  }
}

/*******************************************************************************
 * Disable the debug drawer
 ******************************************************************************/
void Rcs::BulletDebugNode::hide()
{
  Rcs::BulletSimulation* bSim = dynamic_cast<Rcs::BulletSimulation*>(sim);

  if (bSim == NULL)
  {
    return;
  }

  BulletDebugDrawer* dDraw = bSim->getDebugDrawer();

  if (dDraw != NULL)
  {
    dDraw->hide();
    bSim->setDebugDrawer(NULL);
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::BulletDebugNode::isVisible() const
{
  Rcs::BulletSimulation* bSim = dynamic_cast<Rcs::BulletSimulation*>(sim);

  if (bSim == NULL)
  {
    return false;
  }

  BulletDebugDrawer* dDraw = bSim->getDebugDrawer();

  if (dDraw == NULL)
  {
    return false;
  }

  return true;
}
