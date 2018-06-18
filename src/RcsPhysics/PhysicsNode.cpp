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

#include "PhysicsNode.h"
#include "ContactsNode.h"
#include "ForceDragger.h"

#if defined (USE_BULLET)
#include "BulletSimulation.h"
#include "BulletRigidBody.h"
#include "BulletDebugDrawer.h"
#endif

#include <Rcs_typedef.h>
#include <Rcs_macros.h>
#include <KeyCatcherBase.h>
#include <FTSensorNode.h>
#include <CapsuleNode.h>
#include <BoxNode.h>



/*******************************************************************************
 * We instantiate a GraphNode without resizing and without TargetSetters.
 ******************************************************************************/
Rcs::PhysicsNode::PhysicsNode(PhysicsBase* sim_):
  NodeBase(), modelNd(NULL), physicsNd(NULL), sim(sim_), displayMode(0)
{
  KeyCatcherBase::registerKey("C", "Toggle contacts node", "PhysicsNode");
  KeyCatcherBase::registerKey("f", "Scale drag force by 0.1", "PhysicsNode");
  KeyCatcherBase::registerKey("F", "Scale drag force by 10.0", "PhysicsNode");
  KeyCatcherBase::registerKey("T", "Toggle physics or graph transform display",
                              "PhysicsNode");

  this->modelNd = new GraphNode(sim_->getGraph(), false, false);
  modelNd->toggleGraphicsModel();
  modelNd->togglePhysicsModel();
  modelNd->setGhostMode(true, "RED");
  addChild(modelNd);

  osg::ref_ptr<ContactsNode> cn = new ContactsNode(sim, 0.1, "RUBY");
  cn->toggle();   // Start with contacts
  addChild(cn.get());







  this->physicsNd = new GraphNode(RcsGraph_clone(sim_->getGraph()),
                                  false, false);
  physicsNd->toggleGraphicsModel();
  physicsNd->togglePhysicsModel();
  addChild(physicsNd);

  RCSGRAPH_TRAVERSE_BODIES(physicsNd->getGraphPtr())
  {
    physicsNd->setBodyTransformPtr(BODY, sim->getPhysicsTransformPtr(RcsGraph_getBodyByName(modelNd->getGraphPtr(), BODY->name)));
  }


  RCSGRAPH_TRAVERSE_SENSORS(sim_->getGraph())
  {
    if (SENSOR->type == RCSSENSOR_LOAD_CELL)
    {
      osg::ref_ptr<FTSensorNode> ftn = new FTSensorNode(SENSOR);
      ftn->setTransformPtr(sim->getPhysicsTransformPtr(ftn->getMountBody()));
      addChild(ftn.get());
    }
  }






  osg::ref_ptr<ForceDragger> draggerNd = new ForceDragger(sim_);
  addChild(draggerNd.get());

#if defined (USE_BULLET)
  Rcs::BulletSimulation* bSim = dynamic_cast<Rcs::BulletSimulation*>(sim_);
  if (bSim != NULL)
  {
    KeyCatcherBase::registerKey("d", "Toggle debug viewer", "PhysicsNode");
    BulletDebugDrawer* gDebugDrawer = new BulletDebugDrawer();
    gDebugDrawer->setDebugMode(btIDebugDraw::DBG_DrawWireframe |
                               btIDebugDraw::DBG_DrawContactPoints);
    addChild(gDebugDrawer);
  }
#endif





  makeDynamic();
}

/*******************************************************************************
 * Virtual destructor for possible inheritance.
 ******************************************************************************/
Rcs::PhysicsNode::~PhysicsNode()
{
}

/*******************************************************************************
 * Track physics transformation
 ******************************************************************************/
void Rcs::PhysicsNode::setModelTransform(bool enable)
{
  if (enable)
  {
    modelNd->show();
  }
  else
  {
    modelNd->hide();
  }
}

/*******************************************************************************
 * Track physics transformation
 ******************************************************************************/
void Rcs::PhysicsNode::setPhysicsTransform(bool enable)
{
  if (enable)
  {
    physicsNd->show();
  }
  else
  {
    physicsNd->hide();
  }

  // RCSGRAPH_TRAVERSE_BODIES(getGraphPtr())
  // {
  //   if (enable==true)
  //   {
  //     setBodyTransformPtr(BODY, sim->getPhysicsTransformPtr(BODY));
  //   }
  //   else
  //   {
  //     setBodyTransformPtr(BODY, BODY->A_BI);
  //   }
  // }

  // for (unsigned int i=0; i<getNumChildren(); ++i)
  // {
  //   osg::Node* nd_i = getChild(i);
  //   FTSensorNode* ftNd = dynamic_cast<FTSensorNode*>(nd_i);
  //   if (ftNd != NULL)
  //   {
  //     if (enable==true)
  //     {
  //       ftNd->setTransformPtr(sim->getPhysicsTransformPtr(ftNd->getMountBody()));
  //     }
  //     else
  //     {
  //       ftNd->setTransformPtr(ftNd->getMountBody()->A_BI);
  //     }
  //   }

  // }

  // showPhysicsTransforms = enable;
}

/*******************************************************************************
 * Frame update
 ******************************************************************************/
bool Rcs::PhysicsNode::eventCallback(const osgGA::GUIEventAdapter& ea,
                                     osgGA::GUIActionAdapter& aa)
{
  switch (ea.getEventType())
  {
    case (osgGA::GUIEventAdapter::KEYDOWN):
    {
      /////////////////////////////////////////////////////////////
      // Toggle Bullet physics debug node
      /////////////////////////////////////////////////////////////
      if (ea.getKey() == 'd')
      {
#if defined (USE_BULLET)
        Rcs::BulletSimulation* s = dynamic_cast<Rcs::BulletSimulation*>(sim);

        if (s == NULL)
        {
          RLOG(1, "Can't toggle debug drawer for other simulations than Bullet");
          break;
        }

        BulletDebugDrawer* dDraw = s->getDebugDrawer();

        if (dDraw != NULL)   // Disable the debug drawer
        {
          dDraw->hide();
          s->setDebugDrawer(NULL);
          RLOG(5, "Hiding debug drawer");
        }
        else   // Enable the debug drawer
        {
          BulletDebugDrawer* nd_i = NULL;

          // Find the debug drawer from the children of this class
          for (unsigned int i=0; i<getNumChildren(); ++i)
          {
            nd_i = dynamic_cast<BulletDebugDrawer*>(getChild(i));
          }

          if (nd_i != NULL)
          {
            nd_i->show();
            s->setDebugDrawer(nd_i);
            RLOG(5, "Showing debug drawer");
          }

        }
#endif
      }
      /////////////////////////////////////////////////////////////
      // Toggle ContactsNode
      /////////////////////////////////////////////////////////////
      else if (ea.getKey() == 'C')
      {
        for (unsigned int i=0; i<getNumChildren(); ++i)
        {
          Node* ndi = getChild(i);
          Rcs::ContactsNode* cn = dynamic_cast<Rcs::ContactsNode*>(ndi);
          if (cn != NULL)
          {
            cn->toggle();
          }
        }
      }
      /////////////////////////////////////////////////////////////
      // Reduce force scaling by an order of magnitude
      /////////////////////////////////////////////////////////////
      else if (ea.getKey() == 'f')
      {
        for (unsigned int i=0; i<getNumChildren(); ++i)
        {
          Node* ndi = getChild(i);
          Rcs::ForceDragger* dn = dynamic_cast<Rcs::ForceDragger*>(ndi);
          if (dn != NULL)
          {
            dn->scaleDragForce(0.1*dn->getForceScaling());
          }
        }
      }
      /////////////////////////////////////////////////////////////
      // Increase force scaling by an order of magnitude
      /////////////////////////////////////////////////////////////
      else if (ea.getKey() == 'F')
      {
        for (unsigned int i=0; i<getNumChildren(); ++i)
        {
          Node* ndi = getChild(i);
          Rcs::ForceDragger* dn = dynamic_cast<Rcs::ForceDragger*>(ndi);
          if (dn != NULL)
          {
            dn->scaleDragForce(10.0*dn->getForceScaling());
          }
        }
      }
      /////////////////////////////////////////////////////////////
      // Toggle physics or graph transform display
      /////////////////////////////////////////////////////////////
      else if (ea.getKey() == 'T')
      {
        setDisplayMode(this->displayMode+1);
      }

      break;
    }   // KEYDOWN


    default:
      break;
  }

  return false;
}

/*******************************************************************************
 * Set transparency for individual bodies
 ******************************************************************************/
bool Rcs::PhysicsNode::setGhostMode(const std::string& bodyName,
                                    const std::string& matname)
{
  bool success = true;
  Rcs::BodyNode* bNd = modelNd->getBodyNode(bodyName.c_str());

  if (bNd != NULL)
  {
    bNd->setGhostMode(true, matname);
  }
  else
  {
    RLOG(4, "Can't set ghost mode for body \"%s\" in kinematic model",
         bodyName.c_str());
    success = false;
  }

  bNd = physicsNd->getBodyNode(bodyName.c_str());

  if (bNd != NULL)
  {
    bNd->setGhostMode(true, matname);
  }
  else
  {
    RLOG(4, "Can't set ghost mode for body \"%s\" in physics model",
         bodyName.c_str());
    success = false;
  }

  return success;
}

/*******************************************************************************
 *
 ******************************************************************************/
int Rcs::PhysicsNode::getDisplayMode() const
{
  return this->displayMode;
}

/*******************************************************************************
 * Set transparency for individual bodies
 ******************************************************************************/
void Rcs::PhysicsNode::setDisplayMode(int mode)
{
  this->displayMode = mode;

  Rcs::ContactsNode* cnd = NULL;

  for (unsigned int i=0; i<getNumChildren(); ++i)
  {
    cnd = dynamic_cast<Rcs::ContactsNode*>(getChild(i));
    if (cnd != NULL)
    {
      break;
    }
  }

  if (this->displayMode > 2)
  {
    this->displayMode = 0;
  }

  switch (displayMode)
  {
    case 0:   // Both
      setPhysicsTransform(true);
      modelNd->show();
      modelNd->setGhostMode(true, "RED");
      if (cnd)
      {
        cnd->show();
      }
      RLOG(4, "Showing both transforms");
      break;
    case 1:   // Graph only
      setPhysicsTransform(false);
      setModelTransform(true);
      modelNd->setGhostMode(false);
      if (cnd)
      {
        cnd->hide();
      }
      RLOG(4, "Showing graph transform only");
      break;
    case 2:   // Physics only
      setPhysicsTransform(true);
      setModelTransform(false);
      if (cnd)
      {
        cnd->show();
      }
      RLOG(4, "Showing physics transform only");
      break;
    default:
      RLOG(1, "Unhandled display mode %d", displayMode);
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::PhysicsNode::showWorldBoundingBox()
{
#if defined (USE_BULLET)
  Rcs::BulletSimulation* bSim = dynamic_cast<Rcs::BulletSimulation*>(sim);

  if (bSim==NULL)
  {
    RLOG(4, "Can't show world bounding box for physics other than Bullet");
    return;
  }

  btVector3 aabbMin, aabbMax, center;
  bSim->getWorldBoundingBox(aabbMin, aabbMax);
  center = aabbMin + 0.5*(aabbMax-aabbMin);
  double c[3];
  c[0] = center.x();
  c[1] = center.y();
  c[2] = center.z();

  double lx = aabbMax[0] - aabbMin[0];
  double ly = aabbMax[1] - aabbMin[1];
  double lz = aabbMax[2] - aabbMin[2];

  osg::ref_ptr<Rcs::BoxNode> bn = new Rcs::BoxNode(c, NULL, lx, ly, lz);
  bn->setWireframe(true);
  addChild(bn.get());
#endif
}

/*******************************************************************************
 * This shows each Bullet bodie's COM
 ******************************************************************************/
void Rcs::PhysicsNode::showBodyCOMs()
{
#if defined (USE_BULLET)
  Rcs::BulletSimulation* bSim = dynamic_cast<Rcs::BulletSimulation*>(sim);

  if (bSim==NULL)
  {
    RLOG(4, "Can't show body COMs for physics other than Bullet");
    return;
  }

  RCSGRAPH_TRAVERSE_BODIES(sim->getGraph())
  {
    BulletRigidBody* rb = bSim->getRigidBody(BODY);
    if (rb != NULL)
    {
      const double* pos = rb->getCOMTransformPtr()->org;
      osg::ref_ptr<Rcs::CapsuleNode> cn = new Rcs::CapsuleNode(pos, NULL,
                                                               0.025, 0.0);
      cn->makeDynamic(pos);
      addChild(cn.get());
    }
  }
#endif
}
