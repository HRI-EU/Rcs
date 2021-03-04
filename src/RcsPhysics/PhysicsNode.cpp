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
#include <Rcs_basicMath.h>
#include <KeyCatcherBase.h>
#include <FTSensorNode.h>
#include <CapsuleNode.h>
#include <BoxNode.h>



/*******************************************************************************
 * We instantiate a GraphNode without resizing and without TargetSetters.
 ******************************************************************************/
Rcs::PhysicsNode::PhysicsNode(PhysicsBase* sim_, bool resizeable_):
  NodeBase(), modelNd(NULL), physicsNd(NULL), sim(sim_), displayMode(0),
  resizeable(resizeable_)
{
  RCHECK(sim);
  setName("PhysicsNode");

  KeyCatcherBase::registerKey("C", "Toggle contacts node", "PhysicsNode");
  KeyCatcherBase::registerKey("f", "Scale drag force by 0.1", "PhysicsNode");
  KeyCatcherBase::registerKey("F", "Scale drag force by 10.0", "PhysicsNode");
  KeyCatcherBase::registerKey("T", "Toggle physics or graph transform display",
                              "PhysicsNode");

  // This node shows the simulation result mapped to the minimal coordinate
  // description used in the graph. We acquire all joint angles and use the
  // results from the forward kinematics. This may not reflect the phyics
  // simulation result, since there is no joint separation etc. visible. For
  // this, the below instantiated physicsNd is responsible.
  // \todo: Maybe only use one node, and make it toggleable?
  this->modelNd = new GraphNode(sim_->getGraph(), resizeable, false);
  modelNd->displayGraphicsModel(false);
  modelNd->displayPhysicsModel(true);
  modelNd->setGhostMode(true, "RED");
  modelNd->setName("PhysicsNode::modelNd");
  pat->addChild(modelNd);

  osg::ref_ptr<ContactsNode> cn = new ContactsNode(sim, 0.1, "RUBY");
  cn->toggle();   // Start with contacts
  pat->addChild(cn.get());

  // This node displays the bodies at the transformations coming natively
  // from the phyics engine. Since these in some cases don't use minimal
  // coordinates, one might see separation of objects in case of large
  // forces or other effects. This node also updates soft body meshes if any.
  this->physicsNd = new GraphNode(sim_->getGraph(), resizeable, false);
  physicsNd->displayGraphicsModel(false);
  physicsNd->displayPhysicsModel(true);
  if (std::string(sim->getClassName())=="SoftBullet")
  {
    physicsNd->setDynamicMeshUpdate(true);
  }
  physicsNd->setName("PhysicsNode::physicsNd");
  pat->addChild(physicsNd);

  updateTransformPointers();


  RCSGRAPH_TRAVERSE_SENSORS(sim_->getGraph())
  {
    if (SENSOR->type == RCSSENSOR_LOAD_CELL)
    {
      osg::ref_ptr<FTSensorNode> ftn = new FTSensorNode(SENSOR);
      ftn->setTransformPtr(sim->getPhysicsTransformPtr(ftn->getMountBody()));
      pat->addChild(ftn.get());
    }
  }

  osg::ref_ptr<ForceDragger> draggerNd = new ForceDragger(sim_);
  pat->addChild(draggerNd.get());

#if defined (USE_BULLET)
  Rcs::BulletSimulation* bSim = dynamic_cast<Rcs::BulletSimulation*>(sim_);
  if (bSim != NULL)
  {
    KeyCatcherBase::registerKey("d", "Toggle debug viewer", "PhysicsNode");
    BulletDebugDrawer* gDebugDrawer = new BulletDebugDrawer();
    gDebugDrawer->setDebugMode(btIDebugDraw::DBG_DrawWireframe |
                               btIDebugDraw::DBG_DrawContactPoints);
    pat->addChild(gDebugDrawer);
  }
#endif

  setDisplayMode(2);

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
  // In case the graph is resizeable, bodies might be replaced on the fly. To
  // make sure we always point to the correct ones, we update the transformation
  // pointers before each iteration.
  if (resizeable==true && modelNd->isVisible())
  {
    updateTransformPointers();
  }



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
          RLOG(1, "No debug drawer for other simulations than Bullet");
          break;
        }

        BulletDebugDrawer* dDraw = s->getDebugDrawer();

        if (dDraw != NULL)   // Disable the debug drawer
        {
          RLOG(0, "Disabling debug drawer");
          dDraw->hide();
          s->setDebugDrawer(NULL);
        }
        else   // Enable the debug drawer
        {
          BulletDebugDrawer* nd_i = NULL;

          // Find the debug drawer from the children of this class
          for (unsigned int i=0; i<pat->getNumChildren(); ++i)
          {
            nd_i = dynamic_cast<BulletDebugDrawer*>(pat->getChild(i));
          }

          if (nd_i != NULL)
          {
            RLOG(0, "Enabling debug drawer");
            nd_i->show();
            s->setDebugDrawer(nd_i);
          }

        }
#endif
      }
      /////////////////////////////////////////////////////////////
      // Toggle ContactsNode
      /////////////////////////////////////////////////////////////
      else if (ea.getKey() == 'C')
      {
        for (unsigned int i=0; i<pat->getNumChildren(); ++i)
        {
          Node* ndi = pat->getChild(i);
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
        for (unsigned int i=0; i<pat->getNumChildren(); ++i)
        {
          Node* ndi = pat->getChild(i);
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
        for (unsigned int i=0; i<pat->getNumChildren(); ++i)
        {
          Node* ndi = pat->getChild(i);
          Rcs::ForceDragger* dn = dynamic_cast<Rcs::ForceDragger*>(ndi);
          if (dn != NULL)
          {
            RLOG(5, "Scaling drag force to %f", 10.0*dn->getForceScaling());
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
*
******************************************************************************/
const char* Rcs::PhysicsNode::getDisplayModeStr() const
{
  static char modeName[][32] = { "graph and physics",
                                 "graph only",
                                 "physics only",
                                 "Unknown display mode"
                               };

  int idx = Math_iClip(this->displayMode, 0, 3);
  return modeName[idx];
}

/*******************************************************************************
 * Set transparency for individual bodies
 ******************************************************************************/
void Rcs::PhysicsNode::setDisplayMode(int mode)
{
  this->displayMode = mode;

  Rcs::ContactsNode* cnd = NULL;

  for (unsigned int i=0; i<pat->getNumChildren(); ++i)
  {
    cnd = dynamic_cast<Rcs::ContactsNode*>(pat->getChild(i));
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
      RLOG(5, "Showing both transforms");
      break;
    case 1:   // Graph only
      setPhysicsTransform(false);
      setModelTransform(true);
      modelNd->setGhostMode(false);
      if (cnd)
      {
        cnd->hide();
      }
      RLOG(5, "Showing graph transform only");
      break;
    case 2:   // Physics only
      setPhysicsTransform(true);
      setModelTransform(false);
      if (cnd)
      {
        cnd->show();
      }
      RLOG(5, "Showing physics transform only");
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

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::PhysicsNode::updateTransformPointers()
{
  RCSGRAPH_TRAVERSE_BODIES(physicsNd->getGraphPtr())
  {
    const RcsBody* simBdy = RcsGraph_getBodyByName(modelNd->getGraphPtr(),
                                                   BODY->name);
    const HTr* physicsTrf = sim->getPhysicsTransformPtr(simBdy);
    physicsNd->setBodyTransformPtr(BODY, physicsTrf);
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::PhysicsNode::removeBodyNode(const char* body)
{
  bool success = modelNd->removeBodyNode(body);
  success = physicsNd->removeBodyNode(body) && success;

  return success;
}

/*******************************************************************************
 * Adds a body node to both GraphNodes.
 ******************************************************************************/
void Rcs::PhysicsNode::addBodyNode(const RcsBody* body)
{
  modelNd->addBodyNode(body, 1.0, false);
  BodyNode* node = physicsNd->addBodyNode(body, 1.0, false);

  const RcsBody* simBdy = RcsGraph_getBodyByName(sim->getGraph(), body->name);
  const HTr* physicsTrf = sim->getPhysicsTransformPtr(simBdy);
  node->setTransformPtr(physicsTrf);
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::PhysicsNode::setDebugDrawer(bool enable)
{
#if defined (USE_BULLET)

  Rcs::BulletSimulation* s = dynamic_cast<Rcs::BulletSimulation*>(sim);

  if (s == NULL)
  {
    RLOG(1, "No debug drawer for other simulations than Bullet");
    return false;
  }


  if (enable==false)   // Disable the debug drawer
  {
    RLOG(0, "Disabling debug drawer");
    BulletDebugDrawer* dDraw = s->getDebugDrawer();
    if (dDraw)
    {
      dDraw->hide();
    }

    s->setDebugDrawer(NULL);
  }
  else   // Enable the debug drawer
  {
    BulletDebugDrawer* nd_i = NULL;

    // Find the debug drawer from the children of this class
    for (unsigned int i=0; i<pat->getNumChildren(); ++i)
    {
      nd_i = dynamic_cast<BulletDebugDrawer*>(pat->getChild(i));
    }

    if (nd_i != NULL)
    {
      RLOG(0, "Enabling debug drawer");
      nd_i->show();
      s->setDebugDrawer(nd_i);
    }

  }

  return true;

#else

  RLOG(1, "No debug drawer for other simulations than Bullet");
  return false;

#endif
}
