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

#include "GraphNode.h"
#include "Rcs_graphicsUtils.h"
#include "TargetSetter.h"

#include <KeyCatcherBase.h>
#include <Rcs_typedef.h>
#include <Rcs_macros.h>
#include <Rcs_utils.h>
#include <Rcs_body.h>

#include <osg/StateSet>
#include <osg/PolygonMode>


typedef std::list< osg::ref_ptr<Rcs::BodyNode> > GraphNodeList;


/*******************************************************************************
 * Update callback: Handle some keys etc.
 ******************************************************************************/
namespace Rcs
{
class GraphNodeEventHandler : public osgGA::GUIEventHandler
{
public:

  GraphNodeEventHandler(Rcs::GraphNode* gNode): _gNode(gNode)
  {
  }

  virtual bool handle(const osgGA::GUIEventAdapter& ea,
                      osgGA::GUIActionAdapter& aa)
  {
    return _gNode->callback(ea, aa);
  }

  Rcs::GraphNode* _gNode;
};
}

/*******************************************************************************
 * Body node visitor class. Creates a vector of BodyNode pointers from the
 * given root.
 ******************************************************************************/
class BodyNodeVisitor : public osg::NodeVisitor
{
public:
  BodyNodeVisitor() : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
  {
  }

  virtual void apply(osg::Node& node)
  {
    Rcs::BodyNode* bNode = dynamic_cast<Rcs::BodyNode*>(&node);
    if (bNode)
    {
      nodes.push_back(bNode);
    }
    traverse(node);
  }

  GraphNodeList nodes;
};


namespace Rcs
{

/*******************************************************************************
 * RcsGraph root node.
 ******************************************************************************/
GraphNode::GraphNode() :
  osg::PositionAttitudeTransform(),
  graph(NULL),
  wireframe(false),
  ghostMode(false)
{
  setName("GraphNode");
  this->switchNode = new osg::Switch;
  addChild(switchNode.get());
}

/*******************************************************************************
 * RcsGraph root node.
 ******************************************************************************/
GraphNode::GraphNode(const RcsGraph* g, bool resizeable, bool addSetters) :
  osg::PositionAttitudeTransform(),
  graph(g),
  wireframe(false),
  ghostMode(false)
{
  setName("GraphNode");
  this->switchNode = new osg::Switch;
  addChild(switchNode.get());

  init(graph, resizeable, addSetters);
}

/*******************************************************************************
 * Destructor
 ******************************************************************************/
GraphNode::~GraphNode()
{
  RLOG(5, "Destroying GraphNode");
}

/*******************************************************************************
 * RcsGraph root node.
 ******************************************************************************/
bool GraphNode::init(const RcsGraph* g, bool resizeable,
                     bool automatically_add_target_setters)
{
  if (g==NULL)
  {
    RLOG(1, "GraphNode initialized with NULL graph - you won't see anything");
    return false;
  }

  this->graph = g;

  if (frameHandler.valid())
  {
    RLOG(1, "GraphNode for %s already initialized - skipping", graph->xmlFile);
    return false;
  }

  KeyCatcherBase::registerKey("r", "Toggle visualization of reference frames",
                              "GraphNode");
  KeyCatcherBase::registerKey("c", "Toggle visualization of collision model",
                              "GraphNode");
  KeyCatcherBase::registerKey("g", "Toggle visualization of graphics model",
                              "GraphNode");
  KeyCatcherBase::registerKey("P", "Toggle visualization of physics model",
                              "GraphNode");
  KeyCatcherBase::registerKey("D", "Toggle visualization of debug nodes",
                              "GraphNode");
  KeyCatcherBase::registerKey("G", "Toggle ghost mode", "GraphNode");
  KeyCatcherBase::registerKey("i", "Display name and coordinates of node under"
                              " the mouse", "GraphNode");
  KeyCatcherBase::registerKey("I", "Display information about RcsBody under the"
                              " mouse", "GraphNode");

  RCSGRAPH_TRAVERSE_BODIES(g)
  {
    if (BODY->shape == NULL)
    {
      continue;
    }

    osg::ref_ptr<Rcs::BodyNode> tn = new Rcs::BodyNode(BODY, 1.0, resizeable);
    switchNode->addChild(tn.get());
  }

  if (automatically_add_target_setters)
  {
    // Add target setters
    RCSGRAPH_TRAVERSE_BODIES(this->graph)
    {
      RLOG(5, "Scanning body \"%s\" for rigid body joints", BODY->name);

      /// \todo: Implement for rigid bodies with parent body
      if (BODY->rigid_body_joints==true)
      {
        RLOG(5, "Adding TargetSetter for body %s", BODY->name);

        RCHECK(BODY->jnt);
        double* x = &graph->q->ele[BODY->jnt->jointIndex];
        double* a = &graph->q->ele[BODY->jnt->jointIndex+3];
        RLOG(5, "index is %d", BODY->jnt->jointIndex);
        osg::ref_ptr<Rcs::TargetSetter> ts = new Rcs::TargetSetter(x, a);
        if (BODY->parent)
        {
          ts->setReferenceFrame(BODY->parent->A_BI->org,
                                BODY->parent->A_BI->rot);
        }
        addChild(ts.get());
      }   // if(BODY->rigid_body_joints==true)

    }   // RCSGRAPH_TRAVERSE_BODIES(graph)
  }

  // Assign transformation update callback upon scene traversal
  this->frameHandler = new GraphNodeEventHandler(this);
  addEventCallback(this->frameHandler.get());

  return true;
}

/*******************************************************************************
 * Toggles visibility of the graphics model.
 ******************************************************************************/
void GraphNode::toggleGraphicsModel()
{
  GraphNodeList::iterator li;
  BodyNodeVisitor bnv;
  this->accept(bnv);

  for (li = bnv.nodes.begin(); li != bnv.nodes.end(); ++li)
  {
    Rcs::BodyNode* nd = (*li).get();
    nd->toggleGraphicsNode();
  }

}

/*******************************************************************************
 * Toggles visibility of the physics model.
 ******************************************************************************/
void GraphNode::togglePhysicsModel()
{
  GraphNodeList::iterator li;
  BodyNodeVisitor bnv;
  this->accept(bnv);

  for (li = bnv.nodes.begin(); li != bnv.nodes.end(); ++li)
  {
    Rcs::BodyNode* nd = (*li).get();
    nd->togglePhysicsNode();
  }

}

/*******************************************************************************
 * Toggles visibility of the collision model.
 ******************************************************************************/
void GraphNode::toggleCollisionModel()
{
  GraphNodeList::iterator li;
  BodyNodeVisitor bnv;
  this->accept(bnv);

  for (li = bnv.nodes.begin(); li != bnv.nodes.end(); ++li)
  {
    Rcs::BodyNode* nd = (*li).get();
    nd->toggleCollisionNode();
  }

}

/*******************************************************************************
 * Toggles visibility of the collision model.
 ******************************************************************************/
void GraphNode::toggleReferenceFrames()
{
  GraphNodeList::iterator li;
  BodyNodeVisitor bnv;
  this->accept(bnv);

  for (li = bnv.nodes.begin(); li != bnv.nodes.end(); ++li)
  {
    Rcs::BodyNode* nd = (*li).get();
    nd->toggleReferenceNode();
  }
}

/*******************************************************************************
 * Toggles visibility of the debugging information
 ******************************************************************************/
void GraphNode::toggleDebugInformation()
{
  GraphNodeList::iterator li;
  BodyNodeVisitor bnv;
  this->accept(bnv);

  for (li = bnv.nodes.begin(); li != bnv.nodes.end(); ++li)
  {
    Rcs::BodyNode* nd = (*li).get();
    nd->toggleDebugInformation();
  }

}

/*******************************************************************************
 * Show / hide the graphics model.
 ******************************************************************************/
void GraphNode::displayGraphicsModel(bool visibility)
{
  GraphNodeList::iterator li;
  BodyNodeVisitor bnv;
  this->accept(bnv);

  for (li = bnv.nodes.begin(); li != bnv.nodes.end(); ++li)
  {
    Rcs::BodyNode* nd = (*li).get();
    nd->displayGraphicsNode(visibility);
  }

}

/*******************************************************************************
 * Show / hide the physics model.
 ******************************************************************************/
void GraphNode::displayPhysicsModel(bool visibility)
{
  GraphNodeList::iterator li;
  BodyNodeVisitor bnv;
  this->accept(bnv);

  for (li = bnv.nodes.begin(); li != bnv.nodes.end(); ++li)
  {
    Rcs::BodyNode* nd = (*li).get();
    nd->displayPhysicsNode(visibility);
  }

}

/*******************************************************************************
 * Show / hide the collision model.
 ******************************************************************************/
void GraphNode::displayCollisionModel(bool visibility)
{
  GraphNodeList::iterator li;
  BodyNodeVisitor bnv;
  this->accept(bnv);

  for (li = bnv.nodes.begin(); li != bnv.nodes.end(); ++li)
  {
    Rcs::BodyNode* nd = (*li).get();
    nd->displayCollisionNode(visibility);
  }

}

/*******************************************************************************
 * Show / hide the reference frames.
 ******************************************************************************/
void GraphNode::displayReferenceFrames(bool visibility)
{
  GraphNodeList::iterator li;
  BodyNodeVisitor bnv;
  this->accept(bnv);

  for (li = bnv.nodes.begin(); li != bnv.nodes.end(); ++li)
  {
    Rcs::BodyNode* nd = (*li).get();
    nd->displayReferenceNode(visibility);
  }

}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool GraphNode::collisionModelVisible() const
{
  std::vector<const BodyNode*> bnVec = getBodyNodes();

  for (unsigned int i=0; i< bnVec.size(); ++i)
  {
    if (bnVec[i]->collisionNodeVisible() == true)
    {
      return true;
    }
  }

  return false;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool GraphNode::graphicsModelVisible() const
{
  std::vector<const BodyNode*> bnVec = getBodyNodes();

  for (unsigned int i=0; i< bnVec.size(); ++i)
  {
    if (bnVec[i]->graphicsNodeVisible() == true)
    {
      return true;
    }
  }

  return false;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool GraphNode::physicsModelVisible() const
{
  std::vector<const BodyNode*> bnVec = getBodyNodes();

  for (unsigned int i=0; i< bnVec.size(); ++i)
  {
    if (bnVec[i]->physicsNodeVisible() == true)
    {
      return true;
    }
  }

  return false;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool GraphNode::referenceFramesVisible() const
{
  std::vector<const BodyNode*> bnVec = getBodyNodes();

  for (unsigned int i=0; i< bnVec.size(); ++i)
  {
    if (bnVec[i]->referenceFramesVisible() == true)
    {
      return true;
    }
  }

  return false;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool GraphNode::debugInformationVisible() const
{
  std::vector<const BodyNode*> bnVec = getBodyNodes();

  for (unsigned int i=0; i< bnVec.size(); ++i)
  {
    if (bnVec[i]->debugInformationVisible() == true)
    {
      return true;
    }
  }

  return false;
}

/*******************************************************************************
 * Toggles between solid and wireframe display.
 ******************************************************************************/
void GraphNode::showWireframe(bool status)
{
  if (this->wireframe == status)
  {
    return;
  }

  this->wireframe = status;
  osg::StateSet* pStateSet = getOrCreateStateSet();

  if (this->wireframe)
  {
    pStateSet->setAttribute(new osg::PolygonMode
                            (osg::PolygonMode::FRONT_AND_BACK,
                             osg::PolygonMode::LINE));
  }
  else
  {
    pStateSet->setAttribute(new osg::PolygonMode
                            (osg::PolygonMode::FRONT_AND_BACK,
                             osg::PolygonMode::FILL));
  }

}

/*******************************************************************************
 * Toggles between solid and wireframe display.
 ******************************************************************************/
void GraphNode::toggleWireframe()
{
  showWireframe(!this->wireframe);
}

/*******************************************************************************
 *
 ******************************************************************************/
bool GraphNode::getWireframe() const
{
  return this->wireframe;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool GraphNode::getGhostMode() const
{
  return this->ghostMode;
}

/*******************************************************************************
 * Toggles between transparent and solid display.
 ******************************************************************************/
void GraphNode::toggleGhostMode()
{
  setGhostMode(!this->ghostMode);
}

/*******************************************************************************
 * Toggles between transparent and non-transparent display.
 ******************************************************************************/
void GraphNode::setGhostMode(bool enabled, const std::string& matname)
{
  this->ghostMode = enabled;
  osg::StateSet* pStateSet = getOrCreateStateSet();

  if (this->ghostMode)
  {
    osg::Material* material = new osg::Material();
    if (!matname.empty())
    {
      RcsMaterialData* matDataPtr = getMaterial(matname);

      if (matDataPtr)
      {
        material->setAmbient(osg::Material::FRONT_AND_BACK, matDataPtr->amb);
        material->setDiffuse(osg::Material::FRONT_AND_BACK, matDataPtr->diff);
        material->setSpecular(osg::Material::FRONT_AND_BACK, matDataPtr->spec);
        material->setShininess(osg::Material::FRONT_AND_BACK, matDataPtr->shininess);
      }
      else
      {
        RLOG(4, "Couldn't set material to \"%s\"", matname.c_str());
      }
    }

    material->setTransparency(osg::Material::FRONT, 0.6);
    pStateSet->setAttributeAndModes(material, osg::StateAttribute::OVERRIDE);

    // set render bin to depthsorted in order to handle transparency correctly
    pStateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
  }
  else
  {
    pStateSet->removeAttribute(osg::StateAttribute::MATERIAL);

    // disable depth sorting for better performance
    pStateSet->setRenderingHint(osg::StateSet::OPAQUE_BIN);
  }
}

/*******************************************************************************
 * Makes the node visible.
 ******************************************************************************/
void GraphNode::show()
{
  switchNode->setAllChildrenOn();
}

/*******************************************************************************
 * Makes the node invisible.
 ******************************************************************************/
void GraphNode::hide()
{
  switchNode->setAllChildrenOff();
}

/*******************************************************************************
 * Toggles visibility of the node.
 ******************************************************************************/
void GraphNode::toggle()
{
  if (isVisible())
  {
    switchNode->setAllChildrenOff();
  }
  else
  {
    switchNode->setAllChildrenOn();
  }
}

/*******************************************************************************
 * Makes the node visible.
 ******************************************************************************/
bool GraphNode::isVisible() const
{
  bool visible = switchNode.get()->getValue(0);
  return visible;
}

/*******************************************************************************
 * Returns a pointer to the underlying graph.
 ******************************************************************************/
const RcsGraph* GraphNode::getGraphPtr() const
{
  return this->graph;
}

/*******************************************************************************
* Adds a body node.
******************************************************************************/
BodyNode* GraphNode::addBodyNode(const RcsBody* body, double scale,
                                 bool resizeable, pthread_mutex_t* mtx)
{
  osg::ref_ptr<BodyNode> bNd = new BodyNode(body, scale, resizeable);

  if (mtx != NULL)
  {
    pthread_mutex_lock(mtx);
  }

  switchNode->addChild(bNd.get());

  if (mtx != NULL)
  {
    pthread_mutex_unlock(mtx);
  }

  return bNd;
}

/*******************************************************************************
 * Removes the body node for the given pointer.
 ******************************************************************************/
bool GraphNode::removeBodyNode(const RcsBody* body)
{
  if (body==NULL)
  {
    RLOG(4, "Can't remove NULL body - skipping");
    return false;
  }

  GraphNodeList::iterator li;
  BodyNodeVisitor bnv;
  this->accept(bnv);

  for (li = bnv.nodes.begin(); li != bnv.nodes.end(); ++li)
  {
    osg::ref_ptr<Rcs::BodyNode> node = *li;

    if (node->body() == body)
    {
      bool success = switchNode->removeChild(node);
      if (success == false)
      {
        RLOG(4, "BodyNode %s is not child of GraphNode - skipping", body->name);
      }
      return true;
    }
  }

  RLOG(4, "BodyNode %s is not child of GraphNode - skipping", body->name);

  return false;
}

/*******************************************************************************
* Removes the body node for the given pointer.
******************************************************************************/
bool GraphNode::removeBodyNode(BodyNode* bdyNode)
{
  if (bdyNode == NULL)
  {
    RLOG(4, "Can't remove NULL node - skipping");
    return false;
  }

  GraphNodeList::iterator li;
  BodyNodeVisitor bnv;
  this->accept(bnv);

  for (li = bnv.nodes.begin(); li != bnv.nodes.end(); ++li)
  {
    if (bdyNode == *li)
    {
      bool success = switchNode->removeChild(*li);
      if (success == false)
      {
        RLOG(4, "BodyNode %s is not child of GraphNode - skipping",
             bdyNode->body()->name);
      }
      return true;
    }
  }

  RLOG(4, "BodyNode %s is not child of GraphNode - skipping",
       bdyNode->body()->name);

  return false;
}

/*******************************************************************************
 * Removes the body node for the given name.
 ******************************************************************************/
bool GraphNode::removeBodyNode(const char* name)
{
  if (name == NULL)
  {
    RLOG(4, "Can't remove node with NULL name - skipping");
    return false;
  }

  GraphNodeList::iterator li;
  BodyNodeVisitor bnv;
  this->accept(bnv);

  for (li = bnv.nodes.begin(); li != bnv.nodes.end(); ++li)
  {
    osg::ref_ptr<Rcs::BodyNode> node = *li;

    if (node->getName() == std::string(name))
    {
      bool success = switchNode->removeChild(node);
      if (success == false)
      {
        RLOG(4, "BodyNode %s is not child of GraphNode - skipping", name);
      }
      return true;
    }
  }

  RLOG(4, "BodyNode %s is not child of GraphNode - skipping", name);

  return false;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool GraphNode::hideBodyNode(const RcsBody* body)
{
  if (body==NULL)
  {
    return false;
  }

  GraphNodeList::iterator li;
  BodyNodeVisitor bnv;
  this->accept(bnv);

  for (li = bnv.nodes.begin(); li != bnv.nodes.end(); ++li)
  {
    Rcs::BodyNode* node = (*li).get();
    if (node->body() == body)
    {
      RLOG(3, "Hiding body node %s", body->name);
      node->displayGraphicsNode(false);
      return true;
    }
  }

  return false;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool GraphNode::hideBodyNode(const char* body)
{
  return hideBodyNode(RcsGraph_getBodyByName(this->graph, body));
}

/*******************************************************************************
 *
 ******************************************************************************/
bool GraphNode::hideSubGraph(const char* bodyName)
{
  return hideSubGraph(RcsGraph_getBodyByName(this->graph, bodyName));
}

/*******************************************************************************
 *
 ******************************************************************************/
bool GraphNode::hideSubGraph(const RcsBody* bdy)
{
  if (bdy==NULL)
  {
    return false;
  }

  RCSBODY_TRAVERSE_BODIES((RcsBody*)bdy)
  {
    hideBodyNode(BODY);
  }

  return true;
}

/*******************************************************************************
 * Returns a pointer to the body node with the indicated name. If
 * several bodies have the same name, the closest one to the root
 * node will be taken. If no matching body is found, NULL will be
 * returned. If name is NULL, also NULL is returned.
 ******************************************************************************/
BodyNode* GraphNode::getBodyNode(const char* name)
{
  if (name==NULL)
  {
    return NULL;
  }

  GraphNodeList::iterator li;
  BodyNodeVisitor bnv;
  this->accept(bnv);

  for (li = bnv.nodes.begin(); li != bnv.nodes.end(); ++li)
  {
    Rcs::BodyNode* nd = (*li).get();

    if ((nd != NULL) && STREQ(name, nd->getName().c_str()))
    {
      return nd;
    }
  }

  return NULL;
}

/*******************************************************************************
 * Returns a pointer to the body node with the indicated name. If
 * several bodies have the same name, the closest one to the root
 * node will be taken. If no matching body is found, NULL will be
 * returned. If name is NULL, also NULL is returned.
 ******************************************************************************/
BodyNode* GraphNode::getBodyNode(const RcsBody* body)
{
  if (body==NULL)
  {
    return NULL;
  }

  GraphNodeList::iterator li;
  BodyNodeVisitor bnv;
  this->accept(bnv);

  for (li = bnv.nodes.begin(); li != bnv.nodes.end(); ++li)
  {
    Rcs::BodyNode* nd = (*li).get();
    if ((nd != NULL) && (nd->body()==body))
    {
      return nd;
    }
  }

  return NULL;
}

/*******************************************************************************
 *
 ******************************************************************************/
void GraphNode::setBodyTransformPtr(const RcsBody* body, const HTr* A_BI)
{
  if (body==NULL)
  {
    return;
  }

  GraphNodeList::iterator li;
  BodyNodeVisitor bnv;
  this->accept(bnv);

  for (li = bnv.nodes.begin(); li != bnv.nodes.end(); ++li)
  {
    Rcs::BodyNode* nd = (*li).get();
    if ((nd!=NULL) && (nd->body()==body))
    {
      nd->setTransformPtr(A_BI);
    }
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
void GraphNode::setDynamicMeshUpdate(bool enabled)
{
  GraphNodeList::iterator li;
  BodyNodeVisitor bnv;
  this->accept(bnv);

  for (li = bnv.nodes.begin(); li != bnv.nodes.end(); ++li)
  {
    osg::ref_ptr<Rcs::BodyNode> nd = *li;

    if ((nd.valid()))
    {
      nd->setDynamicMeshUpdate(enabled);
    }
  }

}

/******************************************************************************
 * Add a node after the switch
 *****************************************************************************/
void Rcs::GraphNode::addNode(osg::Node* nd)
{
  switchNode->addChild(nd);
}

/******************************************************************************
 * Frame update
 *****************************************************************************/
bool Rcs::GraphNode::callback(const osgGA::GUIEventAdapter& ea,
                              osgGA::GUIActionAdapter& aa)
{
  switch (ea.getEventType())
  {
    case (osgGA::GUIEventAdapter::KEYDOWN):
    {
      //
      // Toggle reference frames
      //
      if (ea.getKey() == 'r')
      {
        toggleReferenceFrames();
      }
      //
      // Toggle collision model
      //
      else if (ea.getKey() == 'c')
      {
        toggleCollisionModel();
      }

      //
      // Toggle graphics model
      //
      else if (ea.getKey() == 'g')
      {
        toggleGraphicsModel();
      }

      //
      // Toggle physics model
      //
      else if (ea.getKey() == 'P')
      {
        togglePhysicsModel();
      }

      //
      // Toggle debug information
      //
      else if (ea.getKey() == 'D')
      {
        toggleDebugInformation();
      }

      //
      // Toggle ghost mode
      //
      else if (ea.getKey() == 'G')
      {
        toggleGhostMode();
      }

      //
      // Display node name and coordinates
      //
      else if (ea.getKey() == 'i')
      {
        double pt[3];
        BodyNode* nd = getNodeUnderMouse<BodyNode*>(ea, aa, pt);
        if (nd) RMSG("%s [%.5f   %.5f   %.5f]", nd->getName().c_str(),
                       pt[0], pt[1], pt[2]);
      }

      //
      // Display RcsBody
      //
      else if (ea.getKey() == 'I')
      {
        double pt[3];
        BodyNode* nd = getNodeUnderMouse<BodyNode*>(ea, aa, pt);
        if (nd)
        {
          RMSG("%s [%.3f   %.3f   %.3f]", nd->getName().c_str(),
               pt[0], pt[1], pt[2]);
          RcsBody_fprint(stdout, nd->body());
        }
      }

      break;
    }   // KEYDOWN


    default:
      break;
  }

  return false;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
std::vector<const BodyNode*> GraphNode::getBodyNodes() const
{
  std::vector<const BodyNode*> bnVec;

  for (unsigned int i=0; i< switchNode->getNumChildren(); ++i)
  {
    const osg::Node* n1 = switchNode->getChild(i);
    osg::Node* n2 = const_cast<osg::Node*>(n1);
    const BodyNode* n3 = dynamic_cast<BodyNode*>(n2);

    if (n3 != NULL)
    {
      bnVec.push_back(n3);
    }
  }

  return bnVec;
}


}   // namespace Rcs
