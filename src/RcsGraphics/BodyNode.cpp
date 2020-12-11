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

#include "BodyNode.h"
#include "Rcs_graphicsUtils.h"

#include <Rcs_resourcePath.h>
#include <Rcs_Vec3d.h>
#include <Rcs_Mat3d.h>
#include <Rcs_typedef.h>
#include <Rcs_joint.h>
#include <Rcs_macros.h>
#include <Rcs_utils.h>
#include <MeshNode.h>

#ifdef USE_OCTOMAP
#include "OctomapNode.h"
#include <octomap/OcTree.h>
#include <octomap/octomap_types.h>
#endif

#include <osgDB/ReadFile>
#include <osg/MatrixTransform>
#include <osg/PolygonMode>
#include <osg/Version>


namespace Rcs
{

static char _fontFile[256];
static bool _fontFileFound = false;
static bool _fontFileSearched = false;



/*******************************************************************************
 * Update callback: Sets the bodies transformation to the node.
 ******************************************************************************/
class BodyUpdateCallback : public osg::NodeCallback
{

public:

  BodyUpdateCallback(BodyNode* bdyNode)
  {
    RCHECK(bdyNode);
    _bdyNode = bdyNode;
  }

  virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
  {
    _bdyNode->updateCallback(node, nv);
    traverse(node, nv);
  }

protected:

  BodyNode* _bdyNode;
};

/*******************************************************************************
 * Recursively adds the bodies collision shapes to the node.
 *
 * nodeSwitch (osg::Switch)
 *     |
 *     ---> shapeTransform (PAT) with A_CB
 *               |
 *               ---> geode (osg::Geode)
 *                      |
 *                      ---> shape (osg::Drawable)
 *                             |
 *                             ---> capsule, box, etc (osg::Capsule ...)
*******************************************************************************/
BodyNode::BodyNode(const RcsBody* b, float scale, bool resizeable) :
  bdy(b),
  A_BI(b->A_BI),
  ghostMode(false),
  dynamicMeshUpdate(false),
  refNode(false),
  initializeDebugInfo(false)
{
  RCHECK(this->bdy);
  setName(this->bdy->name ? this->bdy->name : "Unnamed body");

  _nodeSwitch = new osg::Switch();
  addChild(_nodeSwitch.get());

  RCHECK(scale > 0.0);
  setScale(osg::Vec3(scale, scale, scale));

  _collisionNode = new osg::Switch;
  _graphicsNode = new osg::Switch;
  _physicsNode = new osg::Switch;
  _depthNode = new osg::Switch;


  RCSBODY_TRAVERSE_SHAPES(b)
  {
    osg::ref_ptr<ShapeNode> sni = new ShapeNode(SHAPE, resizeable);
    _shapeNodes.push_back(sni);

    if ((SHAPE->computeType & RCSSHAPE_COMPUTE_DISTANCE) != 0)
    {
      _collisionNode->addChild(sni);
    }

    if ((SHAPE->computeType & RCSSHAPE_COMPUTE_GRAPHICS) != 0)
    {
      _graphicsNode->addChild(sni);
    }

    if ((SHAPE->computeType & RCSSHAPE_COMPUTE_PHYSICS) != 0 ||
        (SHAPE->computeType & RCSSHAPE_COMPUTE_SOFTPHYSICS) != 0)
    {
      _physicsNode->addChild(sni);
    }

    if ((SHAPE->computeType & RCSSHAPE_COMPUTE_DEPTHBUFFER) != 0)
    {
      _depthNode->addChild(sni);
    }

  }

  _collisionNode->setAllChildrenOff();
  _graphicsNode->setAllChildrenOn();
  _physicsNode->setAllChildrenOff();
  _depthNode->setAllChildrenOff();

  _nodeSwitch->addChild(_collisionNode.get());
  _nodeSwitch->addChild(_graphicsNode.get());
  _nodeSwitch->addChild(_physicsNode.get());
  _nodeSwitch->addChild(_depthNode.get());

  _collisionNode->setName("BodyNode::CollisionNode");
  _graphicsNode->setName("BodyNode::GraphicsNode");
  _physicsNode->setName("BodyNode::PhysicsNode");
  _depthNode->setName("BodyNode::DepthBufferNode");


  // Assign the initial transformation to the node
  if (HTr_isValid(this->A_BI))
  {
    setTransformation(this->A_BI);
  }
  else
  {
    RLOG(4, "Invalid transform in \"%s\"", getName().c_str());
  }

  // Switch off all coordinate frames per default
  displayReferenceNode(false);

  // Assign transformation update callback upon scene traversal
  setUpdateCallback(new BodyUpdateCallback(this));
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
osg::Switch* BodyNode::addDebugInformation()
{
  osg::ref_ptr<osg::TessellationHints> hints = new osg::TessellationHints;
  hints->setDetailRatio(0.5f);

  osg::Switch* debugNode = new osg::Switch;
  osg::Geode* geode = new osg::Geode();
  debugNode->addChild(geode);

  // Add some text
  _debugText = new osgText::Text();
  _debugText->setCharacterSize(0.01);

  // Search font file only once and store it statically
  if (!_fontFileSearched)
  {
    _fontFileFound = Rcs_getAbsoluteFileName("VeraMono.ttf", _fontFile);

    if (_fontFileFound == false)
    {
      _fontFileFound = Rcs_getAbsoluteFileName("fonts/VeraMono.ttf", _fontFile);
    }

    _fontFileSearched = true;
  }

  if (_fontFileFound == true)
  {
    _debugText->setFont(_fontFile);
  }
  else
  {
    RLOG(4, "Couldn't find font file \"VeraMono.ttf\" in resource path");
    REXEC(5)
    {
      Rcs_printResourcePath();
    }
    _debugText->setCharacterSize(50);
  }

  _debugText->setText(std::string("-----") + getName());
  _debugText->setAlignment(osgText::Text::LEFT_CENTER);
  _debugText->setAxisAlignment(osgText::Text::SCREEN);
  _debugText->setColor(colorFromString("RED"));
  geode->addDrawable(_debugText.get());


  // add a small sphere representing the origin of the body
  double r = 0.01;
  osg::Sphere* sphere = new osg::Sphere(osg::Vec3(0.0, 0.0, 0.0), r);
  osg::ShapeDrawable* shape = new osg::ShapeDrawable(sphere, hints.get());
  setNodeMaterial("RED", geode);
  geode->addDrawable(shape);

  // add a small sphere representing the COM of the body
  if (this->bdy->m > 0.0)
  {
    osg::Geode* comGeode = new osg::Geode();
    setNodeMaterial("BLUE", comGeode);
    osg::Sphere* sphere = new osg::Sphere(osg::Vec3(this->bdy->Inertia->org[0],
                                                    this->bdy->Inertia->org[1],
                                                    this->bdy->Inertia->org[2]),
                                          1.01*r);
    osg::ShapeDrawable* shape = new osg::ShapeDrawable(sphere, hints.get());
    comGeode->addDrawable(shape);
    debugNode->addChild(comGeode);
  }

  // Add a small cylinder for each joint
  RCSBODY_TRAVERSE_JOINTS(this->bdy)
  {
    if (!RcsJoint_isRotation(JNT))
    {
      continue;
    }

    // Transfomation of Joint
    osg::PositionAttitudeTransform* joint_transform =
      new osg::PositionAttitudeTransform;
    osg::Geode* joint_geode = new osg::Geode();

    switch (JNT->type)
    {
      case RCSJOINT_ROT_X:
      {
        osg::Quat qA(M_PI_2, osg::Vec3d(0.0, 1.0, 0.0));
        joint_transform->setAttitude(qA);
        break;
      }
      case RCSJOINT_ROT_Y:
      {
        osg::Quat qA(M_PI_2, osg::Vec3d(1.0, 0.0, 0.0));
        joint_transform->setAttitude(qA);
        break;
      }
    }

    debugNode->addChild(joint_transform);
    joint_transform->addChild(joint_geode);

    // Cylinder for joint
    osg::Cylinder* cylinder =
      new osg::Cylinder(osg::Vec3(0.0, 0.0, 0.0), 0.005, 0.04);

    osg::ShapeDrawable* shape = new osg::ShapeDrawable(cylinder, hints.get());

    joint_geode->addDrawable(shape);
    setNodeMaterial("GREEN", joint_geode);

  }   // RCSBODY_TRAVERSE_JOINTS(this->bdy)

  if (this->bdy->parent)
  {
    // add a line between this and the previous body
    _debugLine = new osg::Vec3Array;
    _debugLineGeometry = new osg::Geometry;
    _debugLineGeometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES));

    // add the two body positions in local reference
    HTr A_12;
    HTr_invTransform(&A_12, this->bdy->A_BI, this->bdy->parent->A_BI);

    _debugLine->push_back(osg::Vec3(0.0, 0.0, 0.0));
    _debugLine->push_back(osg::Vec3(A_12.org[0], A_12.org[1], A_12.org[2]));

    osg::DrawArrays* ps =
      ((osg::DrawArrays*)(_debugLineGeometry)->getPrimitiveSet(0));
    ps->setCount(_debugLine->size());
    _debugLineGeometry->setVertexArray(_debugLine.get());
    _debugLineGeometry->setPrimitiveSet(0, ps);
    geode->addDrawable(_debugLineGeometry.get());
  }

  osg::StateSet* pStateSet = debugNode->getOrCreateStateSet();

  pStateSet->setAttributeAndModes(new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::FILL), osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);

  return debugNode;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
BodyNode::~BodyNode()
{
  RLOG(5, "Destroying BodyNode of body \"%s\"", getName().c_str());
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void BodyNode::setAlpha(float alpha)
{
  osg::StateSet* stateset = getOrCreateStateSet();

  if (stateset == NULL)
  {
    RLOG(8, "Could not set alpha - stateset is NULL");
    return;
  }

  osg::Material* material =
    dynamic_cast<osg::Material*>(stateset->getAttribute(osg::StateAttribute::MATERIAL));

  if (material == NULL)
  {
    RLOG(8, "Could not assign material - material is NULL");
    return;
  }



  // Add transparency
#if 1
  material->setAlpha(osg::Material::FRONT_AND_BACK, alpha);

  //    stateset->setAttributeAndModes(material,
  //                                 osg::StateAttribute::OVERRIDE|
  //                                 osg::StateAttribute::ON);
  //
  //    setStateSet(stateset);
#endif
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void BodyNode::setAlpha(float alpha, osg::Node* node)
{
  // first check if we should set the transparency of the current node
  osg::StateSet* stateset = node->getOrCreateStateSet();
  if (stateset)
  {
    osg::Material* material = dynamic_cast<osg::Material*>(stateset->getAttribute(osg::StateAttribute::MATERIAL));
    if (material)
    {
      // Add transparency
      material->setAlpha(osg::Material::FRONT_AND_BACK, alpha);
      //    stateset->setAttributeAndModes(material,
      //                                 osg::StateAttribute::OVERRIDE|
      //                                 osg::StateAttribute::ON);
      //    setStateSet(stateset);
    }
  }

  // then traverse the group and call setAlpha on all children
  osg::Group* group = node->asGroup();
  if (group)
  {
    for (size_t i = 0; i < group->getNumChildren(); i++)
    {
      setAlpha(alpha, group->getChild(i));
    }
  }
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void BodyNode::setTransformation(const HTr* A_BI)
{
  setPosition(osg::Vec3(A_BI->org[0], A_BI->org[1], A_BI->org[2]));

  osg::Quat qA;
  qA.set(osg::Matrix(A_BI->rot[0][0], A_BI->rot[0][1], A_BI->rot[0][2], 0.0,
                     A_BI->rot[1][0], A_BI->rot[1][1], A_BI->rot[1][2], 0.0,
                     A_BI->rot[2][0], A_BI->rot[2][1], A_BI->rot[2][2], 0.0,
                     0.0, 0.0, 0.0, 1.0));

  setAttitude(qA);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool BodyNode::setTexture(std::string textureFile)
{
  osg::Image* image = osgDB::readImageFile(textureFile);

  if (!image)
  {
    RLOG(1, "Texture file \"%s\" not found! Ignoring",
         textureFile.c_str());
    return false;
  }
  else
  {
    RLOG(5, "Applying Texture file \"%s\"", textureFile.c_str());
  }

  // Assign texture through state set
  osg::Material* material = new osg::Material;
  osg::StateSet* stateset = getOrCreateStateSet();
  stateset->setAttributeAndModes(material,
                                 osg::StateAttribute::OVERRIDE |
                                 osg::StateAttribute::ON);
  // Makes material scale-invariant
  stateset->setMode(GL_RESCALE_NORMAL, osg::StateAttribute::ON);
  osg::Texture2D* texture = new osg::Texture2D;
  texture->setImage(image);
  stateset->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);

  return true;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void BodyNode::toggleCollisionNode()
{
  displayCollisionNode(!collisionNodeVisible());
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void BodyNode::toggleGraphicsNode()
{
  displayGraphicsNode(!graphicsNodeVisible());
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void BodyNode::togglePhysicsNode()
{
  displayPhysicsNode(!physicsNodeVisible());
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void BodyNode::toggleReferenceNode()
{
  displayReferenceNode(!referenceFramesVisible());
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void BodyNode::toggleDepthNode()
{
  displayDepthNode(!depthNodeVisible());
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void BodyNode::toggleDebugInformation()
{
  displayDebugInformation(!debugInformationVisible());
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void BodyNode::displayCollisionNode(bool visible)
{
  if (visible)
  {
    _collisionNode->setAllChildrenOn();
  }
  else
  {
    _collisionNode->setAllChildrenOff();
  }
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void BodyNode::displayGraphicsNode(bool visible)
{
  if (visible)
  {
    _graphicsNode->setAllChildrenOn();
  }
  else
  {
    _graphicsNode->setAllChildrenOff();
  }
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void BodyNode::displayPhysicsNode(bool visible)
{
  if (visible)
  {
    _physicsNode->setAllChildrenOn();
  }
  else
  {
    _physicsNode->setAllChildrenOff();
  }
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void BodyNode::displayReferenceNode(bool visible)
{
  refNode = visible;

  for (size_t i=0; i<_shapeNodes.size(); ++i)
  {
    _shapeNodes[i]->displayFrames(visible);
  }
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void BodyNode::displayDepthNode(bool visible)
{
  if (visible)
  {
    _depthNode->setAllChildrenOn();
  }
  else
  {
    _depthNode->setAllChildrenOff();
  }
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void BodyNode::displayDebugInformation(bool visible)
{
  if (visible)
  {
    if (_debugNode.valid())
    {
      _debugNode->setAllChildrenOn();
    }

    this->initializeDebugInfo = true;
  }
  else
  {
    if (_debugNode.valid())
    {
      _debugNode->setAllChildrenOff();
    }
  }
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool BodyNode::collisionNodeVisible() const
{
  return _collisionNode->getValue(0);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool BodyNode::graphicsNodeVisible() const
{
  return _graphicsNode->getValue(0);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool BodyNode::physicsNodeVisible() const
{
  return _physicsNode->getValue(0);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool BodyNode::depthNodeVisible() const
{
  return _depthNode->getValue(0);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool BodyNode::referenceFramesVisible() const
{
  return refNode;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool BodyNode::debugInformationVisible() const
{
  if (!_debugNode.valid())
  {
    return false;
  }

  return _debugNode->getValue(0);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void BodyNode::setMaterial(const std::string& material, double alpha)
{
  setNodeMaterial(material, this, alpha);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
const RcsBody* BodyNode::body() const
{
  return this->bdy;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
const char* BodyNode::className() const
{
  return "Rcs::BodyNode";
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
const HTr* BodyNode::getTransformPtr() const
{
  return this->A_BI;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void BodyNode::setTransformPtr(const HTr* A_BI_)
{
  RCHECK_MSG(A_BI_ != NULL, "BodyNode \"%s\": pointer is NULL", body()->name);
  this->A_BI = A_BI_;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void BodyNode::hide()
{
  _nodeSwitch->setAllChildrenOff();
}

/*******************************************************************************
* See header.
******************************************************************************/
void BodyNode::show()
{
  _nodeSwitch->setAllChildrenOn();
}

/*******************************************************************************
* See header.
******************************************************************************/
void BodyNode::setVisibility(bool visible)
{
  if (visible)
  {
    show();
  }
  else
  {
    hide();
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void BodyNode::setDynamicMeshUpdate(bool enabled)
{
  this->dynamicMeshUpdate = enabled;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool BodyNode::getDynamicMeshUpdate() const
{
  return this->dynamicMeshUpdate;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void BodyNode::setGhostMode(bool enabled, const std::string& matname)
{
  this->ghostMode = enabled;
  osg::StateSet* pStateSet = getOrCreateStateSet();

  if (ghostMode == true)
  {
    osg::Material* material = new osg::Material();
    if (!matname.empty())
    {
      RcsMaterialData* matDataPtr = getMaterial(matname);

      if (matDataPtr != NULL)
      {
        material->setAmbient(osg::Material::FRONT_AND_BACK, matDataPtr->amb);
        material->setDiffuse(osg::Material::FRONT_AND_BACK, matDataPtr->diff);
        material->setSpecular(osg::Material::FRONT_AND_BACK, matDataPtr->spec);
        material->setShininess(osg::Material::FRONT_AND_BACK,
                               matDataPtr->shininess);
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
 * For soft physics etc.
 ******************************************************************************/
void BodyNode::updateDynamicMeshes()
{
  RCSBODY_TRAVERSE_SHAPES(body())
  {
    if ((SHAPE->computeType & RCSSHAPE_COMPUTE_SOFTPHYSICS) == 0)
    {
      continue;
    }

    RcsMeshData* meshDat = (RcsMeshData*) SHAPE->userData;

    if (meshDat == NULL)
    {
      RLOG(1, "Body \"%s\" has mesh shape without RcsMeshData", body()->name);
      continue;
    }

    std::vector<MeshNode*> m = findChildrenOfType<MeshNode>(_physicsNode.get());

    for (size_t i=0; i<m.size(); ++i)
    {
      RLOG_CPP(6, "Updating mesh " << i+1 << " from " << m.size() << " with "
               << meshDat->nVertices << " vertices and " << meshDat->nFaces
               << " faces : " << getName());
      m[i]->setMesh(meshDat->vertices, meshDat->nVertices,
                    meshDat->faces, meshDat->nFaces);
    }

  }   // RCSBODY_TRAVERSE_SHAPES(body())
}

/*******************************************************************************
 * Called from node callback
 ******************************************************************************/
void BodyNode::updateCallback(osg::Node* node, osg::NodeVisitor* nv)
{
  const HTr* A = getTransformPtr();

  if (!HTr_isValid(A))
  {
    RLOG(3, "Body \"%s\" has invalid transformation - skipping update",
         getName().c_str());
    REXEC(4)
    {
      Mat3d_print((double(*)[3])A->rot);
    }
    return;
  }

  // Set the nodes transform
  setTransformation(A);

  // Dynamically create debug info, so that we don't need to do it in the
  // constructor each and every time. We can safely add it to the scene
  // graph here, since this function is called within the update callback.
  if (initializeDebugInfo && (!_debugNode.valid()))
  {
    _debugNode = addDebugInformation();
    _debugNode->setAllChildrenOn();
    addChild(_debugNode.get());
  }

  // Update debug lines
  if (debugInformationVisible() && (body()->parent) && (_debugLine.valid()))
  {
    _debugLine->clear();

    // Add the two body positions in local reference
    HTr A_12;
    HTr_invTransform(&A_12, body()->A_BI, body()->parent->A_BI);

    _debugLine->push_back(osg::Vec3(0.0, 0.0, 0.0));
    _debugLine->push_back(osg::Vec3(A_12.org[0], A_12.org[1], A_12.org[2]));

    osg::DrawArrays* ps =
      ((osg::DrawArrays*)(_debugLineGeometry)->getPrimitiveSet(0));
    ps->setCount(_debugLine->size());
    _debugLineGeometry->setVertexArray(_debugLine.get());
    _debugLineGeometry->setPrimitiveSet(0, ps);
  }

  // Mesh dynamic update
  if (getDynamicMeshUpdate())
  {
    updateDynamicMeshes();
  }
}




















}   // namespace Rcs
