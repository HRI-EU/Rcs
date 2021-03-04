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

#include "ContactsNode.h"

#include <Rcs_macros.h>
#include <Rcs_math.h>
#include <RcsViewer.h>

#include <osg/Geode>
#include <osg/LineWidth>



/*******************************************************************************
 * Update callback: Calls the contacts update function.
 ******************************************************************************/
namespace Rcs
{

class ContactUpdateCallback : public osg::NodeCallback
{

public:

  ContactUpdateCallback(Rcs::ContactsNode* node) : contactsNode(node)
  {
  }

  virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
  {
    contactsNode->updateContacts();
    traverse(node, nv);
  }

  Rcs::ContactsNode* contactsNode;
};

}

/*******************************************************************************
 * See header.
 ******************************************************************************/
Rcs::ContactsNode::ContactsNode(PhysicsBase* sim, double factor,
                                const std::string& material):
  forceFactor(factor),
  drawContacts(false),
  sim(sim)
{
  // create the Geode (Geometry Node) to contain all our geometry objects.
  osg::ref_ptr<osg::Geode> geode = new osg::Geode();
  addChild(geode.get());
  setNodeMaterial(material, geode.get());
  geode->setNodeMask(geode->getNodeMask() & ~ReceivesShadowTraversalMask);
  geode->setNodeMask(geode->getNodeMask() & ~CastsShadowTraversalMask);

  // Create lines
  this->linesGeom = new osg::Geometry();

  // We'll prealloacte the vertex array to the size we need and then
  // simple set them as array elements, 2 points makes 1 line segments.
  this->vertices = new osg::Vec3Array(2);
  (*this->vertices)[0].set(0.0, 0.0, 0.0);
  (*this->vertices)[1].set(0.0, 0.0, 0.0);

  // pass the created vertex array to the points geometry object.
  this->linesGeom->setVertexArray(this->vertices.get());

  // set the normal in the same way color.
  osg::Vec3Array* normals = new osg::Vec3Array;
  normals->push_back(osg::Vec3(0.0f, -1.0f, 0.0f));
  this->linesGeom->setNormalArray(normals);
  this->linesGeom->setNormalBinding(osg::Geometry::BIND_OVERALL);

  // add primitive set
  this->lineSet = new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 2);
  this->linesGeom->addPrimitiveSet(this->lineSet.get());

  // Adjust the line width: Causes trouble with PSSM shadow (no clue why)
  osg::StateSet* linestateset = geode->getOrCreateStateSet();
  osg::LineWidth* linewidth = new osg::LineWidth();
  linewidth->setWidth(3.0f);
  linestateset->setAttribute(linewidth, osg::StateAttribute::ON);
  linestateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
  geode->setStateSet(linestateset);

  // add the points geomtry to the geode.
  geode->addDrawable(this->linesGeom.get());

  // Add event callback handler
  //setEventCallback(new ContactsNodeHandler(this));
  setUpdateCallback(new ContactUpdateCallback(this));
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Rcs::ContactsNode::scaleLineLength(double factor)
{
  this->forceFactor *= factor;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Rcs::ContactsNode::setForceFactor(double factor)
{
  this->forceFactor = factor;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
double Rcs::ContactsNode::getForceFactor() const
{
  return this->forceFactor;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::ContactsNode::toggle()
{
  this->drawContacts = !this->drawContacts;

  return this->drawContacts;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Rcs::ContactsNode::show()
{
  this->drawContacts = true;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Rcs::ContactsNode::hide()
{
  this->drawContacts = false;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::ContactsNode::isVisible() const
{
  return this->drawContacts;
}

/*******************************************************************************
 * Draws a line for each contact
 ******************************************************************************/
bool Rcs::ContactsNode::updateContacts()
{
  this->vertices->clear();
  unsigned int count = 0;

  if (this->drawContacts && this->sim)
  {
    PhysicsBase::Contacts contacts = sim->getContacts();
    count = contacts.size();

    for (PhysicsBase::Contacts::iterator it = contacts.begin();
         it != contacts.end(); ++it)
    {
      // Point 1
      vertices->push_back(osg::Vec3d(it->pos[0], it->pos[1], it->pos[2]));

      // Point 2
      vertices->push_back(osg::Vec3d(it->pos[0]+forceFactor*it->force[0],
                                     it->pos[1]+forceFactor*it->force[1],
                                     it->pos[2]+forceFactor*it->force[2]));
    }
  }

  this->lineSet->setCount(count * 2);  // two vertices per contact
  this->linesGeom->setVertexArray(this->vertices.get());

  return true;
}
