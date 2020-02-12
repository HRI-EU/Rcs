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

#include "MeshNode.h"
#include "Rcs_graphicsUtils.h"

#include <Rcs_macros.h>
#include <Rcs_utils.h>
#include <Rcs_mesh.h>

#include <osg/Material>

#include <fstream>



/*******************************************************************************
 *
 ******************************************************************************/
Rcs::MeshNode::MeshNode() : NodeBase()
{
  init();
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::MeshNode::MeshNode(const char* meshFile) : NodeBase()
{
  init();

  RcsMeshData* data = RcsMesh_createFromFile(meshFile);

  if (data != NULL)
  {
    setMesh(data->vertices, data->nVertices, data->faces, data->nFaces);
    RLOG(5, "Success to create MeshNode from file \"%s\"",
         meshFile ? meshFile : "NULL");
    RcsMesh_destroy(data);
  }
  else
  {
    RLOG(1, "Failed to create MeshNode from file \"%s\"",
         meshFile ? meshFile : "NULL");
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::MeshNode::MeshNode(const double* vertices, unsigned int numVertices,
                        const unsigned int* faces, unsigned int numFaces) :
  NodeBase()
{
  init();
  setMesh(vertices, numVertices, faces, numFaces);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::MeshNode::init()
{
  setName("MeshNode");
  this->shape = NULL;
  this->geode = new osg::Geode();
  this->patPtr()->addChild(this->geode);

  // Assign some material properties (PEWTER)
  osg::Vec4d amb  = osg::Vec4d(0.105882, 0.058824, 0.113725, 1.0);
  osg::Vec4d diff = osg::Vec4d(0.427451, 0.470588, 0.541176, 1.0);
  osg::Vec4d spec = osg::Vec4d(0.333333, 0.333333, 0.521569, 1.0);
  double shininess = 9.84615;

  osg::ref_ptr<osg::Material> material = new osg::Material;
  material->setAmbient(osg::Material::FRONT_AND_BACK, amb);
  material->setDiffuse(osg::Material::FRONT_AND_BACK, diff);
  material->setSpecular(osg::Material::FRONT_AND_BACK, spec);
  material->setShininess(osg::Material::FRONT_AND_BACK, shininess);

  osg::ref_ptr<osg::StateSet> stateset = getOrCreateStateSet();
  stateset->setAttributeAndModes(material.get(),
                                 osg::StateAttribute::OVERRIDE |
                                 osg::StateAttribute::ON);
  // Makes material scale-invariant
  stateset->setMode(GL_RESCALE_NORMAL, osg::StateAttribute::ON);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::MeshNode::setColor(const char* color)
{
  RcsMaterialData* matDataPtr = getMaterial(color);

  if (matDataPtr == NULL)
  {
    RLOG(1, "Can't set material \"%s\" - not in xml file!", color);
    return;
  }

  osg::ref_ptr<osg::Material> material = new osg::Material;
  material->setAmbient(osg::Material::FRONT_AND_BACK, matDataPtr->amb);
  material->setDiffuse(osg::Material::FRONT_AND_BACK, matDataPtr->diff);
  material->setSpecular(osg::Material::FRONT_AND_BACK, matDataPtr->spec);
  material->setShininess(osg::Material::FRONT_AND_BACK, matDataPtr->shininess);

  osg::ref_ptr<osg::StateSet> stateset = getOrCreateStateSet();

  stateset->setMode(GL_BLEND, osg::StateAttribute::OVERRIDE |
                    osg::StateAttribute::ON);
  stateset->setMode(GL_LIGHTING, osg::StateAttribute::OVERRIDE |
                    osg::StateAttribute::ON);
  stateset->setMode(GL_ALPHA_TEST, osg::StateAttribute::ON);

  // removes artifacts inside of transparent objects
  stateset->setMode(GL_CULL_FACE, osg::StateAttribute::ON);

  stateset->setAttributeAndModes(material.get(),
                                 osg::StateAttribute::OVERRIDE |
                                 osg::StateAttribute::ON);

  // Makes material scale-invariant
  stateset->setMode(GL_RESCALE_NORMAL, osg::StateAttribute::ON);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::MeshNode::setMesh(const double* vertices, unsigned int numVertices,
                            const unsigned int* faces, unsigned int numFaces)
{
  clear();

  osg::ref_ptr<osg::TriangleMesh> mesh = new osg::TriangleMesh;

  // Assign vertices
  osg::ref_ptr<osg::Vec3Array> v = new osg::Vec3Array;
  for (unsigned int i = 0; i < numVertices; i++)
  {
    const unsigned i3 = i * 3;
    v->push_back(osg::Vec3(vertices[i3 + 0], vertices[i3 + 1], vertices[i3 + 2]));
  }

  // Assign index array
  osg::ref_ptr<osg::UIntArray> f = new osg::UIntArray;
  for (unsigned int i = 0; i < 3 * numFaces; i++)
  {
    f->push_back(faces[i]);
  }

  mesh->setVertices(v.get());
  mesh->setIndices(f.get());

  this->shape = new osg::ShapeDrawable(mesh.get());
  this->geode->addDrawable(this->shape);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::MeshNode::clear()
{
  if (this->shape)
  {
    this->geode->removeDrawable(this->shape);
    shape = NULL;
  }
}
