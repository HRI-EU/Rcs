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
#include <osgUtil/SmoothingVisitor>

#include <fstream>

#if 0

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
Rcs::MeshNode::MeshNode(const RcsMeshData* mesh) :
  NodeBase()
{
  init();
  setMesh2(mesh->vertices, mesh->nVertices, mesh->faces, mesh->nFaces);
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::MeshNode::MeshNode(const double* vertices, unsigned int numVertices,
                        const unsigned int* faces, unsigned int numFaces) :
  NodeBase()
{
  init();
  setMesh2(vertices, numVertices, faces, numFaces);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::MeshNode::init()
{
  setName("MeshNode");
  this->geode = new osg::Geode();
  this->patPtr()->addChild(this->geode.get());
  setMaterial("PEWTER");
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::MeshNode::setMesh(const double* vertices, unsigned int numVertices,
                            const unsigned int* faces, unsigned int numFaces)
{
  clear();

  osg::ref_ptr<osg::TriangleMesh> mesh = new osg::TriangleMesh;
  mesh->setDataVariance(osg::Object::DYNAMIC);

  // Assign vertices
  osg::ref_ptr<osg::Vec3Array> v = new osg::Vec3Array;
  for (unsigned int i = 0; i < numVertices; i++)
  {
    const double* vi = &vertices[3*i];
    v->push_back(osg::Vec3(vi[0], vi[1], vi[2]));
  }

  // Assign index array
  osg::ref_ptr<osg::UIntArray> f = new osg::UIntArray;
  for (unsigned int i = 0; i < 3 * numFaces; i++)
  {
    f->push_back(faces[i]);
  }

  mesh->setVertices(v.get());
  mesh->setIndices(f.get());

  osg::ref_ptr<osg::ShapeDrawable> shape = new osg::ShapeDrawable(mesh.get());

  this->geode->addDrawable(shape.get());
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::MeshNode::clear()
{
  geode->removeDrawables(0, geode->getNumDrawables());
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::MeshNode::setMesh2(const double* vertices, unsigned int numVertices,
                             const unsigned int* faces, unsigned int numFaces)
{
  RcsMeshData mesh;
  mesh.vertices = (double*) vertices;
  mesh.faces = (unsigned int*) faces;
  mesh.nVertices = numVertices;
  mesh.nFaces = numFaces;

  osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
  geometry->setUseDisplayList(false);
  geometry->setUseVertexBufferObjects(true);

  createGeometryFromMesh2(geometry, &mesh);
  this->geode->addDrawable(geometry.get());
}

#else

namespace Rcs
{

MeshNode::MeshNode(const char* meshFile)
{
  RcsMeshData* mesh = RcsMesh_createFromFile(meshFile);
  init(mesh);
  RcsMesh_destroy(mesh);
}

MeshNode::MeshNode(const RcsMeshData* mesh)
{
  init(mesh);
}

MeshNode::~MeshNode()
{
}
MeshNode::MeshNode(const double* vertices, unsigned int numVertices,
                   const unsigned int* faces, unsigned int numFaces)
{
  RcsMeshData mesh;
  mesh.nVertices = numVertices;
  mesh.vertices = (double*)vertices;
  mesh.nFaces = numFaces;
  mesh.faces = (unsigned int*)faces;
  init(&mesh);
}

void MeshNode::init(const RcsMeshData* mesh)
{
  this->meshGeo = Rcs::createGeometryFromMesh2(mesh);
  meshGeo->setUseVertexBufferObjects(true);
  meshGeo->setUseDisplayList(false);
  meshGeo->setDataVariance(osg::Object::DYNAMIC);
  osg::StateSet* ss = getOrCreateStateSet();
  ss->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
  ss->setMode(GL_RESCALE_NORMAL, osg::StateAttribute::ON);
  addDrawable(meshGeo.get());
}

void MeshNode::setMesh(const double* vertices, unsigned int numVertices,
                       const unsigned int* faces, unsigned int numFaces)
{
  RcsMeshData mesh;
  mesh.nVertices = numVertices;
  mesh.vertices = (double*)vertices;
  mesh.nFaces = numFaces;
  mesh.faces = (unsigned int*)faces;
  update(&mesh);
}

void MeshNode::update(const RcsMeshData* mesh)
{
  //osg::Vec3Array* v = static_cast<osg::Vec3Array*>(meshGeo->getVertexArray());
  osg::ref_ptr<osg::Vec3Array> v = new osg::Vec3Array(mesh->nVertices);

  bool numVerticesChanged = (v->size() == mesh->nVertices) ? false : true;

  if (numVerticesChanged)
  {
    v->resize(mesh->nVertices);
  }

  for (unsigned int i = 0; i < mesh->nVertices; i++)
  {
    const double* vi = &mesh->vertices[i * 3];
    (*v)[i].set(vi[0], vi[1], vi[2]);
  }

  meshGeo->setVertexArray(v.get());


  //if (numVerticesChanged)
  {
    osg::ref_ptr<osg::DrawElementsUInt> indices = new osg::DrawElementsUInt(GL_TRIANGLES, 3 * mesh->nFaces);

    for (unsigned int i = 0; i < 3 * mesh->nFaces; i++)
    {
      (*indices)[i] = mesh->faces[i];
    }
    meshGeo->setPrimitiveSet(0, indices.get());
    osgUtil::SmoothingVisitor::smooth(*meshGeo, M_PI_4);
  }

  meshGeo->dirtyBound();
}

void MeshNode::setMaterial(const std::string& material, double alpha)
{
  setNodeMaterial(material, this, alpha);
}

}   // namespace Rcs

#endif
