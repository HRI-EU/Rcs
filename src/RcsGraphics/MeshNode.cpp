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

#include "MeshNode.h"
#include "Rcs_graphicsUtils.h"

#include <Rcs_macros.h>
#include <Rcs_utils.h>
#include <Rcs_mesh.h>

#include <osg/Material>
#include <osgUtil/SmoothingVisitor>

#include <fstream>


/*******************************************************************************
 *
 ******************************************************************************/
class MeshUpdateCB : public osg::NodeCallback
{
public:

  MeshUpdateCB(Rcs::MeshNode* mnd) : meshNodePtr(mnd), mesh(NULL)
  {
  }

  ~MeshUpdateCB()
  {
    RcsMesh_destroy(mesh);
  }

  void setMesh(const RcsMeshData* newMesh)
  {
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(meshMtx);
    if (!mesh)
    {
      this->mesh = RcsMesh_clone(newMesh);
    }
    else
    {
      RcsMesh_copy(this->mesh, newMesh);
    }
  }

  void operator()(osg::Node* node, osg::NodeVisitor* nv)
  {

    {
      OpenThreads::ScopedLock<OpenThreads::Mutex> lock(meshMtx);
      if (this->mesh)
      {
        meshNodePtr->updateGraphics(mesh);
      }
    }

    traverse(node, nv);
  }

  Rcs::MeshNode* meshNodePtr;
  RcsMeshData* mesh;
  OpenThreads::Mutex meshMtx;
};



/*******************************************************************************
 *
 ******************************************************************************/
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

void MeshNode::makeDynamic()
{
  setUpdateCallback(new MeshUpdateCB(this));
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

void MeshNode::clearMesh()
{
  RcsMeshData mesh;
  mesh.nVertices = 0;
  mesh.vertices = NULL;
  mesh.nFaces = 0;
  mesh.faces = NULL;
  update(&mesh);
}

void MeshNode::update(const RcsMeshData* mesh)
{
  MeshUpdateCB* cb = dynamic_cast<MeshUpdateCB*>(getUpdateCallback());

  if (!cb)
  {
    updateGraphics(mesh);
  }
  else
  {
    cb->setMesh(mesh);
  }

}

void MeshNode::updateGraphics(const RcsMeshData* mesh)
{
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
    osg::ref_ptr<osg::DrawElementsUInt> indices =
      new osg::DrawElementsUInt(GL_TRIANGLES, 3 * mesh->nFaces);

    for (unsigned int i = 0; i < 3 * mesh->nFaces; i++)
    {
      (*indices)[i] = mesh->faces[i];
    }
    meshGeo->setPrimitiveSet(0, indices.get());
    osgUtil::SmoothingVisitor::smooth(*meshGeo, M_PI_4);
  }

  meshGeo->dirtyBound();
}

}   // namespace Rcs
