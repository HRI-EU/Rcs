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

#include "ShapeNode.h"

#include <COSNode.h>
#include <MeshNode.h>
#include <TorusNode.h>
#include <SSRNode.h>

#include <Rcs_typedef.h>
#include <Rcs_macros.h>
#include <Rcs_graphicsUtils.h>
#include <Rcs_shape.h>
#include <Rcs_math.h>

#include <osgDB/ReadFile>
#include <osg/MatrixTransform>
#include <osg/PolygonMode>
#include <osg/Version>
#include <osg/ShapeDrawable>
#include <osg/Switch>
#include <osg/Material>
#include <osgText/Text>
#include <osg/Texture2D>


namespace Rcs
{

static std::map<std::string, osg::ref_ptr<osg::Node> > _meshBuffer;
static std::map<std::string, osg::ref_ptr<osg::Texture2D> > _textureBuffer;
static OpenThreads::Mutex _meshBufferMtx;
static OpenThreads::Mutex _textureBufferMtx;



static osg::ref_ptr<osg::Node> createMesh(const RcsShape* shape,
                                          bool& meshFromOsgReader)
{
  osg::ref_ptr<osg::Node> meshNode;
  meshFromOsgReader = false;

  if (shape->meshFile==NULL)
  {
    return meshNode;   // is invalid: meshNode.isValid()==false
  }

  // Little map that stores name-pointer pairs of mesh files. If
  // a mesh has already been loaded, we look up its pointer from
  // the map. Otherwise, we load the mesh and store its pointer
  // in the map.
  // \todo(MG): Identical meshes with different colors don't work
  // (only last assigned color is used). Please fix at some time
  std::map<std::string, osg::ref_ptr<osg::Node> >::iterator it;

  _meshBufferMtx.lock();
  for (it = _meshBuffer.begin() ; it != _meshBuffer.end(); ++it)
  {
    if (it->first == shape->meshFile)
    {
      NLOG(0, "Mesh file \"%s\" already loaded", shape->meshFile);
      meshNode = it->second;
      //setNodeMaterial(shape->color, meshNode);
      Rcs::MeshNode* mn = static_cast<Rcs::MeshNode*>(meshNode.get());

      if ((mn != NULL) && (shape->color != NULL))
      {
        NLOG(0, "Setting color of body \"%s\" (%s) to \"%s\"",
             getName().c_str(), shape->meshFile, shape->color);
        mn->setMaterial(shape->color);
      }

      break;
    }

  }
  _meshBufferMtx.unlock();



  // If no mesh was found in the _meshBuffer, we create it here.
  if (!meshNode.valid())
  {
    NLOG(0, "NO mesh file \"%s\" loaded", shape->meshFile);
    RcsMeshData* mesh = (RcsMeshData*)shape->userData;

    // If there's a mesh attached to the shape, we create a MeshNode
    // from it.
    if (mesh && mesh->nFaces>0)
    {
      NLOG(0, "Creating MeshNode from shape (%s)",
           shape->meshFile ? shape->meshFile : "NULL");

      Rcs::MeshNode* mn = new Rcs::MeshNode(mesh->vertices,
                                            mesh->nVertices,
                                            mesh->faces, mesh->nFaces);

      if (shape->color != NULL)
      {
        NLOG(0, "Setting color of body \"%s\" (%s) to \"%s\"",
             getName().c_str(), shape->meshFile, shape->color);
        mn->setMaterial(shape->color);
      }

      meshNode = mn;
    }   // (mesh && mesh->nFaces>0)
    // Otherwise, we use the OpenSceneGraph classes
    else
    {
      // fixes loading of obj without normals (doesn't work for OSG 2.8)
      osg::ref_ptr<osgDB::Options> options =
        new osgDB::Options("generateFacetNormals=true noRotation=true");
      meshNode = osgDB::readNodeFile(shape->meshFile, options.get());
      meshFromOsgReader = true;

      // We only add the non-MeshNodes to the buffer, and recreate
      // everthing else. That's due to an issue with the color
      // assignment for the mesh vertices. It somehow doesn't work. If
      // anyone has an idea, it's appreciated.
      if (meshNode.valid())
      {
        _meshBufferMtx.lock();
        _meshBuffer[std::string(shape->meshFile)] = meshNode;
        _meshBufferMtx.unlock();
      }
    }
  }

  return meshNode;
}

/*******************************************************************************
 * Update callback: Sets the bodies transformation to the node.
 *
 * \todo:
 *  resize if scale is changed
 *  resize meshes, torus
 *  change color
 ******************************************************************************/
class ShapeUpdateCallback : public osg::NodeCallback
{
public:

  ShapeUpdateCallback(ShapeNode* node) : shapeNode(node)
  {
    Vec3d_copy(this->extents, node->shape->extents);
    HTr_copy(&this->A_CB, &node->shape->A_CB);
  }

  virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
  {
    updateDynamicShapes();
    traverse(node, nv);
  }

  void addGeometry(osg::Shape* s)
  {
    geometry.push_back(s);
  }

  void addDrawable(osg::Drawable* d)
  {
    drawable.push_back(d);
  }

  inline const RcsShape* shape()
  {
    return shapeNode->shape;
  }

  void updateDynamicShapes()
  {
    const double eps = 1.0e-8;

    // Update relative transform
    if (!HTr_isEqual(&A_CB, &shape()->A_CB, eps))
    {
      shapeNode->setPosition(osg::Vec3(shape()->A_CB.org[0],
                                       shape()->A_CB.org[1],
                                       shape()->A_CB.org[2]));
      shapeNode->setAttitude(QuatFromHTr(&shape()->A_CB));
    }

    // If the extents didn't change, step to the next geode.
    if (Vec3d_isEqual(shape()->extents, extents, eps))
    {
      return;
    }

    // If the extents changed, we memorize them in the userData
    Vec3d_copy(extents, shape()->extents);

    switch (shape()->type)
    {
      // Here we adjust the size of the SSR. It is assumed that the
      // shape pointers point to the shapes in the order of their
      // creation (4 capsules and 1 box):
      // 1. Capsule: Front y-direction
      // 2. Capsule: Back y-direction
      // 3. Capsule: Right x-direction
      // 4. Capsule: Left x-direction
      // 5. Box
      case RCSSHAPE_SSR:
      {
        std::vector<osg::Shape*>::iterator sit;
        int index = 0;
        double r  = 0.5*extents[2];
        double lx = extents[0];
        double ly = extents[1];

        for (sit = geometry.begin(); sit != geometry.end(); ++sit)
        {

          switch (index)
          {
            case 0:
            {
              osg::Capsule* c = static_cast<osg::Capsule*>(*sit);
              c->setCenter(osg::Vec3(-lx / 2.0, 0.0, 0.0));
              c->setHeight(ly);
              c->setRadius(r);
            }
            break;

            case 1:
            {
              osg::Capsule* c = static_cast<osg::Capsule*>(*sit);
              c->setCenter(osg::Vec3(lx / 2.0, 0.0, 0.0));
              c->setHeight(ly);
              c->setRadius(r);
            }
            break;

            case 2:
            {
              osg::Capsule* c = static_cast<osg::Capsule*>(*sit);
              c->setCenter(osg::Vec3(0.0, ly / 2.0, 0.0));
              c->setHeight(lx);
              c->setRadius(r);
            }
            break;

            case 3:
            {
              osg::Capsule* c = static_cast<osg::Capsule*>(*sit);
              c->setCenter(osg::Vec3(0.0, -ly / 2.0, 0.0));
              c->setHeight(lx);
              c->setRadius(r);
            }
            break;

            case 4:
            {
              osg::Box* b = static_cast<osg::Box*>(*sit);
              b->setHalfLengths(osg::Vec3(0.5 * lx, 0.5 * ly, r));
            }
            break;

            default:
              RLOG(1, "Capsule index out of range: %d", index);
          }   // switch(index)

          index++;

        }   // for(sit=...

        // "Dirty" the shapes bounding box to adjust bounding box
        std::vector<osg::Drawable*>::iterator dit;

        for (dit = drawable.begin(); dit != drawable.end(); ++dit)
        {
          (*dit)->dirtyBound();
        }

      }   // case RCSSHAPE_SSR
      break;

      // Here we adjust the size of a SSL.
      case RCSSHAPE_SSL:
      {
        osg::Capsule* c = static_cast<osg::Capsule*>(geometry.front());
        c->setCenter(osg::Vec3(0.0, 0.0, extents[2]/2.0));
        c->setHeight(extents[2]);
        c->setRadius(extents[0]);

        // "Dirty" the shapes bounding box to adjust bounding box
        drawable.front()->dirtyBound();
      }
      break;

      // Here we adjust the size of a cylinder.
      case RCSSHAPE_CYLINDER:
      {
        osg::Cylinder* c = static_cast<osg::Cylinder*>(geometry.front());
        c->setHeight(extents[2]);
        c->setRadius(extents[0]);

        // "Dirty" the shapes bounding box to adjust bounding box
        drawable.front()->dirtyBound();
      }
      break;

      // Here we adjust the size of a cone.
      case RCSSHAPE_CONE:
      {
        osg::Cone* c = static_cast<osg::Cone*>(geometry.front());
        c->setHeight(extents[2]);
        c->setRadius(extents[0]);

        // "Dirty" the shapes bounding box to adjust bounding box
        drawable.front()->dirtyBound();
      }
      break;

      // Here we adjust the size of a sphere.
      case RCSSHAPE_SPHERE:
      {
        osg::Sphere* s = static_cast<osg::Sphere*>(geometry.front());
        s->setRadius(extents[0]);

        // "Dirty" the shapes bounding box to adjust bounding box
        drawable.front()->dirtyBound();
      }
      break;

      // Here we adjust the size of a BOX.
      case RCSSHAPE_BOX:
      {
        osg::Box* b = static_cast<osg::Box*>(geometry.front());
        b->setHalfLengths(osg::Vec3(0.5*extents[0], 0.5*extents[1],
                                    0.5*extents[2]));

        // "Dirty" the shapes bounding box to adjust bounding box
        drawable.front()->dirtyBound();
      }
      break;

      default:
        break;
    }

  }

protected:

  ShapeNode* shapeNode;
  std::vector<osg::Shape*> geometry;
  std::vector<osg::Drawable*> drawable;
  double extents[3];
  HTr A_CB;
};

/*******************************************************************************
 * Recursively adds the bodies collision shapes to the node.
 *
 * ShapeNode (osg::PositionAttitudeTransform) with A_CB relative to body
 *     |
 *     ---> geode (osg::Geode)
 *               |
 *               ---> sd (osg::Drawable)
 *                      |
 *                      ---> capsule, box, etc (osg::Capsule ...)

*******************************************************************************/
ShapeNode::ShapeNode(const RcsShape* shape_, bool resizeable) : shape(shape_)
{
  addShape(resizeable);

  if (shape->textureFile)
  {
    addTexture(shape->textureFile);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void ShapeNode::addShape(bool resizeable)
{
  osg::ref_ptr<osg::TessellationHints> hints = new osg::TessellationHints;
  hints->setDetailRatio(2.0);
  osg::ref_ptr<osg::Geode> geode = new osg::Geode();
  osg::ref_ptr<ShapeUpdateCallback> dnmData;
  const double* ext = shape->extents;

  if (resizeable || shape->resizeable)
  {
    dnmData = new ShapeUpdateCallback(this);
    setUpdateCallback(dnmData.get());
  }

  // This enables depth sorting by default for correctly drawing
  // transparent objects. However, there is a performance penalty.
  // Thus, it'd be better to enable depth sorting selectively

  // Set render bin to depthsorted in order to handle transparency correctly
  osg::StateSet* state_set = geode->getOrCreateStateSet();
  state_set->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
  geode->setStateSet(state_set);

  // Relative orientation of shape wrt. body
  setPosition(osg::Vec3(shape->A_CB.org[0], shape->A_CB.org[1],
                        shape->A_CB.org[2]));
  setAttitude(QuatFromHTr(&shape->A_CB));
  addChild(geode.get());

  /////////////////////////////////
  // Add a capsule to the shapeNode
  /////////////////////////////////
  if (shape->type == RCSSHAPE_SSL)
  {
    double r = ext[0], length = ext[2];
    osg::Capsule* capsule =
      new osg::Capsule(osg::Vec3(0.0, 0.0, 0.5*length), r, length);
    osg::ref_ptr<osg::Drawable> sd = new osg::ShapeDrawable(capsule, hints.get());
    geode->addDrawable(sd);
    setMaterial(shape->color, geode.get());

    // Add the information for dynamic resizing
    if (dnmData.valid())
    {
      sd->setUseDisplayList(false);
      dnmData->addGeometry(capsule);
      dnmData->addDrawable(sd);
    }

  }

  /////////////////////////////////
  // Add a box to the shapeNode
  /////////////////////////////////
  else if (shape->type == RCSSHAPE_BOX)
  {
    osg::Box* box = new osg::Box(osg::Vec3(), ext[0], ext[1], ext[2]);
    osg::ref_ptr<osg::Drawable> sd = new osg::ShapeDrawable(box, hints.get());
    geode->addDrawable(sd);
    setMaterial(shape->color, geode.get());

    if (dnmData.valid())
    {
      dnmData->addGeometry(box);
      dnmData->addDrawable(sd);
      sd->setUseDisplayList(false);
    }

  }

  /////////////////////////////////
  // Add a sphere to the shapeNode
  /////////////////////////////////
  else if (shape->type == RCSSHAPE_SPHERE)
  {
    osg::Sphere* sphere = new osg::Sphere(osg::Vec3(), shape->extents[0]);
    osg::ref_ptr<osg::Drawable> sd = new osg::ShapeDrawable(sphere, hints.get());
    geode->addDrawable(sd);
    setMaterial(shape->color, geode.get());

    if (dnmData.valid())
    {
      dnmData->addGeometry(sphere);
      dnmData->addDrawable(sd);
      sd->setUseDisplayList(false);
    }

  }

  ///////////////////////////////////////////
  // Add a sphere swept rectangle to the node
  ///////////////////////////////////////////
  else if (shape->type == RCSSHAPE_SSR)
  {
    // If the shape is not resizeable, we create a nice mesh, which in
    // wireframe mode looks a lot better than the capsule composite.
    if ((resizeable==false) && (shape->resizeable==false))
    {
      RcsMeshData* mesh = RcsMesh_createSSR(shape->extents, 64);

      // Assign vertices
      osg::ref_ptr<osg::Vec3Array> v = new osg::Vec3Array;
      for (unsigned int i = 0; i < mesh->nVertices; i++)
      {
        const double* vi = &mesh->vertices[3*i];
        v->push_back(osg::Vec3(vi[0], vi[1], vi[2]));
      }

      // Assign index array
      osg::ref_ptr<osg::UIntArray> f = new osg::UIntArray;
      for (unsigned int i = 0; i < 3 * mesh->nFaces; i++)
      {
        f->push_back(mesh->faces[i]);
      }

      osg::ref_ptr<osg::TriangleMesh> triMesh = new osg::TriangleMesh;
      triMesh->setDataVariance(osg::Object::DYNAMIC);
      triMesh->setVertices(v.get());
      triMesh->setIndices(f.get());

      osg::ref_ptr<osg::Drawable> sd = new osg::ShapeDrawable(triMesh.get());
      geode->addDrawable(sd);
      setMaterial(shape->color, geode.get());

      RcsMesh_destroy(mesh);
    }
    // If the shape is resizeable, we create a capsule box composite,
    // which we can conveniently resize if needed.
    else
    {
      double r  = 0.5*ext[2];
      double lx = ext[0];
      double ly = ext[1];

      osg::ref_ptr<osg::CompositeShape> compoShape = new osg::CompositeShape;

      // Side 1: Front y-direction
      osg::Capsule* cSSR1 =
        new osg::Capsule(osg::Vec3(-lx / 2.0, 0.0, 0.0), r, ly);
      cSSR1->setRotation(osg::Quat(osg::inDegrees(90.0f),
                                   osg::Vec3(1.0f, 0.0f, 0.0f)));
      compoShape->addChild(cSSR1);

      // Side 2: Back y-direction
      osg::Capsule* cSSR2 =
        new osg::Capsule(osg::Vec3(lx / 2.0, 0.0, 0.0), r, ly);
      cSSR2->setRotation(osg::Quat(osg::inDegrees(90.0f),
                                   osg::Vec3(1.0f, 0.0f, 0.0f)));
      compoShape->addChild(cSSR2);

      // Side 3: Right x-direction
      osg::Capsule* cSSR3 =
        new osg::Capsule(osg::Vec3(0.0, ly / 2.0, 0.0), r, lx);
      cSSR3->setRotation(osg::Quat(osg::inDegrees(90.0f),
                                   osg::Vec3(0.0f, 1.0f, 0.0f)));
      compoShape->addChild(cSSR3);

      // Side 4: Left x-direction
      osg::Capsule* cSSR4 =
        new osg::Capsule(osg::Vec3(0.0, -ly / 2.0, 0.0), r, lx);
      cSSR4->setRotation(osg::Quat(osg::inDegrees(90.0f),
                                   osg::Vec3(0.0f, 1.0f, 0.0f)));
      compoShape->addChild(cSSR4);

      // Box part
      osg::Box* bSSR = new osg::Box(osg::Vec3(), lx, ly, 2.0 * r);
      compoShape->addChild(bSSR);

      osg::ref_ptr<osg::Drawable> sd = new osg::ShapeDrawable(compoShape.get(), hints.get());
      geode->addDrawable(sd);
      sd->setUseDisplayList(false);
      setMaterial(shape->color, geode.get());

      // Add the information for dynamic resizing
      dnmData->addGeometry(cSSR1);
      dnmData->addGeometry(cSSR2);
      dnmData->addGeometry(cSSR3);
      dnmData->addGeometry(cSSR4);
      dnmData->addGeometry(bSSR);
      dnmData->addDrawable(sd);
    }   // resizeable

  }

  /////////////////////////////
  // Add a cylinder to the node
  /////////////////////////////
  else if (shape->type == RCSSHAPE_CYLINDER)
  {
    osg::Cylinder* cylinder = new osg::Cylinder(osg::Vec3(), ext[0], ext[2]);
    osg::ref_ptr<osg::Drawable> sd = new osg::ShapeDrawable(cylinder, hints.get());
    geode->addDrawable(sd);
    setMaterial(shape->color, geode.get());

    if (dnmData.valid())
    {
      dnmData->addGeometry(cylinder);
      dnmData->addDrawable(sd);
      sd->setUseDisplayList(false);
    }

  }

  /////////////////////////////
  // Add a cone to the node
  /////////////////////////////
  else if (shape->type == RCSSHAPE_CONE)
  {
    osg::Cone* cone = new osg::Cone();
    cone->setRadius(shape->extents[0]);
    cone->setHeight(shape->extents[2]);
    // For some reason the cone shape is shifted along the z-axis by the
    // below compensated base offset value.
    cone->setCenter(osg::Vec3(0.0f, 0.0f, -cone->getBaseOffset()));
    osg::ref_ptr<osg::Drawable> sd = new osg::ShapeDrawable(cone, hints.get());
    geode->addDrawable(sd);
    setMaterial(shape->color, geode.get());

    if (dnmData.valid())
    {
      dnmData->addGeometry(cone);
      dnmData->addDrawable(sd);
      sd->setUseDisplayList(false);
    }

  }

  /////////////////////////////
  // Add mesh file
  /////////////////////////////
  else if (shape->type == RCSSHAPE_MESH)
  {
    bool meshFromOsgReader;
    osg::ref_ptr<osg::Node> meshNode = createMesh(shape, meshFromOsgReader);

    if (meshNode.valid())
    {
      addChild(meshNode.get());

      // The mesh is only scaled if it has been read from the
      // osgDB::readNodeFile class. Otherwise, the scaling has already
      // been done during parsing the graph (meshes need to be consistent
      // with the physics simulator).
      if (meshFromOsgReader == true)
      {
        setScale(osg::Vec3(shape->scale, shape->scale, shape->scale));
      }

      setMaterial(shape->color, this);
    }

  }

  ////////////////////////////////////////////////
  // Add a reference coordinate system to the node
  ////////////////////////////////////////////////
  else if (shape->type == RCSSHAPE_REFFRAME)
  {
    osg::ref_ptr<COSNode> cFrame = new COSNode(shape->scale,
                                               ext[0], ext[1], ext[2]);
    cFrame->setPosition(osg::Vec3(shape->A_CB.org[0], shape->A_CB.org[1],
                                  shape->A_CB.org[2]));
    cFrame->setRotation(QuatFromHTr(&shape->A_CB));
    addChild(cFrame.get());
    frames.push_back(cFrame);
  }

  /////////////////////////////
  // Add a torus to the node
  /////////////////////////////
  else if (shape->type == RCSSHAPE_TORUS)
  {
    osg::ref_ptr<osg::Drawable> sd = TorusNode::createGeometry(ext[0], ext[2]);
    geode->addDrawable(sd);
    setMaterial(shape->color, geode.get());
  }


  /////////////////////////////
  // Add a octree to the node
  /////////////////////////////
  else if (shape->type == RCSSHAPE_OCTREE)
  {
#ifdef USE_OCTOMAP
    octomap::OcTree* tree;
    if (shape->userData == NULL && shape->meshFile)
    {
      tree = new octomap::OcTree(shape->meshFile);
    }
    else
    {
      tree = reinterpret_cast<octomap::OcTree*>(shape->userData);
    }

    if (tree)
    {
      osg::Geometry* octomap_geom =
        Rcs::OctomapNode::createOctomapGeometry(tree, tree->getTreeDepth());
      geode->addDrawable(octomap_geom);

      // color
      RcsMaterialData* material = NULL;

      if (shape->color != NULL)
      {
        material = getMaterial(shape->color);
      }

      if (material == NULL)
      {
        RLOG(5, "Failed to set color \"%s\"",
             shape->color ? shape->color : "NULL");
      }
      else
      {
        osg::Vec4Array* color = new osg::Vec4Array();
        color->push_back(material->diff);
        octomap_geom->setColorArray(color);
        octomap_geom->setColorBinding(osg::Geometry::BIND_OVERALL);
      }
    }
#else
    RLOGS(0, "No octomap support");
#endif   // USE_OCTOMAP
  }

  /////////////////////////////////
  // RCSSHAPE_POINT: 1mm diameter
  /////////////////////////////////
  else if (shape->type == RCSSHAPE_POINT)
  {
    osg::Sphere* sphere = new osg::Sphere(osg::Vec3(), 0.005);
    osg::ref_ptr<osg::Drawable> sd = new osg::ShapeDrawable(sphere, hints.get());
    geode->addDrawable(sd);
    setMaterial(shape->color, geode.get());
  }

  /////////////////////////////////
  // RCSSHAPE_MARKER
  /////////////////////////////////
  else if (shape->type == RCSSHAPE_MARKER)
  {
    NLOG(5, "Adding marker %d id %d of \"%s\" with length %f ", shapeCount,
         *((int*)shape->userData), getName().c_str(), shape->extents[2]);
    osg::Box* box = new osg::Box(osg::Vec3(), ext[2], ext[2], 0.001);
    osg::ref_ptr<osg::Drawable> sd = new osg::ShapeDrawable(box, hints.get());
    geode->addDrawable(sd);
    setMaterial(shape->color, geode.get());
  }

  ////////////////////////
  // Ignore unknown shapes
  ////////////////////////
  else
  {
    RLOG(1, "Shape type %d of body \"%s\" undefined", shape->type,
         getName().c_str());
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
void ShapeNode::addTexture(const char* textureFile)
{
  // read the texture file
  osg::ref_ptr<osg::Texture2D> texture;

  // Little map that stores name-pointer pairs of texture files. If
  // a texture has already been loaded, we look up its pointer from
  // the map. Otherwise, we load the texture and store its pointer
  // in the map.
  std::map<std::string, osg::ref_ptr<osg::Texture2D> >::iterator it;

  _textureBufferMtx.lock();
  for (it = _textureBuffer.begin() ; it != _textureBuffer.end(); ++it)
  {
    if (it->first == textureFile)
    {
      NLOG(0, "Texture file \"%s\" already loaded", textureFile);
      texture = it->second;
      break;

    }

  }
  _textureBufferMtx.unlock();

  // texture not found, let's create new one
  if (!texture.valid())
  {

    // load an image by reading a file:
    osg::ref_ptr<osg::Image> texture_image = osgDB::readImageFile(textureFile);

    if (texture_image.valid() == false)
    {
      RLOG(1, "couldn't load texture file  \"%s\" for body \"%s\", "
           "omitting...", textureFile, getName().c_str());
    }
    else
    {
      texture = new osg::Texture2D;
      // protect from being optimized away as static state:
      texture->setDataVariance(osg::Object::DYNAMIC);

      texture->setFilter(osg::Texture::MIN_FILTER,
                         osg::Texture::LINEAR_MIPMAP_LINEAR);
      texture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
      texture->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP);
      texture->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP);
      // Assign the texture to the image we read from file
      texture->setImage(texture_image.get());
      _textureBufferMtx.lock();
      _textureBuffer[std::string(textureFile)] = texture;
      _textureBufferMtx.unlock();
    }
  }

  if (texture.valid())
  {

    // this would extend previous setting, e.g. by color settings
    for (size_t i = 0; i < getNumChildren(); i++)
    {
      osg::StateSet* state_set = getChild(i)->getOrCreateStateSet();
      osg::Material* material = new osg::Material;
      material->setAmbient(osg::Material::FRONT_AND_BACK,
                           osg::Vec4(0.6, 0.6, 0.6, 1.0));
      material->setDiffuse(osg::Material::FRONT_AND_BACK,
                           osg::Vec4(1.0, 1.0, 1.0, 1.0));
      material->setSpecular(osg::Material::FRONT_AND_BACK,
                            osg::Vec4(0.3, 0.3, 0.3, 1.0));
      material->setShininess(osg::Material::FRONT_AND_BACK, 100.0);
      state_set->setAttributeAndModes(material,
                                      osg::StateAttribute::OVERRIDE |
                                      osg::StateAttribute::ON);
      state_set->setTextureAttributeAndModes(0, texture.get(),
                                             osg::StateAttribute::OVERRIDE |
                                             osg::StateAttribute::ON);
    }
    //      // Create a new StateSet with default settings
    //      // this means the old color settings are overridden
    //      osg::StateSet* stateOne = new osg::StateSet();
    //      // Assign texture unit 0 of our new StateSet to the texture
    //      // we just created and enable the texture
    //      stateOne->setTextureAttributeAndModes(0, texture,
    //                           osg::StateAttribute::OVERRIDE |
    //                           osg::StateAttribute::ON);
    //      // Associate this state set with the Geode
    //      geode->setStateSet(stateOne);
  }
}

ShapeNode::~ShapeNode()
{
}

void ShapeNode::setMaterial(const char* color, osg::Node* node)
{
  if (color)
  {
    setNodeMaterial(color, node);
  }
}

void ShapeNode::displayFrames(bool visibility)
{
  if (visibility==true)
  {
    for (size_t i=0; i<frames.size(); ++i)
    {
      frames[i]->show();
    }
  }
  else
  {
    for (size_t i=0; i<frames.size(); ++i)
    {
      frames[i]->hide();
    }
  }
}

void ShapeNode::toggleFrames()
{
  for (size_t i=0; i<frames.size(); ++i)
  {
    frames[i]->toggle();
  }
}

}   // namespace Rcs
