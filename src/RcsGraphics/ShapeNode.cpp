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

#include "COSNode.h"
#include "MeshNode.h"
#include "TorusNode.h"
#include "SSRNode.h"
#include "CapsuleNode.h"
#include "RcsViewer.h"

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

static std::map<std::string, osg::ref_ptr<osg::Texture2D> > _textureBuffer;
static OpenThreads::Mutex _textureBufferMtx;



static osg::ref_ptr<osg::Node> createMeshNode(const RcsShape* shape,
                                              bool& meshFromOsgReader)
{
  static std::map<std::string, osg::ref_ptr<osg::Node> > _meshBuffer;
  static OpenThreads::Mutex _meshBufferMtx;

  osg::ref_ptr<osg::Node> meshNode;
  meshFromOsgReader = false;

  // Little map that stores name-pointer pairs of mesh files. If
  // a mesh has already been loaded, we look up its pointer from
  // the map. Otherwise, we load the mesh and store its pointer
  // in the map.
  // \todo: Identical meshes with different colors don't work
  // (only last assigned color is used). Please fix at some time
  std::map<std::string, osg::ref_ptr<osg::Node> >::iterator it;

  _meshBufferMtx.lock();
  it = _meshBuffer.find(std::string(shape->meshFile));
  if (it != _meshBuffer.end())
  {
    NLOG(0, "Creating MeshNode %s from _meshBuffer", shape->meshFile);
    meshNode = it->second;
    Rcs::MeshNode* mn = dynamic_cast<Rcs::MeshNode*>(meshNode.get());
    if (mn)
    {
      mn->setMaterial(std::string(shape->color));
    }
    _meshBufferMtx.unlock();
    return meshNode;
  }
  _meshBufferMtx.unlock();


  // If no mesh was found in the _meshBuffer, we create it here.
  NLOG(0, "No mesh file \"%s\" loaded", shape->meshFile);
  RcsMeshData* mesh = shape->mesh;

  // If there's a mesh attached to the shape, we create a MeshNode
  // from it.
  if (mesh && mesh->nFaces>0)
  {
    NLOG(0, "Creating MeshNode from shape (%s)", shape->meshFile);
    osg::ref_ptr<Rcs::MeshNode> mn;
    mn = new Rcs::MeshNode(mesh);
    mn->setMaterial(shape->color);
    _meshBufferMtx.lock();
    _meshBuffer[std::string(shape->meshFile)] = mn;
    _meshBufferMtx.unlock();
    return mn;
  }


  // Otherwise, we use the OpenSceneGraph classes
  // fixes loading of obj without normals (doesn't work for OSG 2.8)
  NLOG(0, "Creating MeshNode from osg::NodeFileReader (%s)", shape->meshFile);
  osg::ref_ptr<osgDB::Options> options;
  options = new osgDB::Options("generateFacetNormals=true noRotation=true");
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


  return meshNode;
}

/*******************************************************************************
 * Update shape geometry when labelled as resizeable
 *
 * \todo:
 *  resize if scale is changed
 *  resize meshes, torus
 *  change color
 ******************************************************************************/
ShapeNode::ShapeUpdater::ShapeUpdater(ShapeNode* node) : shapeNode(node)
{
  Vec3d_copy(this->extents, node->shape->extents);
  HTr_copy(&this->A_CB, &node->shape->A_CB);
  color = std::string(node->shape->color);
}

void ShapeNode::ShapeUpdater::addDrawable(osg::Drawable* d)
{
  drawable = d;
}

void ShapeNode::ShapeUpdater::addSubNode(osg::Node* nd)
{
  subNode = nd;
}

const RcsShape* ShapeNode::ShapeUpdater::shape()
{
  return shapeNode->shape;
}

void ShapeNode::ShapeUpdater::updateDynamicShapes()
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

  // Apply new color once changed
  if (!STREQ(shape()->color, this->color.c_str()))
  {
    color = std::string(shape()->color);
    setNodeMaterial(color, shapeNode);
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
    case RCSSHAPE_SSR:
    {
      SSRGeometry* geo = dynamic_cast<SSRGeometry*>(drawable.get());
      geo->update(extents);
    }
    break;

    case RCSSHAPE_SSL:
    {
      CapsuleGeometry* geo = dynamic_cast<CapsuleGeometry*>(subNode.get());
      geo->update(extents[0], extents[2]);
    }
    break;

    case RCSSHAPE_CYLINDER:
    case RCSSHAPE_CONE:
    {
      shapeNode->setScale(osg::Vec3(extents[0], extents[0], extents[2]));
    }
    break;

    case RCSSHAPE_SPHERE:
    {
      shapeNode->setScale(osg::Vec3(extents[0], extents[0], extents[0]));
    }
    break;

    case RCSSHAPE_BOX:
    {
      shapeNode->setScale(osg::Vec3(extents[0], extents[1], extents[2]));
    }
    break;

    case RCSSHAPE_TORUS:
    {
      osg::Geometry* geo = dynamic_cast<osg::Geometry*>(drawable.get());
      RCHECK(geo);
      TorusNode::resize(extents[0], extents[2], geo);
      geo->dirtyBound();
    }
    break;

    default:
      break;
  }

}

/*******************************************************************************
 * Recursively adds the bodies collision shapes to the node.
 *
 * ShapeNode (osg::MatrixTransform) with A_CB relative to body
 *     |
 *     ---> geode (osg::Geode)
 *               |
 *               ---> sd (osg::Drawable)
 *                      |
 *                      ---> capsule, box, etc (osg::Capsule ...)

*******************************************************************************/
ShapeNode::ShapeNode(const RcsShape* shape_, bool resizeable) : shape(shape_)
{
  RCHECK(shape);
  addShape(resizeable);

  if (strlen(shape->textureFile)>0)
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
  osg::ref_ptr<osg::Geode> geode = new osg::Geode();
  const double* ext = shape->extents;

  if (resizeable || shape->resizeable)
  {
    resizeable = true;
    hints->setDetailRatio(0.5);
    shapeUpdater = new ShapeUpdater(this);
  }

  // This enables depth sorting by default for correctly drawing
  // transparent objects. However, there is a performance penalty.
  // Thus, it'd be better to enable depth sorting selectively

  // Set render bin to depthsorted in order to handle transparency correctly
  osg::StateSet* ss = geode->getOrCreateStateSet();
  ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
  geode->setStateSet(ss);

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
    int nSeg = shapeUpdater.valid() ? 16 : 32;
    osg::ref_ptr<osg::Group> cap = new CapsuleGeometry(r, length, resizeable, nSeg);
    setNodeMaterial(shape->color, cap.get());
    addChild(cap.get());

    if (shapeUpdater.valid())
    {
      shapeUpdater->addSubNode(cap.get());
    }
  }

  ///////////////////////////////////////////
  // Add a sphere swept rectangle to the node
  ///////////////////////////////////////////
  else if (shape->type == RCSSHAPE_SSR)
  {
    int nSeg = shapeUpdater.valid() ? 6 : 12;
    osg::ref_ptr<SSRGeometry> g = new SSRGeometry(ext, nSeg);
    geode->addDrawable(g.get());
    setNodeMaterial(shape->color, geode.get());
    if (shapeUpdater.valid())
    {
      shapeUpdater->addDrawable(g.get());
      g->setUseDisplayList(false);
    }

  }

  /////////////////////////////////
  // Add a box to the shapeNode
  /////////////////////////////////
  else if (shape->type == RCSSHAPE_BOX)
  {
    osg::Box* box = new osg::Box(osg::Vec3(), 1.0);
    osg::ShapeDrawable* sd = new osg::ShapeDrawable(box, hints.get());
    sd->setUseDisplayList(!resizeable);
    setScale(osg::Vec3(ext[0], ext[1], ext[2]));
    ss->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
    ss->setMode(GL_RESCALE_NORMAL, osg::StateAttribute::ON);
    geode->addDrawable(sd);
    setNodeMaterial(shape->color, geode.get());
  }

  /////////////////////////////////
  // Add a sphere to the shapeNode
  /////////////////////////////////
  else if (shape->type == RCSSHAPE_SPHERE)
  {
    osg::Sphere* sphere = new osg::Sphere(osg::Vec3(), 1.0);
    osg::Drawable* sd = new osg::ShapeDrawable(sphere, hints.get());
    sd->setUseDisplayList(!resizeable);
    geode->addDrawable(sd);
    setNodeMaterial(shape->color, geode.get());
    setScale(osg::Vec3(ext[0], ext[0], ext[0]));
    ss->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
    ss->setMode(GL_RESCALE_NORMAL, osg::StateAttribute::ON);
  }

  /////////////////////////////
  // Add a cylinder to the node
  /////////////////////////////
  else if (shape->type == RCSSHAPE_CYLINDER)
  {
    osg::Cylinder* cylinder = new osg::Cylinder(osg::Vec3(), 1.0, 1.0);
    osg::Drawable* sd = new osg::ShapeDrawable(cylinder, hints.get());
    sd->setUseDisplayList(!resizeable);
    geode->addDrawable(sd);
    setNodeMaterial(shape->color, geode.get());
    setScale(osg::Vec3(ext[0], ext[0], ext[2]));
    ss->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
    ss->setMode(GL_RESCALE_NORMAL, osg::StateAttribute::ON);
  }

  /////////////////////////////
  // Add a cone to the node
  /////////////////////////////
  else if (shape->type == RCSSHAPE_CONE)
  {
    osg::Cone* cone = new osg::Cone();
    cone->setRadius(1.0);
    cone->setHeight(1.0);
    // For some reason the cone shape is shifted along the z-axis by the
    // below compensated base offset value.
    cone->setCenter(osg::Vec3(0.0f, 0.0f, -cone->getBaseOffset()));
    osg::Drawable* sd = new osg::ShapeDrawable(cone, hints.get());
    sd->setUseDisplayList(!resizeable);
    geode->addDrawable(sd);
    setNodeMaterial(shape->color, geode.get());
    setScale(osg::Vec3(ext[0], ext[0], ext[2]));
    ss->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
    ss->setMode(GL_RESCALE_NORMAL, osg::StateAttribute::ON);
  }

  /////////////////////////////
  // Add mesh file
  /////////////////////////////
  else if (shape->type == RCSSHAPE_MESH)
  {
    bool meshFromOsgReader;
    osg::ref_ptr<osg::Node> meshNode = createMeshNode(shape, meshFromOsgReader);

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
        ss->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
        ss->setMode(GL_RESCALE_NORMAL, osg::StateAttribute::ON);
      }

      setNodeMaterial(shape->color, this);
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
    frame = cFrame;
  }

  /////////////////////////////
  // Add a torus to the node
  /////////////////////////////
  else if (shape->type == RCSSHAPE_TORUS)
  {
    osg::ref_ptr<osg::Geometry> g = TorusNode::createGeometry(ext[0], ext[2]);
    geode->addDrawable(g.get());

    if (shapeUpdater.valid())
    {
      shapeUpdater->addDrawable(g);
      g->setUseDisplayList(false);
    }

    setNodeMaterial(shape->color, geode.get());
  }


  /////////////////////////////
  // Add a octree to the node
  /////////////////////////////
  else if (shape->type == RCSSHAPE_OCTREE)
  {
#ifdef USE_OCTOMAP
    octomap::OcTree* tree;
    if (shape->userData == NULL)
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
    osg::Drawable* sd = new osg::ShapeDrawable(sphere, hints.get());
    geode->addDrawable(sd);
    setNodeMaterial(shape->color, geode.get());
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
 *\todo: Make separate texture buffer method similar to mesh buffer
 ******************************************************************************/
bool ShapeNode::addTexture(const char* textureFile)
{
  bool success = false;

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
    if (it->first == std::string(textureFile))
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
      osg::StateSet* ss = getChild(i)->getOrCreateStateSet();
      osg::Material* material = new osg::Material;
      material->setAmbient(osg::Material::FRONT_AND_BACK,
                           osg::Vec4(0.6, 0.6, 0.6, 1.0));
      material->setDiffuse(osg::Material::FRONT_AND_BACK,
                           osg::Vec4(1.0, 1.0, 1.0, 1.0));
      material->setSpecular(osg::Material::FRONT_AND_BACK,
                            osg::Vec4(0.3, 0.3, 0.3, 1.0));
      material->setShininess(osg::Material::FRONT_AND_BACK, 100.0);
      ss->setAttributeAndModes(material,
                               osg::StateAttribute::OVERRIDE |
                               osg::StateAttribute::ON);
      ss->setTextureAttributeAndModes(0, texture.get(),
                                      osg::StateAttribute::OVERRIDE |
                                      osg::StateAttribute::ON);

      success = true;
      ss->setMode(GL_LIGHTING,
                  osg::StateAttribute::PROTECTED |
                  osg::StateAttribute::OFF);
      setNodeMask(getNodeMask() & ~CastsShadowTraversalMask);
      setNodeMask(getNodeMask() & ~ReceivesShadowTraversalMask);
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

  return success;
}

ShapeNode::~ShapeNode()
{
}

void ShapeNode::displayFrame(bool visibility)
{
  if (frame.valid())
  {
    if (visibility==true)
    {
      frame->show();
    }
    else
    {
      frame->hide();
    }
  }

}

void ShapeNode::toggleFrame()
{
  if (frame.valid())
  {
    frame->toggle();
  }
}

void ShapeNode::updateDynamicShapes()
{
  if (shapeUpdater.valid())
  {
    shapeUpdater->updateDynamicShapes();
  }

}

}   // namespace Rcs
