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
#include <Rcs_timer.h>
#include <Rcs_utils.h>
#include <Rcs_mesh.h>
#include <COSNode.h>
#include <MeshNode.h>
#include <TorusNode.h>

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

char BodyNode::_fontFile[256];
bool BodyNode::_fontFileFound = false;
bool BodyNode::_fontFileSearched = false;

std::map<std::string, osg::ref_ptr<osg::Node> > BodyNode::_meshBuffer;
std::map<std::string, osg::ref_ptr<osg::Texture2D> > BodyNode::_textureBuffer;
OpenThreads::Mutex _meshBufferMtx;
OpenThreads::Mutex _textureBufferMtx;



/*******************************************************************************
 * Helper class. It's appended to the userData of the BodyNode's
 *        _dynamicShapes member. It's used for a quick update of
 *        dynamically resized RcsShapes. Meshes are not resized.
 *
 *       TODO: resize if scale is changed
 ******************************************************************************/
class DynamicShapeData: public osg::Referenced
{
public:
  DynamicShapeData(const RcsShape* shape)
  {
    _shape = shape;
    Vec3d_copy(this->extents, shape->extents);
  }
  void addGeometry(osg::Shape* s)
  {
    geometry.push_back(s);
  }
  void addDrawable(osg::ShapeDrawable* d)
  {
    drawable.push_back(d);
  }
  const RcsShape* shape()
  {
    return _shape;
  }
  double extents[3];    // Current shapes extents
  std::vector<osg::Shape*> geometry;
  std::vector<osg::ShapeDrawable*> drawable;

private:
  const RcsShape* _shape;
};

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
 * See header.
 ******************************************************************************/
BodyNode::BodyNode(const RcsBody* b, float scale, bool resizeable) :
  bdy(b),
  ghostMode(false),
  dynamicMeshUpdate(false)
{
  RCHECK(this->bdy);
  setName(this->bdy->name ? this->bdy->name : "Unnamed body");

  if (b->shape == NULL)
  {
    RLOG(4, "Body \"%s\": no shapes - creating empty osg::Node",
         getName().c_str());
    return;
  }

  RCHECK(scale > 0.0);
  setScale(osg::Vec3(scale, scale, scale));

  this->A_BI          = b->A_BI;
  _refNode       = new osg::Switch;
  _debugNode     = addDebugInformation();
  _debugNode->setAllChildrenOff();
  addChild(_debugNode.get());

  _collisionNode = addShapes(RCSSHAPE_COMPUTE_DISTANCE, resizeable);
  _graphicsNode  = addShapes(RCSSHAPE_COMPUTE_GRAPHICS, resizeable);
  _physicsNode   = addShapes(RCSSHAPE_COMPUTE_PHYSICS+
                             RCSSHAPE_COMPUTE_SOFTPHYSICS, resizeable);

  _refNode->setAllChildrenOff();
  _collisionNode->setAllChildrenOff();
  _graphicsNode->setAllChildrenOn();
  _physicsNode->setAllChildrenOff();

  _nodeSwitch = new osg::Switch();
  addChild(_nodeSwitch.get());

  _nodeSwitch->addChild(_collisionNode.get());
  _nodeSwitch->addChild(_graphicsNode.get());
  _nodeSwitch->addChild(_physicsNode.get());
  _nodeSwitch->addChild(_refNode.get());

  _collisionNode->setName("BodyNode::CollisionNode");
  _graphicsNode->setName("BodyNode::GraphicsNode");
  _physicsNode->setName("BodyNode::PhysicsNode");
  _refNode->setName("BodyNode::ReferenceFramesNode");


  // Assign the initial transformation to the node
  if (HTr_isValid(this->A_BI))
  {
    setTransformation(this->A_BI);
  }
  else
  {
    RLOG(4, "Invalid transform in \"%s\"", getName().c_str());
  }

  // Assign transformation update callback upon scene traversal
  setUpdateCallback(new BodyUpdateCallback(this));
}

/*******************************************************************************
 * Recursively adds the bodies collision shapes to the node.
 *
 * shapeNode (osg::Switch)
 *     |
 *     ---> shapeTransform (PAT) with A_CB
 *               |
 *               ---> geode (osg::Geode)
 *                      |
 *                      ---> shape (osg::Drawable)
 *                             |
 *                             ---> capsule, box, etc (osg::Capsule ...)
*******************************************************************************/
osg::Switch* BodyNode::addShapes(int mask, bool resizeable)
{
  int shapeCount = 0;
  RcsShape** s = &this->bdy->shape[0];
  osg::ref_ptr<osg::TessellationHints> hints = new osg::TessellationHints;
  hints->setDetailRatio(2.0);
  osg::Switch* shapeNode = new osg::Switch;

  // Loop through all shapes of the body
  while (*s)
  {
    RcsShape* shape = *s;

    if ((shape->computeType & mask) == 0)
    {
      NLOG(0, "Skipping shape %d of %s", shapeCount, getName().c_str());
      s++;
      continue;
    }

    NLOG(0, "Adding shape %d of \"%s\"", shapeCount, getName().c_str());
    osg::PositionAttitudeTransform* shapeTransform =
      new osg::PositionAttitudeTransform;
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();

    if (resizeable)
    {
      _dynamicShapes.push_back(geode);
    }
    DynamicShapeData* dnmData = new DynamicShapeData(shape);
    geode->setUserData(dnmData);

#if 1
    // This would enable depth sorting by default for correctly drawing
    // transparent objects. However, there is a performance penalty.
    // Thus, it's better to enable dept sorting selectively

    // set render bin to depthsorted in order to handle transparency correctly
    osg::StateSet* state_set = geode->getOrCreateStateSet();
    state_set->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    geode->setStateSet(state_set);
#endif

    // Relative orientation of shape wrt. body
    const HTr* A_CB = &shape->A_CB;
    osg::Quat qA;

    qA.set(osg::Matrix(A_CB->rot[0][0], A_CB->rot[0][1], A_CB->rot[0][2], 0.0,
                       A_CB->rot[1][0], A_CB->rot[1][1], A_CB->rot[1][2], 0.0,
                       A_CB->rot[2][0], A_CB->rot[2][1], A_CB->rot[2][2], 0.0,
                       0.0, 0.0, 0.0, 1.0));
    shapeTransform->setPosition(osg::Vec3(A_CB->org[0], A_CB->org[1],
                                          A_CB->org[2]));
    shapeTransform->setAttitude(qA);

    shapeNode->addChild(shapeTransform);
    shapeTransform->addChild(geode.get());

    /////////////////////////////////
    // Add a capsule to the shapeNode
    /////////////////////////////////

    if (shape->type == RCSSHAPE_SSL)
    {
      double r = shape->extents[0], length = shape->extents[2];
      osg::Capsule* capsule =
        new osg::Capsule(osg::Vec3(0.0, 0.0, length / 2.0), r, length);
      osg::ShapeDrawable* sd = new osg::ShapeDrawable(capsule, hints.get());
      if (resizeable)
      {
        sd->setUseDisplayList(false);
      }
      geode->addDrawable(sd);

      if (shape->color != NULL)
      {
        setNodeMaterial(shape->color, geode.get());
      }

      // Add the information for dynamic resizing
      dnmData->addGeometry(capsule);
      dnmData->addDrawable(sd);
    }



    /////////////////////////////////
    // Add a box to the shapeNode
    /////////////////////////////////

    else if (shape->type == RCSSHAPE_BOX)
    {
      NLOG(5, "Adding box %d of \"%s\" with dimension %f %f %f",
           shapeCount, getName().c_str(),
           shape->extents[0], shape->extents[1], shape->extents[2]);
      osg::Box* box =
        new osg::Box(osg::Vec3(0.0, 0.0, 0.0),
                     shape->extents[0], shape->extents[1], shape->extents[2]);
      osg::ShapeDrawable* sd = new osg::ShapeDrawable(box, hints.get());
      if (resizeable)
      {
        sd->setUseDisplayList(false);
      }
      geode->addDrawable(sd);

      if (shape->color != NULL)
      {
        setNodeMaterial(shape->color, geode.get());
      }

      // Add the information for dynamic resizing
      dnmData->addGeometry(box);
      dnmData->addDrawable(sd);
    }



    /////////////////////////////////
    // Add a sphere to the shapeNode
    /////////////////////////////////

    else if (shape->type == RCSSHAPE_SPHERE)
    {
      NLOG(5, "Adding %s %d of \"%s\" with radius %f",
           resizeable ? "resizeable sphere" : " sphere",
           shapeCount, getName().c_str(), shape->extents[0]);
      osg::Sphere* sphere =
        new osg::Sphere(osg::Vec3(0.0, 0.0, 0.0), shape->extents[0]);
      osg::ShapeDrawable* sd = new osg::ShapeDrawable(sphere, hints.get());
      if (resizeable)
      {
        sd->setUseDisplayList(false);
      }
      geode->addDrawable(sd);

      if (shape->color != NULL)
      {
        setNodeMaterial(shape->color, geode.get());
      }

      // Add the information for dynamic resizing
      dnmData->addGeometry(sphere);
      dnmData->addDrawable(sd);
    }



    ///////////////////////////////////////////
    // Add a sphere swept rectangle to the node
    ///////////////////////////////////////////

    else if (shape->type == RCSSHAPE_SSR)
    {
      double r  = shape->extents[2] / 2.0;
      double lx = shape->extents[0];
      double ly = shape->extents[1];
      NLOG(5, "Adding SSR %d of \"%s\" with dimension %f %f %f",
           shapeCount, getName().c_str(), lx, ly, 2.0 * r);

      // Side 1: Front y-direction
      osg::Capsule* cSSR1 =
        new osg::Capsule(osg::Vec3(-lx / 2.0, 0.0, 0.0), r, ly);
      cSSR1->setRotation(osg::Quat(osg::inDegrees(90.0f),
                                   osg::Vec3(1.0f, 0.0f, 0.0f)));
      osg::ShapeDrawable* sdrC1 = new osg::ShapeDrawable(cSSR1, hints.get());
      if (resizeable)
      {
        sdrC1->setUseDisplayList(false);
      }
      geode->addDrawable(sdrC1);

      // Side 2: Back y-direction
      osg::Capsule* cSSR2 =
        new osg::Capsule(osg::Vec3(lx / 2.0, 0.0, 0.0), r, ly);
      cSSR2->setRotation(osg::Quat(osg::inDegrees(90.0f),
                                   osg::Vec3(1.0f, 0.0f, 0.0f)));
      osg::ShapeDrawable* sdrC2 = new osg::ShapeDrawable(cSSR2, hints.get());
      if (resizeable)
      {
        sdrC2->setUseDisplayList(false);
      }
      geode->addDrawable(sdrC2);

      // Side 3: Right x-direction
      osg::Capsule* cSSR3 =
        new osg::Capsule(osg::Vec3(0.0, ly / 2.0, 0.0), r, lx);
      cSSR3->setRotation(osg::Quat(osg::inDegrees(90.0f),
                                   osg::Vec3(0.0f, 1.0f, 0.0f)));
      osg::ShapeDrawable* sdrC3 = new osg::ShapeDrawable(cSSR3, hints.get());
      if (resizeable)
      {
        sdrC3->setUseDisplayList(false);
      }
      geode->addDrawable(sdrC3);

      // Side 4: Left x-direction
      osg::Capsule* cSSR4 =
        new osg::Capsule(osg::Vec3(0.0, -ly / 2.0, 0.0), r, lx);
      cSSR4->setRotation(osg::Quat(osg::inDegrees(90.0f),
                                   osg::Vec3(0.0f, 1.0f, 0.0f)));
      osg::ShapeDrawable* sdrC4 = new osg::ShapeDrawable(cSSR4, hints.get());
      if (resizeable)
      {
        sdrC4->setUseDisplayList(false);
      }
      geode->addDrawable(sdrC4);

      // Box part
      osg::Box* bSSR =
        new osg::Box(osg::Vec3(0.0, 0.0, 0.0), lx, ly, 2.0 * r);
      osg::ShapeDrawable* sdrBox = new osg::ShapeDrawable(bSSR, hints.get());
      if (resizeable)
      {
        sdrBox->setUseDisplayList(false);
      }
      geode->addDrawable(sdrBox);

      if (shape->color != NULL)
      {
        setNodeMaterial(shape->color, geode.get());
      }

      // Add the information for dynamic resizing
      dnmData->addGeometry(cSSR1);
      dnmData->addGeometry(cSSR2);
      dnmData->addGeometry(cSSR3);
      dnmData->addGeometry(cSSR4);
      dnmData->addGeometry(bSSR);
      dnmData->addDrawable(sdrC1);
      dnmData->addDrawable(sdrC2);
      dnmData->addDrawable(sdrC3);
      dnmData->addDrawable(sdrC4);
      dnmData->addDrawable(sdrBox);
    }



    /////////////////////////////
    // Add a cylinder to the node
    /////////////////////////////

    else if (shape->type == RCSSHAPE_CYLINDER)
    {
      osg::Cylinder* cylinder =
        new osg::Cylinder(osg::Vec3(0.f, 0.0f, 0.0f), shape->extents[0],
                          shape->extents[2]);
      osg::ShapeDrawable* sd = new osg::ShapeDrawable(cylinder, hints.get());
      if (resizeable)
      {
        sd->setUseDisplayList(false);
      }
      geode->addDrawable(sd);

      if (shape->color != NULL)
      {
        setNodeMaterial(shape->color, geode.get());
      }

      // Add the information for dynamic resizing
      dnmData->addGeometry(cylinder);
      dnmData->addDrawable(sd);
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
      osg::ShapeDrawable* sd = new osg::ShapeDrawable(cone, hints.get());
      if (resizeable)
      {
        sd->setUseDisplayList(false);
      }
      geode->addDrawable(sd);

      if (shape->color != NULL)
      {
        setNodeMaterial(shape->color, geode.get());
      }

      // Add the information for dynamic resizing
      dnmData->addGeometry(cone);
      dnmData->addDrawable(sd);
    }



    /////////////////////////////
    // Add mesh file
    /////////////////////////////

    else if (shape->type == RCSSHAPE_MESH)
    {
      if (shape->meshFile != NULL)
      {
        osg::ref_ptr<osg::Node> meshNode;

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
        bool meshFromOsgReader = false;
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


        if (meshNode.valid())
        {
          shapeTransform->addChild(meshNode.get());

          // The mesh is only scaled if it has been read from the
          // osgDB::readNodeFile class. Otherwise, the scaling has already
          // been done during parsing the graph (meshes need to be consistent
          // with the physics simulator).
          if (meshFromOsgReader == true)
          {
            shapeTransform->setScale(osg::Vec3(shape->scale,
                                               shape->scale,
                                               shape->scale));
          }
          if (shape->color != NULL)
          {
            setNodeMaterial(shape->color, shapeTransform);
          }
        }
        else
        {
          RLOG(1, "Failed to generate mesh for \"%s\"", shape->meshFile);
        }



      }
      else   // if(shape->meshFile)
      {
        RLOG(1, "Mesh file of body \"%s\" is NULL", getName().c_str());
      }

    }



    ////////////////////////////////////////////////
    // Add a reference coordinate system to the node
    ////////////////////////////////////////////////

    else if (shape->type == RCSSHAPE_REFFRAME)
    {
      RCHECK(_refNode);
      COSNode* cFrame = new COSNode(shape->scale,
                                    shape->extents[0],
                                    shape->extents[1],
                                    shape->extents[2]);
      cFrame->setPosition(osg::Vec3(A_CB->org[0], A_CB->org[1], A_CB->org[2]));
      cFrame->setRotation(qA);
      _refNode->addChild(cFrame);
    }


    /////////////////////////////
    // Add a torus to the node
    /////////////////////////////

    else if (shape->type == RCSSHAPE_TORUS)
    {
      TorusNode* torus = new TorusNode(shape->extents[0], shape->extents[2]);

      shapeTransform->addChild(torus);

      if (shape->color != NULL)
      {
        setNodeMaterial(shape->color, shapeTransform);
      }
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
    // Add a point to the shapeNode
    /////////////////////////////////

    else if (shape->type == RCSSHAPE_POINT)
    {
      NLOG(5, "Adding point %d of \"%s\" with radius %f",
           shapeCount, getName().c_str(), shape->r);
      osg::Sphere* sphere =
        new osg::Sphere(osg::Vec3(0.0, 0.0, 0.0), 0.005);
      osg::ShapeDrawable* sd = new osg::ShapeDrawable(sphere, hints.get());
      if (resizeable)
      {
        sd->setUseDisplayList(false);
      }
      geode->addDrawable(sd);

      if (shape->color != NULL)
      {
        setNodeMaterial(shape->color, geode.get());
      }

      // Add the information for dynamic resizing
      dnmData->addGeometry(sphere);
      dnmData->addDrawable(sd);
    }

    else if (shape->type == RCSSHAPE_MARKER)
    {
      NLOG(5, "Adding marker %d id %d of \"%s\" with length %f ", shapeCount,
           *((int*)shape->userData), getName().c_str(), shape->extents[2]);
      osg::Box* box =
        new osg::Box(osg::Vec3(0.0, 0.0, 0.0),
                     shape->extents[2], shape->extents[2], 0.001);
      osg::ShapeDrawable* sd = new osg::ShapeDrawable(box, hints.get());
      if (resizeable)
      {
        sd->setUseDisplayList(false);
      }
      geode->addDrawable(sd);

      if (shape->color != NULL)
      {
        setNodeMaterial(shape->color, geode.get());
      }

      // Add the information for dynamic resizing
      dnmData->addGeometry(box);
      dnmData->addDrawable(sd);
    }

    ////////////////////////
    // Ignore unknown shapes
    ////////////////////////
    else
    {
      RLOG(1, "Shape type %d of body \"%s\" undefined", shape->type,
           getName().c_str());
    }



    // read the texture file
    if (shape->textureFile)
    {
      osg::ref_ptr<osg::Texture2D> texture;

      // Little map that stores name-pointer pairs of texture files. If
      // a texture has already been loaded, we look up its pointer from
      // the map. Otherwise, we load the texture and store its pointer
      // in the map.
      std::map<std::string, osg::ref_ptr<osg::Texture2D> >::iterator it;

      _textureBufferMtx.lock();
      for (it = _textureBuffer.begin() ; it != _textureBuffer.end(); ++it)
      {
        if (it->first == shape->textureFile)
        {
          NLOG(0, "Texture file \"%s\" already loaded",
               shape->textureFile);
          texture = it->second;
          break;

        }

      }
      _textureBufferMtx.unlock();

      // texture not found, let's create new one
      if (!texture.valid())
      {

        // load an image by reading a file:
        osg::ref_ptr<osg::Image> texture_image =
          osgDB::readImageFile(shape->textureFile);

        if (texture_image.valid() == false)
        {
          RLOG(1, "couldn't load texture file  \"%s\" for body \"%s\", "
               "omitting...", shape->textureFile, getName().c_str());
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
          _textureBuffer[std::string(shape->textureFile)] = texture;
          _textureBufferMtx.unlock();
        }
      }

      if (texture.valid())
      {

        // this would extend previous setting, e.g. by color settings
        for (size_t i = 0; i < shapeTransform->getNumChildren(); i++)
        {
          osg::StateSet* state_set =
            shapeTransform->getChild(i)->getOrCreateStateSet();
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


    s++;
    shapeCount++;



  }   // while(*s)

  return shapeNode;
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
  bool visible = _collisionNode->getValue(0);
  displayCollisionNode(!visible);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void BodyNode::toggleGraphicsNode()
{
  bool visible = _graphicsNode->getValue(0);
  displayGraphicsNode(!visible);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void BodyNode::togglePhysicsNode()
{
  bool visible = _physicsNode->getValue(0);
  displayPhysicsNode(!visible);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void BodyNode::toggleReferenceNode()
{
  bool visible = _refNode->getValue(0);
  displayReferenceNode(!visible);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void BodyNode::toggleDebugInformation()
{
  bool visible = _debugNode->getValue(0);
  displayDebugInformation(!visible);
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
  if (visible)
  {
    _refNode->setAllChildrenOn();
  }
  else
  {
    _refNode->setAllChildrenOff();
  }
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void BodyNode::displayDebugInformation(bool visible)
{
  if (visible)
  {
    _debugNode->setAllChildrenOn();
  }
  else
  {
    _debugNode->setAllChildrenOff();
  }
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool BodyNode::collisionNodeVisible() const
{
  osg::Switch::ValueList childVisible = _collisionNode->getValueList();
  int vis = 0;

  for (size_t i=0; i<childVisible.size(); ++i)
  {
    vis += (childVisible[i] == true) ? 1 : -1;
  }

  return (vis>0) ? true : false;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool BodyNode::graphicsNodeVisible() const
{
  osg::Switch::ValueList childVisible = _graphicsNode->getValueList();
  int vis = 0;

  for (size_t i=0; i<childVisible.size(); ++i)
  {
    vis += (childVisible[i] == true) ? 1 : -1;
  }

  return (vis>0) ? true : false;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool BodyNode::physicsNodeVisible() const
{
  osg::Switch::ValueList childVisible = _physicsNode->getValueList();
  int vis = 0;

  for (size_t i=0; i<childVisible.size(); ++i)
  {
    vis += (childVisible[i] == true) ? 1 : -1;
  }

  return (vis>0) ? true : false;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool BodyNode::referenceFramesVisible() const
{
  osg::Switch::ValueList childVisible = _refNode->getValueList();
  int vis = 0;

  for (size_t i=0; i<childVisible.size(); ++i)
  {
    vis += (childVisible[i] == true) ? 1 : -1;
  }

  return (vis>0) ? true : false;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool BodyNode::debugInformationVisible() const
{
  osg::Switch::ValueList childVisible = _debugNode->getValueList();
  int vis = 0;

  for (size_t i=0; i<childVisible.size(); ++i)
  {
    vis += (childVisible[i] == true) ? 1 : -1;
  }

  return (vis>0) ? true : false;
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
 * Vector _dynamicShapes holds all osg::Geodes that might be resized
 * dynamically. They have a class DynamicShapeData attached to their userData
 * which holds pointers to the required drawables etc. Meshes are not
 * resizeable, since this probably costs significant graphics performance
 * (Display lists are not handled on the graphics card).
 ******************************************************************************/
void BodyNode::updateDynamicShapes()
{
  std::vector<osg::ref_ptr<osg::Geode> >::iterator it;

  for (it = _dynamicShapes.begin(); it != _dynamicShapes.end(); ++it)
  {
    DynamicShapeData* uDat =
      static_cast<DynamicShapeData*>((*it)->getUserData());

    // If the geode has no userData, we step to the next geode
    if (!uDat)
    {
      continue;
    }

    // If the extents didn't change, step to the next geode.
    if (Vec3d_isEqual(uDat->shape()->extents, uDat->extents, 1.0e-6))
    {
      continue;
    }
    // If the extents changed, we memorize them in the userData
    else
    {
      Vec3d_copy(uDat->extents, uDat->shape()->extents);
    }

    switch (uDat->shape()->type)
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
        double r  = 0.5 * (uDat->shape())->extents[2];
        double lx = uDat->shape()->extents[0];
        double ly = uDat->shape()->extents[1];

        for (sit = uDat->geometry.begin();
             sit != uDat->geometry.end(); ++sit)
        {

          switch (index)
          {
            case 0:
            {
              osg::Capsule* c = static_cast<osg::Capsule*>(*sit);
              RCHECK(c);
              c->setCenter(osg::Vec3(-lx / 2.0, 0.0, 0.0));
              c->setHeight(ly);
              c->setRadius(r);
            }
            break;

            case 1:
            {
              osg::Capsule* c = static_cast<osg::Capsule*>(*sit);
              RCHECK(c);
              c->setCenter(osg::Vec3(lx / 2.0, 0.0, 0.0));
              c->setHeight(ly);
              c->setRadius(r);
            }
            break;

            case 2:
            {
              osg::Capsule* c = static_cast<osg::Capsule*>(*sit);
              RCHECK(c);
              c->setCenter(osg::Vec3(0.0, ly / 2.0, 0.0));
              c->setHeight(lx);
              c->setRadius(r);
            }
            break;

            case 3:
            {
              osg::Capsule* c = static_cast<osg::Capsule*>(*sit);
              RCHECK(c);
              c->setCenter(osg::Vec3(0.0, -ly / 2.0, 0.0));
              c->setHeight(lx);
              c->setRadius(r);
            }
            break;

            case 4:
            {
              osg::Box* b = static_cast<osg::Box*>(*sit);
              RCHECK(b);
              b->setHalfLengths(osg::Vec3(0.5 * lx, 0.5 * ly, r));
            }
            break;

            default:
              RLOG(1, "Capsule index out of range: %d", index);
          }   // switch(index)

          index++;

        }   // for(sit=...

        // "Dirty" the shapes bounding box to adjust bounding box
        std::vector<osg::ShapeDrawable*>::iterator dit;

        for (dit = uDat->drawable.begin();
             dit != uDat->drawable.end(); ++dit)
        {
          (*dit)->dirtyBound();
        }

      }   // case RCSSHAPE_SSR
      break;

      // Here we adjust the size of a SSL.
      case RCSSHAPE_SSL:
      {
        osg::Capsule* c = static_cast<osg::Capsule*>(uDat->geometry.front());
        RCHECK(c);
        c->setCenter(osg::Vec3(0.0, 0.0, uDat->shape()->extents[2]/2.0));
        c->setHeight(uDat->shape()->extents[2]);
        c->setRadius((uDat->shape())->extents[0]);

        // "Dirty" the shapes bounding box to adjust bounding box
        uDat->drawable.front()->dirtyBound();
      }
      break;

      // Here we adjust the size of a cylinder.
      case RCSSHAPE_CYLINDER:
      {
        osg::Cylinder* c =
          static_cast<osg::Cylinder*>(uDat->geometry.front());
        RCHECK(c);
        c->setHeight(uDat->shape()->extents[2]);
        c->setRadius(uDat->shape()->extents[0]);

        // "Dirty" the shapes bounding box to adjust bounding box
        uDat->drawable.front()->dirtyBound();
      }
      break;

      // Here we adjust the size of a cone.
      case RCSSHAPE_CONE:
      {
        osg::Cone* c = static_cast<osg::Cone*>(uDat->geometry.front());
        RCHECK(c);
        c->setHeight(uDat->shape()->extents[2]);
        c->setRadius(uDat->shape()->extents[0]);

        // "Dirty" the shapes bounding box to adjust bounding box
        uDat->drawable.front()->dirtyBound();
      }
      break;

      // Here we adjust the size of a sphere.
      case RCSSHAPE_SPHERE:
      {
        osg::Sphere* s = static_cast<osg::Sphere*>(uDat->geometry.front());
        RCHECK(s);
        s->setRadius(uDat->shape()->extents[0]);

        // "Dirty" the shapes bounding box to adjust bounding box
        uDat->drawable.front()->dirtyBound();
      }
      break;

      // Here we adjust the size of a BOX.
      case RCSSHAPE_BOX:
      {
        osg::Box* b =
          static_cast<osg::Box*>(uDat->geometry.front());
        RCHECK(b);
        b->setHalfLengths(osg::Vec3(0.5 * uDat->shape()->extents[0],
                                    0.5 * uDat->shape()->extents[1],
                                    0.5 * uDat->shape()->extents[2]));

        // "Dirty" the shapes bounding box to adjust bounding box
        uDat->drawable.front()->dirtyBound();
      }
      break;

      default:
        break;
    }

  }   //  for(it=_dynamicShapes.begin() ...

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

  // Update debug lines
  if ((body()->parent) && (_debugLine.valid()))
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

  // Change the geometry according to the bodies shapes
  updateDynamicShapes();

  // Mesh dynamic update
  if (getDynamicMeshUpdate())
  {
    updateDynamicMeshes();
  }
}


}   // namespace Rcs
