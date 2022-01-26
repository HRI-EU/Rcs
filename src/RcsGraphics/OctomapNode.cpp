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

#include "OctomapNode.h"
#include "VertexArrayNode.h"

#include <Rcs_macros.h>


#ifdef USE_OCTOMAP

#include "Rcs_graphicsUtils.h"

#include <octomap/OcTree.h>
#include <octomap/octomap_types.h>

#include <osg/Vec3d>
#include <osg/Geode>
#include <osg/PolygonOffset>
#include <osg/ShapeDrawable>
#include <osg/PolygonMode>

#define MAX_POINTS (4 * 6 * 1000000) // 6 sides * 4 points * 1000000 cubes


namespace Rcs
{

/*******************************************************************************
 *
 ******************************************************************************/
osg::Geometry* Rcs::OctomapNode::createOctomapGeometry(octomap::OcTree* tree,
                                                       unsigned int max_tree_depth)
{
  osg::Geometry* geometry = new osg::Geometry();
  osg::Vec3Array* p = new osg::Vec3Array;
  osg::Vec3Array* n = new osg::Vec3Array;
  osg::Vec4Array* c = new osg::Vec4Array();

  double xyz_min[3], xyz_max[3];
  tree->getMetricMin(xyz_min[0], xyz_min[1], xyz_min[2]);
  tree->getMetricMax(xyz_max[0], xyz_max[1], xyz_max[2]);
  double h = 0.5*(xyz_max[2] - xyz_min[2]);


  unsigned int i = 0;
  for (octomap::OcTree::tree_iterator it = tree->begin_tree(max_tree_depth),
       end = tree->end_tree(); it!= end; ++it)
  {
    if (it.isLeaf())
    {
      // voxels for leaf nodes
      octomap::OcTreeVolume voxel = octomap::OcTreeVolume(it.getCoordinate(),
                                                          it.getSize());

      if (tree->isNodeOccupied(*it))
      {
        double w = voxel.second / 2.0;
        osg::Vec3 p1(voxel.first(0)+w, voxel.first(1)+w, voxel.first(2)+w);   //  1  1  1
        osg::Vec3 p2(voxel.first(0)-w, voxel.first(1)+w, voxel.first(2)+w);   // -1  1  1
        osg::Vec3 p3(voxel.first(0)-w, voxel.first(1)-w, voxel.first(2)+w);   // -1 -1  1
        osg::Vec3 p4(voxel.first(0)+w, voxel.first(1)-w, voxel.first(2)+w);   //  1 -1  1
        osg::Vec3 p5(voxel.first(0)+w, voxel.first(1)+w, voxel.first(2)-w);   //  1  1 -1
        osg::Vec3 p6(voxel.first(0)-w, voxel.first(1)+w, voxel.first(2)-w);   // -1  1 -1
        osg::Vec3 p7(voxel.first(0)-w, voxel.first(1)-w, voxel.first(2)-w);   // -1 -1 -1
        osg::Vec3 p8(voxel.first(0)+w, voxel.first(1)-w, voxel.first(2)-w);   //  1 -1 -1

        // top
        p->push_back(p1);   //  1  1  1
        p->push_back(p2);   // -1  1  1
        p->push_back(p3);   // -1 -1  1
        p->push_back(p4);   //  1 -1  1

        n->push_back(osg::Vec3(0.0, 0.0, 1.0));
        n->push_back(osg::Vec3(0.0, 0.0, 1.0));
        n->push_back(osg::Vec3(0.0, 0.0, 1.0));
        n->push_back(osg::Vec3(0.0, 0.0, 1.0));

        c->push_back(osg::Vec4(0.0, 1.0, 1.0-fabs(p1[2]/h), 1.0));
        c->push_back(osg::Vec4(0.0, 1.0, 1.0-fabs(p2[2]/h), 1.0));
        c->push_back(osg::Vec4(0.0, 1.0, 1.0-fabs(p3[2]/h), 1.0));
        c->push_back(osg::Vec4(0.0, 1.0, 1.0-fabs(p4[2]/h), 1.0));

        // left
        p->push_back(p1);   //  1  1  1
        p->push_back(p5);   //  1  1 -1
        p->push_back(p6);   // -1  1 -1
        p->push_back(p2);   // -1  1  1

        n->push_back(osg::Vec3(0.0, 1.0, 0.0));
        n->push_back(osg::Vec3(0.0, 1.0, 0.0));
        n->push_back(osg::Vec3(0.0, 1.0, 0.0));
        n->push_back(osg::Vec3(0.0, 1.0, 0.0));

        c->push_back(osg::Vec4(0.0, 1.0, 1.0-fabs(p1[2]/h), 1.0));
        c->push_back(osg::Vec4(0.0, 1.0, 1.0-fabs(p5[2]/h), 1.0));
        c->push_back(osg::Vec4(0.0, 1.0, 1.0-fabs(p6[2]/h), 1.0));
        c->push_back(osg::Vec4(0.0, 1.0, 1.0-fabs(p2[2]/h), 1.0));

        // back
        p->push_back(p2);   // -1  1  1
        p->push_back(p6);   // -1  1 -1
        p->push_back(p7);   // -1 -1 -1
        p->push_back(p3);   // -1 -1  1

        n->push_back(osg::Vec3(-1.0, 0.0, 0.0));
        n->push_back(osg::Vec3(-1.0, 0.0, 0.0));
        n->push_back(osg::Vec3(-1.0, 0.0, 0.0));
        n->push_back(osg::Vec3(-1.0, 0.0, 0.0));

        c->push_back(osg::Vec4(0.0, 1.0, 1.0-fabs(p2[2]/h), 1.0));
        c->push_back(osg::Vec4(0.0, 1.0, 1.0-fabs(p6[2]/h), 1.0));
        c->push_back(osg::Vec4(0.0, 1.0, 1.0-fabs(p7[2]/h), 1.0));
        c->push_back(osg::Vec4(0.0, 1.0, 1.0-fabs(p3[2]/h), 1.0));

        // right
        p->push_back(p3);   // -1 -1  1
        p->push_back(p7);   // -1 -1 -1
        p->push_back(p8);   //  1 -1 -1
        p->push_back(p4);   //  1 -1  1

        n->push_back(osg::Vec3(0.0, -1.0, 0.0));
        n->push_back(osg::Vec3(0.0, -1.0, 0.0));
        n->push_back(osg::Vec3(0.0, -1.0, 0.0));
        n->push_back(osg::Vec3(0.0, -1.0, 0.0));

        c->push_back(osg::Vec4(0.0, 1.0, 1.0-fabs(p3[2]/h), 1.0));
        c->push_back(osg::Vec4(0.0, 1.0, 1.0-fabs(p7[2]/h), 1.0));
        c->push_back(osg::Vec4(0.0, 1.0, 1.0-fabs(p8[2]/h), 1.0));
        c->push_back(osg::Vec4(0.0, 1.0, 1.0-fabs(p4[2]/h), 1.0));

        // front
        p->push_back(p1);   //  1  1  1
        p->push_back(p4);   //  1 -1  1
        p->push_back(p8);   //  1 -1 -1
        p->push_back(p5);   //  1  1 -1

        n->push_back(osg::Vec3(1.0, 0.0, 0.0));
        n->push_back(osg::Vec3(1.0, 0.0, 0.0));
        n->push_back(osg::Vec3(1.0, 0.0, 0.0));
        n->push_back(osg::Vec3(1.0, 0.0, 0.0));

        c->push_back(osg::Vec4(0.0, 1.0, 1.0-fabs(p1[2]/h), 1.0));
        c->push_back(osg::Vec4(0.0, 1.0, 1.0-fabs(p4[2]/h), 1.0));
        c->push_back(osg::Vec4(0.0, 1.0, 1.0-fabs(p8[2]/h), 1.0));
        c->push_back(osg::Vec4(0.0, 1.0, 1.0-fabs(p5[2]/h), 1.0));

        // bottom
        p->push_back(p5);   //  1  1 -1
        p->push_back(p8);   //  1 -1 -1
        p->push_back(p7);   // -1 -1 -1
        p->push_back(p6);   // -1  1 -1

        n->push_back(osg::Vec3(0.0, 0.0, -1.0));
        n->push_back(osg::Vec3(0.0, 0.0, -1.0));
        n->push_back(osg::Vec3(0.0, 0.0, -1.0));
        n->push_back(osg::Vec3(0.0, 0.0, -1.0));

        c->push_back(osg::Vec4(0.0, 1.0, 1.0-fabs(p5[2]/h), 1.0));
        c->push_back(osg::Vec4(0.0, 1.0, 1.0-fabs(p8[2]/h), 1.0));
        c->push_back(osg::Vec4(0.0, 1.0, 1.0-fabs(p7[2]/h), 1.0));
        c->push_back(osg::Vec4(0.0, 1.0, 1.0-fabs(p6[2]/h), 1.0));

        i += 24;
      }
    }
  }

  geometry->setVertexArray(p);

  geometry->setNormalArray(n);
  geometry->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);

  geometry->setColorArray(c);
  geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);


  osg::DrawArrays* ps = new osg::DrawArrays(osg::PrimitiveSet::QUADS, 0, i);
  geometry->addPrimitiveSet(ps);
  geometry->setUseVertexBufferObjects(true);
  geometry->setUseDisplayList(false);
  geometry->setDataVariance(osg::Object::DYNAMIC);

  return geometry;
}

/*******************************************************************************
 *
 ******************************************************************************/
void OctomapNode::createOctomapPoints(MatNd* points,
                                      const octomap::OcTree* tree,
                                      unsigned int max_tree_depth)
{
  points->m = points->size / points->n;

  unsigned int i = 0;
  for (octomap::OcTree::tree_iterator it = tree->begin_tree(max_tree_depth),
       end = tree->end_tree(); it!= end; ++it)
  {
    if (it.isLeaf())
    {
      if (tree->isNodeOccupied(*it))
      {
        // voxels for leaf nodes
        octomap::OcTreeVolume voxel = octomap::OcTreeVolume(it.getCoordinate(),
                                                            it.getSize());

        double w = voxel.second / 2.0;
        osg::Vec3d p1(voxel.first(0)+w, voxel.first(1)+w, voxel.first(2)+w);
        osg::Vec3d p2(voxel.first(0)-w, voxel.first(1)+w, voxel.first(2)+w);
        osg::Vec3d p3(voxel.first(0)-w, voxel.first(1)-w, voxel.first(2)+w);
        osg::Vec3d p4(voxel.first(0)+w, voxel.first(1)-w, voxel.first(2)+w);
        osg::Vec3d p5(voxel.first(0)+w, voxel.first(1)+w, voxel.first(2)-w);
        osg::Vec3d p6(voxel.first(0)-w, voxel.first(1)+w, voxel.first(2)-w);
        osg::Vec3d p7(voxel.first(0)-w, voxel.first(1)-w, voxel.first(2)-w);
        osg::Vec3d p8(voxel.first(0)+w, voxel.first(1)-w, voxel.first(2)-w);

        // top
        MatNd_setRow(points, i++, p1.ptr(), 3);
        MatNd_setRow(points, i++, p2.ptr(), 3);
        MatNd_setRow(points, i++, p3.ptr(), 3);
        MatNd_setRow(points, i++, p4.ptr(), 3);

        // left
        MatNd_setRow(points, i++, p1.ptr(), 3);
        MatNd_setRow(points, i++, p5.ptr(), 3);
        MatNd_setRow(points, i++, p6.ptr(), 3);
        MatNd_setRow(points, i++, p2.ptr(), 3);

        // back
        MatNd_setRow(points, i++, p2.ptr(), 3);
        MatNd_setRow(points, i++, p6.ptr(), 3);
        MatNd_setRow(points, i++, p7.ptr(), 3);
        MatNd_setRow(points, i++, p3.ptr(), 3);

        // right
        MatNd_setRow(points, i++, p3.ptr(), 3);
        MatNd_setRow(points, i++, p7.ptr(), 3);
        MatNd_setRow(points, i++, p8.ptr(), 3);
        MatNd_setRow(points, i++, p4.ptr(), 3);

        // front
        MatNd_setRow(points, i++, p1.ptr(), 3);
        MatNd_setRow(points, i++, p4.ptr(), 3);
        MatNd_setRow(points, i++, p8.ptr(), 3);
        MatNd_setRow(points, i++, p5.ptr(), 3);

        // bottom
        MatNd_setRow(points, i++, p5.ptr(), 3);
        MatNd_setRow(points, i++, p8.ptr(), 3);
        MatNd_setRow(points, i++, p7.ptr(), 3);
        MatNd_setRow(points, i++, p6.ptr(), 3);
      }
    }
  }

  points->m = i;
}

/*******************************************************************************
 *
 ******************************************************************************/
OctomapNode::OctomapNode(const char* fileName, const char* colorStr) :
  NodeBase()
{
  octomap::OcTree* tree = new octomap::OcTree(fileName);

  init(tree, colorStr);

  delete tree;
}

/*******************************************************************************
 *
 ******************************************************************************/
OctomapNode::OctomapNode(const octomap::OcTree* tree, const char* colorStr) :
  NodeBase()
{
  init(tree, colorStr);
}

/*******************************************************************************
 *
 ******************************************************************************/
void OctomapNode::init(const octomap::OcTree* tree, const char* color)
{
  _points = MatNd_create(MAX_POINTS, 3);
  _points->m = 0;
  _quads_node = new VertexArrayNode(_points, osg::PrimitiveSet::QUADS, color);
  _quads_node->setManualUpdate(true);
  _quads_node->setLighting(false);

  addChild(_quads_node);

  setOctree(tree);
}

/*******************************************************************************
 *
 ******************************************************************************/
OctomapNode::~OctomapNode()
{
  MatNd_destroy(_points);
}

/*******************************************************************************
 *
 ******************************************************************************/
void OctomapNode::setOctree(const octomap::OcTree* tree)
{
  if (tree)
  {
    createOctomapPoints(_points, tree, tree->getTreeDepth());
    _quads_node->performUpdate();
  }
}


#else   // #ifdef USE_OCTOMAP

namespace Rcs
{

OctomapNode::OctomapNode(const char*, const char*) : NodeBase()
{
  RLOG(1, "No Octomap support in OctomapNode");
}

OctomapNode::OctomapNode(const octomap::OcTree*, const char*) : NodeBase()
{
  RLOG(1, "No Octomap support in OctomapNode");
}

OctomapNode::~OctomapNode()
{
  RLOG(1, "No Octomap support in OctomapNode");
}

void OctomapNode::init(const octomap::OcTree*, const char*)
{
  RLOG(1, "No Octomap support in OctomapNode");
}

void OctomapNode::setOctree(const octomap::OcTree*)
{
  RLOG(1, "No Octomap support in OctomapNode");
}

void OctomapNode::createOctomapPoints(MatNd*, const octomap::OcTree*, unsigned int)
{
  RLOG(1, "No Octomap support in OctomapNode");
}

osg::Geometry* OctomapNode::createOctomapGeometry(octomap::OcTree*, unsigned int)
{
  RLOG(1, "No Octomap support in OctomapNode");
  return NULL;
}

#endif  // #ifdef USE_OCTOMAP

}
