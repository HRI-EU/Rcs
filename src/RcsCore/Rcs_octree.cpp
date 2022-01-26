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

#include "Rcs_octree.h"
#include "Rcs_macros.h"
#include "Rcs_Vec3d.h"

#include <limits>
#include <float.h>

#if defined (USE_OCTOMAP)

#include "Rcs_typedef.h"
#include "Rcs_shape.h"
#include "Rcs_Mat3d.h"

#include <octomap/OcTree.h>

#define _USE_BOX

static const int MAX_TREE_DEPTH = 16;

/*******************************************************************************
 * Traverses all shapes of the graph. If a shape is of the type
 * RCSSHAPE_OCTREE, the function checks for the mesh pointer. If
 * one or more of them is NULL, the function returns false. If all
 * octree shapes have a non-NULL mesh pointer, the function
 * returns true.
 ******************************************************************************/
bool RcsGraph_checkOctreeConsistency(const RcsGraph* graph)
{
  bool isConsistent = true;

  RCSGRAPH_TRAVERSE_BODIES(graph)
  {
    RCSBODY_TRAVERSE_SHAPES(BODY)
    {
      if ((SHAPE->type==RCSSHAPE_OCTREE) && (SHAPE->mesh == NULL))
      {
        isConsistent = false;
      }
    }

  }

  return isConsistent;
}

/*******************************************************************************
 *
 ******************************************************************************/
double Rcs_getOctreeVolume(void* octomap)
{
  octomap::OcTree* tree = static_cast<octomap::OcTree*>(octomap);

  if (tree == NULL)
  {
    RLOG(1, "Octree is NULL - setting volume to zero");
    return 0.0;
  }

  return tree->volume();
}

/*******************************************************************************
 *
 ******************************************************************************/
void* Rcs_loadOctree(const char* fileName)
{
  if (fileName == NULL)
  {
    RLOG(1, "fileName for Octree is NULL - skip loading");
    return NULL;
  }

  return static_cast<void*>(new octomap::OcTree(fileName));
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs_destroyOctree(void* octomap)
{
  if (octomap == NULL)
  {
    return;
  }

  octomap::OcTree* tree = static_cast<octomap::OcTree*>(octomap);
  tree->clear();
  delete tree;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs_getOctreeBoundingBox(void* octomap,
                              double xyz_min[3],
                              double xyz_max[3])
{
  octomap::OcTree* tree = static_cast<octomap::OcTree*>(octomap);

  if (tree == NULL)
  {
    RLOG(1, "fileName for Octree is NULL - setting bounding box to zero");
    Vec3d_setZero(xyz_min);
    Vec3d_setZero(xyz_max);
    return;
  }

  tree->getMetricMin(xyz_min[0], xyz_min[1], xyz_min[2]);
  tree->getMetricMax(xyz_max[0], xyz_max[1], xyz_max[2]);
}

/*******************************************************************************
 *
 ******************************************************************************/
static void computeChildPTr(const HTr* A_tree, double size, unsigned int i,
                            HTr* A_tree_child)
{
  RLOG(5, "i=%d   size=%g", i, size);

  if (i&1)
  {
    A_tree_child->org[0] = size * 0.5;
  }
  else
  {
    A_tree_child->org[0] = -size * 0.5;
  }

  if (i&2)
  {
    A_tree_child->org[1] = size * 0.5;
  }
  else
  {
    A_tree_child->org[1] = -size * 0.5;
  }

  if (i&4)
  {
    A_tree_child->org[2] = size * 0.5;
  }
  else
  {
    A_tree_child->org[2] = -size * 0.5;
  }

  Vec3d_transformSelf(A_tree_child->org, A_tree);
  Mat3d_copy(A_tree_child->rot, (double (*)[3]) A_tree->rot);
}

/*******************************************************************************
 *
 ******************************************************************************/
static void Rcs_distanceOctomapShapeRecurse(octomap::OcTree* tree1,
                                            const octomap::OcTreeNode* root1,
                                            double box_size,
                                            const RcsShape* shape,
                                            const HTr* A_tree,
                                            const HTr* A_shape,
                                            double I_cp_tree[3],
                                            double I_cp_shape[3],
                                            double I_n_tree_shape[3],
                                            double& min_distance)
{
  double icp_shape[3], icp_octree[3], in_shape_octree[3];

  for (unsigned int chld_idx = 0; chld_idx < 8; ++chld_idx)
  {
    //if (root1->childExists(chld_idx))
    if (tree1->nodeChildExists(root1, chld_idx))
    {
      //const octomap::OcTreeNode* child = root1->getChild(chld_idx);
      const octomap::OcTreeNode* child = tree1->getNodeChild(root1, chld_idx);

      if (tree1->isNodeOccupied(child))
      {
        // calculate child box size and translation
        double child_box_size = box_size * 0.5;
        HTr A_tree_child;
        computeChildPTr(A_tree, child_box_size, chld_idx, &A_tree_child);

        RcsShape octant;
        memset(&octant, 0, sizeof(RcsShape));
        HTr_setIdentity(&octant.A_CB);
        Vec3d_setZero(octant.extents);
        octant.scale3d[0] = 1.0;
        octant.scale3d[1] = 1.0;
        octant.scale3d[2] = 1.0;
        octant.computeType = RCSSHAPE_COMPUTE_DISTANCE;

#if defined(_USE_BOX)
        octant.type = RCSSHAPE_BOX;
        octant.extents[0] = child_box_size;
        octant.extents[1] = child_box_size;
        octant.extents[2] = child_box_size;
#else
        octant.type = RCSSHAPE_SPHERE;

        if (!child->hasChildren())
        {
          octant.extents[0] = child_box_size * 0.5;
        }
        else
        {
          octant.extents[0] = child_box_size * 0.5 * 2.0;
        }
#endif

        const double distance = RcsShape_distance(shape,
                                                  &octant,
                                                  A_shape,
                                                  &A_tree_child,
                                                  icp_shape,
                                                  icp_octree,
                                                  in_shape_octree);

        //if (!child->hasChildren()) // Child is leaf
        if (!tree1->nodeHasChildren(child)) // Child is leaf
        {
          if (distance < min_distance)
          {
            min_distance = distance;
            Vec3d_copy(I_cp_tree, icp_octree);
            Vec3d_copy(I_cp_shape, icp_shape);
            Vec3d_constMul(I_n_tree_shape, in_shape_octree, -1.0);
          }
        }
        else
        {
          if (distance < min_distance)
          {
            Rcs_distanceOctomapShapeRecurse(tree1, child, child_box_size,
                                            shape, &A_tree_child, A_shape,
                                            I_cp_tree, I_cp_shape,
                                            I_n_tree_shape, min_distance);
          }
        }
      }
    }
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
double Rcs_distanceOctomapShape(const RcsShape* tree,
                                const RcsShape* shape,
                                const HTr* A_tree,
                                const HTr* A_shape,
                                double I_cp_tree[3],
                                double I_cp_shape[3],
                                double I_n_tree_shape[3])
{
  if (tree->type != RCSSHAPE_OCTREE)
  {
    RLOG(4, "Shape mismatch - expecting octree but getting %s",
         RcsShape_name(tree->type));
    return DBL_MAX;
  }

  if (tree->mesh == NULL)
  {
    RLOG(4, "Octree shape has no octree attached");
    return DBL_MAX;
  }

  octomap::OcTree* treePtr = reinterpret_cast<octomap::OcTree*>(tree->mesh);

  // first node is root node
  octomap::OcTree::tree_iterator it = treePtr->begin_tree(MAX_TREE_DEPTH);
  double dis = DBL_MAX;

  HTr A_root_box;
  HTr_copy(&A_root_box, A_tree);

  double boxOffset[3];
  boxOffset[0] = it.getCoordinate()(0);
  boxOffset[1] = it.getCoordinate()(1);
  boxOffset[2] = it.getCoordinate()(2);
  Vec3d_rotateSelf(boxOffset, A_root_box.rot);
  Vec3d_addSelf(A_root_box.org, boxOffset);

  // Since we call RcsShape_distance, we need to revert the shape's local
  // transformation so that it is consistent.
  HTr A_BI;

  // Rotation part:
  // A_BI = A_BC*A_CI = (A_CB)^T * A_CI
  //      = trans(shape->A_CB.rot)*A_shape.rot
  Mat3d_transposeMul(A_BI.rot, (double(*)[3]) shape->A_CB.rot,
                     (double(*)[3]) A_shape->rot);
  // Translation part:
  // I_r_B = I_r_C + I_r_CB
  //       = I_r_s - A_IB B_r_BC
  //       = A_shape.org - trans(A_BI)*shape->A_CB.org
  Vec3d_transRotate(A_BI.org, A_BI.rot, shape->A_CB.org);
  Vec3d_constMulSelf(A_BI.org, -1.0);
  Vec3d_addSelf(A_BI.org, A_shape->org);

  Rcs_distanceOctomapShapeRecurse(treePtr, treePtr->getRoot(),
                                  it.getSize(), shape, &A_root_box, &A_BI,
                                  I_cp_tree, I_cp_shape, I_n_tree_shape, dis);

  return dis;
}

/*******************************************************************************
 * Computes the distance between an Octree and any shape.
 ******************************************************************************/
static double RcsShape_closestOctreeToShape(const RcsShape* tree,
                                            const RcsShape* shape,
                                            const HTr* A_tree,
                                            const HTr* A_shape,
                                            double I_cp_tree[3],
                                            double I_cp_shape[3],
                                            double I_n_tree_shape[3])
{
  return Rcs_distanceOctomapShape(tree, shape, A_tree, A_shape,
                                  I_cp_tree, I_cp_shape, I_n_tree_shape);
}

/*******************************************************************************
 * Computes the distance between an Octree and any shape.
 ******************************************************************************/
static double RcsShape_closestShapeToOctree(const RcsShape* shape,
                                            const RcsShape* tree,
                                            const HTr* A_shape,
                                            const HTr* A_tree,
                                            double I_cp_shape[3],
                                            double I_cp_tree[3],
                                            double I_n_shape_tree[3])
{
  double d = Rcs_distanceOctomapShape(tree, shape, A_tree, A_shape,
                                      I_cp_tree, I_cp_shape, I_n_shape_tree);

  Vec3d_constMulSelf(I_n_shape_tree, -1.0);

  return d;
}

/*******************************************************************************
 * Add distance functions to the distance function table
 ******************************************************************************/
static bool setOctreeDistanceFunctions()
{
  bool res = true;

  // These are the shapes we can compute distances against an octree
  RCSSHAPE_TYPE combinations[9] =
  {
    RCSSHAPE_SSL,
    RCSSHAPE_SSR,
    RCSSHAPE_MESH,
    RCSSHAPE_BOX,
    RCSSHAPE_CYLINDER,
    RCSSHAPE_SPHERE,
    RCSSHAPE_CONE,
    RCSSHAPE_TORUS,
    RCSSHAPE_POINT
  };

  for (int i=0; i<9; ++i)
  {
    res = RcsShape_setDistanceFunction(combinations[i], RCSSHAPE_OCTREE,
                                       RcsShape_closestShapeToOctree) && res;
    res = RcsShape_setDistanceFunction(RCSSHAPE_OCTREE, combinations[i],
                                       RcsShape_closestOctreeToShape) && res;
  }

  RLOG(5, "%s Octree distance functions",
       res ? "SUCCESFULLY added" : "FAILED to add");

  return res;
}

// This is being called before main()
static bool distanceInitialized = setOctreeDistanceFunctions();

#else   // USE_OCTOMAP

bool RcsGraph_checkOctreeConsistency(const RcsGraph* graph)
{
  RLOG(4, "No octree support");
  return true;
}

void* Rcs_loadOctree(const char* fileName)
{
  RLOG(4, "No octree support");
  return NULL;
}

void Rcs_destroyOctree(void* octomap)
{
  RLOGS(4, "No octree support - skipping to destroy octree");
}

void Rcs_getOctreeBoundingBox(void* octomap,
                              double xyz_min[3],
                              double xyz_max[3])
{
  Vec3d_setZero(xyz_min);
  Vec3d_setZero(xyz_max);
  RLOGS(4, "No octree support - bounding box is zero");
}

double Rcs_getOctreeVolume(void* octomap)
{
  RLOGS(4, "No octree support - volume is zero");
  return 0.0;
}


#endif   // USE_OCTOMAP
