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

#include "Rcs_typedef.h"
#include "Rcs_body.h"
#include "Rcs_joint.h"
#include "Rcs_shape.h"
#include "Rcs_sensor.h"
#include "Rcs_kinematics.h"
#include "Rcs_utils.h"
#include "Rcs_macros.h"
#include "Rcs_math.h"
#include "Rcs_intersectionWM5.h"

#include <float.h>



// If this is defined, both cost and gradient will be superposed of
// a term accounting for the closest point distance, and another
// term accounting for the center distance. The center distance makes
// the term more robust, since sometimes the penetration of bodies is
// not well-defined.
//#define RCS_USE_MIXTURE_GRADIENT

// This is the "steepness" of the closest point cost at the
// distance=0 boundary. The cost is parabolic within d in [0 : db]
// (db being the distance threshold), zero outside and tangentially
// extrapolated when negative.
#define RCS_PENETRATION_SLOPE         (10.0)

// This is the maximum value that the distance center cost can be
// (namely for d=0). The cost is exponentially increasing to this
// maximum value.
#define RCS_MAX_CENTERCOST   (1000.0)

// This is for the gradient tests only. When commenting out this variable,
// the functions related to the closest points (cost, gradient and Hessian)
// are modified so that the closest points are not determined with the
// distance functions, but are assigned to the body origin with some random
// offset. This makes the closest points not change due to the shape geometry
// which corresponds to the underlying assumption that d = p2 - p1.
// #define TEST_FIXED_CLOSESTPOINTS

#ifdef TEST_FIXED_CLOSESTPOINTS
#include "Rcs_gradientTests.h"
#endif



/*******************************************************************************
 * See header.
 ******************************************************************************/
RcsBody* RcsBody_create()
{
  RcsBody* b = RALLOC(RcsBody);
  RcsBody_init(b);
  return b;
}

/*******************************************************************************
 * All names, the Inertia tensor and the joint and shape arrays are initialized
 * to 0 through the memset call.
 ******************************************************************************/
void RcsBody_init(RcsBody* b)
{
  memset(b, 0, sizeof(RcsBody));
  HTr_setIdentity(&b->A_BP);
  HTr_setIdentity(&b->A_BI);
  b->id = -1;
  b->parentId = -1;
  b->firstChildId = -1;
  b->lastChildId = -1;
  b->nextId = -1;
  b->prevId = -1;
  b->jntId = -1;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsBody_destroy(RcsBody* self)
{
  RcsBody_clear(self);

  // Reset all internal memory
  memset(self, 0, sizeof(RcsBody));

  RFREE(self);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsBody_clear(RcsBody* self)
{
  if (self == NULL)
  {
    NLOG(1, "Body is NULL - returning");
    return;
  }

  // Destroy all associated joints
  /* RcsJoint* jnt, *next; */

  /* jnt  = self->jnt; */
  /* next = NULL; */

  /* while (jnt) */
  /* { */
  /*   NLOG(0, "Deleting joint \"%s\"", jnt->name); */
  /*   next = jnt->next; */
  /*   RcsJoint_destroy(jnt); */
  /*   jnt = next; */
  /* } */

  // Destroy all associated shapes
  NLOG(0, "Starting to delete shapes");
  if (self->shape != NULL)
  {
    RcsShape** sPtr = self->shape;
    while (*sPtr)
    {
      RcsShape_destroy(*sPtr);
      sPtr++;
    }

    RFREE(self->shape);
  }

}

/*******************************************************************************
 * See header.
 ******************************************************************************/
unsigned int RcsBody_numJoints(const RcsGraph* graph, const RcsBody* self)
{
  unsigned int nJoints = 0;

  if ((self==NULL) || (self->jntId==-1))
  {
    return 0;
  }

  RCSBODY_FOREACH_JOINT(graph, self)
  {
    nJoints++;
  }

  return nJoints;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
unsigned int RcsBody_numShapes(const RcsBody* self)
{
  unsigned int nShapes = 0;

  if (self == NULL)
  {
    return 0;
  }

  if (self->shape == NULL)
  {
    return 0;
  }

  for (RcsShape** sPtr = self->shape; *sPtr; sPtr++)
  {
    nShapes++;
  }

  return nShapes;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
unsigned int RcsBody_numDistanceShapes(const RcsBody* self)
{
  unsigned int nShapes = 0;

  if (self == NULL)
  {
    return 0;
  }

  RCSBODY_TRAVERSE_SHAPES(self)
  {
    if ((SHAPE->computeType & RCSSHAPE_COMPUTE_DISTANCE) != 0)
    {
      nShapes++;
    }
  }

  return nShapes;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsBody_addShape(RcsBody* self, RcsShape* shape)
{
  unsigned int nShapes = RcsBody_numShapes(self);
  self->shape = (RcsShape**) realloc(self->shape, (nShapes+2)*sizeof(RcsShape*));
  RCHECK(self->shape);
  self->shape[nShapes] = shape;
  self->shape[nShapes+1] = NULL;
}

/*******************************************************************************
 * Destroy all associated shapes.
 ******************************************************************************/
unsigned int RcsBody_removeShapes(RcsBody* self)
{
  unsigned int count = 0;

  if (self->shape == NULL)
  {
    return 0;
  }

  RCSBODY_TRAVERSE_SHAPES(self)
  {
    RcsShape_destroy(SHAPE);
    SHAPE = NULL;
    count++;
  }

  return count;
}

/*******************************************************************************
 * Destroy all associated shapes.
 ******************************************************************************/
bool RcsBody_removeShape(RcsBody* self, unsigned int idx)
{
  unsigned int nShapes = RcsBody_numShapes(self);

  if (nShapes == 0)   // self or self->shape are NULL
  {
    return false;
  }

  if (idx > nShapes-1)   // Index out of range
  {
    return false;
  }
  else if (idx == nShapes-1)   // Delete last one - no re-arranging needed
  {
    RcsShape_destroy(self->shape[idx]);
    self->shape[idx] = NULL;
    return true;
  }

  // Index is somewhere in the middle. We first delete the corresponding shape
  RcsShape_destroy(self->shape[idx]);

  // .. and then move all successors one index to the front. The last index
  // (idx+1) is a NULL pointer, so there is no invalid memory read.
  for (unsigned int i=idx; i<nShapes-1; ++i)
  {
    self->shape[idx] = self->shape[idx+1];
  }

  return true;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
RcsBody* RcsBody_getLastInGraph(const RcsGraph* self)
{
  RcsBody* b = RCSBODY_BY_ID(self, self->rootId);
  RcsBody* next = RCSBODY_BY_ID(self, b->nextId);
  RcsBody* lastChild = RCSBODY_BY_ID(self, b->lastChildId);

  while (b && (next || lastChild))
  {
    if (next)
    {
      b = next;
    }
    else
    {
      b = lastChild;
    }

    next = RCSBODY_BY_ID(self, b->nextId);
    lastChild = RCSBODY_BY_ID(self, b->lastChildId);
  }

  return b;
}

/*******************************************************************************
 *
 ******************************************************************************/
void RcsBody_copy(RcsBody* dst, const RcsBody* src)
{
  dst->A_BP = src->A_BP;
  dst->A_BI = src->A_BI;
  dst->m = src->m;
  dst->rigid_body_joints = src->rigid_body_joints;
  dst->physicsSim = src->physicsSim;
  Vec3d_copy(dst->x_dot, src->x_dot);
  Vec3d_copy(dst->omega, src->omega);
  dst->confidence = src->confidence;
  snprintf(dst->name,  RCS_MAX_NAMELEN,   "%s", src->name);
  snprintf(dst->bdySuffix,  RCS_MAX_NAMELEN, "%s", src->bdySuffix);
  snprintf(dst->bdyXmlName, RCS_MAX_NAMELEN, "%s", src->bdyXmlName);
  dst->Inertia = src->Inertia;
}

/*******************************************************************************
* See header.
******************************************************************************/
/* RcsBody* RcsBody_clone(const RcsBody* src) */
/* { */
/*   if (src == NULL) */
/*   { */
/*     return NULL; */
/*   } */

/*   // Make a copy of the body. */
/*   RcsBody* newBody = RcsBody_create(); */
/*   RcsBody_copy(newBody, src); */

/*   // Make a copy of all body shapes and attach them to the body */
/*   int nShapes = RcsBody_numShapes(src); */
/*   newBody->shape = RNALLOC(nShapes + 1, RcsShape*); */
/*   for (int i = 0; i < nShapes; i++) */
/*   { */
/*     newBody->shape[i] = RALLOC(RcsShape); */
/*     RcsShape_copy(newBody->shape[i], src->shape[i]); */
/*   } */

/*   // Make a copy of all body joints and attach them to the body */
/*   RcsJoint* prevJoint = NULL; */

/*   RCSBODY_TRAVERSE_JOINTS(src) */
/*   { */
/*     RcsJoint* j = RALLOC(RcsJoint); */
/*     RcsJoint_copy(j, JNT); */

/*     // Connect joints that belong to body. All outside body connections are */
/*     // not handled here. */
/*     if (prevJoint == NULL) */
/*     { */
/*       newBody->jnt = j; */
/*     } */
/*     else */
/*     { */
/*       j->prev = prevJoint; */
/*       prevJoint->next = j; */
/*     } */

/*     prevJoint = j; */
/*   }   // RCSBODY_TRAVERSE_JOINTS(BODY) */

/*   return newBody; */
/* } */

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool RcsBody_isChild(const RcsGraph* graph,
                     const RcsBody* possibleChild,
                     const RcsBody* possibleParent)
{
  const RcsBody* b = RCSBODY_BY_ID(graph, possibleChild->parentId);

  while (b)
  {
    if (b->id == possibleParent->id)
    {
      return true;
    }
    b = RCSBODY_BY_ID(graph, b->parentId);
  }

  return false;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool RcsBody_isLeaf(const RcsBody* bdy)
{
  if (bdy->firstChildId==-1)
  {
    return true;
  }

  return false;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool RcsBody_isArticulated(const RcsGraph* graph, const RcsBody* self)
{
  RcsJoint* jnt = RcsBody_lastJointBeforeBody(graph, self);

  if (!jnt)
  {
    return false;
  }

  while (jnt->prevId!=-1)
  {
    if (!jnt->constrained)
    {
      return true;
    }
    jnt = &graph->joints[jnt->prevId];
  }

  return false;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool RcsBody_isInGraph(const RcsBody* self, const RcsGraph* graph)
{
  RCSGRAPH_TRAVERSE_BODIES(graph)
  {
    if (self==BODY)
    {
      return true;
    }
  }

  return false;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsBody_fprint(FILE* out, const RcsBody* b, const RcsGraph* graph)
{
  if (b == NULL)
  {
    RLOG(1, "Body doesn't exist");
    return;
  }

  // Body name
  fprintf(out, "[RcsGraph_fprintBody():%d] \n\tBody \"%s\"\n",
          __LINE__, b->name);

  // Previous body name
  const RcsBody* parentBdy = RCSBODY_BY_ID(graph, b->parentId);
  if (parentBdy)
  {
    fprintf(out, "\tis connected to \"%s\"\n", parentBdy->name);
  }
  else
  {
    fprintf(out, "\tis root-level node\n");
  }

  // Next body name
  const RcsBody* attachedBody = RcsBody_depthFirstTraversalGetNextById(graph, b);

  if (attachedBody!=NULL)
  {
    fprintf(out, "\thas body \"%s\" attached\n", attachedBody->name);
  }
  else
  {
    fprintf(out, "\tis last node\n");
  }

  // Number of joints
  int nJoints = RcsBody_numJoints(graph, b);
  fprintf(out, "\thas %d joints\n", nJoints);

  // Print out joint names
  nJoints = 0;
  RCSBODY_FOREACH_JOINT(graph, b)
  {
    fprintf(out, "\t - joint %d is \"%s\" with type %s",
            nJoints, JNT->name, RcsJoint_typeName(JNT->type));

    if (JNT->constrained)
    {
      fprintf(out, " (constrained)\n");
    }
    else
    {
      fprintf(out, " (unconstrained)\n");
    }
    fprintf(out, "\t   q index: %d, Jacobi index: %d",
            JNT->jointIndex, JNT->jacobiIndex);
    if (JNT->coupledToId!=-1)
    {
      fprintf(out, ", coupled to %s\n", JNT->coupledJntName);
    }
    fprintf(out, "\n");

    nJoints++;
  }

  // mass
  fprintf(out, "\tMass: %5.3f kg\n", b->m);

  // Rigid body joints
  fprintf(out, "\tRigid body joints: %s\n",
          b->rigid_body_joints ? "YES" : "NO");

  // Kind of physics simulation
  switch (b->physicsSim)
  {
    case RCSBODY_PHYSICS_NONE:
      fprintf(out, "\tphysics=\"RCSBODY_PHYSICS_NONE\"\n");
      break;

    case RCSBODY_PHYSICS_KINEMATIC:
      fprintf(out, "\tphysics=\"RCSBODY_PHYSICS_KINEMATIC\"\n");
      break;

    case RCSBODY_PHYSICS_DYNAMIC:
      fprintf(out, "\tphysics=\"RCSBODY_PHYSICS_DYNAMIC\"\n");
      break;

    case RCSBODY_PHYSICS_FIXED:
      fprintf(out, "\tphysics=\"RCSBODY_PHYSICS_FIXED\"\n");
      break;

    default:
      fprintf(out, "\tphysics=\"unknown\" (%d)\n", b->physicsSim);
  }

  // Velocities
  fprintf(out, "\tx_dot: %f %f %f\n", b->x_dot[0], b->x_dot[2], b->x_dot[2]);
  fprintf(out, "\tomega: %f %f %f\n", b->omega[0], b->omega[2], b->omega[2]);

  // Confidence
  fprintf(out, "\tConfidence: %f\n", b->confidence);

  // Absolute transformation
  fprintf(out, "\n\tAbsolute transformation:\n");
  HTr_fprint(out, &b->A_BI);
  double ea[3];
  Mat3d_toEulerAngles(ea, (double(*)[3])b->A_BI.rot);
  fprintf(out, "\n\tEuler angles:%f %f %f [deg]\n",
          RCS_RAD2DEG(ea[0]), RCS_RAD2DEG(ea[1]), RCS_RAD2DEG(ea[2]));

  // Relative transformation
  fprintf(out, "\n\tRelative transformation: ");
  if (!HTr_isIdentity(&b->A_BP))
  {
    fprintf(out, "\n");
    HTr_fprint(out, &b->A_BP);
    Mat3d_toEulerAngles(ea, (double(*)[3])b->A_BP.rot);
    fprintf(out, "\n\tEuler angles:%f %f %f [deg]\n",
            RCS_RAD2DEG(ea[0]), RCS_RAD2DEG(ea[1]), RCS_RAD2DEG(ea[2]));
  }
  else
  {
    fprintf(out, "Identity\n");
  }

  // Inertia tensor
  fprintf(out, "\n\tInertia tensor and COM offset:\n");
  HTr_fprint(out, &b->Inertia);


  // Shapes
  fprintf(out, "\n\tBody has %d shapes:\n", RcsBody_numShapes(b));

  if (b->shape != NULL)
  {
    RCSBODY_TRAVERSE_SHAPES(b)
    {
      RcsShape_fprint(out, SHAPE);
    }
  }

  fprintf(out, "\n\n");
}

/*******************************************************************************
 *  Computes the local COM in the body frame according to the volume
 *     of all shapes.
 ******************************************************************************/
static void RcsBody_computeLocalCOM(const RcsBody* self, double* r_com)
{
  Vec3d_setZero(r_com);

  if (self->m == 0.0)
  {
    NLOG(5, "Body \"%s\" has zero mass - local COM is (0 0 0)", self->name);
    return;
  }

  double v_bdy = RcsBody_computeVolume(self);

  if (v_bdy == 0.0)
  {
    NLOG(5, "Body \"%s\" has zero volume - local COM is (0 0 0)", self->name);
    return;
  }

  RCSBODY_TRAVERSE_SHAPES(self)
  {
    double r_com_sh[3], v_sh;
    v_sh = RcsShape_computeVolume(SHAPE);
    RcsShape_computeLocalCOM(SHAPE, r_com_sh);
    Vec3d_constMulSelf(r_com_sh, v_sh / v_bdy);
    Vec3d_addSelf(r_com, r_com_sh);
  }

}

/*******************************************************************************
 * Returns the volume of the body including all shapes in cubic
 * meter. See RcsShape_computeVolume() for details.
 ******************************************************************************/
double RcsBody_computeVolume(const RcsBody* self)
{
  double v = 0.0;

  RCSBODY_TRAVERSE_SHAPES(self)
  {
    v += RcsShape_computeVolume(SHAPE);
  }

  return v;
}

/*******************************************************************************
 * Density: rho = m / V
 ******************************************************************************/
void RcsBody_computeInertiaTensor(const RcsBody* self, HTr* I)
{
  Mat3d_setZero(I->rot);
  RcsBody_computeLocalCOM(self, I->org);

  if (self->m == 0.0)
  {
    RLOG(5, "Mass of body \"%s\" is 0: Inertia tensor set to 0", self->name);
    return;
  }

  double v_bdy = RcsBody_computeVolume(self);

  if (v_bdy == 0.0)
  {
    RLOG(5, "Volume of body \"%s\" is 0: Inertia tensor set to 0", self->name);
    return;
  }

  double density = self->m/v_bdy;   // Density: rho = m / V

  RCSBODY_TRAVERSE_SHAPES(self)
  {
    // Compute the shape's inertia around its COM (it is represented in the
    // shape's frame of reference)
    double I_s[3][3];
    RcsShape_computeInertiaTensor(SHAPE, density, I_s);

    // Rotate inertia tensor from the shape frame into the body frame:
    // B_I = A_BC^T C_I A_CB
    Mat3d_similarityTransform(I_s, (double (*)[3]) SHAPE->A_CB.rot, I_s);

    // Add Steiner term related to gravity center of shape in body frame
    double r_b_sgc[3];   // Vector from body origin to shape gravity center
    RcsShape_computeLocalCOM(SHAPE, r_b_sgc);
    double r_sgc_bgc[3];   // Vector from shape COM to body COM
    Vec3d_sub(r_sgc_bgc, I->org, r_b_sgc);
    double m_sh = density*RcsShape_computeVolume(SHAPE);
    Math_addSteinerToInertia(I_s, r_sgc_bgc, m_sh);
    Mat3d_addSelf(I->rot, I_s);
  }

}

/*******************************************************************************
  * See header.
******************************************************************************/
double RcsBody_distance(const RcsBody* b1,
                        const RcsBody* b2,
                        double cp1[3],
                        double cp2[3],
                        double n[3])
{
  if ((b1->shape==NULL) || (b2->shape==NULL))
  {
    RFATAL("Body has no shape in distance computation");
  }

  double d_closest = Math_infinity();
  double d, p1[3], p2[3], ni[3];

  if (cp1 != NULL)
  {
    Vec3d_setElementsTo(cp1, DBL_MAX);
  }

  if (cp2 != NULL)
  {
    Vec3d_setElementsTo(cp2, -DBL_MAX);
  }

  if (n != NULL)
  {
    Vec3d_setUnitVector(n, 2);
  }

  RcsShape** sh1Ptr = &b1->shape[0];
  RcsShape** sh2Ptr = &b2->shape[0];

  // Traverse all shapes of the 1st body
  while (*sh1Ptr)
  {
    if (((*sh1Ptr)->computeType & RCSSHAPE_COMPUTE_DISTANCE) == 0)
    {
      sh1Ptr++;
      continue;
    }

    // Traverse all shapes of the 2nd body
    while (*sh2Ptr)
    {
      if (((*sh2Ptr)->computeType & RCSSHAPE_COMPUTE_DISTANCE) == 0)
      {
        sh2Ptr++;
        continue;
      }

      // Compute the closest distance via function table lookup
      d = RcsShape_distance(*sh1Ptr, *sh2Ptr, &b1->A_BI, &b2->A_BI, p1, p2, ni);

      // Copy results if distance is smaller than before
      if (d < d_closest)
      {
        d_closest = d;

        if (cp1 != NULL)
        {
          Vec3d_copy(cp1, p1);
        }
        if (cp2 != NULL)
        {
          Vec3d_copy(cp2, p2);
        }
        if (n != NULL)
        {
          Vec3d_copy(n, ni);
        }
      }

      sh2Ptr++;

    }   // while(*sh2Ptr)

    sh1Ptr++;
    sh2Ptr = &b2->shape[0];   // Reset the 2nd shape pointer

  }   // while(*sh1Ptr)



  if (d_closest == Math_infinity())
  {
    RLOG(5, "Body \"%s\" - \"%s\": distance is INFINITY",
         b1->name, b2->name);
  }

  return d_closest;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
double RcsBody_distanceToPoint_org(const RcsBody* body,
                                   const double I_pt[3],
                                   double I_cpBdy[3],
                                   double I_nBdyPt[3])
{
  RcsShape* ptShape = RALLOC(RcsShape);

  ptShape->type = RCSSHAPE_POINT;
  HTr_setIdentity(&ptShape->A_CB);
  Vec3d_setZero(ptShape->extents);
  ptShape->scale = 1.0;
  ptShape->computeType |= RCSSHAPE_COMPUTE_DISTANCE;


  RcsBody* ptBdy = RcsBody_create();
  ptBdy->shape = RNALLOC(2, RcsShape*);
  ptBdy->shape[0] = ptShape;
  ptBdy->shape[1] = NULL;
  Vec3d_copy(ptBdy->A_BI.org, I_pt);

  double tmp[3];
  double d = RcsBody_distance(body, ptBdy, I_cpBdy, tmp, I_nBdyPt);

  // We don't need to destroy the shape here, that's done by the bodie's
  // destroy function automatically.
  RcsBody_destroy(ptBdy);

  return d;
}

/*******************************************************************************
 * Create a temporary body with a point shape, then call the body distance
 * function.
 ******************************************************************************/
double RcsBody_distanceToPoint(const RcsBody* body,
                               const double I_pt[3],
                               double I_cpBdy[3],
                               double I_nBdyPt[3])
{
  RcsShape ptShape;
  memset(&ptShape, 0, sizeof(RcsShape));
  ptShape.type = RCSSHAPE_POINT;
  HTr_setIdentity(&ptShape.A_CB);
  ptShape.scale = 1.0;
  ptShape.computeType |= RCSSHAPE_COMPUTE_DISTANCE;

  RcsBody ptBdy;
  RcsBody_init(&ptBdy);
  Vec3d_copy(ptBdy.A_BI.org, I_pt);

  RcsShape* shapeVec[2];
  shapeVec[0] = &ptShape;
  shapeVec[1] = NULL;
  ptBdy.shape = shapeVec;

  return RcsBody_distance(body, &ptBdy, I_cpBdy, NULL, I_nBdyPt);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
double RcsBody_centerDistance(const RcsBody* b1,
                              const RcsBody* b2,
                              const double k_p1[3],
                              const double k_p2[3],
                              double I_cp1[3],
                              double I_cp2[3])
{
  double I_p1[3], I_p2[3];

  Vec3d_setZero(I_p1);
  Vec3d_setZero(I_p2);

  if (k_p1 != NULL)
  {
    Vec3d_transRotate(I_p1, (double (*)[3])b1->A_BI.rot, k_p1);
  }

  Vec3d_addSelf(I_p1, b1->A_BI.org);

  if (k_p2 != NULL)
  {
    Vec3d_transRotate(I_p2, (double (*)[3])b2->A_BI.rot, k_p2);
  }

  Vec3d_addSelf(I_p2, b2->A_BI.org);

  if (I_cp1 != NULL)
  {
    Vec3d_copy(I_cp1, I_p1);
  }

  if (I_cp2 != NULL)
  {
    Vec3d_copy(I_cp2, I_p2);
  }

  return Vec3d_distance(I_p1, I_p2);
}

/*******************************************************************************
 * Computes the distance gradient del(d)/del(q).
 *
 *       1. dp = (p2-p1) / |p2-p1| (or the normal vector, if available)
 *       2. dDdq = transpose(Jp2-Jp1) * dp
 *
 *       The points are represented in inertial coordinates, since the
 *       different shapes associated with a body might have different
 *       orientations. This way we save a lot of rotations.
 *
 ******************************************************************************/
void RcsBody_distanceGradient(const RcsGraph* self,
                              const RcsBody* b1,
                              const RcsBody* b2,
                              bool repelling,
                              const double* I_p1,
                              const double* I_p2,
                              const double* I_n12,
                              MatNd* dDdq)
{
  double k_p1[3], k_p2[3], dp[3];

  // If I_p is given, then it's I_p. Otherwise, it's the bodie's origin.
  // If the body is NULL, it is the world origin.
  const double* p1 = I_p1 ? I_p1 : (b1 ? b1->A_BI.org : Vec3d_zeroVec());
  const double* p2 = I_p2 ? I_p2 : (b2 ? b2->A_BI.org : Vec3d_zeroVec());

  // Closest points in body coordinates k_p1 and k_p2 for the
  // closest point Jacobian Jp1 and Jp2
  Vec3d_invTransform(k_p1, b1 ? &b1->A_BI : HTr_identity(), p1);
  Vec3d_invTransform(k_p2, b2 ? &b2->A_BI : HTr_identity(), p2);

  // Transpose of the delta Jacobian: transpose(Jp2-Jp1)
  MatNd* J1 = NULL;
  MatNd_create2(J1, 3, self->nJ);
  MatNd* J2 = NULL;
  MatNd_create2(J2, 3, self->nJ);
  RcsGraph_bodyPointJacobian(self, b1, k_p1, NULL, J1);
  RcsGraph_bodyPointJacobian(self, b2, k_p2, NULL, J2);

  MatNd_subSelf(J2, J1);     // J2 = J2 - J1
  MatNd_reshape(J1, J1->n, J1->m);
  MatNd_transpose(J1, J2);   // J1 = transpose(Jp2-Jp1)

  // If we have no normal vector, we calculate the normal direction from
  // the closest points:  dp = (p2-p1) / |p2-p1|
  // This is a bit problematic for d = 0, since there is no known normal
  // direction. Therefore it is always better to take the normal vector,
  // if available.
  if (I_n12 == NULL)
  {
    Vec3d_sub(dp, p2, p1);

    // In penetration, the virtual force needs to be applied in the opposite
    // direction
    if (repelling == false)
    {
      Vec3d_constMulSelf(dp, -1.0);
    }
  }
  // If we have a normal vector, we don't need to care if we are in penetration
  // or not, since it already points into the correct direction.
  else
  {
    Vec3d_copy(dp, I_n12);
  }

  double d = Vec3d_normalizeSelf(dp);


  // If the points coincide (d=0) and no normal vector is given, the
  // penetration direction is ill-defined. In this case, we assume the
  // gradient to be zero.
  if (d == 0.0)
  {
    RLOG(4, "Closest points of bodies \"%s\" and \"%s\" coincide - assuming "
         "distance gradient to be zero", b1 ? b1->name : "NULL",
         b2 ? b2->name : "NULL");
    MatNd_reshapeAndSetZero(dDdq, 1, self->nJ);
    MatNd_destroy(J1);
    MatNd_destroy(J2);
    return;
  }

  // dDdq = transpose(Jp2-Jp1) * dp
  MatNd arr_dp = MatNd_fromPtr(3, 1, dp);
  MatNd_reshape(dDdq, self->nJ, 1);
  MatNd_mul(dDdq, J1, &arr_dp);
  MatNd_reshape(dDdq, 1, self->nJ);

  // Apply individual weighting factors per joint
  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    if (JNT->jacobiIndex > 0)
    {
      dDdq->ele[JNT->jacobiIndex] *= JNT->weightCA;
    }
  }

  MatNd_destroy(J1);
  MatNd_destroy(J2);
}

/*******************************************************************************
 * Computes the distance Hessian.
 ******************************************************************************/
void RcsBody_distanceHessian(const RcsGraph* self,
                             const RcsBody* b1,
                             const RcsBody* b2,
                             bool repelling,
                             const double* I_p1,
                             const double* I_p2,
                             double* H)
{
  RCHECK_PEDANTIC(b1 && b2 && self && H);

  const double* p1 = I_p1 ? I_p1 : b1->A_BI.org;
  const double* p2 = I_p2 ? I_p2 : b2->A_BI.org;

  int n = self->nJ;
  memset(H, 0, n * n * sizeof(double));

  // Closest points in body coordinates k_p1 and k_p2 for the
  // closest point Jacobian Jp1 and Jp2
  double k_p1[3], k_p2[3];
  Vec3d_invTransform(k_p1, &b1->A_BI, p1);
  Vec3d_invTransform(k_p2, &b2->A_BI, p2);

  // Transpose of the delta Jacobian: transpose(Jp2-Jp1)
  MatNd* J1       = NULL;
  MatNd_create2(J1, 3, n);
  MatNd* J2       = NULL;
  MatNd_create2(J2, 3, n);
  MatNd* J2mJ1    = J2;
  MatNd* J2mJ1_tp = J1;
  RcsGraph_bodyPointJacobian(self, b1, k_p1, NULL, J1);
  RcsGraph_bodyPointJacobian(self, b2, k_p2, NULL, J2);
  MatNd_sub(J2mJ1, J2, J1);
  MatNd_reshape(J2mJ1_tp, n, 3);
  MatNd_transpose(J2mJ1_tp, J2mJ1);

  // dp = (p2-p1) / |p2-p1|
  double dp[3];
  Vec3d_sub(dp, p2, p1);
  double d = Vec3d_getLength(dp);

  // If the points coincide, the problem is ill-defined since there is no
  // unique direction of penetration. Since in the Hessian, this would lead
  // to a bunch of divisions by zero, we assume it to be zero instead.
  if (d == 0.0)
  {
    RLOG(4, "Closest points of bodies \"%s\" and \"%s\" coincide - assuming "
         "distance Hessian to be zero", b1->name, b2->name);
    MatNd_destroy(J1);
    MatNd_destroy(J2);
    return;
  }


  // dDdq = transpose(p2-p1/|p2-p1|)*(Jp2-Jp1)
  MatNd dpT = MatNd_fromPtr(1, 3, dp);
  MatNd* dDdq = NULL;
  MatNd_create2(dDdq, 1, n);
  MatNd_mul(dDdq, &dpT, J2mJ1);
  MatNd_constMulSelf(dDdq, 1.0 / d);

  // (J2-J1)^T * (J2-J1)
  MatNd* J12_sqr = NULL;
  MatNd_create2(J12_sqr, n, n);
  MatNd_mul(J12_sqr, J2mJ1_tp, J2mJ1);

  // dDdq * dDdq^T
  MatNd* dDdq_sqr = NULL;
  MatNd_create2(dDdq_sqr, n, n);
  MatNd_dyadicProduct(dDdq_sqr, dDdq);

  // (p2-p1)^T * (H2-H1)
  MatNd* H1 = NULL;
  MatNd_create2(H1, 3*n, n);
  MatNd* H2 = NULL;
  MatNd_create2(H2, 3*n, n);
  RcsGraph_bodyPointHessian(self, b1, k_p1, NULL, H1);
  RcsGraph_bodyPointHessian(self, b2, k_p2, NULL, H2);
  MatNd_subSelf(H2, H1);
  MatNd_reshape(H2, 3, n*n);

  MatNd* pTH = NULL;
  MatNd_create2(pTH, 1, n*n);
  MatNd_mul(pTH, &dpT, H2);

  // Add stuff together
  if (repelling == true)
  {
    for (int i = 0; i < n * n; i++)
    {
      H[i] = (1.0 / d) * (J12_sqr->ele[i] + pTH->ele[i] - dDdq_sqr->ele[i]);
    }
  }
  else
  {
    for (int i = 0; i < n * n; i++)
    {
      H[i] = (1.0 / d) * (-J12_sqr->ele[i] - pTH->ele[i] - dDdq_sqr->ele[i]);
    }
  }

  // Clean up
  MatNd_destroy(H1);
  MatNd_destroy(H2);
  MatNd_destroy(pTH);
  MatNd_destroy(dDdq);
  MatNd_destroy(J12_sqr);
  MatNd_destroy(dDdq_sqr);
  MatNd_destroy(J1);
  MatNd_destroy(J2);
}

/*******************************************************************************
 * Computes the collision cost. The cost consists of two parts:
 *
 *      1. Portion related to closest point distance.
 *
 *      2. Portion related to the distance of the body centers. This is
 *         added once the distance of the bodies is closer than the
 *         threshold RCS_DISTANCE_THRESHOLD. It is scaled with
 *         (1.0-dClosestPts/RCS_DISTANCE_THRESHOLD)
 *
 *       Closest-point cost: c_clp = (s/db^2)*(d-db)^2
 *       Center-point cost:  c_cep = k*(1.0-d/db)*exp(-d)
 *
 ******************************************************************************/
double RcsBody_collisionCost(const RcsBody* b1,
                             const RcsBody* b2,
                             double dThreshold)
{
  const double db = dThreshold, s = RCS_PENETRATION_SLOPE;
  double cp1[3], cp2[3], n[3], dClosestPts, cost = 0.0;


#ifdef TEST_FIXED_CLOSESTPOINTS
  double* kp1 = RCS_GRADIENTTESTS_BODYPT1, *kp2 = RCS_GRADIENTTESTS_BODYPT2;
  dClosestPts = RcsBody_centerDistance(b1, b2, kp1, kp2, cp1, cp2);
#else
  dClosestPts = RcsBody_distance(b1, b2, cp1, cp2, n);
#endif

  if (dClosestPts > db)
  {
    return 0.0;
  }
  else if (dClosestPts < 0)
  {
    cost += -2.0 * s * dClosestPts / db + s;

    RLOG(6, "Collision[C]: <%s - %s>: %f\n", b1->name, b2->name, dClosestPts);
  }
  else
  {
#if 1
    // quadratic
    cost += (s / (db * db)) * (dClosestPts - db) * (dClosestPts - db);
#else
    // linear
    cost += s / db * (db - dClosestPts);
#endif

    RLOG(6, "Collision[N]: <%s - %s>: %f\n", b1->name, b2->name, dClosestPts);
  }


  // Inverse exponential center distance
#ifdef RCS_USE_MIXTURE_GRADIENT
  double dCenters = Vec3d_distance(b1->A_BI->org, b2->A_BI->org);
  if (dClosestPts < 0)
  {
    cost += RCS_MAX_CENTERCOST * exp(-dCenters);
  }
  else
  {
    cost += RCS_MAX_CENTERCOST * (1.0 - dClosestPts / db) * exp(-dCenters);
  }
#endif

  return cost;
}

/*******************************************************************************
 * Computes the collision gradient del(g)/del(q).
 *
 *        Mixture gradient: cost function is
 *
 *          g(dClosestPts, dCenters) =
 *            CD_GRADIENT_CENTERSCALE*(1.0-dClosestPts/db)*exp(-dCenters)
 *
 *  TODO: - That's damn slow. We should compute the distance gradient only
 *          if needed!
 *        - What is "status"? Let's document it (whoever added this variable)
 *
 ******************************************************************************/
double RcsBody_collisionGradient(const RcsGraph* self,
                                 const RcsBody* b1,
                                 const RcsBody* b2,
                                 double db,
                                 MatNd* dH,
                                 double* status)
{
  double dgpDdp, I_p1_closest[3], I_p2_closest[3], n12[3], dClosestPts,
         cost = 0.0;
  const double s = RCS_PENETRATION_SLOPE;


#ifdef TEST_FIXED_CLOSESTPOINTS
  double* kp1 = RCS_GRADIENTTESTS_BODYPT1, *kp2 = RCS_GRADIENTTESTS_BODYPT2;
  dClosestPts = RcsBody_centerDistance(b1, b2, kp1, kp2,
                                       I_p1_closest, I_p2_closest);
#else
  dClosestPts = RcsBody_distance(b1, b2, I_p1_closest, I_p2_closest, n12);
#endif

  // If the distance is larger than threshold db, cost and gradient are zero.
  if (dClosestPts > db)
  {
    MatNd_setZero(dH);
    *status = 0.0;
    return 0.0;
  }

  bool repelling = dClosestPts >= 0.0 ? true : false;

  MatNd* dDpdq = NULL;
  MatNd_create2(dDpdq, dH->m, dH->n);
  RcsBody_distanceGradient(self, b1, b2, repelling,
                           I_p1_closest, I_p2_closest, n12, dDpdq);

  if (dClosestPts < 0)
  {
    cost  += -2.0 * s * dClosestPts / db + s;
    dgpDdp = -2.0 * s / db;
    *status = 1.0;
    RLOG(6, "Coll: dis: %f cost:%f gain:%f\n",dClosestPts, cost, dgpDdp);
  }
  else
  {
#if 1
    // quadratic
    cost  += (s / (db * db)) * (dClosestPts - db) * (dClosestPts - db);
    dgpDdp = (2.0 * s / (db * db)) * (dClosestPts - db);
    *status = (1.0 / (db * db)) * (dClosestPts - db) * (dClosestPts - db);
#else
    // linear
    cost += s / db * (db - dClosestPts);
    dgpDdp = -s / db;
    *status = 1.0 / db * (db - dClosestPts);
#endif
  }

  MatNd_constMul(dH, dDpdq, dgpDdp);



  // Mixture gradient
#ifdef RCS_USE_MIXTURE_GRADIENT

  double I_p1_center[3], I_p2_center[3];
  double dgcDdc, dgcDdp;

  // Partial derivative of cost with respect to dCenters
  double dCenters = RcsBody_centerDistance(b1, b2, NULL, NULL,
                                           I_p1_center, I_p2_center);
  MatNd* dDcdq = NULL;
  MatNd_create2(dDcdq, dH->m, dH->n);
  RcsBody_distanceGradient(self, b1, b2, true, I_p1_center, I_p2_center,
                           dDcdq);

  // dgc/ddc
  if (dClosestPts < 0)
  {
    dgcDdc = -RCS_MAX_CENTERCOST * exp(-dCenters);
  }
  else
  {
    dgcDdc = -RCS_MAX_CENTERCOST * (1.0 - dClosestPts / db) * exp(-dCenters);
  }

  MatNd_constMulSelf(dDcdq, dgcDdc);
  MatNd_addSelf(dH, dDcdq);

  // dgc/ddp
  if (dClosestPts < 0)
  {
    cost += RCS_MAX_CENTERCOST * exp(-dCenters);
    dgcDdp = 0.0;
  }
  else
  {
    cost += RCS_MAX_CENTERCOST * (1.0 - dClosestPts / db) * exp(-dCenters);
    dgcDdp = RCS_MAX_CENTERCOST * (-1.0 / db) * exp(-dCenters);
  }

  MatNd_constMulSelf(dDpdq, dgcDdp);
  MatNd_addSelf(dH, dDpdq);

  MatNd_destroy(dDcdq);
#endif

  MatNd_destroy(dDpdq);

  return cost;
}

/*******************************************************************************
 * Computes the collision Hessian del^2(g)/del(q)^2.
 * \todo: Make more efficient. There's a bunch of heavy things
 * computed even when they are multiplied with zero.
 *
 ******************************************************************************/
void RcsBody_collisionHessian(const RcsGraph* self,
                              const RcsBody* b1,
                              const RcsBody* b2,
                              double dThreshold,
                              double* H)
{
  RCHECK_PEDANTIC(self && b1 && b2 && H);

  int n = self->nJ;
  double d2gpDdp, dgpDdp, I_p1_closest[3], I_p2_closest[3], I_n12[3];
  const double db = dThreshold;
  const double s  = RCS_PENETRATION_SLOPE;

#ifdef TEST_FIXED_CLOSESTPOINTS
  double* kp1 = RCS_GRADIENTTESTS_BODYPT1, *kp2 = RCS_GRADIENTTESTS_BODYPT2;
  double dClosestPts = RcsBody_centerDistance(b1, b2, kp1, kp2,
                                              I_p1_closest, I_p2_closest);
#else
  double dClosestPts = RcsBody_distance(b1, b2, I_p1_closest, I_p2_closest, I_n12);
#endif

  if (dClosestPts > db)
  {
    memset(H, 0, n * n * sizeof(double));
    return;
  }

  MatNd* H1 = NULL;
  MatNd_create2(H1, n, n);
  bool repelling = dClosestPts >= 0.0 ? true : false;

  MatNd* ddpdq = NULL;
  MatNd_create2(ddpdq, n, 1);
  RcsBody_distanceGradient(self, b1, b2, repelling,
                           I_p1_closest, I_p2_closest, I_n12, ddpdq);
  RcsBody_distanceHessian(self, b1, b2, repelling,
                          I_p1_closest, I_p2_closest, H1->ele);
  if (dClosestPts > db)
  {
    dgpDdp  = 0.0;
    d2gpDdp = 0.0;
  }
  else if (dClosestPts < 0)
  {
    dgpDdp  = -2.0 * s / db;
    d2gpDdp =  0.0;
  }
  else
  {
    dgpDdp  = (2.0 * s / (db * db)) * (dClosestPts - db);
    d2gpDdp =  2.0 * s / (db * db);
  }

  // Assemble the overall Hessian
  MatNd ddpdq_sqr = MatNd_fromPtr(n, n, H);
  MatNd_reshape(ddpdq, 1, n);
  MatNd_dyadicProduct(&ddpdq_sqr, ddpdq);
  for (int i = 0; i < n * n; i++)
  {
    H[i] = d2gpDdp * ddpdq_sqr.ele[i] + dgpDdp * H1->ele[i];
  }



  // Mixture term
#ifdef RCS_USE_MIXTURE_GRADIENT

  double I_p1_cntr[3], I_p2_cntr[3];
  double dgcDdc = 0.0, dgcDdp = 0.0, d2gcDdc = 0.0, d2gcDdcDdp = 0.0;
  MatNd* ddcdq = NULL;
  MatNd_create2(ddcdq, n, 1);
  MatNd* H2 = NULL;
  MatNd_create2(H2, n, n);

  // Partial derivative of cost with respect to dCenters
  double dCenters =
    RcsBody_centerDistance(b1, b2, NULL, NULL, I_p1_cntr, I_p2_cntr);
  RcsBody_distanceHessian(self, b1, b2, true, I_p1_cntr, I_p2_cntr, H2->ele);
  RcsBody_distanceGradient(self, b1, b2, true, I_p1_cntr, I_p2_cntr, ddcdq);

  if (dClosestPts > db)
  {
    dgcDdp     = 0.0;
    dgcDdc     = 0.0;
    d2gcDdc    = 0.0;
    d2gcDdcDdp = 0.0;
  }
  else if (dClosestPts < 0)
  {
    dgcDdp     =  0.0;
    dgcDdc     = -RCS_MAX_CENTERCOST * exp(-dCenters);
    d2gcDdc    = -dgcDdc;
    d2gcDdcDdp =  0.0;
  }
  else
  {
    dgcDdp     =  RCS_MAX_CENTERCOST * (-1.0 / db) * exp(-dCenters);
    dgcDdc     = -RCS_MAX_CENTERCOST * (1.0 - dClosestPts / db) * exp(-dCenters);
    d2gcDdc    = -dgcDdc;
    d2gcDdcDdp =  RCS_MAX_CENTERCOST * (1.0 / db) * exp(-dCenters);
  }

  // TODO: This needs more documentation
  MatNd* a1 = NULL;
  MatNd_create2(a1, n, 1);
  MatNd* a2 = NULL;
  MatNd_create2(a2, n, 1);
  MatNd* term1 = NULL;
  MatNd_create2(term1, n, n);

  MatNd_constMul(a1, ddcdq, d2gcDdc);
  MatNd_reshape(ddcdq, 1, n);

  MatNd_reshape(ddpdq, n, 1);
  MatNd_constMul(a2, ddpdq, d2gcDdcDdp);
  MatNd_reshape(ddpdq, 1, n);
  MatNd_addSelf(a2, a1);
  MatNd_mul(term1, a2, ddcdq);

  MatNd* term2 = NULL;
  MatNd_create2(term2, n, n);
  MatNd_reshape(ddcdq, n, 1);
  MatNd_constMul(a1, ddcdq, d2gcDdcDdp);
  MatNd_mul(term2, a1, ddpdq);

  for (int i = 0; i < n * n; i++)
  {
    H[i] += term1->ele[i];
    H[i] += term2->ele[i];
    H[i] += dgcDdc * H2->ele[i];
    H[i] += dgcDdp * H1->ele[i];
  }

  MatNd_destroy(a1);
  MatNd_destroy(a2);
  MatNd_destroy(term1);
  MatNd_destroy(term2);
  MatNd_destroy(H2);
  MatNd_destroy(ddcdq);
#endif

  MatNd_destroy(H1);
  MatNd_destroy(ddpdq);
}

/*******************************************************************************
 * Finds the last "driving" joint before the body. This is
 * required to recurse through the Jacobians.
 *
 * Starting from the bodie's driving joint, we
 *
 * - first check if it exists. If this is not the case, we climb up the
 *   parents until we find one.
 * - Once found, we traverse it until we get to the last one.
 *
 * That should cover all cases.
 *
 ******************************************************************************/
RcsJoint* RcsBody_lastJointBeforeBody(const RcsGraph* graph,
                                      const RcsBody* body)
{
  if (body == NULL)
  {
    return NULL;
  }

  RcsJoint* jnt = RCSJOINT_BY_ID(graph, body->jntId);

  // If the body is not attached to any joint, we traverse its parents until
  // we find one that's driven by a joint.
  while (jnt == NULL)
  {
    // If we reach the root, it means that the chain does not comprise any
    // joint. In this case, we return NULL.
    if (body->parentId == -1)
    {
      return NULL;
    }

    body = &graph->bodies[body->parentId];
    jnt = RCSJOINT_BY_ID(graph, body->jntId);
  }

  RCHECK(jnt);// \todo: Not needed

  // If we found a joint, we traverse its successors up to the last one
  while (jnt->nextId!=-1)
  {
    RCHECK_MSG(jnt->nextId<(int)graph->dof, "%d %d", jnt->nextId, graph->dof);
    jnt = &graph->joints[jnt->nextId];
  }

  return jnt;
}

/*******************************************************************************
 * Creates and initializes the 6 joints associated to a rigid body joint.
 ******************************************************************************/
RcsJoint* RcsBody_createRBJ(RcsGraph* self, RcsBody* b, const double q_rbj[6])
{
  int indexOrdering[6];
  for (int i=0; i<6; ++i)
  {
    indexOrdering[i] = i;
  }

  return RcsBody_createOrdered6DofJoints(self, b, q_rbj, indexOrdering);
}

/*******************************************************************************
 * Creates and initializes the 6 joints associated to a rigid body joint.
 ******************************************************************************/
RcsJoint* RcsBody_createOrdered6DofJoints(RcsGraph* self, RcsBody* b,
                                          const double q_rbj[6],
                                          const int indexOrdering[6])
{
  RCHECK(b);
  int jntId0 = -1;

  for (int i = 0; i < 6; i++)
  {
    RcsJoint* jnt = RcsGraph_insertGraphJoint(self, b->id);

    int nchars = snprintf(jnt->name, RCS_MAX_NAMELEN, "%s_rigidBodyJnt%d",
                          b->name, i);
    if (nchars>=RCS_MAX_NAMELEN)
    {
      RLOG(1, "Joint name truncation happened: %s", jnt->name);
    }
    jnt->constrained = true;

    RCHECK((indexOrdering[i]>=0) && (indexOrdering[i]<6));

    switch (indexOrdering[i])
    {
      case 0:
        jntId0 = jnt->id;
        jnt->type = RCSJOINT_TRANS_X;
        jnt->dirIdx = 0;
        break;
      case 1:
        jnt->type = RCSJOINT_TRANS_Y;
        jnt->dirIdx = 1;
        break;
      case 2:
        jnt->type = RCSJOINT_TRANS_Z;
        jnt->dirIdx = 2;
        break;
      case 3:
        jnt->type = RCSJOINT_ROT_X;
        jnt->dirIdx = 0;
        break;
      case 4:
        jnt->type = RCSJOINT_ROT_Y;
        jnt->dirIdx = 1;
        break;
      case 5:
        jnt->type = RCSJOINT_ROT_Z;
        jnt->dirIdx = 2;
        break;
      default:
        RFATAL("Joint type is \"%s\" (%d)", RcsJoint_typeName(i), i);
    }

    const double qi = q_rbj ? q_rbj[i] : 0.0;
    jnt->q0     = qi;
    jnt->q_max  = qi+2.0*M_PI;
    jnt->q_min  = qi-2.0*M_PI;
    jnt->q_init = qi;
    jnt->weightMetric = 1.0;
    jnt->speedLimit = DBL_MAX;
    jnt->maxTorque = DBL_MAX;
  }

  return RCSJOINT_BY_ID(self, jntId0);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsBody_fprintXML(FILE* out, const RcsBody* self, const RcsGraph* graph)
{
  char buf[256];

  fprintf(out, "  <Body name=\"%s\" ", self->name);

  if (self->parentId != -1)
  {
    fprintf(out, "prev=\"%s\" ", graph->bodies[self->parentId].name);
  }

  if (!HTr_isIdentity(&self->A_BP))
  {
    double trf[6];
    Vec3d_copy(&trf[0], self->A_BP.org);
    Mat3d_toEulerAngles(&trf[3], (double (*)[3]) self->A_BP.rot);
    Vec3d_constMulSelf(&trf[3], 180.0 / M_PI);

    if (VecNd_maxAbsEle(trf, 6) > 1.0e-8)
    {
      fprintf(out, "transform=\"%s ", String_fromDouble(buf, trf[0], 6));
      fprintf(out, "%s ", String_fromDouble(buf, trf[1], 6));
      fprintf(out, "%s ", String_fromDouble(buf, trf[2], 6));
      fprintf(out, "%s ", String_fromDouble(buf, trf[3], 6));
      fprintf(out, "%s ", String_fromDouble(buf, trf[4], 6));
      fprintf(out, "%s\" ", String_fromDouble(buf, trf[5], 6));
    }
  }

  switch (self->physicsSim)
  {
    case RCSBODY_PHYSICS_NONE:
      break;

    case RCSBODY_PHYSICS_KINEMATIC:
      fprintf(out, "physics=\"kinematic\" ");
      break;

    case RCSBODY_PHYSICS_DYNAMIC:
      fprintf(out, "physics=\"dynamic\" ");
      break;

    case RCSBODY_PHYSICS_FIXED:
      fprintf(out, "physics=\"fixed\" ");
      break;

    default:
      RFATAL("Unknown physics simulation type: %d", self->physicsSim);
  }

  // We create all joints explicitely
  // if(self->rigid_body_joints == true)
  //   {
  //     fprintf(out, "rigid_body_joints=\"true\" ");
  //   }

  if (self->m > 0.0)
  {
    fprintf(out, "mass=\"%s\" ", String_fromDouble(buf, self->m, 6));
  }

  const double* cogVector = self->Inertia.org;
  if ((cogVector[0]!=0.0) && (cogVector[1]!=0.0) && (cogVector[2]!=0.0))
  {
    fprintf(out, "cogVector=\"");
    fprintf(out, "%s ", String_fromDouble(buf, cogVector[0], 6));
    fprintf(out, "%s ", String_fromDouble(buf, cogVector[1], 6));
    fprintf(out, "%s\" ", String_fromDouble(buf, cogVector[2], 6));
  }

  // We print out the inertia with the %g format specifier, since the values
  // are sometimes pretty small.
  if (Mat3d_maxAbsEle((double (*)[3])self->Inertia.rot) > 0.0)
  {
    int len = snprintf(buf, 256, "inertia=\"%g %g %g   %g %g %g   %g %g %g",
                       self->Inertia.rot[0][0],
                       self->Inertia.rot[0][1],
                       self->Inertia.rot[0][2],
                       self->Inertia.rot[1][0],
                       self->Inertia.rot[1][1],
                       self->Inertia.rot[1][2],
                       self->Inertia.rot[2][0],
                       self->Inertia.rot[2][1],
                       self->Inertia.rot[2][2]);

    if (len < 256)
    {
      fprintf(out, "%s\" ", buf);
    }
    else
    {
      RLOG(4, "Failed to write inertia to xml: 256 characters exceeded");
    }
  }

  // End body tag
  fprintf(out, ">\n");

  RCSBODY_FOREACH_JOINT(graph, self)
  {
    RcsJoint_fprintXML(out, JNT, graph);
  }

  if (self->shape != NULL)
  {
    RCSBODY_TRAVERSE_SHAPES(self)
    {
      RcsShape_fprintXML(out, SHAPE);
    }
  }

  // Place the sensor's xml into the body description
  for (unsigned int i=0; i<graph->nSensors; ++i)
  {
    const RcsSensor* si = &graph->sensors[i];
    if (si->bodyId==self->id)
    {
      RcsSensor_fprintXML(out, si);
    }
  }

  fprintf(out, "  </Body>\n\n");
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool RcsBody_isFloatingBase(const RcsGraph* graph, const RcsBody* self)
{
  if (self == NULL)
  {
    return false;
  }

  RCSJOINT_TYPE desiredOrder[6];
  desiredOrder[0] = RCSJOINT_TRANS_X;
  desiredOrder[1] = RCSJOINT_TRANS_Y;
  desiredOrder[2] = RCSJOINT_TRANS_Z;
  desiredOrder[3] = RCSJOINT_ROT_X;
  desiredOrder[4] = RCSJOINT_ROT_Y;
  desiredOrder[5] = RCSJOINT_ROT_Z;

  unsigned int nJoints = 0;
  RCSBODY_FOREACH_JOINT(graph, self)
  {
    if (nJoints > 5)
    {
      return false;
    }

    if (JNT->type != desiredOrder[nJoints])
    {
      return false;
    }

    nJoints++;
  }

  if (nJoints != 6)
  {
    return false;
  }

  return true;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool RcsBody_mergeWithParent(RcsGraph* graph, const char* bodyName)
{
#ifdef OLD_TOPO
  RcsBody* body = RcsGraph_getBodyByName(graph, bodyName);

  if (body == NULL)
  {
    RLOG(4, "Body is NULL");
    return false;
  }

  RcsBody* parent = body->parent;

  if (parent == NULL)
  {
    return true;  // Nothing to be done
  }

  if (body->jnt != NULL)
  {
    RLOG(4, "Can't merge body %s - it has joints", body->name);
    return false;
  }

  // Merge COM
  RLOG(5, "Merge COM");
  double V_r_com[3];
  const double* K_r_com = body->Inertia->org;
  if (body->A_BP != NULL)
  {
    Vec3d_transform(V_r_com, body->A_BP, K_r_com);
  }
  else
  {
    Vec3d_copy(V_r_com, K_r_com);
  }

  if (parent->m+body->m > 0.0)
  {
    for (int i=0; i<3; ++i)
    {
      parent->Inertia->org[i] =
        (parent->Inertia->org[i]*parent->m + V_r_com[i]*body->m) /
        (parent->m+body->m);
    }
  }

  // Merge inertia tensors

  // Rotate inertia tensor from the body frame into the parent frame
  RLOG(5, "Merge inertia tensor");
  if (body->A_BP != NULL)
  {
    double I_parent[3][3];
    Mat3d_similarityTransform(I_parent, body->A_BP->rot, body->Inertia->rot);
    Mat3d_addSelf(parent->Inertia->rot, I_parent);
  }
  else
  {
    Mat3d_addSelf(parent->Inertia->rot, body->Inertia->rot);
  }

  // Steiner term 3 principal and 6 deviation components
  RLOG(5, "Merge Steiner terms of inertia tensor");
  Math_addSteinerToInertia(parent->Inertia->rot, V_r_com, body->m);

  // Merge mass
  RLOG(5, "Merge mass");
  parent->m += body->m;

  // Transform all shapes from the child's into the parent's frame
  RLOG(5, "Transform all shapes");
  RCSBODY_TRAVERSE_SHAPES(body)
  {
    if (body->A_BP != NULL)
    {
      /* double r_rel[3]; */
      /* Vec3d_transform(r_rel, body->A_BP, SHAPE->A_CB.org); */
      /* HTr_transformSelf(&SHAPE->A_CB, body->A_BP); */
      /* Vec3d_copy(SHAPE->A_CB.org, r_rel); */

      HTr A_CI;
      HTr_transform(&A_CI, body->A_BI, &SHAPE->A_CB);
      HTr_invTransform(&SHAPE->A_CB, parent->A_BI, &A_CI);
    }
  }

  // Add them to the parent's shape array
  const unsigned int bDim = RcsBody_numShapes(body);

  for (unsigned int i=0; i<bDim; ++i)
  {
    RcsBody_addShape(parent, body->shape[i]);
  }

  // Go through all bodies and update their parent transformations if the
  // body is their parent
  RLOG(5, "Update their parent transformations");
  RCSGRAPH_TRAVERSE_BODIES(graph)
  {
    if (BODY->parent != body)
    {
      continue;
    }

    // Change traversal connectivity. From here, BODY is a child of body.
    RcsBody* child = BODY;

    // Case 1: only 1 child
    if (parent->firstChild==parent->lastChild)
    {
      parent->firstChild = child;
      parent->lastChild = child;
      child->parent = parent;
      child->prev = NULL;
      child->next = NULL;
    }
    else // Case 2: more than 1 children
    {
      parent->lastChild->next = child;
      child->prev = parent->lastChild;
      child->next = NULL;
      parent->lastChild = child;
      child->parent = parent;
    }

    // Change relative transformations
    if (body->A_BP != NULL)
    {
      if (child->jnt == NULL)   // There is only a fixed transformation
      {
        if (child->A_BP != NULL)
        {
          HTr_transformSelf(child->A_BP, body->A_BP);
        }
        else
        {
          child->A_BP = HTr_clone(body->A_BP);
        }
      }
      else   // There is a joint
      {
        if (child->jnt->A_JP != NULL)
        {
          HTr_transformSelf(child->jnt->A_JP, body->A_BP);
        }
        else
        {
          child->jnt->A_JP = HTr_clone(body->A_BP);
        }
      }
    }
  }



  // Change sensor mountpoints
  RLOG(5, "Update sensor mountpoints");
  RCSGRAPH_TRAVERSE_SENSORS(graph)
  {
    if (SENSOR->body != body)
    {
      continue;
    }

    SENSOR->body = parent;

    if (body->A_BP != NULL)
    {
      if (SENSOR->offset != NULL)
      {
        HTr_transformSelf(SENSOR->offset, body->A_BP);
      }
      else
      {
        SENSOR->offset = HTr_clone(body->A_BP);
      }
    }

  }

  body->shape[0] = NULL;   // Otherwise the shapes will be deleted
  //RcsBody_destroy(body);
  MatNd* qVec[2];
  qVec[0] = graph->q;
  qVec[1] = graph->q_dot;
  RcsGraph_removeBody(graph, body->name, qVec, 2);

  return true;
#else
  RFATAL("Implement me");
  return false;
#endif
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsBody_computeAABB(const RcsBody* self,
                         double xyzMin[3], double xyzMax[3])
{
  if (self == NULL || RcsBody_numShapes(self)==0)
  {
    RLOG(4, "Body is NULL or has no shapes - AABB is set to zero");
    Vec3d_setZero(xyzMin);
    Vec3d_setZero(xyzMax);
    return;
  }

  Vec3d_set(xyzMin, DBL_MAX, DBL_MAX, DBL_MAX);
  Vec3d_set(xyzMax, -DBL_MAX, -DBL_MAX, -DBL_MAX);

  RCSBODY_TRAVERSE_SHAPES(self)
  {
    double B_min[3], B_max[3], C_min[3], C_max[3];
    RcsShape_computeAABB(SHAPE, C_min, C_max);
    Vec3d_transform(B_min, &SHAPE->A_CB, C_min);
    Vec3d_transform(B_max, &SHAPE->A_CB, C_max);

    for (int j = 0; j < 3; ++j)
    {
      if (B_min[j] < xyzMin[j])
      {
        xyzMin[j] = B_min[j];
      }

      if (B_max[j] > xyzMax[j])
      {
        xyzMax[j] = B_max[j];
      }
    }
  }

}

/*******************************************************************************
* See header.
******************************************************************************/
RcsBody* RcsBody_createBouncingSphere(RcsGraph* graph, const double pos[3],
                                      double mass, double radius)
{
  static int sphereCount = 0;
  RcsShape* sphere = RALLOC(RcsShape);

  sphere->type = RCSSHAPE_SPHERE;
  HTr_setIdentity(&sphere->A_CB);
  Vec3d_setElementsTo(sphere->extents, radius);
  sphere->scale = 1.0;
  sphere->computeType |= RCSSHAPE_COMPUTE_GRAPHICS;
  sphere->computeType |= RCSSHAPE_COMPUTE_PHYSICS;
  strcpy(sphere->material, "bouncy");
  strcpy(sphere->color, "YELLOW");

  RcsBody* bdy = RcsGraph_insertGraphBody(graph, -1);
  bdy->shape = RNALLOC(2, RcsShape*);
  bdy->shape[0] = sphere;
  bdy->shape[1] = NULL;
  snprintf(bdy->name, RCS_MAX_NAMELEN, "BouncingSphere_%d", sphereCount);
  bdy->m = mass;
  bdy->physicsSim = RCSBODY_PHYSICS_DYNAMIC;
  bdy->rigid_body_joints = true;
  RcsBody_computeInertiaTensor(bdy, &bdy->Inertia);
  Vec3d_copy(bdy->A_BI.org, pos);

  double q6[6];
  Vec3d_copy(q6, pos);
  Vec3d_setZero(&q6[3]);
  bdy->jntId = RcsBody_createRBJ(graph, bdy, q6)->id;
  sphereCount++;

  return bdy;
}

/*******************************************************************************
 * See header. We don't need to adjust the sensor mount points, since they are
 * represented with respect to the body.
 ******************************************************************************/
bool RcsBody_removeJoints(RcsBody* self, RcsGraph* graph)
{
  // Update body relative transform to predecessor considering the sequence of
  // transforms of all body joints.
  HTr A_JB;
  HTr_setIdentity(&A_JB);

  RCSBODY_FOREACH_JOINT(graph, self)
  {
    HTr_transformSelf(&A_JB, &JNT->A_JP);

    double qi = MatNd_get(graph->q, JNT->jointIndex, 0);

    // Apply transformations of joints
    int dirIdx = RcsJoint_getDirectionIndex(JNT);

    if (RcsJoint_isTranslation(JNT))
    {
      Vec3d_constMulAndAddSelf(A_JB.org, A_JB.rot[dirIdx], qi);
    }
    else
    {
      Mat3d_rotateSelfAboutXYZAxis(A_JB.rot, dirIdx, qi);
    }
  }

  HTr_transformSelf(&self->A_BP, &A_JB);



  // Mark all body joints invalid using RcsJoint_init()
  self->jntId = -1;
  RCSBODY_FOREACH_JOINT(graph, self)
  {
    RcsJoint_init(JNT);
  }



  // Connect child-bodies backwards joint connection to the closest parent
  // joints.
  const RcsJoint* bodiesPrevJnt = RcsBody_lastJointBeforeBody(graph, self);
  int prevId = bodiesPrevJnt ? bodiesPrevJnt->id : -1;
  RcsBody* child = RCSBODY_BY_ID(graph, self->firstChildId);

  while (child)
  {
    child->prevId = prevId;
    child = RCSBODY_BY_ID(graph, child->nextId);
  }


  // Update q-arrays and indices
  RcsGraph_makeJointsConsistent(graph);

  return true;
}

/*******************************************************************************
 * This function needs the GeometricTools library to be linked. Otherwise the
 * OBB cannot be computed and we return false.
 ******************************************************************************/
bool RcsBody_boxify(RcsBody* self, int computeType)
{
  if (self == NULL)
  {
    RLOG(4, "Can't boxify NULL body");
    return false;
  }

  if (self->shape == NULL)
  {
    RLOG(4, "Body \"%s\" has no shapes attached - skipping boxify",
         self->name);
    return false;
  }

  unsigned int nPts = 8*RcsBody_numShapes(self);
  MatNd* vertices = MatNd_create(nPts, 3);
  unsigned int shapeIdx = 0;

  RCSBODY_TRAVERSE_SHAPES(self)
  {
    if ((SHAPE->computeType & computeType) != 0)
    {
      double xyzMin[3], xyzMax[3];
      RcsShape_computeAABB(SHAPE, xyzMin, xyzMax);
      double* row = MatNd_getRowPtr(vertices, 8*shapeIdx);

      Vec3d_set(row,   xyzMin[0], xyzMin[1], xyzMin[2]);
      Vec3d_set(row+3, xyzMax[0], xyzMin[1], xyzMin[2]);
      Vec3d_set(row+6, xyzMax[0], xyzMax[1], xyzMin[2]);
      Vec3d_set(row+9, xyzMin[0], xyzMax[1], xyzMin[2]);

      Vec3d_set(row+12, xyzMin[0], xyzMin[1], xyzMax[2]);
      Vec3d_set(row+15, xyzMax[0], xyzMin[1], xyzMax[2]);
      Vec3d_set(row+18, xyzMax[0], xyzMax[1], xyzMax[2]);
      Vec3d_set(row+21, xyzMin[0], xyzMax[1], xyzMax[2]);

      for (int i=0; i<8; ++i)
      {
        Vec3d_transformSelf(&row[i*3], &SHAPE->A_CB);
      }

      shapeIdx++;
    }

  }

  if (shapeIdx == 0)
  {
    RLOG(4, "Body \"%s\" has no shape to boxify for computeType %d",
         self->name, computeType);
    MatNd_destroy(vertices);
    return false;
  }

  MatNd_reshape(vertices, 8*shapeIdx, 3);
  RcsShape* boxShape = RALLOC(RcsShape);
  boxShape->type = RCSSHAPE_BOX;

  bool success = Rcs_computeOrientedBox(&boxShape->A_CB, boxShape->extents,
                                        vertices->ele, vertices->m);

  if (success==false)
  {
    RLOG(4, "Failed to compute enclosing box for body \"%s\"", self->name);
    MatNd_destroy(vertices);
    return false;
  }

  boxShape->scale = 1.0;
  boxShape->computeType = computeType;

  // Replace body shapes with enclosing box
  RcsShape** sPtr = self->shape;
  while (*sPtr)
  {
    RcsShape_destroy(*sPtr);
    sPtr++;
  }

  // In case there were no shapes, we need to allocate memory
  self->shape = (RcsShape**) realloc(self->shape, 2*sizeof(RcsShape*));
  RCHECK(self->shape);
  self->shape[0] = boxShape;
  self->shape[1] = NULL;

  MatNd_destroy(vertices);

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
void RcsBody_scale(RcsGraph* graph, RcsBody* bdy, double scale)
{
  Vec3d_constMulSelf(bdy->A_BP.org, scale);

  RCSBODY_FOREACH_JOINT(graph, bdy)
  {
    RcsJoint_scale(JNT, scale);
  }

  RCSBODY_TRAVERSE_SHAPES(bdy)
  {
    RcsShape_scale(SHAPE, scale);
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
int RcsBody_getNumDistanceQueries(const RcsBody* b1, const RcsBody* b2)
{
  int fcnCount = 0;
  RcsShape** sh1Ptr = &b1->shape[0];
  RcsShape** sh2Ptr = &b2->shape[0];

  // Traverse all shapes of the 1st body
  while (*sh1Ptr)
  {
    if (((*sh1Ptr)->computeType & RCSSHAPE_COMPUTE_DISTANCE) == 0)
    {
      sh1Ptr++;
      continue;
    }

    // Traverse all shapes of the 2nd body
    while (*sh2Ptr)
    {
      if (((*sh2Ptr)->computeType & RCSSHAPE_COMPUTE_DISTANCE) == 0)
      {
        sh2Ptr++;
        continue;
      }

      // Compute the closest distance via function table lookup
      fcnCount++;
      sh2Ptr++;
    }   // while(*sh2Ptr)

    sh1Ptr++;
    sh2Ptr = &b2->shape[0];   // Reset the 2nd shape pointer

  }   // while(*sh1Ptr)

  return fcnCount;
}


/*******************************************************************************
 * See header.
 ******************************************************************************/
RcsBody* RcsBody_getPrev(RcsGraph* graph, RcsBody* body)
{
  return RCSBODY_BY_ID(graph, body->prevId);
}

RcsBody* RcsBody_getNext(RcsGraph* graph, RcsBody* body)
{
  return RCSBODY_BY_ID(graph, body->nextId);
}

RcsBody* RcsBody_getParent(RcsGraph* graph, RcsBody* body)
{
  return RCSBODY_BY_ID(graph, body->parentId);
}

const RcsBody* RcsBody_getConstParent(const RcsGraph* graph, const RcsBody* body)
{
  return RCSBODY_BY_ID(graph, body->parentId);
}

RcsBody* RcsBody_getFirstChild(RcsGraph* graph, RcsBody* body)
{
  return RCSBODY_BY_ID(graph, body->firstChildId);
}

RcsBody* RcsBody_getLastChild_(RcsGraph* graph, RcsBody* body)
{
  return RCSBODY_BY_ID(graph, body->lastChildId);
}

RcsBody* RcsBody_depthFirstTraversalGetNextById(const RcsGraph* graph,
                                                const RcsBody* body)
{
  if (body == NULL)
  {
    return NULL;
  }

  if (body->firstChildId!=-1)
  {
    //RLOG(0, "First child: %s", graph->bodies[body->firstChildId]->name);
    return &graph->bodies[body->firstChildId];
  }

  if (body->nextId != -1)
  {
    //RLOG(0, "Next: %s", graph->bodies[body->nextId]->name);
    return &graph->bodies[body->nextId];
  }

  RcsBody* body2 = (body->parentId != -1) ? &graph->bodies[body->parentId] : NULL;

  while (body2)
  {
    if (body2->nextId != -1)
    {
      //RLOG(0, "Second next: %s", graph->bodies[body2->nextId]->name);
      return &graph->bodies[body2->nextId];
    }

    body2 = (body2->parentId != -1) ? &graph->bodies[body2->parentId] : NULL;
  }

  //RLOG(0, "Nullinger - no parent");

  return NULL;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
RcsBody* RcsBody_getLastChildById(const RcsGraph* graph, RcsBody* b)
{
  if (!b)
  {
    return NULL;
  }

  while (b->lastChildId != -1)
  {
    b = &graph->bodies[b->lastChildId];
  }

  return b;
}

/*******************************************************************************
 *
 ******************************************************************************/
RcsBody* RcsBody_first(const RcsGraph* graph)
{
  return graph->nBodies > 0 ? &graph->bodies[0] : NULL;
}

/*******************************************************************************
 *
 ******************************************************************************/
RcsBody* RcsBody_last(const RcsGraph* graph)
{
  return graph->nBodies > 0 ? &graph->bodies[graph->nBodies-1] : NULL;
}

/*******************************************************************************
 *
 ******************************************************************************/
RcsJoint* RcsBody_getJoint(const RcsBody* b, const RcsGraph* graph)
{
  if (b==NULL)
  {
    return NULL;
  }

  return RCSJOINT_BY_ID(graph, b->jntId);
}

/*******************************************************************************
 * Transformation from body to world coordinates
 *
 * A_1I   = A_12 * A_2I
 * I_r_1 = I_r_2 + A_I2 * 2_r_21
 *
 * A_1I   = A_21^T * A_2I
 * I_r_1 = I_r_2 - A_1I^T * 1_r_12
 ******************************************************************************/
static void HTr_transform2(HTr* A_1I, const HTr* A_2I, const HTr* A_21)
{
  Mat3d_transposeMul(A_1I->rot, (double(*)[3]) A_21->rot,
                     (double(*)[3]) A_2I->rot);
  Vec3d_transRotate(A_1I->org, (double(*)[3]) A_1I->rot, A_21->org);
  Vec3d_constMulSelf(A_1I->org, -1.0);
  Vec3d_addSelf(A_1I->org, A_2I->org);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool RcsBody_attachToBodyId(RcsGraph* graph, int bodyId, int targetId)
{
  RcsBody* body = RCSBODY_BY_ID(graph, bodyId);

  if (!body)
  {
    RLOG(1, "Body to attach does not exist (id=%d)", bodyId);
    return false;
  }

  RcsBody* target = RCSBODY_BY_ID(graph, targetId);

  if (body->parentId==targetId)
  {
    return true; // Nothing to do
  }

  // take the body and children out of the graph TODO maybe put this in a
  // separate method
  RcsBody* bPrev = RCSBODY_BY_ID(graph, body->prevId);
  if (bPrev)
  {
    bPrev->nextId = body->nextId;
  }

  RcsBody* bNext = RCSBODY_BY_ID(graph, body->nextId);
  if (bNext)
  {
    bNext->prevId = body->prevId;
  }

  RcsBody* bParent = RCSBODY_BY_ID(graph, body->parentId);
  if (bParent)
  {
    if (bParent->firstChildId == bodyId)
    {
      bParent->firstChildId = body->nextId;
    }
    if (bParent->lastChildId == bodyId)
    {
      bParent->lastChildId = body->prevId;
    }
  }

  body->nextId = -1;
  body->prevId = -1;

  // put it into the new position
  if (target)
  {
    body->parentId = targetId;

    if (target->lastChildId != -1)
    {
      graph->bodies[target->lastChildId].nextId = body->id;
      body->prevId = target->lastChildId;
    }
    else
    {
      target->firstChildId = bodyId;
    }

    target->lastChildId = bodyId;
  }
  else
  {
    body->parentId = -1;

    RcsBody* t = &graph->bodies[graph->rootId];
    while (t->nextId!=-1)
    {
      t = RCSBODY_BY_ID(graph, t->nextId);
    }
    t->nextId = body->id;
    body->prevId = t->id;
  }

  // If the body has no rigid body joints, we compute the relative transform
  // so that the attached body will not undergo any jump. This might not be
  // correct in case the body to be attached has joints with relative
  // transforms. \todo: Check and possibly do similar as below in the case of
  // rigid body dofs.
  if (!body->rigid_body_joints)
  {
    if (target)
    {
      HTr_invTransform(&body->A_BP, &target->A_BI, &body->A_BI);
    }
    else
    {
      HTr_copy(&body->A_BP, &body->A_BI);   // Leave where it is
    }
  }

  // Connect the joints
  if (body->jntId!=-1)
  {
    RcsJoint* bdyJoint = &graph->joints[body->jntId];
    RcsJoint* parentJnt = RcsBody_lastJointBeforeBody(graph, target);
    bdyJoint->prevId = parentJnt ? parentJnt->id : -1;

    // Set all rigid body joints to 0. The relative transformation is already
    // handled with A_BP
    if (body->rigid_body_joints)
    {
      // We compute the rigid body joint so that the attached body will not
      // undergo any jump. For this, we need to consider the relative transform
      // of the body to be attached, since it is applied after the rigid body
      // joints. We therefore compute the fame before the relative transform
      // (A_1I) and calculate the rigid body dofs based on the relative
      // transformation between the frame A_1I and the parent's frame.
      HTr A_1I, tmp;
      HTr_transform2(&A_1I, &body->A_BI, &body->A_BP);

      if (target)
      {
        HTr_invTransform(&tmp, &target->A_BI, &A_1I);
      }
      else
      {
        HTr_copy(&tmp, &A_1I);   // Leave where it is
      }

      HTr_to6DVector(&graph->q->ele[bdyJoint->jointIndex], &tmp);
    }

  }

  return true;
}
