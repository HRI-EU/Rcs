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
void RcsBody_destroy(RcsBody* self)
{
  RcsJoint* jnt, *next;

  if (self == NULL)
  {
    NLOG(1, "Body is NULL - returning");
    return;
  }

  // Destroy all associated joints
  jnt  = self->jnt;
  next = NULL;

  while (jnt)
  {
    NLOG(0, "Deleting joint \"%s\"", jnt->name);
    next = jnt->next;
    RcsJoint_destroy(jnt);
    jnt = next;
  }

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

  RFREE(self->name);
  RFREE(self->xmlName);
  RFREE(self->suffix);
  RFREE(self->A_BP);
  RFREE(self->A_BI);
  RFREE(self->Inertia);

  // Reset all internal memory
  memset(self, 0, sizeof(RcsBody));

  RFREE(self);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
unsigned int RcsBody_numJoints(const RcsBody* self)
{
  unsigned int nJoints = 0;

  if (self == NULL)
  {
    return 0;
  }

  RCSBODY_TRAVERSE_JOINTS(self)
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
  RcsBody* b = self->root;

  while (b && (b->next || b->lastChild))
  {
    if (b->next)
    {
      b = b->next;
    }
    else
    {
      b = b->lastChild;
    }
  }

  return b;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
RcsBody* RcsBody_getGraphRoot(const RcsBody* self)
{
  if (self==NULL)
  {
    return NULL;
  }

  while (self->parent)  // Go up to the level 0
  {
    self = self->parent;
  }

  while (self->prev)  // Go left to the root node
  {
    self = self->prev;
  }

  return (RcsBody*) self;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool RcsBody_attachToBody(RcsGraph* graph, RcsBody* body, RcsBody* target,
                          const HTr* A_BP)
{
  if (body == NULL)
  {
    RLOG(1, "Body to attach is NULL");
    return false;
  }

  // We disallow attaching bodies to generic bodies
  if (target != NULL)
  {
    if (STRNEQ(target->name, "GenericBody", 11))
    {
      RLOG(1, "Cannot connect body %s to a GenericBody", body->name);
      return false;
    }
  }

  // If the body to attach is a generic body, we attach the body it is
  // pointing to
  if (STRNEQ(body->name, "GenericBody", 11))
  {
    RcsBody* bPtr = (RcsBody*) body->extraInfo;
    RCHECK(bPtr);

    if (bPtr->parent == target)
    {
      return true; // Nothing to do
    }

    RcsBody_attachToBody(graph, bPtr, target, A_BP);

    // Important: This needs to be reseted manually here, since the address
    // of prev, next, parent, child and last might be changed
    body->parent     = bPtr->parent;
    body->firstChild = bPtr->firstChild;
    body->lastChild  = bPtr->lastChild;
    body->next       = bPtr->next;
    body->prev       = bPtr->prev;

    // the following two lines are needed in case re-attaching a body also
    // leads to a change of joint configuration
    // (e.g., the rigid_body_joints are removed as a free floating body is
    //  attached to a robot gripper)
    body->jnt      = bPtr->jnt;
    body->rigid_body_joints = bPtr->rigid_body_joints;

    NLOG(0, "Attached generic body \"%s\" (pointing to \"%s\") to body"
         " \"%s\"", body->name, bPtr ? bPtr->name : "NULL",
         target ? target->name : "NULL");
    return true;
  }

  if ((body->parent==target) && (target!=NULL))
  {
    return true; // Nothing to do
  }

  // take the body and children out of the graph TODO maybe put this in a
  // separate method
  if (body->prev)
  {
    body->prev->next = body->next;
  }

  if (body->next)
  {
    body->next->prev = body->prev;
  }

  if (body->parent)
  {
    if (body->parent->firstChild == body)
    {
      body->parent->firstChild = body->next;
    }
    if (body->parent->lastChild == body)
    {
      body->parent->lastChild = body->prev;
    }
  }

  body->next = NULL;
  body->prev = NULL;

  // put it into the new position
  body->parent = target;
  if (body->parent)
  {
    if (body->parent->lastChild)
    {
      body->parent->lastChild->next = body;
      body->prev = body->parent->lastChild;
    }
    else
    {
      body->parent->firstChild = body;
    }

    body->parent->lastChild = body;
  }
  else
  {
    RcsBody* t = graph->root;
    if (t)
    {
      while (t->next)
      {
        t = t->next;
      }
      t->next = body;
      body->prev = t;
    }
    else
    {
      graph->root = body;
    }
  }

  if (target != NULL)
  {
    if (body->A_BP == NULL)
    {
      body->A_BP = HTr_create();
    }

    if (!A_BP)
    {
      // Calculate the new A_BP = A_BI * (A_VI)'
      HTr_invTransform(body->A_BP, target->A_BI, body->A_BI);
    }
    else
    {
      HTr_copy(body->A_BP, A_BP);
    }
  }
  else   // no target, but A_BP
  {
    if (A_BP != NULL)
    {
      if (body->A_BP == NULL)
      {
        body->A_BP = HTr_clone(A_BP);
      }
      else
      {
        HTr_copy(body->A_BP, A_BP);
      }
    }
  }

  // Connect the joints
  if (body->jnt != NULL)
  {
    if (body->parent)
    {
      body->jnt->prev = RcsBody_lastJointBeforeBody(body->parent);
    }
    else
    {
      body->jnt->prev = NULL;
    }
  }

  NLOG(0, "Attached %s to %s",
       body->name, target ? target->name : "NULL");

  return true;
}



/*******************************************************************************
 *
 * Makes a deep copy of a RcsBody data structure. The field extraInfo is
 * skipped. The following fields can only get updated on the level of the graph:
 *
 * RcsBody *prev
 * RcsBody *next
 * RcsShape **shape
 * RcsJoint *jnt
 *
 ******************************************************************************/
void RcsBody_copy(RcsBody* dst, const RcsBody* src)
{
  dst->m = src->m;
  dst->rigid_body_joints = src->rigid_body_joints;
  dst->physicsSim = src->physicsSim;
  Vec3d_copy(dst->x_dot, src->x_dot);
  Vec3d_copy(dst->omega, src->omega);

  String_copyOrRecreate(&dst->name, src->name);
  String_copyOrRecreate(&dst->xmlName, src->xmlName);
  String_copyOrRecreate(&dst->suffix, src->suffix);

  HTr_copyOrRecreate(&dst->A_BP, src->A_BP);
  HTr_copyOrRecreate(&dst->A_BI, src->A_BI);
  HTr_copyOrRecreate(&dst->Inertia, src->Inertia);
}

/*******************************************************************************
* See header.
******************************************************************************/
RcsBody* RcsBody_clone(const RcsBody* src)
{
  if (src == NULL)
  {
    return NULL;
  }

  // Make a copy of the body.
  RcsBody* newBody = RALLOC(RcsBody);
  RcsBody_copy(newBody, src);

  // Make a copy of all body shapes and attach them to the body
  int nShapes = RcsBody_numShapes(src);
  newBody->shape = RNALLOC(nShapes + 1, RcsShape*);
  for (int i = 0; i < nShapes; i++)
  {
    newBody->shape[i] = RALLOC(RcsShape);
    RcsShape_copy(newBody->shape[i], src->shape[i]);
  }

  // Make a copy of all body joints and attach them to the body
  RcsJoint* prevJoint = NULL;

  RCSBODY_TRAVERSE_JOINTS(src)
  {
    RcsJoint* j = RALLOC(RcsJoint);
    RcsJoint_copy(j, JNT);

    // Connect joints that belong to body. All outside body connections are
    // not handled here.
    if (prevJoint == NULL)
    {
      newBody->jnt = j;
    }
    else
    {
      j->prev = prevJoint;
      prevJoint->next = j;
    }

    prevJoint = j;
  }   // RCSBODY_TRAVERSE_JOINTS(BODY)

  return newBody;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
RcsBody* RcsBody_depthFirstTraversalGetNext(const RcsBody* body)
{
  if (body==NULL)
  {
    return NULL;
  }

  if (body->firstChild)
  {
    return body->firstChild;
  }

  if (body->next)
  {
    return body->next;
  }

  RcsBody* body2 = body->parent;

  while (body2)
  {
    if (body2->next)
    {
      return body2->next;
    }

    body2 = body2->parent;
  }

  return NULL;
}

/*******************************************************************************
 * \todo Write documentation
 ******************************************************************************/
RcsBody* RcsBody_depthFirstTraversalGetPrevious(const RcsBody* body)
{
  if (!body || (!body->prev && !body->parent))
  {
    return NULL;
  }

  RcsBody* b = NULL;
  if (body->prev)
  {
    b = body->prev;

    while (b->lastChild)// What does this do?
    {
      b = b->lastChild;
    }
  }
  else if (body->parent)
  {
    b = body->parent;
  }

  return b;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
RcsBody* RcsBody_getLastChild(const RcsBody* body)
{
  if (!body)
  {
    return NULL;
  }

  RcsBody* b = (RcsBody*) body;
  while (b->lastChild)
  {
    b = b->lastChild;
  }

  return b;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool RcsBody_isChild(const RcsBody* possibleChild,
                     const RcsBody* possibleParent)
{
  const RcsBody* b = possibleChild->parent;

  while (b)
  {
    if (b == possibleParent)
    {
      return true;
    }
    b = b->parent;
  }

  return false;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool RcsBody_isLeaf(const RcsBody* bdy)
{
  if (bdy->firstChild==NULL)
  {
    return true;
  }

  return false;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool RcsBody_isArticulated(const RcsBody* self)
{
  RcsJoint* jnt = RcsBody_lastJointBeforeBody(self);

  while (jnt)
  {
    if (!jnt->constrained)
    {
      return true;
    }
    jnt = jnt->prev;
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
void RcsBody_fprint(FILE* out, const RcsBody* b)
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
  if (b->parent)
  {
    fprintf(out, "\tis connected to \"%s\"\n", b->parent->name);
  }
  else
  {
    fprintf(out, "\tis root node\n");
  }

  // Next body name
  const RcsBody* attachedBody = RcsBody_depthFirstTraversalGetNext(b);

  if (attachedBody!=NULL)
  {
    fprintf(out, "\thas body \"%s\" attached\n", attachedBody->name);
  }
  else
  {
    fprintf(out, "\tis last node\n");
  }

  // Number of joints
  int nJoints = RcsBody_numJoints(b);
  fprintf(out, "\thas %d joints\n", nJoints);

  // Print out joint names
  nJoints = 0;
  RCSBODY_TRAVERSE_JOINTS(b)
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
    if (JNT->coupledTo)
    {
      fprintf(out, ", coupled to %s\n", JNT->coupledJointName);
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

  // Absolute transformation
  fprintf(out, "\n\tAbsolute transformation:\n");
  HTr_fprint(out, b->A_BI);

  // Relative transformation
  fprintf(out, "\n\tRelative transformation:\n");
  if (b->A_BP)
  {
    HTr_fprint(out, b->A_BP);
    double ea[3];
    Mat3d_toEulerAngles(ea, b->A_BP->rot);
    fprintf(out, "\n\tEuler angles:%f %f %f [deg]\n",
            RCS_RAD2DEG(ea[0]), RCS_RAD2DEG(ea[1]), RCS_RAD2DEG(ea[2]));
  }
  else
  {
    fprintf(out, "\n\tIdentity\n");
  }

  // Inertia tensor
  fprintf(out, "\n\n\tInertia tensor and COM offset:\n");
  HTr_fprint(out, b->Inertia);


  // Shapes
  fprintf(out, "\thas %d shapes\n", RcsBody_numShapes(b));

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
      d = RcsShape_distance(*sh1Ptr, *sh2Ptr, b1->A_BI, b2->A_BI, p1, p2, ni);

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
    RLOG(3, "Body \"%s\" - \"%s\": distance is INFINITY",
         b1->name, b2->name);
  }

  return d_closest;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
double RcsBody_distanceToPoint(const RcsBody* body,
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


  RcsBody* ptBdy = RALLOC(RcsBody);
  ptBdy->shape = RNALLOC(2, RcsShape*);
  ptBdy->shape[0] = ptShape;
  ptBdy->shape[1] = NULL;
  ptBdy->A_BI = HTr_create();
  Vec3d_copy(ptBdy->A_BI->org, I_pt);

  double tmp[3];
  double d = RcsBody_distance(body, ptBdy, I_cpBdy, tmp, I_nBdyPt);

  // We don't need to destroy the shape here, that's done by the bodie's
  // destroy function automatically.
  RcsBody_destroy(ptBdy);

  return d;
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
    Vec3d_transRotate(I_p1, b1->A_BI->rot, k_p1);
  }

  Vec3d_addSelf(I_p1, b1->A_BI->org);

  if (k_p2 != NULL)
  {
    Vec3d_transRotate(I_p2, b2->A_BI->rot, k_p2);
  }

  Vec3d_addSelf(I_p2, b2->A_BI->org);

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
  const double* p1 = I_p1 ? I_p1 : (b1 ? b1->A_BI->org : Vec3d_zeroVec());
  const double* p2 = I_p2 ? I_p2 : (b2 ? b2->A_BI->org : Vec3d_zeroVec());

  // Closest points in body coordinates k_p1 and k_p2 for the
  // closest point Jacobian Jp1 and Jp2
  Vec3d_invTransform(k_p1, b1 ? b1->A_BI : HTr_identity(), p1);
  Vec3d_invTransform(k_p2, b2 ? b2->A_BI : HTr_identity(), p2);

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
    MatNd_reshapeAndSetZero(dDdq, self->nJ, 1);
    MatNd_destroy(J1);
    MatNd_destroy(J2);
    return;
  }

  // dDdq = transpose(Jp2-Jp1) * dp
  MatNd arr_dp = MatNd_fromPtr(3, 1, dp);
  MatNd_reshape(dDdq, self->nJ, 1);
  MatNd_mul(dDdq, J1, &arr_dp);

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

  const double* p1 = I_p1 ? I_p1 : b1->A_BI->org;
  const double* p2 = I_p2 ? I_p2 : b2->A_BI->org;

  int n = self->nJ;
  memset(H, 0, n * n * sizeof(double));

  // Closest points in body coordinates k_p1 and k_p2 for the
  // closest point Jacobian Jp1 and Jp2
  double k_p1[3], k_p2[3];
  Vec3d_invTransform(k_p1, b1->A_BI, p1);
  Vec3d_invTransform(k_p2, b2->A_BI, p2);

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
RcsJoint* RcsBody_lastJointBeforeBody(const RcsBody* body)
{
  if (body == NULL)
  {
    return NULL;
  }

  RcsJoint* jnt = body->jnt;

  // If the body is not attached to any joint, we traverse its parents until
  // we find one that's driven by a joint. If we reach the root, it means that
  // the chain does not comprise any joint. In this case, we return NULL.
  while (jnt == NULL)
  {
    if (body->parent == NULL)
    {
      return NULL;
    }

    body = body->parent;
    jnt = body->jnt;
  }

  // If we found a joint, we traverse its successors up to the last one
  while (jnt->next)
  {
    jnt = jnt->next;
  }

  return jnt;
}

/*******************************************************************************
 * Creates and initializes the 6 joints associated to a rigid body joint.
 ******************************************************************************/
RcsJoint* RcsBody_createRBJ(RcsGraph* self, RcsBody* b, const double q_rbj[6])
{
  int indexOrdering[6];
  for (int i=0;i<6;++i)
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
  RcsJoint* jnt0 = NULL;

  for (int i = 0; i < 6; i++)
  {
    RcsJoint* jnt = RNALLOC(1, RcsJoint);
    RcsGraph_insertJoint(self, b, jnt);
    HTr_setIdentity(&jnt->A_JI);
    jnt->name =
      RNALLOC(strlen(b->name) + strlen("_rigidBodyJnt") + 1 + 1, char);
    sprintf(jnt->name, "%s_rigidBodyJnt%d", b->name, i);
    jnt->constrained = true;

    RCHECK((indexOrdering[i]>=0) && (indexOrdering[i]<6));

    switch (indexOrdering[i])
    {
      case 0:
        jnt0 = jnt;
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
  }

  return jnt0;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsBody_fprintXML(FILE* out, const RcsBody* self, const RcsGraph* graph)
{
  char buf[256];

  fprintf(out, "  <Body name=\"%s\" ", self->name);

  if (self->parent != NULL)
  {
    fprintf(out, "prev=\"%s\" ", self->parent->name);
  }

  if (self->A_BP != NULL)
  {
    double trf[6];
    Vec3d_copy(&trf[0], self->A_BP->org);
    Mat3d_toEulerAngles(&trf[3], (double (*)[3]) self->A_BP->rot);
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

  const double* cogVector = self->Inertia->org;
  if ((cogVector[0]!=0.0) && (cogVector[1]!=0.0) && (cogVector[2]!=0.0))
  {
    fprintf(out, "cogVector=\"");
    fprintf(out, "%s ", String_fromDouble(buf, cogVector[0], 6));
    fprintf(out, "%s ", String_fromDouble(buf, cogVector[1], 6));
    fprintf(out, "%s\" ", String_fromDouble(buf, cogVector[2], 6));
  }

  // We print out the inertia with the %g format specifier, since the values
  // are sometimes pretty small.
  if (Mat3d_maxAbsEle(self->Inertia->rot) > 0.0)
  {
    int len = snprintf(buf, 256, "inertia=\"%g %g %g   %g %g %g   %g %g %g",
                       self->Inertia->rot[0][0],
                       self->Inertia->rot[0][1],
                       self->Inertia->rot[0][2],
                       self->Inertia->rot[1][0],
                       self->Inertia->rot[1][1],
                       self->Inertia->rot[1][2],
                       self->Inertia->rot[2][0],
                       self->Inertia->rot[2][1],
                       self->Inertia->rot[2][2]);

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

  RCSBODY_TRAVERSE_JOINTS(self)
  {
    RcsJoint_fprintXML(out, JNT);
  }

  if (self->shape != NULL)
  {
    RCSBODY_TRAVERSE_SHAPES(self)
    {
      RcsShape_fprintXML(out, SHAPE);
    }
  }

  // Place the sensor's xml into the body description
  RCSGRAPH_TRAVERSE_SENSORS(graph)
  {
    if (SENSOR->body==self)
    {
      RcsSensor_fprintXML(out, SENSOR);
    }
  }

  fprintf(out, "  </Body>\n\n");
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool RcsBody_isFloatingBase(const RcsBody* self)
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
  RCSBODY_TRAVERSE_JOINTS(self)
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
      double r_rel[3];
      Vec3d_transform(r_rel, body->A_BP, SHAPE->A_CB.org);
      HTr_transformSelf(&SHAPE->A_CB, body->A_BP);
      Vec3d_copy(SHAPE->A_CB.org, r_rel);
    }
  }

  // Add them to the parent's shape array
  unsigned int bDim = RcsBody_numShapes(body);
  unsigned int pDim = RcsBody_numShapes(parent);
  parent->shape = (RcsShape**) realloc(parent->shape,
                                       (bDim+pDim+1)*sizeof(RcsShape*));
  RCHECK(parent->shape);
  for (unsigned int i=0; i<bDim; ++i)
  {
    parent->shape[i+pDim] = body->shape[i];
    parent->shape[i+pDim+1] = NULL;
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

    // Change traversal connectivity

    // Case 1: only 1 child
    if (parent->firstChild==parent->lastChild)
    {
      parent->firstChild = BODY;
      parent->lastChild = BODY;
      BODY->parent = body->parent;
      BODY->prev = NULL;
      BODY->next = NULL;
    }
    else // Case 2: more than 1 children
    {
      parent->lastChild->next = BODY;
      BODY->prev = parent->lastChild;
      BODY->next = NULL;
      parent->lastChild = BODY;
      BODY->parent = body->parent;
    }

    // Change relative transformations
    if (body->A_BP != NULL)
    {
      if (BODY->jnt == NULL)   // There is only a fixed transformation
      {
        if (BODY->A_BP != NULL)
        {
          HTr_transformSelf(BODY->A_BP, body->A_BP);
        }
        else
        {
          BODY->A_BP = HTr_clone(body->A_BP);
        }
      }
      else   // There is a joint
      {
        if (BODY->jnt->A_JP != NULL)
        {
          HTr_transformSelf(BODY->jnt->A_JP, body->A_BP);
        }
        else
        {
          BODY->jnt->A_JP = HTr_clone(body->A_BP);
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
  RcsBody_destroy(body);

  return true;
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
RcsBody* RcsBody_createBouncingSphere(const double pos[3],
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
  sphere->material = String_clone("default");
  sphere->color = String_clone("YELLOW");

  RcsBody* bdy = RALLOC(RcsBody);
  bdy->shape = RNALLOC(2, RcsShape*);
  bdy->shape[0] = sphere;
  bdy->shape[1] = NULL;
  char a[64];
  sprintf(a, "BouncingSphere_%d", sphereCount);
  bdy->name = String_clone(a);
  bdy->m = mass;
  bdy->physicsSim = RCSBODY_PHYSICS_DYNAMIC;
  bdy->A_BI = HTr_create();
  bdy->Inertia = HTr_create();
  RcsBody_computeInertiaTensor(bdy, bdy->Inertia);
  double q_rbj[6];
  VecNd_setZero(q_rbj, 6);
  Vec3d_copy(q_rbj, pos);

  bdy->rigid_body_joints = true;

  for (int i = 0; i < 6; i++)
  {
    RcsJoint* jnt = RNALLOC(1, RcsJoint);
    HTr_setIdentity(&jnt->A_JI);
    jnt->name =
      RNALLOC(strlen(bdy->name) + strlen("_rigidBodyJnt") + 1 + 1, char);
    sprintf(jnt->name, "%s_rigidBodyJnt%d", bdy->name, i);
    jnt->constrained = true;

    switch (i)
    {
      case 0:
        bdy->jnt = jnt;
        jnt->type = RCSJOINT_TRANS_X;
        jnt->dirIdx = 0;
        jnt->prev = NULL;
        break;
      case 1:
        jnt->type = RCSJOINT_TRANS_Y;
        jnt->dirIdx = 1;
        jnt->prev = bdy->jnt;
        jnt->prev->next = jnt;
        break;
      case 2:
        jnt->type = RCSJOINT_TRANS_Z;
        jnt->dirIdx = 2;
        jnt->prev = bdy->jnt->next;
        jnt->prev->next = jnt;
        break;
      case 3:
        jnt->type = RCSJOINT_ROT_X;
        jnt->dirIdx = 0;
        jnt->prev = bdy->jnt->next->next;
        jnt->prev->next = jnt;
        break;
      case 4:
        jnt->type = RCSJOINT_ROT_Y;
        jnt->dirIdx = 1;
        jnt->prev = bdy->jnt->next->next->next;
        jnt->prev->next = jnt;
        break;
      case 5:
        jnt->type = RCSJOINT_ROT_Z;
        jnt->dirIdx = 2;
        jnt->prev = bdy->jnt->next->next->next->next;
        jnt->prev->next = jnt;
        jnt->next = NULL;
        break;
      default:
        RFATAL("Joint type is \"%s\" (%d)", RcsJoint_typeName(i), i);
    }

    jnt->q0 = q_rbj[i];
    jnt->q_max = q_rbj[i] + 2.0*M_PI;
    jnt->q_min = q_rbj[i] - 2.0*M_PI;
    jnt->q_init = q_rbj[i];
    jnt->weightMetric = 1.0;
  }

  sphereCount++;

  return bdy;
}

/*******************************************************************************
 * See header. We don't need to adjust the sensor mount points, since they are
 * represented with respect to the body.
 ******************************************************************************/
bool RcsBody_removeJoints(RcsBody* self, RcsGraph* graph)
{
  if (!RcsBody_isInGraph(self, graph))
  {
    RLOG(4, "Body \"%s\" not found in graph - skipping joint removal",
         self ? self->name : "NULL");
    return false;
  }

  HTr A_JB;
  HTr_setIdentity(&A_JB);

  RCSBODY_TRAVERSE_JOINTS(self)
  {
    if (JNT->A_JP)
    {
      HTr_transformSelf(&A_JB, JNT->A_JP);
    }

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

  // Update body relative transform to predecessor
  if (self->A_BP)
  {
    HTr_transformSelf(self->A_BP, &A_JB);
  }
  else
  {
    self->A_BP = HTr_clone(&A_JB);
  }

  // Remove all joints
  unsigned int nJoints = RcsBody_numJoints(self);
  unsigned int ji = 0;
  RcsJoint** jArr = RNALLOC(nJoints, RcsJoint*);

  RCSBODY_TRAVERSE_JOINTS(self)
  {
    jArr[ji] = JNT;
    ji++;
  }

  for (unsigned int i = 0; i < nJoints; ++i)
  {
    RcsJoint_destroy(jArr[i]);
  }

  RFREE(jArr);
  self->jnt = NULL;
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

  self->shape[0] = boxShape;
  self->shape[1] = NULL;

  MatNd_destroy(vertices);

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
void RcsBody_scale(RcsBody* bdy, double scale)
{
  if (bdy->A_BP)
    {
      Vec3d_constMulSelf(bdy->A_BP->org, scale);
    }

  RCSBODY_TRAVERSE_JOINTS(bdy)
    {
      RcsJoint_scale(JNT, scale);
    }

  RCSBODY_TRAVERSE_SHAPES(bdy)
    {
      RcsShape_scale(SHAPE, scale);
    }

}

