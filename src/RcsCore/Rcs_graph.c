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
#include "Rcs_graphParser.h"
#include "Rcs_URDFParser.h"
#include "Rcs_BVHParser.h"
#include "Rcs_macros.h"
#include "Rcs_utils.h"
#include "Rcs_body.h"
#include "Rcs_joint.h"
#include "Rcs_shape.h"
#include "Rcs_kinematics.h"
#include "Rcs_sensor.h"
#include "Rcs_resourcePath.h"
#include "Rcs_math.h"

#include <float.h>



/*******************************************************************************
 * This function computes the body kinematics for one body of the kinematics
 * chain. It propagates the transformation of the previous body through all
 * relative transformations and the transformations due to the joints which
 * connect the body to the previous one. This function is the core of the
 * kinematics chain and Jacobian computation. In this function it is assumed
 * that the state vector has been set so that the joint values pointed to by
 * RcsJoint::q are up to date.
 ******************************************************************************/
static void RcsGraph_bodyKinematics(RcsBody* bdy, unsigned int* nJ,
                                    const MatNd* q, const MatNd* q_dot)
{
  // Copy last body transformation to current one
  HTr_copy(bdy->A_BI, bdy->parent ? bdy->parent->A_BI : HTr_identity());

  // Set pointers to propagated elements
  HTr* A_VI   = bdy->A_BI;
  RcsJoint* j = bdy->jnt;


  // Traverse joints
  while (j != NULL)
  {
    // Joint position: Here we assume that the joint angles already reflect
    // the kinematic joint coupling (if applicable)
    double qi = MatNd_get2(q, j->jointIndex, 0);

    // Propagate the joint's world transform with the joint's relative
    // transform: I_r_IJ' = I_r_IJ + (A_JI)^T J_r_JJ' and A_J'I = A_J'J * A_JI
    if (j->A_JP != NULL)
    {
      Vec3d_transMulAndAdd(j->A_JI.org, A_VI->org, A_VI->rot, j->A_JP->org);
      Mat3d_mul(j->A_JI.rot, j->A_JP->rot, A_VI->rot);
    }
    else
    {
      HTr_copy(&j->A_JI, A_VI);
    }

    // Apply transformations of joints
    switch (j->type)
    {
      case RCSJOINT_TRANS_X:
      case RCSJOINT_TRANS_Y:
      case RCSJOINT_TRANS_Z:
      {
        Vec3d_constMulAndAddSelf(j->A_JI.org, j->A_JI.rot[j->dirIdx], qi);
        break;
      }
      case RCSJOINT_ROT_X:
      {
        Mat3d_rotateSelfAboutXYZAxis(j->A_JI.rot, 0, qi);
        break;
      }
      case RCSJOINT_ROT_Y:
      {
        Mat3d_rotateSelfAboutXYZAxis(j->A_JI.rot, 1, qi);
        break;
      }
      case RCSJOINT_ROT_Z:
      {
        Mat3d_rotateSelfAboutXYZAxis(j->A_JI.rot, 2, qi);
        break;
      }

      default:
      {
        RFATAL("Unhandled joint type %d in joint \"%s\"", j->type, j->name);
      }
    }

    // If the joint is constrained, set the Jacobian index of the joint
    // invalid (-1). Otherwise, set the Jacobian index of the joint to the
    // latest active joint.
    if (j->constrained == true)
    {
      j->jacobiIndex = -1;
    }
    else
    {
      j->jacobiIndex = *nJ;
      *nJ = *nJ + 1;
    }

    // Remember previous transform and propagate to next joint
    A_VI   = &j->A_JI;
    j      = j->next;

  }   // while(jnt)

  // Copy last joint transform to body
  if (bdy->A_BI != A_VI)
  {
    HTr_copy(bdy->A_BI, A_VI);
  }

  // Apply relative rotation of body wrt. last joint
  if (bdy->A_BP != NULL)
  {
    Vec3d_transMulAndAddSelf(bdy->A_BI->org, bdy->A_BI->rot, bdy->A_BP->org);
    Mat3d_preMulSelf(bdy->A_BI->rot, bdy->A_BP->rot);
  }



  // Velocities
  if (q_dot != NULL)
  {
    double q_dot_i, oxr[3], om_i[3];

    // Linear velocity according to parent angular velocity
    if (bdy->parent == NULL)
    {
      Vec3d_setZero(bdy->x_dot);
      Vec3d_setZero(bdy->omega);
    }
    else
    {
      // Reset velocities for root bodies
      Vec3d_copy(bdy->x_dot, bdy->parent->x_dot);
      Vec3d_copy(bdy->omega, bdy->parent->omega);

      // Linear velocity term due to the parent's angular velocity
      double tmp[3];
      Vec3d_sub(tmp, bdy->A_BI->org, bdy->parent->A_BI->org);
      Vec3d_crossProduct(oxr, bdy->parent->omega, tmp);
      Vec3d_addSelf(bdy->x_dot, oxr);
    }

    // Velocity terms
    j = bdy->jnt;

    while (j != NULL)
    {
      // Joint velocity: Here we assume that the joint angles already
      // reflect the kinematic joint coupling (if applicable)
      q_dot_i = MatNd_get2(q_dot, j->jointIndex, 0);

      switch (j->type)
      {
        case RCSJOINT_TRANS_X:
        case RCSJOINT_TRANS_Y:
        case RCSJOINT_TRANS_Z:
        {
          Vec3d_constMulAndAddSelf(bdy->x_dot, j->A_JI.rot[j->dirIdx], q_dot_i);
          break;
        }
        case RCSJOINT_ROT_X:
        case RCSJOINT_ROT_Y:
        case RCSJOINT_ROT_Z:
        {
          // Propagate angular velocity
          Vec3d_constMul(om_i, j->A_JI.rot[j->dirIdx], q_dot_i);
          Vec3d_addSelf(bdy->omega, om_i);

          // Propagate linear velocity due to Euler term
          double tmp[3];
          Vec3d_sub(tmp, bdy->A_BI->org, j->A_JI.org);
          Vec3d_crossProduct(oxr, om_i, tmp);
          Vec3d_addSelf(bdy->x_dot, oxr);
          break;
        }
      }
      j = j->next;

    }   // while(jnt)

  } // if(q_dot != NULL)

}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool RcsGraph_setState(RcsGraph* self, const MatNd* q_, const MatNd* q_dot_)
{
  bool qChanged = false;

  if (q_ != NULL)
  {
    if (q_->m == self->nJ)
    {
      RcsGraph_stateVectorFromIK(self, q_, self->q);
    }
    else if (q_->m == self->dof)
    {
      if (self->q != q_)
      {
        MatNd_copy(self->q, q_);
      }
    }
    else
    {
      RFATAL("q has wrong dimensions: [%d x %d], dof is %d, nJ is %d",
             q_->m, q_->n, self->dof, self->nJ);
    }
  }

  if (q_dot_ != NULL)
  {
    if (q_dot_->m == self->nJ)
    {
      RcsGraph_stateVectorFromIK(self, q_dot_, self->q_dot);
    }
    else if (q_dot_->m == self->dof)
    {
      if (self->q_dot != q_dot_)
      {
        MatNd_copy(self->q_dot,  q_dot_);
      }
    }
    else
    {
      RFATAL("q_dot has wrong dimensions: [%d x %d], dof is %d, nJ is %d",
             q_dot_->m, q_dot_->n, self->dof, self->nJ);
    }
  }

  // If q_dot is NULL, no updating of the joint velocities will be done.
  qChanged = RcsGraph_updateSlaveJoints(self, self->q,
                                        q_dot_ ? self->q_dot : NULL);

  // Forward kinematics
  unsigned int nJ = 0;
  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    RcsGraph_bodyKinematics(BODY, &nJ, self->q,q_dot_ ? self->q_dot : NULL);
  }

  // In most cases, nJ remains unchanged. We therefore only assign a new value
  // if it is changes, which allows us to access it under many circumstances
  // from other threads without a mutex

  if (self->nJ != nJ)
  {
    self->nJ = nJ;
  }

  return qChanged;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsGraph_computeForwardKinematics(RcsGraph* self,
                                       const MatNd* q,
                                       const MatNd* q_dot)
{
  if (q != NULL)
  {
    RCHECK(q->m == self->dof);
  }

  if (q_dot != NULL)
  {
    RCHECK(q_dot->m == self->dof);
  }

  self->nJ = 0;
  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    RcsGraph_bodyKinematics(BODY, &self->nJ,
                            q ? q : self->q, q_dot ? q_dot : self->q_dot);
  }

}

/*******************************************************************************
 * See header.
 ******************************************************************************/
RcsGraph* RcsGraph_create(const char* configFile)
{
  // Determine absolute file name of config file and return if it doesn't exist
  char filename[256] = "";
  bool fileExists = Rcs_getAbsoluteFileName(configFile, filename);

  if (fileExists==false)
  {
    REXEC(1)
    {
      RMSG("Resource path is:");
      Rcs_printResourcePath();
      RMSG("RcsGraph configuration file \"%s\" not found in "
           "ressource path - exiting", configFile ? configFile : "NULL");
    }
    return NULL;
  }

  // Try parsing bounding volume hierarchy file for .bvh suffix
  if (String_hasEnding(filename, ".bvh", false))
  {
    return RcsGraph_createFromBVHFile(filename, 0.01, true);
  }

  // Read XML file
  xmlDocPtr doc = NULL;
  xmlNodePtr node = parseXMLFile(filename, NULL, &doc);

  if (node == NULL)
  {
    if (doc != NULL)
    {
      xmlFreeDoc(doc);
    }

    RLOG(1, "Failed to parse xml file \"%s\"", filename);
    return NULL;
  }

  // From here, we have a valid xmlNode pointer and xml document pointer.

  // If the top-level xml tag is <Graph>, we directly parse from there.
  if (!xmlStrcmp(node->name, (const xmlChar*) "Graph"))
  {
    RcsGraph* self = RcsGraph_createFromXmlNode(node);

    if (self != NULL)
    {
      String_copyOrRecreate(&self->xmlFile, filename);
    }

    xmlFreeDoc(doc);
    return self;
  }

  // From here, we don't have a Graph element on the top-level node.

  // It might be one level deeper (for instance included in a controller
  // xml file). We therefore search through the node's children.
  RLOG(5, "Node type is \"%s\" - looking for \"Graph\" in children",
       node->name);
  xmlNodePtr childNode = getXMLChildByName(node, "Graph");

  // In case we found it, the node is set to the corresponding child
  // and we continue parsing from there.
  if (childNode)
  {
    RLOG(5, "Found child node \"Graph\" in children");
    RcsGraph* self = RcsGraph_createFromXmlNode(childNode);

    if (self != NULL)
    {
      String_copyOrRecreate(&self->xmlFile, filename);
    }

    xmlFreeDoc(doc);
    return self;
  }



  // We test if the top-level node has a string property "graph". In this case,
  // it's likely to be a controller file, and we parse the graph by its name.
  RLOG(5, "Didn't find child node \"Graph\" in children - trying"
       " graph property");

  char graphFilename[256] = "";
  unsigned int len = getXMLNodePropertyStringN(node, "graph",
                                               graphFilename, 256);

  // If this test succeeds, we clear all memory and call this
  // function recursively with the graph's file name.
  if (len>0)
  {
    RLOG(5, "Found \"graph\" property - parsing \"%s\"", graphFilename);
    xmlFreeDoc(doc);
    return RcsGraph_create(graphFilename);
  }

  // If nothing worked, we try if it is an URDF file
  RLOG(5, "Trying URDF file");
  xmlFreeDoc(doc);

  return RcsGraph_fromURDFFile(filename);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
RcsGraph* RcsGraph_createFromBuffer(const char* buffer, unsigned int size)
{
  if (buffer == NULL)
  {
    RLOG(1, "buffer node is NULL");
    return NULL;
  }

  // Read XML file
  xmlDocPtr doc;
  xmlNodePtr node = parseXMLMemory(buffer, size, &doc);

  if (node == NULL)
  {
    RLOG(1, "xmlNodePtr node is NULL");
    RLOG(4, "buffer is \"%s\"", buffer);
    return NULL;
  }

  RcsGraph* self = RcsGraph_createFromXmlNode(node);

  // Free the xml memory
  xmlFreeDoc(doc);

  if (self == NULL)
  {
    return NULL;
  }

  RFREE(self->xmlFile);
  self->xmlFile = String_clone("Created_from_memory_buffer");

  return self;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsGraph_destroy(RcsGraph* self)
{
  if (self == NULL)
  {
    return;
  }

  NLOG(5, "Deleting graph \"%s\" (addr 0x%x)",
       self->xmlFile, (unsigned int) self);

  // Delete generic bodies dynamic memory. If the generic body points to
  // NULL, it had its own memory and we need to delete it here. This must
  // go before deleting the bodies.
  for (int i = 0; i < 10; i++)
  {
    NLOG(0, "Deleting generic body %d", i);
    if (self->gBody[i].extraInfo == NULL)
    {
      // Those elements point to another body if not "NULL"
      RFREE(self->gBody[i].A_BI);
      RFREE(self->gBody[i].A_BP);
      RFREE(self->gBody[i].Inertia);
    }

    // These elements belong exclusively to the gBody
    RFREE(self->gBody[i].name);
    RFREE(self->gBody[i].xmlName);
    RFREE(self->gBody[i].suffix);
  }

  // Destroy all bodies
  RcsBody* b    = RcsBody_getLastInGraph(self);
  RcsBody* prev = NULL;

  while (b != NULL)
  {
    NLOG(0, "Deleting body \"%s\" (addr %p)", b->name, b);
    prev = RcsBody_depthFirstTraversalGetPrevious(b);
    RcsBody_destroy(b);
    b = prev;
  }
  NLOG(0, "Deleted body list");

  // Destroy all sensors
  RcsSensor* curr_sensor = self->sensor;
  RcsSensor* next_sensor = NULL;

  while (curr_sensor != NULL)
  {
    NLOG(0, "Deleting sensor \"%s\" (addr %p)",
         curr_sensor->name, curr_sensor);
    next_sensor = curr_sensor->next;
    RcsSensor_destroy(curr_sensor);
    curr_sensor = next_sensor;
  }
  NLOG(0, "Deleted sensor list");

  MatNd_destroy(self->q);
  MatNd_destroy(self->q_dot);

  RFREE(self->xmlFile);
  NLOG(0, "Deleted internal memory");

  // Reset all memory to 0
  memset(self, 0, sizeof(RcsGraph));
  NLOG(0, "Resetted internal memory");

  RFREE(self);

  NLOG(5, "RcsGraph deleted");
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
double RcsGraph_mass(const RcsGraph* self)
{
  double m = 0.0;

  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    m += BODY->m;
  }

  return m;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
double RcsGraph_massFromBody(const RcsGraph* self, const char* root)
{
  double m = 0.0;

  RcsBody* r = NULL;
  if (root && (r = RcsGraph_getBodyByName(self, root)))
  {
    RCSBODY_TRAVERSE_BODIES(r)
    {
      m += BODY->m;
    }
  }
  else
  {
    RCSGRAPH_TRAVERSE_BODIES(self)
    {
      m += BODY->m;
    }
  }

  return m;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsGraph_stateVectorFromIK(const RcsGraph* self,
                                const MatNd* q_ik,
                                MatNd* q_full)
{
  RCHECK_MSG(q_ik->m   == self->nJ,  "%d != %d", q_ik->m, self->nJ);
  RCHECK_MSG(q_full->m == self->dof, "%d != %d", q_full->m, self->dof);

  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    if (JNT->jacobiIndex != -1)
    {
      q_full->ele[JNT->jointIndex] = q_ik->ele[JNT->jacobiIndex];
    }
  }

}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsGraph_stateVectorFromIKSelf(const RcsGraph* self, MatNd* q)
{
  RCHECK_MSG(q->m == self->nJ,  "%d != %d", q->m, self->nJ);

  MatNd* tmp = NULL;
  MatNd_clone2(tmp, q);
  MatNd_reshapeAndSetZero(q, self->dof, 1);

  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    if (JNT->jacobiIndex != -1)
    {
      q->ele[JNT->jointIndex] = tmp->ele[JNT->jacobiIndex];
    }
  }

  MatNd_destroy(tmp);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsGraph_stateVectorToIK(const RcsGraph* self,
                              const MatNd* q_full,
                              MatNd* q_ik)
{
  int rdof = 0;

  // State vector must contain all dof
  RCHECK_MSG(q_full->m == self->dof, "%d != %d", q_full->m, self->dof);

  // Target vector must be large enough
  RCHECK_MSG(q_ik->size >= self->nJ, "size=%d  nJ=%d", q_ik->size, self->nJ);

  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    if (JNT->jacobiIndex != -1)
    {
      q_ik->ele[JNT->jacobiIndex] = q_full->ele[JNT->jointIndex];
      rdof++;
    }
  }

  // Consistency check: Number of unconstrained joints must match nJ
  RCHECK(rdof == self->nJ);

  // Reshape
  q_ik->m = rdof;
  q_ik->n = 1;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsGraph_stateVectorToIKSelf(const RcsGraph* self, MatNd* q)
{
  MatNd* tmp = NULL;
  MatNd_clone2(tmp, q);
  RcsGraph_stateVectorToIK(self, tmp, q);
  MatNd_destroy(tmp);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsGraph_getDefaultState(const RcsGraph* self, MatNd* q0)
{
  MatNd_reshape(q0, self->dof, 1);

  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    MatNd_set2(q0, JNT->jointIndex, 0, JNT->q0);
  }
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsGraph_getJointLimits(const RcsGraph* self, MatNd* q_lower,
                             MatNd* q_upper, RcsStateType type)
{
  unsigned int dim = (type == RcsStateFull) ? self->dof : self->nJ;

  MatNd_reshape(q_lower, dim, 1);
  MatNd_reshape(q_upper, dim, 1);

  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    if (type == RcsStateFull)
    {
      MatNd_set(q_lower, JNT->jointIndex, 0, JNT->q_min);
      MatNd_set(q_upper, JNT->jointIndex, 0, JNT->q_max);
    }
    else if ((type == RcsStateIK) && (JNT->jacobiIndex != -1))
    {
      MatNd_set(q_lower, JNT->jacobiIndex, 0, JNT->q_min);
      MatNd_set(q_upper, JNT->jacobiIndex, 0, JNT->q_max);
    }
  }
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsGraph_getSpeedLimits(const RcsGraph* self, MatNd* q_dot_limit,
                             RcsStateType type)
{
  MatNd_reshape(q_dot_limit, (type==RcsStateFull) ? self->dof : self->nJ, 1);

  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    if (type == RcsStateFull)
    {
      MatNd_set(q_dot_limit, JNT->jointIndex, 0, JNT->speedLimit);
    }
    else if ((type == RcsStateIK) && (JNT->jacobiIndex != -1))
    {
      MatNd_set(q_dot_limit, JNT->jacobiIndex, 0, JNT->speedLimit);
    }
  }
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsGraph_getTorqueLimits(const RcsGraph* self, MatNd* T_limit,
                              RcsStateType type)
{
  MatNd_reshape(T_limit, (type==RcsStateFull) ? self->dof : self->nJ, 1);

  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    if (type == RcsStateFull)
    {
      MatNd_set(T_limit, JNT->jointIndex, 0, JNT->maxTorque);
    }
    else if ((type == RcsStateIK) && (JNT->jacobiIndex != -1))
    {
      MatNd_set(T_limit, JNT->jacobiIndex, 0, JNT->maxTorque);
    }
  }
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsGraph_setDefaultState(RcsGraph* self)
{
  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    MatNd_set2(self->q, JNT->jointIndex, 0, JNT->q0);
  }

  MatNd_setZero(self->q_dot);
  RcsGraph_setState(self, NULL, self->q_dot);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool RcsGraph_checkJointTrajectory(const RcsGraph* self, const MatNd* q,
                                   double dt)
{
  bool success = true;

  if ((q->n!=self->dof) && (q->n!=self->nJ))
  {
    RLOG(1, "Dimension mismatch: Array has %d columns, dof is %d and nJ is %d",
         q->n, self->dof, self->nJ);
    return false;
  }

  RcsStateType sType = (q->n==self->dof) ? RcsStateFull: RcsStateIK;

  MatNd* q_lower = MatNd_create(q->n, 1);
  MatNd* q_upper = MatNd_create(q->n, 1);
  MatNd* q_dot_limit = MatNd_create(q->n, 1);

  RcsGraph_getJointLimits(self, q_lower, q_upper, sType);
  RcsGraph_getSpeedLimits(self, q_dot_limit, sType);

  for (unsigned int i=0; i<q->m; ++i)
  {
    MatNd q_i = MatNd_getRowViewTranspose(q, i);

    // Check joint limit violations
    for (unsigned int j=0; j<q->n; ++j)
    {
      if ((q_i.ele[j]<q_lower->ele[j]) || (q_i.ele[j]>q_upper->ele[j]))
      {
        REXEC(3)
        {
          RcsJoint* jnt = RcsGraph_getJointByIndex(self, j, sType);
          double scale = RcsJoint_isRotation(jnt) ? (180.0/M_PI) : 1.0;
          RLOG(3, "Joint %s (%d) out of range: %f outside [%f %f] %s",
               jnt ? jnt->name : "NULL", j, scale*q_i.ele[j],
               scale*q_lower->ele[j],scale* q_upper->ele[j],
               scale!=1.0 ? "deg" : "m");
        }
        success = false;
      }
    }


    // Check speed limit violations
    if (i > 0)
    {
      MatNd q_im1 = MatNd_getRowViewTranspose(q, i-1);

      for (unsigned int j=0; j<q->n; ++j)
      {
        double q_dot_i = (q_i.ele[j] - q_im1.ele[j])/dt;

        if (fabs(q_dot_i) > q_dot_limit->ele[j])
        {
          REXEC(3)
          {
            RcsJoint* jnt = RcsGraph_getJointByIndex(self, j, sType);
            double scale = RcsJoint_isRotation(jnt) ? (180.0/M_PI) : 1.0;
            RLOG(3, "Joint speed %s (%d) violated: %f > %f %s",
                 jnt ? jnt->name : "NULL", j, scale*q_dot_i,
                 scale*q_dot_limit->ele[j], scale!=1.0 ? "deg/s" : "m/s");
          }
          success = false;
        }
      }

    }

  }   // for (unsigned int i=0;i<q->m; ++i)

  MatNd_destroy(q_lower);
  MatNd_destroy(q_upper);
  MatNd_destroy(q_dot_limit);

  return success;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsGraph_changeDefaultState(RcsGraph* self, const MatNd* q0)
{
  RCHECK_EQ(q0->n, 1);

  if (q0->m == self->dof)
  {
    RCSGRAPH_TRAVERSE_JOINTS(self)
    {
      JNT->q0 = MatNd_get2(q0, JNT->jointIndex, 0);
    }
  }
  else if (q0->m == self->nJ)
  {
    RCSGRAPH_TRAVERSE_JOINTS(self)
    {
      JNT->q0 = MatNd_get2(q0, JNT->jacobiIndex, 0);
    }
  }
  else
  {
    RFATAL("Size mismatch: dof=%d   nJ=%d but array q0 has %d rows",
           self->dof, self->nJ, q0->m);
  }

}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsGraph_resetRigidBodyDofs(RcsGraph* self)
{
  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    if (BODY->rigid_body_joints)
    {
      RCSBODY_TRAVERSE_JOINTS(BODY)
      {
        MatNd_set(self->q, JNT->jointIndex, 0, JNT->q_init);
        MatNd_set(self->q_dot, JNT->jointIndex, 0, 0.0);
      }
    }
  }

  RcsGraph_setState(self, NULL, self->q_dot);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool RcsGraph_copyRigidBodyDofs(MatNd* q, const RcsGraph* self,
                                const MatNd* _src)
{
  const MatNd* src = _src != NULL ? _src : self->q;

  RCHECK((q->m==src->m) && (q->n==1));

  bool qChanged = false;

  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    if (BODY->rigid_body_joints == false)
    {
      continue;
    }

    double* q_dst = &q->ele[BODY->jnt->jointIndex];
    double* q_src = &src->ele[BODY->jnt->jointIndex];

    for (unsigned int i=0; i<6; i++)
    {
      if (q_dst[i] != q_src[i])
      {
        q_dst[i] = q_src[i];
        qChanged = true;
      }
    }

  }   // RCSGRAPH_TRAVERSE_BODIES(self)

  return qChanged;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool RcsGraph_limitJoints(const RcsGraph* self, MatNd* q, RcsStateType type)
{
  MatNd* _q = q ? q : self->q;
  RCHECK((_q->m==(type == RcsStateFull ? self->dof : self->nJ)) && (_q->n==1));
  bool violated = false;

  RLOG(5, "This function has changed: Please check if it is still Ok");

  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    // Only limit non-rigid body joints
    if (JNT->constrained == false &&
        (JNT->jacobiIndex > 0 || type == RcsStateFull))
    {
      int idx = (type == RcsStateFull) ? JNT->jointIndex : JNT->jacobiIndex;
      double qi = MatNd_get2(_q, idx, 0);

      if (qi < JNT->q_min)
      {
        MatNd_set2(_q, idx, 0, JNT->q_min);
        violated = true;
      }
      else if (qi > JNT->q_max)
      {
        MatNd_set2(_q, idx, 0, JNT->q_max);
        violated = true;
      }
    }
  }

  return violated;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
unsigned int RcsGraph_numJointLimitsViolated(const RcsGraph* self,
                                             bool verbose)
{
  unsigned int violations = 0;

  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    // We skip updating the matrix for constrained dofs
    if (JNT->constrained==true)
    {
      continue;
    }

    double qi = MatNd_get2(self->q, JNT->jointIndex, 0);

    if ((qi < JNT->q_min) || (qi > JNT->q_max))
    {
      if (verbose)
      {
        RMSG("Joint limit of \"%s\" violated: %f [%f %f]",
             JNT->name, qi, JNT->q_min, JNT->q_max);
      }
      violations++;
    }
  }

  return violations;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsGraph_getInvWq(const RcsGraph* self, MatNd* invWq, RcsStateType type)
{
  if (type == RcsStateFull)
  {
    MatNd_reshape(invWq, self->dof, 1);

    RCSGRAPH_TRAVERSE_JOINTS(self)
    {
      MatNd_set2(invWq, JNT->jointIndex, 0,
                 JNT->weightMetric * (JNT->q_max - JNT->q_min));
    }
  }
  else   // type == RcsStateIK
  {
    MatNd_reshape(invWq, self->nJ, 1);

    RCSGRAPH_TRAVERSE_JOINTS(self)
    {
      if ((JNT->jacobiIndex != -1))
        MatNd_set2(invWq, JNT->jacobiIndex, 0,
                   JNT->weightMetric * (JNT->q_max - JNT->q_min));
    }
  }
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
RcsBody* RcsGraph_getBodyByName(const RcsGraph* self, const char* name)
{
  if ((name==NULL) || (self==NULL))
  {
    return NULL;
  }

  if (STRNEQ(name, "GenericBody", 11))
  {
    if (strlen(name) != 12)
    {
      RLOG(1, "GenericBody \"%s\": suffix must be [0...9]", name);
      return NULL;
    }

    int num = atoi(&name[11]);

    if ((num < 0) || (num >= 10))
    {
      RLOG(1, "GenericBody \"%s\": suffix must be [0...9]", name);
      return NULL;
    }

    return (RcsBody*) &self->gBody[num];
  }

  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    if (STREQ(name, BODY->name))
    {
      return BODY;
    }
  }

  return NULL;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
RcsBody* RcsGraph_getBodyByTruncatedName(const RcsGraph* self, const char* name)
{
  if ((name==NULL) || (self==NULL))
  {
    return NULL;
  }

  if (STRNEQ(name, "GenericBody", 11))
  {
    if (strlen(name) != 12)
    {
      RLOG(1, "GenericBody \"%s\": suffix must be [0...9]", name);
      return NULL;
    }

    int num = atoi(&name[11]);

    if ((num < 0) || (num >= 10))
    {
      RLOG(1, "GenericBody \"%s\": suffix must be [0...9]", name);
      return NULL;
    }

    return (RcsBody*) &self->gBody[num];
  }

  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    if (STRNEQ(name, BODY->name, strlen(name)))
    {
      return BODY;
    }
  }

  return NULL;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
double RcsGraph_getJointValue(const RcsGraph* self, const char* name)
{
  RcsJoint* jnt = RcsGraph_getJointByName(self, name);
  RCHECK(jnt);
  return MatNd_get2(self->q, jnt->jointIndex, 0);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
RcsJoint* RcsGraph_getJointByName(const RcsGraph* self, const char* name)
{
  if ((name==NULL) || (self==NULL))
  {
    return NULL;
  }

  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    if (STREQ(name, JNT->name))
    {
      return JNT;
    }
  }
  return NULL;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
RcsJoint* RcsGraph_getJointByIndex(const RcsGraph* self, unsigned int idx,
                                   RcsStateType type)
{
  if (self==NULL)
  {
    return NULL;
  }

  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    int jointsIdx = (type==RcsStateFull) ? JNT->jointIndex : JNT->jacobiIndex;

    if (idx == jointsIdx)
    {
      return JNT;
    }
  }
  return NULL;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
RcsJoint* RcsGraph_getJointByTruncatedName(const RcsGraph* self,
                                           const char* name)
{
  if ((name==NULL) || (self==NULL))
  {
    return NULL;
  }

  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    if (STRNEQ(name, JNT->name, strlen(name)))
    {
      return JNT;
    }
  }
  return NULL;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsGraph_fprint(FILE* out, const RcsGraph* self)
{
  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    RcsBody_fprint(out, BODY);
  }

  int numSensors = 0;
  RCSGRAPH_TRAVERSE_SENSORS(self)
  {
    fprintf(out, "Sensor %d:\n", numSensors++);
    RcsSensor_fprint(out, SENSOR);
  }

}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsGraph_fprintJoints(FILE* out, const RcsGraph* self)
{
  fprintf(out, "[%s: %s(%d)]:\n", __FILE__, __FUNCTION__, __LINE__);

  RCSGRAPH_TRAVERSE_JOINTS(self)
  {

    fprintf(out, "Joint[%d] = %s   with type %s (ctrlType=%d)",
            JNT->jointIndex, JNT->name, RcsJoint_typeName(JNT->type),
            JNT->ctrlType);

    if (JNT->constrained)
    {
      fprintf(out, "\tJoint is constrained\n");
    }
    else
    {
      fprintf(out, "\tJoint is active (unconstrained)\n");
    }

  }
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsGraph_fprintJointRecursion(FILE* out,
                                   const RcsGraph* self,
                                   const char* bdyName)
{
  RcsBody* b = RcsGraph_getBodyByName(self, bdyName);

  if (!b)
  {
    RLOG(4, "Body \"%s\" not found - skipping backward recursion", bdyName);
    return;
  }

  RMSG("Backward pass starting from body \"%s\"", b->name);

  RcsJoint* jnt = b->jnt;
  RCHECK(jnt);

  while (jnt->next)
  {
    jnt = jnt->next;
  }


  while (jnt->prev)
  {
    RMSG("Joint %s", jnt->name);
    jnt = jnt->prev;
  }

  RMSG("Joint %s", jnt->name);
  RMSG("Backward pass finished");
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool RcsGraph_setJoint(RcsGraph* self, const char* jntName, double val)
{
  RcsJoint* j = RcsGraph_getJointByName(self, jntName);

  if (j == NULL)
  {
    RLOG(1, "Can't set joint angle of non-existing joint \"%s\"", jntName);
    return false;
  }

  MatNd_set2(self->q, j->jointIndex, 0, val);

  return true;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsGraph_printState(const RcsGraph* self, const MatNd* q)
{
  RcsStateType stateType;

  if (q->m == self->dof)
  {
    stateType = RcsStateFull;
  }
  else if (q->m == self->nJ)
  {
    stateType = RcsStateIK;
  }
  else
  {
    RLOG(4, "Warning: q->m=%d is neither dof (%d) nor nJ (%d)",
         q->m, self->dof, self->nJ);
    MatNd_print(q);
    return;
  }

  if (q->n != 1)
  {
    RMSG("Warning: q->n=%d but should be 1", q->n);
    MatNd_print(q);
    return;
  }

  fprintf(stderr, "[%s: %s(%d)]: %s state vector is\n",
          __FILE__, __FUNCTION__, __LINE__,
          (stateType==RcsStateIK) ? "IK" : "Full");

  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    // Skip constrained joints if state to be printed is of type IK
    if ((stateType==RcsStateIK) && (JNT->jacobiIndex==-1))
    {
      continue;
    }

    int index = (stateType==RcsStateIK) ? JNT->jacobiIndex : JNT->jointIndex;

    if (RcsJoint_isRotation(JNT))
    {
      fprintf(stderr, "%d\t%5.14f [deg]\t(%s)\n", index,
              q->ele[index] * (180. / M_PI), JNT->name);
    }
    else
    {
      fprintf(stderr, "%d\t%5.16f [mm]\t(%s)\n", index,
              q->ele[index] * 1000.0, JNT->name);
    }
  }

}

/*******************************************************************************
 * This is the xml format:
 *
 * <model_state model="Name of the model" time_stamp="0">
 *   <joint_state joint="jnt1" position="30" />
 *   <joint_state joint="jnt2" position="20" />
 * </model_state>
 ******************************************************************************/
void RcsGraph_fprintModelState(FILE* out, const RcsGraph* self, const MatNd* q)
{
  RcsStateType stateType;

  if (q->m == self->dof)
  {
    stateType = RcsStateFull;
  }
  else if (q->m == self->nJ)
  {
    stateType = RcsStateIK;
  }
  else
  {
    fprintf(out, "Warning: q->m=%d is neither dof (%d) nor nJ (%d)",
            q->m, self->dof, self->nJ);
    return;
  }

  if (q->n != 1)
  {
    fprintf(out, "Warning: q->n=%d but should be 1", q->n);
    return;
  }


  fprintf(out, "<model_state model=\"\" time_stamp=\"\">\n");



  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    // Skip constrained joints if state to be printed is of type IK
    if ((stateType==RcsStateIK) && (JNT->jacobiIndex==-1))
    {
      continue;
    }

    int index = (stateType==RcsStateIK) ? JNT->jacobiIndex : JNT->jointIndex;

    if (RcsJoint_isRotation(JNT))
    {
      fprintf(out, "  <joint_state joint=\"%s\" position=\"%.3f\" />\n",
              JNT->name, RCS_RAD2DEG(q->ele[index]));
    }
    else
    {
      fprintf(out, "  <joint_state joint=\"%s\" position=\"%.3f\" />\n",
              JNT->name, q->ele[index]);
    }
  }





  fprintf(out, "</model_state>\n");
}

/*******************************************************************************
 * Returns the index of the body from the root body. If the body is NULL or
 * doesn't exist, -1 is returned. The index is used as a unique id for the dot
 * file output.
 ******************************************************************************/
static int RcsBody_getIndex(const RcsGraph* self, const RcsBody* body)
{
  if (!body)
  {
    return -1;
  }

  int index = 0;

  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    if (STREQ(BODY->name, body->name))
    {
      NLOG(0, "Body \"%s\" has index %d", body->name, index);
      return index;
    }
    index++;
  }

  NLOG(0, "Body \"%s\" not found in graph!", body->name);
  return -1;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsGraph_writeDotFile(const RcsGraph* self, const char* filename)
{
  int i = 0, i_prev;
  RCHECK(filename);
  FILE* fd = fopen(filename, "w+");
  RCHECK_MSG(fd, "Couldn't open file \"%s\"", filename);

  // File header
  fprintf(fd, "digraph G {\nbgcolor=\"white\"\n");
  fprintf(fd, "graph [fontname=\"fixed\"];\n");
  fprintf(fd, "node [fontname=\"fixed\"];\n");
  fprintf(fd, "edge [fontname=\"fixed\"];\n");

  // Body labels
  fprintf(fd, "-1[label=\"Root\" style=filled color=\"0.7 0.3 1\"];\n");
  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    fprintf(fd, "%d[label=\"%s\" ", i++, BODY->name);
    fprintf(fd, "shape=box style=filled color=\"0.7 0.3 1\"];\n");
  }

  // Joint labels
  i = 0;   // Running body index
  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    i_prev = RcsBody_getIndex(self, BODY->parent); // Determine preceeding body

    if (BODY->jnt)    // Body is connected to predecessor by joints
    {
      fprintf(fd, "%d->%d [label=\"", i_prev, i);
      RCSBODY_TRAVERSE_JOINTS(BODY)
      {
        fprintf(fd, "%s%s \\n",
                JNT->name, JNT->constrained ? " (constrained)" : "");
      }
      fprintf(fd, "\"];\n");
    }
    else     // Body is fixed to predecessor by fixed transformation
    {
      fprintf(fd, "%d->%d [label=\"Fixed\" ", i_prev, i);
      fprintf(fd, "color=\"red\" fontcolor=\"red\"];\n");
    }

    i++;
  }

  // File end
  fprintf(fd, "}\n");
  fclose(fd);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsGraph_writeDotFileDfsTraversal(const RcsGraph* self,
                                       const char* filename)
{
  int i = 0;
  RCHECK(filename);
  FILE* fd = fopen(filename, "w+");
  RCHECK_MSG(fd, "Couldn't open file \"%s\"", filename);

  // File header
  fprintf(fd, "digraph G {\nbgcolor=\"white\"\n");
  fprintf(fd, "graph [fontname=\"fixed\"];\n");
  fprintf(fd, "node [fontname=\"fixed\"];\n");
  fprintf(fd, "edge [fontname=\"fixed\"];\n");

  // Body labels
  fprintf(fd, "-1[label=\"Root\" style=filled color=\"0.7 0.3 1\"];\n");
  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    fprintf(fd, "%d[label=\"%s\" ", i++, BODY->name);
    fprintf(fd, "shape=box style=filled color=\"0.7 0.3 1\"];\n");
  }

  // Joint labels
  i = 0;   // Running body index
  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    int i_parent = RcsBody_getIndex(self, BODY->parent);

    if (i_parent != -1)
    {
      fprintf(fd, "%d->%d [label=\"Parent: %s \\n",
              i, i_parent, BODY->parent->name);
      fprintf(fd, "\"];\n");
    }
    else
    {
      fprintf(fd, "%d->-1 [label=\"Parent: Root \\n", i);
      fprintf(fd, "\"];\n");
    }

    int i_child = RcsBody_getIndex(self, BODY->firstChild);

    if (i_child != -1)
    {
      fprintf(fd, "%d->%d [label=\"Child: %s \\n",
              i, i_child, BODY->firstChild->name);
      fprintf(fd, "\"];\n");
    }

    int i_prev = RcsBody_getIndex(self, BODY->prev);

    if (i_prev != -1)
    {
      fprintf(fd, "%d->%d [label=\"Prev: %s \\n",
              i, i_prev, BODY->prev->name);
      fprintf(fd, "\"];\n");
    }

    int i_next = RcsBody_getIndex(self, BODY->next);

    if (i_next != -1)
    {
      fprintf(fd, "%d->%d [label=\"Next: %s \\n",
              i, i_next, BODY->next->name);
      fprintf(fd, "\"];\n");
    }

    int i_last = RcsBody_getIndex(self, BODY->lastChild);

    if (i_last != -1)
    {
      fprintf(fd, "%d->%d [label=\"Last: %s \\n",
              i, i_last, BODY->lastChild->name);
      fprintf(fd, "\"];\n");
    }

    i++;
  }

  // File end
  fprintf(fd, "}\n");
  fclose(fd);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
unsigned int RcsGraph_numBodies(const RcsGraph* self)
{
  unsigned int n = 0;
  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    n++;
  }
  return n;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsGraph_bodyTorque(const RcsGraph* self, const RcsBody* body,
                         MatNd* torque)
{
  if (body->m == 0.0)
  {
    return;
  }

  MatNd* J_cog = NULL;
  MatNd_create2(J_cog, 3, self->nJ);
  MatNd* force = NULL;
  MatNd_create2(force, 3, 1);
  MatNd* bodyTorque = NULL;
  MatNd_create2(bodyTorque, self->nJ, 1);

  RcsGraph_bodyPointJacobian(self, body, body->Inertia->org, NULL, J_cog);
  MatNd_transposeSelf(J_cog);
  MatNd_set2(force, 2, 0, -body->m * RCS_GRAVITY);
  MatNd_mul(bodyTorque, J_cog, force);
  MatNd_addSelf(torque, bodyTorque);

  MatNd_destroy(J_cog);
  MatNd_destroy(force);
  MatNd_destroy(bodyTorque);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
double RcsGraph_limitJointSpeeds(const RcsGraph* self, MatNd* dq, double dt,
                                 RcsStateType type)
{
  int dimension = (type == RcsStateFull) ? self->dof : self->nJ;
  RCHECK_MSG(dq->m == dimension, "dq->m is %d     dimension is %d"
             "     graph->dof is %d     graph->nJ is %d     type is %d",
             dq->m , dimension, self->dof, self->nJ, type);
  RCHECK(dq->n == 1);

  int theEvilIndex = 0;
  double sc_i = 1.0, sc = 1.0;
  RcsJoint* theEvilJoint = NULL;

  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    if ((JNT->jacobiIndex == -1) && (type == RcsStateIK))
    {
      continue;
    }

    int index = (type == RcsStateIK) ? JNT->jacobiIndex : JNT->jointIndex;

    // Get speed limits in units/sec
    double q_dot_max = JNT->speedLimit;
    double q_dot     = fabs(dq->ele[index] / dt);
    NLOG(0, "Comparing joint %s: q_dot = %f   q_dot_max = %f",
         JNT->name, q_dot, q_dot_max);
    if (q_dot > q_dot_max)
    {
      sc_i = q_dot_max / q_dot;
    }
    if (sc_i < sc)
    {
      sc = sc_i;
      theEvilJoint = JNT;
      theEvilIndex = index;
    }
  }

  REXEC(6)
  {
    if ((sc<1.0) && (theEvilJoint!=NULL))
    {
      double sLim = theEvilJoint->speedLimit;
      RMSG("Scaling speeds with %5.5f (due to joint \"%s\": speed=%5.6f   "
           "limit=%5.6f)", sc, theEvilJoint->name,
           fabs(dq->ele[theEvilIndex] / dt), sLim);
    }
  }

  // Scale it down
  MatNd_constMulSelf(dq, sc);

  return sc;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
int RcsGraph_check(const RcsGraph* self)
{
  int nErrors  = 0;

  if (self==NULL)
  {
    nErrors++;
    RLOG(1, "Graph is NULL");
    return nErrors;
  }


  // Check for correctness of rigid body joints
  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    if (BODY->rigid_body_joints)
    {
      // Test if rigid body has 6 joints
      if (RcsBody_numJoints(BODY) != 6)
      {
        nErrors++;
        RLOG(1, "Body \"%s\" is a rigid body joint, but has %d joints "
             "(instead of 6)", BODY->name, RcsBody_numJoints(BODY));
        continue;
      }

      // Test if joints are in order transX-transY-transZ-rotX-rotY-rotZ
      RcsJoint* jnt = BODY->jnt;
      int desiredOrder[6] = { RCSJOINT_TRANS_X,
                              RCSJOINT_TRANS_Y,
                              RCSJOINT_TRANS_Z,
                              RCSJOINT_ROT_X,
                              RCSJOINT_ROT_Y,
                              RCSJOINT_ROT_Z
                            };

      for (int i=0; i<6; i++)
      {
        if (jnt->type != desiredOrder[i])
        {
          nErrors++;
          RLOG(1, "Joint %d of rigid body \"%s\" is of type \"%s\", "
               "but should be \"%s\"", i, BODY->name,
               RcsJoint_typeName(jnt->type),
               RcsJoint_typeName(desiredOrder[i]));
        }

        // Test if the relative transformations between the rigid body joints
        // are identity transformations. We skip the first joint, since this
        // sometimes inherits a transform when included from a group.
        if (i>0)
        {
          if (jnt->A_JP != NULL)
          {
            if (HTr_isIdentity(jnt->A_JP) == false)
            {
              nErrors++;
              REXEC(1)
              {
                RMSG("Joint %d of rigid body \"%s\" has non-identity "
                     "relative transform:", i, BODY->name);
                HTr_fprint(stderr, jnt->A_JP);
              }
            }
          }
        }

        jnt = jnt->next;
      }

    }

  }

  // Check for duplicate body names
  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    int n = 0;
    RcsBody* b = BODY;
    RCSGRAPH_TRAVERSE_BODIES(self)
    {
      if (STREQ(b->name, BODY->name))
      {
        n++;
      }
    }
    if (n > 1)
    {
      nErrors++;
      RLOG(1, "Body name \"%s\" found %d times", b->name, n);
    }
  }

  // Check for correct mass properties
  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    if (BODY->m<0.0)
    {
      nErrors++;
      RLOG(1, "Body \"%s\" has negative mass: %g", BODY->name, BODY->m);
    }

    // Check if we have a finite inertia but no mass
    if ((Mat3d_getFrobeniusnorm(BODY->Inertia->rot)>0.0) && (BODY->m<=0.0))
    {
      nErrors++;
      RLOG(1, "Body \"%s\" has positive inertia zero mass", BODY->name);
    }
  }

  // Check for joint centers out of range and for valid indices of the
  // joint array pointers
  RCSGRAPH_TRAVERSE_JOINTS(self)
  {

    if ((JNT->q0<JNT->q_min) && (JNT->coupledTo==NULL))
    {
      nErrors++;
      double s = RcsJoint_isRotation(JNT) ? 180.0/M_PI : 1.0;
      RLOG(1, "Joint \"%s\": q0 < q_min (q_min=%f   q0=%f   q_max=%f [%s])",
           JNT->name, s*JNT->q_min, s*JNT->q0, s*JNT->q_max,
           RcsJoint_isRotation(JNT) ? "deg" : "m");
    }

    if ((JNT->q0>JNT->q_max) && (JNT->coupledTo==NULL))
    {
      nErrors++;
      double s = RcsJoint_isRotation(JNT) ? 180.0/M_PI : 1.0;
      RLOG(1, "Joint \"%s\": q0 > q_max (q_min=%f   q0=%f   q_max=%f [%s])",
           JNT->name, s*JNT->q_min, s*JNT->q0, s*JNT->q_max,
           RcsJoint_isRotation(JNT) ? "deg" : "m");
    }

    if ((JNT->jointIndex < 0) || (JNT->jointIndex >= (int) self->dof))
    {
      nErrors++;
      RLOG(1, "Joint \"%s\": joint index is %d", JNT->name, JNT->jointIndex);
    }

    if ((JNT->jacobiIndex < -1) || (JNT->jacobiIndex >= (int) self->dof))
    {
      nErrors++;
      RLOG(1, "Joint \"%s\": Jacobi index is %d, should be [-1 ... %d]",
           JNT->name, JNT->jacobiIndex, self->dof);
    }

    // Check if coupled joint factors match
    if (JNT->couplingFactors != NULL)
    {
      if (JNT->coupledTo == NULL)
      {
        nErrors++;
        RLOG(1, "Joint \"%s\": Coupled joint is NULL, but coupling factors"
             " are defined", JNT->name);
      }

      if ((JNT->couplingFactors->size!=1) &&
          (JNT->couplingFactors->size!=5) &&
          (JNT->couplingFactors->size!=9))
      {
        nErrors++;
        RLOG(1, "Incorrect number of coupling factors in joint \"%s\": %d",
             JNT->name, JNT->couplingFactors->size);
      }

    }

    if (JNT->speedLimit < 0.0)
    {
      nErrors++;
      RLOG(1, "Joint \"%s\" has negative speed limit: %f",
           JNT->name, JNT->speedLimit);
    }

  }

  // Check for body connection consistencs
  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    if ((BODY->firstChild!=NULL) && (BODY->firstChild->prev!=NULL))
    {
      RLOG(1, "Body \"%s\" has firstChild (\"%s\") with prev body (\"%s\")",
           BODY->name, BODY->firstChild->name, BODY->firstChild->prev->name);
      nErrors++;
    }

    if ((BODY->lastChild != NULL) && (BODY->lastChild->next != NULL))
    {
      RLOG(1, "Body \"%s\" has lastChild (\"%s\") with next body (\"%s\")",
           BODY->name, BODY->lastChild->name, BODY->lastChild->next->name);
      nErrors++;
    }

    if ((BODY->firstChild == NULL) && (BODY->lastChild != NULL))
    {
      RLOG(1, "Body \"%s\" has no firstChild but lastChild (\"%s\")",
           BODY->name, BODY->lastChild->name);
      nErrors++;
    }
  }

  RLOG(6, "Graph check : %d errors", nErrors);

  return nErrors;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
RcsGraph* RcsGraph_clone(const RcsGraph* src)
{
  if (src==NULL)
  {
    return NULL;
  }

  RcsGraph* dst = RALLOC(RcsGraph);
  RCHECK(dst);

  // Copy the full memory block
  memmove(dst, src, sizeof(RcsGraph));

  // Adjust pointers to local data
  dst->q       = MatNd_clone(src->q);
  dst->q_dot      = MatNd_clone(src->q_dot);
  dst->xmlFile = String_clone(src->xmlFile);

  // Traverse the graph and create all elements
  dst->root = NULL;

  RCSGRAPH_TRAVERSE_BODIES(src)
  {
    // Make a copy of the body
    RcsBody* b = RALLOC(RcsBody);
    RcsBody_copy(b, BODY);

    // to which body does the new one needs to be attached?
    RcsBody* parent = NULL;

    if (BODY->parent != NULL)
    {
      parent = RcsGraph_getBodyByName(dst, BODY->parent->name);
    }

    // insert body into graph, the resulting graph structure will be the same
    // as in src
    RcsGraph_insertBody(dst, parent, b);

    // Make a copy of all body joints and attach them to the body

    // Determine the previous joint to which the body is connected. That's
    // the relevant one for the backward chain connectivity
    RcsJoint* prevJoint = RcsBody_lastJointBeforeBody(b->parent);

    if (prevJoint != NULL)
    {
      prevJoint->next = NULL;
    }

    int nBdyJnts = 0;

    RCSBODY_TRAVERSE_JOINTS(BODY)
    {
      RcsJoint* j = RALLOC(RcsJoint);
      RcsJoint_copy(j, JNT);

      // Connect forward chain.
      if ((nBdyJnts!=0) && (prevJoint!=NULL))
      {
        prevJoint->next = j;
      }

      if (nBdyJnts == 0)
      {
        b->jnt = j;
      }

      // Connect backward chain.
      j->prev = prevJoint;
      prevJoint = j;
      nBdyJnts++;
      RLOG(6, "Copying joint %s of body %s - next is %s",
           JNT->name, BODY->name,
           JNT->next ? JNT->next->name : "NULL");

    }   // RCSBODY_TRAVERSE_JOINTS(BODY)

    // Make a copy of all body shapes and attach them to the body
    int nShapes = RcsBody_numShapes(BODY);
    b->shape = RNALLOC(nShapes + 1, RcsShape*);
    for (int i = 0; i < nShapes; i++)
    {
      b->shape[i] = RALLOC(RcsShape);
      RcsShape_copy(b->shape[i], BODY->shape[i]);
    }

  }   // RCSGRAPH_TRAVERSE_BODIES(src)


  // Initialize generic bodies. Here we allocate memory for names
  // and body transforms. They are deleted once relinked to another
  // body.
  for (int i = 0; i < 10; i++)
  {
    RcsBody* gSrc = (RcsBody*) src->gBody[i].extraInfo;

    memset(&dst->gBody[i], 0, sizeof(RcsBody));
    dst->gBody[i].name      = RNALLOC(64, char);
    dst->gBody[i].xmlName   = RNALLOC(64, char);
    dst->gBody[i].suffix    = RNALLOC(64, char);

    if (gSrc != NULL)
    {
      dst->gBody[i].A_BI      = HTr_create();
      dst->gBody[i].A_BP      = HTr_create();
      dst->gBody[i].Inertia   = HTr_create();
      HTr_setZero(dst->gBody[i].Inertia);
    }

    sprintf(dst->gBody[i].name, "GenericBody%d", i);

    RcsGraph_linkGenericBody(dst, i, gSrc ? gSrc->name : "");
  }

  // now, also create the sensors
  dst->sensor = NULL;

  RCSGRAPH_TRAVERSE_SENSORS(src)
  {
    RcsSensor* sensor = RcsSensor_clone(SENSOR, dst);
    RcsGraph_addSensor(dst, sensor);
  }

  // Handle coupled joints
  RCSGRAPH_TRAVERSE_JOINTS(dst)
  {
    if ((JNT->coupledJointName!=NULL) && (JNT->couplingFactors!=NULL))
    {
      JNT->coupledTo = RcsGraph_getJointByName(dst, JNT->coupledJointName);
      RCHECK_MSG(JNT->coupledTo, "Missing coupled joint \"%s\"",
                 JNT->coupledJointName);
    }
  }


  REXEC(4)
  {
    // Check for consistency
    RCHECK_MSG(RcsGraph_check(dst) == 0,
               "Consistency check for graph \"%s\" failed",
               dst->xmlFile ? dst->xmlFile : "NULL");
  }

  return dst;
}

/*******************************************************************************
 * See header. \todo: Connectivity
 ******************************************************************************/
void RcsGraph_copy(RcsGraph* dst, const RcsGraph* src)
{
  RcsBody* dstPtr = dst->root;
  const RcsBody* srcPtr = src->root;

  while (dstPtr != NULL)
  {
    // Copy body
    RCHECK_MSG(srcPtr, "Body to copy from is NULL (should be \"%s\")",
               dstPtr->name);
    RCHECK_MSG(STREQ(dstPtr->name, srcPtr->name),
               "Bodies dst and src differ: %s != %s",
               dstPtr->name, srcPtr->name);
    RcsBody_copy(dstPtr, srcPtr);

    // Copy all joints of the body
    RcsJoint* dstJntPtr = dstPtr->jnt;
    const RcsJoint* srcJntPtr = srcPtr->jnt;

    while (dstJntPtr != NULL)
    {
      RCHECK_MSG(srcJntPtr, "Joint to copy from is NULL (should be \"%s\")",
                 dstJntPtr->name);
      RCHECK_MSG(STREQ(dstJntPtr->name, srcJntPtr->name),
                 "Joints dst and src differ: %s != %s",
                 dstJntPtr->name, srcJntPtr->name);
      RcsJoint_copy(dstJntPtr, srcJntPtr);

      // Move on to next joint
      dstJntPtr = dstJntPtr->next;
      srcJntPtr = srcJntPtr->next;
    }

    // Copy all shapes of the body
    RcsShape** dstShapePtr = dstPtr->shape;
    RcsShape** srcShapePtr = srcPtr->shape;

    while (*dstShapePtr != NULL)
    {
      RCHECK_MSG(*srcShapePtr, "Shape to copy from is NULL (should be \"%s\")",
                 RcsShape_name((*dstShapePtr)->type));
      RcsShape_copy(*dstShapePtr, *srcShapePtr);
      srcShapePtr++;
      dstShapePtr++;
    }

    // Move on to next body
    dstPtr = RcsBody_depthFirstTraversalGetNext(dstPtr);
    srcPtr = RcsBody_depthFirstTraversalGetNext(srcPtr);
  }

  dst->dof = src->dof;
  dst->nJ = src->nJ;
  String_copyOrRecreate(&dst->xmlFile, src->xmlFile);
  MatNd_resizeCopy(&dst->q, src->q);
  MatNd_resizeCopy(&dst->q_dot, src->q_dot);



  RcsSensor* dstSensorPtr = dst->sensor;
  const RcsSensor* srcSensorPtr = src->sensor;

  while (dstSensorPtr != NULL)
  {
    RcsSensor_copy(dstSensorPtr, srcSensorPtr);
    dstSensorPtr = dstSensorPtr->next;
    srcSensorPtr = srcSensorPtr->next;
  }

  // GenericBodies
  for (int i=0; i<10; ++i)
  {
    RcsBody_copy(&dst->gBody[i], &src->gBody[i]);

    if (src->gBody[i].extraInfo != NULL)
    {
      const RcsBody* linkedBdySrc = (const RcsBody*) src->gBody[i].extraInfo;
      const RcsBody* linkedBdyDst = RcsGraph_linkGenericBody(dst, i, linkedBdySrc->name);
      RCHECK_MSG(linkedBdyDst, "Couldn't find linked body \"%s\" in graph",
                 linkedBdySrc->name);
    }
  }
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
RcsBody* RcsGraph_linkGenericBody(RcsGraph* self, int bdyNum,
                                  const char* bdyName)
{
  RCHECK(bdyNum >= 0 && bdyNum < 10);

  // If the generic body has to point to NULL, we remove all pointers from the
  // body it has previously pointed to and recreate the elements on the heap.
  // Now they belong to the generic body.
  RcsBody* b = RcsGraph_getBodyByName(self, bdyName);

  if (b == NULL)
  {
    NLOG(1, "Unlinking generic body %d", bdyNum);

    // If the body is already connected to NULL, nothing remains to do.
    if (self->gBody[bdyNum].extraInfo == NULL)
    {
      NLOG(1, "Generic body is already NULL - doing nothing");
      return NULL;
    }

    // If the body was already connected to another body, we redirect its
    // extraInfo to NULL and recreate its own memory for the transformations.
    self->gBody[bdyNum].extraInfo  = NULL;
    self->gBody[bdyNum].A_BI       = HTr_create();
    self->gBody[bdyNum].A_BP       = HTr_create();
    self->gBody[bdyNum].Inertia    = HTr_create();
    HTr_setZero(self->gBody[bdyNum].Inertia);
    self->gBody[bdyNum].parent     = NULL;
    self->gBody[bdyNum].firstChild = NULL;
    self->gBody[bdyNum].lastChild  = NULL;
    self->gBody[bdyNum].next       = NULL;
    self->gBody[bdyNum].prev       = NULL;
    return NULL;
  }


  // From here on, b points to a body of self.
  NLOG(1, "Linking generic body %d to \"%s\"", bdyNum, b->name);


  // If the body is to be linked against itself, we do nothing. We also check
  // if it is to be linked against another GenericBody. In this case, we do
  // nothing and return NULL, since the side effects might be significant.
  if (STRNEQ(b->name, "GenericBody", 11))
  {
    RLOG(1, "Generic body points to itself or another generic body - doing "
         "nothing");
    return NULL;
  }


  // From here on, RcsBody b points to a body within the graph which does
  // not belong to the generic bodies.

  // If the generic body's extraInfo field  previously pointed to NULL, it
  // had its own memory and we need to delete it here. Otherwise, we just
  // redirect the pointers.
  if (self->gBody[bdyNum].extraInfo == NULL)
  {
    RFREE(self->gBody[bdyNum].A_BI);
    RFREE(self->gBody[bdyNum].A_BP);
    RFREE(self->gBody[bdyNum].Inertia);
  }

  self->gBody[bdyNum].m                 = b->m;
  self->gBody[bdyNum].rigid_body_joints = b->rigid_body_joints;
  self->gBody[bdyNum].physicsSim        = b->physicsSim;
  Vec3d_setZero(self->gBody[bdyNum].x_dot);
  Vec3d_setZero(self->gBody[bdyNum].omega);
  self->gBody[bdyNum].A_BP              = b->A_BP;
  self->gBody[bdyNum].A_BI              = b->A_BI;
  self->gBody[bdyNum].Inertia           = b->Inertia;
  self->gBody[bdyNum].parent            = b->parent;
  self->gBody[bdyNum].firstChild        = b->firstChild;
  self->gBody[bdyNum].lastChild         = b->lastChild;
  self->gBody[bdyNum].next              = b->next;
  self->gBody[bdyNum].prev              = b->prev;
  self->gBody[bdyNum].shape             = b->shape;
  self->gBody[bdyNum].jnt               = b->jnt;
  self->gBody[bdyNum].extraInfo         = b;

  return b;
}

/*******************************************************************************
 * See header. \todo: can be done more efficient
 ******************************************************************************/
RcsBody* RcsGraph_getGenericBodyPtr(const RcsGraph* self, int bdyNum)
{
  char a[32];
  sprintf(a, "GenericBody%d", bdyNum);
  RcsBody* b = RcsGraph_getBodyByName(self, a);

  if (b == NULL)
  {
    return NULL;
  }

  return (RcsBody*) b->extraInfo;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsGraph_insertBody(RcsGraph* graph, RcsBody* parent, RcsBody* body)
{
  if (body == NULL)
  {
    return;
  }

  // TODO for now only bodies without joints are supported
  RCHECK_MSG(RcsBody_numJoints(body) == 0, "for now only bodies without "
             "joints are supported");

  // TODO for now only bodies without children or sibbling are supported
  RCHECK_MSG(body->prev == NULL, "for now only bodies without sibbling are "
             "supported - your body %s has a \"prev\" sibling: %s",
             body->name, body->prev->name);
  RCHECK_MSG(body->next == NULL, "for now only bodies without sibbling are "
             "supported - your body %s has a \"next\" sibling: %s",
             body->name, body->next->name);
  RCHECK_MSG(body->firstChild == NULL, "for now only bodies without children "
             "are supported - your body %s has a \"child\": %s",
             body->name, body->firstChild->name);
  RCHECK_MSG(body->lastChild == NULL, "for now only bodies without children "
             "are supported - your body %s has a \"lastChild\": %s",
             body->name, body->lastChild->name);

  body->parent = parent;

  if (parent != NULL)
  {
    if (parent->lastChild != NULL)  // parent already has children
    {
      parent->lastChild->next = body;
      body->prev = parent->lastChild;
    }
    else
    {
      parent->firstChild = body;
    }

    parent->lastChild = body;
  }
  else
  {
    if (graph->root != NULL)
    {
      RcsBody* b = graph->root;
      while (b->next)
      {
        b = b->next;
      }

      b->next = body;
      body->prev = b;
    }
    else
    {
      graph->root = body;
    }
  }

  RLOG(9, "Inserted Body into Graph: name=%s parent=%s prev=%s",
       body->name,
       body->parent ? body->parent->name : "NULL",
       body->prev ? body->prev->name : "NULL");
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsGraph_insertJoint(RcsGraph* graph, RcsBody* body, RcsJoint* jnt)
{
  graph->dof++;

  if ((graph->q==NULL)|| (graph->q->m<graph->dof))
  {
    NLOG(9, "Resizing q to %d elements", graph->dof);
    graph->q = MatNd_realloc(graph->q, graph->dof, 1);
    RCHECK(graph->q);
  }

  jnt->jointIndex = graph->dof - 1;

  // Find last joint of body

  // The joint is the first joint of body
  if (body->jnt == NULL)
  {
    body->jnt = jnt;
    NLOG(9, "Added first joint \"%s\" to body \"%s\"", jnt->name, body->name);

    // Find last joint of previous body
    if (body->parent != NULL)    // The body has a predecessor
    {
      RcsJoint* prevJnt = RcsBody_lastJointBeforeBody(body->parent);

      body->jnt->prev = prevJnt;
    }
    else    // The body has no predecessor
    {
      body->jnt->prev = NULL;
      NLOG(9, "First joint: Connected joint \"%s\" to the world",
           body->jnt->name);
    }
  }
  // If the joint is not the first joint of the body, we insert it between the
  // current last joint and the body.
  else   // if(body->jnt==NULL)
  {
    RcsJoint* prevJnt = body->jnt;

    while (prevJnt->next != NULL)
    {
      // Append after last joint
      prevJnt = prevJnt->next;
    }
    prevJnt->next = jnt;
    jnt->prev = prevJnt;
  }  // End of joint being not the first joint of body

  NLOG(9, "Joint with jointIndex %d inserted", jnt->jointIndex);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsGraph_relativeRigidBodyDoFs(const RcsBody* body,
                                    const HTr* new_A_BI_body,
                                    const HTr* new_A_BI_parent,
                                    double angles[6])
{
  RCHECK(body);
  RCHECK(body->rigid_body_joints);

  const HTr* A_BI_body = NULL;
  const HTr* A_BI_parent = NULL;
  HTr A_BP_body;

  if (new_A_BI_body != NULL)
  {
    A_BI_body = new_A_BI_body;
  }
  else
  {
    A_BI_body = body->A_BI;
  }

  if (body->parent != NULL)
  {
    if (new_A_BI_parent != NULL)
    {
      A_BI_parent = new_A_BI_parent;
    }
    else
    {
      A_BI_parent = body->parent->A_BI;
    }
  }

  if (body->A_BP != NULL)
  {
    HTr_copy(&A_BP_body, body->A_BP);
  }
  else
  {
    HTr_setIdentity(&A_BP_body);
  }

  HTr trans;
  HTr_copy(&trans, A_BI_body);

  Mat3d_transposeMul(trans.rot, A_BP_body.rot, (double(*)[3])A_BI_body->rot);

  double temp3d[3];
  Vec3d_rotate(temp3d, A_BP_body.rot, A_BP_body.org);
  Vec3d_transRotateSelf(temp3d, (double(*)[3])A_BI_body->rot);
  Vec3d_subSelf(trans.org, temp3d);

  if (A_BI_parent != NULL)
  {
    double temp_mat[3][3];
    Mat3d_transpose(temp_mat, (double(*)[3])A_BI_parent->rot);
    Mat3d_preMulSelf(temp_mat, trans.rot);
    Mat3d_copy(trans.rot, temp_mat);

    Vec3d_subSelf(trans.org, A_BI_parent->org);
    Vec3d_rotateSelf(trans.org, (double(*)[3])A_BI_parent->rot);
  }

  Vec3d_copy(angles, trans.org);
  Mat3d_toEulerAngles(&angles[3], trans.rot);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool RcsGraph_setRigidBodyDoFs(RcsGraph* self, const RcsBody* body,
                               const double angles[6])
{
  if (self==NULL)
  {
    return false;
  }

  if (body==NULL)
  {
    return false;
  }

  if (body->rigid_body_joints==false)
  {
    return false;
  }

  VecNd_copy(&self->q->ele[body->jnt->jointIndex], angles, 6);

  return true;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool RcsGraph_updateSlaveJoints(const RcsGraph* self, MatNd* q, MatNd* q_dot)
{
  RCHECK_MSG((q->m==self->dof) && (q->n==1), "%d %d %d", q->m, self->dof, q->n);

  if (q_dot != NULL)
  {
    RCHECK((q_dot->m==self->dof) && (q_dot->n==1));
  }

  bool qHasChanged = false;

  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    if (JNT->coupledTo != NULL)
    {
      // Joint positions
      double q_master   = q->ele[JNT->coupledTo->jointIndex];
      double q_slave    = RcsJoint_computeSlaveJointAngle(JNT, q_master);

      // Compare with original joint angle
      if (q_slave != q->ele[JNT->jointIndex])
      {
        qHasChanged = true;
      }

      q->ele[JNT->jointIndex] =  q_slave;



      // Joint velocities
      if (q_dot != NULL)
      {
        double q_dot_master = q_dot->ele[JNT->coupledTo->jointIndex];
        q_dot->ele[JNT->jointIndex] =
          RcsJoint_computeSlaveJointVelocity(JNT, q_master, q_dot_master);
      }

    }
  }

  return qHasChanged;
}

/*******************************************************************************
 * Returns the number of kinematically coupled joints.
 ******************************************************************************/
int RcsGraph_countCoupledJoints(const RcsGraph* self)
{
  int n = 0;

  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    if ((JNT->coupledTo!=NULL) && (JNT->constrained==false))
    {
      n++;
    }

  }

  return n;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
int RcsGraph_coupledJointMatrix(const RcsGraph* self, MatNd* A, MatNd* invA)
{
  int nCoupledJoints = 0, nq = self->nJ;
  MatNd* H = NULL, *colSum = NULL;

  MatNd_create2(H, nq, nq);
  MatNd_create2(colSum, 1, nq);

  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    // We skip updating the matrix for constrained dofs
    if (JNT->constrained==true)
    {
      continue;
    }

    RcsJoint* masterJnt = JNT->coupledTo;

    // Here we construct a square nq x nq matrix. Each column corresponds to a
    // joint: If a joint is not coupled to another one, its jacobi index'th row
    // is set to 1. If another joint is coupled to it, the slave joint's row
    // is set to the coupling factor. In the end, the columns of the slave
    // joints are zero, and the other columns comprise a 1 at the joint's row,
    // and possibly additional coupling factors in other rows.
    if (masterJnt == NULL)
    {
      MatNd_set2(H, JNT->jacobiIndex, JNT->jacobiIndex, 1.0);
      MatNd_addToEle(colSum, 0, JNT->jacobiIndex, 1.0);
    }
    else
    {
      double q_master, sensitivity;
      q_master = MatNd_get2(self->q, masterJnt->jointIndex, 0);
      sensitivity = RcsJoint_computeSlaveJointVelocity(JNT, q_master, 1.0);
      MatNd_set2(H, JNT->jacobiIndex, masterJnt->jacobiIndex, sensitivity);
      MatNd_addToEle(colSum, 0, masterJnt->jacobiIndex, sensitivity);
      nCoupledJoints++;

      RCHECK_MSG(masterJnt->coupledTo==NULL, "Currently a slave joint (%s)"
                 " can't be coupled to another slave joint (%s)!",
                 JNT->name, masterJnt->name);
    }

  }

  // nqr is the number of reduced degrees of freedom (coupled dofs)
  int nqr = nq - nCoupledJoints;
  MatNd* invDiagAtA = NULL;
  MatNd_create2(invDiagAtA, nqr, 1);

  MatNd_reshape(A, nq, nqr);
  MatNd_setZero(A);

  MatNd_reshape(invA, nqr, nq);
  MatNd_setZero(invA);

  unsigned int nColumsA = 0;

  for (int col=0; col<nq; col++)
  {
    // Compute column sum
    double absColSum = MatNd_get2(colSum, 0, col);

    if (absColSum>0.0)
    {
      for (int row=0; row<nq; row++)
      {
        // Here we construct A
        double Hi = MatNd_get2(H, row, col)/absColSum;
        MatNd_set2(A, row, nColumsA, Hi);

        // Here we construct the transpose A^T  of A
        MatNd_set2(invA, nColumsA, row, Hi);

        // Here we construct A^T A. It is diagonal.
        invDiagAtA->ele[nColumsA] += Hi*Hi;
      }

      // Here we take the inverse: inv(A^T A)
      invDiagAtA->ele[nColumsA] = 1.0/invDiagAtA->ele[nColumsA];
      nColumsA++;
    }

  }

  // That's the pseudo-inverse: inv(A^T A) A^T
  MatNd_preMulDiagSelf(invA, invDiagAtA);


  // Test
  // MatNd* invA2 = NULL;
  // MatNd_create2(invA2, nqr, nq);
  // MatNd_rwPinv2(invA2, A, NULL, NULL);
  // MatNd_printCommentDigits("invA", invA, 4);
  // MatNd_printCommentDigits("invA2", invA2, 4);
  // MatNd_destroy(invA2);


  MatNd_destroy(H);
  MatNd_destroy(colSum);
  MatNd_destroy(invDiagAtA);


  return nCoupledJoints;
}

/*******************************************************************************
 * Re-order joint indices according to depth-first traversal. The graph's q and
 * q_dot arrays are kept consistent so that the same joint values can be
 * accessed with the jointIndex even if the index changed.
 * It is assumed that the graph's q and q_dot vector have been adjusted to have
 * enough memory for possibly added joints.
 ******************************************************************************/
static void RcsGraph_recomputeJointIndices(RcsGraph* self, MatNd* stateVec[],
                                           unsigned int nVec)
{
  unsigned int nqCount = 0, njCount = 0;
  MatNd* q_org = NULL;
  MatNd* qd_org = NULL;
  MatNd_clone2(q_org, self->q);
  MatNd_clone2(qd_org, self->q_dot);

  // Create copies of the stateVec arrays so that no overwriting happens
  MatNd** stateVecCp = RNALLOC(nVec, MatNd*);
  for (unsigned int i = 0; i < nVec; ++i)
  {
    stateVecCp[i] = MatNd_clone(stateVec[i]);
  }

  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    if (JNT->jointIndex != nqCount)
    {
      MatNd_set(self->q, nqCount, 0, q_org->ele[JNT->jointIndex]);
      MatNd_set(self->q_dot, nqCount, 0, qd_org->ele[JNT->jointIndex]);

      for (unsigned int i = 0; i < nVec; ++i)
      {
        MatNd_set(stateVec[i], nqCount, 0, stateVecCp[i]->ele[JNT->jointIndex]);
      }

      JNT->jointIndex = nqCount;
    }
    nqCount++;

    JNT->jacobiIndex = (JNT->constrained == false) ? njCount : -1;
    njCount++;
  }

  self->q->m = nqCount;
  self->q_dot->m = nqCount;
  self->dof = nqCount;
  self->nJ = njCount;

  MatNd_destroy(q_org);
  MatNd_destroy(qd_org);

  for (unsigned int i = 0; i < nVec; ++i)
  {
    stateVec[i]->m = nqCount;
    MatNd_destroy(stateVecCp[i]);
  }

  RFREE(stateVecCp);
}

/*******************************************************************************
* See header.
******************************************************************************/
void RcsGraph_makeJointsConsistent(RcsGraph* self)
{
  // Re-order joint indices according to depth-first traversal.
  RcsGraph_recomputeJointIndices(self, NULL, 0);

  // Link pointers of coupled joints and initialize range from master (if its
  // a complex coupling)
  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    MatNd_set(self->q, JNT->jointIndex, 0, JNT->q0);

    if (JNT->coupledJointName != NULL)
    {
      RcsJoint* master = RcsGraph_getJointByName(self, JNT->coupledJointName);
      RCHECK_MSG(master, "No coupled joint \"%s\"", JNT->coupledJointName);
      JNT->coupledTo = master;

      // Calculate range and initial posture
      double q_min  = RcsJoint_computeSlaveJointAngle(JNT, master->q_min);
      double q_max  = RcsJoint_computeSlaveJointAngle(JNT, master->q_max);
      double q0     = RcsJoint_computeSlaveJointAngle(JNT, master->q0);
      double q_init = RcsJoint_computeSlaveJointAngle(JNT, master->q_init);

      bool hasRange = false;

      if ((JNT->q_min!=0.0) || (JNT->q_max!=0.0) || (JNT->q0!=0.0))
      {
        hasRange = true;
      }

      REXEC(4)
      {
        if ((hasRange==true) &&
            ((fabs(JNT->q_min-q_min)>1.0e-4) ||
             (fabs(JNT->q_max-q_max)>1.0e-4) ||
             (fabs(JNT->q0-q0)>1.0e-4) ||
             (fabs(JNT->q_init-q_init)>1.0e-4)))
        {
          double s = RcsJoint_isRotation(JNT) ? 180.0/M_PI : 1.0;
          RLOG(4, "Overwriting range of coupled joint \"%s\" with "
               "values calculated from master range min, q0, max = "
               "[%f %f %f] init = %f (original: min, q0, max = "
               "[%f %f %f] init = %f)",
               JNT->name, s*JNT->q_min, s*JNT->q0, s*JNT->q_max, s*JNT->q_init,
               s*q_min, s*q0, s*q_max, s*q_init);
        }
      }

      JNT->q_min  = q_min;
      JNT->q_max  = q_max;
      JNT->q0     = q0;
      JNT->q_init = q_init;

      double master_q = MatNd_get(self->q, master->jointIndex, 0);
      MatNd_set(self->q, JNT->jointIndex, 0,
                RcsJoint_computeSlaveJointAngle(JNT, master_q));

    }
  }

}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsGraph_fprintXML(FILE* out, const RcsGraph* self)
{
  RCHECK(out);
  RCHECK(self);

  fprintf(out, "<Graph>\n\n");

  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    RcsBody_fprintXML(out, BODY, self);
  }

  // Pedantic: Check for sensors that are not attached to a body of the graph.
  // What to do with these?
  RCSGRAPH_TRAVERSE_SENSORS(self)
  {
    RCHECK_MSG(RcsBody_isInGraph(SENSOR->body, self), "Sensor \"%s\" not "
               "attached to any body (SENSOR->body is \"%s\")!",
               SENSOR->name ? SENSOR->name : "NULL",
               SENSOR->body ? SENSOR->body->name : "NULL");
  }

  fprintf(out, "</Graph>\n");
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsGraph_setRandomState(RcsGraph* self)
{
  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    if (!JNT->constrained)
    {
      MatNd_set2(self->q, JNT->jointIndex, 0,
                 Math_getRandomNumber(JNT->q_min, JNT->q_max));
    }
  }

  MatNd* q_dot = NULL;
  MatNd_create2(q_dot, self->dof, 1);
  RcsGraph_setState(self, NULL, q_dot);
  MatNd_destroy(q_dot);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool RcsGraph_appendCopyOfGraph(RcsGraph* self, RcsBody* root,
                                const RcsGraph* other_,
                                const char* suffix,
                                const HTr* A_BP)
{
  if ((root!=NULL) && (root->firstChild!=NULL))
  {
    RLOG(1, "Can't append graph to body with children (%s)", root->name);
    return false;
  }

  RcsGraph* other = RcsGraph_clone(other_);

  if (other == NULL)
  {
    RLOG(1, "Cloning graph failed");
    return false;
  }

  if (other->root == NULL)
  {
    RLOG(1, "Cloned graph has no root body");
    return false;
  }

  if (other->root->next != NULL)
  {
    RLOG(1, "Currently we can't handle multiple root bodies - your graph has "
         "body \"%s\" next to root (%s)", other->root->next->name,
         other->root->name);
    return false;
  }

  // Apply relative transformation
  // \todo: Rotate the root's next bodies
  RLOG(5, "Applying relative transformation");
  if (A_BP != NULL)
  {
    if (other->root->A_BP == NULL)
    {
      other->root->A_BP = HTr_create();
    }

    HTr_transformSelf(other->root->A_BP, A_BP);
  }

  // Add the suffix to all bodies that come new into the graph
  RLOG(5, "Adding suffixes");
  if (suffix != NULL)
  {
    RCSGRAPH_TRAVERSE_BODIES(other)
    {
      char* newName = RNALLOC(strlen(BODY->name) + strlen(suffix) + 1, char);
      strcpy(newName, BODY->name);
      strcat(newName, suffix);
      RFREE(BODY->name);
      BODY->name = newName;
    }

    // Add the suffix to all joints that come new into the graph
    RCSGRAPH_TRAVERSE_JOINTS(other)
    {
      char* newName = RNALLOC(strlen(JNT->name) + strlen(suffix) + 1, char);
      strcpy(newName, JNT->name);
      strcat(newName, suffix);
      RFREE(JNT->name);
      JNT->name = newName;
    }

    // Add the suffix to all sensors that come new into the graph
    RCSGRAPH_TRAVERSE_SENSORS(other)
    {
      char* newName = RNALLOC(strlen(SENSOR->name) + strlen(suffix) + 1, char);
      strcpy(newName, SENSOR->name);
      strcat(newName, suffix);
      RFREE(SENSOR->name);
      SENSOR->name = newName;
    }
  }

  // Attach to body of original graph
  RcsBody* socket = root;
  RcsBody* plug = other->root;

  RLOG(5, "Setting new graph's parent link");
  plug->parent = socket;
  RLOG(5, "%s->parent = %s", plug->name, socket ? socket->name : "NULL");

  if (socket == NULL)
  {
    RLOG(5, "Attaching to original graph's root");

    // Find last body on the first level
    RcsBody* b = self->root;
    while (b->next)
    {
      b = b->next;
    }

    b->next = plug;
    plug->prev = b;
  }
  else if (socket->lastChild == NULL)
  {
    RLOG(5, "Attaching to original graph's body \"%s\"", socket->name);
    socket->lastChild = plug;
    socket->firstChild = plug;
  }
  else
  {
    RLOG(5, "Attaching to original graph's body \"%s\"", socket->name);
    socket->lastChild->next = plug;
    plug->prev = socket->lastChild;
    socket->lastChild = plug;
  }
  RLOG(5, "Done changing DFS topology");

  // Merge configuration space vectors
  RLOG(5, "Adjusting dof from %d to %d", self->dof, self->dof+other->dof);
  self->dof += other->dof;

  // We search for the joint index of the last joint before the other graph's
  // root joint
  RLOG(5, "Merging state vectors");
  RcsJoint* jntBeforeNewBody = RcsBody_lastJointBeforeBody(root);
  unsigned int splitIdx = jntBeforeNewBody ? jntBeforeNewBody->jointIndex+1 : 0;
  RLOG(5, "Indices: initial=%d split=%d rest=%d",
       splitIdx, other->dof, self->dof-splitIdx);

  MatNd* q  = MatNd_create(self->dof, 1);
  MatNd* q_dot = MatNd_create(self->dof, 1);

  VecNd_copy(&q->ele[0], self->q->ele, splitIdx);
  VecNd_copy(&q->ele[splitIdx], other->q->ele, other->dof);
  VecNd_copy(&q->ele[splitIdx+other->dof], &self->q->ele[splitIdx],
             self->dof-splitIdx-other->dof);

  MatNd_destroy(self->q);
  MatNd_destroy(self->q_dot);
  self->q = q;
  self->q_dot = q_dot;

  // Append the sensors
  RLOG(5, "Adding sensors");
  RcsGraph_addSensor(self, other->sensor);

  RLOG(5, "Setting state");
  int runningJointIndex = 0;
  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    JNT->jointIndex = runningJointIndex++;
  }
  RcsGraph_setState(self, self->q, self->q_dot);

  RLOG(5, "Destroying temporary graph");
  other->root = NULL;
  other->sensor = NULL;
  RcsGraph_destroy(other);

  RLOG(5, "Done RcsGraph_appendCopyOfGraph()");
  return true;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
RcsSensor* RcsGraph_getSensorByName(const RcsGraph* self, const char* name)
{
  RCSGRAPH_TRAVERSE_SENSORS(self)
  {
    if (STREQ(name, SENSOR->name))
    {
      return SENSOR;
    }
  }

  return NULL;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsGraph_scale(RcsGraph* self, double scaleFactor)
{
  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    RCSBODY_TRAVERSE_SHAPES(BODY)
    {
      Vec3d_constMulSelf(SHAPE->extents, scaleFactor);
      Vec3d_constMulSelf(SHAPE->A_CB.org, scaleFactor);
      SHAPE->scale *= scaleFactor;
    }
  }

  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    if (JNT->A_JP)
    {
      Vec3d_constMulSelf(JNT->A_JP->org, scaleFactor);
    }

    if (RcsJoint_isTranslation(JNT) == true)
    {
      const double upperRange = JNT->q_max - JNT->q0;
      const double lowerRange = JNT->q0 - JNT->q_min;
      JNT->q0 *= scaleFactor;
      JNT->q_init *= scaleFactor;
      self->q->ele[JNT->jointIndex] *= scaleFactor;
      JNT->q_min = JNT->q0 - scaleFactor*lowerRange;
      JNT->q_max = JNT->q0 + scaleFactor*upperRange;
    }
  }

  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    Vec3d_constMulSelf(BODY->Inertia->org, scaleFactor);

    if (BODY->A_BP)
    {
      Vec3d_constMulSelf(BODY->A_BP->org, scaleFactor);
    }

  }

  RcsGraph_setState(self, NULL, NULL);
}

/*******************************************************************************
* See header.
******************************************************************************/
bool RcsGraph_removeBody(RcsGraph* self, const char* bdyName,
                         MatNd* stateVec[], unsigned int nVec)
{
  RcsBody* bdy = RcsGraph_getBodyByName(self, bdyName);

  if (bdy == NULL)
  {
    RLOG(1, "Couldn't find body \"%s\" to remove - skipping", bdyName);
    return false;
  }

  if (RcsBody_isLeaf(bdy)==false)
  {
    RLOG(1, "Body \"%s\" has children - currently can't be removed", bdyName);
    return false;
  }

  // Now we re-wire the graph

  // 1. parent
  RcsBody* parent = bdy->parent;

  if (parent)
  {
    if (parent->firstChild == bdy)
    {
      parent->firstChild = bdy->next;
    }

    if (parent->lastChild == bdy)
    {
      parent->lastChild = bdy->prev;
    }
  }

  if (bdy->prev)
  {
    bdy->prev->next = bdy->next;
  }

  if (bdy->next)
  {
    bdy->next->prev = bdy->prev;
  }

  if (bdy == self->root)
  {
    self->root = bdy->next ? bdy->next : bdy->firstChild;
    RCHECK(self->root);
  }

  RcsBody_destroy(bdy);
  RcsGraph_recomputeJointIndices(self, stateVec, nVec);

  return true;
}

/*******************************************************************************
*
******************************************************************************/
bool RcsGraph_addBody(RcsGraph* graph, RcsBody* parent, RcsBody* body,
                      MatNd* stateVec[], unsigned int nVec)
{
  if (body == NULL)
  {
    RLOG(1, "Can't append NULL body");
    return false;
  }

  body->prev = NULL;
  body->next = NULL;
  body->firstChild = NULL;
  body->lastChild = NULL;
  body->parent = parent;

  if (parent != NULL)
  {
    if (parent->lastChild != NULL)  // parent already has children
    {
      parent->lastChild->next = body;
      body->prev = parent->lastChild;
    }
    else
    {
      parent->firstChild = body;
    }

    parent->lastChild = body;
  }
  else
  {
    if (graph->root != NULL)
    {
      RcsBody* b = graph->root;
      while (b->next)
      {
        b = b->next;
      }

      b->next = body;
      body->prev = b;
    }
    else
    {
      graph->root = body;
    }
  }

  // Connect joints.
  if (body->jnt != NULL)
  {
    body->jnt->prev = RcsBody_lastJointBeforeBody(parent);
    unsigned int nj = RcsBody_numJoints(body);

    graph->q = MatNd_realloc(graph->q, graph->q->m + nj, 1);
    graph->q_dot = MatNd_realloc(graph->q_dot, graph->q_dot->m + nj, 1);

    for (unsigned int i = 0; i < nVec; ++i)
    {
      stateVec[i] = MatNd_realloc(stateVec[i], graph->q_dot->m + nj, 1);
    }

    RcsGraph_recomputeJointIndices(graph, stateVec, nVec);

    RCSBODY_TRAVERSE_JOINTS(body)
    {
      MatNd_set(graph->q, JNT->jointIndex, 0, JNT->q_init);
    }

  }

  NLOG(9, "Inserted Body into Graph: name=%s parent=%s prev=%s",
       body->name,
       body->parent ? body->parent->name : "NULL",
       body->prev ? body->prev->name : "NULL");

  return true;
}
