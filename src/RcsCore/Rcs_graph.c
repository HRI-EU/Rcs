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
static void RcsGraph_bodyKinematics(RcsGraph* graph,
                                    RcsBody* bdy,
                                    unsigned int* nJ,
                                    const MatNd* q,
                                    const MatNd* q_dot)
{
  // Copy last body transformation to current one
  if (bdy->parentId==-1)
  {
    HTr_setIdentity(&bdy->A_BI);
  }
  else
  {
    HTr_copy(&bdy->A_BI, &graph->bodies[bdy->parentId].A_BI);
  }

  // Set pointers to propagated elements
  HTr* A_VI   = &bdy->A_BI;
  RcsJoint* j = RCSJOINT_BY_ID(graph, bdy->jntId);


  // Traverse joints
  while (j != NULL)
  {
    // Joint position: Here we assume that the joint angles already reflect
    // the kinematic joint coupling (if applicable)
    double qi = MatNd_get2(q, j->jointIndex, 0);

    // Propagate the joint's world transform with the joint's relative
    // transform: I_r_IJ' = I_r_IJ + (A_JI)^T J_r_JJ' and A_J'I = A_J'J * A_JI
    Vec3d_transMulAndAdd(j->A_JI.org, A_VI->org, A_VI->rot, j->A_JP.org);
    Mat3d_mul(j->A_JI.rot, j->A_JP.rot, A_VI->rot);

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
    j      = (j->nextId==-1) ? NULL : &graph->joints[j->nextId];

  }   // while(jnt)

  // Copy last joint transform to body
  HTr_copy(&bdy->A_BI, A_VI);

  // Apply relative rotation of body wrt. last joint
  Vec3d_transMulAndAddSelf(bdy->A_BI.org, bdy->A_BI.rot, bdy->A_BP.org);
  Mat3d_preMulSelf(bdy->A_BI.rot, bdy->A_BP.rot);



  // Velocities
  if (q_dot != NULL)
  {
    double q_dot_i, oxr[3], om_i[3];

    // Linear velocity according to parent angular velocity
    if (bdy->parentId == -1)
    {
      Vec3d_setZero(bdy->x_dot);
      Vec3d_setZero(bdy->omega);
    }
    else
    {
      RcsBody* parent = &graph->bodies[bdy->parentId];

      // Reset velocities for root bodies
      Vec3d_copy(bdy->x_dot, parent->x_dot);
      Vec3d_copy(bdy->omega, parent->omega);

      // Linear velocity term due to the parent's angular velocity
      double tmp[3];
      Vec3d_sub(tmp, bdy->A_BI.org, parent->A_BI.org);
      Vec3d_crossProduct(oxr, parent->omega, tmp);
      Vec3d_addSelf(bdy->x_dot, oxr);
    }

    // Velocity terms
    j = RCSJOINT_BY_ID(graph, bdy->jntId);

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
          Vec3d_sub(tmp, bdy->A_BI.org, j->A_JI.org);
          Vec3d_crossProduct(oxr, om_i, tmp);
          Vec3d_addSelf(bdy->x_dot, oxr);
          break;
        }
      }
      j = (j->nextId==-1) ? NULL : &graph->joints[j->nextId];

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
    RcsGraph_bodyKinematics(self, BODY, &nJ, self->q,
                            q_dot_ ? self->q_dot : NULL);
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
    RcsGraph_bodyKinematics(self, BODY, &self->nJ,
                            q ? q : self->q, q_dot ? q_dot : self->q_dot);
  }

}

/*******************************************************************************
 * See header.
 ******************************************************************************/
static RcsGraph* RcsGraph_createFromBuffer(const char* buffer,
                                           unsigned int size)
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

  strcpy(self->cfgFile, "Created_from_memory_buffer");

  return self;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
RcsGraph* RcsGraph_create(const char* cfgFile)
{
  // Determine absolute file name of config file and return if it doesn't exist
  char filename[256] = "";
  bool fileExists = Rcs_getAbsoluteFileName(cfgFile, filename);

  if ((fileExists==false) && (cfgFile!=NULL))
  {
    RcsGraph* self = RcsGraph_createFromBuffer(cfgFile, strlen(cfgFile)+1);

    if (self==NULL)
    {
      REXEC(1)
      {
        RMSG("Resource path is:");
        Rcs_printResourcePath();
        RMSG("RcsGraph config file \"%s\" not found in resource path", cfgFile);
      }
    }

    return self;
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

    if (self)
    {
      snprintf(self->cfgFile, RCS_MAX_FILENAMELEN, "%s", filename);
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
      snprintf(self->cfgFile, RCS_MAX_FILENAMELEN, "%s", filename);
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
void RcsGraph_destroy(RcsGraph* self)
{
  if (self == NULL)
  {
    return;
  }

  NLOG(5, "Deleting graph \"%s\" (addr 0x%x)",
       self->cfgFile, (unsigned int) self);

  // Destroy all bodies: Just clear shapes
  for (unsigned int i=0; i<self->nBodies; ++i)
  {
    RcsBody_clear(&self->bodies[i]);
  }
  RFREE(self->bodies);
  RFREE(self->joints);

  NLOG(0, "Deleted body list");

  // Destroy all sensors
  for (unsigned int i=0; i<self->nSensors; ++i)
  {
    RcsSensor_clear(&self->sensors[i]);
  }
  RFREE(self->sensors);

  NLOG(0, "Deleted sensor array");

  MatNd_destroy(self->q);
  MatNd_destroy(self->q_dot);

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

  RCSGRAPH_FOREACH_BODY(self)
  {
    if (BODY->id!=-1)
    {
      m += BODY->m;
    }
  }

  return m;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
double RcsGraph_massFromBody(const RcsGraph* self, const char* root)
{
  double m = 0.0;

  RcsBody* r = RcsGraph_getBodyByName(self, root);

  if (r)
  {
    RCSBODY_TRAVERSE_BODIES(self, r)
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
void RcsGraph_getInitState(const RcsGraph* self, MatNd* q_init)
{
  MatNd_reshape(q_init, self->dof, 1);

  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    MatNd_set2(q_init, JNT->jointIndex, 0, JNT->q_init);
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
               jnt->name, j, scale*q_i.ele[j],
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
                 jnt->name, j, scale*q_dot_i,
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
void RcsGraph_changeInitState(RcsGraph* self, const MatNd* q_init)
{
  RCHECK_EQ(q_init->n, 1);

  if (q_init->m == self->dof)
  {
    RCSGRAPH_TRAVERSE_JOINTS(self)
    {
      JNT->q_init = MatNd_get2(q_init, JNT->jointIndex, 0);
    }
  }
  else if (q_init->m == self->nJ)
  {
    RCSGRAPH_TRAVERSE_JOINTS(self)
    {
      JNT->q_init = MatNd_get2(q_init, JNT->jacobiIndex, 0);
    }
  }
  else
  {
    RFATAL("Size mismatch: dof=%d   nJ=%d but array q_init has %d rows",
           self->dof, self->nJ, q_init->m);
  }

}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsGraph_resetRigidBodyDofs(RcsGraph* self)
{
  RCSGRAPH_FOREACH_BODY(self)
  {
    if (BODY->rigid_body_joints && (BODY->id!=-1))
    {
      RCSBODY_FOREACH_JOINT(self, BODY)
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

  RCSGRAPH_FOREACH_BODY(self)
  {
    if ((BODY->rigid_body_joints==false) || (BODY->jntId==-1) || (BODY->id==-1))
    {
      continue;
    }

    const RcsJoint* bdyJnt = RCSJOINT_BY_ID(self, BODY->jntId);
    double* q_dst = &q->ele[bdyJnt->jointIndex];
    double* q_src = &src->ele[bdyJnt->jointIndex];

    for (unsigned int i=0; i<6; i++)
    {
      if (q_dst[i] != q_src[i])
      {
        q_dst[i] = q_src[i];
        qChanged = true;
      }
    }

  }   // RCSGRAPH_FOREACH_BODY

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
                                             double angularMargin,
                                             double linearMargin,
                                             bool verbose)
{
  unsigned int violations = 0;

  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    // We skip the check for constrained dofs
    if (JNT->constrained==true)
    {
      continue;
    }

    double qi = MatNd_get2(self->q, JNT->jointIndex, 0);
    double margin = RcsJoint_isRotation(JNT) ? angularMargin : linearMargin;

    if ((qi < JNT->q_min+margin) || (qi > JNT->q_max-margin))
    {
      if (verbose)
      {
        RMSG("Joint limit of \"%s\" violated: %f [%f %f] margin: %f",
             JNT->name, qi, JNT->q_min, JNT->q_max, margin);
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

  RCSGRAPH_FOREACH_BODY(self)
  {
    if (STREQ(name, BODY->name) && (BODY->id!=-1))
    {
      return BODY;
    }
  }

  if (STRNEQ(name, "GenericBody", 11))
  {
    int num = atoi(&name[11]);

    if ((num < 0) || (num >= RCS_NUM_GENERIC_BODIES))
    {
      RLOG(1, "GenericBody \"%s\": suffix must be [0...%d]",
           name, RCS_NUM_GENERIC_BODIES-1);
      return NULL;
    }

    return RCSBODY_BY_ID(self, self->gBody[num]);
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

  size_t nameLen = strlen(name);
  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    if (STRNEQ(name, BODY->name, nameLen))
    {
      return BODY;
    }
  }

  if (STRNEQ(name, "GenericBody", 11))
  {
    int num = atoi(&name[11]);

    if ((num < 0) || (num >= RCS_NUM_GENERIC_BODIES))
    {
      RLOG(1, "GenericBody \"%s\": suffix must be [0...%d]",
           name, RCS_NUM_GENERIC_BODIES-1);
      return NULL;
    }

    return RCSBODY_BY_ID(self, self->gBody[num]);
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
void RcsGraph_fprint(FILE* out, const RcsGraph* graph)
{
  RCSGRAPH_TRAVERSE_BODIES(graph)
  {
    RcsBody_fprint(out, BODY, graph);
  }

  for (unsigned int i=0; i<graph->nSensors; ++i)
  {
    fprintf(out, "Sensor %d:\n", i);
    RcsSensor_fprint(out, &graph->sensors[i]);
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
                                   const char* name)
{
  RcsBody* b = RcsGraph_getBodyByName(self, name);

  if (!b)
  {
    RLOG(4, "Body \"%s\" not found - skipping backward recursion", name);
    return;
  }

  RMSG("Backward pass starting from body \"%s\"", b->name);

  RcsJoint* jnt = RCSJOINT_BY_ID(self, b->jntId);

  if (!jnt)
  {
    RLOG(4, "Body \"%s\" has no joints", name);
    return;
  }

  while (jnt->nextId!=-1)
  {
    jnt = &self->joints[jnt->nextId];
  }

  while (jnt->prevId!=-1)
  {
    RMSG("Joint %s", jnt->name);
    jnt = &self->joints[jnt->prevId];
  }

  RMSG("Joint %s", jnt->name);
  RMSG("Backward pass finished");
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool RcsGraph_setJoint(RcsGraph* self, const char* name, double val)
{
  RcsJoint* j = RcsGraph_getJointByName(self, name);

  if (j == NULL)
  {
    RLOG(1, "Can't set joint angle of non-existing joint \"%s\"", name);
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
  char buf[256];

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


  fprintf(out, "<model_state model=\"DefaultPose\" time_stamp=\"\">\n");



  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    // Skip constrained joints if state to be printed is of type IK
    if ((stateType==RcsStateIK) && (JNT->jacobiIndex==-1))
    {
      continue;
    }

    int idx = (stateType==RcsStateIK) ? JNT->jacobiIndex : JNT->jointIndex;

    // We ignore entries that equal the initial q_init values
    if (fabs(q->ele[idx]-JNT->q_init)<1.0e-6)
    {
      continue;
    }

    double qi = RcsJoint_isRotation(JNT)?RCS_RAD2DEG(q->ele[idx]):q->ele[idx];

    fprintf(out, "  <joint_state joint=\"%s\" position=\"%s\" />\n",
            JNT->name, String_fromDouble(buf, qi, 6));
  }





  fprintf(out, "</model_state>\n");
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsGraph_writeDotFile(const RcsGraph* self, const char* filename)
{
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
    fprintf(fd, "%d[label=\"%s (%d)\" ", BODY->id, BODY->name, BODY->id);
    fprintf(fd, "shape=box style=filled color=\"0.7 0.3 1\"];\n");
  }

  // Joint labels
  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    if (BODY->jntId!=-1) // Body is connected to predecessor by joints
    {
      fprintf(fd, "%d->%d [label=\"", BODY->parentId, BODY->id);
      RCSBODY_FOREACH_JOINT(self, BODY)
      {
        fprintf(fd, "%s%s \\n",
                JNT->name, JNT->constrained ? " (constrained)" : "");
      }
      fprintf(fd, "\"];\n");
    }
    else     // Body is fixed to predecessor by fixed transformation
    {
      fprintf(fd, "%d->%d [label=\"Fixed\" ", BODY->parentId, BODY->id);
      fprintf(fd, "color=\"red\" fontcolor=\"red\"];\n");
    }
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
    int i_parent = BODY->parentId;

    if (i_parent != -1)
    {
      fprintf(fd, "%d->%d [label=\"Parent: %s \\n",
              i, i_parent, RCSBODY_BY_ID(self, BODY->parentId)->name);
      fprintf(fd, "\"];\n");
    }
    else
    {
      fprintf(fd, "%d->-1 [label=\"Parent: Root \\n", i);
      fprintf(fd, "\"];\n");
    }

    int i_child = BODY->firstChildId;

    if (i_child != -1)
    {
      fprintf(fd, "%d->%d [label=\"Child: %s \\n",
              i, i_child, RCSBODY_BY_ID(self, BODY->firstChildId)->name);
      fprintf(fd, "\"];\n");
    }

    int i_prev = BODY->prevId;

    if (i_prev != -1)
    {
      fprintf(fd, "%d->%d [label=\"Prev: %s \\n",
              i, i_prev, RCSBODY_BY_ID(self, BODY->prevId)->name);
      fprintf(fd, "\"];\n");
    }

    int i_next = BODY->nextId;

    if (i_next != -1)
    {
      fprintf(fd, "%d->%d [label=\"Next: %s \\n",
              i, i_next, RCSBODY_BY_ID(self, BODY->nextId)->name);
      fprintf(fd, "\"];\n");
    }

    int i_last = BODY->lastChildId;

    if (i_last != -1)
    {
      fprintf(fd, "%d->%d [label=\"Last: %s \\n",
              i, i_last, RCSBODY_BY_ID(self, BODY->lastChildId)->name);
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

  RcsGraph_bodyPointJacobian(self, body, body->Inertia.org, NULL, J_cog);
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
  double sc = RcsGraph_checkJointSpeeds(self, dq, dt, type);

  // Scale it down
  if (sc < 1.0)
  {
    MatNd_constMulSelf(dq, sc);
  }

  return sc;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
double RcsGraph_checkJointSpeeds(const RcsGraph* self, const MatNd* dq,
                                 double dt, RcsStateType type)
{
  int dimension = (type == RcsStateFull) ? self->dof : self->nJ;

  RCHECK_MSG((dq->m==dimension && dq->n==1) || (dq->m==1 &&  dq->n==dimension),
             "dq is of size %d x %d - should be %d x 1 or 1 x %d",
             dq->m, dq->n, dimension, dimension);

  double sc = 1.0;
  RcsJoint* badJnt = NULL;

  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    if ((JNT->jacobiIndex == -1) && (type == RcsStateIK))
    {
      continue;
    }

    int index = (type == RcsStateIK) ? JNT->jacobiIndex : JNT->jointIndex;

    // Get speed limits in units/sec
    const double q_dot = fabs(dq->ele[index] / dt);
    const double sc_i = (q_dot > JNT->speedLimit) ? JNT->speedLimit/q_dot : 1.0;

    if (sc_i < sc)
    {
      sc = sc_i;
      badJnt = JNT;
    }
  }

  REXEC(6)
  {
    if ((sc<1.0) && (badJnt!=NULL))
    {
      double sLim = badJnt->speedLimit;
      double toDeg = RcsJoint_isRotation(badJnt) ? 180.0/M_PI : 1.0;
      int i = (type == RcsStateIK) ? badJnt->jacobiIndex : badJnt->jointIndex;
      RMSG("Scaling speeds with %5.5f (due to joint \"%s\": speed=%5.6f %s  "
           "limit=%5.6f)", sc, badJnt->name, toDeg*fabs(dq->ele[i] / dt),
           RcsJoint_isRotation(badJnt) ? "deg" : "m", toDeg*sLim);
    }
  }

  return sc;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
int RcsGraph_clipJointAccelerations(const RcsGraph* self, MatNd* qd,
                                    const MatNd* qd_prev, double dt,
                                    double ratio, RcsStateType type)
{
  int dimension = (type == RcsStateFull) ? self->dof : self->nJ;

  RCHECK_MSG((qd->m==dimension && qd->n==1) || (qd->m==1 &&  qd->n==dimension),
             "qd is of size %d x %d - should be %d x 1 or 1 x %d",
             qd->m, qd->n, dimension, dimension);

  RCHECK_MSG((qd_prev->m==dimension && qd_prev->n==1) ||
             (qd_prev->m==1 &&  qd_prev->n==dimension),
             "qd_prev is of size %d x %d - should be %d x 1 or 1 x %d",
             qd_prev->m, qd_prev->n, dimension, dimension);

  int nClipped = 0;

  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    if ((JNT->jacobiIndex == -1) && (type == RcsStateIK))
    {
      continue;
    }

    if ((JNT->accLimit==DBL_MAX) && (JNT->decLimit==DBL_MAX))
    {
      continue;
    }

    bool accClip = false;
    int index = (type == RcsStateIK) ? JNT->jacobiIndex : JNT->jointIndex;
    double accel = (qd->ele[index] - qd_prev->ele[index])/dt;
    const double accLimit = ratio * JNT->accLimit;
    const double decLimit = ratio * JNT->decLimit;

    // Here we treat acceleration and deceleration differently.
    if (qd_prev->ele[index] < 0.0)
    {
      if (accel > decLimit)   // deceleration
      {
        qd->ele[index] = qd_prev->ele[index] + decLimit*dt;
        nClipped++;
        accClip = true;
      }
      else if (accel < -accLimit)   // acceleration
      {
        qd->ele[index] = qd_prev->ele[index] - accLimit*dt;
        nClipped++;
        accClip = true;
      }
    }
    else   // qp_prev[i] >= 0.0
    {
      if (accel > accLimit)   // acceleration
      {
        qd->ele[index] = qd_prev->ele[index] + accLimit*dt;
        nClipped++;
        accClip = true;
      }
      else if (accel < -decLimit)   // deceleration
      {
        qd->ele[index] = qd_prev->ele[index] - decLimit*dt;
        nClipped++;
        accClip = true;
      }
    }



    if (accClip)
    {
      RLOG(5, "Acceleration clip +: accel=%f qp_curr=%f qp_prev=%f",
           accel, RCS_RAD2DEG(qd->ele[index]),
           RCS_RAD2DEG(qd_prev->ele[index]));
    }



  }   // RCSGRAPH_TRAVERSE_JOINTS(self)


  return nClipped;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool RcsGraph_check(const RcsGraph* self, int* nErrors_, int* nWarnings_)
{
  // Return if graph points to NULL
  if (self==NULL)
  {
    RLOG(1, "Graph is NULL");

    if (nErrors_)
    {
      *nErrors_ = 1;
    }

    if (nWarnings_)
    {
      *nWarnings_ = 0;
    }

    return false;
  }


  int nErrors = 0, nWarnings = 0;

  if (strlen(self->cfgFile) >= RCS_MAX_FILENAMELEN)
  {
    nWarnings++;
    RLOG(1, "Graph's cfgFile \"%s\" very likely has a truncated name",
         self->cfgFile);
  }

  // Check for
  //  - body name truncations
  //  - file name truncations in all shapes
  //  - correctness of rigid body joints
  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    if (strlen(BODY->name) >= RCS_MAX_NAMELEN-1)
    {
      nErrors++;
      RLOG(1, "Body \"%s\" very likely has a truncated name", BODY->name);
    }

    if (strlen(BODY->name) == 0)
    {
      nErrors++;
      RLOG(1, "Found body with empty body name");
    }

    RCSBODY_TRAVERSE_SHAPES(BODY)
    {
      if (strlen(SHAPE->meshFile) >= RCS_MAX_FILENAMELEN-1)
      {
        nErrors++;
        RLOG(1, "Shape of body \"%s\" very likely has a truncated mesh file: %s",
             BODY->name, SHAPE->meshFile);
      }

      if (strlen(SHAPE->textureFile) >= RCS_MAX_FILENAMELEN-1)
      {
        nWarnings++;
        RLOG(1, "Shape of body \"%s\" very likely has a truncated texture file:"
             "%s", BODY->name, SHAPE->textureFile);
      }

      if (strlen(SHAPE->color) >= RCS_MAX_NAMELEN-1)
      {
        nWarnings++;
        RLOG(1, "Shape of body \"%s\" very likely has a truncated color:"
             "%s", BODY->name, SHAPE->color);
      }

      if (strlen(SHAPE->material) >= RCS_MAX_NAMELEN-1)
      {
        nWarnings++;
        RLOG(1, "Shape of body \"%s\" very likely has a truncated material:"
             "%s", BODY->name, SHAPE->material);
      }
    }

    if (BODY->rigid_body_joints)
    {
      // Test if rigid body has 6 joints
      if (RcsBody_numJoints(self, BODY) != 6)
      {
        nErrors++;
        RLOG(1, "Body \"%s\" is a rigid body joint, but has %d joints "
             "(instead of 6)", BODY->name, RcsBody_numJoints(self, BODY));
        continue;
      }

      // Test if joints are in order transX-transY-transZ-rotX-rotY-rotZ
      RcsJoint* jnt = RCSJOINT_BY_ID(self, BODY->jntId);

      if (jnt==NULL)
      {
        nErrors++;
        RLOG(1, "Body \"%s\" with rigid body joint has jntId -1", BODY->name);
        continue;
      }

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
          if (HTr_isIdentity(&jnt->A_JP) == false)
          {
            nErrors++;
            REXEC(1)
            {
              RMSG("Joint %d of rigid body \"%s\" has non-identity "
                   "relative transform:", i, BODY->name);
              HTr_fprint(stderr, &jnt->A_JP);
            }
          }
        }

        //jnt = jnt->next;
        jnt = &self->joints[jnt->nextId];
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
    if ((Mat3d_getFrobeniusnorm(BODY->Inertia.rot)>0.0) && (BODY->m<=0.0))
    {
      nWarnings++;
      RLOG(1, "Body \"%s\" has positive inertia but zero mass", BODY->name);
    }
  }

  // Check for joint centers and positions out of range, and for valid indices
  // of the joint array pointers
  unsigned int numJoints = 0;
  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    numJoints++;

    if (strlen(JNT->name) >= RCS_MAX_NAMELEN-1)
    {
      nWarnings++;
      RLOG(1, "Joint \"%s\" very likely has a truncated name", JNT->name);
    }

    if ((JNT->q0<JNT->q_min) && (JNT->coupledToId==-1))
    {
      nWarnings++;
      double s = RcsJoint_isRotation(JNT) ? 180.0/M_PI : 1.0;
      RLOG(1, "Joint \"%s\": q0 < q_min (q_min=%f   q0=%f   q_max=%f [%s])",
           JNT->name, s*JNT->q_min, s*JNT->q0, s*JNT->q_max,
           RcsJoint_isRotation(JNT) ? "deg" : "m");
    }

    if ((JNT->q0>JNT->q_max) && (JNT->coupledToId==-1))
    {
      nWarnings++;
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

    // Check if coupled joint factors match. Currently only linear
    // couplings work
    if (JNT->nCouplingCoeff>0)
    {
      if (JNT->coupledToId == -1)
      {
        nErrors++;
        RLOG(1, "Joint \"%s\": Coupled joint is NULL, but coupling factors"
             " are defined", JNT->name);
      }

      if ((JNT->nCouplingCoeff!=1))
      {
        nErrors++;
        RLOG(1, "Incorrect number of coupling factors in joint \"%s\": %d",
             JNT->name, JNT->nCouplingCoeff);
      }

    }

    if (JNT->speedLimit < 0.0)
    {
      nErrors++;
      RLOG(1, "Joint \"%s\" has negative speed limit: %f",
           JNT->name, JNT->speedLimit);
    }

    if ((JNT->jointIndex >= 0) || (JNT->jointIndex < (int) self->dof))
    {
      const double qi = self->q->ele[JNT->jointIndex];

      if ((qi<JNT->q_min) && (JNT->coupledToId==-1))
      {
        nErrors++;
        double s = RcsJoint_isRotation(JNT) ? 180.0/M_PI : 1.0;
        RLOG(1, "Joint \"%s\": q < q_min (q_min=%f   q=%f   q_max=%f [%s])",
             JNT->name, s*JNT->q_min, s*qi, s*JNT->q_max,
             RcsJoint_isRotation(JNT) ? "deg" : "m");
      }

      if ((qi>JNT->q_max) && (JNT->coupledToId==-1))
      {
        nErrors++;
        double s = RcsJoint_isRotation(JNT) ? 180.0/M_PI : 1.0;
        RLOG(1, "Joint \"%s\": q > q_max (q_min=%f   q=%f   q_max=%f [%s])",
             JNT->name, s*JNT->q_min, s*qi, s*JNT->q_max,
             RcsJoint_isRotation(JNT) ? "deg" : "m");
      }
    }

  }

  if (numJoints != self->dof)
  {
    RLOG(1, "Number of joints (%d) doesn't match dof (%d)",
         numJoints, self->dof);
    nErrors++;
  }

  if ((self->q->m!=self->dof) || (self->q->n!=1))
  {
    RLOG(1, "Q-vector dimension mismatch: %d x %d != %d (dof) x 1",
         self->q->m, self->q->n, self->dof);
    nErrors++;
  }

  // Check for body connection consistencs
  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    RcsBody* first = RCSBODY_BY_ID(self, BODY->firstChildId);
    RcsBody* last = RCSBODY_BY_ID(self, BODY->lastChildId);

    if (first && (first->prevId!=-1))
    {
      RcsBody* prefOfFirst = &self->bodies[first->prevId];
      RLOG(1, "Body \"%s\" has firstChild (\"%s\") with prev body (\"%s\")",
           BODY->name, first->name, prefOfFirst->name);
      nErrors++;
    }

    if (last && (last->nextId != -1))
    {
      RcsBody* nextAfterLast = &self->bodies[last->nextId];
      RLOG(1, "Body \"%s\" has lastChild (\"%s\") with next body (\"%s\")",
           BODY->name, last->name, nextAfterLast->name);
      nErrors++;
    }

    if ((first == NULL) && (last != NULL))
    {
      RLOG(1, "Body \"%s\" has no firstChild but lastChild (\"%s\")",
           BODY->name, last->name);
      nErrors++;
    }
  }

  // Check that coupled joints are not coupled against other coupled joints
  // if they have no constraint
  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    if (JNT->constrained)
    {
      continue;
    }

    RcsJoint* cj = RCSJOINT_BY_ID(self, JNT->coupledToId);

    if (cj && (cj->coupledToId!=-1))
    {
      RLOG(1, "Joint %s is coupled against the coupled joint %s",
           JNT->name, cj->name);
      nErrors++;
    }
  }

  // Check finiteness of q and q_dot
  if (!MatNd_isFinite(self->q))
  {
    RLOG(1, "Found non-finite values in q");
    REXEC(4)
    {
      MatNd_printCommentDigits("q", self->q, 12);
    }
    nErrors++;
  }

  if (!MatNd_isFinite(self->q_dot))
  {
    RLOG(1, "Found non-finite values in q_dot");
    REXEC(4)
    {
      MatNd_printCommentDigits("q_dot", self->q_dot, 12);
    }
    nErrors++;
  }



  // Copy warning and error statistics
  if (nErrors_)
  {
    *nErrors_ = nErrors;
  }

  if (nWarnings_)
  {
    *nWarnings_ = nWarnings;
  }

  return (nErrors+nWarnings==0) ? true : false;
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
  dst->q_dot   = MatNd_clone(src->q_dot);
  dst->bodies = RNALLOC(src->nBodies, RcsBody);
  dst->nBodies = src->nBodies;
  dst->joints = RNALLOC(src->dof, RcsJoint);
  dst->sensors = RNALLOC(src->nSensors, RcsSensor);
  dst->nSensors = 0;

  // Copy all joints, it's just a big memory block
  memcpy(dst->joints, src->joints, src->dof*sizeof(RcsJoint));

  // Copy all bodies
  for (unsigned int i=0; i<src->nBodies; ++i)
  {
    dst->bodies[i] = src->bodies[i];
    dst->bodies[i].shape = NULL;
  }

  RCSGRAPH_TRAVERSE_BODIES(src)
  {
    // Determine body corresponding to BODY in the dst graph
    RcsBody* b = &dst->bodies[BODY->id];

    // Make a copy of all body shapes and attach them to the body
    int nShapes = RcsBody_numShapes(BODY);
    b->shape = RNALLOC(nShapes + 1, RcsShape*);
    for (int i = 0; i < nShapes; i++)
    {
      b->shape[i] = RALLOC(RcsShape);
      RcsShape_copy(b->shape[i], BODY->shape[i]);
    }

  }   // RCSGRAPH_TRAVERSE_BODIES(src)


  // Create the sensors
  for (unsigned int i=0; i<src->nSensors; ++i)
  {
    RcsSensor* s = RcsGraph_insertSensor(dst);
    RcsSensor_copy(s, &src->sensors[i]);
  }

  REXEC(4)
  {
    // Check for consistency
    RCHECK_MSG(RcsGraph_check(dst, NULL, NULL),
               "Consistency check for graph \"%s\" failed", dst->cfgFile);
  }

  return dst;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
RcsBody* RcsGraph_linkGenericBody(RcsGraph* self, int bdyNum,
                                  const char* name)
{
  RCHECK(bdyNum >= 0 && bdyNum < RCS_NUM_GENERIC_BODIES);

  RcsBody* b = RcsGraph_getBodyByName(self, name);
  self->gBody[bdyNum] = b ? b->id : -1;

  return b;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
RcsBody* RcsGraph_getGenericBodyPtr(const RcsGraph* self, int bdyNum)
{
  return RCSBODY_BY_ID(self, self->gBody[bdyNum]);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsGraph_relativeRigidBodyDoFs(const RcsGraph* self,
                                    const RcsBody* body,
                                    const HTr* new_A_BI_body,
                                    const HTr* new_A_BI_parent,
                                    double angles[6])
{
  RCHECK_MSG(RcsBody_isFloatingBase(self, body),
             "Body \"%s\"", body ? body->name : "NULL");

  const HTr* A_BI_body = NULL;
  const HTr* A_BI_parent = NULL;
  HTr A_BP_body;

  if (new_A_BI_body != NULL)
  {
    A_BI_body = new_A_BI_body;
  }
  else
  {
    A_BI_body = &body->A_BI;
  }

  if (body->parentId != -1)
  {
    if (new_A_BI_parent != NULL)
    {
      A_BI_parent = new_A_BI_parent;
    }
    else
    {
      A_BI_parent = &self->bodies[body->parentId].A_BI;
    }
  }

  HTr_copy(&A_BP_body, &body->A_BP);
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
  if ((self==NULL) || (body==NULL) || (body->rigid_body_joints==false))
  {
    return false;
  }

  const RcsJoint* bdyJnt = RCSJOINT_BY_ID(self, body->jntId);
  RCHECK(bdyJnt);
  VecNd_copy(&self->q->ele[bdyJnt->jointIndex], angles, 6);

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
    if (JNT->coupledToId != -1)
    {
      const RcsJoint* masterJnt = &self->joints[JNT->coupledToId];

      // Joint positions
      double q_master   = q->ele[masterJnt->jointIndex];
      double q_slave    = RcsJoint_computeSlaveJointAngle(self, JNT, q_master);

      // Compare with original joint angle
      const double eps = 1.0e-8;
      double qErr = fabs(q_slave-q->ele[JNT->jointIndex]);
      if (qErr>eps)
      {
        qHasChanged = true;
      }

      q->ele[JNT->jointIndex] =  q_slave;



      // Joint velocities
      if (q_dot != NULL)
      {
        double q_dot_master = q_dot->ele[masterJnt->jointIndex];
        q_dot->ele[JNT->jointIndex] =
          RcsJoint_computeSlaveJointVelocity(self, JNT, q_master, q_dot_master);
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
    if ((JNT->coupledToId!=-1) && (JNT->constrained==false))
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

    RcsJoint* masterJnt = (JNT->coupledToId!=-1) ? &self->joints[JNT->coupledToId] : NULL;

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
      sensitivity = RcsJoint_computeSlaveJointVelocity(self, JNT, q_master, 1.0);
      MatNd_set2(H, JNT->jacobiIndex, masterJnt->jacobiIndex, sensitivity);
      MatNd_addToEle(colSum, 0, masterJnt->jacobiIndex, sensitivity);
      nCoupledJoints++;
    }

  }

  // nqr is the number of reduced degrees of freedom (coupled dofs)
  int nqr = nq - nCoupledJoints;
  MatNd* invDiagAtA = NULL;
  MatNd_create2(invDiagAtA, nqr, 1);

  MatNd_reshapeAndSetZero(A, nq, nqr);
  MatNd_reshapeAndSetZero(invA, nqr, nq);

  unsigned int nColumsA = 0;

  for (int col=0; col<nq; col++)
  {
    // Compute column sum
    double absColSum = fabs(MatNd_get2(colSum, 0, col));

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
  //MatNd* invA2 = NULL;
  //MatNd_create2(invA2, nqr, nq);
  //MatNd_rwPinv2(invA2, A, NULL, NULL);
  //MatNd_printCommentDigits("invA", invA, 4);
  //MatNd_printCommentDigits("invA2", invA2, 4);
  //MatNd_destroy(invA2);


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

    if (JNT->constrained == false)
    {
      JNT->jacobiIndex = njCount;
      njCount++;
    }
    else
    {
      JNT->jacobiIndex = -1;
    }

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
 * \todo: Check if
 * - RcsGraph_recomputeJointIndices() needs self->q and self->q_dot passed
 * - Why MatNd_set(self->q, JNT->jointIndex, 0, JNT->q0) ?
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

    RcsJoint* master = RcsGraph_getJointByName(self, JNT->coupledJntName);

    if (master == NULL)
    {
      continue;
    }

    JNT->coupledToId = master->jointIndex;

    // Calculate range and initial posture
    double q_min  = RcsJoint_computeSlaveJointAngle(self, JNT, master->q_min);
    double q_max  = RcsJoint_computeSlaveJointAngle(self, JNT, master->q_max);
    double q0     = RcsJoint_computeSlaveJointAngle(self, JNT, master->q0);
    double q_init = RcsJoint_computeSlaveJointAngle(self, JNT, master->q_init);

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
              RcsJoint_computeSlaveJointAngle(self, JNT, master_q));
  }

}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsGraph_fprintXML(FILE* out, const RcsGraph* self)
{
  if (out==NULL)
  {
    RLOG(1, "Can't write graph to NULL xml file");
    return;
  }

  fprintf(out, "<Graph name=\"DefaultPose\" >\n\n");

  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    RcsBody_fprintXML(out, BODY, self);
  }

  RcsGraph_fprintModelState(out, self, self->q);

  fprintf(out, "</Graph>\n");
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool RcsGraph_toXML(const char* fileName, const RcsGraph* self)
{
  if (fileName==NULL)
  {
    RLOG(1, "File name is NULL - can't write xml file");
    return false;
  }

  if (self==NULL)
  {
    RLOG(1, "Graph is NULL - can't write xml file");
    return false;
  }


  FILE* fd = fopen(fileName, "w+");

  if (fd==NULL)
  {
    RLOG(1, "Failed to open file \"%s\" - can't write xml file", fileName);
    return false;
  }

  RcsGraph_fprintXML(fd, self);
  fclose(fd);

  return true;
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
RcsSensor* RcsGraph_getSensorByName(const RcsGraph* graph, const char* name)
{
  RCSGRAPH_FOREACH_SENSOR(graph)
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
int RcsGraph_getSensorIdByName(const RcsGraph* graph, const char* name)
{
  for (unsigned int i=0; i<graph->nSensors; ++i)
  {
    if (STREQ(graph->sensors[i].name, name))
    {
      return i;
    }
  }

  return -1;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsGraph_scale(RcsGraph* graph, double scale)
{
  double origin[3];
  Vec3d_setZero(origin);

  RCSGRAPH_TRAVERSE_BODIES(graph)
  {
    double k_org[3];
    Vec3d_add(k_org, origin, BODY->A_BI.org);
    Vec3d_rotateSelf(k_org, BODY->A_BI.rot);
    RcsBody_scale(graph, BODY, scale);
  }

  RCSGRAPH_TRAVERSE_JOINTS(graph)
  {
    if (RcsJoint_isTranslation(JNT) == true)
    {
      graph->q->ele[JNT->jointIndex] *= scale;
    }

  }

}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool RcsGraph_removeBody(RcsGraph* self, const char* name,
                         MatNd* stateVec[], unsigned int nVec)
{
  RcsBody* bdy = RcsGraph_getBodyByName(self, name);

  if (bdy == NULL)
  {
    RLOG(1, "Couldn't find body \"%s\" to remove - skipping", name);
    return false;
  }

  if (RcsBody_isLeaf(bdy)==false)
  {
    RLOG(1, "Body \"%s\" has children - currently can't be removed", name);
    return false;
  }

  // Now we re-wire the graph

  // 1. parent
  RcsBody* parent = RcsBody_getParent(self, bdy);

  if (parent)
  {
    if (parent->firstChildId == bdy->id)
    {
      parent->firstChildId = bdy->nextId;
    }

    if (parent->lastChildId == bdy->id)
    {
      parent->lastChildId = bdy->prevId;
    }
  }

  RcsBody* prevBdy = RcsBody_getPrev(self, bdy);
  if (prevBdy)
  {
    prevBdy->nextId = bdy->nextId;
  }

  RcsBody* nextBdy = RcsBody_getNext(self, bdy);
  if (nextBdy)
  {
    nextBdy->prevId = bdy->prevId;
  }

  // In case we remove the root, there are a few cases to consider.
  if (bdy->id == self->rootId)
  {
    self->rootId = nextBdy ? nextBdy->id : bdy->firstChildId;

    if (nextBdy)
    {
      self->rootId = nextBdy->id;
      // The children of the previous root must be moved to the root's next.
      if (bdy->firstChildId != -1)
      {
        RFATAL("Not yet implemented");
      }
    }
    else
    {
      self->rootId = bdy->firstChildId;
    }
  }

  // In case the body is attached to its parent by one or more joints, we need
  // to update the joint and Jacobi indices.
  if (bdy->jntId!=-1)
  {
    RcsGraph_recomputeJointIndices(self, stateVec, nVec);
  }

  // To indicate that the body is invalid. Consecutive calls to
  // RcsGraph_insertGraphBody() will use this memory for adding new bodies.
  bdy->id = -1;

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
void RcsGraph_addRandomGeometry(RcsGraph* self)
{
  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    if (STREQ(BODY->name, "BVHROOT"))
    {
      continue;
    }

    RcsBody* CHILD = RCSBODY_BY_ID(self, BODY->firstChildId);

    int rr = Math_getRandomInteger(0, 255);
    int gg = Math_getRandomInteger(0, 255);
    int bb = Math_getRandomInteger(0, 255);
    char color[RCS_MAX_NAMELEN];
    sprintf(color, "#%02x%02x%02xff", rr, gg, bb);


    while (CHILD!=NULL)
    {
      RLOG(5, "%s: Traversing child %s", BODY->name, CHILD->name);

      const double* I_p1 = BODY->A_BI.org;
      const double* I_p2 = CHILD->A_BI.org;

      double K_p1[3], K_p2[3], K_p12[3], K_center[3];
      Vec3d_invTransform(K_p1, &BODY->A_BI,I_p1);
      Vec3d_invTransform(K_p2, &BODY->A_BI,I_p2);
      Vec3d_sub(K_p12, K_p2, K_p1);
      Vec3d_constMulAndAdd(K_center, K_p1, K_p12, 0.5);
      double len = 0.8*Vec3d_getLength(K_p12);

      if (len < 0.01)
      {
        len = 0.01;
      }

      // Box from parent to child
      RcsShape* shape = RcsShape_create();
      shape->type = RCSSHAPE_BOX;
      shape->computeType |= RCSSHAPE_COMPUTE_GRAPHICS;
      shape->extents[0] = 0.2*len;
      shape->extents[1] = 0.2*len;
      shape->extents[2] = len;
      snprintf(shape->color, RCS_MAX_NAMELEN, "%s", color);
      Mat3d_fromVec(shape->A_CB.rot, K_p12, 2);
      Vec3d_copy(shape->A_CB.org, K_center);
      RcsBody_addShape(BODY, shape);

      // Sphere at parent origin
      shape = RcsShape_create();
      shape->type = RCSSHAPE_SPHERE;
      shape->computeType |= RCSSHAPE_COMPUTE_GRAPHICS;
      shape->extents[0] = 0.15*len;
      shape->extents[1] = 0.15*len;
      shape->extents[2] = 0.15*len;
      snprintf(shape->color, RCS_MAX_NAMELEN, "%s", color);
      RcsBody_addShape(BODY, shape);

      CHILD=RCSBODY_BY_ID(self, CHILD->nextId);
    }
  }

}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsGraph_computeAABB(const RcsGraph* self,
                          double xyzMin[3], double xyzMax[3])
{
  if (self == NULL || RcsGraph_numBodies(self)==0)
  {
    RLOG(4, "Graph is NULL or has no shapes - AABB is set to zero");
    Vec3d_setZero(xyzMin);
    Vec3d_setZero(xyzMax);
    return;
  }

  Vec3d_set(xyzMin, DBL_MAX, DBL_MAX, DBL_MAX);
  Vec3d_set(xyzMax, -DBL_MAX, -DBL_MAX, -DBL_MAX);

  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    double C_min[3], C_max[3];
    RcsBody_computeAABB(BODY, C_min, C_max);

    for (int j = 0; j < 3; ++j)
    {
      if (C_min[j] < xyzMin[j])
      {
        xyzMin[j] = C_min[j];
      }

      if (C_max[j] > xyzMax[j])
      {
        xyzMax[j] = C_max[j];
      }
    }
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
RcsMeshData* RcsGraph_meshify(const RcsGraph* self, double scale,
                              char computeType)
{
  if (self==NULL)
  {
    RLOG(4, "Can't meshify NULL graph");
    return NULL;
  }


  RcsMeshData* allMesh = NULL;

  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    RCSBODY_TRAVERSE_SHAPES(BODY)
    {
      if ((SHAPE->computeType&computeType)==0)
      {
        continue;
      }

      RcsMeshData* mesh = RcsShape_createMesh(SHAPE);

      if (mesh==NULL)
      {
        continue;
      }

      HTr A_CI;
      HTr_transform(&A_CI, &BODY->A_BI, &SHAPE->A_CB);
      RcsMesh_transform(mesh, A_CI.org, A_CI.rot);

      if (allMesh==NULL)
      {
        allMesh = RcsMesh_clone(mesh);
      }
      else
      {
        RcsMesh_add(allMesh, mesh);
      }

      RcsMesh_destroy(mesh);
    }
  }

  if ((scale!=1.0) && (allMesh!=NULL))
  {
    RcsMesh_scale(allMesh, scale);
  }

  return allMesh;
}

/*******************************************************************************
 *
 ******************************************************************************/
void RcsGraph_setShapesResizeable(RcsGraph* self, bool resizeable)
{
  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    RCSBODY_TRAVERSE_SHAPES(BODY)
    {
      SHAPE->resizeable = resizeable;
    }
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void RcsGraph_copyResizeableShapes(RcsGraph* dst, const RcsGraph* src,
                                   int level)
{
  const RcsBody* bSrc = &src->bodies[src->rootId];
  RcsBody* bDst = &dst->bodies[dst->rootId];

  do
  {
    RcsShape** sSrc = bSrc->shape;
    RcsShape** sDst = bDst->shape;

    while ((*sSrc) && (*sDst))
    {
      RcsShape* shapeDst = *sDst;

      if (!shapeDst->resizeable)
      {
        sSrc++;
        sDst++;
        continue;
      }

      const RcsShape* shapeSrc = *sSrc;
      Vec3d_copy(shapeDst->extents, shapeSrc->extents);
      shapeDst->computeType = shapeSrc->computeType;

      if (level>0)
      {
        HTr_copy(&shapeDst->A_CB, &shapeSrc->A_CB);
      }

      if (level>1)
      {
        RcsShape_copy(shapeDst, shapeSrc);
      }


      sSrc++;
      sDst++;
    }

    bSrc = RcsBody_depthFirstTraversalGetNextById(src, bSrc);
    bDst = RcsBody_depthFirstTraversalGetNextById(dst, bDst);
  }
  while (bSrc && bDst);
}

/*******************************************************************************
 *
 ******************************************************************************/
RcsBody* RcsGraph_insertGraphBody(RcsGraph* graph, int parentId)
{
  RcsBody* body = NULL;

  // We first look through all existing bodies if one of them has been removed.
  // This can be determined through its id which is -1 in that case. If a
  // removed body is found, we return its memory location, so that we fill the
  // memory blocks of prior removed bodies.
  for (unsigned int i=0; i<graph->nBodies; ++i)
  {
    if (graph->bodies[i].id==-1)
    {
      body = &graph->bodies[i];
      RcsBody_init(body);
      body->id = i;
      break;
    }
  }

  // No free memory block has been found, so we allocate a new one. The call to
  // realloc might possibly change the pointer adress of RcsGraph::bodies, so we
  // need to re-assign the root pointer.
  if (!body)
  {
    graph->nBodies++;
    graph->bodies = RREALLOC(graph->bodies, graph->nBodies, RcsBody);
    RCHECK(graph->bodies);

    // This must come after the realloc() call, since this might change the memory
    // location of the bodies aray.
    body = &graph->bodies[graph->nBodies-1];
    RcsBody_init(body);
    body->id = graph->nBodies-1;
  }

  // Special cases for inserting bodies without a parent. For the root body,
  // there remains nothing to do
  if (parentId == -1)
  {
    // Insert body on the root level as the last "next" neighbour of the root
    if (graph->nBodies>1)
    {
      RcsBody* prev = &graph->bodies[0];
      while (prev->nextId!=-1)
      {
        prev = &graph->bodies[prev->nextId];
      }

      body->id = graph->nBodies-1;
      body->prevId = prev->id;
      prev->nextId = body->id;
    }
  }
  // From here, the body to be inserted has a parent
  else
  {
    RcsBody* parent = &graph->bodies[parentId];

    if (parent->lastChildId != -1)  // parent already has children
    {
      RcsBody* parentsLastChild = &graph->bodies[parent->lastChildId];
      parentsLastChild->nextId = body->id;
      body->prevId = parent->lastChildId;
    }
    else
    {
      parent->firstChildId = body->id;
    }

    body->parentId = parentId;
    parent->lastChildId = body->id;
  }

  return body;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
RcsJoint* RcsGraph_insertGraphJoint(RcsGraph* graph, int bodyId)
{
  RcsBody* body = RCSBODY_BY_ID(graph, bodyId);
  RCHECK(body);

  graph->dof++;
  graph->q = MatNd_realloc(graph->q, graph->dof, 1);
  graph->joints = RREALLOC(graph->joints, graph->dof, RcsJoint);
  RCHECK(graph->joints);
  RcsJoint* newJoint = &graph->joints[graph->dof-1];
  RcsJoint_init(newJoint);
  newJoint->id = graph->dof - 1;
  newJoint->jointIndex = graph->dof - 1;

  // Find last joint of body

  // The joint is the first joint of body
  if (body->jntId == -1)
  {
    body->jntId = newJoint->id;

    // Find last joint of previous body
    if (body->parentId == -1)    // The body has no predecessor
    {
      newJoint->prevId = -1;
    }
    else    // The body has a predecessor
    {
      const RcsJoint* lastJnt = RcsBody_lastJointBeforeBody(graph, &graph->bodies[body->parentId]);
      newJoint->prevId = lastJnt ? lastJnt->id : -1;
    }

  }
  // If the joint is not the first joint of the body, we insert it between the
  // current last joint and the body.
  else   // if(body->jnt==NULL)
  {
    RcsJoint* bdyJoint = RCSJOINT_BY_ID(graph, body->jntId);

    if (bdyJoint->nextId == -1)
    {
      bdyJoint->nextId = newJoint->id;
      newJoint->prevId = bdyJoint->id;
    }
    else
    {
      RcsJoint* prevJnt = bdyJoint;
      while (prevJnt->nextId != -1)
      {
        // Append after last joint
        prevJnt = &graph->joints[prevJnt->nextId];
      }
      prevJnt->nextId = newJoint->id;
      newJoint->prevId = prevJnt->id;
    }
  }  // End of joint being not the first joint of body

  return newJoint;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool RcsGraph_addBodyDofs(RcsGraph* graph, RcsBody* parent, RcsBody* body,
                          MatNd* stateVec[], unsigned int nVec)
{
  if (body == NULL)
  {
    RLOG(1, "Can't append NULL body");
    return false;
  }

  // Connect joints.
  unsigned int nj = RcsBody_numJoints(graph, body);

  if (nj > 0)
  {
    graph->q = MatNd_realloc(graph->q, graph->q->m + nj, 1);
    graph->q_dot = MatNd_realloc(graph->q_dot, graph->q_dot->m + nj, 1);

    for (unsigned int i = 0; i < nVec; ++i)
    {
      stateVec[i] = MatNd_realloc(stateVec[i], graph->q_dot->m + nj, 1);
    }

    RcsGraph_recomputeJointIndices(graph, stateVec, nVec);

    RCSBODY_FOREACH_JOINT(graph, body)
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

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsGraph_copy(RcsGraph* dst, const RcsGraph* src)
{
  dst->rootId = src->rootId;


  // Copy memory block of joints. We only reallocate memory in case the
  // memory increased.
  if (src->dof > dst->dof)
  {
    dst->joints = RREALLOC(dst->joints, src->dof, RcsJoint);
    RCHECK(dst->joints);
  }
  dst->dof = src->dof;
  memcpy(dst->joints, src->joints, dst->dof*sizeof(RcsJoint));


  // Copy bodies. We only reallocate memory in case the memory increased.
  if (src->nBodies > dst->nBodies)
  {
    dst->bodies = RREALLOC(dst->bodies, src->nBodies, RcsBody);
    RCHECK(dst->bodies);
  }
  dst->nBodies = src->nBodies;

  for (unsigned int i=0; i<dst->nBodies; ++i)
  {
    const RcsBody* bSrc = &src->bodies[i];
    RcsBody* bDst = &dst->bodies[i];

    // Determine body corresponding to BODY in the dst graph
    RcsShape** shTmp = bDst->shape;
    memcpy(bDst, bSrc, sizeof(RcsBody));
    bDst->shape = shTmp;

    // Make a copy of all body shapes and attach them to the body
    int nSrcShapes = RcsBody_numShapes(&src->bodies[i]);
    int nDstShapes = RcsBody_numShapes(&dst->bodies[i]);

    if (nDstShapes < nSrcShapes)
    {
      bDst->shape = RREALLOC(bDst->shape, nSrcShapes + 1, RcsShape*);
      RCHECK(bDst->shape);
      memset(&bDst->shape[nDstShapes], 0,
             (nSrcShapes-nDstShapes+1)*sizeof(RcsShape*));
    }

    for (int i = 0; i < nSrcShapes; i++)
    {
      if (bDst->shape[i] == NULL)
      {
        bDst->shape[i] = RALLOC(RcsShape);
      }

      // Block-copy of src shape over dst. We make a temporary copy of the
      // mesh pointer and revert it after the memcpy. It is currently the
      // only pointer in the RcsShape structure.
      RcsMeshData* dstMesh = bDst->shape[i]->mesh;
      memcpy(bDst->shape[i], bSrc->shape[i], sizeof(RcsShape));
      bDst->shape[i]->mesh = dstMesh;

      // In case we have a resizeable shape of mesh type, we also make a deep
      // copy of all vertices and faces. This is needed for instance for soft
      // meshes during physics simulation.
      if ((dstMesh) && (bDst->shape[i]->type==RCSSHAPE_MESH) &&
          (bSrc->shape[i]->mesh) && (bDst->shape[i]->resizeable))
      {
        RcsMesh_copy(dstMesh, bSrc->shape[i]->mesh);
      }
    }

  }   // for (unsigned int i=0; i<dst->nBodies; ++i)


  // Copy sensors. We only reallocate memory in case the memory increased.
  if (src->nSensors > dst->nSensors)
  {
    dst->sensors = RREALLOC(dst->sensors, src->nSensors, RcsSensor);
    RCHECK(dst->sensors);
  }
  dst->nSensors = src->nSensors;

  for (unsigned int i=0; i<dst->nSensors; ++i)
  {
    bool texelsChanged = false;

    if ((dst->sensors[i].type==RCSSENSOR_PPS) &&
        (dst->sensors[i].nTexels!=src->sensors[i].nTexels))
    {
      texelsChanged = true;
    }

    MatNd* tmpRaw = dst->sensors[i].rawData;
    RcsTexel* tmpTex = dst->sensors[i].texel;
    memcpy(&dst->sensors[i], &src->sensors[i], sizeof(RcsSensor));
    dst->sensors[i].rawData = tmpRaw;
    dst->sensors[i].texel = tmpTex;
    MatNd_resizeCopy(&dst->sensors[i].rawData, src->sensors[i].rawData);

    // In the unlikely event of different texel numbers, we perform a deep
    // copy of the src texels over the dst texels. This is only performed
    // for PPS-type sensors. The texelChanged flag is not entirely correct,
    // since it does not cover the case of changed texel geometries while
    // not having a different texel number. Once this becomes an issue, we
    // should fix it. It should also be noted that the memory re-allocation
    // could shrink the array of texels. This might become an issue for
    // classes that initialize the texels once at the beginning and do not
    // check for size changes (e.g. Guis or so).
    if (texelsChanged)
    {
      dst->sensors[i].texel = RREALLOC(dst->sensors[i].texel,
                                       dst->sensors[i].nTexels, RcsTexel);
      RCHECK(dst->sensors[i].texel);
      memcpy(&dst->sensors[i].texel, &src->sensors[i].texel,
             dst->sensors[i].nTexels*sizeof(RcsTexel));
      RCHECK_MSG(dst->sensors[i].nTexels ==
                 dst->sensors[i].rawData->m*dst->sensors[i].rawData->n,
                 "%d != %d (%d x %d)", dst->sensors[i].nTexels,
                 dst->sensors[i].rawData->m*dst->sensors[i].rawData->n,
                 dst->sensors[i].rawData->m, dst->sensors[i].rawData->n);
    }
  }


  // Copy the configuration space arrays and corresponding dimensions
  dst->nJ = src->nJ;
  MatNd_resizeCopy(&dst->q, src->q);
  MatNd_resizeCopy(&dst->q_dot, src->q_dot);
  snprintf(dst->cfgFile, RCS_MAX_FILENAMELEN, "%s", src->cfgFile);


  // Copy the connection information of the Generic Bodies
  memcpy(dst->gBody, src->gBody, RCS_NUM_GENERIC_BODIES*sizeof(int));
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool RcsGraph_appendCopyOfGraph(RcsGraph* self,
                                RcsBody* root,
                                const RcsGraph* other_,
                                const char* suffix,
                                const HTr* A_BP)
{
  if (self == NULL)
  {
    RLOG(1, "Can't append graph to NULL graph");
    return false;
  }

  if (root && (root->firstChildId!=-1))
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

  RcsBody* otherRoot = &other->bodies[other->rootId];
  if (otherRoot->nextId != -1)
  {
    RcsBody* otherRootNext = &other->bodies[otherRoot->nextId];
    RLOG(1, "Currently we can't handle multiple root bodies - your graph has "
         "body \"%s\" next to root (%s)", otherRootNext->name,
         otherRoot->name);
    RcsGraph_destroy(other);
    return false;
  }

  // Adjust indices in other graph's bodies and joints arrays
  RLOG(5, "Adjusting body indices");
  for (unsigned int i=0; i<other->nBodies; ++i)
  {
    RcsBody* bdy = &other->bodies[i];
    bdy->id += (bdy->id==-1) ? 0 : self->nBodies;
    bdy->parentId += (bdy->parentId==-1) ? 0 : self->nBodies;
    bdy->firstChildId += (bdy->firstChildId==-1) ? 0 : self->nBodies;
    bdy->lastChildId += (bdy->lastChildId==-1) ? 0 : self->nBodies;
    bdy->nextId += (bdy->nextId==-1) ? 0 : self->nBodies;
    bdy->prevId += (bdy->prevId==-1) ? 0 : self->nBodies;
    bdy->jntId += (bdy->jntId==-1) ? 0 : self->dof;
  }

  RLOG(5, "Adjusting joint indices");
  for (unsigned int i=0; i<other->dof; ++i)
  {
    RcsJoint* jnt = &other->joints[i];
    jnt->id += (jnt->id==-1) ? 0 : self->dof;
    jnt->prevId += (jnt->prevId==-1) ? 0 : self->dof;
    jnt->nextId += (jnt->nextId==-1) ? 0 : self->dof;
    jnt->coupledToId += (jnt->coupledToId==-1) ? 0 : self->dof;
  }

  // Connect other graph root bodies and joints to root body
  RLOG(5, "Adjusting connectivity");
  const RcsJoint* rootsPrevJnt = RcsBody_lastJointBeforeBody(self, root);
  int prevId = rootsPrevJnt ? rootsPrevJnt->id : -1;

  for (unsigned int i=0; i<other->nBodies; ++i)
  {
    RcsBody* bdy = &other->bodies[i];

    // These are root-level bodies
    if (bdy->parentId == -1)
    {
      bdy->parentId = root ? root->id : -1;
      if (root)
      {
        // Root body has no children: new body is first and last child
        if (root->lastChildId==-1)
        {
          root->firstChildId = bdy->id;
          root->lastChildId = bdy->id;
        }
        else
        {
          RcsBody* rootsLastChild = &self->bodies[root->lastChildId];
          rootsLastChild->nextId = bdy->id;
          bdy->prevId = rootsLastChild->id;
          root->lastChildId = bdy->id;
        }
      }
      else
      {
        // Find last body on the first level
        RcsBody* b = &self->bodies[self->rootId];
        while (b->nextId != -1)
        {
          b = &self->bodies[b->nextId];
        }

        b->nextId = bdy->id;
        bdy->prevId = b->id;
      }

      // Apply relative transformation
      RLOG(5, "Applying relative transformation");
      if (A_BP != NULL)
      {
        HTr_transformSelf(&bdy->A_BP, A_BP);
      }

      RcsJoint* bodyJnt = NULL;
      if (bdy->jntId != -1)
      {
        bodyJnt = &other->joints[bdy->jntId-self->dof];// was added
        bodyJnt->prevId = prevId;
      }
    }   // End root-level bodies

    // Append suffix to body names. Bodies with id -1 are considered to
    // be deleted and don't get a suffix.
    if ((suffix != NULL) && (bdy->id!=-1))
    {
      char newName[RCS_MAX_NAMELEN];
      snprintf(newName, RCS_MAX_NAMELEN, "%s%s", bdy->name, suffix);
      strcpy(bdy->name, newName);
    }

  }



  // Add the suffix to all joints and sensors that come new into the graph
  RLOG(5, "Adjusting suffixes");
  if (suffix)
  {
    // Joints
    for (unsigned int i=0; i<other->dof; ++i)
    {
      RcsJoint* jnt = &other->joints[i];
      if (jnt->id!=-1)
      {
        char newName[RCS_MAX_NAMELEN];
        snprintf(newName, RCS_MAX_NAMELEN, "%s%s", jnt->name, suffix);
        snprintf(jnt->name, RCS_MAX_NAMELEN, "%s", newName);
      }
    }

    // Sensors
    for (unsigned int i=0; i<other->nSensors; ++i)
    {
      RcsSensor* sensor = &other->sensors[i];
      char newName[RCS_MAX_NAMELEN];
      snprintf(newName, RCS_MAX_NAMELEN, "%s%s", sensor->name, suffix);
      snprintf(sensor->name, RCS_MAX_NAMELEN, "%s", newName);
    }
  }

  // Append memory blocks to graph that is extended. When deleting the cloned
  // graph, we need to set the pointer members to NULL so that the below
  // memory blocks gain ownership over the shapes, meshes etc.
  RLOG(5, "Copying memory blocks");
  self->bodies = RREALLOC(self->bodies, self->nBodies+other->nBodies, RcsBody);
  RCHECK(self->bodies);
  memcpy(self->bodies+self->nBodies, other->bodies,
         other->nBodies*sizeof(RcsBody));

  self->joints = RREALLOC(self->joints, self->dof+other->dof, RcsJoint);
  RCHECK(self->joints);
  memcpy(self->joints+self->dof, other->joints, other->dof*sizeof(RcsJoint));

  self->sensors = RREALLOC(self->sensors,
                           self->nSensors+other->nSensors, RcsSensor);
  RCHECK(self->sensors);
  memcpy(self->sensors+self->nSensors, other->sensors,
         other->nSensors*sizeof(RcsSensor));

  RLOG(5, "Adjusting dof from %d to %d", self->dof, self->dof+other->dof);
  self->dof += other->dof;
  self->nBodies += other->nBodies;
  self->nSensors += other->nSensors;



  // We search for the joint index of the last joint before the other graph's
  // root joint
  RLOG(5, "Merging state vectors");
  unsigned int splitIdx = 0;

  // If no body of attachement is given, we add the state vector after all
  // already existing state vector elements.
  if (root == NULL)
  {
    splitIdx = self->dof - other->dof;
  }
  else
  {
    splitIdx = rootsPrevJnt ? rootsPrevJnt->jointIndex + 1 : 0;
  }

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


  RLOG(5, "Setting state");
  int runningJointIndex = 0;
  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    JNT->jointIndex = runningJointIndex++;
  }
  RcsGraph_setState(self, self->q, self->q_dot);

  RLOG(5, "Destroying temporary graph");
  for (unsigned int i=0; i<other->nBodies; ++i)
  {
    other->bodies[i].shape = NULL;
  }

  for (unsigned int i=0; i<other->nSensors; ++i)
  {
    other->sensors[i].texel = NULL;
    other->sensors[i].rawData = NULL;
  }

  RcsGraph_destroy(other);

  RLOG(5, "Done RcsGraph_appendCopyOfGraph()");

  return true;
}
