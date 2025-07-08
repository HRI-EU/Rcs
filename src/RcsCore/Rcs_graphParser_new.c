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

#include "Rcs_graphParser.h"
#include "Rcs_URDFParser.h"
#include "Rcs_graphOpenRAVEParser.h"
#include "Rcs_typedef.h"
#include "Rcs_body.h"
#include "Rcs_shape.h"
#include "Rcs_joint.h"
#include "Rcs_sensor.h"
#include "Rcs_macros.h"
#include "Rcs_utils.h"
#include "Rcs_mesh.h"
#include "Rcs_resourcePath.h"
#include "Rcs_math.h"

#include <float.h>

#define SUFFIX_BACKWARDS

typedef struct
{
  RcsGraph*  graph;        ///< pointer to the target graph
  xmlNodePtr parentGroup;  ///< libxml node of the current <Group>

  // These contents are duplicated when assigning RcsXmlParseCtx child = *parent
  // Modifying the copy does not affect the parent.
  HTr        groupTf;                              ///< Accumulated group transform (value).
  char       defaultColor[RCS_MAX_NAMELEN];        ///< Inherited default colour.
  char       suffixAtGroup[RCSGRAPH_MAX_GROUPDEPTH][RCS_MAX_NAMELEN];
  int        level;                                ///< Current depth in <Group> hierarchy.
  bool       verbose;                              ///< Verbose-logging switch.

} RcsXmlParseCtx;


static void RcsGraph_parseRecursive(xmlNodePtr node, RcsXmlParseCtx* ctx);


const char* getXMLNodePropertyStringPtr(xmlNodePtr node, const char* name)
{
  if (!node || !name)
  {
    return NULL;
  }

  for (xmlAttr* a = node->properties; a; a = a->next)
  {
    if (a->name && xmlStrEqual(a->name, (const xmlChar*)name))
    {
      if (a->children && a->children->type == XML_TEXT_NODE)
      {
        return (const char*) a->children->content;
      }
    }
  }

  return NULL;
}

/*******************************************************************************
 * The node points to an OpenRave file
 ******************************************************************************/
static void parseOpenRaveBody(xmlNodePtr node, RcsGraph* self)
{
  char tmp[RCS_MAX_FILENAMELEN] = "";

  // check if prev tag is provided --> first body of openrave graph will be
  // attached to it
  RcsBody* pB = NULL;
  if (getXMLNodePropertyStringN(node, "prev", tmp, RCS_MAX_FILENAMELEN) > 0)
  {
    pB = RcsGraph_getBodyByName(self, tmp);
    RCHECK_MSG(pB, "Body \"%s\" not found, which was specified as prev for an OpenRave node", tmp);
  }

  // Get filename
  strcpy(tmp, "");
  getXMLNodePropertyStringN(node, "file", tmp, RCS_MAX_FILENAMELEN);

  // check if q0 is provided and read it
  double* q0 = NULL;
  unsigned int nq = 0;
  if (getXMLNodeProperty(node, "q0"))
  {
    RLOG(1, "Found q0 tag --> overriding initial values of OpenRave file");

    // get number of provided q0 values
    char q_str[512];
    getXMLNodePropertyStringN(node, "q0", q_str, 512);
    nq = String_countSubStrings(q_str, " ");

    // read q0 values
    q0 = RNALLOC(nq, double);
    getXMLNodePropertyVecN(node, "q0", q0, nq);

    // convert to radian
    VecNd_constMulSelf(q0, M_PI/180.0, nq);
  }

  // parse OpenRave file
  RcsGraph_createBodiesFromOpenRAVEFile(self, pB, tmp, q0, nq);

  // cleanup
  RFREE(q0);
}

/*******************************************************************************
 *
 ******************************************************************************/
static void parseURDFFile(xmlNodePtr node, RcsGraph* self, const char* suffix)
{
  // check if prev tag is provided --> first body of URDF graph will be
  // attached to it
  char tmp[RCS_MAX_FILENAMELEN] = "";
  RcsBody* pB = NULL;
  if (getXMLNodePropertyStringN(node, "prev", tmp, RCS_MAX_FILENAMELEN) > 0)
  {
    pB = RcsGraph_getBodyByName(self, tmp);
    RCHECK_MSG(pB, "Body \"%s\" not found, which was specified as prev for"
               " an URDF node", tmp);
  }

  // Get filename
  strcpy(tmp, "");
  getXMLNodePropertyStringN(node, "file", tmp, RCS_MAX_FILENAMELEN);
  char filename[RCS_MAX_FILENAMELEN] = "";
  bool urdfExists = Rcs_getAbsoluteFileName(tmp, filename);
  RCHECK_MSG(urdfExists, "Couldn't open urdf file \"%s\"", tmp);
  // parse URDF file

  // New extension = suffix + new group name
  char urdfSuffix[RCS_MAX_NAMELEN] = "", ndExt[RCS_MAX_NAMELEN] = "";
  getXMLNodePropertyStringN(node, "suffix", urdfSuffix, RCS_MAX_NAMELEN);
  strcpy(ndExt, suffix);
  strcat(ndExt, urdfSuffix);

  HTr A_local;
  HTr_setIdentity(&A_local);
  getXMLNodePropertyHTr(node, "transform", &A_local);
  unsigned int dof = 0;
  int urdfRootId = RcsGraph_rootBodyFromURDFFile(self, filename, ndExt,
                                                 &A_local, &dof);
  RCHECK_MSG(urdfRootId != -1, "Couldn't get URDF root from file \"%s\"", filename);
  self->dof += dof;
  self->q = MatNd_realloc(self->q, self->dof, 1);

  RcsBody* urdfRoot = &self->bodies[urdfRootId];

  // There is no direct way to determine whether the root link should be fixed or free.
  // However, we can easily use a rgid_body_joints xml parameter to determine this.
  bool hasRBJTag = getXMLNodeProperty(node, "rigid_body_joints");
  double q_rbj[12];
  VecNd_setZero(q_rbj, 12);
  // parse rigid body joints tag
  unsigned int nRBJTagStr = 0;
  if (hasRBJTag == true)
  {
    urdfRoot->rigid_body_joints = true;
    nRBJTagStr = getXMLNodeNumStrings(node, "rigid_body_joints");

    switch (nRBJTagStr)
    {
      case 1:
        getXMLNodePropertyBoolString(node, "rigid_body_joints",
                                     &urdfRoot->rigid_body_joints);
        break;

      case 6:
        getXMLNodePropertyVecN(node, "rigid_body_joints", q_rbj, 6);

        // convert Euler angles from degrees to radians
        Vec3d_constMulSelf(&q_rbj[3], M_PI / 180.0);
        break;

      case 12:
        getXMLNodePropertyVecN(node, "rigid_body_joints", q_rbj, 12);

        // convert Euler angles from degrees to radians
        Vec3d_constMulSelf(&q_rbj[3], M_PI / 180.0);
        break;

      default:
        RFATAL("Tag \"rigid_body_joints\" of body \"%s\" has %d entries"
               " - should be 6 or 1", urdfRoot->name, nRBJTagStr);
    }

    NLOG(5, "[%s]: Found %d strings in rigid_body_joint tag \"%s\", flag is "
         "%s", urdfRoot->name, nStr, "rigid_body_joints",
         urdfRoot->rigid_body_joints ? "true" : "false");
  }
  // create rigid body joints if requested
  if (urdfRoot->rigid_body_joints)
  {
    RcsJoint* rbj0 = RcsBody_createRBJ(self, urdfRoot, q_rbj);

    // Determine constraint dofs for physics simulation. If a dof is
    // constrained will be interpreted by a "0" in the joint's weightMetric
    // property.
    if (nRBJTagStr == 12)
    {
      unsigned int checkRbjNum = 0;
      RCSJOINT_TRAVERSE_FORWARD(self, rbj0)
      {
        JNT->weightMetric = q_rbj[6 + checkRbjNum];
        checkRbjNum++;
      }
      RCHECK(checkRbjNum == 6);
    }

    // Rigid body joints don't have any relative transformations after
    // construction. If there is a transformation coming from a group, it
    // needs to be applied to the first of the six rigid body joints. We can
    // simply clone it.
    if (HTr_isIdentity(&A_local) == false)
    {
      HTr_copy(&rbj0->A_JP, &A_local);

      // since the group transform was already applied to the body, remove it
      // there again
      HTr_setIdentity(&urdfRoot->A_BP);
    }
  }
  else if (urdfRoot->physicsSim != RCSBODY_PHYSICS_NONE)
  {
    // no rigid body joints - urdf root is fixed to it's parent. Make sure
    // that the physics simulation treats it correctly.
    if (pB != NULL && (pB->physicsSim == RCSBODY_PHYSICS_DYNAMIC ||
                       pB->physicsSim == RCSBODY_PHYSICS_FIXED))
    {
      // or to fixed if the parent is dynamic
      urdfRoot->physicsSim = RCSBODY_PHYSICS_FIXED;
    }
    else
    {
      // set it to kinematic if the parent is kinematic or not participating
      // at all
      urdfRoot->physicsSim = RCSBODY_PHYSICS_KINEMATIC;
    }
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
static bool RcsGraph_parseModelStateDetail(xmlNodePtr node,
                                           const RcsGraph* self,
                                           const char* mdlName,
                                           int mdlTimeStamp,
                                           MatNd* q,
                                           MatNd* changedQ,
                                           MatNd* q_dot,
                                           MatNd* changedQ_dot)
{
  bool foundModelState = false;

  // Node is on <Graph> level, we need to descent one level
  node = node->children;

  MatNd_reshapeAndSetZero(changedQ, self->dof, 1);
  MatNd_reshapeAndSetZero(changedQ_dot, self->dof, 1);
  MatNd_reshape(q, self->dof, 1);
  MatNd_reshape(q_dot, self->dof, 1);

  while (node)
  {

    if (!isXMLNodeNameNoCase(node, "model_state"))
    {
      node = node->next;
      continue;
    }

    char stateName[RCS_MAX_NAMELEN] = "";
    getXMLNodePropertyStringN(node, "model", stateName, RCS_MAX_NAMELEN);

    if (!STREQ(mdlName, stateName))
    {
      node = node->next;
      continue;
    }

    int timeStamp = -1;
    getXMLNodePropertyInt(node, "time_stamp", &timeStamp);

    // Here we catch the case that the xml string is empty.
    if (getXMLNodeBytes(node, "time_stamp")==1)
    {
      timeStamp = -1;
    }

    if ((timeStamp!=mdlTimeStamp) && (mdlTimeStamp!=-1))
    {
      node = node->next;
      continue;
    }

    foundModelState = true;

    xmlNodePtr jntStateNode = node->children;

    // Search for all joints that are kinematically coupled. We store them in
    // tuples [MasterJointIdx - JntId] so that we can later adjust all coupled
    // joints of ones that have been specified in the model_state description.
    MatNd* cpldId = MatNd_create(0, 2);

    for (unsigned int i=0; i<self->dof; ++i)
    {
      const RcsJoint* JNT = &self->joints[i];
      if (JNT->coupledToId != -1)
      {
        double tmp[2] = { (double)self->joints[JNT->coupledToId].jointIndex, (double)JNT->id };
        MatNd cplTmp = MatNd_fromPtr(1, 2, tmp);
        MatNd_appendRows(cpldId, &cplTmp);
      }
    }

    // From here, we go through all joints of the joint state xml description.
    while (jntStateNode)
    {
      if (!isXMLNodeNameNoCase(jntStateNode, "joint_state"))
      {
        jntStateNode = jntStateNode->next;
        continue;
      }

      char name[RCS_MAX_NAMELEN] = "";
      getXMLNodePropertyStringN(jntStateNode, "joint", name, RCS_MAX_NAMELEN);
      const RcsJoint* jnt = RcsGraph_getJointByName(self, name);

      if (!jnt)
      {
        RLOG(4, "Joint \"%s\" not found", name);
        jntStateNode = jntStateNode->next;
        continue;
      }

      double qi, qi_dot;
      bool hasPos = getXMLNodePropertyDouble(jntStateNode, "position", &qi);
      bool hasVel = getXMLNodePropertyDouble(jntStateNode, "velocity", &qi_dot);

      if (hasPos || hasVel)
      {
        if (jnt->coupledToId != -1)
        {
          RLOG(4, "You are setting the state of a kinematically coupled"
               " joint (\"%s\") - this has no effect", jnt->name);
        }

        if (hasPos)
        {
          qi *= RcsJoint_isRotation(jnt) ? (M_PI/180.0) : 1.0;
          q->ele[jnt->jointIndex] = qi;
          changedQ->ele[jnt->jointIndex] = 1.0;
        }

        if (hasVel)
        {
          qi_dot *= RcsJoint_isRotation(jnt) ? (M_PI/180.0) : 1.0;
          q_dot->ele[jnt->jointIndex] = qi_dot;
          changedQ_dot->ele[jnt->jointIndex] = 1.0;
        }

      }   // if (hasPos || hasVel)

      jntStateNode = jntStateNode->next;
    }  // while (jntStateNode != NULL)


    // If any of the coupled joints refers to one that has been specified
    // within the model_state, we adjust it here to match the coupling.
    for (unsigned int i=0; i<cpldId->m; ++i)
    {
      // Index mjidx is the joint index of the joint that the jCpld is
      // coupled to (the "master" joint).
      const int mjidx = lround(MatNd_get(cpldId, i, 0));
      const RcsJoint* jCpld = &self->joints[lround(MatNd_get(cpldId, i, 1))];

      if (changedQ->ele[mjidx]>0.0)
      {
        q->ele[jCpld->jointIndex] =
          RcsJoint_computeSlaveJointAngle(self, jCpld, q->ele[mjidx]);
        changedQ->ele[jCpld->jointIndex] = 1.0;
      }

      if (changedQ_dot->ele[mjidx]>0.0)
      {
        q_dot->ele[jCpld->jointIndex] =
          RcsJoint_computeSlaveJointVelocity(self, jCpld, q->ele[mjidx],
                                             q_dot->ele[mjidx]);
        changedQ_dot->ele[jCpld->jointIndex] = 1.0;
      }

    }

    MatNd_destroy(cpldId);

    // Next model state
    node = node->next;
  }

  return foundModelState;
}

/*******************************************************************************
 *
 ******************************************************************************/
static bool RcsGraph_parseModelState(xmlNodePtr node, RcsGraph* self,
                                     const char* mdlName)
{
  bool success = false;
  node = node->children;

  while (node != NULL)
  {
    if (isXMLNodeNameNoCase(node, "model_state"))
    {
      char stateName[RCS_MAX_NAMELEN] = "";
      getXMLNodePropertyStringN(node, "model", stateName, RCS_MAX_NAMELEN);

      if (STREQ(mdlName, stateName))
      {
        success = true;
        xmlNodePtr jntStateNode = node->children;

        while (jntStateNode != NULL)
        {
          if (isXMLNodeNameNoCase(jntStateNode, "joint_state"))
          {
            char name[RCS_MAX_NAMELEN] = "";
            getXMLNodePropertyStringN(jntStateNode, "joint", name,
                                      RCS_MAX_NAMELEN);
            RcsJoint* jnt = RcsGraph_getJointByName(self, name);
            if (jnt != NULL)
            {
              double q;
              bool hasTag;
              hasTag = getXMLNodePropertyDouble(jntStateNode, "position", &q);

              if (hasTag==true)
              {
                if (jnt->coupledToId != -1)
                {
                  RLOG(4, "You are setting the state of a kinematically coupled"
                       " joint (\"%s\") - this has no effect", jnt->name);
                }

                // We don't overwrite the q_init value here, since it has
                // influence on the coupled joints.
                q *= RcsJoint_isRotation(jnt) ? (M_PI/180.0) : 1.0;
                jnt->q0 = q;
                self->q->ele[jnt->jointIndex] = q;
                NLOG(0, "Overwriting joint %s with position %f", jnt->name, q);

                // Here we search for all joints that are kinematically coupled
                // to the current one, and adjust their settings to the current
                // ones.
                RCSGRAPH_TRAVERSE_JOINTS(self)
                {
                  if (JNT->coupledToId == jnt->id)
                  {
                    q = RcsJoint_computeSlaveJointAngle(self, JNT, q);
                    JNT->q0 = q;
                    self->q->ele[JNT->jointIndex] = q;
                  }
                }

              }   // tag "position"

              hasTag = getXMLNodePropertyDouble(jntStateNode, "velocity", &q);
              if (hasTag)
              {
                q *= RcsJoint_isRotation(jnt) ? (M_PI/180.0) : 1.0;
                self->q_dot->ele[jnt->jointIndex] = q;
                NLOG(5, "Overwriting joint %s with velocity %f", jnt->name, q);
              }
            }  // if (jnt != NULL)
            else
            {
              RLOG(4, "Joint \"%s\" not found", name);
            }
          }

          jntStateNode = jntStateNode->next;
        }  // while (jntStateNode != NULL)

      }   // if (STREQ(stateName, mdlStateName))

    }   // if (isXMLNodeNameNoCase(node, "model_state"))

    node = node->next;
  }

  return success;
}


/*******************************************************************************
 * Allocates memory and initializes a RcsSensor data structure from
 *        an XML node. Here's the parsed tags:
 *        - name
 *        - type: LOADCELL, JOINTTORQUE, CONTACTFORCE, PPS
 *        - transform
 ******************************************************************************/
static RcsSensor* RcsSensor_initFromXML(xmlNode* node, RcsBody* parentBody,
                                        RcsGraph* graph)
{
  // Return if node is not a sensor node. This can deal with a NULL node
  if (!isXMLNodeName(node, "Sensor"))
  {
    return NULL;
  }

  // read sensor name
  char name[RCS_MAX_NAMELEN] = "unnamed sensor";
  getXMLNodePropertyStringN(node, "name", name, RCS_MAX_NAMELEN);
  RLOG(5, "found new sensor node \"%s\" attached to body \"%s\"",
       name, parentBody ? parentBody->name : "NULL");

  // read sensor type
  char buffer[RCS_MAX_NAMELEN];
  int xml_type = -1;
  strcpy(buffer, "unknown type");
  getXMLNodePropertyStringN(node, "type", buffer, RCS_MAX_NAMELEN);

  if (STRCASEEQ(buffer, "LOADCELL"))
  {
    xml_type = RCSSENSOR_LOAD_CELL;
  }
  else if (STRCASEEQ(buffer, "JOINTTORQUE"))
  {
    xml_type = RCSSENSOR_JOINT_TORQUE;
  }
  else if (STRCASEEQ(buffer, "CONTACTFORCE"))
  {
    xml_type = RCSSENSOR_CONTACT_FORCE;
  }
  else if (STRCASEEQ(buffer, "PPS"))
  {
    xml_type = RCSSENSOR_PPS;
  }
  else
  {
    RFATAL("Unknown sensor type \"%s\"", buffer);
  }

  // read relative offset transformation
  HTr A_SB;
  HTr_setIdentity(&A_SB);
  if (getXMLNodeProperty(node, "transform"))
  {
    getXMLNodePropertyHTr(node, "transform", &A_SB);
  }

  // Create and return rcs sensor object
  RcsSensor* sensor = RcsGraph_insertSensor(graph);
  RcsSensor_init(sensor, xml_type, name, parentBody, &A_SB);



  // Create texels
  if (sensor->type==RCSSENSOR_PPS)
  {
    unsigned int xy[2] = { sensor->rawData->m, sensor->rawData->n };

    getXMLNodePropertyUnsignedIntN(node, "dimensions", xy, 2);
    sensor->rawData = MatNd_realloc(sensor->rawData, xy[0], xy[1]);

    double extents[3];
    Vec3d_setZero(extents);
    getXMLNodePropertyVec3(node, "extents", extents);

    node = node->children;

    // Allocate memory for shape node lists
    sensor->nTexels = getNumXMLNodes(node, "Texel");
    sensor->texel = RNALLOC(sensor->nTexels+1, RcsTexel);

    int texelCount = 0;

    while (node != NULL)
    {
      if (isXMLNodeName(node, "Texel"))
      {
        RcsTexel* texel = &sensor->texel[texelCount];
        getXMLNodePropertyVec3(node, "position", texel->position);
        getXMLNodePropertyVec3(node, "normal", texel->normal);
        Vec3d_copy(texel->extents, extents);
        getXMLNodePropertyVec3(node, "extents", texel->extents);
        texelCount++;
        REXEC(4)
        {
          if (Vec3d_sqrLength(texel->extents)==0.0)
          {
            RMSG("Found zero size texel in sensor \"%s\"", sensor->name);
          }
        }
      }

      node = node->next;
    }

    if (texelCount>0)
    {
      RCHECK_MSG(xy[0]*xy[1]==texelCount, "[%s]: %d * %d != %d", sensor->name,
                 xy[0], xy[1], texelCount);
    }
  }   // End create texels



  return sensor;
}

/*******************************************************************************
 * Shape for distance computation.
 ******************************************************************************/
static void RcsBody_initShape(RcsShape* shape, xmlNodePtr node,
                              const RcsBody* body, const char* bodyColor)
{
  // Allocate memory and set defaults
  char str[RCS_MAX_FILENAMELEN] = "";
  getXMLNodePropertyStringN(node, "type", str, RCS_MAX_FILENAMELEN);
  if (STREQ(str, "SSL"))
  {
    shape->type = RCSSHAPE_SSL;
  }
  else if (STREQ(str, "SSR"))
  {
    shape->type = RCSSHAPE_SSR;
  }
  else if (STREQ(str, "BOX"))
  {
    shape->type = RCSSHAPE_BOX;
  }
  else if (STREQ(str, "CYLINDER"))
  {
    shape->type = RCSSHAPE_CYLINDER;
  }
  else if (STREQ(str, "MESH"))
  {
    shape->type = RCSSHAPE_MESH;
  }
  else if (STREQ(str, "FRAME"))
  {
    shape->type = RCSSHAPE_REFFRAME;
  }
  else if (STREQ(str, "SPHERE"))
  {
    shape->type = RCSSHAPE_SPHERE;
  }
  else if (STREQ(str, "CONE"))
  {
    shape->type = RCSSHAPE_CONE;
  }
  else if (STREQ(str, "TORUS"))
  {
    shape->type = RCSSHAPE_TORUS;
  }
  else if (STREQ(str, "OCTREE"))
  {
    shape->type = RCSSHAPE_OCTREE;
  }
  else if (STREQ(str, "POINT"))
  {
    shape->type = RCSSHAPE_POINT;
  }
  else
  {
    RMSG("Unknown shape type \"%s\"", str);
  }

  if (shape->type == RCSSHAPE_REFFRAME)
  {
    // set default extents
    shape->extents[0] = 0.9;
    shape->extents[1] = 0.9;
    shape->extents[2] = 0.9;
  }

  getXMLNodePropertyVec3(node, "extents", shape->extents);
  getXMLNodePropertyDouble(node, "radius", &shape->extents[0]);
  getXMLNodePropertyDouble(node, "length", &shape->extents[2]);
  getXMLNodePropertyHTr(node, "transform", &shape->A_CB);

  unsigned int scaleDim = getXMLNodeNumStrings(node, "scale");

  if (scaleDim > 0)
  {
    if (scaleDim==1)
    {
      double scale1d = 1.0;
      getXMLNodePropertyDouble(node, "scale", &scale1d);
      Vec3d_setElementsTo(shape->scale3d, scale1d);
    }
    else if (scaleDim==3)
    {
      getXMLNodePropertyVec3(node, "scale", shape->scale3d);
    }
    else
    {
      RLOG(1, "Non-supported number of entries in attribute \"scale\": %d "
           "- should be 1 or 3", scaleDim);
    }
  }



  // check if the from2Points tag exists and if yes, transform is not allowed
  // to exist
  if (getXMLNodeProperty(node, "from2Points"))
  {
    bool success = !getXMLNodeProperty(node, "length");
    RCHECK_MSG(success, "\"length\" is not "
               "allowed if \"from2Points\" exists.");

    // transformation from 2 points
    HTr trans_from_2_points;
    double points[6];
    getXMLNodePropertyVecN(node, "from2Points", points, 6);
    HTr_from2Points(&trans_from_2_points, &points[0], &points[3]);
    HTr_transformSelf(&shape->A_CB, &trans_from_2_points);

    // length from two points
    double points_vec[3];
    Vec3d_sub(points_vec, &points[0], &points[3]);
    shape->extents[2] = Vec3d_getLength(points_vec);
  }

  // check if the quat tag exists and if yes, transform is not allowed
  if (getXMLNodeProperty(node, "quat"))
  {
    bool success = !getXMLNodeProperty(node, "transform");
    RCHECK_MSG(success, "\"transform\" is not "
               "allowed if \"quat\" exists.");

    success = getXMLNodePropertyQuat(node, "quat", shape->A_CB.rot);
    Vec3d_setZero(shape->A_CB.org);
    getXMLNodePropertyVec3(node, "pos", shape->A_CB.org);
  }



  // Compute type
  bool distance = true, graphics = true, physics = true, softPhysics = false;
  bool depth=false, rgb=false, contact=false, attachment = false;
  bool weldpos=false, weldori=false, marker=false, wireframe=false;
  bool boundingbox = true;

  // Physics and distance computation is not carried out for meshes by default.
  if (shape->type == RCSSHAPE_MESH)
  {
    physics = false;
    distance = false;
  }

  // Only meshes are added to the graphics node by default
  if (shape->type != RCSSHAPE_MESH)
  {
    graphics = false;
  }

  // Reference frames are only considered for the graphics nodes by default
  if (shape->type == RCSSHAPE_REFFRAME)
  {
    graphics = true;
    distance = false;
    physics = false;
  }

  // Bounding box default is the same as distance flag
  boundingbox = distance;

  getXMLNodePropertyBoolString(node, "distance", &distance);
  getXMLNodePropertyBoolString(node, "physics", &physics);
  getXMLNodePropertyBoolString(node, "graphics", &graphics);
  getXMLNodePropertyBoolString(node, "softPhysics", &softPhysics);
  getXMLNodePropertyBoolString(node, "render_depth", &depth);
  getXMLNodePropertyBoolString(node, "render_material", &rgb);
  getXMLNodePropertyBoolString(node, "contact", &contact);
  getXMLNodePropertyBoolString(node, "attachment", &attachment);
  getXMLNodePropertyBoolString(node, "weldpos", &weldpos);
  getXMLNodePropertyBoolString(node, "weldori", &weldori);
  getXMLNodePropertyBoolString(node, "marker", &marker);
  getXMLNodePropertyBoolString(node, "wireframe", &wireframe);
  getXMLNodePropertyBoolString(node, "boundingbox", &boundingbox);

  // Physics computation is not carried out for non-physics objects by default.
  if (body->physicsSim == RCSBODY_PHYSICS_NONE)
  {
    physics = false;
  }

  // Color
  if (bodyColor)
  {
    strcpy(shape->color, bodyColor);
  }

  getXMLNodePropertyStringN(node, "color", shape->color, RCS_MAX_NAMELEN);

  // Material
  strcpy(shape->material, "default");
  getXMLNodePropertyStringN(node, "material", shape->material, RCS_MAX_NAMELEN);

  bool resizeable = false;
  getXMLNodePropertyBoolString(node, "resizeable", &resizeable);
  RcsShape_setComputeType(shape, RCSSHAPE_COMPUTE_RESIZEABLE, resizeable);

  if (distance == true)
  {
    shape->computeType |= RCSSHAPE_COMPUTE_DISTANCE;
  }

  if (physics == true)
  {
    shape->computeType |= RCSSHAPE_COMPUTE_PHYSICS;
  }

  if (graphics == true)
  {
    shape->computeType |= RCSSHAPE_COMPUTE_GRAPHICS;
  }

  if (softPhysics == true)
  {
    shape->computeType |= RCSSHAPE_COMPUTE_SOFTPHYSICS;
    shape->computeType |= RCSSHAPE_COMPUTE_RESIZEABLE;
  }

  if (depth == true)
  {
    shape->computeType |= RCSSHAPE_COMPUTE_DEPTHBUFFER;
  }

  if (rgb == true)
  {
    shape->computeType |= RCSSHAPE_COMPUTE_RGBBUFFER;
  }

  if (contact == true)
  {
    shape->computeType |= RCSSHAPE_COMPUTE_CONTACT;

    double mu = 1.0, stiffness = 2.0e4, z0 = 0.0;
    getXMLNodePropertyDouble(node, "staticFriction", &mu);
    getXMLNodePropertyDouble(node, "stiffness", &stiffness);
    getXMLNodePropertyDouble(node, "height", &z0);
    Vec3d_set(shape->scale3d, mu, stiffness, z0);
  }

  if (attachment == true)
  {
    shape->computeType |= RCSSHAPE_COMPUTE_ATTACHMENT;
    double stiffness = 2.0e4;
    getXMLNodePropertyDouble(node, "stiffness", &stiffness);
    double damping = 0.5*sqrt(4.0*stiffness);
    getXMLNodePropertyDouble(node, "damping", &damping);
    Vec3d_set(shape->scale3d, stiffness, damping, 0.0);
    getXMLNodePropertyStringN(node, "body", shape->meshFile, RCS_MAX_FILENAMELEN);
  }

  if (weldpos == true)
  {
    shape->computeType |= RCSSHAPE_COMPUTE_WELDPOS;
    getXMLNodePropertyDouble(node, "kp", &shape->scale3d[0]);
  }

  if (weldori == true)
  {
    shape->computeType |= RCSSHAPE_COMPUTE_WELDORI;
    getXMLNodePropertyDouble(node, "kp", &shape->scale3d[0]);
  }

  if (weldpos || weldori)
  {
    getXMLNodePropertyStringN(node, "refBdy", shape->material, RCS_MAX_FILENAMELEN);
  }

  if (marker == true)
  {
    shape->computeType |= RCSSHAPE_COMPUTE_MARKER;
    RCHECK(shape->type == RCSSHAPE_REFFRAME);
    getXMLNodePropertyStringN(node, "markerName", shape->material, RCS_MAX_FILENAMELEN);
  }

  if (wireframe == true)
  {
    shape->computeType |= RCSSHAPE_COMPUTE_WIREFRAME;
  }

  if (boundingbox == true)
  {
    shape->computeType |= RCSSHAPE_COMPUTE_BOUNDINGBOX;
  }



  // Mesh file
  int strLength = getXMLNodeBytes(node, "meshFile");

  if (strLength > 0)
  {
    char fileName[RCS_MAX_FILENAMELEN] = "";
    char fullName[RCS_MAX_FILENAMELEN] = "";
    RCHECK((shape->type == RCSSHAPE_MESH) || (shape->type == RCSSHAPE_OCTREE));
    getXMLNodePropertyStringN(node, "meshFile", fileName, RCS_MAX_FILENAMELEN);
    Rcs_getAbsoluteFileName(fileName, fullName);

    if (File_exists(fullName))
    {
      snprintf(shape->meshFile, RCS_MAX_FILENAMELEN, "%s", fullName);

      // If the mesh is to be rendered in RGB, we leave the visual mesh
      // to be creted by the RcsGraphics library.
      if ((shape->type==RCSSHAPE_MESH) &&
          (!RcsShape_isOfComputeType(shape, RCSSHAPE_COMPUTE_RGBBUFFER)))
      {
        RcsMeshData* mesh = RcsMesh_createFromFile(shape->meshFile);
        if (!mesh)
        {
          RLOG(4, "Failed to add mesh \"%s\" to shape", shape->meshFile);
        }
        else if ((shape->scale3d[0]!=1.0) || (shape->scale3d[1]!=1.0) || (shape->scale3d[2]!=1.0))
        {
          RcsMesh_scale3D(mesh, shape->scale3d);
        }
        shape->mesh = mesh;
      }
    }
    else
    {
      RLOG(4, "[%s]: Mesh file \"%s\" (\"%s\") not found!",
           body->name, fileName, fullName);
      REXEC(4)
      {
        Rcs_printResourcePath();
      }
    }

    if (shape->type == RCSSHAPE_OCTREE)
    {
      shape->mesh = (RcsMeshData*)RcsShape_addOctree(shape, shape->meshFile);
      if (shape->mesh == NULL)
      {
        RLOG(1, "Failed to load Octree file \"%s\"", shape->meshFile);
      }
    }

  }

  // Texture file
  strLength = getXMLNodeBytes(node, "textureFile");

  if (strLength > 0)
  {
    getXMLNodePropertyStringN(node, "textureFile", str, RCS_MAX_FILENAMELEN);
    char fullname[RCS_MAX_FILENAMELEN];
    if (Rcs_getAbsoluteFileName(str, fullname))
    {
      snprintf(shape->textureFile, RCS_MAX_FILENAMELEN, "%s", fullname);
    }
    else
    {
      RLOG(4, "Texture file \"%s\" in body \"%s\" not found!", str, body->name);
    }

  }

  // Some pedantic checking on tags that might lead to mistakes
  REXEC(1)
  {
    // Lets be pedantic with the configuration file: Disallow "extents"
    // and require "radius" and "height" for SSLs and cylinders
    if (((shape->type == RCSSHAPE_SSL) || (shape->type == RCSSHAPE_CYLINDER)) &&
        (getXMLNodeProperty(node, "from2Points") == false))
    {
      bool success = getXMLNodeProperty(node, "length");
      RCHECK_MSG(success, "%s has no \"length\" tag", body->name);
      success = getXMLNodeProperty(node, "radius");
      RCHECK_MSG(success, "%s has no \"radius\" tag", body->name);
      success = !getXMLNodeProperty(node, "extents");
      RCHECK_MSG(success, "%s has \"extents\" tag but expects \"length\"",
                 body->name);
    }

    // Lets be pedantic with the configuration file: Disallow "extents"
    // and "height" but require "radius" for spheres
    if (shape->type == RCSSHAPE_SPHERE)
    {
      bool success = !getXMLNodeProperty(node, "length");
      RCHECK_MSG(success, "Found length specifier in sphere: %s", body->name);
      success = getXMLNodeProperty(node, "radius");
      RCHECK_MSG(success, "%s", body->name);
      success = !getXMLNodeProperty(node, "extents");
      RCHECK_MSG(success, "Found extents specifier in sphere: %s", body->name);
    }

    // Lets be pedantic with the configuration file: Disallow "radius"
    // and "height" for SSRs
    if (shape->type == RCSSHAPE_SSR)
    {
      bool success = !getXMLNodeProperty(node, "length");
      RCHECK_MSG(success, "SSR of body \"%s\" has length tag!", body->name);
      success = !getXMLNodeProperty(node, "radius");
      RCHECK_MSG(success, "SSR of body \"%s\" has radius tag!", body->name);
    }

    // Lets be pedantic with the configuration file: Disallow "extents"
    // and force "radius" and "length" for TORUS
    if (shape->type == RCSSHAPE_TORUS)
    {
      bool success = getXMLNodeProperty(node, "length");
      RCHECK_MSG(success, "TORUS of body \"%s\" has not length!", body->name);
      success = getXMLNodeProperty(node, "radius");
      RCHECK_MSG(success, "TORUS of body \"%s\" has no radius tag!",
                 body->name);
      success = !getXMLNodeProperty(node, "extents");
      RCHECK_MSG(success, "TORUS of body \"%s\" has extents tag", body->name);
    }

  }   // REXEC(1)

}

/*******************************************************************************
* Joint initialization function.
******************************************************************************/
static RcsJoint* RcsBody_initJoint(RcsGraph* self,
                                   RcsBody* b,
                                   xmlNodePtr node,
                                   const char* suffix,
                                   const HTr* A_group)
{
  char msg[RCS_MAX_NAMELEN];
  double ka[3];
  bool verbose = false;
  unsigned int strLength = 0;

  RCHECK(HTr_isValid(A_group));

  RcsJoint* jnt = RcsGraph_insertGraphJoint(self, b->id);

  if (verbose == true)
  {
    RMSG("Body %s: inserting joint with id = %d", b->name, jnt->id);
  }

  //  Joint name
  strLength = getXMLNodeBytes(node, "name");

  if (strLength > 0)
  {
    char tmp[RCS_MAX_NAMELEN]="";
    getXMLNodePropertyStringN(node, "name", tmp, RCS_MAX_NAMELEN);
    snprintf(jnt->name, RCS_MAX_NAMELEN, "%s%s", tmp, suffix);
  }
  else
  {
    static int uniqueId = 0;
    snprintf(jnt->name, RCS_MAX_NAMELEN, "unnamed joint %d", uniqueId++);
    RLOG(5, "A joint of body \"%s\" has no name - using \"%s\"",
         b->name, jnt->name);
  }

  NLOG(0, "Inserted Joint into Graph: name=%s id=%d prevId=%d nextId=%d",
       jnt->name, jnt->id, jnt->prevId, jnt->nextId);

  REXEC(5)
  {
    if (jnt->prevId!=-1)
    {
      RcsJoint* pjnt = RCSJOINT_BY_ID(self, jnt->prevId);
      RCHECK(pjnt);
      RLOG(5, "   prev Joint: name=%s id=%d prevId=%d nextId=%d",
           pjnt->name, pjnt->id, pjnt->prevId, pjnt->nextId);
    }
  }

  // Relative transformation from prev. body to joint (in prev. body coords)
  getXMLNodePropertyHTr(node, "transform", &jnt->A_JP);

  // check if the quat tag exists and if yes, transform is not allowed
  if (getXMLNodeProperty(node, "quat"))
  {
    bool success = !getXMLNodeProperty(node, "transform");
    RCHECK_MSG(success, "\"transform\" is not allowed if \"quat\" exists.");
    getXMLNodePropertyQuat(node, "quat", jnt->A_JP.rot);
    getXMLNodePropertyVec3(node, "pos", jnt->A_JP.org);
  }

  if (b->jntId == jnt->id) // the joint is the first joint of the body
  {
    // Here we apply the groups transform to the first joint of the body.
    HTr_transformSelf(&jnt->A_JP, A_group);
    if (verbose && (HTr_isIdentity(A_group) == false))
    {
      RMSG("Applied group transform to joint \"%s\"", jnt->name);
    }
  }

  // Joint constraint
  getXMLNodePropertyBoolString(node, "constraint", &jnt->constrained);

  // Joint type
  if (getXMLNodePropertyStringN(node, "type", msg, RCS_MAX_NAMELEN))
  {
    if (verbose)
    {
      RMSG("%s: Joint (%s):   ", b->name, msg);
    }

    if (STREQ(msg, "TransX"))
    {
      jnt->type = RCSJOINT_TRANS_X;
      jnt->dirIdx = 0;
    }
    else if (STREQ(msg, "TransY"))
    {
      jnt->type = RCSJOINT_TRANS_Y;
      jnt->dirIdx = 1;
    }
    else if (STREQ(msg, "TransZ"))
    {
      jnt->type = RCSJOINT_TRANS_Z;
      jnt->dirIdx = 2;
    }
    else if (STREQ(msg, "RotX"))
    {
      jnt->type = RCSJOINT_ROT_X;
      jnt->dirIdx = 0;
    }
    else if (STREQ(msg, "RotY"))
    {
      jnt->type = RCSJOINT_ROT_Y;
      jnt->dirIdx = 1;
    }
    else if (STREQ(msg, "RotZ"))
    {
      jnt->type = RCSJOINT_ROT_Z;
      jnt->dirIdx = 2;
    }
    else
    {
      RFATAL("Joint \"%s\": Unknown joint type \"%s\"", jnt->name, msg);
    }

  }

  // Joint range (must go after joint type)

  unsigned int rangeEle = getXMLNodeNumStrings(node, "range");

  //if (getXMLNodePrope(rtyVec3(node, "range", ka))
  if ((rangeEle==1) || (rangeEle==2) || (rangeEle==3))
  {
    bool hasRange23 = getXMLNodePropertyVecN(node, "range", ka, rangeEle);
    RCHECK(hasRange23);

    jnt->q_min = ka[0];

    if (rangeEle==3)
    {
      jnt->q0 = ka[1];
      jnt->q_max = ka[2];
    }
    else if (rangeEle==2)
    {
      jnt->q0 = 0.5*(ka[1]+ka[0]);
      jnt->q_max = ka[1];
    }
    else if (rangeEle==1)
    {
      jnt->q_min = -ka[0];
      jnt->q0 = 0.0;
      jnt->q_max = ka[0];
    }

    if (RcsJoint_isRotation(jnt) == true)
    {
      jnt->q_min *= (M_PI / 180.0);
      jnt->q0 *= (M_PI / 180.0);
      jnt->q_max *= (M_PI / 180.0);
    }

    jnt->q_init = jnt->q0;
  }
  else
  {
    if (getXMLNodeProperty(node, "coupledTo") == false)
    {
      RFATAL("Joint \"%s\" has no range and is not coupled to another "
             "joint!", jnt->name);
    }

    jnt->q_init = 0.0;
  }


  // Joint weight. That's used to multply the joint limit gradient,
  // such redistributing the joint speeds in the inverse kinematics
  // computation. A larger weight leads to a relatively higher speed.
  REXEC(4)
  {
    bool success = !getXMLNodeProperty(node, "weight");
    RCHECK_MSG(success,
               "Tag \"weight\" has changed to \"weightJL\" - please update "
               "your xml file (body \"%s\", joint \"%s\")",
               b->name, jnt->name);
  }

  // Joint weight. That's used to multply the joint limit gradient,
  // such redistributing the joint speeds in the inverse kinematics
  // computation. A larger weight leads to a relatively higher speed.
  jnt->weightJL = 1.0;
  getXMLNodePropertyDouble(node, "weightJL", &jnt->weightJL);

  // Proximity weight. That's used to multply the collision gradient,
  // such redistributing the joint speeds in the inverse kinematics
  // computation. A larger weight leads to a relatively higher speed.
  jnt->weightCA = 1.0;
  getXMLNodePropertyDouble(node, "weightCA", &jnt->weightCA);

  // Metric weight. That's used to pre-multply the weighting matrix,
  // such redistributing the joint speeds in the inverse kinematics
  // computation.
  jnt->weightMetric = 1.0;
  getXMLNodePropertyDouble(node, "weightMetric", &jnt->weightMetric);

  // Joint controller type (pos, vel, torque)
  jnt->ctrlType = RCSJOINT_CTRL_POSITION;
  jnt->maxTorque = DBL_MAX; // Default: No limit for non-torque joints

  if (getXMLNodePropertyStringN(node, "ctrlType", msg, RCS_MAX_NAMELEN))
  {
    if (STREQ(msg, "Position") || STREQ(msg, "pos"))
    {
      jnt->ctrlType = RCSJOINT_CTRL_POSITION;
    }
    else if (STREQ(msg, "Velocity") || STREQ(msg, "vel"))
    {
      jnt->ctrlType = RCSJOINT_CTRL_VELOCITY;
    }
    else if (STREQ(msg, "Torque") || STREQ(msg, "tor"))
    {
      jnt->maxTorque = 1.0; // Default: 1 Nm for torque joints
      jnt->ctrlType = RCSJOINT_CTRL_TORQUE;
    }
    else
    {
      RFATAL("Joint \"%s\": Unknown joint controller type (ctrlType) \"%s\"",
             jnt->name, msg);
    }
  }

  // Max. joint torque
  getXMLNodePropertyDouble(node, "torqueLimit", &jnt->maxTorque);
  RCHECK(jnt->maxTorque >= 0.0);

  // Speed limit
  jnt->speedLimit = DBL_MAX;
  getXMLNodePropertyDouble(node, "speedLimit", &jnt->speedLimit);

  // Acceleration limit
  jnt->accLimit = DBL_MAX;
  getXMLNodePropertyDouble(node, "accelerationLimit", &jnt->accLimit);

  // Deceleration limit, default is acceleration limit
  jnt->decLimit = jnt->accLimit;
  getXMLNodePropertyDouble(node, "decelerationLimit", &jnt->decLimit);

  // Gear ratio
  double gearRatio = 1.0;

  if (getXMLNodePropertyDouble(node, "gearRatio", &gearRatio) == true)
  {
    RCHECK_MSG(gearRatio >= 0.0,
               "Joint gear ratio of \"%s\" is negative (%f) - exiting",
               jnt->name, gearRatio);
    const double rpm2rad = M_PI / 30.0;
    jnt->speedLimit *= rpm2rad / gearRatio;
  }

  // If the joint is of type rotation and no gear ratio is given,
  // we convert the value to radians (degrees assumed).
  if ((gearRatio == 1.0) && RcsJoint_isRotation(jnt))
  {
    if (jnt->speedLimit != DBL_MAX)
    {
      jnt->speedLimit *= (M_PI / 180.0);
    }

    if (jnt->accLimit != DBL_MAX)
    {
      jnt->accLimit *= (M_PI / 180.0);
    }

    if (jnt->decLimit != DBL_MAX)
    {
      jnt->decLimit *= (M_PI / 180.0);
    }
  }

  // Coupled joint
  strLength = getXMLNodeBytes(node, "coupledTo");

  if (strLength > 0)
  {
    getXMLNodePropertyStringN(node, "coupledTo", jnt->coupledJntName,
                              RCS_MAX_NAMELEN);
    strcat(jnt->coupledJntName, suffix);
    RLOG(5, "Joint %s is coupled to %s", jnt->name, jnt->coupledJntName);

    unsigned int polyGrad = getXMLNodeNumStrings(node, "couplingFactor");
    if (polyGrad == 0)
    {
      polyGrad = 1;
    }
    RLOG(5, "Coupled joint \"%s\" has %d parameters", jnt->name, polyGrad);
    RCHECK_MSG((polyGrad == 1) || (polyGrad == 5) || (polyGrad == 9),
               "Currently only polynomials of order 1 or 5 or 9 are "
               "supported, and not %d parameters", polyGrad);
    RCHECK(polyGrad<=8);
    jnt->nCouplingCoeff = polyGrad;
    getXMLNodePropertyVecN(node, "couplingFactor", jnt->couplingPoly, polyGrad);

    // Check if a range is given, because it will later be overwritten
    bool hasRangeTag = getXMLNodeProperty(node, "range");
    if (hasRangeTag == true)
    {
      RLOG(5, "Joint \"%s\" has a range, even though it is coupled to "
           "another joint (\"%s\")", jnt->name, jnt->coupledJntName);
    }
  }

  if (verbose)
  {
    RPAUSE();
  }

  return jnt;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool RcsGraph_setModelStateFromXML(RcsGraph* self, const char* modelStateName,
                                   int timeStamp)
{
  if ((!self) || (!modelStateName))
  {
    RLOG(4, "Graph or model state name are NULL");
    return false;
  }

  // Read XML file
  xmlDocPtr doc = NULL;
  xmlNodePtr node = parseXMLFile(self->cfgFile, "Graph", &doc);

  if (!node)
  {
    RLOG(4, "Failed to read xml file \"%s\"", self->cfgFile);
    xmlFreeDoc(doc);
    return false;
  }

  bool success = RcsGraph_parseModelState(node, self, modelStateName);
  if (!success)
  {
    RLOG(1, "Failed to parse model_state \"%s\"", modelStateName);
  }

  xmlFreeDoc(doc);

  return success;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool RcsGraph_getModelStateFromXML(MatNd* q, const RcsGraph* self,
                                   const char* modelStateName, int timeStamp)
{
  if ((!self) || (!modelStateName))
  {
    return false;
  }

  // Read XML file
  xmlDocPtr doc;
  xmlNodePtr node = parseXMLFile(self->cfgFile, "Graph", &doc);

  if (!node)
  {
    xmlFreeDoc(doc);
    return false;
  }

  MatNd* q_dot = MatNd_createLike(self->q);
  MatNd* changedQ = MatNd_createLike(self->q);
  MatNd* changedQ_dot = MatNd_createLike(self->q);
  bool success = RcsGraph_parseModelStateDetail(node, self, modelStateName,
                                                timeStamp, q, changedQ, q_dot,
                                                changedQ_dot);
  MatNd_destroyN(3, q_dot, changedQ, changedQ_dot);

  xmlFreeDoc(doc);

  return success;
}
































/*******************************************************************************
 * Allocates memory and initializes a RcsBody data structure from an XML node.
 ******************************************************************************/
// That's a pretty inefficient way of doing it and it should be improved.
static const RcsBody* findBodyWithSuffix_(const char* name, const RcsXmlParseCtx* ctx, bool forward)
{
  if (!name)
  {
    return NULL;
  }

  const RcsBody* bdy = RcsGraph_getBodyByName(ctx->graph, name);
  RLOG(9, "--- Checking body name '%s'", name);
  if (bdy)
  {
    return bdy;
  }

  RCHECK(ctx->level<RCSGRAPH_MAX_GROUPDEPTH);

  char suffixedBdy[RCS_MAX_NAMELEN];
  strcpy(suffixedBdy, name);

  if (forward)
  {
    for (int i=0; i<ctx->level; ++i)
    {
      if (!bdy)
      {
        strcat(suffixedBdy, ctx->suffixAtGroup[i]);
        bdy = RcsGraph_getBodyByName(ctx->graph, suffixedBdy);
      }
    }
  }
  else // backward
  {
    if (ctx->level==0)
    {
      return NULL;
    }

    for (int i=ctx->level-1; i>=0; --i)
    {
      if (!bdy)
      {
        strcat(suffixedBdy, ctx->suffixAtGroup[i]);
        RLOG(1, "--- Checking body name '%s'", suffixedBdy);
        bdy = RcsGraph_getBodyByName(ctx->graph, suffixedBdy);
      }
    }
  }

  return bdy;
}

static const RcsBody* findBodyWithSuffix(const char* name, const RcsXmlParseCtx* ctx)
{
  const RcsBody* bdy = findBodyWithSuffix_(name, ctx, true);

  if (!bdy)
  {
    bdy = findBodyWithSuffix_(name, ctx, false);
  }

  return bdy;
}

static void composeSuffix(const RcsXmlParseCtx* ctx, char* suffix)
{
  RCHECK(ctx->level<RCSGRAPH_MAX_GROUPDEPTH);

  suffix[0] = '\0';

  if (ctx->level==0)
  {
    return;
  }

#if defined SUFFIX_BACKWARDS
  for (int i=ctx->level-1; i>=0; --i)
  {
    strcat(suffix, ctx->suffixAtGroup[i]);
  }
#else
  for (int i=0; i<ctx->level; ++i)
  {
    strcat(suffix, ctx->suffixAtGroup[i]);
  }
#endif
}

static RcsBody* RcsBody_fromXML(xmlNode* bdyNode, const RcsXmlParseCtx* ctx)
{
  RCHECK(bdyNode);

  char suffix[RCS_MAX_NAMELEN] = "";
  composeSuffix(ctx, suffix);


  // Body name, unique default or as specified in the xml file
  char name[RCS_MAX_NAMELEN];
  snprintf(name, RCS_MAX_NAMELEN, "body %d", ctx->graph->nBodies);
  getXMLNodePropertyStringN(bdyNode, "name", name, RCS_MAX_NAMELEN);
  if (strlen(name)>10)
  {
    RCHECK_MSG(strncmp(name, "GenericBody", 11) != 0,
               "The name \"GenericBody\" is reserved for internal use");
  }

  RLOG(9, "BODY : %s", name);

  bool groupRoot = false;
  const char* prevBdyName = getXMLNodePropertyStringPtr(bdyNode, "prev");
  const RcsBody* parentBdy = NULL;

  // Here we are within a group on the root level
  if (!prevBdyName)
  {
    if (ctx->parentGroup)
    {
      groupRoot = true;
      prevBdyName = getXMLNodePropertyStringPtr(ctx->parentGroup, "prev");
      RLOG(9, "GROUP ROOT BODY FOUND : %s (prev is '%s')", name, prevBdyName);
    }
    else
    {
      RLOG(9, "TOP LEVEL ROOT BODY FOUND : %s", name);
    }
  }
  // Here we are within a group and search through all suffix concatenations.
  else // if (prevBdyName)
  {
    RLOG(9, "INTERMEDIATE BODY FOUND : %s", name);
  }

  parentBdy = findBodyWithSuffix(prevBdyName, ctx);



  if (parentBdy)
  {
    RLOG(9, "Found parentBdy: '%s' - suffix: '%s'",
         parentBdy->name, suffix);
  }
  else
  {
    RLOG(9, "NOT Found parentBdy: '%s' - suffix: '%s'",
         name, suffix);
  }

  // Get the body with the given parent-id from the graph's body array. The
  // RcsGraph_insertGraphBody() method already connects it.
  RcsBody* b = RcsGraph_insertGraphBody(ctx->graph, parentBdy ? parentBdy->id : -1);

  RLOG(5, "Inserted Body into Graph: name=%s id=%d parent=%d "
       "prev=%d next=%d first=%d last=%d",
       name, b->id, b->parentId, b->prevId, b->nextId,
       b->firstChildId, b->lastChildId);

  // Assign body names
  snprintf(b->bdyXmlName, RCS_MAX_NAMELEN, "%s", name);
  snprintf(b->bdySuffix, RCS_MAX_NAMELEN, "%s", suffix);
  snprintf(b->name, RCS_MAX_NAMELEN, "%s%s", name, suffix);

  // Check if we found a first body in the group whose including group has
  // rigid_body_joints defined. In this case, we create the rigid body joints
  // according to the parent group's description.
  xmlNodePtr rbjNode = NULL;
  bool hasGroupRBJTag = false;
  double q_rbj[12];
  VecNd_setZero(q_rbj, 12);

  if (groupRoot)
  {
    RLOG(5, "First group body \"%s\" is first one in a group with parent \"%s\"",
         name, parentBdy ? parentBdy->name : "NULL");

    if (parentBdy)
    {
      hasGroupRBJTag = getXMLNodeProperty(ctx->parentGroup, "rigid_body_joints");

      REXEC(5)
      {
        char tmp[RCS_MAX_NAMELEN] = "";
        getXMLNodePropertyStringN(ctx->parentGroup, "name", tmp, RCS_MAX_NAMELEN);
        RMSG("Body in group \"%s\": %s rigid_body_joints tag",
             tmp, hasGroupRBJTag ? "Found" : "Did not find");
      }

      if (hasGroupRBJTag)
      {
        RLOG(5, "Assigning rbjNode to parentGroupNode");
        rbjNode = ctx->parentGroup;
      }
    }
  }


  // Relative vector from prev. body to body (in prev. body coords)
  // It is only created if the XML file transform is not the identity matrix.
  if (getXMLNodeProperty(bdyNode, "transform"))
  {
    bool success = !getXMLNodeProperty(bdyNode, "quat");
    RCHECK_MSG(success, "\"quat\" is not allowed if \"transform\" exists.");
    getXMLNodePropertyHTr(bdyNode, "transform", &b->A_BP);
  }

  // check if the quat tag exists and if yes, transform is not allowed
  if (getXMLNodeProperty(bdyNode, "quat"))
  {
    bool success = !getXMLNodeProperty(bdyNode, "transform");
    RCHECK_MSG(success, "\"transform\" is not allowed if \"quat\" exists.");
    success = getXMLNodePropertyQuat(bdyNode, "quat", b->A_BP.rot);
    getXMLNodePropertyVec3(bdyNode, "pos", b->A_BP.org);
  }

  // Physics simulation
  char msg[RCS_MAX_NAMELEN] = "none";
  getXMLNodePropertyStringN(bdyNode, "physics", msg, RCS_MAX_NAMELEN);

  if (STREQ(msg, "none"))
  {
    b->physicsSim = RCSBODY_PHYSICS_NONE;
  }
  else if (STREQ(msg, "kinematic"))
  {
    b->physicsSim = RCSBODY_PHYSICS_KINEMATIC;
  }
  else if (STREQ(msg, "dynamic"))
  {
    b->physicsSim = RCSBODY_PHYSICS_DYNAMIC;
  }
  else if (STREQ(msg, "fixed"))
  {
    b->physicsSim = RCSBODY_PHYSICS_FIXED;
  }
  else
  {
    RFATAL("Unknown physics simulation type \"%s\"", msg);
  }

  // Check if this is a rigid body that should be attached to the world
  // by six joints which can be set by sensor information or physics
  int nJoints = 0;
  bool hasRBJTag = getXMLNodeProperty(bdyNode, "rigid_body_joints");
  if (hasRBJTag && rbjNode && (!parentBdy))
  {
    RFATAL("Body \"%s\" without parent has rigid body joints defined both in"
           " the body as well as in the including group", b->name);
  }

  // If the rigid body joint comes from the parent group, we prefer this one.
  if (!rbjNode && hasRBJTag)
  {
    rbjNode = bdyNode;
  }

  if (rbjNode)
  {
    b->rigid_body_joints = true;
    nJoints = 6;
    unsigned int nStr = getXMLNodeNumStrings(rbjNode, "rigid_body_joints");

    switch (nStr)
    {
      case 1:
        getXMLNodePropertyBoolString(rbjNode, "rigid_body_joints",
                                     &b->rigid_body_joints);
        break;

      case 6:
        getXMLNodePropertyVecN(rbjNode, "rigid_body_joints", q_rbj, 6);

        // convert Euler angles from degrees to radians
        Vec3d_constMulSelf(&q_rbj[3], M_PI / 180.0);
        break;

      case 12:
        getXMLNodePropertyVecN(rbjNode, "rigid_body_joints", q_rbj, 12);

        // convert Euler angles from degrees to radians
        Vec3d_constMulSelf(&q_rbj[3], M_PI / 180.0);
        break;

      default:
        RFATAL("Tag \"rigid_body_joints\" of body \"%s\" has %d entries"
               " - should be 6 or 1", b->name, nStr);
    }

    NLOG(5, "[%s]: Found %d strings in rigid_body_joint tag \"%s\", flag is "
         "%s", b->name, nStr, "rigid_body_joints",
         b->rigid_body_joints ? "true" : "false");

    RcsJoint* rbj0 = RcsBody_createRBJ(ctx->graph, b, q_rbj);

    // Determine constraint dofs for physics simulation. If a dof is
    // constrained will be interpreted by a "0" in the joint's weightMetric
    // property.
    if (nStr == 12)
    {
      unsigned int checkRbjNum = 0;
      RCSJOINT_TRAVERSE_FORWARD(ctx->graph, rbj0)
      {
        JNT->weightMetric = q_rbj[6 + checkRbjNum];
        checkRbjNum++;
      }
      RCHECK(checkRbjNum == 6);
    }


    // Rigid body joints don't have any relative transformations after
    // construction. If there is a transformation coming from a group, it needs
    // to be applied to the first of the six rigid body joints. We can simply
    // clone it.
    HTr_copy(&rbj0->A_JP, &ctx->groupTf);
  }



  // Body color. The default color is the one specified in the bodie's xml
  // description
  char bColor[RCS_MAX_NAMELEN];
  strcpy(bColor, ctx->defaultColor);
  getXMLNodePropertyStringN(bdyNode, "color", bColor, RCS_MAX_NAMELEN);

  // Create all shapes. This must be done before computing the inertia tensor,
  // since this depends on the shapes.
  xmlNodePtr shapeNode = bdyNode->children;

  // Allocate memory for shape node lists
  while (shapeNode != NULL)
  {
    if (isXMLNodeName(shapeNode, "Shape"))
    {
      RcsShape* sh = RcsBody_appendShape(b);
      RcsBody_initShape(sh, shapeNode, b, bColor);
    }

    shapeNode = shapeNode->next;
  }



  // Dynamic properties
  Mat3d_setZero(b->Inertia.rot);
  getXMLNodePropertyDouble(bdyNode, "mass", &b->m);

  // Calculate default inertia properties from shapes
  RcsBody_computeInertiaTensor(b, &b->Inertia);

  // Overwrite them if a tag is given
  double inertiaVec[9];
  Mat3d_toArray(inertiaVec, b->Inertia.rot);
  getXMLNodePropertyVecN(bdyNode, "inertia", inertiaVec, 9);
  Mat3d_fromArray(b->Inertia.rot, inertiaVec);
  getXMLNodePropertyVec3(bdyNode, "cogVector", b->Inertia.org);

  // Specifying an inertia tensor, and not the COG offset easily leads to
  // trouble in the equations of motion. We therefore warn to be explicit
  // about it.
  if ((getXMLNodeProperty(bdyNode, "inertia") == true) &&
      (getXMLNodeProperty(bdyNode, "cogVector") == false))
  {
    RLOGS(5, "You specified an inertia but not a cogVector in body \"%s\"",
          b->name);
  }

  // Connect the body to the previous one by his joints.
  xmlNodePtr jntNode = bdyNode->children;
  unsigned int xmlJntCount = 0;

  while (jntNode != NULL)
  {
    if (isXMLNodeName(jntNode, "Joint"))
    {
      RCHECK_MSG(b->rigid_body_joints == false, "Do not define additional "
                 "joints for rigid bodies!");

      // If a non-identity group transform is given, it needs to be applied to
      // the first joint only.
      RcsBody_initJoint(ctx->graph, b, jntNode, suffix,
                        xmlJntCount == 0 ? &ctx->groupTf : HTr_identity());
      nJoints++;
      xmlJntCount++;
    }
    jntNode = jntNode->next;
  }

  // If the body is not attached to any joint and the group transform is not
  // the identity matrix, the group transform is applied to the bodies relative
  // transformation. If it doesn't exist, it will be created.
  if ((nJoints == 0) && (HTr_isIdentity(&ctx->groupTf) == false))
  {
    HTr_transformSelf(&b->A_BP, &ctx->groupTf);
    RLOG(5, "Transformed body \"%s\"", b->name);
  }

  // Search for sensors attached to the body
  xmlNodePtr sensorNode = bdyNode->children;
  while (sensorNode)
  {
    if (isXMLNodeName(sensorNode, "Sensor"))
    {
      RcsSensor_initFromXML(sensorNode, b, ctx->graph);
    }
    sensorNode = sensorNode->next;
  }

  return b;
}

/*******************************************************************************
 *
 ******************************************************************************/
static void parseGraphTag(xmlNodePtr node, const RcsXmlParseCtx* calling_ctx)
{

  if (getXMLNodeProperty(node, "resourcePath"))
  {
    char* resourceDir = RNALLOC(1024, char);
    getXMLNodePropertyStringN(node, "resourcePath", resourceDir, 1024);

    char* saveptr = NULL;
    char* token = String_safeStrtok(resourceDir, " ", &saveptr);

    while (token)
    {
      // Directly use token instead of copying via sscanf
      char* expanded = String_expandEnvironmentVariables(token);
      if (expanded)
      {
        Rcs_addResourcePath(expanded);
        NLOG(0, "Adding to resource path: \"%s\"", expanded);
        RFREE(expanded);
      }

      token = String_safeStrtok(NULL, " ", &saveptr);
    }

    RFREE(resourceDir);
  }

  RcsXmlParseCtx ctx = *calling_ctx;

  if (node->children)
  {
    RcsGraph_parseRecursive(node->children, &ctx);
  }

  /* ctx = *calling_ctx; */
  /* RcsGraph_parseRecursive(node->next, &ctx); */

  // Then we look for the generic bodies and link them accordingly
  for (int i = 0; i < 10; i++)
  {
    char a[16], gBody[32];
    sprintf(a, "GenericBody%d", i);

    if (getXMLNodePropertyStringN(node, a, gBody, 32))
    {
      RLOG(5, "Linking \"%s\" to \"%s\"", a, gBody);
      RcsBody* b = RcsGraph_getBodyByName(ctx.graph, gBody);

      if (b == NULL)
      {
        RLOG(1, "%s points to \"%s\", which does not exist!",
             a, gBody);
      }
      else
      {
        RcsBody* l = RcsGraph_linkGenericBody(ctx.graph, i, b->name);

        if (l == NULL)
        {
          RLOG(1, "Body \"%s\" not found - %s points to NULL", gBody, a);
        }
        else
        {
          RLOG(5, "%s now points to \"%s\"", a, l->name);
        }
      }
    }

  }   // for(int i=0;i<10;i++)

}

/*******************************************************************************
 *
 ******************************************************************************/
static void parseGroupTag(xmlNodePtr node, const RcsXmlParseCtx* calling_ctx)
{
  RcsXmlParseCtx ctx = *calling_ctx;

  // Propagation of group transformation to next level
  HTr_setIdentity(&ctx.groupTf);
  getXMLNodePropertyHTr(node, "transform", &ctx.groupTf);

  // New extension = suffix + new group name
  const char* groupSuffix = getXMLNodePropertyStringPtr(node, "name");
  if (groupSuffix)
  {
    snprintf(ctx.suffixAtGroup[ctx.level], RCS_MAX_NAMELEN, "%s", groupSuffix);
  }

  // Groups default color, inherited from current levels' color
  getXMLNodePropertyStringN(node, "color", ctx.defaultColor, RCS_MAX_NAMELEN);

  // Copy current root node and descend one level
  ctx.level++;
  ctx.parentGroup = node;

  // Parse children
  const char* prevName = getXMLNodePropertyStringPtr(node, "prev");
  RLOG(9, "Start parsing group '%s' with prev '%s'",
       groupSuffix ? groupSuffix : "", prevName ? prevName : "");

  if (node->children)
  {
    RcsGraph_parseRecursive(node->children, &ctx);
  }

  RLOG(9, "End parsing group '%s' with prev '%s'",
       groupSuffix ? groupSuffix : "", prevName ? prevName : "");
}

/*******************************************************************************
 *
 ******************************************************************************/
static void RcsGraph_parseRecursive(xmlNodePtr node, RcsXmlParseCtx* calling_ctx)
{

  if (STREQ((char*) node->name, "Graph") ||
      STREQ((char*) node->name, "Group") ||
      STREQ((char*) node->name, "Body"))
  {
    RLOG(9, "NEW RECURSION: '%s'\n", (char*) node->name);
  }
  RCHECK_MSG(calling_ctx->level < RCSGRAPH_MAX_GROUPDEPTH - 2, "Group level exceeds maximum "
             "level: %d >= %d", calling_ctx->level, RCSGRAPH_MAX_GROUPDEPTH);

  if (isXMLNodeName(node, "Graph"))
  {
    parseGraphTag(node, calling_ctx);
    HTr_setIdentity(&calling_ctx->groupTf);
  }
  else if (isXMLNodeName(node, "Group"))
  {
    parseGroupTag(node, calling_ctx);
    HTr_setIdentity(&calling_ctx->groupTf);
  }
  else if (isXMLNodeName(node, "OpenRave"))
  {
    parseOpenRaveBody(node->next, calling_ctx->graph);
  }
  else if (isXMLNodeName(node, "URDF"))
  {
    char suffix[RCS_MAX_NAMELEN] = "";
    composeSuffix(calling_ctx, suffix);
    parseURDFFile(node->next, calling_ctx->graph, suffix);
  }
  else // can be a body or some junk
  {
    if (isXMLNodeName(node, "Body"))
    {
      RcsBody_fromXML(node, calling_ctx);
      HTr_setIdentity(&calling_ctx->groupTf);
    }

  }

  if (node->next)
  {
    RcsGraph_parseRecursive(node->next, calling_ctx);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
RcsGraph* RcsGraph_createFromXmlNode(const xmlNodePtr node)
{
  if (node == NULL)
  {
    RLOG(1, "XML node is NULL - failed to create RcsGraph");
    return NULL;
  }

  // Get memory for the graph. We initialize the body array with a few entries.
  // The RcsGraph_insertBody() takes care of reallocating it if needed.
  RcsGraph* self = RALLOC(RcsGraph);
  strcpy(self->cfgFile, "Created_from_xml_node");

  // This is the arrays for the state vectors and velocities. We need to
  // create them here, since in the RcsJoint data structure a pointer will
  // point to the self->q values.
  self->q = MatNd_create(0, 1);
  self->q_dot = MatNd_create(0, 1);

  // Initialize generic bodies to refer to no graph body.
  for (int i = 0; i < RCS_NUM_GENERIC_BODIES; ++i)
  {
    self->gBody[i] = -1;
  }

  // Recursively assemble all bodies, joints and shapes.
  RcsXmlParseCtx ctx = { 0 };   // everything 0 / false
  ctx.graph = self;
  HTr_setIdentity(&ctx.groupTf);
  RcsGraph_parseRecursive(node, &ctx);

  // Order joint indices for depth-first traversal, and connect coupled joints
  RcsGraph_makeJointsConsistent(self);

  // Apply model state
  const char* mdlName = getXMLNodePropertyStringPtr(node, "name");
  if (mdlName)
  {
    RcsGraph_parseModelState(node, self, mdlName);
  }

  // Set state vector. For the case there are velocities assigned in the
  // model_state, we perform a velocity forward kinematics pass to compute
  // the graph's corresponding body velocities.
  RcsGraph_setState(self, NULL, self->q_dot);

  return self;
}
