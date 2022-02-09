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

//#define TRANSFORM_ROOT_NEXT



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
        }

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
    int xy[2];
    xy[0] = sensor->rawData->m;
    xy[1] = sensor->rawData->n;

    getXMLNodePropertyIntN(node, "dimensions", xy, 2);
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
  bool depth=false, contact=false, attachment = false;

  // Physics computation is not carried out for non-physics objects by default.
  if (body->physicsSim == RCSBODY_PHYSICS_NONE)
  {
    physics = false;
  }

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

  getXMLNodePropertyBoolString(node, "distance", &distance);
  getXMLNodePropertyBoolString(node, "physics", &physics);
  getXMLNodePropertyBoolString(node, "graphics", &graphics);
  getXMLNodePropertyBoolString(node, "softPhysics", &softPhysics);
  getXMLNodePropertyBoolString(node, "depth", &depth);
  getXMLNodePropertyBoolString(node, "contact", &contact);
  getXMLNodePropertyBoolString(node, "attachment", &attachment);

  shape->resizeable = false;
  getXMLNodePropertyBoolString(node, "resizeable", &shape->resizeable);
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
    shape->resizeable = true;
  }
  if (depth == true)
  {
    shape->computeType |= RCSSHAPE_COMPUTE_DEPTHBUFFER;
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

  // Mesh file
  int strLength = getXMLNodeBytes(node, "meshFile");

  if (strLength > 0)
  {
    char fileName[RCS_MAX_FILENAMELEN] = "";
    char fullName[RCS_MAX_FILENAMELEN] = "";
    RCHECK((shape->type == RCSSHAPE_MESH) || (shape->type == RCSSHAPE_OCTREE));
    getXMLNodePropertyStringN(node, "meshFile", fileName, RCS_MAX_FILENAMELEN);
    Rcs_getAbsoluteFileName(fileName, fullName);

    if (File_exists(fullName) == true)
    {
      snprintf(shape->meshFile, RCS_MAX_FILENAMELEN, "%s", fullName);

      if (shape->type == RCSSHAPE_MESH)
      {
        RcsMeshData* mesh = RcsMesh_createFromFile(shape->meshFile);
        if (mesh == NULL)
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

  // Color
  strcpy(shape->color, bodyColor ? bodyColor : "DEFAULT");
  getXMLNodePropertyStringN(node, "color", shape->color, RCS_MAX_NAMELEN);

  // Material
  strcpy(shape->material, "default");
  getXMLNodePropertyStringN(node, "material", shape->material, RCS_MAX_NAMELEN);

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
    snprintf(jnt->name, RCS_MAX_NAMELEN, "unnamed joint%s %d",
             suffix, uniqueId++);

    REXEC(5)
    {
      RLOG(5, "A joint of body \"%s\" has no name - using \"unnamed joint\"",
           b->name);
    }
  }

  NLOG(0, "Inserted Joint into Graph: name=%s id=%d prevId=%d nextId=%d",
       jnt->name, jnt->id, jnt->prevId, jnt->nextId);
  if (jnt->prevId!=-1)
  {
    RcsJoint* pjnt = RCSJOINT_BY_ID(self, jnt->prevId);
    RLOG(5, "   prev Joint: name=%s id=%d prevId=%d nextId=%d",
         pjnt->name, pjnt->id, pjnt->prevId, pjnt->nextId);
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

    RCHECK_MSG(jnt->q_min <= jnt->q_max, "q_min=%f q_max=%f",
               jnt->q_min, jnt->q_max);
    RCHECK_MSG((jnt->q0 >= jnt->q_min) && (jnt->q0 <= jnt->q_max),
               "q_min=%f q0=%f q_max=%f", jnt->q_min, jnt->q0, jnt->q_max);

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
    getXMLNodePropertyVecN(node, "couplingFactor",
                           jnt->couplingPoly, polyGrad);

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
* Allocates memory and initializes a RcsBody data structure from an XML node.
******************************************************************************/
static RcsBody* RcsBody_createFromXML(RcsGraph* self,
                                      xmlNode* bdyNode,
                                      const char* defaultColor,
                                      const char* suffix,
                                      const char* parentGroup,
                                      HTr* A_group,
                                      bool firstInGroup,
                                      int level,
                                      int rootId,
                                      bool verbose)
{
  // Return if node is not a body node
  if (!isXMLNodeName(bdyNode, "Body"))
  {
    return NULL;
  }

  RCHECK(self);
  RCHECK(suffix);
  RCHECK(A_group);
  RcsBody* root = RCSBODY_BY_ID(self, rootId);

  // Body name
  char name[RCS_MAX_NAMELEN];
  snprintf(name, RCS_MAX_NAMELEN, "body %d", self->nBodies);

  // The name as indicated in the xml file
  getXMLNodePropertyStringN(bdyNode, "name", name, RCS_MAX_NAMELEN);
  if (strlen(name)>10)
  {
    RCHECK_MSG(strncmp(name, "GenericBody", 11) != 0,
               "The name \"GenericBody\" is reserved for internal use");
  }

  /* RLOG(0, "Body %s: firstInGroup is %s", */
  /*      name, firstInGroup ? "TRUE" : "FALSE"); */

  char msg[RCS_MAX_NAMELEN];
  RcsBody* parentBdy = root;
  if (getXMLNodePropertyStringN(bdyNode, "prev", msg, RCS_MAX_NAMELEN) > 0)
  {
    if (parentBdy && firstInGroup)
    {
      RLOG(1, "WARNING: \"prev\"-tag supplied in body \"%s\", but also in "
           "group; body information will be overridden", name);
    }

    // If the body is the first in a group, we search its parent without the
    // group suffix. Otherwise, the suffix is appended to the name to be
    // searched. This way, we don't need to specify the suffix within a
    // group, so that the group can be used generically.
    if (!firstInGroup)
    {
      // first try to find the body with suffix
      char bodyNameWithSuffix[RCS_MAX_NAMELEN];
      snprintf(bodyNameWithSuffix, RCS_MAX_NAMELEN, "%s%s", msg, suffix);
      parentBdy = RcsGraph_getBodyByName(self, bodyNameWithSuffix);
    }

    if (!parentBdy)
    {
      // if we did not find the a body with the group suffix, let's see if
      // there is one without
      parentBdy = RcsGraph_getBodyByName(self, msg);
    }
  }

  // Get the body with the given parent-id from the graph's body array. The
  // RcsGraph_insertGraphBody() method already connects it.
  RLOG(5, "Adding %s with parent %s (%s)",
       name, parentBdy ? parentBdy->name : "NULL", msg);
  RcsBody* b = RcsGraph_insertGraphBody(self, parentBdy ? parentBdy->id : -1);

  RLOG(5, "Inserted Body into Graph: name=%s id=%d parent=%d "
       "prev=%d next=%d first=%d last=%d",
       name, b->id, b->parentId, b->prevId, b->nextId,
       b->firstChildId, b->lastChildId);

  // Assign body names
  snprintf(b->bdyXmlName, RCS_MAX_NAMELEN, "%s", name);
  snprintf(b->bdySuffix, RCS_MAX_NAMELEN, "%s", suffix);
  snprintf(b->name, RCS_MAX_NAMELEN, "%s%s", name, suffix);

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
  strcpy(msg, "none");
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
  double q_rbj[12];
  VecNd_setZero(q_rbj, 12);

  if (hasRBJTag == true)
  {
    b->rigid_body_joints = true;
    nJoints = 6;
    unsigned int nStr = getXMLNodeNumStrings(bdyNode, "rigid_body_joints");

    switch (nStr)
    {
      case 1:
        getXMLNodePropertyBoolString(bdyNode, "rigid_body_joints",
                                     &b->rigid_body_joints);
        break;

      case 6:
        getXMLNodePropertyVecN(bdyNode, "rigid_body_joints", q_rbj, 6);

        // convert Euler angles from degrees to radians
        Vec3d_constMulSelf(&q_rbj[3], M_PI / 180.0);
        break;

      case 12:
        getXMLNodePropertyVecN(bdyNode, "rigid_body_joints", q_rbj, 12);

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

    RcsJoint* rbj0 = RcsBody_createRBJ(self, b, q_rbj);

    // Determine constraint dofs for physics simulation. If a dof is
    // constrained will be interpreted by a "0" in the joint's weightMetric
    // property.
    if (nStr == 12)
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
    // construction. If there is a transformation coming from a group, it needs
    // to be applied to the first of the six rigid body joints. We can simply
    // clone it.
    HTr_copy(&rbj0->A_JP, A_group);
  }



  // Body color. The default color is the one specified in the bodie's xml
  // description
  char bColor[RCS_MAX_NAMELEN];
  strcpy(bColor, defaultColor);
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
      RcsBody_initJoint(self, b, jntNode, suffix,
                        xmlJntCount == 0 ? A_group : HTr_identity());
      nJoints++;
      xmlJntCount++;
    }
    jntNode = jntNode->next;
  }

  // If the body is not attached to any joint and the group transform is not
  // the identity matrix, the group transform is applied to the bodies relative
  // transformation. If it doesn't exist, it will be created.
  if ((nJoints == 0) && (HTr_isIdentity(A_group) == false))
  {
    HTr_transformSelf(&b->A_BP, A_group);
    RLOG(5, "Transformed body \"%s\"", b->name);
  }

  // Reset the groups transform, it only must be applied to the first body.
  // \todo: This must go. All bodies without parent (the ones on root level
  //        next to the level's root) must be transformed.
#ifndef TRANSFORM_ROOT_NEXT
  HTr_setIdentity(A_group);
#endif

  // Search for sensors attached to the body
  xmlNodePtr sensorNode = bdyNode->children;
  while (sensorNode)
  {
    if (isXMLNodeName(sensorNode, "Sensor"))
    {
      RcsBody* mountBdy = &self->bodies[self->nBodies-1];
      RcsSensor_initFromXML(sensorNode, mountBdy, self);
    }
    sensorNode = sensorNode->next;
  }


  return &self->bodies[self->nBodies-1];
}

/*******************************************************************************
*
* This function recursively parses from the given xml node and
* initializes the bodies. It implements a depth-first traversal through
* the RcsGraph tree by the following rules:
*
* 1. If a "Graph" or "Group" node has been found, first its children,
* and then its "next" nodes on the same level are called recursively.
* Then the function returns (to the upper level).
*
* 2. If another node ("Body" or some junk memory) are found, the
* corresponding body will be created and the "next" node on the same
* level is called recursively.
*
* The transformation A is the relative transformation of a group. It
* will be resetted to identity when a body
* has successfully created (in RcsBody_createFromXML(...)). That's
* the case since it only has to be applied to the first body of the
* group.
*
******************************************************************************/
static void RcsGraph_parseBodies(xmlNodePtr node,
                                 RcsGraph* self,
                                 const char* gCol,
                                 const char* suffix,
                                 const char* parentGroup,
                                 HTr* A,
                                 bool firstInGroup,
                                 int level,
                                 int rootId[RCSGRAPH_MAX_GROUPDEPTH],
                                 bool verbose)
{
  if (node == NULL)
  {
    return;
  }

  RCHECK_MSG(level < RCSGRAPH_MAX_GROUPDEPTH - 2, "Group level exceeds maximum "
             "level: %d >= %d", level, RCSGRAPH_MAX_GROUPDEPTH);

  // Set debug level to 9 when verbose
  long lDl = RcsLogLevel;
  if (verbose == true)
  {
    RcsLogLevel = 9;
  }

  if (isXMLNodeName(node, "Graph"))
  {
    RLOG(9, "Found Graph node - descending - firstInGroup is %d",
         firstInGroup);

    if (getXMLNodeProperty(node, "resourcePath"))
    {
      char* resourceDir = RNALLOC(1024, char);
      getXMLNodePropertyStringN(node, "resourcePath", resourceDir, 1024);

      char* pch = strtok(resourceDir, " ");
      char* path = RNALLOC(256, char);
      int nPaths = 0;

      // Determine paths by space-separated tags
      while (pch != NULL)
      {
        sscanf(pch, "%255s", path);
        char* ePath = String_expandEnvironmentVariables(path);
        Rcs_addResourcePath(ePath);
        RFREE(ePath);
        RLOG(9, "Adding path %d to ressource path: \"%s\"", nPaths++, path);
        pch = strtok(NULL, " ");
      }

      RFREE(resourceDir);
      RFREE(path);
    }

    RcsGraph_parseBodies(node->children, self, gCol, suffix,
                         parentGroup, A, firstInGroup, level, rootId, verbose);

    // After we parsed the children of the graph, the graph has been
    // initialized: We reset the firstInGroup flag. This will lead to
    // problems if there's no body created inside a <Graph> tag. TODO: verify
    // and warn
    RLOG(9, "Ascending from Graph node - firstInGroup is false");
    firstInGroup = false;
    RcsGraph_parseBodies(node->next, self, gCol, suffix,
                         parentGroup, A, firstInGroup, level, rootId, verbose);

    // Then we look for the generic bodies and link them accordingly
    for (int i = 0; i < 10; i++)
    {
      char a[16], gBody[32];
      sprintf(a, "GenericBody%d", i);

      if (getXMLNodePropertyStringN(node, a, gBody, 32))
      {
        RLOG(5, "Linking \"%s\" to \"%s\"", a, gBody);
        RcsBody* b = RcsGraph_getBodyByName(self, gBody);

        if (b == NULL)
        {
          RLOG(1, "%s points to \"%s\", which does not exist!",
               a, gBody);
        }
        else
        {
          RcsBody* l = RcsGraph_linkGenericBody(self, i, b->name);

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
  else if (isXMLNodeName(node, "Group"))
  {
    char ndExt[32], tmp[32] = "", pGroupSuffix[32];

    // Propagation of group transformation to next level
    HTr A_local, A_group;
    HTr_copy(&A_group, A);
    HTr_setIdentity(&A_local);
    getXMLNodePropertyHTr(node, "transform", &A_local);
    HTr_transformSelf(&A_group, &A_local);

    // New extension = suffix + new group name
    getXMLNodePropertyStringN(node, "name", tmp, 32);
    snprintf(pGroupSuffix, 32, "%s", suffix);
    snprintf(ndExt, 32, "%s%s", suffix, tmp);

    // Groups default color, inherited from current levels' color
    char col[RCS_MAX_NAMELEN];
    snprintf(col, RCS_MAX_NAMELEN, "%s", gCol);
    getXMLNodePropertyStringN(node, "color", col, RCS_MAX_NAMELEN);

    REXEC(9)
    {
      const RcsBody* levelRoot = RCSBODY_BY_ID(self, rootId[level]);
      RMSG("[Level %d -> %d]: \n\tNew group \"%s\" with root \"%s\" "
           "and color \"%s\"", level, level + 1, tmp,
           levelRoot ? levelRoot->name : "NULL", col);

      fprintf(stderr, "\tA_prev             %5.3f   %5.3f   %5.3f\n",
              A->org[0], A->org[1], A->org[2]);
      fprintf(stderr, "\tA_local            %5.3f   %5.3f   %5.3f\n",
              A_local.org[0], A_local.org[1], A_local.org[2]);
      fprintf(stderr, "\tApplying transform %5.3f   %5.3f   %5.3f\n",
              A_group.org[0], A_group.org[1], A_group.org[2]);
    }

    // Copy current root node and descend one level
    rootId[level + 1] = rootId[level];
    level++;

    if (getXMLNodePropertyStringN(node, "prev", tmp, 32) > 0)
    {
      RcsBody* parent = RcsGraph_getBodyByName(self, tmp);
      if (parent)
      {
        RCHECK(level > 0);
        rootId[level] = parent->id;
        RLOG(9, "Setting root[%d] to \"%s\"", level, parent ? parent->name : "NULL");
      }
    }

    RcsGraph_parseBodies(node->children, self, col, ndExt,
                         pGroupSuffix, &A_group, true, level, rootId,
                         verbose);

    RLOG(9, "[Level %d -> %d]: back from group \"%s\"", level, level - 1, tmp);

    // After we parsed the children of the group, the group has been
    // initialized: The level is decremented, and the suffix is resetted.
    level--;
    strcpy(ndExt, suffix);

    RcsGraph_parseBodies(node->next, self, gCol, ndExt,
                         parentGroup, A, firstInGroup, level, rootId, verbose);
  }









  else if (isXMLNodeName(node, "OpenRave"))
  {
    // The node points to an OpenRave file

    char tmp[RCS_MAX_FILENAMELEN];
    strcpy(tmp, "");

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
      RLOGS(1, "Found q0 tag --> overriding initial values of OpenRave file");

      // get number of provided q0 values
      char q_str[512];
      getXMLNodePropertyStringN(node, "q0", q_str, 512);
      nq = String_countSubStrings(q_str, " ");

      // read q0 values
      q0 = RNALLOC(nq, double);
      getXMLNodePropertyVecN(node, "q0", q0, nq);

      // convert to radian
      VecNd_constMulSelf(q0, M_PI / 180.0, nq);
    }

    // parse OpenRave file
    RcsGraph_createBodiesFromOpenRAVEFile(self, pB, tmp, q0, nq);

    // cleanup
    RFREE(q0);

    RcsGraph_parseBodies(node->next, self, gCol, suffix,
                         parentGroup, A, firstInGroup, level, rootId, verbose);
  }












  else if (isXMLNodeName(node, "URDF"))
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
    char urdfSuffix[132] = "", ndExt[132] = "";
    getXMLNodePropertyStringN(node, "suffix", urdfSuffix, 132);
    strcpy(ndExt, suffix);
    strcat(ndExt, urdfSuffix);

    HTr A_local;
    HTr_setIdentity(&A_local);
    getXMLNodePropertyHTr(node, "transform", &A_local);
    unsigned int dof = 0;
    int urdfRootId = RcsGraph_rootBodyFromURDFFile(self, filename, ndExt,
                                                   &A_local, &dof);
    RCHECK_MSG(urdfRootId!=-1, "Couldn't get URDF root from file \"%s\"", filename);
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
        //for (RcsJoint* JNT = rbj0; JNT; JNT = JNT->next)
        //for (RcsJoint* JNT = rbj0; JNT; JNT = (JNT->nextId==-1) ? NULL : &self->joints[JNT->nextId])
        RCSJOINT_TRAVERSE_FORWARD(self, rbj0)
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
      if (HTr_isIdentity(&A_local) == false)
      {
        HTr_copy(&rbj0->A_JP, &A_local);
        // since the group transform was already applied to the body, remove it there again
        HTr_setIdentity(&urdfRoot->A_BP);
      }
    }
    else if (urdfRoot->physicsSim != RCSBODY_PHYSICS_NONE)
    {
      // no rigid body joints - urdf root is fixed to it's parent. Make sure that the physics simulation treats it correctly.
      if (pB != NULL && (pB->physicsSim == RCSBODY_PHYSICS_DYNAMIC || pB->physicsSim == RCSBODY_PHYSICS_FIXED))
      {
        // or to fixed if the parent is dynamic
        urdfRoot->physicsSim = RCSBODY_PHYSICS_FIXED;
      }
      else
      {
        // set it to kinematic if the parent is kinematic or not participating at all
        urdfRoot->physicsSim = RCSBODY_PHYSICS_KINEMATIC;
      }
    }

    RcsGraph_parseBodies(node->next, self, gCol, suffix,
                         parentGroup, A, firstInGroup, level, rootId, verbose);
  }










  else // can be a body or some junk
  {
    const RcsBody* levelRoot = RCSBODY_BY_ID(self, rootId[level]);
    RLOG(19, "Creating new body with root[%d] \"%s\"", level,
         (level > 0) ? (levelRoot ? levelRoot->name : "NULL") : "NULL");

    RcsBody* nr = NULL;

    nr = RcsBody_createFromXML(self, node, gCol, suffix, parentGroup,
                               A, firstInGroup, level,
                               rootId[level], verbose);

#ifndef TRANSFORM_ROOT_NEXT
    if (nr)
    {
      firstInGroup = false;
    }
#else
    if (firstInGroup && !node->next)
    {
      firstInGroup = false;
      HTr_setIdentity(A);
    }
#endif
    RcsGraph_parseBodies(node->next, self, gCol, suffix,
                         parentGroup, A, firstInGroup, level, rootId, verbose);

    RLOG(19, "Falling back - root[%d] \"%s\"",
         level, levelRoot ? levelRoot->name : "NULL");
  }



  // Reset debug level
  if (verbose)
  {
    RcsLogLevel = lDl;
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
bool RcsGraph_setModelStateFromXML(RcsGraph* self, const char* modelStateName,
                                   int timeStamp)
{
  if ((self==NULL) || (modelStateName==NULL))
  {
    return false;
  }

  // Read XML file
  xmlDocPtr doc;
  xmlNodePtr node = parseXMLFile(self->cfgFile, "Graph", &doc);

  if (node == NULL)
  {
    xmlFreeDoc(doc);
    return false;
  }

  bool success = RcsGraph_parseModelState(node, self, modelStateName);
  xmlFreeDoc(doc);

  return success;
}

/*******************************************************************************
 * We make a copy, since the RcsGraph_parseModelState() function changes the
 * graph.
 ******************************************************************************/
bool RcsGraph_getModelStateFromXML(MatNd* q, const RcsGraph* self,
                                   const char* modelStateName, int timeStamp)
{
  if ((self==NULL) || (modelStateName==NULL))
  {
    return false;
  }

  RcsGraph* copyOfGraph = RcsGraph_clone(self);

  if (copyOfGraph==NULL)
  {
    return false;
  }

  // Read XML file
  xmlDocPtr doc;
  xmlNodePtr node = parseXMLFile(copyOfGraph->cfgFile, "Graph", &doc);

  if (node == NULL)
  {
    RcsGraph_destroy(copyOfGraph);
    xmlFreeDoc(doc);
    return false;
  }

  bool success = RcsGraph_parseModelState(node, copyOfGraph, modelStateName);
  xmlFreeDoc(doc);

  MatNd_reshapeCopy(q, copyOfGraph->q);

  RcsGraph_destroy(copyOfGraph);

  return success;
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

  // Recurse through bodies
  HTr A_rel;
  HTr_setIdentity(&A_rel);
  int rootId[RCSGRAPH_MAX_GROUPDEPTH];
  for (unsigned int i=0; i<RCSGRAPH_MAX_GROUPDEPTH; ++i)
  {
    rootId[i] = 0;
  }

  // Initialize generic bodies. Here we allocate memory for names and body
  // transforms. They are deleted once relinked to another body. We initialize
  // it before parsing, since they can already be linked in the xml files.
  for (int i = 0; i < RCS_NUM_GENERIC_BODIES; i++)
  {
    self->gBody[i] = -1;
  }

  RcsGraph_parseBodies(node, self, "DEFAULT", "", "",
                       &A_rel, false, 0, rootId, false);

  // Create velocity vector. It is done here since self->dof has been
  // computed during parsing.
  self->q_dot = MatNd_create(self->dof, 1);

  // Re-order joint indices to match depth-first traversal, and connect coupled
  // joints
  RcsGraph_makeJointsConsistent(self);

  // Apply model state
  char mdlName[RCS_MAX_NAMELEN] = "";
  int nBytes = getXMLNodePropertyStringN(node, "name", mdlName, RCS_MAX_NAMELEN);
  if (nBytes > 0)
  {
    RcsGraph_parseModelState(node, self, mdlName);
  }

  // Set state vector. For the case there are velocities assigned in the
  // model_state, we perform a velocity forward kinematics pass to compute
  // the graph's corresponding body velocities.
  RcsGraph_setState(self, NULL, self->q_dot);

  return self;
}
