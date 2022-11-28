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

#include "Rcs_graphOpenRAVEParser.h"
#include "Rcs_parser.h"
#include "Rcs_typedef.h"
#include "Rcs_body.h"
#include "Rcs_joint.h"
#include "Rcs_shape.h"
#include "Rcs_macros.h"
#include "Rcs_utils.h"
#include "Rcs_resourcePath.h"
#include "Rcs_math.h"

#include <float.h>




static unsigned int getXMLNodeString(xmlNodePtr node, char* str)
{
  unsigned int len = 0;
  xmlChar* txt = xmlNodeGetContent(node);

  if (txt == NULL)
  {
    return 0;
  }

  if (str != NULL)
  {
    strcpy(str, (const char*)txt);
  }

  len = strlen((const char*)txt) + 1;  // +1 to account for trailing '\0'

  xmlFree(txt);

  return len;
}

static bool getXMLNodeIntN(xmlNodePtr node, int* x, unsigned int n)
{
  if ((x) && (n > 0))
  {
    char tmp[512];
    int len = getXMLNodeString(node, tmp);

    if (len > 0)
    {
      char* saveptr;
      char* pch = String_safeStrtok(tmp, " ", &saveptr);
      int value;
      unsigned int matched_tags = 0;
      while (pch != NULL)
      {
        if (sscanf(pch, "%d", &value))
        {
          matched_tags++;
          if (matched_tags <= n)
          {
            x[matched_tags - 1] = value;
          }
        }
        pch = String_safeStrtok(NULL, " ", &saveptr);
      }
      RCHECK_MSG(matched_tags == n, "during parsing, not all "
                 "(or more) values could be found (found %d, should be %d)",
                 matched_tags, n);
    }
    else
    {
      return false;
    }
  }
  return true;
}

static bool getXMLNodeVecN(xmlNodePtr nd, double* x, unsigned int n)
{
  if ((x != NULL) && (n > 0))
  {
    char tmp[512];
    int len = getXMLNodeString(nd, tmp);

    RCHECK_MSG(len < 512, "Max. string length is limited to 512 bytes, yours "
               "is %d long", len);

    if (len > 0)
    {
      char* saveptr;
      char* pch = String_safeStrtok(tmp, " ", &saveptr);
      double value;
      unsigned int matched_tags = 0;

      while (pch != NULL)
      {
        if (sscanf(pch, "%lf", &value))
        {
          RCHECK(isfinite(value));
          matched_tags++;
          if (matched_tags <= n)
          {
            x[matched_tags - 1] = value;
          }
        }
        pch = String_safeStrtok(NULL, " ", &saveptr);
      }
      RCHECK_MSG(matched_tags == n, "during parsing, not all "
                 "(or more) values could be found (found %d, should be %d)",
                 matched_tags, n);

    }   // if(len>0)
    else
    {
      return false;
    }

  }   // if ((x) && (n > 0))

  return true;
}

static bool getXMLNodeVec3(xmlNodePtr node, double* x)
{
  return getXMLNodeVecN(node, x, 3);
}


static bool getXMLNodeQuat(xmlNodePtr node, double A_BI[3][3])
{
  double x[4], qw, qx, qy, qz;

  if (!getXMLNodeVecN(node, x, 4))
  {
    return false;
  }

  qw = x[0];
  qx = x[1];
  qy = x[2];
  qz = x[3];

  // Set matrix with transposed style to fit HRI rotation axis
  A_BI[0][0] = 1.0f - 2.0f*qy*qy - 2.0f*qz*qz;
  A_BI[1][0] = 2.0f*qx*qy - 2.0f*qz*qw;
  A_BI[2][0] = 2.0f*qx*qz + 2.0f*qy*qw;

  A_BI[0][1] = 2.0f*qx*qy + 2.0f*qz*qw;
  A_BI[1][1] = 1.0f - 2.0f*qx*qx - 2.0f*qz*qz;
  A_BI[2][1] = 2.0f*qy*qz - 2.0f*qx*qw;

  A_BI[0][2] = 2.0f*qx*qz - 2.0f*qy*qw;
  A_BI[1][2] = 2.0f*qy*qz + 2.0f*qx*qw;
  A_BI[2][2] = 1.0f - 2.0f*qx*qx - 2.0f*qy*qy;

  return true;
}

static xmlNodePtr getXMLNodeForOpenRave(const char* filename, xmlDocPtr* doc)
{
  xmlNodePtr rave_node = NULL;

  rave_node = parseXMLFile(filename, "Robot", doc);

  if (rave_node == NULL)
  {
    RLOG(5, "Couldn't find node \"Robot\" - checking for \"Environment\"");
    rave_node = parseXMLFile(filename, "Environment", doc);
  }

  if (rave_node == NULL)
  {
    RLOG(5, "Couldn't find node \"Environment\" neither - returning NULL");
  }

  return rave_node;
}

static void RcsShape_initFromOpenRAVEXML(RcsShape* shape, xmlNode* node,
                                         RcsBody* body)
{
  /// \todo: not all shape types are supported yet
  char str[RCS_MAX_NAMELEN];
  getXMLNodePropertyStringN(node, "type", str, RCS_MAX_NAMELEN);
  if (STRCASEEQ(str, "box"))
  {
    shape->type = RCSSHAPE_BOX;
  }
  else if (STRCASEEQ(str, "sphere"))
  {
    shape->type = RCSSHAPE_SPHERE;
  }
  else if (STRCASEEQ(str, "capsule"))
  {
    shape->type = RCSSHAPE_SSL;
  }
  else if (STRCASEEQ(str, "trimesh"))
  {
    shape->type = RCSSHAPE_MESH;
  }
  else if (STRCASEEQ(str, "frame"))
  {
    shape->type = RCSSHAPE_REFFRAME;
  }
  else if (STRCASEEQ(str, "cylinder"))
  {
    shape->type = RCSSHAPE_CYLINDER;
  }
  else
  {
    RMSG("OpenRave: Unsupported shape type in \"%s\"", str);
  }

  if (shape->type == RCSSHAPE_REFFRAME)
  {
    // set default extents
    Vec3d_setElementsTo(shape->extents, 0.9);
  }

  xmlNodePtr child = getXMLChildByName(node, "extents");
  getXMLNodeVecN(child, shape->extents, 3);
  if (shape->type == RCSSHAPE_BOX)
  {
    Vec3d_constMulSelf(shape->extents, 2.0);
  }

  child = getXMLChildByName(node, "radius");
  getXMLNodeVecN(child, &shape->extents[0], 1);

  child = getXMLChildByName(node, "height");
  getXMLNodeVecN(child, &shape->extents[2], 1);

  child = getXMLChildByName(node, "translation");
  getXMLNodeVec3(child, shape->A_CB.org);

  child = getXMLChildByName(node, "quat");
  getXMLNodeQuat(child, shape->A_CB.rot);

  child = getXMLChildByName(node, "scale");
  double scale1d = 1.0;
  getXMLNodeVecN(child, &scale1d, 1);
  Vec3d_setElementsTo(shape->scale3d, scale1d);

  /// \todo (AH, Jan 27, 2014): rotationaxis parsing should be removed assumptions and cleaned up.
  child = node->children;
  double ea[3] = { 0.0, 0.0, 0.0 };
  while (child)
  {
    if (STREQ((const char*)BAD_CAST child->name, "rotationaxis"))
    {
#if 1
      double x[4];
      int axis[3];
      getXMLNodeVecN(child, x, 4);

      for (unsigned int i = 0; i < 3; i++)
      {
        axis[i] = (int)(x[i]);
      }

      if (axis[0] == 1)
      {
        ea[0] = x[3];
      }
      else if (axis[1] == 1)
      {
        ea[1] = x[3];
      }
      else if (axis[2] == 1)
      {
        if (shape->type == RCSSHAPE_SSL)
        {
          ea[1] = x[3];
        }
        else
        {
          ea[2] = x[3];
        }
      }
#else
      double x[4], A_KI[3][3];
      getXMLNodeVecN(child, x, 4);
      Mat3d_fromAxisAngle(A_KI, x, x[3]);
      Mat3d_getEulerAngles(A_KI, ea);
#endif
    }
    child = child->next;
  }

  // angles are specified in degree
  Vec3d_constMulSelf(ea, M_PI / 180.0);


  Mat3d_fromEulerAngles(shape->A_CB.rot, ea);

  // need to adapt for HRI
  if (shape->type == RCSSHAPE_SSL)
  {
    // offset of thx (-90 deg)
    ea[0] -= 0.5*M_PI;
    Mat3d_fromEulerAngles(shape->A_CB.rot, ea);

    HTr local;
    HTr_setIdentity(&local);
    local.org[2] -= 0.5 * shape->extents[2];

    HTr temp;
    HTr_copy(&temp, &shape->A_CB);
    HTr_transform(&shape->A_CB, &temp, &local);
  }

  // Compute type
  bool distance = true, graphics = true, physics = true;

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

  // Visualizing all openRAVE bodies for now
#if 0
  // Only meshes are added to the graphics node by default (also SSLs are visualized by default here)
  if (shape->type != RCSSHAPE_MESH)
  {
    graphics = false;
  }
#endif

  // Reference frames are only considered for the graphics nodes by default
  if (shape->type == RCSSHAPE_REFFRAME)
  {
    graphics = true;
    distance = false;
    physics = false;
  }

  getXMLNodePropertyBoolString(node, "distance", &distance);
  getXMLNodePropertyBoolString(node, "physics", &physics);
  getXMLNodePropertyBoolString(node, "render", &graphics);

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

  // Mesh file
  char fileName[RCS_MAX_FILENAMELEN] = "-";
  child = getXMLChildByName(node, "render");
  if (child == NULL)
  {
    child = getXMLChildByName(node, "Data");
  }
  int strLength = getXMLNodeString(child, fileName);

  if (strLength > 0)
  {
    char fullName[RCS_MAX_FILENAMELEN] = "-";
    RCHECK(shape->type == RCSSHAPE_MESH);
    Rcs_getAbsoluteFileName(fileName, fullName);
    snprintf(shape->meshFile, RCS_MAX_FILENAMELEN, "%s", fullName);

    if (File_exists(shape->meshFile) == false)
    {
      RLOG(4, "Mesh file \"%s\" not found!", fileName);
    }

  }

  /// \todo textures, colors, materials not supported yet
  strcpy(shape->color, "DEFAULT");
  strcpy(shape->material, "default");
}

static RcsJoint* RcsJoint_createFromOpenRAVEXML(RcsGraph* self, xmlNode* node,
                                                const double* q0)
{
  char msg[RCS_MAX_NAMELEN];
  bool verbose = false;
  int strLength = 0;

  // if type is fix, we do not create the joint
  getXMLNodePropertyStringN(node, "type", msg, RCS_MAX_NAMELEN);
  if (STREQ(msg, "fix"))
  {
    return NULL;
  }


  // first find body that the joint is attached to
  /// \todo: offsetfrom is used and Body nodes are ignored,
  // that may not be valid for all OpenRAVE files
  xmlNodePtr child = getXMLChildByName(node, "offsetfrom");
  RCHECK_MSG(child, "Couldn't find tag \"offsetfrom\"");


  char buff[RCS_MAX_NAMELEN];
  getXMLNodeString(child, buff);

  RcsBody* bdy = RcsGraph_getBodyByName(self, buff);
  RCHECK(bdy);

  RcsJoint* jnt = RcsGraph_insertGraphJoint(self, bdy->id);

  jnt->weightJL = 1.0;
  jnt->weightCA = 1.0;
  jnt->weightMetric = 1.0;
  jnt->maxTorque = 1.0;
  jnt->ctrlType = RCSJOINT_CTRL_POSITION;

  //  Joint name
  strLength = getXMLNodePropertyStringN(node, "name", NULL, 0);

  if (strLength > 0)
  {
    getXMLNodePropertyStringN(node, "name", jnt->name, RCS_MAX_NAMELEN);
  }
  else
  {
    snprintf(jnt->name, RCS_MAX_NAMELEN, "%s", "unnamed joint");
    RLOG(4, "A joint between bodies \"%s\" and \"%s\" has no name - using \""
         "unnamed joint\"", bdy->name,
         bdy->parentId != -1 ? self->bodies[bdy->parentId].name : "NULL");
  }

  RLOG(5, "Inserted joint \"%s\" into body \"%s\"", jnt->name, bdy->name);

  if (verbose == true)
  {
    RMSG("Body %s: Adding joint with id = %d", bdy->name, jnt->id);
  }

  // joints in OpenRAVE format don't have transformations, but we have
  // thus the first joint in the list of joints of the body, takes over the
  // body's transformation
  if (bdy->jntId == jnt->id)
  {
    HTr_copy(&jnt->A_JP, &bdy->A_BP);
    HTr_setIdentity(&bdy->A_BP);
  }

  /// \todo: Groups are not supported yet

  // Joint constraint
  /// \todo: Constrained joints are not supported yet
  jnt->constrained = false;

  // Joint type
  /// \todo: Currently only rotational joints are supported
  xmlNodePtr axis_node = getXMLChildByName(node, "axis");
  RCHECK(axis_node);

  int axis[3];
  getXMLNodeIntN(axis_node, axis, 3);

  if (STREQ(msg, "hinge"))
  {
    if (axis[0] == 1)
    {
      jnt->type = RCSJOINT_ROT_X;
      jnt->dirIdx = 0;
    }
    else if (axis[1] == 1)
    {
      jnt->type = RCSJOINT_ROT_Y;
      jnt->dirIdx = 1;
    }
    else if (axis[2] == 1)
    {
      jnt->type = RCSJOINT_ROT_Z;
      jnt->dirIdx = 2;
    }
  }
  else if (STREQ(msg, "slider"))
  {
    if (axis[0] == 1)
    {
      jnt->type = RCSJOINT_TRANS_X;
      jnt->dirIdx = 0;
    }
    else if (axis[1] == 1)
    {
      jnt->type = RCSJOINT_TRANS_Y;
      jnt->dirIdx = 1;
    }
    else if (axis[2] == 1)
    {
      jnt->type = RCSJOINT_TRANS_Z;
      jnt->dirIdx = 2;
    }
  }
  else
  {
    RFATAL("Joint \"%s\": Unknown joint type \"%s\"", jnt->name, msg);
  }

  // limits
  xmlNodePtr limits_node = getXMLChildByName(node, "limitsdeg");
  if (limits_node == NULL)
  {
    limits_node = getXMLChildByName(node, "limits");
  }
  RCHECK(limits_node);

  double range[2];
  getXMLNodeVecN(limits_node, range, 2);

  jnt->q_min = range[0];
  jnt->q_max = range[1];

  RCHECK(jnt->q_min <= jnt->q_max);
  if (RcsJoint_isRotation(jnt) == true)
  {
    jnt->q_min *= (M_PI / 180.0);
    jnt->q_max *= (M_PI / 180.0);
  }

  if (!q0)
  {
    // q0 from openrave model with tag "initial" [rad]
    xmlNodePtr initial_node = getXMLChildByName(node, "initial");
    if (initial_node != NULL)
    {
      double initial;
      getXMLNodeVecN(initial_node, &initial, 1);
      jnt->q0 = initial;
    }
    else
    {
      // there is no initial tag in openrave, so we set the half of the range
      jnt->q0 = (jnt->q_max + jnt->q_min) / 2.0;
      RLOG(1, "Initial tag is not found. q0 is set the half of the range");
    }

    if ((jnt->q0 < jnt->q_min) || (jnt->q0 > jnt->q_max))
    {
      RLOGS(1, "initial value not between q_min and q_max for joint \"%s\": q0"
            "is set the half of the range", jnt->name);
      jnt->q0 = (jnt->q_max + jnt->q_min) / 2.0;
    }
  }
  else
  {
    jnt->q0 = *q0;
  }

  jnt->q_init = jnt->q0;

  /// \todo: HGF coupled joint equations is not supported, this is just a workaround
  // Coupled joint
  strLength = getXMLNodePropertyStringN(node, "coupledTo", NULL, 0);

  if (strLength > 0)
  {
    getXMLNodePropertyStringN(node, "coupledTo", jnt->coupledJntName, RCS_MAX_NAMELEN);

    unsigned int polyGrad = getXMLNodeNumStrings(node, "couplingFactor");
    if (polyGrad == 0)
    {
      polyGrad = 1;
    }
    RLOG(5, "Coupled joint \"%s\" has %d parameters", jnt->name, polyGrad);
    RCHECK_MSG(polyGrad == 1, "Currently only polynomials of order 1 are "
               "supported, and not %d parameters", polyGrad);
    getXMLNodePropertyVecN(node, "couplingFactor", jnt->couplingPoly, polyGrad);
    jnt->nCouplingCoeff = polyGrad;
  }


  /// \todo: additional parameters like gearRatio and so on are not supported

  if (verbose)
  {
    RPAUSE();
  }

  return jnt;
}

static RcsBody* RcsBody_createFromOpenRAVEXML(RcsGraph* self, xmlNode* bdyNode,
                                              RcsBody* root)
{
  // Return if node is not a body node
  if (!isXMLNodeNameNoCase(bdyNode, "Body"))
  {
    return NULL;
  }

  // Body name
  char msg[RCS_MAX_NAMELEN];
  strcpy(msg, "unnamed body");

  // The name as indicated in the xml file
  getXMLNodePropertyStringN(bdyNode, "name", msg, RCS_MAX_NAMELEN);
  RCHECK_MSG(strncmp(msg, "GenericBody", 11) != 0,
             "The name \"GenericBody\" is reserved for internal use");

  /// \todo Groups not yet supported by OpenRAVE parser
  RcsBody* parentBdy = root;

  // Find predecessor
  xmlNodePtr child = getXMLChildByName(bdyNode, "offsetfrom");
  if (child)
  {
    char buff[RCS_MAX_NAMELEN];
    getXMLNodeString(child, buff);
    parentBdy = RcsGraph_getBodyByName(self, buff);
  }

  RcsBody* b = RcsGraph_insertGraphBody(self, parentBdy ? parentBdy->id : -1);

  // Fully qualified name
  snprintf(b->bdyXmlName, RCS_MAX_NAMELEN, "%s", msg);
  snprintf(b->name, RCS_MAX_NAMELEN, "%s", msg);

  // check for translation
  child = getXMLChildByName(bdyNode, "Translation");

  if (child && !root)
  {
    getXMLNodeVec3(child, b->A_BP.org);
  }

  // check for quat
  child = getXMLChildByName(bdyNode, "Quat");
  if (child && !root)
  {
    getXMLNodeQuat(child, b->A_BP.rot);
  }

  /// \todo Physics not supported yet

  /// \todo Rigid body joints not supported yet

  // Things one level deeper
  xmlNodePtr shapeNode = bdyNode->children;

  // Populate shapes array
  while (shapeNode != NULL)
  {
    if (isXMLNodeNameNoCase(shapeNode, "geom"))
    {
      RcsShape* sh = RcsBody_appendShape(b);
      RcsShape_initFromOpenRAVEXML(sh, shapeNode, b);
    }

    shapeNode = shapeNode->next;
  }

  // Dynamic properties
  Mat3d_setZero(b->Inertia.rot);

  // get mass
  xmlNodePtr mass_node = getXMLChildByName(bdyNode, "mass");

  if (mass_node)
  {
    char buff[RCS_MAX_NAMELEN];
    getXMLNodePropertyStringN(mass_node, "type", buff, RCS_MAX_NAMELEN);

    // we only support custom mass types for now
    RCHECK(STREQ(buff, "custom"));

    // get total node
    xmlNodePtr total_node = getXMLChildByName(mass_node, "total");
    if (total_node != NULL)
    {
      getXMLNodeVecN(total_node, &b->m, 1);
      RCHECK_MSG(b->m >= 0.0, "Body \"%s\" has negative mass: %f",
                 b->name, b->m);
    }
    else
    {
      RLOG(4, "Couldn't find node \"total\" for body \"%s\"", b->name);
    }

    // get cog
    xmlNodePtr com_node = getXMLChildByName(mass_node, "com");
    RCHECK(com_node);
    getXMLNodeVec3(com_node, b->Inertia.org);


    // get inertia
    xmlNodePtr inertia_node = getXMLChildByName(mass_node, "inertia");

    if (inertia_node)
    {
      getXMLNodeVecN(inertia_node, &b->Inertia.rot[0][0], 9);
    }
    else
    {
      // If not specified in the xml file, compute the inertia tensor based on the
      // volume of the shapes and the given body mass
      RcsBody_computeInertiaTensor(b, &b->Inertia);
    }
  }


  // Check if we have a finite inertia but no mass
  if (Mat3d_getFrobeniusnorm(b->Inertia.rot) > 0.0)
  {
    RCHECK_MSG(b->m > 0.0, "You specified a non-zero inertia but a zero mass "
               "for body \"%s\". Shame on you!", b->name);
  }

  /// \todo: Sensors are not supported yet

  return b;
}













void RcsGraph_createBodiesFromOpenRAVEFile(RcsGraph* self, RcsBody* parent,
                                           const char* configFile, double* q0,
                                           unsigned int nq)
{
  // Determine absolute file name of config file and copy the XML file name
  char filename[RCS_MAX_FILENAMELEN] = "";
  bool fileExists = Rcs_getAbsoluteFileName(configFile, filename);

  if (fileExists==false)
  {
    REXEC(4)
    {
      RMSG("Ressource path is:");
      Rcs_printResourcePath();
    }

    RFATAL("RcsGraph configuration file \"%s\" not found in "
           "ressource path - exiting", configFile ? configFile : "NULL");
  }

  // Read XML file
  xmlDocPtr doc;
  // Try to load file
  xmlNodePtr node = getXMLNodeForOpenRave(filename, &doc);

  if (node == NULL)
  {
    RLOG(1, "Failed to parse XML-file \"%s\" - returning NULL", filename);
    return;
  }

  RcsGraph_createBodiesFromOpenRAVENode(self, parent, node, q0, nq);


  // Free the xml memory
  xmlFreeDoc(doc);
}

void RcsGraph_createBodiesFromOpenRAVENode(RcsGraph* self, RcsBody* parent,
                                           const xmlNodePtr node, double* q0,
                                           unsigned int nq)
{
  if (node == NULL)
  {
    RLOG(1, "XML node is NULL - failed to create RcsGraph");
    return;
  }

  xmlNodePtr kinbody_node = getXMLChildByName(node, "KinBody");

  if (kinbody_node== NULL)
  {
    char msg[RCS_MAX_NAMELEN] = "";
    getXMLNodeName(node, msg);
    RFATAL("Couldn't find child node \"KinBody\" from node \"%s\"", msg);
  }

  xmlNodePtr child = kinbody_node->children;
  unsigned int jnt_idx = 0;

  // parse bodies
  while (child)
  {
    if (isXMLNodeNameNoCase(child, "Body"))
    {
      RcsBody_createFromOpenRAVEXML(self, child, parent);
      parent = NULL; // only the first parsed body will have it as root
    }
    else if (isXMLNodeNameNoCase(child, "Joint"))
    {
      RcsJoint* jnt = NULL;

      if (q0 != NULL)
      {
        jnt = RcsJoint_createFromOpenRAVEXML(self, child, jnt_idx < nq ? &q0[jnt_idx] : NULL);
      }
      else
      {
        jnt = RcsJoint_createFromOpenRAVEXML(self, child, NULL);
      }

      // we only increment if a valid joint has been created
      if (jnt != NULL)
      {
        jnt_idx++;
      }
    }

    child = child->next;
  }

  if (q0 != NULL)
  {
    RCHECK_MSG(jnt_idx == nq,
               "Number of joints provided in q0 tag (%u) does not match"
               "the number of joints in the OpenRave file (%u)", nq, jnt_idx);
  }

}
