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

#include "Rcs_URDFParser.h"
#include "EulerAngles.h"
#include "Rcs_body.h"
#include "Rcs_macros.h"
#include "Rcs_Mat3d.h"
#include "Rcs_parser.h"
#include "Rcs_resourcePath.h"
#include "Rcs_typedef.h"
#include "Rcs_utils.h"
#include "Rcs_Vec3d.h"
#include "Rcs_mesh.h"



/*******************************************************************************
 *
 ******************************************************************************/
static void HTr_fromURDFOrigin(HTr* A, double rpy[3], double xyz[3])
{
  double rpy_transformed[3];

  // convert XYZ static to XYZ relative Euler angles
  convertEulerAngles(rpy_transformed, EulOrdXYZr, rpy, EulOrdXYZs);

  Vec3d_copy(A->org, xyz);
  Mat3d_fromEulerAngles(A->rot, rpy_transformed);
}

/*******************************************************************************
 * The link element has these attributes: http://wiki.ros.org/urdf/XML/link
 ******************************************************************************/
static RcsShape* parseShapeURDF(xmlNode* node, RcsBody* body)
{
  // Allocate memory and set defaults
  RcsShape* shape = RALLOC(RcsShape);

  // origin (identity if not specified)
  double xyz[3], rpy[3];
  Vec3d_setZero(xyz);
  Vec3d_setZero(rpy);
  xmlNodePtr childNode = getXMLChildByName(node, "origin");
  if (childNode)
  {
    getXMLNodePropertyVec3(childNode, "xyz", xyz);
    getXMLNodePropertyVec3(childNode, "rpy", rpy);
  }

  HTr_fromURDFOrigin(&shape->A_CB, rpy, xyz);

  // default scale
  shape->scale = 1.0;

  // geometry
  xmlNodePtr geometry_node = getXMLChildByName(node, "geometry");
  RCHECK(geometry_node);
  RLOG(5, "URDF: Parsing \"%s\"", (const char*)geometry_node->name);
  xmlNodePtr geometry_type_node = NULL;

  if ((geometry_type_node = getXMLChildByName(geometry_node, "box")))
  {
    getXMLNodePropertyVec3(geometry_type_node, "size", shape->extents);
    shape->type = RCSSHAPE_BOX;
  }
  else if ((geometry_type_node = getXMLChildByName(geometry_node, "cylinder")))
  {
    getXMLNodePropertyDouble(geometry_type_node, "radius", &shape->extents[0]);
    getXMLNodePropertyDouble(geometry_type_node, "length", &shape->extents[2]);
    shape->type = RCSSHAPE_CYLINDER;
  }
  else if ((geometry_type_node = getXMLChildByName(geometry_node, "sphere")))
  {
    getXMLNodePropertyDouble(geometry_type_node, "radius", &shape->extents[0]);
    shape->type = RCSSHAPE_SPHERE;
  }
  else if ((geometry_type_node = getXMLChildByName(geometry_node, "mesh")))
  {
    shape->type = RCSSHAPE_MESH;

    char meshFile[256];
    getXMLNodePropertyStringN(geometry_type_node, "filename", meshFile, 256);
    RLOG(5, "Adding mesh file \"%s\"", meshFile);

    char meshFileFull[512] = "";

    if (STRNEQ(meshFile, "package://", 10))
    {
      const char* hgrDir = getenv("SIT");

      if (hgrDir != NULL)
      {
        snprintf(meshFileFull, 512, "%s%s%s", hgrDir,
                 "/Data/RobotMeshes/1.0/data/", &meshFile[10]);
      }

      if (File_exists(meshFileFull)==false)
      {
        RLOG(0, "File \"%s\" not found", meshFileFull);
      }

    }
    else
    {
      Rcs_getAbsoluteFileName(meshFile, meshFileFull);
    }

    shape->meshFile = RNALLOC(strlen(meshFileFull) + 1, char);
    strcpy(shape->meshFile, meshFileFull);

    if (File_exists(shape->meshFile) == false)
    {
      RLOG(4, "Mesh file \"%s\" not found!", meshFile);
      RFREE(shape->meshFile);
      shape->meshFile = NULL;
    }
    else
    {
      RcsMeshData* mesh = RcsMesh_createFromFile(shape->meshFile);
      if (mesh==NULL)
      {
        RLOG(4, "Couldn't create mesh for file \"%s\"", shape->meshFile);
      }
      shape->userData = (void*) mesh;
    }

    double scale_3d[3] = {1.0, 1.0, 1.0};
    getXMLNodePropertyVec3(geometry_type_node, "scale", scale_3d);
    RCHECK_MSG(scale_3d[0] == scale_3d[1] && scale_3d[0] == scale_3d[2],
               "Only uniform scaling is supported in Rcs");

    shape->scale = scale_3d[0];

    if ((RcsMeshData*) shape->userData)
    {
      RcsMesh_scale((RcsMeshData*) shape->userData, shape->scale);
    }
  }
  else
  {
    RLOG(1, "URDF: Unsupported shape type: \"%s\" (parent: \"%s\")",
         geometry_type_node ? (const char*)geometry_type_node->name : "NULL",
         (const char*)geometry_node->name);
  }

  // check for color and texture
  xmlNodePtr material_node = getXMLChildByName(node, "material");

  if (material_node)
  {
    xmlNodePtr color_node = getXMLChildByName(material_node, "color");
    xmlNodePtr texture_node = getXMLChildByName(material_node, "texture");

    if (color_node)
    {
      double rgba[4] = {1.0, 1.0, 1.0, 1.0};
      getXMLNodePropertyVecN(color_node, "rgba", rgba, 4);

      // create color string of form #RRGGBBAA
      char color_string[10];
      snprintf(color_string, 10, "#%2x%2x%2x%2x",
               (int)(rgba[0] * 255),
               (int)(rgba[1] * 255),
               (int)(rgba[2] * 255),
               (int)(rgba[3] * 255));

      shape->color = String_clone(color_string);
    }
    else
    {
      char shape_color[256] = "DEFAULT";
      shape->color = String_clone(shape_color);
    }

    if (texture_node)
    {
      // Texture file
      char str[256] = "";
      getXMLNodePropertyStringN(texture_node, "filename", str, 256);

      char fullname[512] = "";
      if (Rcs_getAbsoluteFileName(str, fullname))
      {
        shape->textureFile = String_clone(fullname);
      }
      else
      {
        RLOG(4, "Texture file \"%s\" in body \"%s\" not found!",
             str, body->name);
        shape->textureFile = NULL;
      }
    }
  }
  else
  {
    char shape_color[256] = "DEFAULT";
    shape->color = String_clone(shape_color);
  }

  // physics material is default
  char shapeMaterial[256] = "default";
  shape->material = String_clone(shapeMaterial);

  if (isXMLNodeNameNoCase(node, "visual"))
  {
    // set graphics true
    shape->computeType = RCSSHAPE_COMPUTE_GRAPHICS;
  }
  else
  {
    // set distance and physics true
    shape->computeType = RCSSHAPE_COMPUTE_DISTANCE | RCSSHAPE_COMPUTE_PHYSICS;
  }

  return shape;
}

/*******************************************************************************
 * See http://wiki.ros.org/urdf/XML/link
 ******************************************************************************/
static RcsBody* parseBodyURDF(xmlNode* node)
{
  // Return if node is not a body node
  if (!isXMLNodeNameNoCase(node, "link"))
  {
    RLOG(5, "Parsing URDF body but xml node doesn't contain link information");
    return NULL;
  }

  // Body name as indicated in the xml file
  int len = getXMLNodeBytes(node, "name");

  if (len==0)
  {
    RLOG(4, "Body name not specified in URDF description");
    return NULL;
  }

  RcsBody* body = RALLOC(RcsBody);

  // Default transformation
  body->A_BI = HTr_create();

  // Dynamic properties
  body->Inertia = HTr_create();
  Mat3d_setZero(body->Inertia->rot);

  body->xmlName = RNALLOC(len, char);
  getXMLNodePropertyStringN(node, "name", body->xmlName, len);
  RCHECK(body->xmlName);

  if (strncmp(body->xmlName, "GenericBody", 11) == 0)
  {
    RLOG(4, "The name \"GenericBody\" is reserved for internal use");
    RFREE(body->xmlName);
    return NULL;
  }

  RLOG(5, "Creating body \"%s\"", body->xmlName);

  // Fully qualified name
  body->name = String_clone(body->xmlName);

  // Go one level deeper
  node = node->children;

  // Allocate memory for shape node lists (visual and collision tags)
  size_t inertialTagCount = 0;
  size_t shapeCount = 0;
  size_t numCollisionShapes = getNumXMLNodes(node, "collision");
  size_t numShapes = getNumXMLNodes(node, "visual") +
                     numCollisionShapes;
  body->shape = RNALLOC(numShapes+1, RcsShape*);


  while (node != NULL)
  {
    if (isXMLNodeNameNoCase(node, "inertial"))
    {
      inertialTagCount++;

      xmlNodePtr childNode = getXMLChildByName(node, "origin");
      if (childNode)
      {
        getXMLNodePropertyVec3(childNode, "xyz", body->Inertia->org);
      }

      childNode = getXMLChildByName(node, "mass");
      if (childNode)
      {
        getXMLNodePropertyDouble(childNode, "value", &body->m);
      }

      childNode = getXMLChildByName(node, "inertia");
      if (childNode)
      {
        getXMLNodePropertyDouble(childNode, "ixx", &body->Inertia->rot[0][0]);
        getXMLNodePropertyDouble(childNode, "ixy", &body->Inertia->rot[0][1]);
        getXMLNodePropertyDouble(childNode, "ixz", &body->Inertia->rot[0][2]);
        body->Inertia->rot[1][0] = -body->Inertia->rot[0][1];
        getXMLNodePropertyDouble(childNode, "iyy", &body->Inertia->rot[1][1]);
        getXMLNodePropertyDouble(childNode, "iyz", &body->Inertia->rot[1][2]);
        body->Inertia->rot[2][0] = -body->Inertia->rot[0][2];
        body->Inertia->rot[2][1] = -body->Inertia->rot[1][2];
        getXMLNodePropertyDouble(childNode, "izz", &body->Inertia->rot[2][2]);
      }
    }  // "inertial"
    else if (isXMLNodeNameNoCase(node, "visual") ||
             isXMLNodeNameNoCase(node, "collision"))
    {
      body->shape[shapeCount] = parseShapeURDF(node, body);

      if (body->shape[shapeCount] != NULL)
      {
        shapeCount++;
      }
    } // "visual" or "collision"

    node = node->next;
  }


  // If not specified in the xml file, compute the inertia tensor based on the
  // volume of the shapes and the given body mass
  if (inertialTagCount==0)
  {
    RcsBody_computeInertiaTensor(body, body->Inertia);
  }

  // Check if we have a finite inertia but no mass
  if ((Mat3d_getFrobeniusnorm(body->Inertia->rot)>0.0) && (body->m<=0.0))
  {
    RLOG(4, "You specified a non-zero inertia but a zero mass for body \"%s\"."
         " Shame on you!", body->name);
  }

  // Rigid body joints not supported
  body->rigid_body_joints = false;

  // determine physics type
  if (body->m > 0.0)
  {
    // bodies with non-zero mass can be simulated dynamically
    body->physicsSim = RCSBODY_PHYSICS_DYNAMIC;
    if (numCollisionShapes == 0)
    {
      RLOG(1, "You specified a non-zero mass but no collision shapes for body "
           "\"%s\". It will not work in physics simulations", body->name);
      body->physicsSim = RCSBODY_PHYSICS_NONE;
    }
  }
  else if (numCollisionShapes > 0)
  {
    // collision, but no inertia
    body->physicsSim = RCSBODY_PHYSICS_KINEMATIC;
  }
  else
  {
    // neither inertia nor collision, ignore in physics
    body->physicsSim = RCSBODY_PHYSICS_NONE;
  }

  return body;
}

/*******************************************************************************
 *
 ******************************************************************************/
static RcsJoint* findJntByNameNoCase(const char* name, RcsJoint** jntVec)
{
  while (*jntVec)
  {
    if (STRCASEEQ(name, (*jntVec)->name))
    {
      return *jntVec;
    }
    jntVec++;
  }

  return NULL;
}

/*******************************************************************************
 *
 ******************************************************************************/
static RcsBody* findBdyByNameNoCase(const char* name, RcsBody** bdyVec)
{
  while (*bdyVec)
  {
    if (STRCASEEQ(name, (*bdyVec)->name))
    {
      return *bdyVec;
    }
    bdyVec++;
  }

  return NULL;
}

/*******************************************************************************
 * The joint element has these attributes: http://wiki.ros.org/urdf/XML/joint
 * \todo Floating joints
 ******************************************************************************/
RcsJoint* parseJointURDF(xmlNode* node)
{
  // Return if node is not a joint node
  if (!isXMLNodeNameNoCase(node, "joint"))
  {
    RLOG(5, "Parsing URDF joint but xml node doesn't contain joint data");
    return NULL;
  }

  // Name as given in the xml file. Contains the trailing zero.
  int len = getXMLNodeBytes(node, "name");
  RCHECK_MSG(len>0, "Joint name not specified in URDF description");

  // Relative transformation: "origin" (NULL if not specified means identity
  // transform)
  xmlNodePtr originNode = getXMLChildByName(node, "origin");
  HTr* A_JP = NULL;
  if (originNode)
  {
    double xyz[3], rpy[3];
    Vec3d_setZero(xyz);
    Vec3d_setZero(rpy);
    getXMLNodePropertyVec3(originNode, "xyz", xyz);
    getXMLNodePropertyVec3(originNode, "rpy", rpy);
    A_JP = HTr_create();
    HTr_fromURDFOrigin(A_JP, rpy, xyz);
  }

  // We create the memory here since for fixed joints, none is needed
  RcsJoint* jnt = RALLOC(RcsJoint);
  jnt->name = RNALLOC(len, char);
  getXMLNodePropertyStringN(node, "name", jnt->name, len);
  jnt->weightJL = 1.0;
  jnt->weightCA = 1.0;
  jnt->weightMetric = 1.0;
  jnt->ctrlType = RCSJOINT_CTRL_POSITION;
  jnt->constrained = false;
  HTr_setIdentity(&jnt->A_JI);

  if (A_JP!=NULL)
  {
    // We only create a relative transform if it is not identity.
    if (HTr_isIdentity(A_JP)==false)
    {
      jnt->A_JP = HTr_clone(A_JP);
    }
    RFREE(A_JP);
  }

  // limits tag
  xmlNodePtr limitNode = getXMLChildByName(node, "limit");

  if (limitNode)
  {
    getXMLNodePropertyDouble(limitNode, "effort", &jnt->maxTorque);
    getXMLNodePropertyDouble(limitNode, "lower", &jnt->q_min);
    getXMLNodePropertyDouble(limitNode, "upper", &jnt->q_max);
    getXMLNodePropertyDouble(limitNode, "velocity", &jnt->speedLimit);
  }

  // Joint type. We skip the relative transformation at this point. The reason
  // lies in the axis direction specification. URDF allows for an arbitrary
  // axis direction vector for linear and rotational degrees of freedom. To
  // match this properly to the model, we need to apply an additional rotation
  // to the joint, and its transpose to the child body.Therefore this
  // computation is shifted to a later point (where bodies are connected
  // through joints: connectURDF()).
  char type[256] = "";
  len = getXMLNodePropertyStringN(node, "type", type, 256);
  RCHECK(len > 0);

  // \todo: This might have influence on the weighting matrices in the IK.
  //        We therefore constrain continuous joints here.
  if (STRCASEEQ(type, "continuous"))
  {
    jnt->q_min = -1.0;
    jnt->q_max =  1.0;
    jnt->constrained = true;
  }

  jnt->q0 = (jnt->q_max + jnt->q_min) / 2.0;
  jnt->q_init = jnt->q0;

  // Create coupled joint if mimic tag is present
  xmlNodePtr mimicNode = getXMLChildByName(node, "mimic");
  if (mimicNode)
  {
    // Coupled to is required
    char coupledTo[256] = "";
    len = getXMLNodePropertyStringN(mimicNode, "joint", coupledTo, 256);
    RCHECK_MSG(len > 0, "Couldn't find tag \"joint\" in mimic joint %s",
               jnt->name);
    jnt->coupledJointName = String_clone(coupledTo);

    // Here we force-set the constraint of the joint. This invalidates the
    // joint coupling projection. The joint is not treated in the inverse
    // kinematics, and its value is kinematically overwritten after the
    // forward kinematics step.
    jnt->constrained = true;

    // Multiplier is optional (default: 1.0)
    jnt->couplingFactors = MatNd_create(1, 1);
    MatNd_setElementsTo(jnt->couplingFactors, 1.0);
    getXMLNodePropertyDouble(mimicNode, "multiplier", jnt->couplingFactors->ele);

    RLOG(5, "Joint \"%s\" coupled to \"%s\" with factor %lf",
         jnt->name, jnt->coupledJointName, *jnt->couplingFactors->ele);
  }

  return jnt;
}

/*******************************************************************************
 * Connect bodies and joints.
 * URDF is a bit limited in the sense that there's only one actuated joint
 * between two links. This makes it simple.
 ******************************************************************************/
static void connectURDF(xmlNode* node, RcsBody** bdyVec, RcsJoint** jntVec,
                        const char* suffix)
{
  char jointName[256] = "";
  unsigned len = getXMLNodePropertyStringN(node, "name", jointName, 256);
  RCHECK(len > 0);

  xmlNodePtr parentNode = getXMLChildByName(node, "parent");
  xmlNodePtr childNode = getXMLChildByName(node, "child");
  RCHECK_MSG(parentNode && childNode, "\"parent\" and \"child\" are "
             "required in joint definition of \"%s\"", jointName);

  char parentName[256] = "";
  getXMLNodePropertyStringN(parentNode, "link", parentName, 256);
  if (suffix != NULL)
  {
    strcat(parentName, suffix);
  }
  RcsBody* parentBody = findBdyByNameNoCase(parentName, bdyVec);
  RCHECK_MSG(parentBody, "Parent body \"%s\" not found (joint "
             "definition \"%s\")", parentName, jointName);

  char childName[256] = "";
  getXMLNodePropertyStringN(childNode, "link", childName, 256);
  if (suffix != NULL)
  {
    strcat(childName, suffix);
  }
  RcsBody* childBody = findBdyByNameNoCase(childName, bdyVec);
  RCHECK_MSG(childBody, "Child body \"%s\" not found (joint "
             "definition \"%s\")", childName, jointName);

  RLOG(5, "Connecting joint \"%s\": parent=%s   child=%s",
       jointName, parentName, childName);

  // Joint type
  char type[256] = "";
  len = getXMLNodePropertyStringN(node, "type", type, 256);
  RCHECK(len > 0);



  if (STRCASEEQ(type, "revolute") ||
      STRCASEEQ(type, "continuous") ||
      STRCASEEQ(type, "prismatic"))
  {
    bool prismatic = STRCASEEQ(type, "prismatic");

    // The child has only one parent
    RCHECK(childBody->parent == NULL);
    childBody->parent = parentBody;

    // Child connectivity
    // If it is the first child, child and last are the same
    if (parentBody->firstChild == NULL)
    {
      parentBody->firstChild = childBody;
      parentBody->lastChild = childBody;
    }
    // If there are already children, we need to consider the prev, next
    // and last pointers
    else
    {
      RcsBody* lastChild = parentBody->lastChild;
      lastChild->next = childBody;
      childBody->prev = lastChild;
      parentBody->lastChild = childBody;
    }

    // Backward connection of joints
    char jntName[256] = "";
    getXMLNodePropertyStringN(node, "name", jntName, 256);
    if (suffix != NULL)
    {
      strcat(jntName, suffix);
    }
    RcsJoint* jnt = findJntByNameNoCase(jntName, jntVec);
    RCHECK_MSG(jnt, "Joint \"%s\" not found", jntName);

    // Relative rotation.
    jnt->dirIdx = 2;   // URDF joint direction default is x
    jnt->type = prismatic ? RCSJOINT_TRANS_Z : RCSJOINT_ROT_Z;

    xmlNodePtr axisNode = getXMLChildByName(node, "axis");

    if (axisNode)
    {
      double axis_xyz[3];
      Vec3d_set(axis_xyz, 1.0, 0.0, 0.0);
      getXMLNodePropertyVec3(axisNode, "xyz", axis_xyz);

      // Axis "should" be normalized according to the URDF specs. We therefore
      // only emit a warning.
      if (Vec3d_sqrLength(axis_xyz)<1.0e-3)
      {
        RLOG(1, "Axis vector of joint \"%s\" is degraded: [%f %f %f] - should"
             " be normalized",
             jnt->name, axis_xyz[0], axis_xyz[1], axis_xyz[2]);
      }

      // We do these quick checks since that's probably valid for most
      // of the joints. In the cases where the axes are aligned with
      // ex, ey or ez, we don't need to create a relative transformation
      // for the sake of the joint axis direction. That's a bit more
      // efficient in the forward kinematics, since we save a few
      // rotations.
      if ((axis_xyz[0]==1.0) && (axis_xyz[1]==0.0) && (axis_xyz[2]==0.0))
      {
        jnt->dirIdx = 0;
        jnt->type = prismatic ? RCSJOINT_TRANS_X : RCSJOINT_ROT_X;
      }
      else if ((axis_xyz[0]==0.0) && (axis_xyz[1]==1.0) && (axis_xyz[2]==0.0))
      {
        jnt->dirIdx = 1;
        jnt->type = prismatic ? RCSJOINT_TRANS_Y : RCSJOINT_ROT_Y;
      }
      else if ((axis_xyz[0]==0.0) && (axis_xyz[1]==0.0) && (axis_xyz[2]==1.0))
      {
        jnt->dirIdx = 2;
        jnt->type = prismatic ? RCSJOINT_TRANS_Z : RCSJOINT_ROT_Z;
      }
      else   // Axis is skew
      {
        NLOG(0, "Need to fix axis transform for joint \"%s\" has weird axis "
             "direction: [%f %f %f]",
             jnt->name, axis_xyz[0], axis_xyz[1], axis_xyz[2]);
        jnt->dirIdx = 2;
        jnt->type = prismatic ? RCSJOINT_TRANS_Z : RCSJOINT_ROT_Z;

        double A_NJ[3][3];
        Mat3d_fromVec(A_NJ, axis_xyz, jnt->dirIdx);

        // The relative transform is A_NV = A_NJ*A_JV
        if (jnt->A_JP != NULL)
        {
          Mat3d_preMulSelf(jnt->A_JP->rot, A_NJ);
        }
        else
        {
          jnt->A_JP = HTr_create();
          Mat3d_copy(jnt->A_JP->rot, A_NJ);
        }

        // Apply transpose of this to child body
        if (childBody->A_BP == NULL)
        {
          childBody->A_BP = HTr_create();
          Mat3d_transpose(childBody->A_BP->rot, A_NJ);
        }
        // The relative transform is A_KN = A_BP*A_JN
        else
        {
          RLOG(0, "TODO: Check axis transform for joint \"%s\"", jnt->name);
          double A_JN[3][3];
          Mat3d_transpose(A_JN, A_NJ);
          Mat3d_postMulSelf(jnt->A_JP->rot, A_JN);
        }
      }    // Axis is skew

    }   // if (axisNode)

    // Bodie's "driving" joint
    childBody->jnt = jnt;

    // Find previous ("driving") joint of the joint for the Jacobian
    // backward traversal.
    RcsJoint* prevJnt = RcsBody_lastJointBeforeBody(parentBody);
    jnt->prev = prevJnt;
    jnt->next = NULL;

  }   // if (STRCASEEQ(type, "revolute") || STRCASEEQ(type, "continuous"))

  else if (STRCASEEQ(type, "fixed"))
  {
    // Relative transformation: "origin" (NULL if not specified means identity
    // transform)
    xmlNodePtr originNode = getXMLChildByName(node, "origin");
    if (originNode)
    {
      double xyz[3], rpy[3];
      Vec3d_setZero(xyz);
      Vec3d_setZero(rpy);
      getXMLNodePropertyVec3(originNode, "xyz", xyz);
      getXMLNodePropertyVec3(originNode, "rpy", rpy);
      HTr A_rel;
      HTr_fromURDFOrigin(&A_rel, rpy, xyz);
      if (HTr_isIdentity(&A_rel)==false)
      {
        childBody->A_BP = HTr_clone(&A_rel);
      }
    }

    // The child has only one parent
    RCHECK(childBody->parent == NULL);
    childBody->parent = parentBody;

    // Child connectivity
    // If it is the first child, child and last are the same
    if (parentBody->firstChild == NULL)
    {
      parentBody->firstChild = childBody;
      parentBody->lastChild = childBody;
      childBody->prev = NULL;
      childBody->next = NULL;
    }
    // If there are already children, we need to consider the prev, next
    // and last pointers
    else
    {
      RcsBody* lastChild = parentBody->firstChild;

      while (lastChild->next)
      {
        lastChild = lastChild->next;
      }
      RCHECK(lastChild);
      lastChild->next = childBody;
      childBody->prev = lastChild;
      parentBody->lastChild = childBody;
    }

    if (childBody->physicsSim == RCSBODY_PHYSICS_DYNAMIC)
    {
      // Rcs physics module expects physics=fixed for fixed joints
      childBody->physicsSim = RCSBODY_PHYSICS_FIXED;
    }

  }   // STRCASEEQ(type, "fixed")

  else if (STRCASEEQ(type, "floating"))
  {
    RFATAL("Implement me");
  }
  else
  {
    RLOG(0, "Unknown joint type: %s", type);
  }

}

/*******************************************************************************
 * <model_state>
 *
 *    model (required) (string)
 *       The name of the model in corresponding URDF.
 *
 *    time_stamp (optional) (float) (sec)
 *        Time stamp of this state in seconds.
 *
 *        <joint_state> (optional) (string)
 *
 *           joint (required) (string)
 *                The name of the joint this state refers to.
 *
 *            position (optional) (float or array of floats)
 *                position for each degree of freedom of this joint
 *
 *            velocity (optional) (float or array of floats)
 *                velocity for each degree of freedom of this joint
 *
 *            effort (optional) (float or array of floats)
 *                effort for each degree of freedom of this joint
 *
 *
 * Example:
 *      <model_state model="pr2" time_stamp="0.1">
 *        <joint_state joint="r_shoulder_pan_joint" position="0" velocity="0"
 *                     effort="0"/>
 *        <joint_state joint="r_shoulder_lift_joint" position="0" velocity="0"
 *                     effort="0"/>
 *      </model_state>
 *
 ******************************************************************************/
static void RcsGraph_parseModelState(const char* modelName, xmlNodePtr node,
                                     RcsJoint** jntVec, const char* suffix)
{
  node = node->children;

  while (node != NULL)
  {
    if (isXMLNodeNameNoCase(node, "model_state"))
    {
      xmlNodePtr jntStateNode = node->children;

      while (jntStateNode != NULL)
      {
        if (isXMLNodeNameNoCase(jntStateNode, "joint_state"))
        {
          char jntName[256] = "";
          getXMLNodePropertyStringN(jntStateNode, "joint", jntName, 256);
          if (suffix != NULL)
          {
            strcat(jntName, suffix);
          }
          RcsJoint* jnt = findJntByNameNoCase(jntName, jntVec);
          RCHECK_MSG(jnt, "Joint \"%s\" not found", jntName);
          getXMLNodePropertyDouble(jntStateNode, "position", &jnt->q_init);
          jnt->q0 = jnt->q_init;
        }

        jntStateNode = jntStateNode->next;
      }

    }

    node = node->next;
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
RcsBody* RcsGraph_rootBodyFromURDFFile(const char* filename,
                                       const char* suffix,
                                       const HTr* A_BP,
                                       unsigned int* dof)
{
  // Read XML file
  xmlDocPtr doc;
  xmlNodePtr node = parseXMLFile(filename, "robot", &doc);

  if (node == NULL)
  {
    RLOG(1, "Failed to parse XML-file \"%s\" - returning NULL", filename);
    return NULL;
  }

  // Name of the model
  char modelName[256] = "";
  int len = getXMLNodeBytes(node, "name");

  if (len==0)
  {
    RLOG(1, "Robot name not specified in URDF description");
  }
  else
  {
    getXMLNodePropertyStringN(node, "name", modelName, 256);
  }

  // Parse all links and put them into a vector. We do this separately here,
  // since in some urdf files the link ordering does not match the depth-
  // first traversal order. Therefore we cannot create the link connections,
  // since a parent body might not yet have been created.
  xmlNodePtr linkNode = node->children;
  unsigned int numLinks = getNumXMLNodes(linkNode, "link");
  RLOG(5, "Found %d links", numLinks);

  RcsBody** bdyVec = RNALLOC(numLinks+1, RcsBody*);
  unsigned int bdyIdx = 0;

  while (linkNode != NULL)
  {
    RcsBody* b = parseBodyURDF(linkNode);
    if (b != NULL)
    {
      RLOG(5, "Adding body %d: %s", bdyIdx, b->name);
      if (suffix != NULL)
      {
        size_t nameLen = strlen(b->name) + strlen(suffix) + 1;
        char* newName = RNALLOC(nameLen, char);
        strcpy(newName, b->name);
        strcat(newName, suffix);
        String_copyOrRecreate(&b->name, newName);
        RFREE(newName);
      }

      RCHECK(bdyIdx<numLinks);
      bdyVec[bdyIdx++] = b;
    }
    linkNode = linkNode->next;
  }
  RCHECK_MSG(bdyIdx==numLinks, "%d != %d", bdyIdx, numLinks);

  // Parse all joints. We do this separately here for the same reason as
  // stated above for the link creation. The connection is done below.
  xmlNodePtr jntNode = node->children;
  unsigned int numJnts = getNumXMLNodes(jntNode, "joint");
  RLOG(5, "Found %d joints", numJnts);

  RcsJoint** jntVec = RNALLOC(numJnts+1, RcsJoint*);
  unsigned int jntIdx = 0;

  while (jntNode != NULL)
  {
    RcsJoint* j = parseJointURDF(jntNode);
    if (j != NULL)
    {
      RLOG(5, "Adding joint %d: %s", jntIdx, j->name);
      if (suffix != NULL)
      {
        size_t nameLen = strlen(j->name) + strlen(suffix) + 1;
        char* newName = RNALLOC(nameLen, char);
        strcpy(newName, j->name);
        strcat(newName, suffix);
        String_copyOrRecreate(&j->name, newName);
        RFREE(newName);

        // Also need to add the suffix to the coupledJointName
        if (j->coupledJointName != NULL)
        {
          size_t nameLen = strlen(j->coupledJointName) + strlen(suffix) + 1;
          char* newName = RNALLOC(nameLen, char);
          strcpy(newName, j->coupledJointName);
          strcat(newName, suffix);
          String_copyOrRecreate(&j->coupledJointName, newName);
          RFREE(newName);
        }
      }
      RCHECK(jntIdx<numJnts);
      jntVec[jntIdx++] = j;
    }
    jntNode = jntNode->next;
  }
  RCHECK_MSG(jntIdx==numJnts, "%d != %d", jntIdx, numJnts);

  if (dof != NULL)
  {
    *dof = numJnts;
  }

  // Connect bodies and joints. Here we have already created all joints and
  // links. Therefore, we can create all connections. There is only one
  // assumption: Each link has a unique name. This is required since we do the
  // connections by searching links by name in the recently created body and
  // joint arrays.
  xmlNodePtr childNode = node->children;
  while (childNode != NULL)
  {
    if (isXMLNodeNameNoCase(childNode, "joint"))
    {
      connectURDF(childNode, bdyVec, jntVec, suffix);
    }
    childNode = childNode->next;
  }


  // Parse model state and apply values to each joints q0 and q_init.
  RcsGraph_parseModelState(modelName, node, jntVec, suffix);

  // Create graph and find the root node. It is the first one without parent.
  // Consecutive parent-less nodes are attached it in a prev-next style.
  RcsBody* root = NULL;
  RcsBody** bvPtr = bdyVec;

  while (*bvPtr)
  {
    // We found a top-level body with no parent
    if ((*bvPtr)->parent==NULL)
    {
      // If root has not yet been assigned, we assign the first found top-
      // level body as the root. This is arbitrary, also the consecutive
      // top-level links could be assigned.
      if (root==NULL)
      {
        root = *bvPtr;
        RLOG(5, "Found root link: %s", root->name);
      }
      // If root has already been assigned, we assign the found top-level
      // body as the last next-body on the top-level.
      else  // root already exists
      {
        // Find last body on the first level
        unsigned int nextCount = 0;
        RcsBody* b = root;
        while (b->next)
        {
          b = b->next;
          nextCount++;
        }

        // Connect new parent-less body with last ones
        b->next = *bvPtr;
        (*bvPtr)->prev = b;

        RLOG(5, "Found the %d root link: %s (root is %s)",
             nextCount+1, (*bvPtr)->name, root->name);
      }

      // Apply relative transformation
      RLOG(5, "Applying relative transformation");
      if (A_BP != NULL)
      {
        if ((*bvPtr)->A_BP == NULL)
        {
          (*bvPtr)->A_BP = HTr_create();
        }

        HTr_transformSelf((*bvPtr)->A_BP, A_BP);
      }

    }

    bvPtr++;
  }   // while (*bvPtr)

  RCHECK_MSG(root, "Couldn't find root link in URFD model - did you "
             "define a cyclic model?");

  // Clean up
  xmlFreeDoc(doc);
  RFREE(bdyVec);
  RFREE(jntVec);

  return root;
}

/*******************************************************************************
 *
 ******************************************************************************/
RcsGraph* RcsGraph_fromURDFFile(const char* configFile)
{
  // Determine absolute file name of config file and copy the XML file name
  char filename[256] = "";
  bool fileExists = Rcs_getAbsoluteFileName(configFile, filename);

  if (fileExists==false)
  {
    REXEC(4)
    {
      RMSG("Resource path is:");
      Rcs_printResourcePath();
    }

    RLOG(1, "RcsGraph configuration file \"%s\" not found in "
         "ressource path - exiting", configFile ? configFile : "NULL");

    return NULL;
  }

  RcsGraph* self = RALLOC(RcsGraph);
  self->root = RcsGraph_rootBodyFromURDFFile(filename, NULL, NULL, NULL);

  RCSGRAPH_TRAVERSE_JOINTS(self)
  {
    JNT->jointIndex = self->dof;
    self->dof++;

    if (JNT->constrained == true)
    {
      JNT->jacobiIndex = -1;
    }
    else
    {
      JNT->jacobiIndex = self->nJ;
      self->nJ++;
    }
  }

  RLOG(5, "Found %d dof with %d active ones", self->dof, self->nJ);

  // Create state vectors
  self->q  = MatNd_create(self->dof, 1);
  self->q_dot = MatNd_create(self->dof, 1);
  self->xmlFile = String_clone(configFile);

  // Initialize generic bodies. Here we allocate memory for names and body
  // transforms. They are deleted once relinked to another body.
  for (int i = 0; i < 10; i++)
  {
    memset(&self->gBody[i], 0, sizeof(RcsBody));
    self->gBody[i].name      = RNALLOC(64, char);
    self->gBody[i].xmlName   = RNALLOC(64, char);
    self->gBody[i].suffix    = RNALLOC(64, char);
    self->gBody[i].A_BI      = HTr_create();
    self->gBody[i].A_BP      = HTr_create();
    self->gBody[i].Inertia   = HTr_create();
    HTr_setZero(self->gBody[i].Inertia);
    sprintf(self->gBody[i].name, "GenericBody%d", i);
  }

  // Order joint indices according to depth-first traversal and compute
  // forward kinematics
  RcsGraph_makeJointsConsistent(self);
  RcsGraph_setState(self, NULL, NULL);

  // Check for consistency
  int graphErrors = RcsGraph_check(self);
  RCHECK_MSG(graphErrors == 0, "Check for graph \"%s\" failed: %d errors",
             self->xmlFile, graphErrors);

  return self;
}
