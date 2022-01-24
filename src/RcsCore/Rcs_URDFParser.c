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

#include "Rcs_URDFParser.h"
#include "EulerAngles.h"
#include "Rcs_body.h"
#include "Rcs_joint.h"
#include "Rcs_shape.h"
#include "Rcs_macros.h"
#include "Rcs_Mat3d.h"
#include "Rcs_parser.h"
#include "Rcs_resourcePath.h"
#include "Rcs_typedef.h"
#include "Rcs_utils.h"
#include "Rcs_Vec3d.h"
#include "Rcs_mesh.h"

#include <float.h>



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
  RcsShape* shape = RcsShape_create();

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

    char meshFile[RCS_MAX_FILENAMELEN];
    getXMLNodePropertyStringN(geometry_type_node, "filename", meshFile,
                              RCS_MAX_FILENAMELEN);
    RLOG(5, "Adding mesh file \"%s\"", meshFile);

    char meshFileFull[512] = "";
    bool meshFileFound = false;

    // If there is a package:// attribute, we
    // - first try to find the mesh file in the resource path
    // - second try to find it in the SIT resource directory.
    if (STRNEQ(meshFile, "package://", 10))
    {
      // Iterate over ressource paths and try to find mesh
      unsigned int pathIdx = 0;
      const char* resourcePath = NULL;
      while ((resourcePath = Rcs_getResourcePath(pathIdx)) != NULL)
      {
        snprintf(meshFileFull, 512, "%s%s", resourcePath, &meshFile[10]);
        meshFileFound = File_exists(meshFileFull);

        if (meshFileFound)
        {
          RLOG(5, "Found mesh file \"%s\"", meshFileFull);
          break;
        }

        pathIdx++;
      }

      // Try to find in SIT data directory
      if (!meshFileFound)
      {
        const char* hgrDir = getenv("SIT");

        if (hgrDir != NULL)
        {
          snprintf(meshFileFull, 512, "%s%s%s", hgrDir,
                   "/Data/RobotMeshes/1.0/data/", &meshFile[10]);
          meshFileFound = File_exists(meshFileFull);
          RLOG(5, "%s to open file \"%s\"",
               meshFileFound ? "Success" : "Failed", &meshFile[10]);
        }
      }
    }
    else
    {
      meshFileFound = Rcs_getAbsoluteFileName(meshFile, meshFileFull);
    }

    int nchars = snprintf(shape->meshFile, RCS_MAX_FILENAMELEN,"%s",
                          meshFileFull);

    if (nchars>=RCS_MAX_FILENAMELEN-1)
    {
      RLOG(1, "Mesh file name truncation happened: %s", shape->meshFile);
    }

    if (!meshFileFound)
    {
      RLOG(4, "Mesh file \"%s\" not found!", meshFile);
    }
    else
    {
      shape->mesh = RcsMesh_createFromFile(shape->meshFile);
      if (shape->mesh==NULL)
      {
        RLOG(4, "Couldn't create mesh for file \"%s\"", meshFile);
      }
    }

    getXMLNodePropertyVec3(geometry_type_node, "scale", shape->scale3d);

    if (shape->mesh &&
        ((shape->scale3d[0]!=1.0) ||
         (shape->scale3d[1]!=1.0) ||
         (shape->scale3d[2]!=1.0)))
    {
      RcsMesh_scale3D(shape->mesh, shape->scale3d);
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
    xmlNodePtr texNode = getXMLChildByName(material_node, "texture");

    if (color_node)
    {
      double rgba[4] = {1.0, 1.0, 1.0, 1.0};
      getXMLNodePropertyVecN(color_node, "rgba", rgba, 4);

      // create color string of form #RRGGBBAA
      snprintf(shape->color, 10, "#%2x%2x%2x%2x",
               (int)(rgba[0] * 255),
               (int)(rgba[1] * 255),
               (int)(rgba[2] * 255),
               (int)(rgba[3] * 255));

    }
    else
    {
      strcpy(shape->color, "DEFAULT");
    }

    if (texNode)
    {
      // Texture file
      char str[RCS_MAX_FILENAMELEN] = "";
      getXMLNodePropertyStringN(texNode, "filename", str, RCS_MAX_FILENAMELEN);

      char fullname[RCS_MAX_FILENAMELEN] = "";
      if (Rcs_getAbsoluteFileName(str, fullname))
      {
        snprintf(shape->textureFile, RCS_MAX_FILENAMELEN, "%s", fullname);
      }
      else
      {
        RLOG(4, "Texture file \"%s\" in body \"%s\" not found!",
             str, body->name);
      }
    }
  }
  else
  {
    strcpy(shape->color, "DEFAULT");
  }

  // physics material is default
  strcpy(shape->material, "default");

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
 * Returns the body id. Since no connection information is available during
 * this stage of parsing, all bodies will be created on the top level.
 ******************************************************************************/
static int parseBodyURDF(xmlNode* node, RcsGraph* graph, int parentId)
{
  // Return if node is not a body node
  if (!isXMLNodeNameNoCase(node, "link"))
  {
    RLOG(5, "Parsing URDF body but xml node doesn't contain link information");
    return -1;
  }

  // Body name as indicated in the xml file
  char bdyName[RCS_MAX_NAMELEN];
  int len = getXMLNodePropertyStringN(node, "name", bdyName, RCS_MAX_NAMELEN);

  if (len==0)
  {
    RLOG(4, "Body name not specified in URDF description");
    return -1;
  }

  if (strncmp(bdyName, "GenericBody", 11) == 0)
  {
    RLOG(4, "The name \"GenericBody\" is reserved for internal use");
    return -1;
  }

  // Create new body in the graph's bodies array
  RcsBody* body = RcsGraph_insertGraphBody(graph, parentId);
  snprintf(body->bdyXmlName, RCS_MAX_NAMELEN, "%s", bdyName);

  RLOG(5, "Creating body \"%s\"", body->bdyXmlName);

  // Fully qualified name
  snprintf(body->name, RCS_MAX_NAMELEN, "%s", body->bdyXmlName);
  RLOG(5, "Adding body %s (%d) with parent %s (%d)",
       body->name, body->id,
       parentId<0?"NULL":graph->bodies[parentId].name,
       parentId);

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
        getXMLNodePropertyVec3(childNode, "xyz", body->Inertia.org);
      }

      childNode = getXMLChildByName(node, "mass");
      if (childNode)
      {
        getXMLNodePropertyDouble(childNode, "value", &body->m);
      }

      childNode = getXMLChildByName(node, "inertia");
      if (childNode)
      {
        getXMLNodePropertyDouble(childNode, "ixx", &body->Inertia.rot[0][0]);
        getXMLNodePropertyDouble(childNode, "ixy", &body->Inertia.rot[0][1]);
        getXMLNodePropertyDouble(childNode, "ixz", &body->Inertia.rot[0][2]);
        body->Inertia.rot[1][0] = -body->Inertia.rot[0][1];
        getXMLNodePropertyDouble(childNode, "iyy", &body->Inertia.rot[1][1]);
        getXMLNodePropertyDouble(childNode, "iyz", &body->Inertia.rot[1][2]);
        body->Inertia.rot[2][0] = -body->Inertia.rot[0][2];
        body->Inertia.rot[2][1] = -body->Inertia.rot[1][2];
        getXMLNodePropertyDouble(childNode, "izz", &body->Inertia.rot[2][2]);
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
    RcsBody_computeInertiaTensor(body, &body->Inertia);
  }

  // Check if we have a finite inertia but no mass
  if ((Mat3d_getFrobeniusnorm(body->Inertia.rot)>0.0) && (body->m<=0.0))
  {
    RLOG(4, "Found non-zero inertia but zero mass for body \"%s\".",
         body->name);
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
      RLOG(4, "You specified a non-zero mass but no collision shapes for body "
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

  return body->id;
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
static RcsBody* findBdyByNameNoCase(const char* name, RcsGraph* graph, int rootIdx)
{
  RCHECK(rootIdx>=0);
  RCHECK(rootIdx<(int)graph->nBodies);

  for (int i=rootIdx; i<(int)graph->nBodies; ++i)
  {
    RcsBody* b = &graph->bodies[i];
    if (STRCASEEQ(name, b->name))
    {
      return b;
    }
  }

  return NULL;
}

/*******************************************************************************
 * The joint element has these attributes: http://wiki.ros.org/urdf/XML/joint
 * \todo Floating joints
 ******************************************************************************/
static RcsJoint* parseJointURDF(xmlNode* node)
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

  // We create the memory here since for fixed joints, none is needed
  RcsJoint* jnt = RALLOC(RcsJoint);
  RcsJoint_init(jnt);

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
    HTr_fromURDFOrigin(&jnt->A_JP, rpy, xyz);
  }
  getXMLNodePropertyStringN(node, "name", jnt->name, RCS_MAX_NAMELEN);
  jnt->weightJL = 1.0;
  jnt->weightCA = 1.0;
  jnt->weightMetric = 1.0;
  jnt->ctrlType = RCSJOINT_CTRL_POSITION;
  jnt->constrained = false;

  // limits tag
  xmlNodePtr limitNode = getXMLChildByName(node, "limit");

  if (limitNode)
  {
    getXMLNodePropertyDouble(limitNode, "effort", &jnt->maxTorque);
    getXMLNodePropertyDouble(limitNode, "lower", &jnt->q_min);
    getXMLNodePropertyDouble(limitNode, "upper", &jnt->q_max);
    getXMLNodePropertyDouble(limitNode, "velocity", &jnt->speedLimit);
    jnt->accLimit = DBL_MAX;
  }

  // Joint type. We skip the relative transformation at this point. The reason
  // lies in the axis direction specification. URDF allows for an arbitrary
  // axis direction vector for linear and rotational degrees of freedom. To
  // match this properly to the model, we need to apply an additional rotation
  // to the joint, and its transpose to the child body.Therefore this
  // computation is shifted to a later point (where bodies are connected
  // through joints: connectURDF()).
  char type[RCS_MAX_NAMELEN] = "";
  len = getXMLNodePropertyStringN(node, "type", type, RCS_MAX_NAMELEN);
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
    len = getXMLNodePropertyStringN(mimicNode, "joint", jnt->coupledJntName,
                                    RCS_MAX_NAMELEN);
    RCHECK_MSG(len > 0, "Couldn't find tag \"joint\" in mimic joint %s",
               jnt->name);

    // Here we force-set the constraint of the joint. This invalidates the
    // joint coupling projection. The joint is not treated in the inverse
    // kinematics, and its value is kinematically overwritten after the
    // forward kinematics step.
    jnt->constrained = true;

    // Multiplier is optional (default: 1.0)
    jnt->couplingPoly[0] = 1.0;
    jnt->nCouplingCoeff = 1;
    getXMLNodePropertyDouble(mimicNode, "multiplier", &jnt->couplingPoly[0]);

    // Offset is optional (default: 0.0)
    // We temporarily write the offset into the q_init field which will be overwritten
    // with the correct value later on
    jnt->q_init = 0.0;
    getXMLNodePropertyDouble(mimicNode, "offset", &jnt->q_init);

    RLOG(5, "Joint \"%s\" coupled to \"%s\" with factor %lf",
         jnt->name, jnt->coupledJntName, jnt->couplingPoly[0]);
  }

  return jnt;
}

/*******************************************************************************
 * Connect bodies and joints.
 * URDF is a bit limited in the sense that there's only one actuated joint
 * between two links. This makes it simple.
 ******************************************************************************/
static void connectURDF(xmlNode* node, RcsGraph* graph, int rootIdx,
                        RcsJoint** jntVec, const char* suffix)
{
  char jointName[RCS_MAX_NAMELEN] = "";
  unsigned len = getXMLNodePropertyStringN(node, "name", jointName,
                                           RCS_MAX_NAMELEN);
  RCHECK(len > 0);

  xmlNodePtr parentNode = getXMLChildByName(node, "parent");
  xmlNodePtr childNode = getXMLChildByName(node, "child");
  RCHECK_MSG(parentNode && childNode, "\"parent\" and \"child\" are "
             "required in joint definition of \"%s\"", jointName);

  char parentName[RCS_MAX_NAMELEN] = "";
  getXMLNodePropertyStringN(parentNode, "link", parentName, RCS_MAX_NAMELEN);
  if (suffix != NULL)
  {
    int len = RCS_MAX_NAMELEN - strlen(parentName) - 1;
    RCHECK(len>0);
    strncat(parentName, suffix, len);
  }

  RcsBody* parentBody = findBdyByNameNoCase(parentName, graph, rootIdx);
  RCHECK_MSG(parentBody, "Parent body \"%s\" not found (joint "
             "definition \"%s\")", parentName, jointName);

  char childName[RCS_MAX_NAMELEN] = "";
  getXMLNodePropertyStringN(childNode, "link", childName, RCS_MAX_NAMELEN);
  if (suffix != NULL)
  {
    int len = RCS_MAX_NAMELEN - strlen(childName) - 1;
    RCHECK(len>0);
    strcat(childName, suffix);
  }
  RcsBody* childBody = findBdyByNameNoCase(childName, graph, rootIdx);
  RCHECK_MSG(childBody, "Child body \"%s\" not found (joint "
             "definition \"%s\")", childName, jointName);

  RLOG(5, "Connecting joint \"%s\": parent=%s   child=%s",
       jointName, parentName, childName);

  // Joint type
  char type[RCS_MAX_NAMELEN] = "";
  len = getXMLNodePropertyStringN(node, "type", type, RCS_MAX_NAMELEN);
  RCHECK(len > 0);





  // Re-connect. The child must have only one parent
  RCHECK(childBody->parentId == -1);
  childBody->parentId = parentBody->id;

  // Child connectivity: Take out body from where it was before
  RcsBody* prevBdy = RCSBODY_BY_ID(graph, childBody->prevId);
  RcsBody* nextBdy = RCSBODY_BY_ID(graph, childBody->nextId);

  if (prevBdy)
  {
    prevBdy->nextId = nextBdy ? nextBdy->id : -1;
  }

  if (nextBdy)
  {
    nextBdy->prevId = prevBdy ? prevBdy->id : -1;
  }

  // If it is the first child, child and last are the same
  if (parentBody->firstChildId == -1)
  {
    parentBody->firstChildId = childBody->id;
    parentBody->lastChildId = childBody->id;
    childBody->prevId = -1;
    childBody->nextId = -1;
  }
  // If there are already children, we need to consider the prev, next
  // and last pointers
  else
  {
    RcsBody* lastChild = RCSBODY_BY_ID(graph, parentBody->lastChildId);
    lastChild->nextId = childBody->id;
    childBody->prevId = lastChild->id;
    childBody->nextId = -1;
    parentBody->lastChildId = childBody->id;
  }
  // Done re-connect





  if (STRCASEEQ(type, "revolute") ||
      STRCASEEQ(type, "continuous") ||
      STRCASEEQ(type, "prismatic"))
  {
    bool prismatic = STRCASEEQ(type, "prismatic");

    // Backward connection of joints
    char name[RCS_MAX_NAMELEN] = "";
    getXMLNodePropertyStringN(node, "name", name, RCS_MAX_NAMELEN);
    if (suffix != NULL)
    {
      int len = RCS_MAX_NAMELEN - strlen(name) - 1;
      RCHECK(len>0);
      strcat(name, suffix);
    }
    RcsJoint* jnt_ = findJntByNameNoCase(name, jntVec);
    RCHECK_MSG(jnt_, "Joint \"%s\" not found", name);
    RcsJoint* jnt = RcsGraph_insertGraphJoint(graph, childBody->id);
    RcsJoint_copy(jnt, jnt_);

    // Relative rotation.
    jnt->dirIdx = 2;   // URDF joint direction default is x
    jnt->type = prismatic ? RCSJOINT_TRANS_Z : RCSJOINT_ROT_Z;

    xmlNodePtr axisNode = getXMLChildByName(node, "axis");

    if (axisNode)
    {
      double axis_xyz[3];
      Vec3d_setUnitVector(axis_xyz, 0);
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
        RLOG(0, "Need to fix axis transform for joint \"%s\" has weird axis "
             "direction: [%f %f %f]",
             jnt->name, axis_xyz[0], axis_xyz[1], axis_xyz[2]);
        jnt->dirIdx = 2;
        jnt->type = prismatic ? RCSJOINT_TRANS_Z : RCSJOINT_ROT_Z;

        double A_NJ[3][3];
        Mat3d_fromVec(A_NJ, axis_xyz, jnt->dirIdx);

        // The relative transform is A_NV = A_NJ*A_JV
        Mat3d_preMulSelf(jnt->A_JP.rot, A_NJ);

        // Apply transpose of this to child body
        //RFATAL("Fix this");
        /* if (childBody->A_BP == NULL) */
        /* { */
        /*   childBody->A_BP = HTr_create(); */
        Mat3d_transpose(childBody->A_BP.rot, A_NJ);
        /* } */
        // The relative transform is A_KN = A_BP*A_JN
        /* else */
        /* { */
        /* RLOG(0, "TODO: Check axis transform for joint \"%s\"", jnt->name); */
        /* double A_JN[3][3]; */
        /* Mat3d_transpose(A_JN, A_NJ); */
        /* Mat3d_postMulSelf(jnt->A_JP->rot, A_JN); */
        /* } */
      }    // Axis is skew

    }   // if (axisNode)

    // Bodie's "driving" joint
    childBody->jntId = jnt->id;

    // Find previous ("driving") joint of the joint for the Jacobian
    // backward traversal.
    RcsJoint* prevJnt = RcsBody_lastJointBeforeBody(graph, parentBody);
    jnt->prevId = prevJnt ? prevJnt->id : -1;
    jnt->nextId = -1;

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
      HTr_fromURDFOrigin(&childBody->A_BP, rpy, xyz);
    }

    if (childBody->physicsSim == RCSBODY_PHYSICS_DYNAMIC)
    {
      // Rcs physics module expects physics=fixed for fixed joints
      childBody->physicsSim = RCSBODY_PHYSICS_FIXED;
    }

  }   // STRCASEEQ(type, "fixed")

  else if (STRCASEEQ(type, "floating"))
  {
    RLOG(1, "Joint type \"floating\" not yet implemented - skipping");
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
static void RcsGraph_parseUrdfModelState(const char* modelName, xmlNodePtr node,
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
          char name[256] = "";
          getXMLNodePropertyStringN(jntStateNode, "joint", name, 256);
          if (suffix != NULL)
          {
            strcat(name, suffix);
          }
          RcsJoint* jnt = findJntByNameNoCase(name, jntVec);
          RCHECK_MSG(jnt, "Joint \"%s\" not found", name);
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
int RcsGraph_rootBodyFromURDFFile(RcsGraph* graph,
                                  const char* filename,
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
    return -1;
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

  int urdfRootId = graph->nBodies;

  while (linkNode != NULL)
  {
    // No connection information is available, so all bodies will be created
    // on the top level with parent-id being -1
    int bdyId = parseBodyURDF(linkNode, graph, -1);

    if ((bdyId!=-1) && (suffix != NULL))
    {
      RcsBody* b = &graph->bodies[bdyId];
      char newName[RCS_MAX_NAMELEN];
      snprintf(newName, RCS_MAX_NAMELEN, "%s%s", b->name, suffix);
      snprintf(b->name, RCS_MAX_NAMELEN, "%s", newName);

    }
    linkNode = linkNode->next;
  }

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
        char newName[RCS_MAX_NAMELEN];
        snprintf(newName, RCS_MAX_NAMELEN, "%s%s", j->name, suffix);
        snprintf(j->name, RCS_MAX_NAMELEN, "%s", newName);

        // Also need to add the suffix to the coupledJointName
        if (strlen(j->coupledJntName) != 0)
        {
          snprintf(newName, RCS_MAX_NAMELEN, "%s%s", j->coupledJntName, suffix);
          snprintf(j->coupledJntName, RCS_MAX_NAMELEN, "%s", newName);
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
      connectURDF(childNode, graph, urdfRootId, jntVec, suffix);
    }
    childNode = childNode->next;
  }

  // The urdf-file's root node is the first one without parent, starting from
  // the urdfRootId (last body entry before calling this function).
  // We do not need to take care about connecting bodies on the root level,
  // this has already happened by RcsGraph_insertGraphBody() with parent id
  // being -1.
  int nRootNodes = 0, idxStart = urdfRootId;
  urdfRootId = -1;
  for (int i=idxStart; i<(int)graph->nBodies; ++i)
  {
    RcsBody* bdy = &graph->bodies[i];

    if (bdy->parentId!=-1)
    {
      continue;
    }

    // We apply the A_BP transform to all bodies on the root level.
    if (A_BP)
    {
      HTr_copy(&bdy->A_BP, A_BP);
    }

    if (bdy->prevId==-1)
    {
      RLOG(5, "Found root node for body \"%s\"", bdy->name);
      urdfRootId = bdy->id;
      nRootNodes++;
    }

  }

  RCHECK_MSG(nRootNodes==1, "Found %d root bodies, but must be 1", nRootNodes);
  RCHECK_MSG(urdfRootId != -1, "Couldn't find root link in URFD model - did "
             "you define a cyclic model?");

  // Correct the joint order. When executing the connectURDF function,
  // previous joints do not necessarily exist (yet) which can mess up the joint
  // order for the Jacobian backward traversal. Therefore, after calling
  // connectURDF, this function must be called for making the joint order
  // consistent.
  RcsBody* rootBdy = RCSBODY_BY_ID(graph, graph->rootId);
  RCSBODY_TRAVERSE_BODIES(graph, rootBdy)
  {
    const RcsBody* parentBdy = RCSBODY_BY_ID(graph, BODY->parentId);
    const RcsJoint* prevJnt = RcsBody_lastJointBeforeBody(graph, parentBdy);

    RCSBODY_FOREACH_JOINT(graph, BODY)
    {
      if (prevJnt->id != JNT->prevId)
      {
        RLOG(5, "Joint \"%s\": previous joint changed from \"%s\" to \"%s\"",
             JNT->name, RCSBODY_NAME_BY_ID(graph, JNT->prevId),
             (prevJnt == NULL) ? "NULL" : prevJnt->name);
        JNT->prevId = prevJnt->id;
        JNT->nextId = -1;
      }
    }
  }

  // Correct q_init of mimic joints.  Here, we assume that the offset that is
  // read from the urdf file is stored in the q_init field.
  // Urdf mimic joint computation:
  //   q = c*q_master + o (c = coupling factor, o = offset)
  // Rcs  mimic joint computation:
  //   q = q_init + c*(q_master - q_master_init)
  // Hence, we can use the Rcs computation if we set q_init to:
  //   q_init = c*q_master_init - o
  RCSBODY_TRAVERSE_BODIES(graph, rootBdy)
  {
    RCSBODY_FOREACH_JOINT(graph, BODY)
    {
      if (strlen(JNT->coupledJntName)>0)
      {
        RcsJoint* master = NULL;
        for (unsigned int idx = 0; idx < numJnts; idx++)
        {
          if (STREQ(jntVec[idx]->name, JNT->coupledJntName))
          {
            master = jntVec[idx];
            break;
          }
        }
        RCHECK(master != NULL);
        RCHECK(JNT->nCouplingCoeff == 1);
        JNT->q_init = JNT->couplingPoly[0]*master->q_init + JNT->q_init;
      }
    }
  }

  // Parse model state and apply values to each joints q0 and q_init.
  RcsGraph_parseUrdfModelState(modelName, node, jntVec, suffix);

  // Clean up
  xmlFreeDoc(doc);
  RFREE(jntVec);

  return urdfRootId;
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
  snprintf(self->cfgFile, RCS_MAX_FILENAMELEN, "%s", configFile);
  self->rootId = RcsGraph_rootBodyFromURDFFile(self, filename, NULL, NULL, NULL);
  RCHECK(self->rootId!=-1);

  // Check that traversal leads to same number of bodies as in body array.
  int nBodies = 0;
  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    nBodies++;
  }

  RCHECK_MSG(self->nBodies==nBodies, "%d != %d", self->nBodies, nBodies);

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

  // Initialize generic bodies. Here we allocate memory for names and body
  // transforms. They are deleted once relinked to another body.
  for (int i = 0; i < RCS_NUM_GENERIC_BODIES; i++)
  {
    self->gBody[i] = -1;
  }

  // Order joint indices according to depth-first traversal and compute
  // forward kinematics
  RcsGraph_makeJointsConsistent(self);
  RcsGraph_setState(self, NULL, NULL);

  // Check for consistency
  int errs=0, warnings=0;
  RcsGraph_check(self, &errs, &warnings);
  RCHECK_MSG(errs==0, "Check for graph \"%s\" failed: %d errors %d warnings",
             self->cfgFile, errs, warnings);

  if (warnings>0)
  {
    RLOG(1, "Found %d warnings for graph \"%s\"", warnings, self->cfgFile);
  }

  return self;
}
