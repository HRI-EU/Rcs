/*******************************************************************************

  Copyright (c) Honda Research Institute Europe GmbH

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

#include "Rcs_mujocoParser.h"

#include <Rcs_typedef.h>
#include <Rcs_body.h>
#include <Rcs_shape.h>
#include <Rcs_joint.h>
#include <Rcs_utils.h>
#include <Rcs_math.h>
#include <Rcs_macros.h>
#include <Rcs_material.h>



/*******************************************************************************
 * |attrib="1 2 3 ... nEle" |
 ******************************************************************************/
static void writeArray(FILE* fd, const char* attrib, const double* arr,
                       int nEle, int digits)
{
  RCHECK(nEle>=1);
  char buf[64];
  fprintf(fd, "%s=\"", attrib);

  for (int i=0; i<nEle-1; ++i)
  {
    fprintf(fd, "%s ", String_fromDouble(buf, arr[i], digits));
  }

  fprintf(fd, "%s\" ", String_fromDouble(buf, arr[nEle-1], digits));
}

/*******************************************************************************
 * |pos="1 2 3" |
 ******************************************************************************/
static void writePos(FILE* fd, const double pos[3], int digits)
{
  writeArray(fd, "pos", pos, 3, digits);
}

/*******************************************************************************
 *
 ******************************************************************************/
static void parseJoint(FILE* fd,
                       const RcsBody* bdy,
                       const RcsJoint* jnt,
                       const char* indentStr)
{
  fprintf(fd, "%s  <joint name=\"%s\" ", indentStr, jnt->name);

  // type: [free, ball, slide, hinge]
  if (RcsJoint_isRotation(jnt))
  {
    fprintf(fd, "type=\"hinge\" ");
  }
  else
  {
    fprintf(fd, "type=\"slide\" ");
  }

  // Mujoco expects the joint anchor and axis in the frame of the body that
  // the joint belongs to. We therefore express the joint frame with respect
  // to the body frame. The rows of a Rcs rotation matrix are the axes of the
  // frame, therefore row dirIdx is the joint's axis of motion.
  HTr A_JB;
  HTr_invTransform(&A_JB, &bdy->A_BI, &jnt->A_JI);
  writePos(fd, A_JB.org, 6);
  writeArray(fd, "axis", A_JB.rot[jnt->dirIdx], 3, 6);

  fprintf(fd, "/>\n");
}

/*******************************************************************************
 *
 ******************************************************************************/
static void parseShape(FILE* fd,
                       const RcsBody* bdy,
                       const RcsShape* shape,
                       const char* indentStr)
{
  // Only the shapes that have a Mujoco counterpart are written out. Emitting
  // an unsupported shape with an invalid geom type makes the whole model fail
  // to load, therefore anything else is skipped here.
  switch (shape->type)
  {
    case RCSSHAPE_CYLINDER:
    case RCSSHAPE_SPHERE:
    case RCSSHAPE_SSL:
    case RCSSHAPE_SSR:
    case RCSSHAPE_BOX:
      break;

    case RCSSHAPE_MESH:
      // A mesh shape without a file cannot be referenced from the asset
      // section, and Mujoco rejects an asset with an empty name.
      if (!shape->meshFile || (shape->meshFile[0] == '\0'))
      {
        RLOG(4, "Skipping mesh shape of body %s: no mesh file", bdy->name);
        return;
      }
      break;

    default:
      RLOG(4, "Shape %s of body %s has no Mujoco counterpart - skipping",
           RcsShape_name(shape->type), bdy->name);
      return;
  }

  // Mujoco requires all geom sizes to be strictly positive. Rcs does permit
  // degenerate shapes, which we skip rather than letting the model fail. The
  // comparison is against the resolution of the written numbers and not
  // against zero: a smaller extent is written out as "0.000000", which Mujoco
  // rejects just the same.
  {
    const double minExtent = 1.0e-6;
    bool degenerate = false;

    switch (shape->type)
    {
      case RCSSHAPE_SPHERE:
      case RCSSHAPE_SSL:
        degenerate = (shape->extents[0] < minExtent);
        break;

      case RCSSHAPE_CYLINDER:
        degenerate = (shape->extents[0] < minExtent) ||
                     (shape->extents[2] < minExtent);
        break;

      case RCSSHAPE_SSR:
      case RCSSHAPE_BOX:
        degenerate = (shape->extents[0] < minExtent) ||
                     (shape->extents[1] < minExtent) ||
                     (shape->extents[2] < minExtent);
        break;

      default:
        break;
    }

    if (degenerate)
    {
      RLOG(4, "Skipping degenerate %s shape of body %s: extents %f %f %f",
           RcsShape_name(shape->type), bdy->name, shape->extents[0],
           shape->extents[1], shape->extents[2]);
      return;
    }
  }

  // Shape's transform with respect to the body frame. Mujoco interprets a
  // geom's transform relative to the body it is a child of, which is exactly
  // how Rcs stores it, so no composition is needed here.
  HTr A_CB;
  HTr_copy(&A_CB, &shape->A_CB);

  // All shapes of the body:
  // [plane, hfield, sphere, capsule, ellipsoid, cylinder, box, mesh]
  fprintf(fd, "%s  <geom ", indentStr);

  switch (shape->type)
  {
    case RCSSHAPE_CYLINDER:
    {
      double cylExtents[3];
      Vec3d_set(cylExtents, shape->extents[0], 0.5*shape->extents[2], 0.0);
      fprintf(fd, "type=\"cylinder\" ");
      writeArray(fd, "size", cylExtents, 2, 6);
      break;
    }

    case RCSSHAPE_SPHERE:
      fprintf(fd, "type=\"sphere\" ");
      writeArray(fd, "size", shape->extents, 1, 6);
      break;

    case RCSSHAPE_SSL:
    {
      double fromTo[6];
      Vec3d_copy(fromTo, A_CB.org);
      Vec3d_constMulAndAdd(fromTo + 3, A_CB.org, A_CB.rot[2], shape->extents[2]);
      fprintf(fd, "type=\"capsule\" ");
      writeArray(fd, "size", shape->extents, 1, 6);
      writeArray(fd, "fromto", fromTo, 6, 6);
      break;
    }

    case RCSSHAPE_SSR:
    case RCSSHAPE_BOX:
    {
      double halfExtents[3];
      Vec3d_constMul(halfExtents, shape->extents, 0.5);
      fprintf(fd, "type=\"box\" ");
      writeArray(fd, "size", halfExtents, 3, 6);
      break;
    }

    case RCSSHAPE_MESH:
    {
      char* meshFile = String_clone(shape->meshFile);
      String_removeSuffix(meshFile, shape->meshFile, '.');
      fprintf(fd, "type=\"mesh\" ");
      fprintf(fd, "mesh=\"%s\" ", String_stripPath(meshFile));
      RFREE(meshFile);
      break;
    }

    default:
      // Cannot happen: the shape types are filtered above.
      RFATAL("Unhandled shape type %d (%s)", shape->type,
             RcsShape_name(shape->type));
  }

  double ea_deg[3];
  Mat3d_toEulerAngles(ea_deg, A_CB.rot);
  Vec3d_constMulSelf(ea_deg, 180.0/M_PI);

  if (shape->type != RCSSHAPE_SSL)
  {
    writePos(fd, A_CB.org, 6);
    writeArray(fd, "euler", ea_deg, 3, 6);
  }

  // Color
  double geomRGBA[4];
  Rcs_colorFromString(shape->color, geomRGBA);
  writeArray(fd, "rgba", geomRGBA, 4, 6);

  fprintf(fd, "/>\n");
}

/*******************************************************************************
 *
 ******************************************************************************/
static void printBdy(FILE* fd,
                     const RcsGraph* graph,
                     const RcsBody* bdy,
                     const HTr* A_BA,
                     const char* indentStr)
{
  // Inertia tensor frame: Compute the transformation from the body (Index B)
  // to the COM (Index P) frame. The principal axes of inertia correspond to
  // the Eigenvectors. They are sorted in decreasing order.
  double diagInertia[3], ea_deg[3];
  HTr A_PB, A_rel;

  // Body to inertia frame
  Mat3d_getEigenVectors(A_PB.rot, diagInertia, (double(*)[3])bdy->Inertia.rot);
  Mat3d_transposeSelf(A_PB.rot);
  Vec3d_copy(A_PB.org, bdy->Inertia.org);

  // Body transform with respect to the enclosing Mujoco body (Index A)
  HTr_copy(&A_rel, A_BA);
  Mat3d_toEulerAngles(ea_deg, A_rel.rot);
  Vec3d_constMulSelf(ea_deg, 180.0/M_PI);

  fprintf(fd, "%s<body name=\"%s\" ", indentStr, bdy->name);
  writePos(fd, A_rel.org, 6);
  writeArray(fd, "euler", ea_deg, 3, 6);
  fprintf(fd, ">\n");

  // Mass and inertia properties. Both the COM and the principal axes are
  // already expressed with respect to the body frame, which is what Mujoco
  // expects for a child element of a body.
  //
  // Mujoco insists on a positive mass and inertia for any body that can move.
  // Rcs does allow massless bodies, so for these we omit the inertial element
  // and let the Mujoco compiler derive the inertial properties from the
  // body's geoms (its inertiafromgeom default is "auto", which does exactly
  // that if no inertial element is given).
  if (bdy->m > 0.0)
  {
    fprintf(fd, "%s  <inertial ", indentStr);
    writeArray(fd, "mass", &bdy->m, 1, 6);
    writeArray(fd, "diaginertia", diagInertia, 3, 6);
    writePos(fd, A_PB.org, 6);

    Mat3d_toEulerAngles(ea_deg, A_PB.rot);
    Vec3d_constMulSelf(ea_deg, 180.0 / M_PI);
    writeArray(fd, "euler", ea_deg, 3, 6);

    fprintf(fd, "/>\n");
  }
  else if (bdy->jntId != -1)
  {
    // No mass and no geoms to derive one from: Mujoco will refuse the model.
    RLOG(1, "Body \"%s\" has joints but no mass - if it has no physics "
         "shapes either, Mujoco will reject the model", bdy->name);
  }


  // Here we construct 3 translations and a Mujoco ball joint consecutively. The
  // First joint gets an absolute transformation so that the Rcs joint offset is
  // properly applied. This seems to be more convenient than trying to transform
  // to and from a Mujoco "free" joint.
  if (RcsBody_isFloatingBase(graph, bdy))
  {
    RcsJoint* jnt = RCSJOINT_BY_ID(graph, bdy->jntId);
    RCHECK(jnt);

    // The three slide joints translate along the world axes. Since Mujoco
    // interprets joint axes in the body frame, the world axes have to be
    // rotated into it.
    HTr A_JB;
    double axis[3];
    HTr_invTransform(&A_JB, &bdy->A_BI, &jnt->A_JI);

    fprintf(fd, "%s  <joint name=\"%s_x\" type=\"slide\" ", indentStr, bdy->name);
    writePos(fd, A_JB.org, 6);
    Vec3d_rotate(axis, (double(*)[3])bdy->A_BI.rot, Vec3d_ex());
    writeArray(fd, "axis", axis, 3, 6);
    fprintf(fd, " />\n");

    fprintf(fd, "%s  <joint name=\"%s_y\" type=\"slide\" ", indentStr, bdy->name);
    Vec3d_rotate(axis, (double(*)[3])bdy->A_BI.rot, Vec3d_ey());
    writeArray(fd, "axis", axis, 3, 6);
    fprintf(fd, " />\n");

    fprintf(fd, "%s  <joint name=\"%s_z\" type=\"slide\" ", indentStr, bdy->name);
    Vec3d_rotate(axis, (double(*)[3])bdy->A_BI.rot, Vec3d_ez());
    writeArray(fd, "axis", axis, 3, 6);
    fprintf(fd, " />\n");

    fprintf(fd, "%s  <joint name=\"%s_quat\" type=\"ball\" />\n", indentStr, bdy->name);
  }
  else
  {
    RCSBODY_FOREACH_JOINT(graph, bdy)
    {
      parseJoint(fd, bdy, JNT, indentStr);
    }
  }

  // All shapes of the body
  // [plane, hfield, sphere, capsule, ellipsoid, cylinder, box, mesh]
  RCSBODY_TRAVERSE_SHAPES(bdy)
  {
    if (RcsShape_isOfComputeType(SHAPE, RCSSHAPE_COMPUTE_PHYSICS))
    {
      parseShape(fd, bdy, SHAPE, indentStr);
    }
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
/*! \brief Writes bdy and its subtree. A_AI is the world transform of the
 *         closest enclosing body that has been written out, or the identity
 *         if that is the worldbody. Bodies that are not part of the physics
 *         simulation are skipped, but their children are still written. Since
 *         all transforms are relative to the enclosing Mujoco body, and not
 *         to the Rcs parent, the transform of a skipped body is automatically
 *         folded into the ones of its children.
 */
static void recurse(FILE* fd,
                    const RcsGraph* graph,
                    const RcsBody* bdy,
                    const HTr* A_AI,
                    unsigned int indent)
{
  char indentStr[64];
  snprintf(indentStr, 64, "%*s", indent, " ");

  const bool emit = (bdy->physicsSim != RCSBODY_PHYSICS_NONE);

  if (emit)
  {
    HTr A_BA;
    HTr_invTransform(&A_BA, A_AI, &bdy->A_BI);
    printBdy(fd, graph, bdy, &A_BA, indentStr);
  }

  if (bdy->firstChildId != -1)
  {
    recurse(fd, graph, &graph->bodies[bdy->firstChildId],
            emit ? &bdy->A_BI : A_AI, emit ? indent+2 : indent);
  }

  if (emit)
  {
    fprintf(fd, "%s</body>\n", indentStr);
  }

  if (bdy->nextId != -1)
  {
    recurse(fd, graph, &graph->bodies[bdy->nextId], A_AI, indent);
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
bool RcsGraph_toMujocoFile(const char* fileName, const RcsGraph* graph)
{
  if (!graph)
  {
    RLOG(1, "Graph is NULL - can't convert to mujoco");
    return false;
  }

  if (!fileName)
  {
    RLOG(1, "Couldn't open NULL file for writing");
    return false;
  }

  FILE* fd = fopen(fileName, "w+");
  if (!fd)
  {
    RLOG(1, "Couldn't open file %s for writing", fileName);
    return false;
  }

  fprintf(fd, "<mujoco model=\"%s\" >\n\n", fileName);

  // All mesh files
  unsigned int nMeshShapes = 1;   // for NULL termination
  RCSGRAPH_FOREACH_BODY(graph)
  {
    nMeshShapes += RcsBody_numShapesOfType(BODY, RCSSHAPE_MESH);
  }

  RLOG(5, "Found %d meshes", nMeshShapes);
  if (nMeshShapes > 1)
  {
    char** meshFileArray = RNALLOC(nMeshShapes, char*);
    unsigned int nMeshEntries = 0;
    fprintf(fd, "<asset>\n");
    RCSGRAPH_FOREACH_BODY(graph)
    {
      RCSBODY_TRAVERSE_SHAPES(BODY)
      {
        // Skipped in parseShape as well, see there.
        if ((SHAPE->type==RCSSHAPE_MESH) && SHAPE->meshFile &&
            (SHAPE->meshFile[0] != '\0'))
        {
          RLOG(5, "Checking %s", SHAPE->meshFile);
          bool alreadyAdded = false;

          for (unsigned int i=0; i<nMeshEntries; ++i)
          {
            if (STREQ(meshFileArray[i], SHAPE->meshFile))
            {
              alreadyAdded = true;
            }
          }

          if (!alreadyAdded)
          {
            // The asset is given the same name that parseShape refers to,
            // rather than relying on Mujoco deriving it from the file name.
            char* meshName = String_clone(SHAPE->meshFile);
            String_removeSuffix(meshName, SHAPE->meshFile, '.');
            meshFileArray[nMeshEntries] = SHAPE->meshFile;
            fprintf(fd, "  <mesh name=\"%s\" file=\"%s\"/>\n",
                    String_stripPath(meshName), SHAPE->meshFile);
            RFREE(meshName);
            nMeshEntries++;
            RCHECK_MSG(nMeshEntries<nMeshShapes, "%d %d", nMeshEntries, nMeshShapes);
          }

        }
      }
    }
    fprintf(fd, "</asset>\n\n");

    RFREE(meshFileArray);
  }


  // Options for integrators and disabling collisions
  fprintf(fd, "<option ");
  fprintf(fd, "integrator=\"RK4\" ");
  fprintf(fd, "timestep=\"0.002\" ");
  //fprintf(fd, "collision = \"predefined\" ");
  fprintf(fd, ">\n");
  fprintf(fd, "</option>\n\n");

  // All transforms are written relative to the enclosing body. The global
  // coordinate option that was used here before has been removed from Mujoco
  // in version 2.3.4. Angles and the Euler sequence are stated explicitly,
  // so that the file does not depend on the Mujoco defaults.
  fprintf(fd, "<compiler angle=\"degree\" eulerseq=\"xyz\" ");
  /* fprintf(fd, "inertiafromgeom=\"true\""); */
  fprintf(fd, " >\n");
  fprintf(fd, "</compiler>\n\n");

  fprintf(fd, "<worldbody>\n\n");

  // Create light shining down from the top and casting shadows
  fprintf(fd, "<light directional=\"true\" pos=\"0 0 10\" dir=\"0 0 -1\" />\n");

  // Mujoco body transforms refer to the joint-zero pose. We therefore create
  // a copy of the graph, bring it into the zero-configuration, and create all
  // Mujoco bodies from that. The forward kinematics of the copy give us the
  // world transform of every body, from which the relative transforms that
  // Mujoco expects are computed in recurse().
  RcsGraph* gCopy = RcsGraph_clone(graph);
  RCHECK(gCopy);
  MatNd_setZero(gCopy->q);
  MatNd_setZero(gCopy->q_dot);
  RcsGraph_setState(gCopy, gCopy->q, gCopy->q_dot);
  recurse(fd, gCopy, RcsGraph_getRootBody(gCopy), HTr_identity(), 2);



  fprintf(fd, "\n</worldbody>\n\n");


  // Add actuators here
  fprintf(fd, "<actuator>\n");
  RCSGRAPH_FOREACH_BODY(gCopy)
  {
    RCSBODY_FOREACH_JOINT(gCopy, BODY)
    {
      if (BODY->physicsSim==RCSBODY_PHYSICS_NONE)
      {
        continue;
      }

      // A floating base is written out as three slide joints and a ball
      // joint with generated names (see printBdy), so the Rcs joint names
      // of such a body do not exist in the Mujoco model. Note that this
      // must use the same predicate as printBdy, otherwise we end up with
      // actuators referring to non-existing joints, and the model does not
      // load at all.
      if (RcsBody_isFloatingBase(gCopy, BODY))
      {
        continue;
      }

      // See https://github.com/willwhitney/jaco-simulation/blob/master/jaco_other.xml
      if (JNT->ctrlType == RCSJOINT_CTRL_POSITION)
      {
        //fprintf(fd, "  <motor ctrllimited=\"false\" ctrlrange=\" -0.4 0.4\" ");
        //fprintf(fd, "joint=\"%s\" name=\"%s\" gear=\"1\" />\n", JNT->name, JNT->name);


        //fprintf(fd, "  <position joint=\"%s\" name=\"%s\" gear=\"20\" ctrllimited=\"false\" kp=\"5\" forcelimited=\"true\" forcerange=\"-20 20\" ctrlrange=\"-1.0 1.0\" />", JNT->name, JNT->name);
        //fprintf(fd, "  <position joint=\"%s\" gear=\"1\" name=\"%s\" ctrllimited=\"false\" kp=\"100\" ctrlrange=\"-10.0 10.0\" />\n", JNT->name, JNT->name);
        /* const double kp = 10.0; */
        /* double kv = 0.5 * sqrt(4.0 * kp); */

        //fprintf(fd, "  <position joint=\"%s\" name=\"%s\" kp=\"%f\" />\n", JNT->name, JNT->name, kp);
        //fprintf(fd, "  <velocity joint=\"%s\" name=\"%s_vel\" kv=\"%f\" />\n", JNT->name, JNT->name, kv);
      }


      fprintf(fd, "  <velocity joint='%s'  name='%s' kv='0.1' ctrlrange='-1 1' />\n", JNT->name, JNT->name);



    }
  }


  fprintf(fd, "</actuator>\n");
  // End actuators here



  fprintf(fd, "</mujoco>\n");

  RcsGraph_destroy(gCopy);
  fclose(fd);

  return true;
}
