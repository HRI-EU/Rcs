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

  HTr A_21;
  HTr_copy(&A_21, &jnt->A_JI);
  writePos(fd, A_21.org, 6);
  writeArray(fd, "axis", A_21.rot[jnt->dirIdx], 3, 6);

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
  if ((shape->type == RCSSHAPE_REFFRAME) ||
      (shape->type == RCSSHAPE_TORUS) ||
      (shape->type == RCSSHAPE_POINT) ||
      (shape->type == RCSSHAPE_CONE))
  {
    if (shape->type != RCSSHAPE_REFFRAME)
    {
      RLOG(0, "Shape %s not yet supported", RcsShape_name(shape->type));
    }
    return;
  }

  // Shape's transform in world coordinates
  HTr A_CI;
  HTr_transform(&A_CI, &bdy->A_BI, &shape->A_CB);

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
      Vec3d_copy(fromTo, A_CI.org);
      Vec3d_constMulAndAdd(fromTo + 3, A_CI.org, A_CI.rot[2], shape->extents[2]);
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
      fprintf(fd, "type=\"unsupported (%s)\"", RcsShape_name(shape->type));
  }

  double ea_deg[3];
  Mat3d_toEulerAngles(ea_deg, A_CI.rot);
  Vec3d_constMulSelf(ea_deg, 180.0/M_PI);

  if (shape->type != RCSSHAPE_SSL)
  {
    writePos(fd, A_CI.org, 6);
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
                     const char* indentStr)
{
  // Inertia tensor frame: Compute the transformation from body (Index B) to
  // COM (Index P) frame to body frame. The principal axes of inertia
  // correspond to the Eigenvectors. They are sorted in decreasing order.
  double diagInertia[3], ea_deg[3];
  HTr A_PB, A_PI;

  // Body to inertia frame
  Mat3d_getEigenVectors(A_PB.rot, diagInertia, (double(*)[3])bdy->Inertia.rot);
  Mat3d_transposeSelf(A_PB.rot);
  Vec3d_copy(A_PB.org, bdy->Inertia.org);

  // World to inertia frame
  HTr_transform(&A_PI, &bdy->A_BI, &A_PB);

  // Body transform in world coordinates
  Mat3d_toEulerAngles(ea_deg, (double(*)[3]) &bdy->A_BI.rot);
  Vec3d_constMulSelf(ea_deg, 180.0/M_PI);

  fprintf(fd, "%s<body name=\"%s\" ", indentStr, bdy->name);
  writePos(fd, bdy->A_BI.org, 6);
  writeArray(fd, "euler", ea_deg, 3, 6);
  fprintf(fd, ">\n");

  // Mass and inertia properties
  fprintf(fd, "%s  <inertial ", indentStr);
  writeArray(fd, "mass", &bdy->m, 1, 6);
  writeArray(fd, "diaginertia", diagInertia, 3, 6);

  // Compute COM in world coordinates
  double com[3];
  Vec3d_add(com, bdy->A_BI.org, bdy->Inertia.org);
  writePos(fd, com, 6);

  // Compute rotation from world frame into diagonal inertia tensor
  Mat3d_toEulerAngles(ea_deg, A_PI.rot);
  Vec3d_constMulSelf(ea_deg, 180.0 / M_PI);
  writeArray(fd, "euler", ea_deg, 3, 6);

  fprintf(fd, "/>\n");


  // Here we construct 3 translations and a Mujoco ball joint consecutively. The
  // First joint gets an absolute transformation so that the Rcs joint offset is
  // properly applied. This seems to be more convenient than trying to transform
  // to and from a Mujoco "free" joint.
  if (RcsBody_isFloatingBase(graph, bdy))
  {
    RcsJoint* jnt = RCSJOINT_BY_ID(graph, bdy->jntId);
    RCHECK(jnt);
    fprintf(fd, "%s  <joint name=\"%s_x\" type=\"slide\" ", indentStr, bdy->name);
    writePos(fd, jnt->A_JI.org, 6);
    writeArray(fd, "axis", Vec3d_ex(), 3, 6);
    fprintf(fd, " />\n");

    fprintf(fd, "%s  <joint name=\"%s_y\" type=\"slide\" ", indentStr, bdy->name);
    writeArray(fd, "axis", Vec3d_ey(), 3, 6);
    fprintf(fd, " />\n");

    fprintf(fd, "%s  <joint name=\"%s_z\" type=\"slide\" ", indentStr, bdy->name);
    writeArray(fd, "axis", Vec3d_ez(), 3, 6);
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
static void recurse(FILE* fd,
                    const RcsGraph* graph,
                    const RcsBody* bdy,
                    unsigned int indent)
{
  char indentStr[64];
  snprintf(indentStr, 64, "%*s", indent, " ");

  if (bdy->physicsSim != RCSBODY_PHYSICS_NONE)
  {
    //fprintf(fd, "%s<body name=\"%s\" >\n ", indentStr, bdy->name);
    printBdy(fd, graph, bdy, indentStr);
  }


  if (bdy->firstChildId != -1)
  {
    recurse(fd, graph, &graph->bodies[bdy->firstChildId], indent + 2);
  }

  if (bdy->physicsSim != RCSBODY_PHYSICS_NONE)
  {
    //fprintf(fd, "%s</body (%s)>\n", indentStr, bdy->name);
    fprintf(fd, "%s</body>\n", indentStr);
  }

  if (bdy->nextId != -1)
  {
    recurse(fd, graph, &graph->bodies[bdy->nextId], indent);
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
        if (SHAPE->type==RCSSHAPE_MESH)
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
            meshFileArray[nMeshEntries] = SHAPE->meshFile;
            fprintf(fd, "  <mesh file=\"%s\"/>\n", SHAPE->meshFile);
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

  // Option for considering transforms in world frame
  fprintf(fd, "<compiler coordinate=\"global\" ");
  /* fprintf(fd, "inertiafromgeom=\"true\""); */
  fprintf(fd, " >\n");
  fprintf(fd, "</compiler>\n\n");

  fprintf(fd, "<worldbody>\n\n");

  // Create light shining down from the top and casting shadows
  fprintf(fd, "<light directional=\"true\" pos=\"0 0 10\" dir=\"0 0 -1\" />\n");

  // The most convenient way to convert the model is making use of the Mujoco
  // global coordinate option. For this, we create a copy of the graph, bring
  // it into the zero-configuration, and create all Mujoco bodies from that.
  RcsGraph* gCopy = RcsGraph_clone(graph);
  RCHECK(gCopy);
  MatNd_setZero(gCopy->q);
  MatNd_setZero(gCopy->q_dot);
  RcsGraph_setState(gCopy, gCopy->q, gCopy->q_dot);
  recurse(fd, gCopy, RcsGraph_getRootBody(gCopy), 2);



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

      if (BODY->rigid_body_joints)
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
