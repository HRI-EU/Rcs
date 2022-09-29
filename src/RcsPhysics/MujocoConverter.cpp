/*******************************************************************************

  Copyright Honda Research Institute Europe GmbH

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

#include "MujocoConverter.h"

#include <Rcs_typedef.h>
#include <Rcs_body.h>
#include <Rcs_shape.h>
#include <Rcs_joint.h>
#include <Rcs_utils.h>
#include <Rcs_math.h>
#include <Rcs_macros.h>
#include <Rcs_material.h>








namespace Rcs
{
static void parseJoint(FILE* fd,
                       const RcsBody* bdy,
                       const RcsJoint* jnt,
                       const char* indentStr)
{
  char buf[64];
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
  fprintf(fd, "pos=\"%s ", String_fromDouble(buf, A_21.org[0], 6));
  fprintf(fd, "%s ", String_fromDouble(buf, A_21.org[1], 6));
  fprintf(fd, "%s\" ", String_fromDouble(buf, A_21.org[2], 6));

  fprintf(fd, "axis=\"%s ", String_fromDouble(buf, A_21.rot[jnt->dirIdx][0], 6));
  fprintf(fd, "%s ", String_fromDouble(buf, A_21.rot[jnt->dirIdx][1], 6));
  fprintf(fd, "%s\" ", String_fromDouble(buf, A_21.rot[jnt->dirIdx][2], 6));

  fprintf(fd, "/>\n");
}


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
    RLOG(0, "Shape %s not yet supported", RcsShape_name(shape->type));
    return;
  }

  char buf[64];

  // Shape's transform in world coordinates
  HTr A_CI;
  HTr_transform(&A_CI, &bdy->A_BI, &shape->A_CB);

  // All shapes of the body
  // [plane, hfield, sphere, capsule, ellipsoid, cylinder, box, mesh]
  fprintf(fd, "%s  <geom ", indentStr);

  switch (shape->type)
  {
    case RCSSHAPE_CYLINDER:
      fprintf(fd, "type=\"cylinder\" ");
      fprintf(fd, "size=\"%s", String_fromDouble(buf, shape->extents[0], 6));
      fprintf(fd, " %s\" ", String_fromDouble(buf, 0.5 * shape->extents[2], 6));
      break;

    case RCSSHAPE_SPHERE:
      fprintf(fd, "type=\"sphere\" ");
      fprintf(fd, "size=\"%s\" ", String_fromDouble(buf, shape->extents[0], 6));
      break;

    case RCSSHAPE_SSL:
      fprintf(fd, "type=\"capsule\" ");
      fprintf(fd, "size=\"%s\" ", String_fromDouble(buf, shape->extents[0], 6));
      //fprintf(fd, " %s\" ", String_fromDouble(buf, 0.5*shape->extents[2], 6));

      fprintf(fd, "fromto=\"%s", String_fromDouble(buf, A_CI.org[0], 6));
      fprintf(fd, " %s ", String_fromDouble(buf, A_CI.org[1], 6));
      fprintf(fd, " %s ", String_fromDouble(buf, A_CI.org[2], 6));
      fprintf(fd, " %s ", String_fromDouble(buf, A_CI.org[0] + shape->extents[2] * A_CI.rot[2][0], 6));
      fprintf(fd, " %s ", String_fromDouble(buf, A_CI.org[1] + shape->extents[2] * A_CI.rot[2][1], 6));
      fprintf(fd, " %s\" ", String_fromDouble(buf, A_CI.org[2] + shape->extents[2] * A_CI.rot[2][2], 6));
      break;

    case RCSSHAPE_SSR:
    case RCSSHAPE_BOX:
      fprintf(fd, "type=\"box\" ");
      fprintf(fd, "size=\"%s", String_fromDouble(buf, 0.5 * shape->extents[0], 6));
      fprintf(fd, " %s ", String_fromDouble(buf, 0.5 * shape->extents[1], 6));
      fprintf(fd, " %s\" ", String_fromDouble(buf, 0.5 * shape->extents[2], 6));
      break;

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

  double ea[3];
  Mat3d_toEulerAngles(ea, A_CI.rot);

  if (shape->type != RCSSHAPE_SSL)
  {
    fprintf(fd, "pos=\"%s ", String_fromDouble(buf, A_CI.org[0], 6));
    fprintf(fd, "%s ", String_fromDouble(buf, A_CI.org[1], 6));
    fprintf(fd, "%s\" ", String_fromDouble(buf, A_CI.org[2], 6));

    fprintf(fd, "euler=\"%s ", String_fromDouble(buf, RCS_RAD2DEG(ea[0]), 6));
    fprintf(fd, "%s ", String_fromDouble(buf, RCS_RAD2DEG(ea[1]), 6));
    fprintf(fd, "%s\" ", String_fromDouble(buf, RCS_RAD2DEG(ea[2]), 6));
  }

  // Color
  double geomRGBA[4];
  Rcs_colorFromString(shape->color, geomRGBA);
  fprintf(fd, "rgba=\"%s ", String_fromDouble(buf, geomRGBA[0], 6));
  fprintf(fd, "%s ", String_fromDouble(buf, geomRGBA[1], 6));
  fprintf(fd, "%s ", String_fromDouble(buf, geomRGBA[2], 6));
  fprintf(fd, "%s\" ", String_fromDouble(buf, geomRGBA[3], 6));

  fprintf(fd, "/>\n");
}

static void printBdy(FILE* fd,
                     const RcsGraph* graph,
                     const RcsBody* bdy,
                     const char* indentStr)
{
  char buf[64];
  double ea[3];

  fprintf(fd, "%s<body name=\"%s\" ", indentStr, bdy->name);

  const RcsBody* parent = RCSBODY_BY_ID(graph, bdy->parentId);
  if (parent)
  {
    HTr A_21;
    HTr_copy(&A_21, &bdy->A_BI);
    fprintf(fd, "pos=\"%s ", String_fromDouble(buf, A_21.org[0], 6));
    fprintf(fd, "%s ", String_fromDouble(buf, A_21.org[1], 6));
    fprintf(fd, "%s\" ", String_fromDouble(buf, A_21.org[2], 6));

    Mat3d_toEulerAngles(ea, A_21.rot);
    fprintf(fd, "euler=\"%s ", String_fromDouble(buf, RCS_RAD2DEG(ea[0]), 6));
    fprintf(fd, "%s ", String_fromDouble(buf, RCS_RAD2DEG(ea[1]), 6));
    fprintf(fd, "%s\" ", String_fromDouble(buf, RCS_RAD2DEG(ea[2]), 6));
  }

  fprintf(fd, ">\n");


  // Mass and inertia properties
  fprintf(fd, "%s  <inertial mass=\"%s\" ",
          indentStr, String_fromDouble(buf, bdy->m, 6));

  // Inertia tensor
  // Compute the transformation from body (Index B) to COM (Index P) frame to
  // body frame. The principal axes of inertia correspond to the Eigenvectors
  double diagInertia[3], A_BP[3][3];
  Mat3d_getEigenVectors(A_BP, diagInertia, (double(*)[3])bdy->Inertia.rot);

  fprintf(fd, "diaginertia=\"%s ", String_fromDouble(buf, diagInertia[0], 10));
  fprintf(fd, "%s ", String_fromDouble(buf, diagInertia[1], 10));
  fprintf(fd, "%s\" ", String_fromDouble(buf, diagInertia[2], 10));

  Mat3d_toEulerAngles(ea, A_BP);

  fprintf(fd, "pos=\"%s ", String_fromDouble(buf, bdy->A_BI.org[0] + bdy->Inertia.org[0], 6));
  fprintf(fd, "%s ", String_fromDouble(buf, bdy->A_BI.org[1] + bdy->Inertia.org[1], 6));
  fprintf(fd, "%s\" ", String_fromDouble(buf, bdy->A_BI.org[2] + bdy->Inertia.org[2], 6));

  fprintf(fd, "euler=\"%s ", String_fromDouble(buf, RCS_RAD2DEG(ea[0]), 6));
  fprintf(fd, "%s ", String_fromDouble(buf, RCS_RAD2DEG(ea[1]), 6));
  fprintf(fd, "%s\" ", String_fromDouble(buf, RCS_RAD2DEG(ea[2]), 6));

  fprintf(fd, "/>\n");

  if (RcsBody_isFloatingBase(graph, bdy))
  {
    fprintf(fd, "%s  <joint name=\"%s_6dof\" ", indentStr, bdy->name);
    fprintf(fd, "type=\"free\" ");
    fprintf(fd, "/>\n");
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













static void RcsGraph_recurse(FILE* fd,
                             const RcsGraph* graph,
                             const RcsBody* bdy,
                             unsigned int indent)
{
  char indentStr[64], buf[64];
  double ea[3];
  snprintf(indentStr, 64, "%*s", indent, " ");

  if (bdy->physicsSim)
  {
    fprintf(fd, "%s<body name=\"%s\" ", indentStr, bdy->name);

    const RcsBody* parent = RCSBODY_BY_ID(graph, bdy->parentId);
    if (parent)
    {
      HTr A_21;
      HTr_copy(&A_21, &bdy->A_BI);
      fprintf(fd, "pos=\"%s ", String_fromDouble(buf, A_21.org[0], 6));
      fprintf(fd, "%s ", String_fromDouble(buf, A_21.org[1], 6));
      fprintf(fd, "%s\" ", String_fromDouble(buf, A_21.org[2], 6));

      Mat3d_toEulerAngles(ea, A_21.rot);
      fprintf(fd, "euler=\"%s ", String_fromDouble(buf, RCS_RAD2DEG(ea[0]), 6));
      fprintf(fd, "%s ", String_fromDouble(buf, RCS_RAD2DEG(ea[1]), 6));
      fprintf(fd, "%s\" ", String_fromDouble(buf, RCS_RAD2DEG(ea[2]), 6));
    }

    fprintf(fd, ">\n");


    // Mass and inertia properties
    fprintf(fd, "%s  <inertial mass=\"%s\" ",
            indentStr, String_fromDouble(buf, bdy->m, 6));

    // Inertia tensor
    // Compute the transformation from body (Index B) to COM (Index P) frame to
    // body frame. The principal axes of inertia correspond to the Eigenvectors
    double diagInertia[3], A_BP[3][3];
    Mat3d_getEigenVectors(A_BP, diagInertia, (double(*)[3])bdy->Inertia.rot);

    fprintf(fd, "diaginertia=\"%s ", String_fromDouble(buf, diagInertia[0], 10));
    fprintf(fd, "%s ", String_fromDouble(buf, diagInertia[1], 10));
    fprintf(fd, "%s\" ", String_fromDouble(buf, diagInertia[2], 10));

    Mat3d_toEulerAngles(ea, A_BP);

    fprintf(fd, "pos=\"%s ", String_fromDouble(buf, bdy->A_BI.org[0] + bdy->Inertia.org[0], 6));
    fprintf(fd, "%s ", String_fromDouble(buf, bdy->A_BI.org[1] + bdy->Inertia.org[1], 6));
    fprintf(fd, "%s\" ", String_fromDouble(buf, bdy->A_BI.org[2] + bdy->Inertia.org[2], 6));

    fprintf(fd, "euler=\"%s ", String_fromDouble(buf, RCS_RAD2DEG(ea[0]), 6));
    fprintf(fd, "%s ", String_fromDouble(buf, RCS_RAD2DEG(ea[1]), 6));
    fprintf(fd, "%s\" ", String_fromDouble(buf, RCS_RAD2DEG(ea[2]), 6));

    fprintf(fd, "/>\n");

    if (RcsBody_isFloatingBase(graph, bdy))
    {
      fprintf(fd, "%s  <joint name=\"%s_6dof\" ", indentStr, bdy->name);
      fprintf(fd, "type=\"free\" ");
      fprintf(fd, "/>\n");
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

  }   // bdy->physicsSim

  // Child in direct sequence
  if ((bdy->firstChildId!=-1)) //&& (bdy->firstChildId==bdy->lastChildId))
  {
    const RcsBody* child = RCSBODY_BY_ID(graph, bdy->firstChildId);
    //RcsGraph_recurse(fd, graph, child, indent+2);
    while (child)
    {
      RcsGraph_recurse(fd, graph, child, indent+2);
      child = RCSBODY_BY_ID(graph, child->nextId);
    }
  }
  //else if ((bdy->firstChildId!=-1) && (bdy->firstChildId!=bdy->lastChildId))
  //{
  //  const RcsBody* next = RCSBODY_BY_ID(graph, bdy->firstChildId);
  //  while (next)
  //  {
  //    RcsGraph_recurse(fd, graph, next, indent+2);
  //    next = RCSBODY_BY_ID(graph, next->nextId);
  //  }
  //}

  if (bdy->physicsSim)
  {
    fprintf(fd, "%s</body>\n", indentStr);
  }

  if (bdy->nextId != -1)
  {
    const RcsBody* next = RCSBODY_BY_ID(graph, bdy->nextId);
    RcsGraph_recurse(fd, graph, next, indent + 0);
  }

}



bool RcsGraph_convertToMujoco(const char* fileName, const RcsGraph* graph)
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
  RLOG(0, "Found %d meshes", nMeshShapes);
  if (nMeshShapes > 0)
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
          RLOG(0, "Checking %s", SHAPE->meshFile);
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
  fprintf(fd, "<compiler coordinate=\"global\" >\n");
  fprintf(fd, "</compiler>\n\n");

  fprintf(fd, "<worldbody>\n\n");

  // Create light shining down from the top and casting shadows
  fprintf(fd, "<light directional=\"true\" pos=\"0 0 10\" dir=\"0 0 -1\" />\n");

  RcsGraph* gCopy = RcsGraph_clone(graph);
  MatNd_setZero(gCopy->q);
  MatNd_setZero(gCopy->q_dot);
  RcsGraph_setState(gCopy, gCopy->q, gCopy->q_dot);
  //RcsGraph_recurse(fd, gCopy, &gCopy->bodies[gCopy->rootId], 2);
  recurse(fd, gCopy, &gCopy->bodies[gCopy->rootId], 2);
  RcsGraph_destroy(gCopy);



  fprintf(fd, "\n</worldbody>\n\n");
  fprintf(fd, "</mujoco>\n");

  fclose(fd);

  return true;
}

}   // namespace Rcs
