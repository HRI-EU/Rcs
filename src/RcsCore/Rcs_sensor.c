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
#include "Rcs_sensor.h"
#include "Rcs_kinematics.h"
#include "Rcs_shape.h"
#include "Rcs_body.h"
#include "Rcs_macros.h"
#include "Rcs_utils.h"
#include "Rcs_math.h"
#include "Rcs_parser.h"



/*******************************************************************************
 * See header.
 ******************************************************************************/
const char* RcsSensor_name(int sensorType)
{
  static char sName[][32] = {"RCSSENSOR_CUSTOM",
                             "RCSSENSOR_LOAD_CELL",
                             "RCSSENSOR_JOINT_TORQUE",
                             "RCSSENSOR_CONTACT_FORCE",
                             "RCSSENSOR_PPS",
                             "Unknown sensor type"
                            };
  const char* ptr = NULL;

  switch (sensorType)
  {
    case RCSSENSOR_CUSTOM:
      ptr = sName[0];
      break;
    case RCSSENSOR_LOAD_CELL:
      ptr = sName[1];
      break;
    case RCSSENSOR_JOINT_TORQUE:
      ptr = sName[2];
      break;
    case RCSSENSOR_CONTACT_FORCE:
      ptr = sName[3];
      break;
    case RCSSENSOR_PPS:
      ptr = sName[4];
      break;
    default:
      ptr = sName[5];
      break;
  }

  return ptr;
}

/*******************************************************************************
 * Computes the sensor force compensation.
 ******************************************************************************/
static double RcsSensor_computeForceCompensation(const RcsGraph* graph,
                                                 const RcsSensor* fts,
                                                 const MatNd* q_ddot_curr,
                                                 double S_f_compensation[6],
                                                 bool staticForce,
                                                 bool dynamicForce)
{
  RCHECK(fts->type == RCSSENSOR_LOAD_CELL);
  RCHECK(fts->bodyId!=-1);
  RcsBody* ftsBdy = &graph->bodies[fts->bodyId];


  /////////////////////////////////////////////////////////
  //
  // Static forces in world coordinates
  //
  /////////////////////////////////////////////////////////

  // Calculate the force and torque due to the bodies (e.g., the hand) after
  // the sensor
  double I_r_cog_fts[3];
  double m_fts = RcsGraph_COG_Body(graph, ftsBdy, I_r_cog_fts);

  // Calculate the gravity force at the COG of the kinematic chain that comes
  // after the sensor (in world coordinates)
  double f_gravity[3];

  if (staticForce==true)
  {
    Vec3d_set(f_gravity, 0.0, 0.0, m_fts*RCS_GRAVITY);
  }
  else
  {
    Vec3d_setZero(f_gravity);
  }





  /////////////////////////////////////////////////////////
  //
  // Dynamics forces in world coordinates
  //
  /////////////////////////////////////////////////////////
  if (dynamicForce==true)
  {
    double acc_ftsBuf[3];
    MatNd acc_fts = MatNd_fromPtr(3, 1, acc_ftsBuf);

    int n = graph->nJ;

    // Get the current joint velocities in IK coordinates
    MatNd* q_dot_ik = NULL;
    MatNd_create2(q_dot_ik, n, 1);
    RcsGraph_stateVectorToIK(graph, graph->q_dot, q_dot_ik);

    // Calculate the Hessian, and with this dot(J) = q_dot^T H
    MatNd* H_cog = NULL, *buf = NULL;
    MatNd_create2(H_cog, 3*n*n, 1);
    MatNd_create2(buf, 3*n*n, 1);
    RcsGraph_computeCOGHessian_Body_(graph, ftsBdy, H_cog, buf);
    MatNd_reshape(H_cog, 3*n, n);

    // J_dot
    MatNd* Jdot = NULL;
    MatNd_create2(Jdot, 3*n, 1);
    MatNd_mul(Jdot, H_cog, q_dot_ik);
    MatNd_reshape(Jdot, 3, n);

    // J_dot*q_dot
    MatNd_mul(&acc_fts, Jdot, q_dot_ik);
    MatNd_destroy(Jdot);

    MatNd_destroy(H_cog);
    MatNd_destroy(buf);
    MatNd_destroy(q_dot_ik);

    // J*q_ddot
    if (q_ddot_curr != NULL)
    {
      MatNd* J_cog = NULL;
      MatNd_create2(J_cog, 3, n);
      RcsGraph_COGJacobian_Body(graph, ftsBdy, J_cog);

      MatNd* q_ddot = NULL;
      MatNd_create2(q_ddot, n, 1);
      RcsGraph_stateVectorToIK(graph, q_ddot_curr, q_ddot);

      MatNd_mulAndAddSelf(&acc_fts, J_cog, q_ddot);
      MatNd_destroy(J_cog);
      MatNd_destroy(q_ddot);
    }

    // Add dynamic force component
    Vec3d_constMulAndAddSelf(f_gravity, acc_fts.ele, m_fts);
  }




  // Calculate the gravity torque at the sensor that results from the COG of
  // the kinematic chain that comes after the sensor (in world coordinates)
  double t_gravity[3];
  Vec3d_subSelf(I_r_cog_fts, ftsBdy->A_BI.org);

  double I_sensorOffset[3];
  Vec3d_transRotate(I_sensorOffset, ftsBdy->A_BI.rot, fts->A_SB.org);
  Vec3d_subSelf(I_r_cog_fts, I_sensorOffset);
  Vec3d_crossProduct(t_gravity, I_r_cog_fts, f_gravity);

  // Rotate into sensor frame
  Vec3d_rotateSelf(f_gravity, ftsBdy->A_BI.rot);
  Vec3d_rotate(&S_f_compensation[0], (double(*)[3])fts->A_SB.rot, f_gravity);

  Vec3d_rotateSelf(t_gravity, ftsBdy->A_BI.rot);
  Vec3d_rotate(&S_f_compensation[3], (double(*)[3])fts->A_SB.rot, t_gravity);

  VecNd_constMulSelf(S_f_compensation, -1.0, 6);

  return m_fts;
}

/*******************************************************************************
 *
 ******************************************************************************/
double RcsSensor_computeStaticForceCompensation(const RcsGraph* graph,
                                                const RcsSensor* fts,
                                                double S_f_gravity[6])
{
  return RcsSensor_computeForceCompensation(graph, fts, NULL, S_f_gravity,
                                            true, false);
}

/*******************************************************************************
 *
 ******************************************************************************/
double RcsSensor_computeDynamicForceCompensation(const RcsGraph* graph,
                                                 const RcsSensor* fts,
                                                 const MatNd* q_ddot_curr,
                                                 double S_f_dynamic[6])
{
  return RcsSensor_computeForceCompensation(graph, fts, q_ddot_curr, S_f_dynamic,
                                            false, true);
}

/*******************************************************************************
 *
 ******************************************************************************/
double RcsSensor_computeFullForceCompensation(const RcsGraph* graph,
                                              const RcsSensor* fts,
                                              const MatNd* q_ddot_curr,
                                              double S_f_compensation[6])
{
  return RcsSensor_computeForceCompensation(graph, fts, q_ddot_curr,
                                            S_f_compensation, true, true);
}

/*******************************************************************************
 *
 ******************************************************************************/
void RcsSensor_init(RcsSensor* self,
                    unsigned int type,
                    const char* name,
                    RcsBody* parentBody,
                    HTr* A_SB)
{
  self->type = (RCSSENSOR_TYPE) type;

  if (parentBody && parentBody->bdySuffix)
  {
    snprintf(self->name, RCS_MAX_NAMELEN, "%s%s", name, parentBody->bdySuffix);
  }
  else
  {
    snprintf(self->name, RCS_MAX_NAMELEN, "%s", name);
  }

  self->bodyId = parentBody->id;
  HTr_copy(&self->A_SB, A_SB);
  self->rawData = MatNd_create(1, RcsSensor_dim(self));
}

/*******************************************************************************
 * Copies a RcsSensor data structure except for a few unknown members.
 ******************************************************************************/
void RcsSensor_copy(RcsSensor* self, const RcsSensor* src)
{
  self->type = src->type;
  self->bodyId = src->bodyId;
  snprintf(self->name, RCS_MAX_NAMELEN, "%s", src->name);
  HTr_copy(&self->A_SB, &src->A_SB);
  MatNd_resizeCopy(&self->rawData, src->rawData);
  self->nTexels = src->nTexels;
  if (src->nTexels>0)
  {
    self->texel = RREALLOC(self->texel, src->nTexels, RcsTexel);
    memcpy(self->texel, src->texel, src->nTexels*sizeof(RcsTexel));
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
void RcsSensor_clear(RcsSensor* self)
{
  if (self==NULL)
  {
    return;
  }

  RFREE(self->texel);
  MatNd_destroy(self->rawData);
}

/*******************************************************************************
 *
 ******************************************************************************/
void RcsSensor_destroy(RcsSensor* self)
{
  if (self==NULL)
  {
    return;
  }

  RcsSensor_clear(self);

  // Reset all internal memory
  memset(self, 0, sizeof(RcsSensor));
  RFREE(self);
}

/*******************************************************************************
 *  These are the available sensors:
 *  RCSSENSOR_CUSTOM = 0,
 *  RCSSENSOR_LOAD_CELL,
 *  RCSSENSOR_JOINT_TORQUE,
 *  RCSSENSOR_CONTACT_FORCE,
 *  RCSSENSOR_PPS
 ******************************************************************************/
unsigned int RcsSensor_dim(RcsSensor* self)
{
  if (self->type == RCSSENSOR_LOAD_CELL)
  {
    return 9;// 3xforce, 3xtorque, 3xacc
  }
  else if (self->type == RCSSENSOR_JOINT_TORQUE)
  {
    return 1;
  }
  else if (self->type == RCSSENSOR_CONTACT_FORCE)
  {
    return 3;
  }
  else if (self->type == RCSSENSOR_PPS)
  {
    return self->nTexels;
  }
  else
  {
    RFATAL("Unsupported sensor type");
  }

  return 0;
}

/*******************************************************************************
 *
 ******************************************************************************/
RcsSensor* RcsGraph_insertSensor(RcsGraph* graph)
{
  graph->nSensors++;
  graph->sensors = (RcsSensor*) realloc(graph->sensors, graph->nSensors*sizeof(RcsSensor));
  RCHECK(graph->sensors);
  RcsSensor* newSensor = &graph->sensors[graph->nSensors-1];
  memset(newSensor, 0, sizeof(RcsSensor));
  return newSensor;
}

/*******************************************************************************
 *
 ******************************************************************************/
void RcsSensor_fprint(FILE* out, const RcsSensor* s)
{
  if (s == NULL)
  {
    RLOG(1, "Sensor doesn't exist");
    return;
  }

  // Sensor name and type
  fprintf(out, "[RcsSensor_fprint():%d] \n\tSensor \"%s\" of type \"%s\"\n",
          __LINE__, s->name, RcsSensor_name(s->type));

  fprintf(out, "\tAttached to body with id %d\n", s->bodyId);

  if (!HTr_isIdentity(&s->A_SB))
  {
    HTr_fprint(out, &s->A_SB);
  }
  else
  {
    fprintf(out, "\tNo offset\n");
  }

  fprintf(out, "\tRaw sensor data:\n\t");

  if (s->rawData)
  {
    for (unsigned int i = 1; i <= s->rawData->size; i++)
    {
      fprintf(out, "%+.3f ", s->rawData->ele[i-1]);
      if (i%10==0)
      {
        fprintf(out, "\n\t");
      }
    }
  }
  else
  {
    fprintf(out, "NULL");
  }

  fprintf(out, "\n");
}

/*******************************************************************************
 * These tags need to be written:
 *        - name
 *        - type: LOADCELL, JOINTTORQUE, CONTACTFORCE, PPS
 *        - transform
 *        - extra_info
 ******************************************************************************/
void RcsSensor_fprintXML(FILE* out, const RcsSensor* self)
{
  char buf[256];
  bool ppsExtentsEqual = true;

  fprintf(out, "    <Sensor name=\"%s\" ", self->name);

  switch (self->type)
  {
    case RCSSENSOR_LOAD_CELL:
      fprintf(out, "type=\"LOADCELL\" ");
      break;

    case RCSSENSOR_JOINT_TORQUE:
      fprintf(out, "type=\"JOINTTORQUE\" ");
      break;

    case RCSSENSOR_CONTACT_FORCE:
      fprintf(out, "type=\"CONTACTFORCE\" ");
      break;

    case RCSSENSOR_PPS:
      fprintf(out, "type=\"PPS\" ");
      if (self->nTexels>0)
      {
        double* extents0 = self->texel[0].extents;
        for (unsigned int i=1; i<self->nTexels; ++i)
        {
          if (!Vec3d_isEqual(extents0, self->texel[i].extents, 1.0e-8))
          {
            ppsExtentsEqual = false;
            break;
          }
        }

        fprintf(out, "dimensions=\"%d %d\" ",
                self->rawData->m, self->rawData->n);

        if (ppsExtentsEqual)
        {
          fprintf(out, "extents=\"%s ", String_fromDouble(buf, extents0[0], 6));
          fprintf(out, "%s ", String_fromDouble(buf, extents0[1], 6));
          fprintf(out, "%s\" ", String_fromDouble(buf, extents0[2], 6));
        }

      }

      break;

    default:
      RFATAL("Unknown sensor type: %d", self->type);
  }

  // Relative transformation only if non-zero elements exist
  {
    double trf[6];
    Vec3d_copy(&trf[0], self->A_SB.org);
    Mat3d_toEulerAngles(&trf[3], (double (*)[3]) self->A_SB.rot);
    Vec3d_constMulSelf(&trf[3], 180.0 / M_PI);

    if (VecNd_maxAbsEle(trf, 6) > 1.0e-8)
    {
      fprintf(out, "transform=\"%s ", String_fromDouble(buf, trf[0], 6));
      fprintf(out, "%s ", String_fromDouble(buf, trf[1], 6));
      fprintf(out, "%s ", String_fromDouble(buf, trf[2], 6));
      fprintf(out, "%s ", String_fromDouble(buf, trf[3], 6));
      fprintf(out, "%s ", String_fromDouble(buf, trf[4], 6));
      fprintf(out, "%s\" ", String_fromDouble(buf, trf[5], 6));
    }
  }

  // PPS parameter string
  if (self->nTexels>0)
  {
    fprintf(out, ">\n");

    for (unsigned int i=0; i<self->nTexels; ++i)
    {
      const double* val = self->texel[i].position;
      fprintf(out, "      <Texel position=\"");
      fprintf(out, "%s ", String_fromDouble(buf, val[0], 6));
      fprintf(out, "%s ", String_fromDouble(buf, val[1], 6));
      fprintf(out, "%s\" ", String_fromDouble(buf, val[2], 6));

      val = self->texel[i].normal;
      fprintf(out, "normal=\"%s ", String_fromDouble(buf, val[0], 6));
      fprintf(out, "%s ", String_fromDouble(buf, val[1], 6));
      fprintf(out, "%s\" ", String_fromDouble(buf, val[2], 6));

      if (!ppsExtentsEqual)
      {
        val = self->texel[i].extents;
        fprintf(out, "extents=\"%s ", String_fromDouble(buf, val[0], 6));
        fprintf(out, "%s ", String_fromDouble(buf, val[1], 6));
        fprintf(out, "%s\" ", String_fromDouble(buf, val[2], 6));
      }

      fprintf(out, "/>\n");
    }

    fprintf(out, "    </Sensor>\n");
  }
  else
  {
    fprintf(out, "/>\n");
  }
}

/******************************************************************************
 * Compute PPS sensor pressure distribution
 *****************************************************************************/
bool RcsSensor_computePPS(RcsGraph* graph,
                          const RcsSensor* self,
                          MatNd* ppsResult,
                          const double contactForce[3])
{
  if (self==NULL)
  {
    RLOG(4, "NULL sensor found in PPS function - skipping");
    return false;
  }

  if (self->bodyId==-1)
  {
    RLOG(4, "Sensor \"%s\" has no mount body - skipping", self->name);
    return false;
  }

  RcsBody* sensorBdy = &graph->bodies[self->bodyId];

  unsigned int dimension = self->nTexels;

  // This array has as many columns as texels. It has 6 rows: Rows 1-3 hold
  // the texel position, rows 4-6 the corresponding texel normal vector.
  MatNd* lineDirMat = MatNd_create(dimension, 3);
  MatNd* lineOriginMat = MatNd_create(dimension, 3);

  // Determine the mount body transformation from physics
  HTr A_SI;
  HTr_transform(&A_SI, &sensorBdy->A_BI, &self->A_SB);

  // calculate the maximum distance of the PPS elements to the mount body origin
  // used to check whether a shape can be completely ignored
  double sensorDepth = 0.005, maxDist = 0.0;

  // Pre-compute lineDir and lineOrigin arrays in world coordinates
  int id = 0;
  for (unsigned int i=0; i<self->nTexels; ++i)
  {
    RcsTexel* txi = &self->texel[i];
    double offset[3], lineDir[3];
    for (size_t dim = 0; dim < 3; dim++)
    {
      lineDir[dim] = txi->normal[dim];
      offset[dim] = txi->position[dim] + sensorDepth*lineDir[dim]/3.0;
    }

    if (fabs(Vec3d_getLength(lineDir)-1.0)>1.0e-3)
    {
      Vec3d_setZero(MatNd_getRowPtr(lineDirMat, id));
    }
    else
    {
      if (fabs(Vec3d_getLength(lineDir)-1.0)>1.0e-5)
      {
        Vec3d_normalizeSelf(lineDir);
      }

      maxDist = fmax(maxDist, Vec3d_getLength(offset));

      Vec3d_transRotateSelf(lineDir, A_SI.rot);
      Vec3d_transRotateSelf(offset, A_SI.rot);
      Vec3d_add(MatNd_getRowPtr(lineOriginMat, id), offset, A_SI.org);
      Vec3d_copy(MatNd_getRowPtr(lineDirMat, id), lineDir);
    }

    id++;
  }

  maxDist += sensorDepth; //just a safety margin



  double* skinSensorData = RNALLOC(dimension, double);
  double f[3];
  Vec3d_constMul(f, contactForce, -1.0);

  for (size_t id = 0; id < dimension; id++)
  {
    // project onto PPS normal
    const double* lineDir = MatNd_getRowPtr(lineDirMat, id);
    double projectedContactForce = Vec3d_innerProduct(lineDir, f);

    // It shouldn't become negative, since this would mean there is a
    // pulling force on the texel. This could however happen if the
    // curvature of the PPS is quite strong.
    if (projectedContactForce>0.0)
    {
      skinSensorData[id] += projectedContactForce;
    }
  }

  // Pinscreen / Pin Art
  MatNd_reshapeAndSetZero(ppsResult, 1, dimension);

  if (VecNd_sqrLength(skinSensorData, dimension) == 0.0)
  {
    MatNd_destroy(lineDirMat);
    MatNd_destroy(lineOriginMat);
    RFREE(skinSensorData);
    return true;
  }

  // Go through all the shapes in the graph to calculate the ray distances
  RCSGRAPH_TRAVERSE_BODIES(graph)
  {
    // Skipping mount body & immediate parent & immediate children
    if ((BODY->id==sensorBdy->id) ||
        (BODY->id==sensorBdy->parentId) ||
        (BODY->parentId==sensorBdy->id))
    {
      continue;
    }


    RCSBODY_TRAVERSE_SHAPES(BODY)
    {
      // Ignore shapes that don't calculate distances and those that are
      // too far away anyhow
      if (((SHAPE->computeType & RCSSHAPE_COMPUTE_DISTANCE) != 0) &&
          (RcsShape_boundingSphereDistance(A_SI.org, &BODY->A_BI, SHAPE) < maxDist))
      {
        for (unsigned int id = 0; id < dimension; ++id)
        {
          // if there is no force in the direction of this PPS element, we
          // can ignore it (set zero on top). This also takes care of the
          // "dead" elements
          if (skinSensorData[id]<=0.0)
          {
            continue;
          }

          const double* lineDir = MatNd_getRowPtr(lineDirMat, id);
          const double* lineOrigin = MatNd_getRowPtr(lineOriginMat, id);
          double closestLinePt[3];

          if (RcsShape_computeLineIntersection(lineOrigin, lineDir, &BODY->A_BI,
                                               SHAPE, closestLinePt))
          {
            Vec3d_subSelf(closestLinePt, lineOrigin);

            double dist = Vec3d_innerProduct(closestLinePt, lineDir);

            // ignore points in front of the sensor and too far behind
            if ((dist<0.0) && (dist>-4.0*sensorDepth))
            {
              double temp_pps = Math_clip(-dist/sensorDepth, 0.0, 1.0);

              // take value with maximum penetration per PPS element
              ppsResult->ele[id] = fmax(ppsResult->ele[id], temp_pps);
            }
          }
        } //PPS elements
      } //RCSSHAPE_COMPUTE_DISTANCE
    } //RCSBODY_TRAVERSE_SHAPES
  } //RCSGRAPH_TRAVERSE_BODIES

  // make sure the norm of the PPS elements corresponds to the measured total
  // force
  VecNd_eleMulSelf(ppsResult->ele, skinSensorData, dimension);
  double ppsLength = VecNd_getLength(ppsResult->ele, dimension);
  if (ppsLength > 0.0)
  {
    VecNd_constMulSelf(ppsResult->ele,
                       0.5*VecNd_getLength(skinSensorData, dimension)/ppsLength,
                       dimension);
  }

  MatNd_destroy(lineDirMat);
  MatNd_destroy(lineOriginMat);
  RFREE(skinSensorData);

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
RcsSensor* RcsSensor_first(const RcsGraph* graph)
{
  return (graph->nSensors>0) ? &graph->sensors[0] : NULL;
}

/*******************************************************************************
 *
 ******************************************************************************/
RcsSensor* RcsSensor_last(const RcsGraph* graph)
{
  return (graph->nSensors>0) ? &graph->sensors[graph->nSensors-1] : NULL;
}
