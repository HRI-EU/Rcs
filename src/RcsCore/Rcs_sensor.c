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



/******************************************************************************

  \brief Computes the sensor force compensation.

******************************************************************************/
static double RcsSensor_computeForceCompensation(const RcsGraph* graph,
                                                 const RcsSensor* fts,
                                                 const MatNd* q_ddot_curr,
                                                 double S_f_compensation[6],
                                                 bool staticForce,
                                                 bool dynamicForce)
{
  RCHECK(fts->type == RCSSENSOR_LOAD_CELL);





  /////////////////////////////////////////////////////////
  //
  // Static forces in world coordinates
  //
  /////////////////////////////////////////////////////////

  // Calculate the force and torque due to the bodies (e.g., the hand) after
  // the sensor
  double I_r_cog_fts[3];
  double m_fts = RcsGraph_COG_Body(fts->body, I_r_cog_fts);

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
    RcsGraph_computeCOGHessian_Body_(graph, fts->body, H_cog, buf);
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
      RcsGraph_COGJacobian_Body(graph, fts->body, J_cog);

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
  Vec3d_subSelf(I_r_cog_fts, fts->body->A_BI->org);

  double I_sensorOffset[3];
  Vec3d_transRotate(I_sensorOffset, fts->body->A_BI->rot, fts->offset->org);
  Vec3d_subSelf(I_r_cog_fts, I_sensorOffset);
  Vec3d_crossProduct(t_gravity, I_r_cog_fts, f_gravity);

  // Rotate into sensor frame
  Vec3d_rotateSelf(f_gravity, fts->body->A_BI->rot);
  Vec3d_rotate(&S_f_compensation[0], fts->offset->rot, f_gravity);

  Vec3d_rotateSelf(t_gravity, fts->body->A_BI->rot);
  Vec3d_rotate(&S_f_compensation[3], fts->offset->rot, t_gravity);

  VecNd_constMulSelf(S_f_compensation, -1.0, 6);

  return m_fts;
}



/******************************************************************************

  \brief See header.

******************************************************************************/
double RcsSensor_computeStaticForceCompensation(const RcsSensor* fts,
                                                double S_f_gravity[6])
{
  return RcsSensor_computeForceCompensation(NULL, fts, NULL, S_f_gravity,
                                            true, false);
}



/******************************************************************************

  \brief See header.

******************************************************************************/
double RcsSensor_computeDynamicForceCompensation(const RcsGraph* graph,
                                                 const RcsSensor* fts,
                                                 const MatNd* q_ddot_curr,
                                                 double S_f_dynamic[6])
{
  return RcsSensor_computeForceCompensation(graph, fts, q_ddot_curr, S_f_dynamic,
                                            false, true);
}



/******************************************************************************

  \brief See header.

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

  \brief See header.

*******************************************************************************/

RcsSensor* RcsSensor_create(unsigned int type, const char* name,
                            RcsBody* parentBody, HTr* offset,
                            const char* extraInfo)
{
  RcsSensor* self = RALLOC(RcsSensor);
  RCHECK(self);

  RCHECK(parentBody);

  self->type = (RCSSENSOR_TYPE) type;

  char sensorName[256];
  strcpy(sensorName, name);
  if (parentBody && parentBody->suffix)
  {
    strcat(sensorName, parentBody->suffix);
  }
  self->name = String_clone(sensorName);
  self->body = parentBody;
  self->offset = HTr_clone(offset);
  self->next = NULL;
  self->extraInfo = String_clone(extraInfo);
  self->rawData = MatNd_create(1, RcsSensor_dim(self));

  return self;
}



/******************************************************************************

  \brief Allocates memory and initializes a RcsSensor data structure from
         an XML node. Here's the parsed tags:
         - name
         - type: LOADCELL, JOINTTORQUE, CONTACTFORCE, PPS
         - transform
         - extra_info

******************************************************************************/

RcsSensor* RcsSensor_createFromXML(xmlNode* node, RcsBody* parentBody)
{
  // Return if node is not a sensor node. This can deal with a NULL node
  if (!isXMLNodeName(node, "Sensor"))
  {
    return NULL;
  }

  // read sensor name
  char name[100] = "unnamed sensor";
  getXMLNodePropertyStringN(node, "name", name, 100);
  RLOG(5, "found new sensor node \"%s\" attached to body \"%s\"",
       name, parentBody ? parentBody->name : "NULL");

  // read sensor type
  char buffer[100];
  int xml_type = -1;
  strcpy(buffer, "unknown type");
  getXMLNodePropertyStringN(node, "type", buffer, 100);

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
  HTr offset;
  HTr_setIdentity(&offset);
  if (getXMLNodeProperty(node, "transform"))
  {
    getXMLNodePropertyHTr(node, "transform", &offset);
  }

  // read extraInfo
  char* extraInfo = NULL;
  unsigned int nBytes = getXMLNodeBytes(node, "extra_info");

  if (nBytes>0)
  {
    extraInfo = RNALLOC(nBytes, char);
    getXMLNodePropertyStringN(node, "extra_info", extraInfo, nBytes);
  }

  // Create and return rcs sensor object
  RcsSensor* sensor = RcsSensor_create(xml_type, name, parentBody, &offset, extraInfo);
  RFREE(extraInfo);  // It's Ok to call free() on NULL

  if (sensor->type==RCSSENSOR_PPS)
  {
    int xy[2];
    xy[0] = sensor->rawData->m;
    xy[1] = sensor->rawData->n;

    getXMLNodePropertyIntN(node, "dimensions", xy, 2);
    MatNd_reshape(sensor->rawData, xy[0], xy[1]);
  }


  return sensor;
}



/*******************************************************************************

  \brief Clones a RcsSensor data structure.

*******************************************************************************/

RcsSensor* RcsSensor_clone(const RcsSensor* src, const RcsGraph* dstGraph)
{
  if (src==NULL)
  {
    RLOG(4, "Cloning NULL sensor");
    return NULL;
  }

  if (src->body==NULL)
  {
    RLOG(4, "Sensor \"%s\" has no mount body - skip cloning", src->name);
    return NULL;
  }

  RcsBody* myMountBody = RcsGraph_getBodyByName(dstGraph, src->body->name);

  if (myMountBody==NULL)
  {
    RLOG(4, "Couldn't find mount body \"%s\" for sensor \"%s\" in new graph"
         "- skip cloning", src->body->name, src->name);
    return NULL;
  }

  RcsSensor* self = RALLOC(RcsSensor);
  self->type = src->type;
  self->name = String_clone(src->name);
  self->body = myMountBody;
  self->offset = HTr_clone(src->offset);
  self->extraInfo = String_clone(src->extraInfo);
  self->rawData = MatNd_clone(src->rawData);

  return self;
}



/*******************************************************************************

  \brief Copies a RcsSensor data structure except for a few unknown members.

*******************************************************************************/

void RcsSensor_copy(RcsSensor* self, const RcsSensor* src)
{
  self->type = src->type;
  String_copyOrRecreate(&self->name, src->name);
  HTr_copyOrRecreate(&self->offset, src->offset);
  MatNd_resizeCopy(&self->rawData, src->rawData);
}



/*******************************************************************************

  \brief See header.

*******************************************************************************/

void RcsSensor_destroy(RcsSensor* self)
{
  if (self==NULL)
  {
    return;
  }

  RFREE(self->name);
  RFREE(self->offset);
  RFREE(self->extraInfo);
  RCHECK(self->rawData);
  MatNd_destroy(self->rawData);

  // Reset all internal memory
  memset(self, 0, sizeof(RcsSensor));
  RFREE(self);
}



/*******************************************************************************

  These are the available sensors:
  RCSSENSOR_CUSTOM = 0,
  RCSSENSOR_LOAD_CELL,
  RCSSENSOR_JOINT_TORQUE,
  RCSSENSOR_CONTACT_FORCE,
  RCSSENSOR_PPS


*******************************************************************************/

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
    MatNd* ppsParams = MatNd_createFromString(self->extraInfo);
    unsigned int dim = ppsParams->n;
    MatNd_destroy(ppsParams);
    RCHECK_MSG(dim>0, "Couldn't create PPS sensor \"%s\"", self->name);
    return dim;
  }
  else
  {
    RFATAL("Unsupported sensor type");
  }

  return 0;
}



/*******************************************************************************

  \brief See header.

*******************************************************************************/

void RcsGraph_addSensor(RcsGraph* self, RcsSensor* newSensor)
{
  if (newSensor == NULL)
  {
    return;
  }

  newSensor->next = NULL;

  // check if its the first sensor and assign it
  if (self->sensor == NULL)
  {
    self->sensor = newSensor;
  }
  else
  {
    RcsSensor* lastSensor = self->sensor;

    // find the last of the existing sensors
    while (lastSensor->next != NULL)
    {
      lastSensor = lastSensor->next;
    }
    // copy the new sensor pointer into the next pointer of the previously
    // last sensor
    lastSensor->next = newSensor;
  }
}




/*******************************************************************************
 * See header.
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

  fprintf(out, "\tAttached to body \"%s\"\n", s->body ? s->body->name : "NULL");

  if (s->offset != NULL)
  {
    HTr_fprint(out, s->offset);
  }
  else
  {
    fprintf(out, "\tNo offset\n");
  }

  fprintf(out, "\tRaw sensor data:\n\t");

  for (int i = 1; i <= s->rawData->size; i++)
  {
    fprintf(out, "%+.3f ", s->rawData->ele[i-1]);
    if (i%10==0)
    {
      fprintf(out, "\n\t");
    }
  }

  fprintf(out, "\n");
}



/******************************************************************************

  \brief See header. These tags need to be written:
         - name
         - type: LOADCELL, JOINTTORQUE, CONTACTFORCE, PPS
         - transform
         - extra_info

******************************************************************************/
void RcsSensor_fprintXML(FILE* out, const RcsSensor* self)
{
  char buf[256];

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
      break;

    default:
      RFATAL("Unknown sensor type: %d", self->type);
  }

  // Relative transformation only if non-zero elements exist
  if ((Vec3d_sqrLength(self->offset->org)>1.0e-8) ||
      (Mat3d_getFrobeniusnorm(self->offset->rot)>1.0e-8))
  {
    double trf[6];
    Vec3d_copy(&trf[0], self->offset->org);
    Mat3d_toEulerAngles(&trf[3], (double (*)[3]) self->offset->rot);
    Vec3d_constMulSelf(&trf[3], 180.0 / M_PI);
    fprintf(out, "transform=\"%s ", String_fromDouble(buf, trf[0], 6));
    fprintf(out, "%s ", String_fromDouble(buf, trf[1], 6));
    fprintf(out, "%s ", String_fromDouble(buf, trf[2], 6));
    fprintf(out, "%s ", String_fromDouble(buf, trf[3], 6));
    fprintf(out, "%s ", String_fromDouble(buf, trf[4], 6));
    fprintf(out, "%s\" ", String_fromDouble(buf, trf[5], 6));
  }

  // PPS parameter string
  if (self->extraInfo != NULL)
  {
    fprintf(out, "extra_info=\"%s\" ", self->extraInfo);
  }

  fprintf(out, "/>\n");
}



/******************************************************************************
 * Compute PPS sensor pressure distribution
 *****************************************************************************/
bool RcsSensor_computePPS(const RcsSensor* self, MatNd* ppsResult,
                          const double contactForce[3])
{
  if (self==NULL)
  {
    RLOG(4, "NULL sensor found in PPS function - skipping");
    return false;
  }

  if (self->body==NULL)
  {
    RLOG(4, "Sensor \"%s\" has no mount body - skipping", self->name);
    return false;
  }

  // This array has as many columns as texels. It has 6 rows: Rows 1-3 hold
  // the texel position, rows 4-6 the corresponding texel normal vector.
  MatNd* offsetNormal = MatNd_createFromString(self->extraInfo);
  unsigned int dimension = offsetNormal->n;
  MatNd* lineDirMat = MatNd_create(dimension, 3);
  MatNd* lineOriginMat = MatNd_create(dimension, 3);

  // Determine the mount body transformation from physics
  HTr A_SI;
  const HTr* A_SB = self->offset ? self->offset : HTr_identity();
  HTr_transform(&A_SI, self->body->A_BI, A_SB);

  // calculate the maximum distance of the PPS elements to the mount body origin
  // used to check whether a shape can be completely ignored
  double sensorDepth = 0.005, maxDist = 0.0;

  // Pre-compute lineDir and lineOrigin arrays in world coordinates
  for (unsigned int id = 0; id < dimension; id++)
  {
    double offset[3], lineDir[3];
    for (size_t dim = 0; dim < 3; dim++)
    {
      lineDir[dim] = MatNd_get(offsetNormal, dim+3, id);
      offset[dim] = MatNd_get(offsetNormal, dim, id) +
                    sensorDepth*lineDir[dim]/3.0;
    }

    if (fabs(Vec3d_getLength(lineDir)-1.0)>1.0e-5)
    {
      //MatNd_printCommentDigits("offsetNormal", offsetNormal, 4);
      /* RFATAL("Length of lineDir[%d] = %f - should be 1", id, */
      /*        Vec3d_getLength(lineDir)); */
      Vec3d_setZero(MatNd_getRowPtr(lineDirMat, id));
    }
    else
    {
      maxDist = fmax(maxDist, Vec3d_getLength(offset));

      Vec3d_transRotateSelf(lineDir, A_SI.rot);
      Vec3d_transRotateSelf(offset, A_SI.rot);
      Vec3d_add(MatNd_getRowPtr(lineOriginMat, id), offset, A_SI.org);
      Vec3d_copy(MatNd_getRowPtr(lineDirMat, id), lineDir);
    }
  }

  maxDist += sensorDepth; //just a safety margin



  double* skinSensorData = RNALLOC(dimension, double);// MALLOC
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
    MatNd_destroy(offsetNormal);
    MatNd_destroy(lineDirMat);
    MatNd_destroy(lineOriginMat);
    RFREE(skinSensorData);
    return true;
  }

  // We do not have access to the graph, so we find the root body manually
  RcsBody* rootBdy = RcsBody_getGraphRoot(self->body);

  // Go through all the shapes in the graph to calculate the ray distances
  for (RcsBody* BODY = rootBdy; BODY;
       BODY = RcsBody_depthFirstTraversalGetNext(BODY))
  {
    // Skipping mount body & immediate parent & immediate children
    if ((BODY==self->body) ||
        (BODY==self->body->parent) ||
        (BODY->parent==self->body))
    {
      continue;
    }

    RCSBODY_TRAVERSE_SHAPES(BODY)
    {
      // Ignore shapes that don't calculate distances and those that are
      // too far away anyhow
      HTr A_CI;
      HTr_transform(&A_CI, BODY->A_BI, &SHAPE->A_CB);

      if (((SHAPE->computeType & RCSSHAPE_COMPUTE_DISTANCE) != 0) &&
          (RcsShape_boundingSphereDistance(A_SI.org, &A_CI, SHAPE) < maxDist))
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

          if (RcsShape_computeLineIntersection(lineOrigin, lineDir, BODY->A_BI,
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

  MatNd_destroy(offsetNormal);
  MatNd_destroy(lineDirMat);
  MatNd_destroy(lineOriginMat);
  RFREE(skinSensorData);

  return true;
}
