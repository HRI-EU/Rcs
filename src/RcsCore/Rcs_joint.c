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
#include "Rcs_joint.h"
#include "Rcs_utils.h"
#include "Rcs_macros.h"
#include "Rcs_math.h"

#include <float.h>



/*******************************************************************************
 *
 ******************************************************************************/
void RcsJoint_init(RcsJoint* self)
{
  memset(self, 0, sizeof(RcsJoint));
  HTr_setIdentity(&self->A_JP);
  HTr_setIdentity(&self->A_JI);
  self->id = -1;
  self->prevId = -1;
  self->nextId = -1;
  self->coupledToId = -1;
  self->jointIndex = -1;
  self->jacobiIndex = -1;
  self->type = -1;
  self->speedLimit = DBL_MAX;
  self->accLimit = DBL_MAX;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool RcsJoint_isRotation(const RcsJoint* joint)
{
  if (joint == NULL)
  {
    return false;
  }

  if ((joint->type == RCSJOINT_ROT_X) ||
      (joint->type == RCSJOINT_ROT_Y) ||
      (joint->type == RCSJOINT_ROT_Z))
  {
    return true;
  }

  return false;
}



/******************************************************************************

  \brief See header.

******************************************************************************/

bool RcsJoint_isTranslation(const RcsJoint* joint)
{
  if (joint == NULL)
  {
    return false;
  }

  if ((joint->type == RCSJOINT_TRANS_X) ||
      (joint->type == RCSJOINT_TRANS_Y) ||
      (joint->type == RCSJOINT_TRANS_Z))
  {
    return true;
  }

  return false;
}



/******************************************************************************

  \brief See header.

******************************************************************************/

void RcsJoint_fprint(FILE* out, const RcsJoint* jnt, const RcsGraph* graph)
{
  if (jnt==NULL)
  {
    fprintf(out, "[RcsJoint_fprint:%d] Joint is NULL\n", __LINE__);
    return;
  }

  fprintf(out, "[RcsJoint_fprint(%s):%d] \n", jnt->name, __LINE__);
  fprintf(out, "\tq0           = %f\n", jnt->q0);
  fprintf(out, "\tq_init       = %f\n", jnt->q_init);
  fprintf(out, "\tq_min        = %f\n", jnt->q_min);
  fprintf(out, "\tq_max        = %f\n", jnt->q_max);
  fprintf(out, "\tmaxTorque    = %f\n", jnt->maxTorque);

  if (jnt->speedLimit==DBL_MAX)
  {
    fprintf(out, "\tspeedLimit   = DBL_MAX\n");
  }
  else
  {
    fprintf(out, "\tspeedLimit   = %f\n", jnt->speedLimit);
  }

  if (jnt->accLimit==DBL_MAX)
  {
    fprintf(out, "\taccLimit   = DBL_MAX\n");
  }
  else
  {
    fprintf(out, "\taccLimit   = %f\n", jnt->accLimit);
  }

  if (jnt->decLimit==DBL_MAX)
  {
    fprintf(out, "\tdecLimit   = DBL_MAX\n");
  }
  else
  {
    fprintf(out, "\tdecLimit   = %f\n", jnt->decLimit);
  }

  fprintf(out, "\tweightJL     = %f\n", jnt->weightJL);
  fprintf(out, "\tweightCA     = %f\n", jnt->weightCA);
  fprintf(out, "\tweightMetric = %f\n", jnt->weightMetric);
  fprintf(out, "\tconstrained  = %d\n", jnt->constrained);
  fprintf(out, "\ttype         = %s\n", RcsJoint_typeName(jnt->type));

  fprintf(out, "\tdirection    = %f   %f   %f\n",
          jnt->A_JI.rot[jnt->dirIdx][0],
          jnt->A_JI.rot[jnt->dirIdx][1],
          jnt->A_JI.rot[jnt->dirIdx][2]);

  const RcsJoint* prevJnt = RCSJOINT_BY_ID(graph, jnt->prevId);
  const RcsJoint* nextJnt = RCSJOINT_BY_ID(graph, jnt->nextId);

  fprintf(out, "\tprev. joint  = %s\n", prevJnt ? prevJnt->name : "NULL");
  fprintf(out, "\tnext  joint  = %s\n", nextJnt ? nextJnt->name : "NULL");
  fprintf(out, "\tJacobi index = %d\n", jnt->jacobiIndex);
  fprintf(out, "\tq-index = %d\n", jnt->jointIndex);
  fprintf(out, "\tid = %d\n", jnt->id);
  fprintf(out, "\tA_JP         = ");
  if (HTr_isIdentity(&jnt->A_JP))
  {
    fprintf(out, "Identity");
  }
  else
  {
    HTr_fprint(out, &jnt->A_JP);
  }

  fprintf(out, "\tA_JI         = ");
  HTr_fprint(out, &jnt->A_JI);
}



/******************************************************************************

  \brief See header.

******************************************************************************/

void RcsJoint_fprintType(FILE* out, const RcsJoint* jnt)
{
  RCHECK(jnt);
  fprintf(out, "[RcsJoint_fprintType():%d] %s\n",
          __LINE__, RcsJoint_typeName(jnt->type));
}



/******************************************************************************

  \brief See header.

******************************************************************************/

const char* RcsJoint_typeName(int type)
{
  static char tStr[][256] =
  {
    "RCSJOINT_ROT_X",
    "RCSJOINT_ROT_Y",
    "RCSJOINT_ROT_Z",
    "RCSJOINT_TRANS_X",
    "RCSJOINT_TRANS_Y",
    "RCSJOINT_TRANS_Z",
    "Unknown joint type"
  };
  const char* ptr = NULL;

  switch (type)
  {
    case RCSJOINT_ROT_X:
      ptr = tStr[0];
      break;
    case RCSJOINT_ROT_Y:
      ptr = tStr[1];
      break;
    case RCSJOINT_ROT_Z:
      ptr = tStr[2];
      break;
    case RCSJOINT_TRANS_X:
      ptr = tStr[3];
      break;
    case RCSJOINT_TRANS_Y:
      ptr = tStr[4];
      break;
    case RCSJOINT_TRANS_Z:
      ptr = tStr[5];
      break;
    default:
      ptr = tStr[6];
      break;
  }

  return ptr;
}



/******************************************************************************

   \brief Destroys and frees all memory for a joint.

******************************************************************************/

void RcsJoint_destroy(RcsJoint* self)
{
  RFREE(self);
}



/******************************************************************************

  \brief Makes a deep copy of a RcsJoint data structure except for the
         connection ids (id, prevId, nextId, coupledToId).

******************************************************************************/

void RcsJoint_copy(RcsJoint* dst, const RcsJoint* src)
{
  int id = dst->id;
  int prevId = dst->prevId;
  int nextId = dst->nextId;
  int coupledToId = dst->coupledToId;

  memcpy(dst, src, sizeof(RcsJoint));

  dst->id = id;
  dst->prevId = prevId;
  dst->nextId = nextId;
  dst->coupledToId = coupledToId;
}



/******************************************************************************

  \brief Return joint angle of salve joint with respect to master joint angle
         analytical solution for Asimo version2.0 elbow joint (should be
         generalized)

         q_sl = Sum_i=0^order(param[i]*q_master^(order-1-i)

******************************************************************************/

static double RcsJoint_calcCouplingPolynomial(const double q_master,
                                              const double* coeff,
                                              unsigned int nCoeff)
{
  const int orderM1 = nCoeff-1;
  double q_slave = 0.0;

  for (int i=0; i<=orderM1; i++)
  {
    q_slave += coeff[i] * pow(q_master, (int)orderM1-i);
  }

  return q_slave;
}



/******************************************************************************

  \brief Calculates the derivative of RcsJoint_calcCouplingPolynomial()

******************************************************************************/

static double RcsJoint_calcCouplingPolynomialDerivative(const double q_master,
                                                        const double* coeff,
                                                        unsigned int nCoeff)
{
  const unsigned int orderM1 = nCoeff-1;
  double dq_slave = 0.0;

  for (unsigned int i=0; i<orderM1; i++)
  {
    dq_slave += (orderM1-i)*coeff[i] * pow(q_master, (int)(orderM1-1-i));
  }

  return dq_slave;
}



/******************************************************************************

  \brief See header.

******************************************************************************/

double RcsJoint_computeSlaveJointAngle(const RcsGraph* graph,
                                       const RcsJoint* slave,
                                       const double q_master)
{
  const RcsJoint* master = RCSJOINT_BY_ID(graph, slave->coupledToId);
  double q_slave = 0.0;

  if (master==NULL)
  {
    RLOG(1, "Master joint of joint \"%s\" is NULL", slave->name);
    return 0.0;
  }

  if (slave->nCouplingCoeff == 1)
  {
    q_slave = slave->q_init + slave->couplingPoly[0]*(q_master-master->q_init);
  }
  else
  {

    // If out of range, do tangent at the border value
    if (q_master < master->q_min)
    {
      const double sens =
        RcsJoint_calcCouplingPolynomialDerivative(master->q_min-master->q_init,
                                                  slave->couplingPoly,
                                                  slave->nCouplingCoeff);
      q_slave = slave->q_min - sens*(master->q_min - q_master);
    }
    else if (q_master > master->q_max)
    {
      const double sens =
        RcsJoint_calcCouplingPolynomialDerivative(master->q_max-master->q_init,
                                                  slave->couplingPoly,
                                                  slave->nCouplingCoeff);
      q_slave = slave->q_max + sens*(q_master - master->q_max);
    }
    else
    {
      q_slave = slave->q_init +
                RcsJoint_calcCouplingPolynomial(q_master-master->q_init,
                                                slave->couplingPoly,
                                                slave->nCouplingCoeff);
    }

  }

  return q_slave;
}



/******************************************************************************

  \brief See header.

******************************************************************************/

double RcsJoint_computeSlaveJointVelocity(const RcsGraph* graph,
                                          const RcsJoint* slave,
                                          const double q_master,
                                          const double q_dot_master)
{
  const RcsJoint* master = RCSJOINT_BY_ID(graph, slave->coupledToId);
  double q_dot_slave = 0.0;

  if (master==NULL)
  {
    RLOG(1, "Master joint of joint \"%s\" is NULL", slave->name);
    return 0.0;
  }

  // Linear scaling of velocity
  if (slave->nCouplingCoeff == 1)
  {
    q_dot_slave = slave->couplingPoly[0]*q_dot_master;
  }
  // Clip to work range
  else
  {
    const double q_ltd = Math_clip(q_master, master->q_min, master->q_max);
    const double s =
      RcsJoint_calcCouplingPolynomialDerivative(q_ltd, slave->couplingPoly,
                                                slave->nCouplingCoeff);
    q_dot_slave = s*q_dot_master;
  }

  return q_dot_slave;
}



/******************************************************************************

  \brief See header.

******************************************************************************/

void RcsJoint_fprintXML(FILE* out, const RcsJoint* self, const RcsGraph* graph)
{
  char buf[256];
  const double scaleToXML = RcsJoint_isRotation(self) ? (180.0/M_PI) : 1.0;

  fprintf(out, "    <Joint ");
  fprintf(out, "name=\"%s\" ", self->name);
  fprintf(out, "range=\"%s ", String_fromDouble(buf, scaleToXML*self->q_min, 6));
  fprintf(out, "%s ", String_fromDouble(buf, scaleToXML*self->q_init, 6));
  fprintf(out, "%s\" ", String_fromDouble(buf, scaleToXML*self->q_max, 6));

  if (self->weightJL != 1.0)
  {
    fprintf(out, "weightJL=\"%s\" ", String_fromDouble(buf, self->weightJL, 6));
  }

  if (self->weightCA != 1.0)
  {
    fprintf(out, "weightCA=\"%s\" ", String_fromDouble(buf, self->weightCA, 6));
  }

  if (self->weightMetric != 1.0)
  {
    fprintf(out, "weightMetric=\"%s\" ",
            String_fromDouble(buf, self->weightMetric, 6));
  }

  if (self->constrained == true)
  {
    fprintf(out, "constraint=\"true\" ");
  }

  switch (self->type)
  {
    case RCSJOINT_TRANS_X:
      fprintf(out, "type=\"TransX\" ");
      break;

    case RCSJOINT_TRANS_Y:
      fprintf(out, "type=\"TransY\" ");
      break;

    case RCSJOINT_TRANS_Z:
      fprintf(out, "type=\"TransZ\" ");
      break;

    case RCSJOINT_ROT_X:
      fprintf(out, "type=\"RotX\" ");
      break;

    case RCSJOINT_ROT_Y:
      fprintf(out, "type=\"RotY\" ");
      break;

    case RCSJOINT_ROT_Z:
      fprintf(out, "type=\"RotZ\" ");
      break;

    default:
      RFATAL("Unknown control type: %d", self->ctrlType);
  }

  if (!HTr_isIdentity(&self->A_JP))
  {
    double trf[6];
    Vec3d_copy(&trf[0], self->A_JP.org);
    Mat3d_toEulerAngles(&trf[3], (double (*)[3]) self->A_JP.rot);
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

  if ((self->maxTorque!=DBL_MAX) &&
      !(self->ctrlType==RCSJOINT_CTRL_TORQUE && self->maxTorque!=1.0))
  {
    fprintf(out, "torqueLimit=\"%s\" ",
            String_fromDouble(buf, self->maxTorque, 6));
  }

  if (self->speedLimit < DBL_MAX)
  {
    fprintf(out, "speedLimit=\"%s\" ",
            String_fromDouble(buf, scaleToXML*self->speedLimit, 6));
  }

  if (self->accLimit < DBL_MAX)
  {
    fprintf(out, "accelerationLimit=\"%s\" ",
            String_fromDouble(buf, scaleToXML*self->accLimit, 6));
  }

  if ((self->decLimit < DBL_MAX) && (self->decLimit!=self->accLimit))
  {
    fprintf(out, "decelerationLimit=\"%s\" ",
            String_fromDouble(buf, scaleToXML*self->decLimit, 6));
  }

  switch (self->ctrlType)
  {
    case RCSJOINT_CTRL_POSITION:
      break;

    case RCSJOINT_CTRL_VELOCITY:
      fprintf(out, "ctrlType=\"Position\" ");
      break;

    case RCSJOINT_CTRL_TORQUE:
      fprintf(out, "ctrlType=\"Torque\" ");
      break;

    default:
      RFATAL("Unknown control type: %d", self->ctrlType);
  }

  if (self->coupledToId != -1)
  {
    RcsJoint* cpldTo = RCSJOINT_BY_ID(graph, self->coupledToId);
    fprintf(out, "coupledTo=\"%s\" ", cpldTo->name);
  }

  if (self->nCouplingCoeff > 0)
  {
    fprintf(out, "couplingFactor=\"");

    for (unsigned int i=0; i<self->nCouplingCoeff; i++)
    {
      fprintf(out, "%s ", String_fromDouble(buf, self->couplingPoly[i], 6));
    }

    fprintf(out, "\"");
  }

  fprintf(out, "/>\n");
}



/******************************************************************************

  \brief See header.

******************************************************************************/

int RcsJoint_getJointIndex(const RcsJoint* self)
{
  return self ? self->jointIndex : -1;
}



/******************************************************************************

\brief See header.

******************************************************************************/

int RcsJoint_getDirectionIndex(const RcsJoint* self)
{
  int idx = -1;

  switch (self->type)
  {
    case RCSJOINT_TRANS_X:
    case RCSJOINT_ROT_X:
    {
      idx = 0;
      break;
    }
    case RCSJOINT_TRANS_Y:
    case RCSJOINT_ROT_Y:
    {
      idx = 1;
      break;
    }
    case RCSJOINT_TRANS_Z:
    case RCSJOINT_ROT_Z:
    {
      idx = 2;
      break;
    }
  }

  return idx;
}

/*******************************************************************************
 *
 ******************************************************************************/
void RcsJoint_scale(RcsJoint* joint, double scale)
{
  Vec3d_constMulSelf(joint->A_JP.org, scale);

  if (RcsJoint_isTranslation(joint) == true)
  {
    const double upperRange = joint->q_max - joint->q0;
    const double lowerRange = joint->q0 - joint->q_min;
    joint->q0 *= scale;
    joint->q_init *= scale;
    joint->q_min = joint->q0 - scale*lowerRange;
    joint->q_max = joint->q0 + scale*upperRange;
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
RcsJoint* RcsJoint_first(const RcsGraph* graph)
{
  return graph->dof > 0 ? &graph->joints[0] : NULL;
}

/*******************************************************************************
 *
 ******************************************************************************/
RcsJoint* RcsJoint_last(const RcsGraph* graph)
{
  return graph->dof > 0 ? &graph->joints[graph->dof-1] : NULL;
}

/*******************************************************************************
 *
 ******************************************************************************/
RcsJoint* RcsJoint_next(const RcsJoint* joint, const RcsGraph* graph)
{
  return RCSJOINT_BY_ID(graph, joint->nextId);
}

/*******************************************************************************
 *
 ******************************************************************************/
RcsJoint* RcsJoint_prev(const RcsJoint* joint, const RcsGraph* graph)
{
  return RCSJOINT_BY_ID(graph, joint->prevId);
}

/*******************************************************************************
 *
 ******************************************************************************/
RcsJoint* RcsJoint_master(const RcsJoint* joint, const RcsGraph* graph)
{
  return RCSJOINT_BY_ID(graph, joint->coupledToId);
}
