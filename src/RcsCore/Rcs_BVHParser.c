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

#include "Rcs_BVHParser.h"
#include "Rcs_macros.h"
#include "Rcs_typedef.h"
#include "Rcs_utils.h"
#include "Rcs_body.h"
#include "Rcs_Vec3d.h"
#include "Rcs_Mat3d.h"
#include "Rcs_basicMath.h"

#include <stdio.h>


#define STRNCASEEQ(a,b,n) (strncasecmp((a),(b),(n))==0)

/*******************************************************************************
 *
 ******************************************************************************/
static RcsShape* createFrameShape(double scale)
{
  RcsShape* shape = RALLOC(RcsShape);
  HTr_setIdentity(&shape->A_CB);
  shape->scale = scale;
  shape->type = RCSSHAPE_REFFRAME;
  shape->computeType |= RCSSHAPE_COMPUTE_GRAPHICS;
  shape->extents[0] = 0.9;
  shape->extents[1] = 0.9;
  shape->extents[2] = 0.9;

  return shape;
}

/*******************************************************************************
 *
 ******************************************************************************/
static bool parseRecursive(char* buf, RcsGraph* self, RcsBody* body, FILE* fd,
                           const double offset[3])
{
  if (STRCASEEQ(buf, "ROOT"))
  {
    RcsBody* child = RALLOC(RcsBody);
    child->A_BI = HTr_create();
    fscanf(fd, "%63s", buf);   // Body name
    child->name = String_clone(buf);
    child->Inertia = HTr_create();
    Mat3d_setZero(child->Inertia->rot);
    RcsBody_addShape(child, createFrameShape(2.0));
    RcsGraph_insertBody(self, body, child);
    fscanf(fd, "%63s", buf);   // Curly brace open
    fscanf(fd, "%63s", buf);   // Next keyword

    RLOG(5, "Recursing after ROOT with next keyword %s", buf);
    parseRecursive(buf, self, child, fd, Vec3d_zeroVec());
  }
  else if (STRCASEEQ(buf, "OFFSET"))
  {
    double offs[3];
    fscanf(fd, "%lf %lf %lf", &offs[0], &offs[1], &offs[2]);
    fscanf(fd, "%63s", buf);   // Next keyword
    RLOG(5, "Recursing after OFFSET with next keyword %s", buf);
    parseRecursive(buf, self, body, fd, offs);
  }
  else if (STRCASEEQ(buf, "CHANNELS"))
  {
    int nChannels = 0;
    fscanf(fd, "%d", &nChannels);
    RLOG(5, "Found %d channels", nChannels);

    for (int i=0; i<nChannels; ++i)
    {
      fscanf(fd, "%63s", buf);   // direction

      RcsJoint* jnt = RALLOC(RcsJoint);
      char a[128];
      sprintf(a, "%s_jnt_%s", body->name, buf);
      jnt->name = String_clone(a);
      jnt->weightJL = 1.0;
      jnt->weightMetric = 1.0;

      if ((i==0) && (Vec3d_sqrLength(offset)>0.0))
      {
        jnt->A_JP = HTr_create();
        Vec3d_copy(jnt->A_JP->org, offset);
      }

      if (STRNCASEEQ(buf, "Xposition", 63))
      {
        jnt->type = RCSJOINT_TRANS_X;
        jnt->q_min = -1.0;
        jnt->q_max = 1.0;
      }
      else if (STRNCASEEQ(buf, "Yposition", 63))
      {
        jnt->type = RCSJOINT_TRANS_Y;
        jnt->q_min = -1.0;
        jnt->q_max = 1.0;
      }
      else if (STRNCASEEQ(buf, "Zposition", 63))
      {
        jnt->type = RCSJOINT_TRANS_Z;
        jnt->q_min = -1.0;
        jnt->q_max = 1.0;
      }
      else if (STRNCASEEQ(buf, "Xrotation", 63))
      {
        jnt->type = RCSJOINT_ROT_X;
        jnt->q_min = -M_PI;
        jnt->q_max = M_PI;
      }
      else if (STRNCASEEQ(buf, "Yrotation", 63))
      {
        jnt->type = RCSJOINT_ROT_Y;
        jnt->q_min = -M_PI;
        jnt->q_max = M_PI;
      }
      else if (STRNCASEEQ(buf, "Zrotation", 63))
      {
        jnt->type = RCSJOINT_ROT_Z;
        jnt->q_min = -M_PI;
        jnt->q_max = M_PI;
      }
      else
      {
        RLOG(1, "Unknown direction \"%s\" of CHANNELS", buf);
        return false;
      }

      RcsGraph_insertJoint(self, body, jnt);
    }

    fscanf(fd, "%63s", buf);   // Next keyword
    RLOG(5, "Recursing after CHANNELS with next keyword %s", buf);
    parseRecursive(buf, self, body, fd, Vec3d_zeroVec());
  }
  else if (STRCASEEQ(buf, "JOINT"))
  {
    fscanf(fd, "%63s", buf);   // Joint link name

    // Create a new body and recursively call this function again
    RcsBody* child = RALLOC(RcsBody);
    child->A_BI = HTr_create();
    child->name = String_clone(buf);
    child->Inertia = HTr_create();
    Mat3d_setZero(child->Inertia->rot);
    RcsBody_addShape(child, createFrameShape(1.0));
    RcsGraph_insertBody(self, body, child);

    fscanf(fd, "%63s", buf);   // Opening curly brace
    fscanf(fd, "%63s", buf);   // Next keyword
    RLOG(5, "Recursing after OFFSET with next keyword %s", buf);
    bool success = parseRecursive(buf, self, child, fd, Vec3d_zeroVec());
    RCHECK(success);
    fscanf(fd, "%63s", buf);   // Closing curly brace
  }
  else if (STRCASEEQ(buf, "End"))
  {
    double endOffset[3];
    fscanf(fd, "%63s", buf);  // Site
    fscanf(fd, "%63s", buf);  // {
    fscanf(fd, "%63s", buf);  // OFFSET
    fscanf(fd, "%lf", &endOffset[0]);  // x
    fscanf(fd, "%lf", &endOffset[1]);  // y
    fscanf(fd, "%lf", &endOffset[2]);  // z
    fscanf(fd, "%63s", buf);  // }
    RCHECK(STREQ(buf,"}"));
    fscanf(fd, "%63s", buf);   // Next keyword

    // Sphere at parent origin
    double len = 0.8*Vec3d_getLength(endOffset);
    RcsShape* shape = RALLOC(RcsShape);
    HTr_setIdentity(&shape->A_CB);
    shape->scale = 1.0;
    shape->type = RCSSHAPE_SPHERE;
    shape->computeType |= RCSSHAPE_COMPUTE_GRAPHICS;
    shape->extents[0] = 0.1*len;
    shape->extents[1] = 0.1*len;
    shape->extents[2] = 0.1*len;
    shape->color = String_clone("BLACK_RUBBER");
    RcsBody_addShape(body, shape);


    RLOG(5, "Recursing after END SITE with next keyword %s", buf);
    parseRecursive(buf, self, body, fd, Vec3d_zeroVec());
  }


  RLOG(5, "Reched end of recursion with next keyword %s", buf);

  if (STREQ(buf, "}"))
  {
    return true;
  }
  else if (STREQ(buf, "MOTION"))
  {
    return true;
  }
  else if (STREQ(buf, "Frames:"))
  {
    return true;
  }
  else if (STREQ(buf, "Frame Time:"))
  {
    return true;
  }
  else
  {
    parseRecursive(buf, self, body, fd, Vec3d_zeroVec());
  }

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
static void addGeometry(RcsGraph* self)
{

  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    RcsBody* CHILD = BODY->firstChild;

    int rr = Math_getRandomInteger(0, 255);
    int gg = Math_getRandomInteger(0, 255);
    int bb = Math_getRandomInteger(0, 255);
    char color[256];
    sprintf(color, "#%02x%02x%02xff", rr, gg, bb);


    while (CHILD!=NULL)
    {
      RLOG(5, "%s: Traversing child %s", BODY->name, CHILD->name);

      const double* I_p1 = BODY->A_BI->org;
      const double* I_p2 = CHILD->A_BI->org;

      double K_p1[3], K_p2[3], K_p12[3], K_center[3];
      Vec3d_invTransform(K_p1, BODY->A_BI,I_p1);
      Vec3d_invTransform(K_p2, BODY->A_BI,I_p2);
      Vec3d_sub(K_p12, K_p2, K_p1);
      Vec3d_constMulAndAdd(K_center, K_p1, K_p12, 0.5);
      double len = 0.8*Vec3d_getLength(K_p12);
      if (len < 0.2)
      {
        len = 0.2;
      }

      // Box from parent to child
      RcsShape* shape = RALLOC(RcsShape);
      HTr_setIdentity(&shape->A_CB);
      shape->scale = 1.0;
      shape->type = RCSSHAPE_BOX;
      shape->computeType |= RCSSHAPE_COMPUTE_GRAPHICS;
      shape->extents[0] = 0.2*len;
      shape->extents[1] = 0.2*len;
      shape->extents[2] = len;
      shape->color = String_clone(color);
      Mat3d_fromVec(shape->A_CB.rot, K_p12, 2);
      Vec3d_copy(shape->A_CB.org, K_center);
      RcsBody_addShape(BODY, shape);

      // Sphere at parent origin
      shape = RALLOC(RcsShape);
      HTr_setIdentity(&shape->A_CB);
      shape->scale = 1.0;
      shape->type = RCSSHAPE_SPHERE;
      shape->computeType |= RCSSHAPE_COMPUTE_GRAPHICS;
      shape->extents[0] = 0.15*len;
      shape->extents[1] = 0.15*len;
      shape->extents[2] = 0.15*len;
      shape->color = String_clone(color);
      RcsBody_addShape(BODY, shape);

      CHILD=CHILD->next;
    }
  }

}

/*******************************************************************************
 * See header.
 ******************************************************************************/
RcsGraph* RcsGraph_createFromBVHFile(const char* fileName)
{
  FILE* fd = fopen(fileName, "r");

  if (fd==NULL)
  {
    RLOG(1, "Error opening BVH file \"%s\"", fileName);
    return NULL;
  }

  char buf[64] = "";
  bool success = false;

  // First entry must be "HIERARCHY"
  fscanf(fd, "%31s", buf);
  RCHECK(STRCASEEQ(buf, "HIERARCHY"));

  // Second entry must be "ROOT"
  fscanf(fd, "%31s", buf);
  RCHECK(STRCASEEQ(buf, "ROOT"));


  // Create an empty graph that will be propagated recursively
  RcsGraph* self = RALLOC(RcsGraph);
  RCHECK(self);
  self->xmlFile = String_clone(fileName);

  // Start recursion with root link
  success = parseRecursive(buf, self, self->root, fd, Vec3d_zeroVec());
  RCHECK(success);

  fclose(fd);

  RcsGraph_setState(self, NULL, NULL);

  addGeometry(self);

  RLOG(5, "Reached end");

  return self;
}
