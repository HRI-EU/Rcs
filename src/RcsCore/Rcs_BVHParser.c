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
#include "Rcs_resourcePath.h"
#include "Rcs_typedef.h"
#include "Rcs_utils.h"
#include "Rcs_body.h"
#include "Rcs_shape.h"
#include "Rcs_joint.h"
#include "Rcs_Vec3d.h"
#include "Rcs_Mat3d.h"
#include "Rcs_basicMath.h"


#define STRNCASEEQ(a,b,n) (strncasecmp((a),(b),(n))==0)

/*******************************************************************************
 *
 ******************************************************************************/
static bool findKeyword(const char* keyword, FILE* fd)
{
  const unsigned int bufLen = 32;
  int nItemsRead = 0;
  char buf[bufLen];

  do
  {
    nItemsRead = fscanf(fd, "%31s", buf);
    RLOG(10, "Reading keyword \"%s\"", buf);

    if (nItemsRead != 1)
    {
      RLOG(1, "Keyword \"%s\": Couldn't read 1 item: %d",
           keyword, nItemsRead);
      continue;
    }

  }
  while ((nItemsRead!=EOF) && (!STRNCASEEQ(buf, keyword, bufLen)));

  return (nItemsRead!=EOF) ? true : false;
}

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
                           const double offset[3], double linearScaleToSI,
                           bool Z_up_x_forward)
{
  int itemsMatched = 0;

  if (STRCASEEQ(buf, "ROOT"))
  {
    RcsBody* child = RcsBody_create();
    itemsMatched = fscanf(fd, "%63s", buf);   // Body name
    RCHECK_MSG(itemsMatched==1, "Couldn't read body name");

    snprintf(child->bdyName, RCS_MAX_NAMELEN, "%s", buf);
    HTr_setZero(&child->Inertia);
    if (Z_up_x_forward)
    {
      //Mat3d_fromEulerAngles2(child->A_BP->rot, M_PI_2, M_PI_2, 0.0);
    }
    RcsBody_addShape(child, createFrameShape(0.5));
#warning "Replace RcsGraph_insertBody in Rcs_BVHParser"
    RcsGraph_insertBody(self, body, child);
    itemsMatched = fscanf(fd, "%63s", buf);   // Curly brace open
    RCHECK_MSG(itemsMatched==1, "Couldn't read curly brace open");
    itemsMatched = fscanf(fd, "%63s", buf);   // Next keyword
    RCHECK_MSG(itemsMatched==1, "Couldn't read next keyword");

    RLOG(5, "Recursing after ROOT with next keyword %s", buf);
    parseRecursive(buf, self, child, fd, Vec3d_zeroVec(), linearScaleToSI,
                   Z_up_x_forward);
  }
  else if (STRCASEEQ(buf, "OFFSET"))
  {
    double offs[3];
    char bufStr[3][256];
    itemsMatched = fscanf(fd, "%255s %255s %255s",
                          bufStr[0], bufStr[1], bufStr[2]);
    RCHECK_MSG(itemsMatched==3, "Couldn't read OFFSET");
    offs[0] = String_toDouble_l(bufStr[0]);
    offs[1] = String_toDouble_l(bufStr[1]);
    offs[2] = String_toDouble_l(bufStr[2]);
    Vec3d_constMulSelf(offs, linearScaleToSI);
    itemsMatched = fscanf(fd, "%63s", buf);   // Next keyword
    RCHECK_MSG(itemsMatched==1, "Couldn't read next keyword");
    RLOG(5, "Recursing after OFFSET with next keyword %s", buf);
    parseRecursive(buf, self, body, fd, offs, linearScaleToSI,
                   Z_up_x_forward);
  }
  else if (STRCASEEQ(buf, "CHANNELS"))
  {
    int nChannels = 0;
    itemsMatched = fscanf(fd, "%d", &nChannels);
    RCHECK_MSG(itemsMatched==1, "Couldn't read number of channels");
    RLOG(5, "Found %d channels", nChannels);

    for (int i=0; i<nChannels; ++i)
    {
      itemsMatched = fscanf(fd, "%63s", buf);   // direction
      RCHECK_MSG(itemsMatched==1, "Couldn't read channel %d", i);

      RcsJoint* jnt = RALLOC(RcsJoint);
      char a[128];
      snprintf(a, 128, "%s_jnt_%s", body->bdyName, buf);
      jnt->name = String_clone(a);
      jnt->weightJL = 1.0;
      jnt->weightMetric = 1.0;
      jnt->ctrlType = RCSJOINT_CTRL_POSITION;

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
        jnt->dirIdx = 0;
      }
      else if (STRNCASEEQ(buf, "Yposition", 63))
      {
        jnt->type = RCSJOINT_TRANS_Y;
        jnt->q_min = -1.0;
        jnt->q_max = 1.0;
        jnt->dirIdx = 1;
      }
      else if (STRNCASEEQ(buf, "Zposition", 63))
      {
        jnt->type = RCSJOINT_TRANS_Z;
        jnt->q_min = -1.0;
        jnt->q_max = 1.0;
        jnt->dirIdx = 2;
      }
      else if (STRNCASEEQ(buf, "Xrotation", 63))
      {
        jnt->type = RCSJOINT_ROT_X;
        jnt->q_min = -M_PI;
        jnt->q_max = M_PI;
        jnt->dirIdx = 0;
      }
      else if (STRNCASEEQ(buf, "Yrotation", 63))
      {
        jnt->type = RCSJOINT_ROT_Y;
        jnt->q_min = -M_PI;
        jnt->q_max = M_PI;
        jnt->dirIdx = 1;
      }
      else if (STRNCASEEQ(buf, "Zrotation", 63))
      {
        jnt->type = RCSJOINT_ROT_Z;
        jnt->q_min = -M_PI;
        jnt->q_max = M_PI;
        jnt->dirIdx = 2;
      }
      else
      {
        RLOG(1, "Unknown direction \"%s\" of CHANNELS", buf);
        return false;
      }

      RcsGraph_insertJoint(self, body, jnt);
    }

    itemsMatched = fscanf(fd, "%63s", buf);   // Next keyword
    RCHECK_MSG(itemsMatched==1, "Couldn't read next keyword");
    RLOG(5, "Recursing after CHANNELS with next keyword %s", buf);
    parseRecursive(buf, self, body, fd, Vec3d_zeroVec(), linearScaleToSI,
                   Z_up_x_forward);
  }
  else if (STRCASEEQ(buf, "JOINT"))
  {
    itemsMatched = fscanf(fd, "%63s", buf);   // Joint link name
    RCHECK_MSG(itemsMatched==1, "Couldn't read joint link name");

    // Create a new body and recursively call this function again
    RcsBody* child = RcsBody_create();
    snprintf(child->bdyName, RCS_MAX_NAMELEN, "%s", buf);
    HTr_setZero(&child->Inertia);
    RcsBody_addShape(child, createFrameShape(0.1));
#warning "Replace RcsGraph_insertBody in Rcs_BVHParser"
    RcsGraph_insertBody(self, body, child);

    itemsMatched = fscanf(fd, "%63s", buf);   // Opening curly brace
    RCHECK_MSG(itemsMatched==1, "Couldn't read opening curly brace");
    itemsMatched = fscanf(fd, "%63s", buf);   // Next keyword
    RCHECK_MSG(itemsMatched==1, "Couldn't read next keyword");
    RLOG(5, "Recursing after OFFSET with next keyword %s", buf);
    bool success = parseRecursive(buf, self, child, fd, Vec3d_zeroVec(),
                                  linearScaleToSI, Z_up_x_forward);
    RCHECK(success);
    itemsMatched = fscanf(fd, "%63s", buf);   // Closing curly brace
    if (itemsMatched < 1)
    {
      RLOG(1, "Couldn't read closing curly brace");
      return false;
    }
  }
  else if (STRCASEEQ(buf, "End"))
  {
    double endOffset[3];
    itemsMatched = fscanf(fd, "%63s %63s %63s", buf, buf, buf); // Site { OFFSET
    RCHECK_MSG(itemsMatched==3, "Couldn't read Site { OFFSET");
    itemsMatched = fscanf(fd, "%63s", buf);  // x
    RCHECK_MSG(itemsMatched==1, "Couldn't read OFFSET x");
    endOffset[0] = String_toDouble_l(buf);
    itemsMatched = fscanf(fd, "%63s", buf);  // y
    RCHECK_MSG(itemsMatched==1, "Couldn't read OFFSET y");
    endOffset[1] = String_toDouble_l(buf);
    itemsMatched = fscanf(fd, "%63s", buf);  // z
    RCHECK_MSG(itemsMatched==1, "Couldn't read OFFSET z");
    endOffset[2] = String_toDouble_l(buf);
    itemsMatched = fscanf(fd, "%63s", buf);  // }
    RCHECK_MSG(itemsMatched==1, "Couldn't read closing curly brace");
    RCHECK(STREQ(buf,"}"));
    itemsMatched = fscanf(fd, "%63s", buf);   // Next keyword
    RCHECK_MSG(itemsMatched==1, "Couldn't read next keyword");

    // Sphere at parent origin
    Vec3d_constMulSelf(endOffset, linearScaleToSI);
    double len = 0.8*Vec3d_getLength(endOffset);

    if (len < 0.01)
    {
      len = 0.01;
    }

    RcsShape* shape = RALLOC(RcsShape);
    HTr_setIdentity(&shape->A_CB);
    shape->scale = 1.0;
    shape->type = RCSSHAPE_SPHERE;
    shape->computeType |= RCSSHAPE_COMPUTE_GRAPHICS;
    shape->extents[0] = 0.1*len;
    shape->extents[1] = 0.1*len;
    shape->extents[2] = 0.1*len;
    strcpy(shape->color, "BLACK_RUBBER");
    RcsBody_addShape(body, shape);


    RLOG(5, "Recursing after END SITE with next keyword %s", buf);
    parseRecursive(buf, self, body, fd, Vec3d_zeroVec(), linearScaleToSI,
                   Z_up_x_forward);
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
  else if (STREQ(buf, "Frame"))   // "Frame Time:"
  {
    return true;
  }
  else
  {
    parseRecursive(buf, self, body, fd, Vec3d_zeroVec(), linearScaleToSI,
                   Z_up_x_forward);
  }

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
static void addGeometry(RcsGraph* self)
{
#ifdef OLD_TOPO

  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    if (STREQ(BODY->name, "BVHROOT"))
    {
      continue;
    }

    RcsBody* CHILD = BODY->firstChild;

    int rr = Math_getRandomInteger(0, 255);
    int gg = Math_getRandomInteger(0, 255);
    int bb = Math_getRandomInteger(0, 255);
    char color[16];
    snprintf(color, 16, "#%02x%02x%02xff", rr, gg, bb);


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

      if (len < 0.01)
      {
        len = 0.01;
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
      snprintf(shape->color, RCS_MAX_NAMELEN, "%s", color);
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
      snprintf(shape->color, RCS_MAX_NAMELEN, "%s", color);
      RcsBody_addShape(BODY, shape);

      CHILD=CHILD->next;
    }
  }

#else
  RFATAL("Implement me");
#endif
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
RcsGraph* RcsGraph_createFromBVHFile(const char* fileName,
                                     double linearScaleToSI,
                                     bool Z_up_x_forward)
{
  FILE* fd = fopen(fileName, "r");

  if (fd==NULL)
  {
    RLOG(1, "Error opening BVH file \"%s\"", fileName);
    return NULL;
  }

  char buf[64] = "";
  bool success = false;
  int itemsMatched = 0;

  // First entry must be "HIERARCHY"
  itemsMatched = fscanf(fd, "%31s", buf);

  if (itemsMatched < 1)
  {
    RLOG(1, "Couldn't read first (HIERARCHY) file item");
    fclose(fd);
    return NULL;
  }

  if (!STRCASEEQ(buf, "HIERARCHY"))
  {
    RLOG(1, "Couldn't find HIERARCHY keyword");
    fclose(fd);
    return NULL;
  }

  // Second entry must be "ROOT"
  itemsMatched = fscanf(fd, "%31s", buf);

  if (itemsMatched < 1)
  {
    RLOG(1, "Couldn't read second (ROOT) file item");
    fclose(fd);
    return NULL;
  }

  if (!STRCASEEQ(buf, "ROOT"))
  {
    RLOG(1, "Couldn't find ROOT keyword");
    fclose(fd);
    return NULL;
  }


  // Create an empty graph that will be propagated recursively
  RcsGraph* self = RALLOC(RcsGraph);
  RCHECK(self);
  self->xmlFile = String_clone(fileName);
  RcsBody* bvhRoot = self->root;

  if (Z_up_x_forward == true)
  {
    RcsBody* xyzRoot = RcsBody_create();
    snprintf(xyzRoot->bdyName, RCS_MAX_NAMELEN, "%s", "BVHROOT");
    HTr_setZero(&xyzRoot->Inertia);
    Mat3d_fromEulerAngles2(xyzRoot->A_BP.rot, M_PI_2, M_PI_2, 0.0);
    RcsBody_addShape(xyzRoot, createFrameShape(1.0));
#warning "Replace RcsGraph_insertBody in Rcs_BVHParser"
    RcsGraph_insertBody(self, NULL, xyzRoot);
    bvhRoot = xyzRoot;
  }

  // Start recursion with root link
  success = parseRecursive(buf, self, bvhRoot, fd, Vec3d_zeroVec(),
                           linearScaleToSI, Z_up_x_forward);
  RCHECK(success);

  fclose(fd);

  RcsGraph_setState(self, NULL, NULL);

  addGeometry(self);

  // Fix reltative transform if translational joints exist. They seem not to
  // be applied on top of what's specified in the offset, but rather overwrite
  // the offset values. We keep it generic here and reset only the tranlsational
  // components that are mentioned in the CHANNEL keyword.
  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    RCSBODY_TRAVERSE_JOINTS(BODY)
    {
      if (BODY->jnt->A_JP == NULL)
      {
        continue;
      }

      if (JNT->type == RCSJOINT_TRANS_X)
      {
        MatNd_set(self->q, JNT->jointIndex, 0, BODY->jnt->A_JP->org[0]);
        BODY->jnt->A_JP->org[0] = 0.0;
      }
      else if (JNT->type == RCSJOINT_TRANS_Y)
      {
        MatNd_set(self->q, JNT->jointIndex, 0, BODY->jnt->A_JP->org[1]);
        BODY->jnt->A_JP->org[1] = 0.0;
      }
      else if (JNT->type == RCSJOINT_TRANS_Z)
      {
        MatNd_set(self->q, JNT->jointIndex, 0, BODY->jnt->A_JP->org[2]);
        BODY->jnt->A_JP->org[2] = 0.0;
      }
    }
  }

  self->q_dot = MatNd_create(self->dof, 1);

  RcsGraph_beautifyHumanModelBVH(self, linearScaleToSI);
  RcsGraph_setState(self, NULL, NULL);

  return self;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
MatNd* RcsGraph_createTrajectoryFromBVHFile(const RcsGraph* graph,
                                            const char* configFile,
                                            double* dt,
                                            double linearScaleToSI,
                                            double angularScaleToSI)
{
  char fileName[256] = "";
  bool fileExists = Rcs_getAbsoluteFileName(configFile, fileName);

  if (fileExists==false)
  {
    REXEC(1)
    {
      RMSG("Resource path is:");
      Rcs_printResourcePath();
      RMSG("RcsGraph bvh file \"%s\" not found in "
           "ressource path - exiting", configFile ? configFile : "NULL");
    }
    return NULL;
  }

  FILE* fd = fopen(fileName, "r");

  if (fd==NULL)
  {
    RLOG(1, "Error opening BVH file \"%s\"", fileName);
    return NULL;
  }


  bool success = findKeyword("MOTION", fd);

  if (success==false)
  {
    RLOG(1, "Couldn't find MOTION keyword - giving up");
    fclose(fd);
    return NULL;
  }

  char buf[64] = "";
  int nItemsRead = fscanf(fd, "%63s", buf);
  RCHECK(nItemsRead==1);
  RCHECK(STRCASEEQ(buf, "Frames:"));

  int numFrames = 0;
  nItemsRead = fscanf(fd, "%d", &numFrames);
  RCHECK(nItemsRead==1);
  RLOG(5, "Trajectory has %d frames", numFrames);

  nItemsRead = fscanf(fd, "%63s", buf);
  RCHECK(nItemsRead==1);
  RCHECK_MSG(STRCASEEQ(buf, "Frame"), "%s", buf);

  nItemsRead = fscanf(fd, "%63s", buf);
  RCHECK(nItemsRead==1);
  RCHECK_MSG(STRCASEEQ(buf, "Time:"), "%s", buf);

  nItemsRead = fscanf(fd, "%63s", buf);
  RCHECK(nItemsRead==1);
  double frameTime = String_toDouble_l(buf);
  RLOG(5, "Trajectory has frameTime %f", frameTime);

  if (dt!=NULL)
  {
    *dt = frameTime;
  }

  fpos_t trajPos;
  int res = fgetpos(fd, &trajPos);
  unsigned int numValues = 0;
  RCHECK(res==0);
  int isEOF = 0;

  do
  {
    isEOF = fscanf(fd, "%63s", buf);
    numValues++;
  }
  while (isEOF != EOF);

  RLOG(5, "Found %d values", numValues);
  numValues--;


  if (numValues%numFrames!=0)
  {
    RLOG(4, "Modulo is %d but should be 0", numValues % numFrames);
    fclose(fd);
    return NULL;
  }

  res = fsetpos(fd, &trajPos);

  RLOG(5, "Creating %d x %d array", numFrames, (int)numValues/numFrames);
  MatNd* data = MatNd_create(numFrames, (int)numValues/numFrames);

  if (graph != NULL)
  {
    RCHECK(graph->dof*numFrames==numValues);
  }

  numValues = 0;
  isEOF = 0;
  do
  {
    isEOF = fscanf(fd, "%63s", buf);
    if (isEOF != EOF)
    {
      data->ele[numValues] = String_toDouble_l(buf);
      numValues++;
    }
  }
  while (isEOF != EOF);



  fclose(fd);

  MatNd* scaleArr = MatNd_create(1, (graph!=NULL) ? graph->dof : data->n);
  if (graph != NULL)
  {
    RCSGRAPH_TRAVERSE_JOINTS(graph)
    {
      scaleArr->ele[JNT->jointIndex] = RcsJoint_isRotation(JNT) ? angularScaleToSI : linearScaleToSI;
    }
  }
  else
  {
    for (unsigned int i=0; i<data->n; ++i)
    {
      scaleArr->ele[i] = (i<3) ? linearScaleToSI : angularScaleToSI;
    }
  }

  for (unsigned int i=0; i<data->m; ++i)
  {
    MatNd row = MatNd_getRowView(data, i);
    MatNd_eleMulSelf(&row, scaleArr);
  }

  MatNd_destroy(scaleArr);

  return data;
}

bool RcsGraph_beautifyHumanModelBVH(RcsGraph* graph,
                                    double linearScaleToSI)
{
  RcsBody* head = RcsGraph_getBodyByName(graph, "Head");
  RcsBody* rightToe = RcsGraph_getBodyByName(graph, "RightToe");
  RcsBody* leftToe = RcsGraph_getBodyByName(graph, "LeftToe");
  RcsBody* rightWrist = RcsGraph_getBodyByName(graph, "RightWrist");
  RcsBody* leftWrist = RcsGraph_getBodyByName(graph, "LeftWrist");
  linearScaleToSI=1;
  if (head==NULL)
  {
    RLOG(4, "No body with name \"Head\" found");
    return false;
  }

  if (rightToe==NULL)
  {
    RLOG(4, "No body with name \"RightToe\" found");
    return false;
  }

  if (leftToe==NULL)
  {
    RLOG(4, "No body with name \"LeftToe\" found");
    return false;
  }

  if (rightWrist==NULL)
  {
    RLOG(4, "No body with name \"RightWrist\" found");
    return false;
  }

  if (leftWrist==NULL)
  {
    RLOG(4, "No body with name \"LeftWrist\" found");
    return false;
  }

  // Head
  RcsShape* shape = RALLOC(RcsShape);
  HTr_setIdentity(&shape->A_CB);
  Vec3d_set(shape->A_CB.org, 0.0, 0.12, 0.0);
  shape->scale = 1.0;
  shape->type = RCSSHAPE_SPHERE;
  shape->computeType |= RCSSHAPE_COMPUTE_GRAPHICS;
  Vec3d_set(shape->extents, 0.12, 0.0, 0.0);
  strcpy(shape->color, "YELLOW");
  Vec3d_constMulSelf(shape->A_CB.org, linearScaleToSI);
  Vec3d_constMulSelf(shape->extents, linearScaleToSI);
  RcsBody_addShape(head, shape);

  shape = RcsShape_clone(shape);
  Vec3d_set(shape->A_CB.org, 0.0, 0.16, 0.05);
  shape->type = RCSSHAPE_BOX;
  Vec3d_set(shape->extents, 0.16, 0.1, 0.08);
  Vec3d_constMulSelf(shape->A_CB.org, linearScaleToSI);
  Vec3d_constMulSelf(shape->extents, linearScaleToSI);
  RcsBody_addShape(head, shape);

  // Feet
  shape = RcsShape_clone(shape);
  Vec3d_set(shape->A_CB.org, 0.0, 0.02, -0.05);
  Vec3d_set(shape->extents, 0.12, 0.04, 0.3);
  strcpy(shape->color, "RED");
  Vec3d_constMulSelf(shape->A_CB.org, linearScaleToSI);
  Vec3d_constMulSelf(shape->extents, linearScaleToSI);
  RcsBody_addShape(rightToe, shape);

  shape = RcsShape_clone(shape);
  Vec3d_set(shape->A_CB.org, 0.0, 0.02, -0.05);
  Vec3d_set(shape->extents, 0.12, 0.04, 0.3);
  Vec3d_constMulSelf(shape->A_CB.org, linearScaleToSI);
  Vec3d_constMulSelf(shape->extents, linearScaleToSI);
  RcsBody_addShape(leftToe, shape);

  // Left hand
  shape = RcsShape_clone(shape);
  Vec3d_set(shape->A_CB.org, 0.1, 0.0, 0.0);
  Vec3d_set(shape->extents, 0.16, 0.02, 0.1);
  strcpy(shape->color, "BLUE");
  Vec3d_constMulSelf(shape->A_CB.org, linearScaleToSI);
  Vec3d_constMulSelf(shape->extents, linearScaleToSI);
  RcsBody_addShape(leftWrist, shape);

  shape = RcsShape_clone(shape);
  Vec3d_set(shape->A_CB.org, 0.03, -0.04, 0.0);
  Vec3d_set(shape->extents, 0.02, 0.08, 0.1);
  Vec3d_constMulSelf(shape->A_CB.org, linearScaleToSI);
  Vec3d_constMulSelf(shape->extents, linearScaleToSI);
  RcsBody_addShape(leftWrist, shape);

  // Right hand
  shape = RcsShape_clone(shape);
  Vec3d_set(shape->A_CB.org, -0.1, 0.0, 0.0);
  Vec3d_set(shape->extents, 0.16, 0.02, 0.1);
  Vec3d_constMulSelf(shape->A_CB.org, linearScaleToSI);
  Vec3d_constMulSelf(shape->extents, linearScaleToSI);
  RcsBody_addShape(rightWrist, shape);

  shape = RcsShape_clone(shape);
  Vec3d_set(shape->A_CB.org, -0.03, -0.04, 0.0);
  Vec3d_set(shape->extents, 0.02, 0.08, 0.1);
  Vec3d_constMulSelf(shape->A_CB.org, linearScaleToSI);
  Vec3d_constMulSelf(shape->extents, linearScaleToSI);
  RcsBody_addShape(rightWrist, shape);

  return true;
}
