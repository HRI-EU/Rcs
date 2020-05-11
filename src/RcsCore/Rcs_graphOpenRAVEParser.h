/******************************************************************************

  Copyright (C) by
  Honda Research Institute Europe GmbH,
  Carl-Legien Str. 30
  63073 Offenbach/Main
  Germany

  UNPUBLISHED PROPRIETARY MATERIAL.
  ALL RIGHTS RESERVED

  Author: Manuel Muehlig

******************************************************************************/

#ifndef RCS_GRAPHOPENRAVEPARSER_H
#define RCS_GRAPHOPENRAVEPARSER_H

#include <libxml/tree.h>
#include "Rcs_graph.h"

#ifdef __cplusplus
extern "C" {
#endif

void RcsGraph_createBodiesFromOpenRAVEFile(RcsGraph* self, RcsBody* parent,
                                           const char* configFile, double* q0,
                                           unsigned int nq);

void RcsGraph_createBodiesFromOpenRAVENode(RcsGraph* self, RcsBody* parent,
                                           const xmlNodePtr node, double* q0,
                                           unsigned int nq);

RcsBody* RcsBody_createFromOpenRAVEXML(RcsGraph* self, xmlNode* bdyNode,
                                       RcsBody* root);

RcsJoint* RcsJoint_createFromOpenRAVEXML(RcsGraph* self, xmlNode* node,
                                         const double* q0);

RcsShape* RcsShape_createFromOpenRAVEXML(xmlNode* node, RcsBody* body);

xmlNodePtr getXMLNodeForOpenRave(const char* filename, xmlDocPtr* doc);

#ifdef __cplusplus
}
#endif

#endif   // RCS_GRAPHOPENRAVEPARSER_H
