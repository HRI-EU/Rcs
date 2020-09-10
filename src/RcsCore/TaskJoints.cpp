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

#include "TaskJoints.h"
#include "TaskJoint.h"
#include "TaskFactory.h"
#include "Rcs_macros.h"
#include "Rcs_typedef.h"
#include "Rcs_parser.h"
#include "Rcs_stlParser.h"
#include "Rcs_VecNd.h"
#include "Rcs_body.h"


static Rcs::TaskFactoryRegistrar<Rcs::TaskJoints> registrar("Joints");


/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::TaskJoints::TaskJoints(const std::string& className_,
                            xmlNode* node,
                            RcsGraph* _graph):
  CompositeTask(className_, node, _graph)
{
  // Parse all joints specified in attribute "jnts"
  std::vector<std::string> jntsVec;
  getXMLNodePropertyVecSTLString(node, "jnts", jntsVec);

  // If atribute "jnts" does not exist or contain anything, we look for
  // an "effector" attribute and add all body joints
  if (jntsVec.empty())
  {
    std::string tmp;
    getXMLNodePropertySTLString(node, "effector", tmp);

    if (!tmp.empty())
    {
      const RcsBody* bdy = RcsGraph_getBodyByName(_graph, tmp.c_str());
      RCHECK_MSG(bdy, "Can't find body \"%s\" for jntsVec", tmp.c_str());
      const RcsJoint* jPtr = bdy->jnt;

      while (jPtr)
      {
        jntsVec.push_back(std::string(jPtr->name));
        jPtr = jPtr->next;
      }
    }
  }

  std::vector<std::string> refJntsVec;
  getXMLNodePropertyVecSTLString(node, "refJnts", refJntsVec);

  // If atribute "refJnts" does not exist or contain anything, we look
  // for a "refBdy" attribute and add all body joints
  if (refJntsVec.empty())
  {
    std::string tmp;
    getXMLNodePropertySTLString(node, "refBdy", tmp);

    if (!tmp.empty())
    {
      const RcsBody* bdy = RcsGraph_getBodyByName(_graph, tmp.c_str());
      RCHECK_MSG(bdy, "Can't find body \"%s\" for refJntsVec", tmp.c_str());
      const RcsJoint* jPtr = bdy->jnt;

      while (jPtr)
      {
        refJntsVec.push_back(std::string(jPtr->name));
        jPtr = jPtr->next;
      }
    }
  }

    if (!refJntsVec.empty())
    {
      RCHECK(refJntsVec.size()==jntsVec.size());
    }

    double* refGains = new double[jntsVec.size()];
    VecNd_setElementsTo(refGains, 1.0, jntsVec.size());

    unsigned int nStrings = getXMLNodeNumStrings(node, "refGains");

    if (nStrings==1)
    {
      getXMLNodePropertyDouble(node, "refGains", &refGains[0]);
      VecNd_setElementsTo(refGains, refGains[0], jntsVec.size());
    }
    else if (nStrings==6)
    {
      getXMLNodePropertyVecN(node, "refGains", refGains, nStrings);
    }
    else
    {
      if (nStrings!=0)
      {
        RLOG(1, "Wrong number of refGains: %u, should be 1 or 6", nStrings);
      }
    }

    for (size_t idx = 0; idx < jntsVec.size(); idx++)
    {
      RcsJoint* jnt = RcsGraph_getJointByName(_graph, jntsVec[idx].c_str());
      RCHECK_MSG(jnt, "Not found: %s", jntsVec[idx].c_str());

      RcsJoint* refJnt = NULL;
      if (!refJntsVec.empty())
      {
        refJnt = RcsGraph_getJointByName(_graph, refJntsVec[idx].c_str());
        RCHECK_MSG(refJnt, "Not found: %s", refJntsVec[idx].c_str());
      }

      addTask(new TaskJoint(jnt, refJnt, node, _graph, refGains[idx]));
    }

    delete [] refGains;
}

/*******************************************************************************
 * Copy constructor doing deep copying
 ******************************************************************************/
Rcs::TaskJoints::TaskJoints(const Rcs::TaskJoints& copyFromMe,
                            RcsGraph* newGraph):
  CompositeTask(copyFromMe, newGraph)
{
}

/*******************************************************************************
 * Clone function
 ******************************************************************************/
Rcs::TaskJoints* Rcs::TaskJoints::clone(RcsGraph* newGraph) const
{
  return new Rcs::TaskJoints(*this, newGraph);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::TaskJoints::isValid(xmlNode* node, const RcsGraph* graph)
{
  bool success = true;
  success = Rcs::Task::isValid(node, graph, "Joints") && success;

  // If the attribute "jnts" is given, TaskJoints are only created
  // for the joints listed within the attribute
  std::vector<std::string> jntVec;
  bool hasJnts = getXMLNodePropertyVecSTLString(node, "jnts", jntVec);

  if (hasJnts)
  {
    for (size_t idx = 0; idx < jntVec.size(); idx++)
    {
      RcsJoint* jnt = RcsGraph_getJointByName(graph, jntVec[idx].c_str());
      if (jnt == NULL)
      {
        success = false;
        RLOG(4, "Joint not found: %s", jntVec[idx].c_str());
      }
    }
  }

  std::string efBdy;
  bool hasEffector = getXMLNodePropertySTLString(node, "effector", efBdy);

  if (hasEffector)
  {
    if (hasJnts)
    {
      RLOG(4, "Both effector and jnts attribute given - giving up");
      success = false;
    }
    else
    {
      const RcsBody* bdy = RcsGraph_getBodyByName(graph, efBdy.c_str());

      if (bdy==NULL)
      {
        RLOG(4, "Effector \"%s\" not found in graph", efBdy.c_str());
        success = false;
      }
      else
      {
        const RcsJoint* jPtr = bdy->jnt;

        while (jPtr)
        {
          jntVec.push_back(std::string(jPtr->name));
          jPtr = jPtr->next;
        }
      }
    }

  }   // if (hasEffector)

  // If the attribute "refJnts" is given, TaskJoints are only created
  // for the joints listed within the attribute
  std::vector<std::string> refJntVec;
  bool hasRefJnts = getXMLNodePropertyVecSTLString(node, "refJnts", refJntVec);

  if (hasRefJnts)
  {
    if (refJntVec.size() != jntVec.size())
    {
      success = false;
      RLOG(4, "Different number of entries in tag \"jnts\" and \"refJnts\":"
           "%zu - %zu", jntVec.size(), refJntVec.size());
    }

    for (size_t idx = 0; idx < refJntVec.size(); idx++)
    {
      RcsJoint* jnt = RcsGraph_getJointByName(graph, refJntVec[idx].c_str());
      if (jnt == NULL)
      {
        success = false;
        RLOG(4, "Ref-joint not found: %s", refJntVec[idx].c_str());
      }
    }
  }

  std::string refBdy;
  bool hasRefBdy = getXMLNodePropertySTLString(node, "refBdy", refBdy);

  if (hasRefBdy)
  {
    if (hasRefJnts)
    {
      RLOG(4, "Both refBdy and refJnts attribute given - giving up");
      success = false;
    }
    else
    {
      const RcsBody* bdy = RcsGraph_getBodyByName(graph, refBdy.c_str());

      if (bdy==NULL)
      {
        RLOG(4, "refBdy \"%s\" not found in graph", refBdy.c_str());
        success = false;
      }
      else
      {
        const RcsJoint* jPtr = bdy->jnt;

        while (jPtr)
        {
          refJntVec.push_back(std::string(jPtr->name));
          jPtr = jPtr->next;
        }
      }
    }

  }   // if (hasRefBdy)

  // If the attribute "refGains" is given, the number of entries must match the number of
  // entries of the joints
  unsigned int nStrings = getXMLNodeNumStrings(node, "refGains");

  if ((nStrings!=0) && (nStrings!=1) && (nStrings!=jntVec.size()))
  {
      RLOG_CPP(4, "Wrong number of refGains: " << nStrings
               <<  "  should be 0, 1 or " << jntVec.size());
      success = false;
  }



  return success;
}
