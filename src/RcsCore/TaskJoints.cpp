/*******************************************************************************

  Copyright (c) 2017, Honda Research Institute Europe GmbH

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

#include "TaskJoints.h"
#include "TaskJoint.h"
#include "TaskFactory.h"
#include "Rcs_macros.h"
#include "Rcs_typedef.h"
#include "Rcs_parser.h"
#include "Rcs_stlParser.h"
#include "Rcs_VecNd.h"
#include "Rcs_body.h"
#include "Rcs_utils.h"
#include "Rcs_joint.h"


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
    std::string tmp = getXMLNodePropertySTLString(node, "effector");

    if (!tmp.empty())
    {
      const RcsBody* bdy = RcsGraph_getBodyByName(_graph, tmp.c_str());
      RCHECK_MSG(bdy, "Can't find body \"%s\" for jntsVec", tmp.c_str());
      setEffector(bdy);
      const RcsJoint* jPtr = RCSJOINT_BY_ID(_graph, bdy->jntId);

      while (jPtr)
      {
        jntsVec.push_back(std::string(jPtr->name));
        jPtr = RCSJOINT_BY_ID(_graph, jPtr->nextId);
      }
    }
  }

  std::vector<std::string> refJntsVec;
  getXMLNodePropertyVecSTLString(node, "refJnts", refJntsVec);

  // If atribute "refJnts" does not exist or contain anything, we look
  // for a "refBdy" attribute and add all body joints
  bool hasRefBdy = false;
  if (refJntsVec.empty())
  {
    std::string tmp = getXMLNodePropertySTLString(node, "refBdy");

    if (!tmp.empty())
    {
      hasRefBdy = true;
      const RcsBody* bdy = RcsGraph_getBodyByName(_graph, tmp.c_str());
      RCHECK_MSG(bdy, "Can't find body \"%s\" for refJntsVec", tmp.c_str());
      setRefBody(bdy);
      const RcsJoint* jPtr = RCSJOINT_BY_ID(_graph, bdy->jntId);

      while (jPtr)
      {
        refJntsVec.push_back(std::string(jPtr->name));
        jPtr = RCSJOINT_BY_ID(graph, jPtr->nextId);
      }
    }
  }

  if (!refJntsVec.empty())
  {
    RCHECK(refJntsVec.size()==jntsVec.size());
  }

  double* refGains = new double[jntsVec.size()];
  VecNd_setElementsTo(refGains, hasRefBdy ? -1.0 : 1.0, jntsVec.size());

  unsigned int nStrings = getXMLNodeNumStrings(node, "refGains");

  if (nStrings==1)
  {
    getXMLNodePropertyDouble(node, "refGains", &refGains[0]);
    VecNd_setElementsTo(refGains, refGains[0], jntsVec.size());
  }
  else if (nStrings != 0)
  {
    getXMLNodePropertyVecN(node, "refGains", refGains, nStrings);
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

  // Re-initialize parameters
  if (getClassName() == "Joints")
  {
    std::vector<const RcsJoint*> jnts = Rcs::TaskJoints::getJoints();
    double* guiMin = new double[jnts.size()];
    double* guiMax = new double[jnts.size()];

    for (size_t i = 0; i < jnts.size(); ++i)
    {
      double xml2SI = RcsJoint_isTranslation(jnts[i]) ? 1.0 : M_PI / 180.0;
      guiMin[i] = jnts[i]->q_min / xml2SI;
      guiMax[i] = jnts[i]->q_max / xml2SI;
    }

    getXMLNodePropertyVecN(node, "guiMax", guiMax, jnts.size());
    getXMLNodePropertyVecN(node, "guiMin", guiMin, jnts.size());

    for (size_t i = 0; i < jnts.size(); ++i)
    {
      double xml2SI = RcsJoint_isTranslation(jnts[i]) ? 1.0 : M_PI / 180.0;
      guiMin[i] *= xml2SI;
      guiMax[i] *= xml2SI;
    }

    bool hide = false;
    getXMLNodePropertyBoolString(node, "hide", &hide);
    if (hide)
    {
      VecNd_setZero(guiMin, jnts.size());
      VecNd_setZero(guiMax, jnts.size());
    }

    clearParameters();

    for (size_t i = 0; i < jnts.size(); ++i)
    {
      if (RcsJoint_isTranslation(jnts[i]) == true)
      {
        std::string label = std::string(jnts[i]->name) + " [m]";
        addParameter(Parameters(guiMin[i], guiMax[i], 1.0, label));
      }
      else
      {
        std::string label = std::string(jnts[i]->name) + " [deg]";
        addParameter(Parameters(guiMin[i], guiMax[i], 180.0 / M_PI, label));
      }
    }

    delete[] guiMin;
    delete[] guiMax;
  }
}

/*******************************************************************************
 * Explicit construction using rigid body joint links
 ******************************************************************************/
Rcs::TaskJoints::TaskJoints(const RcsBody* effector, RcsGraph* graph_):
  CompositeTask(graph_)
{
  setClassName("Joints");
  setEffector(effector);

  RCSBODY_FOREACH_JOINT(graph_, effector)
  {
    addTask(new TaskJoint(graph, JNT, NULL, 1.0));
  }

}

/*******************************************************************************
 * Explicit construction using rigid body joint links
 ******************************************************************************/
Rcs::TaskJoints::TaskJoints(const RcsBody* effector, const RcsBody* refBdy,
                            RcsGraph* graph_) :
  CompositeTask(graph_)
{
  setClassName("Joints");
  setEffector(effector);
  setRefBody(refBdy);
  const unsigned int nJoints = RcsBody_numJoints(graph, effector);
  RCHECK_MSG(nJoints > 0, "Body \"%s\"", effector ? effector->name : "NULL");

  const RcsJoint* jRef = refBdy ? RCSJOINT_BY_ID(graph_, refBdy->jntId) : NULL;

  RCSBODY_FOREACH_JOINT(graph_, effector)
  {
    addTask(new TaskJoint(graph, JNT, jRef, 1.0));
    if (jRef)
    {
      jRef = RCSJOINT_BY_ID(graph, jRef->nextId);
    }
  }

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
 *
 ******************************************************************************/
void Rcs::TaskJoints::setJoints(std::vector<const RcsJoint*> jnt)
{
  RCHECK(subTask.size() == jnt.size());

  for (size_t i = 0; i < subTask.size(); ++i)
  {
    Rcs::TaskJoint* tsk = dynamic_cast<Rcs::TaskJoint*>(subTask[i]);
    RCHECK(tsk);
    tsk->setJoint(jnt[i]);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskJoints::setRefJoints(std::vector<const RcsJoint*> jnt)
{
  RCHECK(jnt.empty() || (jnt.size()==subTask.size()));

  for (size_t i = 0; i < subTask.size(); ++i)
  {
    TaskJoint* tsk = dynamic_cast<Rcs::TaskJoint*>(subTask[i]);
    RCHECK(tsk);
    tsk->setRefJoint(jnt.empty() ? NULL : jnt[i]);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskJoints::setRefJoint(size_t index, const RcsJoint* jnt)
{
  RCHECK(index < subTask.size());
  Rcs::TaskJoint* tsk = dynamic_cast<Rcs::TaskJoint*>(subTask[index]);
  RCHECK(tsk);
  tsk->setRefJoint(jnt);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskJoints::setRefGains(std::vector<double> gains)
{
  RCHECK(gains.empty() || (gains.size()==subTask.size()));

  for (size_t i = 0; i < subTask.size(); ++i)
  {
    Rcs::TaskJoint* tsk = dynamic_cast<Rcs::TaskJoint*>(subTask[i]);
    RCHECK(tsk);
    tsk->setRefGain(gains.empty() ? 1.0 : gains[i]);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskJoints::setRefGains(double gain)
{
  for (size_t i = 0; i < subTask.size(); ++i)
  {
    Rcs::TaskJoint* tsk = dynamic_cast<Rcs::TaskJoint*>(subTask[i]);
    RCHECK(tsk);
    tsk->setRefGain(gain);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
std::vector<const RcsJoint*> Rcs::TaskJoints::getJoints() const
{
  std::vector<const RcsJoint*> jVec;

  for (size_t i = 0; i < subTask.size(); ++i)
  {
    Rcs::TaskJoint* tsk = dynamic_cast<Rcs::TaskJoint*>(subTask[i]);
    RCHECK(tsk);
    jVec.push_back(tsk->getJoint());
  }

  return jVec;
}

/*******************************************************************************
 *
 ******************************************************************************/
std::vector<const RcsJoint*> Rcs::TaskJoints::getRefJoints() const
{
  std::vector<const RcsJoint*> jVec;

  for (size_t i = 0; i < subTask.size(); ++i)
  {
    Rcs::TaskJoint* tsk = dynamic_cast<Rcs::TaskJoint*>(subTask[i]);
    RCHECK(tsk);
    jVec.push_back(tsk->getRefJoint());
  }

  return jVec;
}

/*******************************************************************************
 *
 ******************************************************************************/
std::vector<double> Rcs::TaskJoints::getRefGains() const
{
  std::vector<double> gains;

  for (size_t i = 0; i < subTask.size(); ++i)
  {
    Rcs::TaskJoint* tsk = dynamic_cast<Rcs::TaskJoint*>(subTask[i]);
    RCHECK(tsk);
    gains.push_back(tsk->getRefGain());
  }

  return gains;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskJoints::toXMLBody(FILE* out) const
{
  size_t nRefJoints = 0, nRefGains = 0;

  fprintf(out, " jnts=\"");

  for (size_t i = 0; i < subTask.size(); ++i)
  {
    Rcs::TaskJoint* tsk = dynamic_cast<Rcs::TaskJoint*>(subTask[i]);
    RCHECK(tsk);
    fprintf(out, "%s", tsk->getJoint()->name);

    if (i != subTask.size() - 1)
    {
      fprintf(out, " ");
    }

    if (tsk->getRefJoint())
    {
      nRefJoints++;
    }
    if (tsk->getRefGain()!=1.0)
    {
      nRefGains++;
    }
  }

  fprintf(out, "\"");

  if (nRefJoints== subTask.size())
  {
    fprintf(out, " refJnts=\"");

    for (size_t i = 0; i < subTask.size(); ++i)
    {
      Rcs::TaskJoint* tsk = dynamic_cast<Rcs::TaskJoint*>(subTask[i]);
      fprintf(out, "%s", tsk->getRefJoint()->name);

      if (i != subTask.size() - 1)
      {
        fprintf(out, " ");
      }
    }

    fprintf(out, "\"");
  }

  if (nRefGains > 0)
  {
    char buf[256];
    fprintf(out, " refGains=\"");

    for (size_t i = 0; i < subTask.size(); ++i)
    {
      Rcs::TaskJoint* tsk = dynamic_cast<Rcs::TaskJoint*>(subTask[i]);
      fprintf(out, "%s", String_fromDouble(buf, tsk->getRefGain(), 6));

      if (i != subTask.size()-1)
      {
        fprintf(out, " ");
      }
    }

    fprintf(out, "\"");
  }

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
        const RcsJoint* jPtr = RCSJOINT_BY_ID(graph, bdy->jntId);

        while (jPtr)
        {
          jntVec.push_back(std::string(jPtr->name));
          jPtr = RCSJOINT_BY_ID(graph, jPtr->nextId);
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
        const RcsJoint* jPtr = RCSJOINT_BY_ID(graph, bdy->jntId);

        while (jPtr)
        {
          refJntVec.push_back(std::string(jPtr->name));
          jPtr = RCSJOINT_BY_ID(graph, jPtr->nextId);
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

  RCHECK(success);

  return success;
}
