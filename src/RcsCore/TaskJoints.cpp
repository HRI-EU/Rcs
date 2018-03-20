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
#include "Rcs_stlParser.h"


static Rcs::TaskFactoryRegistrar<Rcs::TaskJoints> registrar("Joints");


/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::TaskJoints::TaskJoints(const std::string& className_,
                            xmlNode* node,
                            RcsGraph* _graph):
  CompositeTask(className_, node, _graph)
{
  // If the attribute "jnts" is given, TaskJoints are only created
  // for the joints listed within the attribute
  std::vector<std::string> jnts_vec;
  getXMLNodePropertyVecSTLString(node, "jnts", jnts_vec);

  if (jnts_vec.size() > 0)
  {
    for (size_t idx = 0; idx < jnts_vec.size(); idx++)
    {
      RcsJoint* jnt = RcsGraph_getJointByName(_graph, jnts_vec[idx].c_str());
      RCHECK_MSG(jnt, "Not found: %s", jnts_vec[idx].c_str());
      addTask(new TaskJoint(jnt, node, _graph));
    }
  }
  else
  {
    RCSGRAPH_TRAVERSE_JOINTS(getGraph())
    {
      addTask(new TaskJoint(JNT, node, _graph));
    }
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
  getXMLNodePropertyVecSTLString(node, "jnts", jntVec);

  if (jntVec.size() > 0)
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

  return success;
}
