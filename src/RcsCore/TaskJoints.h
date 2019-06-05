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

#ifndef RCS_TASKJOINTS_H
#define RCS_TASKJOINTS_H

#include "CompositeTask.h"


namespace Rcs
{
/*! \ingroup RcsTask
 * \brief This tasks allows to control a set of joints in a single task.
 *
 *  If the tag "jnts" is given in the xml description, the task controls
 *  only the specified joints. Otherwise, the task controls all joints
 *  in the graph.
 */
class TaskJoints: public CompositeTask
{
public:

  /*! Constructor based on xml parsing
   */
  TaskJoints(const std::string& className, xmlNode* node, RcsGraph* graph);

  /*! \brief Copy constructor doing deep copying with optional new graph
   *         pointer
   */
  TaskJoints(const TaskJoints& copyFromMe, RcsGraph* newGraph=NULL);

  /*! \brief Virtual copy constructor with optional new graph
   */
  TaskJoints* clone(RcsGraph* newGraph=NULL) const;

  /*! \brief Returns true if the task is specified correctly, false
   *         otherwise. The following checks are performed:
   *         - XML tag "effector" corresponds to body in graph
   *         - XML tag "controlVariable" is "XYZ"
   */
  static bool isValid(xmlNode* xml_node, const RcsGraph* graph);
};

}

#endif // RCS_TASKJOINTS_H
