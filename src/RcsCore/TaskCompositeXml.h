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

#ifndef RCS_TASKCOMPOSITEXML_H
#define RCS_TASKCOMPOSITEXML_H

#include "CompositeTask.h"


namespace Rcs
{
/*! \ingroup RcsTask
 *  \brief This tasks allows to populate a CompositeTask from an xml node. Here
 *         is an example:
 *
 *  \code
 *  <Task name="XZABC Effector" controlVariable="Composite" >
 *    <Task name="c1" controlVariable="X"   effector="Effector" />
 *    <Task name="c2" controlVariable="Z"   effector="Effector" />
 *    <Task name="c3" controlVariable="ABC" effector="Effector" />
 *  </Task>
 *  \endcode
 *
 * This will construct a task that is composed out of the three sub-tasks c1,
 * c2 and c3. The task dimension is 5.
 *
 */
class TaskCompositeXml: public CompositeTask
{
public:

  /*! Constructor based on xml parsing
   */
  TaskCompositeXml(const std::string& className, xmlNode* node,
                   RcsGraph* graph);

  /*! \brief Virtual copy constructor with optional new graph
   */
  TaskCompositeXml* clone(RcsGraph* newGraph=NULL) const;

  /*! \brief Writes the specific task's xml representation to a file
   *         desriptor. Here it is the vector of sub-tasks.
   */
  virtual void toXML(FILE* out, bool activation=true) const;

  /*! \brief Traverses the node's children and tries to instantiate each task.
  *          If none of them fails, the function returns true, otherwise false.
   */
  static bool isValid(xmlNode* node, const RcsGraph* graph);
};

}

#endif // RCS_TASKCOMPOSITEXML_H
