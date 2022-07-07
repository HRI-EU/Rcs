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

#include "TaskCompositeXml.h"
#include "TaskFactory.h"
#include "Rcs_parser.h"
#include "Rcs_stlParser.h"
#include "Rcs_macros.h"


static Rcs::TaskFactoryRegistrar<Rcs::TaskCompositeXml> registrar("Composite");


/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::TaskCompositeXml::TaskCompositeXml(const std::string& className_,
                                        xmlNode* node,
                                        const RcsGraph* _graph):
  CompositeTask(className_, node, _graph)
{
  // Descend one level in XML parsing to find Task et al.
  node = node->children;

  while (node)
  {
    if (isXMLNodeName(node, "Task"))
    {
      Task* ti = TaskFactory::createTask(node, graph);
      addTask(ti);
    }

    node = node->next;
  }

}

/*******************************************************************************
 * Copy constructor doing deep copying
 ******************************************************************************/
Rcs::TaskCompositeXml::TaskCompositeXml(const Rcs::TaskCompositeXml& copyFromMe) :
  CompositeTask(copyFromMe)
{
}

/*******************************************************************************
 * Clone function
 ******************************************************************************/
Rcs::TaskCompositeXml* Rcs::TaskCompositeXml::clone(const RcsGraph* newGraph) const
{
  TaskCompositeXml* task = new Rcs::TaskCompositeXml(*this);
  task->setGraph(newGraph);
  return task;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskCompositeXml::toXML(FILE* out, bool activation) const
{
  Task::toXMLStart(out);
  fprintf(out, " active=\"%s\" >\n", activation ? "true" : "false");

  for (size_t i=0; i<getNumberOfTasks(); ++i)
  {
    fprintf(out, "  ");
    getSubTask(i)->toXML(out, activation);
  }

  fprintf(out, "</Task>\n");
}

/*******************************************************************************
 * We traverse the node's children and try to instantiate each task. If one of
 * them fails, we know that it is invalid, and we can return false.
 ******************************************************************************/
bool Rcs::TaskCompositeXml::isValid(xmlNode* node, const RcsGraph* graph)
{
  bool success = true;

  // Descend one level in XML parsing to find sub-tasks of the composite task.
  node = node->children;

  while (node)
  {
    if (isXMLNodeName(node, "Task"))
    {
      //std::string txt = getXMLNodePropertySTLString(node, "controlVariable");

      // The createTask function performs a validity check of the task. If we
      // get a NULL pointer here, it means that the task is invalid.
      // Task* ti = TaskFactory::createTask(txt, node, (RcsGraph*)graph);
      Task* ti = TaskFactory::createTask(node, (RcsGraph*)graph);

      if (ti == NULL)
      {
        success = false;
      }
      else
      {
        delete ti;
      }
    }

    node = node->next;
  }

  return success;
}
