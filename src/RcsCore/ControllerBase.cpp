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

#include "ControllerBase.h"
#include "TaskFactory.h"
#include "TaskJoints.h"
#include "TaskJoint.h"
#include "Rcs_math.h"
#include "Rcs_macros.h"
#include "Rcs_typedef.h"
#include "Rcs_resourcePath.h"
#include "Rcs_graphParser.h"
#include "Rcs_stlParser.h"
#include "Rcs_kinematics.h"
#include "Rcs_dynamics.h"
#include "Rcs_joint.h"

#include <cfloat>


namespace Rcs
{

/*******************************************************************************
 * Construction without tasks and default settings.
 ******************************************************************************/
ControllerBase::ControllerBase() :
  graph(NULL),
  ownsGraph(true),
  cMdl(NULL)
{
}

/*******************************************************************************
 * Constructor based on xml parsing.
 ******************************************************************************/
ControllerBase::ControllerBase(const std::string& xmlDescription) :
  graph(NULL),
  ownsGraph(true),
  cMdl(NULL),
  xmlFile(xmlDescription)
{
  char txt[RCS_MAX_FILENAMELEN];
  bool fileExists = Rcs_getAbsoluteFileName(xmlDescription.c_str(), txt);
  xmlDocPtr xmlDoc = NULL;
  xmlNodePtr xmlRootNode = NULL;

  if (fileExists==true)
  {
    this->xmlFile.assign(txt);
    xmlRootNode = parseXMLFile(txt, "Controller", &xmlDoc);
  }
  else
  {
    const char* fileExtension = strrchr(xmlDescription.c_str(), '.');

    // Check if the xmlDescription is really a file (with extension .xml). If
    // it has the .xml extension, we assume the file couldn't be opened and
    // exit with a fatal error.
    if (fileExtension && STREQ(fileExtension, ".xml"))
    {
      RMSG("Resource path is:");
      Rcs_printResourcePath();
      RFATAL("Controller configuration file \"%s\" not found in "
             "resource path - exiting", xmlDescription.c_str());
    }

    // From here we assume that is is a memory buffer holding the xml
    // file as a string.
    this->xmlFile.assign("Created_from_memory_buffer");
    xmlRootNode = parseXMLMemory(xmlDescription.c_str(),
                                 xmlDescription.size(),
                                 &xmlDoc);
  }

  // Check if the xml description is malformed
  if (xmlRootNode == NULL)
  {
    RFATAL("Node \"Controller\" not found in \"%s\"", this->xmlFile.c_str());
  }

  bool success = initFromXmlNode(xmlRootNode);
  if (!success)
  {
    RLOG(1, "Found errors in graph for controller");
  }

  // Free the xml memory in case no child ctors need to continue parsing
  if (xmlDoc)
  {
    xmlFreeDoc(xmlDoc);
  }

}

/*******************************************************************************
 * Initialization from xml node "Controller".
 ******************************************************************************/
bool ControllerBase::initFromXmlNode(xmlNodePtr xmlNodeController)
{
  // Create the graph. If no tag "graph" exists, the graph still can
  // be included by an xinclude directive. Therefore it's not fatal
  // if we don't find it here.
  std::string txt;
  if (getXMLNodePropertySTLString(xmlNodeController, "graph", txt))
  {
    this->graph = RcsGraph_create(txt.c_str());
    RCHECK_MSG(this->graph, "Failed to create graph \"%s\"", txt.c_str());
  }

  // Descend one level in XML parsing to find Task et al.
  xmlNodePtr node = xmlNodeController->children;

  while (node)
  {
    if (isXMLNodeName(node, "Task"))
    {
      // Create the new task, and add it to the task list
      if (getXMLNodePropertySTLString(node, "controlVariable", txt))
      {
        add(TaskFactory::createTask(node, graph));
      }
      else
      {
        RLOG(1, "No controlVariable attribute found in task description");
      }

    }
    else if (isXMLNodeName(node, "CollisionModel"))
    {
      this->cMdl = RcsCollisionModel_createFromXML(getGraph(), node);

      if (this->cMdl != NULL)
      {
        RcsCollisionModel_compute(this->cMdl);
      }
      else
      {
        RLOG(1, "Failed to create collision model from xml");
      }
    }
    else if (isXMLNodeName(node, "Graph"))
    {
      RCHECK_MSG(this->graph==NULL, "Found xml tag <Graph>, but already graph"
                 " loaded graph from file \"%s\"", graph->cfgFile);
      this->graph = RcsGraph_createFromXmlNode(node);
      strcpy(graph->cfgFile, xmlFile.c_str());
      RCHECK_MSG(this->graph, "Failed to create inlined graph");
    }

    node = node->next;
  }

  // Check for consistency
  int graphErrors=0, graphWarnings=0;

  bool success = RcsGraph_check(this->graph, &graphErrors, &graphWarnings);

  if (!success)
  {
    if (graphErrors>0)
    {
      RLOG(1, "Check for graph failed: %d errors", graphErrors);
      return false;
    }

    if (graphWarnings>0)
    {
      RLOG(1, "Found %d warnings in graph", graphWarnings);
    }
  }

  return true;
}

/*******************************************************************************
 * Constructor based on a graph object. Takes ownership of the graph.
 ******************************************************************************/
ControllerBase::ControllerBase(RcsGraph* graph_):
  graph(graph_), ownsGraph(true), cMdl(NULL)
{
}

/*******************************************************************************
 * Copy constructor doing deep copying.
 * The xml data is not copied, since this is not present after the
 * construction of the copied constructor.
 ******************************************************************************/
ControllerBase::ControllerBase(const ControllerBase& copyFromMe):
  graph(RcsGraph_clone(copyFromMe.graph)),
  ownsGraph(true),
  cMdl(RcsCollisionModel_clone(copyFromMe.cMdl, this->graph)),
  xmlFile(copyFromMe.xmlFile),
  taskArrayIdx(copyFromMe.taskArrayIdx)
{
  for (std::vector<Task*>::const_iterator itr = copyFromMe.tasks.begin();
       itr != copyFromMe.tasks.end(); ++itr)
  {
    this->tasks.push_back((*itr)->clone(this->graph));
  }

}

/*******************************************************************************
 * Assignment operator. Could be done a bit more nice if the tasks had an
 * assignment operator as well. Currently, we just replace them.
 ******************************************************************************/
ControllerBase& ControllerBase::operator= (const ControllerBase& copyFromMe)
{
  // check for self-assignment by comparing the address of the
  // implicit object and the parameter
  if (this == &copyFromMe)
  {
    return *this;
  }

  // Clean up memory
  if (this->cMdl != NULL)
  {
    RcsCollisionModel_destroy(this->cMdl);
  }

  for (size_t i = 0; i < this->tasks.size(); i++)
  {
    delete (this->tasks[i]);
  }

  this->tasks.clear();

  RcsGraph_destroy(this->graph);

  // do the copy
  this->graph = RcsGraph_clone(copyFromMe.graph);
  this->ownsGraph = true;

  // clone tasks (index lists are cloned above)
  for (std::vector<Task*>::const_iterator itr = copyFromMe.tasks.begin();
       itr != copyFromMe.tasks.end(); ++itr)
  {
    this->tasks.push_back((*itr)->clone(this->graph));
  }

  // Clone the collision model. If it is NULL, the clone function returns NULL.
  this->cMdl = RcsCollisionModel_clone(copyFromMe.cMdl, this->graph);

  // return the existing object
  return *this;
}

/*******************************************************************************
 * Destructor
 ******************************************************************************/
ControllerBase::~ControllerBase()
{
  // Delete collision model. The function accepts a NULL pointer.
  RcsCollisionModel_destroy(this->cMdl);

  for (size_t i = 0; i < this->tasks.size(); i++)
  {
    delete (this->tasks[i]);
  }

  if (this->ownsGraph == true)
  {
    RcsGraph_destroy(this->graph);
  }
}

/*******************************************************************************
 * Returns the name of the controller's xml file.
 ******************************************************************************/
const std::string& ControllerBase::getXmlFileName() const
{
  return this->xmlFile;
}

/*******************************************************************************
 * Returns the name of a task with the given ID.
 ******************************************************************************/
const std::string& ControllerBase::getTaskName(size_t id) const
{
  RCHECK_MSG(id<this->tasks.size(), "id=%zu size=%zu", id, this->tasks.size());
  return this->tasks[id]->getName();
}

/*******************************************************************************
 * Returns the full dimension of a task with the given ID.
 ******************************************************************************/
size_t ControllerBase::getTaskDim() const
{
  size_t dim = 0;

  for (size_t id = 0; id < this->tasks.size(); id++)
  {
    dim += tasks[id]->getDim();
  }
  return dim;
}

/*******************************************************************************
* Returns the full dimension of a task with the given ID.
******************************************************************************/
size_t ControllerBase::getTaskDim(size_t id) const
{
  RCHECK(id < this->tasks.size());
  return this->tasks[id]->getDim();
}

/*******************************************************************************
 * Returns the dimension of all active tasks.
 ******************************************************************************/
size_t ControllerBase::getActiveTaskDim(const MatNd* activation) const
{
  size_t dim = 0;
  RCHECK_MSG(activation->m == this->tasks.size(), "%d != %zu",
             activation->m, this->tasks.size());

  for (size_t id = 0; id < this->tasks.size(); id++)
  {
    if (activation->ele[id] > 0.0)
    {
      dim += getTaskDim(id);
    }
  }

  return dim;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
int ControllerBase::getTaskIndex(const char* name) const
{
  if (name == NULL)
  {
    return -1;
  }

  for (int id = 0; id < (int) this->tasks.size(); id++)
  {
    if (STREQ(tasks[id]->getName().c_str(), name) == true)
    {
      return id;
    }
  }

  return -1;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
int ControllerBase::getTaskIndex(const Task* task) const
{
  if (task == NULL)
  {
    return -1;
  }

  for (size_t id = 0; id < this->tasks.size(); ++id)
  {
    if (task == this->tasks[id])
    {
      return id;
    }
  }

  return -1;
}

/*******************************************************************************
 * Returns the task's index to its entries in x_curr, x_des, and x_dot_des
 * vectors.
 ******************************************************************************/
size_t ControllerBase::getTaskArrayIndex(size_t id) const
{
  RCHECK_MSG(id < this->tasks.size(), "id: %zu   size: %zu",
             id, this->tasks.size());
  return this->taskArrayIdx[id];
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
int ControllerBase::getTaskArrayIndex(const char* name) const
{
  if (name == NULL)
  {
    return -1;
  }

  for (size_t id = 0; id < this->tasks.size(); id++)
  {
    if (tasks[id]->getName()==name)
    {
      return getTaskArrayIndex(id);
    }
  }

  RLOG(4, "No task with name \"%s\" found!", name);

  return -1;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
int ControllerBase::getTaskArrayIndex(const Task* task) const
{
  if (task == NULL)
  {
    return -1;
  }

  for (size_t id = 0; id < this->tasks.size(); id++)
  {
    if (tasks[id]==task)
    {
      return getTaskArrayIndex(id);
    }
  }

  RLOG(4, "No task \"%s\" found!", task->getName().c_str());

  return -1;
}

/*******************************************************************************
 * Returns a const pointer to the task with the given id.
 ******************************************************************************/
const Task* ControllerBase::getTask(size_t id) const
{
  RCHECK(id < this->tasks.size());
  return this->tasks[id];
}

/*******************************************************************************
 * Returns a pointer to the task with the given id.
 ******************************************************************************/
Task* ControllerBase::getTask(size_t id)
{
  RCHECK(id < this->tasks.size());
  return this->tasks[id];
}

/*******************************************************************************
 * Returns a const pointer to the task with the given name.
 ******************************************************************************/
const Task* ControllerBase::getTask(const std::string& name) const
{
  for (size_t i = 0; i < this->tasks.size(); i++)
  {
    const Task* t = this->tasks[i];

    if (t->getName() == name)
    {
      return t;
    }
  }

  return NULL;
}

/*******************************************************************************
 * Returns a pointer to the task with the given name.
 ******************************************************************************/
Task* ControllerBase::getTask(const std::string& name)
{
  for (size_t i = 0; i < this->tasks.size(); i++)
  {
    Task* t = this->tasks[i];

    if (t->getName() == name)
    {
      return t;
    }
  }

  return NULL;
}

/*******************************************************************************
 * Return the number of registered tasks (this->tasks.size())
 ******************************************************************************/
size_t ControllerBase::getNumberOfTasks() const
{
  return this->tasks.size();
}

/*******************************************************************************
 *
 ******************************************************************************/
std::vector<Task*> ControllerBase::getTasks(const MatNd* a) const
{
  std::vector<Task*> tVec;

  for (size_t i = 0; i < this->tasks.size(); i++)
  {
    if ((a==NULL) || (MatNd_get(a, i, 0) > 0.0))
    {
      tVec.push_back(this->tasks[i]);
    }
  }

  return tVec;
}

/*******************************************************************************
 * Returns the type of a task with the given ID.
 ******************************************************************************/
std::string ControllerBase::getTaskType(size_t id) const
{
  RCHECK(id < this->tasks.size());
  return this->tasks[id]->getClassName();
}

/*******************************************************************************
 * Return the controller graph.
 ******************************************************************************/
RcsGraph* ControllerBase::getGraph() const
{
  return this->graph;
}

/*******************************************************************************
 * Return the graph file name (e.g., gScenario.xml).
 ******************************************************************************/
std::string ControllerBase::getGraphFileName() const
{
  return std::string(graph->cfgFile);
}

/*******************************************************************************
 * Return the collision model.
 ******************************************************************************/
RcsCollisionMdl* ControllerBase::getCollisionMdl() const
{
  return this->cMdl;
}

/*******************************************************************************
 * Reads the activation vector from the configuration file. TODO: Check if
 * ctor xml file is open and use this in that case.
 ******************************************************************************/
void ControllerBase::readActivationsFromXML(MatNd* a_init) const
{
  unsigned int nTasks = getNumberOfTasks(), taskCount = 0;

  MatNd_reshapeAndSetZero(a_init, nTasks, 1);

  xmlDocPtr docPtr;
  xmlNodePtr node = parseXMLFile(this->xmlFile.c_str(), "Controller", &docPtr);
  RCHECK_MSG(node, "Node \"Controller\" not found in \"%s\"",
             this->xmlFile.c_str());

  // Descend one level in XML parsing to find Tasks
  node = node->children;

  while (node)
  {
    if (TaskFactory::isValid(node, this->graph) == true)
    {
      bool isActive = false;

      getXMLNodePropertyBoolString(node, "active", &isActive);

      if (isActive==true)
      {
        MatNd_set(a_init, taskCount, 0, 1.0);
      }

      // If a tag "activation" exists, overwrite the value
      getXMLNodePropertyDouble(node, "activation", &a_init->ele[taskCount]);

      taskCount++;

    }   // if (TaskUtils::isValid)

    node = node->next;
  }

  // Free the xml memory in case no child ctors need to continue parsing
  xmlFreeDoc(docPtr);

  if (taskCount != nTasks)
  {
    RLOG(1, "Mismatch in initialization of activations: "
         "Parsed %d tasks, should be %d", taskCount, nTasks);
  }
}

/*******************************************************************************
 * Reads the activation vector from the configuration file.
 ******************************************************************************/
void ControllerBase::readActivationVectorFromXML(MatNd* taskVec,
                                                 const char* tag) const
{
  unsigned int nTasks = getNumberOfTasks(), taskCount = 0;

  MatNd_reshape(taskVec, nTasks, 1);

  xmlDocPtr docPtr;
  xmlNodePtr node = parseXMLFile(this->xmlFile.c_str(), "Controller", &docPtr);
  RCHECK_MSG(node, "Node \"Controller\" not found in \"%s\"",
             this->xmlFile.c_str());

  // Descend one level in XML parsing to find Tasks
  node = node->children;

  while (node)
  {
    if (TaskFactory::isValid(node, this->graph) == true)
    {
      double value;

      if (getXMLNodePropertyDouble(node, tag, &value))
      {
        MatNd_set(taskVec, taskCount, 0, value);
      }

      taskCount++;
    }

    node = node->next;
  }

  // Free the xml memory in case no child ctors need to continue parsing
  xmlFreeDoc(docPtr);

  RCHECK_MSG(taskCount==nTasks, "Mismatch in initialization of activations: "
             "Parsed %d tasks, should be %d", taskCount, nTasks);
}

/*******************************************************************************
 * Reads the activation vector from the configuration file.
 ******************************************************************************/
void ControllerBase::readTaskVectorFromXML(MatNd* x,
                                           const char* tag) const
{
  xmlDocPtr docPtr;
  xmlNodePtr node = parseXMLFile(this->xmlFile.c_str(), "Controller", &docPtr);
  RCHECK_MSG(node, "Node \"Controller\" not found in \"%s\"",
             this->xmlFile.c_str());

  // We don't reset the x-vector, only the elements specified in the config
  // file are overwritten
  MatNd_reshape(x, getTaskDim(), 1);

  // Descend one level in XML parsing to find Tasks
  node = node->children;

  while (node != NULL)
  {

    if (TaskFactory::isValid(node, this->graph) == true)
    {
      xmlChar* taskName = xmlGetProp(node, (const xmlChar*) "name");
      int taskIndex      = getTaskIndex((const char*) taskName);
      RCHECK(taskIndex != -1);
      int taskArrayIndex = getTaskArrayIndex(taskIndex);
      getXMLNodePropertyVecN(node, tag, &x->ele[taskArrayIndex],
                             getTaskDim(taskIndex));
      xmlFree(taskName);
    }

    node = node->next;
  }

  xmlFreeDoc(docPtr);
}

/*******************************************************************************
 * Returns the vector of tasks.
 ******************************************************************************/
const std::vector<Task*>& ControllerBase::taskVec() const
{
  return this->tasks;
}

/*******************************************************************************
 * Returns the vector of tasks.
 ******************************************************************************/
std::vector<Task*>& ControllerBase::taskVec()
{
  return this->tasks;
}

/*******************************************************************************
 * Task Jacobian.
 ******************************************************************************/
void ControllerBase::computeJ(MatNd* J, const MatNd* a_des) const
{
  unsigned int dimTask, nq = this->graph->nJ, nRows = 0;

  for (size_t i = 0; i < this->tasks.size(); i++)
  {
    if ((a_des==NULL) || (MatNd_get(a_des, i, 0)>0.0))
    {
      // Check that appending the current task Jacobian will not write
      // into non-existing memory
      dimTask = tasks[i]->getDim();
      RCHECK_MSG(J->size >= (nRows + dimTask)*nq, "While adding task %zd: "
                 "\"%s\": size of J: %d   m: %d   dimTask: %d   n: %d",
                 i, getTaskName(i).c_str(), J->size, nRows, dimTask, nq);

      // Create a local Jacobian that points to the augmented row and
      // append the task's Jacobian to J.
      MatNd J_task = MatNd_fromPtr(dimTask, nq, &J->ele[nRows*nq]);
      tasks[i]->computeJ(&J_task);
      nRows += dimTask;
    }

  }

  // Reshape the Jacobian. If we do it within the for loop, it might happen that
  // other threads check for the intermediate sizes. Therefore, we just set it
  // to the resulting size here, which has no effect if the size of J didn't
  // change.
  J->m = nRows;
  J->n = nq;
}

/*******************************************************************************
 * Task Hessian.
 ******************************************************************************/
void ControllerBase::computeH(MatNd* H, const MatNd* a_des) const
{
  unsigned int nx, nq = this->graph->nJ;

  MatNd_reshape(H, 0, nq);

  for (size_t i=0; i<this->tasks.size(); i++)
  {
    if ((a_des==NULL) || (MatNd_get(a_des, i, 0)>0.0))
    {
      nx = tasks[i]->getDim();
      MatNd Hi = MatNd_fromPtr(nq, nx*nq, &H->ele[H->m*nq*nq]);
      MatNd_reshape(H, H->m+nx, nq*nq); // This makes sure the size is ok
      tasks[i]->computeH(&Hi);
    }

  }

}

/*******************************************************************************
 * Task Jacobian derivative times q_dot.
 ******************************************************************************/
void ControllerBase::computeJdotQdot(MatNd* JdotQdot,
                                     const MatNd* a_des) const
{
  MatNd_reshape(JdotQdot, 0, 1);

  for (size_t i = 0; i < this->tasks.size(); i++)
  {
    // we only calculate the dot Jacobian if the task is active
    if ((a_des==NULL) || (MatNd_get(a_des, i, 0)>0.0))
    {
      // Check that appending will not write into non-existing memory
      int dimTask = this->tasks[i]->getDim();
      RCHECK_MSG(JdotQdot->size >= JdotQdot->m+dimTask, "While adding task "
                 "\"%s\": size of JdotQdot: %d   m: %d   dimTask: %d",
                 getTaskName(i).c_str(), JdotQdot->size, JdotQdot->m,
                 dimTask);

      // Create a local array that points to the augmented row
      MatNd JdotQdot_i =
        MatNd_fromPtr(dimTask, 1, &JdotQdot->ele[JdotQdot->m]);
      this->tasks[i]->computeJdotQdot(&JdotQdot_i);

      // Reshape the array dimension to include the appended elements
      JdotQdot->m += dimTask;
    }

  }

}

/*******************************************************************************
 * Computes the task vector.
 ******************************************************************************/
void ControllerBase::computeX(MatNd* x, const MatNd* a_des) const
{
  unsigned int dimTask, nRows = 0;

  for (size_t i = 0; i < this->tasks.size(); i++)
  {
    if ((a_des==NULL) || (MatNd_get2(a_des, i, 0)>0.0))
    {
      // Check that appending the current task vector will not write
      // into non-existing memory
      dimTask = tasks[i]->getDim();
      RCHECK_MSG(x->size >= nRows + dimTask, "While adding task "
                 "\"%s\": size of x: %d   m: %d   dimTask: %d",
                 getTaskName(i).c_str(), x->size, nRows, dimTask);

      double* x_i = &x->ele[nRows];
      tasks[i]->computeX(x_i);
      nRows += dimTask;
    }
  }

  // Reshape. See computeJ() for the reason why to do it here.
  x->m = nRows;
  x->n = 1;
}

/*******************************************************************************
 * Delta x for differential kinematics.
 ******************************************************************************/
void ControllerBase::computeDX(MatNd* dx,
                               const MatNd* x_des,
                               const MatNd* a_des) const
{
  unsigned int dimTask, nRowsActive = 0, nRowsAll = 0;
  const double* x_des_i;

  for (size_t i = 0; i < this->tasks.size(); ++i)
  {
    dimTask = tasks[i]->getDim();

    if ((a_des==NULL) || (MatNd_get2(a_des, i, 0)>0.0))
    {
      // Check that appending the current task's dx vector will not write
      // into non-existing memory
      RCHECK_MSG(dx->size >= nRowsActive + dimTask, "While adding task \"%s\":"
                 " size of x: %d   m: %d   dimTask: %d",
                 getTaskName(i).c_str(), dx->size, nRowsActive, dimTask);
      double* dx_i = &dx->ele[nRowsActive];

      // Check that access to x_des is not out of memory bounds
      RCHECK_MSG(x_des->size >= nRowsAll + dimTask, "While adding"
                 " task \"%s\": size of x_des: %u  arrayIdx: %u  dimTask: %d",
                 getTaskName(i).c_str(), x_des->size, nRowsAll, dimTask);
      x_des_i = &x_des->ele[nRowsAll];

      tasks[i]->computeDX(dx_i, x_des_i);

      if (a_des)
      {
        VecNd_constMulSelf(dx_i, MatNd_get(a_des, i, 0), dimTask);
      }

      nRowsActive += dimTask;
    }

    nRowsAll += dimTask;
  }

  // Reshape. See computeJ() for the reason why to do it here.
  dx->m = nRowsActive;
  dx->n = 1;
}

/*******************************************************************************
 *
 ******************************************************************************/
void ControllerBase::computeAx(MatNd* ax,
                               const MatNd* a_des,
                               const MatNd* x_des,
                               const MatNd* xp_des,
                               const MatNd* xpp_des,
                               const MatNd* kp,
                               const MatNd* kd) const
{
  unsigned int dimTask, nRows = 0;

  RCHECK(xp_des);

  for (size_t i = 0; i < this->tasks.size(); i++)
  {
    dimTask = tasks[i]->getDim();
    size_t ti = getTaskArrayIndex(i);   // Array index of the task

    if ((a_des==NULL) || (MatNd_get2(a_des, i, 0)>0.0))
    {
      const double* xpp_des_i = xpp_des ? &xpp_des->ele[ti] : NULL;

      this->tasks[i]->computeAX(&ax->ele[nRows],
                                NULL,
                                &x_des->ele[ti],
                                &xp_des->ele[ti],
                                xpp_des_i,
                                NULL,
                                a_des ? a_des->ele[i] : 1.0,
                                kp->ele[i],
                                kd->ele[i],
                                0.0);

      nRows += dimTask;
    }

  }   // for(i = 0; i < this->tasks.size(); i++)

  // Reshape. See computeJ() for the reason why to do it here.
  ax->m = nRows;
  ax->n = 1;
}

/*******************************************************************************
 * Computes the task velocity vector.
 ******************************************************************************/
void ControllerBase::computeXp(MatNd* x_dot, const MatNd* a_des) const
{
  unsigned int dimTask, nRows = 0;

  for (size_t i = 0; i < this->tasks.size(); i++)
  {
    if ((a_des==NULL) || (MatNd_get2(a_des, i, 0)>0.0))
    {
      // Check that appending the current task velocity vector will not write
      // into non-existing memory
      dimTask = tasks[i]->getDim();
      RCHECK_MSG(x_dot->size >= nRows + dimTask, "While adding task "
                 "\"%s\": size of x_dot: %d   m: %d   dimTask: %d",
                 getTaskName(i).c_str(), x_dot->size, nRows, dimTask);

      double* x_dot_i = &x_dot->ele[nRows];
      tasks[i]->computeXp(x_dot_i);
      nRows += dimTask;
    }
  }

  // Reshape. See computeJ() for the reason why to do it here.
  x_dot->m = nRows;
  x_dot->n = 1;
}

/*******************************************************************************
 * Computes the task velocity vector.
 ******************************************************************************/
void ControllerBase::computeXp_ik(MatNd* x_dot, const MatNd* a_des) const
{
  unsigned int dimTask, nRows = 0;

  for (size_t i = 0; i < this->tasks.size(); i++)
  {
    if ((a_des==NULL) || (MatNd_get2(a_des, i, 0)>0.0))
    {
      // Check that appending the current task velocity vector will not write
      // into non-existing memory
      dimTask = tasks[i]->getDim();
      RCHECK_MSG(x_dot->size >= nRows + dimTask, "While adding task "
                 "\"%s\": size of x_dot: %d   m: %d   dimTask: %d",
                 getTaskName(i).c_str(), x_dot->size, nRows, dimTask);

      double* x_dot_i = &x_dot->ele[nRows];
      tasks[i]->computeXp_ik(x_dot_i);
      nRows += dimTask;
    }
  }

  // Reshape. See computeJ() for the reason why to do it here.
  x_dot->m = nRows;
  x_dot->n = 1;
}

/*******************************************************************************
 * Computes the task velocity vector.
 ******************************************************************************/
void ControllerBase::integrateXp_ik(MatNd* x_res,
                                    const MatNd* x,
                                    const MatNd* x_dot,
                                    double dt,
                                    const MatNd* a_des) const
{
  unsigned int dimTask, nRows = 0;

  for (size_t i = 0; i < this->tasks.size(); i++)
  {
    if ((a_des==NULL) || (MatNd_get2(a_des, i, 0)>0.0))
    {
      // Check that appending the current task velocity vector will not
      // write into non-existing memory
      dimTask = tasks[i]->getDim();
      RCHECK_MSG(x_res->size >= nRows + dimTask, "While adding task "
                 "\"%s\": size of x_dot: %d   m: %d   dimTask: %d",
                 getTaskName(i).c_str(), x_res->size, nRows, dimTask);

      const double* x_dot_i = &x_dot->ele[this->taskArrayIdx[i]];
      const double* x_i = &x->ele[this->taskArrayIdx[i]];

      if (VecNd_sqrLength(x_dot_i, dimTask)>0.0)
      {
        tasks[i]->integrateXp_ik(&x_res->ele[nRows], x_i, x_dot_i, dt);
      }
      else
      {
        VecNd_copy(&x_res->ele[nRows], x_i, dimTask);
      }
      nRows += dimTask;
    }
  }

  // Reshape. See computeJ() for the reason why to do it here.
  x_res->m = nRows;
  x_res->n = 1;
}

/*******************************************************************************
 * Delta x for differential kinematics.
 ******************************************************************************/
void ControllerBase::computeDXp(MatNd* dx_dot,
                                const MatNd* x_dot_des_,
                                const MatNd* a_des) const
{
  unsigned int nRows = 0;
  const MatNd* x_dot_des = x_dot_des_;
  MatNd* x_dot_des_buf = NULL;

  if (x_dot_des_ == NULL)
  {
    MatNd_create2(x_dot_des_buf, getTaskDim(), 1);
    x_dot_des = x_dot_des_buf;
  }

  for (size_t i = 0; i < this->tasks.size(); i++)
  {
    if ((a_des==NULL) || (MatNd_get2(a_des, i, 0)>0.0))
    {
      // Check that appending the current dx vector will not write
      // into non-existing memory
      const unsigned int dimTask = tasks[i]->getDim();
      RCHECK_MSG(dx_dot->size >= nRows + dimTask, "While adding task "
                 "\"%s\": size of dx_dot: %d   m: %d   dimTask: %d",
                 getTaskName(i).c_str(), dx_dot->size, nRows, dimTask);

      const double* x_dot_des_i = &x_dot_des->ele[this->taskArrayIdx[i]];
      double* dx_dot_i = &dx_dot->ele[nRows];
      tasks[i]->computeDXp(dx_dot_i, x_dot_des_i);
      nRows += dimTask;
    }
  }

  // Reshape. See computeJ() for the reason why to do it here.
  dx_dot->m = nRows;
  dx_dot->n = 1;

  MatNd_destroy(x_dot_des_buf);
}

/*******************************************************************************
 * Projects task-space accelerations into the Jacobian coordinates.
 ******************************************************************************/
void ControllerBase::computeFfXpp(MatNd* x_ddot_ik,
                                  const MatNd* x_ddot_,
                                  const MatNd* a_des) const
{
  unsigned int dimTask, nRows = 0;
  const MatNd* x_ddot = x_ddot_;
  MatNd* x_ddot_buf = NULL;

  if (x_ddot_ == NULL)
  {
    MatNd_create2(x_ddot_buf, getTaskDim(), 1);
    x_ddot = x_ddot_buf;
  }

  for (size_t i = 0; i < this->tasks.size(); i++)
  {
    if ((a_des==NULL) || (MatNd_get2(a_des, i, 0)>0.0))
    {
      // Check that appending the current dx vector will not write
      // into non-existing memory
      dimTask = tasks[i]->getDim();
      RCHECK_MSG(x_ddot_ik->size >= nRows + dimTask, "While adding task "
                 "\"%s\": size of dx_dot: %d   m: %d   dimTask: %d",
                 getTaskName(i).c_str(), x_ddot_ik->size, nRows, dimTask);

      const double* x_ddot_i = &x_ddot->ele[this->taskArrayIdx[i]];
      double* x_ddot_ik_i = &x_ddot_ik->ele[nRows];
      tasks[i]->computeFfXpp(x_ddot_ik_i, x_ddot_i);
      nRows += dimTask;
    }
  }

  // Reshape. See computeJ() for the reason why to do it here.
  x_ddot_ik->m = nRows;
  x_ddot_ik->n = 1;

  MatNd_destroy(x_ddot_buf);
}

/*******************************************************************************
 * Computes the task acceleration vector.
 ******************************************************************************/
void ControllerBase::computeXpp(MatNd* x_ddot,
                                const MatNd* q_ddot,
                                const MatNd* a_des) const
{
  unsigned int dimTask, nRows = 0;

  for (size_t i = 0; i < this->tasks.size(); i++)
  {
    if ((a_des==NULL) || (MatNd_get2(a_des, i, 0)>0.0))
    {
      // Check that appending the current task acceleration vector will not
      // write into non-existing memory
      dimTask = tasks[i]->getDim();
      RCHECK_MSG(x_ddot->size >= nRows + dimTask, "While adding task "
                 "\"%s\": size of x_ddot: %d   m: %d   dimTask: %d",
                 getTaskName(i).c_str(), x_ddot->size, nRows, dimTask);

      double* x_ddot_i = &x_ddot->ele[nRows];
      tasks[i]->computeXpp(x_ddot_i, q_ddot);
      nRows += dimTask;
    }
  }

  // Reshape. See computeJ() for the reason why to do it here.
  x_ddot->m = nRows;
  x_ddot->n = 1;
}


/*******************************************************************************
 * Compresses x to xc holding only elements that correspond to active tasks.
 ******************************************************************************/
void ControllerBase::compressToActive(MatNd* xc,
                                      const MatNd* x,
                                      const MatNd* activations) const
{
  unsigned int dimTask, nRows = 0;
  RCHECK(x);

  for (size_t i = 0; i < this->tasks.size(); i++)
  {
    if (MatNd_get(activations, i, 0) > 0.0)
    {
      // Check that appending will not write into non-existing memory
      dimTask = tasks[i]->getDim();
      RCHECK_MSG(xc->size >= nRows + dimTask, "While adding task "
                 "\"%s\": size of xc: %d   m: %d   dimTask: %d",
                 getTaskName(i).c_str(), xc->size, nRows, dimTask);

      VecNd_copy(&xc->ele[nRows], &x->ele[this->taskArrayIdx[i]], dimTask);

      // Reshape the array to include the appended elements
      nRows += dimTask;
    }

  }

  // Reshape. See computeJ() for the reason why to do it here.
  xc->m = nRows;
  xc->n = 1;
}

/*******************************************************************************
 * In-place version of compressToActive().
 ******************************************************************************/
void ControllerBase::compressToActiveSelf(MatNd* x,
                                          const MatNd* activations) const
{
  MatNd* xc = NULL;
  MatNd_create2(xc, getTaskDim(), 1);
  compressToActive(xc, x, activations);
  MatNd_reshape(x, xc->m, xc->n);
  MatNd_copy(x, xc);
  MatNd_destroy(xc);
}

/*******************************************************************************
 * Decompresses xc to x setting elements that don't correspond to active
 * tasks to 0.
 ******************************************************************************/
void ControllerBase::decompressFromActive(MatNd* x, const MatNd* xc,
                                          const MatNd* activations) const
{
  unsigned int dimTask, xc_row = 0, x_row = 0;

  MatNd_reshapeAndSetZero(x, getTaskDim(), 1);

  for (size_t i = 0; i < this->tasks.size(); ++i)
  {
    dimTask = tasks[i]->getDim();

    if (MatNd_get(activations, i, 0) > 0.0)
    {
      VecNd_copy(&x->ele[x_row], &xc->ele[xc_row], dimTask);
      xc_row += dimTask;
    }

    x_row += dimTask;
  }

}

/*******************************************************************************
 * In-place version of above.
 ******************************************************************************/
void ControllerBase::decompressFromActiveSelf(MatNd* xc,
                                              const MatNd* activations) const
{
  MatNd* x = NULL;
  MatNd_create2(x, getTaskDim(), 1);
  decompressFromActive(x, xc, activations);
  MatNd_reshape(xc, x->m, x->n);
  MatNd_copy(xc, x);
  MatNd_destroy(x);
}

/*******************************************************************************
 *
 ******************************************************************************/
void ControllerBase::decompressActivationToTask(MatNd* x,
                                                const MatNd* a) const
{
  RCHECK(a->m == getNumberOfTasks());
  MatNd_reshape(x, getTaskDim(), 1);

  size_t rowIdx = 0;

  for (size_t i = 0; i < getNumberOfTasks(); i++)
  {
    for (size_t j = 0; j < getTaskDim(i); j++)
    {
      x->ele[rowIdx] = a->ele[i];
      rowIdx++;
    }
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
void ControllerBase::decompressActivationToTask(MatNd* x) const
{
  MatNd* a = MatNd_clone(x);
  MatNd_reshape(x, getTaskDim(), 1);
  decompressActivationToTask(x, a);
  MatNd_destroy(a);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
double ControllerBase::computeJointlimitCost() const
{
  return RcsGraph_jointLimitCost(this->graph, RcsStateIK);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void ControllerBase::computeJointlimitGradient(MatNd* grad) const
{
  RcsGraph_jointLimitGradient(this->graph, grad, RcsStateIK);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
double ControllerBase::computeJointlimitBorderCost(double borderRatio) const
{
  return RcsGraph_jointLimitBorderCost(this->graph, borderRatio, RcsStateIK);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void ControllerBase::computeJointlimitBorderGradient(MatNd* grad,
                                                     double borderRatio) const
{
  RcsGraph_jointLimitBorderGradient(this->graph, grad, borderRatio, RcsStateIK);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void ControllerBase::computeCollisionModel()
{
  RcsCollisionModel_compute(this->cMdl);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
double ControllerBase::computeCollisionCost()
{
  computeCollisionModel();
  return RcsCollisionMdl_cost(this->cMdl);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
double ControllerBase::getCollisionCost() const
{
  return RcsCollisionMdl_cost(this->cMdl);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void ControllerBase::computeCollisionGradient(MatNd* grad)
{
  if (this->cMdl == NULL)
  {
    MatNd_reshapeAndSetZero(grad, 1, this->graph->nJ);
    return;
  }

  computeCollisionModel();
  RcsCollisionMdl_gradient(this->cMdl, grad);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void ControllerBase::getCollisionGradient(MatNd* grad) const
{
  if (this->cMdl == NULL)
  {
    MatNd_reshapeAndSetZero(grad, 1, this->graph->nJ);
    return;
  }

  RcsCollisionMdl_gradient(this->cMdl, grad);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
double ControllerBase::computeTaskCost(const MatNd* xDes,
                                       const MatNd* W,
                                       const MatNd* a_des) const
{
  unsigned int nx = getTaskDim();
  double cost = 0.0;

  RCHECK_MSG((xDes->m==nx) && (xDes->n==1), "xDes->m=%d   xDes->n=%d   nx=%d",
             xDes->m, xDes->n, nx);

  if (W != NULL)
  {
    RCHECK_MSG((W->m==nx) && (W->n==1), "W->m=%d   W->n=%d", W->m, W->n);
  }

  for (unsigned int i = 0; i < this->tasks.size(); i++)
  {
    if ((a_des==NULL) || (MatNd_get(a_des, i, 0)>0.0))
    {
      size_t row = this->taskArrayIdx[i];
      cost += tasks[i]->computeTaskCost(&xDes->ele[row],
                                        W ? &W->ele[row] : NULL);
    }

  }

  return cost;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void ControllerBase::computeTaskGradient(MatNd* grad,
                                         const MatNd* xDes,
                                         const MatNd* W,
                                         const MatNd* a_des) const
{
  unsigned int nx = getTaskDim();

  RCHECK_MSG((xDes->m==nx) && (xDes->n==1), "xDes->m=%d   xDes->n=%d   nx=%d",
             xDes->m, xDes->n, nx);

  if (W != NULL)
  {
    RCHECK((W->m==nx) && (W->n==1));
  }

  MatNd_setZero(grad);

  MatNd* partial_grad = NULL;
  MatNd_create2(partial_grad, 1, getGraph()->nJ);

  for (unsigned int i = 0; i < this->tasks.size(); i++)
  {
    if ((a_des==NULL) || (MatNd_get(a_des, i, 0)>0.0))
    {
      size_t row = this->taskArrayIdx[i];
      tasks[i]->computeTaskGradient(partial_grad, &xDes->ele[row],
                                    W ? &W->ele[row] : NULL);
      MatNd_addSelf(grad, partial_grad);
    }

  }

  MatNd_destroy(partial_grad);
}

/*******************************************************************************
 * Manipulability index according to Yoshikawa: w = sqrt(det(J*J^T)).
 ******************************************************************************/
double ControllerBase::computeManipulabilityCost(const MatNd* a_des,
                                                 const MatNd* W) const
{
  size_t nx = (a_des==NULL) ? getTaskDim() : getActiveTaskDim(a_des);
  MatNd* J = NULL;
  MatNd_create2(J, nx, this->graph->nJ);
  computeJ(J, a_des);
  double cost = MatNd_computeManipulabilityIndex(J, W);
  MatNd_destroy(J);

  return cost;
}

/*******************************************************************************
 * Manipulability index according to Yoshikawa: grad = d/dq(sqrt(det(J*J^T))).
 ******************************************************************************/
double ControllerBase::computeManipulabilityGradient(MatNd* grad,
                                                     const MatNd* a_des,
                                                     const MatNd* W) const
{
  size_t nq = this->graph->nJ;
  size_t nx = (a_des==NULL) ? getTaskDim() : getActiveTaskDim(a_des);
  MatNd* J = NULL;
  MatNd_create2(J, nx, nq);
  computeJ(J, a_des);

  MatNd* H = NULL;
  MatNd_create2(H, nx*nq, nq);
  computeH(H, a_des);
  MatNd_reshape(H, nx*nq, nq);

  double sqrtDet = MatNd_computeManipulabilityIndexGradient(grad, J, H, W);

  MatNd_destroy(J);
  MatNd_destroy(H);

  return sqrtDet;
}

/*******************************************************************************
 * See header. TODO: Use optional activation array to test active tasks only.
 ******************************************************************************/
bool ControllerBase::test(bool verbose)
{
  bool success = true;

  // Check for individual tasks. We traverse the task vector and call each
  // tasks's check function.
  for (size_t id = 0; id < getNumberOfTasks(); id++)
  {
    bool success_i = this->tasks[id]->test(verbose);
    success = success && success_i;

    if ((verbose==true) || (RcsLogLevel>0))
    {
      RMSGS("%s when checking task \"%s\" (%s)",
            success_i ? "SUCCESS" : "FAILURE",
            getTaskName(id).c_str(),
            tasks[id]->getClassName().c_str());
    }
  }

  if (verbose==true)
  {
    RMSG("%d task tests %s",
         (int) getNumberOfTasks(), success ? "succeeded" : "failed");
  }

  // Test speed limit check. We go through all joints that have a speedLimit
  // and are not constrained. We roll a dice and violate it or not. The
  // violation is checked with the checkLimits() function and must return the
  // consistent result.
  for (int i=0; i<(int)getGraph()->dof; ++i)
  {
    const RcsJoint* queryJnt = NULL;

    RCSGRAPH_TRAVERSE_JOINTS(getGraph())
    {
      if (JNT->speedLimit == DBL_MAX || JNT->constrained)
      {
        continue;
      }

      getGraph()->q_dot->ele[JNT->jointIndex] =
        Math_getRandomNumber(-JNT->speedLimit, JNT->speedLimit);

      if (i == JNT->jointIndex)
      {
        queryJnt = JNT;
      }
    }

    if (queryJnt)
    {
      bool violates = Math_getRandomBool();

      if (violates)
      {
        getGraph()->q_dot->ele[i] =
          Math_getRandomNumber(1.01, 1.5)*queryJnt->speedLimit;
      }

      bool checkResult = checkLimits(false, false, true,
                                     0.0, 0.0, 0.0, 0.0, 0.0);

      if (checkResult == violates)
      {
        success = false;
        if (verbose)
        {
          RMSG("%s: FAILURE: %g %g", queryJnt->name,
               getGraph()->q_dot->ele[queryJnt->jointIndex],
               queryJnt->speedLimit);
        }

      }
    }

  }




  return success;
}

/*******************************************************************************
 *
 * Computes the current task forces. Here's what we do:
 *
 * 1. Derivation through joint torque equilibrium:
 *    T_task = T_sensor
 *    J_task^T f_task = J_sensor^T f_sensor
 *    J_task J_task^T f_task = J_task J_sensor^T f_sensor
 *    f_task = inv(J_task J_task^T) J_task J_sensor^T f_sensor
 *           = (J_task inv(J_task J_task^T)^T J_sensor^T f_sensor
 *           = J_task#^T J_sensor^T f_sensor
 *           = (J_sensor rightPinv(J_task))^T J_sensor^T f_sensor
 *
 * 2. Derivation with preservation of direction:
 *    J_task# f_task = J_sensor# f_sensor
 *    f_task = J_task J_sensor# f_sensor
 *    f_task = J_task (J_sensor1# f_sensor1 + J_sensor2# f_sensor2 ...)
 *
 ******************************************************************************/
#if 0
void ControllerBase::computeTaskForce(MatNd* ft_task,
                                      const double S_ft[6],
                                      const RcsSensor* loadCell,
                                      const MatNd* activation) const
{

  // Transform sensor values into world coordinates
  double I_ft[6];
  MatNd I_ft_ = MatNd_fromPtr(6, 1, I_ft);
  VecNd_copy(I_ft, S_ft, 6);
  Vec3d_transRotateSelf(&I_ft[0], loadCell->A_SB->rot);
  Vec3d_transRotateSelf(&I_ft[0], loadCell->body->A_BI->rot);
  Vec3d_transRotateSelf(&I_ft[3], loadCell->A_SB->rot);
  Vec3d_transRotateSelf(&I_ft[3], loadCell->body->A_BI->rot);

  // Compute the sensor Jacobian
  MatNd* J_sensor = NULL;
  MatNd_create2(J_sensor, 6, this->graph->nJ);
  RcsGraph_bodyPointJacobian(getGraph(), loadCell->body, loadCell->A_SB->org,
                             NULL, J_sensor);
  MatNd_reshape(J_sensor, 6, this->graph->nJ);
  MatNd J_rot = MatNd_fromPtr(3, this->graph->nJ, MatNd_getRowPtr(J_sensor, 3));
  RcsGraph_rotationJacobian(this->graph, loadCell->body, NULL, &J_rot);

  MatNd* J_task = NULL;
  MatNd_create2(J_task, getActiveTaskDim(activation), this->graph->nJ);
  computeJ(J_task, activation);

  MatNd* pinvJ = NULL;
  MatNd_create2(pinvJ, J_task->n, J_task->m);
  MatNd_rwPinv(pinvJ, J_task, NULL, NULL);

  // Project into task space:
  // F_task = pinv(J_task)^T * J_sensor^T * I_ft
  //        = (J_sensor*pinv(J_task))^T * I_ft
  MatNd* JpinvJ_tp;
  MatNd_create2(JpinvJ_tp, 6, pinvJ->n);
  MatNd_mul(JpinvJ_tp, J_sensor, pinvJ);
  MatNd_transposeSelf(JpinvJ_tp);

  if (activation==NULL)
  {
    MatNd_reshape(ft_task, pinvJ->n, 1);
    MatNd_mul(ft_task, JpinvJ_tp, &I_ft_);
  }
  else
  {
    MatNd* ft_task_full = NULL;
    MatNd_create2(ft_task_full, pinvJ->n, 1);
    MatNd_mul(ft_task_full, JpinvJ_tp, &I_ft_);
    compressToActive(ft_task, ft_task_full, activation);
    MatNd_destroy(ft_task_full);
  }

  MatNd_destroy(J_sensor);
  MatNd_destroy(J_task);
  MatNd_destroy(pinvJ);
  MatNd_destroy(JpinvJ_tp);

  //#else   // f_task = J_task J_sensor# f_sensor

  // Transform sensor values into world coordinates
  double I_ft[6];
  MatNd I_ft_ = MatNd_fromPtr(6, 1, I_ft);
  VecNd_copy(I_ft, S_ft, 6);
  Vec3d_transRotateSelf(&I_ft[0], loadCell->A_SB->rot);
  Vec3d_transRotateSelf(&I_ft[0], loadCell->body->A_BI->rot);
  Vec3d_transRotateSelf(&I_ft[3], loadCell->A_SB->rot);
  Vec3d_transRotateSelf(&I_ft[3], loadCell->body->A_BI->rot);

  // Compute the sensor Jacobian
  MatNd* J_sensor = NULL;
  MatNd_create2(J_sensor, 6, this->graph->nJ);
  RcsGraph_bodyPointJacobian(getGraph(), loadCell->body, loadCell->A_SB->org,
                             NULL, J_sensor);
  MatNd_reshape(J_sensor, 6, this->graph->nJ);
  MatNd J_rot = MatNd_fromPtr(3, this->graph->nJ, MatNd_getRowPtr(J_sensor, 3));
  RcsGraph_rotationJacobian(this->graph, loadCell->body, NULL, &J_rot);

  // Sensor wrench Pseudo-inverse
  MatNd* pinvJ_sensor = NULL;
  MatNd_create2(pinvJ_sensor, J_sensor->n, J_sensor->m);
  MatNd_rwPinv(pinvJ_sensor, J_sensor, NULL, NULL);

  // Task Jacobian
  MatNd* J_task = NULL;
  size_t nx;

  if (activation == NULL)
  {
    nx = getTaskDim();
  }
  else
  {
    nx = getActiveTaskDim(activation);
  }

  MatNd_create2(J_task, nx, this->graph->nJ);
  computeJ(J_task, activation);

  // J_task J_sensor#
  MatNd* JJt;
  MatNd_create2(JJt, J_task->m, pinvJ_sensor->n);
  MatNd_mul(JJt, J_task, pinvJ_sensor);

  MatNd_reshape(ft_task, J_task->m, 1);
  MatNd_mul(ft_task, JJt, &I_ft_);

  MatNd_destroy(J_sensor);
  MatNd_destroy(pinvJ_sensor);
  MatNd_destroy(J_task);

}
#endif

/*******************************************************************************
 *
 * Computes the current task forces. Here's what we do:
 *
 * 1. Derivation through joint torque equilibrium:
 *    T_task = T_sensor
 *    J_task^T f_task = J_sensor1^T f_sensor1 + J_sensor2^T f_sensor2 + ...
 *    J_task J_task^T f_task = J_task (J_sensor1^T f_sensor1 + ...)
 *    f_task = inv(J_task J_task^T) J_task (J_sensor1^T f_sensor1 + ...)
 *           = (J_task inv(J_task J_task^T)^T (J_sensor1^T f_sensor1 + ...)
 *           = J_task#^T (J_sensor1^T f_sensor1 + ...)
 *
 ******************************************************************************/



/******************************************************************************
 * Threshold the force torque values of the sensor, in regards to noise.
 *****************************************************************************/
static void thresholdFtSensor(double* S_ft_f,
                              const double* S_ft,
                              const MatNd* f_treshold,
                              const unsigned int ftDim)
{
  // Threshold the force torque values
  for (unsigned int i = 0; i < ftDim; ++i)
  {
    if (S_ft[i] > f_treshold->ele[i])
    {
      S_ft_f[i] = S_ft[i] - f_treshold->ele[i];
    }
    else if (S_ft[i] < -f_treshold->ele[i])
    {
      S_ft_f[i] = S_ft[i] + f_treshold->ele[i];
    }
    else
    {
      S_ft_f[i] = 0.0;
    }
  }
}

/*******************************************************************************
 *
 * Computes the current task forces. Here's what we do:
 *
 *    J_task# f_task = J_sensor1# f_sensor1 + J_sensor2# f_sensor2 ...
 *
 *    with J_task J_task# = J_task (J_task^T (J_task J_task^T)^-1)
 *                        = J_task J_task^T (J_task J_task^T)^-1 = I
 *
 *    f_task = J_task (J_sensor1# f_sensor1 + J_sensor2# f_sensor2 ...)
 *
 ******************************************************************************/
void ControllerBase::computeTaskForce(MatNd* ft_task,
                                      const MatNd* activation) const
{
  size_t nx = activation ? getActiveTaskDim(activation) : getTaskDim();

  MatNd* J_sensor = NULL;
  MatNd_create2(J_sensor, 6, this->graph->nJ);

  MatNd* pinvJ_sensor = NULL;
  MatNd_create2(pinvJ_sensor, J_sensor->n, J_sensor->m);

  MatNd* pinvJF = NULL;
  MatNd_create2(pinvJF, this->graph->nJ, 1);


  for (unsigned int i=0; i<graph->nSensors; ++i)
  {
    RcsSensor* SENSOR = &graph->sensors[i];

    if (SENSOR->type != RCSSENSOR_LOAD_CELL)
    {
      continue;
    }

    RcsSensor* loadCell = SENSOR;
    RCHECK(loadCell->rawData->size <= 9);
    const double* S_ft = loadCell->rawData->ele;
    // RLOG(1," \n print the extra info %s ", loadCell->extraInfo);

    // ---

    // filtering in sensor space and coordinate system
    // filter vector of sensor values
    double S_ft_f[9];

    MatNd* thrds_abs = MatNd_create(loadCell->rawData->size, 1);
    // Here read the threshold from the xml
    // readTaskVectorFromXML(thrds_abs, "threshold");
    MatNd_addConst(thrds_abs, 10.0);

    // threshold the ft sensor values
    thresholdFtSensor(S_ft_f, S_ft, thrds_abs, loadCell->rawData->size);
    // ---

    // Transform sensor values into world coordinates
    double I_ft[6];
    MatNd I_ft_ = MatNd_fromPtr(6, 1, I_ft);
    VecNd_copy(I_ft, S_ft_f, 6);
    RcsBody* loadCellBdy = &graph->bodies[loadCell->bodyId];
    Vec3d_transRotateSelf(&I_ft[0], loadCell->A_SB.rot);
    Vec3d_transRotateSelf(&I_ft[0], loadCellBdy->A_BI.rot);
    Vec3d_transRotateSelf(&I_ft[3], loadCell->A_SB.rot);
    Vec3d_transRotateSelf(&I_ft[3], loadCellBdy->A_BI.rot);

    // Compute the sensor Jacobian
    RcsGraph_bodyPointJacobian(getGraph(), loadCellBdy,
                               loadCell->A_SB.org, NULL, J_sensor);
    MatNd_reshape(J_sensor, 6, this->graph->nJ);
    MatNd J_rot = MatNd_fromPtr(3, this->graph->nJ,
                                MatNd_getRowPtr(J_sensor, 3));
    RcsGraph_rotationJacobian(this->graph, loadCellBdy, NULL, &J_rot);

    // Sensor wrench Pseudo-inverse
    MatNd_rwPinv(pinvJ_sensor, J_sensor, NULL, NULL);

    // pinvJF += J_sensor_i# f_sensor_i
    MatNd_mulAndAddSelf(pinvJF, pinvJ_sensor, &I_ft_);

  }


  // Task Jacobian
  MatNd* J_task = NULL;
  MatNd_create2(J_task, nx, this->graph->nJ);
  computeJ(J_task, activation);

  MatNd_reshape(ft_task, J_task->m, 1);
  MatNd_mul(ft_task, J_task, pinvJF);

  if (activation != NULL)
  {
    decompressFromActiveSelf(ft_task, activation);
  }

  MatNd_destroy(J_sensor);
  MatNd_destroy(pinvJ_sensor);
  MatNd_destroy(pinvJF);
  MatNd_destroy(J_task);
}




/*******************************************************************************
 * See header.
 ******************************************************************************/
bool ControllerBase::add(const ControllerBase& other,
                         const char* suffix,
                         const HTr* A_BP)
{
  return add(&other, suffix, A_BP);
}

/*******************************************************************************
 *
 ******************************************************************************/
static inline int getBodyIdWithSuffix(const RcsBody* bdy, std::string suffix,
                                      const RcsGraph* graph)
{
  if (bdy==NULL)
  {
    return -1;
  }

  std::string newName = std::string(bdy->name) + suffix;
  const RcsBody* bdy_suffixed = RcsGraph_getBodyByName(graph, newName.c_str());
  return bdy_suffixed ? bdy_suffixed->id : -1;
}

/*******************************************************************************
 *
 ******************************************************************************/
static inline const RcsBody* getBodyWithSuffix(const RcsBody* bdy,
                                               std::string suffix,
                                               const RcsGraph* graph)
{
  if (bdy==NULL)
  {
    return NULL;
  }

  std::string newName = std::string(bdy->name) + suffix;

  return RcsGraph_getBodyByName(graph, newName.c_str());
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool ControllerBase::add(const ControllerBase* other,
                         const char* suffixPtr,
                         const HTr* A_BP)
{
  bool success = RcsGraph_appendCopyOfGraph(getGraph(), NULL, other->getGraph(),
                                            suffixPtr, A_BP);

  if (success == false)
  {
    return false;
  }

  std::string suffix;
  if (suffixPtr)
  {
    suffix = std::string(suffixPtr);
  }

  const std::vector<Task*>& otherTasks = other->taskVec();

  for (size_t i=0; i<otherTasks.size(); ++i)
  {
    Task* copyOfOtherTask = otherTasks[i]->clone(this->graph);

    // Set the new task's name uniquely considering the suffix. We only do this
    // if the task has a name so that unnamed tasks don't show up in the GUIs
    // \todo: Should go into a task-specific function
    if (!copyOfOtherTask->getName().empty())
    {
      copyOfOtherTask->setName(copyOfOtherTask->getName() + suffix);
    }

    // \todo: Should go into a task-specific function
    copyOfOtherTask->setEffectorId(getBodyIdWithSuffix(copyOfOtherTask->getEffector(), suffix, getGraph()));
    copyOfOtherTask->setRefBodyId(getBodyIdWithSuffix(copyOfOtherTask->getRefBody(), suffix, getGraph()));
    copyOfOtherTask->setRefFrameId(getBodyIdWithSuffix(copyOfOtherTask->getRefFrame(), suffix, getGraph()));

    // Reassign sub-tasks in CompositeTask
    // \todo: Should go into a task-specific function
    if (dynamic_cast<CompositeTask*>(copyOfOtherTask))
    {
      CompositeTask* cTask = dynamic_cast<CompositeTask*>(copyOfOtherTask);

      for (size_t i=0; i<cTask->getNumberOfTasks(); ++i)
      {
        Task* ti = cTask->getSubTask(i);
        ti->setEffectorId(getBodyIdWithSuffix(ti->getEffector(), suffix, getGraph()));
        ti->setRefBodyId(getBodyIdWithSuffix(ti->getRefBody(), suffix, getGraph()));
        ti->setRefFrameId(getBodyIdWithSuffix(ti->getRefFrame(), suffix, getGraph()));
      }

    }

    // Rename joint names for TaskJoint
    // \todo: Should go into a task-specific function
    if (dynamic_cast<TaskJoint*>(copyOfOtherTask))
    {
      TaskJoint* jntTask = dynamic_cast<TaskJoint*>(copyOfOtherTask);
      std::string newName = std::string(jntTask->getJoint()->name) + suffix;
      jntTask->setJoint(RcsGraph_getJointByName(getGraph(), newName.c_str()));
      RCHECK_MSG(jntTask->getJoint(), "Not found: joint %s", newName.c_str());
    }

    // Rename joint names for TaskJoints
    // \todo: Should go into a task-specific function
    if (dynamic_cast<TaskJoints*>(copyOfOtherTask))
    {
      RLOG(5, "Copying joints task %s", copyOfOtherTask->getName().c_str());
      TaskJoints* jntsTask = dynamic_cast<TaskJoints*>(copyOfOtherTask);

      for (size_t i=0; i<jntsTask->getNumberOfTasks(); ++i)
      {
        TaskJoint* jntTask = dynamic_cast<TaskJoint*>(jntsTask->getSubTask(i));
        RCHECK(jntTask);
        std::string newName = std::string(jntTask->getJoint()->name) + suffix;
        jntTask->setJoint(RcsGraph_getJointByName(getGraph(), newName.c_str()));
        RCHECK_MSG(jntTask->getJoint(), "Not found: joint %s", newName.c_str());
      }
    }


    add(copyOfOtherTask);
  }

  return true;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool ControllerBase::eraseTask(size_t index)
{
  if (index > tasks.size() - 1)
  {
    RLOG_CPP(1, "Failed to erase task with index " << index
             << " - should be less than " << tasks.size());
    return false;
  }

  delete this->tasks[index];
  this->tasks.erase(tasks.begin() + index);
  recomputeIndices();

  return true;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool ControllerBase::eraseTask(const std::string& taskName)
{
  int index = getTaskIndex(taskName.c_str());

  if (index < 0)
  {
    RLOG_CPP(1, "Failed to erase task with name " << taskName);
    return false;
  }

  return eraseTask(index);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool ControllerBase::replaceTask(size_t index, Task* newTask)
{
  if (newTask==NULL)
  {
    RLOG_CPP(1, "Failed to replace task with NULL task");
    return false;
  }

  if (index > tasks.size() - 1)
  {
    RLOG_CPP(1, "Failed to replace task with index " << index
             << " - should be less than " << tasks.size());
    return false;
  }

  size_t oldDim = tasks[index]->getDim();

  delete this->tasks[index];
  this->tasks[index] = newTask;

  if (oldDim != newTask->getDim())
  {
    recomputeIndices();
  }

  return true;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool ControllerBase::replaceTask(const std::string& taskName,
                                 Task* newTask)
{
  int index = getTaskIndex(taskName.c_str());

  if (index < 0)
  {
    RLOG_CPP(1, "Failed to replace task with name " << taskName);
    return false;
  }

  return replaceTask(index, newTask);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void ControllerBase::add(Task* other)
{
  if (other == NULL)
  {
    RLOG(4, "Ignoring NULL task");
    return;
  }

  if (this->taskArrayIdx.empty())   // back() is undefined if vector is empty
  {
    this->taskArrayIdx.push_back(0);
  }
  else
  {
    size_t idx = this->taskArrayIdx.back() + this->tasks.back()->getDim();
    this->taskArrayIdx.push_back(idx);
  }

  this->tasks.push_back(other);
}

/******************************************************************************
 * Generic for all tasks.
 *****************************************************************************/
static void calcAdmittanceDelta(double* compliantDelta,
                                const double* ft_task,
                                const double* Kp_ext,
                                const double f_treshold,
                                const unsigned int taskDim)
{
  // Compute the compliant displacement
  for (unsigned int i = 0; i < taskDim; ++i)
  {
    if (ft_task[i] > f_treshold/2)
    {
      compliantDelta[i] = Kp_ext[i]*(ft_task[i]-f_treshold);
    }
    else if (ft_task[i] < -f_treshold/2)
    {
      compliantDelta[i] = Kp_ext[i]*(ft_task[i]+f_treshold);
    }
    else
    {
      compliantDelta[i] = 0.0;
    }
  }
}

/*******************************************************************************
 * Computes the admittance control law.
 ******************************************************************************/
void ControllerBase::computeAdmittance(MatNd* compliantFrame,
                                       const MatNd* ft_task,
                                       const MatNd* Kp_ext,
                                       const MatNd* a_des)
{
  double cmplDelta_[6];

  for (size_t taskIdx = 0; taskIdx < this->tasks.size(); taskIdx++)
  {
    if (a_des->ele[taskIdx] <= 0.0)
    {
      continue;
    }

    Task* task_i = getTask(taskIdx);
    size_t xIdx = getTaskArrayIndex(taskIdx);
    size_t taskDim = getTaskDim(taskIdx);

    const double f_treshold= 1.5; // Start above this threshold (in [N])

    // angular Euler
    // const double t_treshold = 1.0; // Start above this threshold (in [Nm])
    double* cmplDelta = cmplDelta_;
    if (taskDim>6)
    {
      cmplDelta = new double[taskDim];
    }

    calcAdmittanceDelta(cmplDelta, &ft_task->ele[xIdx], &Kp_ext->ele[xIdx],
                        f_treshold, taskDim); //0.50

    // Integrate the compliant frame
    if (VecNd_sqrLength(cmplDelta, taskDim)>0.0)
    {
      task_i->integrateXp_ik(&compliantFrame->ele[xIdx],
                             &compliantFrame->ele[xIdx], cmplDelta, 1.0);
    }

    if (taskDim>6)
    {
      delete [] cmplDelta;
    }

  }   // for (int taskIdx = 0; taskIdx < this->tasks.size(); taskIdx++)

}

/*******************************************************************************
 *
 * Computes the current task forces. Here's what we do:
 *
 *    J_task# f_task = J_sensor# f_sensor
 *
 *    with J_sensor and f_sensor being the stacked arrays of all sensors
 *
 *    with J_task J_task# = I , we get f_task = J_task J_sensor# f_sensor
 *
 ******************************************************************************/
#if 0
void ControllerBase::computeTaskForce(MatNd* ft_task,
                                      const MatNd* activation,
                                      // std::vector<MedianFilterND*>* MedFilters_vec,
                                      std::vector<SecondOrderLPFND*>* secOrderfilt,
                                      double* s_ftL, double* s_ft_fL,
                                      double* s_ftR, double* s_ft_fR) const
{
  size_t nx = activation ? getActiveTaskDim(activation) : getTaskDim();
  size_t nLoadCells = 0;

  RCSGRAPH_TRAVERSE_SENSORS(this->graph)
  {
    if (SENSOR->type == RCSSENSOR_LOAD_CELL)
    {
      nLoadCells++;
    }
  }

  MatNd* J_sensor = NULL;
  MatNd_create2(J_sensor, nLoadCells*6, this->graph->nJ);

  MatNd* ft_sensor = NULL;
  MatNd_create2(ft_sensor, nLoadCells*6, 1);

  nLoadCells = 0;


  // init an iterator over the filters for the sensors
  // std::vector<MedianFilterND*>::iterator filter;
  std::vector<SecondOrderLPFND*>::iterator filter;

  // get the first filter, which is meant to match the first
  if (secOrderfilt != NULL)
    // if (MedFilters_vec != NULL)
  {
    filter = secOrderfilt->begin();
    // filter = MedFilters_vec->begin();
  }

  RCSGRAPH_TRAVERSE_SENSORS(this->graph)
  {
    if (SENSOR->type != RCSSENSOR_LOAD_CELL)
    {
      continue;
    }

    RcsSensor* loadCell = SENSOR;
    RCHECK(loadCell->rawData->size <= 9);

    const double* S_ft_init = loadCell->rawData->ele;
    // clone the ft snsor values to clip them
    double S_ft[9];
    VecNd_copy(S_ft, S_ft_init, loadCell->rawData->size);

    // clipping
    MatNd* clipLimitFT = MatNd_create(loadCell->rawData->size, 1);
    MatNd_addConst(clipLimitFT, 20.0);
    MatNd_set(clipLimitFT, 3,0, 3.0);
    MatNd_set(clipLimitFT, 4,0, 3.0);
    MatNd_set(clipLimitFT, 5,0, 3.0);

    // matNd vec to clip the sensor vals
    MatNd S_ft_Mat = MatNd_fromPtr(loadCell->rawData->size, 1, S_ft);
    MatNd_saturateSelf(&S_ft_Mat, clipLimitFT);

    // ---
    // filtering in sensor space and coordinate system
    // filter vector of sensor values
    double  S_ft_f[9];
    MatNd* thrds_abs = MatNd_create(loadCell->rawData->size, 1);
    // Here read the threshold from the xml (PENDING!!!!)
    MatNd_addConst(thrds_abs, 5.0);
    MatNd_set(thrds_abs, 3,0, 0.75); // 0.25
    MatNd_set(thrds_abs, 4,0, 0.75); // 0.25
    MatNd_set(thrds_abs, 5,0, 0.75); // 0.25

    // if a vector of filters has been initialised
    if ((secOrderfilt!=NULL) && (filter!=secOrderfilt->end()))
      // if (MedFilters_vec != NULL)
    {
      // filter the ft sensor values
      // (*filter)->addSample(S_ft);
      // (*filter)->filt();
      // (*filter)->getMedian(S_ft_f);
      (*filter)->setTarget(S_ft);
      (*filter)->iterate();
      (*filter)->getPosition(S_ft_f);

      // threshold the ft sensor values
      thresholdFtSensor(S_ft_f, S_ft_f, thrds_abs, loadCell->rawData->size);
    }
    else
    {
      // threshold the ft sensor values
      thresholdFtSensor(S_ft_f, S_ft, thrds_abs, loadCell->rawData->size);
    }
    // ---


    // Transform sensor values into world coordinates
    double* I_ft = &ft_sensor->ele[6*nLoadCells];
    VecNd_copy(I_ft, S_ft_f, 6);
    Vec3d_transRotateSelf(&I_ft[0], loadCell->A_SB->rot);
    Vec3d_transRotateSelf(&I_ft[0], loadCell->body->A_BI->rot);
    Vec3d_transRotateSelf(&I_ft[3], loadCell->A_SB->rot);
    Vec3d_transRotateSelf(&I_ft[3], loadCell->body->A_BI->rot);

    // VecNd_printComment("I_ft", &I_ft[0], 6);

    // Compute the sensor Jacobian
    MatNd J_trans = MatNd_fromPtr(3, J_sensor->n,
                                  MatNd_getRowPtr(J_sensor, 6*nLoadCells));
    RcsGraph_bodyPointJacobian(getGraph(), loadCell->body,
                               loadCell->A_SB->org, NULL, &J_trans);
    MatNd J_rot = MatNd_fromPtr(3, J_sensor->n,
                                MatNd_getRowPtr(J_sensor, 6*nLoadCells+3));
    RcsGraph_rotationJacobian(getGraph(), loadCell->body, NULL, &J_rot);

    // 0 is for left and 1 for right
    if (nLoadCells == 0 && s_ftL != NULL && s_ft_fL != NULL)
    {
      VecNd_copy(s_ftL, S_ft,6);
      VecNd_copy(s_ft_fL, S_ft_f,6);
    }
    else if (nLoadCells == 1 && s_ftR != NULL && s_ft_fR != NULL)
    {
      VecNd_copy(s_ftR, S_ft,6);
      VecNd_copy(s_ft_fR, S_ft_f,6);
    }

    // count loadcells
    nLoadCells++;

    // increase filter vector iterator (get next filter respective to next
    // sensor)
    filter++;
  }

  // MatNd_setRandom(ft_sensor, -1.0, 1.0);

  // Sensor wrench Pseudo-inverse
  MatNd* pinvJ_sensor = NULL;
  MatNd_create2(pinvJ_sensor, J_sensor->n, J_sensor->m);
  MatNd_generalizedInverse(pinvJ_sensor, J_sensor, NULL, NULL);

  // Projected sensor torque
  MatNd* T_sensor = NULL;
  MatNd_create2(T_sensor, this->graph->nJ, 1);
  MatNd_mul(T_sensor, pinvJ_sensor, ft_sensor);

  // Task Jacobian
  MatNd* J_task = NULL;
  MatNd_create2(J_task, nx, this->graph->nJ);
  computeJ(J_task, activation);

  MatNd_reshape(ft_task, J_task->m, 1);
  MatNd_mul(ft_task, J_task, T_sensor);
  // MatNd_printCommentDigits("T_sensor", T_sensor, 6);

  if (activation != NULL)
  {
    decompressFromActiveSelf(ft_task, activation);
  }
  // MatNd_printCommentDigits("ft_task", ft_task, 6);

  // VecNd_constMulSelf(&ft_sensor->ele[0], -1.0, 6);
  // VecNd_addSelf(&ft_sensor->ele[0], &ft_sensor->ele[6], 6);
  // MatNd_reshape(ft_sensor, 3, 1);

  // Vec3d_rotateSelf(&ft_sensor->ele[0], graph->sensor->body->A_BI->rot);

  // MatNd_printCommentDigits("ft_sensor", ft_sensor, 6);

  MatNd_destroy(J_sensor);
  MatNd_destroy(ft_sensor);
  MatNd_destroy(pinvJ_sensor);
  MatNd_destroy(J_task);
}
#endif

/*******************************************************************************
 *
 ******************************************************************************/
void ControllerBase::printX(const MatNd* x, const MatNd* a_des) const
{
  for (size_t i = 0; i < getNumberOfTasks(); i++)
  {
    if ((a_des==NULL) || (MatNd_get(a_des, i, 0)>0.0))
    {
      const size_t row = this->taskArrayIdx[i];
      for (size_t j = 0; j < getTaskDim(i); j++)
      {
        printf("Task \"%s\"[%d]: %f\n", getTaskName(i).c_str(), (int) j,
               MatNd_get(x, row+j, 0));
      }
    }
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
bool ControllerBase::getModelState(MatNd* q, const char* modelStateName,
                                   int timeStamp)
{
  return RcsGraph_getModelStateFromXML(q, getGraph(), modelStateName,
                                       timeStamp);
}

/*******************************************************************************
 *
 ******************************************************************************/
bool ControllerBase::checkLimits(bool checkJointLimits,
                                 bool checkCollisions,
                                 bool checkJointVelocities,
                                 double jlMarginAngular,
                                 double jlMarginLinear,
                                 double collMargin,
                                 double speedMarginAngular,
                                 double speedMarginLinear) const
{
  bool success = true;

  // Joint limit check
  if (checkJointLimits)
  {
    bool verbose = (RcsLogLevel >= 3) ? true : false;
    unsigned int aor = RcsGraph_numJointLimitsViolated(getGraph(),
                                                       jlMarginAngular,
                                                       jlMarginLinear,
                                                       verbose);
    if (aor > 0)
    {
      success = false;
      RLOG(3, "%d joint limit violations", aor);
      REXEC(4)
      {
        RcsGraph_printState(getGraph(), getGraph()->q);
      }
    }
  }

  // Collision check. It is assumed that the collision model was computed before
  // calling this function. We shouldn't call it here since the function is
  // declared to be const.
  if (checkCollisions && getCollisionMdl())
  {
    const double distLimit = collMargin;
    double minDist = RcsCollisionMdl_getMinDist(getCollisionMdl());
    if (minDist < distLimit)
    {
      success = false;
      RLOG(3, "Found collision distance of %f (must be >%f)",
           minDist, distLimit);
      REXEC(4)
      {
        RcsCollisionModel_fprintCollisions(stdout, getCollisionMdl(),
                                           distLimit);
      }
    }
  }

  // Speed limit check.
  if (checkJointVelocities)
  {
    RCSGRAPH_TRAVERSE_JOINTS(getGraph())
    {
      if (JNT->constrained)
      {
        continue;
      }

      const bool isRot = RcsJoint_isRotation(JNT);
      const double margin = isRot ? speedMarginAngular : speedMarginLinear;
      const double q_dot = getGraph()->q_dot->ele[JNT->jointIndex];

      if ((q_dot<-JNT->speedLimit+margin) || (q_dot>JNT->speedLimit-margin))
      {
        success = false;
        const double sf = isRot ? 180.0 / M_PI : 1.0;
        RLOG(4, "%s: q_dot=%f   limit=%f [%s]", JNT->name, sf*q_dot,
             sf*(JNT->speedLimit-margin), isRot ? "deg/sec" : "m/sec");
      }
    }

  }

  return success;
}

/*******************************************************************************
 *
 ******************************************************************************/
void ControllerBase::swapTaskVec(std::vector<Task*>& newTasks,
                                 bool recomputeArrayIndices)
{
  std::swap(newTasks, this->tasks);

  if (recomputeArrayIndices==true && (!tasks.empty()))
  {
    taskArrayIdx.clear();
    taskArrayIdx.push_back(0);

    for (size_t i=0; i<tasks.size()-1; ++i)
    {
      size_t idx = this->taskArrayIdx.back() + this->tasks.back()->getDim();
      this->taskArrayIdx.push_back(idx);
    }
  }

}

/*******************************************************************************
 * Print the usage description if any
 ******************************************************************************/
void ControllerBase::printUsage(const std::string& xmlFile)
{
  char cfgFile[256];
  bool fileExists = Rcs_getAbsoluteFileName(xmlFile.c_str(), cfgFile);

  if (!fileExists)
  {
    RLOG(1, "XML file \"%s\" not found in resource paths", xmlFile.c_str());
    return;
  }

  xmlDocPtr docPtr;
  xmlNodePtr node = parseXMLFile(cfgFile, "Controller", &docPtr);

  if (node==NULL)
  {
    RLOG(1, "Node \"Controller\" not found in \"%s\"", cfgFile);
    return;
  }

  xmlChar* usage = xmlGetProp(node, (const xmlChar*) "usage");

  if (usage)
  {
    std::cout << "Usage: " << std::endl << usage << std::endl;
  }
  else
  {
    std::cout << "No usage description" << std::endl;
  }

  xmlFreeDoc(docPtr);
}

/*******************************************************************************
 *
 ******************************************************************************/
void ControllerBase::print() const
{
  // Print information for each task
  for (size_t i = 0; i < getNumberOfTasks(); i++)
  {
    tasks[i]->print();
  }

  RcsCollisionModel_fprint(stderr, this->cMdl);

  printUsage(this->xmlFile);
}

/*******************************************************************************
 *
 ******************************************************************************/
bool ControllerBase::toXML(const std::string& fileName,
                           const MatNd* activation) const
{
  if (activation && (activation->m != getNumberOfTasks()))
  {
    RLOG_CPP(1, "Activation vector has " << activation->m
             << " rows, " << getNumberOfTasks() << " tasks in controller");
    return false;
  }

  FILE* out = fopen(fileName.c_str(), "w+");

  if (out == NULL)
  {
    RLOG_CPP(1, "Failed to open file " << fileName);
    return false;
  }

  fprintf(out, "<Controller>\n\n");

  RcsGraph_fprintXML(out, getGraph());

  // Print information for each task
  fprintf(out, "\n");
  for (size_t i = 0; i < getNumberOfTasks(); i++)
  {
    bool ai = false;
    if ((activation == NULL) || (activation->ele[i] != 0.0))
    {
      ai = true;
    }

    tasks[i]->toXML(out, ai);
  }

  fprintf(out, "\n</Controller>\n");

  fclose(out);

  return true;
}

/*******************************************************************************
 * Calculate one step of the inverse dynamics: T = M*aq + h + g
 *
 *      Here we neglect the influence of the accelerations of the
 *      kinematically driven joints. That's fine if there's no large
 *      inertia being accelerated, such as in the case of the finger
 *      movements. However, if the whole robot is for instance attached to
 *      a kinematically driven torso, this must be considered.
 ******************************************************************************/
void ControllerBase::computeInvDynJointSpace(MatNd* T_des,
                                             const RcsGraph* graph,
                                             const MatNd* q_des,
                                             const MatNd* qp_des,
                                             const MatNd* qpp_des,
                                             double positionGain,
                                             double velocityGain)
{
  const unsigned int nq = graph->nJ;
  MatNd* M       = MatNd_create(nq, nq);
  MatNd* g       = MatNd_create(nq, 1);
  MatNd* h       = MatNd_create(nq, 1);
  MatNd* aq      = MatNd_create(graph->dof, 1);
  MatNd* qp_temp = MatNd_create(1, nq);

  if (velocityGain==-1.0)
  {
    velocityGain = 0.5*sqrt(4.0*positionGain);
  }

  const MatNd* q_curr = graph->q;
  MatNd_reshapeAndSetZero(T_des, nq, 1);

  // Dynamics: Mass matrix, gravity load and h-vector
  RcsGraph_computeKineticTerms(graph, NULL, M, h, g);

  // aq = -kp*(q-q_des)
  if (q_des != NULL)
  {
    MatNd_sub(aq, q_curr, q_des);
  }

  RcsGraph_stateVectorToIKSelf(graph, aq);
  MatNd_constMulSelf(aq, -positionGain);

  // Get the current joint velocities
  RcsGraph_stateVectorToIK(graph, graph->q_dot, qp_temp);

  // aq = aq  -kd*qp_curr + kd*qp_des
  MatNd_constMulAndAddSelf(aq, qp_temp, -velocityGain);

  if (qp_des != NULL)
  {
    MatNd_constMulAndAddSelf(aq, qp_des, velocityGain);
  }

  if (qpp_des != NULL)
  {
    MatNd_addSelf(aq, qpp_des);  // +qpp_des
  }

  // Set the speed of the kinematic joints to zero
  RCSGRAPH_TRAVERSE_JOINTS(graph)
  {
    if ((JNT->ctrlType!=RCSJOINT_CTRL_TORQUE) && (JNT->jacobiIndex!=-1))
    {
      MatNd_set(aq, JNT->jacobiIndex, 0, 0.0);
    }
  }

  // Add gravity and coriolis compensation: u += h + Fg
  // u = M*a + h + g
  MatNd_reshape(T_des, nq, 1);
  MatNd_mul(T_des, M, aq);   // Tracking error
  MatNd_subSelf(T_des, h);   // Cancellation of coriolis forces
  MatNd_subSelf(T_des, g);   // Cancellation of gravity forces

  MatNd_destroy(M);
  MatNd_destroy(g);
  MatNd_destroy(h);
  MatNd_destroy(aq);
  MatNd_destroy(qp_temp);
}

/*******************************************************************************
 *
 ******************************************************************************/
void ControllerBase::computeInvDynJointSpace(MatNd* T_des,
                                             const RcsGraph* graph,
                                             const MatNd* q_des,
                                             double positionGain,
                                             double velocityGain)
{
  computeInvDynJointSpace(T_des, graph, q_des, NULL, NULL,
                          positionGain, velocityGain);
}

/*******************************************************************************
 *
 ******************************************************************************/
void ControllerBase::recomputeIndices()
{
  taskArrayIdx.clear();

  if (tasks.empty())
  {
    return;
  }

  taskArrayIdx.push_back(0);

  for (size_t i = 1; i < this->tasks.size(); ++i)
  {
    taskArrayIdx.push_back(taskArrayIdx[i-1] + tasks[i-1]->getDim());
  }

}

}   // namespace Rcs
