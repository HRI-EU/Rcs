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

#include "ControllerBase.h"
#include "TaskFactory.h"
#include "TaskJoints.h"
#include "TaskJoint.h"
#include "Rcs_math.h"
#include "Rcs_macros.h"
#include "Rcs_typedef.h"
#include "Rcs_resourcePath.h"
#include "Rcs_graphParser.h"
#include "Rcs_kinematics.h"
#include "Rcs_dynamics.h"
#include "Rcs_joint.h"




/*******************************************************************************
 * Constructor based on xml parsing.
 ******************************************************************************/
Rcs::ControllerBase::ControllerBase(const std::string& xmlDescription,
                                    bool xmlParsingFinished) :
  graph(NULL),
  ownsGraph(true),
  xmlRootNode(NULL),
  xmlDoc(NULL),
  cMdl(NULL),
  name("Unknown controller"),
  xmlFile(xmlDescription)
{
  char txt[256];
  bool fileExists = Rcs_getAbsoluteFileName(xmlDescription.c_str(), txt);

  if (fileExists==true)
  {
    this->xmlFile.assign(txt);
    this->xmlRootNode = parseXMLFile(txt, "Controller", &this->xmlDoc);
  }
  else
  {
    const char* fileExtension = strrchr(xmlDescription.c_str(), '.');

    // Check if the xmlDescription is really a file (with extension .xml). If
    // it has the .xml extension, we assume the file couldn't be opened and
    // exit with a fatal error.
    if (fileExtension != NULL)
    {
      if (STREQ(fileExtension, ".xml"))
      {
        RMSG("Resource path is:");
        Rcs_printResourcePath();
        RFATAL("Controller configuration file \"%s\" not found in "
               "resource path - exiting", xmlDescription.c_str());
      }
    }

    // From here we assume that is is a memory buffer holding the xml
    // file as a string.
    this->xmlFile.assign("Created_from_memory_buffer");
    this->xmlRootNode = parseXMLMemory(xmlDescription.c_str(),
                                       xmlDescription.size(),
                                       &this->xmlDoc);
  }

  // Check if the xml description is malformed
  if (this->xmlRootNode == NULL)
  {
    RFATAL("Node \"Controller\" not found in \"%s\"", this->xmlFile.c_str());
  }

  initFromXmlNode(this->xmlRootNode);

  // Free the xml memory in case no child ctors need to continue parsing
  if (xmlParsingFinished)
  {
    closeXmlFile();
  }

}

/*******************************************************************************
 * Initialization from xml node "Controller".
 ******************************************************************************/
void Rcs::ControllerBase::initFromXmlNode(xmlNodePtr xmlNodeController)
{
  // Parse controller name
  char txt[256];
  strcpy(txt, "Unknown controller");
  getXMLNodePropertyStringN(xmlNodeController, "name", txt, 256);
  this->name.assign(txt);
  RLOG(5, "Controller name is \"%s\"", this->name.c_str());

  // Create the graph. If no tag "graph" exists, the graph still can
  // be included by an xinclude directive. Therefore it's not fatal
  // if we don't find it here.
  if (getXMLNodePropertyStringN(xmlNodeController, "graph", txt, 256))
  {
    this->xmlGraphFile = std::string(txt);
    this->graph = RcsGraph_create(txt);
    RCHECK_MSG(this->graph, "Failed to create graph \"%s\"", txt);
  }

  // Descend one level in XML parsing to find Task et al.
  xmlNodePtr node = xmlNodeController->children;

  while (node)
  {
    if (isXMLNodeName(node, "Task"))
    {
      // Create the new task, and add it to the task list
      if (getXMLNodePropertyStringN(node, "controlVariable", txt, 256))
      {
        std::string cVar(txt);
        add(TaskFactory::instance()->createTask(cVar, node, graph));
      }
      else
      {
        RLOG(1, "No controlVariable tag found in task description");
      }

    }
    else if (isXMLNodeName(node, "CollisionModel"))
    {
      this->cMdl = RcsCollisionModel_createFromXML(getGraph(), node);

      if (this->cMdl != NULL)
      {
        RcsCollisionModel_compute(this->cMdl);
      }
    }
    else if (isXMLNodeName(node, "Graph"))
    {
      RCHECK_MSG(this->graph==NULL, "Found xml tag <Graph>, but already graph"
                 " loaded graph from file \"%s\"", this->xmlGraphFile.c_str());
      this->graph = RcsGraph_createFromXmlNode(node);
    }

    node = node->next;
  }

}

/*******************************************************************************
 * Constructor based on a graph object. Takes ownership of the graph.
 ******************************************************************************/
Rcs::ControllerBase::ControllerBase(RcsGraph* graph):
  graph(graph),
  ownsGraph(false),
  xmlRootNode(NULL),
  xmlDoc(NULL),
  cMdl(NULL),
  name("Unknown controller"),
  xmlFile("no-xml-file-available")
{
  // nothing more to do in here.
}

/*******************************************************************************
 * Copy constructor doing deep copying.
 * The xml data is not copied, since this is not present after the
 * construction of the copied constructor.
 ******************************************************************************/
Rcs::ControllerBase::ControllerBase(const ControllerBase& copyFromMe):
  graph(NULL),
  ownsGraph(true),
  xmlRootNode(NULL),
  xmlDoc(NULL),
  cMdl(NULL),
  name(copyFromMe.name),
  xmlFile(copyFromMe.xmlFile),
  taskArrayIdx(copyFromMe.taskArrayIdx)
{
  this->graph = RcsGraph_clone(copyFromMe.graph);

  // clone tasks (index lists are cloned above)
  for (std::vector<Task*>::const_iterator itr = copyFromMe.tasks.begin();
       itr != copyFromMe.tasks.end(); ++itr)
  {
    this->tasks.push_back((*itr)->clone(this->graph));
  }

  // Clone the collision model. If it is NULL, the clone function returns NULL.
  this->cMdl = RcsCollisionModel_clone(copyFromMe.cMdl, this->graph);
}

/*******************************************************************************
 * Assignment operator. Could be done a bit more nice if the tasks had an
 * assignment operator as well. Currently, we just replace them.
 ******************************************************************************/
Rcs::ControllerBase& Rcs::ControllerBase::operator= (const Rcs::ControllerBase& copyFromMe)
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
    delete(this->tasks[i]);
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
Rcs::ControllerBase::~ControllerBase()
{
  // Delete collision model. The function accepts a NULL pointer.
  RcsCollisionModel_destroy(this->cMdl);

  for (size_t i = 0; i < this->tasks.size(); i++)
  {
    delete(this->tasks[i]);
  }

  if (this->ownsGraph == true)
  {
    RcsGraph_destroy(this->graph);
  }
}

/*******************************************************************************
 * Returns the name of the controller as indicated in the xml file.
 ******************************************************************************/
const std::string& Rcs::ControllerBase::getName() const
{
  return this->name;
}

/*******************************************************************************
 * Returns the name of the controller's xml file.
 ******************************************************************************/
const std::string& Rcs::ControllerBase::getXmlFileName() const
{
  return this->xmlFile;
}

/*******************************************************************************
 * Returns the name of a task with the given ID.
 ******************************************************************************/
const std::string& Rcs::ControllerBase::getTaskName(size_t id) const
{
  RCHECK_MSG(id<this->tasks.size(), "id=%zu size=%zu", id, this->tasks.size());
  return this->tasks[id]->getName();
}

/*******************************************************************************
 * Returns the full dimension of a task with the given ID.
 ******************************************************************************/
size_t Rcs::ControllerBase::getTaskDim() const
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
size_t Rcs::ControllerBase::getTaskDim(size_t id) const
{
  RCHECK(id < this->tasks.size());
  return this->tasks[id]->getDim();
}

/*******************************************************************************
 * Returns the dimension of all active tasks.
 ******************************************************************************/
size_t Rcs::ControllerBase::getActiveTaskDim(const MatNd* activation) const
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
int Rcs::ControllerBase::getTaskIndex(const char* name) const
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
int Rcs::ControllerBase::getTaskIndex(const Rcs::Task* task) const
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
size_t Rcs::ControllerBase::getTaskArrayIndex(size_t id) const
{
  RCHECK_MSG(id < this->tasks.size(), "id: %zu   size: %zu",
             id, this->tasks.size());
  return this->taskArrayIdx[id];
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
int Rcs::ControllerBase::getTaskArrayIndex(const char* name) const
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
int Rcs::ControllerBase::getTaskArrayIndex(const Task* task) const
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
const Rcs::Task* Rcs::ControllerBase::getTask(size_t id) const
{
  RCHECK(id < this->tasks.size());
  return this->tasks[id];
}

/*******************************************************************************
 * Returns a pointer to the task with the given id.
 ******************************************************************************/
Rcs::Task* Rcs::ControllerBase::getTask(size_t id)
{
  RCHECK(id < this->tasks.size());
  return this->tasks[id];
}

/*******************************************************************************
 * Returns a const pointer to the task with the given name.
 ******************************************************************************/
const Rcs::Task* Rcs::ControllerBase::getTask(const std::string& name) const
{
  for (size_t i = 0; i < this->tasks.size(); i++)
  {
    const Rcs::Task* t = this->tasks[i];

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
Rcs::Task* Rcs::ControllerBase::getTask(const std::string& name)
{
  for (size_t i = 0; i < this->tasks.size(); i++)
  {
    Rcs::Task* t = this->tasks[i];

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
size_t Rcs::ControllerBase::getNumberOfTasks() const
{
  return this->tasks.size();
}

/*******************************************************************************
 *
 ******************************************************************************/
std::vector<Rcs::Task*> Rcs::ControllerBase::getTasks(const MatNd* a) const
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
std::string Rcs::ControllerBase::getTaskType(size_t id) const
{
  RCHECK(id < this->tasks.size());

  return this->tasks[id]->getClassName();
}

/*******************************************************************************
 * Return the controller graph.
 ******************************************************************************/
RcsGraph* Rcs::ControllerBase::getGraph() const
{
  return this->graph;
}

/*******************************************************************************
 * Return the graph file name (e.g., gScenario.xml).
 ******************************************************************************/
const std::string& Rcs::ControllerBase::getGraphFileName() const
{
  return xmlGraphFile;
}

/*******************************************************************************
 * Return the collision model.
 ******************************************************************************/
RcsCollisionMdl* Rcs::ControllerBase::getCollisionMdl() const
{
  return this->cMdl;
}

/*******************************************************************************
 * Reads the activation vector from the configuration file. TODO: Check if
 * ctor xml file is open and use this in that case.
 ******************************************************************************/
void Rcs::ControllerBase::readActivationsFromXML(MatNd* a_init) const
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
 * Reads the activation vector from the configuration file. TODO: Check if
 * ctor xml file is open and use this in that case.
 ******************************************************************************/
void Rcs::ControllerBase::readActivationVectorFromXML(MatNd* taskVec,
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
 * Reads the activation vector from the configuration file. TODO: Check if
 * ctor xml file is open and use this in that case.
 ******************************************************************************/
void Rcs::ControllerBase::readTaskVectorFromXML(MatNd* x,
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
 * Closes the parsed xml file and destroys all xml memory.
 ******************************************************************************/
void Rcs::ControllerBase::closeXmlFile()
{
  if (this->xmlDoc == NULL)
  {
    this->xmlRootNode = NULL;
    return;
  }

  xmlFreeDoc(this->xmlDoc);
  this->xmlDoc = NULL;
  this->xmlRootNode = NULL;
}

/*******************************************************************************
 * Returns the xml root node of the controller definition file.
 ******************************************************************************/
xmlNodePtr Rcs::ControllerBase::getXmlNode() const
{
  return this->xmlRootNode;
}

/*******************************************************************************
 * Returns the vector of tasks.
 ******************************************************************************/
const std::vector<Rcs::Task*>& Rcs::ControllerBase::taskVec() const
{
  return this->tasks;
}

/*******************************************************************************
 * Task Jacobian.
 ******************************************************************************/
void Rcs::ControllerBase::computeJ(MatNd* J, const MatNd* a_des) const
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
void Rcs::ControllerBase::computeH(MatNd* H, const MatNd* a_des) const
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
void Rcs::ControllerBase::computeJdotQdot(MatNd* JdotQdot,
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
void Rcs::ControllerBase::computeX(MatNd* x, const MatNd* a_des) const
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
void Rcs::ControllerBase::computeDX(MatNd* dx,
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
void Rcs::ControllerBase::computeAx(MatNd* ax,
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
void Rcs::ControllerBase::computeXp(MatNd* x_dot, const MatNd* a_des) const
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
void Rcs::ControllerBase::computeXp_ik(MatNd* x_dot, const MatNd* a_des) const
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
void Rcs::ControllerBase::integrateXp_ik(MatNd* x_res,
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
void Rcs::ControllerBase::computeDXp(MatNd* dx_dot,
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
void Rcs::ControllerBase::computeFfXpp(MatNd* x_ddot_ik,
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
void Rcs::ControllerBase::computeXpp(MatNd* x_ddot,
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
void Rcs::ControllerBase::compressToActive(MatNd* xc,
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
void Rcs::ControllerBase::compressToActiveSelf(MatNd* x,
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
void Rcs::ControllerBase::decompressFromActive(MatNd* x, const MatNd* xc,
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
void Rcs::ControllerBase::decompressFromActiveSelf(MatNd* xc,
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
void Rcs::ControllerBase::decompressActivationToTask(MatNd* x,
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
void Rcs::ControllerBase::decompressActivationToTask(MatNd* x) const
{
  MatNd* a = MatNd_clone(x);
  MatNd_reshape(x, getTaskDim(), 1);
  decompressActivationToTask(x, a);
  MatNd_destroy(a);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
double Rcs::ControllerBase::computeJointlimitCost() const
{
  return RcsGraph_jointLimitCost(this->graph, RcsStateIK);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Rcs::ControllerBase::computeJointlimitGradient(MatNd* grad) const
{
  RcsGraph_jointLimitGradient(this->graph, grad, RcsStateIK);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
double Rcs::ControllerBase::computeJointlimitBorderCost(double borderRatio) const
{
  return RcsGraph_jointLimitBorderCost(this->graph, borderRatio, RcsStateIK);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Rcs::ControllerBase::computeJointlimitBorderGradient(MatNd* grad,
                                                          double borderRatio) const
{
  RcsGraph_jointLimitBorderGradient(this->graph, grad, borderRatio, RcsStateIK);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Rcs::ControllerBase::computeCollisionModel()
{
  RcsCollisionModel_compute(this->cMdl);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
double Rcs::ControllerBase::computeCollisionCost()
{
  computeCollisionModel();
  return RcsCollisionMdl_cost(this->cMdl);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
double Rcs::ControllerBase::getCollisionCost() const
{
  return RcsCollisionMdl_cost(this->cMdl);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Rcs::ControllerBase::computeCollisionGradient(MatNd* grad)
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
void Rcs::ControllerBase::getCollisionGradient(MatNd* grad) const
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
double Rcs::ControllerBase::computeTaskCost(const MatNd* xDes,
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
void Rcs::ControllerBase::computeTaskGradient(MatNd* grad,
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
double Rcs::ControllerBase::computeManipulabilityCost(const MatNd* a_des,
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
double Rcs::ControllerBase::computeManipulabilityGradient(MatNd* grad,
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
bool Rcs::ControllerBase::test(bool verbose)
{
  bool success = true;

  for (size_t id = 0; id < getNumberOfTasks(); id++)
  {
    bool success_i = this->tasks[id]->test(verbose);
    success = success && success_i;

    if ((verbose==true) || (RcsLogLevel>0))
    {
      RMSG("%s when checking task \"%s\" (%s)",
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
void Rcs::ControllerBase::computeTaskForce(MatNd* ft_task,
                                           const double S_ft[6],
                                           const RcsSensor* loadCell,
                                           const MatNd* activation) const
{

  // Transform sensor values into world coordinates
  double I_ft[6];
  MatNd I_ft_ = MatNd_fromPtr(6, 1, I_ft);
  VecNd_copy(I_ft, S_ft, 6);
  Vec3d_transRotateSelf(&I_ft[0], loadCell->offset->rot);
  Vec3d_transRotateSelf(&I_ft[0], loadCell->body->A_BI->rot);
  Vec3d_transRotateSelf(&I_ft[3], loadCell->offset->rot);
  Vec3d_transRotateSelf(&I_ft[3], loadCell->body->A_BI->rot);

  // Compute the sensor Jacobian
  MatNd* J_sensor = NULL;
  MatNd_create2(J_sensor, 6, this->graph->nJ);
  RcsGraph_bodyPointJacobian(getGraph(), loadCell->body, loadCell->offset->org,
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
  Vec3d_transRotateSelf(&I_ft[0], loadCell->offset->rot);
  Vec3d_transRotateSelf(&I_ft[0], loadCell->body->A_BI->rot);
  Vec3d_transRotateSelf(&I_ft[3], loadCell->offset->rot);
  Vec3d_transRotateSelf(&I_ft[3], loadCell->body->A_BI->rot);

  // Compute the sensor Jacobian
  MatNd* J_sensor = NULL;
  MatNd_create2(J_sensor, 6, this->graph->nJ);
  RcsGraph_bodyPointJacobian(getGraph(), loadCell->body, loadCell->offset->org,
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
void Rcs::ControllerBase::computeTaskForce(MatNd* ft_task,
                                               const MatNd* activation) const
{
  size_t nx = activation ? getActiveTaskDim(activation) : getTaskDim();

  MatNd* J_sensor = NULL;
  MatNd_create2(J_sensor, 6, this->graph->nJ);

  MatNd* pinvJ_sensor = NULL;
  MatNd_create2(pinvJ_sensor, J_sensor->n, J_sensor->m);

  MatNd* pinvJF = NULL;
  MatNd_create2(pinvJF, this->graph->nJ, 1);


  RCSGRAPH_TRAVERSE_SENSORS(this->graph)
  {
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
    Vec3d_transRotateSelf(&I_ft[0], loadCell->offset->rot);
    Vec3d_transRotateSelf(&I_ft[0], loadCell->body->A_BI->rot);
    Vec3d_transRotateSelf(&I_ft[3], loadCell->offset->rot);
    Vec3d_transRotateSelf(&I_ft[3], loadCell->body->A_BI->rot);

    // Compute the sensor Jacobian
    RcsGraph_bodyPointJacobian(getGraph(), loadCell->body,
                               loadCell->offset->org, NULL, J_sensor);
    MatNd_reshape(J_sensor, 6, this->graph->nJ);
    MatNd J_rot = MatNd_fromPtr(3, this->graph->nJ,
                                MatNd_getRowPtr(J_sensor, 3));
    RcsGraph_rotationJacobian(this->graph, loadCell->body, NULL, &J_rot);

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
bool Rcs::ControllerBase::add(const ControllerBase& other,
                              const char* suffix,
                              const HTr* A_BP)
{
  bool success = RcsGraph_appendCopyOfGraph(getGraph(), NULL, other.getGraph(),
                                            suffix, A_BP);

  if (success == false)
  {
    return false;
  }

  const std::vector<Task*>& otherTasks = other.taskVec();
  const size_t otherTasksSize = otherTasks.size();

  if (otherTasksSize == 0)
  {
    return true;
  }

  for (size_t i=0; i<otherTasksSize; ++i)
  {
    Task* copyOfOtherTask = otherTasks[i]->clone(this->graph);

    // Set the new task's name uniquely considering the suffix
    if (suffix)
    {
      copyOfOtherTask->setName(otherTasks[i]->getName() + std::string(suffix));
    }

    const RcsBody* effector = copyOfOtherTask->getEffector();
    if (effector != NULL)
    {
      std::string newName;

      if (otherTasks[i]->getEffector())
      {
        newName.assign(otherTasks[i]->getEffector()->name);
      }

      if (suffix)
      {
        newName.append(suffix);
      }

      effector = RcsGraph_getBodyByName(this->graph, newName.c_str());
      RCHECK_MSG(effector, "Not found: %s", newName.c_str());
      copyOfOtherTask->setEffector(effector);
    }

    const RcsBody* refBdy = copyOfOtherTask->getRefBody();
    if (refBdy != NULL)
    {
      std::string newName;

      if (otherTasks[i]->getRefBody())
      {
        newName.assign(otherTasks[i]->getRefBody()->name);
      }

      if (suffix)
      {
        newName.append(suffix);
      }

      refBdy = RcsGraph_getBodyByName(this->graph, newName.c_str());
      RCHECK_MSG(refBdy, "Not found: %s", newName.c_str());
      copyOfOtherTask->setRefBody(refBdy);
    }

    const RcsBody* refFrame = copyOfOtherTask->getRefFrame();
    if (refFrame != NULL)
    {
      std::string newName;

      if (otherTasks[i]->getRefFrame())
      {
        newName.assign(otherTasks[i]->getRefFrame()->name);
      }

      if (suffix)
      {
        newName.append(suffix);
      }

      refFrame = RcsGraph_getBodyByName(this->graph, newName.c_str());
      RCHECK_MSG(refFrame, "Not found: %s", newName.c_str());
      copyOfOtherTask->setRefFrame(refFrame);
    }

    // Rename joint names for TaskJoint
    if (dynamic_cast<TaskJoint*>(copyOfOtherTask))
      {
        TaskJoint* jntTask = dynamic_cast<TaskJoint*>(copyOfOtherTask);
        std::string newName = std::string(jntTask->getJoint()->name);
        if (suffix)
          {
            newName.append(suffix);
          }
        jntTask->setJoint(RcsGraph_getJointByName(getGraph(), newName.c_str()));
        RCHECK_MSG(jntTask->getJoint(), "Not found: joint %s", newName.c_str());
      }
 
    // Rename joint names for TaskJoints
    if (dynamic_cast<TaskJoints*>(copyOfOtherTask))
      {
        RLOG(0, "Copying joints task %s", copyOfOtherTask->getName().c_str());
        TaskJoints* jntsTask = dynamic_cast<TaskJoints*>(copyOfOtherTask);

        for (size_t i=0;i<jntsTask->getNumberOfTasks(); ++i)
          {
            TaskJoint* jntTask = dynamic_cast<TaskJoint*>(jntsTask->getSubTask(i));
            RCHECK(jntTask);
            std::string newName = std::string(jntTask->getJoint()->name);
            if (suffix)
              {
                newName.append(suffix);
              }
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
void Rcs::ControllerBase::add(Task* other)
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
void Rcs::ControllerBase::computeAdmittance(MatNd* compliantFrame,
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
void Rcs::ControllerBase::computeTaskForce(MatNd* ft_task,
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
    Vec3d_transRotateSelf(&I_ft[0], loadCell->offset->rot);
    Vec3d_transRotateSelf(&I_ft[0], loadCell->body->A_BI->rot);
    Vec3d_transRotateSelf(&I_ft[3], loadCell->offset->rot);
    Vec3d_transRotateSelf(&I_ft[3], loadCell->body->A_BI->rot);

    // VecNd_printComment("I_ft", &I_ft[0], 6);

    // Compute the sensor Jacobian
    MatNd J_trans = MatNd_fromPtr(3, J_sensor->n,
                                  MatNd_getRowPtr(J_sensor, 6*nLoadCells));
    RcsGraph_bodyPointJacobian(getGraph(), loadCell->body,
                               loadCell->offset->org, NULL, &J_trans);
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

    // increase filter vector iterator (get next filter respective to next sensor)
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
void Rcs::ControllerBase::printX(const MatNd* x, const MatNd* a_des) const
{
  for (size_t i = 0; i < getNumberOfTasks(); i++)
  {
    size_t row = this->taskArrayIdx[i];

    for (size_t j = 0; j < getTaskDim(i); j++)
    {
      printf("Task \"%s\"[%d]: %f\n", getTaskName(i).c_str(), (int) j,
             MatNd_get(x, row+j, 0));
    }
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::ControllerBase::getModelState(MatNd* q, const char* modelStateName,
                                        int timeStamp)
{
  return RcsGraph_getModelStateFromXML(q, getGraph(), modelStateName,
                                       timeStamp);
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::ControllerBase::checkLimits(bool checkJointLimits,
                                      bool checkCollisions,
                                      bool checkJointVelocities) const
{
  bool success = true;

  // Joint limit check
  if (checkJointLimits)
  {
    bool verbose = (RcsLogLevel >= 3) ? true : false;
    unsigned int aor = RcsGraph_numJointLimitsViolated(getGraph(), verbose);
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
    const double distLimit = 0.001;
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

  // Speed limit check
  if (checkJointVelocities)
  {
    MatNd* q_dot = MatNd_clone(getGraph()->q_dot);
    double scaling = RcsGraph_limitJointSpeeds(getGraph(), q_dot,
                                               1.0, RcsStateFull);
    if (scaling < 1.0)
    {
      success = false;
      RLOG(3, "Joint speed limit violation");
      RCSGRAPH_TRAVERSE_JOINTS(getGraph())
      {
        if ((!JNT->constrained) &&
            (fabs(q_dot->ele[JNT->jointIndex]) >= JNT->speedLimit))
        {
          double sf = RcsJoint_isRotation(JNT) ? 180.0 / M_PI : 1.0;
          RLOG(4, "%s: q_dot=%f   limit=%f [%s]", JNT->name,
               sf*q_dot->ele[JNT->jointIndex],
               sf*JNT->speedLimit, sf == 1.0 ? "m/sec" : "deg/sec");
        }
      }
    }
    MatNd_destroy(q_dot);
  }

  return success;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::ControllerBase::swapTaskVec(std::vector<Task*>& newTasks,
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
 *
 ******************************************************************************/
void Rcs::ControllerBase::print() const
{
  for (size_t i = 0; i < getNumberOfTasks(); i++)
  {
    tasks[i]->print();
  }
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
void Rcs::ControllerBase::computeInvDynJointSpace(MatNd* T_des,
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
  RcsGraph_computeKineticTerms(graph, M, h, g);

  // aq = -kp*(q-q_des)
  MatNd_sub(aq, q_curr, q_des);
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
void Rcs::ControllerBase::computeInvDynJointSpace(MatNd* T_des,
                                                  const RcsGraph* graph,
                                                  const MatNd* q_des,
                                                  double positionGain,
                                                  double velocityGain)
{
  computeInvDynJointSpace(T_des, graph, q_des, NULL, NULL,
                          positionGain, velocityGain);
}
