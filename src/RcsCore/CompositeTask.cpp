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

#include "CompositeTask.h"
#include "Rcs_typedef.h"
#include "Rcs_macros.h"



/*******************************************************************************
 * Default constructor
 ******************************************************************************/
Rcs::CompositeTask::CompositeTask(): Task()
{
}

/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::CompositeTask::CompositeTask(const std::string& className,
                                  xmlNode* node,
                                  RcsGraph* _graph):
  Task(className, node, _graph, 0)
{
}

/*******************************************************************************
 * Copy constructor doing deep copying
 ******************************************************************************/
Rcs::CompositeTask::CompositeTask(const Rcs::CompositeTask& copyFromMe,
                                  RcsGraph* newGraph):
  Task(copyFromMe, newGraph)
{
  // The addTask method assigns the parameter list, so we need to delete it
  // before going trough te sub tasks.
  clearParameters();

  for (size_t i=0; i<copyFromMe.subTask.size(); ++i)
  {
    addTask(copyFromMe.subTask[i]->clone(newGraph));
  }
}

/*******************************************************************************
 * Constructor based on a graph reference
 ******************************************************************************/
Rcs::CompositeTask::CompositeTask(RcsGraph* _graph): Task()
{
  this->graph = _graph;
}

/*******************************************************************************
 * Destructor
 ******************************************************************************/
Rcs::CompositeTask::~CompositeTask()
{

  for (size_t i=0; i<subTask.size(); ++i)
  {
    delete this->subTask[i];
  }
}

/*******************************************************************************
 * Clone function
 ******************************************************************************/
Rcs::CompositeTask* Rcs::CompositeTask::clone(RcsGraph* newGraph) const
{
  return new Rcs::CompositeTask(*this, newGraph);
}

/*******************************************************************************
 * Clone function
 ******************************************************************************/
void Rcs::CompositeTask::addTask(Task* tsk)
{
  this->subTask.push_back(tsk);

  // Update the Task's private taskDim member.
  unsigned int compositeDim = 0;

  for (size_t i=0; i<subTask.size(); ++i)
  {
    compositeDim += subTask[i]->getDim();
  }

  setDim(compositeDim);

  // Add a parameter class instance for each sub-task. This is required since
  // the parents constructor is called with dimension 0 (We can't know it at
  // that point), and therefore doesn't create the parameter class instances
  // in the CompositeTask's constructor.

  // Copy the parameters of the subtasks to the CompositeTask so that they will
  // be properly displayed in the Guis etc.
  for (size_t j=0; j<tsk->getParameters().size(); ++j)
  {
    addParameter(tsk->getParameter(j));
  }

}

/*******************************************************************************
 * Clone function
 ******************************************************************************/
bool Rcs::CompositeTask::removeTask(size_t index)
{
  if (index > subTask.size() - 1)
  {
    RLOG_CPP(1, "Failed to erase task with index " << index
             << " - should be less than " << subTask.size());
    return false;
  }

  delete subTask[index];
  this->subTask.erase(subTask.begin() + index);

  // Update the task's private taskDim member.
  unsigned int compositeDim = 0;

  for (size_t i = 0; i < subTask.size(); ++i)
  {
    compositeDim += subTask[i]->getDim();
  }

  setDim(compositeDim);

  removeParameter(index);

  return true;
}

/*******************************************************************************
 * Compute task vector over all subtasks
 ******************************************************************************/
void Rcs::CompositeTask::computeX(double* x_res) const
{
  double* resPtr = x_res;

  for (size_t i=0; i<subTask.size(); ++i)
  {
    subTask[i]->computeX(resPtr);
    resPtr += subTask[i]->getDim();
  }
}

/*******************************************************************************
 * Compute Jacobian over all subtasks
 ******************************************************************************/
void Rcs::CompositeTask::computeJ(MatNd* J) const
{
  MatNd_reshapeAndSetZero(J, getDim(), this->graph->nJ);
  unsigned int rowIdx = 0;

  for (size_t i=0; i<subTask.size(); ++i)
  {
    unsigned int subTaskDim = subTask[i]->getDim();
    MatNd subJ = MatNd_fromPtr(subTaskDim, this->graph->nJ,
                               MatNd_getRowPtr(J, rowIdx));
    subTask[i]->computeJ(&subJ);
    rowIdx += subTaskDim;
  }
}

/*******************************************************************************
 * Compute Hessian over all subtasks
 ******************************************************************************/
void Rcs::CompositeTask::computeH(MatNd* H) const
{
  unsigned int nq = this->graph->nJ;
  unsigned int rowIdx = 0;

  MatNd_reshapeAndSetZero(H, getDim(), nq*nq);

  for (size_t i=0; i<subTask.size(); ++i)
  {
    unsigned int subTaskDim = subTask[i]->getDim();
    MatNd subH = MatNd_fromPtr(subTaskDim, nq*nq, MatNd_getRowPtr(H, rowIdx));
    subTask[i]->computeH(&subH);
    rowIdx += subTaskDim;
  }

  MatNd_reshape(H, getDim()*nq, nq);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::CompositeTask::computeDX(double* dx, const double* x_des) const
{
  unsigned int rowIdx = 0;

  for (size_t i=0; i<subTask.size(); ++i)
  {
    subTask[i]->computeDX(&dx[rowIdx], &x_des[rowIdx]);
    rowIdx += subTask[i]->getDim();
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::CompositeTask::computeDX(double* dx, const double* x_des,
                                   const double* x_curr) const
{
  unsigned int rowIdx = 0;

  for (size_t i=0; i<subTask.size(); ++i)
  {
    subTask[i]->computeDX(&dx[rowIdx], &x_des[rowIdx], &x_curr[rowIdx]);
    rowIdx += subTask[i]->getDim();
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::CompositeTask::computeXp(double* x_dot) const
{
  unsigned int rowIdx = 0;

  for (size_t i=0; i<subTask.size(); ++i)
  {
    subTask[i]->computeXp(&x_dot[rowIdx]);
    rowIdx += subTask[i]->getDim();
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::CompositeTask::computeDXp(double* dx_dot, const double* x_dot_des) const
{
  unsigned int rowIdx = 0;

  for (size_t i=0; i<subTask.size(); ++i)
  {
    subTask[i]->computeDXp(&dx_dot[rowIdx], &x_dot_des[rowIdx]);
    rowIdx += subTask[i]->getDim();
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::CompositeTask::computeXpp(double* x_ddot, const MatNd* q_ddot) const
{
  unsigned int rowIdx = 0;

  for (size_t i=0; i<subTask.size(); ++i)
  {
    subTask[i]->computeXpp(&x_ddot[rowIdx], q_ddot);
    rowIdx += subTask[i]->getDim();
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::CompositeTask::computeFfXpp(double* x_ddot, const double* x_ddot_des) const
{
  unsigned int rowIdx = 0;

  for (size_t i=0; i<subTask.size(); ++i)
  {
    subTask[i]->computeFfXpp(&x_ddot[rowIdx], &x_ddot_des[rowIdx]);
    rowIdx += subTask[i]->getDim();
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::CompositeTask::integrateXp_ik(double* x_res, const double* x,
                                        const double* x_dot, double dt) const
{
  unsigned int rowIdx = 0;

  for (size_t i=0; i<subTask.size(); ++i)
  {
    subTask[i]->integrateXp_ik(&x_res[rowIdx], &x[rowIdx], &x_dot[rowIdx], dt);
    rowIdx += subTask[i]->getDim();
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::CompositeTask::forceTrafo(double* ft_task) const
{
  unsigned int rowIdx = 0;

  for (size_t i=0; i<subTask.size(); ++i)
  {
    subTask[i]->forceTrafo(&ft_task[rowIdx]);
    rowIdx += subTask[i]->getDim();
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::CompositeTask::selectionTrafo(double* S_trans, const double* S) const
{
  unsigned int rowIdx = 0;

  for (size_t i=0; i<subTask.size(); ++i)
  {
    subTask[i]->selectionTrafo(&S_trans[rowIdx], &S[rowIdx]);
    rowIdx += subTask[i]->getDim();
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::CompositeTask::setEffector(const RcsBody* effector)
{
  for (size_t i=0; i<subTask.size(); ++i)
  {
    subTask[i]->setEffector(effector);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::CompositeTask::setRefBody(const RcsBody* referenceBody)
{
  for (size_t i=0; i<subTask.size(); ++i)
  {
    subTask[i]->setRefBody(referenceBody);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::CompositeTask::setRefFrame(const RcsBody* referenceFrame)
{
  for (size_t i=0; i<subTask.size(); ++i)
  {
    subTask[i]->setRefFrame(referenceFrame);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
const Rcs::Task* Rcs::CompositeTask::getSubTask(size_t index) const
{
  return this->subTask[index];
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::Task* Rcs::CompositeTask::getSubTask(size_t index)
{
  return this->subTask[index];
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::CompositeTask::print() const
{
  printf("CompositeTask %s: type %s with %zu sub-tasks\n",
         getName().c_str(), getClassName().c_str(), subTask.size());

  for (size_t i=0; i<subTask.size(); ++i)
  {
    printf("   Sub-task %zd: ", i);
    subTask[i]->print();
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
size_t Rcs::CompositeTask::getNumberOfTasks() const
{
  return this->subTask.size();
}
