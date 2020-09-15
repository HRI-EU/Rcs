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

#ifndef RCS_COMPOSITETASK_H
#define RCS_COMPOSITETASK_H

#include "Task.h"



namespace Rcs
{

/*! \ingroup RcsTask
 *  \brief This tasks composes several sub-tasks into one. This class is not
 *         directly accessible through the TaskFactory. The task activation
 *         will be distributed to all sub-tasks, and all task calculations
 *         will be carried out by going through the vector of sub-tasks and
 *         augmenting the result. A number of classes inherit from it, for
 *         instance TaskPose5D or TaskCompositeXml. Please have a look at
 *         their implementation for details.
 */
class CompositeTask: public Task
{
public:


  /*! \brief Calls the constructor of Task
   */
  CompositeTask();

  /*! \brief Calls the constructor of Task with the same arguments
   */
  CompositeTask(const std::string& className, xmlNode* node, RcsGraph* graph);

  /*! \brief Copy constructor doing deep copying with optional new graph
   *         pointer.
   *
   *  \param[in] copyFromMe Task to be copied from.
   *  \param[in] newGraph   If it is not NULL, all tasks's members refer to
   *                        this graph.
   */
  CompositeTask(const CompositeTask& copyFromMe, RcsGraph* newGraph=NULL);

  /*! \brief Constructor based on a graph reference.
   */
  CompositeTask(RcsGraph* graph);

  /*! \brief Deletes all subtasks.
   */
  virtual ~CompositeTask();

  /*! \brief Returns a deep copy of the class created by the copy constructor.
   */
  virtual CompositeTask* clone(RcsGraph* newGraph=NULL) const;

  /*! \brief Adds a sub-task, updates the task dimension, and appends the
   *         sub-tasks parameters.
   */
  virtual void addTask(Task* subTask);

  /*! \brief Removes the sub-task with the given dimension and updates the
   *         task dimension and parameters vector. Returns true for success,
   *         false otherwise.
   */
  virtual bool removeTask(size_t index);

  /*! \brief Calls \ref Task::computeX(double*) const on all subtasks and
   *         stacks the results in vector x_res.
   */
  virtual void computeX(double* x_res) const;

  /*! \brief Calls \ref Task::computeJ(MatNd*) const on all subtasks and
   *         stacks the Jacobians.
   */
  virtual void computeJ(MatNd* jacobian) const;

  /*! \brief Calls \ref Task::computeH(MatNd*) const on all subtasks and
   *         stacks the Hessians.
   */
  virtual void computeH(MatNd* hessian) const;

  /*! \brief Calls \ref Task::computeDX(double*, const double*) const on all
   *         subtasks and stacks the results in vector dx. The current
   *         state will be extracted from the task's graph.
   */
  virtual void computeDX(double* dx, const double* x_des) const;

  /*! \brief Calls \ref Task::computeDX(double*, const double*) const on all
   *         subtasks and stacks the results in vector dx. The current
   *         state will be extracted from x_curr, and not from the
   *         task's graph.
   */
  virtual void computeDX(double* dx, const double* x_des,
                         const double* x_curr) const;

  /*! \brief Calls \ref Task::computeXp(double*) const on all subtasks and
   *         stacks the results in vector xp.
   */
  virtual void computeXp(double* xp) const;

  /*! \brief Calls \ref Task::computeDXp(double*, const double*) const on all
   *         subtasks and stacks the results in vector dxp. The current
   *         state will be extracted from the task's graph.
   */
  virtual void computeDXp(double* dxp, const double* xp_des) const;

  /*! \brief Calls \ref Task::computeXpp(double*, const MatNd*) const on all
   *         subtasks and stacks the results in vector dxpxpp. The current
   *         state will be extracted from the task's graph.
   */
  virtual void computeXpp(double* xpp, const MatNd* qpp) const;

  virtual void computeFfXpp(double* xpp_res, const double* xpp_des) const;
  virtual void integrateXp_ik(double* x_res, const double* x,
                              const double* x_dot, double dt) const;
  virtual void forceTrafo(double* ft_task) const;
  virtual void selectionTrafo(double* S_trans, const double* S) const;

  /*! \brief Sets the effector body to all sub-tasks.
   */
  virtual void setEffector(const RcsBody* effector);

  /*! \brief Sets the reference body to all sub-tasks.
   */
  virtual void setRefBody(const RcsBody* referenceBody);

  /*! \brief Sets the reference frame body to all sub-tasks.
   */
  virtual void setRefFrame(const RcsBody* referenceFrame);

  /*! \brief Returns the index-th task of the composite task. If index is out
   *         of range, the default exception for accessing vector elements
   *         out of range will be thrown.
   */
  virtual const Task* getSubTask(size_t index) const;

  /*! \brief See \ref getSubTask(size_t) const
   */
  virtual Task* getSubTask(size_t index);

  /*! \brief Calls the print function of all sub-tasks.
   */
  virtual void print() const;

  /*! \brief Returns the size of the subTask vector.
   */
  virtual size_t getNumberOfTasks() const;

protected:

  std::vector<Task*> subTask;
};


}


#endif // RCS_COMPOSITETASK_H
