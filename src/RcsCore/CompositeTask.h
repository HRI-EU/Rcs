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

class CompositeTask: public Task
{
public:


  /*! \brief Default constructor
   */
  CompositeTask();

  /*! \brief Constructor based on xml parsing
   */
  CompositeTask(const std::string& className, xmlNode* node, RcsGraph* graph);

  /*! \brief Copy constructor doing deep copying with optional new graph
   *         pointer
   */
  CompositeTask(const CompositeTask& copyFromMe, RcsGraph* newGraph=NULL);

  /*! \brief Constructor based on a graph reference
   */
  CompositeTask(RcsGraph* graph);

  /*! \brief Destructor
   */
  virtual ~CompositeTask();

  /*! \brief Virtual copy constructor with optional new graph
   */
  virtual CompositeTask* clone(RcsGraph* newGraph=NULL) const;

  virtual void addTask(Task* subTask);

  /*! \brief Returns the dimension of the task.
   */
  virtual unsigned int getDim() const;

  virtual void computeX(double* x_res) const;
  virtual void computeJ(MatNd* jacobian) const;
  virtual void computeH(MatNd* hessian) const;

  virtual void computeDX(double* dx, const double* x_des) const;
  virtual void computeDX(double* dx, const double* x_des,
                         const double* x_curr) const;
  virtual void computeXp(double* xp) const;
  virtual void computeDXp(double* dxp, const double* xp_des) const;
  virtual void computeXpp(double* xpp, const MatNd* qpp) const;
  virtual void computeFfXpp(double* xpp_res, const double* xpp_des) const;
  virtual void integrateXp_ik(double* x_res, const double* x,
                              const double* x_dot, double dt) const;
  virtual void forceTrafo(double* ft_task) const;
  virtual void selectionTrafo(double* S_trans, const double* S) const;

  virtual void setEffector(const RcsBody* effector);
  virtual void setRefBody(const RcsBody* referenceBody);
  virtual void setRefFrame(const RcsBody* referenceFrame);

protected:

  std::vector<Task*> subTask;
};


}


#endif // RCS_COMPOSITETASK_H
