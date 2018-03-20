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

#ifndef RCS_TASKCOLLISION_H
#define RCS_TASKCOLLISION_H

#include "TaskGenericIK.h"


namespace Rcs
{

/*! \ingroup RcsTask
 *  \brief Collision avoidance task.
 */
class TaskCollision: public Rcs::TaskGenericIK
{
public:

  /*! Constructor based on xml parsing
   */
  TaskCollision(const std::string& className, xmlNode* node,
                RcsGraph* graph, int dim=1);

  /*! \brief Copy constructor, sharing the same graph.
   */
  TaskCollision(const TaskCollision& copyFromMe);

  /*! \brief Copy constructor doing deep copying with new graph pointer
   */
  TaskCollision(const TaskCollision& copyFromMe, RcsGraph* newGraph);

  /*! Destructor
   */
  virtual ~TaskCollision();

  /*!
   * \brief Virtual copy constructor with optional new graph
   */
  virtual TaskCollision* clone(RcsGraph* newGraph=NULL) const;

  /*! \brief Computes the current value of the task variable
   *
   *  \param[out] x_res The result of the calculation
   */
  virtual void computeX(double* x_res) const;

  /*! \brief Computes current task Jacobian to parameter \param jacobian
   *
   *  Returns false for empty Jacobian.
   */
  virtual void computeJ(MatNd* jacobian) const;

  /*! \brief Computes the collision Hessian to parameter \e hessian
   */
  virtual void computeH(MatNd* hessian) const;

  /*! \brief Returns always true.
   */
  static bool isValid(xmlNode* node, const RcsGraph* graph);

protected:
  RcsPair** pair;

private:
  TaskCollision();
  TaskCollision& operator=(const TaskCollision&);
  void copyCollisionModel(RcsPair** srcPair, const RcsGraph* newGraph);
};

}

#endif // RCS_TASKCOLLISION_H
