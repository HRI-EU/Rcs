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

#ifndef RCS_TASKJOINT_H
#define RCS_TASKJOINT_H

#include "TaskGenericIK.h"


namespace Rcs
{

/*! \ingroup RcsTask
 * \brief Control an individual joint angle
 *
 *  This task creates a task Jacobian with a single row of zeros
 *  and a single one at the column of the controlled DoF
 */
class TaskJoint: public Rcs::TaskGenericIK
{
public:

  /*! Constructor based on xml parsing
   */
  TaskJoint(const std::string& className, xmlNode* node, RcsGraph* graph,
            int dim=1);

  /*! \brief Copy constructor doing deep copying with optional new graph
   *         pointer
   */
  TaskJoint(const TaskJoint& copyFromMe, RcsGraph* newGraph=NULL);

  /*! Constructor for tasks using the joint pointer directly.
   */
  TaskJoint(RcsJoint* joint, xmlNode* node, RcsGraph* graph=NULL);

  /*! Destructor
   */
  virtual ~TaskJoint();

  /*!
   * \brief Virtual copy constructor with optional new graph
   */
  virtual TaskJoint* clone(RcsGraph* newGraph=NULL) const;

  /*! \brief Computes the current value of the task variable
   *
   *  The result is written to parameter \e x_res.
   */
  virtual void computeX(double* x_res) const;

  /*! \brief Computes the current joint velocity.
   *
   *  \param[out] xp_curr Joint velocity
   */
  virtual void computeXp(double* xp_curr) const;

  /*! \brief Computes current task Jacobian to parameter \e jacobian
   */
  virtual void computeJ(MatNd* jacobian) const;

  /*! \brief Computes current task Hessian to parameter \e hessian
   */
  virtual void computeH(MatNd* hessian) const;

  /*! \brief Returns the pointer to the RcsJoints
   */
  RcsJoint* getJoint() const;

  /*! \brief Returns true if the task is specified correctly, false
   *         otherwise:
   *         - Xml tag "controlVariable" is not "Joint"
   *         - Joint with the tag "jnt" must exist in the graph
   */
  static bool isValid(xmlNode* node, const RcsGraph* graph);

  /*! \brief See \ref TaskGenericIK::computeAX
   */
  virtual void computeAX(double* a_res,
                         double* integral_x,
                         const double* x_des,
                         const double* xp_des,
                         const double* xpp_des,
                         const double* S_des,
                         const double a_des,
                         const double kp,
                         const double kd,
                         const double ki) const;

protected:
  RcsJoint* joint;         //!< Joint for single joint tasks
  bool rad_diff_jnt_fb;    //!< Joint error based on radian feedback
};

}

#endif // RCS_TASKJOINT_H
