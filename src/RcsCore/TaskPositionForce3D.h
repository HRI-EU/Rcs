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

#ifndef TASKPOSITIONFORCE3D_H
#define TASKPOSITIONFORCE3D_H


#include "TaskPosition3D.h"


namespace Rcs
{

/*! \ingroup RcsTask
 * \brief This tasks allows to set a 3D force (X, Y, and Z) of an effector
 *
 *  The force can also be relative to another body and reference frame.
 */
class TaskPositionForce3D: public Rcs::TaskPosition3D
{
public:

  /*! Constructor based on xml parsing
   */
  TaskPositionForce3D(const std::string& className, xmlNode* node,
                      RcsGraph* graph, int dim=3);

  /*! \brief Copy constructor doing deep copying with optional new graph pointer
   */
  TaskPositionForce3D(const TaskPositionForce3D& copyFromMe,
                      RcsGraph* newGraph=NULL);

  /*! Destructor
   */
  virtual ~TaskPositionForce3D();

  /*!
   * \brief Virtual copy constructor with optional new graph
   */
  virtual TaskPositionForce3D* clone(RcsGraph* newGraph=NULL) const;

  virtual void computeX(double* x_res) const;

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

  virtual void computeAF(double* ft_res,
                         double* ft_int,
                         const double* ft_des,
                         const double* selection,
                         const double* ft_task,
                         const double a_des,
                         const double kp,
                         const double ki) const;

  virtual void forceTrafo(double* ft_task) const;

  /*! \brief Returns true if the task is specified correctly, false
   *         otherwise. The following checks are performed:
   *         - XML tag "effector" corresponds to body in graph
   *         - XML tag "controlVariable" is "ForceXYZ"
   */
  static bool isValid(xmlNode* node, const RcsGraph* graph);

protected:
  double ft_curr_temp[3];
  double ft_des_temp[3];
  bool force_feedback;
  RcsSensor* fts;
};

}

#endif // TASKPOSITIONFORCE3D_H
