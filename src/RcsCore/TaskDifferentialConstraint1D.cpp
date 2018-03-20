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

#include "TaskDifferentialConstraint1D.h"
#include "TaskFactory.h"
#include "Rcs_typedef.h"
#include "Rcs_macros.h"
#include "Rcs_parser.h"
#include "Rcs_Vec3d.h"
#include "Rcs_kinematics.h"



static Rcs::TaskFactoryRegistrar<Rcs::TaskDifferentialConstraint1D> registrar1("DiffConstraintX");
static Rcs::TaskFactoryRegistrar<Rcs::TaskDifferentialConstraint1D> registrar2("DiffConstraintY");
static Rcs::TaskFactoryRegistrar<Rcs::TaskDifferentialConstraint1D> registrar3("DiffConstraintZ");



/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::TaskDifferentialConstraint1D::
TaskDifferentialConstraint1D(const std::string& taskType,
                             xmlNode* node,
                             RcsGraph* _graph,
                             int dim):
  TaskGenericIK(taskType, node, _graph, dim),
  index(0)
{

  if (taskType=="DiffConstraintX")
  {
    this->index = 0;
    getParameter(0)->setParameters(-1.0, 1.0, 1.0, "Delta x [m]");
  }
  else if (taskType=="DiffConstraintY")
  {
    this->index = 1;
    getParameter(0)->setParameters(-1.0, 1.0, 1.0, "Delta y [m]");
  }
  else if (taskType=="DiffConstraintZ")
  {
    this->index = 2;
    getParameter(0)->setParameters(-1.0, 1.0, 1.0, "Delta z [m]");
  }

}

/*******************************************************************************
 * Copy constructor doing deep copying
 ******************************************************************************/
Rcs::TaskDifferentialConstraint1D::
TaskDifferentialConstraint1D(const TaskDifferentialConstraint1D& src,
                             RcsGraph* newGraph):
  TaskGenericIK(src, newGraph),
  index(src.index)
{
}

/*******************************************************************************
 * Destructor
 ******************************************************************************/
Rcs::TaskDifferentialConstraint1D::~TaskDifferentialConstraint1D()
{
}

/*******************************************************************************
 * Clone function
 ******************************************************************************/
Rcs::TaskDifferentialConstraint1D*
Rcs::TaskDifferentialConstraint1D::clone(RcsGraph* newGraph) const
{
  return new Rcs::TaskDifferentialConstraint1D(*this, newGraph);
}

/*******************************************************************************
 * Computes the current value of the task variable: The position difference:
 *
 * x = A_FI (I_r_ef - I_r_ref)
 ******************************************************************************/
void Rcs::TaskDifferentialConstraint1D::computeX(double* I_r) const
{
  double F_x_ef[3], F_x_ref[3], diff[3];

  Vec3d_sub(F_x_ef, this->ef->A_BI->org, this->refFrame->A_BI->org);
  Vec3d_sub(F_x_ref, this->refBody->A_BI->org, this->refFrame->A_BI->org);
  Vec3d_add(diff, F_x_ef, F_x_ref);
  Vec3d_rotateSelf(diff, this->refFrame->A_BI->rot);

  *I_r = diff[this->index];
}

/*******************************************************************************
 * Computes the task velocity:
 *
 * A: refFrame
 * B: refBody
 * C: effector
 *
 * refBody speed wrt. refFrame:
 * A_v_AB = A_AI { (I_v_B - I_v_A) + I_om_A x I_r_AB }
 *
 * Effector speed wrt. refFrame:
 * A_v_AC = A_AI { (I_v_C - I_v_A) + I_om_A x I_r_AC }
 *
 * x = A_v_B + A_v_C
 *   = A_AI { (I_v_C + I_v_B - 2 I_v_A) + I_om_A x (I_r_AB + I_r_AC) }
 *
 ******************************************************************************/
void Rcs::TaskDifferentialConstraint1D::computeXp(double* xp_res) const
{
  const double* I_v_A  = this->refFrame->x_dot;
  const double* I_v_B  = this->refBody->x_dot;
  const double* I_v_C  = this->ef->x_dot;
  const double* I_om_A = this->refFrame->omega;
  const double* I_r_B  = this->refBody->A_BI->org;
  const double* I_r_A  = this->refFrame->A_BI->org;
  const double* I_r_C  = this->ef->A_BI->org;

  // refBody velocity
  double A_v_AB[3], A_v_AC[3], tmp1[3], tmp2[3], tmp3[3];

  Vec3d_sub(tmp1, I_v_B, I_v_A);             // I_v_B - I_v_A
  Vec3d_sub(tmp2, I_r_B, I_r_A);             // I_r_AB
  Vec3d_crossProduct(tmp3, I_om_A, tmp2);    // I_om_A x I_r_AB
  Vec3d_addSelf(tmp3, tmp1);                 // I_v_B - I_v_A + I_om_A x I_r_AB
  Vec3d_rotate(A_v_AB, this->refFrame->A_BI->rot, tmp3);

  // effector velocity
  Vec3d_sub(tmp1, I_v_C, I_v_A);             // I_v_C - I_v_A
  Vec3d_sub(tmp2, I_r_C, I_r_A);             // I_r_AC
  Vec3d_crossProduct(tmp3, I_om_A, tmp2);    // I_om_A x I_r_AC
  Vec3d_addSelf(tmp3, tmp1);                 // I_v_C - I_v_A + I_om_A x I_r_AC
  Vec3d_rotate(A_v_AC, this->refFrame->A_BI->rot, tmp3);

  *xp_res = A_v_AC[this->index] + A_v_AB[this->index];
}

/*******************************************************************************
 * Computes current task Jacobian for the constraint equation A_v_B + A_v_C = 0
 *
 * A: refFrame
 * B: refBody
 * C: effector
 *
 * J = A_AI (JB + JC - 2 JA) + A_AI JRA x (I_r_AB + I_r_AC)
 *
 * J = A_AI (JB - JA) + A_AI (JC - JA) + A_AI JRA x I_r_AB + A_AI JRA x I_r_AC
 *   = A_AI (JB - JA + JRA x I_r_AB) + A_AI (JC - JA + JRA x I_r_AC)
 ******************************************************************************/
void Rcs::TaskDifferentialConstraint1D::computeJ(MatNd* jacobian) const
{
  MatNd* J_buf = NULL;
  MatNd_create2(J_buf, 1, graph->nJ);

  RcsGraph_1dPosJacobian(graph, refBody, refFrame, refFrame, index, J_buf);
  RcsGraph_1dPosJacobian(graph, ef, refFrame, refFrame, index, jacobian);
  MatNd_addSelf(jacobian, J_buf);

  MatNd_destroy(J_buf);
}

/****************************************************************************
 * \brief see header
 ***************************************************************************/
void Rcs::TaskDifferentialConstraint1D::computeH(MatNd* hessian) const
{
  MatNd* H_buf = NULL;
  MatNd_create2(H_buf, 1, this->graph->nJ*this->graph->nJ);

  RcsGraph_1dPosHessian(graph, refBody, refFrame, refFrame, index, H_buf);
  RcsGraph_1dPosHessian(graph, ef, refFrame, refFrame, index, hessian);
  MatNd_addSelf(hessian, H_buf);

  MatNd_destroy(H_buf);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::TaskDifferentialConstraint1D::isValid(xmlNode* node,
                                                const RcsGraph* graph)
{
  std::vector<std::string> classNameVec;
  classNameVec.push_back(std::string("DiffConstraintX"));
  classNameVec.push_back(std::string("DiffConstraintY"));
  classNameVec.push_back(std::string("DiffConstraintZ"));

  bool success = Rcs::Task::isValid(node, graph, classNameVec);

  if (success == false)
  {
    return false;
  }

  char taskName[256];
  strcpy(taskName, "unnamed task");
  getXMLNodePropertyStringN(node, "name", taskName, 256);

  // Check if effector exists
  char tag[256];
  getXMLNodePropertyStringN(node, "effector", tag, 256);
  RcsBody* ef = RcsGraph_getBodyByName(graph, tag);

  if (ef==NULL)
  {
    RLOG(3, "Task \"%s\": Effector (tag \"effector\") \"%s\" not found",
         taskName, tag);
    return false;
  }

  getXMLNodePropertyStringN(node, "refBdy", tag, 256);
  RcsBody* refBdy = RcsGraph_getBodyByName(graph, tag);

  if (refBdy==NULL)
  {
    RLOG(3, "Task \"%s\": Reference body (tag \"refBdy\") \"%s\" not found",
         taskName, tag);
    return false;
  }

  getXMLNodePropertyStringN(node, "refFrame", tag, 256);
  RcsBody* refFrame = RcsGraph_getBodyByName(graph, tag);

  if (refFrame==NULL)
  {
    RLOG(3, "Task \"%s\": Reference body (tag \"refBdy\") \"%s\" not found",
         taskName, tag);
    return false;
  }

  // Check that the 3 bodies are different.
  if ((ef==refBdy) || (ef==refFrame) || (refBdy==refFrame))
  {
    RLOG(3, "Task \"%s\": Bodies must be different, but effector=\"%s\", "
         "refBdy=\"%s\" and refFrame=\"%s\"",
         taskName, ef->name, refBdy->name, refFrame->name);

  }

  return true;
}
