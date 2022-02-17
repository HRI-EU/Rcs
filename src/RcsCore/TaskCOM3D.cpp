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

#include "TaskCOM3D.h"
#include "TaskFactory.h"
#include "Rcs_typedef.h"
#include "Rcs_macros.h"
#include "Rcs_body.h"
#include "Rcs_utils.h"
#include "Rcs_kinematics.h"
#include "Rcs_Vec3d.h"


static Rcs::TaskFactoryRegistrar<Rcs::TaskCOM3D> registrar1("COG");


/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::TaskCOM3D::TaskCOM3D(const std::string& className,
                          xmlNode* node,
                          const RcsGraph* _graph,
                          int dim):
  Rcs::TaskGenericIK(className, node, _graph, dim)
{
  if (getClassName()=="COG")
  {
    resetParameter(Parameters(-2.5, 2.5, 1.0, "X Position [m]"));
    addParameter(Parameters(-2.5, 2.5, 1.0, "Y Position [m]"));
    addParameter(Parameters(-2.5, 2.5, 1.0, "Z Position [m]"));
  }
}

/*******************************************************************************
 * Clone function
 ******************************************************************************/
Rcs::TaskCOM3D* Rcs::TaskCOM3D::clone(const RcsGraph* newGraph) const
{
  TaskCOM3D* task = new Rcs::TaskCOM3D(*this);
  task->setGraph(newGraph);
  return task;
}

/*******************************************************************************
 * Computes the overall center of mass of the system in world coordinates.
 ******************************************************************************/
void Rcs::TaskCOM3D::computeX(double* x_res) const
{
  const RcsBody* ef = getEffector();
  const RcsBody* refBody = getRefBody();

  if (ef)
  {
    RcsGraph_COG_Body(this->graph, ef, x_res);
  }
  else
  {
    RcsGraph_COG(this->graph, x_res);
  }

  if (refBody)  // ref_r = A_ref-I * (I_r - I_r_refBody)
  {
    Vec3d_subSelf(x_res, refBody->A_BI.org);    // I_r -= I_r_refBody
    Vec3d_rotateSelf(x_res, (double (*)[3])refBody->A_BI.rot); // ref_r = A_ref_I*I_r
  }
}

/*******************************************************************************
 * Computes current task Jacobian to parameter jacobian
 ******************************************************************************/
void Rcs::TaskCOM3D::computeJ(MatNd* jacobian) const
{
  MatNd* J_cog = NULL;
  MatNd_create2(J_cog, 3, this->graph->nJ);

  // Compute COG Jacobian and COG
  const RcsBody* ef = getEffector();
  if (ef != NULL)
  {
    RcsGraph_COGJacobian_Body(this->graph, ef, J_cog);
  }
  else
  {
    RcsGraph_COGJacobian(this->graph, J_cog);
  }

  // If a reference body is given, we need to consider both its linear and
  // angular velocity: J = JT_cog - JT_ref + JR_ref x (I_cog-I_ref)
  const RcsBody* refBody = getRefBody();
  if (refBody != NULL)
  {
    if (RcsBody_isArticulated(this->graph, refBody)==false)
    {
      MatNd_rotateSelf(J_cog, (double (*)[3])refBody->A_BI.rot);
    }
    else
    {
      // J = JT_cog - JT_ref
      MatNd* J_rel = NULL;
      MatNd_create2(J_rel, 3, this->graph->nJ);
      RcsGraph_bodyPointJacobian(this->graph, refBody, NULL, NULL, J_rel);
      MatNd_subSelf(J_cog, J_rel);

      // J = J + JR_ref x (I_cog-I_ref)
      double tmp[3], I_r_cog[3];
      RcsGraph_rotationJacobian(this->graph, refBody, NULL, J_rel);

      if (ef != NULL)
      {
        RcsGraph_COG_Body(this->graph, ef, I_r_cog);
      }
      else
      {
        RcsGraph_COG(this->graph, I_r_cog);
      }

      Vec3d_sub(tmp, I_r_cog, refBody->A_BI.org);
      MatNd_columnCrossProductSelf(J_rel, tmp);
      MatNd_addSelf(J_cog, J_rel);

      // J = A_ref-I * J
      MatNd_rotateSelf(J_cog, (double (*)[3])refBody->A_BI.rot);

      MatNd_destroy(J_rel);
    }

  }

  MatNd_reshapeCopy(jacobian, J_cog);

  MatNd_destroy(J_cog);
}

/*******************************************************************************
 * Computes current task Hessian
 ******************************************************************************/
void Rcs::TaskCOM3D::computeH(MatNd* H) const
{
  MatNd_reshape(H, 3*this->graph->nJ, this->graph->nJ);

  const RcsBody* ef = getEffector();
  if (ef != NULL)
  {
    RcsGraph_computeCOGHessian_Body(this->graph, ef, H);
  }
  else
  {
    RcsGraph_computeCOGHessian(this->graph, H->ele);
  }

  const RcsBody* refBody = getRefBody();
  if (refBody != NULL)
  {
    if (RcsBody_isArticulated(this->graph, refBody)==false)
    {
      MatNd_reshape(H, 3, this->graph->nJ*this->graph->nJ);
      MatNd_rotateSelf(H, (double (*)[3])refBody->A_BI.rot);
      MatNd_reshape(H, 3*this->graph->nJ, this->graph->nJ);
    }
    else
    {
      RFATAL("COM Hessian with moving reference body not yet implemented!");
    }
  }

}

/*******************************************************************************
 * \todo: Check mass > 0
 ******************************************************************************/
bool Rcs::TaskCOM3D::isValid(xmlNode* node, const RcsGraph* graph)
{
  return Rcs::Task::isValid(node, graph, "COG");
}
