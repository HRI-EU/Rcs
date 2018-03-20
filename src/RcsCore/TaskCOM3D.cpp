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

#include "TaskCOM3D.h"
#include "TaskFactory.h"
#include "Rcs_typedef.h"
#include "Rcs_macros.h"
#include "Rcs_body.h"
#include "Rcs_parser.h"
#include "Rcs_utils.h"
#include "Rcs_kinematics.h"
#include "Rcs_Vec3d.h"


static Rcs::TaskFactoryRegistrar<Rcs::TaskCOM3D> registrar1("COG");


/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::TaskCOM3D::TaskCOM3D(const std::string& className,
                          xmlNode* node,
                          RcsGraph* _graph,
                          int dim):
  Rcs::TaskGenericIK(className, node, _graph, dim)
{
  if (getDim() == 3)
  {
    getParameter(0)->setParameters(-2.5, 2.5, 1.0, "X Position [m]");
    getParameter(1)->setParameters(-2.5, 2.5, 1.0, "Y Position [m]");
    getParameter(2)->setParameters(-2.5, 2.5, 1.0, "Z Position [m]");
  }
}

/*******************************************************************************
 * Copy constructor doing deep copying
 ******************************************************************************/
Rcs::TaskCOM3D::TaskCOM3D(const TaskCOM3D& copyFromMe, RcsGraph* newGraph):
  Rcs::TaskGenericIK(copyFromMe, newGraph)
{
}

/*******************************************************************************
 * Destructor
 ******************************************************************************/
Rcs::TaskCOM3D::~TaskCOM3D()
{
}

/*******************************************************************************
 * Clone function
 ******************************************************************************/
Rcs::TaskCOM3D* Rcs::TaskCOM3D::clone(RcsGraph* newGraph) const
{
  return new Rcs::TaskCOM3D(*this, newGraph);
}

/*******************************************************************************
 * Computes the overall center of mass of the system in world coordinates.
 ******************************************************************************/
void Rcs::TaskCOM3D::computeX(double* x_res) const
{
  if (this->ef)
  {
    RcsGraph_COG_Body(this->ef, x_res);
  }
  else
  {
    RcsGraph_COG(this->graph, x_res);
  }

  if (this->refBody)  // ref_r = A_ref-I * (I_r - I_r_refBody)
  {
    Vec3d_subSelf(x_res, this->refBody->A_BI->org);    // I_r -= I_r_refBody
    Vec3d_rotateSelf(x_res, this->refBody->A_BI->rot); // ref_r = A_ref_I*I_r
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
  if (this->ef != NULL)
  {
    RcsGraph_COGJacobian_Body(this->graph, this->ef, J_cog);
  }
  else
  {
    RcsGraph_COGJacobian(this->graph, J_cog);
  }

  // If a reference body is given, we need to consider both its linear and
  // angular velocity: J = JT_cog - JT_ref + JR_ref x (I_cog-I_ref)
  if (this->refBody != NULL)
  {

    if (RcsBody_isArticulated(this->refBody)==false)
    {
      MatNd_rotateSelf(J_cog, this->refBody->A_BI->rot);
    }
    else
    {
      // J = JT_cog - JT_ref
      MatNd* J_rel = NULL;
      MatNd_create2(J_rel, 3, this->graph->nJ);
      RcsGraph_bodyPointJacobian(this->graph, this->refBody, NULL, NULL,
                                 J_rel);
      MatNd_subSelf(J_cog, J_rel);

      // J = J + JR_ref x (I_cog-I_ref)
      double tmp[3], I_r_cog[3];
      RcsGraph_rotationJacobian(this->graph, this->refBody, NULL, J_rel);

      if (this->ef != NULL)
      {
        RcsGraph_COG_Body(this->ef, I_r_cog);
      }
      else
      {
        RcsGraph_COG(this->graph, I_r_cog);
      }

      Vec3d_sub(tmp, I_r_cog, this->refBody->A_BI->org);
      MatNd_columnCrossProductSelf(J_rel, tmp);
      MatNd_addSelf(J_cog, J_rel);

      // J = A_ref-I * J
      MatNd_rotateSelf(J_cog, this->refBody->A_BI->rot);

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

  if (this->ef != NULL)
  {
    RcsGraph_computeCOGHessian_Body(this->graph, this->ef, H);
  }
  else
  {
    RcsGraph_computeCOGHessian(this->graph, H->ele);
  }

  if (this->refBody != NULL)
  {
    if (RcsBody_isArticulated(this->refBody)==false)
    {
      MatNd_reshape(H, 3, this->graph->nJ*this->graph->nJ);
      MatNd_rotateSelf(H, this->refBody->A_BI->rot);
      MatNd_reshape(H, 3*this->graph->nJ, this->graph->nJ);
    }
    else
    {
      RFATAL("COM Hessian with moving reference body not yet implemented!");
    }
  }

}

/*******************************************************************************
 * See header
 ******************************************************************************/
bool Rcs::TaskCOM3D::isValid(xmlNode* node, const RcsGraph* graph)
{
  return Rcs::Task::isValid(node, graph, "COG");
}
