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

#include "TaskCylindrical1D.h"
#include "TaskFactory.h"
#include "Rcs_typedef.h"
#include "Rcs_macros.h"
#include "Rcs_parser.h"
#include "Rcs_Vec3d.h"



static Rcs::TaskFactoryRegistrar<Rcs::TaskCylindrical1D> registrar1("CylR");
static Rcs::TaskFactoryRegistrar<Rcs::TaskCylindrical1D> registrar2("CylP");



/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::TaskCylindrical1D::TaskCylindrical1D(const std::string& className,
                                          xmlNode* node,
                                          RcsGraph* _graph,
                                          int _dim):
  TaskCylindrical3D(className, node, _graph, _dim),
  index(0)
{

  if (className == "CylR")
  {
    this->index = 0;
    resetParameter(Parameters(0.0, 2.5, 1.0, "Radius [m]"));
  }
  else if (className == "CylP")
  {
    this->index = 1;
    resetParameter(Parameters(-M_PI, M_PI, 180.0/M_PI, "Phi [deg]"));
  }

}

/*******************************************************************************
 * Copy constructor doing deep copying
 ******************************************************************************/
Rcs::TaskCylindrical1D::TaskCylindrical1D(const TaskCylindrical1D& copyFromMe,
                                          RcsGraph* newGraph):
  TaskCylindrical3D(copyFromMe, newGraph),
  index(copyFromMe.index)
{
}

/*******************************************************************************
 * Destructor
 ******************************************************************************/
Rcs::TaskCylindrical1D::~TaskCylindrical1D()
{
}

/*******************************************************************************
 * Clone function
 ******************************************************************************/
Rcs::TaskCylindrical1D* Rcs::TaskCylindrical1D::clone(RcsGraph* newGraph) const
{
  return new Rcs::TaskCylindrical1D(*this, newGraph);
}

/*******************************************************************************
 * Computes the current value of the task variable. We reuse implementation of
 * TaskCylindrical3D, but select only relevant component
 ******************************************************************************/
void Rcs::TaskCylindrical1D::computeX(double* x_res) const
{
  double result_3d[3];
  TaskCylindrical3D::computeX(result_3d);
  x_res[0] = result_3d[this->index];
}

/*******************************************************************************
 * Computes the current velocity in task space. We reuse implementation of
 * TaskCylindrical3D, but select only relevant component
 ******************************************************************************/
void Rcs::TaskCylindrical1D::computeXp(double* xp_res) const
{
  double result_3d[3];
  TaskCylindrical3D::computeXp(result_3d);
  xp_res[0] = result_3d[this->index];
}

/*******************************************************************************
 * Computes current task Jacobian to parameter jacobian. We reuse implementation
 * of TaskCylindrical3D, but select only relevant component
 ******************************************************************************/
void Rcs::TaskCylindrical1D::computeJ(MatNd* jacobian) const
{
  MatNd* Jpos = NULL;
  MatNd_create2(Jpos, 3, this->graph->nJ);
  TaskCylindrical3D::computeJ(Jpos);
  MatNd_getRow(jacobian, this->index, Jpos);
  MatNd_destroy(Jpos);
}

/*******************************************************************************
 * Computes the displacement in task space
 * TaskCylindrical3D::computeDX cannot be reused as it calls
 * TaskCylindrical1D::computeX (virtual function)
 ******************************************************************************/
void Rcs::TaskCylindrical1D::computeDX(double* dx, const double* x_des) const
{
  //
  double x_des_modif = x_des[0];
  if (this->index == 0)
  {
    x_des_modif = fabs(x_des_modif);
  }
  double x_curr;
  computeX(&x_curr);
  dx[0] = x_des_modif - x_curr;

  if (this->index == 1)
  {
    while (dx[0]>M_PI)
    {
      dx[0] -= 2*M_PI;
    }
    while (dx[0]<=-M_PI)
    {
      dx[0] += 2*M_PI;
    }
  }
}

/*******************************************************************************
 * Computes current task Jacobian derivative to parameter \e Jdot
 * Reuse implementation of TaskCylindrical3D, but select only relevant
 * component
 ******************************************************************************/
void Rcs::TaskCylindrical1D::computeJdot(MatNd* Jdot) const
{
  MatNd* JdotPos = NULL;
  MatNd_create2(JdotPos, 3, this->graph->nJ);
  TaskCylindrical3D::computeJdot(JdotPos);
  MatNd_getRow(Jdot, this->index, JdotPos);
  MatNd_destroy(JdotPos);
}

/*******************************************************************************
 * Computes current task Hessian to parameter hessian. We reuse implementation
 * of TaskCylindrical3D, but select only relevant component
 ******************************************************************************/
void Rcs::TaskCylindrical1D::computeH(MatNd* hessian) const
{
  MatNd* H3 = NULL;
  MatNd_create2(H3, 3, this->graph->nJ*this->graph->nJ);
  TaskCylindrical3D::computeH(H3);
  MatNd_getRow(hessian, this->index, H3);
  MatNd_destroy(H3);
  MatNd_reshape(hessian, this->graph->nJ, this->graph->nJ);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::TaskCylindrical1D::isValid(xmlNode* node, const RcsGraph* graph)
{
  std::vector<std::string> classNameVec;
  classNameVec.push_back(std::string("CylR"));
  classNameVec.push_back(std::string("CylP"));

  bool success = Rcs::Task::isValid(node, graph, classNameVec);

  return success;
}
