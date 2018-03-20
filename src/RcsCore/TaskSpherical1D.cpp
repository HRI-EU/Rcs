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

#include "TaskSpherical1D.h"
#include "TaskFactory.h"
#include "Rcs_typedef.h"
#include "Rcs_parser.h"
#include "Rcs_macros.h"
#include "Rcs_Vec3d.h"



static Rcs::TaskFactoryRegistrar<Rcs::TaskSpherical1D> registrar1("SphR");
static Rcs::TaskFactoryRegistrar<Rcs::TaskSpherical1D> registrar2("SphT");
static Rcs::TaskFactoryRegistrar<Rcs::TaskSpherical1D> registrar3("SphP");



/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::TaskSpherical1D::TaskSpherical1D(const std::string& className_,
                                      xmlNode* node,
                                      RcsGraph* _graph,
                                      int dim):
  TaskSpherical3D(className_, node, _graph, dim)
{

  if (getClassName()=="SphR")
  {
    this->index = 0;
    getParameter(0)->setParameters(0, 2.5, 1.0, "Radius [m]");
  }
  else if (getClassName()=="SphT")
  {
    this->index = 1;
    getParameter(0)->setParameters(0., M_PI, 180.0/M_PI, "Theta [deg]");
  }
  else if (getClassName()=="SphP")
  {
    this->index = 2;
    getParameter(0)->setParameters(-M_PI, M_PI, 180.0/M_PI, "Phi [deg]");
  }
  else
  {
    RFATAL("Unknown class name: %s", getClassName().c_str());
  }

}

/*******************************************************************************
 * Copy constructor doing deep copying
 ******************************************************************************/
Rcs::TaskSpherical1D::TaskSpherical1D(const TaskSpherical1D& copyFromMe,
                                      RcsGraph* newGraph):
  TaskSpherical3D(copyFromMe, newGraph),
  index(copyFromMe.index)
{
}

/*******************************************************************************
 * Destructor
 ******************************************************************************/
Rcs::TaskSpherical1D::~TaskSpherical1D()
{
}

/*******************************************************************************
 * Clone function
 ******************************************************************************/
Rcs::TaskSpherical1D* Rcs::TaskSpherical1D::clone(RcsGraph* newGraph) const
{
  return new Rcs::TaskSpherical1D(*this, newGraph);
}

/*******************************************************************************
 * Computes the current value of the task variable. We reuse implementation of
 * TaskSpherical3D, but select only relevant component
 ******************************************************************************/
void Rcs::TaskSpherical1D::computeX(double* x_res) const
{
  double result_3d[3];
  TaskSpherical3D::computeX(result_3d);
  x_res[0] = result_3d[this->index];
}

/*******************************************************************************
 * Computes the current velocity in task space. We reuse implementation of
 * TaskSpherical3D, but select only relevant component
 ******************************************************************************/
void Rcs::TaskSpherical1D::computeXp(double* xp_res) const
{
  double result_3d[3];
  TaskSpherical3D::computeXp(result_3d);
  xp_res[0] = result_3d[this->index];
}

/*******************************************************************************
 * Computes current task Jacobian to parameter jacobian. We reuse implementation
 * of TaskSpherical3D, but select only relevant component
 ******************************************************************************/
void Rcs::TaskSpherical1D::computeJ(MatNd* jacobian) const
{
  MatNd* Jpos = NULL;
  MatNd_create2(Jpos, 3, this->graph->nJ);
  TaskSpherical3D::computeJ(Jpos);
  MatNd_getRow(jacobian, this->index, Jpos);
  MatNd_destroy(Jpos);
}

/*******************************************************************************
 * Computes the displacement in task space
 * TaskSpherical3D::computeDX cannot be reused as it calls
 * TaskSpherical1D::computeX (virtual function)
 ******************************************************************************/
void Rcs::TaskSpherical1D::computeDX(double* dx, const double* x_des) const
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

  if (this->index == 1 || this->index == 2)
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
 * Reuse implementation of TaskSpherical3D, but select only relevant
 * component
 ******************************************************************************/
void Rcs::TaskSpherical1D::computeJdot(MatNd* Jdot) const
{
  MatNd* JdotPos = NULL;
  MatNd_create2(JdotPos, 3, this->graph->nJ);
  TaskSpherical3D::computeJdot(JdotPos);
  MatNd_getRow(Jdot, this->index, JdotPos);
  MatNd_destroy(JdotPos);
}

/*******************************************************************************
 * Computes current task Hessian to parameter hessian. We reuse implementation
 * of TaskSpherical3D, but select only relevant component
 ******************************************************************************/
void Rcs::TaskSpherical1D::computeH(MatNd* hessian) const
{
  MatNd* H3 = NULL;
  MatNd_create2(H3, 3, this->graph->nJ*this->graph->nJ);
  TaskSpherical3D::computeH(H3);
  MatNd_getRow(hessian, this->index, H3);
  MatNd_destroy(H3);
  MatNd_reshape(hessian, this->graph->nJ, this->graph->nJ);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::TaskSpherical1D::isValid(xmlNode* node, const RcsGraph* graph)
{
  std::vector<std::string> classNameVec;
  classNameVec.push_back(std::string("SphR"));
  classNameVec.push_back(std::string("SphT"));
  classNameVec.push_back(std::string("SphP"));

  bool success = Rcs::Task::isValid(node, graph, classNameVec);

  return success;
}
