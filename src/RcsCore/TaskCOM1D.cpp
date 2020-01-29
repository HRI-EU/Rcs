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

#include "TaskCOM1D.h"
#include "TaskFactory.h"
#include "Rcs_typedef.h"
#include "Rcs_macros.h"
#include "Rcs_parser.h"
#include "Rcs_utils.h"


static Rcs::TaskFactoryRegistrar<Rcs::TaskCOM1D> registrar1("COGX");
static Rcs::TaskFactoryRegistrar<Rcs::TaskCOM1D> registrar2("COGY");
static Rcs::TaskFactoryRegistrar<Rcs::TaskCOM1D> registrar3("COGZ");



/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::TaskCOM1D::TaskCOM1D(const std::string& className,
                          xmlNode* node,
                          RcsGraph* _graph,
                          int dim):
  Rcs::TaskCOM3D(className, node, _graph, dim),
  index(0)
{

  if (className=="COGX")
  {
    this->index = 0;
    resetParameter(Parameters(-2.5, 2.5, 1.0, "X Position [m]"));
  }
  else if (className=="COGY")
  {
    this->index = 1;
    resetParameter(Parameters(-2.5, 2.5, 1.0, "Y Position [m]"));
  }
  else if (className=="COGZ")
  {
    this->index = 2;
    resetParameter(Parameters(-2.5, 2.5, 1.0, "Z Position [m]"));
  }

}

/*******************************************************************************
 * Copy constructor doing deep copying
 ******************************************************************************/
Rcs::TaskCOM1D::TaskCOM1D(const TaskCOM1D& copyFromMe, RcsGraph* newGraph):
  TaskCOM3D(copyFromMe, newGraph),
  index(copyFromMe.index)
{
}

/*******************************************************************************
 * Destructor
 ******************************************************************************/
Rcs::TaskCOM1D::~TaskCOM1D()
{
}

/*******************************************************************************
 * Clone function
 ******************************************************************************/
Rcs::TaskCOM1D* Rcs::TaskCOM1D::clone(RcsGraph* newGraph) const
{
  return new Rcs::TaskCOM1D(*this, newGraph);
}

/*******************************************************************************
 * Computes the overall center of mass of the system in world coordinates for
 * the respective component. The 3d functions of the parent class are called,
 * and just the relevant elements are copied. Their index is given in
 * this->index (see constructor).
 ******************************************************************************/
void Rcs::TaskCOM1D::computeX(double* x_res) const
{
  double result_3d[3];
  TaskCOM3D::computeX(result_3d);
  *x_res = result_3d[this->index];
}

/*******************************************************************************
 * COM Jacobian for one direction element. The 3d functions of the parent class
 * are called, and just the relevant elements are copied. Their index is given
 * in this->index (see constructor).
 ******************************************************************************/
void Rcs::TaskCOM1D::computeJ(MatNd* jacobian) const
{
  MatNd* J1 = NULL;
  MatNd_create2(J1, 3, this->graph->nJ);
  TaskCOM3D::computeJ(J1);
  MatNd_getRow(jacobian, this->index, J1);
  MatNd_destroy(J1);
}

/*******************************************************************************
 *  COM Hessian for one direction element. The 3d functions of the parent class
 *  are called, and just the relevant elements are copied. Their index is given
 *  in this->index (see constructor).
 ******************************************************************************/
void Rcs::TaskCOM1D::computeH(MatNd* hessian) const
{
  MatNd* H3 = NULL;
  MatNd_create2(H3, 3, this->graph->nJ*this->graph->nJ);
  TaskCOM3D::computeH(H3);
  MatNd_reshape(H3, 3, this->graph->nJ*this->graph->nJ);
  MatNd_getRow(hessian, this->index, H3);
  MatNd_destroy(H3);
  MatNd_reshape(hessian, this->graph->nJ, this->graph->nJ);
}

/*******************************************************************************
 * This task is always valid.
 ******************************************************************************/
bool Rcs::TaskCOM1D::isValid(xmlNode* node, const RcsGraph* graph)
{
  std::vector<std::string> classNameVec;
  classNameVec.push_back(std::string("COGX"));
  classNameVec.push_back(std::string("COGY"));
  classNameVec.push_back(std::string("COGZ"));

  return Rcs::Task::isValid(node, graph, classNameVec);
}
