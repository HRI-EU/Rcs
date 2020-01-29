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

#include "TaskOmega1D.h"
#include "TaskFactory.h"
#include "Rcs_typedef.h"
#include "Rcs_macros.h"
#include "Rcs_kinematics.h"
#include "Rcs_VecNd.h"
#include "Rcs_parser.h"


static Rcs::TaskFactoryRegistrar<Rcs::TaskOmega1D> registrar1("Ad");
static Rcs::TaskFactoryRegistrar<Rcs::TaskOmega1D> registrar2("Bd");
static Rcs::TaskFactoryRegistrar<Rcs::TaskOmega1D> registrar3("Cd");


/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::TaskOmega1D::TaskOmega1D(const std::string& className_,
                              xmlNode* node,
                              RcsGraph* _graph,
                              int dim):
  TaskGenericIK(className_, node, _graph, dim), index(-1)
{
  double guiMax = M_PI, guiMin = -M_PI;

  if (getDim() == 1)
  {
    getXMLNodePropertyDouble(node, "guiMax", &guiMax);
    getXMLNodePropertyDouble(node, "guiMin", &guiMin);
  }

  if (getClassName()=="Ad")
  {
    this->index = 0;
    resetParameter(Task::Parameters(guiMin, guiMax, 180.0/M_PI, "Ad [deg/sec]"));
  }
  else if (getClassName()=="Bd")
  {
    this->index = 1;
    resetParameter(Task::Parameters(guiMin, guiMax, 180.0/M_PI, "Bd [deg/sec]"));
  }
  else if (getClassName()=="Cd")
  {
    this->index = 2;
    resetParameter(Task::Parameters(guiMin, guiMax, 180.0/M_PI, "Cd [deg/sec]"));
  }

  // Other tasks inherit from this. Therefore we can't do any hard checking
  // on classNames etc.

}

/*******************************************************************************
 * For programmatic creation
 ******************************************************************************/
Rcs::TaskOmega1D::TaskOmega1D(const std::string& className,
                              RcsGraph* graph_,
                              const RcsBody* effector,
                              const RcsBody* refBdy,
                              const RcsBody* refFrame):
  TaskGenericIK(), index(-1)
{
  this->graph = graph_;
  setClassName(className);
  setDim(1);
  setEffector(effector);
  setRefBody(refBdy);
  setRefFrame(refFrame ? refFrame : refBdy);

  if (getClassName()=="Ad")
  {
    this->index = 0;
    resetParameter(Parameters(-M_PI_2, M_PI_2, 180.0/M_PI, "Ad [deg/sec]"));
  }
  else if (getClassName()=="Bd")
  {
    this->index = 1;
    resetParameter(Parameters(-M_PI_2, M_PI_2, 180.0/M_PI, "Bd [deg/sec]"));
  }
  else if (getClassName()=="Cd")
  {
    this->index = 2;
    resetParameter(Parameters(-M_PI_2, M_PI_2, 180.0/M_PI, "Cd [deg/sec]"));
  }
}

/*******************************************************************************
 * Copy constructor doing deep copying
 ******************************************************************************/
Rcs::TaskOmega1D::TaskOmega1D(const TaskOmega1D& copyFromMe,
                              RcsGraph* newGraph):
  TaskGenericIK(copyFromMe, newGraph), index(copyFromMe.index)
{
}

/*******************************************************************************
 * Destructor
 ******************************************************************************/
Rcs::TaskOmega1D::~TaskOmega1D()
{
}

/*******************************************************************************
 * Clone function
 ******************************************************************************/
Rcs::TaskOmega1D* Rcs::TaskOmega1D::clone(RcsGraph* newGraph) const
{
  return new Rcs::TaskOmega1D(*this, newGraph);
}

/*******************************************************************************
 * Omega Jacobian.
 ******************************************************************************/
void Rcs::TaskOmega1D::computeJ(MatNd* jacobian) const
{
  // Compute the relative angular velocity Jacobian
  MatNd* J1 = NULL;
  MatNd_create2(J1, 3, this->graph->nJ);
  RcsGraph_3dOmegaJacobian(this->graph, this->ef, this->refBody,
                           this->refFrame, J1);

  // Copy the row to the result
  MatNd_reshape(jacobian, 1, this->graph->nJ);
  MatNd_getRow(jacobian, this->index, J1);
  MatNd_destroy(J1);
}

/*******************************************************************************
 * Omega Jacobian.
 ******************************************************************************/
void Rcs::TaskOmega1D::computeH(MatNd* hessian) const
{
  const unsigned int nq = this->graph->nJ;
  const unsigned int nqSqr =nq*nq;

  // Compute the relative angular velocity Hessian
  MatNd* H3 = NULL;
  MatNd_create2(H3, 3*nq, nq);
  RcsGraph_3dOmegaHessian(this->graph, this->ef, this->refBody, this->refFrame,
                          H3);

  // Copy the row to the result
  MatNd_reshape(H3, 3, nqSqr);
  MatNd_reshape(hessian, 1, nqSqr);
  MatNd_getRow(hessian, this->index, H3);
  MatNd_destroy(H3);
  MatNd_reshape(hessian, nq, nq);
}

/*******************************************************************************
 * Computes the current value of the task variable: XYZ Euler angles
 ******************************************************************************/
void Rcs::TaskOmega1D::computeX(double* x_res) const
{
  double temp;
  this->computeXp(&temp);
  *x_res = temp;
}

/*******************************************************************************
 * Computes the delta in task space for the differential kinematics. That's a
 * bit hacky. Shouldn't we better have a function to set the desired velocities?
 ******************************************************************************/
void Rcs::TaskOmega1D::computeDX(double* dx, const double* x_des) const
{
  *dx = *x_des;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::TaskOmega1D::isValid(xmlNode* node, const RcsGraph* graph)
{
  std::vector<std::string> classNameVec;
  classNameVec.push_back(std::string("Ad"));
  classNameVec.push_back(std::string("Bd"));
  classNameVec.push_back(std::string("Cd"));

  bool success = Rcs::Task::isValid(node, graph, classNameVec);

  return success;
}
