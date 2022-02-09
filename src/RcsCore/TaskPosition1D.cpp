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

#include "TaskPosition1D.h"
#include "TaskFactory.h"
#include "Rcs_typedef.h"
#include "Rcs_parser.h"
#include "Rcs_macros.h"
#include "Rcs_utils.h"
#include "Rcs_VecNd.h"
#include "Rcs_kinematics.h"


static Rcs::TaskFactoryRegistrar<Rcs::TaskPosition1D> registrar1("X");
static Rcs::TaskFactoryRegistrar<Rcs::TaskPosition1D> registrar2("Y");
static Rcs::TaskFactoryRegistrar<Rcs::TaskPosition1D> registrar3("Z");
static Rcs::TaskFactoryRegistrar<Rcs::TaskPosition1D> registrar4("CylZ");



/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::TaskPosition1D::TaskPosition1D(const std::string& className,
                                    xmlNode* node,
                                    RcsGraph* graph_,
                                    int dim):
  TaskPosition3D(className, node, graph_, dim), index(-1)
{
  if ((getClassName() == "X") || (getClassName() == "Y") ||
      (getClassName() == "Z") || (getClassName() == "CylZ"))
  {
    double guiMax = 1.0, guiMin = -1.0;
    getXMLNodePropertyDouble(node, "guiMax", &guiMax);
    getXMLNodePropertyDouble(node, "guiMin", &guiMin);

    bool hide = false;
    getXMLNodePropertyBoolString(node, "hide", &hide);
    if (hide)
    {
      guiMin = 0.0;
      guiMax = 0.0;
    }

    if (getClassName() == "X")
    {
      this->index = 0;
      resetParameter(Parameters(guiMin, guiMax, 1.0, "X [m]"));
    }
    else if (getClassName() == "Y")
    {
      this->index = 1;
      resetParameter(Parameters(guiMin, guiMax, 1.0, "Y [m]"));
    }
    else if ((getClassName() == "Z") || (getClassName() == "CylZ"))
    {
      this->index = 2;
      resetParameter(Parameters(guiMin, guiMax, 1.0, "Z [m]"));
    }

  }   // if (getClassName() ...

}

/*******************************************************************************
 * For programmatic creation
 ******************************************************************************/
Rcs::TaskPosition1D::TaskPosition1D(const std::string& className,
                                    RcsGraph* graph,
                                    const RcsBody* effector,
                                    const RcsBody* refBdy,
                                    const RcsBody* refFrame):
  TaskPosition3D(graph, effector, refBdy, refFrame), index(-1)
{
  setClassName(className);
  setDim(1);

  if (getClassName()=="X")
  {
    this->index = 0;
    resetParameter(Parameters(-2.5, 2.5, 1.0, "X Position [m]"));
  }
  else if (getClassName()=="Y")
  {
    this->index = 1;
    resetParameter(Parameters(-2.5, 2.5, 1.0, "Y Position [m]"));
  }
  else if ((getClassName()=="Z") || (getClassName()=="CylZ"))
  {
    this->index = 2;
    resetParameter(Task::Parameters(-2.5, 2.5, 1.0, "Z Position [m]"));
  }
  else
  {
    RFATAL("Unknown classname \"%s\" - should be X, Y or Z",
           getClassName().c_str());
  }

}

/*******************************************************************************
 * Destructor
 ******************************************************************************/
Rcs::TaskPosition1D::~TaskPosition1D()
{
}

/*******************************************************************************
 * Clone function
 ******************************************************************************/
Rcs::TaskPosition1D* Rcs::TaskPosition1D::clone(RcsGraph* newGraph) const
{
  TaskPosition1D* task = new Rcs::TaskPosition1D(*this);
  task->setGraph(newGraph);
  return task;
}

/*******************************************************************************
 * Computes the current value of the task variable. We reuse implementation of
 * TaskPosition3D, but select only relevant component
 ******************************************************************************/
void Rcs::TaskPosition1D::computeX(double* x_res) const
{
  double result_3d[3];
  TaskPosition3D::computeX(result_3d);
  x_res[0] = result_3d[this->index];
}

/*******************************************************************************
 * Computes the current velocity in task space:
 * ref_x_dot = A_ref-I * (I_x_dot_ef - I_x_dot_ref)
 ******************************************************************************/
void Rcs::TaskPosition1D::computeXp_ik(double* x_dot_res) const
{
  double result_3d[3];
  TaskPosition3D::computeXp_ik(result_3d);
  x_dot_res[0] = result_3d[this->index];
}

/*******************************************************************************
 * Computes current task Jacobian to parameter jacobian. We reuse implementation
 * of TaskPosition3D, but select only relevant component
 ******************************************************************************/
void Rcs::TaskPosition1D::computeJ(MatNd* jacobian) const
{
  MatNd* Jpos = NULL;
  MatNd_create2(Jpos, 3, this->graph->nJ);
  TaskPosition3D::computeJ(Jpos);
  MatNd_getRow(jacobian, this->index, Jpos);
  MatNd_destroy(Jpos);
}

/*******************************************************************************
 * \brief Computes current task Hessian to parameter hessian. See
 *        RcsGraph_3dPosHessian() for details.
 ******************************************************************************/
void Rcs::TaskPosition1D::computeH(MatNd* hessian) const
{
  RcsGraph_1dPosHessian(this->graph, getEffector(), getRefBody(), getRefFrame(),
                        this->index, hessian);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::TaskPosition1D::isValid(xmlNode* node, const RcsGraph* graph)
{
  std::vector<std::string> classNameVec;
  classNameVec.push_back(std::string("X"));
  classNameVec.push_back(std::string("Y"));
  classNameVec.push_back(std::string("Z"));
  classNameVec.push_back(std::string("CylZ"));

  bool success = Rcs::Task::isValid(node, graph, classNameVec);

  return success;
}
