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

#include "TaskPosition2D.h"
#include "TaskFactory.h"
#include "Rcs_typedef.h"
#include "Rcs_parser.h"
#include "Rcs_macros.h"
#include "Rcs_utils.h"
#include "Rcs_VecNd.h"
#include "Rcs_kinematics.h"


static Rcs::TaskFactoryRegistrar<Rcs::TaskPosition2D> registrar1("XY");
static Rcs::TaskFactoryRegistrar<Rcs::TaskPosition2D> registrar2("XZ");
static Rcs::TaskFactoryRegistrar<Rcs::TaskPosition2D> registrar3("YZ");



/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::TaskPosition2D::TaskPosition2D(const std::string& className,
                                    xmlNode* node,
                                    const RcsGraph* graph_,
                                    int dim):
  TaskPosition3D(className, node, graph_, dim), index1(-1), index2(-1)
{
  if ((getClassName() == "XY") || (getClassName() == "XZ") ||
      (getClassName() == "YZ"))
  {
    double guiMax[2], guiMin[2];
    VecNd_setElementsTo(guiMax, 2.5, 2);
    VecNd_setElementsTo(guiMin, -2.5, 2);
    getXMLNodePropertyVec2(node, "guiMax", guiMax);
    getXMLNodePropertyVec2(node, "guiMin", guiMin);
    bool hide = false;
    getXMLNodePropertyBoolString(node, "hide", &hide);
    if (hide)
    {
      VecNd_setZero(guiMin, 2);
      VecNd_setZero(guiMax, 2);
    }


    if (getClassName() == "XY")
    {
      this->index1 = 0;
      this->index2 = 1;
      resetParameter(Parameters(guiMin[0], guiMax[0], 1.0, "X [m]"));
      addParameter(Parameters(guiMin[1], guiMax[1], 1.0, "Y [m]"));
    }
    else if (getClassName() == "XZ")
    {
      this->index1 = 0;
      this->index2 = 2;
      resetParameter(Parameters(guiMin[0], guiMax[0], 1.0, "X [m]"));
      addParameter(Parameters(guiMin[1], guiMax[1], 1.0, "Z [m]"));
    }
    else if ((getClassName() == "YZ"))
    {
      this->index1 = 1;
      this->index2 = 2;
      resetParameter(Parameters(guiMin[0], guiMax[0], 1.0, "Y [m]"));
      addParameter(Parameters(guiMin[1], guiMax[1], 1.0, "Z [m]"));
    }

  }   // if (getClassName() ...

}

/*******************************************************************************
 * For programmatic creation
 ******************************************************************************/
Rcs::TaskPosition2D::TaskPosition2D(const std::string& className,
                                    const RcsGraph* graph,
                                    const RcsBody* effector,
                                    const RcsBody* refBdy,
                                    const RcsBody* refFrame):
  TaskPosition3D(graph, effector, refBdy, refFrame), index1(-1), index2(-1)
{
  setClassName(className);
  setDim(2);

  if (getClassName()=="XY")
  {
    this->index1 = 0;
    this->index2 = 1;
    resetParameter(Parameters(-2.5, 2.5, 1.0, "X Position [m]"));
    addParameter(Parameters(-2.5, 2.5, 1.0, "Y [m]"));
  }
  else if (getClassName()=="XZ")
  {
    this->index1 = 0;
    this->index2 = 2;
    resetParameter(Parameters(-2.5, 2.5, 1.0, "X Position [m]"));
    addParameter(Parameters(-2.5, 2.5, 1.0, "Z [m]"));
  }
  else if ((getClassName()=="YZ"))
  {
    this->index1 = 1;
    this->index2 = 2;
    resetParameter(Task::Parameters(-2.5, 2.5, 1.0, "Y Position [m]"));
    addParameter(Parameters(-2.5, 2.5, 1.0, "Z [m]"));
  }
  else
  {
    RFATAL("Unknown classname \"%s\" - should be XY, XZ or YZ",
           getClassName().c_str());
  }
}

/*******************************************************************************
 * Clone function
 ******************************************************************************/
Rcs::TaskPosition2D* Rcs::TaskPosition2D::clone(const RcsGraph* newGraph) const
{
  TaskPosition2D* task = new Rcs::TaskPosition2D(*this);
  task->setGraph(newGraph);
  return task;
}

/*******************************************************************************
 * Computes the current value of the task variable. We reuse implementation of
 * TaskPosition3D, but select only relevant component
 ******************************************************************************/
void Rcs::TaskPosition2D::computeX(double* x_res) const
{
  double result_3d[3];
  TaskPosition3D::computeX(result_3d);
  x_res[0] = result_3d[this->index1];
  x_res[1] = result_3d[this->index2];
}

/*******************************************************************************
 * Computes the current velocity in task space:
 * ref_x_dot = A_ref-I * (I_x_dot_ef - I_x_dot_ref)
 ******************************************************************************/
void Rcs::TaskPosition2D::computeXp_ik(double* x_dot_res) const
{
  double result_3d[3];
  TaskPosition3D::computeXp_ik(result_3d);
  x_dot_res[0] = result_3d[this->index1];
  x_dot_res[1] = result_3d[this->index2];
}

/*******************************************************************************
 * Computes current task Jacobian to parameter jacobian. We reuse implementation
 * of TaskPosition3D, but select only relevant component
 ******************************************************************************/
void Rcs::TaskPosition2D::computeJ(MatNd* jacobian) const
{
  MatNd* Jpos = NULL;
  MatNd_create2(Jpos, 3, this->graph->nJ);
  TaskPosition3D::computeJ(Jpos);
  MatNd_copyRow(jacobian, 0, Jpos, this->index1);
  MatNd_copyRow(jacobian, 1, Jpos, this->index2);
  MatNd_destroy(Jpos);
}

/*******************************************************************************
 * \brief Computes current task Hessian to parameter hessian. See
 *        RcsGraph_3dPosHessian() for details.
 ******************************************************************************/
void Rcs::TaskPosition2D::computeH(MatNd* hessian) const
{
  int n = graph->nJ, nn = n * n;
  MatNd* H3 = NULL;
  MatNd_create2(H3, 3 * n, n);
  RcsGraph_3dPosHessian(graph, getEffector(), getRefBody(), getRefFrame(), H3);

  MatNd_reshape(hessian, 2*n, n);
  memcpy(&hessian->ele[0], &H3->ele[index1 * nn], nn * sizeof(double));
  memcpy(&hessian->ele[nn], &H3->ele[index2 * nn], nn * sizeof(double));
  MatNd_destroy(H3);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::TaskPosition2D::isValid(xmlNode* node, const RcsGraph* graph)
{
  std::vector<std::string> classNameVec;
  classNameVec.push_back(std::string("XY"));
  classNameVec.push_back(std::string("XZ"));
  classNameVec.push_back(std::string("YZ"));

  bool success = Rcs::Task::isValid(node, graph, classNameVec);

  return success;
}
