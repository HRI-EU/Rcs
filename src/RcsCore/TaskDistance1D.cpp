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

#include "TaskDistance1D.h"
#include "TaskFactory.h"
#include "Rcs_typedef.h"
#include "Rcs_macros.h"
#include "Rcs_parser.h"


static Rcs::TaskFactoryRegistrar<Rcs::TaskDistance1D> registrar1("DistanceX");
static Rcs::TaskFactoryRegistrar<Rcs::TaskDistance1D> registrar2("DistanceY");
static Rcs::TaskFactoryRegistrar<Rcs::TaskDistance1D> registrar3("DistanceZ");


/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::TaskDistance1D::TaskDistance1D(const std::string& className_,
                                    xmlNode* node,
                                    RcsGraph* _graph,
                                    int dim):
  TaskDistance3D(className_, node, _graph, dim), index(-1)
{
  if (getClassName()=="DistanceX")
  {
    this->index = 0;
    getParameter(0)->name.assign("X [m]");
  }
  else if (getClassName()=="DistanceY")
  {
    this->index = 1;
    getParameter(0)->name.assign("Y [m]");
  }
  else if (getClassName()=="DistanceZ")
  {
    this->index = 2;
    getParameter(0)->name.assign("Z [m]");
  }

}

/*******************************************************************************
 * Copy constructor doing deep copying
 ******************************************************************************/
Rcs::TaskDistance1D::TaskDistance1D(const TaskDistance1D& copyFromMe,
                                    RcsGraph* newGraph):
  TaskDistance3D(copyFromMe, newGraph),
  index(copyFromMe.index)
{
}

/*******************************************************************************
* Constructor based on body pointers
******************************************************************************/
//! \todo Memory leak when derieved class calls params.clear()
Rcs::TaskDistance1D::TaskDistance1D(RcsGraph* graph_,
                                    const RcsBody* effector,
                                    const RcsBody* refBdy,
                                    int idx) :
  TaskDistance3D(graph_, effector, refBdy), index(idx)
{
  switch (idx)
    {
    case 0:
      setClassName("DistanceX");
      break;

    case 1:
      setClassName("DistanceY");
      break;

    case 2:
      setClassName("DistanceZ");
      break;

    default:
      RFATAL("Unsupported index %d - should be 0, 1 or 2", idx);
    }

  setName("Dist1D " + std::string(effector ? effector->name : "NULL") + "-"
          + std::string(refBdy ? refBdy->name : NULL));
  setDim(1);
  std::vector<Parameters*>& params = getParameters();
  params.clear();
  params.push_back(new Task::Parameters(-1.0, 1.0, 1.0, "Dist [m]"));
}

/*******************************************************************************
 * Destructor
 ******************************************************************************/
Rcs::TaskDistance1D::~TaskDistance1D()
{
}

/*******************************************************************************
 * Clone function
 ******************************************************************************/
Rcs::TaskDistance1D* Rcs::TaskDistance1D::clone(RcsGraph* newGraph) const
{
  return new Rcs::TaskDistance1D(*this, newGraph);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskDistance1D::computeX(double* x_res) const
{
  double result_3d[3];
  TaskDistance3D::computeX(result_3d);
  x_res[0] = result_3d[this->index];
}

/*******************************************************************************
 * Computes current task Jacobian to parameter jacobian. See
 * RcsGraph_3dPosJacobian() for details.
 ******************************************************************************/
void Rcs::TaskDistance1D::computeJ(MatNd* jacobian) const
{
  MatNd* Jpos = NULL;
  MatNd_create2(Jpos, 3, this->graph->nJ);
  TaskDistance3D::computeJ(Jpos);
  MatNd_getRow(jacobian, this->index, Jpos);
  MatNd_destroy(Jpos);
}

/*******************************************************************************
 * Computes current task Hessian to parameter hessian. See
 * RcsGraph_3dPosHessian() for details.
 ******************************************************************************/
void Rcs::TaskDistance1D::computeH(MatNd* hessian) const
{
  int n = graph->nJ, nn = n * n;
  MatNd* H3 = NULL;
  MatNd_create2(H3, 3 * n, n);

  TaskDistance3D::computeH(H3);

  MatNd_reshape(hessian, n, n);
  MatNd slice = MatNd_fromPtr(n, n, &H3->ele[this->index*nn]);
  MatNd_copy(hessian, &slice);
  //memcpy(hessian->ele, &H3->ele[index * nn], nn * sizeof(double));
  MatNd_destroy(H3);
}

/*******************************************************************************
 * This task required an effector and a reference body.
 ******************************************************************************/
bool Rcs::TaskDistance1D::isValid(xmlNode* node, const RcsGraph* graph)
{
  std::vector<std::string> classNameVec;
  classNameVec.push_back(std::string("DistanceX"));
  classNameVec.push_back(std::string("DistanceY"));
  classNameVec.push_back(std::string("DistanceZ"));

  bool success = Rcs::Task::isValid(node, graph, classNameVec);

  char taskName[256] = "Unnamed task";
  getXMLNodePropertyStringN(node, "name", taskName, 256);

  // This task requires an effector and a reference body
  if (getXMLNodeProperty(node, "effector")==false)
  {
    RLOG(3, "Task \"%s\" requires \"effector\", none found", taskName);
    success = false;
  }

  if ((getXMLNodeProperty(node, "refBdy")==false) &&
      (getXMLNodeProperty(node, "refBody")==false))
  {
    RLOG(3, "Task \"%s\" requires \"refBdy\" or \"refBody\", none found",
         taskName);
    success = false;
  }

  return success;
}
