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

#include "TaskPosition3D.h"
#include "TaskFactory.h"
#include "Rcs_typedef.h"
#include "Rcs_macros.h"
#include "Rcs_parser.h"
#include "Rcs_utils.h"
#include "Rcs_VecNd.h"
#include "Rcs_Vec3d.h"
#include "Rcs_basicMath.h"
#include "Rcs_kinematics.h"



namespace Rcs
{

static TaskFactoryRegistrar<TaskPosition3D> registrar("XYZ");

/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
TaskPosition3D::TaskPosition3D(const std::string& className,
                               xmlNode* node,
                               RcsGraph* graph_,
                               int dim) :
  TaskGenericIK(className, node, graph_, dim)
{
  if (getClassName() == "XYZ")
  {
    double guiMax[3], guiMin[3];
    Vec3d_setElementsTo(guiMax, 2.5);
    Vec3d_setElementsTo(guiMin, -2.5);
    getXMLNodePropertyVec3(node, "guiMax", guiMax);
    getXMLNodePropertyVec3(node, "guiMin", guiMin);
    bool hide = false;
    getXMLNodePropertyBoolString(node, "hide", &hide);
    if (hide)
    {
      Vec3d_setZero(guiMin);
      Vec3d_setZero(guiMax);
    }

    resetParameter(Parameters(guiMin[0], guiMax[0], 1.0, "X [m]"));
    addParameter(Parameters(guiMin[1], guiMax[1], 1.0, "Y [m]"));
    addParameter(Parameters(guiMin[2], guiMax[2], 1.0, "Z [m]"));
  }
}

/*******************************************************************************
 * Copy constructor doing deep copying
 ******************************************************************************/
TaskPosition3D::TaskPosition3D(const TaskPosition3D& src,
                               RcsGraph* newGraph) :
  TaskGenericIK(src, newGraph)
{
}

/*******************************************************************************
 * Constructor based on body pointers
 ******************************************************************************/
TaskPosition3D::TaskPosition3D(RcsGraph* graph_,
                               const RcsBody* effector,
                               const RcsBody* refBdy,
                               const RcsBody* refFrame) : TaskGenericIK()
{
  this->graph = graph_;
  setClassName("XYZ");
  setDim(3);
  setEffector(effector);
  setRefBody(refBdy);
  setRefFrame(refFrame ? refFrame : refBdy);
  resetParameter(Parameters(-2.5, 2.5, 1.0, "X Position [m]"));
  addParameter(Parameters(-2.5, 2.5, 1.0, "Y Position [m]"));
  addParameter(Parameters(-2.5, 2.5, 1.0, "Z Position [m]"));
}

/*******************************************************************************
 * Destructor
 ******************************************************************************/
TaskPosition3D::~TaskPosition3D()
{
}

/*******************************************************************************
 * Clone function
 ******************************************************************************/
TaskPosition3D* TaskPosition3D::clone(RcsGraph* newGraph) const
{
  return new TaskPosition3D(*this, newGraph);
}

/*******************************************************************************
 * See class description for details
 ******************************************************************************/
void TaskPosition3D::computeX(double* I_r) const
{
  const RcsBody* ef = getEffector();
  const RcsBody* refBody = getRefBody();
  const RcsBody* refFrame = getRefFrame();

  // Effector-fixed reference point in world coordinates
  Vec3d_copy(I_r, ef ? ef->A_BI.org : Vec3d_zeroVec());

  // Transform to reference frame: ref_r = A_ref-I * (I_r - I_r_refBdy)
  if (refBody != NULL)
  {
    Vec3d_subSelf(I_r, refBody->A_BI.org);     // I_r -= I_r_refBdy

    // refBody and refFrame, but they differ: refFrame_r = A_refFrame-I*I_r
    if ((refFrame != NULL) && (refFrame != refBody))
    {
      Vec3d_rotateSelf(I_r, (double(*)[3])refFrame->A_BI.rot);
    }
    // refBody and refFrame are the same: refFrame_r = A_refFrame-I*I_r
    else
    {
      Vec3d_rotateSelf(I_r, (double(*)[3])refBody->A_BI.rot);
    }
  }
  // No refBody, but refFrame: Rotate into refFrame coordinates
  else
  {
    // Rotate into refFrame if it exists
    if (refFrame != NULL)
    {
      Vec3d_rotateSelf(I_r, (double(*)[3])refFrame->A_BI.rot);
    }

  }

}

/*******************************************************************************
 * Computes the current velocity in task space:
 *
 * x_dot = A_1I * (x_dot_2 - x_dot_1 + r_12 x omega_1)
 *
 ******************************************************************************/
void TaskPosition3D::computeXp_ik(double* x_dot) const
{
  const RcsBody* ef = getEffector();
  const RcsBody* refBody = getRefBody();
  const RcsBody* refFrame = getRefFrame();

  const double* I_xp_ef = ef ? ef->x_dot : Vec3d_zeroVec();

  if (!refBody)
  {
    Vec3d_copy(x_dot, I_xp_ef);

    if (refFrame)
    {
      Vec3d_rotateSelf(x_dot, (double(*)[3])refFrame->A_BI.rot);
    }
  }
  else
  {
    const double* I_xp_ref = refBody->x_dot;
    Vec3d_sub(x_dot, I_xp_ef, I_xp_ref);

    double r_12[3], eul[3];
    Vec3d_sub(r_12, ef ? ef->A_BI.org : Vec3d_zeroVec(), refBody->A_BI.org);
    Vec3d_crossProduct(eul, r_12, refBody->omega);
    Vec3d_addSelf(x_dot, eul);

    if (refFrame)
    {
      Vec3d_rotateSelf(x_dot, (double(*)[3])refFrame->A_BI.rot);
    }
    else
    {
      Vec3d_rotateSelf(x_dot, (double(*)[3])refBody->A_BI.rot);
    }
  }

}

/*******************************************************************************
 * Computes current task Jacobian to parameter jacobian. See
 * RcsGraph_3dPosJacobian() for details.
 ******************************************************************************/
void TaskPosition3D::computeJ(MatNd* jacobian) const
{
  RcsGraph_3dPosJacobian(this->graph, getEffector(), getRefBody(),
                         getRefFrame(), jacobian);
}

/*******************************************************************************
 * Computes current task Hessian to parameter hessian. See
 * RcsGraph_3dPosHessian() for details.
 ******************************************************************************/
void TaskPosition3D::computeH(MatNd* hessian) const
{
  RcsGraph_3dPosHessian(this->graph, getEffector(), getRefBody(),
                        getRefFrame(), hessian);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool TaskPosition3D::isValid(xmlNode* node, const RcsGraph* graph)
{
  bool success = Task::isValid(node, graph, "XYZ");

  return success;
}

}   // namespace Rcs
