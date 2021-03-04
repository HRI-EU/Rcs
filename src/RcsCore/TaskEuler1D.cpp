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

#include "TaskEuler1D.h"
#include "TaskEuler3D.h"
#include "TaskFactory.h"
#include "Rcs_typedef.h"
#include "Rcs_macros.h"
#include "Rcs_parser.h"
#include "Rcs_utils.h"
#include "Rcs_kinematics.h"
#include "Rcs_math.h"



static Rcs::TaskFactoryRegistrar<Rcs::TaskEuler1D> registrar1("A");
static Rcs::TaskFactoryRegistrar<Rcs::TaskEuler1D> registrar2("B");
static Rcs::TaskFactoryRegistrar<Rcs::TaskEuler1D> registrar3("C");



/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::TaskEuler1D::TaskEuler1D(const std::string& className, xmlNode* node,
                              RcsGraph* _graph, int dim):
  TaskGenericIK(className, node, _graph, dim)
{

  if (className=="A")
  {
    this->index = 0;
    resetParameter(Parameters(-M_PI, M_PI, (180.0/M_PI), "A [deg]"));
  }
  else if (className=="B")
  {
    this->index = 1;
    resetParameter(Parameters(-M_PI, M_PI, (180.0/M_PI), "B [deg]"));
  }
  else if (className=="C")
  {
    this->index = 2;
    resetParameter(Parameters(-M_PI, M_PI, (180.0/M_PI), "C [deg]"));
  }
  else
  {
    RFATAL("Unknown class name: %s", className.c_str());
  }
}

/*******************************************************************************
 * Copy constructor doing deep copying
 ******************************************************************************/
Rcs::TaskEuler1D::TaskEuler1D(const TaskEuler1D& copyFromMe,
                              RcsGraph* newGraph):
  TaskGenericIK(copyFromMe, newGraph),
  index(copyFromMe.index)
{
}

/*******************************************************************************
 * For programmatic creation
 ******************************************************************************/
Rcs::TaskEuler1D::TaskEuler1D(const std::string& className,
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

  if (getClassName()=="A")
  {
    this->index = 0;
    resetParameter(Task::Parameters(-M_PI, M_PI, 180.0/M_PI, "A [deg]"));
  }
  else if (getClassName()=="B")
  {
    this->index = 1;
    resetParameter(Task::Parameters(-M_PI, M_PI, 180.0/M_PI, "B [deg]"));
  }
  else if (getClassName()=="C")
  {
    this->index = 2;
    resetParameter(Task::Parameters(-M_PI, M_PI, 180.0/M_PI, "C [deg]"));
  }
}

/*******************************************************************************
 * Destructor
 ******************************************************************************/
Rcs::TaskEuler1D::~TaskEuler1D()
{
}

/*******************************************************************************
 * Clone function
 ******************************************************************************/
Rcs::TaskEuler1D* Rcs::TaskEuler1D::clone(RcsGraph* newGraph) const
{
  return new Rcs::TaskEuler1D(*this, newGraph);
}

/*******************************************************************************
 * Computes the current value of the task variable: XYZ Euler angles
 ******************************************************************************/
void Rcs::TaskEuler1D::computeX(double* x_res) const
{
  double ea[3];
  Rcs::TaskEuler3D::computeEulerAngles(ea, getEffector(), getRefBody());
  *x_res = ea[this->index];
}

/*******************************************************************************
 * Euler velocity component
 ******************************************************************************/
void Rcs::TaskEuler1D::computeXp_ik(double* x_res) const
{
  double omega[3], ea[3], eap[3], H[3][3];
  computeOmega(omega);
  computeX(ea);
  Mat3d_getEulerVelocityMatrix(H, ea);
  Vec3d_rotate(eap, H, omega);
  *x_res = eap[this->index];
}

/*******************************************************************************
 * Computes the current Euler angles accelerations
 *
 * app = H (omega_dot - d(H^-1)/dt ap)
 ******************************************************************************/
void Rcs::TaskEuler1D::computeXpp(double* eapp_i, const MatNd* q_ddot) const
{
  // Angular accelerations: omega_dot = Jdot q_dot + J q_ddot
  double ea[3], eap[3], eapp[3];
  computeXpp_ik(eapp, q_ddot);
  computeX(ea);
  computeXp(eap);

  // d(invH)/dt
  double dInvH[3][3];
  Mat3d_getInverseEulerVelocityMatrixDerivative(dInvH, ea, eap);

  double H[3][3];
  Mat3d_getEulerVelocityMatrix(H, ea);

  Vec3d_rotateSelf(eap, dInvH); // d(H^-1)/dt ap
  Vec3d_subSelf(eapp, eap); // (wp - d(H^-1)/dt ap)
  Vec3d_rotateSelf(eapp, H); // H (wp - d(H^-1)/dt ap)

  *eapp_i = eap[this->index];
}

/*******************************************************************************
 *
 * Euler angle Jacobian. It gets singular for cos(beta) = 0 which is
 * periodically happening for pi/2 + k*pi
 *
 * The Euler velocity projection matrices are for instance written out
 * in Bremer, Dynamik und Regelung Mechanischer Systeme, pp.37.
 * For abbreviations alpha=a, beta=b, gamma=c, they are:
 *
 *    / da \   / cos(c)/cos(b)         -sin(c)/cos(b)         0 \  / k_omx \
 *    | db | = | sin(c)                 cos(c)                0 |  | k_omy |
 *    \ dc /   \ -sin(b)cos(c)/cos(b)   sin(c)sin(b)/cos(b)   1 /  \ k_omz /
 *
 *    / da \   / 1   sin(b)*sin(a)/cos(b)  -sin(b)*cos(a)/cos(b) \  / I_omx \
 *    | db | = | 0   cos(a)                 sin(a)               |  | I_omy |
 *    \ dc /   \ 0  -sin(a)/cos(b)          cos(a)/cos(b)        /  \ I_omz /
 *
 ******************************************************************************/
void Rcs::TaskEuler1D::computeJ(MatNd* jacobian) const
{
  const RcsBody* ef = getEffector();
  const RcsBody* refBody = getRefBody();
  const RcsBody* refFrame = getRefFrame();

  // Compute the relative angular velocity Jacobian
  MatNd* J1 = NULL;
  MatNd_create2(J1, 3, this->graph->nJ);
  RcsGraph_3dOmegaJacobian(this->graph, ef, refBody, refFrame, J1);

  // Compute the current rotation matrix
  double A_curr[3][3];
  computeRelativeRotationMatrix(A_curr, ef, refBody);

  // Get Euler angles and set up matrix H (for world-fixed omega:
  // dot(ea) = H * I_om)
  double ea[3], H[3][3];
  Mat3d_toEulerAngles(ea, A_curr);
  Mat3d_getEulerVelocityMatrix(H, ea);

  // Copy the weighted rows on the result
  MatNd_reshape(jacobian, 1, this->graph->nJ);
  MatNd_setZero(jacobian);

  for (int i = 0; i < 3; i++)
  {
    double* row_i = MatNd_getRowPtr(J1, i);
    double H_i    = H[this->index][i];

    if (H_i != 0.0)
    {
      for (unsigned int j = 0; j < this->graph->nJ; j++)
      {
        jacobian->ele[j] += H_i * row_i[j];
      }
    }
  }

  MatNd_destroy(J1);
}

/*******************************************************************************
 * See header
 ******************************************************************************/
void Rcs::TaskEuler1D::computeH(MatNd* hessian) const
{
  RLOG(4, "Hessian computation for task \"%s\" not yet implemented!",
       getName().c_str());
  MatNd_reshapeAndSetZero(hessian, this->graph->nJ, this->graph->nJ);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::TaskEuler1D::isValid(xmlNode* node, const RcsGraph* graph)
{
  std::vector<std::string> classNameVec;
  classNameVec.push_back(std::string("A"));
  classNameVec.push_back(std::string("B"));
  classNameVec.push_back(std::string("C"));

  bool success = Rcs::Task::isValid(node, graph, classNameVec);

  return success;
}
