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

#include "TaskRadial.h"
#include "TaskFactory.h"
#include "Rcs_typedef.h"
#include "Rcs_macros.h"
#include "Rcs_Vec3d.h"
#include "Rcs_basicMath.h"
#include "Rcs_parser.h"



static Rcs::TaskFactoryRegistrar<Rcs::TaskRadial> registrar("Radial");



/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::TaskRadial::TaskRadial(const std::string& className,
                            xmlNode* node,
                            const RcsGraph* _graph,
                            int _dim):
  TaskPosition3D(className, node, _graph, _dim)
{
  if (getClassName()=="Radial")
  {
    resetParameter(Parameters(0, 2.5, 1.0, "Radius [m]"));
  }
}

/*******************************************************************************
 * Clone function
 ******************************************************************************/
Rcs::TaskRadial* Rcs::TaskRadial::clone(const RcsGraph* newGraph) const
{
  TaskRadial* task = new Rcs::TaskRadial(*this);
  task->setGraph(newGraph);
  return task;
}

/*******************************************************************************
 * Computes the current value of the task variable
 * Reuses TaskPosition3D::computeX and then converts to cylindrical coordinates
 ******************************************************************************/
void Rcs::TaskRadial::computeX(double* x_res) const
{
  double pos[3], tmp[3];
  Rcs::TaskPosition3D::computeX(pos);
  Math_Cart2Cyl(pos, &x_res[0], &tmp[1], &tmp[2]);
}

/*******************************************************************************
 * Computes the current velocity in task space
 * Reuses TaskPosition3D::computeXp and then converts to cylindrical coordinates
 ******************************************************************************/
void Rcs::TaskRadial::computeXp(double* xp_res) const
{
  double XYZ[3];
  Rcs::TaskPosition3D::computeX(XYZ);

  double r = sqrt(XYZ[0]*XYZ[0]+XYZ[1]*XYZ[1]);

  double XYZp[3];
  Rcs::TaskPosition3D::computeXp(XYZp);

  xp_res[0] = (XYZ[0]*XYZp[0] + XYZ[1]*XYZp[1])/r;
  //xp_res[1] = (-XYZ[1]*XYZp[0]+XYZ[0]*XYZp[1])/(r*r);
  //xp_res[2] = XYZp[2];
}

/*******************************************************************************
 * \brief Computes current task Jacobian to parameter jacobian.
 *
 * Reuses TaskPosition3D::computeJ and then converts to cylindrical coordinates
 ******************************************************************************/
void Rcs::TaskRadial::computeJ(MatNd* jacobian) const
{
  MatNd* J3 = NULL;
  MatNd_create2(J3, 3, graph->nJ);

  Rcs::TaskPosition3D::computeJ(J3);

  double XYZ[3];
  Rcs::TaskPosition3D::computeX(XYZ);

  double dCyldCart[3][3];
  Math_dCyldCart(dCyldCart, XYZ);
  MatNd trafo = MatNd_fromPtr(1, 3, &dCyldCart[0][0]);
  MatNd_mul(jacobian, &trafo, J3);

  MatNd_destroy(J3);
}

/*******************************************************************************
 * Computes the delta in task space for the differential kinematics.
 * Ensures that always the shortest path in phi is followed
 ******************************************************************************/
void Rcs::TaskRadial::computeDX(double* dx_, const double* x_des) const
{
  double x_des_modif[3];
  Vec3d_copy(x_des_modif, x_des);
  x_des_modif[0] = fabs(x_des_modif[0]);

  double dx[3];
  Rcs::TaskPosition3D::computeDX(dx, x_des_modif);
  while (dx[1]>M_PI)
  {
    dx[1] -= 2*M_PI;
  }
  while (dx[1]<=-M_PI)
  {
    dx[1] += 2*M_PI;
  }

  dx_[0] = dx[0];
}

/*******************************************************************************
 * Computes the Hessian for the cylinder coordinates by applying the chain rule
 * to the
 * J_cyl = A J_pos   =>   H_cyl = A H_pos + dA J_pos
 ******************************************************************************/
void Rcs::TaskRadial::computeH(MatNd* hessian) const
{
  RFATAL("Not yet implemented");
}

/*******************************************************************************
 * Computes the current derivative Jacobian
 * Reuses TaskPosition3D::computeJdot and then converts to cylindrical
 * coordinates
 ******************************************************************************/
void Rcs::TaskRadial::computeJdot(MatNd* Jdot) const
{
  //Jdot = A*JdotPos + Adot*Jpos

  MatNd* jacobianPos = NULL;
  MatNd_create2(jacobianPos, Jdot->m, Jdot->n);

  ////////////////////////////////////////////////////////////////
  // calling Rcs::TaskPosition3D::computeJdot() does not work as it invokes
  // Rcs::TaskRadial::computeH()
  // so we have a copy here
  ////////////////////////////////////////////////////////////////
  size_t nq = this->graph->nJ;
  size_t nx = getDim();
  MatNd* Hessian = NULL;
  MatNd_create2(Hessian, nx*nq, nq);

  Rcs::TaskPosition3D::computeH(Hessian);
  MatNd_reshape(jacobianPos, nx*nq, 1);

  // Get the current joint velocities
  MatNd* q_dot = NULL;
  MatNd_create2(q_dot, this->graph->nJ, 1);
  RcsGraph_stateVectorToIK(this->graph, this->graph->q_dot, q_dot);

  MatNd_mul(jacobianPos, Hessian, q_dot);
  MatNd_reshape(jacobianPos, nx, nq);

  MatNd_destroy(Hessian);
  MatNd_destroy(q_dot);
  ////////////////////////////////////////////////////////////////

  double XYZ[3];
  Rcs::TaskPosition3D::computeX(XYZ);

  double r = sqrt(XYZ[0]*XYZ[0]+XYZ[1]*XYZ[1]);
  double r2 = r*r;

  MatNd* trafo = NULL;
  MatNd_create2(trafo, 3, 3);
  MatNd_setIdentity(trafo);

  MatNd_set2(trafo, 0, 0, XYZ[0]/r);
  MatNd_set2(trafo, 0, 1, XYZ[1]/r);
  MatNd_set2(trafo, 1, 0, -XYZ[1]/r2);
  MatNd_set2(trafo, 1, 1, XYZ[0]/r2);

  MatNd_mul(Jdot, trafo, jacobianPos); // Jdot = A*JdotPos

  Rcs::TaskPosition3D::computeJ(jacobianPos);

  double XYZp[3];
  Rcs::TaskPosition3D::computeXp(XYZp);

  double rDot = (XYZ[0]*XYZp[0] + XYZ[1]*XYZp[1])/r;

  MatNd_setZero(trafo);

  MatNd_set2(trafo, 0, 0, (XYZp[0]*r-rDot*XYZ[0])/r2);
  MatNd_set2(trafo, 0, 1, (XYZp[1]*r-rDot*XYZ[1])/r2);
  MatNd_set2(trafo, 1, 0, -(XYZp[1]*r2-2*r*rDot*XYZ[1])/(r2*r2));
  MatNd_set2(trafo, 1, 1, (XYZp[0]*r2-2*r*rDot*XYZ[0])/(r2*r2));

  MatNd_mulAndAddSelf(Jdot, trafo, jacobianPos); // Jdot += Adot*Jpos

  MatNd_destroy(trafo);
  MatNd_destroy(jacobianPos);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::TaskRadial::isValid(xmlNode* node, const RcsGraph* graph)
{
  bool success = Rcs::Task::isValid(node, graph, "Radial");

  return success;
}

/*******************************************************************************
 * Remove this once we have an implementation for the Hessian.
 ******************************************************************************/
bool Rcs::TaskRadial::testHessian(bool verbose) const
{
  return true;
}
