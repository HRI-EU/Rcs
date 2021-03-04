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

#include "TaskCylindrical3D.h"
#include "TaskFactory.h"
#include "Rcs_typedef.h"
#include "Rcs_macros.h"
#include "Rcs_Vec3d.h"
#include "Rcs_basicMath.h"
#include "Rcs_parser.h"



static Rcs::TaskFactoryRegistrar<Rcs::TaskCylindrical3D> registrar("CylRPZ");



/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::TaskCylindrical3D::TaskCylindrical3D(const std::string& className,
                                          xmlNode* node,
                                          RcsGraph* _graph,
                                          int _dim):
  TaskPosition3D(className, node, _graph, _dim)
{
  if (getClassName()=="CylRPZ")
  {
    resetParameter(Parameters(0, 2.5, 1.0, "Radius [m]"));
    addParameter(Parameters(-M_PI, M_PI, 180.0/M_PI, "Phi [deg]"));
    addParameter(Parameters(-2.5, 2.5, 1.0, "Z Position [m]"));
  }
}

/*******************************************************************************
 * Copy constructor doing deep copying
 ******************************************************************************/
Rcs::TaskCylindrical3D::TaskCylindrical3D(const TaskCylindrical3D& src,
                                          RcsGraph* newGraph):
  TaskPosition3D(src, newGraph)
{
}

/*******************************************************************************
 * Destructor
 ******************************************************************************/
Rcs::TaskCylindrical3D::~TaskCylindrical3D()
{
}

/*******************************************************************************
 * Clone function
 ******************************************************************************/
Rcs::TaskCylindrical3D* Rcs::TaskCylindrical3D::clone(RcsGraph* newGraph) const
{
  return new Rcs::TaskCylindrical3D(*this, newGraph);
}

/*******************************************************************************
 * Computes the current value of the task variable
 * Reuses TaskPosition3D::computeX and then converts to cylindrical coordinates
 ******************************************************************************/
void Rcs::TaskCylindrical3D::computeX(double* x_res) const
{
  double pos[3];
  Rcs::TaskPosition3D::computeX(pos);
  Math_Cart2Cyl(pos, &x_res[0], &x_res[1], &x_res[2]);
}

/*******************************************************************************
 * Computes the current velocity in task space
 * Reuses TaskPosition3D::computeXp and then converts to cylindrical coordinates
 ******************************************************************************/
void Rcs::TaskCylindrical3D::computeXp(double* xp_res) const
{
  double XYZ[3];
  Rcs::TaskPosition3D::computeX(XYZ);

  double r = sqrt(XYZ[0]*XYZ[0]+XYZ[1]*XYZ[1]);

  double XYZp[3];
  Rcs::TaskPosition3D::computeXp(XYZp);

  xp_res[0] = (XYZ[0]*XYZp[0] + XYZ[1]*XYZp[1])/r;
  xp_res[1] = (-XYZ[1]*XYZp[0]+XYZ[0]*XYZp[1])/(r*r);
  xp_res[2] = XYZp[2];
}

/*******************************************************************************
 * \brief Computes current task Jacobian to parameter jacobian.
 *
 * Reuses TaskPosition3D::computeJ and then converts to cylindrical coordinates
 ******************************************************************************/
void Rcs::TaskCylindrical3D::computeJ(MatNd* jacobian) const
{
  Rcs::TaskPosition3D::computeJ(jacobian);

  double XYZ[3];
  Rcs::TaskPosition3D::computeX(XYZ);

  double dCyldCart[3][3];
  MatNd trafo = MatNd_fromPtr(3, 3, &dCyldCart[0][0]);
  Math_dCyldCart(dCyldCart, XYZ);
  MatNd_preMulSelf(jacobian, &trafo);
}

/*******************************************************************************
 * Computes the delta in task space for the differential kinematics.
 * Ensures that always the shortest path in phi is followed
 ******************************************************************************/
void Rcs::TaskCylindrical3D::computeDX(double* dx, const double* x_des) const
{
  double x_des_modif[3];
  Vec3d_copy(x_des_modif, x_des);
  x_des_modif[0] = fabs(x_des_modif[0]);

  Rcs::TaskPosition3D::computeDX(dx, x_des_modif);
  while (dx[1]>M_PI)
  {
    dx[1] -= 2*M_PI;
  }
  while (dx[1]<=-M_PI)
  {
    dx[1] += 2*M_PI;
  }
}

/*******************************************************************************
 * Computes the Hessian for the cylinder coordinates by applying the chain rule
 * to the
 * J_cyl = A J_pos   =>   H_cyl = A H_pos + dA J_pos
 ******************************************************************************/
void Rcs::TaskCylindrical3D::computeH(MatNd* hessian) const
{
  RFATAL("Not yet implemented");
}

/*******************************************************************************
 * Computes the current derivative Jacobian
 * Reuses TaskPosition3D::computeJdot and then converts to cylindrical
 * coordinates
 ******************************************************************************/
void Rcs::TaskCylindrical3D::computeJdot(MatNd* Jdot) const
{
  //Jdot = A*JdotPos + Adot*Jpos

  MatNd* jacobianPos = NULL;
  MatNd_create2(jacobianPos, Jdot->m, Jdot->n);

  ////////////////////////////////////////////////////////////////
  // calling Rcs::TaskPosition3D::computeJdot() does not work as it invokes
  // Rcs::TaskCylindrical3D::computeH()
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
bool Rcs::TaskCylindrical3D::isValid(xmlNode* node, const RcsGraph* graph)
{
  bool success = Rcs::Task::isValid(node, graph, "CylRPZ");

  return success;
}

/*******************************************************************************
 * Remove this once we have an implementation for the Hessian.
 ******************************************************************************/
bool Rcs::TaskCylindrical3D::testHessian(bool verbose)
{
  return true;
}
