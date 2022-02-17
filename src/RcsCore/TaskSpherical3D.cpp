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

#include "TaskSpherical3D.h"
#include "TaskFactory.h"
#include "Rcs_typedef.h"
#include "Rcs_parser.h"
#include "Rcs_macros.h"
#include "Rcs_Vec3d.h"
#include "Rcs_basicMath.h"



static Rcs::TaskFactoryRegistrar<Rcs::TaskSpherical3D> registrar("SphRTP");



/******************************************************************************
 * Helper Function.
 *****************************************************************************/
static inline double sqr(double x)
{
  return x*x;
}

/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::TaskSpherical3D::TaskSpherical3D(const std::string& className,
                                      xmlNode* node,
                                      const RcsGraph* _graph,
                                      int dim):
  TaskPosition3D(className, node, _graph, dim)
{
  if (getClassName()=="SphRTP")
  {
    resetParameter(Parameters(0.0, 2.5, 1.0, "Radius [m]"));
    addParameter(Parameters(0.0, M_PI, 180.0/M_PI, "Theta [deg]"));
    addParameter(Parameters(-M_PI, M_PI, 180.0/M_PI, "Phi [deg]"));
  }
}

/*******************************************************************************
 * Clone function
 ******************************************************************************/
Rcs::TaskSpherical3D* Rcs::TaskSpherical3D::clone(const RcsGraph* newGraph) const
{
  TaskSpherical3D* task = new Rcs::TaskSpherical3D(*this);
  task->setGraph(newGraph);
  return task;
}

/*******************************************************************************
 * Computes the current value of the task variable
 * Reuses TaskPosition3D::computeX and then converts to spherical coordinates
 ******************************************************************************/
void Rcs::TaskSpherical3D::computeX(double* x_res) const
{
  double pos[3];
  Rcs::TaskPosition3D::computeX(pos);
  x_res[0] = Vec3d_getLength(pos);
  x_res[1] = Math_acos(pos[2]/x_res[0]);
  x_res[2] = atan2(pos[1],pos[0]);
}

/*******************************************************************************
 * Reuses TaskPosition3D::computeXp and then converts to spherical coordinates
 ******************************************************************************/
void Rcs::TaskSpherical3D::computeXp(double* xp_res) const
{
  double pos[3];
  Rcs::TaskPosition3D::computeX(pos);

  double r = Vec3d_getLength(pos);
  double c = pos[2]/(sqrt(sqr(r)-sqr(pos[2]))*sqr(r));

  double vel[3];
  Rcs::TaskPosition3D::computeXp(vel);

  xp_res[0] = (pos[0]*vel[0] + pos[1]*vel[1] + pos[2]*vel[2])/r;
  xp_res[1] = c*pos[0]*vel[0] + c*pos[1]*vel[1] + c*(pos[2]-sqr(r)/pos[2])*vel[2];
  xp_res[2] = -pos[1]/(sqr(pos[0])+sqr(pos[1]))*vel[0] + pos[0]/(sqr(pos[0])+sqr(pos[1]))*vel[1];
}

/*******************************************************************************
 * Reuses TaskPosition3D::computeJ and then converts to spherical coordinates
 ******************************************************************************/
void Rcs::TaskSpherical3D::computeJ(MatNd* jacobian) const
{
  Rcs::TaskPosition3D::computeJ(jacobian);

  double pos[3];
  Rcs::TaskPosition3D::computeX(pos);

  // Projection matrix: v_spherical = B*v_linear
  double r = Vec3d_getLength(pos);

  if (r<1.0e-6)
  {
    return;
  }


  double c = pos[2]/(sqrt(sqr(r)-sqr(pos[2]))*sqr(r));

  MatNd* trafo = NULL;
  MatNd_create2(trafo, 3, 3);

  MatNd_set2(trafo, 0, 0, pos[0]/r);
  MatNd_set2(trafo, 0, 1, pos[1]/r);
  MatNd_set2(trafo, 0, 2, pos[2]/r);

  MatNd_set2(trafo, 1, 0, c*pos[0]);
  MatNd_set2(trafo, 1, 1, c*pos[1]);
  MatNd_set2(trafo, 1, 2, c*(pos[2]-sqr(r)/pos[2]));

  MatNd_set2(trafo, 2, 0, -pos[1]/(sqr(pos[0])+sqr(pos[1])));
  MatNd_set2(trafo, 2, 1, pos[0]/(sqr(pos[0])+sqr(pos[1])));
  MatNd_set2(trafo, 2, 2, 0.0);

  MatNd_preMulSelf(jacobian, trafo);
  MatNd_destroy(trafo);
}

/*******************************************************************************
 * Computes the delta in task space for the differential kinematics.
 * Ensures that always the shortest path in phi is followed
 ******************************************************************************/
void Rcs::TaskSpherical3D::computeDX(double* dx, const double* x_des,
                                     const double* x_curr) const
{
  double x_des_modif[3];
  Vec3d_copy(x_des_modif, x_des);
  x_des_modif[0] = fabs(x_des_modif[0]);   // Radius always positive

  Rcs::TaskPosition3D::computeDX(dx, x_des_modif, x_curr);
  dx[1] = Math_fmodAngle(dx[1]);
  dx[2] = Math_fmodAngle(dx[2]);
}

/*******************************************************************************
 * Computes current task Hessian to parameter \e hessian
 * Not implemented!
 ******************************************************************************/
void Rcs::TaskSpherical3D::computeH(MatNd* hessian) const
{
  //Rcs::TaskGenericIK::computeH(hessian);
  RFATAL("Not yet implemented");
}

/*******************************************************************************
 * Computes the current derivative Jacobian
 * Reuses TaskPosition3D::computeJdot and then converts to spherical coordinates
 ******************************************************************************/
void Rcs::TaskSpherical3D::computeJdot(MatNd* Jdot) const
{
  //Jdot = A*JdotPos + Adot*Jpos

  MatNd* jacobianPos = NULL;
  MatNd_create2(jacobianPos, Jdot->m, Jdot->n);

  ////////////////////////////////////////////////////////////////
  // calling Rcs::TaskPosition3D::computeJdot() does not work as it invokes Rcs::TaskSpherical3D::computeH()
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

  double pos[3];
  Rcs::TaskPosition3D::computeX(pos);

  double r = Vec3d_getLength(pos);
  double c = pos[2]/(sqrt(sqr(r)-sqr(pos[2]))*sqr(r));

  MatNd* trafo = NULL;
  MatNd_create2(trafo, 3, 3);

  MatNd_set2(trafo, 0, 0, pos[0]/r);
  MatNd_set2(trafo, 0, 1, pos[1]/r);
  MatNd_set2(trafo, 0, 2, pos[2]/r);

  MatNd_set2(trafo, 1, 0, c*pos[0]);
  MatNd_set2(trafo, 1, 1, c*pos[1]);
  MatNd_set2(trafo, 1, 2, c*(pos[2]-sqr(r)/pos[2]));

  MatNd_set2(trafo, 2, 0, -pos[1]/(sqr(pos[0])+sqr(pos[1])));
  MatNd_set2(trafo, 2, 1, pos[0]/(sqr(pos[0])+sqr(pos[1])));
  MatNd_set2(trafo, 2, 2, 0.);

  MatNd_mul(Jdot, trafo, jacobianPos); // Jdot = B*JdotPos

  Rcs::TaskPosition3D::computeJ(jacobianPos);

  double vel[3];
  Rcs::TaskPosition3D::computeXp(vel);

  double rDot = (pos[0]*vel[0] + pos[1]*vel[1] + pos[2]*vel[2])/r;
  double cDot = (2.*r*rDot - 2.*pos[2]*vel[2])*pos[2]/(sqr(r)+sqr(pos[2])-2./rDot*rDot*pos[2]+vel[2])/(sqrt(sqr(r)-sqr(pos[2]))*sqr(r));

  MatNd_setZero(trafo);

  MatNd_set2(trafo, 0, 0, (vel[0]*r-rDot*pos[0])/sqr(r));
  MatNd_set2(trafo, 0, 1, (vel[1]*r-rDot*pos[1])/sqr(r));
  MatNd_set2(trafo, 0, 2, (vel[2]*r-rDot*pos[2])/sqr(r));

  MatNd_set2(trafo, 1, 0, cDot*pos[0]+c*vel[0]);
  MatNd_set2(trafo, 1, 1, cDot*pos[1]+c*vel[1]);
  MatNd_set2(trafo, 1, 2, cDot*(pos[2]-sqr(r)/pos[2])+c*(vel[2]-(2*r*rDot*pos[2]-sqr(r)*vel[2])/sqr(pos[2])));

  MatNd_set2(trafo, 2, 0, (-vel[1]*(sqr(pos[0])+sqr(pos[1]))+pos[1]*(2*pos[0]*vel[0]+2*pos[1]*vel[1]))/sqr(sqr(pos[0])+sqr(pos[1])));
  MatNd_set2(trafo, 2, 1, (vel[0]*(sqr(pos[0])+sqr(pos[1]))-pos[0]*(2*pos[0]*vel[0]+2*pos[1]*vel[1]))/sqr(sqr(pos[0])+sqr(pos[1])));
  MatNd_set2(trafo, 2, 2, 0.);

  MatNd_mulAndAddSelf(Jdot, trafo, jacobianPos); // Jdot += Bdot*Jpos

  MatNd_destroy(trafo);
  MatNd_destroy(jacobianPos);
}

/*******************************************************************************
 * See header
 ******************************************************************************/
bool Rcs::TaskSpherical3D::test(bool verbose) const
{
  const double delta = 0.01;
  bool success = true;
  bool success_i;
  bool relativeError = true;
  double errorLimit;

  if (relativeError == true)
  {
    errorLimit = 5.0e-2; // 5.0% error limit
  }
  else
  {
    errorLimit = 1.0e-4;
  }

  success_i = testJacobian(errorLimit, delta, relativeError, verbose);
  success = success && success_i;

  // success_i = testHessian(verbose);
  // success = success && success_i;

  return success;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::TaskSpherical3D::isValid(xmlNode* node, const RcsGraph* graph)
{
  bool success = Rcs::Task::isValid(node, graph, "SphRTP");

  return success;
}
