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

#include "TaskCylindricalPos3D.h"
#include "TaskFactory.h"
#include "Rcs_typedef.h"
#include "Rcs_macros.h"
#include "Rcs_Vec3d.h"
#include "Rcs_basicMath.h"
#include "Rcs_parser.h"



static Rcs::TaskFactoryRegistrar<Rcs::TaskCylindricalPos3D> registrar("CylXYZ");



/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::TaskCylindricalPos3D::TaskCylindricalPos3D(const std::string& className,
                                                xmlNode* node,
                                                RcsGraph* _graph,
                                                int _dim):
  TaskPosition3D(className, node, _graph, _dim)
{
  if (getClassName()=="CylXYZ")
  {
    resetParameter(Parameters(0, 2.5, 1.0, "Radius [m]"));
    addParameter(Parameters(-M_PI, M_PI, 180.0/M_PI, "Phi [deg]"));
    addParameter(Parameters(-2.5, 2.5, 1.0, "Z Position [m]"));
  }
}

/*******************************************************************************
 * Destructor
 ******************************************************************************/
Rcs::TaskCylindricalPos3D::~TaskCylindricalPos3D()
{
}

/*******************************************************************************
 * Clone function
 ******************************************************************************/
Rcs::TaskCylindricalPos3D* Rcs::TaskCylindricalPos3D::clone(RcsGraph* newGraph) const
{
  TaskCylindricalPos3D* task = new Rcs::TaskCylindricalPos3D(*this);
  task->setGraph(newGraph);
  return task;
}

/*******************************************************************************
 * Computes the current value of the task variable
 * Reuses TaskPosition3D::computeX and then converts to cylindrical coordinates
 ******************************************************************************/
void Rcs::TaskCylindricalPos3D::computeX(double* x_res) const
{
  double pos[3];
  Rcs::TaskPosition3D::computeX(pos);
  Math_Cart2Cyl(pos, &x_res[0], &x_res[1], &x_res[2]);
}

/*******************************************************************************
 * Computes the current velocity in task space
 * Reuses TaskPosition3D::computeXp and then converts to cylindrical coordinates
 ******************************************************************************/
void Rcs::TaskCylindricalPos3D::computeXp(double* xp_res) const
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
 * Computes the delta in task space for the differential kinematics.
 * Ensures that always the shortest path in phi is followed
 ******************************************************************************/
void Rcs::TaskCylindricalPos3D::computeDX(double* dx,
                                          const double* cyl_des) const
{
  double x_curr[3], x_des[3];
  Rcs::TaskPosition3D::computeX(x_curr);

  Math_Cyl2Cart(cyl_des[0], cyl_des[1], cyl_des[2], x_des);
  Vec3d_sub(dx, x_des, x_curr);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::TaskCylindricalPos3D::isValid(xmlNode* node, const RcsGraph* graph)
{
  bool success = Rcs::Task::isValid(node, graph, "CylXYZ");

  return success;
}
