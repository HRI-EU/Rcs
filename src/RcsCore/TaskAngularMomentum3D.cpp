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

#include "TaskAngularMomentum3D.h"
#include "TaskFactory.h"
#include "Rcs_typedef.h"
#include "Rcs_macros.h"
#include "Rcs_parser.h"
#include "Rcs_utils.h"
#include "Rcs_kinematics.h"
#include "Rcs_Vec3d.h"



static Rcs::TaskFactoryRegistrar<Rcs::TaskAngularMomentum3D> registrar1("AngularMomentum");



/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::TaskAngularMomentum3D::TaskAngularMomentum3D(const std::string& className,
                                                  xmlNode* node,
                                                  RcsGraph* _graph,
                                                  int dim):
  TaskGenericIK(className, node, _graph, dim)
{

  if (getClassName()=="AngularMomentum")
  {
    resetParameter(Parameters(-1.0, 1.0, 1.0, "Lx"));
    addParameter(Parameters(-1.0, 1.0, 1.0, "Ly"));
    addParameter(Parameters(-1.0, 1.0, 1.0, "Lz"));
  }

}

/*******************************************************************************
 * Copy constructor doing deep copying
 ******************************************************************************/
Rcs::TaskAngularMomentum3D::TaskAngularMomentum3D(const Rcs::TaskAngularMomentum3D& copyFromMe, RcsGraph* newGraph):
  Rcs::TaskGenericIK(copyFromMe, newGraph)
{
}


/*******************************************************************************
 * Destructor
 ******************************************************************************/
Rcs::TaskAngularMomentum3D::~TaskAngularMomentum3D()
{
}

/*******************************************************************************
 * Clone function
 ******************************************************************************/
Rcs::TaskAngularMomentum3D* Rcs::TaskAngularMomentum3D::clone(RcsGraph* newGraph) const
{
  return new Rcs::TaskAngularMomentum3D(*this, newGraph);
}

/*******************************************************************************
 * Vector x is the angular momentum change. Other than the linear
 * momentum, there's no corresponding consistent state associated with
 * it. This means that the pose might numerically drift for a given
 * value for x.
 ******************************************************************************/
void Rcs::TaskAngularMomentum3D::computeX(double* x_res) const
{
  Vec3d_setZero(x_res);
}

/*******************************************************************************
 * Angular momentum L = JL * q_dot for the inertial reference frame.
 * JL = Sum { Inertia * J_rot + m * (r_cog x J_cog) }
 ******************************************************************************/
void Rcs::TaskAngularMomentum3D::computeJ(MatNd* jacobian) const
{
  double I_r_cog[3];
  MatNd* J1=NULL, *J2=NULL;

  MatNd_create2(J1, 3, this->graph->nJ);
  MatNd_create2(J2, 3, this->graph->nJ);
  MatNd_reshape(jacobian, 3, this->graph->nJ);
  MatNd_setZero(jacobian);

  // Compute the overall angular momentum Jacobian
  RCSGRAPH_TRAVERSE_BODIES(this->graph)
  {
    if (BODY->m <= 0.0)
    {
      continue;
    }

    // Rotational part of the angular momentum Jacobian: J1 = Inertia * J_rot
    // with the inertia tensor referring to the bodies COG, represented in
    // the bodies frame of reference.
    RcsGraph_rotationJacobian(this->graph, BODY, NULL, J1);
    MatNd Inertia = MatNd_fromPtr(3, 3, &BODY->Inertia->rot[0][0]);
    MatNd_mul(J2, &Inertia, J1);
    MatNd_addSelf(jacobian, J2);

    // Steiner portion of angular momentum: J2 = m * (r_cog x J_cog)
    // Currently the reference point is (0 0 0).
    RcsGraph_bodyPointJacobian(this->graph, BODY, BODY->Inertia->org, NULL, J1);
    MatNd_constMulSelf(J1, BODY->m);
    RcsGraph_bodyPoint(BODY, BODY->Inertia->org, I_r_cog);
    MatNd_columnCrossProductSelf(J1, I_r_cog);
    MatNd_addSelf(jacobian, J1);
  }

  MatNd_destroy(J1);
  MatNd_destroy(J2);
}

/*******************************************************************************
 * \brief see header
 ******************************************************************************/
void Rcs::TaskAngularMomentum3D::computeH(MatNd* hessian) const
{
  RFATAL("Hessian computation for task \"%s\" not yet implemented!",
         getName().c_str());
}

/*******************************************************************************
 * \brief See header.
 ******************************************************************************/
bool Rcs::TaskAngularMomentum3D::isValid(xmlNode* node,
                                         const RcsGraph* graph)
{
  return Rcs::Task::isValid(node, graph, "AngularMomentum");
}
