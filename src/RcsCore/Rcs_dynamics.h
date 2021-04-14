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

#ifndef RCS_DYNAMICS_H
#define RCS_DYNAMICS_H

#include "Rcs_graph.h"

#ifdef __cplusplus
extern "C" {
#endif


/*!
 * \defgroup RcsKineticsFunctions Dynamics
 *
 */

typedef struct
{
  RcsGraph* graph;
  MatNd* F_ext;

} DirDynParams;

/*! \ingroup RcsKineticsFunctions
 *  \brief Integrator with step width adaptation using a 2nd-3rd order
 *         Runge-Kutta-Fehlberg method. It uses currently some static
 *         variables and is not reentrant. Calling this function with
 *         different equations of motion will lead to trouble...
 *
 *  \param[in] FCN      Evaluation function for the differential equations
 *  \param[in] param    Pointer to your use-case specific data
 *  \param[in] nz       Number of differential equations
 *  \param[in] t1       Integration start time
 *  \param[in] t2       Integration end time
 *  \param[in] dt_opt   optimal integration step size
 *  \param[in] x        State at t
 *  \param[out] x2      State at t t2
 *  \param[in] maxErr   Permissable integration error per dimension
 *  \return Number of integration steps
 */
int integration_t1_t2(void (*FCN)(const double*, void*, double*, double),
                      void* param, int nz, double t1, double t2,
                      double* dt_opt, const double* x, double* x2,
                      double* maxErr);

/*! \ingroup RcsKineticsFunctions
 *  \brief Euler single step integrator.
 *
 *  \param[in] FCN      Evaluation function for the differential equations
 *  \param[in] param    Pointer to your use-case specific data
 *  \param[in] nz       Number of differential equations
 *  \param[in] dt       Integration interval
 *  \param[in] x        State at t
 *  \param[out] x2      State vector after integration
 */
void integration_euler(void (*FCN)(const double*, void*, double*, double),
                       void* param,
                       int nz,
                       double dt,
                       const double* x,
                       double* x2);

/*! \ingroup RcsKineticsFunctions
 *  \brief Computes the mass matrix, h-vector and gravity vector of a graph
 *         with a given state, and the current joint velocity vector qp. The
 *         function returns the sum of the kinetic and potential energy of
 *         the system. The dimensions of the arrays correspond to the
 *         unconstrained degrees of freedom of the system. Arguments qp, M,
 *         h and F_gravity may point to NULL. In this case, some terms will
 *         not be computed and the function gets more efficient. For
 *         instance it is possible to compute the overall energy only by
 *         calling<br>
 *         double E =
 *           RcsGraph_computeKineticTerms(self, qp, NULL, NULL, NULL);
 *         <br>
 *         or
 *         <br>
 *         RcsGraph_computeKineticTerms(self, NULL, NULL, NULL, F_gravity);
 *         <br>
 *         computes the gravity load only. The returned energy is then the
 *         potential energy only.
 *         If the dimension of the active dofs is smaller or equal
 *         MATND_MAX_STACK_VECTOR_SIZE doubles, no heap memory will be
 *         allocated (except if M is NULL).
 */
double RcsGraph_computeKineticTerms(const RcsGraph* graph,
                                    MatNd* M,
                                    MatNd* h,
                                    MatNd* F_gravity);

/*! \ingroup RcsKineticsFunctions
 *  \brief Computes the gravity torques
 */
void RcsGraph_computeGravityTorque(const RcsGraph* graph,
                                   MatNd* T_gravity);

/*! \ingroup RcsKineticsFunctions
 *  \brief Computes the mass matrix.
 */
void RcsGraph_computeMassMatrix(const RcsGraph* graph, MatNd* M);

/*! \ingroup RcsKineticsFunctions
 *  \brief Solves the multibody equations of motion for the joint
 *         accelerations:
 *
 *         M(q) qpp + h(q,qp) + F_gravity + F_ext = F_jnt
 *
 *         q, qp, qpp: Generalized coordinates, velocities and accelerations
 *                     The dimension is graph->nJ (num. of unconstrained dof)
 *         M:          Mass and inertia matrix
 *         F_gravity:  Gravity force projected on generalized coordinates
 *         h:          Coriolis vector
 *         F_ext:      External forces projected on generalized coordinates
 *         F_jnt:      Joint torque vector
 *
 *         Currently F_jnt is projected on all unconstrained degrees of
 *         freedom. Arrays F_ext and F_jnt can be pointers to NULL. In this
 *         case, they are assumed to be zero.
 *
 *         The function returns the overall energy of the system. It will
 *         warn on debug level 1 if
 *         - the Cholesky decomposition failed
 *         - one of the elements of qpp is not finite.
 */
double Rcs_directDynamics(const RcsGraph* graph,
                          const MatNd* F_ext,
                          const MatNd* F_jnt,
                          MatNd* qpp);

/*! \ingroup RcsKineticsFunctions
 *  \brief Wrapper function of the direct dynamics to match the function
 *         signature of the integrator. Argument param is assumed to
 *         point to a RcsGraph structure. It's userData field is assumed
 *         to point to an MatNd holding the external forces projected into
 *         the configuration space. Please see the implementation for
 *         details.
 */
void Rcs_directDynamicsIntegrationStep(const double* x, void* param,
                                       double* xp, double dt);



#ifdef __cplusplus
}
#endif

#endif   // RCS_DYNAMICS_H
