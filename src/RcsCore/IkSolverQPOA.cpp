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

#include "IkSolverQPOA.h"

#if defined (USE_QPOASES)

#include <Rcs_macros.h>
#include <Rcs_cmdLine.h>
#include <Rcs_math.h>
#include <Rcs_timer.h>
#include <Rcs_typedef.h>
#include <Rcs_kinematics.h>
#include <Rcs_joint.h>
#include <Rcs_body.h>
#include <Rcs_shape.h>
#include <Rcs_utils.h>

#include <qpOASES.hpp>

#include <iostream>

USING_NAMESPACE_QPOASES


QProblem* qp_hack = NULL;


static const char* errorCode(int idx)
{
  static char errCode[][256] =
  {
    "SUCCESSFUL_RETURN                Successful return. ",
    "RET_DIV_BY_ZERO,                   Division by zero. ",
    "RET_INDEX_OUT_OF_BOUNDS,             Index out of bounds. ",
    "RET_INVALID_ARGUMENTS,               At least one of the arguments is invalid. ",
    "RET_ERROR_UNDEFINED,               Error number undefined. ",
    "RET_WARNING_UNDEFINED,               Warning number undefined. ",
    "RET_INFO_UNDEFINED,                Info number undefined. ",
    "RET_EWI_UNDEFINED,                 Error/warning/info number undefined. ",
    "RET_AVAILABLE_WITH_LINUX_ONLY,           This function is available under Linux only. ",
    "RET_UNKNOWN_BUG,                 The error occurred is not yet known. ",
    "RET_PRINTLEVEL_CHANGED,              Print level changed. (10) ",
    "RET_NOT_YET_IMPLEMENTED,             Requested function is not yet implemented in this version of qpOASES. ",

    "RET_INDEXLIST_MUST_BE_REORDERD,          Index list has to be reordered. ",
    "RET_INDEXLIST_EXCEEDS_MAX_LENGTH,        Index list exceeds its maximal physical length. ",
    "RET_INDEXLIST_CORRUPTED,             Index list corrupted. ",
    "RET_INDEXLIST_OUTOFBOUNDS,             Physical index is out of bounds. ",
    "RET_INDEXLIST_ADD_FAILED,            Adding indices from another index set failed. ",
    "RET_INDEXLIST_INTERSECT_FAILED,          Intersection with another index set failed. ",

    "RET_INDEX_ALREADY_OF_DESIRED_STATUS,       Index is already of desired status. (18) ",
    "RET_ADDINDEX_FAILED,               Adding index to index set failed. ",
    "RET_REMOVEINDEX_FAILED,              Removing index from index set failed. (20) ",
    "RET_SWAPINDEX_FAILED,              Cannot swap between different indexsets. ",
    "RET_NOTHING_TO_DO,                 Nothing to do. ",
    "RET_SETUP_BOUND_FAILED,              Setting up bound index failed. ",
    "RET_SETUP_CONSTRAINT_FAILED,           Setting up constraint index failed. ",
    "RET_MOVING_BOUND_FAILED,             Moving bound between index sets failed. ",
    "RET_MOVING_CONSTRAINT_FAILED,          Moving constraint between index sets failed. ",
    "RET_SHIFTING_FAILED,               Shifting of bounds/constraints failed. ",
    "RET_ROTATING_FAILED,               Rotating of bounds/constraints failed. ",

    "RET_QPOBJECT_NOT_SETUP,              The QP object has not been setup correctly, use another constructor. ",
    "RET_QP_ALREADY_INITIALISED,            QProblem has already been initialised. (30) ",
    "RET_NO_INIT_WITH_STANDARD_SOLVER,        Initialisation via extern QP solver is not yet implemented. ",
    "RET_RESET_FAILED,                Reset failed. ",
    "RET_INIT_FAILED,                 Initialisation failed. ",
    "RET_INIT_FAILED_TQ,                Initialisation failed due to TQ factorisation. ",
    "RET_INIT_FAILED_CHOLESKY,            Initialisation failed due to Cholesky decomposition. ",
    "RET_INIT_FAILED_HOTSTART,            Initialisation failed! QP could not be solved! ",
    "RET_INIT_FAILED_INFEASIBILITY,           Initial QP could not be solved due to infeasibility! ",
    "RET_INIT_FAILED_UNBOUNDEDNESS,           Initial QP could not be solved due to unboundedness! ",
    "RET_INIT_FAILED_REGULARISATION,          Initialisation failed as Hessian matrix could not be regularised. ",
    "RET_INIT_SUCCESSFUL,               Initialisation done. (40) ",
    "RET_OBTAINING_WORKINGSET_FAILED,         Failed to obtain working set for auxiliary QP. ",
    "RET_SETUP_WORKINGSET_FAILED,           Failed to setup working set for auxiliary QP. ",
    "RET_SETUP_AUXILIARYQP_FAILED,          Failed to setup auxiliary QP for initialised homotopy. ",
    "RET_NO_CHOLESKY_WITH_INITIAL_GUESS, Externally computed Cholesky factor cannot be combined with an initial guess.",
    "RET_NO_EXTERN_SOLVER,              No extern QP solver available. ",
    "RET_QP_UNBOUNDED,                QP is unbounded. ",
    "RET_QP_INFEASIBLE,                 QP is infeasible. ",
    "RET_QP_NOT_SOLVED,                 Problems occurred while solving QP with standard solver. ",
    "RET_QP_SOLVED,                   QP successfully solved. ",
    "RET_UNABLE_TO_SOLVE_QP,              Problems occurred while solving QP. (50) ",
    "RET_INITIALISATION_STARTED,            Starting problem initialisation... ",
    "RET_HOTSTART_FAILED,               Unable to perform homotopy due to internal error. ",
    "RET_HOTSTART_FAILED_TO_INIT,           Unable to initialise problem. ",
    "RET_HOTSTART_FAILED_AS_QP_NOT_INITIALISED,     Unable to perform homotopy as previous QP is not solved. ",
    "RET_ITERATION_STARTED,               Iteration... ",
    "RET_SHIFT_DETERMINATION_FAILED,          Determination of shift of the QP data failed. ",
    "RET_STEPDIRECTION_DETERMINATION_FAILED,      Determination of step direction failed. ",
    "RET_STEPLENGTH_DETERMINATION_FAILED,       Determination of step direction failed. ",
    "RET_OPTIMAL_SOLUTION_FOUND,            Optimal solution of neighbouring QP found. ",
    "RET_HOMOTOPY_STEP_FAILED,            Unable to perform homotopy step. (60) ",
    "RET_HOTSTART_STOPPED_INFEASIBILITY,        Premature homotopy termination because QP is infeasible. ",
    "RET_HOTSTART_STOPPED_UNBOUNDEDNESS,        Premature homotopy termination because QP is unbounded. ",
    "RET_WORKINGSET_UPDATE_FAILED,          Unable to update working sets according to initial guesses. ",
    "RET_MAX_NWSR_REACHED,              Maximum number of working set recalculations performed. ",
    "RET_CONSTRAINTS_NOT_SPECIFIED, Problem does comprise constraints! You also must specify new constraints' bounds.",
    "RET_INVALID_FACTORISATION_FLAG,          Invalid factorisation flag. ",
    "RET_UNABLE_TO_SAVE_QPDATA,             Unable to save QP data. ",
    "RET_STEPDIRECTION_FAILED_TQ,           Abnormal termination due to TQ factorisation. ",
    "RET_STEPDIRECTION_FAILED_CHOLESKY,         Abnormal termination due to Cholesky factorisation. ",
    "RET_CYCLING_DETECTED,              Cycling detected. (70) ",
    "RET_CYCLING_NOT_RESOLVED,            Cycling cannot be resolved, QP probably infeasible. ",
    "RET_CYCLING_RESOLVED,              Cycling probably resolved. ",
    "RET_STEPSIZE,                  For displaying performed stepsize. ",
    "RET_STEPSIZE_NONPOSITIVE,            For displaying non-positive stepsize. ",
    "RET_SETUPSUBJECTTOTYPE_FAILED,           Setup of SubjectToTypes failed. ",
    "RET_ADDCONSTRAINT_FAILED,            Addition of constraint to working set failed. ",
    "RET_ADDCONSTRAINT_FAILED_INFEASIBILITY, Addition of constraint to working set failed (due to QP infeasibility). ",
    "RET_ADDBOUND_FAILED,               Addition of bound to working set failed. ",
    "RET_ADDBOUND_FAILED_INFEASIBILITY,         Addition of bound to working set failed (due to QP infeasibility). ",
    "RET_REMOVECONSTRAINT_FAILED,           Removal of constraint from working set failed. (80) ",
    "RET_REMOVEBOUND_FAILED,              Removal of bound from working set failed. ",
    "RET_REMOVE_FROM_ACTIVESET,             Removing from active set... ",
    "RET_ADD_TO_ACTIVESET,              Adding to active set... ",
    "RET_REMOVE_FROM_ACTIVESET_FAILED,        Removing from active set failed. ",
    "RET_ADD_TO_ACTIVESET_FAILED,           Adding to active set failed. ",
    "RET_CONSTRAINT_ALREADY_ACTIVE,           Constraint is already active. ",
    "RET_ALL_CONSTRAINTS_ACTIVE,            All constraints are active, no further constraint can be added. ",
    "RET_LINEARLY_DEPENDENT,              New bound/constraint is linearly dependent. ",
    "RET_LINEARLY_INDEPENDENT,            New bound/constraint is linearly independent. ",
    "RET_LI_RESOLVED,                 Linear independence of active constraint matrix successfully resolved. (90) ",
    "RET_ENSURELI_FAILED,               Failed to ensure linear independence of active constraint matrix. ",
    "RET_ENSURELI_FAILED_TQ,              Abnormal termination due to TQ factorisation. ",
    "RET_ENSURELI_FAILED_NOINDEX,           QP is infeasible. ",
    "RET_ENSURELI_FAILED_CYCLING,           QP is infeasible. ",
    "RET_BOUND_ALREADY_ACTIVE,            Bound is already active. ",
    "RET_ALL_BOUNDS_ACTIVE,               All bounds are active, no further bound can be added. ",
    "RET_CONSTRAINT_NOT_ACTIVE,             Constraint is not active. ",
    "RET_BOUND_NOT_ACTIVE,              Bound is not active. ",
    "RET_HESSIAN_NOT_SPD,               Projected Hessian matrix not positive definite. ",
    "RET_HESSIAN_INDEFINITE,              Hessian matrix is indefinite. (100) ",
    "RET_MATRIX_SHIFT_FAILED,             Unable to update matrices or to transform vectors. ",
    "RET_MATRIX_FACTORISATION_FAILED,         Unable to calculate new matrix factorisations. ",
    "RET_PRINT_ITERATION_FAILED,            Unable to print information on current iteration. ",
    "RET_NO_GLOBAL_MESSAGE_OUTPUTFILE,        No global message output file initialised. ",
    "RET_DISABLECONSTRAINTS_FAILED,           Unable to disbable constraints. ",
    "RET_ENABLECONSTRAINTS_FAILED,          Unable to enbable constraints. ",
    "RET_ALREADY_ENABLED,               Bound or constraint is already enabled. ",
    "RET_ALREADY_DISABLED,              Bound or constraint is already disabled. ",
    "RET_NO_HESSIAN_SPECIFIED,              No Hessian matrix has been specified. ",
    "RET_USING_REGULARISATION,            Using regularisation as Hessian matrix is not positive definite. (110) ",
    "RET_EPS_MUST_BE_POSITVE,             Eps for regularisation must be sufficiently positive. ",
    "RET_REGSTEPS_MUST_BE_POSITVE,            Maximum number of regularisation steps must be non-negative. ",
    "RET_HESSIAN_ALREADY_REGULARISED,         Hessian has been already regularised. ",
    "RET_CANNOT_REGULARISE_IDENTITY,          Identity Hessian matrix cannot be regularised. ",
    "RET_CANNOT_REGULARISE_SPARSE,          Sparse matrix cannot be regularised as diagonal entry is missing. ",
    "RET_NO_REGSTEP_NWSR,               No additional regularisation step could be performed due to limits. ",
    "RET_FEWER_REGSTEPS_NWSR,             Fewer additional regularisation steps have been performed due to limits. ",
    "RET_CHOLESKY_OF_ZERO_HESSIAN,            Cholesky decomposition of (unregularised) zero Hessian matrix. ",
    "RET_ZERO_HESSIAN_ASSUMED, Zero Hessian matrix assumed as null pointer passed without specifying hessianType. ",
    "RET_CONSTRAINTS_ARE_NOT_SCALED,          (no longer in use) (120) ",
    "RET_INITIAL_BOUNDS_STATUS_NYI,           (no longer in use) ",
    "RET_ERROR_IN_CONSTRAINTPRODUCT,          Error in user-defined constraint product function. ",
    "RET_FIX_BOUNDS_FOR_LP,               All initial bounds must be fixed when solving an (unregularised) LP. ",
    "RET_USE_REGULARISATION_FOR_LP,           Set options.enableRegularisation=BT_TRUE for solving LPs. ",

    "RET_UPDATEMATRICES_FAILED,             Unable to update QP matrices. ",
    "RET_UPDATEMATRICES_FAILED_AS_QP_NOT_SOLVED,    Unable to update matrices as previous QP is not solved. ",

    "RET_UNABLE_TO_OPEN_FILE,             Unable to open file. ",
    "RET_UNABLE_TO_WRITE_FILE,            Unable to write into file. ",
    "RET_UNABLE_TO_READ_FILE,             Unable to read from file. ",
    "RET_FILEDATA_INCONSISTENT,             File contains inconsistent data. (130) ",

    "RET_OPTIONS_ADJUSTED,              Options needed to be adjusted for consistency reasons. ",

    "RET_UNABLE_TO_ANALYSE_QPROBLEM,          Unable to analyse (S)QProblem(B) object. ",

    "RET_NWSR_SET_TO_ONE,               Maximum number of working set changes was set to 1. ",
    "RET_UNABLE_TO_READ_BENCHMARK,          Unable to read benchmark data. ",
    "RET_BENCHMARK_ABORTED,               Benchmark aborted. ",
    "RET_INITIAL_QP_SOLVED,               Initial QP solved. ",
    "RET_QP_SOLUTION_STARTED,             Solving QP... ",
    "RET_BENCHMARK_SUCCESSFUL,            Benchmark terminated successfully. ",

    "RET_NO_DIAGONAL_AVAILABLE,             Sparse matrix does not have entries on full diagonal. ",
    "RET_DIAGONAL_NOT_INITIALISED,          Diagonal data of sparse matrix has not been initialised. (140) ",

    "RET_ENSURELI_DROPPED,              Linear independence resolved by dropping blocking constraint. ",

    "RET_KKT_MATRIX_SINGULAR,             KKT matrix is singular. ",
    "RET_QR_FACTORISATION_FAILED,           QR factorization of Schur complement failed. ",
    "RET_INERTIA_CORRECTION_FAILED, Inertia correction failed after KKT matrix had too many negative eigenvalues. ",
    "RET_NO_SPARSE_SOLVER,              No factorization routine for the KKT matrix installed. ",

    "RET_SIMPLE_STATUS_P1,              QP problem could not be solved within given number of iterations. ",
    "RET_SIMPLE_STATUS_P0,              QP problem solved. ",
    "RET_SIMPLE_STATUS_M1,              QP problem could not be solved due to an internal error. ",
    "RET_SIMPLE_STATUS_M2,              QP problem is infeasible (and thus could not be solved). ",
    "RET_SIMPLE_STATUS_M3               QP problem is unbounded (and thus could not be solved). (150) "
  };

  return errCode[idx];
}

static void setOptions(Options& myOptions)
{
  myOptions.setToReliable();
  //myOptions.enableRegularisation = BT_TRUE;
  myOptions.enableCholeskyRefactorisation = 1;
  myOptions.enableDriftCorrection = BT_TRUE;
  RLOG_CPP(1, "epsRegularisation: " << myOptions.epsRegularisation);
  RLOG_CPP(1, "numRegularisationSteps: " << myOptions.numRegularisationSteps);
  RLOG_CPP(1, "numRefinementSteps: " << myOptions.numRefinementSteps);
  RLOG_CPP(1, "maxPrimalJump: " << myOptions.maxPrimalJump);
  RLOG_CPP(1, "maxDualJump: " << myOptions.maxDualJump);
  RLOG_CPP(1, "boundTolerance: " << myOptions.boundTolerance);
  RLOG_CPP(1, "boundRelaxation: " << myOptions.boundRelaxation);
  RLOG_CPP(1, "terminationTolerance: " << myOptions.terminationTolerance);
  //myOptions.maxPrimalJump = 0.1;
  //myOptions.maxDualJump = 0.1;
  myOptions.numRegularisationSteps = 10;
  myOptions.numRefinementSteps = 20;
  myOptions.epsRegularisation = 0.1;
  //myOptions.terminationTolerance = 1.0e-3;
  //myOptions.boundTolerance = 0.001;
  //myOptions.boundRelaxation = 0.001;
  //myOptions.setToMPC();
  //myOptions.setToDefault();
  //myOptions.setToReliable();
  myOptions.enableEqualities = BT_TRUE;
  myOptions.printLevel = (RcsLogLevel == 0) ? PL_NONE : PL_HIGH;
}

/*******************************************************************************
*
******************************************************************************/
namespace Rcs
{


IkSolverQPOA::IkSolverQPOA(ControllerBase* controller_) : IkSolverRMR(controller_), nConstraints(0)
{
}

IkSolverQPOA::~IkSolverQPOA()
{
}

void IkSolverQPOA::getCollisionConstraints(MatNd* J, MatNd* x)
{
  if (controller->getCollisionMdl() == NULL)
  {
    J->m = 0;
    x->m = 0;
    return;
  }

  unsigned int nPairs = controller->getCollisionMdl()->cp->m;

  MatNd_realloc(J, nPairs, J->n);
  MatNd_realloc(x, nPairs, x->n);

  unsigned int idx = 0;

  for (unsigned int i = 0; i < controller->getCollisionMdl()->nPairs; ++i)
  {
    RcsPair* PAIR = &controller->getCollisionMdl()->pair[i];
    const RcsBody* b1 = RCSBODY_BY_ID(controller->getGraph(), PAIR->b1);
    const RcsBody* b2 = RCSBODY_BY_ID(controller->getGraph(), PAIR->b2);
    MatNd Ji = MatNd_getRowView(J, idx);
    double cpEf[3], cpRef[3], nRE[3];
    x->ele[idx] = -RcsBody_distance(b1, b2, cpRef, cpEf, nRE);
    RcsBody_distanceGradient(controller->getGraph(), b1, b2, true,
                             cpRef, cpEf, nRE, &Ji);
    idx++;
  }

}

double IkSolverQPOA::solveLeftInverse(MatNd* dq, const MatNd* dx, const MatNd* dH, const MatNd* activation, double lambda, bool recomputeKinematics)
{
  // Compute system matrices
  const unsigned int nq = controller->getGraph()->nJ;
  const unsigned int nx = controller->getTaskDim();

  // Compute the joint coupling matrices. This is required only for joints
  // that have a non-constant coupling (e.g. polynomial coupling as in some
  // parallel linkage joints).
  MatNd* Aj = MatNd_create(nq, nq);
  MatNd* invAj = MatNd_create(nq, nq);

  RcsGraph_coupledJointMatrix(controller->getGraph(), Aj, invAj);
  const unsigned int nqr = Aj->n;

  // Joint space weghting matrix
  MatNd* invWq = MatNd_create(nq, 1);
  RcsGraph_getInvWq(controller->getGraph(), invWq, RcsStateIK);
  MatNd_preMulSelf(invWq, invAj);
  MatNd_setElementsTo(invWq, 1.0);

  // Jacobian for task constraints
  MatNd* J = MatNd_create(nx, nq);
  controller->computeJ(J, activation);

  // Task constraints
  MatNd* dxr_lb = MatNd_clone(dx);
  controller->compressToActive(dxr_lb, dx, activation);
  MatNd* dxr_ub = MatNd_clone(dxr_lb);

  // Collision constraints
  MatNd* Jc = MatNd_create(1, nq);
  MatNd* xc = MatNd_create(1, 1);
  getCollisionConstraints(Jc, xc);
  //MatNd_appendRows(dxr_lb, xc);
  MatNd_appendRows(J, Jc);
  MatNd* coll_ub = MatNd_create(xc->m, xc->n);
  MatNd_setElementsTo(coll_ub, 100000.0);
  //MatNd_appendRows(dxr_ub, coll_ub);
  MatNd_destroy(coll_ub);

  // Joint limit constraints
  MatNd* q_lower = MatNd_create(controller->getGraph()->dof, 1);
  MatNd* q_upper = MatNd_create(controller->getGraph()->dof, 1);
  RcsGraph_getJointLimits(controller->getGraph(), q_lower, q_upper, RcsStateFull);
  MatNd_subSelf(q_lower, controller->getGraph()->q);
  MatNd_subSelf(q_upper, controller->getGraph()->q);
  RcsGraph_stateVectorToIKSelf(controller->getGraph(), q_lower);
  RcsGraph_stateVectorToIKSelf(controller->getGraph(), q_upper);

  const int nc = J->m;
  // QProblem qp(nq, nc);
  Options myOptions;
  setOptions(myOptions);
  // qp.setOptions(myOptions);



  MatNd* H_ = MatNd_create(nq, nq);
  MatNd_setDiag(H_, invWq);

  // Setup QP
  real_t* H = H_->ele;
  real_t* A = J->ele;
  real_t* g = dH->ele;
  real_t* lb = NULL;//q_lower->ele;
  real_t* ub = NULL;//q_upper->ele;
  real_t* lbA = dxr_lb->ele;
  real_t* ubA = dxr_ub->ele;

  /* Solve first QP. */
  int_t nWSR = 1000;
  returnValue ret;

  if (qp_hack==NULL)
  {
    // ret = qp.init(H, g, A, lb, ub, lbA, ubA, nWSR);
    qp_hack = new QProblem(nq, nc);
    qp_hack->setOptions(myOptions);
    ret = qp_hack->init(H, g, A, lb, ub, lbA, ubA, nWSR);
  }
  else
  {
    // ret = qp.hotstart(g, lb, ub, lbA, ubA, nWSR);
    ret = qp_hack->hotstart(g, lb, ub, lbA, ubA, nWSR);
    //ret = qp_hack->init(H, g, A, lb, ub, lbA, ubA, nWSR);
  }


  /* Get and print solution of second QP. */
  MatNd* dq_ = MatNd_create(nqr, 1);
  real_t* xOpt = dq_->ele;
  // qp.getPrimalSolution(xOpt);
  qp_hack->getPrimalSolution(xOpt);

  if (ret != SUCCESSFUL_RETURN)
  {
    RLOG(0, "No solution: %d (%s)", ret, errorCode(ret));
    MatNd_setZero(dq_);
  }

  //qp.printOptions();

  // Expand from coupled joint space to constraint joints and uncompress
  // to all states
  MatNd_reshapeCopy(dq, dq_);
  //MatNd_mul(dq, Aj, dq_);
  RcsGraph_stateVectorFromIKSelf(controller->getGraph(), dq);

  MatNd_destroy(Aj);
  MatNd_destroy(invAj);
  MatNd_destroy(invWq);
  MatNd_destroy(J);
  MatNd_destroy(H_);
  MatNd_destroy(dxr_lb);
  MatNd_destroy(dxr_ub);
  MatNd_destroy(q_lower);
  MatNd_destroy(q_upper);
  MatNd_destroy(dq_);
  MatNd_destroy(Jc);
  MatNd_destroy(xc);

  // return qp.getObjVal();
  return qp_hack->getObjVal();
}

double IkSolverQPOA::solveRightInverse(MatNd* dq, const MatNd* dx, const MatNd* dH, const MatNd* activation, double lambda)
{
  // Compute system matrices
  unsigned int nq = controller->getGraph()->nJ;
  unsigned int nx = controller->getTaskDim();

  // Compute the joint coupling matrices. This is required only for joints
  // that have a non-constant coupling (e.g. polynomial coupling as in some
  // parallel linkage joints).
  MatNd* Aj = MatNd_create(nq, nq);
  MatNd* invAj = MatNd_create(nq, nq);

  RcsGraph_coupledJointMatrix(controller->getGraph(), Aj, invAj);
  unsigned int nqr = Aj->n;

  // Joint space weghting matrix
  MatNd* invWq = MatNd_create(nq, 1);
  RcsGraph_getInvWq(controller->getGraph(), invWq, RcsStateIK);
  MatNd_preMulSelf(invWq, invAj);
  MatNd_setElementsTo(invWq, 1.0);

  // Jacobian for task constraints
  MatNd* J = MatNd_create(nx, nq);
  controller->computeJ(J, activation);

  // Task constraints
  MatNd* dxr_lb = MatNd_clone(dx);
  controller->compressToActive(dxr_lb, dx, activation);
  MatNd* dxr_ub = MatNd_clone(dxr_lb);
  unsigned int nxa = dxr_lb->m;

  // Collision constraints
  MatNd* Jc = MatNd_create(1, nq);
  MatNd* xc = MatNd_create(1, 1);
  getCollisionConstraints(Jc, xc);
  MatNd* coll_ub = MatNd_create(xc->m, xc->n);
  MatNd_setElementsTo(coll_ub, 100000.0);
  MatNd* coll_lb = MatNd_create(xc->m, xc->n);
  MatNd_setElementsTo(coll_lb, -100000.0);

  MatNd_appendRows(dxr_ub, coll_ub);
  MatNd_appendRows(dxr_lb, coll_lb);
  MatNd_appendRows(J, Jc);
  MatNd_postMulSelf(J, Aj);


  // Joint limit constraints
  MatNd* q_lower = MatNd_create(controller->getGraph()->dof, 1);
  MatNd* q_upper = MatNd_create(controller->getGraph()->dof, 1);
  MatNd* q_dot_limit = MatNd_create(controller->getGraph()->dof, 1);
  RcsGraph_getJointLimits(controller->getGraph(), q_lower, q_upper, RcsStateFull);
  RcsGraph_getSpeedLimits(controller->getGraph(), q_dot_limit, RcsStateFull);
  MatNd_subSelf(q_lower, controller->getGraph()->q);
  MatNd_subSelf(q_upper, controller->getGraph()->q);

  //for (unsigned int i = 0; i < controller->getGraph()->dof; ++i)
  //{
  //  const double dt = 0.01;
  //  q_lower->ele[i] = std::min(q_lower->ele[i], -q_dot_limit->ele[i]*dt);
  //  q_upper->ele[i] = std::min(q_upper->ele[i], +q_dot_limit->ele[i]*dt);
  //}

  RcsGraph_stateVectorToIKSelf(controller->getGraph(), q_lower);
  RcsGraph_stateVectorToIKSelf(controller->getGraph(), q_upper);
  MatNd_preMulSelf(q_lower, invAj);
  MatNd_preMulSelf(q_upper, invAj);

  int nc = J->m;
  QProblem qp(nqr, nc);
  Options myOptions;
  setOptions(myOptions);
  qp.setOptions(myOptions);

  MatNd* H_ = MatNd_create(nqr, nqr);
  MatNd_setDiag(H_, invWq);

  // Setup QP
  real_t* H = H_->ele;
  real_t* A = J->ele;
  real_t* g = dH->ele;
  real_t* lb = NULL;
  real_t* ub = NULL;
  real_t* lbA = dxr_lb->ele;
  real_t* ubA = dxr_ub->ele;

  /* Solve first QP. */
  int_t nWSR = 100;
  returnValue ret = qp.init(H, g, A, lb, ub, lbA, ubA, nWSR);

  /* Get and print solution of second QP. */
  MatNd* dq_ = MatNd_create(nqr, 1);

  if (ret == SUCCESSFUL_RETURN)
  {
    real_t* xOpt = dq_->ele;
    qp.getPrimalSolution(xOpt);

    MatNd* dx_opt = MatNd_create(dxr_lb->m, dxr_lb->n);
    //real_t* yOpt = dx_opt->ele;
    //qp.getDualSolution(yOpt);
    //REXEC(1)
    //{
    //  RLOG(1, "dx dx_qp");
    //  MatNd_printTwoArraysDiff(dxr_lb, dx_opt, 5);
    //}
    MatNd_destroy(dx_opt);
  }
  else
  {
    RLOG(0, "No solution for task-only QP: %d (%s)", ret, errorCode(ret));
  }

  double fail = 1.0;

#if 1
  // Solve second QP
  MatNd_reshapeAndSetZero(dxr_lb, nxa, 1);
  MatNd_reshapeAndSetZero(dxr_ub, nxa, 1);
  MatNd_appendRows(dxr_ub, coll_ub);
  MatNd_appendRows(dxr_lb, xc);

  lb = q_lower->ele;
  ub = q_upper->ele;
  ret = qp.hotstart(g, lb, ub, lbA, ubA, nWSR);

  this->nConstraints = qp.getNAC();

  if (ret == SUCCESSFUL_RETURN)
  {
    MatNd* dq2_ = MatNd_create(nqr, 1);
    real_t* xOpt = dq2_->ele;
    qp.getPrimalSolution(xOpt);
    MatNd_addSelf(dq_, dq2_);
    MatNd_destroy(dq2_);
  }
  else
  {
    //MatNd_setZero(dq_);
    fail = -1.0;
    RLOG(0, "No solution for constraint-only QP: %d (%s)", ret, errorCode(ret));
  }
#endif

  // Expand from coupled joint space to constraint joints and uncompress
  // to all states
  MatNd_reshape(dq, nq, 1);
  MatNd_mul(dq, Aj, dq_);
  RcsGraph_stateVectorFromIKSelf(controller->getGraph(), dq);

  MatNd_destroy(Aj);
  MatNd_destroy(invAj);
  MatNd_destroy(invWq);
  MatNd_destroy(J);
  MatNd_destroy(H_);
  MatNd_destroy(dxr_lb);
  MatNd_destroy(dxr_ub);
  MatNd_destroy(q_lower);
  MatNd_destroy(q_upper);
  MatNd_destroy(dq_);
  MatNd_destroy(Jc);
  MatNd_destroy(xc);
  MatNd_destroy(coll_lb);
  MatNd_destroy(coll_ub);
  MatNd_destroy(q_dot_limit);

  return fail;// qp.getObjVal();
}

}   // namespace Rcs


#else   //  defined (USE_QPOASES)

#include <Rcs_macros.h>

namespace Rcs
{
IkSolverQPOA::IkSolverQPOA(ControllerBase* controller) : IkSolverRMR(controller)
{
  RFATAL("QPOases not linked into RcsCore - solver not available");
}

IkSolverQPOA::~IkSolverQPOA() { }

void IkSolverQPOA::getCollisionConstraints(MatNd* J, MatNd* x) { }

double IkSolverQPOA::solveLeftInverse(MatNd* dq, const MatNd* dx, const MatNd* dH, const MatNd* activation, double lambda, bool recomputeKinematics)
{
  return 0.0;
}

double IkSolverQPOA::solveRightInverse(MatNd* dq, const MatNd* dx, const MatNd* dH, const MatNd* activation, double lambda)
{
  return 0.0;
}
}

#endif   // defined (USE_QPOASES)
