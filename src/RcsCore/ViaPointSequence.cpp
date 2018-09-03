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


/*******************************************************************************

  \todo:
  - getmaxvel for Lagrange multipliers
  - better polyroot function
  - scale to max. vel. (somehow)
  - analytical gradients
  - document
  - test what happens if constraint is shifted on the state. Does the global
    shape change?
  - Interactive test with OpenGL and MatNdWidget

*******************************************************************************/

#include "ViaPointSequence.h"
#include "Rcs_math.h"
#include "Rcs_macros.h"
#include "Rcs_utils.h"

#include <string>
#include <cstdio>


#define ERR_POS (1.0e-5)
#define ERR_VEL (1.0e-4)
#define ERR_ACC (1.0e-4)
#define ERR_INV (1.0e-3)

using namespace Rcs;

/*******************************************************************************
 *
 ******************************************************************************/
static inline void getUniqueFileName(char* fileName)
{
#if defined (_MSC_VER)
  char* ch = File_createUniqueName(fileName, "C:\\\\Temp\\trajectoy", "dat");
#else
  char* ch = File_createUniqueName(fileName, "trajectoy", "dat");
#endif
  RCHECK(ch);
}

/*******************************************************************************
 *
 ******************************************************************************/
ViaPointSequence::ViaPointSequence() :
  viaDescr(NULL), B(NULL), invB(NULL), x(NULL), p(NULL), computeAllParams(true)
{
}

/*******************************************************************************
 *
 ******************************************************************************/
ViaPointSequence::ViaPointSequence(const MatNd* viaDescr_) :
  viaDescr(NULL), B(NULL), invB(NULL), x(NULL), p(NULL), computeAllParams(true)
{
  init(viaDescr_);
}

/*******************************************************************************
 *
 ******************************************************************************/
ViaPointSequence::ViaPointSequence(const char* viaString) :
  viaDescr(NULL), B(NULL), invB(NULL), x(NULL), p(NULL), computeAllParams(true)
{
  MatNd* tmp = MatNd_createFromString(viaString);
  init(tmp);
  MatNd_destroy(tmp);
}

/*******************************************************************************
 *
 ******************************************************************************/
ViaPointSequence::~ViaPointSequence()
{
  MatNd_destroy(this->B);
  MatNd_destroy(this->invB);
  MatNd_destroy(this->x);
  MatNd_destroy(this->p);
  MatNd_destroy(this->viaDescr);
}

/*******************************************************************************
 * Copy constructor doing deep copying.
 ******************************************************************************/
ViaPointSequence::ViaPointSequence(const ViaPointSequence& copyFromMe):
  computeAllParams(copyFromMe.computeAllParams),
  constraintType(copyFromMe.constraintType),
  viaTime(copyFromMe.viaTime)
{
  this->viaDescr = MatNd_clone(copyFromMe.viaDescr);
  this->B = MatNd_clone(copyFromMe.B);
  this->invB = MatNd_clone(copyFromMe.invB);
  this->x = MatNd_clone(copyFromMe.x);
  this->p = MatNd_clone(copyFromMe.p);
}

/*******************************************************************************
 * Copy constructor doing deep copying.
 ******************************************************************************/
ViaPointSequence& ViaPointSequence::operator=(const ViaPointSequence& rhs)
{
  if (this == &rhs)
  {
    return *this;
  }

  this->viaDescr = MatNd_realloc(this->viaDescr, rhs.viaDescr->m, rhs.viaDescr->n);
  this->B = MatNd_realloc(this->B, rhs.B->m, rhs.B->n);
  this->invB = MatNd_realloc(this->invB, rhs.invB->m, rhs.invB->n);
  this->x = MatNd_realloc(this->x, rhs.x->m, rhs.x->n);
  this->p = MatNd_realloc(this->p, rhs.p->m, rhs.p->n);
  this->computeAllParams = rhs.computeAllParams;

  MatNd_copy(this->viaDescr, rhs.viaDescr);
  MatNd_copy(this->B, rhs.B);
  MatNd_copy(this->invB, rhs.invB);
  MatNd_copy(this->x, rhs.x);
  MatNd_copy(this->p, rhs.p);

  return *this;
}

/*******************************************************************************
 * Pointer version of copy
 ******************************************************************************/
ViaPointSequence* ViaPointSequence::clone() const
{
  return new ViaPointSequence(*this);
}

/*******************************************************************************
 * Needed by qsort
 ******************************************************************************/
static int DoubleComparator(const void* x, const void* y)
{
  if (*(double*) x > *(double*) y)
  {
    return 1;
  }
  else if (*(double*) x < *(double*) y)
  {
    return -1;
  }
  else
  {
    return 0;
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void ViaPointSequence::sort(MatNd* desc) const
{
  qsort(desc->ele, desc->m, desc->n*sizeof(double), DoubleComparator);
}

/*******************************************************************************
 *
 ******************************************************************************/
void ViaPointSequence::compressDescriptor(MatNd* desc) const
{
  for (unsigned int i=1; i<desc->m; ++i)
  {
    unsigned int flag = lround(MatNd_get2(desc, i, 4));

    if (flag==7)
    {
      desc->m = i+1;
      return;
    }

  }

}

/*******************************************************************************
 *
 ******************************************************************************/
bool ViaPointSequence::init(const MatNd* viaDescr_)
{
  RCHECK_MSG(viaDescr_->m >= 2, "m = %d (should be >= 2)", viaDescr_->m);
  RCHECK_MSG(viaDescr_->n >= 5, "n = %d (should be >= 5)", viaDescr_->n);


  // Resize arrays to allow for larger descriptors.
  if (this->viaDescr != viaDescr_)
  {
    this->viaDescr = MatNd_realloc(this->viaDescr, viaDescr_->m, viaDescr_->n);
    RCHECK(this->viaDescr);
    MatNd_copy(this->viaDescr, viaDescr_);
  }

  // Sort rows so that the time always increases. Sorting is done by row
  // swapping, rows with more than 5 elements get completely swapped. This
  // allows to add additional information to the descriptor columns right of
  // index 5.
  sort(this->viaDescr);

  // Disregard all constraints after a flag 7 constraint. This must be called
  // after sorting.
  if (this->computeAllParams==false)
  {
    // truncate trajectory to the first full constraint via
    compressDescriptor(viaDescr);
  }

  // Reset vectors to make sure the init function can be called several times.
  this->viaTime.clear();
  this->constraintType.clear();


  // Initial and final boundary conditions must be position, velocity and
  // acceleration. We do a fatal check here, since the following steps would
  // screw up memory allocation for the matrices otherwise.
  int bConstraint = lround(MatNd_get2(this->viaDescr, 0, 4));

  if (bConstraint != 7)
  {
    MatNd_printCommentDigits("viaDescr_", viaDescr_, 3);
    MatNd_printCommentDigits("this->viaDescr", this->viaDescr, 3);
    RFATAL("Initial condition lacks constraint: %d. Did you specify position"
           ", velocity and acceleration?", bConstraint);
  }

  bConstraint = lround(MatNd_get2(this->viaDescr, this->viaDescr->m-1, 4));

  if (bConstraint != 7)
  {
    MatNd_printCommentDigits("Descr", this->viaDescr, 3);
    RFATAL("Final condition lacks constraint: %d. Did you specify position"
           ", velocity and acceleration?", bConstraint);
  }


  // Count constraints, set up constraint vector and some helping arrays
  size_t nConstraints = computeNumberOfConstraints(this->viaDescr);
  this->x = MatNd_realloc(this->x, nConstraints, 1);
  RCHECK(this->x);
  this->x->m = 0;

  for (size_t i=0; i<this->viaDescr->m; i++)
  {
    unsigned int flag = lround(MatNd_get2(this->viaDescr, i, 4));

    if (Math_isBitSet(flag, VIA_POS))
    {
      this->constraintType.push_back(VIA_POS);
      this->viaTime.push_back(MatNd_get2(this->viaDescr, i, 0));
      this->x->ele[this->x->m] = MatNd_get(this->viaDescr, i, 1);
      this->x->m++;
    }

    if (Math_isBitSet(flag, VIA_VEL))
    {
      this->constraintType.push_back(VIA_VEL);
      this->viaTime.push_back(MatNd_get2(this->viaDescr, i, 0));
      this->x->ele[this->x->m] = MatNd_get(this->viaDescr, i, 2);
      this->x->m++;
    }

    if (Math_isBitSet(flag, VIA_ACC))
    {
      this->constraintType.push_back(VIA_ACC);
      this->viaTime.push_back(MatNd_get2(this->viaDescr, i, 0));
      this->x->ele[this->x->m] = MatNd_get(this->viaDescr, i, 3);
      this->x->m++;
    }

  }


  // Reallocate arrays to make sure the init function can be called several
  // times.
  this->B = MatNd_realloc(this->B, nConstraints, nConstraints);
  RCHECK(this->B);
  MatNd_setZero(this->B);
  computeB(this->B, this->viaDescr);


  // Delete arrays to make sure the init function can be called several times.
  this->invB = MatNd_realloc(this->invB, this->B->m, this->B->n);
  RCHECK(this->invB);


  // It is faster to solve the linear equation system. But we will need the
  // explicit inverse later for the gradient calculation.
  double det = MatNd_gaussInverse(this->invB, this->B);


  // Compute parameter vector
  this->p = MatNd_realloc(this->p, nConstraints, 1);
  RCHECK(this->p);
  MatNd_mul(this->p, this->invB, this->x);

  return (det==0.0) ? false : true;
}

/*******************************************************************************
 * Static function to compute the B matrix.
 ******************************************************************************/
void ViaPointSequence::computeB(MatNd* B, const MatNd* vDescr)
{
  // Vectors with vDescr->m-2 elements: The initial and final conditions are
  // not represented. Each element of the index vector corresponds to the
  // respective via point. If it is -1, the via point does not have a
  // position / velocity / acceleration constraint. If it is a value >=0, it
  // means the column index of the matrix B:
  // pos/vel/accIdx[viaPoint] = column of B of the parameter
  std::vector<int> pIdx, vIdx, aIdx;

  // Indices 0 - 5 correspond to a5...0. Their indices in matrix B are the
  // block (0,0,5,5). Therefore we start counting from 6, and skip the first
  // via point.
  int idxPi = 6;

  for (size_t i=1; i<vDescr->m-1; i++)
  {
    unsigned int flag = lround(MatNd_get2(vDescr, i, 4));

    // These flags ensure that we push the indices only once per row
    bool posIsUpdated = false;
    bool velIsUpdated = false;
    bool accIsUpdated = false;

    if (Math_isBitSet(flag, VIA_POS))
    {
      pIdx.push_back(idxPi);

      if (!Math_isBitSet(flag, VIA_VEL))
      {
        vIdx.push_back(-1);
        velIsUpdated = true;
      }

      if (!Math_isBitSet(flag, VIA_ACC))
      {
        aIdx.push_back(-1);
        accIsUpdated = true;
      }

      idxPi++;
    }

    if (Math_isBitSet(flag, VIA_VEL))
    {
      vIdx.push_back(idxPi);

      if (!Math_isBitSet(flag, VIA_POS))
      {
        pIdx.push_back(-1);
        posIsUpdated = true;
      }

      if ((!Math_isBitSet(flag, VIA_ACC)) && (!accIsUpdated))
      {
        aIdx.push_back(-1);
      }

      idxPi++;
    }

    if (Math_isBitSet(flag, VIA_ACC))
    {
      aIdx.push_back(idxPi);

      if ((!Math_isBitSet(flag, VIA_POS)) && (!posIsUpdated))
      {
        pIdx.push_back(-1);
      }

      if ((!Math_isBitSet(flag, VIA_VEL)) && (!velIsUpdated))
      {
        vIdx.push_back(-1);
      }

      idxPi++;
    }

  }


  // From here on we construct the matrix B
  size_t row = 0;

  for (size_t i=0; i<vDescr->m; i++)
  {
    double t = MatNd_get2(vDescr, i, 0);
    double t2 = t*t;
    double t3 = t2*t;
    double t4 = t2*t2;
    double t5 = t3*t2;

    unsigned int flag = lround(MatNd_get(vDescr, i, 4));

    // Position polynomial elements first
    if (Math_isBitSet(flag, VIA_POS))
    {
      MatNd_set(B, row, 0, t5);
      MatNd_set(B, row, 1, t4);
      MatNd_set(B, row, 2, t3);
      MatNd_set(B, row, 3, t2);
      MatNd_set(B, row, 4, t);
      MatNd_set(B, row, 5, 1.0);

      // After the first via point, we need to fill the lower triangular
      // sub-matrices according to the Lagrange Multipliers pi. Here we
      // assume that the via-descriptor is sorted with rows being in
      // increasing time.
      for (size_t j=1; j<i; j++)
      {
        double t_base = MatNd_get(vDescr, j, 0);

        if (t_base>=t)
        {
          MatNd_printCommentDigits("viaDesc", vDescr, 5);
          RFATAL("Row %d, pivot %d: t_base: %f t: %f",
                 (int) i, (int) j, t_base, t);
        }

        if (pIdx[j-1] != -1)
        {
          MatNd_set(B, row, pIdx[j-1], pow(t-t_base, 5));
        }

        if (vIdx[j-1] != -1)
        {
          MatNd_set(B, row, vIdx[j-1], pow(t-t_base, 4));
        }

        if (aIdx[j-1] != -1)
        {
          MatNd_set(B, row, aIdx[j-1], pow(t-t_base, 3));
        }

      }

      row++;
    }

    // Velocity polynomial elements second
    if (Math_isBitSet(flag, VIA_VEL))
    {
      MatNd_set(B, row, 0, 5.0*t4);
      MatNd_set(B, row, 1, 4.0*t3);
      MatNd_set(B, row, 2, 3.0*t2);
      MatNd_set(B, row, 3, 2.0*t);
      MatNd_set(B, row, 4, 1.0);

      // After the first via point, we need to fill the lower triangular
      // sub-matrices according to the Lagrange Multipliers pi
      for (size_t j=1; j<i; j++)
      {
        double t_base = MatNd_get(vDescr, j, 0);

        if (t_base>=t)
        {
          MatNd_printCommentDigits("viaDesc", vDescr, 5);
          RFATAL("Row %d, pivot %d: t_base: %f t: %f",
                 (int) i, (int) j, t_base, t);
        }

        if (vIdx[j-1] != -1)
        {
          MatNd_set(B, row, vIdx[j-1], 4.0*pow(t-t_base, 3));
        }

        if (pIdx[j-1] != -1)
        {
          MatNd_set(B, row, pIdx[j-1], 5.0*pow(t-t_base, 4));
        }

        if (aIdx[j-1] != -1)
        {
          MatNd_set(B, row, aIdx[j-1], 3.0*pow(t-t_base, 2));
        }

      }

      row++;
    }


    // Acceleration polynomial elements second
    if (Math_isBitSet(flag, VIA_ACC))
    {
      MatNd_set(B, row, 0, 20.0*t3);
      MatNd_set(B, row, 1, 12.0*t2);
      MatNd_set(B, row, 2, 6.0*t);
      MatNd_set(B, row, 3, 2.0);

      // After the first via point, we need to fill the lower triangular
      // sub-matrices according to the Lagrange Multipliers pi
      for (size_t j=1; j<i; j++)
      {
        double t_base = MatNd_get(vDescr, j, 0);

        if (t_base>=t)
        {
          MatNd_printCommentDigits("viaDesc", vDescr, 5);
          RFATAL("Row %d, pivot %d: t_base: %f t: %f",
                 (int) i, (int) j, t_base, t);
        }

        if (aIdx[j-1] != -1)
        {
          MatNd_set(B, row, aIdx[j-1], 6.0*(t-t_base));
        }

        if (vIdx[j-1] != -1)
        {
          MatNd_set(B, row, vIdx[j-1], 12.0*pow(t-t_base, 2));
        }

        if (pIdx[j-1] != -1)
        {
          MatNd_set(B, row, pIdx[j-1], 20.0*pow(t-t_base, 3));
        }

      }

      row++;
    }

  }   // for(size_t i=0;i<vDescr->m;i++)

}

/*******************************************************************************
 * We do the tests in computational effort's order, so that the function
 * returns rapidly once easy to calculate failures happen (e.g. without
 * matrix multiplies).
 ******************************************************************************/
bool ViaPointSequence::check() const
{
  bool success = true;

  // Check if viaDescr has successfully allocated
  if (this->viaDescr == NULL)
  {
    RLOGS(1, "this->viaDescr is NULL");
    return false;
  }

  // From here on, this->viaDescr exists. We first check for the proper number
  // of columns
  if (this->viaDescr->n < 5)
  {
    RLOGS(1, "this->viaDescr has wrong number of columns: %d (should be >= 5)",
          this->viaDescr->n);
    return false;
  }

  // It must have 2 or more rows
  if (this->viaDescr->m < 2)
  {
    RLOGS(1, "this->viaDescr has wrong number of rows: %d (must be >= 2)",
          this->viaDescr->m);
    return false;
  }

  // Initial boundary conditions must be position, velocity and acceleration
  if (MatNd_get2(this->viaDescr, 0, 4) != 7)
  {
    RLOGS(1, "Initial condition lacks constraint: %f. Did you specify position"
          ", velocity and acceleration?", MatNd_get(this->viaDescr, 0, 4));
    return false;
  }

  // Final boundary conditions must be position, velocity and acceleration
  if (MatNd_get2(this->viaDescr, this->viaDescr->m-1, 4) != 7)
  {
    RLOGS(1, "Initial condition lacks constraint: %f. Did you specify position"
          ", velocity and acceleration?", MatNd_get(this->viaDescr, 0, 4));
    return false;
  }

  // We must have at least 6 constraints (for 0 or more via points)
  unsigned int nConstraints = computeNumberOfConstraints(this->viaDescr);
  if (nConstraints < 6)
  {
    RLOGS(1, "Less than 6 constraints: %u", nConstraints);
    return false;
  }

  // Check if p has successfully allocated
  if (this->p == NULL)
  {
    RLOGS(1, "this->p == NULL");
    return false;
  }

  // From here on, this->p exists. We check for the size of it
  if (this->p->m != nConstraints)
  {
    RLOGS(1, "this->p has not dimension of constraints: %u != %u",
          this->p->m, nConstraints);
    return false;
  }

  // It must be a row vector
  if (this->p->n != 1)
  {
    RLOGS(1, "this->p has %d columns (should be 1)", this->p->n);
    return false;
  }

  // Check if x has successfully allocated
  if (this->x == NULL)
  {
    RLOGS(1, "this->x == NULL");
    return false;
  }

  // From here on, this->p exists. We check for the size of it
  if (this->x->m != nConstraints)
  {
    RLOGS(1, "this->x has not dimension of constraints: %u != %u",
          this->x->m, nConstraints);
    return false;
  }

  // It must be a row vector
  if (this->x->n != 1)
  {
    RLOGS(1, "this->x has %d columns (should be 1)", this->x->n);
    return false;
  }

  // Next we check if the time is strictly monotonically increasing
  for (unsigned int i=1; i<this->viaDescr->m; i++)
  {
    const double t_curr = MatNd_get(this->viaDescr, i, 0);
    const double t_prev = MatNd_get(this->viaDescr, i-1, 0);
    if (t_curr <= t_prev)
    {
      RLOGS(1, "Index %u: t_curr=%f <=  t_prev=%f", i, t_curr, t_prev);
      REXEC(2)
      {
        MatNd_printCommentDigits("viaDescr", this->viaDescr, 6);
      }
      return false;
    }
  }

  // Next we check if B has finite values
  if (MatNd_isFinite(this->B)==false)
  {
    REXEC(1)
    {
      RMSG("Found non-finite elements in B!");
      MatNd_printCommentDigits("B", B, 4);
    }
    return false;
  }

  // Next we check if the inversion of B succeeded
  MatNd* B_invB = MatNd_create(this->B->m, this->B->m);
  MatNd_mul(B_invB, this->B, this->invB);

  if (MatNd_isIdentity(B_invB, ERR_INV)==false)
  {
    RLOG(1, "Inversion with accuracy %g failed for B*inv(B)", ERR_INV);
    REXEC(2)
    {
      MatNd_printCommentDigits("B*inv(B)", B_invB, 12);
    }
    MatNd_destroy(B_invB);
    return false;
  }

  MatNd_destroy(B_invB);

  // Next we check if the trajectory matches the constraints
  unsigned int constraintErrors = 0;

  for (size_t j=0; j<this->viaDescr->m; j++)
  {
    double t_via = MatNd_get(this->viaDescr, j, 0);
    unsigned int flag = lround(MatNd_get(this->viaDescr, j, 4));

    double xt, xt_dot, xt_ddot;
    computeTrajectoryPoint(xt, xt_dot, xt_ddot, t_via);

    // Position
    if (Math_isBitSet(flag, VIA_POS))
    {

      if (fabs(MatNd_get(this->viaDescr, j, 1)-xt)>ERR_POS)
      {
        RLOG(2, "Error at position constraint at t = %.3f:   "
             "x_des = %.3f   x = %.3f   err = %g",
             t_via, MatNd_get(this->viaDescr, j, 1), xt,
             fabs(MatNd_get(this->viaDescr, j, 1)-xt));
        constraintErrors++;
      }
    }

    // Velocity
    if (Math_isBitSet(flag, VIA_VEL))
    {
      if (fabs(MatNd_get(this->viaDescr, j, 2)-xt_dot)>ERR_VEL)
      {
        RLOG(2, "Error at velocity constraint at t = %.3f:   "
             "x_dot_des = %.3f   x_dot = %.3f   err = %g",
             t_via, MatNd_get(this->viaDescr, j, 2), xt_dot,
             fabs(MatNd_get(this->viaDescr, j, 2)-xt_dot));
        constraintErrors++;
      }
    }

    // Acceleration
    if (Math_isBitSet(flag, VIA_ACC))
    {
      if (fabs(MatNd_get(this->viaDescr, j, 3)-xt_ddot)>ERR_ACC)
      {
        RLOG(2, "Error at acceleration constraint at t = %.3f:   "
             "x_ddot_des = %.3f   x_ddot = %.3f   err = %g",
             t_via, MatNd_get(this->viaDescr, j, 3), xt_ddot,
             fabs(MatNd_get(this->viaDescr, j, 3)-xt_ddot));
        constraintErrors++;
      }
    }

  }

  if (constraintErrors>0)
  {
    RLOGS(1, "Found %d constraint errors", constraintErrors);
    success = false;
  }

  return success;
}

/*******************************************************************************
 *
 ******************************************************************************/
static bool mergeViaPoints(double* dst, const double* newPt)
{
  bool conflict = false;
  int newFlag = lround(newPt[4]);
  int dstFlag = lround(dst[4]);

  dstFlag |= newFlag;
  dst[4] = dstFlag;


  if (Math_isBitSet(newFlag, VIA_POS))
  {
    if (dst[1] != newPt[1])
    {
      conflict = true;
    }
    dst[1] = newPt[1];
  }

  if (Math_isBitSet(newFlag, VIA_VEL))
  {
    if (dst[2] != newPt[2])
    {
      conflict = true;
    }
    dst[2] = newPt[2];
  }

  if (Math_isBitSet(newFlag, VIA_ACC))
  {
    if (dst[3] != newPt[3])
    {
      conflict = true;
    }
    dst[3] = newPt[3];
  }

  return conflict;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool ViaPointSequence::addViaPoint(double t, double x, double x_dot,
                                   double x_ddot, int flag)
{
  MatNd* newRow = NULL;
  MatNd_fromStack(newRow, 1, 5);
  newRow->ele[0] = t;
  newRow->ele[1] = x;
  newRow->ele[2] = x_dot;
  newRow->ele[3] = x_ddot;
  newRow->ele[4] = flag;

  if (t < t0())
  {
    if (flag != 7)
    {
      return false;
    }
    MatNd_prependRows(this->viaDescr, newRow);
  }
  else  if (t == t0())
  {
    mergeViaPoints(this->viaDescr->ele, newRow->ele);
  }
  else if (t == t1())
  {
    mergeViaPoints(MatNd_getRowPtr(this->viaDescr, viaDescr->m-1), newRow->ele);
  }
  else if (t > t1())
  {
    if (flag != 7)
    {
      return false;
    }
    MatNd_appendRows(this->viaDescr, newRow);
  }
  else   // t > t0 and t < t1
  {
    for (unsigned int i=0; i<this->viaDescr->m-1; ++i)
    {
      double t_lower = MatNd_get(viaDescr, i, 0);
      double t_upper = MatNd_get(viaDescr, i+1, 0);

      if ((t>t_lower) && (t<t_upper))
      {
        MatNd_insertRows(this->viaDescr, i, newRow, 0, 1);
      }
      else if (t==t_lower)
      {
        mergeViaPoints(MatNd_getRowPtr(this->viaDescr, i), newRow->ele);
      }
      else if (t==t_upper)
      {
        mergeViaPoints(MatNd_getRowPtr(this->viaDescr, i+1), newRow->ele);
      }
    }
  }

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
void ViaPointSequence::gnuplot(double dt, int flag) const
{
  gnuplot(t0(), t1(), dt, flag);
}

/*******************************************************************************
 *
 ******************************************************************************/
void ViaPointSequence::gnuplot(double t0, double t1, double dt, int flag) const
{
  if (t1 <= t0)
  {
    RLOG(1, "t1 <= t0: t1=%f t0=%f", t1, t0);
    return;
  }

  // Calculate trajectory
  MatNd* traj = MatNd_create(3, 4*lround((t1-t0)/dt));
  computeTrajectory(traj, t0, t1, dt);

  const size_t maxFileNameSize = 64;

  // Do Gnuplot stuff
  char fTraj[maxFileNameSize];
  getUniqueFileName(fTraj);
  MatNd_transposeSelf(traj);
  MatNd_toFile(traj, fTraj);

  MatNd* viaPos = MatNd_create(this->x->m, 2);
  MatNd* viaVel = MatNd_create(this->x->m, 2);
  MatNd* viaAcc = MatNd_create(this->x->m, 2);

  MatNd_reshape(viaPos, 0, 2);
  MatNd_reshape(viaVel, 0, 2);
  MatNd_reshape(viaAcc, 0, 2);



  for (size_t i=0; i<this->viaDescr->m; i++)
  {
    unsigned int flag = lround(MatNd_get(this->viaDescr, i, 4));
    double t = MatNd_get(this->viaDescr, i, 0);

    if (Math_isBitSet(flag, VIA_POS))
    {
      viaPos->m++;
      MatNd_set(viaPos, viaPos->m-1, 0, t);
      MatNd_set(viaPos, viaPos->m-1, 1, MatNd_get(this->viaDescr, i, 1));
    }

    if (Math_isBitSet(flag, VIA_VEL))
    {
      viaVel->m++;
      MatNd_set(viaVel, viaVel->m-1, 0, t);
      MatNd_set(viaVel, viaVel->m-1, 1, MatNd_get(this->viaDescr, i, 2));
    }

    if (Math_isBitSet(flag, VIA_ACC))
    {
      viaAcc->m++;
      MatNd_set(viaAcc, viaAcc->m-1, 0, t);
      MatNd_set(viaAcc, viaAcc->m-1, 1, MatNd_get(this->viaDescr, i, 3));
    }

  }

  char fdataPos[maxFileNameSize], fdataVel[maxFileNameSize],
       fdataAcc[maxFileNameSize];

  getUniqueFileName(fdataPos);
  getUniqueFileName(fdataVel);
  getUniqueFileName(fdataAcc);

  MatNd_toFile(viaPos, fdataPos);
  MatNd_toFile(viaVel, fdataVel);
  MatNd_toFile(viaAcc, fdataAcc);

  MatNd_destroy(viaPos);
  MatNd_destroy(viaVel);
  MatNd_destroy(viaAcc);



  char gpCmdPos[256];
  sprintf(gpCmdPos,
          "set grid\nplot \"%s\" u 1:2 w l title \"x\", \"%s\" "
          "u 1:2 w p pointsize 3 title \"x_via\"\n", fTraj, fdataPos);

  char gpCmdVel[256];
  sprintf(gpCmdVel,
          "set grid\nplot \"%s\" u 1:3 w l title \"x_dot\", "
          "\"%s\" u 1:2  title \"x_dot_via\" w p pointsize 3\n",
          fTraj, fdataVel);

  char gpCmdAcc[256];
  sprintf(gpCmdAcc,
          "set grid\nplot \"%s\" u 1:4 w l title \"x_ddot\", "
          "\"%s\" u 1:2  title \"x_ddot_via\" w p pointsize 3\n",
          fTraj, fdataAcc);

  char gpCmd[512];
  sprintf(gpCmd,
          "set grid\nplot \"%s\" u 1:2 w l title \"x\", \"%s\" u 1:3 w l"
          " title \"x_dot\", \"%s\" u 1:4 w l title \"x_ddot\", \"%s\" "
          "u 1:2 w p pointsize 3 title \"x_via\", \"%s\" u 1:2 w p pointsize"
          " 3 title \"x_dot_via\", \"%s\" u 1:2 w p pointsize 3 title"
          " \"x_ddot_via\"\n",
          fTraj, fTraj, fTraj, fdataPos, fdataVel, fdataAcc);

  char fAll[maxFileNameSize], fPos[maxFileNameSize], fVel[maxFileNameSize],
       fAcc[maxFileNameSize];

  getUniqueFileName(fAll);
  getUniqueFileName(fPos);
  getUniqueFileName(fVel);
  getUniqueFileName(fAcc);

  FILE* outDat = fopen(fAll, "w+");
  RCHECK(outDat);
  fprintf(outDat, "%s", gpCmd);
  fflush(outDat);
  fclose(outDat);

  outDat = fopen(fPos, "w+");
  RCHECK(outDat);
  fprintf(outDat, "%s", gpCmdPos);
  fflush(outDat);
  fclose(outDat);

  outDat = fopen(fVel, "w+");
  RCHECK(outDat);
  fprintf(outDat, "%s", gpCmdVel);
  fflush(outDat);
  fclose(outDat);

  outDat = fopen(fAcc, "w+");
  RCHECK(outDat);
  fprintf(outDat, "%s", gpCmdAcc);
  fflush(outDat);
  fclose(outDat);

#if !defined (_MSC_VER)
  std::string gpExec = "/usr/bin/gnuplot -persist ";
#else
  std::string gpExec = "wgnuplot.exe -persist ";
#endif

  int err = -1;

  if (flag==7)
  {
    err = system((gpExec+std::string(fAll)).c_str());
  }
  else
  {
    if (Math_isBitSet(flag, VIA_POS))
    {
      err = system((gpExec+std::string(fPos)).c_str());
    }

    if (Math_isBitSet(flag, VIA_VEL))
    {
      err = system((gpExec+std::string(fVel)).c_str());
    }

    if (Math_isBitSet(flag, VIA_ACC))
    {
      err = system((gpExec+std::string(fAcc)).c_str());
    }
  }

  if (err == -1)
  {
    RLOG(1, "Couldn't start gnuplot");
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
void ViaPointSequence::computeTrajectory(MatNd* traj, double dt) const
{
  computeTrajectory(traj, t0(), t1(), dt);
}

/*******************************************************************************
 *
 ******************************************************************************/
void ViaPointSequence::computeTrajectory(MatNd* traj, double t0, double t1,
                                         double dt) const
{
  int nSteps = lround((t1-t0)/dt);

  MatNd_reshape(traj, 4, nSteps+1);

  for (int i=0; i<nSteps+1; i++)
  {
    const double t = (double)(t1-t0)*i/nSteps+t0;
    MatNd_set(traj, 0, i, t);

    double xt, xt_dot, xt_ddot;
    computeTrajectoryPoint(xt, xt_dot, xt_ddot, t);

    MatNd_set(traj, 1, i, xt);
    MatNd_set(traj, 2, i, xt_dot);
    MatNd_set(traj, 3, i, xt_ddot);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void ViaPointSequence::computeTrajectoryPoint(double& xt, double& xt_dot,
                                              double& xt_ddot, double t) const
{
  const double t2 = t*t;
  const double t3 = t2*t;
  const double t4 = t2*t2;
  const double t5 = t3*t2;

  const double a5 = MatNd_get2(p, 0, 0);
  const double a4 = MatNd_get2(p, 1, 0);
  const double a3 = MatNd_get2(p, 2, 0);
  const double a2 = MatNd_get2(p, 3, 0);
  const double a1 = MatNd_get2(p, 4, 0);
  const double a0 = MatNd_get2(p, 5, 0);


  // Trajectory according to boundary constraints
  xt      = a5*t5 + a4*t4 + a3*t3 + a2*t2 + a1*t + a0;
  xt_dot  = 5.0*a5*t4 + 4.0*a4*t3 + 3.0*a3*t2 + 2.0*a2*t + a1;
  xt_ddot = 20.0*a5*t3 + 12.0*a4*t2 + 6.0*a3*t + 2.0*a2;


  // Lagrange Multiplier contribution. We spare out the first and last 3
  // indices, since they correspond to the initial and target boundary
  // conditions.
  for (size_t j=3; j<this->constraintType.size()-3; j++)
  {
    const double dt = t-this->viaTime[j] > 0.0 ? t-this->viaTime[j] : 0.0;

    if (dt > 0.0)
    {
      const double dt2 = dt*dt;
      const double dt3 = dt2*dt;
      const double param = MatNd_get(p, j+3, 0);

      if (this->constraintType[j]==VIA_POS)
      {
        xt      += param*dt3*dt2;
        xt_dot  += param*5.0*dt2*dt2;
        xt_ddot += param*20.0*dt3;
      }
      else if (this->constraintType[j]==VIA_VEL)
      {
        xt      += param*dt2*dt2;
        xt_dot  += param*4.0*dt3;
        xt_ddot += param*12.0*dt2;
      }
      else if (this->constraintType[j]==VIA_ACC)
      {
        xt      += param*dt3;
        xt_dot  += param*3.0*dt2;
        xt_ddot += param*6.0*dt;
      }
    }

  }
}

/*******************************************************************************
 *
 ******************************************************************************/
double ViaPointSequence::computeTrajectoryPos(double t) const
{
#if 0
  double xt, xt_dot, xt_ddot;
  computeTrajectoryPoint(xt, xt_dot, xt_ddot, t);
#else
  MatNd* rhs = NULL;
  MatNd_create2(rhs, B->m, 1);
  computeRHS(rhs, t);
  double xt = VecNd_innerProduct(rhs->ele, this->p->ele, B->m);
  MatNd_destroy(rhs);
#endif

  return xt;
}

/*******************************************************************************
 *
 ******************************************************************************/
double ViaPointSequence::computeTrajectoryVel(double t) const
{
  double xt, xt_dot, xt_ddot;
  computeTrajectoryPoint(xt, xt_dot, xt_ddot, t);
  return xt_dot;
}

/*******************************************************************************
 *
 ******************************************************************************/
double ViaPointSequence::computeTrajectoryAcc(double t) const
{
  double xt, xt_dot, xt_ddot;
  computeTrajectoryPoint(xt, xt_dot, xt_ddot, t);
  return xt_ddot;
}

/*******************************************************************************
 *
 ******************************************************************************/
double ViaPointSequence::computeTrajectoryJerk(double t) const
{
  const double t2 = t*t;
  const double a5 = MatNd_get(p, 0, 0);
  const double a4 = MatNd_get(p, 1, 0);
  const double a3 = MatNd_get(p, 2, 0);


  // Trajectory according to boundary constraints
  double xt_dddot = 60.0*a5*t2 + 24.0*a4*t + 6.0*a3;


  // Lagrange Multiplier contribution
  for (size_t j=3; j<this->constraintType.size()-3; j++)
  {
    const double dt = t-this->viaTime[j]>0.0 ? t-this->viaTime[j]:0.0;
    const double dt2 = dt*dt;
    const double param = MatNd_get(p, j+3, 0);

    if (this->constraintType[j]==VIA_POS)
    {
      xt_dddot += param*60.0*dt2;
    }

    if (this->constraintType[j]==VIA_VEL)
    {
      xt_dddot += param*24.0*dt;
    }

    if (this->constraintType[j]==VIA_ACC)
    {
      xt_dddot += param*6.0;
    }

  }

  return xt_dddot;
}

/*******************************************************************************
 *
 ******************************************************************************/
void ViaPointSequence::print() const
{
  unsigned int nConstraintsPos = 0;
  unsigned int nConstraintsVel = 0;
  unsigned int nConstraintsAcc = 0;

  for (unsigned int i=0; i<this->constraintType.size(); i++)
  {
    if (this->constraintType[i]==VIA_POS)
    {
      nConstraintsPos++;
    }
    else if (this->constraintType[i]==VIA_VEL)
    {
      nConstraintsVel++;
    }
    else if (this->constraintType[i]==VIA_ACC)
    {
      nConstraintsAcc++;
    }
  }

  MatNd_printCommentDigits("Via point descriptor:", this->viaDescr, 6);

  printf("nTimePoints: %d   nConstraintsPos: %u   nConstraintsVel: %u   "
         "nConstraintsAcc: %u\n",
         this->viaDescr->m, nConstraintsPos, nConstraintsVel, nConstraintsAcc);

  MatNd_printCommentDigits("B", this->B, 6);
  MatNd_printCommentDigits("invB", this->invB, 6);

  printf("%u time points: ", (unsigned int) this->viaTime.size());
  for (size_t i=0; i<this->viaTime.size(); i++)
  {
    printf("%.3f ", this->viaTime[i]);
  }
  printf("\n");

  printf("%u constraints: ", (unsigned int) this->constraintType.size());
  for (size_t i=0; i<this->constraintType.size(); i++)
  {
    if (this->constraintType[i]==VIA_POS)
    {
      printf("%d: POS ", (int) i);
    }
    else if (this->constraintType[i]==VIA_VEL)
    {
      printf("%d: VEL ", (int) i);
    }
    else if (this->constraintType[i]==VIA_ACC)
    {
      printf("%d: ACC ", (int) i);
    }
    else
    {
      printf("ERROR");
    }
  }
  printf("\n");

  MatNd_printCommentDigits("x", this->x, 6);
  MatNd_printCommentDigits("p", this->p, 6);

  printf("\n");




  // Print the trajectory at the via points
  unsigned int constraintErrors = 0;

  for (size_t j=0; j<this->viaDescr->m; j++)
  {
    double t_via = MatNd_get(this->viaDescr, j, 0);
    unsigned int flag = lround(MatNd_get(this->viaDescr, j, 4));

    double xt, xt_dot, xt_ddot;
    computeTrajectoryPoint(xt, xt_dot, xt_ddot, t_via);

    // Position
    if (Math_isBitSet(flag, VIA_POS))
    {
      RLOGS(0, "Found position constraint at t = %.3f:   "
            "x_des = %.3f   x = %.3f",
            t_via, MatNd_get(this->viaDescr, j, 1), xt);

      if (fabs(MatNd_get(this->viaDescr, j, 1)-xt)>ERR_POS)
      {
        RLOGS(0, "ERROR is %g !!!",
              fabs(MatNd_get(this->viaDescr, j, 1)-xt));
        constraintErrors++;
      }
    }

    // Velocity
    if (Math_isBitSet(flag, VIA_VEL))
    {
      RLOGS(0, "Found velocity constraint at t = %.3f:   "
            "x_dot_des = %.3f   x_dot = %.3f",
            t_via, MatNd_get(this->viaDescr, j, 2), xt_dot);

      if (fabs(MatNd_get(this->viaDescr, j, 2)-xt_dot)>ERR_VEL)
      {
        RLOGS(0, "ERROR is %g !!!",
              fabs(MatNd_get(this->viaDescr, j, 2)-xt_dot));
        constraintErrors++;
      }
    }

    // Acceleration
    if (Math_isBitSet(flag, VIA_ACC))
    {
      RLOGS(0, "Found acceleration constraint at t = %.3f:   "
            "x_ddot_des = %.3f   x_ddot = %.3f",
            t_via, MatNd_get(this->viaDescr, j, 3), xt_ddot);

      if (fabs(MatNd_get(this->viaDescr, j, 3)-xt_ddot)>ERR_ACC)
      {
        RLOGS(0, "ERROR is %g !!!",
              fabs(MatNd_get(this->viaDescr, j, 3)-xt_ddot));
        constraintErrors++;
      }
    }

  }

  RLOGS(0, "Found %d constraint errors", constraintErrors);
}

/*******************************************************************************
 *
 ******************************************************************************/
double ViaPointSequence::t0() const
{
  return MatNd_get2(this->viaDescr, 0, 0);
}

/*******************************************************************************
 *
 ******************************************************************************/
double ViaPointSequence::t1() const
{
  return MatNd_get2(this->viaDescr, this->viaDescr->m-1, 0);
}

/*******************************************************************************
 *
 ******************************************************************************/
double ViaPointSequence::duration() const
{
  return t1() - t0();
}

/*******************************************************************************
 * TODO: Consider the via points along with their Lagrange Multipliers
 ******************************************************************************/
double ViaPointSequence::getMaxVelocity(double& t_vmax) const
{
  RFATAL("NIY");
  return 0.0;
  // RCHECK(this->viaDescr->m==2);

  // // Acceleration polynomial: The roots of this polynomials correspond to the
  // // extramals of the velocity.
  // const unsigned int degree = 3;
  // double coeff[degree+1], roots[degree+2];// 2 more elements for t0 and t1
  // coeff[3] = 20.0*MatNd_get(p, 0, 0);
  // coeff[2] = 12.0*MatNd_get(p, 1, 0);
  // coeff[1] =  6.0*MatNd_get(p, 2, 0);
  // coeff[0] =  2.0*MatNd_get(p, 3, 0);

  // // Find roots
  // int nRoots = Math_findPolyRoots(roots, coeff, degree);

  // // Add initial and final time in case polynomial is strictly monotonous
  // // within [t0 ... t1]
  // roots[nRoots] = t0();
  // roots[nRoots+1] = t1();

  // double vmax = 0.0;
  // t_vmax = t0();   // Otherwise uninitialized if velocity is constantly 0

  // // Go through all roots and the initial and final time point, and find the
  // // maximum absolute value of the velocity.
  // for (int i=0; i<nRoots+2; i++)
  // {
  //   // Only consider roots in the time interval of the polynomial
  //   if ((roots[i]>=t0()) && (roots[i]<=t1()))
  //   {
  //     double v = computeTrajectoryVel(roots[i]);
  //     if (fabs(v) > vmax)
  //     {
  //       vmax = fabs(v);
  //       t_vmax = roots[i];
  //     }
  //   }
  // }

  // return vmax;
}

/*******************************************************************************
 *
 ******************************************************************************/
size_t ViaPointSequence::computeNumberOfConstraints(const MatNd* descr)
{
  size_t numConstraints = 0;

  for (size_t i=0; i<descr->m; i++)
  {
    unsigned int flag = lround(MatNd_get2(descr, i, 4));

    if (Math_isBitSet(flag, VIA_POS))
    {
      numConstraints++;
    }

    if (Math_isBitSet(flag, VIA_VEL))
    {
      numConstraints++;
    }

    if (Math_isBitSet(flag, VIA_ACC))
    {
      numConstraints++;
    }

  }

  return numConstraints;
}

/*******************************************************************************
 *
 ******************************************************************************/
double ViaPointSequence::getPolynomialParameter(size_t index) const
{
  return MatNd_get(this->p, index, 0);
}

/*******************************************************************************
 * Constructs a randomized via sequence descriptor and tests it
 ******************************************************************************/
bool ViaPointSequence::test()
{
  int nVia = Math_getRandomInteger(2, 10);

  MatNd* viaDesc = MatNd_create(nVia, 5);
  MatNd_set(viaDesc, 0, 0, Math_getRandomNumber(-10.0, 10.0));

  for (int i=0; i<nVia; i++)
  {

    if (i>0)
    {
      double t_prev = MatNd_get(viaDesc, i-1, 0);
      double ti = t_prev + Math_getRandomNumber(0.2, 2.0);
      MatNd_set(viaDesc, i, 0, ti);
    }

    MatNd_set(viaDesc, i, 1, Math_getRandomNumber(-5.0, 5.0));   // x
    MatNd_set(viaDesc, i, 2, Math_getRandomNumber(-5.0, 5.0));   // x_dot
    MatNd_set(viaDesc, i, 3, Math_getRandomNumber(-5.0, 5.0));   // x_ddot

    if ((i==0) || (i==nVia-1))
    {
      MatNd_set(viaDesc, i, 4, 7);
    }
    else
    {
      MatNd_set(viaDesc, i, 4, Math_getRandomInteger(1, 7));
    }

  }

  ViaPointSequence v(viaDesc);

  REXEC(1)
  {
    v.gnuplot();
  }

  bool success = v.check();

  if (success == false)
  {
    int tmp = RcsLogLevel;
    RcsLogLevel = 5;
    v.check();
    v.gnuplot();
    v.print();
    RcsLogLevel = tmp;
    RPAUSE();
  }

  return success;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool ViaPointSequence::gradientDxDvia(MatNd* dxdvia, unsigned int row,
                                      double t0, double dt,
                                      unsigned int nSteps) const
{
  MatNd_reshapeAndSetZero(dxdvia, nSteps, 1);

  unsigned int flag = lround(MatNd_get2(viaDescr, row, 4));

  if (!Math_isBitSet(flag, VIA_POS))
  {
    RLOG(4, "No position constraint on row %d", row);
    return false;
  }

  const double eps = 1.0e-3;
  ViaPointSequence via(*this);
  MatNd_addToEle(via.viaDescr, row, 1, eps);
  via.init(via.viaDescr);

  for (unsigned int i=0; i<nSteps; ++i)
  {
    const double ti = t0 + i*dt;
    const double x0 = computeTrajectoryPos(ti);
    const double x1 = via.computeTrajectoryPos(ti);
    dxdvia->ele[i] = (x1-x0)/eps;
    //RLOG(0, "dxdvia[%f] = %f", ti, dxdvia->ele[i]);
  }

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool ViaPointSequence::gradientDxDvia(MatNd* dxdvia, unsigned int row,
                                      double t0, double t1, double dt) const
{
  unsigned int nSteps = lround((t1-t0)/dt)+1;
  return gradientDxDvia(dxdvia, row, t0, dt, nSteps);
}

/*******************************************************************************
 *
 ******************************************************************************/
int ViaPointSequence::getConstraintIndex(const MatNd* desc,
                                         unsigned int row,
                                         unsigned int pos_vel_or_acc)
{
  if (row > desc->m)
  {
    RLOG(1, "Row %d out of range: only %d rows in descriptor", row, desc->m);
    return -1;
  }

  unsigned int flag, numConstraints = 0;

  for (unsigned int i=0; i<row; i++)
  {
    flag = lround(MatNd_get2(desc, i, 4));

    if (Math_isBitSet(flag, VIA_POS))
    {
      numConstraints++;
    }

    if (Math_isBitSet(flag, VIA_VEL))
    {
      numConstraints++;
    }

    if (Math_isBitSet(flag, VIA_ACC))
    {
      numConstraints++;
    }

  }



  flag = lround(MatNd_get2(desc, row, 4));

  switch (pos_vel_or_acc)
  {
    case VIA_POS:
      if (!Math_isBitSet(flag, VIA_POS))
      {
        RLOG(1, "No position constraint on row %d", row);
        return -1;
      }
      break;

    case VIA_VEL:
      if (!Math_isBitSet(flag, VIA_VEL))
      {
        RLOG(1, "No velocity constraint on row %d", row);
        return -1;
      }
      if (Math_isBitSet(flag, VIA_POS))
      {
        numConstraints++;
      }
      break;

    case VIA_ACC:
      if (!Math_isBitSet(flag, VIA_ACC))
      {
        RLOG(1, "No acceleration constraint on row %d", row);
        return -1;
      }
      if (Math_isBitSet(flag, VIA_POS))
      {
        numConstraints++;
      }
      if (Math_isBitSet(flag, VIA_VEL))
      {
        numConstraints++;
      }
      break;

    default:
      RLOG(1, "Wrong index: %d - must be VIA_POS (0), VIA_VEL (1) or VIA_ACC (2)",
           pos_vel_or_acc);
      return -1;
  }

  return numConstraints;
}

/*******************************************************************************
 * grad = t^T B^-1, dimension is 1 x nVia
 ******************************************************************************/
bool ViaPointSequence::gradientDxDvia_a(MatNd* dxdvia, unsigned int row,
                                        double t0, double t1, double dt) const
{
  // Number of time steps including t0 and t1
  const int nSteps = lround((t1-t0)/dt)+1;

  if (nSteps < 0)
  {
    RLOG(1, "Negative number of steps: %d (t0=%g t1=%g)", nSteps, t0, t1);
    return false;
  }

  MatNd_reshape(dxdvia, nSteps, 1);

  // Get the row number of the B-matrix that corresponds to the constraint,
  // and store the column of the inverse of B corresponding to the constraint.
  const int idx = getConstraintIndex(this->viaDescr, row, VIA_POS);

  if (idx < 0)
  {
    RLOG(1, "No position constraint i row %d", row);
    MatNd_setZero(dxdvia);
    return false;
  }

  MatNd* invB_i = MatNd_create(invB->m, 1);
  MatNd_copyColumn(invB_i, 0, this->invB, idx);

  // Right hand side vector consisting of time polynomial entries
  MatNd* rhs = MatNd_create(viaTime.size(), 1);

  for (int i=0; i<nSteps; ++i)
  {
    computeRHS(rhs, t0 + i*dt);
    dxdvia->ele[i] = VecNd_innerProduct(rhs->ele, invB_i->ele, invB->m);
  }

  MatNd_destroy(rhs);
  MatNd_destroy(invB_i);

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
void ViaPointSequence::computeRHS(MatNd* rhs, double t) const
{
  MatNd_reshape(rhs, this->constraintType.size(), 1);

  double t2 = t*t;
  double t3 = t2*t;
  double t4 = t2*t2;
  double t5 = t3*t2;

  MatNd_set2(rhs, 0, 0, t5);
  MatNd_set2(rhs, 1, 0, t4);
  MatNd_set2(rhs, 2, 0, t3);

  MatNd_set2(rhs, 3, 0, t2);
  MatNd_set2(rhs, 4, 0, t);
  MatNd_set2(rhs, 5, 0, 1.0);

  for (size_t j=3; j<this->constraintType.size()-3; j++)
  {
    const double dt = t-this->viaTime[j];

    if (dt>0.0)
    {
      const double dt2 = dt*dt;

      if (this->constraintType[j]==VIA_POS)
      {
        MatNd_set2(rhs, j+3, 0, dt2*dt2*dt);
      }
      else if (this->constraintType[j]==VIA_VEL)
      {
        MatNd_set2(rhs, j+3, 0, dt2*dt2);
      }
      else if (this->constraintType[j]==VIA_ACC)
      {
        MatNd_set2(rhs, j+3, 0, dt2*dt);
      }

    }

  }

}

/*******************************************************************************
 *
 ******************************************************************************/
void ViaPointSequence::setTurboMode(bool enable)
{
  this->computeAllParams = !enable;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool ViaPointSequence::getTurboMode() const
{
  return !this->computeAllParams;
}









/*******************************************************************************
 * Gnuplot class for ViaPointSequences
 ******************************************************************************/
ViaPointSequencePlotter::ViaPointSequencePlotter(): pipe(NULL), fixAxes(false),
  lowerLimitX(0.0),
  upperLimitX(0.0)
{
#if defined(_MSC_VER)
  this->pipe = _popen("pgnuplot.exe -persist", "w");
#else
  this->pipe = popen("gnuplot -persist", "w");
#endif

  Vec3d_setZero(lowerLimitY);
  Vec3d_setZero(upperLimitY);

  if (this->pipe == NULL)
  {
    RLOGS(1, "Couldn't open pipe to gnuplot");
    throw (std::string("Couldn't open pipe to gnuplot"));
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
ViaPointSequencePlotter::~ViaPointSequencePlotter()
{
  pclose(this->pipe);
}

/*******************************************************************************
 *
 ******************************************************************************/
void ViaPointSequencePlotter::enableFixedAxes(const ViaPointSequence& via,
                                              bool enable, double margin)
{
  this->fixAxes = enable;

  if (this->fixAxes == false)
  {
    return;
  }

  const double dt = 0.01;
  MatNd* traj = MatNd_create(4, 1+lround(via.duration()/dt));
  via.computeTrajectory(traj, via.t0(), via.t1(), dt);
  lowerLimitY[0] = MatNd_get(traj, 1, 0);
  lowerLimitY[1] = MatNd_get(traj, 2, 0);
  lowerLimitY[2] = MatNd_get(traj, 3, 0);
  Vec3d_copy(upperLimitY, lowerLimitY);

  for (unsigned int i=0; i<traj->n; ++i)
  {
    double pos_i = MatNd_get(traj, 1, i);
    double vel_i = MatNd_get(traj, 2, i);
    double acc_i = MatNd_get(traj, 3, i);

    // Update position limits
    if (pos_i < lowerLimitY[0])
    {
      lowerLimitY[0] = pos_i;
    }
    if (pos_i > upperLimitY[0])
    {
      upperLimitY[0] = pos_i;
    }

    // Update velocity limits
    if (vel_i < lowerLimitY[1])
    {
      lowerLimitY[1] = vel_i;
    }
    if (vel_i > upperLimitY[1])
    {
      upperLimitY[1] = vel_i;
    }

    // Update acceleration limits
    if (acc_i < lowerLimitY[2])
    {
      lowerLimitY[2] = acc_i;
    }
    if (acc_i > upperLimitY[2])
    {
      upperLimitY[2] = acc_i;
    }
  }

  for (int i=0; i<3; ++i)
  {
    double range = upperLimitY[i] - lowerLimitY[i];
    upperLimitY[i] += margin*range;
    lowerLimitY[i] -= margin*range;
  }

  this->lowerLimitX = via.t0() - margin*via.duration();
  this->upperLimitX = via.t1() + margin*via.duration();
}

/*******************************************************************************
 *
 ******************************************************************************/
void ViaPointSequencePlotter::setRangeX(double lowerLimit, double upperLimit)
{
  this->lowerLimitX = lowerLimit;
  this->upperLimitX = upperLimit;
  this->fixAxes = true;
}

/*******************************************************************************
 *
 ******************************************************************************/
void ViaPointSequencePlotter::setRangeY(double lowerLimit, double upperLimit)
{
  this->lowerLimitY[0] = lowerLimit;
  this->upperLimitY[0] = upperLimit;
  this->fixAxes = true;
}

/*******************************************************************************
 *
 ******************************************************************************/
void ViaPointSequencePlotter::plot(const ViaPointSequence& via,
                                   double t0, double t1, double dt, int flag)
{
  if (t1 <= t0)
  {
    RLOG(1, "t1 <= t0: t1=%f t0=%f", t1, t0);
    return;
  }


  // Calculate trajectory and write it to file in gnuplot-compatible conventions
  char trajFile[64] = "traj.dat";
  MatNd* traj = MatNd_create(3, 4*lround((t1-t0)/dt));
  via.computeTrajectory(traj, t0, t1, dt);
  MatNd_transposeSelf(traj);
  MatNd_toFile(traj, trajFile);

  size_t nConstr = ViaPointSequence::computeNumberOfConstraints(via.viaDescr);

  MatNd* viaPos = MatNd_create(nConstr, 2);
  MatNd* viaVel = MatNd_create(nConstr, 2);
  MatNd* viaAcc = MatNd_create(nConstr, 2);

  MatNd_reshape(viaPos, 0, 2);
  MatNd_reshape(viaVel, 0, 2);
  MatNd_reshape(viaAcc, 0, 2);


  // Write files for the display of the points
  for (size_t i=0; i<via.viaDescr->m; i++)
  {
    unsigned int flag = lround(MatNd_get(via.viaDescr, i, 4));
    double t = MatNd_get(via.viaDescr, i, 0);

    if (Math_isBitSet(flag, VIA_POS))
    {
      viaPos->m++;
      MatNd_set(viaPos, viaPos->m-1, 0, t);
      MatNd_set(viaPos, viaPos->m-1, 1, MatNd_get(via.viaDescr, i, 1));
    }

    if (Math_isBitSet(flag, VIA_VEL))
    {
      viaVel->m++;
      MatNd_set(viaVel, viaVel->m-1, 0, t);
      MatNd_set(viaVel, viaVel->m-1, 1, MatNd_get(via.viaDescr, i, 2));
    }

    if (Math_isBitSet(flag, VIA_ACC))
    {
      viaAcc->m++;
      MatNd_set(viaAcc, viaAcc->m-1, 0, t);
      MatNd_set(viaAcc, viaAcc->m-1, 1, MatNd_get(via.viaDescr, i, 3));
    }

  }

  char viaPosFile[64] = "viaPos.dat";
  char viaVelFile[64] = "viaVel.dat";
  char viaAccFile[64] = "viaAcc.dat";

  MatNd_toFile(viaPos, viaPosFile);
  MatNd_toFile(viaVel, viaVelFile);
  MatNd_toFile(viaAcc, viaAccFile);

  MatNd_destroy(viaPos);
  MatNd_destroy(viaVel);
  MatNd_destroy(viaAcc);


  // Write gnuplot command strings to pipe
  if (this->fixAxes == true)
  {
    fprintf(this->pipe, "set xrange [%f:%f]\n", lowerLimitX, upperLimitX);
  }
  else
  {
    fprintf(this->pipe, "set autoscale\n");
  }


  if (flag==7)
  {

    if (this->fixAxes == true)
    {
      double ll = Math_fmin3(lowerLimitY[0], lowerLimitY[1], lowerLimitY[2]);
      double ul = Math_fmax3(upperLimitY[0], upperLimitY[1], upperLimitY[2]);
      fprintf(this->pipe, "set yrange [%f:%f]\n", ll, ul);
    }

    fprintf(this->pipe,
            "set grid\nplot \"%s\" u 1:2 w l title \"x\", \"%s\" u 1:3 w l"
            " title \"x_dot\", \"%s\" u 1:4 w l title \"x_ddot\", \"%s\" "
            "u 1:2 w p pointsize 3 title \"x_via\", \"%s\" u 1:2 w p pointsize"
            " 3 title \"x_dot_via\", \"%s\" u 1:2 w p pointsize 3 title"
            " \"x_ddot_via\"\n",
            trajFile, trajFile, trajFile, viaPosFile, viaVelFile, viaAccFile);
  }
  else
  {
    if (Math_isBitSet(flag, VIA_POS))
    {
      if (this->fixAxes == true)
      {
        fprintf(this->pipe, "set yrange [%f:%f]\n",
                lowerLimitY[0], upperLimitY[0]);
      }

      fprintf(this->pipe,
              "set grid\nplot \"%s\" u 1:2 w l title \"x\", \"%s\" "
              "u 1:2 w p pointsize 3 title \"x_via\"\n", trajFile, viaPosFile);
    }

    if (Math_isBitSet(flag, VIA_VEL))
    {
      if (this->fixAxes == true)
      {
        fprintf(this->pipe, "set yrange [%f:%f]\n",
                lowerLimitY[1], upperLimitY[1]);
      }

      fprintf(this->pipe,
              "set grid\nplot \"%s\" u 1:3 w l title \"x_dot\", "
              "\"%s\" u 1:2  title \"x_dot_via\" w p pointsize 3\n",
              trajFile, viaVelFile);
    }

    if (Math_isBitSet(flag, VIA_ACC))
    {
      if (this->fixAxes == true)
      {
        fprintf(this->pipe, "set yrange [%f:%f]\n",
                lowerLimitY[2], upperLimitY[2]);
      }

      fprintf(this->pipe,
              "set grid\nplot \"%s\" u 1:4 w l title \"x_ddot\", "
              "\"%s\" u 1:2  title \"x_ddot_via\" w p pointsize 3\n",
              trajFile, viaAccFile);
    }
  }

  fprintf(this->pipe, "\n");
  fflush(this->pipe);
}

/*******************************************************************************
 *
 ******************************************************************************/
void ViaPointSequencePlotter::plot2(const ViaPointSequence& via,
                                    double t0, double t1, double dt, int flag)
{
#if defined (_MSC_VER)
  return plot(via, t0, t1, dt, flag);
#endif

  if (t1 <= t0)
  {
    RLOG(1, "t1 <= t0: t1=%f t0=%f", t1, t0);
    return;
  }


  // Calculate trajectory and write it to file in gnuplot-compatible conventions
  char trajFile[64] = "traj.dat";
  MatNd* traj = MatNd_create(3, 4*lround((t1-t0)/dt));
  via.computeTrajectory(traj, t0, t1, dt);
  MatNd_transposeSelf(traj);
  MatNd_toFile(traj, trajFile);

  size_t nConstr = ViaPointSequence::computeNumberOfConstraints(via.viaDescr);

  MatNd* viaPos = MatNd_create(nConstr, 2);
  MatNd* viaVel = MatNd_create(nConstr, 2);
  MatNd* viaAcc = MatNd_create(nConstr, 2);

  MatNd_reshape(viaPos, 0, 2);
  MatNd_reshape(viaVel, 0, 2);
  MatNd_reshape(viaAcc, 0, 2);


  // Write files for the display of the points
  for (size_t i=0; i<via.viaDescr->m; i++)
  {
    unsigned int flag = lround(MatNd_get(via.viaDescr, i, 4));
    double t = MatNd_get(via.viaDescr, i, 0);

    if (Math_isBitSet(flag, VIA_POS))
    {
      viaPos->m++;
      MatNd_set(viaPos, viaPos->m-1, 0, t);
      MatNd_set(viaPos, viaPos->m-1, 1, MatNd_get(via.viaDescr, i, 1));
    }

    if (Math_isBitSet(flag, VIA_VEL))
    {
      viaVel->m++;
      MatNd_set(viaVel, viaVel->m-1, 0, t);
      MatNd_set(viaVel, viaVel->m-1, 1, MatNd_get(via.viaDescr, i, 2));
    }

    if (Math_isBitSet(flag, VIA_ACC))
    {
      viaAcc->m++;
      MatNd_set(viaAcc, viaAcc->m-1, 0, t);
      MatNd_set(viaAcc, viaAcc->m-1, 1, MatNd_get(via.viaDescr, i, 3));
    }

  }

  char viaPosFile[64] = "viaPos.dat";
  char viaVelFile[64] = "viaVel.dat";
  char viaAccFile[64] = "viaAcc.dat";

  MatNd_toFile(viaPos, viaPosFile);
  MatNd_toFile(viaVel, viaVelFile);
  MatNd_toFile(viaAcc, viaAccFile);


  // Write gnuplot command strings to pipe
  if (this->fixAxes == true)
  {
    fprintf(this->pipe, "set xrange [%f:%f]\n", lowerLimitX, upperLimitX);
  }
  else
  {
    //fprintf(this->pipe, "set autoscale\n");
  }

  fprintf(this->pipe, "set grid\n");


  if (flag==7)
  {

    if (this->fixAxes == true)
    {
      double ll = Math_fmin3(lowerLimitY[0], lowerLimitY[1], lowerLimitY[2]);
      double ul = Math_fmax3(upperLimitY[0], upperLimitY[1], upperLimitY[2]);
      fprintf(this->pipe, "set yrange [%f:%f]\n", ll, ul);
    }

    fprintf(this->pipe,
            "plot \"%s\" u 1:2 w l title \"x\", \"%s\" u 1:3 w l"
            " title \"x_dot\", \"%s\" u 1:4 w l title \"x_ddot\", \"%s\" "
            "u 1:2 w p pointsize 3 title \"x_via\", \"%s\" u 1:2 w p pointsize"
            " 3 title \"x_dot_via\", \"%s\" u 1:2 w p pointsize 3 title"
            " \"x_ddot_via\"\n",
            trajFile, trajFile, trajFile, viaPosFile, viaVelFile, viaAccFile);
  }
  else
  {
    // ==============================================================
    // Position trajectory only
    // ==============================================================
    if (Math_isBitSet(flag, VIA_POS))
    {
      if ((this->fixAxes==true) && (upperLimitX-lowerLimitX>0.0) // &&
          // (upperLimitY[0]>lowerLimitY[0])
         )
      {
        fprintf(this->pipe, "set xrange [%f:%f]\n", lowerLimitX, upperLimitX);
        fprintf(this->pipe, "set yrange [%f:%f]\n",
                lowerLimitY[0], upperLimitY[0]);
      }
      else
      {
        //fprintf(this->pipe, "set autoscale\n");
      }

      fprintf(this->pipe, "plot '-' w l title \"x\", "
              "'-' w p pointsize 3 title \"x_via\"\n");

      for (unsigned int i=0; i<traj->m; ++i)
      {
        fprintf(pipe,"%f, %f\n", MatNd_get(traj, i, 0), MatNd_get(traj, i, 1));
      }
      fprintf(this->pipe,"e\n");

      for (unsigned int i=0; i<traj->m; ++i)
      {
        unsigned int rowIdx = Math_iClip(i, 0, viaPos->m-1);
        fprintf(this->pipe,"%f, %f\n",
                MatNd_get(viaPos, rowIdx, 0), MatNd_get(viaPos, rowIdx, 1));
      }
      fprintf(this->pipe,"e\n");
      fprintf(this->pipe, "\n");
    }

    // ==============================================================
    // Velocity trajectory only
    // ==============================================================
    if (Math_isBitSet(flag, VIA_VEL))
    {
      if (this->fixAxes == true)
      {
        fprintf(this->pipe, "set yrange [%f:%f]\n",
                lowerLimitY[1], upperLimitY[1]);
      }

      fprintf(this->pipe,
              "plot \"%s\" u 1:3 w l title \"x_dot\", "
              "\"%s\" u 1:2  title \"x_dot_via\" w p pointsize 3\n",
              trajFile, viaVelFile);
    }

    if (Math_isBitSet(flag, VIA_ACC))
    {
      if (this->fixAxes == true)
      {
        fprintf(this->pipe, "set yrange [%f:%f]\n",
                lowerLimitY[2], upperLimitY[2]);
      }

      fprintf(this->pipe,
              "plot \"%s\" u 1:4 w l title \"x_ddot\", "
              "\"%s\" u 1:2  title \"x_ddot_via\" w p pointsize 3\n",
              trajFile, viaAccFile);
    }
  }

  MatNd_destroy(viaPos);
  MatNd_destroy(viaVel);
  MatNd_destroy(viaAcc);

  fprintf(this->pipe, "\n");
  fflush(this->pipe);
}
