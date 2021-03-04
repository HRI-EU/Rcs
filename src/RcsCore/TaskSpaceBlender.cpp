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

#include "TaskSpaceBlender.h"

#include "Task.h"
#include "TaskJoint.h"
#include "Rcs_macros.h"
#include "Rcs_math.h"
#include "Rcs_eigen.h"
#include "Rcs_typedef.h"

#include <cmath>
#include <algorithm>


#define PER_TASK // first per task, than per Jacobian entry of tasks

namespace Rcs
{

/*******************************************************************************
 *
 ******************************************************************************/
TaskSpaceBlender::TaskSpaceBlender(const ControllerBase* controller_) :
  controller(controller_), mode(BlendingMode::Binary), maxIterations(10),
  incrementalWeightUpdate(false)
{
}

/*******************************************************************************
 *
 ******************************************************************************/
TaskSpaceBlender::~TaskSpaceBlender()
{
}

/*******************************************************************************
 *
 ******************************************************************************/
void TaskSpaceBlender::setBlendingMode(BlendingMode newMode)
{
  this->mode = newMode;
}

/*******************************************************************************
 *
 ******************************************************************************/
TaskSpaceBlender::BlendingMode TaskSpaceBlender::getBlendingMode() const
{
  return this->mode;
}

/*******************************************************************************
 *
 ******************************************************************************/
void TaskSpaceBlender::setMaxIterations(unsigned int iterations)
{
  this->maxIterations = iterations;
}

/*******************************************************************************
 *
 ******************************************************************************/
unsigned int TaskSpaceBlender::getMaxIterations() const
{
  return this->maxIterations;
}

/*******************************************************************************
 *
 ******************************************************************************/
void TaskSpaceBlender::setIncrementalWeightUpdate(bool enable)
{
  this->incrementalWeightUpdate = enable;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool TaskSpaceBlender::getIncrementalWeightUpdate() const
{
  return this->incrementalWeightUpdate;
}

/*******************************************************************************
 *
 ******************************************************************************/
void TaskSpaceBlender::compute(MatNd* Wx,
                               MatNd* C_curr,
                               const MatNd* a_des,
                               const MatNd* J,
                               const MatNd* invWq) const
{
  switch (this->mode)
  {
    case Binary:
      computeBinary(Wx, C_curr, a_des);
      break;

    case Linear:
      computeLinear(Wx, C_curr, a_des);
      break;

    case Approximate:
      computeApproximate(Wx, C_curr, a_des, J, invWq, true);
      break;

    case IndependentTasks:
      computeIterative(Wx, C_curr, a_des, J, invWq, false);
      break;

    case DependentTasks:
      computeIterative(Wx, C_curr, a_des, J, invWq, true);
      break;

    default:
      RFATAL("No blending mode %d", this->mode);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void TaskSpaceBlender::compute(MatNd* Wx,
                               MatNd* C_curr,
                               const MatNd* a_des,
                               const MatNd* J,
                               double lambda) const
{
  MatNd invWq = MatNd_fromPtr(1, 1, &lambda);
  compute(Wx, C_curr, a_des, J, &invWq);
}

/*******************************************************************************
 *
 ******************************************************************************/
void TaskSpaceBlender::computeIterative(MatNd* Wx,
                                        MatNd* C_curr,
                                        const MatNd* a_des,
                                        const MatNd* J,
                                        const MatNd* invWq,
                                        bool rankActivation) const
{
  const unsigned int nx = J->m;
  const unsigned int nTasks = a_des->m;

  MatNd* Wq = NULL;
  MatNd_create2(Wq, invWq->m, invWq->n);
  MatNd* invWx = NULL;
  MatNd_create2(invWx, nx, 1);
  MatNd* invWxMat = NULL;
  MatNd_create2(invWxMat, nx, nx);
  MatNd* invWxPrev = NULL;
  MatNd_create2(invWxPrev, nx, 1);
  MatNd* Cdes = NULL;
  MatNd_create2(Cdes, nx, 1);
  MatNd* invCdes = NULL;
  MatNd_create2(invCdes, nx, 1);
  MatNd* minimumWx = NULL;
  MatNd_create2(minimumWx, nx, 1);
  MatNd* L = NULL;
  MatNd_create2(L, nx, nx);
  MatNd* C = NULL;
  MatNd_create2(C, nx, nx);
  MatNd* offDiagLogical = NULL;
  MatNd_create2(offDiagLogical, nx, nx);
  MatNd* rhs = NULL;
  MatNd_create2(rhs, nx, nx);
  MatNd* invWplusL = NULL;
  MatNd_create2(invWplusL, nx, nx);
  MatNd* temp = NULL;
  MatNd_create2(temp, nx, nx);
  MatNd* wJpinv = NULL;
  MatNd_create2(wJpinv, J->n, nx);

  Cdes->m = 0;

  MatNd_inverseDiag(Wq, invWq);

  // expand a_des for the components of each task, skip tasks with a_des = 0
  for (unsigned int i = 0; i < nTasks; i++)
  {
    if (MatNd_get(a_des, i, 0) > 0.0)
    {
      const unsigned int dimTask = controller->getTaskDim(i);
      VecNd_setElementsTo(&Cdes->ele[Cdes->m], a_des->ele[i], dimTask);
      Cdes->m += dimTask;
    }
  }

  // Rank activation
  if (rankActivation == true)
  {
    MatNd_reshape(temp, Cdes->m, 1);

    double error = getActivationScaling(temp, J, a_des, 1.0e-5);

    ///////////////////////////////////////////////////
    // blending between scalings
    MatNd* scale2 = NULL;
    MatNd_create2(scale2, nx, 1);
    MatNd_reshape(scale2, temp->m, temp->n);

    getActivationScaling(scale2, J, a_des, 1e-6);

    MatNd_constMulSelf(temp, 1.0-error);
    MatNd_constMulAndAddSelf(temp, scale2, error);

    // Low pass filter
    //  decompressFromActive(scale2, temp, a_des);
    //  double filterRatio = 1.e-1;
    //  MatNd_constMulSelf(scale2, filterRatio);
    //  MatNd_constMulAndAddSelf(scale2, this->activationScaling,
    //  1.-filterRatio);
    //  decompressFromActive(this->activationScaling, scale2, a_des);
    //  compressToActive(temp, scale2, a_des);

    MatNd_destroy(scale2);
    ///////////////////////////////////////////////////

    MatNd_printComment("scaling", temp);
    MatNd_eleMulSelf(Cdes, temp);
  }

  MatNd_inverseDiag(invCdes, Cdes);

  // reshape Wx and invWx to match Cdes, initialize Wx and invWx with Cdes
  // re-using the old Wx will not be beneficial as they can change drastically
  if (incrementalWeightUpdate==false)
  {
    MatNd_reshape(Wx, Cdes->m, 1);
    MatNd_copy(Wx, Cdes);
  }
  else
  {
    if (Wx->m != Cdes->m)
    {
      MatNd_reshape(Wx, Cdes->m, 1);
      MatNd_copy(Wx, Cdes);
    }
  }

  MatNd_reshape(invWx, Wx->m, 1);
  MatNd_inverseDiag(invWx, Wx);

  // Pre-calculate L = J*Wq
  MatNd_sqrMulABAt(L, J, Wq);

  // Regularization that avoids numerical instabilities if a_des not possible
  // set invCdes as minimum activation
  MatNd_copy(minimumWx, invCdes);
  // multiply self by minimal element
  MatNd_constMulSelf(minimumWx, MatNd_minEle(minimumWx));

  // calculate C = J*wJpinv with current Wx
  MatNd_rwPinv2(wJpinv, J, Wx, invWq);
  MatNd_mul(C, J, wJpinv);

  // put desired values on diagonal of C
  MatNd_overwriteDiag(C, Cdes);

  if (Cdes->m > 0)
  {
    MatNd* A = NULL;
    MatNd_create2(A, nx, nx);
    MatNd_reshape(A, nx - 1, nx - 1);
    MatNd* CoffDiag = NULL;
    MatNd_create2(CoffDiag, nx, 1);

    // indicator matrix for offDiagonal elements
    MatNd_setElementsTo(offDiagLogical, 1.);
    MatNd_addConstToDiag(offDiagLogical, -1.);

    // fixed-point stlye algorithm
    for (unsigned int iter = 0; iter < this->maxIterations; iter++)
    {
      MatNd_copy(invWxPrev, invWx);
      // Cdiag = invCdes*(I-C)*L;
      MatNd_constMul(invWxMat, C, -1.);
      MatNd_addConstToDiag(invWxMat, 1.);
      MatNd_preMulDiagSelf(invWxMat, invCdes);
      MatNd_postMulSelf(invWxMat, L);

      // apply regularization
      MatNd_getDiag(invWx, invWxMat);
      MatNd_maxSelf(invWx, minimumWx);
      MatNd_applyFctEle(invWx, &fabs);

      // simple low-pass filter as this method tends to oscillate
      MatNd_addSelf(invWx, invWxPrev);
      MatNd_constMulSelf(invWx, 0.5);

      // rhs = (I-Cdes)*L - Cdes*invWx
      MatNd_reshape(rhs, nx, nx);
      MatNd_reshape(temp,nx, 1);
      MatNd_setIdentity(rhs);
      MatNd_copy(temp, Cdes);
      MatNd_constMulSelf(temp, -1.0);
      MatNd_addDiag(rhs, temp);
      MatNd_postMulSelf(rhs, L);
      //- Cdes*invWx only affects the diagonal, which we ignore anyhow
      //      MatNd_eleMul(temp, Cdes, invWx);
      //      MatNd_constMulSelf(temp, -1.);
      //      MatNd_addDiag(rhs, temp);

      // lhs = CnonDiag*(invWx+L)
      MatNd_copy(invWplusL, L);
      MatNd_addDiag(invWplusL, invWx);

      // we want to ignore the diagonal elements of CnonDiag
      // this can be written in the form b = A*x where each row corresponds
      // to one off-diagonal element of CnonDiag
      // there we have a block structure on the diagonal, which corresponds
      // to the rows of CnonDiag
      // see 2015blending/blending.pdf
      for (unsigned int dimRow = 0; dimRow < C->m; dimRow++)
      {
        MatNd offDiagLogicalRow =
          MatNd_fromPtr(1, offDiagLogical->n,
                        MatNd_getRowPtr(offDiagLogical, dimRow));

        MatNd_reshapeCopy(A, invWplusL);
        MatNd_deleteRow(A, dimRow);
        MatNd_deleteColumn(A, dimRow);

        MatNd rhsRow = MatNd_fromPtr(1, rhs->n,
                                     MatNd_getRowPtr(rhs, dimRow));
        MatNd_getSubset(temp, &rhsRow, &offDiagLogicalRow);
        MatNd_reshape(CoffDiag, nx-1, 1);
        MatNd Crow = MatNd_fromPtr(1, C->n, MatNd_getRowPtr(C, dimRow));

#if 1
        //        MatNd_svdSolve(CoffDiag, A, temp);
        MatNd_choleskySolve(CoffDiag, A, temp); // seems to be the fastest
#else
        // GaussSeidel allows us to control the trade-off between
        // precision and time
        MatNd_getSubset(CoffDiag, &Crow, &offDiagLogicalRow);
        // MatNd_gaussSeidelIterate(CoffDiag, A, temp, 1.0e-8, -1);
        MatNd_gaussSeidelIterate(CoffDiag, A, temp, 1.0e-6, 10);

#endif
        MatNd_setSubset(&Crow, CoffDiag, &offDiagLogicalRow);
      } //dimRow

      MatNd_overwriteDiag(C, Cdes);
    } //iter

    MatNd_destroy(CoffDiag);
    MatNd_destroy(A);
  }

  MatNd_copy(invWxPrev, invWx);

  // Cdiag = invCdes*(I-C)*L;
  MatNd_constMul(invWxMat, C, -1.0);
  MatNd_addConstToDiag(invWxMat, 1.0);
  MatNd_preMulDiagSelf(invWxMat, invCdes);
  MatNd_postMulSelf(invWxMat, L);

  MatNd_getDiag(invWx, invWxMat);
  MatNd_maxSelf(invWx, minimumWx);
  MatNd_fabsEleSelf(invWx);
  MatNd_inverseDiag(Wx, invWx);

  if (C_curr != NULL)
  {
    computeActivation(C_curr, Wx, a_des, J, invWq);
  }

  MatNd_destroy(Wq);
  MatNd_destroy(invWx);
  MatNd_destroy(invWxMat);
  MatNd_destroy(invWxPrev);
  MatNd_destroy(Cdes);
  MatNd_destroy(invCdes);
  MatNd_destroy(minimumWx);
  MatNd_destroy(L);
  MatNd_destroy(C);
  MatNd_destroy(offDiagLogical);
  MatNd_destroy(rhs);
  MatNd_destroy(invWplusL);
  MatNd_destroy(temp);
  MatNd_destroy(wJpinv);
}

/*******************************************************************************
 *
 ******************************************************************************/
void TaskSpaceBlender::computeActivation(MatNd* ax_curr,
                                         const MatNd* Wx,
                                         const MatNd* a_des,
                                         const MatNd* J,
                                         const MatNd* invWq) const
{
  MatNd* wJpinv = NULL;
  MatNd_create2(wJpinv, J->n, J->m);
  MatNd* C = NULL;
  MatNd_create2(C, J->m, J->m);

  MatNd_rwPinv2(wJpinv, J, Wx, invWq);
  MatNd_mul(C, J, wJpinv);
  MatNd_reshape(ax_curr, J->m, 1);
  MatNd_getDiag(ax_curr, C);
  controller->decompressFromActiveSelf(ax_curr, a_des);

  MatNd_destroy(wJpinv);
  MatNd_destroy(C);
}

//sets a global scale per task rather than scales for individual components
//#define TASK_SCALE

//first per task, than per Jacobian entry of tasks,
//needed for the above but also seems to work stand alone
//less combinations to check but larger matrix junks for rank calculations, not sure what is fast
//#define PER_TASK

/*******************************************************************************
 * nextComb(int comb[], int k, int n)
 *   Generates the next combination of n elements as k after comb
 *
 * comb => the previous combination ( use (0, 1, 2, ..., k) for first)
 * n => the size of the original set
 * k => the size of the subsets to generate
 *
 *
 * Returns: 1 if a valid combination was found
 *   0, otherwise
 *
 * http://compprog.wordpress.com/2007/10/17/generating-combinations-1/
 ******************************************************************************/
static bool nextComb(double comb[], int n, int k)
{
  int i = k - 1;
  ++comb[i];
  while ((i >= 0) && (comb[i] >= n - k + 1 + i))
  {
    --i;
    ++comb[i];
  }

  if (comb[0] > n - k) /* Combination (n-k, n-k+1, ..., n) reached */
  {
    return false; /* No more combinations can be generated */
  }

  /* comb now looks like (..., x, n, n, n, ..., n).
     Turn it into (..., x, x + 1, x + 2, ...) */
  for (i = i + 1; i < k; ++i)
  {
    comb[i] = comb[i - 1] + 1;
  }

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
static void remainderOfSet(MatNd* remainder, const MatNd* set, const MatNd* subset)
{
  MatNd_reshape(remainder, 1, 0);

  for (unsigned int setEntry = 0; setEntry < set->n; setEntry++)
  {
    bool inSubset = false;
    int setMember = (int) set->ele[setEntry];

    for (unsigned int subsetEntry = 0; subsetEntry < subset->n; subsetEntry++)
    {
      if ((int) subset->ele[subsetEntry] == setMember)
      {
        inSubset = true;
        break;
      }
    }

    if (inSubset == false)
    {
      remainder->ele[remainder->n] = setMember;
      remainder->n++;
    }
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
static unsigned int removeRowsContainingMember(MatNd* A, const MatNd* member)
{
  for (int rowA = A->m - 1; rowA >= 0; rowA--)
  {
    bool containsMember = false;

    for (unsigned int colA = 0; colA < A->n; colA++)
    {
      for (unsigned int colMember = 0; colMember < A->n; colMember++)
      {
        if ((int) member->ele[colMember] == (int) MatNd_get(A, rowA, colA))
        {
          containsMember = true;
          break;
        }
      }
      if (containsMember == true)
      {
        break;
      }
    }

    if (containsMember == true)
    {
      MatNd_deleteRow(A, rowA);
    }
  }
  return A->m;
}

/*******************************************************************************
 *
 ******************************************************************************/
static double subJacobian(MatNd* subJ,
                          const MatNd* J,
                          const MatNd* subJindex,
                          const MatNd* a)
{
  double aSum = 0;

  if (subJ != NULL)
  {
    MatNd_reshape(subJ, subJindex->n, J->n);
  }
  for (unsigned int colJindex = 0; colJindex < subJindex->n; colJindex++)
  {
    unsigned int rowJ = (unsigned int) MatNd_get(subJindex, 0, colJindex);
    if (subJ != NULL)
    {
      MatNd_setRow(subJ, colJindex, MatNd_getRowPtr(J, rowJ), J->n);
    }
    if (a != NULL)
    {
      aSum += MatNd_get(a, rowJ, 0);
    }
  }
  return aSum;
}

/*******************************************************************************
 *
 ******************************************************************************/
static void enumerateSubsets(MatNd* subsets,
                             const unsigned int n,
                             const unsigned int k,
                             const MatNd* mapping)
{
  // first subset 0, 1, 2, ..., k-1
  for (unsigned int col = 0; col < k; col++)
  {
    MatNd_set(subsets, 0, col, (double) col);
  }

  // remaining subsets
  for (unsigned int row = 1; row < subsets->m; row++)
  {
    // copy previous row
    MatNd_setRow(subsets, row, MatNd_getRowPtr(subsets, row - 1), k);
    // generate next combination based on this
    if (nextComb(MatNd_getRowPtr(subsets, row), n, k) == false)
    {
      break;
    }
  }

  if (mapping != NULL)
  {
    // now we replace those by the actual row numbers
    for (unsigned int elem = 0; elem < (subsets->m * k); elem++)
    {
      subsets->ele[elem] = mapping->ele[(int) subsets->ele[elem]];
    }
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
#ifdef TASK_SCALE
static void updateMinScaleFactor(double& scaleFactor, int rank, double aSum)
{
  double newScaleFactor = 1.;
  if (aSum > (double) rank)
  {
    newScaleFactor = (double) rank / aSum;
  }
  if (scaleFactor > newScaleFactor)
  {
    scaleFactor = newScaleFactor;
  }
}
#endif

/*******************************************************************************
 *
 ******************************************************************************/
static void setScaleFactors(MatNd* scaleFactors,
                            int rank,
                            double aSum,
                            const MatNd* subJindex)
{
  double scaleFactor = 1.;
  if (aSum > (double) rank)
  {
    scaleFactor = (double) rank / aSum;
  }

  for (unsigned int colJindex = 0; colJindex < subJindex->n; colJindex++)
  {
    unsigned int rowJ = (unsigned int) MatNd_get(subJindex, 0, colJindex);
    scaleFactors->ele[rowJ] = scaleFactor;
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
static int MyRank(const MatNd* J, double eps, double& relaxedRank, int& diffCount)
{
  int m = J->m, n = J->n, rank = 0;
  int dim = m > n ? m : n;

  double eps2 = 0.1*eps;

  relaxedRank = 0.0;
  diffCount = 0;

  MatNd* S= NULL;
  MatNd_create2(S, n, 1);

  MatNd_SVD(NULL, S, NULL, J, eps);

  // Compute the rank of the system
  for (int i = 0; i < dim; i++)
  {
    if (S->ele[i] > eps)
    {
      rank++;
    }

    double temp = (S->ele[i] - eps2)/(eps - eps2);
    relaxedRank += Math_clip(temp, 0.0, 1.0);

    if ((temp>0.0) && (temp<1.0))
    {
      diffCount++;
    }

  }

  MatNd_destroy(S);

  return rank;
}

/*******************************************************************************
 *
 ******************************************************************************/
static double getScaleFactors(MatNd* scaleFactors,
                              const MatNd* J,
                              const MatNd* a_des,
                              const int rankJ,
                              const double tol)
{
  double error = 0.0;

  MatNd_reshape(scaleFactors, J->m, 1);

#ifdef TASK_SCALE
  double minScaleFactor = 1.0;
#endif
  //  MatNd_printComment("Jsubset",J);

  MatNd* candJ = NULL;
  MatNd_create2(candJ, J->m, J->n);
  MatNd* WOcandJ = NULL;
  MatNd_create2(WOcandJ, J->m, J->n);

  MatNd* globIndex = NULL;
  MatNd_create2(globIndex, 1, J->m);

  for (unsigned int index = 0; index < J->m; index++)
  {
    globIndex->ele[index] = double(index);
  }

  // total rank of the Jacobian
  int globRank = rankJ;

  // find the smallest subsets first, checking for the smaller half is
  // sufficient
  for (unsigned int subsetN = 1; subsetN <= J->m / 2; subsetN++)
  {
    // the remaining tasks are considered the final subset, so we can stop
    //  if we have less than 2 full subsets
    if (globIndex->n < 2 * subsetN)
    {
      break;
    }

    // now we enumerate all possible subsets of size subsetN
    MatNd* candidatesIndex = NULL;
    MatNd_create2(candidatesIndex, Math_NchooseK(globIndex->n, subsetN), subsetN);
    enumerateSubsets(candidatesIndex, globIndex->n, subsetN, globIndex);

    // going through the candidates
    for (unsigned int candN = 0; candN < candidatesIndex->m; candN++)
    {
      // creating a vector of the tasks not in the candidate subset
      MatNd* WOcandIndex = NULL;
      MatNd_create2(WOcandIndex, 1, globIndex->n - candidatesIndex->n);
      MatNd candIndex = MatNd_fromPtr(1, candidatesIndex->n, MatNd_getRowPtr(candidatesIndex, candN));
      remainderOfSet(WOcandIndex, globIndex, &candIndex);

      // assembling the Jacobian of the tasks not in the subset
      subJacobian(WOcandJ, J, WOcandIndex, NULL);
      // assembling the Jacobian of the tasks in the subset
      double candAsum = subJacobian(candJ, J, &candIndex, a_des);

      double candRelaxedRank;
      double WOcandRelaxedRank;
      int candDiffCount;
      int WOcandDiffCount;

      // now we check whether the subset is independent of the remaining
      // rows
      int candRank = MyRank(candJ, tol, candRelaxedRank, candDiffCount);
      int WOcandRank = MyRank(WOcandJ, tol, WOcandRelaxedRank, WOcandDiffCount);

      if ((candRank + WOcandRank) == globRank)
      {
        double newQuality = std::max((WOcandRelaxedRank - (double)WOcandRank)/std::max(1., (double)WOcandDiffCount),
                                     (candRelaxedRank - (double)candRank)/std::max(1., (double)candDiffCount));

        error = std::max(error, newQuality);

        // MatNd_printComment("within task subset candIndex", &candIndex);
        // fprintf(stderr, "of rank %d\n", candRank);

#ifdef TASK_SCALE
        updateMinScaleFactor(minScaleFactor, candRank, candAsum);
#else
        setScaleFactors(scaleFactors, candRank, candAsum, &candIndex);
#endif
        // printf("minScaleFactor %f\n\n",minScaleFactor);

        // remove the independent rows of the Jacobian we still need to
        // consider
        MatNd_reshape(globIndex, WOcandIndex->m, WOcandIndex->n);
        MatNd_copy(globIndex, WOcandIndex);

        // and set the according rank
        globRank = WOcandRank;

        // we cannot possibly find another subset
        if (globIndex->n <= subsetN || candN == candidatesIndex->m - 1)
        {
          break;
        }

        // remove those candidates from the remaining candidates that
        // contain rows that we just removed
        MatNd remainingCandidates =
          MatNd_fromPtr(candidatesIndex->m-candN-1, candidatesIndex->n,
                        MatNd_getRowPtr(candidatesIndex, candN + 1));
        candidatesIndex->m = removeRowsContainingMember(&remainingCandidates, &candIndex) + candN + 1;
      } //subset found

      MatNd_destroy(WOcandIndex);
    } //for candidateN

    MatNd_destroy(candidatesIndex);

  } //for subsetN

  MatNd_destroy(candJ);
  MatNd_destroy(WOcandJ);

  //  MatNd_printComment("within task subset independentSubset", globIndex);
  //  fprintf(stderr, "of rank %d\n", globRank);
  double globAsum = subJacobian(NULL, NULL, globIndex, a_des);

#ifdef TASK_SCALE
  updateMinScaleFactor(minScaleFactor, globRank, globAsum);
  //  printf("minScaleFactor %f\n\n",minScaleFactor);
  MatNd_setElementsTo(scaleFactors, minScaleFactor);
#else
  setScaleFactors(scaleFactors, globRank, globAsum, globIndex);
#endif

  MatNd_destroy(globIndex);

  return error;
}

/*******************************************************************************
 *
 ******************************************************************************/
static void subTaskJacobian(MatNd* subJ,
                            MatNd* subA,
                            const MatNd* J,
                            const MatNd* subJindex,
                            const MatNd* j2task,
                            const MatNd* a)
{
  if (subJ != NULL)
  {
    subJ->m = 0;
    subJ->n = J->n;
  }
  if (subA != NULL)
  {
    subA->m = 0;
    subA->n = 1;
  }
  unsigned int rowJ = 0;
  for (unsigned int colJindex = 0; colJindex < subJindex->n; colJindex++)
  {
    bool nextTask = false;
    while (rowJ < j2task->m && nextTask == false)
    {
      unsigned int task = (unsigned int) subJindex->ele[colJindex];
      if (j2task->ele[rowJ] <= task)
      {
        if (j2task->ele[rowJ] == task)
        {
          if (subJ != NULL)
          {
            subJ->m++;
            MatNd_setRow(subJ, subJ->m - 1,
                         MatNd_getRowPtr(J, rowJ), J->n);
          }
          if (subA != NULL)
          {
            subA->ele[subA->m] = a->ele[task];
            subA->m++;
          }
        }
        rowJ++;
      }
      else
      {
        nextTask = true;
      }
    }
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
static void setScale(MatNd* scale,
                     const MatNd* scaleFactors,
                     const MatNd* subJindex,
                     const MatNd* j2task)
{
  unsigned int rowJ = 0;
  unsigned int colScaleFactors = 0;
  for (unsigned int colJindex = 0; colJindex < subJindex->n; colJindex++)
  {
    bool nextTask = false;
    while (rowJ < j2task->m && nextTask == false)
    {
      unsigned int task = (unsigned int) subJindex->ele[colJindex];
      if (j2task->ele[rowJ] <= task)
      {
        if (j2task->ele[rowJ] == task)
        {
          scale->ele[rowJ] = scaleFactors->ele[colScaleFactors];
          colScaleFactors++;
        }
        rowJ++;
      }
      else
      {
        nextTask = true;
      }
    }
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
double TaskSpaceBlender::getActivationScaling(MatNd* scaling,
                                              const MatNd* J,
                                              const MatNd* a_des,
                                              const double tol) const
{
  double error = 0.0;

  MatNd_reshape(scaling, J->m, 1);
  MatNd_setElementsTo(scaling, 1.);

  MatNd* candJ = NULL;
  MatNd_create2(candJ, J->m, J->n);
  MatNd* WOcandJ = NULL;
  MatNd_create2(WOcandJ, J->m, J->n);
  MatNd* candA = NULL;
  MatNd_create2(candA, J->m, 1);

  MatNd* globIndex = NULL;
  MatNd_create2(globIndex, 1, a_des->m);

  MatNd* j2task = NULL;
  MatNd_create2(j2task, J->m, 1);

  MatNd* scaleFactors = NULL;
  MatNd_create2(scaleFactors, J->m, 1);

  globIndex->m = 1;
  globIndex->n = 0;

  j2task->m = 0;
  j2task->n = 1;

  for (unsigned int i = 0; i < a_des->m; i++)
  {
    if (MatNd_get(a_des, i, 0) > 0.0)
    {
      // write array that contains the IDs of the active tasks
      globIndex->ele[globIndex->n] = (double) i;
      globIndex->n++;

      // write array that indicates which row of the Jacobian corresponds
      // to which task ID
      // the considered Jacobian does not contain the kinematic joint tasks
      const Rcs::Task* task = controller->getTask(i);

      if (task->getClassName() == "Joint")
      {
        const Rcs::TaskJoint* jntTask =
          static_cast<const Rcs::TaskJoint*>(task);
        const RcsJoint* ji = jntTask->getJoint();

        if (ji->ctrlType != RCSJOINT_CTRL_TORQUE)
        {
          continue;
        }
      }
      unsigned int dimTask = controller->getTaskDim(i);
      VecNd_setElementsTo(&j2task->ele[j2task->m], (double) i, dimTask);
      j2task->m += dimTask;
    }
  }

  // total rank of the Jacobian
  int globRank = MatNd_rank(J, tol);

#ifdef PER_TASK // first per task, than per Jacobian entry of tasks
  // globIndex->n is going to change, so store it for the termination condition
  unsigned int totalTasks = globIndex->n;

  // find the smallest subsets first, checking for the smaller half is
  // sufficient
  for (unsigned int subsetN = 1; subsetN <= totalTasks / 2; subsetN++)
  {
    // the remaining tasks are considered the final subset, so we can stop if
    // we have less than 2 full subsets
    if (globIndex->n < 2 * subsetN)
    {
      break;
    }

    // now we enumerate all possible subsets of size subsetN
    MatNd* candidatesIndex = NULL;
    MatNd_create2(candidatesIndex, Math_NchooseK(globIndex->n, subsetN), subsetN);
    enumerateSubsets(candidatesIndex, globIndex->n, subsetN, globIndex);

    // going through the candidates
    for (unsigned int candN = 0; candN < candidatesIndex->m; candN++)
    {
      // creating a vector of the tasks not in the candidate subset
      MatNd* WOcandIndex = NULL;
      MatNd_create2(WOcandIndex, 1, globIndex->n - candidatesIndex->n);
      MatNd candIndex = MatNd_fromPtr(1, candidatesIndex->n, MatNd_getRowPtr(candidatesIndex, candN));
      remainderOfSet(WOcandIndex, globIndex, &candIndex);

      // assembling the Jacobian of the tasks not in the subset (we can
      // have multiple rows per task)
      subTaskJacobian(WOcandJ, NULL, J, WOcandIndex, j2task, NULL);

      // assembling the Jacobian of the tasks in the subset
      subTaskJacobian(candJ, candA, J, &candIndex, j2task, a_des);

      double candRelaxedRank;
      double WOcandRelaxedRank;
      int candDiffCount;
      int WOcandDiffCount;

      // now we check whether the subset is independent of the remaining
      // rows
      int candRank = MyRank(candJ, tol, candRelaxedRank, candDiffCount);
      int WOcandRank = MyRank(WOcandJ, tol, WOcandRelaxedRank, WOcandDiffCount);

      if ((candRank + WOcandRank) == globRank)
      {
        double newQuality = std::max((WOcandRelaxedRank - (double)WOcandRank)/std::max(1., (double)WOcandDiffCount),
                                     (candRelaxedRank - (double)candRank)/std::max(1., (double)candDiffCount));
        error = std::max(error, newQuality);

        //        MatNd_printComment("independentSubset", &candIndex);
        //        fprintf(stderr, "of rank %d\n\n", candRank);

        newQuality = getScaleFactors(scaleFactors, candJ, candA, candRank, tol);
        error = std::max(error, newQuality);

        setScale(scaling, scaleFactors, &candIndex, j2task);

        // remove the independent tasks of the tasks we still need to
        // consider
        MatNd_reshape(globIndex, WOcandIndex->m, WOcandIndex->n);
        MatNd_copy(globIndex, WOcandIndex);

        // and set the according rank
        globRank = WOcandRank;

        // we cannot possibly find another subset
        if (globIndex->n <= subsetN || candN == candidatesIndex->m - 1)
        {
          break;
        }

        // remove those candidates from the remaining candidates that
        // contain tasks that we just removed
        MatNd remainingCandidates =
          MatNd_fromPtr(candidatesIndex->m-candN-1, candidatesIndex->n,
                        MatNd_getRowPtr(candidatesIndex, candN + 1));
        candidatesIndex->m = removeRowsContainingMember(&remainingCandidates, &candIndex) + candN + 1;
      } //subset found

      MatNd_destroy(WOcandIndex);
    } //for candidateN

    MatNd_destroy(candidatesIndex);

  } //for subsetN

  // assembling the Jacobian of the remaining tasks
  subTaskJacobian(candJ, candA, J, globIndex, j2task, a_des);

  //  MatNd_printComment("independentSubset", globIndex);
  //  fprintf(stderr, "of rank %d\n\n", globRank);

#else // per entry in global Jacobian
  subTaskJacobian(candJ, candA, J, globIndex, j2task, a_des);
#endif

  double newQuality = getScaleFactors(scaleFactors, candJ, candA, globRank, tol);

  error = std::max(error, newQuality);
  setScale(scaling, scaleFactors, globIndex, j2task);

  //  MatNd_printComment("scaling", scaling);

  MatNd_destroy(candJ);
  MatNd_destroy(WOcandJ);
  MatNd_destroy(candA);
  MatNd_destroy(globIndex);
  MatNd_destroy(j2task);
  MatNd_destroy(scaleFactors);

  return error;
}

/*******************************************************************************
 *
 ******************************************************************************/
void TaskSpaceBlender::computeApproximate(MatNd* Wx,
                                          MatNd* ax_curr,
                                          const MatNd* a_des,
                                          const MatNd* J,
                                          const MatNd* invWq,
                                          const bool useInnerProduct) const
{
  RCHECK_MSG((invWq->m==1) && (invWq->n==1), "Currently, approximate weighting"
             " only works for a scalar lambda");

  unsigned int i, j, dimTask, nRows = 0;
  const double lambda0 = invWq->ele[0];

  for (i=0; i<a_des->m; i++)
  {

    if (MatNd_get(a_des, i, 0) == 0.0)
    {
      NLOG(4, "Skipping task \"%s\": activation is %f",
           controller->getTaskName(i).c_str(), a_des->ele[i]);
      continue;
    }

    const double ci = Math_clip(MatNd_get(a_des, i, 0), 0.0, 1.0);

    dimTask = controller->getTaskDim(i);

    RCHECK_MSG(Wx->size >= nRows + dimTask, "While adding task "
               "\"%s\": size of Wx: %d   m: %d   dimTask: %d",
               controller->getTaskName(i).c_str(), Wx->size, nRows, dimTask);

    for (j=0; j<dimTask; j++)
    {
      // Task space blending part 1
      double aBuf = 1.0, wi1 = 1.0;

      if (useInnerProduct==true)
      {
        MatNd Ji = MatNd_fromPtr(1, J->n, MatNd_getRowPtr(J, nRows));
        MatNd a = MatNd_fromPtr(1, 1, &aBuf);
        MatNd_sqrMulABAt(&a, &Ji, NULL);
      }

      if (ci < 1.0)
      {
        if (aBuf == 0.0)
        {
          wi1 = 0.0;
        }
        else
        {
          wi1 = lambda0 * ci / (aBuf * (1.0 - ci));
        }
      }

      // Task space blending part 2
      // The minimum lambda_i is one or two orders of magnitude smaller
      // than the joint space metric lambda
      double wi2 = pow(0.01*lambda0, 1.0-ci);

      // Fusion of both parts
      Wx->ele[nRows] = a_des->ele[i]*wi2 + (1.0-a_des->ele[i])*wi1;
      nRows++;

    }   // for(j=0;j<dimTask;j++)


  }   // for(i=0; i<nTasks; i++)

  // Reshape
  Wx->m = nRows;
  Wx->n = 1;


  if (ax_curr != NULL)
  {
    computeActivation(ax_curr, Wx, a_des, J, invWq);
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
void TaskSpaceBlender::computeBinary(MatNd* Wx,
                                     MatNd* ax_curr,
                                     const MatNd* a_des) const
{
  const size_t wDim = controller->getActiveTaskDim(a_des);
  MatNd_reshape(Wx, wDim, 1);
  MatNd_setElementsTo(Wx, 1.0);

  if (ax_curr != NULL)
  {
    const unsigned int nTasks = a_des->m;
    unsigned nRows = 0;

    for (unsigned int i=0; i<nTasks; i++)
    {
      const unsigned int dimTask = controller->getTaskDim(i);
      const double a_i = MatNd_get(a_des, i, 0) > 0.0 ? 1.0 : 0.0;

      RCHECK_MSG(ax_curr->size >= nRows + dimTask, "While adding task "
                 "\"%s\": size of Wx: %d   m: %d   dimTask: %d",
                 controller->getTaskName(i).c_str(), ax_curr->size, nRows, dimTask);

      VecNd_setElementsTo(&ax_curr->ele[nRows], a_i, dimTask);
      nRows += dimTask;
    }

    ax_curr->m = nRows;
    ax_curr->n = 1;
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
void TaskSpaceBlender::computeLinear(MatNd* Wx,
                                     MatNd* ax_curr,
                                     const MatNd* a_des) const
{
  unsigned int nRows = 0;

  for (unsigned int i=0; i<a_des->m; i++)
  {
    const double a = MatNd_get(a_des, i, 0);

    if (a <= 0.0)
    {
      continue;
    }

    unsigned int dimTask = controller->getTaskDim(i);

    RCHECK_MSG(Wx->size >= nRows + dimTask, "While adding task "
               "\"%s\": size of Wx: %d   m: %d   dimTask: %d",
               controller->getTaskName(i).c_str(), Wx->size, nRows, dimTask);

    VecNd_setElementsTo(&Wx->ele[nRows], a, dimTask);
    nRows += dimTask;
  }

  // Reshape
  Wx->m = nRows;
  Wx->n = 1;

  if (ax_curr)
  {
    MatNd_reshapeCopy(ax_curr, Wx);
  }
}

}   // namespace Rcs
