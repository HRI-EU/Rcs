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

/******************************************************************************

  http://eigen.tuxfamily.org/dox/TopicPreprocessorDirectives.html:

  EIGEN_NO_MALLOC - if defined, any request from inside the Eigen to allocate
                    memory from the heap results in an assertion failure. This
                    is useful to check that some routine does not allocate
                    memory dynamically. Not defined by default.

  EIGEN_RUNTIME_NO_MALLOC - if defined, a new switch is introduced which can be
                            turned on and off by calling set_is_malloc_allowed
                            (bool). If malloc is not allowed and Eigen tries to
                            allocate memory dynamically anyway, an assertion
                            failure results. Not defined by default.

  Mapping C-Arrays:
   http://eigen.tuxfamily.org/dox/group__TutorialMapClass.html#
          TutorialMapPlacementNew

  Malloc checking:
   Eigen::internal::set_is_malloc_allowed(true, false) will create a fatal
   error if set to true, and disable checking for false. However, this is
   not thread-safe, and therefore should only be used for testing.

******************************************************************************/

#include <Rcs_eigen.h>
#include <Rcs_macros.h>

#define EIGEN_RUNTIME_NO_MALLOC

#include <Eigen/Core>
#include <Eigen/Dense>

#include <Eigen/QR>
#include <Eigen/SVD>
#include <Eigen/Cholesky>
#include <Eigen/LU>

#include <iostream>
#include <cstdio>

using namespace Eigen;

#if defined (MATND_MAX_STACK_VECTOR_SIZE)
#undef MATND_MAX_STACK_VECTOR_SIZE
#endif

#if defined (MATND_MAX_STACK_MATRIX_SIZE)
#undef MATND_MAX_STACK_MATRIX_SIZE
#endif

#define MATND_MAX_STACK_VECTOR_SIZE (25)
#define MATND_MAX_STACK_MATRIX_SIZE (25*25)


// These types have an unknown size at compile time, and therefore are
// allocated on the heap. The storage order of the matrices is set to row
// major, so that the Eigen's Map class can be used to cast them over a MatNd
// type without any additional memory allocation.
typedef Matrix<double, Dynamic, Dynamic, RowMajor> HeapMat;
typedef Matrix<double , Dynamic, 1, ColMajor> HeapVec;



// These types have a known size at compile time, and therefore are allocated
// on the stack. The storage order of the matrices is set to row major, so that
// the Eigen's Map class can be used to cast them over a MatNd type without any
// additional memory allocation.
typedef Matrix<double, Dynamic, Dynamic, RowMajor,
        MATND_MAX_STACK_VECTOR_SIZE,
        MATND_MAX_STACK_VECTOR_SIZE> StackMat;
typedef Matrix<double , Dynamic, 1, ColMajor,
        MATND_MAX_STACK_VECTOR_SIZE , 1> StackVec;



template <typename T>
static inline void MatNd_fromEigen3(MatNd* dst, const T& src,
                                    bool reshape=false)
{
  if (reshape==false)
  {
    RCHECK_EQ((int)dst->m, src.rows());
    RCHECK_EQ((int)dst->n, src.cols());
  }
  else
  {
    MatNd_reshape(dst, src.rows(), src.cols());
  }

  for (size_t m=0; m<dst->m; m++)
  {
    for (size_t n=0; n<dst->n; n++)
    {
      MatNd_set2(dst, m, n, src(m,n));
    }
  }
}

template <typename T>
static inline double CholDet(const T& L)
{
  double det = 1.0;

  for (int i=0; i<L.rows(); i++)
  {
    det *= L(i, i)*L(i, i);
  }

  return det;
}

template <typename MatT>
static inline double CholDC(MatNd* L_, const MatNd* A_)
{
  Map<MatT> A(A_->ele, A_->m, A_->n);
  LLT<MatT> llt;
  llt.compute(A);

  if (llt.info() != Eigen::Success)
  {
    RLOG(1, "Error on LLT!");
    MatNd_setZero(L_);
    return 0.0;
  }

  MatT L(llt.matrixL());
  double det = CholDet<MatT>(L);
  MatNd_fromEigen3<MatT>(L_, L);

  return det;
}

template <typename MatT>
static inline double CholSolve(MatNd* x_, const MatNd* A_, const MatNd* b_)
{
  Map<const MatT> A(A_->ele, A_->m, A_->n);
  LLT<MatT> llt;
  llt.compute(A);

  if (llt.info() != Eigen::Success)
  {
    RLOG(1, "Error on LLT!");
    return 0.0;
  }

  Map<MatT> x(x_->ele, x_->m, x_->n);
  Map<const MatT> b(b_->ele, b_->m, b_->n);
  x = llt.solve(b);

  // Compute determinant
  MatT L(llt.matrixL());

  return CholDet<MatT>(L);
}

template <typename MatT>
static inline double CholInv(MatNd* A_inv_, const MatNd* A_)
{
  size_t n = A_->m;
  Map<const MatT> A(A_->ele, n, n);

  LLT<MatT> llt;
  llt.compute(A);

  if (llt.info() != Eigen::Success)
  {
    RLOG(1, "Error on LLT!");
    MatNd_setZero(A_inv_);
    return 0.0;
  }

  Map<MatT> invA(A_inv_->ele, n, n);
  invA = llt.solve(MatT::Identity(n,n));

  MatT L(llt.matrixL());

  return CholDet<MatT>(L);
}

template <typename MatT>
static inline int SvdDC(MatNd* U, MatNd* S, MatNd* V, const MatNd* J,
                        double eps)
{
  Map<MatT> mapJ(J->ele, J->m, J->n);

  // We need to compute the full SVD, since otherwise there will dynamic
  // memory allocation for the StackMat types due to the QR preconditioning
  // (on Linux)
  JacobiSVD<MatT> svd(mapJ, ComputeFullU | ComputeFullV);

  // Cut off singular values smaller than eps
  int rank = svd.nonzeroSingularValues();

  for (int i=0; i<svd.nonzeroSingularValues(); ++i)
  {
    if (svd.singularValues()(i) < eps)
    {
      rank = i;
      break;
    }
  }

  if (S != NULL)
  {
    MatNd_fromEigen3<MatT>(S, svd.singularValues(), true);
    S->m = rank;
  }

  if (U != NULL)
  {
    MatT svdU = svd.matrixU();
    MatNd_reshape(U, svdU.rows(), rank);

    for (unsigned int m=0; m<U->m; m++)
    {
      for (unsigned int n=0; n<U->n; n++)
      {
        MatNd_set2(U, m, n, svdU(m,n));
      }
    }
  }

  if (V != NULL)
  {
    MatT svdV = svd.matrixV();
    MatNd_reshape(V, svdV.rows(), rank);

    for (unsigned int m=0; m<V->m; m++)
    {
      for (unsigned int n=0; n<V->n; n++)
      {
        MatNd_set2(V, m, n, svdV(m,n));
      }
    }
  }

  return rank;
}

template <typename MatT>
static inline double SvdSolve(MatNd* x_, const MatNd* A_, const MatNd* b_)
{
  Map<MatT> x(x_->ele, x_->m, x_->n);
  Map<MatT> A(A_->ele, A_->m, A_->n);
  Map<MatT> b(b_->ele, b_->m, b_->n);

  // We need to compute the full SVD, since otherwise there will dynamic
  // memory allocation for the StackMat types due to the QR preconditioning
  // (on Linux)
  JacobiSVD<MatT> svd(A, ComputeFullU | ComputeFullV);
  x = svd.solve(b);

  // Compute determinant
  double det = 1.0;
  MatT S = svd.singularValues();

  // Cut off singular values smaller than eps
  for (int i=0; i<S.rows(); ++i)
  {
    det *= S(i);
  }

  return det;
}

template <typename MatT>
static inline size_t SvdInverse(MatNd* A_, double eps=1.0e-12)
{
  Map<MatT> A(A_->ele, A_->m, A_->n);

  // We need to compute the full SVD, since otherwise there will dynamic
  // memory allocation for the StackMat types due to the QR preconditioning
  // (on Linux)
  JacobiSVD<MatT> svd;
  //  svd.setThreshold(eps);
  svd.compute(A, ComputeThinU | ComputeThinV);

  MatT invS = svd.singularValues();

  // Cut off singular values smaller than eps
  size_t rank = 0;
  for (int i=0; i<invS.rows(); ++i)
  {
    if (invS(i) > eps)
    {
      invS(i) = 1.0/invS(i);
      rank++;
    }
  }

  // inv = V diag(S^-1) U^T
  A.noalias() = svd.matrixV()*invS.asDiagonal()*svd.matrixU().transpose();

  return rank;
}

template <typename MatT>
static inline void QRDC(MatNd* Q_, MatNd* R_, const MatNd* A_)
{
  Map<const MatT> A(A_->ele, A_->m, A_->n);
  Map<MatT> Q(Q_->ele, Q_->m, Q_->n);
  Map<MatT> R(R_->ele, R_->m, R_->n);

  HouseholderQR<MatT> qr(A);

  Q = qr.householderQ();
  R = qr.matrixQR().template triangularView<Eigen::Upper>();
}

template <typename MatT>
static inline bool EigenVectors(MatNd* V_, double* d_, const MatNd* A_)
{
  Map<const MatT> A(A_->ele, A_->m, A_->n);

  Eigen::SelfAdjointEigenSolver<MatT> eigensolver(A);

  if (eigensolver.info() != Eigen::Success)
  {
    return false;
  }

  Map<MatT> V(V_->ele, V_->m, V_->n);
  Map<MatT> d(d_, A_->m, 1);
  V = eigensolver.eigenvectors();
  d = eigensolver.eigenvalues();

  return true;
}

template <typename MatT>
static inline void ColPivHhQR(MatNd* X_, const MatNd* A_, const MatNd* B_)
{
  Map<const MatT> A(A_->ele, A_->m, A_->n);
  Map<const MatT> B(B_->ele, B_->m, B_->n);
  Map<MatT> X(X_->ele, X_->m, X_->n);

  // A X = B
  X = A.colPivHouseholderQr().solve(B);
}







#ifdef __cplusplus
extern "C" {
#endif


double MatNd_choleskyDecomposition_E3(MatNd* L, const MatNd* A)
{
  RCHECK_EQ(A->m, A->n);
  RCHECK_EQ(L->m, L->n);
  RCHECK_EQ(A->m, L->m);

  if (A->m > MATND_MAX_STACK_VECTOR_SIZE)
  {
    return CholDC<HeapMat>(L, A);
  }
  else
  {
#ifdef EIGEN_RUNTIME_NO_MALLOC
    Eigen::internal::set_is_malloc_allowed(false);
#endif
    double det = CholDC<StackMat>(L, A);
#ifdef EIGEN_RUNTIME_NO_MALLOC
    Eigen::internal::set_is_malloc_allowed(true);
#endif
    return det;
  }
}

double MatNd_choleskySolve_E3(MatNd* x, const MatNd* A, const MatNd* b)
{
  RCHECK_EQ(A->m, A->n);
  RCHECK_EQ(x->m, A->m);
  RCHECK_EQ(b->m, x->m);
  RCHECK_EQ(b->n, x->n);

  if (A->m > MATND_MAX_STACK_VECTOR_SIZE)
  {
    return CholSolve<HeapMat>(x, A, b);
  }
  else
  {
#ifdef EIGEN_RUNTIME_NO_MALLOC
    Eigen::internal::set_is_malloc_allowed(false);
#endif
    double det = CholSolve<StackMat>(x, A, b);
#ifdef EIGEN_RUNTIME_NO_MALLOC
    Eigen::internal::set_is_malloc_allowed(true);
#endif
    return det;
  }
}

double MatNd_choleskyInverse_E3(MatNd* invA, const MatNd* A)
{
  RCHECK_EQ(A->m, A->n);
  RCHECK_EQ(invA->m, invA->n);
  RCHECK_EQ(A->m, invA->m);

  if (A->m > MATND_MAX_STACK_VECTOR_SIZE)
  {
    return CholInv<HeapMat>(invA,A);
  }
  else
  {
#ifdef EIGEN_RUNTIME_NO_MALLOC
    Eigen::internal::set_is_malloc_allowed(false);
#endif
    double det = CholInv<StackMat>(invA,A);
#ifdef EIGEN_RUNTIME_NO_MALLOC
    Eigen::internal::set_is_malloc_allowed(true);
#endif
    return det;
  }
}

void MatNd_mul_E3(MatNd* C_, const MatNd* A_, const MatNd* B_)
{
  if ((A_->m > MATND_MAX_STACK_VECTOR_SIZE) ||
      (B_->n > MATND_MAX_STACK_VECTOR_SIZE))
  {
    Map<HeapMat> C(C_->ele, C_->m, C_->n);
    Map<HeapMat> A(A_->ele, A_->m, A_->n);
    Map<HeapMat> B(B_->ele, B_->m, B_->n);

    // The noalias function inhibits the creation of a temporary object which
    // would lead to dynamic memory alllocation.
    C.noalias() = A*B;
  }
  else
  {
#ifdef EIGEN_RUNTIME_NO_MALLOC
    Eigen::internal::set_is_malloc_allowed(false);
#endif

    Map<StackMat> C(C_->ele, C_->m, C_->n);
    Map<StackMat> A(A_->ele, A_->m, A_->n);
    Map<StackMat> B(B_->ele, B_->m, B_->n);

    C.noalias() = A*B;

#ifdef EIGEN_RUNTIME_NO_MALLOC
    Eigen::internal::set_is_malloc_allowed(true);
#endif
  }
}







unsigned int MatNd_SVD(MatNd* U, MatNd* S, MatNd* V, const MatNd* J,
                       double eps)
{
  if ((J->m > MATND_MAX_STACK_VECTOR_SIZE) ||
      (J->n > MATND_MAX_STACK_VECTOR_SIZE))
  {
    return SvdDC<HeapMat>(U, S, V, J, eps);
  }
  else
  {
#ifdef EIGEN_RUNTIME_NO_MALLOC
    Eigen::internal::set_is_malloc_allowed(false);
#endif
    int rank = SvdDC<StackMat>(U, S, V, J, eps);
#ifdef EIGEN_RUNTIME_NO_MALLOC
    Eigen::internal::set_is_malloc_allowed(true);
#endif
    return rank;
  }
}

double MatNd_SVDSolve(MatNd* x, const MatNd* A, const MatNd* b)
{
  if ((A->m > MATND_MAX_STACK_VECTOR_SIZE) ||
      (A->n > MATND_MAX_STACK_VECTOR_SIZE))
  {
    return SvdSolve<HeapMat>(x, A, b);
  }
  else
  {
#ifdef EIGEN_RUNTIME_NO_MALLOC
    Eigen::internal::set_is_malloc_allowed(false);
#endif
    double det = SvdSolve<StackMat>(x, A, b);
#ifdef EIGEN_RUNTIME_NO_MALLOC
    Eigen::internal::set_is_malloc_allowed(true);
#endif
    return det;
  }
}

unsigned int MatNd_SVDInverse(MatNd* inv, const MatNd* src)
{
  MatNd_copy(inv, src);
  return MatNd_SVDInverseSelf(inv);
}

unsigned int MatNd_SVDInverseSelf(MatNd* A)
{
  if ((A->m > MATND_MAX_STACK_VECTOR_SIZE) ||
      (A->n > MATND_MAX_STACK_VECTOR_SIZE))
  {
    return SvdInverse<HeapMat>(A);
  }
  else
  {
#ifdef EIGEN_RUNTIME_NO_MALLOC
    Eigen::internal::set_is_malloc_allowed(false);
#endif
    int rank = SvdInverse<StackMat>(A);
#ifdef EIGEN_RUNTIME_NO_MALLOC
    Eigen::internal::set_is_malloc_allowed(true);
#endif
    return rank;
  }
}

unsigned int MatNd_rank(const MatNd* J, double eps)
{
  return MatNd_SVD(NULL, NULL, NULL, J, eps);
}

void MatNd_QRDecomposition(MatNd* Q, MatNd* R, const MatNd* A)
{
  if ((A->m > MATND_MAX_STACK_VECTOR_SIZE) ||
      (A->n > MATND_MAX_STACK_VECTOR_SIZE) ||
      (A->n > A->m))   // That's a bug in Eigen3
  {
    QRDC<HeapMat>(Q, R, A);
  }
  else
  {
#ifdef EIGEN_RUNTIME_NO_MALLOC
    Eigen::internal::set_is_malloc_allowed(false);
#endif
    QRDC<StackMat>(Q, R, A);
#ifdef EIGEN_RUNTIME_NO_MALLOC
    Eigen::internal::set_is_malloc_allowed(true);
#endif
  }
}

bool MatNd_getEigenVectors(MatNd* V, double* d, const MatNd* A)
{
  RCHECK_EQ(A->m, A->n);
  RCHECK_EQ(A->n, V->m);
  RCHECK_EQ(V->m, V->n);

  bool success;

  if (A->m > MATND_MAX_STACK_VECTOR_SIZE)
  {
    success = EigenVectors<HeapMat>(V, d, A);
  }
  else
  {
#ifdef EIGEN_RUNTIME_NO_MALLOC
    Eigen::internal::set_is_malloc_allowed(false);
#endif
    success = EigenVectors<StackMat>(V, d, A);
#ifdef EIGEN_RUNTIME_NO_MALLOC
    Eigen::internal::set_is_malloc_allowed(true);
#endif
  }

  return success;
}

void MatNd_HouseholderQR(MatNd* X, const MatNd* A, const MatNd* B)
{
  if ((A->m > MATND_MAX_STACK_VECTOR_SIZE) ||
      (A->n > MATND_MAX_STACK_VECTOR_SIZE))
  {
    return ColPivHhQR<HeapMat>(X, A, B);
  }
  else
  {
#ifdef EIGEN_RUNTIME_NO_MALLOC
    Eigen::internal::set_is_malloc_allowed(false);
#endif
    return ColPivHhQR<StackMat>(X, A, B);
#ifdef EIGEN_RUNTIME_NO_MALLOC
    Eigen::internal::set_is_malloc_allowed(true);
#endif
  }
}




#ifdef __cplusplus
}
#endif
