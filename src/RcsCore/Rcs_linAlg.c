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

#include "Rcs_MatNd.h"
#include "Rcs_macros.h"



/*******************************************************************************
 * Dot product of two vectors considering stride. This way, any combination of
 * rows and columns can be multiplied.
 ******************************************************************************/
static inline double dot(const double* aPtr, int strideA,
                         const double* bPtr, int strideB,
                         unsigned int n)
{
  double sum = 0.0;

  for (unsigned int i=0; i<n; ++i)
  {
    sum += *aPtr * *bPtr;
    aPtr += strideA;
    bPtr += strideB;
  }

  return sum;
}

/*******************************************************************************
 * Tadeusz Banachiewicz Cholesky Decomposition
 * From: Locher, Numerische Mathematik fuer Informatiker, 2nd edition,
 * Springer Verlag, p. 248
 * The Cholesky factor L (lower triangle) is copied over A_, the superdiagonal
 * elements are untouched.
 ******************************************************************************/
static double MatNd_choleskyDecompositionSelf(MatNd* A_)
{
  RCHECK_EQ(A_->m, A_->n);

  const unsigned int n = A_->m;
  double det = 1.0;
  double* A = A_->ele;

  for (unsigned int i=0; i<n; ++i)
  {
    unsigned int i_n = i*n;

    for (unsigned int k=0; k<=i; ++k)
    {
      double sum = A[i_n+k] - dot(&A[i_n], 1, &A[k*n], 1, k);

      if (i>k)
      {
        A[i_n+k] = sum/A[k*n+k];
      }
      else if (sum > 0.0)
      {
        A[i_n+i] = sqrt(sum);
        det *= A[i_n+i]*A[i_n+i];
      }
      else
      {
        RLOG(4, "Cholesky decomposition for %d x %d matrix failed", n, n);
        MatNd_setZero(A_);
        return 0.0;
      }
    }
  }

  return det;
}

/*******************************************************************************
 *
 ******************************************************************************/
double MatNd_choleskyDecomposition(MatNd* L, const MatNd* A)
{
  MatNd_copy(L, A);

  // In-place Cholesky decomposition leaves the upper triangle untouched.
  double det = MatNd_choleskyDecompositionSelf(L);

  // Set upper triangle to zero. We don't need the last index, since in the
  // row, there is no element to be set to zero.
  if (A->n>1)
  {
    for (unsigned int i=0; i<A->n-1; ++i)
    {
      memset(&L->ele[i*A->n+i+1], 0, (A->n-i-1)*sizeof(double));
    }
  }

  return det;
}

/*******************************************************************************
  Solves for x with forward and back substitution:
  z = L*b
  X = L^T*z
  Following the explanations of
  https://en.wikipedia.org/wiki/Triangular_matrix#Forward_substitution
*******************************************************************************/
double MatNd_choleskySolve(MatNd* x_, const MatNd* A, const MatNd* b_)
{
  const int cols = x_->n;
  const int n = A->m;
  int i, j, i_n, col, i_cols;

  RCHECK_EQ(x_->m, b_->m);
  RCHECK_EQ(x_->n, b_->n);
  RCHECK_EQ(x_->m, n);

  MatNd* L_ = NULL;
  MatNd_clone2(L_, A);
  const double* L = L_->ele;
  const double* b = b_->ele;
  double* x = x_->ele;

  double det = MatNd_choleskyDecompositionSelf(L_);

  if (det==0.0)
  {
    MatNd_destroy(L_);
    return 0.0;
  }

  // Extract column of b and x
  for (col=0; col<cols; col++)
  {
    // Forward solve L*z = b
    for (i = 0; i < n; i++)
    {
      i_n = i*n;
      i_cols = i*cols;
      x[col+i_cols] = b[col+i_cols];

      for (j = 0; j < i; j++)
      {
        x[col+i_cols] -= L[i_n+j] * x[col+j*cols];
      }

      x[col+i_cols] /= L[i_n+i];
    }

    // Backward solve L^T*x = z
    for (i = n - 1; i >= 0; i--)
    {
      i_cols = i*cols;
      for (j = i + 1; j < n; j++)
      {
        x[col+i_cols] -= L[j*n+i] * x[col+j*cols];
      }

      x[col+i_cols] /= L[i*n+i];
    }
  }

  MatNd_destroy(L_);

  return det;
}

/*******************************************************************************
 *
 *  Based on the algorithm 2.9 page 94 from the book
 *
 *  Matrix Algorithms, Volume I:Basic Decompositions
 *  G. W. Stewart, University of Maryland, College Park, Maryland
 *  Publisher: Society for Industrial and Applied Mathematics (SIAM),
 *             Philadelphia
 *
 *  1. for k = 1 to n
 *  2.   L[k,k] = 1/L[k,k]
 *  3.     for i = k+1 to n
 *  4.     L[i, k] = -L[i, k:i-1]*L[k:i-1, k]/L[k, k]
 *  5.   end for i
 *  6. end for k
 *
 ******************************************************************************/
static inline void MatNd_inverseLowerTriangularMatrixSelf(MatNd* L_)
{
  const int n = L_->m;
  double* L = L_->ele;

  for (int k=0; k<n; k++)                                            // 1
  {
    const int knpk = k*n+k;
    L[knpk] = 1.0/L[knpk];                                           // 2

    for (int i=k+1; i<n; i++)                                        // 3
    {
      const int in   = i*n;
      const int inpk = in+k;
      L[inpk] = -dot(&L[inpk], 1, &L[knpk], n, i-k)/L[in+i];         // 4
    }                                                                // 5
  }                                                                  // 6
}

/*******************************************************************************
 *
 *  Based on the algorithm 2.3 page 94 from the book
 *
 *  Matrix Algorithms, Volume I:Basic Decompositions
 *  G. W. Stewart, University of Maryland, College Park, Maryland
 *  Publisher: Society for Industrial and Applied Mathematics (SIAM),
 *             Philadelphia
 *
 *  1. for k = 1 to n
 *  2.   X[k,k] = 1/L[k,k]
 *  3.     for i = k+1 to n
 *  4.     X[i, k] = -L[i, k:i-1]*X[k:i-1, k]/L[i, i]
 *  5.   end for i
 *  6. end for k
 *
 ******************************************************************************/
static inline void MatNd_inverseLowerTriangularMatrix(MatNd* X_,
                                                      const MatNd* L_)
{
  const int n = L_->m;
  double* L = L_->ele;
  double* X = X_->ele;

  for (int k=0; k<n; ++k)                                            // 1
  {
    const int knpk = k*n+k;
    X[knpk] = 1.0/L[knpk];                                           // 2

    for (int i=k+1; i<n; i++)                                        // 3
    {
      const int in   = i*n;
      const int inpk = in+k;
      X[inpk] = -dot(&L[inpk], 1, &X[knpk], n, i-k)/L[in+i];         // 4
    }                                                                // 5
  }                                                                  // 6
}

/*******************************************************************************
 * Inverse using Cholesky factors: inv(A) = inv(L)^T*inv(L) = inv(U)*inv(U)^T
 ******************************************************************************/
double MatNd_choleskyInverse(MatNd* A_inv, const MatNd* A)
{
  const unsigned int n = A->m;

  MatNd* L = NULL;
  MatNd_clone2(L, A);

  // Decompose A into Cholesky factors L (L is on lower triangle only)
  double det = MatNd_choleskyDecomposition(L, A);

  if (det == 0.0)
  {
    MatNd_setZero(A_inv);
  }
  else
  {
    // Invert L
    MatNd_inverseLowerTriangularMatrixSelf(L);

    // A_inv = inv(L)^T*inv(L). Zero elements are spared out.
    for (unsigned int row=0; row<n; ++row)
    {
      unsigned int row_n = row*n;

      for (unsigned int col=row; col<n; ++col)
      {
        unsigned int tmp = row > col ? row : col;
        double sum = dot(&L->ele[tmp*n+row], n, &L->ele[tmp*n+col], n, n-tmp);
        A_inv->ele[row_n+col] = sum;
        A_inv->ele[col*n+row] = sum;
      }
    }
  }

  MatNd_destroy(L);

  return det;
}

/*******************************************************************************
 * Returns the determinant of a square matrix using Gaussian elimination.
 ******************************************************************************/
//! \todo Could be done more efficiently (decomposition rather than inverse)
double MatNd_determinant(const MatNd* src)
{
  MatNd* inv = NULL;
  MatNd_clone2(inv, src);
  double det = MatNd_gaussInverse(inv, src);
  MatNd_destroy(inv);

  return det;
}

/*******************************************************************************
 * Gauss-Seidel solver. See also:
 * http://en.wikipedia.org/wiki/Gauss%E2%80%93Seidel_method
 ******************************************************************************/
int MatNd_gaussSeidelIterate(MatNd* x, const MatNd* A, const MatNd* b,
                             const double eps, const int maxIter)
{
  int i, j, steps = 0, n = A->m;
  double change;

  do
  {
    steps++;
    change = 0.;

    // The algorithm
    for (i = 0; i < n; i++)
    {
      double x_old = x->ele[i];

      x->ele[i] = b->ele[i];

      for (j = 0; j < n; j++)
      {
        if (j != i)
        {
          x->ele[i] -= A->ele[i * n + j] * x->ele[j];
        }
      }
      x->ele[i] /= A->ele[i * n + i];

      double x_diff = fabs(x->ele[i] - x_old);
      if (x_diff > change)
      {
        change = x_diff;
      }
    }
  }
  while (!(change < eps || steps==maxIter));   //accuracy or maxIter reached

  return steps;
}

/*******************************************************************************
 *
 ******************************************************************************/
int MatNd_gaussSeidelSolve(MatNd* x, const MatNd* A, const MatNd* b)
{
  int n = A->m;
  RCHECK_MSG(A->m == A->n, "A->m=%d   A->n=%d", A->m, A->n);
  RCHECK_MSG(x->m == n, "x->m=%d   n=%d", x->m, n);
  RCHECK_MSG(x->n == 1, "x->n=%d", x->n);
  RCHECK_MSG(b->m == n, "x->m=%d   n=%d", x->m, n);
  RCHECK_MSG(b->n == 1, "b->n=%d", b->n);

  return MatNd_gaussSeidelIterate(x, A, b, 1.0e-8, -1);
}

/*******************************************************************************
 * We define these here so that there's no linker errors in case no Eigen3
 * library is present.
 ******************************************************************************/
#if !defined (USE_EIGEN3)
// Under Visual Studio, we compile all C-files with the C++ compiler due to the
// lack of C99 support. Therefore we need the extern C directive here.
#ifdef __cplusplus
extern "C" {
#endif

unsigned int MatNd_SVD(MatNd* U, MatNd* S, MatNd* V, const MatNd* J,
                       double eps)
{
  RFATAL("Function not available. Did you compile with Eigen3 support?");
  return 0;
}

double MatNd_SVDSolve(MatNd* x, const MatNd* A, const MatNd* b)
{
  RFATAL("Function not available. Did you compile with Eigen3 support?");
  return 0.0;
}

unsigned int MatNd_SVDInverse(MatNd* inv, const MatNd* src)
{
  RFATAL("Function not available. Did you compile with Eigen3 support?");
  return 0;
}

unsigned int MatNd_SVDInverseSelf(MatNd* A)
{
  RFATAL("Function not available. Did you compile with Eigen3 support?");
  return 0;
}

unsigned int MatNd_rank(const MatNd* J, double eps)
{
  RFATAL("Function not available. Did you compile with Eigen3 support?");
  return 0;
}

void MatNd_QRDecomposition(MatNd* Q, MatNd* R, const MatNd* A)
{
  RFATAL("Function not available. Did you compile with Eigen3 support?");
}

bool MatNd_getEigenVectors(MatNd* V, double* d, const MatNd* A)
{
  RFATAL("Function not available. Did you compile with Eigen3 support?");
  return false;
}

void MatNd_HouseholderQR(MatNd* X, const MatNd* A, const MatNd* B)
{
  RFATAL("Function not available. Did you compile with Eigen3 support?");
}

#ifdef __cplusplus
}
#endif
#endif   // USE_EIGEN3
